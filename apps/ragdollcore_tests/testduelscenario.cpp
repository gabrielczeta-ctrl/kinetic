#include <gtest/gtest.h>

#include <ragdollcore/duelscenario.hpp>
#include <ragdollcore/scenario.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

using namespace RagdollCore;

namespace
{
    DuelSettings armedSettings()
    {
        DuelSettings settings;
        settings.mDurationSeconds = 12.0f;
        settings.mSeparation = 46.0f;
        settings.mLeft.mWeapon.mArmed = true;
        settings.mRight.mWeapon.mArmed = true;
        settings.mLeft.mHealth = 24.0f;
        settings.mRight.mHealth = 24.0f;
        settings.mLeft.mFirstAttackSeconds = 0.8f;
        settings.mRight.mFirstAttackSeconds = 1.6f;
        settings.mLeft.mSwingIntervalSeconds = 1.6f;
        settings.mRight.mSwingIntervalSeconds = 1.6f;
        return settings;
    }

    DuelSettings unarmedSettings()
    {
        DuelSettings settings = armedSettings();
        settings.mLeft.mWeapon.mArmed = false;
        settings.mRight.mWeapon.mArmed = false;
        settings.mLeft.mFistMinimumDamage = 2.0f;
        settings.mLeft.mFistMaximumDamage = 6.0f;
        settings.mRight.mFistMinimumDamage = 2.0f;
        settings.mRight.mFistMaximumDamage = 6.0f;
        return settings;
    }

    std::uint64_t hashResult(const DuelResult& result)
    {
        std::uint64_t hash = 1469598103934665603ull;
        const auto mix = [&hash](std::uint64_t value) {
            hash ^= value;
            hash *= 1099511628211ull;
        };
        for (const DuelFrame& frame : result.mFrames)
        {
            const std::array<const DuelActorFrame*, 2> actors{ &frame.mLeft, &frame.mRight };
            for (const DuelActorFrame* actor : actors)
            {
                float health = actor->mHealth;
                std::uint32_t bits = 0;
                std::memcpy(&bits, &health, sizeof(bits));
                mix(bits);
                mix(static_cast<std::uint64_t>(actor->mBehavior));
                for (const JPH::Vec3& position : actor->mBodyPositions)
                {
                    const std::array<float, 3> values{ position.GetX(), position.GetY(), position.GetZ() };
                    for (const float value : values)
                    {
                        const auto quantized = static_cast<std::int64_t>(std::llround(value * 100.0f));
                        mix(static_cast<std::uint64_t>(quantized));
                    }
                }
            }
        }
        return hash;
    }
}

TEST(RagdollCoreDuelScenarioTest, ArmedDuelConnectsAndDamages)
{
    const DuelResult result = runDuelScenario(
        makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), armedSettings());
    ASSERT_TRUE(result.completed()) << (result.mErrors.empty() ? "" : result.mErrors.front());
    EXPECT_EQ(result.mFrames.size(), result.mStepCount);
    EXPECT_GT(result.mLeftHits + result.mRightHits, 0);
    EXPECT_GT(result.mLeftDamageDealt + result.mRightDamageDealt, 0.0f);
    for (const DuelFrame& frame : result.mFrames)
    {
        EXPECT_TRUE(std::isfinite(frame.mLeft.mHealth));
        EXPECT_TRUE(std::isfinite(frame.mRight.mHealth));
        ASSERT_EQ(frame.mLeft.mBodyPositions.size(), result.mBodyCount[0]);
        ASSERT_EQ(frame.mRight.mBodyPositions.size(), result.mBodyCount[1]);
    }
}

TEST(RagdollCoreDuelScenarioTest, SwingExtendsTheArmForward)
{
    DuelSettings settings = armedSettings();
    settings.mRight.mAttacksEnabled = false; // one attacker only, no interference
    const DuelResult result = runDuelScenario(
        makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), settings);
    ASSERT_TRUE(result.completed());
    const auto hand = std::find(result.mBodyNames[0].begin(), result.mBodyNames[0].end(), "right_hand");
    ASSERT_NE(hand, result.mBodyNames[0].end());
    const std::size_t handIndex = static_cast<std::size_t>(std::distance(result.mBodyNames[0].begin(), hand));
    float maximumForward = -1.0e9f;
    for (const DuelFrame& frame : result.mFrames)
    {
        const float forward = frame.mLeft.mBodyPositions[handIndex].GetY()
            - static_cast<float>(frame.mLeft.mBodyPositions[0].GetY());
        maximumForward = std::max(maximumForward, forward);
    }
    // With the authored shoulder cone, spring and torque limit intact, the
    // thrust must still produce a clear forward reach. Earlier readings near
    // twenty relied on a spawn-height error or temporary limit inflation.
    EXPECT_GT(maximumForward, 11.5f) << "maximum hand forward offset " << maximumForward;
}

TEST(RagdollCoreDuelScenarioTest, UnarmedCloseRangeConnects)
{
    // The scripted thrust reaches roughly twenty units; keep one unarmed
    // proof at that distance. A proper punch drive is a later milestone.
    DuelSettings settings = armedSettings();
    settings.mSeparation = 18.0f;
    settings.mLeft.mWeapon.mArmed = false;
    settings.mRight.mWeapon.mArmed = false;
    settings.mLeft.mFistMinimumDamage = 2.0f;
    settings.mLeft.mFistMaximumDamage = 6.0f;
    settings.mRight.mFistMinimumDamage = 2.0f;
    settings.mRight.mFistMaximumDamage = 6.0f;
    const DuelResult result = runDuelScenario(
        makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), settings);
    ASSERT_TRUE(result.completed()) << (result.mErrors.empty() ? "" : result.mErrors.front());
    EXPECT_GT(result.mLeftHits + result.mRightHits, 0)
        << "close-range fist hits L=" << result.mLeftHits << " R=" << result.mRightHits;
}

TEST(RagdollCoreDuelScenarioTest, ReactionsRespondToRealHits)
{
    DuelSettings settings = armedSettings();
    settings.mReactionsEnabled = true;
    settings.mBehavior.mEnabled = true;
    // The authored thrust now keeps its production joint limits and torque.
    // Lower only this test's perception thresholds so its real, non-zero
    // contact estimate exercises the transient reaction consumer.
    settings.mBehavior.mFaceGuardImpulse = 1.0f;
    settings.mBehavior.mFaceGuardFullImpulse = 50.0f;
    settings.mBehavior.mDoubledOverImpulse = 1.0f;
    settings.mBehavior.mDoubledOverFullImpulse = 50.0f;
    settings.mLeft.mScriptedWoundSeverity = 0.5f;
    settings.mRight.mScriptedWoundSeverity = 0.5f;
    const DuelResult result = runDuelScenario(
        makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), settings);
    ASSERT_TRUE(result.completed()) << (result.mErrors.empty() ? "" : result.mErrors.front());
    ASSERT_GT(result.mLeftHits + result.mRightHits, 0);

    bool sawBehavior = false;
    bool sawLimbFlag = false;
    for (std::size_t i = 0; i < result.mFrames.size(); ++i)
    {
        const DuelFrame& frame = result.mFrames[i];
        const std::array<const DuelActorFrame*, 2> actors{ &frame.mLeft, &frame.mRight };
        for (const DuelActorFrame* actor : actors)
        {
            if (actor->mBehavior != BehaviorId::None)
                sawBehavior = true;
            if (actor->mCradleLeft || actor->mCradleRight || actor->mLimpLeft || actor->mLimpRight)
                sawLimbFlag = true;
        }
    }
    EXPECT_TRUE(sawBehavior || sawLimbFlag);
    // Authority must stay bounded and finite everywhere.
    for (const DuelFrame& frame : result.mFrames)
    {
        for (const DuelActorFrame* actor : { &frame.mLeft, &frame.mRight })
        {
            EXPECT_GE(actor->mAuthority, 0.0f);
            EXPECT_LE(actor->mAuthority, 1.0f);
            EXPECT_GE(actor->mStability, 0.3f);
            EXPECT_LE(actor->mStability, 1.0f);
        }
    }
}

// Dynamic-root standing: the measured support plan plus bounded upright
// control must hold both fighters on the floor for six seconds. Attacks stay
// off so the gate qualifies balance, not combat endurance. This gate belongs
// to the dynamic-root milestone, so it lives outside the RXN-01 exact set.
TEST(RagdollCoreDuelBalanceTest, KeepsFightersStanding)
{
    DuelSettings settings = armedSettings();
    settings.mPhysicalBalance = true;
    settings.mSeparation = 40.0f;
    settings.mDurationSeconds = 6.0f;
    settings.mLeft.mAttacksEnabled = false;
    settings.mRight.mAttacksEnabled = false;
    const DuelResult result = runDuelScenario(
        makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), settings);
    ASSERT_TRUE(result.completed()) << (result.mErrors.empty() ? "" : result.mErrors.front());
    const auto pelvis = std::find(result.mBodyNames[0].begin(), result.mBodyNames[0].end(), "pelvis");
    ASSERT_NE(pelvis, result.mBodyNames[0].end());
    const std::size_t pelvisIndex = static_cast<std::size_t>(std::distance(result.mBodyNames[0].begin(), pelvis));
    float minimumHeight[2] = { 1.0e9f, 1.0e9f };
    for (const DuelFrame& frame : result.mFrames)
    {
        minimumHeight[0] = std::min(minimumHeight[0], frame.mLeft.mBodyPositions[pelvisIndex].GetZ());
        minimumHeight[1] = std::min(minimumHeight[1], frame.mRight.mBodyPositions[pelvisIndex].GetZ());
    }
    float maximumHeight[2] = { 0.0f, 0.0f };
    std::size_t maximumHeightFrame[2] = { 0, 0 };
    for (std::size_t index = 0; index < result.mFrames.size(); ++index)
    {
        const DuelFrame& frame = result.mFrames[index];
        if (frame.mLeft.mBodyPositions[pelvisIndex].GetZ() > maximumHeight[0])
        {
            maximumHeight[0] = frame.mLeft.mBodyPositions[pelvisIndex].GetZ();
            maximumHeightFrame[0] = index;
        }
        if (frame.mRight.mBodyPositions[pelvisIndex].GetZ() > maximumHeight[1])
        {
            maximumHeight[1] = frame.mRight.mBodyPositions[pelvisIndex].GetZ();
            maximumHeightFrame[1] = index;
        }
    }
    EXPECT_GT(minimumHeight[0], 40.0f) << "left minimum pelvis height " << minimumHeight[0];
    EXPECT_GT(minimumHeight[1], 40.0f) << "right minimum pelvis height " << minimumHeight[1];
    EXPECT_LT(maximumHeight[0], 120.0f) << "left maximum pelvis height " << maximumHeight[0]
                                         << " at frame " << maximumHeightFrame[0] << " of " << result.mFrames.size();
    EXPECT_LT(maximumHeight[1], 120.0f) << "right maximum pelvis height " << maximumHeight[1]
                                         << " at frame " << maximumHeightFrame[1] << " of " << result.mFrames.size();
    EXPECT_TRUE(result.mFrames.back().mLeft.mAlive);
    EXPECT_TRUE(result.mFrames.back().mRight.mAlive);
    SUCCEED() << "physical duel hits L=" << result.mLeftHits << " R=" << result.mRightHits;
}

TEST(RagdollCoreDuelScenarioTest, DeadFighterDoesNotFlyAway)
{
    DuelSettings settings = armedSettings();
    settings.mLeft.mHealth = 8.0f; // one clean strike ends the left fighter
    settings.mRight.mHealth = 8.0f;
    settings.mLeft.mWeapon.mMinimumDamage = 0.1f;
    settings.mLeft.mWeapon.mMaximumDamage = 0.1f;
    settings.mRight.mWeapon.mMinimumDamage = 12.0f;
    settings.mRight.mWeapon.mMaximumDamage = 40.0f;
    const DuelResult result = runDuelScenario(
        makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), settings);
    ASSERT_TRUE(result.completed());
    ASSERT_TRUE(result.mLeftDead || result.mRightDead)
        << "hits L=" << result.mLeftHits << " R=" << result.mRightHits
        << " damage L=" << result.mLeftDamageDealt << " R=" << result.mRightDamageDealt;
    const bool leftDead = result.mLeftDead;
    const std::size_t frames = result.mFrames.size();
    std::optional<JPH::Vec3> deathPosition;
    for (std::size_t i = 0; i < frames; ++i)
    {
        const DuelActorFrame& actor = leftDead ? result.mFrames[i].mLeft : result.mFrames[i].mRight;
        if (!actor.mAlive && !deathPosition)
            deathPosition = actor.mBodyPositions.empty() ? std::optional<JPH::Vec3>{}
                                                         : std::optional<JPH::Vec3>(actor.mBodyPositions[0]);
    }
    ASSERT_TRUE(deathPosition.has_value());
    const DuelActorFrame& finalActor = leftDead ? result.mFrames.back().mLeft : result.mFrames.back().mRight;
    const JPH::Vec3 delta = finalActor.mBodyPositions[0] - *deathPosition;
    const float travel = JPH::Vec3(delta.GetX(), delta.GetY(), 0.0f).Length();
    EXPECT_LT(travel, 60.0f) << "dead fighter travelled " << travel << " units horizontally"
        << " leftDead=" << leftDead
        << " deathPos=" << deathPosition->GetX() << "," << deathPosition->GetY() << "," << deathPosition->GetZ()
        << " finalPos=" << finalActor.mBodyPositions[0].GetX() << ","
        << finalActor.mBodyPositions[0].GetY() << "," << finalActor.mBodyPositions[0].GetZ();
}

TEST(RagdollCoreDuelScenarioTest, DeterministicRepeat)
{
    const DuelResult first = runDuelScenario(
        makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), armedSettings());
    const DuelResult second = runDuelScenario(
        makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), armedSettings());
    ASSERT_TRUE(first.completed());
    ASSERT_TRUE(second.completed());
    EXPECT_EQ(hashResult(first), hashResult(second));
    EXPECT_EQ(first.mLeftHits, second.mLeftHits);
    EXPECT_EQ(first.mRightHits, second.mRightHits);
    EXPECT_FLOAT_EQ(first.mLeftDamageDealt, second.mLeftDamageDealt);
    EXPECT_FLOAT_EQ(first.mRightDamageDealt, second.mRightDamageDealt);
}

TEST(RagdollCoreDuelScenarioTest, RejectsInvalidSettings)
{
    DuelSettings settings = armedSettings();
    settings.mDurationSeconds = 0.0f;
    const DuelResult result = runDuelScenario(
        makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), settings);
    EXPECT_FALSE(result.completed());
    EXPECT_FALSE(result.mErrors.empty());

    settings = armedSettings();
    settings.mLeft.mScriptedWoundSeverity = 1.1f;
    const DuelResult badSeverity = runDuelScenario(
        makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), settings);
    EXPECT_FALSE(badSeverity.completed());
    EXPECT_FALSE(badSeverity.mErrors.empty());
}

namespace
{
    std::string viewerNumber(float value)
    {
        std::ostringstream out;
        out.setf(std::ios::fixed);
        out.precision(3);
        out << (std::isfinite(value) ? value : 0.0f);
        return out.str();
    }
}

TEST(RagdollCoreDuelScenarioTest, DISABLED_writesDuelViewerTrace)
{
    const char* outputPath = std::getenv("OPENMW_RAGDOLL_DUEL_VIEWER");
    if (outputPath == nullptr || *outputPath == '\0')
        GTEST_SKIP() << "set OPENMW_RAGDOLL_DUEL_VIEWER to an output json path";
    DuelSettings settings = armedSettings();
    settings.mReactionsEnabled = true;
    settings.mBehavior.mEnabled = true;
    // Showcase tuning: a longer exchange than the test defaults.
    settings.mLeft.mHealth = settings.mRight.mHealth = 40.0f;
    settings.mLeft.mWeapon.mMinimumDamage = settings.mRight.mWeapon.mMinimumDamage = 2.0f;
    settings.mLeft.mWeapon.mMaximumDamage = settings.mRight.mWeapon.mMaximumDamage = 6.0f;
    const DuelResult result = runDuelScenario(
        makeScenarioHumanoidSkeleton(), makeScenarioHumanoidProfile(), settings);
    ASSERT_TRUE(result.completed()) << (result.mErrors.empty() ? "" : result.mErrors.front());

    std::ostringstream json;
    json << "{\n\"bodyNames\":[";
    for (std::size_t i = 0; i < result.mBodyNames[0].size(); ++i)
        json << (i ? "," : "") << "\"" << result.mBodyNames[0][i] << "\"";
    json << "],\n\"parents\":[";
    for (std::size_t i = 0; i < result.mBodyParents[0].size(); ++i)
        json << (i ? "," : "") << result.mBodyParents[0][i];
    json << "],\n\"frames\":[";
    for (std::size_t f = 0; f < result.mFrames.size(); ++f)
    {
        const DuelFrame& frame = result.mFrames[f];
        json << (f ? "," : "") << "\n{\"t\":" << viewerNumber(frame.mTime);
        for (int actor = 0; actor < 2; ++actor)
        {
            const DuelActorFrame& row = actor == 0 ? frame.mLeft : frame.mRight;
            json << (actor == 0 ? ",\"left\":{" : ",\"right\":{");
            json << "\"alive\":" << (row.mAlive ? 1 : 0)
                 << ",\"health\":" << viewerNumber(row.mHealth)
                 << ",\"behavior\":\"" << behaviorName(row.mBehavior) << "\""
                 << ",\"swinging\":" << (row.mSwinging ? 1 : 0)
                 << ",\"hit\":" << (row.mHitThisStep ? 1 : 0)
                 << ",\"region\":\"" << regionName(row.mLatestRegion) << "\""
                 << ",\"bodies\":[";
            for (std::size_t b = 0; b < row.mBodyPositions.size(); ++b)
            {
                const JPH::Vec3& point = row.mBodyPositions[b];
                json << (b ? "," : "") << "[" << viewerNumber(point.GetX()) << ","
                     << viewerNumber(point.GetY()) << "," << viewerNumber(point.GetZ()) << "]";
            }
            json << "]";
            if (row.mWeaponGrip && row.mWeaponTip)
                json << ",\"grip\":[" << viewerNumber(row.mWeaponGrip->GetX()) << ","
                     << viewerNumber(row.mWeaponGrip->GetY()) << "," << viewerNumber(row.mWeaponGrip->GetZ())
                     << "],\"tip\":[" << viewerNumber(row.mWeaponTip->GetX()) << ","
                     << viewerNumber(row.mWeaponTip->GetY()) << "," << viewerNumber(row.mWeaponTip->GetZ()) << "]";
            json << "}";
        }
        json << "}";
    }
    json << "]}\n";

    std::ofstream stream(outputPath, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(stream.is_open()) << "cannot write " << outputPath;
    stream << json.str();
    stream.close();
    SUCCEED() << "duel viewer trace written to " << outputPath;
}

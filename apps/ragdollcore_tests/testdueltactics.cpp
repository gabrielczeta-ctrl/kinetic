#include <gtest/gtest.h>

#include <ragdollcore/dueltactics.hpp>

#include <algorithm>
#include <cmath>

namespace
{
    using namespace RagdollCore;

    constexpr float sStep = 1.0f / 60.0f;

    DuelOpponentView calmOpponent(float distance)
    {
        DuelOpponentView view;
        view.mDistance = distance;
        view.mReach = 40.0f;
        return view;
    }

    DuelSelfView fighter(float aggression = 0.0f, float skill = 1.0f)
    {
        DuelSelfView self;
        self.mReach = 40.0f;
        self.mAggression = aggression;
        self.mSkill = skill;
        return self;
    }

    // Runs until the tactics ask to attack or the time is up; returns the
    // seconds it took (or a negative value).
    float secondsUntilAttack(DuelTacticsState& state, const DuelSelfView& self, DuelOpponentView opponent,
        float seconds, bool closeOnStepIn = true)
    {
        for (float t = 0.0f; t < seconds; t += sStep)
        {
            const auto out = stepDuelTactics(state, self, opponent, sStep);
            if (out.mAttack)
                return t;
            if (closeOnStepIn && out.mPhase == DuelPhase::StepIn)
                opponent.mDistance = std::max(35.0f, opponent.mDistance - 200.0f * sStep);
        }
        return -1.0f;
    }
}

TEST(RagdollCoreDuelTacticsTest, farOpponentIsApproachedRunningWithoutAttacking)
{
    DuelTacticsState state;
    const auto out = stepDuelTactics(state, fighter(), calmOpponent(600.0f), sStep);
    EXPECT_EQ(out.mPhase, DuelPhase::Approach);
    EXPECT_FLOAT_EQ(out.mForward, 1.0f);
    EXPECT_TRUE(out.mRun);
    EXPECT_FALSE(out.mAttack);
}

TEST(RagdollCoreDuelTacticsTest, holdsAtTheEdgeOfReachInsteadOfCharging)
{
    DuelTacticsState state;
    stepDuelTactics(state, fighter(), calmOpponent(80.0f), sStep);
    // Hold distance: the longer reach (40) plus the margin (14) = 54.
    const auto atHold = stepDuelTactics(state, fighter(), calmOpponent(54.0f), sStep);
    EXPECT_EQ(atHold.mPhase, DuelPhase::Measure);
    EXPECT_NEAR(atHold.mForward, 0.0f, 1e-4f);
    EXPECT_FALSE(atHold.mAttack);
    const auto tooClose = stepDuelTactics(state, fighter(), calmOpponent(30.0f), sStep);
    EXPECT_LT(tooClose.mForward, 0.0f);
    // An opponent stepping in a little is not run from.
    const auto stepped = stepDuelTactics(state, fighter(), calmOpponent(46.0f), sStep);
    EXPECT_FLOAT_EQ(stepped.mForward, 0.0f);
}

TEST(RagdollCoreDuelTacticsTest, withoutAnOpeningACalmFighterWaitsSeconds)
{
    DuelTacticsState state;
    state.mSeed = 7;
    stepDuelTactics(state, fighter(0.0f), calmOpponent(54.0f), sStep);
    const float waited = secondsUntilAttack(state, fighter(0.0f), calmOpponent(54.0f), 20.0f);
    // Patience at aggression 0: 3.5 s x 0.6-1.4.
    EXPECT_GE(waited, 2.0f);
    EXPECT_LE(waited, 6.0f);
}

TEST(RagdollCoreDuelTacticsTest, aFierceFighterWaitsLess)
{
    DuelTacticsState calm, fierce;
    calm.mSeed = fierce.mSeed = 11;
    const float calmWait = secondsUntilAttack(calm, fighter(0.0f), calmOpponent(54.0f), 20.0f);
    const float fierceWait = secondsUntilAttack(fierce, fighter(1.0f), calmOpponent(54.0f), 20.0f);
    EXPECT_GT(fierceWait, 0.0f);
    EXPECT_LT(fierceWait, calmWait);
}

TEST(RagdollCoreDuelTacticsTest, anOpponentRecoveringFromItsSwingIsPunishedQuickly)
{
    DuelTacticsState state;
    stepDuelTactics(state, fighter(0.0f, 1.0f), calmOpponent(54.0f), sStep);
    DuelOpponentView recovering = calmOpponent(54.0f);
    recovering.mSinceSwing = 0.1f;
    const float waited = secondsUntilAttack(state, fighter(0.0f, 1.0f), recovering, 3.0f);
    EXPECT_GE(waited, 0.1f);
    EXPECT_LT(waited, 0.6f);
}

TEST(RagdollCoreDuelTacticsTest, aSlowReaderSeesTheSameOpeningLater)
{
    DuelTacticsState quick, slow;
    DuelOpponentView bounced = calmOpponent(54.0f);
    bounced.mSinceBounce = 0.0f;
    const float quickWait = secondsUntilAttack(quick, fighter(0.0f, 1.0f), bounced, 3.0f);
    const float slowWait = secondsUntilAttack(slow, fighter(0.0f, 0.0f), bounced, 3.0f);
    EXPECT_GT(slowWait, quickWait);
}

TEST(RagdollCoreDuelTacticsTest, anOpponentThatCannotBlockIsStruckQuickly)
{
    DuelTacticsState state;
    stepDuelTactics(state, fighter(0.0f, 1.0f), calmOpponent(54.0f), sStep);
    DuelOpponentView broken = calmOpponent(54.0f);
    broken.mCannotBlock = true;
    const float waited = secondsUntilAttack(state, fighter(0.0f, 1.0f), broken, 3.0f);
    EXPECT_GE(waited, 0.1f);
    EXPECT_LT(waited, 0.6f);
}

TEST(RagdollCoreDuelTacticsTest, givesGroundWhenItsGuardIsBroken)
{
    DuelTacticsState state;
    stepDuelTactics(state, fighter(), calmOpponent(54.0f), sStep);
    DuelSelfView broken = fighter();
    broken.mGuardBroken = true;
    const auto out = stepDuelTactics(state, broken, calmOpponent(54.0f), sStep);
    EXPECT_EQ(out.mPhase, DuelPhase::StepOut);
    EXPECT_FALSE(out.mAttack);
}

TEST(RagdollCoreDuelTacticsTest, givesGroundToAnIncomingBlade)
{
    DuelTacticsState state;
    stepDuelTactics(state, fighter(), calmOpponent(54.0f), sStep);
    DuelOpponentView swinging = calmOpponent(50.0f);
    swinging.mSwinging = true;
    swinging.mSinceSwing = 0.0f;
    const auto out = stepDuelTactics(state, fighter(), swinging, sStep);
    EXPECT_LT(out.mForward, 0.0f);
    EXPECT_FALSE(out.mAttack);
}

TEST(RagdollCoreDuelTacticsTest, stepsOutAfterItsStrike)
{
    DuelTacticsState state;
    DuelOpponentView recovering = calmOpponent(54.0f);
    recovering.mSinceSwing = 0.1f;
    ASSERT_GE(secondsUntilAttack(state, fighter(0.0f, 1.0f), recovering, 3.0f), 0.0f);
    DuelSelfView attacking = fighter(0.0f, 1.0f);
    attacking.mAttacking = true;
    DuelOpponentView close = calmOpponent(38.0f);
    stepDuelTactics(state, attacking, close, sStep);
    const auto after = stepDuelTactics(state, fighter(0.0f, 1.0f), close, sStep);
    EXPECT_EQ(after.mPhase, DuelPhase::StepOut);
    EXPECT_FLOAT_EQ(after.mForward, -1.0f);
    EXPECT_FALSE(after.mAttack);
}

TEST(RagdollCoreDuelTacticsTest, aDownedOpponentIsGivenTimeToRise)
{
    // A calm fighter (below mStrikeDownedAggression); fierce ones press in.
    DuelTacticsState state;
    stepDuelTactics(state, fighter(0.0f), calmOpponent(54.0f), sStep);
    DuelOpponentView down = calmOpponent(40.0f);
    down.mDown = true;
    for (int i = 0; i < 600; ++i)
    {
        const auto out = stepDuelTactics(state, fighter(0.0f), down, sStep);
        ASSERT_FALSE(out.mAttack);
        ASSERT_EQ(out.mPhase, DuelPhase::Recover);
    }
    const auto up = stepDuelTactics(state, fighter(0.0f), calmOpponent(54.0f), sStep);
    EXPECT_EQ(up.mPhase, DuelPhase::Measure);
}

TEST(RagdollCoreDuelTacticsTest, aTiredFighterCatchesItsBreathUntilRested)
{
    DuelTacticsState state;
    DuelSelfView tired = fighter(1.0f);
    tired.mFatigue = 0.2f;
    auto out = stepDuelTactics(state, tired, calmOpponent(54.0f), sStep);
    EXPECT_EQ(out.mPhase, DuelPhase::Recover);
    tired.mFatigue = 0.4f; // above the tired line, below rested
    out = stepDuelTactics(state, tired, calmOpponent(54.0f), sStep);
    EXPECT_EQ(out.mPhase, DuelPhase::Recover);
    tired.mFatigue = 0.6f;
    out = stepDuelTactics(state, tired, calmOpponent(54.0f), sStep);
    EXPECT_EQ(out.mPhase, DuelPhase::Measure);
}

// Alpha 0.4 item 4: an opponent backing away is run down and lunged at.
DuelTacticsSettings chaseOn()
{
    DuelTacticsSettings settings;
    settings.mRetreatSpeed = 45.0f;
    return settings;
}

TEST(RagdollCoreDuelTacticsTest, aRetreatingOpponentIsChasedAndLungedAt)
{
    DuelTacticsState state;
    DuelOpponentView opponent = calmOpponent(54.0f);
    // At measure first.
    for (int i = 0; i < 30; ++i)
        stepDuelTactics(state, fighter(), opponent, sStep, chaseOn());
    ASSERT_EQ(state.mPhase, DuelPhase::Measure);
    // The opponent backs away at 150 u/s; the fighter follows at 250 u/s when running.
    bool ran = false, lunged = false;
    float lungeGap = -1.0f;
    for (int i = 0; i < 240 && !lunged; ++i)
    {
        const auto out = stepDuelTactics(state, fighter(), opponent, sStep, chaseOn());
        opponent.mDistance += 150.0f * sStep;
        if (out.mPhase == DuelPhase::Chase && out.mRun)
        {
            ran = true;
            opponent.mDistance -= 250.0f * sStep;
        }
        else
            opponent.mDistance -= 100.0f * out.mForward * sStep;
        if (out.mAttack)
        {
            lunged = true;
            lungeGap = opponent.mDistance;
        }
    }
    EXPECT_TRUE(ran);
    EXPECT_TRUE(lunged);
    EXPECT_LE(lungeGap, 40.0f + 30.0f + 5.0f);
}

TEST(RagdollCoreDuelTacticsTest, aStillOpponentIsNotChased)
{
    DuelTacticsState state;
    for (int i = 0; i < 240; ++i)
    {
        const auto out = stepDuelTactics(state, fighter(), calmOpponent(54.0f), sStep, chaseOn());
        EXPECT_NE(out.mPhase, DuelPhase::Chase);
    }
}

TEST(RagdollCoreDuelTacticsTest, itsOwnStepOutIsNotAChase)
{
    DuelTacticsState state;
    state.mPhase = DuelPhase::StepOut;
    DuelOpponentView opponent = calmOpponent(40.0f);
    for (int i = 0; i < 30; ++i)
    {
        const auto out = stepDuelTactics(state, fighter(), opponent, sStep, chaseOn());
        opponent.mDistance -= 100.0f * out.mForward * sStep;
        EXPECT_NE(out.mPhase, DuelPhase::Chase);
    }
}

TEST(RagdollCoreDuelTacticsTest, aFierceFighterStrikesADownedOpponent)
{
    DuelTacticsState state;
    DuelOpponentView opponent = calmOpponent(54.0f);
    for (int i = 0; i < 30; ++i)
        stepDuelTactics(state, fighter(0.8f), opponent, sStep);
    opponent.mDown = true;
    EXPECT_GE(secondsUntilAttack(state, fighter(0.8f), opponent, 3.0f), 0.0f);
}

// Bookmark 8: fighters circle instead of standing chest to chest.
TEST(RagdollCoreDuelTacticsTest, circlesWhileMeasuringAndUnlocksWhenPressed)
{
    DuelTacticsState state;
    bool strafed = false;
    for (int i = 0; i < 120; ++i)
    {
        const auto out = stepDuelTactics(state, fighter(), calmOpponent(54.0f), sStep);
        strafed |= out.mPhase == DuelPhase::Measure && out.mStrafe != 0.0f;
    }
    EXPECT_TRUE(strafed);
    const auto pressed = stepDuelTactics(state, fighter(), calmOpponent(3.0f), sStep);
    EXPECT_LT(pressed.mForward, 0.0f);
    EXPECT_NE(pressed.mStrafe, 0.0f);
}

// User bookmark 13 (2026-09-30): "they should be able to sidestep and dodge the
// player attacks". A skilled fighter inside a swinging opponent's reach backs
// off AND side-steps, over many seeds both sides; also while stepping in.
TEST(RagdollCoreDuelTacticsTest, aReadSwingIsSideStepped)
{
    int left = 0, right = 0, dodges = 0;
    for (std::uint32_t seed = 1; seed <= 40; ++seed)
    {
        DuelTacticsState state;
        state.mSeed = seed;
        DuelOpponentView opponent = calmOpponent(48.0f);
        const DuelSelfView self = fighter(0.0f, 1.0f);
        for (int i = 0; i < 30; ++i)
            stepDuelTactics(state, self, opponent, sStep);
        opponent.mSwinging = true;
        const auto out = stepDuelTactics(state, self, opponent, sStep);
        if (out.mReason == "side-step")
        {
            ++dodges;
            EXPECT_LT(out.mForward, 0.0f);
            (out.mStrafe < 0.0f ? left : right) += 1;
        }
    }
    EXPECT_GE(dodges, 20); // skill 1: 75% of read swings
    EXPECT_GT(left, 0);
    EXPECT_GT(right, 0);
}

TEST(RagdollCoreDuelTacticsTest, aSwingDuringTheStepInIsDodgedBeforeItsOwnAttack)
{
    int dodges = 0;
    for (std::uint32_t seed = 1; seed <= 40; ++seed)
    {
        DuelTacticsState state;
        state.mSeed = seed;
        state.mPhase = DuelPhase::StepIn;
        DuelOpponentView opponent = calmOpponent(55.0f);
        opponent.mSwinging = true;
        const auto out = stepDuelTactics(state, fighter(0.0f, 1.0f), opponent, sStep);
        if (out.mReason == "side-step")
            ++dodges;
    }
    EXPECT_GE(dodges, 20);
}

// User bookmark 10 (2026-09-30): a fighter that has just got up steadies
// before attacking or pressing in again.
TEST(RagdollCoreDuelTacticsTest, aFighterJustUpSteadiesBeforeAttacking)
{
    DuelTacticsState state;
    DuelOpponentView opponent = calmOpponent(35.0f);
    DuelSelfView self = fighter(1.0f, 1.0f);
    self.mSinceDown = 0.3f;
    for (int i = 0; i < 30; ++i)
    {
        const auto out = stepDuelTactics(state, self, opponent, sStep);
        EXPECT_FALSE(out.mAttack);
        EXPECT_LE(out.mForward, 0.0f);
    }
    self.mSinceDown = 5.0f;
    EXPECT_GE(secondsUntilAttack(state, self, opponent, 6.0f), 0.0f);
}

TEST(RagdollCoreDuelTacticsTest, duelStatsTurnTheGameStatsIntoFighterTerms)
{
    RagdollCore::DuelStatsInput master;
    master.mWeaponSkill = 90.0f;
    master.mBlock = 80.0f;
    master.mShield = true;
    master.mAgility = 80.0f;
    master.mLuck = 60.0f;
    master.mSpeed = 70.0f;
    master.mWillpower = 70.0f;
    master.mFight = 90.0f;
    RagdollCore::DuelStatsInput novice;
    novice.mWeaponSkill = 15.0f;
    novice.mBlock = 5.0f;
    novice.mAgility = 30.0f;
    novice.mLuck = 30.0f;
    novice.mSpeed = 30.0f;
    novice.mWillpower = 30.0f;
    novice.mFight = 30.0f;
    novice.mFlee = 60.0f;
    const auto a = RagdollCore::duelStats(master);
    const auto b = RagdollCore::duelStats(novice);
    EXPECT_GT(a.mAttack, 0.9f); // capped at 1
    EXPECT_LT(b.mAttack, 0.3f);
    EXPECT_GT(a.mGuard, b.mGuard + 0.5f);
    EXPECT_GT(a.mNerve, 0.9f);
    EXPECT_LT(b.mNerve, 0.4f);
    EXPECT_GT(a.mFootwork, b.mFootwork);
    // Exhaustion takes the skill away.
    master.mFatigueTerm = 0.5f;
    EXPECT_LT(RagdollCore::duelStats(master).mAttack, 0.65f);
    // Without a shield the guard leans on the weapon skill too.
    novice.mShield = false;
    novice.mWeaponSkill = 60.0f;
    EXPECT_NEAR(RagdollCore::duelStats(novice).mGuard, (32.5f + 6.0f + 3.0f) / 100.0f, 1e-4f);
}

// AI-02 (docs/ragdoll/designs/AI-02-smart-duel-ai.md): items 1a, 2, 3 and 5.
namespace
{
    // A fixed charge: 80 units out (at measure, inside hold + 150), closing at
    // `closing` u/s, its weapon not raised.
    DuelOpponentView charger(float closing = 250.0f)
    {
        DuelOpponentView view = calmOpponent(80.0f);
        view.mClosing = closing;
        return view;
    }

    DuelSelfView duellist(float nerve, float attack, float footwork)
    {
        DuelSelfView self = fighter(0.0f, 1.0f);
        self.mNerve = nerve;
        self.mAttackSkill = attack;
        self.mFootwork = footwork;
        return self;
    }

    // secondsUntilAttack with settings.
    float secondsUntilAttackWith(DuelTacticsState& state, const DuelSelfView& self, DuelOpponentView opponent,
        float seconds, const DuelTacticsSettings& settings)
    {
        for (float t = 0.0f; t < seconds; t += sStep)
        {
            const auto out = stepDuelTactics(state, self, opponent, sStep, settings);
            if (out.mAttack)
                return t;
            if (out.mPhase == DuelPhase::StepIn)
                opponent.mDistance = std::max(35.0f, opponent.mDistance - 200.0f * sStep);
        }
        return -1.0f;
    }

    DuelTacticsSettings threatOn()
    {
        DuelTacticsSettings settings;
        settings.mThreat = true;
        return settings;
    }

    // Settles at measure, then faces the charge until it is read; returns the
    // read step's output (mThreatRead empty: never read).
    DuelTacticsOutput readTheCharge(std::uint32_t seed, const DuelSelfView& self, const DuelOpponentView& charge)
    {
        DuelTacticsState state;
        state.mSeed = (seed * 2654435761u) | 1u;
        for (int i = 0; i < 30; ++i)
            stepDuelTactics(state, self, calmOpponent(80.0f), sStep, threatOn());
        for (int i = 0; i < 60; ++i)
        {
            const auto out = stepDuelTactics(state, self, charge, sStep, threatOn());
            if (!out.mThreatRead.empty())
                return out;
        }
        return {};
    }

    struct ChargeResponses
    {
        int mCutOff = 0, mSideStep = 0, mBackpedal = 0, mNone = 0;
    };

    // Over 100 seeds, checking each response's own output on its read step.
    ChargeResponses responsesOf(const DuelSelfView& self)
    {
        ChargeResponses count;
        for (std::uint32_t seed = 1; seed <= 100; ++seed)
        {
            const auto out = readTheCharge(seed, self, charger());
            if (out.mThreatRead == "cut-off")
            {
                ++count.mCutOff;
                EXPECT_TRUE(out.mAttack); // 80 <= reach 40 + 250 x lead 0.45
                EXPECT_FLOAT_EQ(out.mForward, 0.0f);
                EXPECT_EQ(out.mReason, "cut off");
            }
            else if (out.mThreatRead == "side-step")
            {
                ++count.mSideStep;
                EXPECT_FLOAT_EQ(out.mForward, -0.3f);
                EXPECT_NE(out.mStrafe, 0.0f);
            }
            else if (out.mThreatRead == "backpedal")
            {
                ++count.mBackpedal;
                EXPECT_FLOAT_EQ(out.mForward, -1.0f);
                EXPECT_FALSE(out.mRun); // walks back, never runs backwards
                EXPECT_TRUE(out.mGuard);
            }
            else
                ++count.mNone;
        }
        return count;
    }
}

// AI-02 item 2: a fierce skilled fighter cuts a charge off, a timid one
// backpedals, an agile one side-steps.
TEST(RagdollCoreDuelTacticsTest, aChargeIsAnsweredByTheFightersStats)
{
    const auto fierce = responsesOf(duellist(1.0f, 1.0f, 0.1f));
    EXPECT_EQ(fierce.mNone, 0);
    EXPECT_GE(fierce.mCutOff, 50); // 0.6 / (0.6 + 0.082 + 0.2): 68%
    EXPECT_GT(fierce.mCutOff, fierce.mBackpedal);
    const auto timid = responsesOf(duellist(0.1f, 0.3f, 0.1f));
    EXPECT_EQ(timid.mCutOff, 0);
    EXPECT_GE(timid.mBackpedal, 75); // 92%
    const auto agile = responsesOf(duellist(0.9f, 0.2f, 1.0f));
    EXPECT_GE(agile.mSideStep, 50); // 0.82 / (0.1 + 0.82 + 0.28): 68%
    EXPECT_GT(agile.mSideStep, agile.mBackpedal);
}

TEST(RagdollCoreDuelTacticsTest, aWeakChargeIsNotAnswered)
{
    // A walking step-in (125 u/s, under the 180 charge speed) is no charge;
    // with the speed lowered to 100 it is one, T = 125/250 x 0.6 = 0.3 < 0.35:
    // read once, no response.
    DuelTacticsState state;
    const DuelSelfView self = duellist(0.1f, 1.0f, 0.1f);
    {
        DuelTacticsState walking;
        for (int i = 0; i < 90; ++i)
            EXPECT_TRUE(stepDuelTactics(walking, self, charger(125.0f), sStep, threatOn()).mThreatRead.empty());
    }
    DuelTacticsSettings slow = threatOn();
    slow.mChargeSpeed = 100.0f;
    for (int i = 0; i < 30; ++i)
        stepDuelTactics(state, self, calmOpponent(80.0f), sStep, slow);
    int reads = 0;
    for (int i = 0; i < 90; ++i)
    {
        const auto out = stepDuelTactics(state, self, charger(125.0f), sStep, slow);
        if (!out.mThreatRead.empty())
        {
            ++reads;
            EXPECT_EQ(out.mThreatRead, "none");
            EXPECT_NEAR(out.mThreat, 0.3f, 1e-4f);
        }
        EXPECT_FALSE(out.mGuard);
        EXPECT_NE(out.mReason, "cut off");
        EXPECT_NE(out.mReason, "side-step charge");
        EXPECT_NE(out.mReason, "backpedal");
    }
    EXPECT_EQ(reads, 1); // past the 1.2 s limit it waits for the charge to stop: one read
}

// AI-02 item 3: stamina pacing.
DuelTacticsSettings staminaOn()
{
    DuelTacticsSettings settings;
    settings.mStamina = true;
    settings.mStaminaPace = 1.5f;
    settings.mStaminaExploit = 0.3f;
    return settings;
}

TEST(RagdollCoreDuelTacticsTest, patienceGrowsWithFatigue)
{
    DuelTacticsState fresh, tiring;
    fresh.mSeed = tiring.mSeed = 5;
    DuelSelfView rested = fighter(0.0f), tired = fighter(0.0f);
    tired.mFatigue = 0.5f;
    // Step 1 enters measure, step 2 draws its patience.
    bool paceRead = false;
    for (int i = 0; i < 2; ++i)
    {
        stepDuelTactics(fresh, rested, calmOpponent(54.0f), sStep, staminaOn());
        paceRead = stepDuelTactics(tiring, tired, calmOpponent(54.0f), sStep, staminaOn()).mPaceRead;
    }
    EXPECT_TRUE(paceRead);
    ASSERT_EQ(fresh.mPhase, DuelPhase::Measure);
    // x (1 + 1.5 x (1 - 0.5)) = x 1.75 (both have waited one step since).
    EXPECT_NEAR(tiring.mPatience + sStep, 1.75f * (fresh.mPatience + sStep), 1e-3f);
}

TEST(RagdollCoreDuelTacticsTest, belowFatigueHalfNoPatienceStrikes)
{
    DuelSelfView self = fighter(1.0f);
    self.mFatigue = 0.4f; // above the tired line (0.25 - 0.1 x 0.5), below 0.45
    DuelTacticsState paced, old;
    paced.mSeed = old.mSeed = 3;
    EXPECT_LT(secondsUntilAttackWith(paced, self, calmOpponent(54.0f), 20.0f, staminaOn()), 0.0f);
    // AI_STAMINA off: the fierce fighter's patience runs out as before.
    EXPECT_GE(secondsUntilAttack(old, self, calmOpponent(54.0f), 20.0f), 0.0f);
}

TEST(RagdollCoreDuelTacticsTest, anExhaustedOpponentIsAnOpening)
{
    DuelOpponentView spent = calmOpponent(54.0f);
    spent.mFatigue = 0.2f; // below the exploit line 0.3
    DuelTacticsState paced, old;
    paced.mSeed = old.mSeed = 9;
    const float pacedWait = secondsUntilAttackWith(paced, fighter(0.0f, 1.0f), spent, 6.0f, staminaOn());
    EXPECT_GE(pacedWait, 0.1f);
    EXPECT_LT(pacedWait, 0.6f);
    // Off: a calm fighter waits its patience (>= 2.1 s).
    EXPECT_GE(secondsUntilAttack(old, fighter(0.0f, 1.0f), spent, 6.0f), 2.0f);
}

TEST(RagdollCoreDuelTacticsTest, aSpentOpponentIsPressedLikeADownedOne)
{
    DuelTacticsState state;
    const DuelSelfView self = fighter(0.8f, 0.0f); // slow reader: an opening takes 0.45 s
    for (int i = 0; i < 30; ++i)
        stepDuelTactics(state, self, calmOpponent(54.0f), sStep, staminaOn());
    ASSERT_EQ(state.mPhase, DuelPhase::Measure);
    DuelOpponentView tiring = calmOpponent(54.0f);
    tiring.mFatigue = 0.2f; // an opening only: still measuring after one step
    EXPECT_EQ(stepDuelTactics(state, self, tiring, sStep, staminaOn()).mPhase, DuelPhase::Measure);
    tiring.mFatigue = 0.05f; // spent: pressed at once
    EXPECT_EQ(stepDuelTactics(state, self, tiring, sStep, staminaOn()).mPhase, DuelPhase::StepIn);
}

// AI-02 item 5: a riposte right after its own parry won.
DuelTacticsSettings riposteOn(float chance = 1.0f)
{
    DuelTacticsSettings settings;
    settings.mRiposteWindow = 0.6f;
    settings.mRiposteChance0 = settings.mRiposteChance100 = chance;
    return settings;
}

TEST(RagdollCoreDuelTacticsTest, aParryWonInReachIsRipostedAtOnce)
{
    DuelTacticsState state;
    DuelSelfView self = fighter(0.0f, 1.0f);
    for (int i = 0; i < 30; ++i)
        stepDuelTactics(state, self, calmOpponent(54.0f), sStep, riposteOn());
    ASSERT_EQ(state.mPhase, DuelPhase::Measure);
    self.mSinceParryWon = 0.05f;
    const auto out = stepDuelTactics(state, self, calmOpponent(50.0f), sStep, riposteOn());
    EXPECT_EQ(out.mReason, "riposte");
    EXPECT_TRUE(out.mAttack);
    EXPECT_FLOAT_EQ(out.mForward, 0.5f);
    EXPECT_EQ(out.mPhase, DuelPhase::StepIn);
    EXPECT_EQ(out.mRiposteRead, "yes");
    // Its attack runs, then it steps out as after any strike.
    self.mAttacking = true;
    self.mSinceParryWon += sStep;
    stepDuelTactics(state, self, calmOpponent(45.0f), sStep, riposteOn());
    self.mAttacking = false;
    self.mSinceParryWon += sStep;
    EXPECT_EQ(stepDuelTactics(state, self, calmOpponent(45.0f), sStep, riposteOn()).mPhase, DuelPhase::StepOut);
}

TEST(RagdollCoreDuelTacticsTest, noRiposteWhenTiredOrOutOfReach)
{
    DuelTacticsState tiredState, farState;
    DuelSelfView tired = fighter(0.0f, 1.0f);
    tired.mFatigue = 0.2f;
    DuelSelfView fresh = fighter(0.0f, 1.0f);
    for (int i = 0; i < 36; ++i)
    {
        tired.mSinceParryWon = fresh.mSinceParryWon = i * sStep;
        const auto a = stepDuelTactics(tiredState, tired, calmOpponent(50.0f), sStep, riposteOn());
        EXPECT_FALSE(a.mAttack);
        EXPECT_NE(a.mReason, "riposte");
        // 90 > reach 40 + 20
        const auto b = stepDuelTactics(farState, fresh, calmOpponent(90.0f), sStep, riposteOn());
        EXPECT_NE(b.mReason, "riposte");
    }
}

TEST(RagdollCoreDuelTacticsTest, theRiposteChanceGrowsWithAttackSkill)
{
    DuelTacticsSettings settings = riposteOn();
    settings.mRiposteChance0 = 0.3f;
    settings.mRiposteChance100 = 0.9f;
    int skilled = 0, unskilled = 0;
    for (std::uint32_t seed = 1; seed <= 100; ++seed)
        for (const float attack : { 0.0f, 1.0f })
        {
            DuelTacticsState state;
            state.mSeed = (seed * 2654435761u) | 1u;
            DuelSelfView self = fighter(0.0f, 1.0f);
            self.mAttackSkill = attack;
            for (int i = 0; i < 30; ++i)
                stepDuelTactics(state, self, calmOpponent(54.0f), sStep, settings);
            self.mSinceParryWon = 0.0f;
            if (stepDuelTactics(state, self, calmOpponent(50.0f), sStep, settings).mReason == "riposte")
                (attack > 0.5f ? skilled : unskilled) += 1;
        }
    EXPECT_GE(skilled, 75); // 90%
    EXPECT_LE(unskilled, 50); // 30%
}

// AI-02 item 1a: its own attack in progress holds the strike distance
// instead of walking into the foe, and is not left early.
TEST(RagdollCoreDuelTacticsTest, anAttackInProgressHoldsTheStrikeDistance)
{
    DuelTacticsSettings aim;
    aim.mAim = true;
    DuelSelfView attacking = fighter(0.0f, 1.0f);
    attacking.mAttacking = true;
    for (const bool on : { true, false })
    {
        DuelTacticsState state;
        state.mPhase = DuelPhase::StepIn;
        for (int i = 0; i < 30; ++i) // the wheel's wind-up and cut, 0.5 s
        {
            // Past the hold (strike 40 - depth 30 - 15).
            const auto out
                = stepDuelTactics(state, attacking, calmOpponent(-10.0f), sStep, on ? aim : DuelTacticsSettings{});
            ASSERT_EQ(out.mPhase, DuelPhase::StepIn); // no step-out while it attacks
            if (on)
                EXPECT_FLOAT_EQ(out.mForward, -0.4f);
            else
                EXPECT_FLOAT_EQ(out.mForward, 0.4f); // the old press
        }
    }
    // At the hold (strike - depth) it stands; beyond it, it follows.
    DuelTacticsState state;
    state.mPhase = DuelPhase::StepIn;
    EXPECT_FLOAT_EQ(stepDuelTactics(state, attacking, calmOpponent(5.0f), sStep, aim).mForward, 0.0f);
    EXPECT_FLOAT_EQ(stepDuelTactics(state, attacking, calmOpponent(20.0f), sStep, aim).mForward, 0.6f);
    // The AI attack over, the wheel's cut still to come: it waits the grace
    // before stepping out.
    DuelSelfView idle = fighter(0.0f, 1.0f);
    int steps = 0;
    while (stepDuelTactics(state, idle, calmOpponent(5.0f), sStep, aim).mPhase == DuelPhase::StepIn && steps < 100)
        ++steps;
    EXPECT_GE(steps * sStep, aim.mAimGrace - 2.0f * sStep);
    EXPECT_LE(steps * sStep, aim.mAimGrace + 2.0f * sStep);
}

TEST(RagdollCoreDuelTacticsTest, theStepInLeadsAClosingFoe)
{
    DuelTacticsSettings aim;
    aim.mAim = true;
    DuelOpponentView closing = calmOpponent(80.0f);
    closing.mClosing = 100.0f; // in reach by 40 + 5 + 100 x 0.45 = 90
    DuelTacticsState on, off;
    on.mPhase = off.mPhase = DuelPhase::StepIn;
    EXPECT_TRUE(stepDuelTactics(on, fighter(), closing, sStep, aim).mAttack);
    EXPECT_FALSE(stepDuelTactics(off, fighter(), closing, sStep).mAttack);
}

// Every AI-02 item off by default: the new inputs change nothing.
TEST(RagdollCoreDuelTacticsTest, theAi02InputsChangeNothingWhileOff)
{
    DuelTacticsState plain, fed;
    plain.mSeed = fed.mSeed = 21;
    const DuelSelfView self = fighter(0.6f, 0.7f);
    DuelSelfView selfFed = self;
    selfFed.mAttackSkill = selfFed.mNerve = selfFed.mFootwork = 1.0f;
    selfFed.mHealth = 0.3f;
    selfFed.mWillpower = selfFed.mEndurance = 1.0f;
    for (int i = 0; i < 900; ++i)
    {
        DuelOpponentView opponent = calmOpponent(60.0f + 50.0f * std::sin(static_cast<float>(i) * 0.02f));
        opponent.mSwinging = (i / 40) % 5 == 0;
        opponent.mWindingUp = (i / 40) % 5 == 4;
        DuelOpponentView opponentFed = opponent;
        opponentFed.mClosing = 250.0f;
        opponentFed.mFatigue = 0.05f;
        selfFed.mSinceParryWon = static_cast<float>(i % 60) * sStep;
        const auto a = stepDuelTactics(plain, self, opponent, sStep);
        const auto b = stepDuelTactics(fed, selfFed, opponentFed, sStep);
        ASSERT_EQ(a.mPhase, b.mPhase) << i;
        ASSERT_EQ(a.mReason, b.mReason) << i;
        ASSERT_EQ(a.mForward, b.mForward) << i;
        ASSERT_EQ(a.mStrafe, b.mStrafe) << i;
        ASSERT_EQ(a.mAttack, b.mAttack) << i;
        ASSERT_EQ(a.mRun, b.mRun) << i;
        ASSERT_FALSE(b.mGuard);
        ASSERT_TRUE(b.mThreatRead.empty());
        ASSERT_FALSE(b.mPaceRead);
        ASSERT_TRUE(b.mRiposteRead.empty());
    }
}

// STEPPING audit item 19: the wind-up is read after the reaction delay and the
// dodge goes away from the blade, with one committed-step burst per swing.
TEST(RagdollCoreDuelTactics, aReadDodgeStepsAwayFromTheBladeOnce)
{
    DuelTacticsSettings settings;
    settings.mDodgeRead = 40.0f;
    settings.mDodgeUnskilled = settings.mDodgeSkilled = 1.0f;
    int dodges = 0, bursts = 0;
    for (std::uint32_t seed = 1; seed <= 20; ++seed)
    {
        DuelTacticsState state;
        state.mSeed = seed;
        DuelSelfView self = fighter(0.0f, 0.5f);
        self.mFootwork = 1.0f;
        int burstsThisSwing = 0;
        bool before = true;
        for (int i = 0; i < 60; ++i)
        {
            DuelOpponentView opponent = calmOpponent(50.0f);
            opponent.mWindingUp = true;
            opponent.mBladeSide = 25.0f; // on this fighter's right
            const auto out = stepDuelTactics(state, self, opponent, sStep, settings);
            // Nothing is read before the reaction delay (0.12 s at footwork 1).
            if (static_cast<float>(i + 1) * sStep < settings.mReactionFast - 1e-4f)
                EXPECT_FALSE(out.mDodgeBurst) << seed << " " << i;
            if (out.mDodgeBurst)
            {
                ++burstsThisSwing;
                EXPECT_LT(out.mDodgeSideways, 0.0f) << seed; // to the left, away from the blade
            }
            if (out.mReason == "dodge")
            {
                EXPECT_LT(out.mStrafe, 0.0f) << seed;
                if (before)
                    ++dodges;
                before = false;
            }
        }
        EXPECT_LE(burstsThisSwing, 1) << seed;
        bursts += burstsThisSwing;
    }
    EXPECT_GE(dodges, 12); // the 0.9 cap: most of 20
    EXPECT_EQ(bursts, dodges);
}

TEST(RagdollCoreDuelTactics, theDodgeReadIsOffByDefault)
{
    DuelTacticsState state;
    state.mSeed = 3;
    for (int i = 0; i < 60; ++i)
    {
        DuelOpponentView opponent = calmOpponent(50.0f);
        opponent.mWindingUp = true;
        opponent.mBladeSide = 25.0f;
        EXPECT_FALSE(stepDuelTactics(state, fighter(), opponent, sStep).mDodgeBurst);
    }
}

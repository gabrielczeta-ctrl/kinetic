#include <ragdollcore/footfallgait.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace
{
    using namespace RagdollCore;

    std::string curve(int count, float first, float middle, float last)
    {
        std::string text;
        for (int i = 0; i < count; ++i)
        {
            const float t = static_cast<float>(i) / static_cast<float>(count - 1);
            const float value = t < 0.5f ? first + (middle - first) * t * 2.0f : middle + (last - middle) * (t - 0.5f) * 2.0f;
            text += " " + std::to_string(value);
        }
        return text;
    }

    std::string row(const char* style, int angle, float speed, float duty, float swing, float lead, float crouch)
    {
        return std::string("gait ") + style + " " + std::to_string(angle) + " " + std::to_string(speed) + " "
            + std::to_string(duty) + " " + std::to_string(swing) + " " + std::to_string(lead) + " "
            + std::to_string(crouch) + "\nlift" + curve(21, 0.02f, 0.22f, 0.04f) + "\nprogress" + curve(21, 0.0f, 0.5f, 1.0f)
            + "\nbob" + curve(11, 0.0f, 0.02f, 0.0f) + "\n";
    }

    const std::string table = std::string("# test\nleg_length_m 1.0\n") + row("walk", 0, 1.0f, 0.6f, 0.4f, 0.3f, 0.0f)
        + row("walk", 0, 2.0f, 0.4f, 0.3f, 0.4f, 0.05f) + row("walk", 90, 1.0f, 0.5f, 0.5f, 0.1f, 0.0f)
        + row("walk", 90, 2.0f, 0.5f, 0.5f, 0.1f, 0.0f);

    TEST(RagdollCoreFootfallGait, parsesAndClampsBySpeed)
    {
        const auto gaits = FootfallGaitTable::parse(table);
        ASSERT_TRUE(gaits);
        EXPECT_TRUE(gaits->has("walk"));
        EXPECT_FALSE(gaits->has("run"));
        // Same leg (1 m): 1.5 leg lengths/s sits halfway between the two rows.
        const auto half = gaits->sample("walk", 0.0f, 1.5f, 1.0f, 1.0f);
        ASSERT_TRUE(half);
        EXPECT_NEAR(half->mDuty, 0.5f, 1e-3f);
        EXPECT_NEAR(half->mSwingTime, 0.35f, 1e-3f);
        EXPECT_NEAR(half->mLead, 0.35f, 1e-3f);
        // Beyond the fastest row: clamped to it.
        const auto fast = gaits->sample("walk", 0.0f, 10.0f, 1.0f, 1.0f);
        ASSERT_TRUE(fast);
        EXPECT_NEAR(fast->mDuty, 0.4f, 1e-3f);
        EXPECT_GT(fast->mFroude, fast->mMaximumFroude);
    }

    TEST(RagdollCoreFootfallGait, scalesByFroudeSimilarity)
    {
        const auto gaits = FootfallGaitTable::parse(table);
        ASSERT_TRUE(gaits);
        // A 4 m leg at twice the speed has the same Froude number as the
        // 1 m source leg at 1 m/s: lengths x4 (in units: x4 x 70), times x2.
        const auto big = gaits->sample("walk", 0.0f, 2.0f, 4.0f, 70.0f);
        ASSERT_TRUE(big);
        EXPECT_NEAR(big->mSwingTime, 0.4f * 2.0f, 1e-3f);
        EXPECT_NEAR(big->mLead, 0.3f * 4.0f * 70.0f, 1e-2f);
        EXPECT_NEAR(big->mFroude, 2.0f / std::sqrt(9.81f * 4.0f), 1e-4f);
    }

    TEST(RagdollCoreFootfallGait, blendsAndMirrorsDirections)
    {
        const auto gaits = FootfallGaitTable::parse(table);
        ASSERT_TRUE(gaits);
        const auto diagonal = gaits->sample("walk", 45.0f, 1.0f, 1.0f, 1.0f);
        const auto mirrored = gaits->sample("walk", -45.0f, 1.0f, 1.0f, 1.0f);
        ASSERT_TRUE(diagonal && mirrored);
        EXPECT_NEAR(diagonal->mDuty, 0.55f, 1e-3f);
        EXPECT_NEAR(diagonal->mLead, 0.2f, 1e-3f);
        EXPECT_NEAR(mirrored->mDuty, diagonal->mDuty, 1e-4f);
    }

    TEST(RagdollCoreFootfallGait, liftCurveStartsAndEndsOnTheGround)
    {
        const auto gaits = FootfallGaitTable::parse(table);
        ASSERT_TRUE(gaits);
        const auto walk = gaits->sample("walk", 0.0f, 1.0f, 1.0f, 1.0f);
        ASSERT_TRUE(walk);
        EXPECT_NEAR(walk->lift(0.0f), 0.0f, 1e-5f);
        EXPECT_NEAR(walk->lift(1.0f), 0.0f, 1e-5f);
        EXPECT_GT(walk->lift(0.5f), 0.15f);
        EXPECT_NEAR(walk->progress(0.0f), 0.0f, 1e-5f);
        EXPECT_NEAR(walk->progress(1.0f), 1.0f, 1e-5f);
    }

    TEST(RagdollCoreFootfallGait, readsSwingPathAndStanceKnee)
    {
        const std::string withPath = std::string("leg_length_m 1.0\n") + row("run", 0, 3.0f, 0.4f, 0.4f, 0.3f, 0.05f)
            + "path_x" + curve(21, -0.4f, 0.0f, 0.3f) + "\npath_y" + curve(21, 0.0f, 0.0f, 0.0f) + "\npath_z"
            + curve(21, -0.9f, -0.6f, -0.9f) + "\nstance_knee" + curve(21, 20.0f, 40.0f, 20.0f) + "\n";
        const auto gaits = FootfallGaitTable::parse(withPath);
        ASSERT_TRUE(gaits);
        const auto run = gaits->sample("run", 0.0f, 3.0f, 1.0f, 70.0f);
        ASSERT_TRUE(run);
        ASSERT_TRUE(run->mHasPath);
        EXPECT_NEAR(run->path(0, 0.0f), -0.4f * 70.0f, 0.1f);
        EXPECT_NEAR(run->path(0, 1.0f), 0.3f * 70.0f, 0.1f);
        EXPECT_NEAR(run->stanceKnee(0.5f), 40.0f, 0.1f);
        // Rows without a path blend to a sample without one.
        const auto plain = FootfallGaitTable::parse(table)->sample("walk", 0.0f, 1.0f, 1.0f, 70.0f);
        ASSERT_TRUE(plain);
        EXPECT_FALSE(plain->mHasPath);
        const auto half = FootfallGaitSample::mix(*run, *run, 0.5f);
        EXPECT_TRUE(half.mHasPath);
        EXPECT_NEAR(half.mDuty, run->mDuty, 1e-5f);
        EXPECT_FALSE(FootfallGaitSample::mix(*run, *plain, 0.5f).mHasPath);
    }

    TEST(RagdollCoreFootfallGait, rejectsMalformedInput)
    {
        EXPECT_FALSE(FootfallGaitTable::parse("gait walk 0 1.0\n"));
        EXPECT_FALSE(FootfallGaitTable::parse("# nothing\n"));
    }

#ifdef RAGDOLLCORE_TEST_DATA_DIR
    // The shipped table (STEPPING audit 2026-11-01): the generated "run -90"
    // led 0.048 leg lengths against 0.28 for "run 90", so running right
    // landed every foot under the hips and the feet trailed. Mirror
    // directions must lead alike (within 0.2 leg lengths at the same speed;
    // the armed and slow walks differ by up to 0.16) and every lead stays in
    // a band a foot can land in (the stealth backpedal leads -0.02).
    TEST(RagdollCoreFootfallGait, theShippedTableLeadsAlikeBothWays)
    {
        const auto gaits = FootfallGaitTable::load(std::string(RAGDOLLCORE_TEST_DATA_DIR) + "/footfall-gaits.cfg");
        ASSERT_TRUE(gaits);
        int checked = 0;
        for (const char* style : { "walk", "walk_boxing", "walk_stealth", "slow_walk", "run" })
        {
            if (!gaits->has(style))
                continue;
            for (const float angle : { 45.0f, 90.0f, 135.0f })
                for (float speed = 0.5f; speed <= 6.0f; speed += 0.5f)
                {
                    // A 1 m leg and 1 unit per metre: leads in leg lengths.
                    const auto a = gaits->sample(style, angle, speed, 1.0f, 1.0f);
                    const auto b = gaits->sample(style, -angle, speed, 1.0f, 1.0f);
                    ASSERT_TRUE(a && b);
                    EXPECT_NEAR(a->mLead, b->mLead, 0.2f) << style << " +-" << angle << " at " << speed;
                    for (const auto& s : { *a, *b })
                    {
                        EXPECT_GE(s.mLead, -0.05f) << style << " " << angle << " at " << speed;
                        EXPECT_LE(s.mLead, 0.6f) << style << " " << angle << " at " << speed;
                    }
                    ++checked;
                }
        }
        EXPECT_GT(checked, 100);
        // Running, every direction lands the foot well ahead of the hips.
        for (float angle = -180.0f; angle <= 180.0f; angle += 45.0f)
        {
            const auto run = gaits->sample("run", angle, 5.0f, 1.0f, 1.0f);
            ASSERT_TRUE(run);
            EXPECT_GE(run->mLead, 0.15f) << "run " << angle;
        }
    }
#endif
}

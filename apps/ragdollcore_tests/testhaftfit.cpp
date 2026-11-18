#include <ragdollcore/haftfit.hpp>

#include <gtest/gtest.h>

#include <cstdio>

namespace
{
    using namespace RagdollCore;

    // A haft's frame: the shaft along +Y, the edge along X.
    const WeaponStrikeAxes sAxes{ JPH::Vec3::sAxisY(), JPH::Vec3::sAxisX() };

    ChamberPose poseOf(JPH::Vec3 hand, JPH::Vec3 shaft, float twistDegrees = 0.0f)
    {
        ChamberPose pose;
        pose.mHand = hand;
        pose.mRotation = JPH::Quat::sFromTo(JPH::Vec3::sAxisY(), shaft.Normalized());
        pose.mTwist = twistDegrees * 3.14159265f / 180.0f;
        return pose;
    }

    TEST(HaftFitTest, ALevelSpearInFrontPutsTheSupportHandInTheTrunk)
    {
        // The 11-12 finding: the weapon hand in front of the chest, the shaft
        // level and forward - the support hand, 21 behind, is in the belly.
        const HaftFit front = haftFit(poseOf({ 2.0f, 24.0f, -12.0f }, { 0.0f, 1.0f, 0.0f }), JPH::Vec3::sAxisY());
        EXPECT_GT(front.mInTrunk, 4.0f);
        // Side-on with the spear on the support side, both hands hold it.
        const HaftFit side = haftFit(poseOf({ -9.0f, 18.0f, -10.0f }, { 0.0f, 1.0f, 0.0f }, -35.0f), JPH::Vec3::sAxisY());
        EXPECT_EQ(side.worst(), 0.0f);
        EXPECT_NEAR(side.mSupport.GetY(), -3.0f, 1e-3f);
    }

    TEST(HaftFitTest, ReachAndCrossedArmsAreMeasured)
    {
        // The weapon hand out past its arm.
        EXPECT_GT(haftFit(poseOf({ 11.0f, 40.0f, 6.0f }, { 0.0f, 0.0f, 1.0f }), JPH::Vec3::sAxisY()).mWeaponReach, 5.0f);
        // The shaft pointing left across the body: the support hand ends up
        // on the weapon side of the weapon hand.
        const HaftFit crossed = haftFit(poseOf({ 0.0f, 24.0f, 0.0f }, { -1.0f, 0.0f, 0.0f }), JPH::Vec3::sAxisY());
        EXPECT_NEAR(crossed.mCrossed, 10.5f, 1e-3f);
        // ... and far from the support shoulder.
        EXPECT_GT(haftFit(poseOf({ 14.0f, 26.0f, 0.0f }, { -1.0f, 0.0f, 0.0f }), JPH::Vec3::sAxisY()).mSupportReach, 5.0f);
    }

    // K2_HAFT_LINES: both hands can hold the haft all along every swing (the
    // within 2 units; the sword's lines: none of the seven, the worst 10 out of reach).
    TEST(HaftFitTest, TheHaftsOwnSwingsFitBothHands)
    {
        int swordFits = 0;
        for (std::size_t i = 0; i < 8; ++i)
        {
            const AttackLine& line = haftLines()[i];
            if (line.mName == "thrust")
                continue;
            const HaftLineFit fit = haftLineFit(line, sAxes);
            EXPECT_LT(fit.mWorst.worst(), 2.0f) << line.mName << " at u=" << fit.mU;
            EXPECT_LT(haftLineFit(haftLines(true)[i], sAxes).mWorst.worst(), 2.0f) << line.mName << " (deep chop)";
            EXPECT_LT(haftLineFit(haftLines(true, true, true)[i], sAxes).mWorst.worst(), 2.0f) << line.mName << " (own guard)";
        }
        // K2_HAFT_THRUST: the haft's own thrust fits too; the sword's asks
        // the support hand behind the chest on the weapon side.
        for (std::size_t i = 0; i < 8; ++i)
        {
            if (haftLines()[i].mName != "thrust")
                continue;
            const HaftLineFit own = haftLineFit(haftLines(false, true)[i], sAxes);
            EXPECT_LT(own.mWorst.worst(), 2.0f) << "at u=" << own.mU;
            const HaftLineFit sword = haftLineFit(haftLines()[i], sAxes);
            EXPECT_GT(sword.mWorst.worst(), 3.0f);
            EXPECT_LT(sword.mWorst.mSupport.GetY(), -10.0f);
            EXPECT_GT(sword.mWorst.mSupport.GetX(), 5.0f);
            swordFits += haftLineFit(weaponLines()[i], sAxes).mWorst.worst() < 2.0f ? 1 : 0;
        }
        EXPECT_EQ(swordFits, 0);
    }

    // K2_HAFT_CHOP: the chop's chamber has the point behind the back and
    // below the weapon hand; without it the shaft stands up.
    TEST(HaftFitTest, TheDeepChopHangsThePointBehindTheBack)
    {
        const auto chop = [](bool deep) -> const AttackLine& {
            for (const AttackLine& line : haftLines(deep))
                if (line.mName == "chop")
                    return line;
            return haftLines(deep)[0];
        };
        const JPH::Vec3 deep = linePose(chop(true), sAxes, 1.0f)->mRotation * sAxes.mLongAxis;
        const JPH::Vec3 high = linePose(chop(false), sAxes, 1.0f)->mRotation * sAxes.mLongAxis;
        EXPECT_LT(deep.GetY(), -0.8f);
        EXPECT_LT(deep.GetZ(), -0.3f);
        EXPECT_GT(high.GetZ(), 0.9f);
        // The cut starts where the wind-up ends.
        EXPECT_TRUE(linePose(chop(true), sAxes, 1.0f)->mHand.IsClose(chop(true).mCut.front().mHand, 1e-3f));
    }

    // The table the haft's lines are authored against (printed, not judged).
    TEST(HaftFitTest, DISABLED_ReportTheLines)
    {
        for (const WheelLines* lines : { &weaponLines(), &spearLines(), &haftLines(), &haftLines(true, true, true) })
        {
            std::printf("%s\n", lines == &weaponLines() ? "sword lines" : lines == &spearLines() ? "spear lines" : "haft lines");
            for (const AttackLine& line : *lines)
            {
                const HaftLineFit fit = haftLineFit(line, sAxes);
                std::printf("  %-11s worst %5.1f at u=%.2f (weapon reach %.1f, support reach %.1f, in trunk %.1f, crossed %.1f)"
                            " support=%.0f,%.0f,%.0f\n",
                    std::string(line.mName).c_str(), fit.mWorst.worst(), fit.mU, fit.mWorst.mWeaponReach,
                    fit.mWorst.mSupportReach, fit.mWorst.mInTrunk, fit.mWorst.mCrossed, fit.mWorst.mSupport.GetX(),
                    fit.mWorst.mSupport.GetY(), fit.mWorst.mSupport.GetZ());
            }
        }
    }
}

#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_STRIDESWING_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_STRIDESWING_HPP

#include "recoverycontrol.hpp"

#include <array>
#include <optional>

namespace RagdollCore
{
    // The swing spring (KINETIC-2 phase 2, MOVE-01 4.4): a horizontal spring
    // pulls each swinging foot along its path from lift-off to its landing
    // (quintic ease on the swing's phase), its reaction on the pelvis, so the
    // foot arrives where it was sent instead of trailing the leg motors' goal
    // and touching down short - the game's walking swing since the PGC work
    // (OPENMW_RAGDOLL_PGC's third knob, the frequency from Agility and
    // Acrobatics). Stride adds the reach (mReach > 0): the landing held inside
    // the leg's reach of the hip where the hip will be at touchdown. Height
    // stays with the leg drive.
    struct StrideSwingSettings
    {
        float mFrequency = 0.0f; // Hz; 0 = off
        float mDampingRatio = 1.0f;
        float mCapLegWeights = 3.0f; // force cap: this many swing-leg weights
        // Share of the hip-to-ankle length a landing may lie from the hip at
        // touchdown; 0 = unclamped (the PGC spring as it was).
        float mReach = 0.0f;
        // Stride's lift (MOVE-01 4.4, offline): early in the swing (phase under
        // mLiftUntil) a foot under its lift curve (the landing height plus
        // mLift x sin(pi phase)) is pulled up by a vertical spring of this
        // frequency, at most mLiftCap swing-leg weights, its reaction on the
        // pelvis; and while it is still that low, no horizontal pull drags it
        // along the floor (a sprinting foot stayed down, loaded, sliding at up
        // to 1000 u/s for the swing's first 0.4). 0 = off.
        float mLiftFrequency = 0.0f;
        float mLiftCap = 3.0f;
        float mLiftUntil = 0.5f;
    };

    // OPENMW_RAGDOLL_K2_STRIDE_SWING (the registry's): the reach share.
    float strideSwingReach();

    struct StrideSwingFoot
    {
        bool mSwinging = false;
        float mPhase = 0.0f; // 0-1 (may run past 1)
        float mDuration = 0.3f; // s
        BalanceVec2 mLiftOff; // where the swing started
        BalanceVec2 mLanding; // where it lands
        float mLandingHeight = 0.0f; // the foot origin's height at the landing
        BalanceVec2 mFoot; // the foot's centre of mass
        BalanceVec2 mFootVelocity;
        std::array<float, 3> mHip{ 0.0f, 0.0f, 0.0f }; // the hip joint (thigh origin), world
        float mLegLength = 0.0f; // hip to ankle, fully straight
        float mLegMass = 0.0f; // thigh + calf + foot
        // For the lift: the foot origin's height and vertical speed, and the
        // swing's peak lift over the landing height.
        float mFootHeight = 0.0f;
        float mFootVerticalVelocity = 0.0f;
        float mLift = 0.0f;
    };

    struct StrideSwingInput
    {
        std::array<StrideSwingFoot, 2> mFeet;
        BalanceVec2 mPelvisVelocity;
        float mGravity = 0.0f;
        StrideSwingSettings mSettings;
    };

    struct StrideSwingOutput
    {
        std::array<std::optional<BalanceVec2>, 2> mFootForce; // on each swinging foot
        BalanceVec2 mPelvisForce; // the reactions' sum
        std::array<float, 2> mFootLift{ 0.0f, 0.0f }; // vertical force on each foot (the lift)
        float mPelvisLift = 0.0f; // its reactions' sum (downward)
        std::array<bool, 2> mReachClamped{ false, false };
        std::array<bool, 2> mCapped{ false, false };
    };

    std::optional<StrideSwingOutput> computeStrideSwing(const StrideSwingInput& input);
}

#endif

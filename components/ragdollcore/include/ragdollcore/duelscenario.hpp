#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_DUELSCENARIO_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_DUELSCENARIO_H

// RXN-01 / DUE-01: an offline, deterministic duel between two synthetic
// humanoids with real weapon bodies. Each fighter owns a kinematic root (the
// allowed movement stand-in for this milestone; the end goal stays a
// physics-only controller), a weapon body rigidly attached to the right hand,
// a scripted thrust timeline, real solver contacts and the RXN-01 reaction
// pipeline. No game code, no Lua, no rendering.

#include <ragdollcore/anatomy.hpp>
#include <ragdollcore/reactioncontract.hpp>
#include <ragdollcore/behavior.hpp>
#include <ragdollcore/passiveragdoll.hpp>
#include <ragdollcore/profile.hpp>

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
// clang-format on

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace RagdollCore
{
    struct DuelWeaponProfile
    {
        bool mArmed = false;
        // Model units; the grip sits at 0.44 of the length from the butt, as
        // the live training weapon does.
        float mLength = 40.0f;
        float mThickness = 3.0f;
        float mMass = 4.0f;
        float mMinimumDamage = 3.0f;
        float mMaximumDamage = 10.0f;
    };

    struct DuelActorSettings
    {
        DuelWeaponProfile mWeapon;
        float mHealth = 40.0f;
        float mFirstAttackSeconds = 0.8f;
        float mSwingIntervalSeconds = 1.4f;
        float mSwingDurationSeconds = 0.5f;
        bool mAttacksEnabled = true;
        // Unarmed strikes use this band.
        float mFistMinimumDamage = 1.0f;
        float mFistMaximumDamage = 4.0f;
        // Optional fixture input from an external applied-wound policy. The
        // duel never derives persistent injury severity from contact energy or
        // its local damage band.
        std::optional<float> mScriptedWoundSeverity;
    };

    struct DuelSettings
    {
        float mDurationSeconds = 10.0f;
        float mFixedTimeStep = 1.0f / 60.0f;
        // Distance between the two root positions along Y.
        float mSeparation = 52.0f;
        float mTotalMass = 150.0f;
        DuelActorSettings mLeft;
        DuelActorSettings mRight;
        // Physical balance: dynamic roots with the reference standing load
        // plan instead of the kinematic stand-in. The kinematic mode remains
        // the default until the physical mode is qualified.
        bool mPhysicalBalance = false;
        // Reactions: the real pipeline, disabled by default so the base duel
        // can be measured without them.
        bool mReactionsEnabled = false;
        BehaviorProfile mBehavior;
        // Impulse a contact needs before it counts as a strike (model units/s).
        float mMinimumApproachSpeed = 8.0f;
    };

    struct DuelActorFrame
    {
        bool mAlive = true;
        float mHealth = 0.0f;
        BehaviorId mBehavior = BehaviorId::None;
        ReactionPhase mPhase = ReactionPhase::Idle;
        float mAuthority = 0.0f;
        float mStability = 1.0f;
        bool mCoverFace = false;
        bool mCradleLeft = false;
        bool mCradleRight = false;
        bool mLimpLeft = false;
        bool mLimpRight = false;
        bool mSwinging = false;
        float mSwingProgress = 0.0f;
        bool mHitThisStep = false;
        float mLatestHealthLoss = 0.0f;
        float mLatestImpulse = 0.0f;
        AnatomicalRegion mLatestRegion = AnatomicalRegion::Unknown;
        std::vector<JPH::Vec3> mBodyPositions; // world, model body order
        std::vector<JPH::Quat> mBodyRotations;
        std::optional<JPH::Vec3> mWeaponGrip;
        std::optional<JPH::Vec3> mWeaponTip;
    };

    struct DuelFrame
    {
        float mTime = 0.0f;
        std::uint64_t mStep = 0;
        DuelActorFrame mLeft;
        DuelActorFrame mRight;
    };

    struct DuelResult
    {
        std::vector<DuelFrame> mFrames;
        std::vector<std::string> mErrors;
        std::string mWeaponNames[2];
        std::vector<std::string> mBodyNames[2];
        std::vector<int> mBodyParents[2];
        std::size_t mBodyCount[2] = { 0, 0 };
        int mLeftHits = 0;
        int mRightHits = 0;
        float mLeftDamageDealt = 0.0f;
        float mRightDamageDealt = 0.0f;
        bool mLeftDead = false;
        bool mRightDead = false;
        std::uint64_t mStepCount = 0;
        float mSimulatedSeconds = 0.0f;
        bool completed() const { return mErrors.empty() && !mFrames.empty(); }
    };

    DuelResult runDuelScenario(const SkeletonDescriptor& skeleton, const RagdollProfile& profile,
        const DuelSettings& settings);
}

#endif

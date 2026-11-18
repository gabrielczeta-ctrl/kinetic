#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_LIFECYCLE_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_LIFECYCLE_HPP

#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Real.h>
#include <Jolt/Math/Vec3.h>

#include <ragdollcore/controller.hpp>
#include <ragdollcore/profile.hpp>

#include <optional>
#include <span>
#include <string>

namespace RagdollCore
{
    // KINETIC-2 phase 4, the lifecycle (PLAN.md "KINETIC-2": "building,
    // deaths and corpses as one module"; the pipeline's Lifecycle area, owner
    // of every channel once dead). The rules of a death and a corpse:
    // - the death carry: the powered body goes limp where it is, keeping its
    //   common velocity (capped) and a share of each part's own;
    // - the death drag: the corpse's common horizontal velocity drained for a
    //   few seconds, so it crumples without sliding;
    // - the corpse follow: the actor follows its corpse over the floor;
    // - the load pose: a corpse rebuilt with nothing carried lies on its back
    //   on the floor instead of falling again from the standing pose;
    // - the build frames: a body's joint frames from a standing pose of its
    //   skeleton.
    // Moved from the old integration (physicssystem.cpp) behind
    // OPENMW_RAGDOLL_K2_LIFECYCLE; the old path stays beside it until the
    // user's verdict.
    //
    // Pure: no engine types.

    // OPENMW_RAGDOLL_DEATH_CARRY_VELOCITY = "share,cap" (0.15, 15 since 10-30:
    // 29 of 30 deaths carried the full 50 u/s cap and slid a median 50 u) and
    // OPENMW_RAGDOLL_DEATH_CARRY_TRUE_CAP (the parts keep their share off the
    // body's own mean, not off the capped common velocity, which leaked the
    // whole-body motion past the cap: 15 + 0.15 x 185 = 43 u/s, 11-03).
    struct DeathCarrySettings
    {
        float mShare = 0.15f;
        float mCap = 15.0f; // u/s, 0 = uncapped
        bool mTrueCap = false;
    };

    struct DeathCarry
    {
        JPH::Vec3 mCommon = JPH::Vec3::sZero(); // capped
        JPH::Vec3 mUncapped = JPH::Vec3::sZero(); // the parts' mean
        JPH::Vec3 mPartBase = JPH::Vec3::sZero(); // what each part's own motion is measured from
        float mShare = 0.0f;

        JPH::Vec3 linear(JPH::Vec3Arg part) const { return mCommon + (part - mPartBase) * mShare; }
        JPH::Vec3 angular(JPH::Vec3Arg part) const { return part * mShare; }
    };

    // From the living parts' linear velocities (unweighted mean, as before).
    DeathCarry deathCarry(std::span<const JPH::Vec3> partVelocities, const DeathCarrySettings& settings);

    // The carry and the drag a death uses. OPENMW_RAGDOLL_K2_DEATH_MOMENTUM =
    // "share, cap u/s, drag rate" (registry): set, the corpse keeps `share` of
    // each part's own motion and spin off the body's mean, the mean capped at
    // `cap` (0 = uncapped), and the drag drains its horizontal mean at `drag`
    // per second (0 = none); share 0 = off, DEATH_CARRY_VELOCITY and
    // DEATH_DRAG as before. Measured 11-06: deaths in play moved at 52-374 u/s
    // and every one hit the 15 u/s cap (a killing blow threw no one).
    struct DeathMotion
    {
        DeathCarrySettings mCarry;
        float mDragRate = 3.0f;
        float mDragSeconds = 2.5f;
    };
    DeathMotion deathMotion(const DeathMotion& old, std::span<const float, 3> momentum);

    // How a corpse carried over from a living body is built.
    // OPENMW_RAGDOLL_K2_DEATH_JOINTS (registry, 0 = off): the corpse's joints
    // take the living body's frames - its build skeleton, its standing frames
    // and the elbows' and knees' anatomical hinges - so every joint starts
    // inside its limits where the living one was, and nothing is blended from
    // the animation pose (it is drawn from the physics at once). Before: the
    // joints came from the animation pose at death, 20-97 u (median ~48) off
    // the body, and the solver snapped them back into range on the first
    // steps; the first 0.08 s were drawn blending from that pose.
    struct CorpseBuild
    {
        bool mLivingFrames = false;
        float mBlendSeconds = 0.0f;
    };
    // `carried`: the corpse takes a living body's state; `livingKnown`: that
    // body's build skeleton is at hand; `blendSeconds`: the profile's blend.
    CorpseBuild corpseBuild(bool setting, bool carried, bool livingKnown, float blendSeconds);

    // The death limits (OPENMW_RAGDOLL_K2_DEATH_LIMITS = seconds, registry;
    // 0 = off). A corpse's joints are the authored, narrow ones, and the
    // living body's were opened while it fought (a weapon hand's wrist to
    // 2.0 rad against the authored 0.45/0.35, the shoulders for a swing, the
    // forearm's roll): a joint carried over outside the corpse's range was
    // pulled into it on the corpse's first step, all of it at once (the limp
    // solver runs 32 position iterations). Each such joint's range is opened
    // to hold where the joint is and closes to the authored one over
    // `seconds`.
    //
    // How far a joint is outside its range, radians (0 = inside). A swing is
    // read as Jolt reads it: an ellipse over the swing quaternion's y and z.
    float jointExcess(const JointRange& range, const JointAt& at);
    // The range opened to hold the joint where it is (a little past it), no
    // narrower anywhere than `authored`.
    JointRange deathRangeOpened(const JointRange& authored, const JointAt& at);
    // `opened` at `progress` 0, `authored` from 1 on.
    JointRange deathRangeEased(const JointRange& authored, const JointRange& opened, float progress);
    // A joint being closed: the body it joins to its parent and its two ranges.
    struct DeathLimit
    {
        std::size_t mBody = 0;
        JointRange mAuthored;
        JointRange mOpened;
    };

    // OPENMW_RAGDOLL_DEATH_DRAG = "rate, seconds" (3, 2.5; 0 = off): the
    // number of physics steps the drag runs, and the horizontal velocity it
    // takes off every part this step from the body's mass-weighted mean.
    int deathDragSteps(float rate, float seconds, float dt);
    JPH::Vec3 deathDragDrain(JPH::Vec3Arg meanVelocity, float rate, float dt);

    // The corpse follow (CORPSE_FOLLOW): where the actor goes - over the
    // floor under the root body or, with no floor in reach, no higher than
    // the root body; nullopt when the corpse and actor are still (within
    // 0.25 u), so leftover inertia does not move it.
    std::optional<JPH::RVec3> corpseFollowTarget(JPH::RVec3Arg root, JPH::RVec3Arg reference,
        std::optional<JPH::Real> floorZ, JPH::RVec3Arg actor);

    // The load pose (CORPSE_LOAD_POSE): a body still standing (root more than
    // 30 u over the floor) is turned onto its back about its pelvis, set down
    // over the actor at a body's half depth (8 u) over the floor.
    struct CorpseLoadPose
    {
        JPH::Quat mTurn = JPH::Quat::sIdentity();
        JPH::RVec3 mPivot = JPH::RVec3::sZero();

        // A body at `position` with `rotation`, the root body at `root`.
        JPH::RVec3 place(JPH::RVec3Arg position, JPH::RVec3Arg root) const
        {
            return mPivot + mTurn * JPH::Vec3(position - root);
        }
        JPH::Quat turn(JPH::QuatArg rotation) const { return (mTurn * rotation).Normalized(); }
    };
    std::optional<CorpseLoadPose> corpseLoadPose(
        float yaw, JPH::RVec3Arg actor, JPH::RVec3Arg root, JPH::Real floorZ);

    // The build frames (BUILD_FRAMES; user 11-01 bookmarks 5-11, "my right
    // foot is always forward": a body rebuilt at a door while running took
    // that stride as its joints' zero). The legs' and spine's joint frames
    // come from a standing pose of the same skeleton, the first powered build
    // of it standing still; the arms keep the build pose. Named by the
    // skeleton's id, its bones and its leg lengths (thigh-calf, calf-foot:
    // no joint between, so they do not change with the pose).
    struct BuildFrames
    {
        std::string mKey;
        // The pelvis, spine, head and both legs' bones, unparented.
        SkeletonDescriptor mFrames;
        // This build pose stands: on the ground and still (`onGroundStill`),
        // each foot within 15 u of under the pelvis, the feet level within
        // 4 u, the chest within 8 u over the pelvis.
        bool mStanding = false;
    };
    // `profile` names the landmarks, `runtimeProfile` the bodies' bones and
    // parents (the build's own copy), `skeleton` the build pose.
    BuildFrames buildFrames(const RagdollProfile& profile, const RagdollProfile& runtimeProfile,
        const SkeletonDescriptor& skeleton, bool onGroundStill);

    // A rebuild for the rig (a weapon or shield changed, or a drawn weapon
    // built unready) waits for a quiet moment: no swing, no AI strike, no
    // held attack, no hit stun, standing on the held root.
    struct RebuildInput
    {
        bool mRigChanged = false;
        bool mUnready = false;
        bool mRegrip = false; // regripAfterRelease below
        bool mBusy = false; // swinging, an AI strike, a held attack or a hit stun
        bool mStanding = false; // the controller's mode
        bool mRootKinematic = false;
    };
    bool rebuildForRig(const RebuildInput& input);
    // K2_REGRIP: a weapon whose grip joint broke under load while it is still
    // the equipped weapon is gripped again, through the rig rebuild, `delay`
    // seconds after it let go (0 = never: it stayed released until the weapon
    // was taken off and put back on - sheathing did not rebuild). A weapon
    // let go for any other reason (dropped, unequipped, death) stays let go.
    struct RegripInput
    {
        bool mReleased = false; // the body has a weapon and its grip is not attached
        bool mBroke = false; // the release's reason was the grip joint's break
        bool mStillEquipped = false; // the same weapon is in the weapon slot
        float mSinceRelease = 0.0f; // seconds
    };
    bool regripAfterRelease(const RegripInput& input, float delay);
    // The rebuilt body keeps the standing pelvis height the old one learned
    // (a fresh tracker learns it only standing still: no upright control
    // until then, the legs never stepped - the user's "stuck in a frame",
    // 11-03). OPENMW_RAGDOLL_REBUILD_KEEPS_HEIGHT (0 = the old rule).
    bool rebuildKeepsHeight(bool setting, bool heightLearned);

    // The physical death (DEATH_PHYSICAL; STEPPING audit 11-01 item 20,
    // "dying on the feet -> collapse -> catch -> limp"): a powered body
    // killed on its feet stays powered while it dies - its fall decided along
    // the killing blow, its motors held weak by the hit stun, the fall
    // protection catching it - and goes limp once it lies or after `most`
    // seconds (at least 0.2). OPENMW_RAGDOLL_DEATH_PHYSICAL = "on, most s,
    // stun" (on 0 = off: limp the instant the blow lands).
    struct PhysicalDeathSettings
    {
        bool mOn = false;
        float mMost = 1.4f;
        float mStun = 6000.0f;
    };
    struct PhysicalDeathState
    {
        bool mDying = false;
        bool mDone = false;
        float mTime = 0.0f;
    };
    // Begins it (true) for a powered body on its feet, not already dying.
    bool beginPhysicalDeath(PhysicalDeathState& state, const PhysicalDeathSettings& settings, bool powered,
        std::optional<ActiveControllerMode> mode);
    // One step; true when it ends this step (the body goes limp).
    bool stepPhysicalDeath(
        PhysicalDeathState& state, const PhysicalDeathSettings& settings, bool lying, bool powered, float dt);
}

#endif

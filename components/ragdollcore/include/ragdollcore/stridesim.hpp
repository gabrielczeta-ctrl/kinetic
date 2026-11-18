#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_STRIDESIM_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_STRIDESIM_HPP

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
// clang-format on

#include <ragdollcore/controller.hpp>
#include <ragdollcore/footfall.hpp>
#include <ragdollcore/passiveragdoll.hpp>
#include <ragdollcore/profile.hpp>
#include <ragdollcore/recoverycontrol.hpp>
#include <ragdollcore/roottarget.hpp>
#include <ragdollcore/runtimeprofile.hpp>
#include <ragdollcore/stepplanner.hpp>
#include <ragdollcore/strideswing.hpp>

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace RagdollCore
{
    // A small offline world for the game's walking loop and for training
    // (MOVE-01 section 9, the training pilot). One scenario humanoid on a flat
    // floor in its own Jolt world, stepped at 60 Hz:
    //   walking: a capsule target moves at the asked velocity, the soft root
    //   pulls the pelvis toward it (the game's body masks, the PGC budget's
    //   standing / moving strengths), Footfall steps the legs from the game's
    //   gait table and its drive pose goes to the joint motors;
    //   limp: the root is let go and a caller sets the motor targets (a
    //   policy getting up from the floor).
    // It is the game's loop reduced to its core: no animation clip (the
    // standing bind pose is the base pose), no PGC catches, no arms or weapon.
    // A Jolt Factory and the registered types must exist (the caller's main).
    class StrideSim
    {
    public:
        struct Settings
        {
            // files/data/ragdoll/active-body.cfg (the soft root's springs and
            // caps) and footfall-gaits.cfg; empty: built-in defaults / no table.
            std::string mActiveBodyPath;
            std::string mGaitTablePath;
            // A body the game built (OPENMW_RAGDOLL_SKELETON_DUMP): its skeleton,
            // standing frames, scale and mass, the profile's elbows and knees
            // framed from it as the game frames them (applyAnatomicalFrames).
            // Empty: the scenario humanoid.
            std::string mSkeletonPath;
            // The profile the game loads (files/data/ragdoll/humanoid-v1.yaml,
            // through RagdollProfile::load). Empty: the scenario fixture,
            // whose motors and spine limits have drifted from the authored
            // profile (W1, 11-03: every joint at 4 Hz against 8-80) and which
            // the offline tests are tuned to.
            std::optional<RagdollProfile> mProfile;
            // The game's clip, recorded each physics step
            // (OPENMW_RAGDOLL_CLIP_DUMP), played back as the base pose: the
            // track's first frame with movement asked plays at step
            // mClipTrackAlign, the rest frame by frame (clamped at the ends).
            // Empty: the skeleton's still pose.
            std::string mClipTrackPath;
            std::size_t mClipTrackAlign = 0;
            // The actor (capsule) origin's height over the floor. The game's
            // stands 4.2 units over flat ground (W1: the drive pelvis 74.8
            // against the clip's 70.6 at a stand); every target from the clip
            // is relative to it. 0: on the floor, as before.
            float mActorHeight = 0.0f;
            // Lowers the root's target under the drive pose's pelvis (units;
            // a parity experiment, 11-04: the course's walking pelvis stands
            // ~5 u over the game's, 67.3 against 62.3, and its swing foot
            // falls behind its goal late in the swing). 0 = as before.
            float mRootLower = 0.0f;
            FootfallSettings mFootfall;
            // The PGC root budget (OPENMW_RAGDOLL_PGC_BUDGET's standing and
            // moving shares) and the speed above which only the horizontal
            // hold is budgeted (PGC_MOVE_VERTICAL).
            float mStandingStrength = 0.15f;
            float mMovingStrength = 0.3f;
            float mMoveVertical = 180.0f;
            float mGroundFriction = 0.8f;
            // The game's Jolt settings (configureScenarioPhysics: tolerances in
            // world units, manifold reduction off). Off: Jolt's metre
            // tolerances read as units, as every run before 11-03 08:30.
            bool mGameContacts = false;
            // MOVE-01 S1, the floor. HoldSlab: the prison ship's hold floor as
            // its collision mesh has it (In_prison_ship.NIF): a slab 0.6 u
            // thick, top and underside each two 481 x 600 u triangles per
            // section, four sections; its top at z 0 with the ship's
            // (mSlabStart) at the origin, and the hull as a box 15 u under it.
            enum class Floor
            {
                Box,
                HoldSlab
            };
            Floor mFloor = Floor::Box;
            std::array<float, 2> mSlabStart{ 61.0f, -135.0f };
            float mTotalMass = 75.0f;
            // Start lying: the root turned by this rotation, its centre this
            // high, limp (no soft root, no Footfall).
            std::optional<JPH::Quat> mLyingRotation;
            float mLyingHeight = 14.0f;
            // Lying: the soft root may help (setAssist), as the game's get-up
            // holds the pelvis while the clip plays. The joint motors alone
            // lift a lying body's pelvis to ~14 of its standing 66 units.
            bool mLimpAssist = false;
            // MOVE-01 S3 offline: the step planner aims each swing's landing
            // (from the measured momentum about the stance foot) through
            // Footfall's command; Footfall still times the steps.
            bool mPlanner = false;
            StrideLimits mPlannerLimits;
            // A swing is aimed while its phase is at most this (then held).
            float mPlanUntil = 0.5f;
            // MOVE-01 S4 offline, the legs lead (the game's LEGS_LEAD): the
            // capsule follows the body, the root's horizontal hold drops to
            // `mLegsLeadRootShare`, and the stance force (computeStanceForce:
            // the upper body pushed toward the asked velocity against the
            // planted feet, friction-capped) moves the body.
            bool mLegsLead = false;
            // Leading, Footfall steps for the asked velocity (the intent,
            // MOVE-01 4.6) instead of the following capsule's.
            bool mLegsLeadStepForIntent = false;
            // Leading, the joint support's motion reference (a centre-of-mass
            // target moving at the asked velocity) moves the body through
            // the stance legs, instead of the upper-body force pair.
            bool mLegsLeadThroughSupport = false;
            // Leading through the support, the stance push (MOVE-01 4.5): the
            // support's reference is the centre of mass itself along the
            // travel (no pull back over the feet) moving at the asked
            // velocity, so the stance legs push on the velocity error alone;
            // across the travel it stays the feet's middle. The planner's
            // landings then catch the body falling ahead.
            bool mLegsLeadPush = false;
            float mLegsLeadRootShare = 0.1f;
            // The root as a budget (MOVE-01 4.6, `assist`): the horizontal
            // share grows by this times the velocity error over the asked
            // speed (a start, a stop, a reversal), up to 1; 0 = the fixed share.
            float mLegsLeadErrorShare = 0.0f;
            // The legs lead only up to this asked speed (u/s); faster, the
            // capsule leads (W4: the stance force pair walks but its runs
            // fell). 0 = any speed.
            float mLegsLeadMaxSpeed = 0.0f;
            // The capsule leads while the body is off the asked velocity by
            // more than this share of it (a start, a stop, a reversal: the
            // root drags, at once); the legs take over within it, and give
            // back past twice it. 0 = the legs lead whenever asked.
            float mLegsLeadCruiseError = 0.0f;
            // Leading, above 0: the reference leads on a leash (Coros 2010's
            // reference tracking). The capsule moves at the asked velocity
            // but never more than this far ahead of the pelvis, so the drive
            // pose is solved for a pelvis ahead of the planted foot and the
            // leg motors push the body; the root's horizontal hold is
            // `mLegsLeadRootShare`.
            float mLegsLeadLeash = 0.0f;
            // On the leash, each swing's landing is moved by the reference's
            // lead over the pelvis when it lifts (then held): the leg is solved
            // for the leading pelvis, so without this a swing lands that much
            // short of its target.
            bool mLegsLeadAimSwing = false;
            // Leading through the support, steps by events (MOVE-01 4.3):
            // Footfall's clock is held; with both feet down the support moves
            // the centre of mass over the next stance foot (across the
            // travel), and the other foot lifts once it carries less than
            // `mTransferLoad` of the weight (or after `mTransferTime` s).
            bool mTransfer = false;
            float mTransferLoad = 0.2f;
            float mTransferTime = 0.4f;
            float mTransferSwingTime = 0.35f;
            StanceForceSettings mStanceForce{ 20.0f, 6.0f, 0.35f, 15.0f };
            // The standing joint support (calculateStandingJointSupport: the
            // inverse-static leg loads from the measured ground contacts) on
            // the joint motors while walking, as the game's active controller
            // adds it. Off: the soft root and the pose motors alone.
            bool mJointSupport = false;
            // The game's grip per foot (W1, the game's swing-foot friction):
            // a foot Footfall has pinned at OPENMW_RAGDOLL_PINNED_FOOT_FRICTION
            // (2) times its friction, a swinging one at 0.05, the rest at 1;
            // the joint support takes the contacts' own combined friction.
            // Off: every foot at 1, the support at mGroundFriction.
            bool mGameFootGrip = false;
            // Footfall's swing feed-forward on the legs it drives, as the game
            // (applyLegFeedForward): each joint's target angular velocity is
            // its body's planned one less its parent's, times
            // OPENMW_RAGDOLL_FOOTFALL_FF (0.5) and OPENMW_RAGDOLL_LEG_FEEDFORWARD
            // (1), at most 12 rad/s. Off: the motors drive to the pose at rest.
            bool mSwingFeedForward = false;
            // The game's root routing (SOFT_ROOT_AIRFADE off): the weight
            // support stays whole, a foot routes the weight when it has a
            // supporting contact (the support estimate), and standing, a foot
            // Footfall swings while the other is down routes none
            // (SOFT_ROOT_FOOTFALL_UNROUTE). Off: a foot within 2 units of the
            // floor routes it and the support follows any grounded foot.
            bool mGameRootSupport = false;
            // The game's floor-underside rule (rejectsFloorUnderside,
            // OPENMW_RAGDOLL_FLOOR_UNDERSIDE): a thin floor's underside never
            // pushes a body down from below its centre.
            bool mRejectFloorUnderside = false;
            // The root's weight support while walking (15-150 u/s asked;
            // MOVE-01 4.6 `support`, the vertical handover): 1 = whole.
            float mWalkSupport = 1.0f;
            // The game's upright spine torque while it walks (the animation
            // route's applyUprightControl under the held root: the chest held
            // to world up by the profile's spine upright spring, capped, its
            // reaction on the stance thigh). Off: the trunk is held by the
            // spine motors and the soft root alone, as before (W1, 11-06).
            bool mUprightSpine = false;
            // The game's run bob on the root's target (roottarget applyRunBob,
            // OPENMW_RAGDOLL_RUN_BOB); unset: the clip's pelvis as it is.
            std::optional<RunBobSettings> mRunBob;
            // The game's walking swing spring (strideswing: PGC's, its frequency
            // from the actor's Agility and Acrobatics) and Stride's reach on it;
            // off at frequency 0, as before 11-06.
            StrideSwingSettings mStrideSwing;
            // Footfall's per-step trace lines (RAGDOLL_FOOTFALL*), as in the game log.
            std::function<void(const std::string&)> mTrace;
        };

        explicit StrideSim(const Settings& settings);
        ~StrideSim();
        StrideSim(const StrideSim&) = delete;
        StrideSim& operator=(const StrideSim&) = delete;

        bool valid() const { return mValid; }
        const std::string& error() const { return mError; }

        // Walking: the capsule's velocity (world, horizontal) from now on.
        void setRequestVelocity(JPH::Vec3Arg velocity) { mRequest = velocity; }
        // Where the capsule faces (radians, Morrowind's yaw: forward =
        // (sin, cos)); unset faces the travel. Strafes and backpedals set it.
        void setFacing(std::optional<float> yaw) { mFacing = yaw; }
        // Limp: the joint motors' target this step (model space, body order),
        // each motor's spring scaled by `stiffness` (0-1 of the profile's).
        void setLimpTarget(const RagdollPose& target, float stiffness);
        // Limp with mLimpAssist: the soft root pulls the pelvis toward this
        // world target with `strength` (0 = no help, 1 = the root's whole caps).
        void setAssist(JPH::RVec3Arg target, JPH::QuatArg rotation, float strength);
        // MOVE-01 W1, the replay bridge: one line of the game's
        // OPENMW_RAGDOLL_STATE_DUMP (the actor's origin x y z and yaw, the body
        // count, then per body its centre of mass, rotation x y z w, linear and
        // angular velocity). Every body is put there, carried from the game's
        // actor origin and yaw to the capsule's (mActorHeight over the floor)
        // facing `yaw`. False (nothing moved) when the line does not fit the body.
        bool setGameState(const std::string& line, float yaw);
        // Each body's mass and shape bounds, as the game's RAGDOLL_BODY_MASSES line.
        std::string bodyMassesLine() const;
        // A body's motor target this step (the drive pose), world rotation.
        std::optional<JPH::Quat> driveRotation(std::size_t body) const
        {
            return mRagdoll ? mRagdoll->getWorldBodyRotation(mDrivePose, body) : std::nullopt;
        }
        void step();

        std::size_t steps() const { return mSteps; }
        float time() const { return static_cast<float>(mSteps) / 60.0f; }
        std::size_t bodyCount() const;
        // The ragdoll's ground contacts in the last step.
        const std::vector<GroundContact>& groundContacts() const;
        const RagdollProfile& profile() const;
        const RagdollPose& basePose() const { return mBasePose; }
        // World centre of mass position, rotation and linear velocity of a body.
        JPH::RVec3 bodyPosition(std::size_t body) const;
        JPH::Quat bodyRotation(std::size_t body) const;
        JPH::Vec3 bodyVelocity(std::size_t body) const;
        JPH::RVec3 capsulePosition() const { return mCapsule; }
        // The asked velocity (the keys), whether the capsule moves at it or
        // follows the body.
        JPH::Vec3 requestVelocity() const { return mRequest; }
        // The soft root's last force on the body (MOVE-01's `assist`: its
        // horizontal part against the weight).
        JPH::Vec3 rootForce() const { return mRagdoll->getSoftRootTelemetry().mForce; }
        std::size_t pelvis() const { return mPelvis; }
        std::size_t head() const { return mHead; }
        std::array<std::size_t, 2> feet() const { return { mLegs[0].mFoot, mLegs[1].mFoot }; }
        // Footfall's last output and its feet (walking only).
        const FootfallOutput& footfallOutput() const { return mFootfallOutput; }
        const FootfallController* footfall() const { return mFootfallController.get(); }
        // The pelvis height when standing on the bind pose (for rewards).
        float standingPelvisHeight() const { return mStandingPelvisHeight; }
        // With mPlanner: the last plan per side, and how many plans handed
        // over to the catch or gave up.
        const std::array<std::optional<StridePlan>, 2>& plans() const { return mPlans; }
        std::size_t recoveryPlans() const { return mRecoveryPlans; }
        std::size_t giveUps() const { return mGiveUps; }

    private:
        struct World;
        std::unique_ptr<World> mWorld;
        std::unique_ptr<PassiveRagdoll> mRagdoll;
        std::unique_ptr<FootfallController> mFootfallController;
        std::optional<FootfallGaitTable> mGaitTable;
        Settings mSettings;
        // The skeleton and profile the body was built from (the scenario
        // humanoid's, or the game's from mSkeletonPath).
        SkeletonDescriptor mSkeleton;
        std::optional<SkeletonDescriptor> mFrames;
        RagdollProfile mProfile;
        std::optional<ClipTrack> mClipTrack;
        std::size_t mClipTrackStart = 0;
        RagdollPose mBasePose;
        RagdollPose mPhysicsPose;
        RagdollPose mDrivePose;
        FootfallOutput mFootfallOutput;
        std::array<FootfallLeg, 2> mLegs;
        std::vector<std::size_t> mSpine;
        std::vector<std::pair<std::size_t, bool>> mChestCarried;
        std::size_t mPelvis = 0;
        std::size_t mHead = 0;
        // The chest's up in its own frame, from the standing pose (mUprightSpine).
        JPH::Vec3 mTorsoLocalUp = JPH::Vec3::sAxisZ();
        std::size_t mLastStance = 0;
        float mRootWalkSupport = 1.0f; // the walking support share, eased
        RunBobState mRunBob;
        JPH::RVec3 mCapsule = JPH::RVec3::sZero();
        JPH::Vec3 mRequest = JPH::Vec3::sZero();
        std::optional<float> mFacing;
        std::optional<RagdollPose> mLimpTarget;
        float mLimpStiffness = 1.0f;
        JPH::RVec3 mAssistTarget = JPH::RVec3::sZero();
        JPH::Quat mAssistRotation = JPH::Quat::sIdentity();
        float mAssistStrength = 0.0f;
        void setUpSoftRoot();
        std::vector<std::pair<float, float>> mProfileSprings; // per body: frequency, damping
        float mStandingPelvisHeight = 0.0f;
        bool mLimp = false;
        bool mValid = false;
        std::string mError;
        std::size_t mSteps = 0;
        std::array<std::optional<StridePlan>, 2> mPlans;
        std::size_t mRecoveryPlans = 0, mGiveUps = 0;
        void plan(float yaw);
        void legsLead(const std::array<bool, 2>& grounded);
        bool legsLeadNow() const
        {
            const float asked = mRequest.Length();
            return mSettings.mLegsLead && asked >= 15.0f
                && (mSettings.mLegsLeadMaxSpeed <= 0.0f || asked <= mSettings.mLegsLeadMaxSpeed)
                && (mSettings.mLegsLeadCruiseError <= 0.0f || mLegsCruising);
        }
        bool mLegsCruising = false;
        void updateLegsCruising();
        JPH::Vec3 mCapsuleVelocity = JPH::Vec3::sZero();
        std::optional<JPH::RVec3> mMotionTarget;
        std::array<bool, 2> mSwingAimed{ false, false };
        // mTransfer: the foot lifting next, and how long its weight has been moving off.
        std::optional<std::size_t> mNextSwing;
        float mTransferElapsed = 0.0f;
        std::size_t mTransferLifts = 0, mTransferTimeouts = 0;

    public:
        // mTransfer: lifts after the weight came off the foot, and lifts on the time limit.
        std::size_t transferLifts() const { return mTransferLifts; }
        std::size_t transferTimeouts() const { return mTransferTimeouts; }
    };
}

#endif

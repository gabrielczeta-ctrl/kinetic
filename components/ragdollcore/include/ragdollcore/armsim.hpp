#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_ARMSIM_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_ARMSIM_HPP

#include <ragdollcore/handdrive.hpp>
#include <ragdollcore/profile.hpp>

#include <memory>
#include <string>
#include <vector>

namespace RagdollCore
{
    class PassiveRagdoll;

    // KINETIC-2 phase 1, the hand drive's offline bench (order step 3): the
    // game's body (a skeleton dump and the authored profile) with a weapon
    // on the right hand built as the engine builds it (a box along the
    // blade, fixed to the hand, its mass a share of the body's, the arm's
    // motors given the engine's holding allowance), the pelvis held still,
    // the rest of the body on its motors at the dumped pose, and the right
    // arm moved only by the hand drive (its motors off). Each step takes a
    // goal for the grip and gives what the arm did. Its own Jolt world at
    // 60 Hz; a Jolt Factory and the registered types must exist.
    class ArmSim
    {
    public:
        struct Weapon
        {
            std::string mName = "iron longsword";
            float mMass = 1.0f; // kg (the game's bodies are 1 kg today)
            float mLength = 95.0f; // units
            float mTip = 85.0f; // the tip's distance from the grip (units)
            float mWidth = 1.4f; // units
        };

        struct Settings
        {
            std::string mSkeletonPath; // OPENMW_RAGDOLL_SKELETON_DUMP
            RagdollProfile mProfile; // the profile the game loads
            Weapon mWeapon;
        };

        explicit ArmSim(const Settings& settings);
        ~ArmSim();

        bool valid() const { return mValid; }
        const std::string& error() const { return mError; }

        // The chest-centre frame (the pelvis is held, so it stays): between
        // the shoulders, 6 units down; x right, y forward, z up.
        JPH::RVec3 centre() const { return mCentre; }
        JPH::Quat bodyToWorld() const { return mBodyToWorld; }
        float armLength() const { return mArmLength; }
        // The blade's axis in the hand's frame, as the game's weapon meshes
        // sit in Morrowind's hand bone (RAGDOLL_WEAPON_PROXY: the long axis is
        // the bone's +Z, the edge its X, the grip 6.3 out along X in the palm).
        JPH::Vec3 bladeAxis() const { return JPH::Vec3::sAxisZ(); }
        JPH::Vec3 edgeAxis() const { return JPH::Vec3::sAxisX(); }
        static JPH::Vec3 gripOffset() { return JPH::Vec3(6.3f, 0.8f, 0.0f); }

        struct Step
        {
            HandChain mChain; // the arm before the step
            HandDriveOutput mDrive;
            JPH::RVec3 mGrip = JPH::RVec3::sZero(); // after the step
            JPH::Quat mHandRotation = JPH::Quat::sIdentity();
            JPH::RVec3 mTip = JPH::RVec3::sZero();
            // stepPosed: how far the solved pose's own blade points from the
            // goal's (degrees) and its hand from the goal (units) - what the
            // arm's limits allow before any dynamics. -1 when not solved.
            float mSolvedBladeError = -1.0f;
            float mSolvedReachError = -1.0f;
        };

        // One 60 Hz step toward `goal` (world).
        Step step(const HandDriveGoal& goal, const HandDriveGains& gains);

        // The arm posed by the game's arm solver (applyArmTargetPose: the
        // hand's place and rotation with the real shoulder, elbow and wrist
        // limits, the key's elbow) and driven there by its own motors at
        // `strength` x the profile's torque, plus the hand drive's gravity
        // support and grip push (gains: the rotation is the motors'). No
        // force from outside the body. The old drive minus its spring.
        Step stepPosed(const HandDriveGoal& goal, const HandDriveGains& gains, float strength);

        // Diagnostics: each arm joint's limits as built and how far its
        // child sits from the nearest allowed orientation (0 inside).
        std::string jointReport() const;

        JPH::RVec3 grip() const;
        JPH::Quat handRotation() const;

    private:
        HandChain readChain() const;

        struct World;
        std::unique_ptr<World> mWorld;
        std::unique_ptr<PassiveRagdoll> mRagdoll;
        Settings mSettings;
        RagdollProfile mProfile;
        bool mValid = false;
        std::string mError;
        std::size_t mPelvis = 0, mClavicle = 0, mUpperArm = 0, mForearm = 0, mHand = 0, mWeaponBody = 0;
        std::array<float, 3> mStrength{};
        std::vector<std::size_t> mDisabled;
        struct Pose;
        std::unique_ptr<Pose> mBase;
        JPH::RVec3 mRootPosition = JPH::RVec3::sZero();
        JPH::Quat mRootRotation = JPH::Quat::sIdentity();
        JPH::RVec3 mCentre = JPH::RVec3::sZero();
        JPH::Quat mBodyToWorld = JPH::Quat::sIdentity();
        float mArmLength = 55.0f;
        std::optional<JPH::Vec3> mPreviousElbow;
        std::array<float, 3> mMotorTorque{};
    };
}

#endif

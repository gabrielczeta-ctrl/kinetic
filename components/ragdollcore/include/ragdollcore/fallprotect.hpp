#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_FALLPROTECT_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_FALLPROTECT_H

#include <array>
#include <cstdint>

namespace RagdollCore
{
    // Fall protection (roadmap phase 2, "the dolphin dive"): once the balance
    // helper gives a push up (release, topple), the body does not go limp. It
    // reacts after a skill delay, braces the arms toward the ground along the
    // fall, tucks the chin, curls the spine and softens the legs; at the first
    // contact of anything but the feet it yields over an absorb window; once
    // slow it settles with some tone left for the get-up.
    //
    // Pure functions, no Jolt types: world frame, z up, the caller's units
    // (game units, seconds). The caller turns the command into motor targets
    // (arm chain direction, head and spine bends in the body's sagittal plane)
    // and motor stiffness multipliers.

    struct FallVec3
    {
        float mX = 0.0f;
        float mY = 0.0f;
        float mZ = 0.0f;
    };

    enum class FallPhase
    {
        Reacting, // the reaction delay: tone, no protective pose yet
        Bracing, // arms toward the ground, chin, spine, soft legs
        Impact, // something other than the feet touched: yield
        Settled, // slow after the impact: low tone, the get-up's turn
    };

    enum class FallKind
    {
        Forward,
        Backward,
        Side,
    };

    struct FallProtectSettings
    {
        float mReactionDelay = 0.09f; // s; catchCapability's reaction delay
        float mArmRamp = 0.12f; // s to full arm weight once bracing
        float mDirectionRate = 8.0f; // 1/s, the fall direction follows the body
        float mDirectionSpeed = 40.0f; // u/s: below, the torso tilt gives the direction
        // Arm direction (upper arm, world): along the fall and down.
        float mForwardReach = 0.75f; // horizontal share, forward falls
        float mBackwardReach = 0.55f;
        float mSideReach = 0.7f;
        float mFarArmSide = 0.4f; // the arm away from a side fall
        // Bends in the sagittal plane, radians, + = forward (chin to chest).
        float mHeadForward = -0.6f; // forward fall: face up, away from the ground
        float mHeadBackward = 0.7f; // backward fall: chin tucked
        float mHeadSide = 0.3f;
        float mSpineForward = 0.1f;
        float mSpineBackward = 0.35f; // round the back to roll, not slam
        float mSpineSide = 0.2f;
        // Knee and hip flexion targets, radians, + = flexion. Forward and
        // side falls crumple on soft legs alone: a bend target there landed
        // the head harder (FALL-01 forward 199 with 1.0/0.5, 154 with none).
        // Backward falls sit down.
        float mKneeBend = 0.0f;
        float mHipBend = 0.0f;
        float mKneeBendBackward = 0.8f;
        float mHipBendBackward = 0.9f;
        // Motor stiffness multipliers of the profile's springs.
        float mReactArms = 0.8f, mReactUpper = 0.7f, mReactLegs = 0.5f;
        float mBraceArms = 1.0f, mBraceUpper = 0.7f, mBraceLegs = 0.3f;
        // Forward falls crumple: soft legs and trunk drop the body onto the
        // knees and hands (offline FALL-01: head 137 u/s soft against 207
        // firm); backward and side falls keep the firmer trunk (144/139 firm
        // against 187/185 soft). Used for the reaction and the brace.
        float mForwardUpper = 0.4f, mForwardLegs = 0.1f;
        // A leg sweep (the feet knocked out faster than the chest moves, as
        // the owner's calf-hit FALL-01): the trunk and legs stay near limp so
        // the body folds, only the arms reach. Calf sweep offline, head at
        // contact: limp 154, full tone 185, this 157 (at 12 u: 187/198/174).
        float mSweptTone = 0.05f;
        // Which falls are protected (1) or left limp (0), by the latched kind.
        // The game (owner, FALL-01 10-02, head at 12 u, limp -> protected):
        // back chest 391 -> 375, back sweep ~206 -> 206, fwd sweep 267 -> 210,
        // but side sweep 135 -> 225 and fwd chest throw 378 -> 419; the sim
        // wins on all of them. Limp falls set FallProtectCommand::mLimp.
        float mProtectSide = 1.0f;
        float mProtectForwardPushed = 1.0f; // forward falls that were not a leg sweep
        float mLatchKind = 1.0f;
        float mLatchOnPush = 1.0f; // 1: a body already moving fast latches the kind at the first step // 1: the kind is
                                   // latched when bracing starts (0: follows the body)
        float mImpactArms = 0.35f, mImpactUpper = 0.3f, mImpactLegs = 0.2f;
        float mSettleTone = 0.1f;
        float mAbsorbTime = 0.3f; // s, brace -> impact stiffness
        float mMaximumFallTime = 1.5f; // s bracing without a contact -> impact
        float mSettleSpeed = 25.0f; // u/s
        float mSettleHold = 0.3f; // s below the settle speed after the impact
        // Aim the hands at the head's predicted landing point on the floor
        // (with mHasHead): the head's position carried by its velocity for
        // the time it takes to drop to the floor (clamped), mHandLead further
        // along the fall. 0 = the fixed reach above.
        float mAimLanding = 1.0f;
        float mHandLead = 10.0f;
        float mMinimumDropSpeed = 60.0f; // u/s
        float mMaximumLookahead = 0.5f; // s
        // Catch upgrades (CATCH-02, 2026-10-27; our own design and
        // numbers). All 0 = the catch above.
        // Each arm aims at the surface its own probe found from the shoulder
        // along the aim (a wall, a step, a crate), this share of the way.
        float mProbeAim = 0.0f;
        // 1: a hand that touches yields on its own over mAbsorbTime and the
        // other keeps reaching; only the trunk, head or legs touching (or the
        // fall timing out) end the brace for the rest of the body.
        float mArmYield = 0.0f;
        // After the first contact the tone follows the centre of mass's speed:
        // the brace's at this speed (u/s), the impact's at rest (0 = by time
        // only). The time ramp still bounds it, at 3 absorb times.
        float mToneSpeed = 0.0f;
        float mToneRate = 8.0f; // 1/s, the speed's filter
        // Per-fall spread (0-1) of the bends and the reach, so no two falls
        // look the same: up to this share of 0.25 rad on the spine, hips,
        // knees and head and of 0.15 on each arm's reach, seeded per fall.
        float mVariation = 0.0f;
        // Radians the head turns further from the ground once a hand touched
        // (the face up on forward falls, the chin in on backward ones).
        float mContactTuck = 0.0f;
    };

    // One arm's view of the world for the catch.
    struct FallArmInput
    {
        bool mHasProbe = false; // the probe from the shoulder hit a surface
        FallVec3 mProbeHit;
        FallVec3 mShoulder;
        bool mTouching = false; // its hand or forearm touches something
    };

    struct FallProtectInput
    {
        float mDt = 0.0f;
        // False while the body is still up but a fall is predicted (the
        // alert): the arms only, weighted by mRisk; no contact yield, no head,
        // spine or stiffness changes. The controller keeps its clock when the
        // fall comes, so an alert body is already braced.
        bool mFallen = true;
        float mRisk = 1.0f; // 0-1, FallPredictor

        FallVec3 mComVelocity;
        FallVec3 mTorsoUp; // the chest's up axis, world
        FallVec3 mBodyForward; // the chest's forward axis, world
        bool mNonFootContact = false; // anything but the feet touches the ground
        // The fall began with the feet swept from under the body (the caller
        // decides once at the fall: feet faster than the chest, horizontally).
        bool mLegSwept = false;
        // The blow that knocked it down, world (zero = unknown). With
        // mLegSwept the kind is known at the first step: the legs go with the
        // blow, the body falls the other way (a side blow is a side fall).
        // A calf sweep barely moves the centre of mass, so mLatchOnPush
        // cannot decide it, and the reaction tone changed its landing (game
        // 10-02: side sweep limp-at-latch 194 vs plain limp 135).
        FallVec3 mBlowDirection;
        // The head and the shoulders' midpoint, world, and the floor height
        // under the body: the landing aim (FallProtectSettings::mAimLanding).
        bool mHasHead = false;
        FallVec3 mHead;
        FallVec3 mHeadVelocity;
        FallVec3 mShoulders;
        float mFloorHeight = 0.0f;
        // [0] left, [1] right (mProbeAim, mArmYield, mContactTuck).
        std::array<FallArmInput, 2> mArms{};
        // Something but the feet and the arms touches (mArmYield).
        bool mBodyContact = false;
        std::uint32_t mSeed = 0; // mVariation, read at the fall's first step
        FallProtectSettings mSettings;
    };

    struct FallProtectCommand
    {
        FallPhase mPhase = FallPhase::Reacting;
        FallKind mKind = FallKind::Forward;
        FallVec3 mDirection; // horizontal unit, world
        FallVec3 mArmDirection; // the upper arms' target axis, world unit
        // Each arm's own axis (mArmDirection bent to its probe hit and spread).
        std::array<FallVec3, 2> mArmDirections{};
        std::array<float, 2> mArmWeight{ 0.0f, 0.0f }; // left, right, 0-1
        // Each arm's stiffness (mArmStiffness unless mArmYield softens it).
        std::array<float, 2> mArmStiffnesses{ 1.0f, 1.0f };
        std::array<bool, 2> mArmTouched{ false, false };
        float mHeadPitch = 0.0f; // rad, + forward
        float mSpineCurl = 0.0f; // rad, + forward, spread over the spine
        float mKneeBend = 0.0f; // rad, + flexion (the foot goes back)
        float mHipBend = 0.0f; // rad, + flexion (the knee goes forward)
        float mArmStiffness = 1.0f;
        float mUpperStiffness = 1.0f;
        float mLegStiffness = 1.0f;
        float mTime = 0.0f; // since the alert or the fall began
        bool mFallen = true; // false: an alert, the arms only
        // This fall's kind is not protected (FallProtectSettings::mProtect*):
        // the caller lets the body go limp (motors off) as before.
        bool mLimp = false;
        float mImpactTime = -1.0f; // since the first contact, -1 before
    };

    // Starts on the first update after reset(); the caller calls it every step
    // while a fall is predicted (mFallen false) or the body is fallen, and
    // resets it when the alert clears or the get-up takes over.
    class FallProtectController
    {
    public:
        FallProtectCommand update(const FallProtectInput& input);
        void reset() { *this = FallProtectController(); }
        FallPhase phase() const { return mPhase; }

    private:
        FallPhase mPhase = FallPhase::Reacting;
        float mTime = 0.0f;
        float mAlertTime = 0.0f; // the clock when the fall came after an alert
        float mImpactTime = -1.0f;
        float mSlowTime = 0.0f;
        bool mHasDirection = false;
        // The kind is latched when bracing starts: as the trunk lies down its
        // forward axis turns vertical and the horizontal classification flips
        // (game trace 10-02: forward -> side -> backward -> side within 0.6 s,
        // flipping the head pitch and spine curl in mid-air).
        bool mKindLatched = false;
        // The body's forward at the fall's first step (horizontal): the kind is
        // judged against it, not the live trunk, which pitches and rolls (game
        // trace 10-02: a forward chest throw latched as backward).
        bool mHasStartForward = false;
        float mStartForwardX = 0.0f, mStartForwardY = 1.0f;
        FallKind mKind = FallKind::Forward;
        float mDirectionX = 0.0f;
        float mDirectionY = 1.0f;
        std::array<float, 2> mArmTouchTime{ -1.0f, -1.0f }; // since each hand touched
        float mTone = 1.0f; // the filtered speed share (mToneSpeed)
        bool mHasSpread = false;
        // The fall's spread (mVariation): spine, hip, knee, head (rad, at
        // full variation), each arm's reach turn (rad, about world up).
        std::array<float, 4> mBendSpread{};
        std::array<float, 2> mReachSpread{};
    };

    // Fall prediction from the torso's tilt: the tilt looked ahead by its
    // filtered rate. Risk ramps from mOnset to mFull (radians, predicted),
    // zero below mMinimumTilt actual tilt; the alert holds until the risk has
    // been zero for mClearTime.
    struct FallPredictSettings
    {
        float mLookahead = 0.3f; // s
        // Normal motion leans the torso (pelvis to head) up to 19 degrees
        // walking, 32 running backward with rate spikes of 216 deg/s, and a
        // caught 18000 push holds 49 (offline clips): the gates sit above it,
        // still ahead of the topple (60 degrees held 0.15 s).
        float mOnset = 0.87f; // 50 degrees predicted
        float mFull = 1.13f; // 65 degrees predicted
        float mMinimumTilt = 0.61f; // 35 degrees
        float mRateFilter = 20.0f; // 1/s
        float mClearTime = 0.3f; // s
    };

    class FallPredictor
    {
    public:
        // torsoUpDot: the torso's up axis . world up.
        float update(float torsoUpDot, float dt, const FallPredictSettings& settings = {});
        bool alert() const { return mAlert; }
        float rate() const { return mRate; }
        void reset() { *this = FallPredictor(); }

    private:
        bool mHas = false;
        bool mAlert = false;
        float mTilt = 0.0f;
        float mRate = 0.0f;
        float mClear = 0.0f;
    };

    // The topple rule: when a tipped torso counts as fallen. The game's rule
    // is tilt past mUpDot held mHold seconds. mRising counts the hold only
    // while the torso still tips further (offline, PGC, near the catch limit:
    // falls 93 -> 44 of 224, 0-step falls 62 -> 22; the rescues are 3-4 step
    // staggers ending upright); mHardUpDot falls at once below it (-1 = off).
    struct ToppleSettings
    {
        float mUpDot = 0.5f; // 60 degrees
        float mHold = 0.15f; // s
        bool mRising = false;
        float mHardUpDot = -1.0f;
    };

    class ToppleJudge
    {
    public:
        // torsoUpDot: the torso's up axis . world up. True once toppled
        // (latched until reset()).
        bool update(float torsoUpDot, float dt, const ToppleSettings& settings = {});
        bool toppled() const { return mToppled; }
        float held() const { return mHeld; }
        void reset() { *this = ToppleJudge(); }

    private:
        bool mHas = false;
        bool mToppled = false;
        float mLast = 1.0f;
        float mHeld = 0.0f;
    };

    // Skill -> fall protection (the user's skill design: low skill is
    // clumsy). s = (0.6 Agility + 0.4 Acrobatics) / 100, clamped 0-1:
    // reaction 0.30-0.05 s, bracing arms 0.5-1.2 of the profile spring, the
    // landing aim 0.3-1.0 (a clumsy body half reaches along the fall).
    FallProtectSettings fallProtectSettings(float agility, float acrobatics, FallProtectSettings base = {});

    // The fall's kind from its horizontal direction and the body's forward
    // axis: within 45 degrees of forward or backward, else a side fall.
    FallKind classifyFall(FallVec3 direction, FallVec3 bodyForward);
}

#endif

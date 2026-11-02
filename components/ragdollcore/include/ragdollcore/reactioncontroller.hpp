#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_REACTIONCONTROLLER_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_REACTIONCONTROLLER_H

// RXN-02 M3: the reaction controller (docs/ragdoll/designs/
// RXN-02-reaction-controller.md 2.4; numbers from human reaction
// measurements). One per powered body. Each
// counted blow becomes a track with its own clock; per step the controller
// returns what the body should do this step:
//  - Impact (0-60 ms, physics): the struck chain gives (a compliance lease
//    that recovers over 0.3-0.4 s), a snap on the trunk and neck along the
//    blow, a 0.7x reverse snap on heavy blows;
//  - Reflex (from 60-90 ms): additive pose offsets by region, each with a
//    human-timed envelope;
//  - Action: the tier, poise (reaction leakage) and whether the victim's own
//    attack breaks;
//  - Balance: the disturbance PGC decides from (the controller never steps).
// Pure: Jolt math only, no engine or game types. Units are the caller's
// (game units, kg, seconds); thresholds given in m/s are converted with
// sUnitsPerMetre.

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
// clang-format on

#include <ragdollcore/anatomy.hpp>
#include <ragdollcore/blow.hpp>
#include <ragdollcore/bodylease.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace RagdollCore
{
    inline constexpr float sReactionUnitsPerMetre = 70.0f;

    enum class ReactionTier : std::uint8_t
    {
        Absorb, // Impact only
        Flinch, // + a reflex; the guard stays up
        Stagger, // + catch steps; the weapon arm loses the wheel for a while
        Down, // set by balance (PGC), never by the controller alone
    };
    const char* reactionTierName(ReactionTier tier);

    // What a blow did to the body's momentum (Komura 2005), measured or
    // estimated by the caller from the impulse it applied.
    struct BlowDisturbance
    {
        JPH::Vec3 mComVelocityChange = JPH::Vec3::sZero(); // u/s
        JPH::Vec3 mAngularMomentumChange = JPH::Vec3::sZero(); // about the COM, kg u^2/s
        float mPartVelocityChange = 0.0f; // the struck body's dv, u/s
        // Unit vectors in world space: the body's facing and up.
        JPH::Vec3 mBodyForward = JPH::Vec3::sAxisY();
        JPH::Vec3 mBodyUp = JPH::Vec3::sAxisZ();
    };

    // The disturbance of an impulse J at point p on a body of total mass M
    // and centre of mass c, the struck part of mass m.
    BlowDisturbance blowDisturbance(JPH::Vec3Arg impulse, JPH::Vec3Arg leverFromCom, float totalMass,
        float partMass, JPH::Vec3Arg bodyForward, JPH::Vec3Arg bodyUp);

    struct ReactionSettings
    {
        // Tiers from the COM's dv (m/s): absorb < flinch < stagger.
        float mFlinchDv = 0.3f;
        float mStaggerDv = 0.8f;
        // The struck part's dv (m/s) that makes a local blow at least a
        // flinch whatever the COM did (a head or a hand knocked hard).
        float mLocalFlinchDv = 2.5f;
        float mLocalStaggerDv = 6.0f; // head only: a daze-strength blow
        // Balance severity reference: the COM dv (m/s) that needs a step,
        // by direction (backward, forward, sideways; LIP margins).
        float mStepDvBack = 0.18f;
        float mStepDvForward = 0.42f;
        float mStepDvSide = 0.46f;

        // Impact: the struck chain's torque scale falls to between these
        // floors by local severity, holds, then ramps back.
        float mGiveFloorLight = 0.6f;
        float mGiveFloorHeavy = 0.15f;
        float mGiveHold = 0.12f; // s, at heavy (scaled down for lighter)
        float mGiveRamp = 0.35f; // s
        float mNeckGiveFloor = 0.3f; // head blows
        float mLegFloor = 0.8f; // legs never below this on non-leg blows
        // Snap: an angular velocity change on the trunk and the head along
        // the blow (rad/s at a heavy blow), a reverse snap of this ratio
        // after the interval on heavy blows.
        float mSnapTrunk = 4.0f;
        float mSnapHead = 22.0f;
        float mUnsnapRatio = 0.7f;
        float mUnsnapInterval = 0.08f;
        float mHeadSpinCap = 30.0f; // rad/s, never Jolt's 47

        // Reflex timing (s from contact) and sizes (rad at a heavy blow;
        // REACT-02 10-07: the human-sized first guesses read as stoic).
        float mReflexOnset = 0.075f;
        float mArmOnset = 0.12f;
        float mReflexRise = 0.12f;
        float mReflexHold = 0.08f;
        float mReflexRelease = 0.3f;
        float mHeadAway = 0.7f;
        float mTrunkFold = 0.55f;
        float mTrunkTurn = 0.4f;
        float mArmFling = 1.4f;
        float mArmFlingHold = 0.35f;
        float mArmsUpHold = 0.35f; // covering the face holds longer than a flinch
        float mKneeBend = 0.6f;
        float mLegWeaknessDelay = 0.25f;
        float mLegWeaknessTime = 0.4f;
        float mLegWeakness = 0.7f; // torque scale on the struck leg
        // Leg give-out (RXN-02 M5, injury, not momentum: a sword's momentum
        // rarely fells anyone - REACT-03 10-07, 0 falls in 40 random blows -
        // a deep cut to the leg does). A leg blow's injury is its damage over
        // `mLegInjuryDamage` (0 = off), 0-1; at full injury the struck leg's
        // weakness falls to `mLegGiveOutWeakness` and lasts
        // `mLegGiveOutTime`, the knee bends to `mLegGiveOutKnee` and the
        // engine drops the body by the give-out (ReactionStepOutput::
        // mLegGiveOut) - a knee going, which a body already off balance
        // cannot catch.
        float mLegInjuryDamage = 0.0f;
        float mLegGiveOutWeakness = 0.15f;
        float mLegGiveOutTime = 1.2f;
        float mLegGiveOutKnee = 1.3f;
        // Bearings (user 10-07: "the opponent should take a second to get
        // their bearings ... stagger logic based on stamina and health"): a
        // stagger opens a window of `mBearingsBase` + `mBearingsPerSeverity`
        // x severity seconds (0 = off), x (1 + tired x missing fatigue) x
        // (1 + hurt x missing health), at most `mBearingsMax`. Meanwhile the
        // weapon arm is busy (no attack, no guard), the legs and the neck
        // are a little loose (PGC steps it). A tired or hurt body staggers
        // sooner: the tier thresholds x (floor + (1 - floor) x min(fatigue,
        // health)). Blunt blows are concussive: local severity and the
        // window x `mBluntConcussion`.
        float mBearingsBase = 0.0f;
        float mBearingsPerSeverity = 0.6f;
        float mBearingsTired = 1.0f;
        float mBearingsHurt = 0.6f;
        float mBearingsMax = 2.5f;
        float mPoiseFloor = 0.55f;
        float mBluntConcussion = 1.3f;
        float mBearingsLegs = 0.85f;
        float mBearingsNeck = 0.7f;

        // Poise as reaction leakage: each blow multiplies the
        // next reflex and interrupt by (1 - reduction x severity), back at
        // `recovery` per second, never below the floor. Balance is never
        // scaled.
        float mLeakageReduction = 0.35f;
        float mLeakageRecovery = 0.6f;
        float mLeakageFloor = 0.2f;
        // Reflex debounce: a new blow inside this gap only starts a reflex
        // when it is a higher tier than the one running.
        float mReflexGap = 0.35f;
        // Hyper armour: committed swings (cut stage, or a wind-up past this
        // share) break only at Stagger.
        float mCommittedWindShare = 0.6f;
        // At most one stagger interrupt from the same attacker in this
        // window unless the blow is stronger.
        float mStaggerWindow = 2.0f;

        // Output smoothing: the share of the new value per step (DReCon's
        // 0.2 filter, at 60 Hz).
        float mFilter = 0.35f;
    };

    // What the victim was doing this step (refreshed every step).
    struct ReactionVictim
    {
        bool mAttacking = false;
        bool mCutting = false; // the cut stage itself
        float mWindShare = 0.0f; // 0-1 into the wind-up
        bool mGuarding = false;
        bool mDown = false;
        bool mStaggering = false; // balance says a stagger is running
        float mFatigue = 1.0f; // current over base, 0-1
        float mHealth = 1.0f;
    };

    // Additive pose offsets for this step, rotation vectors (rad) in the
    // body frame (x right, y forward, z up), applied by the engine adapter
    // at the named chains.
    struct ReflexPose
    {
        JPH::Vec3 mHead = JPH::Vec3::sZero(); // at the neck and head
        JPH::Vec3 mTrunk = JPH::Vec3::sZero(); // spread over the spine
        std::array<JPH::Vec3, 2> mArm{ JPH::Vec3::sZero(), JPH::Vec3::sZero() }; // at the shoulders, [0] left
        float mArmsUp = 0.0f; // 0-1 toward covering the face
        std::array<float, 2> mKneeBend{ 0.0f, 0.0f }; // rad, [0] left
        bool any() const;
    };

    struct ReactionStepOutput
    {
        ReflexPose mReflex;
        // Angular velocity changes to add this step (world space, rad/s).
        JPH::Vec3 mTrunkSnap = JPH::Vec3::sZero();
        JPH::Vec3 mHeadSnap = JPH::Vec3::sZero();
        // Torque scales per channel from Impact and leg weakness (1 = none).
        std::array<float, sBodyChannelCount> mTorqueScale{};
        ReactionTier mTier = ReactionTier::Absorb; // the highest running
        bool mInterrupt = false; // break the victim's own attack this step
        // The weapon arm is busy (fling, stagger): no wheel until then.
        bool mWeaponArmBusy = false;
        float mLeakage = 1.0f;
        // How far each leg has given out (0-1, [0] left): its injury times
        // its weakness envelope. The engine lowers the body by it.
        std::array<float, 2> mLegGiveOut{ 0.0f, 0.0f };
        // Seconds of a stagger's bearings left (0 = none).
        float mBearings = 0.0f;
    };

    // One blow as the controller saw it.
    struct ReactionEvent
    {
        std::uint64_t mBlowId = 0;
        ReactionTier mTier = ReactionTier::Absorb;
        float mScale = 1.0f; // leakage applied to its reflex and interrupt
        bool mReflex = false; // started a reflex (not debounced)
        bool mInterrupt = false;
        float mBalanceSeverity = 0.0f; // COM dv over the step threshold
        float mLocalSeverity = 0.0f; // struck part dv over its reference
        float mInjury = 0.0f; // leg blows: damage over mLegInjuryDamage, 0-1
    };

    class ReactionController
    {
    public:
        explicit ReactionController(ReactionSettings settings = {})
            : mSettings(settings)
        {
        }

        // A counted blow. The disturbance is what it did to the body.
        ReactionEvent onBlow(const Blow& blow, const BlowDisturbance& disturbance, const ReactionVictim& victim);
        // Advances every track by dt and returns this step's requests.
        ReactionStepOutput step(float dt, const ReactionVictim& victim);

        void reset();
        const ReactionSettings& settings() const { return mSettings; }
        // Live tuning: new settings for the tracks still running and the next blows.
        void setSettings(const ReactionSettings& settings) { mSettings = settings; }
        std::size_t activeTracks() const { return mTracks.size(); }
        float leakage() const { return mLeakage; }

    private:
        struct Track
        {
            std::uint64_t mBlowId = 0;
            std::uint64_t mAttacker = 0;
            AnatomicalRegion mRegion = AnatomicalRegion::Unknown;
            ReactionTier mTier = ReactionTier::Absorb;
            float mAge = 0.0f;
            float mLocal = 0.0f; // 0-1 local severity (0 light .. 1 heavy)
            float mScale = 1.0f; // leakage
            float mInjury = 0.0f; // leg give-out, 0-1 (never scaled by leakage)
            bool mReflex = false;
            bool mUnsnapped = false;
            bool mSnapped = false;
            JPH::Vec3 mDirection = JPH::Vec3::sZero(); // world, into the victim
            JPH::Vec3 mBodyDirection = JPH::Vec3::sZero(); // body frame
            JPH::Vec3 mSpin = JPH::Vec3::sZero(); // body frame, sign of dL
            JPH::Vec3 mWorldSpin = JPH::Vec3::sZero();
        };
        float envelope(float age, float onset, float hold) const;
        // A struck leg's weakness window: its weakness and how far into it.
        float legWeaknessTime(const Track& track) const;
        float lifetime(const Track& track) const;

        ReactionSettings mSettings;
        std::vector<Track> mTracks;
        float mLeakage = 1.0f;
        float mSinceReflex = 1e9f;
        ReactionTier mReflexTier = ReactionTier::Absorb;
        std::uint64_t mLastStaggerAttacker = 0;
        float mSinceStagger = 1e9f;
        float mLastStaggerSeverity = 0.0f;
        ReflexPose mFiltered;
        bool mPendingInterrupt = false;
        float mBearingsLeft = 0.0f;
    };
}

#endif

#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_WEAPONCONTACT_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_WEAPONCONTACT_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include <ragdollcore/anatomy.hpp>
#include <ragdollcore/physicaldefense.hpp>
#include <ragdollcore/wound.hpp>

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>
#include <Jolt/Math/Quat.h>
// clang-format on

namespace RagdollCore
{
    // Value-only contact-time projection for the supported stock blade frames.
    WoundKind classifyBladeContactSnapshot(std::string_view record, JPH::QuatArg rotation,
        JPH::Vec3Arg pointFromCenter, JPH::Vec3Arg centerFromHand, float length,
        JPH::Vec3Arg normal, JPH::Vec3Arg relativeVelocity);

    struct WoundContactSnapshot
    {
        WoundKind mKind = WoundKind::None;
        JPH::Vec3 mTargetLocalPoint = JPH::Vec3::sZero();
        JPH::Vec3 mTargetLocalNormal = JPH::Vec3::sAxisY();
        JPH::Vec3 mTargetLocalTangent = JPH::Vec3::sAxisZ();
        JPH::Vec3 mRelativeVelocity = JPH::Vec3::sZero();
    };
    // A complete value snapshot made while Jolt owns the contact callback.
    // It deliberately contains no Body, PtrHolder, MWWorld pointer, or other
    // borrowed state: consumers may safely retain it after either body dies.
    class WeaponContactEvent final
    {
    public:
        static constexpr std::uint32_t sInvalidBodyIndex = std::numeric_limits<std::uint32_t>::max();

        WeaponContactEvent(std::uint64_t contactIdentity, std::uint64_t attackerActorIdentity,
            std::optional<std::uint64_t> targetActorIdentity, std::string weaponRecordId,
            std::uint32_t weaponBodyIdentity, std::uint32_t targetBodyIdentity, std::uint64_t physicsStep,
            std::uint64_t swingIdentity, bool swingActive, JPH::RVec3Arg point, JPH::Vec3Arg normal,
            float relativeNormalSpeed, float relativeSpeed, float effectiveMass,
            std::uint32_t targetBodyIndex = sInvalidBodyIndex,
            AnatomicalRegion region = AnatomicalRegion::Unknown,
            std::uint64_t targetRegistryGeneration = 0, WoundContactSnapshot wound = {});

        bool valid() const noexcept;

        std::uint64_t contactIdentity() const noexcept { return mContactIdentity; }
        std::uint64_t attackerActorIdentity() const noexcept { return mAttackerActorIdentity; }
        std::optional<std::uint64_t> targetActorIdentity() const noexcept { return mTargetActorIdentity; }
        const std::string& weaponRecordId() const noexcept { return mWeaponRecordId; }
        std::uint32_t weaponBodyIdentity() const noexcept { return mWeaponBodyIdentity; }
        std::uint32_t targetBodyIdentity() const noexcept { return mTargetBodyIdentity; }
        std::uint64_t physicsStep() const noexcept { return mPhysicsStep; }
        std::uint64_t swingIdentity() const noexcept { return mSwingIdentity; }
        bool swingActive() const noexcept { return mSwingActive; }
        JPH::RVec3 point() const noexcept { return mPoint; }
        JPH::Vec3 normal() const noexcept { return mNormal; }
        float relativeNormalSpeed() const noexcept { return mRelativeNormalSpeed; }
        float relativeSpeed() const noexcept { return mRelativeSpeed; }
        float effectiveMass() const noexcept { return mEffectiveMass; }
        float damageSpeed() const noexcept
        {
            return mDamageSpeed >= 0 ? mDamageSpeed : std::max(mRelativeNormalSpeed, mSliceSpeed);
        }
        // A slice: an edge dragged through a body it already touches (low
        // normal speed, high sliding speed) is priced at this speed.
        WeaponContactEvent withSliceSpeed(float speed) const
        {
            auto copy = *this;
            if (std::isfinite(speed) && speed >= 0) copy.mSliceSpeed = speed;
            return copy;
        }
        bool isSlice() const noexcept { return mSliceSpeed > mRelativeNormalSpeed; }
        float damageFraction() const noexcept { return mDamageFraction; }
        // Solver impulse, in kg * world units / second. Unknown for contacts
        // without a cached manifold (e.g. CCD); unknown never funds a draw cut.
        std::optional<float> normalImpulse() const noexcept { return mNormalImpulse; }
        WeaponContactEvent withNormalImpulse(float impulse) const
        {
            auto copy = *this;
            if (std::isfinite(impulse) && impulse >= 0) copy.mNormalImpulse = impulse;
            return copy;
        }
        /// True when the fraction is measured contact work rather than the
        /// legacy default: damage then scales from zero with that fraction.
        bool hasMeasuredWork() const noexcept { return mDamageSpeed >= 0; }
        /// share: this event's part of the blow (DMG-04) - the regional split
        /// of one impact, or a cut's installment; cut marks an edge draw.
        WeaponContactEvent withDamageMeasure(float speed, float fraction, float share = 1, bool cut = false) const
        {
            auto copy = *this;
            copy.mDamageSpeed = speed;
            copy.mDamageFraction = fraction;
            copy.mDamageShare = share;
            copy.mDamageIsCut = cut;
            return copy;
        }
        float damageShare() const noexcept { return mDamageShare; }
        bool damageIsCut() const noexcept { return mDamageIsCut; }
        /// Ragdoll-space body index of the struck body, resolved at contact
        /// time. Invalid when the target has no supported anatomical body.
        std::uint32_t targetBodyIndex() const noexcept { return mTargetBodyIndex; }
        AnatomicalRegion region() const noexcept { return mRegion; }
        const WoundContactSnapshot& wound() const noexcept { return mWound; }
        /// Spawn/lifecycle generation the region was resolved against. A
        /// consumer that sees a different current generation must reject the
        /// event rather than apply a stale region.
        std::uint64_t targetRegistryGeneration() const noexcept { return mTargetRegistryGeneration; }
        bool matchesGeneration(std::uint64_t current) const noexcept
        {
            return mTargetRegistryGeneration == current;
        }

    private:
        std::uint64_t mContactIdentity;
        std::uint64_t mAttackerActorIdentity;
        std::optional<std::uint64_t> mTargetActorIdentity;
        std::string mWeaponRecordId;
        std::uint32_t mWeaponBodyIdentity;
        std::uint32_t mTargetBodyIdentity;
        std::uint64_t mPhysicsStep;
        std::uint64_t mSwingIdentity;
        bool mSwingActive;
        JPH::RVec3 mPoint;
        JPH::Vec3 mNormal;
        float mRelativeNormalSpeed;
        float mRelativeSpeed;
        float mEffectiveMass;
        std::uint32_t mTargetBodyIndex;
        AnatomicalRegion mRegion;
        std::uint64_t mTargetRegistryGeneration;
        WoundContactSnapshot mWound;
        float mDamageSpeed = -1;
        float mSliceSpeed = -1;
        float mDamageFraction = 1;
        float mDamageShare = 1;
        bool mDamageIsCut = false;
        std::optional<float> mNormalImpulse;
    };

    // Contact callbacks can execute concurrently. The game thread is the only
    // consumer and takes ownership of a whole batch after PhysicsSystem::Update.
    class WeaponContactEventQueue final
    {
    public:
        bool publish(WeaponContactEvent event);
        std::vector<WeaponContactEvent> consume();
        std::size_t pending() const;

    private:
        mutable std::mutex mMutex;
        std::vector<WeaponContactEvent> mPending;
    };

    // Stateful game-thread filter for raw physics observations from one
    // attacker. A target actor may expose several Jolt bodies and manifolds;
    // those observations still describe one strike during one swing.
    class WeaponStrikeFilter final
    {
    public:
        // Low enough that a driven swing whose contact arrives at a modest
        // approach rate still prices a blow; the AI strike pushes the weapon
        // out and the contact is often taken up over several steps.
        static constexpr float sDefaultMinimumApproachSpeed = 0.2f;

        explicit WeaponStrikeFilter(float minimumApproachSpeed = sDefaultMinimumApproachSpeed);

        bool accept(const WeaponContactEvent& event);
        std::vector<WeaponContactEvent> consume(std::vector<WeaponContactEvent> events);
        void reset() noexcept;

        float minimumApproachSpeed() const noexcept { return mMinimumApproachSpeed; }

    private:
        struct Target final
        {
            std::optional<std::uint64_t> mActorIdentity;
            std::uint32_t mBodyIdentity = 0;
        };

        bool sameTarget(const Target& target, const WeaponContactEvent& event) const noexcept;

        float mMinimumApproachSpeed;
        std::uint64_t mAttackerActorIdentity = 0;
        std::uint32_t mWeaponBodyIdentity = 0;
        std::uint64_t mSwingIdentity = 0;
        std::vector<Target> mStruckTargets;
    };

    // Prototype material parameters for measured contact injuries. Event speeds
    // are world units/second and masses kilograms; energies are joules.
    // These are gameplay material parameters, not a soft-tissue calibration.
    struct ContactWorkProfile final
    {
        // Contact-axis kinetic energy below which a touch is not a blow.
        float mMinimumImpactEnergy = 0.06f;
        float mFullImpactEnergy = 0.6f;
        // Net edge speed along the surface: slower drift never cuts, and the
        // full-depth speed prices a slow drag as a scratch.
        float mMinimumCutSpeed = 20.0f;
        float mFullCutSpeed = 150.0f;
        // Net travel before any cut is paid: contact chatter of smaller
        // amplitude is not a draw, whatever its instantaneous speed.
        float mMinimumCutTravel = 4.0f;
        float mTravelPerFullCut = 12.0f;
        // Smallest injury worth a receipt, and the larger amount of fresh work
        // a further cut needs once this contact episode has already injured.
        float mMinimumInjuryFraction = 0.1f;
        float mRepeatInjuryFraction = 0.5f;
        // Cutting work is reduced below this measured normal load (newtons).
        float mFullCutLoad = 1.0f;
        // Shape clearance in world units, checked against the whole victim.
        // Missing callbacks or elapsed time alone cannot rearm an impact.
        float mSeparationDistance = 2.0f;
        float mCutMemory = 0.2f;
        // A later impact in the same episode at least this many times the
        // energy already paid pays the difference (0 = one impact per episode).
        // A first touch spent the episode's only impact, so the real blow that
        // followed it paid nothing (CONNECT-01, build 61 bookmarks).
        float mTopUpFactor = 0.0f;
    };

    // Supported dagger/fist damage follows contact episodes, independently of
    // input swing IDs. An injury is paid for by measured physical work: the
    // contact-axis kinetic energy of an impact, or net edge travel drawn along
    // the surface. Resting or pressing overlap, jitter, callbacks and time
    // alone earn nothing. The emitted damage fraction is that work relative to
    // a full injury, from zero; there is no floor for a qualifying contact.
    class ContactDamageTracker final
    {
    public:
        using Pair = std::tuple<std::uint64_t, std::uint32_t, std::uint64_t, std::uint64_t>;
        explicit ContactDamageTracker(const ContactWorkProfile& profile = {});

        std::vector<WeaponContactEvent> consume(
            const std::vector<WeaponContactEvent>& contacts, std::uint64_t step, float duration);
        void reset() { mContacts.clear(); mLastStep = 0; }
        std::vector<Pair> activePairs() const;
        // Adapter calls only after proving shape clearance or body destruction.
        void confirmSeparation(const Pair& pair) { mContacts.erase(pair); }

        const ContactWorkProfile& profile() const noexcept { return mProfile; }

    private:
        struct Contact
        {
            // Net, signed travel: oscillation cancels instead of adding up.
            JPH::Vec3 mCutTravel = JPH::Vec3::sZero();
            JPH::Vec3 mLoadedTravel = JPH::Vec3::sZero();
            float mCutTime = 0;
            std::uint32_t mCutBody = WeaponContactEvent::sInvalidBodyIndex;
            bool mImpactSpent = false;
            bool mInjured = false;
            float mSpentEnergy = 0;
        };
        ContactWorkProfile mProfile;
        std::uint64_t mLastStep = 0;
        std::map<Pair, Contact> mContacts;
    };

    // Legacy selection retained for weapons without a supported surface profile.
    // Deterministic one-strike selection for localized hits.
    //
    // Every qualifying observation for one target actor during one swing is
    // collected. The strike commits from the earliest contacting physics step;
    // within that step the largest bounded normal-impact score wins, with
    // stable region / body / point tie breakers, so callback arrival order
    // cannot decide where a blow lands. Once a swing has committed a strike
    // for a target actor, later observations cannot produce another.
    class LocalizedStrikeAggregator final
    {
    public:
        // A contact rejected as rest by defense must not spend the swing's
        // only hit before a later real impact reaches that same actor.
        static constexpr float sDefaultMinimumApproachSpeed = PhysicalDefenseProfile{}.mMinimumNormalSpeed;

        explicit LocalizedStrikeAggregator(float minimumApproachSpeed = sDefaultMinimumApproachSpeed);

        // Qualification is the same contract the weapon strike filter applies:
        // valid, an active swing with identity, and a minimum approach speed.
        bool qualifies(const WeaponContactEvent& event) const noexcept;
        // Records a qualifying observation. Returns false when it does not
        // qualify or its target actor already committed this swing.
        bool observe(const WeaponContactEvent& event);
        // CONNECT-01: the strike is the strongest qualifying contact over this
        // many steps from the first one (0 = the first step's best), and a
        // contact must approach at least this fast to qualify at all.
        void setSelectionWindow(std::uint64_t steps) noexcept { mWindow = steps; }
        void setMinimumApproachSpeed(float speed) noexcept
        {
            if (std::isfinite(speed) && speed >= 0.0f)
                mMinimumApproachSpeed = speed;
        }
        // Slices (point-blank: the blade already rests in the body): a
        // contact under the approach speed still qualifies when its sliding
        // speed reaches this (0 = off), priced at `share` of that speed.
        void setSlice(float speed, float share) noexcept
        {
            if (std::isfinite(speed) && speed >= 0.0f)
                mMinimumSliceSpeed = speed;
            if (std::isfinite(share) && share >= 0.0f)
                mSliceShare = share;
        }
        // Emits the committed strike per target actor for steps at or before
        // \a physicsStep and forgets them. Observations for later steps are
        // held until that step is consumed.
        std::vector<WeaponContactEvent> consume(std::uint64_t physicsStep);
        // Drops any event whose generation no longer matches the live one.
        void rejectStaleGeneration(std::uint64_t actorIdentity, std::uint64_t currentGeneration);
        void reset() noexcept;

        std::size_t pending() const noexcept { return mPending.size(); }

    private:
        struct Key final
        {
            std::uint64_t mSwingIdentity = 0;
            std::uint64_t mAttackerActorIdentity = 0;
            std::uint64_t mTargetActorIdentity = 0;

            bool operator<(const Key& other) const noexcept
            {
                return std::tie(mSwingIdentity, mAttackerActorIdentity, mTargetActorIdentity)
                    < std::tie(other.mSwingIdentity, other.mAttackerActorIdentity, other.mTargetActorIdentity);
            }
        };

        struct Pending final
        {
            std::uint64_t mStep = 0;
            float mScore = -1.0f;
            WeaponContactEvent mEvent;
        };

        static float impactScore(const WeaponContactEvent& event) noexcept;
        static bool betterOf(const WeaponContactEvent& left, const WeaponContactEvent& right) noexcept;

        float mMinimumApproachSpeed;
        float mMinimumSliceSpeed = 0.0f;
        float mSliceShare = 0.6f;
        std::uint64_t mWindow = 0;
        std::map<Key, Pending> mPending;
        std::set<Key> mCommitted;
    };
}

#endif

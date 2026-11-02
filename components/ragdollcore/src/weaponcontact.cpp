#include <ragdollcore/weaponcontact.hpp>

#include <ragdollcore/unitmodel.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace RagdollCore
{
    namespace
    {
        bool finite(JPH::RVec3Arg value)
        {
            return std::isfinite(value.GetX()) && std::isfinite(value.GetY()) && std::isfinite(value.GetZ());
        }

        bool finite(JPH::Vec3Arg value)
        {
            return std::isfinite(value.GetX()) && std::isfinite(value.GetY()) && std::isfinite(value.GetZ());
        }
    }

    WoundKind classifyBladeContactSnapshot(std::string_view record, JPH::QuatArg rotation,
        JPH::Vec3Arg pointFromCenter, JPH::Vec3Arg centerFromHand, float length,
        JPH::Vec3Arg normal, JPH::Vec3Arg relativeVelocity)
    {
        const auto profile = bladeSurfaceProfile(record);
        const float gripToTip = centerFromHand.GetZ() + 0.5f * length;
        if (!profile || !std::isfinite(gripToTip) || gripToTip <= 0)
            return WoundKind::None;
        const JPH::Quat inverse = rotation.Conjugated();
        const JPH::Vec3 local = inverse * pointFromCenter + centerFromHand;
        const JPH::Vec3 n = inverse * normal;
        const JPH::Vec3 v = inverse * relativeVelocity;
        return classifyBladeContact({local.GetZ() / gripToTip, n.GetZ(), n.GetX(), n.GetY(),
            v.GetZ(), relativeVelocity.Length(), relativeVelocity.Dot(normal)}, *profile);
    }

    WeaponContactEvent::WeaponContactEvent(std::uint64_t contactIdentity, std::uint64_t attackerActorIdentity,
        std::optional<std::uint64_t> targetActorIdentity, std::string weaponRecordId,
        std::uint32_t weaponBodyIdentity, std::uint32_t targetBodyIdentity, std::uint64_t physicsStep,
        std::uint64_t swingIdentity, bool swingActive, JPH::RVec3Arg point, JPH::Vec3Arg normal,
        float relativeNormalSpeed, float relativeSpeed, float effectiveMass,
        std::uint32_t targetBodyIndex, AnatomicalRegion region, std::uint64_t targetRegistryGeneration,
        WoundContactSnapshot wound)
        : mContactIdentity(contactIdentity)
        , mAttackerActorIdentity(attackerActorIdentity)
        , mTargetActorIdentity(targetActorIdentity)
        , mWeaponRecordId(std::move(weaponRecordId))
        , mWeaponBodyIdentity(weaponBodyIdentity)
        , mTargetBodyIdentity(targetBodyIdentity)
        , mPhysicsStep(physicsStep)
        , mSwingIdentity(swingIdentity)
        , mSwingActive(swingActive)
        , mPoint(point)
        , mNormal(normal)
        , mRelativeNormalSpeed(relativeNormalSpeed)
        , mRelativeSpeed(relativeSpeed)
        , mEffectiveMass(effectiveMass)
        , mTargetBodyIndex(targetBodyIndex)
        , mRegion(region)
        , mTargetRegistryGeneration(targetRegistryGeneration)
        , mWound(wound)
    {
    }

    bool WeaponContactEvent::valid() const noexcept
    {
        constexpr std::uint32_t invalidBody = std::numeric_limits<std::uint32_t>::max();
        // An empty weapon record is a bare-handed strike: the weapon body is
        // the fist itself, and the contact still carries everything the
        // damage and reaction paths read.
        return mContactIdentity != 0 && mAttackerActorIdentity != 0
            && (!mTargetActorIdentity || *mTargetActorIdentity != mAttackerActorIdentity)
            && mWeaponBodyIdentity != invalidBody && mTargetBodyIdentity != invalidBody
            && finite(mPoint) && finite(mNormal) && mNormal.LengthSq() > 0.5f
            && std::isfinite(mRelativeNormalSpeed) && std::isfinite(mRelativeSpeed) && mRelativeSpeed >= 0.0f
            && std::abs(mRelativeNormalSpeed) <= mRelativeSpeed + 1e-3f
            && std::isfinite(mEffectiveMass) && mEffectiveMass > 0.0f;
    }

    bool WeaponContactEventQueue::publish(WeaponContactEvent event)
    {
        if (!event.valid())
            return false;
        std::scoped_lock lock(mMutex);
        mPending.push_back(std::move(event));
        return true;
    }

    std::vector<WeaponContactEvent> WeaponContactEventQueue::consume()
    {
        std::vector<WeaponContactEvent> result;
        std::scoped_lock lock(mMutex);
        result.swap(mPending);
        return result;
    }

    std::size_t WeaponContactEventQueue::pending() const
    {
        std::scoped_lock lock(mMutex);
        return mPending.size();
    }

    WeaponStrikeFilter::WeaponStrikeFilter(float minimumApproachSpeed)
        : mMinimumApproachSpeed(minimumApproachSpeed)
    {
        if (!std::isfinite(minimumApproachSpeed) || minimumApproachSpeed < 0.0f)
            mMinimumApproachSpeed = sDefaultMinimumApproachSpeed;
    }

    bool WeaponStrikeFilter::sameTarget(const Target& target, const WeaponContactEvent& event) const noexcept
    {
        if (target.mActorIdentity || event.targetActorIdentity())
            return target.mActorIdentity == event.targetActorIdentity();
        return target.mBodyIdentity == event.targetBodyIdentity();
    }

    bool WeaponStrikeFilter::accept(const WeaponContactEvent& event)
    {
        if (!event.valid() || !event.swingActive() || event.swingIdentity() == 0
            || event.relativeNormalSpeed() < mMinimumApproachSpeed)
            return false;

        if (mAttackerActorIdentity != event.attackerActorIdentity()
            || mWeaponBodyIdentity != event.weaponBodyIdentity() || mSwingIdentity != event.swingIdentity())
        {
            mAttackerActorIdentity = event.attackerActorIdentity();
            mWeaponBodyIdentity = event.weaponBodyIdentity();
            mSwingIdentity = event.swingIdentity();
            mStruckTargets.clear();
        }

        if (std::any_of(mStruckTargets.begin(), mStruckTargets.end(),
                [&](const Target& target) { return sameTarget(target, event); }))
            return false;
        mStruckTargets.push_back({ event.targetActorIdentity(), event.targetBodyIdentity() });
        return true;
    }

    std::vector<WeaponContactEvent> WeaponStrikeFilter::consume(std::vector<WeaponContactEvent> events)
    {
        std::vector<WeaponContactEvent> result;
        result.reserve(events.size());
        for (auto& event : events)
            if (accept(event))
                result.push_back(std::move(event));
        return result;
    }

    void WeaponStrikeFilter::reset() noexcept
    {
        mAttackerActorIdentity = 0;
        mWeaponBodyIdentity = 0;
        mSwingIdentity = 0;
        mStruckTargets.clear();
    }

    ContactDamageTracker::ContactDamageTracker(const ContactWorkProfile& profile)
        : mProfile(profile)
    {
        const auto positive = [](float value) { return std::isfinite(value) && value > 0.0f; };
        if (!positive(profile.mMinimumImpactEnergy) || !positive(profile.mFullImpactEnergy)
            || !positive(profile.mMinimumCutSpeed) || !positive(profile.mFullCutSpeed)
            || !positive(profile.mMinimumCutTravel) || !positive(profile.mTravelPerFullCut)
            || !positive(profile.mMinimumInjuryFraction) || !positive(profile.mRepeatInjuryFraction)
            || !positive(profile.mFullCutLoad) || !positive(profile.mSeparationDistance)
            || !positive(profile.mCutMemory))
            mProfile = {};
    }

    std::vector<ContactDamageTracker::Pair> ContactDamageTracker::activePairs() const
    {
        std::vector<Pair> result;
        for (const auto& [pair, state] : mContacts) result.push_back(pair);
        return result;
    }

    std::vector<WeaponContactEvent> ContactDamageTracker::consume(
        const std::vector<WeaponContactEvent>& contacts, std::uint64_t step, float duration)
    {
        std::vector<WeaponContactEvent> result;
        if (!std::isfinite(duration) || duration <= 0 || step <= mLastStep)
            return result;
        mLastStep = step;
        using RegionKey = std::pair<Pair, std::uint64_t>;
        std::map<RegionKey, WeaponContactEvent> unique;
        const auto energy = [](const WeaponContactEvent& event) {
            const float speed = UnitModel::toMeters(std::max(0.0f, event.relativeNormalSpeed()));
            const float mass = event.effectiveMass();
            float impulse = mass * speed;
            // A cached solver response can only reduce the incoming energy.
            // CCD contacts without a cache retain the conservative incident-
            // energy estimate; they cannot fund continuous cutting below.
            if (event.normalImpulse()) impulse = std::min(impulse, UnitModel::toMeters(*event.normalImpulse()));
            return impulse * (speed - 0.5f * impulse / mass);
        };
        for (const auto& event : contacts)
        {
            if (!event.valid() || !event.targetActorIdentity() || event.physicsStep() != step
                || (!event.weaponRecordId().empty() && !bladeSurfaceProfile(event.weaponRecordId())))
                continue;
            const Pair pair{ event.attackerActorIdentity(), event.weaponBodyIdentity(),
                *event.targetActorIdentity(), event.targetRegistryGeneration() };
            const std::uint64_t region = event.region() == AnatomicalRegion::Unknown
                ? std::uint64_t(event.targetBodyIdentity()) + 256 : static_cast<std::uint64_t>(event.region());
            const RegionKey key{ pair, region };
            const auto score = [&](const WeaponContactEvent& value) {
                return std::tuple(energy(value), value.relativeSpeed(),
                    -static_cast<std::int64_t>(value.targetBodyIdentity()), -value.point().GetX(),
                    -value.point().GetY(), -value.point().GetZ(), -static_cast<std::int64_t>(value.contactIdentity()));
            };
            const auto found = unique.find(key);
            if (found == unique.end()) unique.emplace(key, event);
            else if (score(event) > score(found->second)) found->second = event;
        }
        std::map<Pair, std::vector<const WeaponContactEvent*>> groups;
        for (const auto& [key, event] : unique) groups[key.first].push_back(&event);
        for (auto& [pair, state] : mContacts)
        {
            if (!groups.contains(pair))
            {
                // Forget unpaid motion, never the spent impact, during a gap.
                const float keep = std::exp(-duration / mProfile.mCutMemory);
                state.mCutTravel *= keep;
                state.mLoadedTravel *= keep;
                state.mCutTime *= keep;
            }
        }
        for (const auto& [pair, events] : groups)
        {
            auto& state = mContacts[pair];
            float maximumEnergy = 0, totalEnergy = 0;
            for (const auto* event : events)
            {
                if (event->weaponRecordId().empty() && !event->swingActive()) continue;
                const float value = energy(*event);
                if (value < mProfile.mMinimumImpactEnergy) continue;
                maximumEnergy = std::max(maximumEnergy, value);
                totalEnergy += value;
            }
            if (state.mImpactSpent && mProfile.mTopUpFactor > 0 && maximumEnergy > 0
                && maximumEnergy > state.mSpentEnergy * mProfile.mTopUpFactor)
            {
                // A much harder blow in the same episode pays what the first
                // touch did not.
                const float owed = std::min(1.0f, (maximumEnergy - state.mSpentEnergy) / mProfile.mFullImpactEnergy);
                for (const auto* event : events)
                {
                    if (event->weaponRecordId().empty() && !event->swingActive()) continue;
                    const float value = energy(*event);
                    if (value >= mProfile.mMinimumImpactEnergy)
                        result.push_back(event->withDamageMeasure(
                            event->relativeNormalSpeed(), owed * value / totalEnergy, value / totalEnergy));
                }
                state.mSpentEnergy = maximumEnergy;
                state.mCutTravel = state.mLoadedTravel = JPH::Vec3::sZero();
                state.mCutTime = 0;
                continue;
            }
            if (!state.mImpactSpent && maximumEnergy > 0)
            {
                // Regional observations describe the same blow. Share one
                // bounded budget instead of paying again at every limb seam.
                const float fraction = std::min(1.0f, maximumEnergy / mProfile.mFullImpactEnergy);
                for (const auto* event : events)
                {
                    if (event->weaponRecordId().empty() && !event->swingActive()) continue;
                    const float value = energy(*event);
                    if (value >= mProfile.mMinimumImpactEnergy)
                        result.push_back(event->withDamageMeasure(
                            event->relativeNormalSpeed(), fraction * value / totalEnergy, value / totalEnergy));
                }
                state.mImpactSpent = true;
                state.mInjured = true;
                state.mSpentEnergy = maximumEnergy;
                state.mCutTravel = state.mLoadedTravel = JPH::Vec3::sZero();
                state.mCutTime = 0;
                continue;
            }
            // Select one loaded surface for this step, deterministically. The
            // same edge motion against several colliders cannot multiply work.
            const WeaponContactEvent* drawing = nullptr;
            float drawSpeed = 0, loadFraction = 0, bestWork = 0;
            for (const auto* event : events)
            {
                if (event->weaponRecordId().empty() || event->wound().mKind != WoundKind::Slash
                    || event->relativeNormalSpeed() < -0.5f || !event->normalImpulse()) continue;
                const JPH::Vec3 tangent = event->wound().mRelativeVelocity
                    - event->normal() * event->wound().mRelativeVelocity.Dot(event->normal());
                if (!finite(tangent) || !finite(event->wound().mTargetLocalTangent)) continue;
                const float speed = tangent.Length();
                const float load = std::min(1.0f, UnitModel::toMeters(*event->normalImpulse())
                    / duration / mProfile.mFullCutLoad);
                if (speed < mProfile.mMinimumCutSpeed || load <= 0) continue;
                if (speed * load > bestWork)
                {
                    drawing = event;
                    drawSpeed = speed;
                    loadFraction = load;
                    bestWork = speed * load;
                }
            }
            if (!drawing)
            {
                const float keep = std::exp(-duration / mProfile.mCutMemory);
                state.mCutTravel *= keep;
                state.mLoadedTravel *= keep;
                state.mCutTime *= keep;
                continue;
            }
            // Tangents live in the struck body's coordinates: victim rotation
            // cannot turn cancelling jitter into extra path length. A different
            // body's frame starts a new draw, but keeps the actor impact spent.
            if (state.mCutBody != drawing->targetBodyIdentity())
            {
                state.mCutTravel = state.mLoadedTravel = JPH::Vec3::sZero();
                state.mCutTime = 0;
                state.mCutBody = drawing->targetBodyIdentity();
            }
            const auto delta = drawing->wound().mTargetLocalTangent.NormalizedOr(JPH::Vec3::sZero())
                * drawSpeed * duration;
            state.mCutTravel += delta;
            state.mLoadedTravel += delta * loadFraction;
            state.mCutTime += duration;
            const float travel = state.mCutTravel.Length();
            const float netSpeed = travel / state.mCutTime;
            if (netSpeed < mProfile.mMinimumCutSpeed)
            {
                const float keep = std::exp(-duration / mProfile.mCutMemory);
                state.mCutTravel *= keep;
                state.mLoadedTravel *= keep;
                state.mCutTime *= keep;
                continue;
            }
            float fraction = std::min(travel, state.mLoadedTravel.Length()) / mProfile.mTravelPerFullCut
                * std::min(1.0f, netSpeed / mProfile.mFullCutSpeed);
            if (travel < mProfile.mMinimumCutTravel) continue;
            const float available = fraction;
            // Consume all eligible fresh work even at a coarse physics rate;
            // never build a backlog that pays damage after the edge stops.
            while (fraction >= (state.mInjured ? mProfile.mRepeatInjuryFraction : mProfile.mMinimumInjuryFraction))
            {
                const float paid = std::min(1.0f, fraction);
                result.push_back(drawing->withDamageMeasure(netSpeed, paid, paid, true));
                fraction -= paid;
                state.mImpactSpent = true;
                state.mInjured = true;
            }
            if (available > 0)
            {
                const float owed = fraction / available;
                state.mCutTravel *= owed;
                state.mLoadedTravel *= owed;
                state.mCutTime *= owed;
            }
        }
        return result;
    }

    LocalizedStrikeAggregator::LocalizedStrikeAggregator(float minimumApproachSpeed)
        : mMinimumApproachSpeed(minimumApproachSpeed)
    {
        if (!std::isfinite(minimumApproachSpeed) || minimumApproachSpeed < 0.0f)
            mMinimumApproachSpeed = sDefaultMinimumApproachSpeed;
    }

    bool LocalizedStrikeAggregator::qualifies(const WeaponContactEvent& event) const noexcept
    {
        return event.valid() && event.swingActive() && event.swingIdentity() != 0
            && event.targetActorIdentity().has_value()
            && (event.relativeNormalSpeed() >= mMinimumApproachSpeed
                || (mMinimumSliceSpeed > 0.0f && event.relativeSpeed() >= mMinimumSliceSpeed)
                // A caller-priced slice (a pressure cut) qualifies by its price.
                || (event.isSlice() && event.damageSpeed() >= mMinimumApproachSpeed));
    }

    float LocalizedStrikeAggregator::impactScore(const WeaponContactEvent& event) noexcept
    {
        const float score = event.effectiveMass() * event.damageSpeed();
        return std::isfinite(score) ? score : -1.0f;
    }

    bool LocalizedStrikeAggregator::betterOf(
        const WeaponContactEvent& left, const WeaponContactEvent& right) noexcept
    {
        const float leftScore = impactScore(left);
        const float rightScore = impactScore(right);
        if (leftScore != rightScore)
            return leftScore > rightScore;
        // Equal impact: a supported anatomical region beats an unknown or
        // weapon contact, then the lower body index, then the finer identity
        // and point, so the choice never depends on callback order.
        const bool leftSupported = left.region() != AnatomicalRegion::Unknown;
        const bool rightSupported = right.region() != AnatomicalRegion::Unknown;
        if (leftSupported != rightSupported)
            return leftSupported;
        if (left.region() != right.region())
            return static_cast<int>(left.region()) < static_cast<int>(right.region());
        if (left.targetBodyIndex() != right.targetBodyIndex())
            return left.targetBodyIndex() < right.targetBodyIndex();
        if (left.targetBodyIdentity() != right.targetBodyIdentity())
            return left.targetBodyIdentity() < right.targetBodyIdentity();
        if (left.contactIdentity() != right.contactIdentity())
            return left.contactIdentity() < right.contactIdentity();
        if (left.point().GetX() != right.point().GetX())
            return left.point().GetX() < right.point().GetX();
        if (left.point().GetY() != right.point().GetY())
            return left.point().GetY() < right.point().GetY();
        return left.point().GetZ() < right.point().GetZ();
    }

    bool LocalizedStrikeAggregator::observe(const WeaponContactEvent& raw)
    {
        if (!qualifies(raw))
            return false;
        // A slice is priced by its sliding speed (the normal one is small).
        const WeaponContactEvent event = !raw.isSlice() && raw.relativeNormalSpeed() < mMinimumApproachSpeed
            ? raw.withSliceSpeed(raw.relativeSpeed() * mSliceShare) : raw;
        const Key key{ event.swingIdentity(), event.attackerActorIdentity(), *event.targetActorIdentity() };
        if (mCommitted.contains(key))
            return false;
        const auto found = mPending.find(key);
        if (found == mPending.end())
        {
            mPending.emplace(key, Pending{ event.physicsStep(), impactScore(event), event });
            return true;
        }
        Pending& pending = found->second;
        if (event.physicsStep() < pending.mStep)
        {
            // An earlier contacting step replaces everything seen for a later
            // one; the earliest step is the strike that actually arrived.
            pending = Pending{ event.physicsStep(), impactScore(event), event };
            return true;
        }
        if ((event.physicsStep() == pending.mStep
                || (mWindow > 0 && event.physicsStep() <= pending.mStep + mWindow))
            && betterOf(event, pending.mEvent))
        {
            pending.mScore = impactScore(event);
            pending.mEvent = event;
            return true;
        }
        return false;
    }

    std::vector<WeaponContactEvent> LocalizedStrikeAggregator::consume(std::uint64_t physicsStep)
    {
        std::vector<WeaponContactEvent> strikes;
        for (auto it = mPending.begin(); it != mPending.end();)
        {
            if (it->second.mStep + mWindow > physicsStep)
            {
                ++it;
                continue;
            }
            strikes.push_back(it->second.mEvent);
            const Key committed = it->first;
            mCommitted.insert(committed);
            it = mPending.erase(it);
            // Swing identities only grow for an attacker and a finished swing
            // no longer qualifies, so its record can go; otherwise this set
            // gains an entry for every blow ever landed.
            for (auto old = mCommitted.begin(); old != mCommitted.end();)
            {
                if (old->mSwingIdentity >= committed.mSwingIdentity)
                    break;
                if (old->mAttackerActorIdentity == committed.mAttackerActorIdentity)
                    old = mCommitted.erase(old);
                else
                    ++old;
            }
        }
        return strikes;
    }

    void LocalizedStrikeAggregator::rejectStaleGeneration(
        std::uint64_t actorIdentity, std::uint64_t currentGeneration)
    {
        const auto stale = [&](const Key& key, std::uint64_t generation) {
            return key.mTargetActorIdentity == actorIdentity && generation != currentGeneration;
        };
        for (auto it = mPending.begin(); it != mPending.end();)
        {
            if (stale(it->first, it->second.mEvent.targetRegistryGeneration()))
                it = mPending.erase(it);
            else
                ++it;
        }
    }

    void LocalizedStrikeAggregator::reset() noexcept
    {
        mPending.clear();
        mCommitted.clear();
    }
}

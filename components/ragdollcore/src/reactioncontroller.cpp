#include <ragdollcore/reactioncontroller.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    namespace
    {
        float smooth(float x)
        {
            x = std::clamp(x, 0.0f, 1.0f);
            return x * x * (3.0f - 2.0f * x);
        }

        bool finite(JPH::Vec3Arg v)
        {
            return std::isfinite(v.GetX()) && std::isfinite(v.GetY()) && std::isfinite(v.GetZ());
        }

        JPH::Vec3 unitOr(JPH::Vec3Arg v, JPH::Vec3Arg fallback)
        {
            return finite(v) && v.LengthSq() > 1e-12f ? v.Normalized() : fallback;
        }

        bool isHead(AnatomicalRegion region)
        {
            return region == AnatomicalRegion::Head || region == AnatomicalRegion::Neck;
        }

        bool isTrunk(AnatomicalRegion region)
        {
            return region == AnatomicalRegion::Chest || region == AnatomicalRegion::Abdomen
                || region == AnatomicalRegion::Pelvis;
        }

        bool isLow(AnatomicalRegion region)
        {
            return region == AnatomicalRegion::Abdomen || region == AnatomicalRegion::Pelvis;
        }

        // -1 none, 0 left, 1 right.
        int armSide(AnatomicalRegion region)
        {
            switch (region)
            {
                case AnatomicalRegion::LeftClavicle:
                case AnatomicalRegion::LeftUpperArm:
                case AnatomicalRegion::LeftForearm:
                case AnatomicalRegion::LeftHand:
                    return 0;
                case AnatomicalRegion::RightClavicle:
                case AnatomicalRegion::RightUpperArm:
                case AnatomicalRegion::RightForearm:
                case AnatomicalRegion::RightHand:
                    return 1;
                default:
                    return -1;
            }
        }

        int legSide(AnatomicalRegion region)
        {
            switch (region)
            {
                case AnatomicalRegion::LeftThigh:
                case AnatomicalRegion::LeftCalf:
                case AnatomicalRegion::LeftFoot:
                    return 0;
                case AnatomicalRegion::RightThigh:
                case AnatomicalRegion::RightCalf:
                case AnatomicalRegion::RightFoot:
                    return 1;
                default:
                    return -1;
            }
        }

        std::size_t channelIndex(BodyChannel channel)
        {
            return static_cast<std::size_t>(channel);
        }

        BodyChannel channelFor(AnatomicalRegion region)
        {
            if (isHead(region))
                return BodyChannel::Head;
            if (const int side = armSide(region); side >= 0)
                return side == 0 ? BodyChannel::LeftArm : BodyChannel::RightArm;
            if (const int side = legSide(region); side >= 0)
                return side == 0 ? BodyChannel::LeftLeg : BodyChannel::RightLeg;
            return BodyChannel::Trunk;
        }

        JPH::Vec3 clampLength(JPH::Vec3Arg v, float cap)
        {
            const float length = v.Length();
            return length > cap && length > 0.0f ? v * (cap / length) : JPH::Vec3(v);
        }
    }

    const char* reactionTierName(ReactionTier tier)
    {
        switch (tier)
        {
            case ReactionTier::Absorb: return "absorb";
            case ReactionTier::Flinch: return "flinch";
            case ReactionTier::Stagger: return "stagger";
            case ReactionTier::Down: return "down";
        }
        return "absorb";
    }

    bool ReflexPose::any() const
    {
        const auto nonZero = [](JPH::Vec3Arg v) { return v.LengthSq() > 1e-8f; };
        return nonZero(mHead) || nonZero(mTrunk) || nonZero(mArm[0]) || nonZero(mArm[1]) || mArmsUp > 1e-4f
            || std::abs(mKneeBend[0]) > 1e-4f || std::abs(mKneeBend[1]) > 1e-4f;
    }

    BlowDisturbance blowDisturbance(JPH::Vec3Arg impulse, JPH::Vec3Arg leverFromCom, float totalMass,
        float partMass, JPH::Vec3Arg bodyForward, JPH::Vec3Arg bodyUp)
    {
        BlowDisturbance result;
        result.mBodyUp = unitOr(bodyUp, JPH::Vec3::sAxisZ());
        JPH::Vec3 forward = bodyForward - result.mBodyUp * bodyForward.Dot(result.mBodyUp);
        result.mBodyForward = unitOr(forward, JPH::Vec3::sAxisY());
        if (!finite(impulse) || !finite(leverFromCom))
            return result;
        if (std::isfinite(totalMass) && totalMass > 0.0f)
            result.mComVelocityChange = impulse / totalMass;
        result.mAngularMomentumChange = leverFromCom.Cross(impulse);
        if (std::isfinite(partMass) && partMass > 0.0f)
            result.mPartVelocityChange = impulse.Length() / partMass;
        return result;
    }

    void ReactionController::reset()
    {
        mTracks.clear();
        mLeakage = 1.0f;
        mSinceReflex = 1e9f;
        mReflexTier = ReactionTier::Absorb;
        mLastStaggerAttacker = 0;
        mSinceStagger = 1e9f;
        mLastStaggerSeverity = 0.0f;
        mFiltered = {};
        mPendingInterrupt = false;
        mBearingsLeft = 0.0f;
    }

    ReactionEvent ReactionController::onBlow(
        const Blow& blow, const BlowDisturbance& disturbance, const ReactionVictim& victim)
    {
        ReactionEvent event;
        event.mBlowId = blow.mId;
        const ReactionSettings& s = mSettings;
        const float toMetres = 1.0f / sReactionUnitsPerMetre;
        const JPH::Vec3 up = unitOr(disturbance.mBodyUp, JPH::Vec3::sAxisZ());
        const JPH::Vec3 forward = unitOr(disturbance.mBodyForward, JPH::Vec3::sAxisY());
        const JPH::Vec3 right = forward.Cross(up);
        const JPH::Vec3 dv = finite(disturbance.mComVelocityChange) ? disturbance.mComVelocityChange : JPH::Vec3::sZero();
        const float dvCom = dv.Length() * toMetres;
        float dvPart = std::isfinite(disturbance.mPartVelocityChange)
            ? std::max(0.0f, disturbance.mPartVelocityChange) * toMetres : 0.0f;
        // Bearings on: blunt blows concuss, and a tired or hurt body's
        // thresholds are lower.
        const bool bearings = s.mBearingsBase > 0.0f;
        const bool blunt = bearings && blow.mKind == BlowKind::Blunt;
        if (blunt)
            dvPart *= s.mBluntConcussion;
        const auto share = [](float v) { return std::isfinite(v) ? std::clamp(v, 0.0f, 1.0f) : 1.0f; };
        const float poise = bearings
            ? s.mPoiseFloor + (1.0f - s.mPoiseFloor) * std::min(share(victim.mFatigue), share(victim.mHealth))
            : 1.0f;

        // Balance severity: the COM dv against the step threshold for its
        // direction (an ellipse: forward or backward, and sideways).
        const float along = dv.Dot(forward) * toMetres;
        const float across = dv.Dot(right) * toMetres;
        const float alongStep = along >= 0.0f ? s.mStepDvForward : s.mStepDvBack;
        event.mBalanceSeverity = std::sqrt((along / std::max(1e-3f, alongStep)) * (along / std::max(1e-3f, alongStep))
            + (across / std::max(1e-3f, s.mStepDvSide)) * (across / std::max(1e-3f, s.mStepDvSide)));
        event.mLocalSeverity = std::clamp(dvPart / std::max(1e-3f, s.mLocalStaggerDv), 0.0f, 1.0f);

        ReactionTier tier = ReactionTier::Absorb;
        if (dvCom >= s.mFlinchDv * poise || dvPart >= s.mLocalFlinchDv * poise)
            tier = ReactionTier::Flinch;
        if (dvCom >= s.mStaggerDv * poise || (isHead(blow.mRegion) && dvPart >= s.mLocalStaggerDv * poise))
            tier = ReactionTier::Stagger;
        // A blocked or parried blow struck the guard: at most a flinch,
        // unless it beat the guard.
        if ((blow.mBlocked || blow.mParried) && !blow.mGuardBeaten && tier > ReactionTier::Flinch)
            tier = ReactionTier::Flinch;
        // A beaten guard (bearings on): the blow drove the guard into the
        // body - a stagger, whatever the push measured.
        if (bearings && blow.mGuardBeaten)
            tier = ReactionTier::Stagger;
        event.mTier = tier;

        // Poise: this blow's reflex and interrupt at the current leakage;
        // the blow then drains it by its severity (light touches do not).
        event.mScale = mLeakage;
        if (tier >= ReactionTier::Flinch)
        {
            const float severity = std::clamp(std::max(event.mLocalSeverity, event.mBalanceSeverity), 0.0f, 1.0f);
            mLeakage = std::max(s.mLeakageFloor, mLeakage * (1.0f - s.mLeakageReduction * severity));
        }

        // Debounce: a new reflex only after the gap, or a higher tier.
        if (tier >= ReactionTier::Flinch && !victim.mDown
            && (mSinceReflex >= s.mReflexGap || tier > mReflexTier))
        {
            event.mReflex = true;
            mSinceReflex = 0.0f;
            mReflexTier = tier;
        }

        // Interrupt: only the victim's own attack, only by a blow that beats
        // its poise. Committed swings (the cut, or past the wind-up share)
        // break only at Stagger; a stagger from the same attacker inside the
        // window needs a stronger blow.
        if (victim.mAttacking && tier >= ReactionTier::Flinch && event.mScale >= 0.4f)
        {
            const bool committed = victim.mCutting || victim.mWindShare >= s.mCommittedWindShare;
            const ReactionTier needed = committed ? ReactionTier::Stagger : ReactionTier::Flinch;
            bool allowed = tier >= needed;
            const float severity = std::max(event.mLocalSeverity, event.mBalanceSeverity);
            if (allowed && tier == ReactionTier::Stagger && blow.mAttacker != 0 && blow.mAttacker == mLastStaggerAttacker
                && mSinceStagger < s.mStaggerWindow && severity <= mLastStaggerSeverity)
                allowed = false;
            event.mInterrupt = allowed;
            mPendingInterrupt = mPendingInterrupt || allowed;
            if (allowed && tier == ReactionTier::Stagger)
            {
                mLastStaggerAttacker = blow.mAttacker;
                mSinceStagger = 0.0f;
                mLastStaggerSeverity = severity;
            }
        }

        // Bearings: a stagger opens the window to get them back.
        if (bearings && tier >= ReactionTier::Stagger)
        {
            const float severity = std::clamp(std::max(event.mLocalSeverity, event.mBalanceSeverity * 0.5f), 0.0f, 1.0f);
            float seconds = s.mBearingsBase + s.mBearingsPerSeverity * severity;
            seconds *= 1.0f + s.mBearingsTired * (1.0f - share(victim.mFatigue));
            seconds *= 1.0f + s.mBearingsHurt * (1.0f - share(victim.mHealth));
            if (blunt)
                seconds *= s.mBluntConcussion;
            mBearingsLeft = std::max(mBearingsLeft, std::min(seconds, s.mBearingsMax));
        }
        // Injury (leg give-out): damage, not momentum, and not leakage - a
        // third deep cut takes the leg as surely as the first.
        if (legSide(blow.mRegion) >= 0 && s.mLegInjuryDamage > 0.0f && std::isfinite(blow.mDamage)
            && !blow.mBlocked && !blow.mParried)
            event.mInjury = std::clamp(blow.mDamage / s.mLegInjuryDamage, 0.0f, 1.0f);

        Track track;
        track.mBlowId = blow.mId;
        track.mInjury = event.mInjury;
        track.mAttacker = blow.mAttacker;
        track.mRegion = blow.mRegion;
        track.mTier = tier;
        track.mLocal = std::max(event.mLocalSeverity, std::min(1.0f, event.mBalanceSeverity * 0.5f));
        track.mScale = event.mScale;
        track.mReflex = event.mReflex;
        track.mDirection = unitOr(blow.mDirection, -forward);
        track.mBodyDirection = JPH::Vec3(track.mDirection.Dot(right), track.mDirection.Dot(forward), track.mDirection.Dot(up));
        const JPH::Vec3 spin = finite(disturbance.mAngularMomentumChange) ? disturbance.mAngularMomentumChange
                                                                           : JPH::Vec3::sZero();
        track.mWorldSpin = unitOr(spin, JPH::Vec3::sZero());
        track.mSpin = JPH::Vec3(track.mWorldSpin.Dot(right), track.mWorldSpin.Dot(forward), track.mWorldSpin.Dot(up));
        // Tracks are few (overlapping blows); the oldest goes first.
        if (mTracks.size() >= 8)
            mTracks.erase(mTracks.begin());
        mTracks.push_back(track);
        return event;
    }

    float ReactionController::envelope(float age, float onset, float hold) const
    {
        const float t = age - onset;
        if (t <= 0.0f)
            return 0.0f;
        const float rise = std::max(1e-3f, mSettings.mReflexRise);
        if (t < rise)
            return smooth(t / rise);
        if (t < rise + hold)
            return 1.0f;
        return 1.0f - smooth((t - rise - hold) / std::max(1e-3f, mSettings.mReflexRelease));
    }

    float ReactionController::legWeaknessTime(const Track& track) const
    {
        const ReactionSettings& s = mSettings;
        return s.mLegWeaknessTime + (std::max(s.mLegGiveOutTime, s.mLegWeaknessTime) - s.mLegWeaknessTime) * track.mInjury;
    }

    float ReactionController::lifetime(const Track& track) const
    {
        const ReactionSettings& s = mSettings;
        const float give = s.mGiveHold + s.mGiveRamp;
        const float reflex = s.mArmOnset + s.mReflexRise + std::max({ s.mReflexHold, s.mArmFlingHold, s.mArmsUpHold })
            + s.mReflexRelease;
        const float leg = s.mLegWeaknessDelay + legWeaknessTime(track) + 0.1f;
        return std::max({ give, track.mReflex ? reflex : 0.0f, legSide(track.mRegion) >= 0 ? leg : 0.0f,
            s.mUnsnapInterval + 0.05f });
    }

    ReactionStepOutput ReactionController::step(float dt, const ReactionVictim& victim)
    {
        ReactionStepOutput out;
        out.mTorqueScale.fill(1.0f);
        if (!std::isfinite(dt) || dt < 0.0f)
            dt = 0.0f;
        const ReactionSettings& s = mSettings;
        mLeakage = std::min(1.0f, mLeakage + s.mLeakageRecovery * dt);
        mSinceReflex += dt;
        // Bearings: busy, legs and neck loose while they last.
        if (mBearingsLeft > 0.0f && !victim.mDown)
        {
            out.mWeaponArmBusy = true;
            for (const BodyChannel leg : { BodyChannel::LeftLeg, BodyChannel::RightLeg })
            {
                float& scale = out.mTorqueScale[channelIndex(leg)];
                scale = std::min(scale, s.mBearingsLegs);
            }
            float& neck = out.mTorqueScale[channelIndex(BodyChannel::Head)];
            neck = std::min(neck, s.mBearingsNeck);
        }
        out.mBearings = mBearingsLeft;
        mBearingsLeft = std::max(0.0f, mBearingsLeft - dt);
        mSinceStagger += dt;

        ReflexPose target;
        std::array<float, 2> giveOutKnee{ 0.0f, 0.0f }; // the knee of a leg giving out (a floor, not a reflex)
        const JPH::Vec3 upBody = JPH::Vec3::sAxisZ();
        for (Track& track : mTracks)
        {
            // Snaps: once at the contact, the reverse at the interval.
            const JPH::Vec3 topAlong = JPH::Vec3::sAxisZ().Cross(track.mDirection); // world: tips the top along the blow
            const float snapScale = track.mLocal * track.mScale;
            const bool headBlow = isHead(track.mRegion);
            const bool legBlow = legSide(track.mRegion) >= 0;
            if (!track.mSnapped)
            {
                track.mSnapped = true;
                if (!legBlow && !victim.mDown)
                {
                    out.mTrunkSnap += topAlong * (s.mSnapTrunk * snapScale * (headBlow ? 0.4f : 1.0f));
                    out.mHeadSnap += topAlong * (s.mSnapHead * snapScale * (headBlow ? 1.0f : 0.25f));
                }
            }
            else if (!track.mUnsnapped && track.mAge >= s.mUnsnapInterval)
            {
                track.mUnsnapped = true;
                if (!legBlow && !victim.mDown && track.mLocal >= 0.6f)
                {
                    out.mTrunkSnap -= topAlong * (s.mSnapTrunk * snapScale * (headBlow ? 0.4f : 1.0f) * s.mUnsnapRatio);
                    out.mHeadSnap -= topAlong * (s.mSnapHead * snapScale * (headBlow ? 1.0f : 0.25f) * s.mUnsnapRatio);
                }
            }

            // Impact give: the struck channel softens, holds, recovers. A
            // body already staggering or down is not loosened further.
            if (!victim.mDown && !victim.mStaggering)
            {
                const float floor = s.mGiveFloorLight + (s.mGiveFloorHeavy - s.mGiveFloorLight) * track.mLocal;
                const float hold = s.mGiveHold * track.mLocal;
                const float give = track.mAge < hold
                    ? floor : floor + (1.0f - floor) * smooth((track.mAge - hold) / std::max(1e-3f, s.mGiveRamp));
                const BodyChannel channel = channelFor(track.mRegion);
                float& scale = out.mTorqueScale[channelIndex(channel)];
                scale = std::min(scale, give);
                if (headBlow)
                {
                    const float neck = s.mNeckGiveFloor + (1.0f - s.mNeckGiveFloor)
                        * smooth((track.mAge - hold) / std::max(1e-3f, s.mGiveRamp));
                    float& head = out.mTorqueScale[channelIndex(BodyChannel::Head)];
                    head = std::min(head, track.mAge < hold ? s.mNeckGiveFloor : neck);
                }
            }
            // A struck leg loses strength for a while after a delay; an
            // injured one gives out (weaker, longer, the knee going), eased
            // back over the window's last third.
            if (const int side = legSide(track.mRegion);
                side >= 0 && (track.mTier >= ReactionTier::Flinch || track.mInjury > 0.0f))
            {
                const float t = track.mAge - s.mLegWeaknessDelay;
                const float window = legWeaknessTime(track);
                if (t >= 0.0f && t < window)
                {
                    const float weakness = s.mLegWeakness + (s.mLegGiveOutWeakness - s.mLegWeakness) * track.mInjury;
                    const float release = smooth((t - 0.67f * window) / std::max(1e-3f, 0.33f * window));
                    // (Uninjured: the flat weakness of before.)
                    const float e = track.mInjury > 0.0f ? smooth(t / 0.08f) * (1.0f - release) : 1.0f;
                    float& scale = out.mTorqueScale[channelIndex(side == 0 ? BodyChannel::LeftLeg : BodyChannel::RightLeg)];
                    scale = std::min(scale, 1.0f + (weakness - 1.0f) * e);
                    if (track.mInjury > 0.0f && !victim.mDown)
                    {
                        auto& giveOut = out.mLegGiveOut[static_cast<std::size_t>(side)];
                        giveOut = std::max(giveOut, track.mInjury * e);
                        auto& knee = giveOutKnee[static_cast<std::size_t>(side)];
                        knee = std::max(knee, s.mLegGiveOutKnee * track.mInjury * e);
                    }
                }
            }

            // Reflexes by region, in the body frame.
            if (track.mReflex && !victim.mDown)
            {
                const float k = track.mScale * (track.mTier >= ReactionTier::Stagger ? 1.0f : 0.5f + 0.5f * track.mLocal);
                const float e = envelope(track.mAge, s.mReflexOnset, s.mReflexHold);
                const float eArm = envelope(track.mAge, s.mArmOnset, s.mArmsUpHold);
                const JPH::Vec3 away = upBody.Cross(track.mBodyDirection); // tips the top along the blow
                if (headBlow)
                {
                    target.mHead += away * (s.mHeadAway * k * e);
                    if (track.mLocal >= 0.4f && track.mTier >= ReactionTier::Flinch)
                        target.mArmsUp = std::max(target.mArmsUp, k * eArm);
                }
                else if (isTrunk(track.mRegion) || track.mRegion == AnatomicalRegion::Unknown)
                {
                    // Low: fold over the blow (the top comes toward the
                    // attacker); high: the chest is driven back.
                    target.mTrunk += away * ((isLow(track.mRegion) ? -1.0f : 1.0f) * s.mTrunkFold * k * e);
                    target.mTrunk += JPH::Vec3(0.0f, 0.0f, track.mSpin.GetZ()) * (s.mTrunkTurn * k * e);
                    target.mHead += away * (0.3f * s.mHeadAway * k * e);
                }
                else if (const int side = armSide(track.mRegion); side >= 0)
                {
                    const float fling = envelope(track.mAge, s.mArmOnset - 0.02f, s.mArmFlingHold);
                    target.mArm[static_cast<std::size_t>(side)] += away * (s.mArmFling * k * fling);
                    target.mTrunk += JPH::Vec3(0.0f, 0.0f, track.mSpin.GetZ()) * (0.5f * s.mTrunkTurn * k * e);
                    if (side == 1 && fling > 0.1f)
                        out.mWeaponArmBusy = true;
                }
                else if (const int side = legSide(track.mRegion); side >= 0)
                    target.mKneeBend[static_cast<std::size_t>(side)] += s.mKneeBend * k * e;
                if (track.mTier >= ReactionTier::Stagger && e > 0.1f)
                    out.mWeaponArmBusy = true;
            }
            const float age = track.mAge;
            if (track.mReflex && age < lifetime(track))
                out.mTier = std::max(out.mTier, track.mTier);
            track.mAge += dt;
        }
        std::erase_if(mTracks, [&](const Track& track) { return track.mAge > lifetime(track); });

        // Caps, then the output filter (overlapping blows must not twitch).
        target.mHead = clampLength(target.mHead, 1.2f * s.mHeadAway);
        target.mTrunk = clampLength(target.mTrunk, 1.2f * (s.mTrunkFold + s.mTrunkTurn));
        for (auto& arm : target.mArm)
            arm = clampLength(arm, 1.2f * s.mArmFling);
        target.mArmsUp = std::clamp(target.mArmsUp, 0.0f, 1.0f);
        for (std::size_t i = 0; i < 2; ++i)
            target.mKneeBend[i] = std::max(std::clamp(target.mKneeBend[i], 0.0f, 1.2f * s.mKneeBend), giveOutKnee[i]);
        const float a = std::clamp(s.mFilter, 0.0f, 1.0f);
        mFiltered.mHead += (target.mHead - mFiltered.mHead) * a;
        mFiltered.mTrunk += (target.mTrunk - mFiltered.mTrunk) * a;
        for (std::size_t i = 0; i < 2; ++i)
        {
            mFiltered.mArm[i] += (target.mArm[i] - mFiltered.mArm[i]) * a;
            mFiltered.mKneeBend[i] += (target.mKneeBend[i] - mFiltered.mKneeBend[i]) * a;
        }
        mFiltered.mArmsUp += (target.mArmsUp - mFiltered.mArmsUp) * a;
        out.mReflex = mFiltered;
        out.mHeadSnap = clampLength(out.mHeadSnap, s.mHeadSpinCap);
        out.mLeakage = mLeakage;
        out.mInterrupt = mPendingInterrupt;
        mPendingInterrupt = false;
        return out;
    }
}

#include <ragdollcore/palmparry.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace RagdollCore
{
    namespace
    {
        constexpr float sRange = 130.0f; // a blow is read inside this distance of the chest
        constexpr float sClosing = 250.0f; // ... with its stroke running, or closing this fast (u/s)
        constexpr float sBodyRadius = 14.0f;
        constexpr float sLead = 0.1f; // s the striking part is read ahead for the blow's side
        constexpr float sArm = 0.05f; // s after leaving before the palm deflects
        constexpr float sSlow = 200.0f; // u/s: under it a blade on the raised hands is stopped by them
        constexpr float sAfter = 0.12f; // s the palm stays once the blow is no longer coming
        constexpr float sRecover = 0.25f;
        constexpr float sForget = 0.2f; // s with nothing coming in: the next is a new blow
        constexpr float sAlong = 0.5f; // the palm's place on the weapon, hand (0) to striking part (1)
        constexpr float sTravel = 0.08f; // s the weapon is read ahead (the palm's own way out)
        constexpr float sReachShare = 0.95f; // of the arm

        float clamp01(float value)
        {
            return std::clamp(value, 0.0f, 1.0f);
        }

        // Where a hand waits: in front of its shoulder, drawn in toward the
        // face, moved by the zone.
        JPH::Vec3 readyPlace(int side, const PalmParryInput& input)
        {
            const JPH::Vec3& shoulder = input.mShoulder[static_cast<std::size_t>(side)];
            const float arm = std::max(10.0f, input.mArmLength[static_cast<std::size_t>(side)]);
            JPH::Vec3 place(shoulder.GetX() * 0.55f, shoulder.GetY() + 0.62f * arm, shoulder.GetZ() + 7.0f);
            switch (input.mZone)
            {
                case WheelGuardZone::High:
                    place += JPH::Vec3(0.0f, 0.0f, 9.0f);
                    break;
                case WheelGuardZone::Low:
                    place += JPH::Vec3(0.0f, 2.0f, -16.0f);
                    break;
                case WheelGuardZone::Left:
                    place += JPH::Vec3(-7.0f, 0.0f, 0.0f);
                    break;
                case WheelGuardZone::Right:
                    place += JPH::Vec3(7.0f, 0.0f, 0.0f);
                    break;
                case WheelGuardZone::Middle:
                case WheelGuardZone::Shove:
                    break;
            }
            return place;
        }

        JPH::Vec3 withinReach(const JPH::Vec3& place, const JPH::Vec3& shoulder, float arm)
        {
            JPH::Vec3 at = place;
            at.SetY(std::max(at.GetY(), 8.0f));
            const JPH::Vec3 from = at - shoulder;
            const float reach = sReachShare * std::max(10.0f, arm);
            if (const float length = from.Length(); length > reach)
                at = shoulder + from * (reach / length);
            return at;
        }
    }

    PalmParrySettings parsePalmParrySettings(const char* text)
    {
        PalmParrySettings settings;
        if (!text || !*text)
            return settings;
        float on = 0.0f;
        std::sscanf(text, "%f,%f,%f,%f,%f,%f,%f", &on, &settings.mReact, &settings.mHold, &settings.mCover,
            &settings.mKnock, &settings.mOpen, &settings.mWrong);
        settings.mOn = std::isfinite(on) && on > 0.0f;
        const auto sane = [](float& value, float low, float high, float fallback) {
            value = std::isfinite(value) ? std::clamp(value, low, high) : fallback;
        };
        sane(settings.mReact, 0.0f, 1.0f, 0.22f);
        sane(settings.mHold, 0.05f, 3.0f, 1.0f);
        sane(settings.mCover, 0.0f, 40.0f, 12.0f);
        sane(settings.mKnock, 0.0f, 80.0f, 30.0f);
        sane(settings.mOpen, 0.0f, 3.0f, 0.9f);
        sane(settings.mWrong, 0.5f, 3.0f, 1.5f);
        return settings;
    }

    PalmTapSettings parsePalmTapSettings(const char* text)
    {
        PalmTapSettings settings;
        if (!text || !*text)
            return settings;
        std::sscanf(text, "%f,%f,%f", &settings.mWindow, &settings.mShove, &settings.mCarry);
        settings.mWindow = std::isfinite(settings.mWindow) ? std::clamp(settings.mWindow, 0.0f, 2.0f) : 0.0f;
        settings.mShove = std::isfinite(settings.mShove) ? std::clamp(settings.mShove, 0.0f, 1.0f) : 0.3f;
        settings.mCarry = std::isfinite(settings.mCarry) ? std::clamp(settings.mCarry, 0.0f, 2.0f) : 0.0f;
        return settings;
    }

    float palmTapLeft(const PalmTapSettings& settings, float held)
    {
        if (!(settings.mWindow > 0.0f) || !std::isfinite(held) || held >= sPalmTapHeld)
            return 0.0f;
        return std::max(0.0f, settings.mWindow - std::max(0.0f, held));
    }

    bool palmTapShoves(const PalmTapSettings& settings, float sincePress)
    {
        return settings.mWindow > 0.0f && settings.mShove > 0.0f && std::isfinite(sincePress) && sincePress >= 0.0f
            && sincePress < settings.mShove;
    }

    bool palmTapCarries(const PalmTapSettings& settings, float sincePress)
    {
        return settings.mWindow > 0.0f && settings.mCarry > 0.0f && std::isfinite(sincePress) && sincePress >= 0.0f
            && sincePress < settings.mWindow + settings.mCarry;
    }

    float palmParrySkill(const PalmParryStats& stats)
    {
        const float term = stats.mHandToHand + 0.2f * stats.mAgility + 0.1f * stats.mLuck;
        const float fatigue = 0.75f + 0.5f * clamp01(stats.mFatigue);
        const float skill = term * fatigue / 100.0f;
        return std::isfinite(skill) ? clamp01(skill) : 0.0f;
    }

    WheelGuardZone palmZone(const JPH::Vec3& point)
    {
        if (point.GetZ() > 16.0f)
            return WheelGuardZone::High;
        if (point.GetZ() < -12.0f)
            return WheelGuardZone::Low;
        if (point.GetX() < -6.0f)
            return WheelGuardZone::Left;
        if (point.GetX() > 6.0f)
            return WheelGuardZone::Right;
        return WheelGuardZone::Middle;
    }

    void PalmParry::met()
    {
        if (mPhase == PalmPhase::Reach)
            mHoldFor = std::min(mHoldFor, mTime + 0.1f);
    }

    PalmParryOutput PalmParry::step(const PalmParryInput& input, const PalmParrySettings& settings)
    {
        PalmParryOutput output;
        const float dt = std::isfinite(input.mDt) ? std::clamp(input.mDt, 0.0f, 0.1f) : 0.0f;
        if (!settings.mOn || !input.mGuard)
        {
            mPhase = PalmPhase::Down;
            mTime = mSince = mSeen = mQuiet = 0.0f;
            mAnswered = mLive = mCommitted = mComing = false;
            mPalm = -1;
            mStage = -1;
            return output;
        }
        const float skill = clamp01(input.mSkill);
        const std::array<JPH::Vec3, 2> ready{ readyPlace(0, input), readyPlace(1, input) };
        if (mPhase == PalmPhase::Down)
        {
            mPhase = PalmPhase::Ready;
            mTime = 0.0f;
            mShown = ready;
        }
        mTime += dt;
        mSince += dt;

        // The blow: the foe's striking part closing on the chest.
        bool coming = false;
        float arrive = 1e9f;
        JPH::Vec3 arriving = JPH::Vec3::sZero();
        if (input.mThreat && !input.mThreat->mStrike.IsNaN() && !input.mThreat->mVelocity.IsNaN())
        {
            const PalmThreat& threat = *input.mThreat;
            const float distance = threat.mStrike.Length();
            const float closing = distance > 1e-3f ? -threat.mVelocity.Dot(threat.mStrike) / distance : 0.0f;
            // The stroke from its wind-up on (a cut crosses its last 60 u in
            // under 0.1 s: a palm that waited for the blade to close left as it
            // landed, FIST-08 11-16), or a blade flying in with no stroke read.
            const bool stroke = threat.mStage == 1 || threat.mStage == 2 || threat.mAttacking;
            coming = distance < sRange && (stroke || closing > sClosing);
            if (coming)
            {
                if (closing > 1.0f)
                    arrive = std::max(0.0f, distance - sBodyRadius) / closing;
                arriving = threat.mStrike + threat.mVelocity * std::min(arrive, sLead);
            }
        }
        // Each stroke is its own blow (a combo's next wind-up follows its
        // cut with no pause: FIST-08 11-16, the second and third cuts of a
        // combo met no palm).
        {
            const int stage = input.mThreat ? input.mThreat->mStage : -1;
            const bool began = coming && (!mComing || (stage == 1 && mStage != 1));
            mComing = coming;
            if (began)
            {
                mSeen = 0.0f;
                mAnswered = false;
                // A palm still out stays out for it.
                if (mPhase == PalmPhase::Reach)
                {
                    mAnswered = true;
                    mTime = std::min(mTime, sArm);
                }
            }
            mStage = stage;
        }
        if (coming)
        {
            mSeen += dt;
            mQuiet = 0.0f;
        }
        else
        {
            mQuiet += dt;
            if (mQuiet > sForget)
                mSeen = 0.0f;
        }
        output.mBlow = coming;

        if (mPhase == PalmPhase::Ready && coming && !mAnswered)
        {
            const WheelGuardZone zone = palmZone(arriving);
            const bool chosen = input.mZone != WheelGuardZone::Middle && input.mZone != WheelGuardZone::Shove;
            const bool committed = chosen && input.mZone == zone;
            const float react
                = settings.mReact * (1.0f - 0.65f * skill) * (committed ? 0.5f : chosen ? settings.mWrong : 1.0f);
            if (mSeen >= react)
            {
                mPhase = PalmPhase::Reach;
                mTime = mSince = 0.0f;
                mAnswered = true;
                mCommitted = committed;
                // The hand on the blow's side, from the middle of the two
                // hands' places and 4 u toward the hand that leads (the
                // stance is bladed: a blow down the middle meets the front
                // hand); a side the player chose is that hand's.
                const float lead = std::clamp(ready[0].GetY() - ready[1].GetY(), -1.0f, 1.0f) * 4.0f;
                const float middle = 0.5f * (ready[0].GetX() + ready[1].GetX()) + lead;
                mPalm = input.mZone == WheelGuardZone::Left ? 0
                    : input.mZone == WheelGuardZone::Right ? 1
                    : arriving.GetX() < middle ? 0 : 1;
                mHoldFor = settings.mHold * (0.6f + 0.4f * skill) * (committed ? 1.25f : 1.0f);
                mTarget = ready[static_cast<std::size_t>(mPalm)];
                output.mLaunched = true;
            }
        }

        if (mPhase == PalmPhase::Reach)
        {
            const std::size_t palm = static_cast<std::size_t>(mPalm);
            if (input.mThreat && !input.mThreat->mStrike.IsNaN() && !input.mThreat->mVelocity.IsNaN()
                && !input.mThreat->mHand.IsNaN())
            {
                const PalmThreat& threat = *input.mThreat;
                // Between the foe's hand and the striking part, read ahead by
                // the palm's own way out (that point moves at its share of the
                // striking part's speed about the foe's hand).
                const JPH::Vec3 on = threat.mHand + (threat.mStrike - threat.mHand) * sAlong;
                mTarget = withinReach(on + threat.mVelocity * (sAlong * sTravel), input.mShoulder[palm],
                    input.mArmLength[palm]);
            }
            // Out while the blow is coming, a moment after, and no longer than its hold.
            const bool over = mTime > mHoldFor || mQuiet > sAfter;
            mLive = mTime >= sArm && !over;
            if (over)
            {
                mPhase = PalmPhase::Recover;
                mTime = 0.0f;
                mLive = false;
            }
        }
        else
            mLive = false;
        if (mPhase == PalmPhase::Recover && mTime >= sRecover)
        {
            mPhase = PalmPhase::Ready;
            mTime = 0.0f;
            mPalm = -1;
            mCommitted = false;
        }

        for (std::size_t side = 0; side < 2; ++side)
        {
            if (mPhase == PalmPhase::Reach && static_cast<int>(side) == mPalm)
                mShown[side] = mTarget; // at once: the arm's drive is the palm's speed
            else
                mShown[side] += (ready[side] - mShown[side]) * std::min(1.0f, dt * 12.0f);
            output.mHand[side] = mShown[side];
            output.mPole[side] = JPH::Vec3(side == 0 ? -0.5f : 0.5f, -0.1f, -0.85f).Normalized();
        }
        output.mPhase = mPhase;
        output.mPalm = mPalm;
        output.mLive = mLive;
        output.mCommitted = mCommitted;
        return output;
    }

    PalmContactResult resolvePalmContact(const PalmContactInput& input, const PalmParrySettings& settings)
    {
        PalmContactResult result;
        if (!settings.mOn || input.mPhase == PalmPhase::Down)
            return result;
        const float skill = clamp01(input.mSkill);
        const bool met = input.mLive && (input.mOnPalmArm || input.mDistance <= settings.mCover);
        if (!met)
        {
            // FIST-08 11-16: the guard's blade lay on the raised forearm
            // when its strike began - 88-165 u/s, 2 ms after the blow was
            // read - and nothing could have answered it.
            result.mOutcome = input.mCovered && input.mSpeed < sSlow ? PalmContactOutcome::Covered
                : input.mLive ? PalmContactOutcome::Miss
                              : PalmContactOutcome::Late;
            return result;
        }
        result.mOutcome = PalmContactOutcome::Deflect;
        const float commit = input.mCommitted ? 1.3f : 1.0f;
        // What the palm turns: half at no skill, all of it at full; a heavy
        // or two-handed weapon gives up less.
        const float heavy = clamp01((input.mWeaponWeight - 15.0f) / 30.0f);
        const float turned = clamp01(0.5f + 0.5f * skill + (input.mCommitted ? 0.15f : 0.0f) - 0.25f * heavy
            - (input.mTwoHanded ? 0.1f : 0.0f));
        result.mDamageScale = 1.0f - std::max(0.25f, turned);
        result.mKnock = settings.mKnock * (0.5f + 0.5f * skill) * commit;
        result.mOpen = settings.mOpen * (0.4f + 0.6f * skill) * commit;
        result.mImpulseScale = 0.5f + 0.5f * skill;
        return result;
    }

    std::string_view palmContactOutcomeName(PalmContactOutcome outcome)
    {
        switch (outcome)
        {
            case PalmContactOutcome::None: return "none";
            case PalmContactOutcome::Deflect: return "deflect";
            case PalmContactOutcome::Late: return "late";
            case PalmContactOutcome::Miss: return "miss";
            case PalmContactOutcome::Covered: return "covered";
        }
        return "none";
    }

    std::string_view palmPhaseName(PalmPhase phase)
    {
        switch (phase)
        {
            case PalmPhase::Down: return "down";
            case PalmPhase::Ready: return "ready";
            case PalmPhase::Reach: return "reach";
            case PalmPhase::Recover: return "recover";
        }
        return "down";
    }
}

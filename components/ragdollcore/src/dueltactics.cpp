#include <ragdollcore/dueltactics.hpp>

#include <algorithm>

namespace RagdollCore
{
    namespace
    {
        float random01(std::uint32_t& seed) noexcept
        {
            seed = seed * 1664525u + 1013904223u;
            return static_cast<float>(seed >> 8) * (1.0f / 16777216.0f);
        }

        float mix(float a, float b, float t) noexcept
        {
            return a + (b - a) * std::clamp(t, 0.0f, 1.0f);
        }

        void enter(DuelTacticsState& state, DuelPhase phase) noexcept
        {
            state.mPhase = phase;
            state.mTimeInPhase = 0.0f;
            state.mStruck = false;
            state.mOpeningSeen = 0.0f;
            if (phase == DuelPhase::Measure)
                state.mPatience = -1.0f;
        }

        // Toward a distance: forward when too far, back when too close,
        // easing in over 25 units, at most `speed`.
        float hover(float distance, float wanted, float speed) noexcept
        {
            return std::clamp((distance - wanted) / 25.0f, -1.0f, 1.0f) * speed;
        }

        // Waiting at measure: close in when too far, but give way only when
        // pressed well inside the distance, and slowly. Backing off as fast as
        // an opponent stepped in kept every step-in from ever reaching strike
        // range (duel test 2026-09-29: step-ins gave up at 60 units).
        float measureMove(float distance, float hold) noexcept
        {
            if (distance > hold + 6.0f)
                return std::min(0.7f, (distance - hold) / 25.0f);
            if (distance < hold - 12.0f)
                return std::max(-0.35f, (distance - (hold - 12.0f)) / 25.0f);
            return 0.0f;
        }
    }

    std::string_view duelPhaseName(DuelPhase phase) noexcept
    {
        switch (phase)
        {
            case DuelPhase::Approach: return "approach";
            case DuelPhase::Measure: return "measure";
            case DuelPhase::StepIn: return "step-in";
            case DuelPhase::StepOut: return "step-out";
            case DuelPhase::Recover: return "recover";
            case DuelPhase::Chase: return "chase";
        }
        return "approach";
    }

    // Reads the opponent's swing once (dodge or not, by skill) and, dodging
    // while inside its reach, backs off and side-steps. True when it dodges.
    static bool readSwing(DuelTacticsState& state, const DuelSelfView& self, const DuelOpponentView& opponent,
        const DuelTacticsSettings& settings, DuelTacticsOutput& out, float dt)
    {
        if (settings.mDodgeRead > 0.0f)
        {
            // The wind-up is read, after the reaction delay, by footwork.
            const bool threat = opponent.mWindingUp || opponent.mSwinging;
            if (!threat)
            {
                state.mThreatTime = 0.0f;
                state.mSwingSeen = state.mGivingGround = state.mBurstSent = false;
                return false;
            }
            state.mThreatTime += dt;
            const float delay = mix(settings.mReactionSlow, settings.mReactionFast, self.mFootwork);
            if (!state.mSwingSeen && state.mThreatTime >= delay)
            {
                state.mSwingSeen = true;
                state.mGivingGround = random01(state.mSeed)
                    < std::min(0.9f, mix(settings.mDodgeUnskilled, settings.mDodgeSkilled, self.mFootwork) + self.mDodgeBonus);
                // Away from the blade; a coin flip only for a centred one.
                state.mDodgeSideSign = std::abs(opponent.mBladeSide) > 5.0f ? (opponent.mBladeSide > 0.0f ? -1.0f : 1.0f)
                    : random01(state.mSeed) < 0.5f ? -1.0f : 1.0f;
            }
            if (state.mGivingGround && opponent.mDistance < opponent.mReach + settings.mDodgeRead)
            {
                out.mForward = -0.8f;
                out.mStrafe = settings.mDodgeSide * state.mDodgeSideSign;
                out.mReason = "dodge";
                if (!state.mBurstSent)
                {
                    state.mBurstSent = true;
                    out.mDodgeBurst = true;
                    out.mDodgeSideways = state.mDodgeSideSign;
                    out.mDodgeBack = 0.5f;
                }
                return true;
            }
            return false;
        }
        if (opponent.mSwinging && !state.mSwingSeen)
        {
            state.mSwingSeen = true;
            state.mGivingGround
                = random01(state.mSeed)
                < std::min(0.9f, mix(settings.mDodgeUnskilled, settings.mDodgeSkilled, self.mSkill) + self.mDodgeBonus);
            state.mDodgeSideSign = random01(state.mSeed) < 0.5f ? -1.0f : 1.0f;
        }
        else if (!opponent.mSwinging)
            state.mSwingSeen = state.mGivingGround = false;
        if (opponent.mSwinging && state.mGivingGround && opponent.mDistance < opponent.mReach + 20.0f)
        {
            out.mForward = -0.8f;
            out.mStrafe = settings.mDodgeSide * state.mDodgeSideSign;
            out.mReason = settings.mDodgeSide > 0.0f ? "side-step" : "give ground";
            return true;
        }
        return false;
    }

    // AI-02 item 2 (OPENMW_RAGDOLL_AI_THREAT): tracks the opponent's charge -
    // its start (noticed after the reaction delay), its end (closing under
    // 40 u/s for 0.3 s, 1.2 s, or its strike arriving) - and reads it once:
    // threat T, then one seeded draw of the response.
    static void readCharge(DuelTacticsState& state, const DuelSelfView& self, const DuelOpponentView& opponent,
        const DuelTacticsSettings& settings, float hold, float dt, DuelTacticsOutput& out)
    {
        const bool charging = opponent.mClosing > settings.mChargeSpeed && opponent.mDistance <= hold + 150.0f;
        if (state.mCharge == 1)
        {
            state.mChargeTime += dt;
            state.mChargeQuiet = opponent.mClosing < 40.0f ? state.mChargeQuiet + dt : 0.0f;
            // Its strike arriving ends the read: the swing read (dodge) takes it.
            const bool strikes = opponent.mSwinging && opponent.mDistance < opponent.mReach + 20.0f;
            if (state.mChargeQuiet >= 0.3f || state.mChargeTime >= 1.2f || strikes)
            {
                state.mCharge = 2;
                state.mChargeResponse = 0;
            }
        }
        if (state.mCharge == 2 && !charging)
            state.mCharge = 0;
        if (state.mCharge != 0)
            return;
        const bool reads = !self.mAttacking
            && (state.mPhase == DuelPhase::Approach || state.mPhase == DuelPhase::Measure
                || (state.mPhase == DuelPhase::StepIn && !state.mStruck));
        state.mChargeSeen = charging && reads ? state.mChargeSeen + dt : 0.0f;
        if (state.mChargeSeen < mix(0.45f, 0.12f, self.mAttackSkill))
            return;
        state.mCharge = 1;
        state.mChargeSeen = state.mChargeTime = state.mChargeQuiet = 0.0f;
        const float threat = std::clamp(opponent.mClosing / 250.0f, 0.0f, 1.0f)
                * (opponent.mWindingUp || opponent.mSwinging ? 1.0f : 0.6f)
            + std::clamp((opponent.mReach - self.mReach) / 60.0f, 0.0f, 0.3f) + 0.4f * (1.0f - self.mHealth)
            + 0.2f * (1.0f - self.mFatigue);
        out.mThreat = threat;
        if (threat < settings.mThreatMin)
        {
            state.mChargeResponse = 0;
            out.mThreatRead = "none";
            return;
        }
        const float cutOff = std::max(0.0f, self.mNerve - 0.4f) * self.mAttackSkill
            * (self.mReach >= opponent.mReach - 10.0f ? 1.0f : 0.3f);
        const float sideStep = self.mFootwork * (1.0f - 0.3f * threat);
        const float backpedal = 0.2f + 0.8f * (1.0f - self.mNerve);
        const float roll = random01(state.mSeed) * (cutOff + sideStep + backpedal);
        state.mChargeResponse = static_cast<std::uint8_t>(roll < cutOff ? 1 : roll < cutOff + sideStep ? 2 : 3);
        // Toward the charger's off-hand (this fighter's right for a
        // right-handed foe), the other side 30%.
        state.mChargeSide = random01(state.mSeed) < 0.3f ? -1.0f : 1.0f;
        out.mThreatRead = state.mChargeResponse == 1 ? "cut-off"
            : state.mChargeResponse == 2             ? "side-step"
                                                     : "backpedal";
    }

    // The read charge's response, while it lasts and before its own attack
    // has started. True when it moves the fighter; a cut off strikes when the
    // charger will be in reach by the time its blow lands.
    static bool chargeResponse(const DuelTacticsState& state, const DuelSelfView& self,
        const DuelOpponentView& opponent, float strike, DuelTacticsOutput& out)
    {
        if (state.mCharge != 1 || self.mAttacking)
            return false;
        switch (state.mChargeResponse)
        {
            case 1:
                out.mForward = 0.0f;
                out.mStrafe = 0.0f;
                out.mRun = false;
                out.mAttack = opponent.mDistance <= strike + std::max(0.0f, opponent.mClosing) * self.mStrikeLead;
                out.mReason = "cut off";
                return true;
            case 2:
                if (state.mChargeTime >= 0.6f)
                    return false;
                out.mForward = -0.3f;
                out.mStrafe = state.mChargeSide;
                out.mRun = false;
                out.mReason = "side-step charge";
                return true;
            case 3:
                // Walking back (never running backwards), guard up; a fighter
                // with nerve still cuts it off once it is in reach.
                out.mForward = -1.0f;
                out.mStrafe = 0.0f;
                out.mRun = false;
                out.mAttack = self.mNerve > 0.6f && opponent.mDistance <= strike + 5.0f;
                // The guard only while not cutting off (raised into its own
                // attack it ended the cut: review 10-08).
                out.mGuard = !out.mAttack;
                out.mReason = out.mAttack ? "cut off" : "backpedal";
                return true;
            default:
                return false;
        }
    }

    DuelTacticsOutput stepDuelTactics(DuelTacticsState& state, const DuelSelfView& self,
        const DuelOpponentView& opponent, float duration, const DuelTacticsSettings& settings)
    {
        DuelTacticsOutput out;
        const float dt = std::max(0.0f, duration);
        state.mTimeInPhase += dt;
        // Held just outside the longer of the two reaches; a strike needs the
        // gap down to its own reach.
        const float hold = std::max(self.mReach, opponent.mReach) + settings.mMargin;
        const float strike = self.mReach;
        if (state.mPatience < 0.0f)
        {
            state.mPatience = mix(settings.mPatienceCalm, settings.mPatienceFierce, self.mAggression)
                * (0.6f + 0.8f * random01(state.mSeed))
                * (self.mAggression < settings.mFrenzy ? std::max(0.1f, settings.mPatienceScale) : 1.0f);
            // AI-02 item 3 (AI_STAMINA): a tiring fighter waits longer; an
            // exhausted opponent halves it.
            if (settings.mStamina)
            {
                state.mPatience *= 1.0f + settings.mStaminaPace * (1.0f - std::clamp(self.mFatigue, 0.0f, 1.0f));
                if (opponent.mFatigue < settings.mStaminaExploit)
                    state.mPatience *= 0.5f;
                out.mPaceRead = true;
            }
        }

        // Tired, or the opponent is down: keep the distance and strike nothing.
        // A knocked-down opponent is given the time to rise.
        // How fast the gap opens (smoothed over ~0.1 s). Only the opponent's
        // backing away counts: while this fighter steps out or recovers the
        // gap opens by its own doing.
        if (state.mLastDistance >= 0.0f && dt > 0.0f)
        {
            const float rate = (opponent.mDistance - state.mLastDistance) / dt;
            state.mOpening += (rate - state.mOpening) * std::min(1.0f, dt / 0.1f);
        }
        state.mLastDistance = opponent.mDistance;
        const bool ownRetreat = state.mPhase == DuelPhase::StepOut || state.mPhase == DuelPhase::Recover;
        state.mRetreatSeen = !ownRetreat && state.mOpening > settings.mRetreatSpeed ? state.mRetreatSeen + dt : 0.0f;
        // AI-02 item 3 (AI_STAMINA): the strong-willed tire later, the
        // enduring rest sooner.
        const bool stamina = settings.mStamina;
        const float tiredBelow = stamina ? settings.mTiredBelow - 0.1f * self.mWillpower : settings.mTiredBelow;
        const float restedAbove
            = stamina ? settings.mRestedAbove + 0.1f * (1.0f - self.mEndurance) : settings.mRestedAbove;
        const bool tired = self.mFatigue < tiredBelow
            || (state.mPhase == DuelPhase::Recover && self.mFatigue < restedAbove);
        const bool spareDowned = opponent.mDown && self.mAggression < settings.mStrikeDownedAggression;
        if ((tired || spareDowned) && state.mPhase != DuelPhase::Recover && !self.mAttacking
            && state.mPhase != DuelPhase::StepOut)
            enter(state, DuelPhase::Recover);
        // An exhausted opponent is pressed like a downed one (AI-02 item 3).
        const bool pressExhausted = stamina && opponent.mFatigue < 0.1f
            && self.mAggression >= settings.mStrikeDownedAggression;
        // Pressing a downed foe: close in and strike from wherever we are.
        if (((opponent.mDown && !spareDowned) || pressExhausted) && !tired && !self.mAttacking
            && (state.mPhase == DuelPhase::Measure || state.mPhase == DuelPhase::Recover))
            enter(state, DuelPhase::StepIn);

        // A retreating opponent is chased, from anywhere but a strike in
        // progress, a step-out or recovery (nor below fatigue 0.35, AI-02
        // item 3).
        if (state.mRetreatSeen >= settings.mRetreatTime && state.mPhase != DuelPhase::Chase
            && state.mPhase != DuelPhase::Recover && !self.mAttacking && settings.mRetreatSpeed > 0.0f
            && !(stamina && self.mFatigue < 0.35f))
        {
            enter(state, DuelPhase::Chase);
            state.mStill = 0.0f;
        }

        // AI-02 item 2 (AI_THREAT): the opponent's charge, read once.
        if (settings.mThreat)
            readCharge(state, self, opponent, settings, hold, dt, out);
        bool chargeMove = false; // the charge response moves it this step

        // AI-02 item 5 (AI_RIPOSTE): a parry just won, in reach, is answered
        // at once (decided once per parry, by attack skill).
        bool riposteNow = false;
        if (settings.mRiposteWindow > 0.0f)
        {
            // A new parry - not another clash of the same exchange (each
            // bounce restarts the timer: re-rolled, a yes could flip to a no).
            if (self.mSinceParryWon < state.mLastSinceParryWon && state.mLastSinceParryWon > 0.25f)
                state.mRiposteRead = state.mRiposting = false;
            state.mLastSinceParryWon = self.mSinceParryWon;
            const bool canRiposte = self.mSinceParryWon < settings.mRiposteWindow && !tired && !self.mAttacking
                && self.mSinceDown >= settings.mGetUpRest && opponent.mDistance <= strike + 20.0f
                && (state.mPhase == DuelPhase::Measure || state.mPhase == DuelPhase::StepOut
                    || (state.mPhase == DuelPhase::StepIn && !state.mStruck));
            if (!canRiposte)
                state.mRiposting = false;
            else if (!state.mRiposteRead)
            {
                state.mRiposteRead = true;
                const float chance = mix(settings.mRiposteChance0, settings.mRiposteChance100, self.mAttackSkill);
                state.mRiposting = random01(state.mSeed) < chance;
                out.mRiposteRead = state.mRiposting ? "yes" : "no";
                out.mRiposteChance = chance;
            }
            if (state.mRiposting)
            {
                if (state.mPhase != DuelPhase::StepIn)
                    enter(state, DuelPhase::StepIn);
                riposteNow = true;
                out.mRiposte = true;
            }
        }

        switch (state.mPhase)
        {
            case DuelPhase::Chase:
            {
                out.mForward = 1.0f;
                out.mRun = opponent.mDistance > strike + settings.mLunge;
                out.mAttack = opponent.mDistance <= strike + settings.mLunge;
                out.mReason = out.mAttack ? "lunge" : "chase";
                if (self.mAttacking)
                    state.mStruck = true;
                // In reach without an attack starting: the chase is over. (The
                // gap stops opening whenever the chaser gains, so the rate
                // alone cannot tell a stopped opponent from a caught one.)
                state.mStill = opponent.mDistance <= strike + settings.mLunge && !self.mAttacking
                    ? state.mStill + dt : 0.0f;
                if (state.mStruck && !self.mAttacking)
                {
                    enter(state, DuelPhase::StepOut);
                    out = {};
                    out.mForward = -1.0f;
                    out.mReason = "lunged";
                }
                else if (state.mStill >= settings.mChaseSettle || state.mTimeInPhase > settings.mChaseTimeout)
                {
                    enter(state, DuelPhase::Measure);
                    out.mAttack = false;
                    out.mRun = false;
                    out.mReason = "chase over";
                }
                break;
            }
            case DuelPhase::Approach:
                if (settings.mThreat && chargeResponse(state, self, opponent, strike, out))
                {
                    chargeMove = true;
                    if (out.mAttack)
                        enter(state, DuelPhase::StepIn);
                    break;
                }
                if (opponent.mDistance <= hold + 30.0f)
                {
                    enter(state, DuelPhase::Measure);
                    out.mForward = measureMove(opponent.mDistance, hold);
                    out.mReason = "in measure";
                    break;
                }
                out.mForward = 1.0f;
                out.mRun = opponent.mDistance > hold + settings.mRunBeyond;
                out.mReason = "approach";
                break;
            case DuelPhase::Measure:
            {
                // A read charge is answered (AI-02 item 2; replaces the
                // composed back-off below while on).
                if (settings.mThreat)
                {
                    if (chargeResponse(state, self, opponent, strike, out))
                    {
                        chargeMove = true;
                        if (out.mAttack)
                            enter(state, DuelPhase::StepIn);
                        break;
                    }
                }
                // An opponent closing in while winding up is backed away from
                // (read once per wind-up, by skill).
                else if (settings.mBackOffSkilled > 0.0f || settings.mBackOffUnskilled > 0.0f)
                {
                    if (opponent.mWindingUp && !state.mWindUpSeen)
                    {
                        state.mWindUpSeen = true;
                        state.mBackingOff = random01(state.mSeed)
                            < mix(settings.mBackOffUnskilled, settings.mBackOffSkilled, self.mSkill);
                    }
                    else if (!opponent.mWindingUp)
                        state.mWindUpSeen = state.mBackingOff = false;
                    if (state.mBackingOff && state.mOpening < -30.0f && opponent.mDistance < opponent.mReach + 60.0f)
                    {
                        out.mForward = -0.7f;
                        out.mReason = "back off";
                        break;
                    }
                }
                if (opponent.mDistance > hold + 60.0f)
                {
                    enter(state, DuelPhase::Approach);
                    out.mForward = 1.0f;
                    out.mReason = "lost measure";
                    break;
                }
                out.mForward = measureMove(opponent.mDistance, hold);
                out.mReason = "waiting";
                // Its blade is coming and we are inside its reach: give ground,
                // if this swing was read in time (once per swing, by skill).
                // Every fighter always backing away made every blow whiff.
                if (readSwing(state, self, opponent, settings, out, duration))
                    break;
                const bool recovering = !opponent.mSwinging && opponent.mSinceSwing < settings.mOpeningWindow;
                // An exhausted opponent is an opening too (AI-02 item 3).
                const bool opening = recovering || opponent.mSinceBounce < 0.5f || opponent.mOffBalance
                    || opponent.mCannotBlock || (stamina && opponent.mFatigue < settings.mStaminaExploit);
                state.mOpeningSeen = opening ? state.mOpeningSeen + dt : 0.0f;
                // Its own guard broken: give ground, no attack (M6).
                if (self.mGuardBroken && !self.mAttacking)
                {
                    enter(state, DuelPhase::StepOut);
                    out.mReason = "guard broken";
                    break;
                }
                state.mPatience -= dt;
                if (state.mOpeningSeen >= mix(settings.mReactionSlow, settings.mReactionFast, self.mSkill))
                {
                    enter(state, DuelPhase::StepIn);
                    out.mReason = "opening";
                }
                // Below fatigue 0.45 no patience strikes: openings, cut offs
                // and ripostes only (AI-02 item 3).
                else if (state.mPatience <= 0.0f && !(stamina && self.mFatigue < 0.45f))
                {
                    enter(state, DuelPhase::StepIn);
                    out.mReason = "patience";
                }
                break;
            }
            case DuelPhase::StepIn:
                // A charge read before its own attack has started is answered
                // (AI-02 item 2), a cut off striking from here.
                if (settings.mThreat && !state.mStruck && chargeResponse(state, self, opponent, strike, out))
                {
                    chargeMove = true;
                    break;
                }
                // A swing coming before this fighter's own attack has started
                // is dodged rather than walked into.
                if (!state.mStruck && !self.mAttacking && readSwing(state, self, opponent, settings, out, duration))
                    break;
                // Pressing in through the swing so the blade reaches.
                out.mForward = opponent.mDistance > strike - 5.0f ? 1.0f : 0.4f;
                out.mAttack = opponent.mDistance <= strike + 5.0f;
                // AI-02 item 1a (AI_AIM): the attack starts where the closing
                // foe will be in reach when it lands, and its own attack no
                // longer walks into the foe: it holds the strike distance.
                if (settings.mAim)
                {
                    out.mAttack = !state.mStruck
                        && opponent.mDistance
                            <= strike + 5.0f + std::max(0.0f, opponent.mClosing) * settings.mAimLead;
                    state.mAimQuiet = self.mAttacking ? 0.0f : state.mAimQuiet + dt;
                    if (self.mAttacking || state.mStruck)
                    {
                        const float near = strike - settings.mAimDepth;
                        const float held = opponent.mDistance > near
                            ? 0.6f
                            : (opponent.mDistance < near - 15.0f ? -0.4f : 0.0f);
                        out.mForward = mix(out.mForward, held, settings.mAimHoldShare);
                    }
                }
                out.mReason = out.mAttack ? "strike" : "closing";
                if (self.mAttacking)
                    state.mStruck = true;
                if ((state.mStruck && !self.mAttacking && (!settings.mAim || state.mAimQuiet >= settings.mAimGrace))
                    || (!self.mAttacking && !state.mStruck && state.mTimeInPhase > settings.mStrikeTimeout))
                {
                    out.mReason = state.mStruck ? "struck" : "gave up";
                    enter(state, DuelPhase::StepOut);
                    out.mAttack = false;
                    out.mForward = -1.0f;
                }
                break;
            case DuelPhase::StepOut:
                out.mForward = -1.0f;
                out.mReason = "step out";
                if ((state.mTimeInPhase >= settings.mStepOut && opponent.mDistance >= hold - 10.0f)
                    || state.mTimeInPhase >= 2.0f * settings.mStepOut)
                    enter(state, DuelPhase::Measure);
                break;
            case DuelPhase::Recover:
                // AI_STAMINA (AI-02 item 3) holds farther while recovering.
                out.mForward = hover(opponent.mDistance, hold + (stamina ? 80.0f : 50.0f), 0.7f);
                out.mReason = opponent.mDown ? "opponent down" : "catching breath";
                if (!tired && !spareDowned)
                    enter(state, DuelPhase::Measure);
                break;
        }
        // The riposte strikes now, stepping in (AI-02 item 5).
        if (riposteNow)
        {
            out.mAttack = true;
            out.mForward = 0.5f;
            out.mStrafe = 0.0f;
            out.mRun = false;
            out.mGuard = false;
            out.mReason = "riposte";
            chargeMove = false;
        }
        // Footwork: circle while measuring, leave sideways, never stay pressed
        // chest to chest.
        if (settings.mCircle > 0.0f)
        {
            if (state.mCircleLeft < 0.0f)
            {
                state.mCircleLeft = mix(settings.mCircleMin, settings.mCircleMax, random01(state.mSeed));
                state.mCircleSide = random01(state.mSeed) < 0.5f ? -1.0f : 1.0f;
            }
            state.mCircleLeft -= dt;
            if (state.mCircleLeft <= 0.0f)
            {
                state.mCircleSide = -state.mCircleSide;
                state.mCircleLeft = mix(settings.mCircleMin, settings.mCircleMax, random01(state.mSeed));
            }
            if (state.mPhase == DuelPhase::Measure && out.mReason != "side-step" && out.mReason != "dodge" && !chargeMove)
                out.mStrafe = settings.mCircle * state.mCircleSide;
            else if (state.mPhase == DuelPhase::StepOut)
                out.mStrafe = 0.8f * state.mCircleSide;
            if (opponent.mDistance < settings.mLockGap && !self.mAttacking && state.mPhase != DuelPhase::StepIn
                && state.mPhase != DuelPhase::Chase)
            {
                out.mForward = -1.0f;
                out.mStrafe = state.mCircleSide;
                out.mReason = "unlock";
            }
        }
        // Just back up: steady first - no attack, no pressing in, no running.
        if (self.mSinceDown < settings.mGetUpRest && !self.mAttacking)
        {
            out.mAttack = false;
            out.mRun = false;
            out.mForward = std::min(out.mForward, 0.0f);
            out.mReason = "steadying";
        }
        out.mPhase = state.mPhase;
        return out;
    }
}

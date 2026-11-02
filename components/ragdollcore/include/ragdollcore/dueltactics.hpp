#ifndef OPENMW_RAGDOLLCORE_DUELTACTICS_H
#define OPENMW_RAGDOLLCORE_DUELTACTICS_H

#include <cstdint>
#include <string_view>

namespace RagdollCore
{
    // DUEL-AI-01 stage 1 (user 2026-09-29: "the duels slower and more thought
    // out, instead of running and spamming at the player, they wait for the
    // correct moment"). A melee NPC holds at the edge of reach, steps in to
    // strike on an opening (the opponent recovering from a swing, knocked off
    // line by a bounce, off balance) or when its patience runs out, then steps
    // back out. Pure: values in, values out, deterministic from the seed.
    enum class DuelPhase : std::uint8_t
    {
        Approach, // far: close the distance
        Measure, // at the edge of reach, waiting for an opening
        StepIn, // closing to strike
        StepOut, // backing off after a strike
        Recover, // tired or the opponent is down: keep the distance, no strikes
        Chase, // the opponent backs away: run it down and lunge in reach
    };

    std::string_view duelPhaseName(DuelPhase phase) noexcept;

    // What the physics can show of one powered fighter.
    struct DuelBodyView
    {
        float mTipSpeed = 0.0f; // its weapon tip (or striking hand) speed
        float mSinceFast = 1.0e9f; // seconds since that was fast
        float mSinceBounce = 1.0e9f; // seconds since its weapon bounced or clashed
        bool mOffBalance = false; // stumbling
        bool mDown = false; // falling, fallen, getting up
        bool mArmed = false;
        float mWeaponReach = 0.0f; // its weapon's tip distance from the hand (0 unarmed)
        bool mWheel = false; // on the attack wheel (hand control)
        float mHitsTaken = 0.0f; // blows taken lately, decaying (the AI guard's learning)
        bool mWindingUp = false; // winding up an attack on the wheel
        // RXN-02 M6, outcomes the AI reads: its guard was broken or knocked
        // away in a clash (no block for a while), or a blow staggered it.
        bool mGuardBroken = false;
        bool mStaggered = false;
        // AI-02 item 5 (OPENMW_RAGDOLL_AI_RIPOSTE): seconds since it last won
        // a parry (knocked the attacker's blade away or disarmed it).
        float mSinceParryWon = 1.0e9f;
        // AI-02 item 1a (OPENMW_RAGDOLL_AI_AIM): its wheel attack's stage, 0
        // guard/idle, 1 wind-up, 2 cut, 3 recover.
        int mWheelStage = 0;
        // Its weapon tip across its own facing (units, + = to its right): the
        // side a cut comes from (NPC dodges, STEPPING audit item 19).
        float mTipSide = 0.0f;
    };

    struct DuelOpponentView
    {
        float mDistance = 0.0f; // gap between the two bodies (units)
        float mReach = 60.0f; // how far its weapon reaches past its body
        bool mSwinging = false; // its weapon is moving fast now
        float mSinceSwing = 1.0e9f; // seconds since its weapon was last fast
        float mSinceBounce = 1.0e9f; // seconds since its weapon bounced (armour, clash, parry)
        bool mOffBalance = false; // stumbling or bracing hard
        bool mDown = false; // falling, fallen or getting up
        bool mWindingUp = false; // winding up an attack (a held, charging blow)
        // Its blade across this fighter's facing (units, + = on this fighter's
        // right): a cut from there is dodged to the left (mDodgeRead).
        float mBladeSide = 0.0f;
        // It cannot block now (a broken guard, a stagger): strike (JA: the
        // broken parry's opening).
        bool mCannotBlock = false;
        // AI-02 item 2: its own speed toward this fighter (units/s; a charge).
        float mClosing = 0.0f;
        // AI-02 item 3: its fatigue, current / base, 0..1.
        float mFatigue = 1.0f;
    };

    struct DuelSelfView
    {
        float mReach = 60.0f;
        float mFatigue = 1.0f; // current / base, 0..1
        bool mAttacking = false; // an attack of its own is in progress
        float mAggression = 0.5f; // 0..1 (Fight)
        float mSkill = 0.5f; // 0..1 (weapon skill): how fast it sees an opening
        float mSinceDown = 1.0e9f; // seconds since this fighter was last down (falling, fallen, getting up)
        float mDodgeBonus = 0.0f; // added to the dodge chance (blows taken lately: learning)
        bool mGuardBroken = false; // its own guard was just broken: give ground
        // AI-02 fighter terms (duelStats, 0..1; read only by the AI_* items).
        float mAttackSkill = 0.5f;
        float mGuardSkill = 0.5f;
        float mNerve = 0.5f;
        float mFootwork = 0.5f;
        float mHealth = 1.0f; // current / base
        float mWillpower = 0.5f; // attribute / 100 (AI-02 item 3: when it tires)
        float mEndurance = 0.5f; // attribute / 100 (AI-02 item 3: when it is rested)
        // AI-02 item 1: seconds from the attack decision to contact (wind,
        // chamber, cut); the item 2 cut off leads a charger by it.
        float mStrikeLead = 0.45f;
        float mSinceParryWon = 1.0e9f; // AI-02 item 5: seconds since it won a parry
    };

    // AI-02 (docs/ragdoll/designs/AI-02-smart-duel-ai.md): the game's stats as
    // four fighter terms, 0..1. Attack A = (weapon skill + Agility/5 +
    // Luck/10) x fatigue term / 100; guard G the same with Block (averaged with
    // the weapon skill without a shield); nerve N = 0.5 + 0.5 Fight - 0.4 Flee
    // + 0.3 (Willpower - 50)/50 + 0.1 level difference/10; footwork F = (0.6
    // Agility + 0.4 Speed)/100 x fatigue term.
    struct DuelStatsInput
    {
        float mWeaponSkill = 50.0f, mBlock = 50.0f;
        bool mShield = false;
        float mAgility = 50.0f, mLuck = 50.0f, mSpeed = 50.0f, mWillpower = 50.0f;
        float mFight = 50.0f, mFlee = 0.0f; // AI settings, 0..100
        float mLevelDifference = 0.0f; // own level - the opponent's
        float mFatigueTerm = 1.0f; // CreatureStats::getFatigueTerm()
    };
    struct DuelStats
    {
        float mAttack = 0.5f, mGuard = 0.5f, mNerve = 0.5f, mFootwork = 0.5f;
    };
    inline DuelStats duelStats(const DuelStatsInput& in)
    {
        const auto unit = [](float value) { return value > 0.0f ? (value < 1.0f ? value : 1.0f) : 0.0f; };
        const float fatigue = in.mFatigueTerm > 0.0f ? in.mFatigueTerm : 0.0f;
        const float block = in.mShield ? in.mBlock : 0.5f * (in.mBlock + in.mWeaponSkill);
        DuelStats out;
        out.mAttack = unit((in.mWeaponSkill + in.mAgility / 5.0f + in.mLuck / 10.0f) * fatigue / 100.0f);
        out.mGuard = unit((block + in.mAgility / 5.0f + in.mLuck / 10.0f) * fatigue / 100.0f);
        const float level = in.mLevelDifference / 10.0f;
        out.mNerve = unit(0.5f + 0.5f * in.mFight / 100.0f - 0.4f * in.mFlee / 100.0f
            + 0.3f * (in.mWillpower - 50.0f) / 50.0f + 0.1f * (level < -1.0f ? -1.0f : level > 1.0f ? 1.0f : level));
        out.mFootwork = unit((0.6f * in.mAgility + 0.4f * in.mSpeed) / 100.0f * fatigue);
        return out;
    }

    struct DuelTacticsSettings
    {
        float mMargin = 14.0f; // held just outside reach
        float mRunBeyond = 220.0f; // runs only when this far past the hold distance
        float mPatienceCalm = 3.5f; // seconds of waiting at aggression 0
        float mPatienceFierce = 1.0f; // at aggression 1
        float mReactionSlow = 0.45f; // opening noticed after this at skill 0
        float mReactionFast = 0.12f; // at skill 1
        float mOpeningWindow = 0.8f; // an opponent's recovery lasts this long after its swing
        float mStepOut = 0.6f; // seconds backing off after a strike
        float mStrikeTimeout = 1.6f; // gives up a step-in that never reached a strike
        float mDodgeUnskilled = 0.25f; // chance to give ground to a swing at skill 0
        float mDodgeSkilled = 0.75f; // at skill 1
        // Composed fighters (build 59 bookmarks 4-5: "NPCs should aim more,
        // attack less (unless frenzied)"; "back up if the player walks toward
        // them while charging an attack"). Patience is this many times longer
        // below a frenzy (aggression under mFrenzy), so more attacks come from
        // openings; an opponent closing in while winding up is backed away
        // from, at these chances by skill (0 = never).
        float mPatienceScale = 1.0f;
        float mFrenzy = 0.9f;
        float mBackOffUnskilled = 0.0f;
        float mBackOffSkilled = 0.0f;
        // A read swing is side-stepped as well as backed from (user bookmark
        // 13, 2026-09-30: "they should be able to sidestep and dodge the
        // player attacks"): sideways at this share (0 = straight back), the
        // side drawn per dodge; also while stepping in, before its own attack
        // has started.
        // Duels x2: 115/145 side-steps (vs 119/117 straight back), blows
        // landed 127 vs 141, all duel tests pass.
        float mDodgeSide = 0.8f;
        // NPC dodges that read the wind-up (STEPPING audit 11-01, item 19; the
        // movement audit's item 4: the side was a coin flip per swing, the read
        // was the cut itself ~0.15-0.2 s before contact, and the dodge was a
        // walk). Over 0, a wind-up or a swing is read after the reaction delay
        // (mReactionSlow..Fast by footwork), the chance comes from footwork x
        // fatigue (mDodgeUnskilled..Skilled), the step goes away from the
        // side the blade is on (a coin flip only for a centred blade), and
        // the first step is a burst (DuelTacticsOutput::mDodgeBurst: the
        // body's committed dodge step, OPENMW_RAGDOLL_DODGE) when within
        // reach + this many units. 0 = off (the old read of the cut).
        float mDodgeRead = 0.0f;
        // Steadying after its own get-up (user bookmark 10, 2026-09-30: "NPCs
        // need a bit help to not stumble over and over after getting up"):
        // for this long after rising it neither attacks nor presses in.
        float mGetUpRest = 1.2f;
        float mTiredBelow = 0.25f; // fatigue share that makes it recover
        float mRestedAbove = 0.5f;
        // Chase (alpha 0.4 item 4, user 2026-09-30: "an opponent who retreats
        // is sprinted down and lunged at in reach"). The gap opening faster
        // than mRetreatSpeed (units/s) for mRetreatTime starts a chase; in it
        // the fighter runs and attacks once the gap is within its reach plus
        // mLunge (the attack's own step carries it the rest). A chase ends on
        // the strike, after mChaseSettle in reach with no attack starting, or
        // after mChaseTimeout.
        float mRetreatSpeed = 45.0f; // 0 = no chase
        float mRetreatTime = 0.25f;
        float mLunge = 30.0f;
        float mChaseSettle = 1.0f;
        float mChaseTimeout = 5.0f;
        // A downed opponent (user 2026-09-30 bookmark: "enemies should be able
        // to hit the player while he's down"): fighters at or above this
        // aggression press in and strike; calmer ones give it time to rise.
        float mStrikeDownedAggression = 0.3f;
        // Footwork around the foe (user 2026-09-30 bookmark: "two characters
        // bump and lock onto each other as they do not strafe around"):
        // while measuring it circles at mCircle (sideways, -1..1), turning
        // back every mCircleMin..mCircleMax s; a step-out leaves sideways too;
        // closer than mLockGap the bodies are pressed together and it backs off
        // sideways. mCircle 0 = none (the stage-1 tactics).
        float mCircle = 0.5f;
        float mCircleMin = 1.5f;
        float mCircleMax = 3.5f;
        float mLockGap = 6.0f;
        // AI-02 item 1a, strike at the sweet spot (OPENMW_RAGDOLL_AI_AIM =
        // "lead s, hold share", start 0.45,1; NPCHIT-01 10-07: NPCs walked
        // through their own wind-up into the foe and cut while backing out).
        // A step-in attacks when the gap is within strike + 5 + closing x
        // mAimLead; while its attack runs it no longer presses in: forward
        // 0.6 beyond strike, 0 at strike, -0.4 inside strike - 15, mixed with
        // the old press (1 / 0.4) by mAimHoldShare (1 = the new rule only).
        bool mAim = false;
        float mAimLead = 0.45f;
        float mAimHoldShare = 1.0f;
        // ... and holds this far inside the strike distance (measured 10-08:
        // NPC cuts bit 80% from a chest gap under 60 - bounds 0-10 - and none
        // from 100+), staying put through the wheel's wind-up and cut: an NPC's
        // wheel attack starts as the AI attack ends (its release), so "struck"
        // waits `mAimGrace` s of no attack before stepping out.
        float mAimDepth = 30.0f;
        float mAimGrace = 0.3f;
        // AI-02 item 2, threat read on a charge (OPENMW_RAGDOLL_AI_THREAT=1).
        // The opponent closing faster than mChargeSpeed within hold + 150 is
        // a charge, read once after the reaction mix(0.45, 0.12, A): threat T
        // below mThreatMin gets no response, else one seeded draw of cut off
        // (hold ground, strike it as it comes), side-step or backpedal (walk
        // back, guard up). Replaces COMPOSED's mBackOff* while on.
        bool mThreat = false;
        // 180 (10-08): at 120 every step-in of a walking duelist read as a
        // charge (DUEL-A02: 29 backpedals, 16 side-steps in one duel); the
        // test player's run closes at ~220 (AI02-CHARGE), so 230 never fired.
        float mChargeSpeed = 180.0f;
        float mThreatMin = 0.35f;
        // AI-02 item 3, stamina pacing (OPENMW_RAGDOLL_AI_STAMINA = "pace,
        // exploit below", start 1.5,0.3): patience x (1 + pace (1 - fatigue)),
        // no patience strikes below fatigue 0.45, tired/rested lines by
        // Willpower/Endurance, Recover holds farther and never chases below
        // 0.35; an opponent under mStaminaExploit is an opening (patience
        // halved), under 0.1 it is pressed like a downed foe.
        bool mStamina = false;
        float mStaminaPace = 1.5f;
        float mStaminaExploit = 0.3f;
        // AI-02 item 5, riposte after its own parry (OPENMW_RAGDOLL_AI_RIPOSTE
        // = "window s, chance A0, A100, wind share", start 0.6,0.3,0.9,0.5):
        // a parry won less than mRiposteWindow ago, in reach, is answered
        // with an attack at chance mix(A0, A100, A), decided once per parry.
        // 0 = off. The wind share is the physics side's (the riposte's wind
        // time x share); the tactics do not read it.
        float mRiposteWindow = 0.0f;
        float mRiposteChance0 = 0.3f;
        float mRiposteChance100 = 0.9f;
        float mRiposteWindShare = 0.5f;
    };

    struct DuelTacticsState
    {
        DuelPhase mPhase = DuelPhase::Approach;
        float mTimeInPhase = 0.0f;
        float mPatience = -1.0f; // < 0: draw a new one
        float mOpeningSeen = 0.0f; // how long the current opening has been visible
        bool mStruck = false; // the step-in's attack has started
        float mAimQuiet = 0.0f; // AI_AIM: since its attack (AI or wheel) last ran
        bool mSwingSeen = false; // the opponent's current swing has been read
        bool mGivingGround = false; // ... and this fighter backs away from it
        float mDodgeSideSign = 1.0f; // the side-step's direction for this swing
        float mThreatTime = 0.0f; // mDodgeRead: how long the opponent's wind-up or swing has been visible
        bool mBurstSent = false; // mDodgeRead: this swing's burst was asked for
        bool mWindUpSeen = false, mBackingOff = false; // the opponent's current wind-up, read once
        float mLastDistance = -1.0f; // the gap last step (< 0: none yet)
        float mOpening = 0.0f; // how fast the gap opens (units/s, smoothed)
        float mRetreatSeen = 0.0f; // how long the opponent has been backing away
        float mStill = 0.0f; // in a chase: how long it has been in reach without attacking
        float mCircleSide = 1.0f; // current circling direction (+1 right, -1 left)
        float mCircleLeft = -1.0f; // seconds until it turns (< 0: draw)
        // AI-02 item 2: the opponent's charge. mCharge 0 = none, 1 = read
        // (responding), 2 = over (waits for the charge to stop, so one charge
        // is read once); mChargeResponse 0 none, 1 cut off, 2 side-step, 3
        // backpedal.
        std::uint8_t mCharge = 0;
        std::uint8_t mChargeResponse = 0;
        float mChargeSeen = 0.0f; // how long the charge has shown before its read
        float mChargeTime = 0.0f; // since the read
        float mChargeQuiet = 0.0f; // how long it has closed slower than 40 u/s
        float mChargeSide = 1.0f; // the side-step's direction
        // AI-02 item 5: the current parry won, read once.
        float mLastSinceParryWon = 1.0e9f;
        bool mRiposteRead = false;
        bool mRiposting = false;
        std::uint32_t mSeed = 1;
    };

    struct DuelTacticsOutput
    {
        float mForward = 0.0f; // -1 back .. 1 forward
        float mStrafe = 0.0f; // -1 left .. 1 right
        bool mRun = false;
        bool mAttack = false; // may start an attack now
        DuelPhase mPhase = DuelPhase::Approach;
        std::string_view mReason;
        bool mGuard = false; // AI-02 item 2: raise the guard (a backpedal from a charge)
        // AI-02 trace (OPENMW_RAGDOLL_AI_TRACE): the decisions of this step.
        std::string_view mThreatRead; // a charge read: "none", "cut-off", "side-step", "backpedal"
        float mThreat = 0.0f; // ... its threat T
        bool mPaceRead = false; // a patience drawn under AI_STAMINA (DuelTacticsState::mPatience)
        std::string_view mRiposteRead; // a parry won read: "yes" or "no"
        bool mRiposte = false; // answering a won parry now (the physics winds it up short)
        float mRiposteChance = 0.0f;
        // mDodgeRead: dodge now with a committed step - sideways by mDodgeSideways
        // (+1 right, -1 left of the facing) and back by mDodgeBack (0..1).
        bool mDodgeBurst = false;
        float mDodgeSideways = 0.0f;
        float mDodgeBack = 0.0f;
    };

    DuelTacticsOutput stepDuelTactics(DuelTacticsState& state, const DuelSelfView& self,
        const DuelOpponentView& opponent, float duration, const DuelTacticsSettings& settings = {});
}

#endif

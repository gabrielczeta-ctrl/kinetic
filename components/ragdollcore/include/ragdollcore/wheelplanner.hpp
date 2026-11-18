#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_WHEELPLANNER_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_WHEELPLANNER_HPP

#include <ragdollcore/cutwheel.hpp>
#include <ragdollcore/fistcontrol.hpp>
#include <ragdollcore/fiststeer.hpp>
#include <ragdollcore/wheelaim.hpp>

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace RagdollCore
{
    // KINETIC-2 phase 1, the cut-wheel planner's stage machine (part 2;
    // docs/ragdoll/designs/K2-WEAPON-phase1.md, "The wheel's rules"). One
    // fighter's weapon hand: guard -> wind-up -> cut -> hold -> return, the
    // stance circle the mouse moves, clicks and their queue, chains by skill,
    // the block, NPC feints and the chamber tell. Each step it gives the
    // hand's pose in the chest-centre frame (cutwheel.hpp) and the events the
    // engine prices (a cut beginning, a feint's fatigue).
    //
    // With an aim (part 3, wheelaim.hpp) the wind-up and cut are bent to it:
    // the cut plane, thrusts, the reach; without one the keyed lines play.
    // Part 4: the drive's phase, the clash recoil and the trunk (twist, lean,
    // roll, the low stance). The arm request's last touches (first-person
    // clearance, the haft, an NPC's steady guard) are in wheeloutput.hpp.
    // Pure: no engine types.

    enum class WheelStage : std::uint8_t
    {
        Guard,
        Wind,
        Cut,
        Recover,
    };

    enum class WheelGuardZone : std::uint8_t
    {
        Middle,
        High,
        Left,
        Right,
        Low,
        Shove, // the weapon hand thrust out (K2_SHOVE_ARMS; not a guard)
    };

    struct WheelStriker
    {
        const WheelLines* mLines = &weaponLines(); // fistLines() for fists
        WeaponStrikeAxes mAxes; // the blade and edge for the hand's rotations
        JPH::Vec3 mBladeAxis = JPH::Vec3::sAxisY(); // hand frame: grip to tip
        float mTip = 40.0f; // the tip's distance from the hand
        // Where the striking part begins, a share of hand to tip (the head of
        // an axe or hammer, the blade past a sword's guard: weaponShapeProfile).
        float mStrikeStart = 0.0f;
        float mWeight = 20.0f; // Morrowind weight (fists 0)
        bool mTwoHanded = false;
        bool mShort = false; // fists and daggers
        bool mSpearThrusts = false; // every line is the thrust (NPC spears, half-sword)
        bool mSpearSwings = false; // the player's spear swings the lines
        bool mPointThrust = false; // K2_SPEAR_THRUST: a thrust's point is put on the aim's line
        bool mPointGuard = false; // K2_POINT_AIM: a long haft the player holds point-forward
    };

    struct WheelActor
    {
        bool mPlayer = true;
        float mSkill = 100.0f; // the weapon skill (chains from 50)
        float mControl = 1.0f; // 0..1: skill, Agility, Luck, fatigue
        float mResponse = 1.0f; // the weapon's response, x the attack speed (K2_WEAPON_PACE)
        std::uint64_t mSerial = 0; // the actor's draws (NPC feints)
        float mSideTurn = 0.0f; // a sideways cut's extra chest turn, x the keys' (K2_SIDE_TURN)
        float mHipShare = 0.0f; // the hips' share of the trunk's lead turn (K2_HIP_TURN)
        // The cut's chest turn taken this far ahead on the route (seconds;
        // K2_CUT_LEAD; user 11-05: "rotate the chest when the chop/slash is
        // being executed"): the spine's motors trail their target ~0.1-0.2 s,
        // so a cut's turn landed after the blade. 0 = the turn where the
        // blade is, as before.
        float mCutLead = 0.0f;
        // The hips' turn cap in a cut and its follow-through (radians;
        // K2_CUT_HIP_CAP; user 11-05: a bigger follow-through - the slash's
        // wind-up turned the shoulders ~79 deg back, its follow-through
        // stopped ~57 deg the other way, the hips capped at 0.45 rad and the
        // spine at its range). Eased at 1.5 rad/s, so the hips never jump.
        // 0 = 0.45 throughout, as before.
        float mCutHipCap = 0.0f;
        // A one-handed cut reaches out (K2_CUT_REACH; bookmark 11-05: "a left
        // to right hard swing should extend the arm more with 1 handed
        // sword"): this share of the gap between the grip's distance from the
        // shoulder and 0.95 of the arm's reach is closed through the cut,
        // most from a third of the way in to three quarters, half kept at its
        // end. The keys asked a bent arm through the cut (the sparksword's
        // backhand: elbow 104 deg commanded mid-cut, 78 physical), so the
        // blade struck on a folded arm and straightened only in the hold.
        // Not thrusts (their own reach), fists and daggers (theirs), or
        // two-handed grips. 0 = the keys' reach, as before. (The switch's
        // second value, the elbow's speed through such a cut, is the arm
        // solve's, in the engine.)
        float mCutReach = 0.0f;
        // An NPC blocks in the zone the incoming blade comes from
        // (K2_GUARD_THREAT; CLASH-02 11-05: an NPC's raised guard stayed in
        // the middle zone, which only the player's mouse moves, and a chop
        // from above passed over it in 3 of 3 runs while a cover-up pose met
        // it): above the head the hanging high block, to a side that side,
        // low the low block. Not the player. False = the middle zone, as
        // before.
        bool mThreatZone = false;
        // The block's hand moves to meet the incoming blade (K2_PARRY_STEER,
        // units; user 11-12: "add steering and tolerance zones"): each zone
        // was one fixed hand place, so a blow in the right zone still passed
        // beside the blade. The hand goes up to this far from the zone's
        // place (0.4 of it at no weapon control, all of it at full), across
        // the front, to where the foe's striking part will cross the guard.
        // 0 = the zone's place, as before.
        float mParrySteer = 0.0f;
        bool mHeldRecover = false; // a finished cut recovers with the button held (K2_HELD_RECOVER, B14)
        float mTwoHandReach = 1.0f; // the arm's share a two-handed grip reaches with (K2_TWO_HAND_REACH, B6)
        // An NPC's combo (K2_NPC_COMBO): the chance of a second cut at skill
        // 100, and of a third after a second.
        float mComboSecond = 0.0f, mComboThird = 0.0f;
        // The cut's path marks where the blade first meets a foe's body of
        // this radius (K2_AIM_PREVIEW, units; 0 = no mark).
        float mPreviewFoeRadius = 0.0f;
        // K2_WINDUP_RADIUS (user 11-08 bookmark: hold the click and move the
        // weapon around to aim, "mainly ... stab incoming attackers"): the
        // player's stance circle, mouse counts; 0 = sStanceRadius.
        float mStanceRadius = 0.0f;
        // K2_FLICK_CHAIN (user bookmark 11-05: "chop chop chop hack hack slash
        // slash like crazeyyy right now its capped by how we release the click
        // and click again"): the button held, a finished cut (or its return)
        // takes a drag away from where the blade is as the next wind-up at
        // once, from where the blade is, with the momentum - as a press
        // chains, by the same skill rule (from 50 any time; below, after
        // HSC_WAIT_RETURN of the return). False = the held drag waits for the
        // guard, as before.
        bool mFlickChain = false;
        // K2_MOMENTUM_WIND (user bookmark 11-05: "if the body momentum is going
        // that way, charging the attack is much easier ... the inertia helps
        // it build up"): the wind-up is up to 1 + this as fast when the body
        // moves along the coming cut (full at 300 u/s; half credit for moving
        // forward into any cut). 0 = off.
        float mMomentumWind = 0.0f;
        // K2_DAGGER_THRUST (user bookmark 11-05: "thrusting a dagger can be done
        // extremely fast, like lets almost not even cap how fast"): the
        // player's dagger thrust winds up in 0.04 s, strikes in 0.07, returns in
        // 0.16 (/ the weapon's response), and a press chains from any stage at
        // any skill. False = the short blade's pace (0.12, 0.15, 0.5).
        bool mDaggerThrust = false;
        // K2_SPEAR_THRUST_PACE (user 11-16: the spear thrust's wind-up "should
        // be much easier, almost like dagger (but the speed should be slower
        // than dagger)"): the player's spear or staff on the thrust line winds
        // up, strikes and returns in these seconds (/ the response) and, as
        // the dagger, a press chains from any stage at any skill. A wind-up
        // of 0 = the weapon's weight pace and the skill 50 chain rule.
        float mSpearThrustWind = 0.0f, mSpearThrustCut = 0.0f, mSpearThrustReturn = 0.0f;
        // K2_FISTS (user 11-05: "I can only move 1 hand and its impossible to
        // aim"): with bare fists a held drag to the left throws the left fist
        // (FIST-02's punch, at the crosshair) and the right stays on guard; a
        // drag to the right winds the right fist up as before.
        bool mFistSides = false;
        // FIST-05 (K2_FIGHT_NIGHT; user 11-07: "something like fight night
        // fistfighting"): with bare fists the player's press is a flick - a
        // held drag does not steer, its release throws the punch its
        // direction picks (fistcontrol classifyFlick), with either hand,
        // powered by its length; a press without a drag throws the next
        // straight of the jab-cross alternation.
        bool mFightNight = false;
        // FIST-06 (K2_FIST_STEER; user 11-12: "when I click and hold with
        // fists nothing happens"): with bare fists the player's hands are
        // steered (fiststeer.hpp) - the right fist's held place and its
        // punches come in WheelInput (mFistHold, mFistPunch) and the press
        // itself is not read here: no stance circle, no flick, no click.
        bool mFistSteer = false;
        // K2_FIST_TRUE (user 11-16: "the fists feel wobbly and they are not
        // straight/weighted at all"): the player's fist holds this long at
        // the end of its strike and returns in this long (0 = the short
        // weapons' 0.35 and 0.5 s: the fist hung at full stretch); its
        // strike takes mFistSnapCut of the short weapons' time (the route is
        // cut short at the arm's reach).
        float mFistSnapHold = 0.0f, mFistSnapReturn = 0.0f, mFistSnapCut = 1.0f;
        // K2_FIST_WOUND: the player's wound hook or uppercut - its strike's
        // least seconds and its wind-up's share of a quick punch's (0 = as
        // it was: woundStrikeSeconds, woundWindShare).
        float mFistWoundCut = 0.0f, mFistWoundWind = 0.0f;
        // K2_THRUST_KEEP (radians; user 11-06, the spear: "a thrust wind-up
        // that stays a thrust"): the player's long haft (spear, staff) wound
        // up on the thrust line commits as a thrust for any stroke within this
        // of straight ahead. The cut's line came from the stroke alone, so a
        // push more than an eighth's half (22.5 deg) off straight became a
        // shaft swing - 2-3 of 13 thrust wind-ups in the 11-06 18:19 session.
        // 0 = the stroke's line, as before.
        float mThrustKeep = 0.0f;
        // K2_POINT_AIM (user 11-12: "holding click lets you control/aim the
        // tip"): the player's long haft (WheelStriker::mPointGuard), the
        // button held, is carried point-forward and the mouse moves the point:
        // the rear hand stays, the shaft turns about it toward the aim's
        // point (the crosshair's, a near foe's) offset by the mouse, within
        // `mPointCone` of it (radians; 0 = off: a held drag winds up on the
        // wheel, as before), `mPointCounts` mouse counts a radian. The
        // release thrusts straight down the shaft: drawn back `mPointDraw`,
        // driven `mPointReach` past the guard's place (within the arm), the
        // shaft never turning. A swing still starts from a drag: one of
        // `mPointFlick` counts of net travel within 0.2 s, or pushed 300
        // counts past the cone's edge, winds up on the wheel from the point
        // guard. A press let go within 0.2 s without a drag is the click it
        // was.
        float mPointCone = 0.0f;
        float mPointCounts = 1800.0f;
        float mPointFlick = 500.0f;
        // The point guard: the weapon hand's place with the point straight
        // ahead (chest-centre frame; the spear lies on the support hand's
        // side, the body side-on), the chest's turn (radians; negative: the
        // weapon shoulder forward), and the rear hand's distance behind the
        // weapon hand (the shaft turns about the rear hand's place).
        JPH::Vec3 mPointHand = JPH::Vec3(-9.0f, 18.0f, -6.0f);
        float mPointTwist = -0.6f;
        float mPointDraw = 6.0f, mPointReach = 20.0f;
        float mPointGrip = 21.0f;
    };

    // Where the previewed cut meets the foe (chest-centre frame): the blade's
    // first touch of the foe's body, or its closest pass when it never does.
    struct WheelMark
    {
        JPH::Vec3 mPoint = JPH::Vec3::sZero();
        bool mHit = false;
        // The closest pass's distance from the body's surface: outside it
        // positive, inside it negative (how deep the cut goes in; a graze near 0).
        float mGap = 0.0f;
    };

    struct WheelInput
    {
        float mDt = 1.0f / 60.0f;
        bool mHeld = false; // the attack button (not during a clash lockout or a busy arm)
        float mDragRight = 0.0f, mDragUp = 0.0f; // mouse counts this step (sensitivity 1)
        bool mBlocking = false; // the guard key on the attack path
        // A shove's thrust (K2_SHOVE_ARMS; user 11-12: "make it so shoving
        // throws the arms forward"): the weapon hand goes straight out from
        // the shoulder the arm's length, the weapon upright before it, the
        // chest leaning in as for a thrust, for as long as this
        // is set; then the return to the guard, as after a block.
        bool mShove = false;
        bool mCrouched = false; // sneaking (FIST-05: straights go to the body)
        WheelGuardZone mGuardZone = WheelGuardZone::Middle;
        std::optional<JPH::Vec3> mThreat; // an incoming blade, chest-centre frame
        // That blade's striking part and its velocity, chest-centre frame
        // (K2_PARRY_STEER).
        std::optional<JPH::Vec3> mThreatStrike;
        JPH::Vec3 mThreatVelocity = JPH::Vec3::sZero();
        bool mNativeStarted = false; // Morrowind's attack began this step (a tap)
        bool mArmTrapped = false; // an NPC's weapon hand behind its back: a click waits
        JPH::Vec3 mHandVelocity = JPH::Vec3::sZero(); // measured, chest-centre frame (the momentum chains carry)
        float mArmLength = 55.0f;
        std::optional<JPH::Vec3> mShoulder; // the weapon arm's shoulder, chest-centre frame (K2_CUT_REACH)
        float mSinceParryWon = 1e9f; // seconds since this fighter won a parry
        bool mNpcRiposteReady = false; // an NPC's AI granted a riposte
        // Part 4.
        JPH::Quat mChestFromWorld = JPH::Quat::sIdentity(); // the chest-centre frame's rotation from world
        std::optional<JPH::Vec3> mHand; // the physical hand, chest-centre frame (the torso assist)
        std::optional<float> mTipGap; // the physical tip from last step's commanded tip (the cut stays firm)
        float mSwingSpeed = 0.0f; // the commanded tip's speed, world, last step (the follow turn)
        // The body's centre-of-mass velocity, world (K2_MOMENTUM_WIND).
        JPH::Vec3 mBodyVelocity = JPH::Vec3::sZero();
        // The weapon's physical point, chest-centre frame (K2_POINT_AIM: the
        // point guard trims its command until this lies on the aim's line).
        std::optional<JPH::Vec3> mPoint;
        // K2_FIST_STEER: the right fist held at this place (the guard stage
        // shows it), and the right hand's punch to throw now (from the guard
        // or the return).
        std::optional<FistSteerHold> mFistHold;
        std::optional<FistSteerPunch> mFistPunch;
    };

    // The trunk's share of the weapon (radians; positive lean bends back).
    struct WheelTrunk
    {
        float mTwist = 0.0f; // the chest's turn
        float mTwistLead = 0.0f; // the same a moment ahead (the hips lead)
        float mHipTurn = 0.0f; // the pelvis's turn over the legs (taken out of the two above)
        float mLean = 0.0f;
        float mRoll = 0.0f; // the side tilt (the sword shoulder up or down)
        float mThrustDrive = 0.0f; // the thrust's drive as the body takes it (-0.35..1)
        float mLow = 0.0f; // the low stance 0..1 (the knees bend with it)
        // The torso assist's own share (the guard learns the chest's place
        // only with it at rest).
        float mAssistTwist = 0.0f, mAssistLean = 0.0f, mAssistRoll = 0.0f;
    };

    struct WheelHooks
    {
        // The click's line (the intent stage: movement keys, Morrowind's
        // attack type, an NPC's turn, a foe off to one side), asked only when
        // a click fires.
        std::function<float()> mClickLine;
        // FIST-02: true when the left hand's jab takes this click.
        std::function<bool()> mLeftJab;
        // K2_FISTS: throw the left fist now (a drag to the left); true when it went.
        std::function<bool()> mLeftPunch;
        // FIST-05: the left hand throws this punch at this power; true when it went.
        std::function<bool(PunchKind, float)> mLeftFistPunch;
    };

    struct WheelEvents
    {
        bool mPress = false; // a new press (a new swing)
        bool mClick = false;
        float mClickAngle = 0.0f;
        bool mCutStarted = false; // a cut began: an attack (whoosh, fatigue)
        float mSwishSize = 1.0f; // 1 a click's, 0.8 a drag's
        float mFatigueShare = 1.0f; // of a committed attack's fatigue (less for a flowing chain)
        float mFlowGain = 0.0f; // the chain's flow, for the log
        bool mRiposte = false;
        // FIST-05: a punch thrown this step (either hand) and its power.
        bool mPunch = false;
        FistHand mPunchHand = FistHand::Right;
        PunchKind mPunchKind = PunchKind::Straight;
        float mPunchPower = 0.0f;
        bool mChained = false; // a press or a run-on chained, carrying momentum
        float mFeintFatigue = 0.0f; // a dragged-back cut's cost
        bool mNpcFeint = false; // an NPC feinted
    };

    class WheelPlanner
    {
    public:
        // Takes the arm from where the hand is (a draw, the end of a block or
        // a get-up): it eases to the guard through the return.
        void take(const ChamberPose& taken);

        // One physics step. The pose is in the chest-centre frame. `aim`
        // null: the keyed lines.
        const ChamberPose& step(const WheelInput& input, const WheelStriker& striker, const WheelActor& actor,
            const WheelHooks& hooks, const WheelAimInput* aim, WheelEvents& events);

        WheelStage stage() const { return mStage; }
        float angle() const { return mAngle; }
        float cutAngle() const { return mCutAngle; }
        float wind() const { return mWind; }
        float cut() const { return mCut; }
        const RouteFlow& flow() const { return mFlow; }
        // The route position as shown (evened out): 0..1 the wind-up, 1..2 the cut.
        float shownU() const { return mShownU; }
        bool autoAttack() const { return mAuto; }
        bool blocking() const { return mBlocking; }
        bool chaining() const { return mChainFrom.has_value(); }
        float sinceRelease() const { return mReleased; }
        // The line in play (the nearer of the two), -1 in the guard.
        int line() const { return mLine; }
        // The chest's twist taken ahead on the route (the hips lead the
        // chest, the chest the arm: HSC_BODY_LEAD 0.06 s).
        float twistLead() const { return mTwistLead; }
        // The cut's steering wish (radians: azimuth, elevation), for part 3.
        float steerWantX() const { return mSteerWantX; }
        float steerWantY() const { return mSteerWantY; }
        const std::optional<ChamberPose>& lastPose() const { return mLastPose; }
        float returnTime() const { return mReturnTime; }
        // The step's aim (steering applied), from the last wind-up or cut.
        const WheelAim& aim() const { return mAim; }
        int comboLeft() const { return mComboLeft; } // an NPC's cuts still to chain
        float steerX() const { return mSteerX; }
        float steerY() const { return mSteerY; }
        // The player's thrust drive this step (THRUST-02: the body's share).
        float thrustDriveWant() const { return mThrustDriveWant; }
        // The commanded tip along the whole cut, for the reticle (the
        // player's committed attack; chest-centre frame), else empty.
        const std::vector<JPH::Vec3>& preview() const { return mPreview; }
        // Where that cut meets the foe (K2_AIM_PREVIEW's mark), else none.
        const std::optional<WheelMark>& mark() const { return mMark; }
        // How far the commanded tip is out of the cut plane (a diagnostic).
        float cutPlaneMiss() const { return mCutPlaneMiss; }

        // A clash throws the hand target by `worldOffset` (units): out in
        // 0.07 s, back over 0.35-0.7 s by control. `endStroke`: the attacker's
        // stroke is spent and returns from the thrown pose; `chain` (CLASH-V2's
        // even bounce): it becomes a chain point instead, recovering in half
        // the time.
        void beginKnock(const JPH::Vec3& worldOffset, bool endStroke, bool chain, float control);
        bool knocking() const { return mKnockTime >= 0.0f; }

        // The guard opened: for `seconds` the block key does not raise the
        // block (an unparried clash threw the hand, play-6: the struck side is
        // open for a follow-up). Longer windows win over shorter ones.
        void openGuard(float seconds) { mGuardOpen = std::max(mGuardOpen, seconds); }
        float guardOpen() const { return mGuardOpen; }

        // The weapon drive's stiffness: 1.5 while the cut drives, easing to
        // 0.6 (two-handed 1.0) through the follow-through, firm again once it
        // is spent; softened while a clash throws the blade.
        float drivePhase() const { return mDrivePhase; }
        const WheelTrunk& trunk() const { return mTrunk; }
        // Contacts count as a swing: held, within 0.4 s of a release, or in a cut.
        bool swinging(bool held) const { return held || mReleased < 0.4f || mStage == WheelStage::Cut; }
        std::uint64_t swing() const { return mSwing; }
        // Diagnostics and the old path's published fields.
        float stanceX() const { return mStanceX; }
        float stanceY() const { return mStanceY; }
        bool snapCut() const { return mSnapCut; }
        bool playerRiposte() const { return mPlayerRiposte; }
        float chainFlowGain() const { return mChainFlowGain; }
        const std::optional<ChamberPose>& chainFrom() const { return mChainFrom; }
        // K2_POINT_AIM: the point guard is held; the attack under way is its
        // thrust; the point's offset from the aim (radians: right, up).
        bool pointing() const { return mPointing; }
        bool pointThrusting() const { return mPointThrust; }
        float pointX() const { return mPointX; }
        float pointY() const { return mPointY; }
        float pointTrimX() const { return mPointTrimX; }
        float pointTrimY() const { return mPointTrimY; }

    private:
        void beginCut(float sx, float sy, bool snap, float progress, const WheelStriker& striker);
        void startChain(float momentumShare, float dt);

        WheelStage mStage = WheelStage::Guard;
        float mAngle = 0.0f, mCutAngle = 0.0f;
        float mWind = 0.0f, mCut = 0.0f, mRecover = 0.0f;
        RouteFlow mFlow;
        float mShownU = 0.0f;
        bool mAuto = false;
        bool mPathWasHeld = false;
        float mReleased = 1.0f;
        int mLine = -1;
        std::uint64_t mSwing = 0;

        // The stance circle and the stroke.
        float mStanceX = 0.0f, mStanceY = 0.0f;
        float mDragRight = 0.0f, mDragUp = 0.0f; // since the press (a click is undragged)
        float mStrokeVX = 0.0f, mStrokeVY = 0.0f; // filtered mouse velocity, counts/s
        float mCutStrokeX = 0.0f, mCutStrokeY = 0.0f; // the cut stroke's direction
        float mCutAccX = 0.0f, mCutAccY = 0.0f;
        float mCutTravel = 0.0f, mInwardPending = 0.0f;
        bool mSnapCut = false;
        float mCutOverflow = 0.0f;
        float mLastCutAlong = 0.0f;
        float mCutFromU = 0.0f;
        std::optional<ChamberPose> mCutFrom;
        float mSteerWantX = 0.0f, mSteerWantY = 0.0f;

        // Chains.
        std::optional<ChamberPose> mChainFrom;
        float mChainTime = 0.0f;
        JPH::Vec3 mChainVelocity = JPH::Vec3::sZero(), mChainSpin = JPH::Vec3::sZero();
        float mChainPrevAngle = 0.0f, mChainTiming = 0.0f;
        bool mChainFlowPending = false;
        float mChainFlowGain = 0.0f;
        std::optional<JPH::Quat> mChainWayTarget;
        bool mWaitRecover = false;
        bool mClickQueued = false;
        bool mPlayerRiposte = false;
        bool mRiposteAttack = false;

        // The return and the block.
        std::optional<ChamberPose> mRecoverFrom, mLastPose, mPrevPose;
        float mReturnTime = 0.4f;
        bool mBlocking = false;
        ChamberPose mBlockFrom;
        float mBlockTime = 0.0f;
        WheelGuardZone mBlockZone = WheelGuardZone::Middle;
        JPH::Vec3 mBlockSteer = JPH::Vec3::sZero(); // K2_PARRY_STEER: the hand's eased offset
        float mSinceBlock = 1e9f;

        // NPCs.
        float mNpcChamberHeld = 0.0f;
        std::uint64_t mFeintDecided = ~std::uint64_t(0);
        bool mFeinting = false, mFeintedLast = false;

        float mTwistLead = 0.0f;
        ChamberPose mPose;

        // The aim.
        WheelAim mAim;
        float mSteerX = 0.0f, mSteerY = 0.0f;
        float mThrustDriveWant = 0.0f;
        std::vector<JPH::Vec3> mPreview;
        std::optional<WheelMark> mMark;
        float mCutPlaneMiss = 0.0f;
        std::optional<JPH::Vec3> mFoe; // from the aim input
        bool mFoeDowned = false;

        // Part 4.
        void stepDrivePhase(const WheelInput& input, const WheelStriker& striker, float dt);
        void stepKnock(const WheelInput& input, const WheelStriker& striker, float dt);
        void stepTrunk(const WheelInput& input, const WheelStriker& striker, const WheelActor& actor, float dt);
        float mDrivePhase = 1.0f;
        float mFollowHeld = 0.0f;
        JPH::Vec3 mKnock = JPH::Vec3::sZero(); // world
        float mKnockTime = -1.0f, mKnockRecovery = 0.0f;
        bool mKnockAbort = false, mKnockChain = false;
        WheelTrunk mTrunk;
        float mFollowTurnPrevCut = 0.0f, mFollowTurnAmplitude = 0.0f, mFollowTurnTime = 0.0f;
        float mSideShare = 0.0f; // the sideways turn's share in play (eased)
        std::optional<float> mTwistCut; // the chest's turn taken ahead in a cut (K2_CUT_LEAD)
        float mHipCap = 0.45f; // the hips' turn cap in play (K2_CUT_HIP_CAP, eased)
        bool mCutDone = false; // this cut reached its end (B14)
        int mComboLeft = 0; // an NPC's cuts still to chain in this attack
        bool mComboClick = false; // the queued click is the combo's
        float mHeldDone = 0.0f; // held since then
        float mAssistTwist = 0.0f, mAssistLean = 0.0f, mAssistRoll = 0.0f;
        float mFoeTwist = 0.0f, mFoeLean = 0.0f;
        float mGuardOpen = 0.0f;
        float mMomentumHaste = 1.0f; // K2_MOMENTUM_WIND, this step
        float mFlickX = 0.0f, mFlickY = 0.0f; // K2_FLICK_CHAIN: drag since the cut finished
        bool mLeftArmed = true; // K2_FISTS: a left drag may throw the left fist
        float mLeftRearm = 0.0f; // counts dragged back right since it was thrown
        bool mLeftThrown = false; // this press threw the left fist (its release is no click)
        // FIST-05: the press's flick, the next straight's hand, the right punch's power.
        float mFightRight = 0.0f, mFightUp = 0.0f;
        std::optional<FlickPunch> mFightPending; // thrown while its hand was busy
        float mFightPendingAge = 0.0f;
        bool mNextStraightLeft = true;
        float mSinceFightPunch = 1e9f;
        float mPunchPower = 0.0f;
        bool mPunchWound = false; // K2_FIST_STEER: the punch was thrown from a held fist
        PunchKind mPunchKind = PunchKind::Straight; // ... and its kind (K2_FIST_WOUND)
        bool mFistHeld = false; // ... the last step showed a held fist
        std::optional<ChamberPose> mFistHoldFrom; // ... taken from here over 0.12 s
        float mFistHoldTime = 0.0f;
        float mThrustKeep = 0.0f; // K2_THRUST_KEEP, this step's actor's
        // K2_POINT_AIM.
        bool mPointing = false, mPointThrust = false;
        float mPointX = 0.0f, mPointY = 0.0f;
        // The command's trim (radians): the held weapon settles off its
        // command (the support arm's pull; 8 u right and 3 low of it at 70,
        // iron spear 11-12), so the command is moved until the real point
        // is where it was asked. Kept between presses.
        float mPointTrimX = 0.0f, mPointTrimY = 0.0f;
        // The same for the thrust driven out (learned at its end's hold).
        float mPointTrimOutX = 0.0f, mPointTrimOutY = 0.0f;
        bool mPointTrimOutKnown = false;
        float mPointOutTime = 0.0f; // the thrust held at its end
        float mPointTime = 0.0f; // since the point guard or its thrust began
        float mPointShownX = 0.0f, mPointShownY = 0.0f; // the offset the pose follows (eased)
        std::array<float, 12> mPointRecentX{}, mPointRecentY{}; // the last 0.2 s of drag
        std::size_t mPointRecentAt = 0;
        float mPointOverX = 0.0f, mPointOverY = 0.0f; // travel past the cone's edge
        std::optional<ChamberPose> mPointFrom;
    };
}

#endif

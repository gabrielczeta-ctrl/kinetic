#include <ragdollcore/wheelplanner.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <numbers>

namespace RagdollCore
{
    namespace
    {
        constexpr float sPi = std::numbers::pi_v<float>;
        constexpr float sTau = 2.0f * sPi;
        constexpr float sEighth = sPi / 4.0f;
        constexpr float sThrustAngle = 1.5f * sPi;

        // The game's values (play-2026-11-03-7), named; the rules that use
        // them are in the design's inventory.
        constexpr float sPickCounts = 40.0f; // a drag this long is not a click (HSC_PICK_COUNTS)
        constexpr float sMaxRadius = sStanceRadius + sCommitCounts;
        constexpr float sRunOnCounts = 2500.0f; // a stroke runs on into the next wind-up (HSC_RUN_ON_COUNTS)
        constexpr float sCutSpeed = 2500.0f; // counts/s inward: the cut (HSC_CUT_SPEED)
        constexpr float sCutTravel = 150.0f; // counts inward first (HSC_CUT_TRAVEL)
        constexpr float sSnapSpeed = 3500.0f; // a flick inside the circle (HSC_SNAP_SPEED)
        constexpr float sSnapStart = 0.2f; // along its cut route
        constexpr float sSnapRate = 1.6f;
        constexpr float sInwardHold = 120.0f; // counts held while undecided
        constexpr float sSteerCounts = 1500.0f; // counts per radian (HSC_STEER)
        constexpr float sSteerMost = 0.5f; // radians
        constexpr float sChainDrift = 0.12f; // s (HSC_CHAIN_DRIFT)
        constexpr float sChainTurn = 0.2f; // s per half turn (HSC_CHAIN_TURN)
        constexpr float sChainSpeedCap = 200.0f; // u/s carried at most
        constexpr float sChainSpinCap = 30.0f; // rad/s carried at most
        constexpr float sChainSpinMost = 12.0f; // rad/s the carried pose turns at most
        constexpr float sWaitReturn = 0.15f; // a waiting press starts here in the return (HSC_WAIT_RETURN)
        constexpr float sFromBlock = 0.3f; // s after a block a press is the guard-to-attack press
        constexpr float sFeintFatigue = 4.0f; // FEINT[0]
        constexpr float sNpcFeintChance = 0.25f; // FEINT[1], times skill/100
        constexpr float sRiposteWindow = 0.6f; // HSC_RIPOSTE[0]
        constexpr float sChainFlowFatigue = 0.5f; // HSC_CHAIN_FLOW[1]
        constexpr float sBlockTime = 0.12f;
        // K2_PARRY_STEER: where on the blade it meets the blow (a share of
        // hand to tip), the stretch either side the blade covers anyway, how
        // far ahead the blow is read (s) and from what closing speed (u/s).
        constexpr float sSteerMeet = 0.4f;
        constexpr float sSteerCover = 0.25f;
        constexpr float sSteerLead = 0.25f;
        constexpr float sSteerClosing = 30.0f;
        constexpr float sShoveReach = 0.97f; // K2_SHOVE_ARMS: the thrust, a share of the arm
        constexpr float sShoveDrive = 0.8f; // ... and the trunk's lean into it, a share of a thrust's (0.22 rad)
        constexpr float sBodyLead = 0.06f; // s (HSC_BODY_LEAD)
        // K2_POINT_AIM: a press shorter than this without a drag is a click;
        // a fast drag this long, or one this far past the cone, is a swing's;
        // the point guard is taken over this long, its thrust over that.
        constexpr float sPointTap = 0.2f;
        constexpr float sPointOver = 300.0f;
        constexpr float sPointEase = 0.25f;
        constexpr float sPointThrustEase = 0.15f;
        constexpr float sPointDrive = 0.45f; // the share of the cut the drive takes

        float wrap(float a)
        {
            a = std::fmod(a, sTau);
            return a < 0.0f ? a + sTau : a;
        }

        float signedDiff(float a, float b)
        {
            float d = std::fmod(a - b + sPi, sTau);
            return (d < 0.0f ? d + sTau : d) - sPi;
        }

        float smoothStep(float x)
        {
            x = std::clamp(x, 0.0f, 1.0f);
            return x * x * (3.0f - 2.0f * x);
        }

        // The spin between two rotations over dt (axis x rad/s).
        JPH::Vec3 spinBetween(const JPH::Quat& from, const JPH::Quat& to, float dt, float share)
        {
            JPH::Quat delta = (to * from.Conjugated()).Normalized();
            if (delta.GetW() < 0.0f)
                delta = -delta;
            JPH::Vec3 axis;
            float angle = 0.0f;
            delta.GetAxisAngle(axis, angle);
            if (!(angle > 1e-5f) || !(dt > 0.0f))
                return JPH::Vec3::sZero();
            return axis * (std::min(angle / dt, sChainSpinCap) * share);
        }

        // Slerp that keeps the way it started (no side switch mid-blend).
        JPH::Quat slerpWay(JPH::QuatArg from, JPH::QuatArg to, float t)
        {
            const float d = std::clamp(from.Dot(to), -1.0f, 1.0f);
            const float angle = std::acos(d);
            const float sine = std::sin(angle);
            if (sine < 1e-4f)
                return (from * (1.0f - t) + to * t).Normalized();
            return ((from * (std::sin((1.0f - t) * angle) / sine)) + (to * (std::sin(t * angle) / sine))).Normalized();
        }

        std::uint64_t mixHash(std::uint64_t serial, std::uint64_t swing)
        {
            return (serial * 0x9E3779B97F4A7C15ull) ^ (swing * 0xBF58476D1CE4E5B9ull);
        }

        struct BlockKey
        {
            JPH::Vec3 mHand, mBlade, mPole;
            float mTwistDegrees;
        };

        // The zone an incoming blade (chest-centre frame) comes from
        // (K2_GUARD_THREAT): above the head high, below the belt low, wide of
        // a shoulder that side, else the middle.
        WheelGuardZone threatZone(const JPH::Vec3& threat)
        {
            if (threat.GetZ() > 14.0f)
                return WheelGuardZone::High;
            if (threat.GetZ() < -16.0f)
                return WheelGuardZone::Low;
            if (threat.GetX() < -14.0f)
                return WheelGuardZone::Left;
            if (threat.GetX() > 14.0f)
                return WheelGuardZone::Right;
            return WheelGuardZone::Middle;
        }

        BlockKey blockKey(WheelGuardZone zone)
        {
            switch (zone)
            {
                case WheelGuardZone::High:
                    return { { 6.0f, 20.0f, 22.0f }, { -0.94f, 0.25f, 0.24f }, { 0.4f, 0.1f, -0.9f }, 0.0f };
                case WheelGuardZone::Left:
                    return { { -8.0f, 24.0f, 4.0f }, { -0.15f, 0.3f, 0.94f }, { 0.2f, 0.1f, -0.97f }, -10.0f };
                case WheelGuardZone::Right:
                    return { { 16.0f, 22.0f, 2.0f }, { 0.2f, 0.3f, 0.93f }, { 0.85f, 0.1f, -0.5f }, -10.0f };
                case WheelGuardZone::Low:
                    return { { 6.0f, 26.0f, -14.0f }, { -0.3f, 0.4f, -0.86f }, { 0.5f, 0.1f, -0.85f }, -10.0f };
                case WheelGuardZone::Shove:
                    // The weapon upright, leaning back over the hand: its
                    // flat goes first, not its point.
                    return { { 6.0f, 45.0f, 6.0f }, { -0.3f, -0.15f, 0.94f }, { 0.6f, 0.1f, -0.8f }, 0.0f };
                case WheelGuardZone::Middle:
                    break;
            }
            // The high block across the body (the old tip block: azimuth -20,
            // elevation 55).
            return { { 4.0f, 24.0f, 6.0f }, { -0.196f, 0.539f, 0.819f }, { 0.5f, 0.1f, -0.85f }, -10.0f };
        }

        // K2_POINT_AIM: the point guard's pose - the shaft from the rear
        // hand's place (`mPointGrip` behind the weapon hand's, level) to the
        // aim's point (on the aim's line from the chest centre, at the aim's
        // distance, never nearer than 25 past the spear's point), the weapon
        // hand on it; `along`: the hand moved along the shaft (within 0.95 of
        // the arm), the shaft as it was - the thrust runs straight down its
        // own shaft to the aim's point (user 11-12: "the spear should go
        // straight forward and it goes up forward and diagonally": the hand
        // ran along the aim's line, 10 deg off the shaft, came 9 u back
        // across the body and the trim faded through the drive; the user's
        // three point thrusts ended 11-34 u to the side).
        // The rear hand is the pivot because it is the boxed one: beside the
        // hip it has no room to swing, the front hand has (turned about the
        // front hand, the point stopped at the middle going left: commanded
        // -13.5 u, 0.9 real, 11-12).
        ChamberPose pointPose(const WheelStriker& striker, const WheelActor& actor, const WheelInput& input,
            const WheelAim& aim, float along, float twist, const ChamberPose& guard)
        {
            ChamberPose pose;
            pose.mPole = JPH::Vec3(0.3f, 0.1f, -0.95f).Normalized();
            pose.mTwist = twist;
            const float grip = std::max(actor.mPointGrip, 0.0f);
            const JPH::Vec3 rear = actor.mPointHand - JPH::Vec3(0.0f, grip, 0.0f);
            const JPH::Vec3 direction = aim.mDirection.NormalizedOr(JPH::Vec3::sAxisY());
            const float length = grip + std::max(striker.mTip, 1.0f);
            const float depth = std::max(aim.mDistance, rear.Dot(direction) + length + 25.0f);
            const JPH::Vec3 shaft = (direction * depth - rear).NormalizedOr(direction);
            const JPH::Vec3 base = rear + shaft * grip;
            if (input.mShoulder && along > 0.0f)
            {
                // |base + shaft x s - shoulder| = limit, the far root.
                const JPH::Vec3 from = base - *input.mShoulder;
                const float limit = 0.95f * input.mArmLength;
                const float b = from.Dot(shaft);
                const float c = from.LengthSq() - limit * limit;
                const float most = c < 0.0f ? -b + std::sqrt(b * b - c) : 0.0f;
                if (std::isfinite(most))
                    along = std::min(along, std::max(most, 0.0f));
            }
            pose.mHand = base + shaft * along;
            const JPH::Vec3 down(0.0f, 0.0f, -1.0f);
            const auto rotation = alignedStrikeHandRotation(
                striker.mAxes, shaft, (down - shaft * shaft.Dot(down)).NormalizedOr(JPH::Vec3::sAxisX()));
            pose.mRotation = rotation ? rotation->Normalized() : guard.mRotation;
            return pose;
        }
    }

    void WheelPlanner::take(const ChamberPose& taken)
    {
        mStage = WheelStage::Recover;
        mRecover = 0.0f;
        mRecoverFrom = taken;
        mLastPose = taken;
        mPrevPose.reset();
        mChainFrom.reset();
        mAuto = false;
        mReleased = 1.0f;
        mFlow = {};
        mPose = taken;
    }

    void WheelPlanner::beginCut(float sx, float sy, bool snap, float progress, const WheelStriker& striker)
    {
        const float l = std::max(1e-3f, std::hypot(sx, sy));
        mCutStrokeX = sx / l;
        mCutStrokeY = sy / l;
        mCutAccX = mCutAccY = 0.0f;
        mSteerWantX = mSteerWantY = 0.0f;
        mCutOverflow = 0.0f;
        mSnapCut = snap;
        mCutFrom = mLastPose;
        // The wind-up's line, before the cut takes its own.
        const float windAngle = mAngle;
        // The line whose wind-up is opposite the stroke.
        mCutAngle = wrap(std::atan2(-mCutStrokeY, -mCutStrokeX));
        if (striker.mSpearThrusts)
            mCutAngle = sThrustAngle;
        // K2_THRUST_KEEP: a long haft wound up on the thrust line thrusts for
        // any stroke roughly ahead.
        else if (striker.mSpearSwings && mThrustKeep > 0.0f
            && std::abs(signedDiff(windAngle, sThrustAngle)) < sEighth * 0.5f
            && std::abs(signedDiff(mCutAngle, sThrustAngle)) < mThrustKeep)
            mCutAngle = sThrustAngle;
        mAngle = mCutAngle;
        mStage = WheelStage::Cut;
        mCut = std::clamp((snap ? sSnapStart : 0.0f) + progress, 0.0f, 1.0f);
        // The shown route starts at the cut's start and catches up with the
        // mouse (FLOW-01: set to the mouse's place it jumped).
        mCutFromU = snap ? sSnapStart : 0.0f;
        mFlow.mU = 1.0f + mCutFromU;
        mFlow.mV = 0.0f;
        mStanceX = mStanceY = 0.0f;
        mChainFrom.reset();
    }

    void WheelPlanner::startChain(float momentumShare, float dt)
    {
        mChainFrom = mLastPose;
        mChainTime = 0.0f;
        mChainVelocity = mChainSpin = JPH::Vec3::sZero();
        if (mPrevPose && mLastPose && dt > 0.0f)
            mChainSpin = spinBetween(mPrevPose->mRotation, mLastPose->mRotation, dt, momentumShare);
        mChainPrevAngle = mCutAngle;
        mChainFlowPending = true;
        mChainFlowGain = 0.0f;
    }

    const ChamberPose& WheelPlanner::step(const WheelInput& input, const WheelStriker& striker,
        const WheelActor& actor, const WheelHooks& hooks, const WheelAimInput* aimInput, WheelEvents& events)
    {
        events = {};
        const float dt = std::max(0.0f, input.mDt);
        const bool held = input.mHeld;
        const bool npc = !actor.mPlayer;
        const WheelLines& lines = *striker.mLines;
        const bool fists = striker.mLines == &fistLines();
        mThrustKeep = actor.mPlayer ? std::max(0.0f, actor.mThrustKeep) : 0.0f;
        // K2_DAGGER_THRUST: the player's dagger on the thrust line.
        const auto daggerThrustNow = [&] {
            return actor.mDaggerThrust && actor.mPlayer && striker.mShort && !fists
                && std::abs(signedDiff(mStage == WheelStage::Cut ? mCutAngle : mAngle, sThrustAngle)) < sEighth * 0.5f;
        };
        // K2_SPEAR_THRUST_PACE: the player's spear or staff on the thrust line.
        const auto spearThrustNow = [&] {
            return actor.mSpearThrustWind > 0.0f && actor.mPlayer && !striker.mShort && !fists
                && (striker.mSpearSwings || striker.mSpearThrusts || striker.mPointGuard)
                && (striker.mSpearThrusts
                    || std::abs(signedDiff(mStage == WheelStage::Cut ? mCutAngle : mAngle, sThrustAngle)) < sEighth * 0.5f);
        };
        const auto quickThrustNow = [&] { return daggerThrustNow() || spearThrustNow(); };
        // K2_MOMENTUM_WIND: the body's speed along the coming cut (chest frame:
        // x right, y forward, z up; the cut runs from the wind-up's side
        // through the centre, a thrust forward) speeds the wind-up.
        mMomentumHaste = 1.0f;
        if (actor.mMomentumWind > 0.0f && (mStage == WheelStage::Wind || mStage == WheelStage::Guard))
        {
            const JPH::Vec3 v = input.mChestFromWorld * input.mBodyVelocity;
            const bool thrustLine = std::abs(signedDiff(mAngle, sThrustAngle)) < sEighth * 0.5f;
            const float along = thrustLine
                ? v.GetY()
                : -(v.GetX() * std::cos(mAngle) + v.GetZ() * std::sin(mAngle)) + 0.5f * std::max(0.0f, v.GetY());
            if (std::isfinite(along))
                mMomentumHaste = 1.0f + actor.mMomentumWind * std::clamp(along / 300.0f, 0.0f, 1.0f);
        }
        const auto pacing = [&] {
            PaceInput p;
            p.mWeight = striker.mWeight;
            p.mTwoHanded = striker.mTwoHanded;
            p.mShort = striker.mShort;
            p.mPlayer = actor.mPlayer;
            p.mControl = actor.mControl;
            p.mResponse = actor.mResponse;
            p.mPlayerRiposte = mPlayerRiposte;
            p.mNpcRiposte = mRiposteAttack;
            p.mChainFlowGain = mChainFlowGain;
            p.mMomentumHaste = mMomentumHaste;
            p.mDaggerThrust = daggerThrustNow();
            if (spearThrustNow())
            {
                p.mSpearThrustWind = actor.mSpearThrustWind;
                p.mSpearThrustCut = actor.mSpearThrustCut;
                p.mSpearThrustReturn = actor.mSpearThrustReturn;
            }
            Pace result = attackPace(p);
            // FIST-05: a right-hand haymaker winds up longer and strikes quicker.
            // FIST-06: a fist thrown from its held place is wound up already -
            // half a quick punch's wind-up to its chamber, whatever its power.
            if ((mPunchPower > 0.0f || mPunchWound) && fists)
            {
                const PunchTiming quick = punchTiming(PunchKind::Straight, 0.0f, 1.0f);
                const PunchTiming hit = punchTiming(PunchKind::Straight, mPunchPower, 1.0f);
                result.mWind *= mPunchWound ? woundWindShare(mPunchKind, actor.mFistWoundWind) : hit.mWind / quick.mWind;
                result.mCut *= hit.mCut / quick.mCut;
            }
            // K2_FIST_TRUE: a punch snaps back.
            if (fists && actor.mPlayer && actor.mFistSnapReturn > 0.0f)
            {
                result.mReleaseHold = std::min(result.mReleaseHold, std::max(0.0f, actor.mFistSnapHold));
                result.mReturn = std::min(result.mReturn, std::max(0.05f, actor.mFistSnapReturn));
                result.mCut *= std::clamp(actor.mFistSnapCut, 0.3f, 1.0f);
            }
            // K2_FIST_WOUND: a wound hook or uppercut strikes no quicker than this.
            if (fists && actor.mPlayer && mPunchWound)
                result.mCut = woundStrikeSeconds(result.mCut, mPunchKind, actor.mFistWoundCut);
            return result;
        };

        mReleased = held ? 0.0f : mReleased + dt;
        mThrustDriveWant = 0.0f;
        mPreview.clear();
        mMark.reset();
        mCutPlaneMiss = 0.0f;
        mFoe = aimInput ? aimInput->mFoe : std::nullopt;
        mFoeDowned = aimInput && aimInput->mFoeDowned;
        const WheelStage stageBefore = mStage;
        const bool newPress = held && !mPathWasHeld;
        const bool newRelease = !held && mPathWasHeld;
        mPathWasHeld = held;
        if (newPress)
        {
            events.mPress = true;
            ++mSwing;
        }
        const bool autoBeforePress = mAuto;
        if (newPress)
            mAuto = false;
        const bool undragged = std::hypot(mDragRight, mDragUp) < sPickCounts;
        // K2_WINDUP_RADIUS: the player's stance circle (aiming with the click
        // held), every weapon; the commit distance past it stays.
        const float stanceRadius = actor.mPlayer && actor.mStanceRadius > 0.0f ? actor.mStanceRadius : sStanceRadius;
        const float maxRadius = stanceRadius + sCommitCounts;
        // An opened guard does not raise the block until it closes.
        mGuardOpen = std::max(0.0f, mGuardOpen - dt);
        const bool blocking = input.mShove || (input.mBlocking && mGuardOpen <= 0.0f);
        // K2_POINT_AIM: the point guard ends with the button; held a moment
        // or dragged, its release is the thrust (else the click below).
        const bool pointGuard = actor.mPlayer && striker.mPointGuard && actor.mPointCone > 0.0f;
        bool pointThrustNow = false;
        if (mPointing && (!pointGuard || blocking || mStage != WheelStage::Guard))
            mPointing = false;
        else if (mPointing && !held)
        {
            mPointing = false;
            pointThrustNow = mPointTime >= sPointTap || !undragged;
        }

        // FIST-05: Fight Night's punch control. The press's drag is a flick:
        // its release throws the punch it picks, either hand; a press without
        // one throws the next straight (the jab first, then the cross).
        // FIST-06 takes FIST-05's place: the punches come from the steered fists.
        const bool fistSteer = fists && actor.mFistSteer && actor.mPlayer;
        const bool fightNight = fists && actor.mFightNight && actor.mPlayer && !fistSteer;
        std::optional<FlickPunch> fightPunch;
        mSinceFightPunch += dt;
        if (fightNight)
        {
            if (newPress)
                mFightRight = mFightUp = 0.0f;
            if (held)
            {
                mFightRight += input.mDragRight;
                mFightUp += input.mDragUp;
            }
            const bool ready = !blocking && !input.mArmTrapped;
            if ((newRelease || (input.mNativeStarted && !held)) && ready)
            {
                if (mSinceFightPunch > 1.0f)
                    mNextStraightLeft = true;
                const FistHand next = mNextStraightLeft ? FistHand::Left : FistHand::Right;
                fightPunch = classifyFlick(mFightRight, mFightUp, input.mCrouched, next, { sPickCounts, 900.0f });
                if (!fightPunch)
                    fightPunch = FlickPunch{ next, input.mCrouched ? PunchKind::Body : PunchKind::Straight, 0.0f };
            }
            if (!held)
                mFightRight = mFightUp = 0.0f;
            // A punch thrown while its hand was busy waits for it (Fight
            // Night queues a combo), up to 0.6 s; a new one replaces it.
            if (fightPunch)
            {
                mFightPending = fightPunch;
                mFightPendingAge = 0.0f;
            }
            else if (mFightPending && (mFightPendingAge += dt) > 0.6f)
                mFightPending.reset();
            fightPunch = mFightPending;
        }
        else
            mFightPending.reset();
        if (fightPunch && fightPunch->mHand == FistHand::Left)
        {
            if (hooks.mLeftFistPunch && hooks.mLeftFistPunch(fightPunch->mKind, fightPunch->mPower))
            {
                events.mPunch = true;
                events.mPunchHand = FistHand::Left;
                events.mPunchKind = fightPunch->mKind;
                events.mPunchPower = fightPunch->mPower;
                mNextStraightLeft = false;
                mSinceFightPunch = 0.0f;
                mFightPending.reset();
            }
            fightPunch.reset();
        }
        // A right-hand punch goes on the wheel as a click on its line.
        const bool fightRight = fightPunch && fightPunch->mHand == FistHand::Right
            && (mStage == WheelStage::Guard || mStage == WheelStage::Recover);
        if (fightRight)
            mFightPending.reset();
        // FIST-06: the steered right fist's punch, a click on its line.
        const bool steerRight = fistSteer && input.mFistPunch && input.mFistPunch->mHand == FistHand::Right
            && !blocking && !input.mArmTrapped && (mStage == WheelStage::Guard || mStage == WheelStage::Recover);

        // The click: a press released before its drag picked a line, a tap
        // that started Morrowind's attack, or a queued click; from the guard
        // or a kept stance only.
        const bool keptStance = !mAuto && mStage == WheelStage::Wind && mWind <= 0.5f;
        bool clickNow = !blocking && (mStage == WheelStage::Guard || keptStance) && !mAuto
            && ((newRelease && undragged) || (input.mNativeStarted && !held && undragged) || (mClickQueued && !held));
        if (fightNight)
            clickNow = fightRight;
        if (fistSteer)
            clickNow = steerRight;
        if (pointThrustNow)
            clickNow = true;
        // K2_FISTS: the press that threw the left fist is spent - its release
        // (or the native attack it started) is no click.
        if (newPress)
            mLeftThrown = false;
        if (clickNow && mLeftThrown)
        {
            clickNow = false;
            mClickQueued = false;
            if (!held)
                mLeftThrown = false;
        }
        if (clickNow && input.mArmTrapped)
            mClickQueued = true;
        else if (clickNow && !fightNight && !fistSteer && hooks.mLeftJab && hooks.mLeftJab())
            mClickQueued = false;
        else if (clickNow)
        {
            mClickQueued = false;
            float angle = pointThrustNow ? sThrustAngle : hooks.mClickLine ? wrap(hooks.mClickLine()) : 0.5f * sPi;
            mPunchPower = 0.0f;
            mPunchWound = false;
            mPunchKind = PunchKind::Straight;
            if (steerRight)
            {
                const FistSteerPunch& punch = *input.mFistPunch;
                angle = lines[std::min(punch.mLine, lines.size() - 1)].angle();
                mPunchPower = punch.mPower;
                mPunchWound = punch.mWound;
                mPunchKind = punch.mKind;
                events.mPunch = true;
                events.mPunchHand = FistHand::Right;
                events.mPunchKind = punch.mKind;
                events.mPunchPower = punch.mPower;
            }
            if (fightRight)
            {
                if (const AttackLine* line = fistLine(fightPunch->mKind))
                    angle = line->angle();
                mPunchPower = fightPunch->mPower;
                events.mPunch = true;
                events.mPunchHand = FistHand::Right;
                events.mPunchKind = fightPunch->mKind;
                events.mPunchPower = fightPunch->mPower;
                mNextStraightLeft = true;
                mSinceFightPunch = 0.0f;
            }
            // (FIST-06: a wound fist's punch starts where the fist was held.)
            if ((keptStance || (steerRight && mPunchWound)) && mLastPose)
            {
                mChainFrom = mLastPose;
                mChainTime = 0.0f;
                mFlow = {};
            }
            mStanceX = mStanceY = 0.0f;
            mAngle = angle;
            mStage = WheelStage::Wind;
            mWind = mCut = 0.0f;
            mDragRight = mDragUp = 0.0f;
            mAuto = true;
            // K2_POINT_AIM: a long haft's thrust goes along its shaft, from
            // the point guard where the mouse left the point (a click's: at
            // the aim).
            mPointThrust = pointGuard && std::abs(signedDiff(angle, sThrustAngle)) < sEighth * 0.5f;
            if (mPointThrust)
            {
                if (!pointThrustNow)
                    mPointX = mPointY = 0.0f;
                mPointFrom = mLastPose;
                mPointTime = 0.0f;
                mChainFrom.reset();
            }
            mPlayerRiposte = actor.mPlayer && input.mSinceParryWon < sRiposteWindow;
            if (!mChainFlowPending)
                mChainFlowGain = 0.0f;
            events.mClick = true;
            events.mClickAngle = angle;
            // An NPC's attack pattern (K2_NPC_COMBO, play-6 bookmark "enemies
            // should have more than one attack pattern"): drawn once per
            // attack, seeded, by skill - a single cut, two or three.
            if (npc && !mComboClick)
            {
                const float skill = std::clamp(actor.mSkill / 100.0f, 0.0f, 1.0f);
                const auto roll = [&](std::uint64_t salt) {
                    return static_cast<float>((mixHash(actor.mSerial ^ salt, mSwing) >> 11) % 10000) / 10000.0f;
                };
                mComboLeft = 0;
                if (roll(0x5ec0dull) < actor.mComboSecond * skill)
                    mComboLeft = roll(0x7a1edull) < actor.mComboThird * skill ? 2 : 1;
            }
            mComboClick = false;
        }

        // A click plays on its own at the weapon's pace.
        if (mStage == WheelStage::Guard)
            mRiposteAttack = npc && input.mNpcRiposteReady;
        else if (mStage == WheelStage::Recover)
            mRiposteAttack = false;
        Pace pace = pacing();
        if (mAuto && dt > 0.0f)
        {
            if (mStage == WheelStage::Wind)
            {
                mWind = std::min(1.0f, mWind + dt / std::max(1e-3f, pace.mWind));
                const bool chambered = mWind >= 1.0f && mFlow.mU >= 0.97f;
                mNpcChamberHeld = chambered ? mNpcChamberHeld + dt : 0.0f;
                if (chambered && (!npc || mRiposteAttack || mNpcChamberHeld >= pace.mChamberHold))
                {
                    mStage = WheelStage::Cut;
                    mCutAngle = mAngle;
                    mCut = 0.0f;
                }
            }
            else if (mStage == WheelStage::Cut)
            {
                mCut = std::min(1.0f, mCut + dt / std::max(1e-3f, pace.mCut));
                // An NPC's feint: drawn once per swing by skill, never twice
                // running; a third into the cut it pulls back into a chamber a
                // quarter turn round.
                if (npc)
                {
                    if (mFeintDecided != mSwing)
                    {
                        mFeintDecided = mSwing;
                        const float roll
                            = static_cast<float>((mixHash(actor.mSerial, mSwing) >> 11) % 10000) / 10000.0f;
                        mFeinting = !mFeintedLast
                            && roll < sNpcFeintChance * std::clamp(actor.mSkill / 100.0f, 0.0f, 1.0f);
                        mFeintedLast = mFeinting;
                    }
                    if (mFeinting && mCut >= 0.33f)
                    {
                        mFeinting = false;
                        events.mNpcFeint = true;
                        mStage = WheelStage::Wind;
                        mWind = 1.0f;
                        mCut = 0.0f;
                        mAngle = std::fmod(mAngle + ((mSwing & 1) ? 0.5f : 1.5f) * sPi, sTau);
                        mNpcChamberHeld = 0.0f;
                        mCutFrom.reset();
                    }
                }
            }
            else
                mAuto = false;
            // The follow-through holds, then the release rule returns it.
            mReleased = std::min(mReleased, mCut >= 1.0f ? mReleased : 0.0f);
        }

        // Chains by skill: from 50 a press carries the momentum (all of it at
        // 100); below, it waits for the return.
        const float momentumShare = std::clamp((actor.mSkill - 50.0f) / 50.0f, 0.0f, 1.0f);
        const bool stanceHeld = mStage == WheelStage::Wind && !mAuto && mWind <= 0.5f;
        mSinceBlock += dt;
        const bool fromBlock = actor.mPlayer && mStage == WheelStage::Recover && mSinceBlock < sFromBlock;
        // The NPC's combo: its click's cut, at its end, chains into the next
        // click (the line from the click hook: the NPC's turn of lines, an
        // open line by skill), carrying the momentum as a skilled press does.
        if (npc && mComboLeft > 0 && !blocking && mAuto && mStage == WheelStage::Cut && mCut >= 1.0f && mLastPose)
        {
            --mComboLeft;
            const float carriedFlow = std::max(0.0f, mFlow.mV) * momentumShare;
            startChain(momentumShare, dt);
            mChainVelocity = input.mHandVelocity * momentumShare;
            if (mChainVelocity.Length() > sChainSpeedCap)
                mChainVelocity *= sChainSpeedCap / mChainVelocity.Length();
            mChainTiming = 1.0f;
            mStage = WheelStage::Guard;
            mWind = mCut = 0.0f;
            mDragRight = mDragUp = 0.0f;
            mFlow.mU = 0.0f;
            mFlow.mV = carriedFlow;
            mStanceX = mStanceY = 0.0f;
            mCutFrom.reset();
            mAuto = false;
            mClickQueued = true;
            mComboClick = true;
            events.mChained = true;
        }
        if (!blocking && newPress && mStage != WheelStage::Guard && !stanceHeld && mLastPose)
        {
            if (actor.mSkill < 50.0f && !fromBlock && !quickThrustNow())
            {
                // A click's attack and a cut under way finish first.
                const bool cutGoesOn = (mStage == WheelStage::Cut && mCut < 1.0f)
                    || (mStage == WheelStage::Wind && autoBeforePress);
                if (cutGoesOn && autoBeforePress)
                    mAuto = true;
                if (mStage != WheelStage::Recover && !cutGoesOn)
                {
                    mStage = WheelStage::Recover;
                    mRecover = 0.0f;
                    mRecoverFrom = mLastPose;
                }
                mWaitRecover = true;
            }
            else
            {
                startChain(momentumShare, dt);
                // The hand's measured velocity (the weapon's real momentum).
                if (mPrevPose && dt > 0.0f)
                {
                    mChainVelocity = input.mHandVelocity * momentumShare;
                    if (mChainVelocity.Length() > sChainSpeedCap)
                        mChainVelocity *= sChainSpeedCap / mChainVelocity.Length();
                }
                else
                    mChainSpin = JPH::Vec3::sZero();
                mChainTiming = mStage == WheelStage::Cut ? 1.0f
                    : mStage == WheelStage::Recover
                    ? std::clamp(1.0f - mRecover / std::max(0.05f, mReturnTime), 0.0f, 1.0f)
                    : 0.0f; // from a wind-up: a feint, nothing carried
                const float carriedFlow = std::max(0.0f, mFlow.mV) * momentumShare;
                mStage = WheelStage::Guard;
                mWind = mCut = 0.0f;
                mDragRight = mDragUp = 0.0f;
                mFlow.mU = 0.0f;
                mFlow.mV = carriedFlow;
                mStanceX = mStanceY = 0.0f;
                mCutFrom.reset();
                events.mChained = true;
            }
        }
        // A finished cut that a press waits on skips its hold.
        if (mWaitRecover && mStage == WheelStage::Cut && mCut >= 1.0f)
        {
            mStage = WheelStage::Recover;
            mRecover = 0.0f;
            mRecoverFrom = mLastPose;
        }
        // A click released while it waits plays at the guard.
        if (newPress || blocking)
            mClickQueued = false;
        if (mWaitRecover && newRelease)
            mClickQueued = true;
        if (mWaitRecover && mStage == WheelStage::Guard)
            mWaitRecover = false;

        // The carried momentum: the chain's start goes on the way the weapon
        // went, fading over 0.12 s.
        if (mChainFrom && dt > 0.0f && (mChainVelocity.LengthSq() > 1e-4f || mChainSpin.LengthSq() > 1e-6f))
        {
            mChainFrom->mHand += mChainVelocity * dt;
            const float reachLimit = 0.95f * input.mArmLength;
            if (mChainFrom->mHand.Length() > reachLimit)
            {
                mChainFrom->mHand *= reachLimit / mChainFrom->mHand.Length();
                mChainVelocity *= 0.5f;
            }
            const float spin = std::min(mChainSpin.Length(), sChainSpinMost);
            if (spin > 1e-4f)
                mChainFrom->mRotation
                    = (JPH::Quat::sRotation(mChainSpin.Normalized(), spin * dt) * mChainFrom->mRotation).Normalized();
            const float decay = std::exp(-dt / sChainDrift);
            mChainVelocity *= decay;
            mChainSpin *= decay;
        }
        else if (!mChainFrom)
            mChainVelocity = mChainSpin = JPH::Vec3::sZero();

        // The mouse's velocity over about two steps (input comes in bursts).
        const float dragRight = input.mDragRight, dragUp = input.mDragUp;
        if (held && dt > 0.0f)
        {
            const float k = std::min(1.0f, dt / 0.03f);
            mStrokeVX += (dragRight / dt - mStrokeVX) * k;
            mStrokeVY += (dragUp / dt - mStrokeVY) * k;
        }
        else if (!held)
            mStrokeVX = mStrokeVY = 0.0f;

        // K2_FLICK_CHAIN: held, a finished cut or its return takes a drag
        // away from the blade as the next wind-up, from where the blade is.
        const bool flickable = actor.mFlickChain && held && !blocking && !mWaitRecover && mLastPose
            && ((mStage == WheelStage::Cut && mCutDone && !mAuto) || mStage == WheelStage::Recover);
        if (flickable)
        {
            // Along the finished stroke is the run-on's (above); count the rest.
            const float along = dragRight * mCutStrokeX + dragUp * mCutStrokeY;
            if (!(mStage == WheelStage::Cut && along > 0.0f))
            {
                mFlickX += dragRight;
                mFlickY += dragUp;
            }
            const bool skilled = actor.mSkill >= 50.0f || quickThrustNow()
                || (mStage == WheelStage::Recover && mRecover >= sWaitReturn * std::max(0.05f, mReturnTime));
            if (skilled && std::hypot(mFlickX, mFlickY) >= sPickCounts)
            {
                const float carriedFlow = std::max(0.0f, mFlow.mV) * momentumShare;
                startChain(momentumShare, dt);
                mChainVelocity = input.mHandVelocity * momentumShare;
                if (mChainVelocity.Length() > sChainSpeedCap)
                    mChainVelocity *= sChainSpeedCap / mChainVelocity.Length();
                mChainTiming = mStage == WheelStage::Cut
                    ? 1.0f
                    : std::clamp(1.0f - mRecover / std::max(0.05f, mReturnTime), 0.0f, 1.0f);
                mStage = WheelStage::Guard;
                mWind = mCut = 0.0f;
                mFlow.mU = 0.0f;
                mFlow.mV = carriedFlow;
                mCutFrom.reset();
                mCutDone = false;
                mHeldDone = 0.0f;
                mCutOverflow = 0.0f;
                // The travel so far is the new stance (this step's own drag is
                // added below).
                mStanceX = mFlickX - dragRight;
                mStanceY = mFlickY - dragUp;
                mDragRight = mStanceX;
                mDragUp = mStanceY;
                mFlickX = mFlickY = 0.0f;
                events.mChained = true;
            }
        }
        else
            mFlickX = mFlickY = 0.0f;

        // K2_POINT_AIM: a press in the guard takes the point guard; the mouse
        // moves the point inside the cone. A swing's drag leaves it for the
        // wheel (the stance below starts with that travel): `mPointFlick`
        // counts of net travel within 0.2 s, or a drag pushed on past the
        // cone's edge. Not the mouse's speed: counts come in bursts (the
        // user's 11-12 session: 4,000-38,000 counts/s peaks on moves of 60-400
        // counts; a 3,200 counts/s rule ended the point guard in 26 of 29
        // presses).
        if (pointGuard && !blocking && held && !mPointing && !mAuto && !mWaitRecover
            && mStage == WheelStage::Guard && std::hypot(mStanceX, mStanceY) < 1.0f)
        {
            mPointing = true;
            mPointX = mPointY = mPointShownX = mPointShownY = 0.0f;
            mPointTime = 0.0f;
            mPointOverX = mPointOverY = 0.0f;
            mPointRecentX.fill(0.0f);
            mPointRecentY.fill(0.0f);
            mPointFrom = mChainFrom ? mChainFrom : mLastPose;
            mChainFrom.reset();
        }
        if (mPointing)
        {
            mDragRight += dragRight;
            mDragUp += dragUp;
            const float counts = std::max(100.0f, actor.mPointCounts);
            mPointX += dragRight / counts;
            mPointY += dragUp / counts;
            const float length = std::hypot(mPointX, mPointY);
            if (length > actor.mPointCone)
            {
                const float keep = actor.mPointCone / length;
                mPointOverX += mPointX * (1.0f - keep) * counts;
                mPointOverY += mPointY * (1.0f - keep) * counts;
                mPointX *= keep;
                mPointY *= keep;
            }
            else
                mPointOverX = mPointOverY = 0.0f;
            if (!std::isfinite(mPointX) || !std::isfinite(mPointY))
                mPointX = mPointY = 0.0f;
            // The last 0.2 s of travel (12 steps at 60 Hz).
            mPointRecentAt = (mPointRecentAt + 1) % mPointRecentX.size();
            mPointRecentX[mPointRecentAt] = dragRight;
            mPointRecentY[mPointRecentAt] = dragUp;
            float recentX = 0.0f, recentY = 0.0f;
            for (std::size_t i = 0; i < mPointRecentX.size(); ++i)
            {
                recentX += mPointRecentX[i];
                recentY += mPointRecentY[i];
            }
            const bool flicked = std::hypot(recentX, recentY) >= std::max(actor.mPointFlick, 50.0f);
            if (flicked || std::hypot(mPointOverX, mPointOverY) >= sPointOver)
            {
                mPointing = false;
                mChainFrom = mLastPose;
                mChainTime = 0.0f;
                mChainVelocity = mChainSpin = JPH::Vec3::sZero();
                mFlow = {};
                // The stance below adds this step's drag again.
                mStanceX = (flicked ? recentX : mPointOverX) - dragRight;
                mStanceY = (flicked ? recentY : mPointOverY) - dragUp;
                mDragRight -= dragRight;
                mDragUp -= dragUp;
            }
        }

        mLastCutAlong = 0.0f;
        if (held && !blocking && !mAuto && !mWaitRecover && !fightNight && !fistSteer && !mPointing
            && (mStage == WheelStage::Guard || mStage == WheelStage::Wind))
        {
            // The stance circle: inside it the point moves among compact
            // guards; past it the wind-up commits.
            mDragRight += dragRight;
            mDragUp += dragUp;
            float px = mStanceX, py = mStanceY;
            const float r = std::hypot(px, py);
            bool cut = false;
            if (r > stanceRadius)
            {
                const float ex = px / r, ey = py / r;
                const float dr = dragRight * ex + dragUp * ey;
                const float tx = dragRight - ex * dr, ty = dragUp - ey * dr;
                const float tl = std::hypot(tx, ty);
                const float inward = -(mStrokeVX * ex + mStrokeVY * ey);
                const bool radialIn = dr < 0.0f && tl <= std::abs(dr);
                if (dr < 0.0f)
                    mCutTravel += -dr;
                else if (dr > 0.0f)
                    mCutTravel = 0.0f;
                if (radialIn)
                    mInwardPending += -dr;
                else if (dr > 0.0f)
                    mInwardPending = 0.0f;
                if (inward > sCutSpeed && dr < 0.0f && r > stanceRadius + 0.2f * sCommitCounts
                    && mCutTravel >= sCutTravel)
                {
                    mCutTravel = 0.0f;
                    beginCut(mStrokeVX, mStrokeVY, false, std::max(-dr, mInwardPending) / sStanceCutCounts, striker);
                    mInwardPending = 0.0f;
                    cut = true;
                }
                else if (radialIn && mInwardPending < sInwardHold)
                {
                    // Held while it is decided.
                }
                else if (radialIn && mInwardPending - (-dr) < sInwardHold)
                {
                    // Undecided long enough: a slow retraction, all of it.
                    const float rn = std::max(r - mInwardPending, 0.0f);
                    px = ex * rn;
                    py = ey * rn;
                }
                else if (tl <= std::abs(dr))
                {
                    px += dragRight;
                    py += dragUp;
                }
                else
                {
                    // Mostly sideways from deep: retract toward the circle
                    // first, then turn.
                    const float need = r + dr - stanceRadius;
                    if (need <= 0.0f)
                    {
                        px = ex * (r + dr) + tx;
                        py = ey * (r + dr) + ty;
                    }
                    else if (tl <= need)
                    {
                        const float rn = std::min(r + dr - tl, maxRadius);
                        px = ex * rn;
                        py = ey * rn;
                    }
                    else
                    {
                        const float left = tl - need;
                        px = ex * stanceRadius + tx / tl * left;
                        py = ey * stanceRadius + ty / tl * left;
                    }
                }
            }
            else
            {
                // A quick flick across the circle from a stance: a snap cut.
                const float speed = std::hypot(mStrokeVX, mStrokeVY);
                if (speed > sSnapSpeed && r > 0.3f * stanceRadius && mStrokeVX * px + mStrokeVY * py < 0.0f
                    && mStage == WheelStage::Wind)
                {
                    beginCut(mStrokeVX, mStrokeVY, true, 0.0f, striker);
                    cut = true;
                }
                else
                {
                    px += dragRight;
                    py += dragUp;
                }
            }
            if (!cut)
            {
                float rn = std::hypot(px, py);
                if (rn > maxRadius)
                {
                    px *= maxRadius / rn;
                    py *= maxRadius / rn;
                    rn = maxRadius;
                }
                mStanceX = px;
                mStanceY = py;
                // K2_FISTS: a drag to the left throws the left fist; the right
                // stays on guard, and the rest of that press's drag to the left
                // is spent (no stance is left over for the next press).
                const bool leftFist = fists && actor.mFistSides && actor.mPlayer;
                bool leftTaken = leftFist && mLeftThrown && px < 0.0f;
                if (!leftTaken && leftFist && mLeftArmed && px < -sPickCounts && std::abs(py) < 1.5f * -px
                    && hooks.mLeftPunch && hooks.mLeftPunch())
                {
                    mLeftArmed = false;
                    mLeftThrown = true;
                    leftTaken = true;
                }
                if (leftTaken)
                {
                    mStage = WheelStage::Guard;
                    mWind = mCut = 0.0f;
                    mStanceX = mStanceY = 0.0f;
                    mDragRight = mDragUp = 0.0f;
                }
                // A press that has not moved yet stays in the guard (a click).
                else if (!(mStage == WheelStage::Guard && rn < sPickCounts))
                {
                    if (rn > 1.0f)
                        mAngle = striker.mSpearThrusts ? sThrustAngle : wrap(std::atan2(py, px));
                    mStage = WheelStage::Wind;
                    mWind = windAt(rn, stanceRadius);
                    mCut = 0.0f;
                }
            }
        }
        else if (held && !blocking && !mAuto && mStage == WheelStage::Cut)
        {
            // The cut follows the stroke (a snap 1.6x as fast); the line turns
            // to the stroke's own way, at most an eighth from where the cut
            // began, at most 3 rad/s.
            mCutAccX += dragRight;
            mCutAccY += dragUp;
            const float along = dragRight * mCutStrokeX + dragUp * mCutStrokeY;
            mCut += along / sStanceCutCounts * (mSnapCut ? sSnapRate : 1.0f);
            // B14 (K2_HELD_RECOVER): a cut that has reached its end is done;
            // the mouse no longer drags it back along its path (it was a
            // forceless swing at the mouse's pace, the button still down).
            if (actor.mHeldRecover && mCutDone)
                mCut = std::max(mCut, 1.0f);
            mLastCutAlong = along;
            // Steering: the mouse across the stroke moves the cut's aim.
            const float across = dragUp * mCutStrokeX - dragRight * mCutStrokeY;
            mSteerWantX += -mCutStrokeY * across / sSteerCounts;
            mSteerWantY += mCutStrokeX * across / sSteerCounts;
            const float length = std::hypot(mSteerWantX, mSteerWantY);
            if (length > sSteerMost && length > 0.0f)
            {
                mSteerWantX *= sSteerMost / length;
                mSteerWantY *= sSteerMost / length;
            }
            // A spear's thrust stays a thrust (HSC_THRUST_AIM: build 61 "spear
            // is broken").
            const bool spearThrustCut = (striker.mSpearThrusts || striker.mSpearSwings)
                && std::abs(signedDiff(mCutAngle, sThrustAngle)) < sEighth * 0.5f;
            if (std::hypot(mCutAccX, mCutAccY) > 80.0f && !spearThrustCut)
            {
                const float strokeAngle = wrap(std::atan2(-mCutAccY, -mCutAccX));
                const float wanted = std::clamp(signedDiff(strokeAngle, mCutAngle), -sEighth, sEighth);
                const float current = signedDiff(mAngle, mCutAngle);
                const float rate = 3.0f * dt;
                mAngle = wrap(mCutAngle + current + std::clamp(wanted - current, -rate, rate));
            }
            // Dragged on past the follow-through (skill 50 up): the stroke runs
            // on into the next wind-up the same way, with the momentum.
            if (mCut > 1.0f)
                mCutOverflow += (mCut - 1.0f) * sStanceCutCounts / (mSnapCut ? sSnapRate : 1.0f);
            else if (along < 0.0f)
                mCutOverflow = 0.0f;
            const float overflow = mCutOverflow;
            if (actor.mSkill >= 50.0f && overflow > sRunOnCounts && mLastPose)
            {
                const float carriedFlow = std::max(0.0f, mFlow.mV) * momentumShare;
                mCutOverflow = 0.0f;
                startChain(momentumShare, dt);
                mChainVelocity = input.mHandVelocity * momentumShare;
                if (mChainVelocity.Length() > sChainSpeedCap)
                    mChainVelocity *= sChainSpeedCap / mChainVelocity.Length();
                mChainTiming = 1.0f; // a held stroke running on: all of it
                const float rn = std::min(stanceRadius + std::max(overflow - sRunOnCounts, 0.0f), maxRadius);
                mStanceX = mCutStrokeX * rn;
                mStanceY = mCutStrokeY * rn;
                mAngle = wrap(std::atan2(mStanceY, mStanceX));
                mStage = WheelStage::Wind;
                mWind = windAt(rn);
                mCut = 0.0f;
                mFlow.mU = 0.0f;
                mFlow.mV = carriedFlow;
                mCutFrom.reset();
                events.mChained = true;
            }
            else if (mCut <= 0.0f)
            {
                // Dragged back: the wind-up again, in its chamber - a feint,
                // which costs fatigue.
                events.mFeintFatigue = sFeintFatigue;
                mStage = WheelStage::Wind;
                mWind = 1.0f;
                mCut = 0.0f;
                mAngle = mCutAngle;
                mStanceX = -mCutStrokeX * sMaxRadius;
                mStanceY = -mCutStrokeY * sMaxRadius;
                mCutFrom.reset();
            }
            mCut = std::min(1.0f, mCut);
        }
        if (!held)
            mDragRight = mDragUp = 0.0f;
        // K2_FISTS: the left fist is thrown again after the drag came back
        // (or the button came up).
        if (!mLeftArmed)
        {
            mLeftRearm += std::max(0.0f, dragRight);
            if (!held || mLeftRearm > 0.5f * sPickCounts)
            {
                mLeftArmed = true;
                mLeftRearm = 0.0f;
            }
        }

        // A cut beginning is an attack: a whoosh and a committed attack's
        // fatigue (less for a chain that flowed).
        if (mStage == WheelStage::Cut && stageBefore == WheelStage::Wind)
        {
            events.mCutStarted = true;
            // Every cut is its own swing (CONNECT-01; an NPC's attacks start
            // with no press).
            if (!newPress)
                ++mSwing;
            events.mSwishSize = mAuto ? 1.0f : 0.8f;
            events.mFatigueShare = 1.0f - sChainFlowFatigue * mChainFlowGain;
            events.mFlowGain = mChainFlowGain;
            events.mRiposte = mPlayerRiposte;
            mChainFlowGain = 0.0f;
            mChainFlowPending = false;
            mPlayerRiposte = false;
        }
        else if (mStage == WheelStage::Recover || (mStage == WheelStage::Guard && !mChainFrom))
        {
            mChainFlowGain = 0.0f;
            mChainFlowPending = false;
            mPlayerRiposte = false;
        }

        // The route's flow.
        pace = pacing();
        const bool routed = mStage == WheelStage::Wind || mStage == WheelStage::Cut;
        if (!routed)
            mFlow = {};
        else
        {
            const float rawU = mStage == WheelStage::Cut ? 1.0f + mCut : mWind;
            stepRouteFlow(mFlow, rawU, dt, pace, mStage == WheelStage::Wind ? pace.mWindHaste : 1.0f);
        }

        const auto guard = guardPose(lines, striker.mAxes);
        const ChamberPose guardAt = guard ? *guard : ChamberPose{};

        // A stance inside the circle stays when the button comes up.
        const bool stanceStays = !mAuto && !mChainFrom && mStage == WheelStage::Wind && mWind <= 0.5f;
        // A committed cut finishes at least at the click's cut speed, held or
        // released, unless dragged back (user 10-06).
        if (!mAuto && !blocking && mStage == WheelStage::Cut && mCut >= 0.0f && mCut < 1.0f && dt > 0.0f
            && mLastCutAlong >= 0.0f)
        {
            const float minimum = dt / (0.25f * pace.mWeightPace);
            const float dragged = held ? mLastCutAlong / sStanceCutCounts * (mSnapCut ? sSnapRate : 1.0f) : 0.0f;
            if (dragged < minimum)
                mCut = std::min(1.0f, mCut + (minimum - dragged));
            mReleased = 0.0f;
        }
        if (!held && mReleased > pace.mReleaseHold && mStage != WheelStage::Guard && mStage != WheelStage::Recover
            && !stanceStays)
        {
            mStage = WheelStage::Recover;
            mRecover = 0.0f;
            mRecoverFrom = mLastPose;
        }
        // B14 (play bookmark 11-03, "after a swing if I havent recovered if I
        // hold the click button it starts swinging but without any force"):
        // with K2_HELD_RECOVER a finished cut recovers after its hold with the
        // button still down, unless the mouse is dragging on along the stroke
        // (the run-on into the next swing, a combo). Back in the guard, a held
        // drag starts a new wind-up from there.
        if (mStage == WheelStage::Cut && mCut >= 1.0f)
            mCutDone = true;
        else if (mStage != WheelStage::Cut)
            mCutDone = false;
        mHeldDone = mCutDone && held ? mHeldDone + dt : 0.0f;
        if (actor.mHeldRecover && held && !blocking && mCutDone && mHeldDone > pace.mReleaseHold
            && !(mLastCutAlong > 0.0f && !mAuto))
        {
            mStage = WheelStage::Recover;
            mRecover = 0.0f;
            mRecoverFrom = mLastPose;
            mAuto = false;
            mCutDone = false;
            mHeldDone = 0.0f;
            mStanceX = mStanceY = 0.0f;
            mDragRight = mDragUp = 0.0f;
            mCutOverflow = 0.0f;
        }

        std::optional<ChamberPose> pose;
        if (blocking)
        {
            if (!mBlocking)
            {
                mBlocking = true;
                mBlockFrom = mLastPose ? *mLastPose : guardAt;
                mBlockTime = 0.0f;
                mBlockSteer = JPH::Vec3::sZero();
                mStage = WheelStage::Guard;
                mWind = mCut = 0.0f;
                mFlow = {};
                mChainFrom.reset();
                mAuto = false;
            }
            mBlockTime += dt;
            WheelGuardZone zone = input.mGuardZone;
            if (actor.mThreatZone && !actor.mPlayer && input.mThreat)
                zone = threatZone(*input.mThreat);
            const bool shoving = input.mShove;
            if (shoving)
                zone = WheelGuardZone::Shove;
            if (zone != mBlockZone)
            {
                mBlockZone = zone;
                if (mLastPose)
                    mBlockFrom = *mLastPose;
                mBlockTime = 0.0f;
            }
            BlockKey key = blockKey(mBlockZone);
            // The thrust: straight out from the shoulder, the arm's length
            // (the player's guard already holds the hand two thirds of it
            // out: 0.85 added 6 u, SHOVE-03), the chest leaning into it.
            if (shoving)
            {
                key.mHand = (input.mShoulder ? *input.mShoulder : JPH::Vec3(11.5f, 0.0f, 8.0f))
                    + JPH::Vec3(-3.0f, sShoveReach * input.mArmLength, 0.0f);
                mThrustDriveWant = sShoveDrive;
            }
            JPH::Vec3 bladeDir = key.mBlade.Normalized();
            if (input.mThreat && !shoving)
            {
                const JPH::Vec3 threat = (*input.mThreat - key.mHand).NormalizedOr(bladeDir);
                bladeDir = (bladeDir + threat).NormalizedOr(bladeDir);
            }
            // K2_PARRY_STEER: the hand goes to where the foe's striking part
            // will cross the guard, within the zone's tolerance.
            JPH::Vec3 steer = JPH::Vec3::sZero();
            if (actor.mParrySteer > 0.0f && input.mThreatStrike && !shoving)
            {
                // The blade meets it with its strong half.
                const JPH::Vec3 meet = key.mHand + bladeDir * (sSteerMeet * striker.mTip);
                JPH::Vec3 at = *input.mThreatStrike;
                const float closing = -input.mThreatVelocity.GetY();
                if (closing > sSteerClosing)
                    at += input.mThreatVelocity
                        * std::clamp((at.GetY() - meet.GetY()) / closing, 0.0f, sSteerLead);
                steer = at - meet;
                steer.SetY(0.0f);
                // Along its own length the blade already covers a stretch.
                JPH::Vec3 along(bladeDir.GetX(), 0.0f, bladeDir.GetZ());
                if (along.LengthSq() > 0.04f)
                {
                    along = along.Normalized();
                    const float cover = sSteerCover * striker.mTip;
                    steer -= along * std::clamp(steer.Dot(along), -cover, cover);
                }
                const float control = std::clamp(actor.mControl, 0.0f, 1.0f);
                const float reach = actor.mParrySteer * (0.4f + 0.6f * control);
                if (const float length = steer.Length(); length > reach)
                    steer *= reach / length;
            }
            {
                const float control = std::clamp(actor.mControl, 0.0f, 1.0f);
                mBlockSteer += (steer - mBlockSteer) * std::min(1.0f, dt * (8.0f + 12.0f * control));
            }
            ChamberPose block;
            block.mHand = key.mHand + mBlockSteer;
            block.mPole = key.mPole.Normalized();
            block.mTwist = key.mTwistDegrees * (sPi / 180.0f);
            const JPH::Vec3 edge = JPH::Vec3(0.0f, 1.0f, 0.0f) - bladeDir * bladeDir.GetY();
            const auto rotation
                = alignedStrikeHandRotation(striker.mAxes, bladeDir, edge.NormalizedOr(JPH::Vec3::sAxisY()));
            block.mRotation = rotation ? rotation->Normalized() : guardAt.mRotation;
            pose = blendPose(mBlockFrom, block, smoothStep(mBlockTime / sBlockTime));
        }
        else if (mBlocking)
        {
            mBlocking = false;
            // The attack press that ended the block goes straight into the
            // attack from it (GUARD-03).
            if (held && actor.mPlayer && mLastPose)
            {
                mStage = WheelStage::Guard;
                mWind = mCut = 0.0f;
                mChainFrom = mLastPose;
                mChainTime = 0.0f;
                mChainVelocity = mChainSpin = JPH::Vec3::sZero();
                mFlow = {};
                mCutFrom.reset();
            }
            else
            {
                mStage = WheelStage::Recover;
                mRecover = 0.0f;
                mRecoverFrom = mLastPose;
                mSinceBlock = 0.0f;
            }
        }

        if (!(mAuto && (mStage == WheelStage::Wind || mStage == WheelStage::Cut)))
            mPointThrust = false;
        if (pose)
        {
            // The block above.
        }
        else if (mPointing || mPointThrust)
        {
            // K2_POINT_AIM: the point guard, and its thrust along the shaft -
            // drawn back over the wind-up, driven out over the cut's first
            // 0.45 (sPointDrive), the chest turning with it.
            if (aimInput)
                mAim = solveWheelAim(*aimInput, sThrustAngle, striker.mTip);
            else
            {
                mAim = {};
                mAim.mThrustness = 1.0f;
            }
            // The point follows the mouse over 0.05 s (its counts come in bursts).
            {
                const float k = std::min(1.0f, dt / 0.05f);
                mPointShownX += (mPointX - mPointShownX) * k;
                mPointShownY += (mPointY - mPointShownY) * k;
            }
            mSteerX = mSteerY = 0.0f;
            mPointTime += dt;
            const bool cutting = mStage == WheelStage::Cut && mFlow.mU > 1.0f;
            // How far out the thrust is: 0 in the guard and the draw, 1 driven out.
            const auto outAt = [&](float u) {
                return mPointThrust && u > 1.0f ? smoothStep((u - 1.0f) / sPointDrive) : 0.0f;
            };
            const auto poseWith = [&](const WheelAim& aim, float u) {
                float along = 0.0f, twist = actor.mPointTwist;
                if (mPointThrust && u <= 1.0f)
                {
                    const float w = smoothStep(u);
                    along = -actor.mPointDraw * w;
                    twist += 0.2f * w;
                }
                else if (mPointThrust)
                {
                    const float w = outAt(u);
                    along = -actor.mPointDraw + (actor.mPointDraw + actor.mPointReach) * w;
                    twist += 0.2f - 0.5f * w;
                }
                return pointPose(striker, actor, input, aim, along, twist, guardAt);
            };
            const float routeU = mPointThrust ? mFlow.mU : 0.0f;
            // The trims: the command moves until the real point's bearing is
            // the asked one. The held weapon lies off its command by an amount
            // that depends on the arms' place (the iron spear, 11-12: 11 u
            // right of it at 70 in the guard, 3 driven out), so there are two:
            // the guard's, learned at 3/s of the error while the guard is
            // settled, and the driven-out thrust's, learned at 2/s while a
            // thrust is held at its end (before the first: 0.35 of the guard's
            // sideways, all of it in height). The drive goes from one to the
            // other, so the real shaft keeps its line (with the guard's trim
            // kept, the real point ended 12 u left of where it aimed).
            const bool settledGuard = mPointing && mPointTime > 2.0f * sPointEase;
            // (0.12 s after it arrives: learned at once and at 8/s, the trim
            // chased the still-moving weapon and threw the point 7 u aside.)
            mPointOutTime = mPointThrust && mStage == WheelStage::Cut && mCut >= 1.0f && mFlow.mU >= 1.97f
                ? mPointOutTime + dt
                : 0.0f;
            const bool settledOut = mPointOutTime > 0.12f;
            if ((settledGuard || settledOut) && input.mPoint && input.mPoint->GetY() > 10.0f)
            {
                // Where the point was asked: the untrimmed pose's.
                WheelAim asked = mAim;
                steerWheelAim(asked, sThrustAngle, mPointShownX, mPointShownY, false);
                const ChamberPose want = poseWith(asked, routeU);
                const JPH::Vec3 wanted
                    = want.mHand + (want.mRotation * striker.mBladeAxis).NormalizedOr(JPH::Vec3::sAxisY()) * striker.mTip;
                const JPH::Vec3 real = *input.mPoint;
                const auto azimuth = [](const JPH::Vec3& v) { return std::atan2(v.GetX(), v.GetY()); };
                const auto elevation
                    = [](const JPH::Vec3& v) { return std::atan2(v.GetZ(), std::hypot(v.GetX(), v.GetY())); };
                if (settledOut && !mPointTrimOutKnown)
                {
                    mPointTrimOutX = 0.35f * mPointTrimX;
                    mPointTrimOutY = mPointTrimY;
                    mPointTrimOutKnown = true;
                }
                float& trimX = settledGuard ? mPointTrimX : mPointTrimOutX;
                float& trimY = settledGuard ? mPointTrimY : mPointTrimOutY;
                const float gain = std::min(1.0f, (settledGuard ? 3.0f : 2.0f) * dt);
                trimX = std::clamp(trimX + (azimuth(wanted) - azimuth(real)) * gain, -0.3f, 0.3f);
                trimY = std::clamp(trimY + (elevation(wanted) - elevation(real)) * gain, -0.3f, 0.3f);
                if (!std::isfinite(trimX) || !std::isfinite(trimY))
                    trimX = trimY = 0.0f;
            }
            {
                const float outX = mPointTrimOutKnown ? mPointTrimOutX : 0.35f * mPointTrimX;
                const float outY = mPointTrimOutKnown ? mPointTrimOutY : mPointTrimY;
                // By the physical hand's way out, not the command's: the hand
                // trails it (8 u mid-drive), and with the command's the point
                // went 7 u to the weapon side on the way.
                float out = outAt(routeU);
                if (mPointThrust && input.mHand && out > 0.0f)
                {
                    WheelAim asked = mAim;
                    steerWheelAim(asked, sThrustAngle, mPointShownX, mPointShownY, false);
                    const JPH::Vec3 from = poseWith(asked, 1.0f).mHand;
                    const JPH::Vec3 way = poseWith(asked, 2.0f).mHand - from;
                    if (const float span = way.Length(); span > 1.0f)
                        out = std::min(out, std::clamp((*input.mHand - from).Dot(way) / (span * span), 0.0f, 1.0f));
                }
                steerWheelAim(mAim, sThrustAngle, mPointShownX + mPointTrimX + (outX - mPointTrimX) * out,
                    mPointShownY + mPointTrimY + (outY - mPointTrimY) * out, mPointThrust);
            }
            const auto at = [&](float u) { return poseWith(mAim, u); };
            mShownU = mPointThrust ? mFlow.mU : 0.0f;
            pose = at(mShownU);
            if (mPointFrom)
            {
                const float w = smoothStep(mPointTime / (mPointThrust ? sPointThrustEase : sPointEase));
                pose = blendPose(*mPointFrom, *pose, w);
                if (w >= 1.0f)
                    mPointFrom.reset();
            }
            if (mPointThrust)
            {
                mThrustDriveWant = thrustDrive(mAim, actor.mPlayer, mShownU, cutting);
                if (actor.mPlayer && cutting)
                    for (int k = 0; k < 16; ++k)
                    {
                        const ChamberPose p = at(1.0f + static_cast<float>(k) / 15.0f);
                        mPreview.push_back(
                            p.mHand + (p.mRotation * striker.mBladeAxis).NormalizedOr(JPH::Vec3::sAxisY()) * striker.mTip);
                    }
                mTwistLead = pose->mTwist;
                mTwistCut.reset();
                const WheelSlot slot = wheelSlot(mAngle);
                mLine = static_cast<int>(slot.mWeight < 0.5f ? slot.mFirst : slot.mSecond);
                mChainFrom.reset();
            }
        }
        else if ((mStage == WheelStage::Guard || mStage == WheelStage::Recover) && fistSteer && input.mFistHold
            && !blocking)
        {
            // FIST-06: the held fist, where the mouse has it - the hand's
            // turn from the wheel's wind-up there; taken from where the hand
            // was (the guard, a punch's return) over 0.12 s.
            const FistSteerHold& hold = *input.mFistHold;
            const auto turned = wheelPose(lines, striker.mAxes, hold.mAngle, hold.mAmount);
            ChamberPose shown = turned ? *turned : guardAt;
            shown.mHand = hold.mHand;
            shown.mPole = hold.mPole;
            shown.mTwist = hold.mTwist;
            if (!mFistHeld)
            {
                mFistHoldFrom = mLastPose;
                mFistHoldTime = 0.0f;
            }
            mFistHoldTime += dt;
            if (mFistHoldFrom && mFistHoldTime < 0.12f)
                shown = blendPose(*mFistHoldFrom, shown, smoothStep(mFistHoldTime / 0.12f));
            else
                mFistHoldFrom.reset();
            pose = shown;
            mStage = WheelStage::Guard;
            mWind = mCut = 0.0f;
            mChainFrom.reset();
            mFistHeld = true;
        }
        else if (mStage == WheelStage::Recover)
        {
            mRecover += dt;
            mReturnTime = pace.mReturn;
            const float u = std::clamp(mRecover / mReturnTime, 0.0f, 1.0f);
            const float w = smoothStep(u);
            if (!mRecoverFrom || u >= 1.0f)
            {
                mStage = WheelStage::Guard;
                mWind = mCut = 0.0f;
                pose = guardAt;
            }
            else if (mWaitRecover && u >= sWaitReturn)
            {
                // The waiting press goes from here, with no momentum.
                mStage = WheelStage::Guard;
                mWind = mCut = 0.0f;
                mChainFrom = blendPose(*mRecoverFrom, guardAt, w);
                mChainVelocity = mChainSpin = JPH::Vec3::sZero();
                mChainTime = 0.0f;
                pose = *mChainFrom;
            }
            else
                pose = blendPose(*mRecoverFrom, guardAt, w);
        }
        else if (mStage == WheelStage::Guard && mFistHeld && mLastPose)
        {
            // The hold ended with nothing thrown (the guard, a cancel): the
            // fist returns from where it was.
            mFistHeld = false;
            mStage = WheelStage::Recover;
            mRecover = 0.0f;
            mRecoverFrom = mLastPose;
            pose = *mLastPose;
        }
        else if (mStage == WheelStage::Guard)
        {
            pose = mChainFrom ? *mChainFrom : guardAt;
            // Released without a new drag: back to the guard from there.
            if (mChainFrom && !held)
            {
                mStage = WheelStage::Recover;
                mRecover = 0.0f;
                mRecoverFrom = mChainFrom;
                mChainFrom.reset();
            }
        }
        else
        {
            const WheelSlot slot = wheelSlot(mAngle);
            const bool cutting = mStage == WheelStage::Cut && mFlow.mU > 1.0f;
            const bool winding = mStage == WheelStage::Wind;
            // The aim: the crosshair point, the foe assist, then the steering
            // (at most 4 rad/s, let go outside the cut).
            if (mStage != WheelStage::Cut)
                mSteerWantX = mSteerWantY = 0.0f;
            {
                const float rate = 4.0f * dt;
                mSteerX += std::clamp(mSteerWantX - mSteerX, -rate, rate);
                mSteerY += std::clamp(mSteerWantY - mSteerY, -rate, rate);
                if (!std::isfinite(mSteerX) || !std::isfinite(mSteerY))
                    mSteerX = mSteerY = mSteerWantX = mSteerWantY = 0.0f;
            }
            if (aimInput)
            {
                // B6 ("the axe range is too small sometimes"): a two-handed
                // grip reaches with less of the arm (the support hand holds the
                // shoulders square), so its step-in toward a foe past the sweet
                // reach starts sooner (K2_TWO_HAND_REACH, the arm's share; the
                // step-in's size is HSC_LUNGE's).
                if (striker.mTwoHanded && actor.mTwoHandReach > 0.0f && actor.mTwoHandReach < 1.0f)
                {
                    WheelAimInput twoHanded = *aimInput;
                    twoHanded.mArmLength *= actor.mTwoHandReach;
                    mAim = solveWheelAim(twoHanded, mAngle, striker.mTip);
                }
                else
                    mAim = solveWheelAim(*aimInput, mAngle, striker.mTip);
                steerWheelAim(mAim, mAngle, mSteerX, mSteerY, mAuto);
            }
            const auto planeAt = [&](float u, bool asCut, bool asWind) {
                return aimInput ? planeWeight(mAim, mAngle, mAuto, u, asCut, asWind) : 0.0f;
            };
            // A chain goes straight on to the outer half of its wind-up
            // (Meyer's square), blended over the whole wind-up - not into a
            // low chamber, which chains through the guard over the first half.
            const bool directHere = std::sin(mAngle) > -0.3f;
            const float lineSpan = lines[slot.mFirst].mChainSpan
                + (lines[slot.mSecond].mChainSpan - lines[slot.mFirst].mChainSpan) * slot.mWeight;
            const float chainSpan = (directHere ? 1.0f : 0.5f) * std::clamp(lineSpan, 0.1f, 1.0f);
            if (mChainFrom)
            {
                if (mChainTime <= 0.0f)
                    mChainWayTarget.reset();
                mChainTime += dt;
            }
            const float chainByTime = std::clamp(mChainTime / 0.5f, 0.0f, 1.0f);
            // Chain flow: does the new wind-up lie where the last cut ended?
            if (mChainFrom && mChainFlowPending && mStage == WheelStage::Wind)
            {
                mChainFlowPending = false;
                const float along = -std::cos(mAngle - mChainPrevAngle);
                mChainFlowGain = std::clamp(along, 0.0f, 1.0f) * mChainTiming * (0.5f + 0.5f * momentumShare);
            }
            const auto lineBlend = [&](float u) -> std::optional<ChamberPose> {
                const auto a = linePose(lines[slot.mFirst], striker.mAxes, u);
                const auto b = linePose(lines[slot.mSecond], striker.mAxes, u);
                if (!a || !b)
                    return std::nullopt;
                return blendPose(*a, *b, slot.mWeight);
            };
            // Never faster than the arm can turn the blade: 0.12 s plus 0.2 s
            // per half turn to the new chamber.
            float chainCap = 1.0f;
            if (mChainFrom)
                if (const auto chamber = lineBlend(directHere && lineSpan < 1.0f ? 0.5f + 0.5f * chainSpan : 1.0f))
                {
                    const float turn
                        = 2.0f * std::acos(std::clamp(std::abs(chamber->mRotation.Dot(mChainFrom->mRotation)), 0.0f, 1.0f));
                    chainCap = std::clamp(mChainTime / (0.12f + sChainTurn * turn / sPi), 0.0f, 1.0f);
                }
            const float chainProgress = mChainFrom
                ? std::max(std::min(std::clamp(mFlow.mU / chainSpan, 0.0f, 1.0f), chainCap), chainByTime)
                : 1.0f;
            const float chainWeight = mChainFrom ? smoothStep(chainProgress) : 1.0f;
            const float remap = mChainFrom && directHere ? 1.0f : 0.0f;
            const auto lineAt = [&](float u) -> std::optional<ChamberPose> {
                const float routeU = u <= 1.0f ? u + remap * (0.5f - 0.5f * u) : u;
                auto p = lineBlend(routeU);
                if (!p || !mChainFrom)
                    return p;
                ChamberPose blended = blendPose(*mChainFrom, *p, chainWeight);
                JPH::Quat target = p->mRotation;
                if (!mChainWayTarget)
                    target = target.Dot(mChainFrom->mRotation) >= 0.0f ? target : -target;
                else if (target.Dot(*mChainWayTarget) < 0.0f)
                    target = -target;
                mChainWayTarget = target;
                blended.mRotation = slerpWay(mChainFrom->mRotation, target, chainWeight);
                return blended;
            };
            // Even speed along the route: retimed by the tip's arc length
            // (16 samples, shaped as shown).
            const auto evenRoute = [&](float base, float cu) {
                constexpr int samples = 16;
                std::array<float, samples + 1> length{};
                JPH::Vec3 previousTip = JPH::Vec3::sZero();
                for (int k = 0; k <= samples; ++k)
                {
                    const float s = static_cast<float>(k) / samples;
                    auto p = lineBlend(base + s);
                    if (!p)
                        return cu;
                    putInPlane(*p, mAim, planeAt(base + s, cutting, winding), striker.mBladeAxis, striker.mTip);
                    const JPH::Vec3 blade = (p->mRotation * striker.mBladeAxis).NormalizedOr(JPH::Vec3::sAxisY());
                    const JPH::Vec3 tip = p->mHand + blade * striker.mTip;
                    length[static_cast<std::size_t>(k)]
                        = k > 0 ? length[static_cast<std::size_t>(k - 1)] + (tip - previousTip).Length() : 0.0f;
                    previousTip = tip;
                }
                const float total = length[samples];
                if (total < 1e-3f)
                    return cu;
                const float target = std::clamp(cu, 0.0f, 1.0f) * total;
                int k = 0;
                while (k < samples - 1 && length[static_cast<std::size_t>(k + 1)] < target)
                    ++k;
                const float span = length[static_cast<std::size_t>(k + 1)] - length[static_cast<std::size_t>(k)];
                const float f = span > 1e-5f ? (target - length[static_cast<std::size_t>(k)]) / span : 0.0f;
                return (static_cast<float>(k) + std::clamp(f, 0.0f, 1.0f)) / samples;
            };
            // K2_CUT_REACH: a one-handed cut reaches out from the shoulder at
            // route position u (1..2), most from a third of the cut to three
            // quarters, half kept at its end; none on the thrust's side of
            // the wheel (the low diagonals half).
            const auto reachOut = [&](ChamberPose& p, float u) {
                if (actor.mCutReach <= 0.0f || !input.mShoulder || striker.mTwoHanded || striker.mShort
                    || striker.mSpearThrusts)
                    return;
                const float c = u - 1.0f;
                const float along = smoothStep((c - 0.05f) / 0.3f) * (1.0f - 0.5f * smoothStep((c - 0.75f) / 0.25f));
                const float down = std::max(0.0f, -std::sin(mAngle));
                const float share = std::min(actor.mCutReach, 1.0f) * along * (1.0f - down * down);
                const JPH::Vec3 out = p.mHand - *input.mShoulder;
                const float now = out.Length();
                const float most = 0.95f * input.mArmLength;
                if (share <= 0.0f || now < 1.0f || now >= most || !std::isfinite(now))
                    return;
                p.mHand = *input.mShoulder + out * ((now + (most - now) * share) / now);
            };
            mShownU = cutting ? 1.0f + evenRoute(1.0f, mFlow.mU - 1.0f) : evenRoute(0.0f, std::min(mFlow.mU, 1.0f));
            pose = lineAt(mShownU);
            if (!pose)
                pose = mLastPose ? *mLastPose : guardAt;
            // A cut along another line than its wind-up's (or a snap) starts
            // from where the weapon was.
            if (mCutFrom && mStage == WheelStage::Cut)
            {
                const float c = (mFlow.mU - 1.0f - mCutFromU) / 0.25f;
                pose = blendPose(*mCutFrom, *pose, smoothStep(c));
                if (c >= 1.0f)
                    mCutFrom.reset();
            }
            else if (mStage != WheelStage::Cut)
                mCutFrom.reset();
            if (aimInput)
            {
                putInPlane(*pose, mAim, planeAt(mShownU, cutting, winding), striker.mBladeAxis, striker.mTip);
                // Thrusts turn to the aim, in full from the wind-up's second half.
                turnThrust(*pose, mAim,
                    mAim.mThrustness * (cutting ? 1.0f : winding ? smoothStep((mShownU - 0.5f) / 0.5f) : 0.0f));
                if (striker.mShort && cutting)
                    reachShort(*pose, mAim, mShownU, striker.mTip);
                mThrustDriveWant = thrustDrive(mAim, actor.mPlayer, mShownU, cutting);
                reachThrust(*pose, mThrustDriveWant, striker.mBladeAxis, striker.mShort);
                if (striker.mPointThrust)
                    pointThrust(*pose, mAim,
                        mAim.mThrustness * (cutting ? 1.0f : winding ? smoothStep((mShownU - 0.5f) / 0.5f) : 0.0f),
                        striker.mBladeAxis, striker.mTip);
                // The swing's trajectory for the reticle (TRAJ-01): the
                // commanded tip along the cut once it is under way; none while
                // winding up (user 11-08: "they dont provide much info").
                if (actor.mPlayer && cutting)
                {
                    constexpr int points = 16;
                    // The mark: the foe's body as an upright capsule through
                    // its chest (thighs to crown; a downed foe's a ball).
                    const float radius = actor.mPreviewFoeRadius;
                    const bool marking = radius > 0.0f && mFoe.has_value();
                    const JPH::Vec3 foeLow = marking ? *mFoe - JPH::Vec3(0.0f, 0.0f, mFoeDowned ? 10.0f : 50.0f)
                                                     : JPH::Vec3::sZero();
                    const JPH::Vec3 foeHigh = marking ? *mFoe + JPH::Vec3(0.0f, 0.0f, mFoeDowned ? 10.0f : 22.0f)
                                                      : JPH::Vec3::sZero();
                    // The blade's nearest point to the body and its gap.
                    const auto bladeGap = [&](const JPH::Vec3& hand, const JPH::Vec3& tip) {
                        std::pair<float, JPH::Vec3> best{ std::numeric_limits<float>::max(), tip };
                        // Only the striking part (an axe's head, not its haft).
                        constexpr int along = 6;
                        const float from = std::clamp(striker.mStrikeStart, 0.0f, 0.95f);
                        for (int i = 0; i <= along; ++i)
                        {
                            const float share = from + (1.0f - from) * (static_cast<float>(i) / along);
                            const JPH::Vec3 at = hand + (tip - hand) * share;
                            const JPH::Vec3 axis = foeHigh - foeLow;
                            const float t = std::clamp((at - foeLow).Dot(axis) / std::max(axis.LengthSq(), 1e-4f), 0.0f, 1.0f);
                            const float gap = (at - (foeLow + axis * t)).Length() - radius;
                            if (gap < best.first)
                                best = { gap, at };
                        }
                        return best;
                    };
                    std::optional<WheelMark> closest;
                    std::optional<std::pair<float, JPH::Vec3>> before;
                    // The preview only reads: lineAt keeps a chain's turn
                    // direction, which the swing itself must not see moved.
                    const std::optional<JPH::Quat> chainWay = mChainWayTarget;
                    for (int k = 0; k < points; ++k)
                    {
                        const float u = 1.0f + static_cast<float>(k) / static_cast<float>(points - 1);
                        auto p = lineAt(u);
                        if (!p)
                        {
                            mPreview.clear();
                            closest.reset();
                            mMark.reset();
                            break;
                        }
                        putInPlane(*p, mAim, planeAt(u, true, false), striker.mBladeAxis, striker.mTip);
                        turnThrust(*p, mAim, mAim.mThrustness);
                        reachThrust(*p, thrustDrive(mAim, actor.mPlayer, u, true), striker.mBladeAxis, striker.mShort);
                        if (striker.mPointThrust)
                            pointThrust(*p, mAim, mAim.mThrustness, striker.mBladeAxis, striker.mTip);
                        reachOut(*p, u);
                        const JPH::Vec3 tip
                            = p->mHand + (p->mRotation * striker.mBladeAxis).NormalizedOr(JPH::Vec3::sAxisY()) * striker.mTip;
                        mPreview.push_back(tip);
                        if (!marking)
                            continue;
                        const auto gap = bladeGap(p->mHand, tip);
                        if (!std::isfinite(gap.first))
                            continue;
                        if (gap.first <= 0.0f && !mMark)
                        {
                            // The first touch, between this sample and the last.
                            JPH::Vec3 point = gap.second;
                            if (before && before->first > 0.0f)
                            {
                                const float f = before->first / (before->first - gap.first);
                                point = before->second + (gap.second - before->second) * std::clamp(f, 0.0f, 1.0f);
                            }
                            mMark = WheelMark{ point, true, gap.first };
                        }
                        if (!closest || gap.first < closest->mGap)
                            closest = WheelMark{ gap.second, false, gap.first };
                        before = gap;
                    }
                    if (marking && closest && !mPreview.empty())
                    {
                        if (!mMark)
                            mMark = closest;
                        else
                            mMark->mGap = closest->mGap; // the deepest the cut goes in
                    }
                    mChainWayTarget = chainWay;
                }
                if (cutting)
                    mCutPlaneMiss = std::abs(
                        mAim.mPlaneNormal.Dot(pose->mHand + (pose->mRotation * striker.mBladeAxis) * striker.mTip));
            }
            if (cutting)
                reachOut(*pose, mShownU);
            // A steady NPC keeps its weapon hand in front of its body: deeper
            // than 6 behind the chest centre is pressed to 30% of the excess
            // (NPC_STEADY).
            if (!actor.mPlayer && pose->mHand.GetY() < -6.0f)
                pose->mHand.SetY(-6.0f + (pose->mHand.GetY() + 6.0f) * 0.3f);
            // The body leads the arm: the chest's turn taken ahead on the route.
            mTwistLead = pose->mTwist;
            mTwistCut.reset();
            if (std::abs(mFlow.mV) > 1e-3f)
            {
                // In a cut the chest itself is taken `mCutLead` ahead, and the
                // hips lead it as before (K2_CUT_LEAD).
                const float cutLead = mStage == WheelStage::Cut && actor.mCutLead > 0.0f ? actor.mCutLead : 0.0f;
                if (cutLead > 0.0f)
                    if (const auto ahead = lineAt(std::clamp(mShownU + mFlow.mV * cutLead, 0.0f, 2.0f)))
                        mTwistCut = ahead->mTwist;
                if (const auto ahead = lineAt(std::clamp(mShownU + mFlow.mV * (cutLead + sBodyLead), 0.0f, 2.0f)))
                    mTwistLead = ahead->mTwist;
            }
            if (mChainFrom && mFlow.mU >= chainSpan && chainProgress >= 1.0f)
                mChainFrom.reset();
            mLine = static_cast<int>(slot.mWeight < 0.5f ? slot.mFirst : slot.mSecond);
        }
        if (mStage == WheelStage::Guard)
            mLine = -1;
        if (mStage == WheelStage::Recover || mBlocking || (mStage == WheelStage::Guard && !held))
        {
            mStanceX = mStanceY = 0.0f;
            mCutFrom.reset();
        }
        if (!(mStage == WheelStage::Wind || mStage == WheelStage::Cut))
        {
            mTwistLead = pose->mTwist;
            mShownU = 0.0f;
        }
        if (mStage != WheelStage::Cut)
            mTwistCut.reset();
        if (mStage != WheelStage::Cut)
            mSteerWantX = mSteerWantY = 0.0f;
        if (mStage != WheelStage::Guard)
            mFistHeld = false;
        mPose = *pose;
        stepDrivePhase(input, striker, dt);
        stepKnock(input, striker, dt);
        mPrevPose = mLastPose;
        mLastPose = mPose;
        stepTrunk(input, striker, actor, dt);
        return mPose;
    }

    void WheelPlanner::beginKnock(const JPH::Vec3& worldOffset, bool endStroke, bool chain, float control)
    {
        if (worldOffset.IsNearZero())
            return;
        const float skillLoss = std::clamp((1.0f - control) / 0.8f, 0.0f, 1.0f);
        mKnock = worldOffset;
        mKnockTime = 0.0f;
        mKnockRecovery = (0.35f + 0.35f * skillLoss) * (chain ? 0.5f : 1.0f);
        mKnockAbort = mKnockAbort || endStroke;
        mKnockChain = endStroke && chain;
    }

    void WheelPlanner::stepDrivePhase(const WheelInput& input, const WheelStriker& striker, float dt)
    {
        // Stiff while the cut drives (to two thirds), soft in the
        // follow-through where the weapon's weight carries it (two-handers
        // firm: a soft heavy blade swung 50-140 deg past the cut's end).
        constexpr float cutStiff = 1.5f; // HSC_CUT_STIFF
        const float followSoft = striker.mTwoHanded ? 1.0f : 0.6f; // HSC_FOLLOW_SOFT(_TWO)
        float phase = 1.0f;
        if (mStage == WheelStage::Cut)
        {
            const float cu = mFlow.mU - 1.0f;
            phase = cutStiff + (followSoft - cutStiff) * std::clamp((cu - 0.65f) / 0.25f, 0.0f, 1.0f);
            // Firm again once the follow-through has spent itself (0.15 s at
            // the cut's end, over as long again: HSC_FOLLOW_HOLD).
            mFollowHeld = mFlow.mU >= 1.99f ? mFollowHeld + dt : 0.0f;
            phase += (1.0f - phase) * std::clamp(mFollowHeld / 0.15f - 1.0f, 0.0f, 1.0f);
            // Firm until the real tip has caught up within 40 (HSC_CATCH_UP).
            if (input.mTipGap && std::isfinite(*input.mTipGap) && *input.mTipGap > 40.0f)
                phase = std::max(phase, cutStiff);
        }
        else
            mFollowHeld = 0.0f;
        mDrivePhase += std::clamp(phase - mDrivePhase, -8.0f * dt, 8.0f * dt);
    }

    void WheelPlanner::stepKnock(const WheelInput& input, const WheelStriker& striker, float dt)
    {
        if (mKnockTime < 0.0f)
            return;
        if (mKnockAbort && !mBlocking && (mStage == WheelStage::Cut || mStage == WheelStage::Wind))
        {
            // The stroke is spent: it returns from where the clash left it,
            // or (the even bounce) the rebound is the next chain point.
            mStage = mKnockChain ? WheelStage::Guard : WheelStage::Recover;
            mRecover = 0.0f;
            mRecoverFrom = mPose;
            if (mKnockChain)
                mChainFrom = mPose;
            else
                mChainFrom.reset();
            mAuto = false;
            mFlow = {};
            mStanceX = mStanceY = 0.0f;
            mCutFrom.reset();
        }
        mKnockAbort = false;
        mKnockTime += dt;
        constexpr float out = 0.07f;
        const float t = mKnockTime;
        float envelope = 0.0f;
        if (t < out)
        {
            const float u = t / out;
            envelope = u * (2.0f - u);
        }
        else if (t < out + mKnockRecovery)
        {
            const float u = (t - out) / mKnockRecovery;
            envelope = 1.0f - u * u * (3.0f - 2.0f * u);
        }
        else
            mKnockTime = -1.0f;
        mPose.mHand += (input.mChestFromWorld * mKnock) * envelope;
        // The blade is let go while it flies (two-handers less: a loose heavy
        // blade pulls the support hand off).
        mDrivePhase = std::min(mDrivePhase, 1.0f - (striker.mTwoHanded ? 0.3f : 0.5f) * envelope);
    }

    void WheelPlanner::stepTrunk(const WheelInput& input, const WheelStriker& striker, const WheelActor& actor, float dt)
    {
        const bool attacking = mStage == WheelStage::Wind || mStage == WheelStage::Cut;
        // A sideways cut turns the chest more than its keys (play-6 bookmark:
        // "a lot more chest rotation when swinging sideways"): the keys' turn
        // x (1 + K2_SIDE_TURN x cos^2 of the line's angle), a forehand or a
        // backhand all of it, a diagonal half, a chop or a thrust none. The
        // share eases at 4/s, so a change of line or the return to the guard
        // never jumps the chest. Weapons only (a punch has its own turn).
        {
            float want = 0.0f;
            if (actor.mSideTurn > 0.0f && striker.mLines != &fistLines() && mStage != WheelStage::Guard)
            {
                const float c = std::cos(mStage == WheelStage::Recover ? mCutAngle : mAngle);
                want = actor.mSideTurn * c * c;
            }
            if (!std::isfinite(want))
                want = 0.0f;
            mSideShare += std::clamp(want - mSideShare, -4.0f * dt, 4.0f * dt);
        }
        mTrunk.mTwist = mTwistCut.value_or(mPose.mTwist) * (1.0f + mSideShare);
        mTrunk.mTwistLead = mTwistLead * (1.0f + mSideShare);
        mTrunk.mLean = 0.0f;
        mTrunk.mRoll = 0.0f;
        // The thrust's drive in the trunk (THRUST-02): bent forward 0.22 rad
        // as it drives, back a little in the wind-up; eased in at 10/s, out at 3.
        {
            const float rate = (std::abs(mThrustDriveWant) > std::abs(mTrunk.mThrustDrive) ? 10.0f : 3.0f) * dt;
            mTrunk.mThrustDrive += std::clamp(mThrustDriveWant - mTrunk.mThrustDrive, -rate, rate);
            if (!std::isfinite(mTrunk.mThrustDrive))
                mTrunk.mThrustDrive = 0.0f;
            mTrunk.mLean -= mTrunk.mThrustDrive * 0.22f;
        }
        // The cut's momentum turns the chest on past its end, the way the
        // blade went across: up to 18 deg at a 1200 u/s tip (from 30%), in
        // 0.08 s, back over 0.6 s (HSC_FOLLOW_TURN). Chops and thrusts stay square.
        {
            constexpr float degrees = 18.0f, seconds = 0.6f;
            const bool cutEnded = mStage == WheelStage::Cut && mFollowTurnPrevCut < 1.0f && mCut >= 1.0f;
            mFollowTurnPrevCut = mStage == WheelStage::Cut ? mCut : 0.0f;
            if (cutEnded)
            {
                const float across = -std::cos(mCutAngle);
                if (std::abs(across) > 0.2f && std::isfinite(input.mSwingSpeed))
                {
                    mFollowTurnAmplitude = degrees * (std::numbers::pi_v<float> / 180.0f) * across
                        * std::clamp(input.mSwingSpeed / 1200.0f, 0.3f, 1.0f);
                    mFollowTurnTime = 0.0f;
                }
            }
            if (mFollowTurnAmplitude != 0.0f)
            {
                mFollowTurnTime += dt;
                const float t = mFollowTurnTime, rise = 0.08f, fall = std::max(0.1f, seconds - rise);
                const float envelope = smoothStep(t < rise ? t / rise : 1.0f - std::clamp((t - rise) / fall, 0.0f, 1.0f));
                mTrunk.mTwist += mFollowTurnAmplitude * envelope;
                mTrunk.mTwistLead += mFollowTurnAmplitude * envelope;
                if (t >= rise + fall || !std::isfinite(mFollowTurnAmplitude))
                    mFollowTurnAmplitude = 0.0f;
            }
        }
        // The torso helps the hand reach: in an attack, while the physical
        // hand trails its target by over 2, the chest turns toward it (0.5 rad
        // at most), bends forward (0.35) or back (0.15) and tilts sideways
        // (0.35) at 1.5 rad/s; otherwise it comes back (HSC_TORSO_ASSIST).
        {
            const float rate = 1.5f * dt;
            const auto ease = [&](float& v) { v -= std::clamp(v, -rate, rate); };
            if (input.mHand && attacking)
            {
                const JPH::Vec3 t = mPose.mHand, h = *input.mHand;
                if ((t - h).Length() > 2.0f)
                {
                    const auto wrapPi = [](float a) {
                        constexpr float pi = std::numbers::pi_v<float>;
                        while (a > pi)
                            a -= 2.0f * pi;
                        while (a < -pi)
                            a += 2.0f * pi;
                        return a;
                    };
                    const float bearing = wrapPi(std::atan2(t.GetX(), t.GetY()) - std::atan2(h.GetX(), h.GetY()));
                    const float height = std::atan2(t.GetZ(), std::hypot(t.GetX(), t.GetY()))
                        - std::atan2(h.GetZ(), std::hypot(h.GetX(), h.GetY()));
                    const float forward = (t.GetY() - h.GetY()) - (t.GetZ() - h.GetZ());
                    mAssistTwist += std::clamp(0.5f * bearing, -rate, rate);
                    mAssistLean += std::clamp(-0.02f * forward, -rate, rate);
                    mAssistRoll += std::clamp(0.5f * height, -rate, rate);
                }
                else
                {
                    ease(mAssistTwist);
                    ease(mAssistLean);
                    ease(mAssistRoll);
                }
            }
            else if (!attacking)
            {
                ease(mAssistTwist);
                ease(mAssistLean);
                ease(mAssistRoll);
            }
            mAssistTwist = std::clamp(mAssistTwist, -0.5f, 0.5f);
            mAssistLean = std::clamp(mAssistLean, -0.35f, 0.15f);
            mAssistRoll = std::clamp(mAssistRoll, -0.35f, 0.35f);
            mTrunk.mAssistTwist = mAssistTwist;
            mTrunk.mAssistLean = mAssistLean;
            mTrunk.mAssistRoll = mAssistRoll;
            mTrunk.mTwist += mAssistTwist;
            mTrunk.mTwistLead += mAssistTwist;
            mTrunk.mLean += mAssistLean;
            mTrunk.mRoll += mAssistRoll;
        }
        // The body follows the foe assist: in an attack the chest turns toward
        // an aimed foe (0.4 rad at most) and bends toward one up to 45 past the
        // blade's sweet spot (0.3 at most), at 3 rad/s (HSC_FOE_BODY 1, 0.006).
        {
            float wantTwist = 0.0f, wantLean = 0.0f;
            if (attacking && mFoe && mAim.mFoeAssist > 0.0f)
            {
                const float fx = mFoe->GetX(), fy = mFoe->GetY();
                const float flat = std::hypot(fx, fy);
                wantTwist = std::clamp(std::atan2(fx, fy) * mAim.mFoeAssist, -0.4f, 0.4f);
                const float excess = flat - (35.0f + 0.55f * striker.mTip);
                const float window = excess < 25.0f ? excess : std::max(0.0f, 25.0f - (excess - 25.0f) * 1.25f);
                wantLean = -std::clamp(0.006f * window * mAim.mFoeAssist, 0.0f, 0.3f);
            }
            if (!std::isfinite(wantTwist) || !std::isfinite(wantLean))
                wantTwist = wantLean = 0.0f;
            const float rate = 3.0f * dt;
            mFoeTwist += std::clamp(wantTwist - mFoeTwist, -rate, rate);
            mFoeLean += std::clamp(wantLean - mFoeLean, -rate, rate);
            mTrunk.mTwist += mFoeTwist;
            mTrunk.mTwistLead += mFoeTwist;
            mTrunk.mLean += mFoeLean;
        }
        // The low stance: the trunk bends 0.6 rad further at a full low cut,
        // at 2.5/s, held over a downed foe within the tip + 60 ahead even
        // before the attack (DOWN-01).
        {
            bool overDowned = false;
            if (mFoeDowned && mFoe)
                overDowned = std::hypot(mFoe->GetX(), mFoe->GetY()) < striker.mTip + 60.0f && mFoe->GetY() > 0.0f;
            const float want = overDowned ? 1.0f : attacking ? mAim.mLowWant : 0.0f;
            mTrunk.mLow += std::clamp(want - mTrunk.mLow, -2.5f * dt, 2.5f * dt);
            if (!std::isfinite(mTrunk.mLow))
                mTrunk.mLow = 0.0f;
            mTrunk.mLean -= mTrunk.mLow * 0.6f;
        }
        // The hips take their share of the turn (K2_HIP_TURN; play-6 bookmark,
        // "a lot more chest rotation when swinging sideways"): the spine's
        // three joints twist 53 deg at most, so a turn past that stopped at
        // the spine. The pelvis turns over the planted legs by `share` of the
        // lead turn (0.45 rad at most, inside the hips' 0.6), and the spine
        // is asked only the rest: the chest's turn from the actor stays the
        // keys'. The engine turns the root by mHipTurn.
        {
            // In a cut and its follow-through the cap rises to K2_CUT_HIP_CAP
            // (the hips' range is 0.6), so the chest turns further past the
            // stroke's end; it eases back in the recovery.
            const bool through = mStage == WheelStage::Cut || mFollowTurnAmplitude != 0.0f;
            const float capWant = through && actor.mCutHipCap > 0.45f ? std::min(actor.mCutHipCap, 0.6f) : 0.45f;
            mHipCap += std::clamp(capWant - mHipCap, -1.5f * dt, 1.5f * dt);
            float hip = 0.0f;
            if (actor.mHipShare > 0.0f)
                hip = std::clamp(actor.mHipShare * mTrunk.mTwistLead, -mHipCap, mHipCap);
            if (!std::isfinite(hip))
                hip = 0.0f;
            mTrunk.mHipTurn = hip;
            mTrunk.mTwist -= hip;
            mTrunk.mTwistLead -= hip;
        }
    }
}

#include <ragdollcore/stride.hpp>

#include <ragdollcore/stepplanner.hpp>
#include <ragdollcore/unitmodel.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    namespace
    {
        BalanceVec2 flat(const JPH::RVec3& v)
        {
            return { static_cast<float>(v.GetX()), static_cast<float>(v.GetY()) };
        }

        // A landing (x, y) on the ground under it, or the floor's height.
        JPH::RVec3 landingAt(const FootfallInput& footfall, BalanceVec2 at, double floorZ)
        {
            const JPH::RVec3 point(at.mX, at.mY, floorZ);
            const auto ground = footfall.mGround(point, footfall.mActorZ);
            return JPH::RVec3(at.mX, at.mY, ground ? *ground : floorZ);
        }

        MovingFootState movingFoot(const FootfallController::Foot& foot, BalanceVec2 position, bool supported,
            bool pinned)
        {
            MovingFootState out;
            out.mPosition = position;
            out.mSupported = supported;
            out.mPinned = pinned;
            out.mSwinging = foot.mSwinging;
            out.mTarget = flat(foot.mTarget);
            out.mPhase = foot.mPhase;
            out.mDuration = foot.mDuration;
            return out;
        }
    }

    StrideCatchResult stepStrideCatches(
        const StrideCatchInput& input, const StrideCatchSettings& settings, StrideCatchState& state)
    {
        StrideCatchResult result;
        const FootfallInput* footfall = input.mFootfall;
        // While the request moves, the gait keeps stepping after a hit (user
        // bookmarks 1-3, 2026-09-30): the hit part of the stage catch only
        // while it stands; the weapon's footwork always.
        const bool requestMoving = input.mRequestSpeed >= 15.0f;
        const bool hitWindow
            = (settings.mCatchOnHit && (input.mHitWeakened || state.mRecovering) && !requestMoving)
            || input.mSwingFootwork;
        const bool pgcCatch = settings.mStandingCatch && input.mRequestSpeed < 15.0f;
        if (!pgcCatch)
            state.mStanding.reset();
        const bool pgcLanding = settings.mLanding && (state.mLandingTouchdown || state.mLanding.active());
        if (!pgcLanding)
            state.mLandingRootFloor = 0.0f;
        const bool pgcMoving = settings.mMovingGain > 0.0f && settings.mStandingCatch && !pgcCatch && !hitWindow;
        if (!pgcMoving)
            state.mMoving.reset();
        if (!(settings.mCatchSteps || hitWindow || pgcCatch || pgcMoving || pgcLanding) || !footfall
            || !footfall->mEligible || !input.mHaveBody || !footfall->mGround || !footfall->mLegs[0]
            || !footfall->mLegs[1] || !input.mPelvisKnown)
        {
            state.mRecovering = false;
            return result;
        }
        const float dt = footfall->mDt;
        if (!input.mPelvis || !(dt > 0.0f))
            return result;
        const JPH::RVec3 pelvis = *input.mPelvis;
        std::array<BalanceVec2, 2> feet;
        std::array<bool, 2> supported{ false, false };
        float legReach = 0.0f;
        for (std::size_t side = 0; side < 2; ++side)
        {
            if (!input.mFootWorld[side])
                return result;
            const JPH::RVec3 foot = *input.mFootWorld[side];
            feet[side] = flat(foot);
            const auto ground = footfall->mGround(foot, footfall->mActorZ);
            supported[side] = ground && foot.GetZ() - *ground < 12.0;
            // A foot in contact with supporting ground stands whatever the
            // probe says (the probe's window around the capsule read both
            // planted feet of a body sagging on a slope as floating).
            if (settings.mContactSupport && input.mFootContact && (*input.mFootContact)[side])
                supported[side] = true;
            if (const auto& geometry = footfall->mLegs[side]->mGeometry)
                legReach = std::max(legReach,
                    (geometry->mUpperOffset.Length() + geometry->mLowerOffset.Length())
                        / std::max(footfall->mFrame.mInverseUniformScale, 1e-3f));
        }
        if (!(legReach > 0.0f))
            legReach = 70.0f;
        const auto floorGround = footfall->mGround(pelvis, footfall->mActorZ);
        const double floorZ = floorGround ? *floorGround : footfall->mActorZ;
        // An attack's planted shift (FootfallInput::mPlantedShift) is neither a
        // disturbance nor the body off its feet: the catches read the body
        // without it.
        const BalanceVec2 planted = { footfall->mPlantedShift.GetX(), footfall->mPlantedShift.GetY() };
        JPH::Vec3 disturbance = input.mBodyVelocity - footfall->mCommandVelocity - footfall->mPlantedShiftVelocity;
        disturbance.SetZ(0.0f);
        BalanceState body;
        body.mCenterOfMass = flat(pelvis);
        body.mVelocity = { disturbance.GetX(), disturbance.GetY() };
        body.mHeight = static_cast<float>(pelvis.GetZ() - floorZ) + 15.0f;
        if (settings.mRealCom && input.mSituationCom)
        {
            body.mCenterOfMass = flat(*input.mSituationCom);
            body.mHeight = std::max(20.0f, static_cast<float>(input.mSituationCom->GetZ() - floorZ));
        }
        body.mCenterOfMass.mX -= planted.mX;
        body.mCenterOfMass.mY -= planted.mY;
        body.mGravity = UnitModel::GravityUnitsPerSecondSquared;
        const auto capture = capturePoint(body);
        if (!capture)
            return result;
        const BalanceVec2 bodyVelocity{ input.mBodyVelocity.GetX() - footfall->mPlantedShiftVelocity.GetX(),
            input.mBodyVelocity.GetY() - footfall->mPlantedShiftVelocity.GetY() };
        const auto trace = [&](const char* branch, const BalanceState& traced, int stage, int steps, bool swingNow,
                               bool release) {
            result.mTraces.push_back({ branch, traced, feet, supported, stage, steps, swingNow, release });
        };

        // Stuck lean (10-10 PGC matrix): still and leaning past `upDot` for
        // `stepAfter` s, the foot farthest from the body steps under it;
        // still leaning at `fallAfter` s, the body falls.
        const auto stuckStep = [&]() -> bool {
            const auto& stuck = settings.mStuck;
            const auto& feetNow = input.mFeet;
            const bool leaning = stuck[0] > 0.0f && input.mTorsoUpDot < stuck[0]
                && std::hypot(body.mVelocity.mX, body.mVelocity.mY) < 20.0f && input.mRequestSpeed < 15.0f
                && !feetNow[0].mSwinging && !feetNow[1].mSwinging && (supported[0] || supported[1]);
            state.mStuckLeanTime = leaning ? state.mStuckLeanTime + dt : 0.0f;
            if (!leaning)
                state.mStuckStepped = false;
            if (leaning && state.mStuckLeanTime >= stuck[2])
            {
                result.mFall = StrideFall{ "stuck",
                    { body.mCenterOfMass.mX - 0.5f * (feet[0].mX + feet[1].mX),
                        body.mCenterOfMass.mY - 0.5f * (feet[0].mY + feet[1].mY) } };
                ++state.mStuckFalls;
                state.mStuckLeanTime = 0.0f;
                return true;
            }
            if (leaning && state.mStuckLeanTime >= stuck[1] && !state.mStuckStepped && supported[0] && supported[1])
            {
                const std::size_t swing
                    = std::hypot(feet[0].mX - body.mCenterOfMass.mX, feet[0].mY - body.mCenterOfMass.mY)
                        >= std::hypot(feet[1].mX - body.mCenterOfMass.mX, feet[1].mY - body.mCenterOfMass.mY)
                    ? 0
                    : 1;
                // Under the body, kept a half stance width to its own side of
                // the line from the other foot to the body.
                const JPH::Vec3 other(feet[1 - swing].mX, feet[1 - swing].mY, 0.0f);
                const JPH::Vec3 centre(body.mCenterOfMass.mX, body.mCenterOfMass.mY, 0.0f);
                const JPH::Vec3 toBody = centre - other;
                const JPH::Vec3 across
                    = JPH::Vec3(-toBody.GetY(), toBody.GetX(), 0.0f).NormalizedOr(JPH::Vec3::sAxisX());
                const JPH::Vec3 own(feet[swing].mX - other.GetX(), feet[swing].mY - other.GetY(), 0.0f);
                const float sign = own.Dot(across) >= 0.0f ? 1.0f : -1.0f;
                const JPH::Vec3 point = centre + across * (sign * 6.0f);
                FootfallCommand command;
                command.mHoldClock = true;
                command.mSwingNow = swing;
                command.mSwingTime = 0.3f;
                command.mLanding[swing] = landingAt(*footfall, { point.GetX(), point.GetY() }, floorZ);
                result.mCommand = command;
                state.mStuckStepped = true;
                ++state.mStuckSteps;
                ++state.mStepCount;
                result.mStuckStepFoot = swing;
                return true;
            }
            return false;
        };

        // A landing runs out its carried speed (PGC landing).
        const auto landingStep = [&]() -> bool {
            if (!pgcLanding)
                return false;
            LandingInput landing;
            landing.mState = body;
            // While the request moves, the run-out reads the body's speed
            // against it (PGC_LANDING_RELATIVE); else the absolute speed.
            if (!(settings.mLandingRelative && requestMoving))
                landing.mState.mVelocity = bodyVelocity;
            for (std::size_t side = 0; side < 2; ++side)
                landing.mFeet[side] = movingFoot(input.mFeet[side], feet[side], supported[side], input.mPinned[side]);
            const CaptureStepLimits limits = catchStepLimits(input.mCapability, legReach);
            landing.mDt = dt;
            landing.mReach = limits.mMaximumReach;
            landing.mMinimumSwingTime = limits.mMinimumSwingTime;
            landing.mTouchdown = state.mLandingTouchdown;
            landing.mImpactSpeed = state.mLandingImpactSpeed;
            state.mLandingTouchdown = false;
            const auto landingResult = state.mLanding.update(landing);
            trace("landing", landing.mState, -1, landingResult ? landingResult->mSteps : -1,
                landingResult && landingResult->mSwingNow, landingResult && landingResult->mRelease);
            if (!landingResult)
            {
                state.mLandingRootFloor = 0.0f;
                return true;
            }
            state.mLandingRootFloor = landingResult->mRootStrengthFloor;
            if (landingResult->mRelease)
            {
                result.mFall = StrideFall{ "landing", {} };
                state.mLanding.reset();
                state.mLandingRootFloor = 0.0f;
                return true;
            }
            FootfallCommand command;
            command.mHoldClock = landingResult->mHoldClock;
            command.mSwingNow = landingResult->mSwingNow;
            command.mSwingTime = landingResult->mSwingTime;
            for (std::size_t side = 0; side < 2; ++side)
                if (landingResult->mLanding[side])
                    command.mLanding[side] = landingAt(*footfall, *landingResult->mLanding[side], floorZ);
            if (landingResult->mSwingNow)
                ++state.mStepCount;
            result.mCommand = command;
            return true;
        };

        // Walking or running, a catch beside the gait (PGC_MOVING_CATCH).
        const auto movingStep = [&]() -> bool {
            if (!pgcMoving)
                return false;
            MovingCatchInput moving;
            moving.mState = body;
            moving.mState.mVelocity = bodyVelocity;
            moving.mCommandedVelocity = { footfall->mCommandVelocity.GetX(), footfall->mCommandVelocity.GetY() };
            for (std::size_t side = 0; side < 2; ++side)
                moving.mFeet[side] = movingFoot(input.mFeet[side], feet[side], supported[side], input.mPinned[side]);
            moving.mDt = dt;
            moving.mReach = catchStepLimits(input.mCapability, legReach).mMaximumReach;
            moving.mSettings.mGain = settings.mMovingGain;
            // No release in the game yet (PGC session: start with it off).
            moving.mSettings.mRelease = false;
            const auto movingResult = state.mMoving.update(moving);
            trace("moving", moving.mState, -1, -1, movingResult && movingResult->mSwingNow,
                movingResult && movingResult->mRelease);
            if (const auto& moved = movingResult)
            {
                FootfallCommand command;
                command.mSwingNow = moved->mSwingNow;
                command.mSwingTime = moved->mSwingTime;
                bool any = moved->mSwingNow.has_value();
                for (std::size_t side = 0; side < 2; ++side)
                    if (moved->mLanding[side])
                    {
                        command.mLanding[side] = landingAt(*footfall, *moved->mLanding[side], floorZ);
                        any = true;
                    }
                if (moved->mTrail)
                    ++state.mMovingTrails;
                if (moved->mSwingNow)
                    ++state.mStepCount;
                if (any)
                    result.mCommand = command;
            }
            return true;
        };

        // Standing, the capture-point catch (PGC B, with H's hover rules).
        const auto standingStep = [&]() -> bool {
            if (!pgcCatch)
                return false;
            CatchStepInput catchInput;
            catchInput.mState = body;
            catchInput.mState.mVelocity = bodyVelocity;
            catchInput.mReferenceVelocity
                = { footfall->mCommandVelocity.GetX(), footfall->mCommandVelocity.GetY() };
            catchInput.mBodyForward = { std::sin(footfall->mYaw), std::cos(footfall->mYaw) };
            for (std::size_t side = 0; side < 2; ++side)
            {
                auto& foot = catchInput.mFeet[side];
                foot.mPosition = feet[side];
                foot.mSupported = supported[side];
                foot.mSwinging = input.mFeet[side].mSwinging;
                foot.mTargetHeld = input.mFeet[side].mTargetHeld;
                foot.mPhase = input.mFeet[side].mPhase;
                foot.mDuration = input.mFeet[side].mDuration;
            }
            catchInput.mDt = dt;
            catchInput.mLimits = catchStepLimits(input.mCapability, legReach);
            catchInput.mReactionDelay = input.mCapability.mReactionDelay;
            if (settings.mHover)
            {
                catchInput.mStepFreeFoot = true;
                catchInput.mReleaseWhenUncatchable = true;
                catchInput.mReleaseAfterSteps = settings.mReleaseAfterSteps;
                catchInput.mArmDriven = input.mArmDriven;
                catchInput.mMaximumHoverTime = 0.5f;
                const JPH::Vec3 request(input.mRawRequest.GetX(), input.mRawRequest.GetY(), 0.0f);
                if (request.Length() > 1.0f)
                    catchInput.mDirectionBias
                        = { request.GetX() / request.Length(), request.GetY() / request.Length() };
            }
            catchInput.mStumbleSteps = state.mStumbleLeft;
            catchInput.mStumbleSpeed = settings.mStumbleSpeed;
            const auto caught = state.mStanding.update(catchInput);
            if (caught && caught->mSwingNow && state.mStumbleLeft > 0)
            {
                --state.mStumbleLeft;
                if (caught->mStumble)
                    ++state.mStumbleSteps;
            }
            trace("catch", catchInput.mState, caught ? static_cast<int>(caught->mStage) : -1,
                static_cast<int>(state.mStanding.stepsStarted()), caught && caught->mSwingNow,
                caught && caught->mRelease);
            if (caught && caught->mRelease)
            {
                // An upright body not moving off its request is not falling
                // (10-30 play: two NPCs dropped every 0.6 s while standing at
                // up 0.999): the catch ends instead (CATCH_VETO).
                const JPH::Vec3 off(input.mBodyVelocity.GetX() - footfall->mCommandVelocity.GetX(),
                    input.mBodyVelocity.GetY() - footfall->mCommandVelocity.GetY(), 0.0f);
                if (settings.mVeto[0] > 0.0f && input.mTorsoUpDot >= settings.mVeto[0]
                    && off.Length() <= settings.mVeto[1])
                {
                    ++state.mVetoes;
                    result.mVetoed = true;
                    result.mVetoSpeed = off.Length();
                    state.mStanding.reset();
                    return true;
                }
                // No step can catch it: fall now, as a topple does.
                result.mFall = StrideFall{ "catch", caught->mFallDirection };
                ++state.mReleases;
                state.mStanding.reset();
                return true;
            }
            if (caught && caught->mHoldClock)
            {
                FootfallCommand command;
                command.mHoldClock = true;
                command.mSwingNow = caught->mSwingNow;
                command.mSwingTime = caught->mSwingTime;
                for (std::size_t side = 0; side < 2; ++side)
                    if (caught->mLanding[side])
                        command.mLanding[side] = landingAt(*footfall, *caught->mLanding[side], floorZ);
                if (caught->mSwingNow)
                    ++state.mStepCount;
                result.mCommand = command;
            }
            return true;
        };

        // After a hit, the stage-only catch (CATCH_STEPS / CATCH_ON_HIT).
        const auto stageStep = [&]() -> bool {
            BalanceStageInput stageInput;
            stageInput.mCapturePoint = *capture;
            stageInput.mLeftFoot = feet[0];
            stageInput.mRightFoot = feet[1];
            stageInput.mLeftSupported = supported[0];
            stageInput.mRightSupported = supported[1];
            stageInput.mPrevious = state.mStage;
            stageInput.mTimeInPrevious = state.mStageTime;
            stageInput.mLimits = { 6.0f, 8.0f, 1.5f, 0.3f };
            if (const auto decision = classifyBalanceStage(stageInput))
            {
                state.mStageTime = decision->mStage == state.mStage ? state.mStageTime + dt : 0.0f;
                state.mStage = decision->mStage;
            }
            if (!state.mRecovering && state.mStage == BalanceStage::Step)
            {
                state.mRecovering = true;
                state.mRecoveryTime = 0.0f;
                state.mHeld = 0.0f;
            }
            trace("hitcatch", body, static_cast<int>(state.mStage), state.mRecovering ? 1 : 0, false, false);
            if (!state.mRecovering)
                return false;
            state.mRecoveryTime += dt;
            const auto& footState = input.mFeet;
            RecoveryEndInput endInput;
            endInput.mCapturePoint = *capture;
            endInput.mVelocity = body.mVelocity;
            endInput.mLeftFoot = feet[0];
            endInput.mRightFoot = feet[1];
            endInput.mLeftSupported = supported[0];
            endInput.mRightSupported = supported[1];
            endInput.mHeld = state.mHeld;
            endInput.mDt = dt;
            endInput.mLimits = { 6.0f, 2.0f, 25.0f, 0.2f };
            const auto end = evaluateRecoveryEnd(endInput);
            state.mHeld = end ? end->mHeld : 0.0f;
            const bool swinging = footState[0].mSwinging || footState[1].mSwinging;
            if ((end && end->mCaught && !swinging) || state.mRecoveryTime > 3.0f)
            {
                state.mRecovering = false;
                return false;
            }
            FootfallCommand command;
            command.mHoldClock = true;
            CaptureStepLimits limits;
            limits.mFootRadius = 6.0f;
            limits.mMaximumReach = 0.8f * legReach;
            limits.mMinimumWidth = 8.0f;
            limits.mMinimumSwingTime = 0.2f;
            limits.mMaximumSwingTime = 0.45f;
            limits.mSwingFootSpeed = 400.0f;
            limits.mCrossoverClearance = 0.0f;
            limits.mBrakeMargin = 0.0f;
            const BalanceVec2 forward{ std::sin(footfall->mYaw), std::cos(footfall->mYaw) };
            const auto planFor = [&](std::size_t swing, std::optional<float> timeToTouchdown) {
                CaptureStepInput stepInput;
                stepInput.mState = body;
                stepInput.mStanceFoot = feet[1 - swing];
                stepInput.mSwingFoot = feet[swing];
                stepInput.mBodyForward = forward;
                stepInput.mSwingIsLeft = swing == 0;
                stepInput.mTimeToTouchdown = timeToTouchdown;
                stepInput.mLimits = limits;
                return planRecoverySteps(stepInput, 3);
            };
            std::optional<std::size_t> inFlight;
            for (std::size_t side = 0; side < 2; ++side)
                if (footState[side].mSwinging)
                    inFlight = side;
            if (inFlight)
            {
                if (!footState[*inFlight].mTargetHeld && supported[1 - *inFlight])
                {
                    const float remaining = std::max(
                        0.05f, (1.0f - footState[*inFlight].mPhase) * footState[*inFlight].mDuration);
                    if (const auto plan = planFor(*inFlight, remaining); plan && plan->mCount > 0)
                        command.mLanding[*inFlight] = landingAt(*footfall, plan->mSteps[0].mLanding, floorZ);
                }
            }
            else if (supported[0] && supported[1] && state.mStage == BalanceStage::Step)
            {
                std::optional<RecoveryPlan> best;
                std::size_t bestSide = 0;
                for (std::size_t side = 0; side < 2; ++side)
                {
                    const auto plan = planFor(side, std::nullopt);
                    if (!plan || plan->mCount == 0)
                        continue;
                    const auto better = [](const RecoveryPlan& a, const RecoveryPlan& b) {
                        if (a.mCaptured != b.mCaptured)
                            return a.mCaptured;
                        if (a.mCount != b.mCount)
                            return a.mCount < b.mCount;
                        return a.mSteps[a.mCount - 1].mResidual < b.mSteps[b.mCount - 1].mResidual;
                    };
                    if (!best || better(*plan, *best))
                    {
                        best = plan;
                        bestSide = side;
                    }
                }
                if (best)
                {
                    command.mSwingNow = bestSide;
                    command.mSwingTime = best->mSteps[0].mSwingTime;
                    command.mLanding[bestSide] = landingAt(*footfall, best->mSteps[0].mLanding, floorZ);
                    ++state.mStepCount;
                }
            }
            result.mCommand = command;
            result.mStageCapture = *capture;
            result.mStageVelocity = body.mVelocity;
            return true;
        };

        for (const StepSource source : settings.mOrder)
        {
            bool took = false;
            switch (source)
            {
                case StepSource::Stuck:
                    took = stuckStep();
                    break;
                case StepSource::Landing:
                    took = landingStep();
                    break;
                case StepSource::MovingCatch:
                    took = movingStep();
                    break;
                case StepSource::StandingCatch:
                    took = standingStep();
                    break;
                case StepSource::StageCatch:
                    took = stageStep();
                    break;
                default:
                    break;
            }
            if (took)
            {
                result.mSource = source;
                return result;
            }
        }
        return result;
    }

    std::optional<FootfallCommand> aimStrideLandings(const StrideAimInput& input)
    {
        const auto& s = input.mSettings;
        if (!(s[0] > 0.0f) || !input.mEligible || input.mStepOwned || input.mCommandVelocity.Length() < 15.0f)
            return std::nullopt;
        if (!input.mMomentum || !(input.mMomentum->mTotalMass > 0.0f))
            return std::nullopt;
        const RagdollMomentum& momentum = *input.mMomentum;
        const auto& feet = input.mFeet;
        FootfallCommand command;
        bool any = false;
        for (std::size_t side = 0; side < 2; ++side)
        {
            // Aimed in the swing's first half only; Footfall then holds that
            // landing (re-aiming to touchdown moved the goal mid-air).
            if (!feet[side].mSwinging || feet[1 - side].mSwinging || feet[side].mPhase > 0.5f)
                continue;
            const auto& stance = input.mFootWorld[1 - side];
            const auto& swing = input.mFootWorld[side];
            if (!stance || !swing)
                continue;
            const double groundZ = feet[1 - side].mPin.GetZ();
            const JPH::RVec3 com = momentum.mCenterOfMass;
            const JPH::Vec3 p = momentum.mLinearMomentum;
            const JPH::Vec3 l = momentum.mAngularMomentum;
            StrideInput plan;
            plan.mState.mCenterOfMass = { static_cast<float>(com.GetX()), static_cast<float>(com.GetY()) };
            plan.mState.mVelocity = { p.GetX() / momentum.mTotalMass, p.GetY() / momentum.mTotalMass };
            plan.mState.mAngularMomentum = strideAngularMomentumAbout(
                { static_cast<float>(com.GetX()), static_cast<float>(com.GetY()), static_cast<float>(com.GetZ()) },
                { p.GetX(), p.GetY(), p.GetZ() }, { l.GetX(), l.GetY(), l.GetZ() },
                { static_cast<float>(stance->GetX()), static_cast<float>(stance->GetY()),
                    static_cast<float>(groundZ) });
            plan.mState.mMass = momentum.mTotalMass;
            plan.mState.mHeight = std::max(20.0f, static_cast<float>(com.GetZ() - groundZ));
            plan.mState.mGravity = UnitModel::GravityUnitsPerSecondSquared;
            plan.mStanceFoot = { static_cast<float>(stance->GetX()), static_cast<float>(stance->GetY()) };
            plan.mSwingFoot = { static_cast<float>(swing->GetX()), static_cast<float>(swing->GetY()) };
            plan.mSwingIsLeft = side == 0;
            plan.mForward = { std::sin(input.mYaw), std::cos(input.mYaw) };
            plan.mDesiredVelocity = { input.mCommandVelocity.GetX(), input.mCommandVelocity.GetY() };
            plan.mElapsed = std::max(0.0f, feet[side].mPhase * feet[side].mDuration);
            StrideLimits limits;
            limits.mMaximumReach = s[1];
            limits.mMinimumWidth = s[2];
            plan.mLimits = strideLimitsFromGait(feet[side].mDuration, 0.6f, 6.0f, limits);
            plan.mLimits.mMaximumSwingTime = std::max(plan.mLimits.mMaximumSwingTime, feet[side].mDuration + 0.1f);
            const auto planned = planStride(plan);
            if (!planned || planned->mGiveUp)
                continue;
            command.mLanding[side] = JPH::RVec3(planned->mLanding.mX, planned->mLanding.mY, groundZ);
            any = true;
        }
        if (!any)
            return std::nullopt;
        return command;
    }

    StrideLeadResult stepStrideLead(
        const StrideLeadInput& input, const StrideLeadSettings& settings, StrideLeadState& state)
    {
        StrideLeadResult result;
        const auto& s = settings.mLead;
        const bool was = state.mLeading;
        state.mLeading = false;
        const auto off = [&] {
            result.mChanged = was;
            return result;
        };
        if (!(s[0] > 0.0f || input.mDodging) || !input.mEligible || !input.mGateOpen)
            return off();
        const JPH::Vec3 request = input.mDodging
            ? input.mDodgeDirection * (input.mDodgeLength / std::max(0.05f, settings.mDodgeTime))
            : JPH::Vec3(input.mRequest.GetX(), input.mRequest.GetY(), 0.0f);
        const auto& grip = settings.mGrip;
        if (grip[0] > 0.0f && !input.mDodging)
        {
            const float asked = request.Length();
            const float error = asked > 1.0f
                ? (request - JPH::Vec3(input.mBodyVelocity.GetX(), input.mBodyVelocity.GetY(), 0.0f)).Length() / asked
                : 1e9f;
            if (!state.mCruising && error <= grip[1])
                state.mCruising = true;
            else if (state.mCruising && error > 2.0f * grip[1])
                state.mCruising = false;
            if (!state.mCruising || (grip[2] > 0.0f && asked > grip[2]))
                return off();
        }
        if (request.Length() < 15.0f || !input.mTelemetry)
            return result;
        StanceForceInput stance;
        stance.mCenterOfMass = input.mCenterOfMass;
        stance.mVelocity = { input.mBodyVelocity.GetX(), input.mBodyVelocity.GetY() };
        stance.mRequestVelocity = { request.GetX(), request.GetY() };
        stance.mPlantedFeet = input.mPlantedFeet;
        stance.mTotalMass = input.mTotalMass;
        stance.mGravity = UnitModel::GravityUnitsPerSecondSquared;
        stance.mSettings.mFrictionShare = std::max(0.0f, input.mDodging ? std::max(s[2], 0.6f) : s[2]);
        stance.mSettings.mDamping = std::max(0.0f, s[3]);
        if (grip[0] > 0.0f && !input.mDodging)
        {
            stance.mSettings.mGripMargin = grip[0];
            float friction = 1e9f;
            stance.mFootLoad = { 0.0f, 0.0f };
            for (const StrideLeadContact& contact : input.mContacts)
            {
                stance.mFootLoad[contact.mSide] += contact.mNormalForce;
                // K2_STRIDE_GRIP: the planted feet's grip only (a swinging
                // foot brushing the floor capped the push at its 0.05 grip).
                if (contact.mFriction && (!settings.mPlantedGrip || stance.mPlantedFeet[contact.mSide]))
                    friction = std::min(friction, *contact.mFriction);
            }
            stance.mFriction = friction < 1e8f ? friction : 0.0f;
        }
        // Between two planted steps the body coasts: the capsule still follows it.
        state.mLeading = true;
        result.mLeading = true;
        result.mChanged = !was;
        const auto force = computeStanceForce(stance);
        if (!force || !force->mActive)
            return result;
        ++state.mSteps;
        result.mForce = force;
        return result;
    }

    std::array<std::optional<SwingSpringOutput>, 2> stepStrideSwingSpring(
        const StrideSpringInput& input, std::array<JPH::Vec3, 2>& goalVelocity)
    {
        std::array<std::optional<SwingSpringOutput>, 2> out;
        for (std::size_t side = 0; side < 2; ++side)
        {
            const StrideSpringFoot& foot = input.mFeet[side];
            if (!foot.mSwinging || !foot.mValid)
                continue;
            SwingSpringInput spring;
            spring.mFoot = foot.mFoot;
            spring.mFootVelocity = foot.mFootVelocity;
            spring.mLiftOff = foot.mLiftOff;
            spring.mLanding = foot.mLanding;
            spring.mPhase = foot.mPhase;
            spring.mDuration = foot.mDuration;
            spring.mLegMass = foot.mLegMass;
            spring.mGravity = UnitModel::GravityUnitsPerSecondSquared;
            spring.mSettings = { input.mFrequency, 1.0f, 3.0f };
            if (input.mScoped && foot.mGoal && input.mScopedDt > 0.0f && input.mPelvisMass)
            {
                JPH::Vec3& velocity = goalVelocity[side];
                if (foot.mSwingStarted)
                    velocity = JPH::Vec3::sZero();
                else
                {
                    JPH::Vec3 raw = foot.mGoalVelocity;
                    raw.SetZ(0.0f);
                    velocity += (raw - velocity) * (1.0f - std::exp(-input.mScopedDt / 0.03f));
                    if (velocity.Length() > 400.0f)
                        velocity = velocity.Normalized() * 400.0f;
                }
                spring.mGoal = flat(*foot.mGoal);
                spring.mGoalVelocity = BalanceVec2{ velocity.GetX(), velocity.GetY() };
                const float pelvisMass = *input.mPelvisMass;
                spring.mImplicitDt = input.mScopedDt;
                spring.mEffectiveMass = foot.mFootMass * pelvisMass / std::max(1e-3f, foot.mFootMass + pelvisMass);
            }
            else
                goalVelocity[side] = JPH::Vec3::sZero();
            out[side] = computeSwingSpring(spring);
        }
        return out;
    }

    StrideBraceResult stepStrideBrace(
        const StrideBraceInput& input, const StrideBraceSettings& settings, StrideBraceState& state)
    {
        StrideBraceResult result;
        const FootfallInput& footfall = *input.mFootfall;
        const BalanceVec2 lean = state.mPosture.update(
            uprightTilt(input.mTorsoUp.GetX(), input.mTorsoUp.GetY(), input.mTorsoUp.GetZ()), input.mDt);
        FootfallCommand command;
        command.mStanceHoldsPelvis = true;
        // The root is let go while bracing: stepping toward the anchor walked
        // the feet out from under the body (PGC viewer: 52-65 u backsteps).
        command.mNoAnchorSettle = settings.mNoAnchor;
        command.mPelvisLean = JPH::Vec3(lean.mX, lean.mY, 0.0f);
        float legReach = 0.0f;
        for (std::size_t side = 0; side < 2; ++side)
            if (const auto& geometry = footfall.mLegs[side] ? footfall.mLegs[side]->mGeometry : std::nullopt)
                legReach = std::max(legReach,
                    (geometry->mUpperOffset.Length() + geometry->mLowerOffset.Length())
                        / std::max(footfall.mFrame.mInverseUniformScale, 1e-3f));
        if (!(legReach > 0.0f))
            legReach = 70.0f;
        const JPH::RVec3 com = input.mCenterOfMass;
        const auto ground = footfall.mGround(com, footfall.mActorZ);
        const double floorZ = ground ? *ground : footfall.mActorZ;
        CatchStepInput catchInput;
        catchInput.mState.mCenterOfMass = flat(com);
        catchInput.mState.mVelocity = { input.mBodyVelocity.GetX(), input.mBodyVelocity.GetY() };
        catchInput.mState.mHeight = static_cast<float>(com.GetZ() - floorZ);
        catchInput.mState.mGravity = UnitModel::GravityUnitsPerSecondSquared;
        catchInput.mBodyForward = { std::sin(footfall.mYaw), std::cos(footfall.mYaw) };
        for (std::size_t side = 0; side < 2; ++side)
        {
            auto& foot = catchInput.mFeet[side];
            foot.mPosition = input.mFeetCom[side];
            foot.mSupported = input.mSupported[side];
            foot.mSwinging = input.mFeet[side].mSwinging;
            foot.mTargetHeld = input.mFeet[side].mTargetHeld;
            foot.mPhase = input.mFeet[side].mPhase;
            foot.mDuration = input.mFeet[side].mDuration;
        }
        catchInput.mDt = input.mDt;
        catchInput.mLimits = catchStepLimits(input.mCapability, legReach);
        catchInput.mReactionDelay = input.mCapability.mReactionDelay;
        catchInput.mStepFreeFoot = true;
        catchInput.mReleaseWhenUncatchable = true;
        catchInput.mReleaseAfterSteps = settings.mReleaseAfterSteps;
        catchInput.mArmDriven = input.mArmDriven;
        catchInput.mMaximumHoverTime = 0.5f;
        const JPH::Vec3 request(input.mRawRequest.GetX(), input.mRawRequest.GetY(), 0.0f);
        if (request.Length() > 1.0f)
            catchInput.mDirectionBias = { request.GetX() / request.Length(), request.GetY() / request.Length() };
        catchInput.mStumbleSteps = state.mStumbleLeft;
        catchInput.mStumbleSpeed = settings.mStumbleSpeed;
        const auto caught = state.mCatch.update(catchInput);
        if (caught && caught->mSwingNow && state.mStumbleLeft > 0)
        {
            --state.mStumbleLeft;
            if (caught->mStumble)
                ++state.mStumbleSteps;
        }
        result.mTrace = { "brace", catchInput.mState, input.mFeetCom, input.mSupported,
            caught ? static_cast<int>(caught->mStage) : -1, static_cast<int>(state.mCatch.stepsStarted()),
            caught && caught->mSwingNow, caught && caught->mRelease };
        result.mBody = catchInput.mState;
        if (caught)
        {
            if (caught->mRelease)
            {
                // Nothing can catch it: fall now, as a topple does.
                result.mFall = StrideFall{ "brace", caught->mFallDirection };
                ++state.mReleases;
                state.mCatch.reset();
                state.mPosture.reset();
                return result;
            }
            if (caught->mHoldClock)
            {
                command.mHoldClock = true;
                command.mSwingNow = caught->mSwingNow;
                command.mSwingTime = caught->mSwingTime;
                for (std::size_t side = 0; side < 2; ++side)
                    if (caught->mLanding[side])
                        command.mLanding[side] = landingAt(footfall, *caught->mLanding[side], floorZ);
            }
        }
        result.mCommand = command;
        return result;
    }

    std::optional<StanceForceOutput> strideBraceStance(const BalanceState& body,
        const std::array<BalanceVec2, 2>& feet, const std::array<bool, 2>& supported,
        const std::array<FootfallController::Foot, 2>& feetAfterStep, float totalMass)
    {
        StanceForceInput stance;
        stance.mCenterOfMass = body.mCenterOfMass;
        stance.mVelocity = body.mVelocity;
        for (std::size_t side = 0; side < 2; ++side)
            if (supported[side] && !feetAfterStep[side].mSwinging)
                stance.mPlantedFeet[side] = feet[side];
        stance.mTotalMass = totalMass;
        stance.mGravity = UnitModel::GravityUnitsPerSecondSquared;
        const auto force = computeStanceForce(stance);
        if (!force || !force->mActive)
            return std::nullopt;
        return force;
    }

    bool stepStrideHealth(
        const StrideHealthInput& input, const std::array<float, 2>& settings, StrideHealthState& state)
    {
        if (settings[0] <= 0.0f || state.mFailing)
            return false;
        if (input.mRequestSpeed > 10.0f || input.mBodySpeed > 10.0f || input.mDisturbed)
        {
            state.mStillTime = 0.0f;
            state.mStillSteps = 0;
            return false;
        }
        state.mStillTime += input.mDt;
        if (state.mStillTime < settings[0])
            return false;
        for (const bool started : input.mSwingStarted)
            state.mStillSteps += started ? 1 : 0;
        if (state.mStillSteps >= static_cast<unsigned>(std::max(1.0f, settings[1])))
        {
            state.mFailing = true;
            return true;
        }
        return false;
    }
}

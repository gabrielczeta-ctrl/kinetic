#include <ragdollcore/footfall.hpp>
#include <ragdollcore/tuning.hpp>

#include <ragdollcore/gaitcontroller.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string_view>
#include <iomanip>
#include <span>
#include <sstream>

// Line-for-line port of applyFootfall (task/footfall-motionbricks
// 5898dbd9f9); the comments that explain each rule stay with it there and
// are summarised here. Keep the two in step until the adapter calls this.

namespace RagdollCore
{
    namespace
    {
        constexpr float sPi = 3.14159265f;

        JPH::Vec3 horizontal(JPH::Vec3 v)
        {
            v.SetZ(0.0f);
            return v;
        }

        // weaponDriveSetting: a non-negative finite float, else the fallback.
        float envSetting(const char* name, float fallback)
        {
            const char* text = RagdollCore::tuningText(name);
            const float value = text ? static_cast<float>(std::atof(text)) : fallback;
            return std::isfinite(value) && value >= 0.0f ? value : fallback;
        }

        float smoothstep(float x)
        {
            x = std::clamp(x, 0.0f, 1.0f);
            return x * x * (3.0f - 2.0f * x);
        }
    }

    FootfallSettings FootfallSettings::fromEnvironment()
    {
        FootfallSettings s;
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_GAIT", s.mGait);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_REF", s.mReference);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_CUSHION", s.mCushion);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_TURN", s.mTurn);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_ANCHOR_SETTLE", s.mAnchorSettle);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_STAND_TURN", s.mStandTurn);
        if (const char* value = RagdollCore::tuningText("OPENMW_RAGDOLL_FOOTFALL_SETTLE_STEP"))
            s.mSettleStepMax = std::strtof(value, nullptr);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_CROUCH", s.mCrouch);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_SNEAK", s.mSneak);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_CROUCH_LEGS", s.mCrouchLegs);
        if (const char* value = RagdollCore::tuningText("OPENMW_RAGDOLL_FOOTFALL_STEP_UP"))
            s.mStepUp = std::strtof(value, nullptr);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_EXTEND", s.mExtend);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_EXTEND_I", s.mExtendIntegral);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_LEAN", s.mLean);
        s.mCushion[1] = std::max(s.mCushion[1], 0.02f);
        s.mCushion[2] = std::clamp(s.mCushion[2], 0.0f, 0.9f);
        s.mLateralTime = envSetting("OPENMW_RAGDOLL_FOOTFALL_LATERAL_TIME", s.mLateralTime);
        s.mBackTime = envSetting("OPENMW_RAGDOLL_FOOTFALL_BACK_TIME", s.mBackTime);
        s.mLandTaper = std::clamp(envSetting("OPENMW_RAGDOLL_FOOTFALL_LAND_TAPER", s.mLandTaper), 0.0f, 0.9f);
        s.mClipSync = envSetting("OPENMW_RAGDOLL_FOOTFALL_CLIP_SYNC", s.mClipSync);
        s.mMoveWidth = envSetting("OPENMW_RAGDOLL_FOOTFALL_MOVE_WIDTH", s.mMoveWidth);
        s.mClipWidthMoving = envSetting("OPENMW_RAGDOLL_FOOTFALL_CLIP_WIDTH", 1.0f) > 0.0f;
        s.mLeadScale = envSetting("OPENMW_RAGDOLL_FOOTFALL_LEAD_SCALE", 1.0f) > 0.0f;
        s.mSideCorrection = envSetting("OPENMW_RAGDOLL_FOOTFALL_SIDE_CORRECTION", s.mSideCorrection);
        s.mRunPathGain = envSetting("OPENMW_RAGDOLL_FOOTFALL_RUN_PATH_GAIN", s.mRunPathGain);
        s.mTreadSnap = envSetting("OPENMW_RAGDOLL_FOOTFALL_TREAD_SNAP", s.mTreadSnap);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_STAIR", s.mStair);
        s.mRunLiftGain = envSetting("OPENMW_RAGDOLL_FOOTFALL_RUN_LIFT_GAIN", s.mRunLiftGain);
        s.mFollowPath = envSetting("OPENMW_RAGDOLL_FOOTFALL_PATH", 1.0f) > 0.0f;
        s.mPathMatch = envSetting("OPENMW_RAGDOLL_FOOTFALL_PATH_MATCH", s.mPathMatch);
        s.mRunPathMatch = envSetting("OPENMW_RAGDOLL_FOOTFALL_RUN_PATH_MATCH", s.mRunPathMatch);
        s.mOverrunRun = envSetting("OPENMW_RAGDOLL_FOOTFALL_OVERRUN_RUN", s.mOverrunRun);
        s.mFirstStep = envSetting("OPENMW_RAGDOLL_FOOTFALL_FIRST_STEP", s.mFirstStep);
        s.mRunSwingDamping = envSetting("OPENMW_RAGDOLL_FOOTFALL_RUN_SWING_DAMPING", s.mRunSwingDamping);
        s.mRunSwingStiffness = envSetting("OPENMW_RAGDOLL_FOOTFALL_RUN_SWING_STIFFNESS", s.mRunSwingStiffness);
        s.mLookAhead = envSetting("OPENMW_RAGDOLL_FOOTFALL_LEAD", s.mLookAhead);
        s.mPathLead = envSetting("OPENMW_RAGDOLL_FOOTFALL_PATH_LEAD", s.mPathLead);
        s.mSettle = envSetting("OPENMW_RAGDOLL_FOOTFALL_SETTLE", s.mSettle);
        s.mClearance = envSetting("OPENMW_RAGDOLL_FOOTFALL_CLEARANCE", s.mClearance);
        s.mStartBlend = envSetting("OPENMW_RAGDOLL_FOOTFALL_START_BLEND", s.mStartBlend);
        s.mReach = envSetting("OPENMW_RAGDOLL_FOOTFALL_REACH", s.mReach);
        s.mFootPitch = envSetting("OPENMW_RAGDOLL_FOOTFALL_FOOT_PITCH", s.mFootPitch);
        s.mFootPitchStance = envSetting("OPENMW_RAGDOLL_FOOTFALL_FOOT_PITCH_STANCE", s.mFootPitchStance);
        s.mFootPitchRate = envSetting("OPENMW_RAGDOLL_FOOTFALL_FOOT_PITCH_RATE", s.mFootPitchRate);
        s.mStanceKnee = envSetting("OPENMW_RAGDOLL_FOOTFALL_STANCE_KNEE", s.mStanceKnee);
        s.mRunStanceKnee = envSetting("OPENMW_RAGDOLL_FOOTFALL_RUN_STANCE_KNEE", s.mRunStanceKnee);
        s.mArmedStanceKnee = envSetting("OPENMW_RAGDOLL_FOOTFALL_ARMED_STANCE_KNEE", s.mArmedStanceKnee);
        s.mFootLevel = envSetting("OPENMW_RAGDOLL_FOOTFALL_FOOT_LEVEL", s.mFootLevel);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_STANCE_KNEE_AIR", s.mStanceKneeAir);
        s.mSwingFeedForward = envSetting("OPENMW_RAGDOLL_FOOTFALL_SWING_FF", s.mSwingFeedForward);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_CAPTURE", s.mCapture);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_SLOPE", s.mSlope);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_OVERRUN_PLAN", s.mOverrunPlan);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_CONTACT_LAND", s.mContactLand);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_MOTOR_LEAD", s.mMotorLead);
        s.mPhaseLock = std::clamp(envSetting("OPENMW_RAGDOLL_FOOTFALL_PHASE_LOCK", s.mPhaseLock), 0.0f, 1.0f);
        RagdollCore::tuningList("OPENMW_RAGDOLL_FOOTFALL_LIFT_LEFT", s.mLiftLeft);
        s.mHipRange = envSetting("OPENMW_RAGDOLL_FOOTFALL_HIP_RANGE", s.mHipRange);
        s.mWidthBlend = envSetting("OPENMW_RAGDOLL_FOOTFALL_WIDTH_BLEND", 1.0f) > 0.0f;
        s.mStopLead = envSetting("OPENMW_RAGDOLL_FOOTFALL_STOP_LEAD", s.mStopLead);
        s.mHipGap = envSetting("OPENMW_RAGDOLL_FOOTFALL_HIP_GAP", s.mHipGap);
        return s;
    }

    JPH::RVec3 FootfallFrame::worldPosition(JPH::Vec3Arg model) const
    {
        const JPH::Vec3 scaled = model / mInverseUniformScale;
        return mReferencePosition + mInverseReferenceRotation.Conjugated() * scaled;
    }

    JPH::Vec3 FootfallFrame::modelPosition(JPH::RVec3Arg world) const
    {
        return (mInverseReferenceRotation * JPH::Vec3(world - mReferencePosition)) * mInverseUniformScale;
    }

    JPH::Vec3 FootfallFrame::modelVector(JPH::Vec3Arg world) const
    {
        return (mInverseReferenceRotation * world) * mInverseUniformScale;
    }

    float footfallFootPitch(int clip, bool swinging, float phase)
    {
        static const float curves[3][2][11] = {
            { { 50, 71, 67, 52, 31, 23, 0, -21, -39, -31, -11 }, { -11, 0, 0, 0, 0, 0, 0, 2, 5, 19, 50 } },
            { { 60, 81, 99, 107, 94, 61, 48, 19, 1, -10, -3 }, { -3, -1, 0, 0, 1, 4, 10, 19, 35, 49, 60 } },
            { { 40, 52, 51, 45, 38, 35, 17, 1, -13, -23, -24 }, { -24, -5, 0, 0, 0, 1, 4, 8, 10, 20, 40 } },
        };
        const float* curve = curves[std::clamp(clip, 0, 2)][swinging ? 0 : 1];
        const float x = std::clamp(phase, 0.0f, 1.0f) * 10.0f;
        const std::size_t i = std::min<std::size_t>(static_cast<std::size_t>(x), 9);
        return curve[i] + (curve[i + 1] - curve[i]) * (x - static_cast<float>(i));
    }

    void FootfallController::reset(FootfallOutput& output)
    {
        mSettleCenter.reset();
        for (std::size_t side = 0; side < 2; ++side)
        {
            if (mLegDampingScale[side] != 1.0f || mLegStiffnessScale[side] != 1.0f || mKneeGive[side] != 1.0f)
            {
                output.mSprings[side] = {};
                output.mSpringsChanged[side] = true;
            }
            mLegDampingScale[side] = 1.0f;
            mLegStiffnessScale[side] = 1.0f;
            mFootPitch[side] = 0.0f;
            mKneeGive[side] = 1.0f;
        }
        mPelvisDrop = std::max(0.0f, mPelvisDrop - 0.5f);
        mCushionDrop = 0.0f;
        mCommand = {};
        mFeet = {};
        mReady = false;
        mDoubleSupport = 0.0f;
        mClock = 0.0f;
        mWalking = false;
        mSinceLanding = 0.0f;
        mLiftDue = { false, false };
        mPreviousPelvis.reset();
        mSurplus = JPH::Vec3::sZero();
        mPreviousCommand.reset();
        mTurnAcceleration = JPH::Vec3::sZero();
        mExtendIntegral = 0.0f;
        mCrouchLength = -1.0f;
        mPinned = { false, false };
        mSwinging = { false, false };
        mPreviousRotation.clear();
        mPhysToDrive.reset();
        mHip = {};
        output.mAngularVelocity.clear();
    }

    void FootfallController::step(const FootfallInput& input, RagdollPose& drivePose, FootfallOutput& output)
    {
        const FootfallSettings& settings = mSettings;
        const bool loadPolicy = input.mLoadPolicyActive;
        output.mSwingStarted = { false, false };
        output.mMotorFootGoal = {};
        output.mMotorFootVelocity = { JPH::Vec3::sZero(), JPH::Vec3::sZero() };
        mPinned = { false, false };
        mSwinging = { false, false };
        output.mPinned = { false, false };
        output.mSwinging = { false, false };
        output.mSpringsChanged = { false, false };
        output.mAngularVelocity.clear();
        const float dt = input.mDt;
        auto& drive = drivePose.mBodyTransforms;
        const bool eligible = input.mEligible && input.mLegs[0] && input.mLegs[1] && input.mPhysicsPose
            && input.mGround && dt > 0.0f;
        const std::array<const FootfallLeg*, 2> legs{ input.mLegs[0] ? &*input.mLegs[0] : nullptr,
            input.mLegs[1] ? &*input.mLegs[1] : nullptr };
        const std::size_t pelvis = input.mPelvis;
        bool sized = eligible && pelvis < drive.size() && pelvis < input.mPhysicsPose->mBodyTransforms.size();
        for (const auto* leg : legs)
            sized = sized && leg && leg->mHip < drive.size() && leg->mKnee < drive.size() && leg->mFoot < drive.size()
                && leg->mHip < input.mPhysicsPose->mBodyTransforms.size()
                && leg->mFoot < input.mPhysicsPose->mBodyTransforms.size();
        const JPH::Vec3 upModelRaw = input.mFrame.modelVector(JPH::Vec3::sAxisZ());
        if (!sized || upModelRaw.IsNearZero())
        {
            reset(output);
            return;
        }
        const auto& phys = input.mPhysicsPose->mBodyTransforms;
        const FootfallFrame& frame = input.mFrame;
        const JPH::Vec3 upModel = upModelRaw.Normalized();
        // The frame the feet are held in: the pelvis drive, or only its
        // heading (FootfallSettings::mFootLevel).
        const auto footBase = [&](JPH::QuatArg pelvisRotation) {
            if (settings.mFootLevel <= 0.0f)
                return JPH::Quat(pelvisRotation);
            const JPH::Quat level = (JPH::Quat::sFromTo(pelvisRotation * upModel, upModel) * pelvisRotation).Normalized();
            return settings.mFootLevel >= 1.0f ? level : pelvisRotation.SLERP(level, settings.mFootLevel).Normalized();
        };
        const auto physWorld = [&](std::size_t body) { return frame.worldPosition(phys[body].mTranslation); };
        const auto& trace = input.mTrace;

        const float smoothing = 1.0f - std::exp(-dt * 2.0f * sPi * 10.0f);
        const JPH::Quat rawPhysToDrive
            = (drive[pelvis].mRotation * phys[pelvis].mRotation.Conjugated()).Normalized().EnsureWPositive();
        mPhysToDrive = mPhysToDrive ? mPhysToDrive->SLERP(rawPhysToDrive, smoothing).Normalized() : rawPhysToDrive;
        const JPH::Quat physToDrive = *mPhysToDrive;
        const JPH::RVec3 physPelvis = physWorld(pelvis);
        std::array<JPH::RVec3, 2> physFeet{ physWorld(legs[0]->mFoot), physWorld(legs[1]->mFoot) };
        // _HIP_GAP: the physical hip line, left hip minus right (horizontal).
        JPH::Vec3 hipLeft(JPH::Vec3(physWorld(legs[0]->mHip) - physWorld(legs[1]->mHip)));
        hipLeft.SetZ(0.0f);
        const bool hipLineKnown = hipLeft.LengthSq() > 1.0f;
        if (hipLineKnown)
            hipLeft = hipLeft.Normalized();
        // The clip's own foot placement around its pelvis.
        std::array<JPH::Vec3, 2> clipOffset{ JPH::Vec3::sZero(), JPH::Vec3::sZero() };
        if (input.mClipPose && pelvis < input.mClipPose->mBodyTransforms.size())
        {
            const auto& clip = input.mClipPose->mBodyTransforms;
            const JPH::RVec3 clipPelvis = frame.worldPosition(clip[pelvis].mTranslation);
            for (std::size_t side = 0; side < 2; ++side)
                if (legs[side]->mFoot < clip.size())
                {
                    clipOffset[side] = JPH::Vec3(frame.worldPosition(clip[legs[side]->mFoot].mTranslation) - clipPelvis);
                    clipOffset[side].SetZ(0.0f);
                }
        }
        const auto groundAt = [&](const JPH::RVec3& point, double reference) { return input.mGround(point, reference); };
        const double actorZ = input.mActorZ;
        const float yaw = input.mYaw;
        const JPH::Vec3 forward(std::sin(yaw), std::cos(yaw), 0.0f);
        const JPH::Vec3 left(-forward.GetY(), forward.GetX(), 0.0f);
        const JPH::Vec3 command = horizontal(input.mCommandVelocity);
        const float speed = command.Length();
        JPH::Vec3 pelvisVelocity = command;
        if (mPreviousPelvis)
            pelvisVelocity = horizontal(JPH::Vec3(physPelvis - *mPreviousPelvis)) / dt;
        mPreviousPelvis = physPelvis;
        const auto& gait = settings.mGait;
        // WALK-02 (mCapture): the surplus motion, filtered at 4 Hz (the
        // pelvis velocity is a one-step difference), and the capture time.
        const bool captureOn = settings.mCapture[0] > 0.0f;
        const float captureTime = std::sqrt(
            std::clamp(static_cast<float>(physPelvis.GetZ() - actorZ), 40.0f, 100.0f) / (9.81f * sUnitsPerMetre));
        if (captureOn)
            mSurplus += (horizontal(pelvisVelocity - command) - mSurplus) * (1.0f - std::exp(-dt * 2.0f * sPi * 4.0f));
        const float legReach = legs[0]->mGeometry
            ? legs[0]->mGeometry->mUpperOffset.Length() + legs[0]->mGeometry->mLowerOffset.Length()
            : 72.0f;

        // Turning: a foot planted outward by a * h / g holds the sideways acceleration.
        JPH::Vec3 turnOffset = JPH::Vec3::sZero();
        {
            const auto& turn = settings.mTurn;
            JPH::Vec3 sideways = JPH::Vec3::sZero();
            if (mPreviousCommand && speed > 15.0f)
            {
                const JPH::Vec3 acceleration = (command - *mPreviousCommand) / dt;
                const JPH::Vec3 travel = command / speed;
                sideways = acceleration - travel * acceleration.Dot(travel);
            }
            mPreviousCommand = command;
            mTurnAcceleration += (sideways - mTurnAcceleration) * (1.0f - std::exp(-dt * 2.0f * sPi * 3.0f));
            const float gravity = 9.81f * sUnitsPerMetre;
            const float hipHeight = std::clamp(static_cast<float>(physPelvis.GetZ() - actorZ), 40.0f, 100.0f);
            turnOffset = -mTurnAcceleration * (turn[0] * hipHeight / gravity);
            if (turnOffset.Length() > turn[1])
                turnOffset = turnOffset.Normalized() * turn[1];
        }

        // Standing: learn the foot height, the stance width and the foot's
        // orientation relative to the pelvis.
        if (speed < 10.0f && !mFeet[0].mSwinging && !mFeet[1].mSwinging)
        {
            float width = 0.0f;
            for (std::size_t side = 0; side < 2; ++side)
            {
                if (const auto ground = groundAt(physFeet[side], actorZ))
                {
                    const float height = static_cast<float>(physFeet[side].GetZ() - *ground);
                    if (height > 0.0f && height < 15.0f)
                        mRestHeight = mRestHeight ? *mRestHeight + (height - *mRestHeight) * 0.05f : height;
                }
                width += std::abs(JPH::Vec3(physFeet[side] - physPelvis).Dot(left)) * 0.5f;
                mFootRotation[side] = (footBase(drive[pelvis].mRotation).Conjugated() * drive[legs[side]->mFoot].mRotation).Normalized();
            }
            width = std::clamp(width, 6.0f, 14.0f);
            mHalfWidth = mHalfWidth ? *mHalfWidth + (width - *mHalfWidth) * 0.05f : width;
        }
        const float restHeight = mRestHeight.value_or(6.5f);
        const float halfWidth = mHalfWidth.value_or(10.0f) * gait[2];

        const bool procedural = loadPolicy && settings.mLoadProceduralStride.has_value();
        const float strideScale = procedural ? *settings.mLoadProceduralStride : 1.0f;
        float stepLength = std::clamp(12.0f + gait[0] * speed * strideScale, 20.0f, std::max(20.0f, gait[3]));
        float period = speed > 15.0f ? std::clamp(stepLength / speed, 0.26f, 0.6f) : 0.45f;
        float liftHeight = std::clamp(5.0f + 0.05f * speed, 5.0f, 16.0f) * gait[1];
        if (std::isfinite(input.mMinimumSwingLift) && input.mMinimumSwingLift > 0.0f && input.mMinimumSwingLift <= 20.0f)
            liftHeight = std::max(liftHeight, input.mMinimumSwingLift);
        const float running = std::clamp((speed - 140.0f) / 90.0f, 0.0f, 1.0f);
        float duty = 0.62f - 0.24f * running;

        // The reference gait for this speed and direction.
        std::optional<FootfallGaitSample> reference;
        std::optional<FootfallGaitSample> referenceMirror;
        float referenceWeight = 0.0f;
        float runWeight = 0.0f;
        float forwardness = 1.0f;
        const auto& referenceScales = settings.mReference;
        const FootfallGaitTable* gaitTable = procedural ? nullptr : input.mGaitTable;
        if (gaitTable && speed > 15.0f)
        {
            const bool armed = input.mArmed;
            // Sneaking walks the stealth reference (alpha 0.6 item 6, crouch).
            const bool sneakStyle = input.mSneaking && settings.mSneak[0] > 0.0f && gaitTable->has("walk_stealth");
            const std::string style = sneakStyle ? "walk_stealth"
                : armed && gaitTable->has("walk_boxing")       ? "walk_boxing"
                                                               : "walk";
            const float travelAngle = std::atan2(command.Dot(left), command.Dot(forward)) * 180.0f / sPi;
            forwardness = std::abs(std::cos(travelAngle * sPi / 180.0f));
            const auto& geometry = legs[0]->mGeometry;
            const float reach = geometry ? geometry->mUpperOffset.Length() + geometry->mLowerOffset.Length() : 72.0f;
            const float legMetres = reach / sUnitsPerMetre;
            const float speedMetres = speed / sUnitsPerMetre;
            const auto sampleAt = [&](float angle, float& runShare) -> std::optional<FootfallGaitSample> {
                auto walk = gaitTable->sample(style, angle, speedMetres, legMetres, sUnitsPerMetre);
                runShare = 0.0f;
                if (!walk)
                    return walk;
                const float walkMaximum = walk->mMaximumFroude;
                if (walk->mFroude > walkMaximum)
                    walk->mSwingTime *= std::sqrt(walkMaximum / walk->mFroude);
                if (armed)
                {
                    const float ramp = std::clamp((walk->mFroude - 0.6f) / 0.3f, 0.0f, 1.0f);
                    runShare = ramp * ramp * (3.0f - 2.0f * ramp);
                }
                if (!armed && !sneakStyle && gaitTable->has("run"))
                    if (const auto run = gaitTable->sample("run", angle, speedMetres, legMetres, sUnitsPerMetre))
                    {
                        const float ramp = std::clamp((walk->mFroude - walkMaximum - 0.15f) / 0.25f, 0.0f, 1.0f);
                        runShare = ramp * ramp * (3.0f - 2.0f * ramp);
                        if (runShare > 0.0f)
                            return FootfallGaitSample::mix(*walk, *run, runShare);
                    }
                return walk;
            };
            reference = sampleAt(travelAngle, runWeight);
            float mirrorRun = 0.0f;
            referenceMirror = sampleAt(-travelAngle, mirrorRun);
            if (reference)
            {
                const float sideways = std::sqrt(std::max(0.0f, 1.0f - forwardness * forwardness));
                reference->mSwingTime *= 1.0f - (1.0f - settings.mLateralTime) * sideways;
                // Backward: shorter, quicker steps - a leg reaches less far
                // behind the body than in front of it.
                const float backward = std::max(0.0f, -std::cos(travelAngle * sPi / 180.0f));
                reference->mSwingTime *= 1.0f - (1.0f - settings.mBackTime) * backward;
            }
            if (reference)
                referenceWeight = 1.0f;
        }
        float referenceSwing = 0.0f;
        if (reference && referenceWeight > 0.0f)
        {
            const float cycle = reference->mSwingTime / std::max(0.2f, 1.0f - reference->mDuty);
            const float referenceDuty = std::max(
                reference->mDuty, referenceScales[0] * (0.72f + 0.28f * forwardness) * (1.0f - runWeight));
            referenceSwing = (1.0f - referenceDuty) * cycle;
            const auto mix = [&](float procedural, float table) { return procedural + (table - procedural) * referenceWeight; };
            period = mix(period, 0.5f * cycle);
            duty = mix(duty, referenceDuty);
            stepLength = std::max(20.0f, speed * period);
        }

        // The clip's stride: cadence and phase from the clip's feet.
        mClipStrideTime += dt;
        mClipStrideCorrection = 0.0f;
        {
            const float clipSync = settings.mClipSync;
            const JPH::Vec3 along = speed > 1e-3f ? command / speed : forward;
            const float d = (clipOffset[0] - clipOffset[1]).Dot(along);
            if (clipSync <= 0.0f || speed <= 15.0f)
            {
                mClipStrideHalf.reset();
                mClipStrideLast.reset();
                mClipStrideRising.reset();
                mClipStridePhaseError = 0.0f;
            }
            else
            {
                if (mClipStridePrevious)
                {
                    const bool rising = d > *mClipStridePrevious;
                    if (mClipStrideRising && rising != *mClipStrideRising
                        && (!mClipStrideLast || std::abs(*mClipStridePrevious - mClipStrideExtremum) > 6.0f))
                    {
                        const float expected = *mClipStrideRising ? 0.5f : 0.0f;
                        if (mClipStrideLast)
                        {
                            const float half = mClipStrideTime - *mClipStrideLast;
                            if (half > 0.18f && half < 0.9f)
                                mClipStrideHalf = mClipStrideHalf ? *mClipStrideHalf + (half - *mClipStrideHalf) * 0.5f : half;
                        }
                        mClipStrideLast = mClipStrideTime;
                        mClipStrideExtremum = *mClipStridePrevious;
                        float error = expected - mClock;
                        error -= std::round(error);
                        mClipStridePhaseError = error * clipSync;
                    }
                    if (!mClipStrideRising || rising != *mClipStrideRising)
                        mClipStrideRising = rising;
                }
                if (mClipStrideHalf)
                {
                    const float scale = *mClipStrideHalf / std::max(period, 0.1f);
                    period = *mClipStrideHalf;
                    referenceSwing *= scale;
                    stepLength = std::max(20.0f, speed * period);
                    mClipStrideCorrection = std::clamp(mClipStridePhaseError, -0.25f, 0.25f);
                    mClipStridePhaseError *= std::exp(-dt / 0.25f);
                }
            }
            mClipStridePrevious = d;
        }

        const float moveWidth = settings.mMoveWidth;
        const float footWidth = speed >= 15.0f ? std::min(halfWidth, moveWidth * (1.0f - 0.3f * runWeight)) : halfWidth;
        // The stance spot laid out along `facing` (the body's heading, or
        // the planted feet's while standing: mStandTurn); the clip's foot
        // offsets are read in the body's frame.
        const auto spotFacing = [&](std::size_t side, const JPH::RVec3& at, JPH::Vec3Arg facing) {
            const JPH::Vec3 facingLeft(-facing.GetY(), facing.GetX(), 0.0f);
            const float sign = side == 0 ? 1.0f : -1.0f;
            const float clipWidth = sign * clipOffset[side].Dot(left);
            float width = clipWidth > 3.0f && (speed < 15.0f || settings.mClipWidthMoving)
                ? std::clamp(clipWidth, 4.0f, 16.0f) * gait[2]
                : halfWidth;
            if (speed >= 15.0f)
            {
                const float narrow = std::min(width, moveWidth * (1.0f - 0.3f * runWeight));
                const float u = std::clamp((speed - 15.0f) / 60.0f, 0.0f, 1.0f);
                width += (narrow - width) * (settings.mWidthBlend ? u * u * (3.0f - 2.0f * u) : 1.0f);
            }
            mDebug[2] = width;
            if (speed < 15.0f && !clipOffset[side].IsNearZero())
            {
                const float along = clipOffset[side].Dot(forward);
                return at + JPH::RVec3(facingLeft * (sign * width) + facing * std::clamp(along, -20.0f, 20.0f));
            }
            return at + JPH::RVec3(facingLeft * (sign * width));
        };
        const auto spot = [&](std::size_t side, const JPH::RVec3& at) { return spotFacing(side, at, forward); };
            // Treads (PGC-03 C9): on stairs the landing moves off a riser edge
            // onto the tread, the whole foot on it (39% of landings on the
            // test stairs had the foot's centre within 8 u of an edge -
            // random placement on 40 u treads). Two probes along the travel
            // find no step on flat ground; on a step the ground is sampled
            // every 4 u and the target shifted at most mTreadSnap.
        // The stair gait's probe fix (mStair[5]): the ground probe sees from
        // 40 u below the capsule to 30 above; descending tall stairs the
        // capsule rides a riser above the body's ground and a landing a tread
        // further down was missed (the target set at the capsule's height,
        // the foot hovering). A miss retries from the foot's own height - on
        // a ledge the foot stands at the capsule's: the same window.
        const bool stairOn = settings.mStair[0] > 0.0f;
        const bool probeFix = stairOn && settings.mStair[5] > 0.0f;
        int probeMisses = 0;
        const auto groundNear = [&](const JPH::RVec3& point, const JPH::RVec3& foot) -> std::optional<double> {
            auto ground = groundAt(point, actorZ);
            if (ground || !probeFix)
                return ground;
            // Follow the ground from under the foot toward the point, 8 u at
            // a time, each step at most a riser (30 u) up or down: a
            // staircase is found however far it goes down, a ledge (a drop
            // past a riser) is not - as before, the capsule's height.
            auto at = groundAt(foot, foot.GetZ() - restHeight);
            if (!at)
                return std::nullopt;
            const JPH::Vec3 way = horizontal(JPH::Vec3(point - foot));
            const int steps = std::clamp(static_cast<int>(std::ceil(way.Length() / 8.0f)), 1, 16);
            for (int k = 1; k <= steps; ++k)
            {
                const auto next = groundAt(foot + JPH::RVec3(way * (static_cast<float>(k) / steps)), *at);
                if (!next || std::abs(*next - *at) > 30.0)
                    return std::nullopt;
                at = next;
            }
            ++probeMisses;
            return at;
        };
        // WALK-02 slopes (mSlope): the ground's normal under a foot point
        // (straight up without a probe or on walls), and the foot centre's
        // height over the ground with the foot tilted onto it.
        const bool slopeOn = settings.mSlope[0] > 0.0f && static_cast<bool>(input.mGroundNormal);
        const auto normalAt = [&](const JPH::RVec3& point) -> JPH::Vec3 {
            if (!slopeOn)
                return JPH::Vec3::sAxisZ();
            const auto normal = input.mGroundNormal(point, point.GetZ() - restHeight);
            if (!normal || normal->IsNearZero() || normal->Normalized().GetZ() < 0.5f)
                return JPH::Vec3::sAxisZ();
            return normal->Normalized();
        };
        const auto restOn = [&](JPH::Vec3Arg normal) {
            if (!slopeOn)
                return restHeight;
            const float share = std::min(settings.mSlope[0], 1.0f);
            return restHeight * (1.0f + share * (1.0f / std::max(normal.GetZ(), 0.8f) - 1.0f));
        };
        // The planar finish: the scoped recovery's (load policy, no path) or a
        // step down's (mStair[1]); 1 = none.
        const auto planarFinishOf = [&](const Foot& foot) {
            if (foot.mStepDown && settings.mStair[1] < 1.0f)
                return settings.mStair[1];
            if (loadPolicy && !foot.mPathCurve && settings.mLoadPlanarFinish < 1.0f)
                return settings.mLoadPlanarFinish;
            return 1.0f;
        };
        const auto onTread = [&](JPH::RVec3 target, std::size_t side) {
            if (settings.mTreadSnap > 0.0f)
            {
                const JPH::Vec3 along = speed > 1.0f ? command / speed : forward;
                const JPH::RVec3 footAt = physFeet[side];
                const auto heightAt = [&](float offset) -> std::optional<double> {
                    return groundNear(target + JPH::RVec3(along * offset), footAt);
                };
                // Hysteresis: a raw target a few units either side of a riser
                // (pelvis-velocity noise) flipped the snap between the treads
                // every step - 18 u along and a riser up or down.
                if (const auto& kept = mFeet[side].mTreadZ)
                {
                    const auto now = heightAt(0.0f);
                    if (now && std::abs(*now - *kept) > 4.0)
                        for (const float offset : { 6.0f, -6.0f })
                            if (const auto h = heightAt(offset); h && std::abs(*h - *kept) <= 4.0)
                            {
                                target += JPH::RVec3(along * offset);
                                break;
                            }
                }
                const auto here = heightAt(0.0f);
                const auto before = heightAt(-12.0f), after = heightAt(12.0f);
                const auto differs = [&](const std::optional<double>& a) {
                    return here && a && std::abs(*a - *here) > 4.0;
                };
                if (here && (differs(before) || differs(after)))
                {
                    constexpr float spacing = 4.0f;
                    constexpr int samples = 6; // each way
                    float edgeBehind = -1e9f, edgeAhead = 1e9f;
                    std::optional<double> previous = here;
                    for (int k = 1; k <= samples; ++k)
                    {
                        const auto h = heightAt(k * spacing);
                        if (!h || !previous)
                            break;
                        if (std::abs(*h - *previous) > 4.0)
                        {
                            edgeAhead = (k - 0.5f) * spacing;
                            break;
                        }
                        previous = h;
                    }
                    previous = here;
                    for (int k = 1; k <= samples; ++k)
                    {
                        const auto h = heightAt(-k * spacing);
                        if (!h || !previous)
                            break;
                        if (std::abs(*h - *previous) > 4.0)
                        {
                            edgeBehind = -(k - 0.5f) * spacing;
                            break;
                        }
                        previous = h;
                    }
                    // The foot's extent along the travel: 11 u forward, 5 u
                    // strafing (the stair gait; else 11).
                    const float halfFoot = stairOn
                        ? 11.0f * std::abs(forward.Dot(along)) + 5.0f * std::abs(left.Dot(along)) : 11.0f;
                    float wanted = 0.0f;
                    if (edgeAhead - edgeBehind >= 2.0f * halfFoot)
                        wanted = std::clamp(0.0f, edgeBehind + halfFoot, edgeAhead - halfFoot);
                    else if (edgeAhead < 1e8f && edgeBehind > -1e8f)
                        wanted = 0.5f * (edgeAhead + edgeBehind);
                    target += JPH::RVec3(along * std::clamp(wanted, -settings.mTreadSnap, settings.mTreadSnap));
                    // A tread only where an edge was found: on a smooth slope
                    // the kept height dragged every re-aim 6 u back (SLOPE-01
                    // stand 40 fell 3 of 4).
                    if (edgeAhead < 1e8f || edgeBehind > -1e8f)
                        mFeet[side].mTreadZ = groundNear(target, footAt);
                    else
                        mFeet[side].mTreadZ.reset();
                }
                else
                    mFeet[side].mTreadZ.reset();
            }
            return target;
        };
        // _HIP_GAP: the landing kept apart from the other foot across the
        // hips too.
        const auto keepAcrossHips = [&](JPH::RVec3 target, std::size_t side, const JPH::RVec3& otherAt) {
            if (settings.mHipGap <= 0.0f || !hipLineKnown)
                return target;
            const float sign = side == 0 ? 1.0f : -1.0f;
            const float gap = settings.mHipGap * footWidth;
            const float across = sign * JPH::Vec3(target - otherAt).Dot(hipLeft);
            if (across < gap)
                target += JPH::RVec3(hipLeft * (sign * (gap - across)));
            return target;
        };
        const auto landingTarget = [&](std::size_t side, float remaining) {
            // _STOP_LEAD: stopping, beside where the pelvis comes to rest.
            if (input.mStopping && settings.mStopLead > 0.0f)
            {
                JPH::RVec3 target = spot(side, physPelvis + JPH::RVec3(horizontal(pelvisVelocity) * settings.mStopLead));
                const auto& other = mFeet[1 - side];
                const JPH::RVec3 otherAt = other.mSwinging ? other.mTarget : other.mPin;
                const float sign = side == 0 ? 1.0f : -1.0f;
                const float across = sign * JPH::Vec3(target - otherAt).Dot(left);
                if (across < 1.2f * footWidth)
                    target += JPH::RVec3(left * (sign * (1.2f * footWidth - across)));
                target = keepAcrossHips(target, side, otherAt);
                const auto ground = groundNear(target, physFeet[side]);
                target.SetZ((ground ? *ground : actorZ) + restHeight);
                return target;
            }
            JPH::Vec3 lead = command * (0.5f * duty * 2.0f * period);
            if (reference && referenceWeight > 0.0f && speed > 1e-3f)
            {
                float scale = 1.0f;
                if (settings.mLeadScale)
                {
                    const float tableCycle = reference->mSwingTime / std::max(0.2f, 1.0f - reference->mDuty);
                    const float tableStance = reference->mDuty * tableCycle;
                    scale = std::clamp(duty * 2.0f * period / std::max(tableStance, 0.05f), 0.5f, 2.5f);
                }
                lead += (command / speed * (reference->mLead * scale) - lead) * referenceWeight;
            }
            mDebug[0] = speed > 1e-3f ? lead.Dot(command) / speed : 0.0f;
            if (!captureOn)
            {
                const JPH::Vec3 excess = pelvisVelocity - command;
                const JPH::Vec3 along = speed > 1e-3f ? command / speed : forward;
                const JPH::Vec3 ahead = along * excess.Dot(along);
                lead += (ahead + (excess - ahead) * settings.mSideCorrection) * 0.12f;
            }
            mDebug[1] = speed > 1e-3f ? ((pelvisVelocity - command) * 0.12f).Dot(command) / speed : 0.0f;
            if (lead.Length() > 0.6f * stepLength + 10.0f)
                lead = lead.Normalized() * (0.6f * stepLength + 10.0f);
            // WALK-02: the landing goes where the surplus carries the weight
            // (the capture point), sideways too, up to a share of the leg.
            if (captureOn)
            {
                JPH::Vec3 surplus = mSurplus * (settings.mCapture[0] * captureTime);
                const float most = std::max(0.0f, settings.mCapture[2]) * legReach;
                if (surplus.Length() > most)
                    surplus = surplus.Normalized() * most;
                lead += surplus;
                mDebug[1] = speed > 1e-3f ? surplus.Dot(command) / speed : surplus.Length();
            }
            // An anchor settle lands about where the body should stand.
            const JPH::RVec3 base = mSettleCenter ? *mSettleCenter : physPelvis;
            JPH::RVec3 target = spot(side, base + JPH::RVec3(command * remaining + lead + turnOffset));
            const auto& other = mFeet[1 - side];
            const JPH::RVec3 otherAt = other.mSwinging ? other.mTarget : other.mPin;
            const float sign = side == 0 ? 1.0f : -1.0f;
            const float across = sign * JPH::Vec3(target - otherAt).Dot(left);
            if (across < 1.2f * footWidth)
                target += JPH::RVec3(left * (sign * (1.2f * footWidth - across)));
            // Strafing, the tread shift runs along `left`: it must not undo
            // the gap above (it brought the feet together or crossed them).
            const JPH::RVec3 unsnapped = target;
            target = onTread(target, side);
            if (sign * JPH::Vec3(target - otherAt).Dot(left) < 1.2f * footWidth - 0.01f)
            {
                target = unsnapped;
                mFeet[side].mTreadZ.reset();
            }
            target = keepAcrossHips(target, side, otherAt);
            // _HIP_RANGE (item 15): no further out to the side than the hip
            // swings the leg (abduction), measured from the hip across the body.
            if (settings.mHipRange > 0.0f)
            {
                const JPH::RVec3 hipWorld = physWorld(legs[side]->mHip);
                const float most = legReach * std::sin(std::min(settings.mHipRange, 80.0f) * sPi / 180.0f);
                const float out = sign * JPH::Vec3(target - hipWorld).Dot(left);
                if (out > most)
                    target -= JPH::RVec3(left * (sign * (out - most)));
            }
            const auto ground = groundNear(target, physFeet[side]);
            target.SetZ((ground ? *ground : actorZ) + restHeight);
            return target;
        };
        const JPH::Vec3 travelDir = speed > 1.0f ? command / speed : forward;
        // The stair gait's riser scan: the ground from pin to target every
        // 4 u (at most 20 rays). A jump of 6-30 u is a riser (a slope gives at
        // most ~5 per 4 u); a step down lands 4-72 u under the pin's ground
        // (up to three risers: a long stride on tall stairs crosses two).
        const auto scanStair = [&](std::size_t side) {
            Foot& foot = mFeet[side];
            foot.mStair = false;
            foot.mStepDown = false;
            if (!stairOn)
                return;
            ++foot.mScans;
            JPH::Vec3 dir = horizontal(JPH::Vec3(foot.mTarget - foot.mPin));
            const float distance = dir.Length();
            if (distance < 4.0f)
                return;
            dir /= distance;
            const JPH::RVec3 footAt = foot.mPin;
            const int samples = std::clamp(static_cast<int>(std::ceil(distance / 4.0f)), 1, 20);
            std::optional<double> previous;
            bool riser = false;
            for (int k = 0; k <= samples; ++k)
            {
                const auto g = groundNear(
                    foot.mPin + JPH::RVec3(dir * (distance * static_cast<float>(k) / static_cast<float>(samples))), footAt);
                if (!g)
                    continue;
                if (previous && std::abs(*g - *previous) > 6.0 && std::abs(*g - *previous) <= 30.0)
                    riser = true;
                previous = g;
            }
            const auto pinGround = groundNear(foot.mPin, footAt);
            const auto targetGround = groundNear(foot.mTarget, footAt);
            foot.mStair = riser;
            foot.mStepDown = riser && pinGround && targetGround && *pinGround - *targetGround > 4.0
                && *pinGround - *targetGround <= 72.0;
            foot.mPinGround = pinGround ? *pinGround : foot.mPin.GetZ() - restHeight;
            foot.mLockedZ = foot.mTarget.GetZ();
            foot.mSwingDir = dir;
            if (trace && riser)
            {
                std::ostringstream line;
                line << std::fixed << std::setprecision(2) << "RAGDOLL_FOOTFALL_STAIR side=" << side
                     << " down=" << foot.mStepDown << " scan=" << foot.mScans << " upper="
                     << (pinGround ? *pinGround : 0.0) << " lower=" << (targetGround ? *targetGround : 0.0)
                     << " probe_miss=" << probeMisses;
                trace(line.str());
            }
        };
        const auto swingOffset = [&](std::size_t /*side*/, float t, const std::array<std::array<float, 21>, 3>& path) {
            const float x = std::clamp(t, 0.0f, 1.0f) * 20.0f;
            const std::size_t i = std::min<std::size_t>(static_cast<std::size_t>(x), 19);
            const auto at = [&](const std::array<float, 21>& c) { return c[i] + (c[i + 1] - c[i]) * (x - static_cast<float>(i)); };
            const float gain = 1.0f + (settings.mRunPathGain - 1.0f) * running;
            const float kick = at(path[0]) - path[0][0];
            const float along = path[0][0] + (kick < 0.0f ? kick * gain : kick);
            return travelDir * along; // the reference's sideways swing spread the legs
        };
        const auto startSwing = [&](std::size_t side, float duration) {
            Foot& foot = mFeet[side];
            foot.mSwinging = true;
            output.mSwingStarted[side] = true;
            foot.mPin = physFeet[side];
            foot.mPhase = 0.0f;
            foot.mDuration = duration;
            foot.mLift = liftHeight;
            foot.mLiftCurve.reset();
            foot.mProgressCurve.reset();
            if (reference && referenceWeight > 0.0f && mWalking)
            {
                std::array<float, 21> lift, progress;
                const float liftGain = 1.0f + (settings.mRunLiftGain - 1.0f) * running;
                for (std::size_t i = 0; i < lift.size(); ++i)
                {
                    const float t = static_cast<float>(i) / static_cast<float>(lift.size() - 1);
                    const float arc = liftHeight * std::sin(sPi * t);
                    const float ease = t * t * t * (10.0f + t * (-15.0f + 6.0f * t));
                    lift[i] = (arc + (reference->mLift[i] * referenceScales[1] - arc) * referenceWeight) * liftGain;
                    if (settings.mLandTaper > 0.0f && t > 1.0f - settings.mLandTaper)
                        lift[i] *= smoothstep((1.0f - t) / settings.mLandTaper);
                    progress[i] = ease + (reference->mProgress[i] - ease) * referenceWeight;
                }
                foot.mLiftCurve = lift;
                foot.mProgressCurve = progress;
            }
            foot.mTreadZ.reset();
            foot.mScans = 0;
            foot.mTarget = landingTarget(side, duration);
            foot.mTargetNormal = normalAt(foot.mTarget);
            foot.mTargetNormalAt = foot.mTarget;
            foot.mPathCurve.reset();
            const auto& legReference = side == 0 || !referenceMirror ? reference : referenceMirror;
            if (settings.mFollowPath && legReference && referenceWeight > 0.0f && mWalking && legReference->mHasPath)
            {
                foot.mPathCurve = legReference->mPath;
                const JPH::RVec3 hip = physWorld(legs[side]->mHip);
                foot.mPathStart = horizontal(JPH::Vec3(physFeet[side] - hip) - swingOffset(side, 0.0f, *foot.mPathCurve));
                const float pathMatch = settings.mPathMatch + (settings.mRunPathMatch - settings.mPathMatch) * running;
                if (trace)
                {
                    std::ostringstream line;
                    line << std::fixed << std::setprecision(3) << "RAGDOLL_FOOTFALL_PATH side=" << side
                         << " mismatch=" << foot.mPathStart.Length() << " along=" << foot.mPathStart.Dot(travelDir)
                         << " match=" << pathMatch << " kept=" << (foot.mPathStart.Length() <= pathMatch);
                    trace(line.str());
                }
                if (foot.mPathStart.Length() > pathMatch)
                    foot.mPathCurve.reset();
            }
            // A step down follows the planar finish, not the reference path.
            scanStair(side);
            if (foot.mStepDown)
                foot.mPathCurve.reset();
        };

        if (!mReady)
        {
            for (std::size_t side = 0; side < 2; ++side)
            {
                Foot& foot = mFeet[side];
                foot = {};
                const auto ground = groundAt(physFeet[side], actorZ);
                foot.mPin = physFeet[side];
                foot.mPin.SetZ((ground ? *ground : actorZ) + restHeight);
                foot.mPinNormal = normalAt(foot.mPin);
                foot.mPin.SetZ((ground ? *ground : actorZ) + restOn(foot.mPinNormal));
            }
            mReady = true;
            mDoubleSupport = 0.0f;
        }

        // Advance a swing; land it.
        const bool moving = speed > 15.0f;
        bool anySwing = false;
        for (std::size_t side = 0; side < 2; ++side)
        {
            Foot& foot = mFeet[side];
            if (!foot.mSwinging)
                continue;
            foot.mPhase += dt / foot.mDuration;
            const float finish = planarFinishOf(foot);
            const float retargetUntil = finish < 1.0f ? std::min(0.75f, std::clamp(finish, 0.4f, 1.0f) - 0.15f) : 0.75f;
            if (foot.mPhase < retargetUntil && !foot.mTargetHeld)
            {
                const float remaining = (1.0f - foot.mPhase) * foot.mDuration;
                const JPH::RVec3 aimed = moving ? landingTarget(side, remaining) : landingTarget(side, 0.0f);
                const bool otherTread = std::abs(aimed.GetZ() - foot.mLockedZ) > 4.0;
                if (foot.mStair && otherTread && foot.mPhase >= settings.mStair[0])
                {
                    // Locked: the landing keeps its tread (a re-aim on it is fine).
                    if (trace)
                    {
                        std::ostringstream line;
                        line << std::fixed << std::setprecision(2) << "RAGDOLL_FOOTFALL_STAIR lock_reject side=" << side
                             << " from_z=" << foot.mLockedZ << " to_z=" << aimed.GetZ() << " phase=" << foot.mPhase;
                        trace(line.str());
                    }
                }
                else
                {
                    foot.mTarget = aimed;
                    if (slopeOn && horizontal(JPH::Vec3(aimed - foot.mTargetNormalAt)).Length() > 4.0f)
                    {
                        foot.mTargetNormal = normalAt(aimed);
                        foot.mTargetNormalAt = aimed;
                    }
                    if (stairOn && otherTread && foot.mScans < 3)
                    {
                        const bool hadPath = foot.mPathCurve.has_value();
                        scanStair(side);
                        if (foot.mStepDown && hadPath)
                            foot.mPathCurve.reset();
                    }
                    else
                        foot.mLockedZ = aimed.GetZ();
                }
            }
            const bool airborne = physFeet[side].GetZ() - foot.mTarget.GetZ() > 4.0f;
            const bool otherPlanted = !mFeet[1 - side].mSwinging;
            // _CONTACT_LAND (item 6): a foot that touches down once it stops
            // closing on its target lands then; one without contact may run
            // on past the phase end, up to `extra` seconds.
            const float gap = horizontal(JPH::Vec3(foot.mTarget - physFeet[side])).Length();
            const bool contactLand = settings.mContactLand[0] > 0.0f && input.mFootContact.has_value();
            bool landNow = foot.mPhase >= 1.0f;
            float extra = 0.2f;
            if (contactLand)
            {
                const bool touching = (*input.mFootContact)[side];
                const bool closing = mSwingGap[side] >= 0.0f && gap < mSwingGap[side] - 0.05f;
                if (touching && !closing && foot.mPhase >= settings.mContactLand[0])
                    landNow = true;
                extra = std::max(0.0f, settings.mContactLand[1]);
                // Past the end, only a foot still in the air waits.
                if (landNow && !touching && foot.mPhase >= 1.0f && otherPlanted
                    && foot.mPhase < 1.0f + extra / foot.mDuration)
                {
                    mSwingGap[side] = gap;
                    anySwing = true;
                    continue;
                }
            }
            else if (foot.mPhase >= 1.0f && airborne && otherPlanted && foot.mPhase < 1.0f + extra / foot.mDuration)
            {
                anySwing = true;
                continue;
            }
            mSwingGap[side] = gap;
            if (landNow)
            {
                mSwingGap[side] = -1.0f;
                if (trace)
                {
                    std::ostringstream line;
                    line << std::fixed << std::setprecision(1) << "RAGDOLL_FOOTFALL_LAND side=" << side
                         << " miss=" << gap << " phase=" << std::setprecision(2) << foot.mPhase
                         << " contact=" << (input.mFootContact ? static_cast<int>((*input.mFootContact)[side]) : -1)
                         << std::setprecision(1) << " speed=" << speed;
                    trace(line.str());
                }
                // _PHASE_LOCK (item 9): this landing's place in the cycle.
                if (settings.mPhaseLock > 0.0f && mWalking && speed > 15.0f)
                {
                    const float liftedAt = side == 0 ? 0.0f : 0.5f;
                    const float wanted = std::fmod(liftedAt + (1.0f - duty) + 2.0f, 1.0f);
                    float error = wanted - mClock;
                    error -= std::round(error);
                    const float before = mClock;
                    mClock = std::fmod(mClock + error * settings.mPhaseLock + 2.0f, 1.0f);
                    // A jump forward past the other foot's lift point makes it due.
                    const float otherAt = side == 0 ? 0.5f : 0.0f;
                    const float moved = error * settings.mPhaseLock;
                    const float toOther = std::fmod(otherAt - before + 2.0f, 1.0f);
                    if (moved > 0.0f && toOther > 0.0f && toOther <= moved)
                        mLiftDue[1 - side] = true;
                }
                foot.mSwinging = false;
                foot.mTargetHeld = false;
                foot.mStair = false;
                foot.mStepDown = false;
                JPH::RVec3 at = physFeet[side];
                const auto ground = groundNear(at, physFeet[side]);
                at.SetZ((ground ? *ground : actorZ) + restHeight);
                foot.mPinNormal = normalAt(at);
                at.SetZ((ground ? *ground : actorZ) + restOn(foot.mPinNormal));
                foot.mPin = at;
                foot.mStanceKnee.reset();
                foot.mStanceTime = 0.0f;
                const auto& legReference = side == 0 || !referenceMirror ? reference : referenceMirror;
                if (legReference && referenceWeight > 0.0f && mWalking && legReference->mHasPath)
                {
                    foot.mStanceKnee = legReference->mStanceKnee;
                    foot.mStanceDuration = std::max(0.1f, duty * 2.0f * period);
                }
                mLastLanded = side;
                mDoubleSupport = 0.0f;
                mSinceLanding = 0.0f;
            }
            else
                anySwing = true;
        }

        // The balance helper's command: a swing now, landings, a held clock.
        const FootfallCommand helperCommand = mCommand;
        mCommand = {};
        if (helperCommand.mSwingNow && *helperCommand.mSwingNow < 2)
        {
            const std::size_t side = *helperCommand.mSwingNow;
            if (!mFeet[side].mSwinging && !mFeet[1 - side].mSwinging)
            {
                startSwing(side, std::max(0.1f, helperCommand.mSwingTime));
                if (trace)
                {
                    std::ostringstream line;
                    line << std::fixed << std::setprecision(1) << "RAGDOLL_FOOTFALL_HELPER_SWING side=" << side
                         << " time=" << helperCommand.mSwingTime;
                    trace(line.str());
                }
                anySwing = true;
            }
        }
        for (std::size_t side = 0; side < 2; ++side)
            if (helperCommand.mLanding[side] && mFeet[side].mSwinging)
            {
                JPH::RVec3 target = onTread(*helperCommand.mLanding[side], side);
                const auto ground = groundAt(target, actorZ);
                target.SetZ((ground ? *ground : actorZ) + restHeight);
                mFeet[side].mTarget = target;
                mFeet[side].mTargetHeld = true;
                mFeet[side].mTargetNormal = normalAt(target);
                mFeet[side].mTargetNormalAt = target;
            }
        if (helperCommand.mHoldClock)
        {
            // No steps of Footfall's own; released, it starts over from the
            // current feet (the foot further behind the travel lifts first).
            mWalking = false;
            mLiftDue = { false, false };
            mDoubleSupport = 0.0f;
        }

        // Start the next steps from the gait clock.
        if (!helperCommand.mHoldClock)
        {
            const float cycle = 2.0f * period;
            float swingDuration = std::clamp((1.0f - duty) * cycle, 0.2f, 0.45f);
            if (referenceSwing > 0.0f)
                swingDuration += (std::clamp(referenceSwing, 0.2f, 0.7f) - swingDuration) * referenceWeight;
            const auto hipOverrun = [&](std::size_t side) {
                const JPH::RVec3 hipWorld = physWorld(legs[side]->mHip);
                const auto& geometry = legs[side]->mGeometry;
                const float reach = geometry ? geometry->mUpperOffset.Length() + geometry->mLowerOffset.Length() : 72.0f;
                const auto& plan = settings.mOverrunPlan;
                const bool planned = plan[0] > 0.0f && speed > 15.0f;
                const float runFactor = planned ? std::min(settings.mOverrunRun, std::max(plan[1], 1.08f)) : settings.mOverrunRun;
                const float overrun = 1.08f + (runFactor - 1.08f) * running;
                if (JPH::Vec3(mFeet[side].mPin - hipWorld).Length() > reach * overrun)
                    return true;
                if (!planned)
                    return false;
                // _OVERRUN_PLAN (item 5): behind the hip along the travel by more
                // than the stance's own travel past the landing lead, plus a share
                // of the step.
                const float behind = -horizontal(JPH::Vec3(mFeet[side].mPin - hipWorld)).Dot(travelDir);
                const float stanceTravel = speed * duty * cycle;
                const float expected = std::max(10.0f, stanceTravel - std::max(0.0f, mDebug[0]));
                return behind > expected + plan[0] * stepLength;
            };
            // The stair trail (descending only): a planted foot left on a
            // higher step behind the hip, the leg past a share of its reach,
            // lifts (the strafe falls: the trailing foot stayed on the upper
            // step until the 3D overrun, the leg overstretched).
            const auto stairTrail = [&](std::size_t side) {
                if (!stairOn || !(settings.mStair[3] > 0.0f) || mFeet[side].mSwinging)
                    return false;
                const JPH::RVec3 hipWorld = physWorld(legs[side]->mHip);
                const JPH::RVec3& pin = mFeet[side].mPin;
                if (JPH::Vec3(pin - hipWorld).Dot(travelDir) > -10.0f)
                    return false;
                const auto& geometry = legs[side]->mGeometry;
                const float reach = geometry ? geometry->mUpperOffset.Length() + geometry->mLowerOffset.Length() : 72.0f;
                if (JPH::Vec3(pin - hipWorld).Length() <= settings.mStair[3] * reach)
                    return false;
                const auto pinGround = groundNear(pin, pin);
                const JPH::RVec3 hipAt(hipWorld.GetX(), hipWorld.GetY(), pin.GetZ());
                const auto hipGround = groundNear(hipAt, pin);
                if (!pinGround || !hipGround || !(*pinGround > *hipGround + 4.0))
                    return false;
                // A riser between (a downhill slope is not a stair).
                const JPH::Vec3 toHip = horizontal(JPH::Vec3(hipAt - pin));
                const int samples = std::clamp(static_cast<int>(std::ceil(toHip.Length() / 4.0f)), 1, 10);
                std::optional<double> previous = pinGround;
                for (int k = 1; k <= samples; ++k)
                {
                    const auto g = groundNear(pin + JPH::RVec3(toHip * (static_cast<float>(k) / samples)), pin);
                    if (g && previous && *previous - *g > 6.0 && *previous - *g <= 30.0)
                        return true;
                    if (g)
                        previous = g;
                }
                return false;
            };
            if (moving)
            {
                if (!mWalking)
                {
                    const JPH::Vec3 travel = command.Normalized();
                    const float behind0 = JPH::Vec3(mFeet[0].mPin - physPelvis).Dot(travel);
                    const float behind1 = JPH::Vec3(mFeet[1].mPin - physPelvis).Dot(travel);
                    const std::size_t first = behind0 <= behind1 ? 0 : 1;
                    mClock = first == 0 ? 0.999f : 0.499f;
                    mWalking = true;
                    mSettleCenter.reset();
                    mStanceForward.reset();
                    mSpotErrorTime = 0.0f;
                    mStillTime = 0.0f;
                    mStepsSinceStart = 0;
                }
                // WALK-02 (mCapture), the foot choice: with both feet
                // down and the surplus carrying the weight more than the
                // margin off course, the foot further from its landing steps
                // now - the clock's foot or not - and the clock resyncs to it.
                // A body lagging behind the command is no reason to step.
                if (captureOn && mWalking && mStepsSinceStart > 0 && !mFeet[0].mSwinging && !mFeet[1].mSwinging
                    && mSinceLanding >= 0.05f)
                {
                    const JPH::Vec3 along = command / speed;
                    JPH::Vec3 off = mSurplus * captureTime;
                    const float back = off.Dot(along);
                    if (back < 0.0f)
                        off -= along * back;
                    if (off.Length() > settings.mCapture[1])
                    {
                        std::array<float, 2> distance{};
                        for (std::size_t side = 0; side < 2; ++side)
                            distance[side]
                                = horizontal(JPH::Vec3(landingTarget(side, swingDuration) - mFeet[side].mPin)).Length();
                        const std::size_t far = distance[1] > distance[0] ? 1 : 0;
                        const std::size_t clockFoot = mClock < 0.5f ? 1 : 0;
                        startSwing(far, swingDuration);
                        ++mStepsSinceStart;
                        mLiftDue = { false, false };
                        mClock = far == 0 ? 0.0f : 0.5f;
                        ++mCaptureSteps;
                        if (far != clockFoot)
                            ++mCaptureSwaps;
                        if (trace)
                        {
                            std::ostringstream line;
                            line << std::fixed << std::setprecision(1) << "RAGDOLL_FOOTFALL_CAPTURE side=" << far
                                 << " clock_foot=" << clockFoot << " off=" << off.Length() << " (" << off.GetX() << ","
                                 << off.GetY() << ") dist=" << distance[0] << "," << distance[1] << " speed=" << speed;
                            trace(line.str());
                        }
                    }
                }
                const float previous = mClock;
                mClock += std::max(0.0f, dt / cycle + mClipStrideCorrection * dt / 0.25f);
                // WALK-02: the foot whose turn it is lifts before the other's
                // overrun step (a body carried sideways overran the left leg
                // landing after landing and the right foot never stepped).
                std::array<std::size_t, 2> order{ 0, 1 };
                if (captureOn)
                {
                    const auto due = [&](std::size_t side) {
                        const float at = side == 0 ? 1.0f : 0.5f;
                        return mLiftDue[side] || (previous < at && mClock >= at);
                    };
                    if (due(1) && !due(0))
                        order = { 1, 0 };
                }
                for (const std::size_t side : order)
                {
                    const float liftAt = side == 0 ? 1.0f : 0.5f;
                    const bool crossed = previous < liftAt && mClock >= liftAt;
                    const Foot& other = mFeet[1 - side];
                    const bool otherDown = !other.mSwinging || duty < 0.5f;
                    if (crossed && !mFeet[side].mSwinging)
                        mLiftDue[side] = true;
                    const bool trail = !mFeet[side].mSwinging && otherDown && !mLiftDue[side] && !hipOverrun(side)
                        && stairTrail(side);
                    if (!mFeet[side].mSwinging && otherDown && (mLiftDue[side] || hipOverrun(side) || trail))
                    {
                        if (trace)
                        {
                            std::ostringstream line;
                            line << std::fixed << std::setprecision(3) << "RAGDOLL_FOOTFALL_LIFT side=" << side
                                 << " due=" << mLiftDue[side] << " crossed=" << crossed << " overrun=" << hipOverrun(side)
                                 << " trail=" << trail
                                 << " clock=" << mClock << " cycle=" << cycle << " speed=" << speed;
                            trace(line.str());
                        }
                        startSwing(side, swingDuration * (mStepsSinceStart == 0 ? settings.mFirstStep : 1.0f));
                        ++mStepsSinceStart;
                        mLiftDue[side] = false;
                    }
                }
                if (mClock >= 1.0f)
                    mClock -= 1.0f;
            }
            else
            {
                const bool stopping = mWalking;
                mWalking = false;
                mLiftDue = { false, false };
                for (auto& foot : mFeet)
                    foot.mStanceKnee.reset();
                if (!anySwing)
                {
                    mDoubleSupport += dt;
                    if (stopping)
                        mDoubleSupport = 1.0f;
                    std::optional<std::size_t> next;
                    const char* reason = "";
                    mSettleCenter.reset();
                    for (std::size_t side = 0; side < 2 && !next; ++side)
                        if (hipOverrun(side))
                        {
                            next = side;
                            reason = "overrun";
                        }
                    // Anchor settle (the game's applyFootfall, d9c2ae2aa7): a
                    // body resting off where it should stand, feet planted
                    // under the physical pelvis, steps toward the drive pelvis.
                    // The standing rules below (mStandTurn) are for a body at rest:
                    // still under 10 u/s for a second. Right after a stop on a
                    // slope the body sags and must step at once (the Balmora hill
                    // stop fell in 10 of 13 runs with them, 0 of 6 without).
                    mStillTime = horizontal(pelvisVelocity).Length() < 10.0f ? mStillTime + dt : 0.0f;
                    const bool calm = settings.mStandTurn[0] > 0.0f && mStillTime > 1.0f;
                    const auto& anchor = settings.mAnchorSettle;
                    if (!next && anchor[0] > 0.0f && mDoubleSupport > 0.15f && !helperCommand.mNoAnchorSettle)
                    {
                        JPH::Vec3 offset
                            = horizontal(JPH::Vec3(frame.worldPosition(drive[pelvis].mTranslation) - physPelvis));
                        if (offset.Length() > anchor[0])
                        {
                            if (offset.Length() > anchor[1])
                                offset = offset.Normalized() * anchor[1];
                            const JPH::RVec3 center = physPelvis + JPH::RVec3(offset);
                            float farthest = 0.0f;
                            for (std::size_t side = 0; side < 2; ++side)
                            {
                                const float error
                                    = horizontal(JPH::Vec3(mFeet[side].mPin - spot(side, center))).Length();
                                if (error > farthest)
                                {
                                    farthest = error;
                                    next = side;
                                }
                            }
                            // A foot already within a few units of its spot cannot
                            // close the offset: the anchor stepped 1-2 units every
                            // 0.45 s for as long as the body stood 9.8 units off
                            // (IDLE-01 unarmed after walk: 22 steps in 10 s, the
                            // offset unchanged - the "nonstop small steps").
                            if (next && settings.mStandTurn[0] > 0.0f && farthest < 4.0f)
                                next.reset();
                            if (next)
                            {
                                mSettleCenter = center;
                                reason = "anchor";
                            }
                        }
                    }
                    // The feet's heading: kept through small turns, re-squared
                    // to the body's past the tolerance (mStandTurn).
                    const auto& standTurn = settings.mStandTurn;
                    JPH::Vec3 stanceForward = forward;
                    if (!calm)
                        mStanceForward = forward;
                    else
                    {
                        if (!mStanceForward || stopping)
                            mStanceForward = forward;
                        const float turned = std::acos(std::clamp(mStanceForward->Dot(forward), -1.0f, 1.0f));
                        if (turned > standTurn[0] * 3.14159265f / 180.0f)
                            mStanceForward = forward;
                        stanceForward = *mStanceForward;
                    }
                    bool spotError = false;
                    if (!next && mDoubleSupport > 0.15f)
                    {
                        float worst = 0.0f;
                        for (std::size_t side = 0; side < 2; ++side)
                        {
                            const float error = horizontal(
                                JPH::Vec3(mFeet[side].mPin - spotFacing(side, physPelvis, stanceForward))).Length();
                            if (error > 12.0f && error > worst)
                            {
                                worst = error;
                                next = side;
                                reason = "spot";
                            }
                        }
                        spotError = next.has_value();
                        // A passing sway is not a reason to step; a body still
                        // moving (a stop on a slope, a shove) steps at once (the
                        // Balmora hill stop fell 5 of 8 with the wait).
                        if (spotError && calm && mSpotErrorTime + dt < standTurn[1])
                            next.reset();
                    }
                    mSpotErrorTime = spotError ? mSpotErrorTime + dt : 0.0f;
                    // A settle that steps a foot back where its last settle came
                    // from (within 1.5 s) holds settling for 2 s: the anchor and
                    // spot rules stepped the same foot 12 units there and back
                    // every 0.45-0.7 s while the body hung 13 units off its
                    // anchor (IDLE-01 warhammer after walk: 17 steps); the anchor
                    // yield ends that offset meanwhile. Overrun steps always go.
                    mSettleClock += dt;
                    if (next && calm && std::string_view(reason) != "overrun")
                    {
                        const JPH::Vec3 move = horizontal(
                            JPH::Vec3(spot(*next, mSettleCenter ? *mSettleCenter : physPelvis) - mFeet[*next].mPin));
                        const auto& last = mLastSettle[*next];
                        if (mSettleClock - last.first < 1.5f
                            && move.Dot(last.second) < -0.5f * move.Length() * last.second.Length())
                            mSettleHoldUntil = mSettleClock + 2.0f;
                        if (mSettleClock < mSettleHoldUntil)
                        {
                            next.reset();
                            mSettleCenter.reset();
                        }
                        else
                            mLastSettle[*next] = { mSettleClock, move };
                    }
                    if (next)
                    {
                        // The step lands squared to the body.
                        mStanceForward = forward;
                        mSpotErrorTime = 0.0f;
                        startSwing(*next, 0.3f);
                        // Only the tidy-up step to the stance spot is capped; an overrun or
                        // anchor step must get the foot under the body (capping the overrun
                        // step felled the PGC viewer's agile catch).
                        if (settings.mSettleStepMax > 0.0f && !helperCommand.mFullSettle && std::string_view(reason) == "spot")
                        {
                            Foot& foot = mFeet[*next];
                            const JPH::Vec3 way = horizontal(JPH::Vec3(foot.mTarget - physFeet[*next]));
                            if (way.Length() > settings.mSettleStepMax)
                            {
                                const JPH::Vec3 step = way * (settings.mSettleStepMax / way.Length());
                                JPH::RVec3 target = physFeet[*next] + JPH::RVec3(step);
                                target.SetZ(foot.mTarget.GetZ());
                                foot.mTarget = target;
                                foot.mTargetHeld = true;
                            }
                        }
                        if (trace)
                        {
                            const JPH::Vec3 move = horizontal(JPH::Vec3(mFeet[*next].mTarget - physFeet[*next]));
                            const JPH::Vec3 anchorOffset
                                = horizontal(JPH::Vec3(frame.worldPosition(drive[pelvis].mTranslation) - physPelvis));
                            std::ostringstream line;
                            line << std::fixed << std::setprecision(1) << "RAGDOLL_FOOTFALL_SETTLE side=" << *next
                                 << " reason=" << reason << " move=" << move.Length() << " (" << move.GetX() << ","
                                 << move.GetY() << ") anchor_offset=" << anchorOffset.Length() << " double="
                                 << mDoubleSupport;
                            trace(line.str());
                        }
                    }
                }
            }
        }

        // Pelvis drop and landing cushion.
        const auto cushionEnvelope = [&](float age) {
            const float duration = settings.mCushion[1];
            return age >= 0.0f && age < duration ? std::sin(sPi * age / duration) : 0.0f;
        };
        {
            const auto& crouch = settings.mCrouch;
            float wanted = 0.0f;
            mSinceLanding += dt;
            if (moving)
            {
                const float bob = 0.5f * (1.0f + std::cos(2.0f * sPi * 2.0f * mClock));
                wanted = crouch[0] + (crouch[1] - crouch[0]) * running + crouch[2] * bob;
                if (reference && referenceWeight > 0.0f)
                {
                    const float stepPhase = std::clamp(mSinceLanding / std::max(0.1f, period), 0.0f, 1.0f);
                    const float table = reference->mCrouch * referenceScales[2] - reference->bob(stepPhase) * referenceScales[3];
                    wanted += (table - wanted) * referenceWeight;
                }
            }
            // Sneaking crouches: the stealth gait's own pelvis drop moving
            // (its measured crouch, not the scaled-off table crouch), kept
            // when stopping, never under the standing floor.
            if (input.mSneaking && settings.mSneak[0] > 0.0f)
            {
                if (moving && reference && referenceWeight > 0.0f)
                    mSneakCrouch = reference->mCrouch * settings.mSneak[1];
                wanted += std::max(mSneakCrouch, settings.mSneak[2]);
            }
            else
                mSneakCrouch = 0.0f;
            if (std::isfinite(input.mExtraCrouch))
                wanted += std::clamp(input.mExtraCrouch, 0.0f, 40.0f);
            mPelvisDrop += (wanted - mPelvisDrop) * std::clamp(dt * 8.0f, 0.0f, 1.0f);
            mCushionDrop = moving ? settings.mCushion[0] * cushionEnvelope(mSinceLanding) : 0.0f;
        }

        // Swing and stance leg springs.
        for (std::size_t side = 0; side < 2; ++side)
        {
            const float settle = std::clamp((mFeet[side].mPhase - 0.5f) / 0.3f, 0.0f, 1.0f);
            const float ease = settle * settle * (3.0f - 2.0f * settle);
            const float light = (settings.mRunSwingDamping - 1.0f) * running * (1.0f - ease);
            const float window = running * (1.0f - ease);
            const float scale = mFeet[side].mSwinging ? 1.0f + light : 1.0f;
            const float stiff = mFeet[side].mSwinging ? 1.0f + (settings.mRunSwingStiffness - 1.0f) * window : 1.0f;
            float kneeGive = 1.0f;
            if (!mFeet[side].mSwinging && moving && settings.mCushion[2] > 0.0f)
                kneeGive = 1.0f - settings.mCushion[2] * cushionEnvelope(mFeet[side].mStanceTime);
            if (std::abs(scale - mLegDampingScale[side]) < 0.02f && std::abs(stiff - mLegStiffnessScale[side]) < 0.02f
                && std::abs(kneeGive - mKneeGive[side]) < 0.02f)
                continue;
            mLegDampingScale[side] = scale;
            mLegStiffnessScale[side] = stiff;
            mKneeGive[side] = kneeGive;
            output.mSprings[side] = { stiff, scale, kneeGive };
            output.mSpringsChanged[side] = true;
        }

        // Drive the legs to the feet.
        const auto sampleVelocity = [&](std::size_t body) {
            const JPH::Quat rotation = drive[body].mRotation;
            const auto previous = mPreviousRotation.find(body);
            JPH::Vec3 velocity = JPH::Vec3::sZero();
            if (previous != mPreviousRotation.end())
            {
                const JPH::Quat delta = (rotation * previous->second.Conjugated()).Normalized().EnsureWPositive();
                JPH::Vec3 axis;
                float angle = 0.0f;
                delta.GetAxisAngle(axis, angle);
                if (angle > 1e-5f)
                    velocity = axis * (angle / dt);
            }
            if (!moving && !mFeet[0].mSwinging && !mFeet[1].mSwinging)
                velocity = JPH::Vec3::sZero();
            output.mAngularVelocity[body] = velocity;
            mPreviousRotation[body] = rotation;
        };
        sampleVelocity(pelvis);
        // CROUCH-01: sneaking, the planted legs shorten to the clip's (its
        // longer leg, the one carrying it), at a person's pace both ways.
        if (settings.mCrouchLegs[0] > 0.0f && legs[0]->mGeometry)
        {
            const auto& geometry = *legs[0]->mGeometry;
            const float full = geometry.mUpperOffset.Length() + geometry.mLowerOffset.Length();
            float wanted = full;
            if (input.mSneaking && input.mClipPose)
            {
                const auto& clip = input.mClipPose->mBodyTransforms;
                float clipLeg = 0.0f;
                for (std::size_t side = 0; side < 2; ++side)
                    if (legs[side]->mHip < clip.size() && legs[side]->mFoot < clip.size())
                        clipLeg = std::max(clipLeg,
                            (clip[legs[side]->mFoot].mTranslation - clip[legs[side]->mHip].mTranslation).Length());
                if (clipLeg > 0.3f * full)
                    wanted = full - (full - std::min(clipLeg, full)) * std::min(settings.mCrouchLegs[0], 1.0f);
            }
            if (mCrouchLength < 0.0f)
                mCrouchLength = full;
            const float rate = wanted < mCrouchLength ? settings.mCrouchLegs[1] : settings.mCrouchLegs[2];
            const float most = (rate > 0.0f ? rate : 1e4f) * dt * frame.mInverseUniformScale;
            mCrouchLength += std::clamp(wanted - mCrouchLength, -most, most);
        }
        else
            mCrouchLength = -1.0f;
        const bool crouchCap = mCrouchLength > 0.0f && legs[0]->mGeometry
            && mCrouchLength < legs[0]->mGeometry->mUpperOffset.Length() + legs[0]->mGeometry->mLowerOffset.Length() - 0.25f;
        float extend = 0.0f;
        {
            const auto& extendSetting = settings.mExtend;
            const auto& integralSetting = settings.mExtendIntegral;
            const JPH::RVec3 wanted = frame.worldPosition(drive[pelvis].mTranslation);
            const float sag = static_cast<float>(wanted.GetZ() - physPelvis.GetZ());
            mExtendIntegral = std::clamp(mExtendIntegral + integralSetting[0] * sag * dt, 0.0f, integralSetting[1]);
            // CROUCH-01: crouched, the body sits a little under the drive
            // pelvis by design and the integral wound up against it, pushing
            // planted feet into the floor wherever the cap did not bind (a
            // sideways clip's spread legs, a fresh landing) until one fell
            // through (3 of 15 sideways sneaks, both ankles at the floor for
            // 4 s). Without any extension the crouch walk sank 4-12 units
            // under the clip: the proportional part stays.
            if (crouchCap)
                mExtendIntegral = 0.0f;
            extend = std::clamp(extendSetting[0] * sag + mExtendIntegral, 0.0f, extendSetting[1] + integralSetting[1]);
        }
        for (std::size_t side = 0; side < 2; ++side)
        {
            const Foot& foot = mFeet[side];
            const FootfallLeg& leg = *legs[side];
            JPH::RVec3 goal = foot.mPin;
            JPH::Vec3 groundNormal = foot.mPinNormal; // mSlope: the ground the foot is tilted onto
            if (foot.mSwinging)
            {
                const float t = std::clamp(foot.mPhase + settings.mLookAhead, 0.0f, 1.0f);
                // Scoped recovery: the horizontal travel finishes early.
                const float finish = planarFinishOf(foot);
                const float planar = finish < 1.0f ? std::clamp(t / std::clamp(finish, 0.4f, 1.0f), 0.0f, 1.0f) : t;
                const auto sampleCurve = [](const std::array<float, 21>& curve, float phase) {
                    const float x = phase * 20.0f;
                    const std::size_t i = std::min<std::size_t>(static_cast<std::size_t>(x), 19);
                    return curve[i] + (curve[i + 1] - curve[i]) * (x - static_cast<float>(i));
                };
                const float ease = foot.mProgressCurve
                    ? sampleCurve(*foot.mProgressCurve, planar)
                    : planar * planar * planar * (10.0f + planar * (-15.0f + 6.0f * planar));
                float lift = foot.mLiftCurve ? sampleCurve(*foot.mLiftCurve, t) : foot.mLift * std::sin(sPi * t);
                goal = foot.mPin + (foot.mTarget - foot.mPin) * ease;
                if (foot.mPathCurve)
                {
                    const JPH::RVec3 hipWorld = physWorld(leg.mHip);
                    const float u = std::clamp(foot.mPhase + settings.mPathLead, 0.0f, 1.0f);
                    const float remaining = std::max(0.0f, 1.0f - foot.mPhase) * foot.mDuration;
                    const JPH::Vec3 hipNow = horizontal(JPH::Vec3(hipWorld));
                    const JPH::Vec3 end = hipNow + command * remaining + swingOffset(side, 1.0f, *foot.mPathCurve);
                    const JPH::Vec3 correction = horizontal(JPH::Vec3(foot.mTarget)) - end;
                    const JPH::Vec3 onPath = hipNow + swingOffset(side, u, *foot.mPathCurve)
                        + correction * smoothstep((u - 0.3f) / 0.7f) + foot.mPathStart * (1.0f - smoothstep(u / 0.4f));
                    const float settle = settings.mSettle;
                    const float rest = settle > 0.0f ? smoothstep((u - (1.0f - settle)) / settle) : 0.0f;
                    const JPH::Vec3 at = onPath + (horizontal(JPH::Vec3(foot.mTarget)) - onPath) * rest;
                    goal = JPH::RVec3(at.GetX(), at.GetY(), goal.GetZ());
                    const float x = u * 20.0f;
                    const std::size_t i = std::min<std::size_t>(static_cast<std::size_t>(x), 19);
                    if (foot.mLiftCurve)
                    {
                        const auto& curve = *foot.mLiftCurve;
                        lift = curve[i] + (curve[i + 1] - curve[i]) * (x - static_cast<float>(i));
                    }
                    lift = std::max(lift, settings.mClearance * std::sin(sPi * u));
                }
                const auto ground = groundNear(goal, foot.mPin);
                double groundZ = ground ? *ground : actorZ;
                // Step up (user bookmark 9, 2026-09-30: "running up stairs too
                // quickly causes some weird drag"; STAIRS-01: the pelvis 72
                // units under its target): the swing followed the ground under
                // the foot, so it rose for a step only once over it and the toe
                // caught the riser. A higher landing is reached early in the
                // swing, with a clearance of a share of the rise.
                if (settings.mStepUp > 0.0f && t > 0.15f)
                {
                    const auto pinGround = groundAt(foot.mPin, actorZ);
                    const auto targetGround = groundAt(foot.mTarget, actorZ);
                    if (pinGround && targetGround && *targetGround > *pinGround + 2.0)
                    {
                        const double rise = *targetGround - *pinGround;
                        groundZ = std::max(groundZ, *targetGround + settings.mStepUp * rise * std::sin(sPi * t));
                    }
                }
                // Step down: the sole stays over the highest ground under the
                // foot's back (the nosing) plus a clearance, never above the
                // step it left, until the release - a floor under the lift,
                // not added to it (the reverted 2808791cab stacked both).
                // WALK-02 slopes: the sole turns from the pin's ground to the
                // landing's with the swing; uphill it lifts higher (a step
                // rises with the ground's tilt), so the toe clears.
                if (slopeOn)
                {
                    groundNormal = (foot.mPinNormal * (1.0f - ease) + foot.mTargetNormal * ease)
                                       .NormalizedOr(JPH::Vec3::sAxisZ());
                    if (foot.mTarget.GetZ() > foot.mPin.GetZ() + 2.0)
                        lift *= 1.0f
                            + settings.mSlope[2]
                                * (1.0f - std::min(foot.mPinNormal.GetZ(), foot.mTargetNormal.GetZ()));
                }
                // _LIFT_LEFT (item 10): a foot still far from its landing clears
                // the floor by a share of the way it has left.
                if (settings.mLiftLeft[0] > 0.0f && foot.mPhase < 0.9f)
                {
                    const float toGo = horizontal(JPH::Vec3(foot.mTarget - physFeet[side])).Length();
                    lift = std::max(lift, std::min(settings.mLiftLeft[1], settings.mLiftLeft[0] * toGo));
                }
                float raised = lift;
                if (foot.mStepDown)
                {
                    const JPH::Vec3 dir = foot.mSwingDir;
                    const float extent = 11.0f * std::abs(dir.Dot(forward)) + 5.0f * std::abs(dir.Dot(left));
                    if (const auto heel = groundNear(goal - JPH::RVec3(dir * extent), foot.mPin))
                    {
                        const double raise
                            = std::max(0.0, std::min(*heel, foot.mPinGround) + settings.mStair[4] - groundZ);
                        const float hold = 1.0f - smoothstep((t - (settings.mStair[2] - 0.2f)) / 0.2f);
                        raised = std::max(lift, static_cast<float>(raise) * hold);
                    }
                }
                goal.SetZ(groundZ + restOn(groundNormal) + raised);
                const float startBlend = settings.mStartBlend;
                if (startBlend > 0.0f && foot.mPhase < startBlend)
                {
                    const float b = std::clamp(foot.mPhase / startBlend, 0.0f, 1.0f);
                    goal = foot.mPin + (goal - foot.mPin) * (b * b * (3.0f - 2.0f * b));
                }
                const JPH::RVec3 hipWorld = physWorld(leg.mHip);
                if (leg.mGeometry && settings.mReach > 0.0f)
                {
                    const float reach = settings.mReach * (leg.mGeometry->mUpperOffset.Length() + leg.mGeometry->mLowerOffset.Length());
                    const float dz = static_cast<float>(hipWorld.GetZ() - goal.GetZ());
                    const float across = std::sqrt(std::max(0.0f, reach * reach - dz * dz));
                    JPH::Vec3 offset = horizontal(JPH::Vec3(goal - hipWorld));
                    if (offset.Length() > across)
                    {
                        offset = offset.Normalized() * across;
                        goal = JPH::RVec3(hipWorld.GetX() + offset.GetX(), hipWorld.GetY() + offset.GetY(), goal.GetZ());
                    }
                }
            }
            else
                mFeet[side].mStanceTime += dt;
            // _MOTOR_LEAD (item 7): the swing goal where it will be when the
            // motors get there.
            const float motorLead = foot.mSwinging ? settings.mMotorLead[0] : settings.mMotorLead[1];
            if (foot.mSwinging && motorLead > 0.0f && mPreviousGoal[side] && !output.mSwingStarted[side])
            {
                JPH::Vec3 ahead = JPH::Vec3(goal - *mPreviousGoal[side]) * (motorLead / dt);
                if (ahead.Length() > 15.0f)
                    ahead = ahead.Normalized() * 15.0f;
                mPreviousGoal[side] = goal;
                goal += JPH::RVec3(ahead);
            }
            else
                mPreviousGoal[side] = foot.mSwinging ? std::optional<JPH::RVec3>(goal) : std::nullopt;
            const JPH::Vec3 goalModel = frame.modelPosition(goal);
            if (!std::isfinite(goalModel.GetX()) || !std::isfinite(goalModel.GetY()) || !std::isfinite(goalModel.GetZ()))
                continue;
            if (mFootRotation[side])
            {
                drive[leg.mFoot].mRotation = (footBase(drive[pelvis].mRotation) * *mFootRotation[side]).Normalized();
                // WALK-02 slopes: the sole tilted onto the ground (a share of
                // the slope, at most mSlope[1] degrees); a level foot touched a
                // hillside with its heel or toe only.
                if (slopeOn)
                {
                    const float slope = std::acos(std::clamp(groundNormal.GetZ(), -1.0f, 1.0f));
                    const float tilt = std::min(slope, settings.mSlope[1] * sPi / 180.0f) * std::min(settings.mSlope[0], 1.0f);
                    const JPH::Vec3 axis = frame.modelVector(JPH::Vec3::sAxisZ().Cross(groundNormal));
                    if (tilt > 1e-3f && !axis.IsNearZero())
                        drive[leg.mFoot].mRotation
                            = (JPH::Quat::sRotation(axis.Normalized(), tilt) * drive[leg.mFoot].mRotation).Normalized();
                }
                const float along = speed > 1.0f ? command.Dot(forward) / speed : 0.0f;
                const float fade = std::clamp((speed - 15.0f) / 45.0f, 0.0f, 1.0f);
                float wanted = 0.0f;
                if (settings.mFootPitch > 0.0f && mWalking && fade > 0.0f)
                {
                    const bool swinging = foot.mSwinging;
                    const float phase = swinging ? foot.mPhase
                                                 : (foot.mStanceDuration > 0.0f ? foot.mStanceTime / foot.mStanceDuration : 0.5f);
                    const float walkPitch = footfallFootPitch(input.mArmed ? 2 : 0, swinging, phase);
                    const float runPitch = footfallFootPitch(1, swinging, phase);
                    wanted = (walkPitch + (runPitch - walkPitch) * running) * (swinging ? 1.0f : settings.mFootPitchStance)
                        * settings.mFootPitch * fade * along;
                }
                float& degrees = mFootPitch[side];
                degrees += std::clamp(wanted - degrees, -settings.mFootPitchRate * dt, settings.mFootPitchRate * dt);
                if (std::abs(degrees) > 0.01f)
                {
                    const JPH::Vec3 leftModel = frame.modelVector(left);
                    if (!leftModel.IsNearZero())
                        drive[leg.mFoot].mRotation
                            = (JPH::Quat::sRotation(leftModel.Normalized(), degrees * sPi / 180.0f) * drive[leg.mFoot].mRotation)
                                  .Normalized();
                }
            }
            auto& hipFiltered = mHip[side];
            hipFiltered = hipFiltered ? *hipFiltered + (phys[leg.mHip].mTranslation - *hipFiltered) * smoothing
                                      : phys[leg.mHip].mTranslation;
            JPH::Vec3 fromHip = goalModel - *hipFiltered;
            // _MOTOR_LEAD: from where the hip will be.
            if (motorLead > 0.0f)
                fromHip -= frame.modelVector(horizontal(pelvisVelocity)) * motorLead;
            if (!foot.mSwinging)
                fromHip -= upModel * extend;
            const auto& geometry = leg.mGeometry;
            float stanceKneeScale = settings.mStanceKnee + (settings.mRunStanceKnee - settings.mStanceKnee) * running;
            if (input.mArmed && !input.mSneaking && settings.mArmedStanceKnee > 0.0f)
                stanceKneeScale = std::max(stanceKneeScale, settings.mArmedStanceKnee);
            // The reference knee bend caps the stance leg's length. Running,
            // a stance whose pelvis rides high (the clip's bob, the soft root
            // lifting the body) then never reaches the floor: the landed foot
            // hangs 3-6 units up for the whole stance while the other foot
            // swings (the limp, 2% of run stances, 2026-09-29). Letting the
            // leg extend past the cap from 0.17 s into stance whenever the
            // foot is above its pin (mostly the late-stance heel lift, a
            // fuller push-off) cut those stances to 0.4-0.6%; releasing it
            // only for feet that never came down did nothing (2.1%).
            // Running on the level only: stepping down a slope lands every
            // foot above the floor (it never fires on the Balmora downhill).
            const bool footInAir = settings.mStanceKneeAir[0] > 0.0f && foot.mStanceTime >= settings.mStanceKneeAir[1]
                && running > 0.5f && std::abs(foot.mPin.GetZ() - mFeet[1 - side].mPin.GetZ()) < 2.0
                && physFeet[side].GetZ() - foot.mPin.GetZ() > settings.mStanceKneeAir[0];
            if (footInAir && trace)
            {
                std::ostringstream line;
                line << std::fixed << std::setprecision(2) << "RAGDOLL_FOOTFALL_KNEE_AIR side=" << side
                     << " t=" << foot.mStanceTime << " up=" << physFeet[side].GetZ() - foot.mPin.GetZ();
                trace(line.str());
            }
            if (!foot.mSwinging && foot.mStanceKnee && geometry && moving && stanceKneeScale > 0.0f && !footInAir)
            {
                const float phase = std::clamp(foot.mStanceTime / foot.mStanceDuration, 0.0f, 1.0f);
                const float x = phase * 20.0f;
                const std::size_t i = std::min<std::size_t>(static_cast<std::size_t>(x), 19);
                const auto& curve = *foot.mStanceKnee;
                const float knee = (curve[i] + (curve[i + 1] - curve[i]) * (x - static_cast<float>(i))) * stanceKneeScale * sPi / 180.0f;
                const float a = geometry->mUpperOffset.Length();
                const float b = geometry->mLowerOffset.Length();
                const float length = std::sqrt(a * a + b * b + 2.0f * a * b * std::cos(knee));
                if (fromHip.Length() > length)
                    fromHip = fromHip.Normalized() * length;
            }
            if (!foot.mSwinging && geometry && !footInAir && std::isfinite(input.mExtraKnee) && input.mExtraKnee > 0.0f)
            {
                const float knee = std::min(input.mExtraKnee, 100.0f) * sPi / 180.0f;
                const float a = geometry->mUpperOffset.Length();
                const float b = geometry->mLowerOffset.Length();
                const float length = std::sqrt(a * a + b * b + 2.0f * a * b * std::cos(knee));
                if (fromHip.Length() > length)
                    fromHip = fromHip.Normalized() * length;
            }
            if (crouchCap && !foot.mSwinging && !footInAir && fromHip.Length() > mCrouchLength)
                fromHip = fromHip.Normalized() * mCrouchLength;
            const JPH::Vec3 hip = drive[leg.mHip].mTranslation;
            JPH::Vec3 held = fromHip;
            if (helperCommand.mStanceHoldsPelvis && !foot.mSwinging && !helperCommand.mPelvisLean.IsNearZero())
            {
                // Legs posed for a pelvis turned by the lean: the leg vectors
                // turn the other way in the pelvis frame.
                const JPH::Vec3 axis = frame.modelVector(helperCommand.mPelvisLean);
                const float angle = axis.Length();
                if (angle > 1e-5f)
                    held = JPH::Quat::sRotation(axis / angle, -angle) * fromHip;
            }
            const JPH::Vec3 target
                = hip + ((helperCommand.mStanceHoldsPelvis && !foot.mSwinging) ? held : physToDrive * fromHip);
            const JPH::Vec3 facing = frame.modelVector(forward);
            TwoBoneLegTargetInput legInput;
            legInput.mHipBodyIndex = leg.mHip;
            legInput.mKneeBodyIndex = leg.mKnee;
            legInput.mFootBodyIndex = leg.mFoot;
            legInput.mTargetFootPosition = target;
            legInput.mPreferredBendDirection = facing.NormalizedOr(JPH::Vec3::sAxisY());
            legInput.mHingeGeometry = leg.mGeometry;
            if (!applyTwoBoneLegTarget(std::span<Transform>(drive), legInput))
                continue;
            (foot.mSwinging ? mSwinging : mPinned)[side] = true;
            float feed = 1.0f;
            if (foot.mSwinging)
            {
                const float late = std::clamp((foot.mPhase - 0.5f) / 0.4f, 0.0f, 1.0f);
                feed = settings.mSwingFeedForward * (1.0f - late * late * (3.0f - 2.0f * late));
            }
            for (const std::size_t body : { leg.mHip, leg.mKnee, leg.mFoot })
            {
                sampleVelocity(body);
                output.mAngularVelocity[body] *= feed;
            }
        }

        // Torso lean toward a natural walk or run lean.
        {
            const auto& leanSetting = settings.mLean;
            float wanted = 0.0f;
            const JPH::Vec3 upVec = frame.modelVector(JPH::Vec3::sAxisZ());
            const JPH::Vec3 fwdVec = frame.modelVector(forward);
            const JPH::Vec3 leftVec = frame.modelVector(left);
            // CROUCH-01: sneaking keeps the clip's hunch (the walk lean took
            // up to 10 deg off it).
            const bool keepHunch = input.mSneaking && settings.mCrouchLegs[0] > 0.0f && settings.mCrouchLegs[3] > 0.0f;
            const bool leanable = moving && !keepHunch && !input.mSpine.empty()
                && (leanSetting[0] != 0.0f || leanSetting[1] != 0.0f);
            if (leanable)
            {
                const JPH::Vec3 torso = drive[input.mSpine.back()].mTranslation - drive[pelvis].mTranslation;
                const float pitch = std::atan2(torso.Dot(fwdVec.Normalized()), torso.Dot(upVec.Normalized())) * 180.0f / sPi;
                const float ahead = speed > 1e-3f ? command.Dot(forward) / speed : 1.0f;
                const float desired
                    = runWeight * leanSetting[1] + (1.0f - runWeight) * leanSetting[0] * (ahead > 0.0f ? ahead : -0.6f * ahead);
                wanted = std::clamp(desired - pitch, -10.0f, 10.0f);
            }
            mLean += (wanted - mLean) * std::clamp(dt * 4.0f, 0.0f, 1.0f);
            if (leanable && std::abs(mLean) > 0.05f)
            {
                bool valid = true;
                for (std::size_t index : input.mSpine)
                    valid = valid && index < drive.size();
                if (valid)
                {
                    const JPH::Quat full = JPH::Quat::sRotation(leftVec.Normalized(), mLean * sPi / 180.0f);
                    const JPH::Vec3 pivot = drive[input.mSpine.front()].mTranslation;
                    const std::size_t count = input.mSpine.size();
                    for (std::size_t k = 0; k < count; ++k)
                    {
                        const JPH::Quat q
                            = JPH::Quat::sIdentity().SLERP(full, static_cast<float>(k + 1) / static_cast<float>(count));
                        auto& transform = drive[input.mSpine[k]];
                        transform.mTranslation = pivot + q * (transform.mTranslation - pivot);
                        transform.mRotation = (q * transform.mRotation).Normalized();
                    }
                    for (const auto& [index, isHead] : input.mChestCarried)
                    {
                        if (!isHead || index >= drive.size())
                            continue;
                        auto& transform = drive[index];
                        transform.mTranslation = pivot + full * (transform.mTranslation - pivot);
                    }
                }
            }
        }
        if (trace)
            for (std::size_t side = 0; side < 2; ++side)
            {
                const auto& foot = mFeet[side];
                std::ostringstream line;
                line << std::fixed << std::setprecision(1) << "RAGDOLL_FOOTFALL side=" << side << " swing=" << foot.mSwinging
                     << " phase=" << foot.mPhase << " foot=" << physFeet[side].GetX() << "," << physFeet[side].GetY() << ","
                     << physFeet[side].GetZ() << " pin=" << foot.mPin.GetX() << "," << foot.mPin.GetY() << ","
                     << foot.mPin.GetZ() << " target=" << foot.mTarget.GetX() << "," << foot.mTarget.GetY() << ","
                     << foot.mTarget.GetZ() << " pelvis=" << physPelvis.GetX()
                     << "," << physPelvis.GetY() << " speed=" << speed << " period=" << period << std::setprecision(2)
                     << " ref=" << referenceWeight << " duty=" << duty << " swingT=" << foot.mDuration
                     << " drop=" << mPelvisDrop << " lead=" << mDebug[0] << " vcorr=" << mDebug[1] << " width=" << mDebug[2];
                trace(line.str());
            }
        if (settings.mObserveMotorFootGoal)
            for (std::size_t side = 0; side < 2; ++side)
            {
                if (!mSwinging[side] && !mPinned[side])
                {
                    mPreviousMotorFootGoal[side].reset();
                    continue;
                }
                const auto footIndex = legs[side]->mFoot;
                const auto rotation = (phys[pelvis].mRotation * drive[pelvis].mRotation.Conjugated()).Normalized();
                const auto target = frame.worldPosition(
                    phys[pelvis].mTranslation + rotation * (drive[footIndex].mTranslation - drive[pelvis].mTranslation));
                output.mMotorFootGoal[side] = target;
                if (mSwinging[side] && mPreviousMotorFootGoal[side] && !output.mSwingStarted[side] && dt > 0.0f)
                    output.mMotorFootVelocity[side] = JPH::Vec3(target - *mPreviousMotorFootGoal[side]) / dt;
                if (mSwinging[side])
                    mPreviousMotorFootGoal[side] = target;
                else
                    mPreviousMotorFootGoal[side].reset();
            }
        output.mPinned = mPinned;
        output.mSwinging = mSwinging;
    }
}

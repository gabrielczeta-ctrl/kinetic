#include <ragdollcore/uprightholder.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace RagdollCore
{
    float torsoTiltDegrees(float torsoUpDot)
    {
        return std::acos(std::clamp(torsoUpDot, -1.0f, 1.0f)) * (180.0f / std::numbers::pi_v<float>);
    }

    float updateTiltHold(TiltHoldState& state, const TiltHoldInput& input, const TiltHoldSettings& settings, float dt)
    {
        float want = 0.0f;
        if (settings.mFrom > 0.0f && input.mKnown && !input.mHit && input.mBodySpeed < 25.0f
            && input.mRequestSpeed < 25.0f)
        {
            const float tilt = torsoTiltDegrees(input.mTorsoUpDot);
            if (!state.mHolding)
            {
                state.mTiltTime = tilt > settings.mFrom ? state.mTiltTime + dt : 0.0f;
                state.mHolding = state.mTiltTime >= settings.mAfter;
                state.mUprightTime = 0.0f;
            }
            else
            {
                state.mUprightTime = tilt < settings.mFrom - 3.0f ? state.mUprightTime + dt : 0.0f;
                if (state.mUprightTime >= 0.5f)
                {
                    state.mHolding = false;
                    state.mTiltTime = 0.0f;
                }
            }
            if (state.mHolding)
                want = std::max(0.5f,
                    std::clamp((tilt - settings.mFrom) / std::max(1.0f, settings.mFull - settings.mFrom), 0.0f, 1.0f));
        }
        else
        {
            state.mTiltTime = 0.0f;
            state.mHolding = false;
            if (input.mHit)
                state.mShare = 0.0f;
        }
        state.mShare += std::clamp(want - state.mShare, -dt / 0.4f, dt / 0.4f);
        return state.mShare;
    }

    UprightControllerInput assembleUprightInput(const UprightAssembly& a)
    {
        UprightControllerInput upright;
        // Joint motors only control bodies relative to the pelvis; the free
        // pelvis needs a world-space torque to follow a get-up, so recovery
        // keeps bounded orientation control even without foot support.
        // Without a support surface translation is not corrected. The get-up's
        // lift-only unweighting is off under the soft root: its spring
        // already carries the get-up, and the extra 2.5 g lift threw a
        // kneeling body 50 units above the clip (2026-09-24 R5).
        upright.mRecoveryUnweight = a.mRecovering && (a.mSoftRootUnweight || !a.mSoftRoot) && !a.mRecoveryHeight
            && a.mGettingUp && a.mGroundContact
            && (!a.mRecoveryHeightAssist || a.mPelvisUpDot < a.mStandingMinimumUpDot);
        upright.mTargetPelvisHeight = a.mCorrectTranslation ? a.mTargetPelvisHeight
            : a.mGroundContact                               ? a.mPelvisHeight
                                                             : a.mTargetPelvisHeight;
        if (a.mCorrectTranslation && a.mHandoverPelvisHeight)
            upright.mTargetPelvisHeight = *a.mHandoverPelvisHeight;
        upright.mPelvisHeight = a.mGroundContact ? a.mPelvisHeight : a.mTargetPelvisHeight;
        if (a.mCorrectTranslation)
            upright.mPelvisVerticalVelocity = a.mPelvisVerticalVelocity;
        if (a.mCorrectTranslation && a.mHasSupport)
            upright.mCenterOfMassOffset = a.mCenterOfMassOffset;
        if (a.mCorrectTranslation)
            upright.mCenterOfMassVelocity = a.mCenterOfMassVelocity;
        // In a brief gait flight the decision's target keeps orientation
        // torque alive without accelerating the body through the air.
        upright.mTargetHorizontalVelocity = a.mTargetHorizontalVelocity;
        upright.mPelvisUp = a.mPelvisUp;
        upright.mPelvisAngularVelocity = a.mPelvisAngularVelocity;
        upright.mTorsoUp = a.mTorsoUp;
        upright.mTorsoAngularVelocity = a.mTorsoAngularVelocity;
        // Standing and gait balance against world up: following the animated
        // pelvis up would make the walk cycle's lean the physical upright and
        // can topple the body. Only the get-up follows the clip.
        if (a.mGetUpTargetUp)
            upright.mTargetUp = *a.mGetUpTargetUp;
        else if (a.mHeldFollowTargetUp)
            upright.mTargetUp = *a.mHeldFollowTargetUp;
        upright.mTotalMass = a.mTotalMass;
        upright.mSupportFriction = a.mSupportFriction;
        upright.mGravityMagnitude = a.mGravity;
        upright.mCenterOfMassPosition = a.mCenterOfMassPosition;
        upright.mCenterOfMassHeight = a.mCenterOfMassHeight;
        return upright;
    }

    void limitStandingFriction(UprightControllerInput& input, float standFriction, bool correctTranslation)
    {
        if (standFriction > 0.0f && correctTranslation && !input.mLimitToSupportFriction)
        {
            input.mLimitToSupportFriction = true;
            input.mSupportFriction = standFriction;
        }
    }

    UprightControllerOutput composeUprightControl(const UprightControllerOutput& output, bool heldAssisted,
        float heldSpineScale, bool horizontalReactions, bool verticalReactions)
    {
        UprightControllerOutput control = output;
        if (heldAssisted)
        {
            control.mPelvisForce = JPH::Vec3::sZero();
            control.mPelvisTorque = JPH::Vec3::sZero();
            control.mSpineTorque *= heldSpineScale;
        }
        if (!horizontalReactions)
        {
            control.mPelvisForce.SetX(0.0f);
            control.mPelvisForce.SetY(0.0f);
        }
        if (!verticalReactions)
            control.mPelvisForce.SetZ(0.0f);
        return control;
    }
}

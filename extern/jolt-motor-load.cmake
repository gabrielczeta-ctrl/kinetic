# Pinned Jolt 5.5.0: add a feed-forward load to the native soft motor.
# gamma * (lambda - dt * load) retains the original spring/damping response
# and clamps the combined motor impulse once, within the native torque limits.
# Position motors also consume reference angular velocity in child constraint
# axes, so moving world-frame references retain their intended damping.
# Zero reference velocity preserves the original positive-zero bias exactly.
function(replace_jolt_motor_code relative_path before after)
    set(path "${JOLT_SOURCE_DIR}/${relative_path}")
    file(READ "${path}" content)
    string(FIND "${content}" "${after}" applied)
    if(NOT applied EQUAL -1)
        return()
    endif()
    # Upgrade an earlier version of a patched block without nesting it again.
    if(ARGC GREATER 3)
        string(FIND "${content}" "${ARGV3}" legacy)
        if(NOT legacy EQUAL -1)
            set(before "${ARGV3}")
        endif()
    endif()
    string(FIND "${content}" "${before}" found)
    if(found EQUAL -1)
        message(FATAL_ERROR "Jolt motor load patch anchor missing: ${relative_path}")
    endif()
    string(REPLACE "${before}" "${after}" content "${content}")
    file(WRITE "${path}" "${content}")
endfunction()

replace_jolt_motor_code("Jolt/Physics/Constraints/ConstraintPart/SpringPart.h"
[=[private:
	float]=]
[=[	/// Feed-forward impulse inside the soft constraint, before total motor clamping.
	inline void AddImpulseBias(float inImpulse) { mBias -= mSoftness * inImpulse; }

private:
	float]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/ConstraintPart/AngleConstraintPart.h"
[=[	/// Save state of this constraint part]=]
[=[	inline void AddSpringImpulseBias(float inImpulse) { mSpringPart.AddImpulseBias(inImpulse); }

	/// Save state of this constraint part]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/SwingTwistConstraint.h"
[=[	inline Vec3					GetTotalLambdaMotor() const									{ return Vec3(mMotorConstraintPart[0].GetTotalLambda(), mMotorConstraintPart[1].GetTotalLambda(), mMotorConstraintPart[2].GetTotalLambda()); }]=]
[=[	/// World-space feed-forward load, summed within the position motor torque budget.
	void SetMotorLoadTorque(Vec3Arg inTorque) { mMotorLoadTorque = inTorque; }
	inline Vec3					GetTotalLambdaMotor() const									{ return Vec3(mMotorConstraintPart[0].GetTotalLambda(), mMotorConstraintPart[1].GetTotalLambda(), mMotorConstraintPart[2].GetTotalLambda()); }]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/SwingTwistConstraint.h"
[=[	Vec3						mTargetAngularVelocity = Vec3::sZero();]=]
[=[	Vec3 mMotorLoadTorque = Vec3::sZero();
	Vec3						mTargetAngularVelocity = Vec3::sZero();]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/SwingTwistConstraint.cpp"
[=[	inStream.Write(mTargetAngularVelocity);]=]
[=[	inStream.Write(mMotorLoadTorque);
	inStream.Write(mTargetAngularVelocity);]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/SwingTwistConstraint.cpp"
[=[	inStream.Read(mTargetAngularVelocity);]=]
[=[	inStream.Read(mMotorLoadTorque);
	inStream.Read(mTargetAngularVelocity);]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/SwingTwistConstraint.cpp"
[=[					mMotorConstraintPart[i].CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, *mBody2, mWorldSpaceMotorAxis[i], 0.0f, rotation_error[i], mSwingMotorSettings.mSpringSettings);]=]
[=[				{
					const float velocity_bias = mTargetAngularVelocity[i] != 0.0f? -mTargetAngularVelocity[i] : 0.0f;
					mMotorConstraintPart[i].CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, *mBody2, mWorldSpaceMotorAxis[i], velocity_bias, rotation_error[i], mSwingMotorSettings.mSpringSettings);
					mMotorConstraintPart[i].AddSpringImpulseBias(inDeltaTime * mMotorLoadTorque.Dot(mWorldSpaceMotorAxis[i]));
				}]=]
[=[				{
					mMotorConstraintPart[i].CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, *mBody2, mWorldSpaceMotorAxis[i], 0.0f, rotation_error[i], mSwingMotorSettings.mSpringSettings);
					mMotorConstraintPart[i].AddSpringImpulseBias(inDeltaTime * mMotorLoadTorque.Dot(mWorldSpaceMotorAxis[i]));
				}]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/SwingTwistConstraint.cpp"
[=[				mMotorConstraintPart[0].CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, *mBody2, mWorldSpaceMotorAxis[0], 0.0f, rotation_error[0], mTwistMotorSettings.mSpringSettings);]=]
[=[			{
				const float velocity_bias = mTargetAngularVelocity[0] != 0.0f? -mTargetAngularVelocity[0] : 0.0f;
				mMotorConstraintPart[0].CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, *mBody2, mWorldSpaceMotorAxis[0], velocity_bias, rotation_error[0], mTwistMotorSettings.mSpringSettings);
				mMotorConstraintPart[0].AddSpringImpulseBias(inDeltaTime * mMotorLoadTorque.Dot(mWorldSpaceMotorAxis[0]));
			}]=]
[=[			{
				mMotorConstraintPart[0].CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, *mBody2, mWorldSpaceMotorAxis[0], 0.0f, rotation_error[0], mTwistMotorSettings.mSpringSettings);
				mMotorConstraintPart[0].AddSpringImpulseBias(inDeltaTime * mMotorLoadTorque.Dot(mWorldSpaceMotorAxis[0]));
			}]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/HingeConstraint.h"
[=[	inline float				GetTotalLambdaMotor() const								{ return mMotorConstraintPart.GetTotalLambda(); }]=]
[=[	/// World-space feed-forward load, summed within the position motor torque budget.
	void SetMotorLoadTorque(Vec3Arg inTorque) { mMotorLoadTorque = inTorque; }
	inline float				GetTotalLambdaMotor() const								{ return mMotorConstraintPart.GetTotalLambda(); }]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/HingeConstraint.h"
[=[	float						mTargetAngularVelocity = 0.0f;]=]
[=[	Vec3 mMotorLoadTorque = Vec3::sZero();
	float						mTargetAngularVelocity = 0.0f;]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/HingeConstraint.cpp"
[=[	inStream.Write(mTargetAngularVelocity);]=]
[=[	inStream.Write(mMotorLoadTorque);
	inStream.Write(mTargetAngularVelocity);]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/HingeConstraint.cpp"
[=[	inStream.Read(mTargetAngularVelocity);]=]
[=[	inStream.Read(mMotorLoadTorque);
	inStream.Read(mTargetAngularVelocity);]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/HingeConstraint.cpp"
[=[			mMotorConstraintPart.CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, *mBody2, mA1, 0.0f, CenterAngleAroundZero(mTheta - mTargetAngle), mMotorSettings.mSpringSettings);]=]
[=[		{
			const float velocity_bias = mTargetAngularVelocity != 0.0f? -mTargetAngularVelocity : 0.0f;
			mMotorConstraintPart.CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, *mBody2, mA1, velocity_bias, CenterAngleAroundZero(mTheta - mTargetAngle), mMotorSettings.mSpringSettings);
			mMotorConstraintPart.AddSpringImpulseBias(inDeltaTime * mMotorLoadTorque.Dot(mA1));
		}]=]
[=[		{
			mMotorConstraintPart.CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, *mBody2, mA1, 0.0f, CenterAngleAroundZero(mTheta - mTargetAngle), mMotorSettings.mSpringSettings);
			mMotorConstraintPart.AddSpringImpulseBias(inDeltaTime * mMotorLoadTorque.Dot(mA1));
		}]=])


# Expose the existing anatomical projection without mutating a joint target.
# World-space foot servos must obey the same target limits as native pose motors.
replace_jolt_motor_code("Jolt/Physics/Constraints/SwingTwistConstraint.h"
[=[	void						SetTargetOrientationCS(QuatArg inOrientation);]=]
[=[	Quat GetClampedTargetOrientationCS(QuatArg inOrientation) const;
	void						SetTargetOrientationCS(QuatArg inOrientation);]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/SwingTwistConstraint.cpp"
[=[void SwingTwistConstraint::SetTargetOrientationCS(QuatArg inOrientation)
{
	Quat q_swing, q_twist;
	inOrientation.GetSwingTwist(q_swing, q_twist);

	uint clamped_axis;
	mSwingTwistConstraintPart.ClampSwingTwist(q_swing, q_twist, clamped_axis);

	if (clamped_axis != 0)
		mTargetOrientation = q_swing * q_twist;
	else
		mTargetOrientation = inOrientation;
}]=]
[=[Quat SwingTwistConstraint::GetClampedTargetOrientationCS(QuatArg inOrientation) const
{
	Quat q_swing, q_twist;
	inOrientation.GetSwingTwist(q_swing, q_twist);
	uint clamped_axis;
	mSwingTwistConstraintPart.ClampSwingTwist(q_swing, q_twist, clamped_axis);
	return clamped_axis != 0 ? q_swing * q_twist : inOrientation;
}

void SwingTwistConstraint::SetTargetOrientationCS(QuatArg inOrientation)
{
	mTargetOrientation = GetClampedTargetOrientationCS(inOrientation);
}]=])

# World-reference damping observes solver-updated child velocity, not a cached parent velocity.
replace_jolt_motor_code("Jolt/Physics/Constraints/ConstraintPart/AngleConstraintPart.h"
[=[	inline bool					SolveVelocityConstraint(Body &ioBody1, Body &ioBody2, Vec3Arg inWorldSpaceAxis, float inMinLambda, float inMaxLambda)]=]
[=[	inline bool					SolveVelocityConstraint(Body &ioBody1, Body &ioBody2, Vec3Arg inWorldSpaceAxis, float inMinLambda, float inMaxLambda, bool inWorldVelocityReference = false)]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/ConstraintPart/AngleConstraintPart.h"
[=[		float lambda = mEffectiveMass * (inWorldSpaceAxis.Dot(ioBody1.GetAngularVelocity() - ioBody2.GetAngularVelocity()) - mSpringPart.GetBias(mTotalLambda));]=]
[=[		const float velocity_error = inWorldVelocityReference? -inWorldSpaceAxis.Dot(ioBody2.GetAngularVelocity())
			: inWorldSpaceAxis.Dot(ioBody1.GetAngularVelocity() - ioBody2.GetAngularVelocity());
		float lambda = mEffectiveMass * (velocity_error - mSpringPart.GetBias(mTotalLambda));]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/SwingTwistConstraint.h"
[=[	/// World-space feed-forward load, summed within the position motor torque budget.]=]
[=[	/// Position motor damping can reference the child's current world velocity.
	/// Impulses still act equally and oppositely on the two connected bodies.
	void SetPositionMotorWorldVelocityReference(bool inWorld) { mPositionMotorWorldVelocityReference = inWorld; }
	bool GetPositionMotorWorldVelocityReference() const { return mPositionMotorWorldVelocityReference; }

	/// World-space feed-forward load, summed within the position motor torque budget.]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/SwingTwistConstraint.h"
[=[	Vec3 mMotorLoadTorque = Vec3::sZero();]=]
[=[	bool mPositionMotorWorldVelocityReference = false;
	Vec3 mMotorLoadTorque = Vec3::sZero();]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/SwingTwistConstraint.cpp"
[=[	inStream.Write(mMotorLoadTorque);]=]
[=[	inStream.Write(mPositionMotorWorldVelocityReference);
	inStream.Write(mMotorLoadTorque);]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/SwingTwistConstraint.cpp"
[=[	inStream.Read(mMotorLoadTorque);]=]
[=[	inStream.Read(mPositionMotorWorldVelocityReference);
	inStream.Read(mMotorLoadTorque);]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/SwingTwistConstraint.cpp"
[=[		impulse |= mMotorConstraintPart[0].SolveVelocityConstraint(*mBody1, *mBody2, mWorldSpaceMotorAxis[0], min_twist_limit, max_twist_limit);]=]
[=[		impulse |= mMotorConstraintPart[0].SolveVelocityConstraint(*mBody1, *mBody2, mWorldSpaceMotorAxis[0], min_twist_limit, max_twist_limit,
			mPositionMotorWorldVelocityReference && mTwistMotorState == EMotorState::Position);]=])

replace_jolt_motor_code("Jolt/Physics/Constraints/SwingTwistConstraint.cpp"
[=[			impulse |= mMotorConstraintPart[i].SolveVelocityConstraint(*mBody1, *mBody2, mWorldSpaceMotorAxis[i], min_swing_limit, max_swing_limit);]=]
[=[			impulse |= mMotorConstraintPart[i].SolveVelocityConstraint(*mBody1, *mBody2, mWorldSpaceMotorAxis[i], min_swing_limit, max_swing_limit,
				mPositionMotorWorldVelocityReference && mSwingMotorState == EMotorState::Position);]=])

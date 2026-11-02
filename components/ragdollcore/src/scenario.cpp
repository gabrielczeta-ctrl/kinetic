#include <ragdollcore/scenario.hpp>
#include <ragdollcore/tuning.hpp>
#include <ragdollcore/armtarget.hpp>
#include <ragdollcore/limbclearance.hpp>
#include <ragdollcore/weaponswing.hpp>

#include <ragdollcore/replay.hpp>
#include <ragdollcore/contact.hpp>

// clang-format off
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/MotionType.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Core/StreamWrapper.h>
#include <cstdio>
#include <cstdlib>
#include <Jolt/Physics/PhysicsSystem.h>
// clang-format on

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <memory>
#include <mutex>
#include <numeric>
#include <numbers>
#include <span>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace RagdollCore
{
    namespace
    {
        constexpr JPH::ObjectLayer sGroundLayer = 0;
        constexpr JPH::ObjectLayer sRagdollLayer = 1;
        constexpr std::uint64_t sRagdollUserData = 1;
        constexpr std::uint32_t sTelemetryVersion = 76;

        bool isFinite(float value)
        {
            return std::isfinite(value);
        }

        bool isFinite(JPH::Vec3Arg value)
        {
            return isFinite(value.GetX()) && isFinite(value.GetY()) && isFinite(value.GetZ());
        }

        bool isFinite(JPH::RVec3Arg value)
        {
            return std::isfinite(static_cast<double>(value.GetX())) && std::isfinite(static_cast<double>(value.GetY()))
                && std::isfinite(static_cast<double>(value.GetZ()));
        }

        bool isFinite(JPH::QuatArg value)
        {
            return isFinite(value.GetX()) && isFinite(value.GetY()) && isFinite(value.GetZ()) && isFinite(value.GetW());
        }

        bool isFinite(const Transform& transform)
        {
            return isFinite(transform.mTranslation) && isFinite(transform.mRotation);
        }

        JPH::Vec3 horizontal(JPH::Vec3Arg value)
        {
            return JPH::Vec3(value.GetX(), value.GetY(), 0.0f);
        }

        class ScenarioBroadPhaseLayers final : public JPH::BroadPhaseLayerInterface
        {
        public:
            JPH::uint GetNumBroadPhaseLayers() const override { return 2; }
            JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override
            {
                return JPH::BroadPhaseLayer(static_cast<JPH::BroadPhaseLayer::Type>(layer == sGroundLayer ? 0 : 1));
            }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
            const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override
            {
                return layer == JPH::BroadPhaseLayer(0) ? "ground" : "ragdoll";
            }
#endif
        };

        class ScenarioObjectVsBroadPhase final : public JPH::ObjectVsBroadPhaseLayerFilter
        {
        public:
            bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer broadPhase) const override
            {
                return layer != sGroundLayer || broadPhase != JPH::BroadPhaseLayer(0);
            }
        };

        class ScenarioObjectPairs final : public JPH::ObjectLayerPairFilter
        {
        public:
            bool ShouldCollide(JPH::ObjectLayer first, JPH::ObjectLayer second) const override
            {
                return first != sGroundLayer || second != sGroundLayer;
            }
        };

        // Mirrors the live JoltContactListener: every manifold point is
        // observed at the midpoint between the two surfaces, and the normal is
        // oriented away from the observed body's counterpart so an upward
        // normal means the ground pushes the ragdoll body up.
        class ScenarioContactListener final : public JPH::ContactListener
        {
        public:
            void registerRagdoll(const PassiveRagdoll& ragdoll)
            {
                mBodyIndices.clear();
                for (std::size_t i = 0; i < ragdoll.getBodyCount(); ++i)
                    mBodyIndices.emplace(ragdoll.getBodyId(i).GetIndexAndSequenceNumber(), i);
            }

            void beginStep()
            {
                std::scoped_lock lock(mMutex);
                mContacts.clear();
                ++mDiagnosticStep;
            }

            std::vector<GroundContact> takeSortedContacts(const JPH::PhysicsSystem& physics, bool preserveOrder = false)
            {
                std::vector<GroundContact> contacts;
                {
                    std::scoped_lock lock(mMutex);
                    contacts = resolveGroundContacts(mContacts, physics);
                    mContacts.clear();
                }
                if (preserveOrder) return contacts;
                std::sort(contacts.begin(), contacts.end(), [](const GroundContact& a, const GroundContact& b) {
                    return std::make_tuple(a.mBodyIndex, static_cast<double>(a.mPosition.GetX()),
                               static_cast<double>(a.mPosition.GetY()), static_cast<double>(a.mPosition.GetZ()),
                               a.mNormal.GetX(), a.mNormal.GetY(), a.mNormal.GetZ(), a.mPenetrationDepth)
                        < std::make_tuple(b.mBodyIndex, static_cast<double>(b.mPosition.GetX()),
                            static_cast<double>(b.mPosition.GetY()), static_cast<double>(b.mPosition.GetZ()),
                            b.mNormal.GetX(), b.mNormal.GetY(), b.mNormal.GetZ(), b.mPenetrationDepth);
                });
                return contacts;
            }

            void OnContactAdded(const JPH::Body& body1, const JPH::Body& body2, const JPH::ContactManifold& manifold,
                JPH::ContactSettings& settings) override
            {
                observe(body1, body2, manifold, settings.mCombinedFriction);
            }

            void OnContactPersisted(const JPH::Body& body1, const JPH::Body& body2,
                const JPH::ContactManifold& manifold, JPH::ContactSettings& settings) override
            {
                observe(body1, body2, manifold, settings.mCombinedFriction);
            }

        private:
            std::optional<std::size_t> find(const JPH::Body& body) const
            {
                const auto it = mBodyIndices.find(body.GetID().GetIndexAndSequenceNumber());
                if (it == mBodyIndices.end())
                    return std::nullopt;
                return it->second;
            }

            void observe(const JPH::Body& body1, const JPH::Body& body2, const JPH::ContactManifold& manifold, float friction)
            {
                const std::optional<std::size_t> first = find(body1);
                const std::optional<std::size_t> second = find(body2);
                if (mDiagnosticStep < 16 && RagdollCore::tuningText("OPENMW_RAGDOLL_REPLAY_POSE_TRACE"))
                    std::fprintf(stderr, "RAGDOLL_STARTUP_CONTACT step=%d first=%d second=%d points=%zu depth=%.17g normal=%.17g,%.17g,%.17g first_id=%u second_id=%u sub1=%u sub2=%u\n",
                        mDiagnosticStep, first ? static_cast<int>(*first) : -1, second ? static_cast<int>(*second) : -1,
                        static_cast<std::size_t>(manifold.mRelativeContactPointsOn1.size()), double(manifold.mPenetrationDepth),
                        double(manifold.mWorldSpaceNormal.GetX()), double(manifold.mWorldSpaceNormal.GetY()), double(manifold.mWorldSpaceNormal.GetZ()),
                        body1.GetID().GetIndexAndSequenceNumber(), body2.GetID().GetIndexAndSequenceNumber(), manifold.mSubShapeID1.GetValue(), manifold.mSubShapeID2.GetValue());
                // Contacts between two ragdoll bodies are self-collisions, not
                // support; the live adapter filters them by shared user data.
                if (first && second && mDiagnosticStep < 1800
                    && RagdollCore::tuningText("OPENMW_RAGDOLL_SELF_CONTACT_TRACE"))
                    std::fprintf(stderr,
                        "RAGDOLL_SELF_CONTACT schema=1 phase=contact_generation step=%d first=%zu second=%zu points=%zu depth=%.9g normal=%.9g,%.9g,%.9g\n",
                        mDiagnosticStep, *first, *second, static_cast<std::size_t>(manifold.mRelativeContactPointsOn1.size()),
                        double(manifold.mPenetrationDepth), double(manifold.mWorldSpaceNormal.GetX()),
                        double(manifold.mWorldSpaceNormal.GetY()), double(manifold.mWorldSpaceNormal.GetZ()));
                if (first.has_value() == second.has_value())
                    return;
                const std::size_t bodyIndex = first ? *first : *second;
                std::scoped_lock lock(mMutex);
                appendGroundContactObservations(mContacts, bodyIndex, body1, body2, manifold, first.has_value(), friction);
            }

            std::unordered_map<std::uint32_t, std::size_t> mBodyIndices;
            int mDiagnosticStep = -1;
            std::vector<GroundContactObservation> mContacts;
            std::mutex mMutex;
        };

        struct CrowdStepContacts
        {
            std::vector<std::vector<GroundContact>> mGroundContacts;
            std::size_t mCrossActorContactCount = 0;
        };

        class CrowdContactListener final : public JPH::ContactListener
        {
        public:
            explicit CrowdContactListener(std::size_t actorCount)
                : mActorCount(actorCount)
                , mGroundContacts(actorCount)
            {
            }

            void registerRagdoll(std::size_t actorIndex, const PassiveRagdoll& ragdoll)
            {
                for (std::size_t bodyIndex = 0; bodyIndex < ragdoll.getBodyCount(); ++bodyIndex)
                {
                    mBodies.emplace(
                        ragdoll.getBodyId(bodyIndex).GetIndexAndSequenceNumber(), BodyOwner{ actorIndex, bodyIndex });
                }
            }

            void beginStep()
            {
                std::scoped_lock lock(mMutex);
                for (auto& contacts : mGroundContacts)
                    contacts.clear();
                mCrossActorContactCount = 0;
            }

            CrowdStepContacts takeSortedContacts(const JPH::PhysicsSystem& physics)
            {
                CrowdStepContacts result;
                {
                    std::scoped_lock lock(mMutex);
                    for (auto& observations : mGroundContacts)
                    {
                        result.mGroundContacts.push_back(resolveGroundContacts(observations, physics));
                        observations.clear();
                    }
                    result.mCrossActorContactCount = mCrossActorContactCount;
                    mGroundContacts.resize(mActorCount);
                }
                for (auto& contacts : result.mGroundContacts)
                {
                    std::sort(contacts.begin(), contacts.end(), [](const GroundContact& a, const GroundContact& b) {
                        return std::make_tuple(a.mBodyIndex, static_cast<double>(a.mPosition.GetX()),
                                   static_cast<double>(a.mPosition.GetY()), static_cast<double>(a.mPosition.GetZ()),
                                   a.mNormal.GetX(), a.mNormal.GetY(), a.mNormal.GetZ(), a.mPenetrationDepth)
                            < std::make_tuple(b.mBodyIndex, static_cast<double>(b.mPosition.GetX()),
                                static_cast<double>(b.mPosition.GetY()), static_cast<double>(b.mPosition.GetZ()),
                                b.mNormal.GetX(), b.mNormal.GetY(), b.mNormal.GetZ(), b.mPenetrationDepth);
                    });
                }
                return result;
            }

            void OnContactAdded(const JPH::Body& body1, const JPH::Body& body2, const JPH::ContactManifold& manifold,
                JPH::ContactSettings& settings) override
            {
                observe(body1, body2, manifold, settings.mCombinedFriction);
            }

            void OnContactPersisted(const JPH::Body& body1, const JPH::Body& body2,
                const JPH::ContactManifold& manifold, JPH::ContactSettings& settings) override
            {
                observe(body1, body2, manifold, settings.mCombinedFriction);
            }

        private:
            struct BodyOwner
            {
                std::size_t mActorIndex = 0;
                std::size_t mBodyIndex = 0;
            };

            std::optional<BodyOwner> find(const JPH::Body& body) const
            {
                const auto it = mBodies.find(body.GetID().GetIndexAndSequenceNumber());
                if (it == mBodies.end())
                    return std::nullopt;
                return it->second;
            }

            void observe(const JPH::Body& body1, const JPH::Body& body2, const JPH::ContactManifold& manifold, float friction)
            {
                const std::optional<BodyOwner> first = find(body1);
                const std::optional<BodyOwner> second = find(body2);
                const JPH::uint pointCount
                    = std::min(manifold.mRelativeContactPointsOn1.size(), manifold.mRelativeContactPointsOn2.size());

                std::scoped_lock lock(mMutex);
                if (first && second)
                {
                    if (first->mActorIndex != second->mActorIndex)
                        mCrossActorContactCount += std::max<std::size_t>(1, pointCount);
                    return;
                }
                if (first.has_value() == second.has_value())
                    return;

                const BodyOwner owner = first ? *first : *second;
                appendGroundContactObservations(mGroundContacts[owner.mActorIndex], owner.mBodyIndex,
                    body1, body2, manifold, first.has_value(), friction);
            }

            std::size_t mActorCount;
            std::unordered_map<std::uint32_t, BodyOwner> mBodies;
            std::vector<std::vector<GroundContactObservation>> mGroundContacts;
            std::size_t mCrossActorContactCount = 0;
            std::mutex mMutex;
        };

        void configureScenarioPhysics(JPH::PhysicsSystem& physics, std::uint32_t velocitySteps)
        {
            JPH::PhysicsSettings settings = physics.GetPhysicsSettings();
            if (velocitySteps != 0)
                settings.mNumVelocitySteps = velocitySteps;
            constexpr float unitScale = UnitModel::UnitsPerMeter;
            constexpr float unitScaleSquared = unitScale * unitScale;
            settings.mSpeculativeContactDistance *= unitScale;
            settings.mPenetrationSlop *= unitScale;
            settings.mManifoldTolerance *= unitScale;
            settings.mMaxPenetrationDistance *= unitScale;
            settings.mBodyPairCacheMaxDeltaPositionSq *= unitScaleSquared;
            settings.mContactPointPreserveLambdaMaxDistSq *= unitScaleSquared;
            settings.mMinVelocityForRestitution *= unitScale;
            settings.mPointVelocitySleepThreshold *= unitScale;
            // Contact probes (2026-09-11, strafe-native-input): env-gated,
            // unset is byte-identical. Used to find why a box foot straddling
            // a coplanar mesh edge sinks ~1.6 units and loads asymmetrically.
            // Manifold reduction OFF by default since 2026-09-11 (matches the game); OPENMW_RAGDOLL_PHYS_MANIFOLD=1 restores it.
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_PHYS_MANIFOLD"); !(text && std::atoi(text) != 0))
                settings.mUseManifoldReduction = false;
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_PHYS_ACTIVE_EDGES"); text && std::atoi(text) == 0)
                settings.mCheckActiveEdges = false;
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_PHYS_SLOP"))
            {
                const float slop = std::strtof(text, nullptr);
                if (std::isfinite(slop) && slop >= 0.0f)
                    settings.mPenetrationSlop = slop;
            }
            physics.SetPhysicsSettings(settings);
        }

        class TelemetryEncoder
        {
        public:
            void u8(std::uint8_t value) { mBytes.push_back(static_cast<std::byte>(value)); }
            void boolean(bool value) { u8(value ? 1 : 0); }
            void u32(std::uint32_t value)
            {
                for (int i = 0; i < 4; ++i)
                    u8(static_cast<std::uint8_t>((value >> (8 * i)) & 0xffu));
            }
            void u64(std::uint64_t value)
            {
                for (int i = 0; i < 8; ++i)
                    u8(static_cast<std::uint8_t>((value >> (8 * i)) & 0xffu));
            }
            void f32(float value) { u32(std::bit_cast<std::uint32_t>(value)); }
            void f64(double value) { u64(std::bit_cast<std::uint64_t>(value)); }
            void vec3(JPH::Vec3Arg value)
            {
                f32(value.GetX());
                f32(value.GetY());
                f32(value.GetZ());
            }
            void rvec3(JPH::RVec3Arg value)
            {
                f64(static_cast<double>(value.GetX()));
                f64(static_cast<double>(value.GetY()));
                f64(static_cast<double>(value.GetZ()));
            }
            void quat(JPH::QuatArg value)
            {
                f32(value.GetX());
                f32(value.GetY());
                f32(value.GetZ());
                f32(value.GetW());
            }
            void transform(const Transform& value)
            {
                vec3(value.mTranslation);
                quat(value.mRotation);
            }
            void text(const std::string& value)
            {
                u64(value.size());
                for (const char c : value)
                    u8(static_cast<std::uint8_t>(c));
            }
            void optionalVec3(const std::optional<JPH::Vec3>& value)
            {
                boolean(value.has_value());
                if (value)
                    vec3(*value);
            }
            void optionalRVec3(const std::optional<JPH::RVec3>& value)
            {
                boolean(value.has_value());
                if (value)
                    rvec3(*value);
            }
            void optionalFloat(const std::optional<float>& value)
            {
                boolean(value.has_value());
                if (value)
                    f32(*value);
            }
            void optionalMotorTarget(const std::optional<MotorTargetTelemetry>& value)
            {
                boolean(value.has_value());
                if (!value)
                    return;
                u32(static_cast<std::uint32_t>(value->mConstraintType));
                boolean(value->mWorldOrientation);
                vec3(value->mTargetAngularVelocity);
                boolean(value->mCurrentOrientation.has_value());
                if (value->mCurrentOrientation)
                    quat(*value->mCurrentOrientation);
                boolean(value->mRequestedOrientation.has_value());
                if (value->mRequestedOrientation)
                    quat(*value->mRequestedOrientation);
                boolean(value->mTargetOrientation.has_value());
                if (value->mTargetOrientation)
                    quat(*value->mTargetOrientation);
                optionalFloat(value->mCurrentHingeAngle);
                optionalFloat(value->mTargetHingeAngle);
                optionalFloat(value->mHingeMinimum);
                optionalFloat(value->mHingeMaximum);
                optionalFloat(value->mNormalHalfConeAngle);
                optionalFloat(value->mPlaneHalfConeAngle);
                optionalFloat(value->mTwistMinimum);
                optionalFloat(value->mTwistMaximum);
                f32(value->mAngularError);
                f32(value->mTargetClampDelta);
                const auto spring = [&](const MotorSpringTelemetry& settings) {
                    boolean(settings.mStiffnessMode);
                    f32(settings.mFrequencyOrStiffness);
                    f32(settings.mDamping);
                };
                spring(value->mSpring);
                boolean(value->mTwistSpring.has_value());
                if (value->mTwistSpring)
                    spring(*value->mTwistSpring);
            }
            void indices(const std::vector<std::size_t>& value)
            {
                u64(value.size());
                for (const std::size_t index : value)
                    u64(index);
            }
            void support(const SupportEstimate& value)
            {
                boolean(value.mLeftFootSupported);
                boolean(value.mRightFootSupported);
                boolean(value.mCenterOfMassSupported);
                u64(value.mFootContactCount);
                u64(value.mGroundContactCount);
                rvec3(value.mCenter);
                rvec3(value.mGroundCenter);
                rvec3(value.mClosestPoint);
                f32(value.mCenterOfMassDistance);
                f32(value.mCenterOfMassHeight);
                optionalFloat(value.mFriction);
            }
            void uprightInput(const UprightControllerInput& value)
            {
                f32(value.mTargetPelvisHeight);
                f32(value.mPelvisHeight);
                f32(value.mPelvisVerticalVelocity);
                vec3(value.mCenterOfMassOffset);
                vec3(value.mCenterOfMassVelocity);
                vec3(value.mTargetHorizontalVelocity);
                vec3(value.mTargetUp);
                vec3(value.mPelvisUp);
                vec3(value.mPelvisAngularVelocity);
                vec3(value.mTorsoUp);
                vec3(value.mTorsoAngularVelocity);
                f32(value.mTotalMass);
                f32(value.mSupportFriction);
                f32(value.mGravityMagnitude);
                boolean(value.mLimitToSupportFriction);
                rvec3(value.mCenterOfMassPosition);
                f32(value.mCenterOfMassHeight);
                u64(value.mSupportContactPoints.size());
                for (const JPH::RVec3& point : value.mSupportContactPoints)
                    rvec3(point);
                boolean(value.mLimitToSupportMoment);
                boolean(value.mRecoveryUnweight);
            }
            void uprightOutput(const UprightControllerOutput& value)
            {
                vec3(value.mPelvisForce);
                vec3(value.mPelvisTorque);
                vec3(value.mSpineTorque);
                boolean(value.mPelvisForceSaturated);
                boolean(value.mPelvisMagnitudeSaturated);
                boolean(value.mPelvisHorizontalForceSaturated);
                boolean(value.mPelvisSupportMomentSaturated);
                boolean(value.mPelvisVerticalForceSaturated);
                boolean(value.mPelvisTorqueSaturated);
                boolean(value.mSpineTorqueSaturated);
            }
            void gait(const GaitControllerOutput& value)
            {
                u32(static_cast<std::uint32_t>(value.mPhase));
                f32(value.mTimeInPhase);
                f32(value.mPhaseProgress);
                f32(value.mSwingContactCandidateTime);
                f32(value.mSwingTimeSinceLift);
                boolean(value.mStanceReady);
                boolean(value.mSwingFootLifted);
                boolean(value.mSwingTouchdownCandidate);
                boolean(value.mTransitioned);
                boolean(value.mPhaseTimedOut);
            }

            std::vector<std::byte> take() { return std::move(mBytes); }

        private:
            std::vector<std::byte> mBytes;
        };

        struct BodyDefinition
        {
            const char* mName;
            const char* mParent;
            JPH::Vec3 mPosition;
        };

        BodyProfile makeBody(const std::string& name, const char* parent, float massFraction, std::uint32_t group)
        {
            BodyProfile body;
            body.mName = name;
            body.mBone = name;
            if (parent != nullptr)
                body.mParentBody = parent;
            body.mMassFraction = massFraction;
            body.mSelfCollisionGroup = group;
            return body;
        }

        ShapeProfile fittedCapsule(float radius, float halfHeight, const std::string& fitToBone)
        {
            ShapeProfile shape;
            shape.mType = ShapeType::Capsule;
            shape.mRadius = radius;
            shape.mHalfHeight = halfHeight;
            shape.mFitToBone = fitToBone;
            return shape;
        }

        ShapeProfile box(float x, float y, float z)
        {
            ShapeProfile shape;
            shape.mType = ShapeType::Box;
            shape.mHalfExtents = JPH::Vec3(x, y, z);
            return shape;
        }

        ShapeProfile sphere(float radius)
        {
            ShapeProfile shape;
            shape.mType = ShapeType::Sphere;
            shape.mRadius = radius;
            return shape;
        }

        ConstraintProfile swingTwist(float normalHalfCone, float planeHalfCone, float twistMin, float twistMax,
            float motorFrequency, float motorDamping, float motorTorque)
        {
            ConstraintProfile constraint;
            constraint.mType = ConstraintType::SwingTwist;
            constraint.mDeriveFrameFromSkeleton = true;
            constraint.mNormalHalfConeAngle = normalHalfCone;
            constraint.mPlaneHalfConeAngle = planeHalfCone;
            constraint.mTwistMinAngle = twistMin;
            constraint.mTwistMaxAngle = twistMax;
            constraint.mMotor.mFrequency = motorFrequency;
            constraint.mMotor.mDamping = motorDamping;
            constraint.mMotor.mMaximumTorque = motorTorque;
            return constraint;
        }

        ConstraintProfile hinge(
            float minAngle, float maxAngle, float motorFrequency, float motorDamping, float motorTorque)
        {
            ConstraintProfile constraint;
            constraint.mType = ConstraintType::Hinge;
            constraint.mDeriveFrameFromSkeleton = true;
            constraint.mHingeMinAngle = minAngle;
            constraint.mHingeMaxAngle = maxAngle;
            constraint.mMotor.mFrequency = motorFrequency;
            constraint.mMotor.mDamping = motorDamping;
            constraint.mMotor.mMaximumTorque = motorTorque;
            return constraint;
        }

        // Body order matches humanoid-v1.yaml. Positions are world units in
        // actor-root space; the character faces +Y with +X to its right.
        const BodyDefinition sHumanoidBodies[] = {
            { "pelvis", nullptr, JPH::Vec3(0.0f, 0.0f, 66.0f) },
            { "spine", "pelvis", JPH::Vec3(0.0f, 0.0f, 72.0f) },
            { "spine1", "spine", JPH::Vec3(0.0f, 0.0f, 82.0f) },
            { "spine2", "spine1", JPH::Vec3(0.0f, 0.0f, 92.0f) },
            { "neck", "spine2", JPH::Vec3(0.0f, 0.0f, 104.0f) },
            { "head", "neck", JPH::Vec3(0.0f, 0.0f, 112.0f) },
            { "left_clavicle", "spine2", JPH::Vec3(-5.0f, 0.0f, 100.0f) },
            { "left_upper_arm", "left_clavicle", JPH::Vec3(-17.0f, 0.0f, 100.0f) },
            { "left_forearm", "left_upper_arm", JPH::Vec3(-17.0f, 0.0f, 81.0f) },
            { "left_hand", "left_forearm", JPH::Vec3(-17.0f, 0.0f, 64.0f) },
            { "right_clavicle", "spine2", JPH::Vec3(5.0f, 0.0f, 100.0f) },
            { "right_upper_arm", "right_clavicle", JPH::Vec3(17.0f, 0.0f, 100.0f) },
            { "right_forearm", "right_upper_arm", JPH::Vec3(17.0f, 0.0f, 81.0f) },
            { "right_hand", "right_forearm", JPH::Vec3(17.0f, 0.0f, 64.0f) },
            { "left_thigh", "pelvis", JPH::Vec3(-8.0f, 0.0f, 62.0f) },
            { "left_calf", "left_thigh", JPH::Vec3(-8.0f, 0.0f, 33.0f) },
            { "left_foot", "left_calf", JPH::Vec3(-8.0f, 0.0f, 2.5f) },
            { "right_thigh", "pelvis", JPH::Vec3(8.0f, 0.0f, 62.0f) },
            { "right_calf", "right_thigh", JPH::Vec3(8.0f, 0.0f, 33.0f) },
            { "right_foot", "right_calf", JPH::Vec3(8.0f, 0.0f, 2.5f) },
        };

        std::uint64_t fnv1a(const std::vector<std::byte>& data)
        {
            std::uint64_t hash = 14695981039346656037ull;
            for (const std::byte value : data)
            {
                hash ^= std::to_integer<std::uint8_t>(value);
                hash *= 1099511628211ull;
            }
            return hash;
        }

        struct LegChain
        {
            std::size_t mHip = 0;
            std::size_t mKnee = 0;
            std::size_t mFoot = 0;
        };
    }

    SkeletonDescriptor makeScenarioHumanoidSkeleton()
    {
        SkeletonDescriptor skeleton;
        skeleton.mId = "scenario_humanoid";
        std::unordered_map<std::string, int> indices;
        for (const BodyDefinition& definition : sHumanoidBodies)
        {
            SkeletonBone bone;
            bone.mName = definition.mName;
            bone.mParent = definition.mParent == nullptr ? -1 : indices.at(definition.mParent);
            bone.mModelTransform.mTranslation = definition.mPosition;
            indices.emplace(bone.mName, static_cast<int>(skeleton.mBones.size()));
            skeleton.mBones.push_back(std::move(bone));
        }
        return skeleton;
    }

    RagdollProfile makeScenarioHumanoidProfile()
    {
        RagdollProfile profile;
        profile.mId = "scenario_humanoid_v1";

        BodyProfile pelvis = makeBody("pelvis", nullptr, 0.142f, 30);
        pelvis.mShape = box(8.0f, 11.0f, 7.0f);
        pelvis.mMaterial.mLinearDamping = 0.12f;
        pelvis.mMaterial.mAngularDamping = 0.18f;
        profile.mBodies.push_back(pelvis);

        BodyProfile spine = makeBody("spine", "pelvis", 0.100f, 30);
        spine.mShape = fittedCapsule(7.0f, 5.0f, "spine1");
        spine.mConstraint = swingTwist(0.22f, 0.18f, -0.14f, 0.14f, 5.0f, 1.0f, 343023.969f);
        profile.mBodies.push_back(spine);

        BodyProfile spine1 = makeBody("spine1", "spine", 0.100f, 30);
        spine1.mShape = fittedCapsule(7.5f, 5.5f, "spine2");
        spine1.mConstraint = swingTwist(0.20f, 0.18f, -0.14f, 0.14f, 5.0f, 1.0f, 277184.438f);
        profile.mBodies.push_back(spine1);

        BodyProfile spine2 = makeBody("spine2", "spine1", 0.100f, 30);
        spine2.mShape = fittedCapsule(8.0f, 6.0f, "neck");
        spine2.mConstraint = swingTwist(0.24f, 0.20f, -0.18f, 0.18f, 5.0f, 1.0f, 211344.906f);
        profile.mBodies.push_back(spine2);

        BodyProfile neck = makeBody("neck", "spine2", 0.015f, 0);
        neck.mShape = fittedCapsule(3.0f, 2.5f, "head");
        neck.mConstraint = swingTwist(0.35f, 0.30f, -0.35f, 0.35f, 4.0f, 1.0f, 26665.0098f);
        profile.mBodies.push_back(neck);

        BodyProfile head = makeBody("head", "neck", 0.066f, 0);
        head.mShape = sphere(7.0f);
        head.mConstraint = swingTwist(0.30f, 0.28f, -0.30f, 0.30f, 4.0f, 1.0f, 21727.0449f);
        profile.mBodies.push_back(head);

        const auto addArm = [&](const std::string& side, std::uint32_t group) {
            const std::string clavicleName = side + "_clavicle";
            const std::string upperArmName = side + "_upper_arm";
            const std::string forearmName = side + "_forearm";
            const std::string handName = side + "_hand";

            BodyProfile clavicle = makeBody(clavicleName, "spine2", 0.020f, group);
            clavicle.mShape = fittedCapsule(2.5f, 5.0f, upperArmName);
            clavicle.mConstraint = swingTwist(0.25f, 0.20f, -0.15f, 0.15f, 4.0f, 1.0f, 92175.3438f);
            profile.mBodies.push_back(clavicle);

            BodyProfile upperArm = makeBody(upperArmName, clavicleName.c_str(), 0.028f, group);
            upperArm.mShape = fittedCapsule(3.0f, 8.0f, forearmName);
            upperArm.mConstraint = swingTwist(0.85f, 0.75f, -0.65f, 0.65f, 4.0f, 1.0f, 49379.6484f);
            upperArm.mConstraint->mFrameAxisBone = forearmName;
            profile.mBodies.push_back(upperArm);

            // Derived hinge axes are mapped from model space into each bone.
            // Mirrored local bone rotations do not reverse model-space flexion:
            // both elbows use the positive range to bend forward.
            BodyProfile forearm = makeBody(forearmName, upperArmName.c_str(), 0.016f, group);
            forearm.mShape = fittedCapsule(2.4f, 7.0f, handName);
            forearm.mConstraint = hinge(-0.15f, 2.0f, 4.0f, 1.0f, 14484.6973f);
            profile.mBodies.push_back(forearm);

            BodyProfile hand = makeBody(handName, forearmName.c_str(), 0.006f, group);
            hand.mShape = box(3.0f, 4.5f, 1.8f);
            hand.mConstraint = swingTwist(0.45f, 0.35f, -0.30f, 0.30f, 4.0f, 1.0f, 1975.18604f);
            profile.mBodies.push_back(hand);
        };
        addArm("left", 10);
        addArm("right", 11);

        const auto addLeg = [&](const std::string& side, std::uint32_t group) {
            const std::string thighName = side + "_thigh";
            const std::string calfName = side + "_calf";
            const std::string footName = side + "_foot";

            BodyProfile thigh = makeBody(thighName, "pelvis", 0.100f, group);
            thigh.mShape = fittedCapsule(4.5f, 10.5f, calfName);
            thigh.mConstraint = swingTwist(0.65f, 0.90f, -0.60f, 0.60f, 4.0f, 1.0f, 822994.125f);
            thigh.mConstraint->mFrameAxisBone = calfName;
            profile.mBodies.push_back(thigh);

            BodyProfile calf = makeBody(calfName, thighName.c_str(), 0.0465f, group);
            calf.mShape = fittedCapsule(3.5f, 10.0f, footName);
            calf.mConstraint = hinge(-2.1f, 0.12f, 4.0f, 1.0f, 822994.125f);
            profile.mBodies.push_back(calf);

            BodyProfile foot = makeBody(footName, calfName.c_str(), 0.022f, group);
            // Scenario-only foot patch scale (env-gated counterfactual, inert
            // by default): scales the planar half extents of the scenario
            // rig's foot box to measure how much of the per-contact friction
            // budget failure is patch geometry. This changes the SCENARIO rig,
            // never the game rig; results are counterfactual measurements,
            // not acceptance evidence (friction-budget-decomposition-2026-09-10.txt).
            static const auto footPlanarScale_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SCENARIO_FOOT_PLANAR_SCALE");
                if (!text)
                    return 1.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed >= 0.5f && parsed <= 2.0f ? parsed : 1.0f;
            });
            static const auto& footPlanarScale = footPlanarScale_tuned.get();
            foot.mShape = box(4.0f * footPlanarScale, 8.0f * footPlanarScale, 2.5f);
            // Probe (2026-09-11): capsule foot (radius 2.5, axis y) instead of
            // the sharp box. Env-gated; unset is byte-identical.
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_FOOT_CAPSULE"); text && std::atoi(text) != 0)
            {
                foot.mShape = ShapeProfile{};
                foot.mShape.mType = ShapeType::Capsule;
                foot.mShape.mRadius = 2.5f;
                foot.mShape.mHalfHeight = 8.0f * footPlanarScale - 2.5f;
            }
            // Scenario-only ankle torque scale (env-gated counterfactual,
            // inert by default): scales the foot constraint's motor torque
            // limit to test whether the realized CoP range (and with it the
            // support-moment clamp) is bound by ankle moment CAPACITY rather
            // than patch geometry. Counterfactual measurement only.
            static const auto ankleTorqueScale_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_SCENARIO_ANKLE_TORQUE_SCALE");
                if (!text)
                    return 1.0f;
                const float parsed = std::strtof(text, nullptr);
                return std::isfinite(parsed) && parsed >= 0.5f && parsed <= 3.0f ? parsed : 1.0f;
            });
            static const auto& ankleTorqueScale = ankleTorqueScale_tuned.get();
            foot.mConstraint = swingTwist(0.32f, 0.25f, -0.18f, 0.18f, 4.0f, 1.0f, 329197.656f * ankleTorqueScale);
            profile.mBodies.push_back(foot);
        };
        addLeg("left", 20);
        addLeg("right", 21);

        profile.mLandmarks.mPelvis = "pelvis";
        profile.mLandmarks.mSpine = { "spine", "spine1", "spine2" };
        profile.mLandmarks.mHead = "head";
        profile.mLandmarks.mLeftHand = "left_hand";
        profile.mLandmarks.mRightHand = "right_hand";
        profile.mLandmarks.mLeftFoot = "left_foot";
        profile.mLandmarks.mRightFoot = "right_foot";

        // Every active-controller threshold, gain, limit, and gait value is the
        // authored humanoid-v1.yaml value. The components loader test enforces
        // parity, including removal of the former swing-strength multiplier.
        profile.mActiveController = ActiveControllerProfile{};
        profile.mActiveController.mLocomotionRoute = LocomotionRoute::KinematicRoot;
        profile.mActiveController.mLocomotionSupportGraceDuration = 0.30f;
        profile.mActiveController.mAnimationAssistStanceAnchorCompliance = 0.40f;
        // Match the explicit live YAML value; the generic profile's 1 Hz
        // default is an experiment, not the authored humanoid baseline.
        // Driver experiments can still override this gain explicitly.
        profile.mActiveController.mCenterOfMassFrequency = 0.5f;
        profile.mActiveController.mGait.mSwingLegMotorTorqueScale = 1.0f;
        return profile;
    }

    ScenarioResult runRagdollScenario(const SkeletonDescriptor& skeleton, const RagdollProfile& profile,
        const ScenarioSettings& settings, const ScenarioScript& script, const ScenarioInputProvider& inputProvider)
    {
        ScenarioResult result;
        result.mSettings = settings;
        result.mProfileHash = hashProfile(profile);
        const auto fail = [&](std::string message) { result.mErrors.push_back(std::move(message)); };
        const bool directMotorTracking
            = settings.mLocomotionController == ScenarioLocomotionController::DirectMotorTracking;
        const bool productionResidual
            = settings.mLocomotionController == ScenarioLocomotionController::ProductionResidual;

        if (!isFinite(settings.mFixedTimeStep) || settings.mFixedTimeStep <= 0.0f || settings.mCollisionSteps == 0
            || !isFinite(settings.mGravity) || settings.mGravity < 0.0f || !isFinite(settings.mGroundHeight)
            || !isFinite(settings.mGroundHalfExtent) || settings.mGroundHalfExtent <= 0.0f
            || !isFinite(settings.mGroundThickness) || settings.mGroundThickness <= 0.0f
            || !isFinite(settings.mGroundFriction) || settings.mGroundFriction < 0.0f || !isFinite(settings.mTotalMass)
            || settings.mTotalMass <= 0.0f || !isFinite(settings.mUniformScale) || settings.mUniformScale <= 0.0f
            || !isFinite(settings.mRootPosition) || !isFinite(settings.mRootRotation)
            || !settings.mRootRotation.IsNormalized(1e-3f)
            || (settings.mPreLiftSwingUnloadFraction
                && (!isFinite(*settings.mPreLiftSwingUnloadFraction)
                    || *settings.mPreLiftSwingUnloadFraction < 0.0f
                    || *settings.mPreLiftSwingUnloadFraction > 1.0f))
            || (settings.mTranslationRouting != ScenarioTranslationRouting::InternalForcePair
                && settings.mTranslationRouting != ScenarioTranslationRouting::GroundContactForce)
            || (settings.mSwingAssistAxes != ScenarioSwingAssistAxes::VerticalOnly
                && settings.mSwingAssistAxes != ScenarioSwingAssistAxes::Full3D)
            || (settings.mSwingHorizontalReaction != ScenarioSwingHorizontalReaction::SwingHip
                && settings.mSwingHorizontalReaction != ScenarioSwingHorizontalReaction::Pelvis
                && settings.mSwingHorizontalReaction != ScenarioSwingHorizontalReaction::StanceHip
                && settings.mSwingHorizontalReaction != ScenarioSwingHorizontalReaction::StanceFoot)
            || (settings.mLocomotionController != ScenarioLocomotionController::Legacy
                && settings.mLocomotionController != ScenarioLocomotionController::Reference
                && !directMotorTracking && !productionResidual))
            fail("Scenario settings must be finite, positive where required, and use a normalized root rotation.");
        if (directMotorTracking
            && (settings.mPinPelvis || settings.mUseSolverActuation
                || settings.mUseWorldSwingHipControl || settings.mCompensateSwingHipMotorImpulse
                || settings.mReserveStanceHipMotor || settings.mDisableSwingHipMotor
                || settings.mReplayNativeBraceLimits || settings.mReplayBraceTargets
                || settings.mReplayAutomaticRecoveryRequest || settings.mReplayRecoveryFinishEventsCaptured
                || !settings.mReplayRecoveryRequestSteps.empty() || !settings.mReplayRecoveryFinishSteps.empty()
                || !script.mGaitPoseCycle.empty()))
            fail("Direct motor tracking cannot combine with pinned support, actuator overrides, recovery replay "
                "or a procedural gait cycle.");
        if (settings.mCompensateSwingHipMotorImpulse && !settings.mReserveStanceHipMotor)
            fail("Swing-hip motor compensation requires a reserved stance-hip motor.");
        if (!directMotorTracking && settings.mUseJointSupportControl && settings.mGravity <= 0.0f)
            fail("Joint support requires positive gravity.");
        if (settings.mUseWorldSwingHipControl
            && (settings.mDisableSwingHipMotor || settings.mReserveStanceHipMotor
                || settings.mCompensateSwingHipMotorImpulse))
            fail("World swing-hip control owns both hip motors and cannot be combined with hip diagnostics.");

        if (!isFinite(script.mDurationSeconds) || script.mDurationSeconds <= 0.0f)
            fail("Scenario duration must be finite and positive.");
        else if (isFinite(settings.mFixedTimeStep) && settings.mFixedTimeStep > 0.0f)
        {
            const double steps = std::round(static_cast<double>(script.mDurationSeconds) / settings.mFixedTimeStep);
            if (steps < 1.0 || steps > 10000000.0)
                fail("Scenario duration must cover between one and ten million fixed steps.");
            else
                result.mStepCount = static_cast<std::uint64_t>(steps);
        }

        if (script.mGaitPoseCycle.size() == 1)
            fail("A scenario gait pose cycle requires at least two samples.");
        for (std::size_t sampleIndex = 0; sampleIndex < script.mGaitPoseCycle.size(); ++sampleIndex)
        {
            const RagdollPose& sample = script.mGaitPoseCycle[sampleIndex];
            if (sample.mBodyTransforms.size() != profile.mBodies.size())
            {
                fail("Scenario gait pose sample " + std::to_string(sampleIndex) + " does not match the body count.");
                continue;
            }
            for (const Transform& transform : sample.mBodyTransforms)
            {
                if (!isFinite(transform))
                {
                    fail("Scenario gait pose sample " + std::to_string(sampleIndex) + " contains non-finite values.");
                    break;
                }
            }
        }

        const auto validateImpulse = [&](const ScenarioImpulse& impulse, std::uint64_t i) {
            if (impulse.mBodyIndex >= profile.mBodies.size())
                fail("Scenario impulse " + std::to_string(i) + " targets a body outside the profile.");
            if (!isFinite(impulse.mImpulse) || (impulse.mWorldPoint && !isFinite(*impulse.mWorldPoint)))
                fail("Scenario impulse " + std::to_string(i) + " contains non-finite values.");
        };
        const auto validateKeyframe = [&](const ScenarioInputKeyframe& keyframe, std::uint64_t i) {
            for (const auto& impulse : keyframe.mPrePlanningImpulses)
            {
                validateImpulse(impulse, i);
                if (impulse.mStep != keyframe.mStep) fail("Pre-planning impulse must name its keyframe step.");
            }
            for (std::size_t j = 0; j < keyframe.mImpulses.size(); ++j)
            {
                const auto& impulse = keyframe.mImpulses[j];
                if (impulse.mStep != keyframe.mStep)
                    fail("Scenario keyframe impulse must name its keyframe's step.");
                validateImpulse(impulse, j);
            }
            if (!isFinite(keyframe.mRequestedHorizontalVelocity))
                fail("Scenario keyframe " + std::to_string(i) + " has a non-finite requested velocity.");
            if (!std::isfinite(keyframe.mStabilizationStrength)
                || keyframe.mStabilizationStrength < 0.0f || keyframe.mStabilizationStrength > 1.0f)
                fail("Scenario keyframe " + std::to_string(i)
                    + " has stabilization strength outside [0, 1].");
            if (keyframe.mRecoveryPelvisHeightScale
                && (!std::isfinite(*keyframe.mRecoveryPelvisHeightScale)
                    || *keyframe.mRecoveryPelvisHeightScale < 0.15f
                    || *keyframe.mRecoveryPelvisHeightScale > 1.0f
                    || !keyframe.mRecoveryMotorTarget))
                fail("Recovery height scale requires a recovery pose and a fraction in [0.15, 1].");
            if (directMotorTracking && (keyframe.mSupportMotion || keyframe.mRecoveryMotorTarget
                    || keyframe.mRecoveryRequested || keyframe.mRecoveryAnimationFinished))
                fail("Direct motor tracking does not accept support or recovery actuation inputs.");
            if (keyframe.mSupportMotion)
            {
                const auto& motion = *keyframe.mSupportMotion;
                if (!isFinite(motion.mPosition) || !isFinite(motion.mVelocity) || !isFinite(motion.mAcceleration))
                    fail("Scenario support motion contains non-finite values.");
                if (!horizontal(keyframe.mRequestedHorizontalVelocity).IsNearZero()
                    || !settings.mUseJointSupportControl || settings.mPinPelvis)
                    fail("Scenario support motion requires unpinned standing joint support.");
            }
            if (keyframe.mRecoveryMotorTarget && !keyframe.mMotorTarget)
                fail("Recovery motor candidate requires a paired locomotion candidate.");
            if (keyframe.mKinematicRecovery && (!keyframe.mRecoveryMotorTarget
                    || !keyframe.mKinematicRecoveryTerminalPose || settings.mPinPelvis))
                fail("Kinematic recovery requires recovery/terminal poses and an unpinned pelvis.");
            if (keyframe.mHybridStage > 3 || (keyframe.mKinematicLocomotion
                    && (!keyframe.mMotorTarget || settings.mPinPelvis || directMotorTracking
                        || keyframe.mKinematicRecovery)))
                fail("Kinematic locomotion requires a pose and exclusive root ownership.");
            for (const auto* candidate : { &keyframe.mMotorTarget, &keyframe.mRecoveryMotorTarget,
                     &keyframe.mKinematicRecoveryTerminalPose })
            {
                if (!*candidate) continue;
                if ((*candidate)->mBodyTransforms.size() != profile.mBodies.size())
                    fail("Scenario keyframe " + std::to_string(i) + " motor target does not match the body count.");
                for (const Transform& transform : (*candidate)->mBodyTransforms)
                {
                    if (!isFinite(transform))
                    {
                        fail("Scenario keyframe " + std::to_string(i) + " motor target contains non-finite values.");
                        break;
                    }
                    if (!transform.mRotation.IsNormalized(1e-3f))
                    {
                        fail("Scenario keyframe " + std::to_string(i) + " motor target rotation must be normalized.");
                        break;
                    }
                }
            }
            if (!keyframe.mMotorTargetAngularVelocities.empty())
            {
                if (productionResidual || keyframe.mMotorTargetAngularVelocities.size() != profile.mBodies.size()
                    || (!directMotorTracking && !keyframe.mRequestedHorizontalVelocity.IsNearZero()))
                {
                    fail("Motor velocity reference requires one entry per body and standing or direct tracking; "
                        "production residual retains production velocity targets.");
                    return;
                }
                for (std::size_t body = 0; body < profile.mBodies.size(); ++body)
                    if (!isFinite(keyframe.mMotorTargetAngularVelocities[body])
                        || (!profile.mBodies[body].mConstraint
                            && !keyframe.mMotorTargetAngularVelocities[body].IsNearZero())
                        || (directMotorTracking
                            && (!profile.mBodies[body].mConstraint
                                || profile.mBodies[body].mConstraint->mMotor.mMaximumTorque == 0.0f)
                            && keyframe.mMotorTargetAngularVelocities[body].LengthSq() != 0.0f))
                        fail("Motor velocity reference is invalid or drives a root/passive joint.");
            }
            if (!keyframe.mMotorTargetRotationOffsets.empty())
            {
                if ((!directMotorTracking && !productionResidual)
                    || keyframe.mMotorTargetRotationOffsets.size() != profile.mBodies.size())
                {
                    fail("Motor rotation offsets require direct tracking or production residual and one entry per body.");
                    return;
                }
                const float maximum = productionResidual
                    ? ScenarioInputKeyframe::sMaximumProductionMotorRotationOffset
                    : ScenarioInputKeyframe::sMaximumDirectMotorRotationOffset;
                for (std::size_t body = 0; body < profile.mBodies.size(); ++body)
                {
                    const auto offset = keyframe.mMotorTargetRotationOffsets[body];
                    const auto& constraint = profile.mBodies[body].mConstraint;
                    if (!isFinite(offset) || offset.LengthSq() > maximum * maximum
                        || ((!constraint || constraint->mMotor.mMaximumTorque == 0.0f)
                            && offset.LengthSq() != 0.0f))
                        fail("Motor rotation offset is non-finite, outside its bound, or drives a root/passive joint.");
                }
            }
        };
        for (std::size_t i = 0; i < script.mKeyframes.size(); ++i)
        {
            const ScenarioInputKeyframe& keyframe = script.mKeyframes[i];
            if (i > 0 && keyframe.mStep <= script.mKeyframes[i - 1].mStep)
                fail("Scenario keyframes must be sorted by step and unique.");
            validateKeyframe(keyframe, i);
        }
        for (std::size_t i = 0; i < script.mImpulses.size(); ++i)
            validateImpulse(script.mImpulses[i], i);
        if (script.mKillingBlow
            && (!isFinite(script.mKillingBlow->mWorldPoint) || !isFinite(script.mKillingBlow->mDirection)
                || script.mKillingBlow->mDirection.IsNearZero() || !isFinite(script.mKillingBlow->mHealthDamage)
                || script.mKillingBlow->mHealthDamage <= 0.0f))
            fail("Scenario killing blow requires a finite point, a non-zero direction, and positive damage.");
        if (!result.mErrors.empty())
            return result;

        const float preLiftSwingUnloadFraction = settings.mPreLiftSwingUnloadFraction.value_or(
            profile.mActiveController.mGait.mPreLiftSwingUnloadFraction);

        ScenarioBroadPhaseLayers broadPhaseLayers;
        ScenarioObjectVsBroadPhase objectVsBroadPhase;
        ScenarioObjectPairs objectPairs;
        ScenarioContactListener listener;
        if (!std::is_sorted(settings.mReplayRecoveryRequestSteps.begin(), settings.mReplayRecoveryRequestSteps.end())
            || std::adjacent_find(settings.mReplayRecoveryRequestSteps.begin(), settings.mReplayRecoveryRequestSteps.end())
                != settings.mReplayRecoveryRequestSteps.end()
            || (!settings.mReplayRecoveryRequestSteps.empty() && settings.mReplayRecoveryRequestSteps.back() >= result.mStepCount))
            fail("Invalid recovery request event timeline.");
        if ((!settings.mReplayRecoveryFinishEventsCaptured && !settings.mReplayRecoveryFinishSteps.empty())
            || !std::is_sorted(settings.mReplayRecoveryFinishSteps.begin(), settings.mReplayRecoveryFinishSteps.end())
            || std::adjacent_find(settings.mReplayRecoveryFinishSteps.begin(), settings.mReplayRecoveryFinishSteps.end())
                != settings.mReplayRecoveryFinishSteps.end()
            || (!settings.mReplayRecoveryFinishSteps.empty()
                && settings.mReplayRecoveryFinishSteps.back() >= result.mStepCount))
        { fail("Invalid replay recovery-finish event timeline"); return result; }
        if (!settings.mReplayReferenceFrames.empty())
        {
            if (settings.mReplayReferenceFrames.size() != result.mStepCount)
            { fail("Replay reference-frame count mismatch"); return result; }
            for (const auto& reference : settings.mReplayReferenceFrames)
                if (!isFinite(reference.mPosition) || !isFinite(reference.mRotation)
                    || !reference.mRotation.IsNormalized())
                { fail("Invalid replay reference frame"); return result; }
        }
        std::uint32_t groundIndex = 0;
        if (settings.mReplayGroundId)
        {
            const auto id = *settings.mReplayGroundId;
            if ((id & JPH::BodyID::cBroadPhaseBit) || (id & JPH::BodyID::cMaxBodyIndex) >= 4096
                || settings.mReplayGroundShape.empty())
            { fail("Invalid replay ground capture"); return result; }
            groundIndex = JPH::BodyID(id).GetIndex();
        }
        std::vector<std::uint32_t> indices;
        if (!settings.mReplayBodyIds.empty())
        {
            if (settings.mReplayBodyIds.size() != profile.mBodies.size())
            { fail("Replay body ID count mismatch"); return result; }
            for (const auto raw : settings.mReplayBodyIds)
            {
                if (raw & JPH::BodyID::cBroadPhaseBit)
                { fail("Invalid replay body ID"); return result; }
                const JPH::BodyID id(raw);
                if (id.GetIndex() == groundIndex || id.GetIndex() >= 4096 || id.GetSequenceNumber() < 1
                    || id.GetSequenceNumber() > 16
                    || std::find(indices.begin(), indices.end(), id.GetIndex()) != indices.end())
                { fail("Replay body ID outside supported allocation history"); return result; }
                indices.push_back(id.GetIndex());
            }
        }
        JPH::PhysicsSystem physics;
        physics.Init(settings.mReplayBodyIds.empty() ? 256 : 4096, 0, 1024, 1024, broadPhaseLayers, objectVsBroadPhase, objectPairs);
        physics.SetGravity(JPH::Vec3(0.0f, 0.0f, -settings.mGravity));
        physics.SetContactListener(&listener);
        // The scenario uses OpenMW world units while Jolt's default length
        // and linear-velocity tolerances are metres. Mirror the live engine's
        // unit scaling so loaded feet do not creep through the ground merely
        // because the offline harness has a different solver configuration.
        configureScenarioPhysics(physics, settings.mVelocitySolverSteps);
        JPH::TempAllocatorImpl allocator(8 * 1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        JPH::BodyInterface& bodies = physics.GetBodyInterface();

        JPH::Ref<JPH::Shape> groundShape = new JPH::BoxShape(
            JPH::Vec3(settings.mGroundHalfExtent, settings.mGroundHalfExtent, settings.mGroundThickness * 0.5f));
        if (settings.mTriangleMeshGround)
        {
            const float e = settings.mGroundHalfExtent;
            const float h = settings.mGroundThickness * 0.5f;
            JPH::VertexList vertices{
                { -e, -e, h }, { e, -e, h }, { e, e, h }, { -e, e, h },
                { -e, -e, -h }, { e, -e, -h }, { e, e, -h }, { -e, e, -h },
            };
            JPH::IndexedTriangleList triangles{
                { 0, 1, 2 }, { 0, 2, 3 }, { 4, 6, 5 }, { 4, 7, 6 },
                { 0, 4, 5 }, { 0, 5, 1 }, { 1, 5, 6 }, { 1, 6, 2 },
                { 2, 6, 7 }, { 2, 7, 3 }, { 3, 7, 4 }, { 3, 4, 0 },
            };
            // Mesh-floor probes (2026-09-11, strafe-native-input): the
            // two-triangle top face puts its internal diagonal under the
            // walker's start. OPENMW_RAGDOLL_MESH_GRID=N replaces the top
            // face by an N x N grid of triangles (many internal edges,
            // like a real cell floor); OPENMW_RAGDOLL_MESH_OFFSET=dx,dy
            // shifts the whole mesh so the diagonal moves away from the
            // start. Env-gated; unset is byte-identical.
            if (const char* gridText = RagdollCore::tuningText("OPENMW_RAGDOLL_MESH_GRID"); gridText && std::atoi(gridText) >= 1)
            {
                const int n = std::atoi(gridText);
                vertices.clear();
                triangles.clear();
                for (int j = 0; j <= n; ++j)
                    for (int i = 0; i <= n; ++i)
                        vertices.push_back({ -e + 2.0f * e * static_cast<float>(i) / static_cast<float>(n),
                            -e + 2.0f * e * static_cast<float>(j) / static_cast<float>(n), h });
                const auto at = [n](int i, int j) { return static_cast<std::uint32_t>(j * (n + 1) + i); };
                for (int j = 0; j < n; ++j)
                    for (int i = 0; i < n; ++i)
                    {
                        triangles.push_back({ at(i, j), at(i + 1, j), at(i + 1, j + 1) });
                        triangles.push_back({ at(i, j), at(i + 1, j + 1), at(i, j + 1) });
                    }
            }
            if (const char* offsetText = RagdollCore::tuningText("OPENMW_RAGDOLL_MESH_OFFSET"))
            {
                float dx = 0.0f, dy = 0.0f;
                if (std::sscanf(offsetText, "%f,%f", &dx, &dy) == 2)
                    for (auto& vertex : vertices)
                    {
                        vertex.x += dx;
                        vertex.y += dy;
                    }
            }
            const auto mesh = JPH::MeshShapeSettings(vertices, triangles).Create();
            if (mesh.HasError())
            {
                fail("Jolt could not create the triangle-mesh floor: " + std::string(mesh.GetError().c_str()));
                return result;
            }
            groundShape = mesh.Get();
        }
        if (settings.mReplayGroundId)
        {
            const auto& bytes = settings.mReplayGroundShape;
            std::istringstream data(std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size()), std::ios::binary);
            JPH::StreamInWrapper stream(data);
            JPH::Shape::IDToShapeMap shapes; JPH::Shape::IDToMaterialMap materials;
            auto restored = JPH::Shape::sRestoreWithChildren(stream, shapes, materials);
            if (restored.HasError() || stream.IsFailed())
            { fail("Unable to restore captured ground shape"); return result; }
            groundShape = restored.Get();
        }
        JPH::BodyCreationSettings groundSettings(groundShape,
            JPH::RVec3(0.0, 0.0, static_cast<double>(settings.mGroundHeight) - settings.mGroundThickness * 0.5),
            JPH::Quat::sIdentity(), JPH::EMotionType::Static, sGroundLayer);
        groundSettings.mFriction = settings.mGroundFriction;
        groundSettings.mRestitution = 0.0f;
        // Probe (2026-09-11): internal edge removal on the floor body too.
        groundSettings.mEnhancedInternalEdgeRemoval = RagdollCore::tuningText("OPENMW_RAGDOLL_GROUND_EIER") != nullptr;
        const JPH::BodyID ground = [&]() {
            if (settings.mReplayGroundId)
            {
                groundSettings.mPosition = settings.mReplayGroundPosition;
                groundSettings.mRotation = settings.mReplayGroundRotation;
                groundSettings.mRestitution = settings.mReplayGroundRestitution;
                auto* body = bodies.CreateBodyWithID(JPH::BodyID(*settings.mReplayGroundId), groundSettings);
                if (!body) return JPH::BodyID();
                bodies.AddBody(body->GetID(), JPH::EActivation::DontActivate);
                return body->GetID();
            }
            return bodies.CreateAndAddBody(groundSettings, JPH::EActivation::DontActivate);
        }();
        if (ground.IsInvalid())
        {
            fail("Jolt could not create the ground plane.");
            return result;
        }

        BuildOptions buildOptions;
        buildOptions.mTotalMass = settings.mTotalMass;
        buildOptions.mUseContinuousCollision = settings.mUseContinuousCollision;
        buildOptions.mUniformScale = settings.mUniformScale;
        buildOptions.mRootPosition = settings.mRootPosition;
        buildOptions.mRootRotation = settings.mRootRotation;
        buildOptions.mObjectLayer = sRagdollLayer;
        PassiveRagdollInitialState initialState;
        initialState.mActorMotion = settings.mInitialActorMotion;
        // Reserve native indices without adding placeholder bodies to physics.
        // Recycle selected slots to reproduce sequence numbers, then release
        // in reverse profile order so normal ragdoll creation consumes them.
        struct ReservedIds
        {
            JPH::BodyInterface& mBodies;
            std::vector<JPH::BodyID> mIds;
            ~ReservedIds() { for (const auto id : mIds) if (!id.IsInvalid()) mBodies.DestroyBody(id); }
        } reserved{ bodies, {} };
        if (!settings.mReplayBodyIds.empty())
        {
            const auto maximum = *std::max_element(indices.begin(), indices.end());
            reserved.mIds.resize(maximum + 1);
            JPH::BodyCreationSettings placeholder(groundShape, JPH::RVec3::sZero(), JPH::Quat::sIdentity(),
                JPH::EMotionType::Static, sGroundLayer);
            for (std::uint32_t index = 0; index <= maximum; ++index)
            {
                if (index == groundIndex) continue;
                auto* body = bodies.CreateBodyWithID(JPH::BodyID(index, 0), placeholder);
                if (!body) { fail("Unable to reserve replay body IDs"); return result; }
                reserved.mIds[index] = body->GetID();
            }
            for (const auto raw : settings.mReplayBodyIds)
            {
                const JPH::BodyID wanted(raw); auto& held = reserved.mIds[wanted.GetIndex()];
                while (held.GetSequenceNumber() + 1 < wanted.GetSequenceNumber())
                {
                    bodies.DestroyBody(held); held = JPH::BodyID();
                    auto* body = bodies.CreateBody(placeholder);
                    if (!body) { fail("Unable to recycle replay body ID"); return result; }
                    held = body->GetID();
                }
            }
            for (auto it = settings.mReplayBodyIds.rbegin(); it != settings.mReplayBodyIds.rend(); ++it)
            {
                auto& held = reserved.mIds[JPH::BodyID(*it).GetIndex()];
                bodies.DestroyBody(held); held = JPH::BodyID();
            }
        }
        PassiveRagdollCreateResult created
            = createPassiveRagdoll(physics, skeleton, profile, buildOptions, initialState, 1, sRagdollUserData);
        std::unique_ptr<PassiveRagdoll> ragdoll = std::move(created.mRagdoll);
        struct GroundCleanup
        {
            JPH::BodyInterface& mBodies;
            JPH::BodyID mGround;
            std::unique_ptr<PassiveRagdoll>& mRagdoll;
            ~GroundCleanup()
            {
                mRagdoll.reset();
                mBodies.RemoveBody(mGround);
                mBodies.DestroyBody(mGround);
            }
        } cleanup{ bodies, ground, ragdoll };
        if (ragdoll == nullptr || !created.mValidation.isValid())
        {
            for (const ValidationIssue& issue : created.mValidation.mIssues)
                fail(issue.mPath + ": " + issue.mMessage);
            if (result.mErrors.empty())
                fail("Ragdoll creation failed without a validation issue.");
            return result;
        }

        for (std::size_t i = 0; i < settings.mReplayBodyIds.size(); ++i)
            if (ragdoll->getBodyId(i).GetIndexAndSequenceNumber() != settings.mReplayBodyIds[i])
            { fail("Replay body ID allocation did not match"); return result; }
        if (!ragdoll->addToPhysicsSystem())
        {
            fail("Ragdoll bodies could not be added to the physics system.");
            return result;
        }
        listener.registerRagdoll(*ragdoll);
        const std::size_t bodyCount = ragdoll->getBodyCount();
        for (const BodyProfile& body : profile.mBodies)
            result.mBodyNames.push_back(body.mName);

        const auto findBody = [&](const std::string& name) -> std::optional<std::size_t> {
            for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
            {
                if (profile.mBodies[i].mName == name)
                    return i;
            }
            return std::nullopt;
        };
        const std::optional<std::size_t> pelvisIndex = findBody(profile.mLandmarks.mPelvis);
        const std::optional<std::size_t> torsoIndex
            = profile.mLandmarks.mSpine.empty() ? std::nullopt : findBody(profile.mLandmarks.mSpine.back());
        const std::optional<std::size_t> leftFootIndex
            = profile.mLandmarks.mLeftFoot ? findBody(*profile.mLandmarks.mLeftFoot) : std::nullopt;
        const std::optional<std::size_t> rightFootIndex
            = profile.mLandmarks.mRightFoot ? findBody(*profile.mLandmarks.mRightFoot) : std::nullopt;
        if (!pelvisIndex || !torsoIndex || !leftFootIndex || !rightFootIndex || *pelvisIndex == *torsoIndex
            || *leftFootIndex == *rightFootIndex)
        {
            fail("Scenario profiles require pelvis, spine, left-foot, and right-foot landmarks.");
            return result;
        }
        result.mLandmarks = { *pelvisIndex, *torsoIndex, *leftFootIndex, *rightFootIndex };
        const FootBodyIndices feet{ *leftFootIndex, *rightFootIndex };
        const auto validateSupportFoot = [&](const ScenarioInputKeyframe& key) {
            if (key.mSupportMotion && key.mSupportMotion->mSupportFoot
                && *key.mSupportMotion->mSupportFoot != feet.mLeft
                && *key.mSupportMotion->mSupportFoot != feet.mRight)
                fail("Scenario support motion must select a foot landmark.");
        };
        for (const auto& key : script.mKeyframes)
            validateSupportFoot(key);
        if (!result.mErrors.empty())
            return result;
        if (settings.mPinPelvis)
        {
            bodies.SetMotionType(
                ragdoll->getBodyId(*pelvisIndex), JPH::EMotionType::Static, JPH::EActivation::DontActivate);
        }

        // Hip, knee, and foot bodies for the physics-owned swing trajectory.
        const auto parentBody = [&](std::size_t index) -> std::optional<std::size_t> {
            if (!profile.mBodies[index].mParentBody)
                return std::nullopt;
            return findBody(*profile.mBodies[index].mParentBody);
        };
        const auto legChain = [&](std::size_t foot) -> std::optional<LegChain> {
            const std::optional<std::size_t> knee = parentBody(foot);
            if (!knee)
                return std::nullopt;
            const std::optional<std::size_t> hip = parentBody(*knee);
            if (!hip)
                return std::nullopt;
            return LegChain{ *hip, *knee, foot };
        };
        const std::optional<LegChain> leftLeg = legChain(*leftFootIndex);
        const std::optional<LegChain> rightLeg = legChain(*rightFootIndex);
        if (!leftLeg || !rightLeg)
        {
            fail("Scenario profiles require thigh, calf, and foot bodies on each leg.");
            return result;
        }

        // The scripted killing blow uses the same conversion as a gameplay hit
        // and is applied after the bodies have entered broadphase.
        if (script.mKillingBlow)
        {
            const std::optional<KillingImpulse> impulse = makeKillingImpulse(skeleton, profile, buildOptions,
                script.mKillingBlow->mWorldPoint, script.mKillingBlow->mDirection, script.mKillingBlow->mHealthDamage);
            if (!impulse || impulse->mBodyIndex >= bodyCount)
            {
                fail("The killing blow could not be converted into a bounded impulse.");
                return result;
            }
            bodies.AddImpulse(ragdoll->getBodyId(impulse->mBodyIndex), impulse->mImpulse, impulse->mWorldPoint);
            result.mKillingImpulse = impulse;
        }

        RagdollTelemetry telemetry;
        RagdollPose pose;
        if (!ragdoll->captureTelemetry(telemetry) || telemetry.mBodies.size() != bodyCount
            || !ragdoll->captureModelPose(pose) || pose.mBodyTransforms.size() != bodyCount)
        {
            fail("Initial ragdoll telemetry could not be captured.");
            return result;
        }
        result.mInitialTelemetry = telemetry;
        const JPH::Vec3 pelvisLocalUp = telemetry.mBodies[*pelvisIndex].mRotation.Conjugated() * JPH::Vec3::sAxisZ();
        const JPH::Vec3 torsoLocalUp = telemetry.mBodies[*torsoIndex].mRotation.Conjugated() * JPH::Vec3::sAxisZ();
        const JPH::Vec3 torsoLocalForward = telemetry.mBodies[*torsoIndex].mRotation.Conjugated()
            * (settings.mRootRotation * JPH::Vec3::sAxisY());
        const float initialPelvisHeight = static_cast<float>(
            static_cast<double>(telemetry.mBodies[*pelvisIndex].mCenterOfMass.GetZ()) - settings.mGroundHeight);
        if (!pelvisLocalUp.IsNormalized() || !torsoLocalUp.IsNormalized() || !isFinite(initialPelvisHeight)
            || initialPelvisHeight < 0.0f)
        {
            fail("Initial pose does not define an upright pelvis above the feet.");
            return result;
        }
        std::unordered_map<std::string, const SkeletonBone*> bones;
        for (const SkeletonBone& bone : skeleton.mBones)
            bones.emplace(bone.mName, &bone);
        RagdollPose motorTarget;
        motorTarget.mBodyTransforms.reserve(bodyCount);
        for (const BodyProfile& body : profile.mBodies)
            motorTarget.mBodyTransforms.push_back(bones.at(body.mBone)->mModelTransform);

        // World-to-model conversion for a fixed reference transform. The
        // ordinary harness uses its creation root; replay follows recorded
        // coordinate frames without moving the physical bodies.
        JPH::RVec3 currentRootPosition = settings.mRootPosition;
        JPH::Quat currentRootRotation = settings.mRootRotation;
        const auto toModel = [&](JPH::RVec3Arg world) {
            return (currentRootRotation.Conjugated() * JPH::Vec3(world - currentRootPosition)) / settings.mUniformScale;
        };
        const auto toWorld = [&](JPH::Vec3Arg model) {
            return currentRootPosition + currentRootRotation * (model * settings.mUniformScale);
        };

        // Order impulses by step while preserving authored order within a
        // step. std::sort is used with an explicit index key because MSVC's
        // stable sort allocates aligned temporary storage that rejects the
        // extended alignment of Jolt vectors.
        std::vector<std::pair<std::size_t, ScenarioImpulse>> indexedImpulses;
        indexedImpulses.reserve(script.mImpulses.size());
        for (std::size_t i = 0; i < script.mImpulses.size(); ++i)
            indexedImpulses.emplace_back(i, script.mImpulses[i]);
        std::sort(indexedImpulses.begin(), indexedImpulses.end(),
            [](const std::pair<std::size_t, ScenarioImpulse>& a, const std::pair<std::size_t, ScenarioImpulse>& b) {
                return std::make_pair(a.second.mStep, a.first) < std::make_pair(b.second.mStep, b.first);
            });
        std::vector<ScenarioImpulse> impulses;
        impulses.reserve(indexedImpulses.size());
        for (const auto& [_, impulse] : indexedImpulses)
            impulses.push_back(impulse);

        ActiveRagdollController controller;
        SettledPelvisHeightTracker pelvisHeightTracker;
        SettledPelvisHeightOutput pelvisHeightOutput;
        GaitPhaseController gaitController;
        std::optional<ActiveControllerOutput> controllerOutput;
        bool observedSupportMotorEligible = false;
        bool replayRecoveryHandoverPending = false;
        bool replayRecoveryPosePending = false;
        std::optional<GaitControllerOutput> gaitOutput;
        std::optional<JPH::RVec3> swingFootTarget;
        bool backwardRecoveryStep = false;
        std::optional<JPH::Vec3> swingStartModel;
        std::optional<UprightControllerInput> uprightInput;
        std::optional<UprightControllerOutput> uprightOutput;
        GaitActuationStage uprightStage = GaitActuationStage::Inactive;
        std::optional<SupportEstimate> supportEstimate;
        ScenarioInputKeyframe active;
        std::size_t keyframeCursor = 0;
        std::size_t impulseCursor = 0;
        // Reference locomotion route state. Load and measured motor torques
        // of the previous step feed the root reaction accounting.
        const bool useReferenceGait
            = settings.mLocomotionController == ScenarioLocomotionController::Reference
            || productionResidual;
        ReferenceGaitStateMachine referenceGait;
        ReferenceGaitState referenceState;
        ReferenceGaitState steeringObservedGaitState;
        // The raw keyframe request and the steering filter that turns it into
        // the command the reference route actually follows.
        ReferenceGaitSteering referenceSteering;
        // i46 (turn-rate-sweep-2026-09-12.txt): measurement probe. Scale the
        // steering turn rate via env to test whether the turn90f slide is
        // rate-bound; unset leaves the profile value exactly (the copy is
        // then identical and every downstream byte is unchanged).
        ReferenceGaitProfile steeringGait = settings.mReferenceGait;
        static const auto turnRateScale_tuned = RagdollCore::tuned<float>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_TURN_RATE_SCALE");
            if (!text)
                return 1.0f;
            const float parsed = std::strtof(text, nullptr);
            return std::isfinite(parsed) && parsed > 0.0f && parsed <= 1.0f ? parsed : 1.0f;
        });
        static const auto& turnRateScale = turnRateScale_tuned.get();
        steeringGait.mSteeringTurnRate = settings.mReferenceGait.mSteeringTurnRate * turnRateScale;
        JPH::Vec3 steeringRequest = JPH::Vec3::sZero();
        // i39 (delivery-comp-2026-09-11.txt): last control plan's predicted
        // touchdown velocity, fed back to the state machine for the
        // delivery-error observer. Only updated from operative plans
        // (never the read-only queries) that actually computed a placement.
        std::optional<JPH::Vec3> referencePlacementPredicted;
        std::optional<JPH::Vec3> referenceSwingStartModel;
        std::optional<SwingTrajectoryHistory> referenceSwingHistory;
        // A116 temporary online intervention state, independent of read-only diagnostics.
        std::optional<ClearLimbPose> diagnosticOnlinePose;
        std::optional<ClearLimbPose> diagnosticOnlineNext;
        std::array<LimbShapePose, 3> diagnosticOnlineSupport;
        JPH::Quat diagnosticOnlineParent = JPH::Quat::sIdentity();
        std::optional<std::uint64_t> diagnosticOnlineStep;
        std::size_t diagnosticOnlineFoot = 0;
        // Read-only replay route query. Never feeds the controller or capture.
        std::optional<ClearLimbPose> diagnosticLimbPose;
        std::array<LimbShapePose, 3> diagnosticLimbSupport;
        JPH::Quat diagnosticLimbParent = JPH::Quat::sIdentity();
        std::optional<std::uint64_t> diagnosticLimbStep;
        std::size_t diagnosticLimbFoot = 0;
        bool diagnosticLimbBlocked = false;
        struct DiagnosticLimbRoute
        {
            std::vector<ClearLimbPose> mPath;
            double mCost = 0;
        };
        std::vector<DiagnosticLimbRoute> diagnosticLimbRoutes;
        bool diagnosticRouteBlocked = false;
        std::optional<JPH::Vec3> referenceLandingFootModel;
        std::optional<JPH::RVec3> referencePlacement;
        // Stance-knee target of the previous accepted plan (body index and
        // model rotation), for the takeoff command-discontinuity metric.
        std::optional<std::pair<std::size_t, JPH::Quat>> previousStanceKneeTarget;
        std::optional<std::tuple<std::size_t, std::uint64_t, float>> previousStanceHingeTarget;
        std::vector<JPH::Vec3> referenceLoadTorques(bodyCount, JPH::Vec3::sZero());
        std::vector<JPH::Vec3> referenceMotorTorques(bodyCount, JPH::Vec3::sZero());
        bool referenceHistory = false;
        bool referenceSpringsActive = false;
        bool directMotorAxesCaptured = false;
        ReferenceGaitHandover referenceHandover;
        std::optional<RagdollPose> referenceStandingLegPose;
        std::vector<std::size_t> referenceSpringBodies;
        if (useReferenceGait && leftLeg && rightLeg)
        {
            for (const LegChain* leg : { &*leftLeg, &*rightLeg })
                for (const std::size_t bodyIndex : { leg->mHip, leg->mKnee, leg->mFoot })
                    referenceSpringBodies.push_back(bodyIndex);
            for (const std::string& spineName : profile.mLandmarks.mSpine)
                if (const auto spineIndex = findBody(spineName))
                    referenceSpringBodies.push_back(*spineIndex);
        }
        const auto referenceLeg = [&](const LegChain& leg) {
            ReferenceGaitLeg result{ leg.mHip, leg.mKnee, leg.mFoot,
                ragdoll->getHingeLegGeometry(leg.mHip, leg.mKnee, leg.mFoot) };
            if (settings.mReferenceGaitCoordinatesLegPose || settings.mReferenceGait.mCoordinateInitialTransfer
                || settings.mReferenceGait.mCoordinateWeightTransfers
                || settings.mReferenceGait.mFinalCommandFeasibilityStanceMargin > 0.0f
                || settings.mReferenceGait.mFinalCommandFeasibilitySwingMargin > 0.0f)
            {
                result.mHipGeometry = ragdoll->getSwingTwistJointGeometry(leg.mHip);
                result.mAnkleGeometry = ragdoll->getSwingTwistJointGeometry(leg.mFoot);
            }
            return result;
        };
        const auto recordForcePair
            = [&](ScenarioFrame& frame, std::span<const std::size_t> reactionBodies, JPH::Vec3Arg force) {
                  if (reactionBodies.empty() || force.IsNearZero())
                      return JPH::Vec3::sZero();
                  const JPH::Vec3 share = force / static_cast<float>(reactionBodies.size());
                  frame.mAppliedNetForce += force;
                  frame.mAppliedNetForce -= share * static_cast<float>(reactionBodies.size());
                  // PassiveRagdoll applies both forces at the same world point, so
                  // the pair also contributes exactly zero net moment.
                  return JPH::Vec3::sZero();
              };
        result.mFrames.reserve(static_cast<std::size_t>(result.mStepCount));
        if (!settings.mPowered)
            ragdoll->disableMotors();

        bool replayBraceLimitsOpened = false;
        bool replayRecoveryRequested = false;
        struct ReplayBraceArm
        {
            std::array<std::size_t, 4> mBodies{}; // frame, shoulder, elbow, hand
            std::optional<BraceHandTarget> mTarget;
            float mWeight = 0;
            bool mOpen = false;
        };
        std::array<std::optional<ReplayBraceArm>, 2> replayArms;
        if (settings.mReplayBraceTargets)
        {
            const std::array hands{ profile.mLandmarks.mLeftHand, profile.mLandmarks.mRightHand };
            for (std::size_t side = 0; side < hands.size(); ++side)
            {
                auto body = hands[side] ? findBody(*hands[side]) : std::nullopt;
                ReplayBraceArm arm;
                bool valid = true;
                for (int joint = 3; joint >= 0; --joint)
                {
                    if (!body) { valid = false; break; }
                    arm.mBodies[joint] = *body;
                    if (joint > 0) body = profile.mBodies[*body].mParentBody
                        ? findBody(*profile.mBodies[*body].mParentBody) : std::nullopt;
                }
                if (valid) replayArms[side] = arm;
            }
        }
        // v2b phase-1 Q1 diagnostic (arm-repayment-v2b-2026-09-11.txt;
        // env-gated, scenario-only, inert when unset): swing both arms
        // forward-up through one windmill arc (theta max 0.8 rad, inside
        // the authored shoulder cone - limits untouched) during a cruise
        // window, and print trunk angular velocities before/during/after.
        // Measures the angular impulse the arm motors can deliver to the
        // trunk. The arm chains reuse the ReplayBraceArm walk; mTarget
        // stays null so the replay brace block above never fires.
        int armSupplyStart = -1;
        bool armSupplySwing = true;
        bool loadedBalanceArm = false;
        if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_LOADED_BALANCE_TEST"))
        {
            const int parsed = std::atoi(text);
            if (parsed > 0)
            {
                armSupplyStart = parsed;
                loadedBalanceArm = true;
            }
        }
        if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_ARM_SUPPLY_TEST"))
        {
            const int parsed = std::atoi(text);
            if (parsed != 0 && !loadedBalanceArm)
            {
                armSupplyStart = parsed > 0 ? parsed : -parsed;
                armSupplySwing = parsed > 0;
            }
        }
        std::array<std::optional<ReplayBraceArm>, 2> supplyArms;
        std::array<JPH::Vec3, 2> supplyRest{ JPH::Vec3::sZero(), JPH::Vec3::sZero() };
        bool supplyRestInit = false;
        Transform recoveryAcquireRoot;
        JPH::Vec3 recoveryPlacement = JPH::Vec3::sZero();
        float recoveryAcquireTime = 0.0f;
        bool wasWalkingKinematic = false;
        JPH::Vec3 walkAcquireVelocity = JPH::Vec3::sZero();
        JPH::Vec3 walkAcquireAngularVelocity = JPH::Vec3::sZero();
        if (armSupplyStart > 0)
        {
            const std::array hands{ profile.mLandmarks.mLeftHand, profile.mLandmarks.mRightHand };
            for (std::size_t side = 0; side < hands.size(); ++side)
            {
                if (loadedBalanceArm && side == 0)
                    continue;
                auto body = hands[side] ? findBody(*hands[side]) : std::nullopt;
                ReplayBraceArm arm;
                bool valid = true;
                for (int joint = 3; joint >= 0; --joint)
                {
                    if (!body) { valid = false; break; }
                    arm.mBodies[joint] = *body;
                    if (joint > 0) body = profile.mBodies[*body].mParentBody
                        ? findBody(*profile.mBodies[*body].mParentBody) : std::nullopt;
                }
                if (valid) supplyArms[side] = arm;
            }
        }
        for (std::uint64_t step = 0; step < result.mStepCount; ++step)
        {
            if (std::binary_search(settings.mReplayRecoveryRequestSteps.begin(), settings.mReplayRecoveryRequestSteps.end(), step))
            {
                if (!controllerOutput || controllerOutput->mMode != ActiveControllerMode::Fallen)
                { fail("Recorded recovery request does not meet native Fallen precondition"); return result; }
                referenceStandingLegPose.reset();
                referenceHandover.reset();
                replayRecoveryHandoverPending = false;
                replayRecoveryRequested = true;
            }
            if (settings.mReplayAutomaticRecoveryRequest && settings.mPowered
                && profile.mActiveController.mAutomaticRecovery && !result.mFrames.empty()
                && result.mFrames.back().mController.mMode == ActiveControllerMode::Fallen)
                replayRecoveryRequested = true;
            if (settings.mReplayNativeBraceLimits && !settings.mReplayBraceTargets && !replayBraceLimitsOpened && settings.mPowered
                && profile.mActiveController.mAutomaticRecovery && profile.mActiveController.mBracingArms
                && !result.mFrames.empty() && result.mFrames.back().mController.mMode == ActiveControllerMode::Fallen
                && result.mFrames.back().mSupport.hasGroundContact())
            {
                // Native updateBraceArms/stepArmTargets opens the shoulder
                // constraints before physics, even while drive is false.
                for (const auto& handName : { profile.mLandmarks.mLeftHand, profile.mLandmarks.mRightHand })
                {
                    auto body = handName ? findBody(*handName) : std::nullopt;
                    for (int parent = 0; parent < 2 && body; ++parent)
                        body = profile.mBodies[*body].mParentBody
                            ? findBody(*profile.mBodies[*body].mParentBody) : std::nullopt;
                    const auto& activeProfile = profile.mActiveController;
                    if (!body || !ragdoll->setSwingTwistJointLimits(*body,
                            activeProfile.mBraceShoulderNormalHalfCone, activeProfile.mBraceShoulderPlaneHalfCone,
                            -activeProfile.mBraceShoulderTwistLimit, activeProfile.mBraceShoulderTwistLimit))
                    {
                        fail("Replay native brace shoulder geometry unavailable");
                        return result;
                    }
                }
                replayBraceLimitsOpened = true;
            }
            if (settings.mReplayBraceTargets)
            {
                const auto& brace = profile.mActiveController;
                const bool wanted = settings.mPowered && brace.mBracingArms && !result.mFrames.empty()
                    && result.mFrames.back().mSupport.hasGroundContact()
                    && (result.mFrames.back().mController.mMode == ActiveControllerMode::GettingUp
                        || (result.mFrames.back().mController.mMode == ActiveControllerMode::Fallen && replayRecoveryRequested));
                for (auto& optionalArm : replayArms)
                {
                    if (!optionalArm) continue;
                    auto& arm = *optionalArm;
                    std::optional<BraceHandTarget> target;
                    if (wanted)
                    {
                        const auto pelvis = ragdoll->getWorldBodyPosition(pose, *pelvisIndex);
                        const auto shoulder = ragdoll->getWorldBodyPosition(pose, arm.mBodies[1]);
                        const auto elbow = ragdoll->getWorldBodyPosition(pose, arm.mBodies[2]);
                        const auto hand = ragdoll->getWorldBodyPosition(pose, arm.mBodies[3]);
                        if (pelvis && shoulder && elbow && hand)
                            target = calculateBraceHandTarget(*pelvis, *shoulder, *elbow, *hand,
                                result.mFrames.back().mSupport.mGroundCenter.GetZ(),
                                ragdoll->getHingeLegGeometry(arm.mBodies[1], arm.mBodies[2], arm.mBodies[3]), brace);
                    }
                    if (target) arm.mTarget = target;
                    const float desired = target ? std::clamp(brace.mBraceWeight, 0.0f, 1.0f) : 0.0f;
                    const float rate = std::max(brace.mBraceBlendRate, 0.0f) * settings.mFixedTimeStep;
                    const float delta = desired - arm.mWeight;
                    arm.mWeight = std::abs(delta) <= rate || rate <= 0.0f
                        ? desired : arm.mWeight + std::copysign(rate, delta);
                    if (!target && arm.mWeight <= 0) arm.mTarget.reset();
                    const bool open = arm.mTarget.has_value();
                    if (open != arm.mOpen)
                    {
                        const auto& authored = *profile.mBodies[arm.mBodies[1]].mConstraint;
                        if (!ragdoll->setSwingTwistJointLimits(arm.mBodies[1],
                                open ? brace.mBraceShoulderNormalHalfCone : authored.mNormalHalfConeAngle,
                                open ? brace.mBraceShoulderPlaneHalfCone : authored.mPlaneHalfConeAngle,
                                open ? -brace.mBraceShoulderTwistLimit : authored.mTwistMinAngle,
                                open ? brace.mBraceShoulderTwistLimit : authored.mTwistMaxAngle))
                        {
                            fail("Replay brace shoulder limit update rejected");
                            return result;
                        }
                        arm.mOpen = open;
                    }
                }
            }
            while (keyframeCursor < script.mKeyframes.size() && script.mKeyframes[keyframeCursor].mStep <= step)
            {
                const ScenarioInputKeyframe& keyframe = script.mKeyframes[keyframeCursor];
                active = keyframe;
                steeringRequest = horizontal(keyframe.mRequestedHorizontalVelocity);
                ++keyframeCursor;
            }
            if (inputProvider)
            {
                const auto generated = inputProvider(step, result.mFrames.empty() ? nullptr : &result.mFrames.back());
                if (generated)
                {
                    if (generated->mStep != step)
                        fail("Generated scenario input must name the current step.");
                    validateKeyframe(*generated, step);
                    validateSupportFoot(*generated);
                    if (!result.mErrors.empty())
                        return result;
                    active = *generated;
                    steeringRequest = horizontal(generated->mRequestedHorizontalVelocity);
                }
            }
            // Release before any impulse, including impulses applied ahead of planning.
            const bool recoveryImpact = active.mRecoveryInterrupted
                || !active.mPrePlanningImpulses.empty() || !active.mImpulses.empty()
                || std::any_of(script.mImpulses.begin(), script.mImpulses.end(),
                    [step](const ScenarioImpulse& impulse) { return impulse.mStep == step; });
            const bool recoveryMode = controllerOutput
                && (controllerOutput->mMode == ActiveControllerMode::GettingUp
                    || controllerOutput->mMode == ActiveControllerMode::BlendBack);
            const bool ownsRecoveryRoot = settings.mPowered && active.mKinematicRecovery
                && active.mMotorsEnabled && recoveryMode && !recoveryImpact;
            const bool ownsWalkingRoot = settings.mPowered && active.mKinematicLocomotion
                && active.mMotorsEnabled && !recoveryMode && !recoveryImpact
                && (!controllerOutput || controllerOutput->mDriveJointMotors);
            std::optional<float> releaseVelocityError;
            if (!ownsRecoveryRoot && !ownsWalkingRoot && ragdoll->isRootKinematic())
            {
                const auto velocity = bodies.GetLinearVelocity(ragdoll->getBodyId(0));
                const auto angular = bodies.GetAngularVelocity(ragdoll->getBodyId(0));
                if (!ragdoll->releaseRootKinematic(velocity))
                { fail("Failed to release recovery root"); return result; }
                releaseVelocityError = std::max((bodies.GetLinearVelocity(ragdoll->getBodyId(0)) - velocity).Length(),
                    (bodies.GetAngularVelocity(ragdoll->getBodyId(0)) - angular).Length());
                recoveryAcquireTime = 0.0f;
            }
            const auto prePlanningImpulseCount = active.mPrePlanningImpulses.size();
            for (const auto& impulse : active.mPrePlanningImpulses)
            {
                const auto id = ragdoll->getBodyId(impulse.mBodyIndex);
                if (impulse.mWorldPoint) bodies.AddImpulse(id, impulse.mImpulse, *impulse.mWorldPoint);
                else bodies.AddImpulse(id, impulse.mImpulse);
            }
            if (prePlanningImpulseCount && !ragdoll->captureTelemetry(telemetry))
            { fail("Pre-planning impulse telemetry failed"); return result; }
            active.mPrePlanningImpulses.clear();
            if (useReferenceGait && !settings.mReferenceInputsAreFiltered)
            {
                // Standing may take over only with both feet planted and the
                // body nearly still: outside an active gait, or in the double
                // support of the previous step with the center of mass below
                // twice the stop speed (the standing controller cannot absorb
                // a walking speed).
                // Native steering runs before the post-solve gait observation.
                // Use that same phase with the newly solved contacts/velocity;
                // the updated phase belongs to the next motor plan, not this
                // steering decision (notably at touchdown while stopping).
                const bool bothFeetPlanted = !steeringObservedGaitState.hasStance()
                    || (steeringObservedGaitState.mStage == ReferenceGaitStage::DoubleSupport && !result.mFrames.empty()
                        && result.mFrames.back().mSupport.mLeftFootSupported
                        && result.mFrames.back().mSupport.mRightFootSupported);
                const bool standingReady = bothFeetPlanted
                    && (!steeringObservedGaitState.hasStance()
                        || horizontal(result.mFrames.back().mCenterOfMassVelocity).Length()
                            <= 2.0f * settings.mReferenceGait.mSteeringStopSpeed);
                active.mRequestedHorizontalVelocity = referenceSteering.step(steeringRequest,
                    settings.mFixedTimeStep, steeringGait, standingReady, bothFeetPlanted,
                    settings.mReferenceGaitPlayerInput,
                    !settings.mReferenceGaitMeasuredStopping || result.mFrames.empty() ? std::nullopt
                        : std::optional<JPH::Vec3>(result.mFrames.back().mCenterOfMassVelocity));
                if (settings.mReferenceGaitPlayerInput)
                    referenceSteering.stepFacing(active.mFacing, settings.mFixedTimeStep, steeringGait);
            }
            if (active.mMotorTarget)
                motorTarget = *active.mMotorTarget;
            if (active.mRecoveryMotorTarget && controllerOutput
                && (controllerOutput->mMode == ActiveControllerMode::GettingUp
                    || controllerOutput->mMode == ActiveControllerMode::BlendBack))
                motorTarget = *active.mRecoveryMotorTarget;
            active.mMotorTarget.reset();
            active.mRecoveryMotorTarget.reset();

            ScenarioFrame frame;
            frame.mImpulseCount = static_cast<std::uint32_t>(prePlanningImpulseCount);
            frame.mStep = step;
            frame.mTime = static_cast<float>(step + 1) * settings.mFixedTimeStep;
            frame.mRequestedHorizontalVelocity = horizontal(active.mRequestedHorizontalVelocity);
            frame.mSupportMotion = active.mSupportMotion;
            frame.mMotorsEnabled = active.mMotorsEnabled;
            frame.mTranslationActuationEnabled = active.mTranslationActuationEnabled;
            frame.mSwingFootActuationEnabled = active.mSwingFootActuationEnabled;
            frame.mUprightTorqueEnabled = active.mUprightTorqueEnabled;
            frame.mStabilizationStrength = active.mStabilizationStrength;
            frame.mHybridStage = active.mHybridStage;
            frame.mRootReleaseVelocityError = releaseVelocityError;
            if (ownsWalkingRoot)
            {
                const auto position = ragdoll->getWorldBodyPosition(motorTarget, 0);
                const auto rotation = ragdoll->getWorldBodyRotation(motorTarget, 0);
                const auto leftFloor = ragdoll->getPoseBodyFloorHeight(motorTarget, *leftFootIndex);
                const auto rightFloor = ragdoll->getPoseBodyFloorHeight(motorTarget, *rightFootIndex);
                if (!position || !rotation || !leftFloor || !rightFloor)
                { fail("Invalid kinematic walking pose"); return result; }
                if (!wasWalkingKinematic)
                {
                    recoveryAcquireRoot.mTranslation = JPH::Vec3(bodies.GetPosition(ragdoll->getBodyId(0)));
                    recoveryAcquireRoot.mRotation = bodies.GetRotation(ragdoll->getBodyId(0));
                    walkAcquireVelocity = bodies.GetLinearVelocity(ragdoll->getBodyId(0));
                    walkAcquireAngularVelocity = bodies.GetAngularVelocity(ragdoll->getBodyId(0));
                    recoveryAcquireTime = 0.0f;
                    if (!ragdoll->isRootKinematic() && !ragdoll->setRootKinematic(true))
                    { fail("Failed to acquire walking root"); return result; }
                }
                recoveryAcquireTime = std::min(0.75f, recoveryAcquireTime + settings.mFixedTimeStep);
                const float t = recoveryAcquireTime / 0.75f;
                const float blend = t * t * (3 - 2 * t);
                JPH::Vec3 desired(*position);
                desired.SetZ(desired.GetZ() + settings.mGroundHeight - std::min(*leftFloor, *rightFloor));
                const auto movingStart = recoveryAcquireRoot.mTranslation + walkAcquireVelocity * recoveryAcquireTime;
                const float angularSpeed = walkAcquireAngularVelocity.Length();
                const auto movingRotation = angularSpeed > 1e-5f
                    ? JPH::Quat::sRotation(walkAcquireAngularVelocity / angularSpeed, angularSpeed * recoveryAcquireTime)
                        * recoveryAcquireRoot.mRotation : recoveryAcquireRoot.mRotation;
                Transform target;
                target.mTranslation = movingStart + (desired - movingStart) * blend;
                target.mRotation = movingRotation.SLERP(*rotation, blend).Normalized();
                if (!ragdoll->moveRootKinematic(JPH::RVec3(target.mTranslation), target.mRotation, settings.mFixedTimeStep))
                { fail("Failed to move walking root"); return result; }
                frame.mKinematicRootTarget = target;
            }
            else if (ownsRecoveryRoot)
            {
                const auto position = ragdoll->getWorldBodyPosition(motorTarget, 0);
                const auto rotation = ragdoll->getWorldBodyRotation(motorTarget, 0);
                if (!position || !rotation)
                { fail("Invalid kinematic recovery root target"); return result; }
                if (!ragdoll->isRootKinematic())
                {
                    if (!active.mKinematicRecoveryTerminalPose)
                    { fail("Kinematic recovery needs a terminal pose for floor alignment"); return result; }
                    const auto leftFloor = ragdoll->getPoseBodyFloorHeight(*active.mKinematicRecoveryTerminalPose, *leftFootIndex);
                    const auto rightFloor = ragdoll->getPoseBodyFloorHeight(*active.mKinematicRecoveryTerminalPose, *rightFootIndex);
                    if (!leftFloor || !rightFloor)
                    { fail("Invalid recovery terminal foot pose"); return result; }
                    recoveryAcquireRoot.mTranslation = JPH::Vec3(bodies.GetPosition(ragdoll->getBodyId(0)));
                    recoveryAcquireRoot.mRotation = bodies.GetRotation(ragdoll->getBodyId(0));
                    recoveryPlacement = recoveryAcquireRoot.mTranslation - JPH::Vec3(*position);
                    recoveryPlacement.SetZ(settings.mGroundHeight - std::min(*leftFloor, *rightFloor));
                    if (!ragdoll->setRootKinematic(true))
                    { fail("Failed to acquire recovery root"); return result; }
                }
                recoveryAcquireTime = std::min(0.75f, recoveryAcquireTime + settings.mFixedTimeStep);
                const float t = recoveryAcquireTime / 0.75f;
                const float blend = t * t * (3.0f - 2.0f * t);
                Transform target;
                target.mTranslation = recoveryAcquireRoot.mTranslation
                    + (JPH::Vec3(*position) + recoveryPlacement - recoveryAcquireRoot.mTranslation) * blend;
                target.mRotation = recoveryAcquireRoot.mRotation.SLERP(*rotation, blend).Normalized();
                if (!ragdoll->moveRootKinematic(JPH::RVec3(target.mTranslation), target.mRotation, settings.mFixedTimeStep))
                { fail("Failed to move recovery root"); return result; }
                frame.mKinematicRootTarget = target;
            }
            frame.mRootKinematic = ragdoll->isRootKinematic();
            frame.mKinematicLocomotion = ownsWalkingRoot;
            wasWalkingKinematic = ownsWalkingRoot;
            frame.mActuationGait = gaitOutput;
            if (step == 0 && result.mKillingImpulse)
                ++frame.mImpulseCount;
            const auto recordDrivenMotorPose = [&](const RagdollPose& target) {
                if (target.mBodyTransforms.size() != bodyCount)
                    return false;
                frame.mDrivenMotorLocalRotations.resize(bodyCount, JPH::Quat::sIdentity());
                for (std::size_t body = 0; body < bodyCount; ++body)
                {
                    const auto parent = parentBody(body);
                    frame.mDrivenMotorLocalRotations[body] = parent
                        ? (target.mBodyTransforms[*parent].mRotation.Conjugated()
                            * target.mBodyTransforms[body].mRotation).Normalized()
                        : target.mBodyTransforms[body].mRotation.Normalized();
                }
                return true;
            };
            const auto correctedProductionPose = [&](const RagdollPose& target) -> std::optional<RagdollPose> {
                if (!productionResidual || active.mMotorTargetRotationOffsets.empty()
                    || std::none_of(active.mMotorTargetRotationOffsets.begin(), active.mMotorTargetRotationOffsets.end(),
                        [](JPH::Vec3Arg offset) { return offset.LengthSq() != 0.0f; }))
                    return std::nullopt; // Do no quaternion work on the exact production path.
                RagdollPose correctedTarget = target;
                std::vector<bool> corrected(bodyCount, false);
                for (std::size_t body = 0; body < bodyCount; ++body)
                {
                    const auto parent = parentBody(body);
                    if (!parent) continue;
                    const JPH::Vec3 offset = active.mMotorTargetRotationOffsets[body];
                    const float angle = offset.Length();
                    corrected[body] = corrected[*parent] || angle != 0.0f;
                    if (!corrected[body]) continue;
                    const auto relative = target.mBodyTransforms[*parent].mRotation.Conjugated()
                        * target.mBodyTransforms[body].mRotation;
                    const auto delta = angle == 0.0f ? JPH::Quat::sIdentity()
                        : JPH::Quat::sRotation(offset / angle, angle);
                    correctedTarget.mBodyTransforms[body].mRotation
                        = (correctedTarget.mBodyTransforms[*parent].mRotation * delta * relative).Normalized();
                }
                return correctedTarget;
            };

            listener.beginStep();
            std::optional<JPH::Quat> swingHipWorldTarget;
            const bool drive = settings.mPowered && (!controllerOutput || controllerOutput->mDriveJointMotors);
            GaitActuationDecisionInput actuationDecisionInput;
            actuationDecisionInput.mLocomotionRequested
                = !horizontal(active.mRequestedHorizontalVelocity).IsNearZero();
            actuationDecisionInput.mPelvisHeightSettled = pelvisHeightOutput.mSettled;
            actuationDecisionInput.mDriveJointMotors = drive;
            actuationDecisionInput.mRecovering = controllerOutput
                && (controllerOutput->mMode == ActiveControllerMode::GettingUp
                    || controllerOutput->mMode == ActiveControllerMode::BlendBack);
            actuationDecisionInput.mBothFeetSupported = supportEstimate && supportEstimate->mLeftFootSupported
                && supportEstimate->mRightFootSupported;
            actuationDecisionInput.mRequestedHorizontalVelocity = horizontal(active.mRequestedHorizontalVelocity);
            actuationDecisionInput.mSwingTargetAvailable = swingFootTarget.has_value();
            actuationDecisionInput.mMeasuredHorizontalVelocity = result.mFrames.empty()
                ? JPH::Vec3::sZero() : result.mFrames.back().mCenterOfMassVelocity;
            actuationDecisionInput.mStanceFootSupported = supportEstimate && gaitOutput && gaitOutput->hasStance()
                && (gaitOutput->leftFootIsStance() ? supportEstimate->mLeftFootSupported
                                                  : supportEstimate->mRightFootSupported);
            if (gaitOutput)
                actuationDecisionInput.mGait = *gaitOutput;
            GaitControllerProfile actuationProfile = profile.mActiveController.mGait;
            actuationProfile.mPreLiftSwingUnloadFraction = preLiftSwingUnloadFraction;
            const auto actuationDecision
                = calculateGaitActuationDecision(actuationDecisionInput, actuationProfile);
            if (!actuationDecision)
            {
                fail("The gait actuation decision rejected its observation at step " + std::to_string(step) + ".");
                return result;
            }
            frame.mActuationStage = directMotorTracking ? GaitActuationStage::Inactive : actuationDecision->mStage;
            // The reference route owns actuation from the first observed stance
            // of a settled locomotion request. Until then, and whenever motors
            // are released, standing keeps the shared joint-support controller.
            const bool referenceLocomotion = useReferenceGait && actuationDecisionInput.mLocomotionRequested
                && !ragdoll->isRootKinematic();
            if (useReferenceGait && !referenceLocomotion)
            {
                // Zero speed selects standing observation below. Explicitly
                // discard the reference episode that will no longer receive
                // disabled observations, including its previous-step loads.
                referenceGait.reset();
                referenceSwingHistory.reset();
                referenceState = {};
                steeringObservedGaitState = {};
                referenceSwingStartModel.reset();
                referenceLandingFootModel.reset();
                referencePlacement.reset();
                previousStanceKneeTarget.reset();
                previousStanceHingeTarget.reset();
                referenceHistory = false;
            }
            const bool referenceActive = referenceLocomotion
                && pelvisHeightOutput.mSettled && drive
                && active.mMotorsEnabled && !actuationDecisionInput.mRecovering && gaitOutput
                && gaitOutput->hasStance() && supportEstimate;
            // The standing hand-over blend runs only while standing drives
            // the motors; an active gait, a release or a recovery ends it.
            // A pending movement request must retain the supported targets
            // while height/stance readiness still prevents a plan.
            const bool standingDrives = drive && active.mMotorsEnabled && !actuationDecisionInput.mRecovering;
            // Native standing support is observed before the next motor
            // handover advance/begin. Preserve that clock separately from
            // the pose blend used by this step's motor commands.
            const ReferenceGaitHandover supportHandover = referenceHandover;
            if (referenceActive || !standingDrives)
            {
                referenceHandover.reset();
                referenceStandingLegPose.reset();
            }
            else
                referenceHandover.step(settings.mFixedTimeStep, settings.mReferenceGait);
            // Historical native stop reset discarded the standing support plan.
            // Replay-only diagnostic; default scenarios retain normal support.
            const bool replayResetSupportProbe = settings.mReplayDiscardSupportOnReferenceReset
                && referenceSpringsActive && !referenceActive;
            if (referenceSpringsActive && !referenceActive)
            {
                // Restore before standing, release or recovery takes over.
                // driveToPoseUsingMotors and disableMotors change motor state,
                // but neither restores the native spring settings.
                for (const std::size_t bodyIndex : referenceSpringBodies)
                {
                    const auto& motor = profile.mBodies[bodyIndex].mConstraint->mMotor;
                    if (!ragdoll->setMotorSpring(bodyIndex, motor.mFrequency, motor.mDamping))
                    {
                        fail("Reference motor springs could not be restored on exit at step "
                            + std::to_string(step) + ".");
                        return result;
                    }
                }
                referenceSpringsActive = false;
                // Standing takes over from the measured walking end pose and
                // blends into its own pose and height instead of snapping.
                if (standingDrives && !result.mFrames.empty())
                {
                    referenceHandover.begin(pose, result.mFrames.back().mPelvisHeight, telemetry.mCenterOfMass,
                        settings.mReferenceGait);
                    if (referenceHandover.active())
                        referenceStandingLegPose = pose;
                    else
                        referenceStandingLegPose.reset();
                }
            }
            if (settings.mReplayRecoveryFinishEventsCaptured)
                replayRecoveryPosePending = std::binary_search(settings.mReplayRecoveryFinishSteps.begin(),
                    settings.mReplayRecoveryFinishSteps.end(), step);
            if (replayRecoveryPosePending)
            {
                referenceStandingLegPose = pose;
                replayRecoveryPosePending = false;
                replayRecoveryHandoverPending = true;
            }
            // A pinned-pelvis diagnostic already receives external support and
            // does not have the floating base assumed by the joint-load model.
            const bool useJointSupport = !directMotorTracking && settings.mUseJointSupportControl && !settings.mPinPelvis
                && !ragdoll->isRootKinematic()
                && (actuationDecision->mUseJointSupportControl
                    || (referenceLocomotion && !referenceActive && drive && !actuationDecisionInput.mRecovering));
            ragdoll->beginActuationStep(!directMotorTracking && (settings.mUseSolverActuation || useJointSupport));
            // Policy may only give up authority that already exists. Apply the
            // fraction to every authored joint limit before any route targets
            // a motor; route-specific leg scaling below composes with it.
            for (std::size_t body = 1; body < bodyCount; ++body)
            {
                if (!profile.mBodies[body].mConstraint)
                    continue;
                const float maximumTorque
                    = profile.mBodies[body].mConstraint->mMotor.mMaximumTorque
                    * active.mStabilizationStrength;
                if (!ragdoll->setMotorMaximumTorque(body, maximumTorque))
                {
                    fail("Stabilization motor budget was rejected at step " + std::to_string(step) + ".");
                    return result;
                }
            }
            if (!directMotorTracking && !referenceLocomotion && active.mSwingFootActuationEnabled
                && actuationDecision->mApplyPreLiftSwingUnload && gaitOutput)
            {
                const LegChain& stanceLeg = gaitOutput->leftFootIsStance() ? *leftLeg : *rightLeg;
                const LegChain& swingLeg = gaitOutput->leftFootIsStance() ? *rightLeg : *leftLeg;
                const float swingLegMass = telemetry.mBodies[swingLeg.mHip].mMass
                    + telemetry.mBodies[swingLeg.mKnee].mMass + telemetry.mBodies[swingLeg.mFoot].mMass;
                const float bodyWeight = telemetry.mTotalMass * settings.mGravity;
                const auto unload = calculatePreLiftSwingUnloadForce(
                    *actuationDecision, telemetry.mTotalMass, swingLegMass, settings.mGravity, actuationProfile);
                if (!unload)
                {
                    fail("The shared unloading budget was rejected at step " + std::to_string(step) + ".");
                    return result;
                }
                const float unloadForce = *unload;
                const JPH::Vec3 force(0.0f, 0.0f, unloadForce);
                if (!ragdoll->applyBodyForcePair(swingLeg.mFoot, stanceLeg.mFoot, force, bodyWeight))
                {
                    fail("Pre-lift swing-foot unloading was rejected at step " + std::to_string(step) + ".");
                    return result;
                }
                frame.mPreLiftSwingUnloadForce = unloadForce;
                const std::array<std::size_t, 1> reactions{ stanceLeg.mFoot };
                frame.mAppliedSwingForcePairMoment += recordForcePair(frame, reactions, force);
            }
            if (directMotorTracking || ownsWalkingRoot)
            {
                // Own the complete ordinary motor command, before the shared
                // final-command capture below. This branch deliberately never
                // enters procedural pose, load, spring or world-target paths.
                if (drive && active.mMotorsEnabled)
                {
                    RagdollPose directTarget = motorTarget;
                    if (!active.mMotorTargetRotationOffsets.empty())
                    {
                        std::vector<bool> corrected(bodyCount, false);
                        for (std::size_t body = 0; body < bodyCount; ++body)
                        {
                            const auto parent = parentBody(body);
                            if (!parent)
                                continue;
                            const JPH::Vec3 offset = active.mMotorTargetRotationOffsets[body];
                            const float angle = offset.Length();
                            corrected[body] = corrected[*parent] || angle != 0.0f;
                            if (!corrected[body])
                                continue; // Preserve exact zero-action bytes.
                            const auto relative = motorTarget.mBodyTransforms[*parent].mRotation.Conjugated()
                                * motorTarget.mBodyTransforms[body].mRotation;
                            const auto delta = angle == 0.0f ? JPH::Quat::sIdentity()
                                : JPH::Quat::sRotation(offset / angle, angle);
                            directTarget.mBodyTransforms[body].mRotation
                                = (directTarget.mBodyTransforms[*parent].mRotation * delta * relative).Normalized();
                        }
                    }
                    frame.mMotorsApplied = ragdoll->driveToPoseUsingMotors(directTarget);
                    if (!frame.mMotorsApplied)
                    {
                        fail("Direct tracking motor pose rejected at step " + std::to_string(step));
                        return result;
                    }
                    if (!recordDrivenMotorPose(directTarget))
                    {
                        fail("Direct tracking motor pose telemetry rejected at step " + std::to_string(step));
                        return result;
                    }
                    directMotorAxesCaptured = true;
                    if (!active.mMotorTargetAngularVelocities.empty())
                        for (std::size_t body = 0; body < bodyCount; ++body)
                        {
                            if (!profile.mBodies[body].mConstraint)
                                continue;
                            if (!ragdoll->setMotorTargetAngularVelocity(body, active.mMotorTargetAngularVelocities[body]))
                            {
                                fail("Direct tracking motor velocity rejected at step " + std::to_string(step));
                                return result;
                            }
                        }
                }
                else
                    ragdoll->disableMotors();
            }
            else if (referenceActive)
            {
                referenceSpringsActive = true;
                ReferenceGaitPlanInput planInput;
                planInput.mTelemetry = &telemetry;
                // Fable i56 (tracked-cycle-2026-09-11.txt): with the env gate and a
                // scripted pose cycle, the reference plan tracks the authored
                // walk cycle sampled by the reference phase clock instead of the
                // frozen base pose. Unset or no cycle: byte-identical.
                static const auto trackCycle_tuned = RagdollCore::tuned<bool>([] { return RagdollCore::tuningText("OPENMW_RAGDOLL_TRACK_CYCLE") != nullptr; });
                static const auto& trackCycle = trackCycle_tuned.get();
                RagdollPose trackedCyclePose;
                bool trackedCycleSampled = false;
                if (trackCycle && !script.mGaitPoseCycle.empty() && referenceState.hasStance())
                {
                    const GaitControllerOutput gaitView = toGaitControllerOutput(referenceState, settings.mReferenceGait);
                    trackedCycleSampled = sampleGaitPoseCycle(script.mGaitPoseCycle, gaitView, trackedCyclePose)
                        && trackedCyclePose.mBodyTransforms.size() == motorTarget.mBodyTransforms.size();
                }
                planInput.mBasePose = trackedCycleSampled ? trackedCyclePose.mBodyTransforms : motorTarget.mBodyTransforms;
                planInput.mMeasuredPose = pose.mBodyTransforms;
                if (!result.mFrames.empty())
                    planInput.mContacts = result.mFrames.back().mContacts;
                planInput.mSupport = *supportEstimate;
                planInput.mPelvis = *pelvisIndex;
                for (const std::string& spineName : profile.mLandmarks.mSpine)
                    if (const auto spineIndex = findBody(spineName))
                        planInput.mTrunk.push_back(*spineIndex);
                planInput.mLeftLeg = referenceLeg(*leftLeg);
                planInput.mRightLeg = referenceLeg(*rightLeg);
                // Native reference planning samples live body velocities. An
                // external impulse between solves changes these before the
                // cached controller observation advances.
                for (const auto& body : telemetry.mBodies)
                    planInput.mCenterOfMassVelocity += body.mMotion.mLinearVelocity * body.mMass;
                planInput.mCenterOfMassVelocity /= telemetry.mTotalMass;
                planInput.mRequestedHorizontalVelocity = frame.mRequestedHorizontalVelocity;
                planInput.mStopping = referenceSteering.stopping();
                planInput.mFacing = settings.mReferenceGaitPlayerInput ? referenceSteering.facing() : active.mFacing;
                // i33 (rotation-predict-2026-09-11.txt): signed yaw rate of
                // the travel command, from successive steering outputs.
                // Measurement only; consumed under the placement env gate.
                if (!result.mFrames.empty() && settings.mFixedTimeStep > 0.0f)
                {
                    const JPH::Vec3 previous = horizontal(result.mFrames.back().mRequestedHorizontalVelocity);
                    const JPH::Vec3 current = horizontal(frame.mRequestedHorizontalVelocity);
                    if (!previous.IsNearZero() && !current.IsNearZero())
                    {
                        const float cosine = std::clamp(
                            previous.Dot(current) / (previous.Length() * current.Length()), -1.0f, 1.0f);
                        const float cross = previous.GetX() * current.GetY() - previous.GetY() * current.GetX();
                        planInput.mTravelYawRate = std::atan2(cross, cosine) / settings.mFixedTimeStep;
                    }
                }
                planInput.mGravity = settings.mGravity;
                planInput.mFixedTimeStep = settings.mFixedTimeStep;
                planInput.mRootPosition = currentRootPosition;
                planInput.mRootRotation = currentRootRotation;
                planInput.mUniformScale = settings.mUniformScale;
                planInput.mState = referenceState;
                planInput.mSwingStartModel = referenceSwingStartModel;
                planInput.mSwingTrajectoryHistory = referenceSwingHistory;
                planInput.mLandingFootModel = referenceLandingFootModel;
                if (referenceHistory)
                {
                    planInput.mPreviousLoadTorques = referenceLoadTorques;
                    planInput.mPreviousMotorTorques = referenceMotorTorques;
                }
                planInput.mPreviousPlacement = referencePlacement;
                planInput.mHoldSwingFoot = settings.mReferenceGaitHoldSwingFoot;
                planInput.mCoordinateLegPose = settings.mReferenceGaitCoordinatesLegPose;
                if (settings.mReferenceGaitCoordinatesLegPose && pelvisHeightOutput.mTargetHeight)
                    planInput.mPreferredPelvisCenterHeight = *pelvisHeightOutput.mTargetHeight;
                if (settings.mReferenceGaitGroundsPlacement && settings.mReplayGroundShape.empty())
                {
                    // This scenario's ground is horizontal. Heading correction
                    // is world-Z yaw, which preserves the lower Z bound. Native
                    // shapes already include uniform scale and authored offsets.
                    const auto& landingLeg = referenceState.mPhase == GaitPhase::LeftStance ? *rightLeg : *leftLeg;
                    const auto shape = physics.GetBodyInterface().GetShape(ragdoll->getBodyId(landingLeg.mFoot));
                    const auto rotation = (currentRootRotation
                        * motorTarget.mBodyTransforms[landingLeg.mFoot].mRotation).Normalized();
                    // Place the BODY origin at zero, not the COM: decorated
                    // shapes may have a nonzero authored center-of-mass offset.
                    const auto transform = JPH::Mat44::sRotationTranslation(rotation,
                        rotation * shape->GetCenterOfMass());
                    const auto bounds = shape->GetWorldSpaceBounds(transform, JPH::Vec3::sReplicate(1.0f));
                    // Contact observations are surface MIDPOINTS and therefore
                    // move with penetration. This flat-floor fixture supplies
                    // its actual collision surface; a live adapter needs a
                    // terrain query at the chosen landing point instead. The
                    // fixture surface remains known during contact loss.
                    const auto floorBounds = groundShape->GetWorldSpaceBounds(
                        bodies.GetCenterOfMassTransform(ground), JPH::Vec3::sReplicate(1.0f));
                    planInput.mLandingFootOriginHeight = floorBounds.mMax.GetZ() - bounds.mMin.GetZ();
                }
                std::string rejection;
                // Bounded replay-only observation of the exact history used
                // for root routing. No controller state or settings change.
                if (!settings.mReplayBodyIds.empty() && step < 600 && referenceHistory
                    && RagdollCore::tuningText("OPENMW_RAGDOLL_BRANCH_TRACE"))
                {
                    const bool swinging = referenceState.mStage == ReferenceGaitStage::Swing;
                    const auto stanceHip = referenceState.leftFootIsStance()
                        ? planInput.mLeftLeg.mHip : planInput.mRightLeg.mHip;
                    for (const auto& joint : telemetry.mConstraints)
                    {
                        if (joint.mParentBodyIndex != *pelvisIndex)
                            continue;
                        const auto body = joint.mChildBodyIndex;
                        const bool routing = body == stanceHip || (!swinging
                            && (body == planInput.mLeftLeg.mHip || body == planInput.mRightLeg.mHip));
                        const auto load = planInput.mPreviousLoadTorques[body];
                        const auto motor = planInput.mPreviousMotorTorques[body];
                        std::cerr << std::setprecision(9) << "RAGDOLL_BRANCH_INPUT step=" << step
                                  << " body=" << body << " contributes=" << !routing
                                  << " previous_load=" << load.GetX() << ',' << load.GetY() << ',' << load.GetZ()
                                  << " previous_motor=" << motor.GetX() << ',' << motor.GetY() << ',' << motor.GetZ() << '\n';
                    }
                }
                auto plan = planReferenceGaitActuation(planInput, settings.mReferenceGait, &rejection);
                if (plan && plan->mPlacement && settings.mReferenceGaitGroundsPlacement
                    && !settings.mReplayGroundShape.empty() && referenceState.mStage == ReferenceGaitStage::Swing)
                {
                    const auto& leg = referenceState.leftFootIsStance() ? planInput.mRightLeg : planInput.mLeftLeg;
                    const auto shape = bodies.GetShape(ragdoll->getBodyId(leg.mFoot));
                    const float length = leg.mHingeGeometry ? settings.mUniformScale
                        * (leg.mHingeGeometry->mUpperOffset.Length() + leg.mHingeGeometry->mLowerOffset.Length()) : 0;
                    if (shape && length > 0)
                    {
                        // Native forms endpoints in float OSG coordinates,
                        // then promotes the origin for Jolt's double ray cast.
                        const JPH::Vec3 from(static_cast<float>(plan->mPlacement->GetX()),
                            static_cast<float>(plan->mPlacement->GetY()),
                            static_cast<float>(telemetry.mBodies[*pelvisIndex].mPosition.GetZ() + length));
                        const JPH::Vec3 to = from - JPH::Vec3(0, 0, 3 * length);
                        const JPH::RRayCast ray(JPH::RVec3(from), to - from);
                        struct GroundOnly final : JPH::BodyFilter
                        {
                            JPH::BodyID mGround;
                            explicit GroundOnly(JPH::BodyID id) : mGround(id) {}
                            bool ShouldCollide(const JPH::BodyID& id) const override { return id == mGround; }
                        } filter(ground);
                        JPH::RayCastResult hit;
                        if (physics.GetNarrowPhaseQuery().CastRay(ray, hit, {}, {}, filter))
                        {
                            const auto position = ray.GetPointOnRay(hit.mFraction);
                            JPH::BodyLockRead lock(physics.GetBodyLockInterface(), hit.mBodyID);
                            if (lock.Succeeded() && lock.GetBody().GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, position).GetZ()
                                >= profile.mActiveController.mSupportNormalMinimumDot)
                            {
                                const auto rotation = (currentRootRotation * motorTarget.mBodyTransforms[leg.mFoot].mRotation).Normalized();
                                const auto transform = JPH::Mat44::sRotationTranslation(rotation, rotation * shape->GetCenterOfMass());
                                const auto bounds = shape->GetWorldSpaceBounds(transform, JPH::Vec3::sOne());
                                planInput.mLandingFootOriginHeight = static_cast<float>(position.GetZ()) - bounds.mMin.GetZ();
                                plan = planReferenceGaitActuation(planInput, settings.mReferenceGait, &rejection);
                            }
                        }
                    }
                }
                if (!plan)
                {
                    fail("The reference gait plan rejected its observation at step " + std::to_string(step) + ": "
                        + rejection);
                    return result;
                }
                // i39 (delivery-comp-2026-09-11.txt): remember the operative
                // plan's predicted touchdown velocity for the state
                // machine's delivery-error observer. Only when a placement
                // was actually computed (swing); double-support plans leave
                // the last swing's prediction standing.
                if (plan->mPlacement)
                    referencePlacementPredicted = plan->mPlacementVelocity;
                if (RagdollCore::tuningText("OPENMW_RAGDOLL_LIMB_FORECAST_TRACE") && step >= 582 && step <= 604
                    && referenceState.mStage == ReferenceGaitStage::Swing)
                {
                    const auto& swing = referenceState.leftFootIsStance() ? planInput.mRightLeg : planInput.mLeftLeg;
                    const auto& stance = referenceState.leftFootIsStance() ? planInput.mLeftLeg : planInput.mRightLeg;
                    for (const unsigned velocity : { 0u, 1u, 2u, 3u })
                        for (const unsigned ahead : { 0u, 1u, 2u, 4u, 8u })
                        {
                            auto query = planInput;
                            auto predicted = telemetry;
                            std::vector<Transform> measured(planInput.mMeasuredPose.begin(), planInput.mMeasuredPose.end());
                            const float time = ahead * settings.mFixedTimeStep;
                            bool predictionValid = true;
                            if ((ahead > 0 || velocity == 2) && velocity)
                            {
                                predicted.mCenterOfMass = JPH::RVec3::sZero();
                                for (std::size_t body = 0; body < predicted.mBodies.size(); ++body)
                                {
                                    auto& state = predicted.mBodies[body];
                                    if (body != stance.mFoot)
                                    {
                                        const auto localCenter = state.mRotation.Conjugated()
                                            * JPH::Vec3(state.mCenterOfMass - state.mPosition);
                                        const auto omega = state.mMotion.mAngularVelocity;
                                        const float speed = omega.Length();
                                        if (speed > 0) state.mRotation = (JPH::Quat::sRotation(omega / speed, speed * time)
                                            * state.mRotation).Normalized();
                                        state.mCenterOfMass += JPH::RVec3(state.mMotion.mLinearVelocity * time);
                                        state.mPosition = state.mCenterOfMass - JPH::RVec3(state.mRotation * localCenter);
                                    }
                                    predicted.mCenterOfMass += state.mCenterOfMass * (state.mMass / predicted.mTotalMass);
                                    measured[body].mTranslation = currentRootRotation.Conjugated()
                                        * JPH::Vec3(state.mPosition - currentRootPosition) / settings.mUniformScale;
                                    measured[body].mRotation = (currentRootRotation.Conjugated() * state.mRotation).Normalized();
                                }
                                if (velocity >= 2)
                                {
                                    const auto project = [&](std::span<const Transform> source)
                                        -> std::optional<std::vector<Transform>> {
                                        std::vector<Transform> projected(source.begin(), source.end());
                                        const auto anchor = source[stance.mFoot];
                                        const auto axis = (anchor.mTranslation - source[stance.mHip].mTranslation)
                                            .NormalizedOr(JPH::Vec3::sAxisZ());
                                        auto bend = stance.mHingeGeometry
                                            ? (source[stance.mHip].mRotation * stance.mHingeGeometry->mHingeAxis).Cross(axis)
                                            : source[stance.mKnee].mTranslation - source[stance.mHip].mTranslation;
                                        bend -= axis * bend.Dot(axis);
                                        const TwoBoneLegTargetInput ik{ stance.mHip, stance.mKnee, stance.mFoot,
                                            anchor.mTranslation, bend.NormalizedOr(JPH::Vec3::sAxisY()), stance.mHingeGeometry };
                                        if (!applyTwoBoneLegTarget(projected, ik)
                                            || (projected[stance.mFoot].mTranslation - anchor.mTranslation).Length() > 1e-3f)
                                            return std::nullopt;
                                        projected[stance.mFoot].mRotation = anchor.mRotation;
                                        return projected;
                                    };
                                    const auto projection = project(measured);
                                    auto constrained = projection.value_or(measured);
                                    const auto footAnchor = measured[stance.mFoot];
                                    predictionValid = projection.has_value();
                                    if (predictionValid && velocity == 3)
                                    {
                                        const auto initial = project(planInput.mMeasuredPose);
                                        predictionValid = initial.has_value();
                                        if (initial)
                                            for (const auto body : { stance.mHip, stance.mKnee, stance.mFoot })
                                            {
                                                constrained[body].mTranslation += planInput.mMeasuredPose[body].mTranslation
                                                    - (*initial)[body].mTranslation;
                                                constrained[body].mRotation = (constrained[body].mRotation
                                                    * (*initial)[body].mRotation.Conjugated()
                                                    * planInput.mMeasuredPose[body].mRotation).Normalized();
                                            }
                                        constrained[stance.mFoot] = footAnchor;
                                    }
                                    if (predictionValid)
                                    {
                                        constrained[stance.mFoot].mRotation = footAnchor.mRotation;
                                        for (const auto body : { stance.mHip, stance.mKnee, stance.mFoot })
                                        {
                                            const auto& original = telemetry.mBodies[body];
                                            const auto localCenter = original.mRotation.Conjugated()
                                                * JPH::Vec3(original.mCenterOfMass - original.mPosition);
                                            auto& state = predicted.mBodies[body];
                                            measured[body] = constrained[body];
                                            state.mPosition = currentRootPosition
                                                + JPH::RVec3(currentRootRotation * measured[body].mTranslation * settings.mUniformScale);
                                            state.mRotation = (currentRootRotation * measured[body].mRotation).Normalized();
                                            state.mCenterOfMass = state.mPosition + JPH::RVec3(state.mRotation * localCenter);
                                        }
                                        predicted.mCenterOfMass = JPH::RVec3::sZero();
                                        for (const auto& body : predicted.mBodies)
                                            predicted.mCenterOfMass += body.mCenterOfMass * (body.mMass / predicted.mTotalMass);
                                    }
                                }
                                query.mTelemetry = &predicted;
                                query.mMeasuredPose = measured;
                            }
                            query.mState.mTimeInStage += time;
                            query.mState.mTimeInPhase += time;
                            if (query.mState.mSwingFootLifted) query.mState.mSwingTimeSinceLift += time;
                            const auto forecast = !predictionValid ? std::nullopt
                                : ahead == 0 && velocity != 2 ? plan : planReferenceGaitActuation(query, settings.mReferenceGait);
                            std::cerr << std::setprecision(9) << "RAGDOLL_LIMB_FORECAST step=" << step
                                << " ahead=" << ahead << " velocity=" << velocity << " swing_foot=" << swing.mFoot
                                << " valid=" << forecast.has_value();
                            if (forecast && forecast->mSwingHipWorldTarget && swing.mHingeGeometry)
                            {
                                const auto& hip = *forecast->mSwingHipWorldTarget;
                                const auto& target = forecast->mTargetPose.mBodyTransforms;
                                const auto calf = (hip * target[swing.mHip].mRotation.Conjugated() * target[swing.mKnee].mRotation).Normalized();
                                const auto& geometry = *swing.mHingeGeometry;
                                const auto foot = predicted.mBodies[swing.mHip].mPosition
                                    + JPH::RVec3(hip * geometry.mUpperOffset * settings.mUniformScale)
                                    + JPH::RVec3(calf * (geometry.mLowerRotation.Conjugated() * geometry.mLowerOffset) * settings.mUniformScale);
                                const auto& support = predicted.mBodies[stance.mKnee];
                                std::cerr << " hip=" << hip.GetX() << ',' << hip.GetY() << ',' << hip.GetZ() << ',' << hip.GetW()
                                    << " foot=" << foot.GetX() << ',' << foot.GetY() << ',' << foot.GetZ()
                                    << " support_p=" << support.mPosition.GetX() << ',' << support.mPosition.GetY() << ',' << support.mPosition.GetZ()
                                    << " support_q=" << support.mRotation.GetX() << ',' << support.mRotation.GetY() << ','
                                    << support.mRotation.GetZ() << ',' << support.mRotation.GetW();
                            }
                            std::cerr << '\n';
                        }
                }
                if (RagdollCore::tuningText("OPENMW_RAGDOLL_COORDINATED_QUERY") && step >= 540 && step <= 700 && step % 20 == 0)
                {
                    auto measuredForward = planInput.mTelemetry->mBodies[planInput.mPelvis].mRotation * JPH::Vec3::sAxisY();
                    measuredForward.SetZ(0);
                    measuredForward = measuredForward.NormalizedOr(JPH::Vec3::sAxisY());
                    auto requestedForward = planInput.mFacing;
                    requestedForward.SetZ(0);
                    requestedForward = requestedForward.NormalizedOr(measuredForward);
                    const float headingGap = std::atan2(measuredForward.Cross(requestedForward).GetZ(),
                        measuredForward.Dot(requestedForward));
                    for (const float headingFraction : { 1.0f, 0.75f, 0.5f, 0.25f, 0.125f, 0.0f })
                    {
                        auto queryInput = planInput;
                        queryInput.mCoordinateLegPose = true;
                        if (headingFraction == 0.0f) queryInput.mFacing = measuredForward;
                        else if (headingFraction != 1.0f)
                            queryInput.mFacing = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), headingGap * headingFraction)
                                * measuredForward;
                        const auto query = planReferenceGaitActuation(queryInput, settings.mReferenceGait);
                        const auto shift = query && query->mCoordinatedPoseApplied
                            ? query->mTargetPose.mBodyTransforms[queryInput.mPelvis].mTranslation
                                - queryInput.mMeasuredPose[queryInput.mPelvis].mTranslation
                            : JPH::Vec3::sZero();
                        std::cerr << "COORDINATED_QUERY schema=2 step=" << step << " heading_fraction=" << headingFraction
                            << " heading_advance=" << headingGap * headingFraction
                            << " valid=" << query.has_value() << " applied=" << (query && query->mCoordinatedPoseApplied)
                            << " shift=" << shift.GetX() << ',' << shift.GetY() << ',' << shift.GetZ() << '\n';
                    }
                }
                if (settings.mReferenceGaitPlanObserver)
                    settings.mReferenceGaitPlanObserver(step, *plan);
                // Restore the authored leg springs before assigning the swing
                // override so a new stance leg never inherits it.
                for (const LegChain* leg : { &*leftLeg, &*rightLeg })
                    for (const std::size_t bodyIndex : { leg->mHip, leg->mKnee, leg->mFoot })
                    {
                        const auto& motor = profile.mBodies[bodyIndex].mConstraint->mMotor;
                        if (!ragdoll->setMotorSpring(bodyIndex, motor.mFrequency, motor.mDamping))
                        {
                            fail("Leg motor springs could not be restored at step " + std::to_string(step) + ".");
                            return result;
                        }
                    }
                for (const std::size_t bodyIndex : plan->mStanceLegSpringBodies)
                    if (!ragdoll->setMotorSpring(bodyIndex, plan->mStanceLegFrequency, plan->mStanceLegDamping))
                    {
                        fail("Loaded-leg motor spring override was rejected at step " + std::to_string(step) + ".");
                        return result;
                    }
                for (const std::size_t bodyIndex : plan->mStanceHipSpringBodies)
                    if (!ragdoll->setMotorSpring(bodyIndex, plan->mStanceHipFrequency, plan->mStanceHipDamping))
                    {
                        fail("Loaded-hip motor spring override was rejected at step " + std::to_string(step) + ".");
                        return result;
                    }
                for (const std::size_t bodyIndex : plan->mSwingLegSpringBodies)
                    if (!ragdoll->setMotorSpring(bodyIndex, plan->mSwingLegFrequency, plan->mSwingLegDamping))
                    {
                        fail("Swing-leg motor spring override was rejected at step " + std::to_string(step) + ".");
                        return result;
                    }
                for (const std::size_t bodyIndex : planInput.mTrunk)
                {
                    const auto& motor = profile.mBodies[bodyIndex].mConstraint->mMotor;
                    if (!ragdoll->setMotorSpring(bodyIndex, motor.mFrequency, motor.mDamping))
                    {
                        fail("Trunk motor springs could not be restored at step " + std::to_string(step) + ".");
                        return result;
                    }
                }
                for (const std::size_t bodyIndex : plan->mTrunkSpringBodies)
                    if (!ragdoll->setMotorSpring(bodyIndex, plan->mTrunkFrequency, plan->mTrunkDamping))
                    {
                        fail("Trunk motor spring override was rejected at step " + std::to_string(step) + ".");
                        return result;
                    }
                for (const std::size_t bodyIndex : plan->mLoadedAnkleBodies)
                    if (!ragdoll->setMotorSpring(bodyIndex, plan->mLoadedAnkleFrequency, plan->mLoadedAnkleDamping))
                    {
                        fail("Loaded-ankle motor spring override was rejected at step " + std::to_string(step) + ".");
                        return result;
                    }
                // v2b phase-1 Q1 diagnostic (arm-repayment-v2b-2026-09-11.txt;
                // env-gated, scenario-only, inert when unset): swing both arms
                // forward-up through one windmill arc (theta max 0.8 rad, inside
                // the authored shoulder cone - limits untouched) during a cruise
                // window, and print trunk angular velocities before/during/after.
                // Measures the angular impulse the arm motors can deliver to the
                // trunk. The arm chains reuse the ReplayBraceArm walk; mTarget
                // stays null so the replay brace path never fires.
                std::optional<RagdollPose> armSupplyPose;
                if (armSupplyStart > 0)
                {
                    const int windowStep = static_cast<int>(step) - armSupplyStart;
                    const auto spine2 = findBody("spine2");
                    if (windowStep >= -12 && windowStep < 200 && pelvisIndex && spine2)
                    {
                        const auto& pelvisW = telemetry.mBodies[*pelvisIndex].mMotion.mAngularVelocity;
                        const auto& spineW = telemetry.mBodies[*spine2].mMotion.mAngularVelocity;
                        JPH::Vec3 handRel = JPH::Vec3::sZero();
                        const std::size_t measuredSide = loadedBalanceArm ? 1 : 0;
                        if (supplyArms[measuredSide])
                        {
                            const auto shoulder
                                = ragdoll->getWorldBodyPosition(pose, supplyArms[measuredSide]->mBodies[1]);
                            const auto hand
                                = ragdoll->getWorldBodyPosition(pose, supplyArms[measuredSide]->mBodies[3]);
                            if (shoulder && hand)
                                handRel = JPH::Vec3(*hand) - JPH::Vec3(*shoulder);
                        }
                        std::fprintf(stderr,
                            "arm-supply step=%d w=%d pelvis_w=(%.3f,%.3f,%.3f) spine2_w=(%.3f,%.3f,%.3f) hand_rel=(%.1f,%.1f,%.1f)\n",
                            static_cast<int>(step), windowStep, pelvisW.GetX(), pelvisW.GetY(), pelvisW.GetZ(),
                            spineW.GetX(), spineW.GetY(), spineW.GetZ(), handRel.GetX(), handRel.GetY(), handRel.GetZ());
                    }
                    if (armSupplySwing && windowStep >= 0 && windowStep < 36)
                    {
                        // Windmill pitch about the lateral (x) axis: 18
                        // steps up, 6 hold, 12 back. Theta max 0.8 rad
                        // stays inside the authored shoulder cone.
                        float theta;
                        if (windowStep < 18)
                            theta = 0.8f * static_cast<float>(windowStep + 1) / 18.0f;
                        else if (windowStep < 24)
                            theta = 0.8f;
                        else
                            theta = 0.8f * (1.0f - static_cast<float>(windowStep - 23) / 12.0f);
                        const JPH::Quat swing = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), theta);
                        armSupplyPose = plan->mTargetPose;
                        for (std::size_t side = 0; side < supplyArms.size(); ++side)
                        {
                            if (!supplyArms[side]) continue;
                            const auto& arm = *supplyArms[side];
                            const auto shoulder = ragdoll->getWorldBodyPosition(pose, arm.mBodies[1]);
                            const auto hand = ragdoll->getWorldBodyPosition(pose, arm.mBodies[3]);
                            if (!shoulder || !hand) continue;
                            if (!supplyRestInit)
                            {
                                for (std::size_t other = 0; other < supplyArms.size(); ++other)
                                {
                                    if (!supplyArms[other]) continue;
                                    const auto s = ragdoll->getWorldBodyPosition(pose, supplyArms[other]->mBodies[1]);
                                    const auto h = ragdoll->getWorldBodyPosition(pose, supplyArms[other]->mBodies[3]);
                                    if (s && h)
                                        supplyRest[other] = JPH::Vec3(*h) - JPH::Vec3(*s);
                                }
                                supplyRestInit = true;
                            }
                            const JPH::Vec3 rotated = swing * supplyRest[side];
                            const auto target = ragdoll->getModelPosition(JPH::RVec3(*shoulder + rotated));
                            const auto bend = ragdoll->getModelVector(JPH::Vec3(0.0f, 1.0f, 0.0f));
                            if (!target || !bend) continue;
                            ArmTargetInput armInput;
                            armInput.mShoulderBodyIndex = arm.mBodies[1];
                            armInput.mElbowBodyIndex = arm.mBodies[2];
                            armInput.mHandBodyIndex = arm.mBodies[3];
                            armInput.mTargetHandPosition = *target;
                            armInput.mPreferredBendDirection
                                = bend->IsNearZero() ? JPH::Vec3::sAxisZ() : bend->Normalized();
                            armInput.mWeight = loadedBalanceArm ? WeaponSwingProfile{}.mWeight : 1.0f;
                            armInput.mHingeGeometry
                                = ragdoll->getHingeLegGeometry(arm.mBodies[1], arm.mBodies[2], arm.mBodies[3]);
                            armInput.mShoulderGeometry = ragdoll->getSwingTwistJointGeometry(arm.mBodies[1]);
                            applyArmTargetPose(armSupplyPose->mBodyTransforms, plan->mTargetPose.mBodyTransforms,
                                pose.mBodyTransforms, arm.mBodies[0], armInput);
                        }
                    }
                }
                const RagdollPose& productionPose = armSupplyPose ? *armSupplyPose : plan->mTargetPose;
                const auto residualPose = correctedProductionPose(productionPose);
                const RagdollPose& drivenPose = residualPose ? *residualPose : productionPose;
                frame.mMotorsApplied = ragdoll->driveToPoseUsingMotors(drivenPose);
                if (!frame.mMotorsApplied)
                {
                    fail("Joint motors rejected the reference target pose at step " + std::to_string(step) + ".");
                    return result;
                }
                if (!recordDrivenMotorPose(drivenPose))
                {
                    fail("Reference target pose telemetry rejected at step " + std::to_string(step) + ".");
                    return result;
                }
                for (std::size_t i = 1; i < bodyCount; ++i)
                {
                    std::optional<JPH::Quat> worldTarget;
                    if (plan->mSwingHip && *plan->mSwingHip == i)
                        worldTarget = plan->mSwingHipWorldTarget;
                    for (const auto& [foot, rotation] : plan->mWorldFootTargets)
                        if (foot == i)
                            worldTarget = rotation;
                    std::optional<JPH::Vec3> angularVelocityTarget;
                    for (const auto& [body, velocity] : plan->mJointAngularVelocityTargets)
                        if (body == i)
                            angularVelocityTarget = velocity;
                    if (!ragdoll->driveMotorWithLoadTorque(i, plan->mLoadTorques[i], settings.mFixedTimeStep,
                            std::nullopt, worldTarget, angularVelocityTarget))
                    {
                        fail("Reference load motor rejected body " + std::to_string(i) + " at step "
                            + std::to_string(step) + ".");
                        return result;
                    }
                }
                referenceLoadTorques = plan->mLoadTorques;
                referencePlacement = plan->mPlacement;
                referenceSwingHistory = plan->mSwingTrajectoryHistory;
                frame.mJointSupport = plan->mSupport;
                frame.mSwingFootMotorTarget = plan->mSwingFootTarget;
                if (plan->mSwingPoseApplied && plan->mSwingHip)
                {
                    const LegChain& swingLeg = gaitOutput->leftFootIsStance() ? *rightLeg : *leftLeg;
                    frame.mSwingFootPoseTarget = plan->mTargetPose.mBodyTransforms[swingLeg.mFoot].mTranslation;
                }
                ReferenceGaitFrameRecord record;
                record.mStage = referenceState.mStage;
                record.mTimeInStage = referenceState.mTimeInStage;
                record.mEmergencyStep = referenceState.mEmergencyStep;
                record.mEmergencyStepCount = referenceState.mEmergencyStepCount;
                record.mSwingPoseApplied = plan->mSwingPoseApplied;
                record.mPlacement = plan->mPlacement;
                record.mRequestedAcceleration = plan->mRequestedAcceleration;
                record.mPelvisOrientationError = plan->mPelvisOrientationError;
                record.mBranchReaction = plan->mBranchReaction;
                record.mStanceHipTorque = plan->mStanceHipTorque;
                // Legacy CSV names: these are root-torque routing weights,
                // forced to the designated stance in Swing, not measured
                // contact-load fractions. Solved foot impulses are separate.
                record.mLeftLoadFraction = plan->mLeftLoadFraction;
                record.mRightLoadFraction = plan->mRightLoadFraction;
                record.mPlannedStanceFootLoad = plan->mPlannedStanceFootLoad;
                record.mStanceHipBody = plan->mStanceHip;
                record.mStanceAnkleBody = plan->mStanceAnkle;
                record.mStanceHipLoadTorque = plan->mLoadTorques[plan->mStanceHip];
                record.mStanceAnkleLoadTorque = plan->mLoadTorques[plan->mStanceAnkle];
                record.mTransferVelocity = plan->mTransferVelocity;
                record.mTransferReferenceVelocity = plan->mTransferReferenceVelocity;
                record.mSwingReachLimited = plan->mSwingReachLimited;
                record.mSwingHipFeedForwardTorque = plan->mSwingHipFeedForwardTorque;
                record.mSwingKneeFeedForwardTorque = plan->mSwingKneeFeedForwardTorque;
                record.mSwingHipRateTarget = plan->mSwingHipRateTarget;
                record.mSwingHipMeasuredRate = plan->mSwingHipMeasuredRate;
                record.mPlacementCenterOfMass = plan->mPlacementCenterOfMass;
                record.mPlacementVelocity = plan->mPlacementVelocity;
                record.mFeasibilityClampedCount = plan->mFeasibilityClampedCount;
                // Takeoff command-discontinuity metric: frame-to-frame change
                // of the stance knee target while the SAME leg keeps the
                // stance role. The swing -> stance touchdown swaps the stance
                // body and resets the comparison; the landing-press handover
                // at takeoff keeps it, which is where E10 found the jump.
                {
                    const LegChain& stanceChain = gaitOutput->leftFootIsStance() ? *leftLeg : *rightLeg;
                    const JPH::Quat kneeTarget
                        = plan->mTargetPose.mBodyTransforms[stanceChain.mKnee].mRotation;
                    if (previousStanceKneeTarget && previousStanceKneeTarget->first == stanceChain.mKnee)
                    {
                        const JPH::Quat jump
                            = (kneeTarget * previousStanceKneeTarget->second.Conjugated()).Normalized().EnsureWPositive();
                        JPH::Vec3 jumpAxis;
                        float jumpAngle = 0.0f;
                        jump.GetAxisAngle(jumpAxis, jumpAngle);
                        record.mStanceKneeModelTargetJumpLegacy = jumpAngle;
                    }
                    previousStanceKneeTarget = { stanceChain.mKnee, kneeTarget };
                    MotorTargetTelemetry hinge;
                    if (ragdoll->captureMotorTargetTelemetry(stanceChain.mKnee, hinge) && hinge.mTargetHingeAngle)
                    {
                        record.mStanceKneeHingeTarget = *hinge.mTargetHingeAngle;
                        if (previousStanceHingeTarget
                            && std::get<0>(*previousStanceHingeTarget) == stanceChain.mKnee
                            && std::get<1>(*previousStanceHingeTarget) + 1 == step)
                            record.mStanceKneeTargetJump
                                = std::abs(*hinge.mTargetHingeAngle - std::get<2>(*previousStanceHingeTarget));
                        previousStanceHingeTarget = { stanceChain.mKnee, step, *hinge.mTargetHingeAngle };
                    }
                    else
                        previousStanceHingeTarget.reset();
                }
                if (plan->mCoordinatedPoseApplied)
                {
                    const auto& pelvis = plan->mTargetPose.mBodyTransforms[*pelvisIndex];
                    const auto rotation = (currentRootRotation * pelvis.mRotation).Normalized();
                    const auto shape = bodies.GetShape(ragdoll->getBodyId(*pelvisIndex));
                    const auto center = currentRootPosition
                        + currentRootRotation * pelvis.mTranslation * settings.mUniformScale
                        + rotation * shape->GetCenterOfMass();
                    record.mCoordinatedPelvisTargetZ = center.GetZ();
                }
                frame.mReferenceGait = record;
                const LegChain& swingLeg = gaitOutput->leftFootIsStance() ? *rightLeg : *leftLeg;
                MotorTargetTelemetry motorTelemetry;
                if (ragdoll->captureMotorTargetTelemetry(swingLeg.mHip, motorTelemetry))
                    frame.mSwingHipMotorTarget = motorTelemetry;
                if (ragdoll->captureMotorTargetTelemetry(swingLeg.mKnee, motorTelemetry))
                    frame.mSwingKneeMotorTarget = motorTelemetry;
                if (ragdoll->captureMotorTargetTelemetry(swingLeg.mFoot, motorTelemetry))
                    frame.mSwingFootJointMotorTarget = motorTelemetry;
            }
            else if (drive && active.mMotorsEnabled)
            {
                // Physics owns the walk phase: while a stance is active the
                // swing leg's motor target follows a lifted trajectory from
                // its lift-off position to the capture-point placement, and
                // the two-bone solver rewrites hip, knee, and foot targets.
                RagdollPose sampledGaitPose;
                const RagdollPose* basePose = &motorTarget;
                if (!script.mGaitPoseCycle.empty() && gaitOutput
                    && (actuationDecision->mPoseSource == GaitPoseSource::GaitCycle
                        || actuationDecision->mPoseSource == GaitPoseSource::MeasuredTouchdown))
                {
                    if (!sampleGaitPoseCycle(script.mGaitPoseCycle, *gaitOutput, sampledGaitPose))
                    {
                        fail("The gait pose cycle could not be sampled at step " + std::to_string(step) + ".");
                        return result;
                    }
                    basePose = &sampledGaitPose;
                }
                RagdollPose standingPose = *basePose;
                if (referenceStandingLegPose && pelvisIndex && leftLeg && rightLeg)
                {
                    const std::array<std::size_t, 6> legs{ leftLeg->mHip, leftLeg->mKnee, leftLeg->mFoot,
                        rightLeg->mHip, rightLeg->mKnee, rightLeg->mFoot };
                    if (!applyGaitMeasuredLegTargets(standingPose.mBodyTransforms,
                            referenceStandingLegPose->mBodyTransforms, *pelvisIndex, legs, GaitMeasuredPoseFrame::Joint))
                    {
                        fail("The supported standing leg pose was rejected at step " + std::to_string(step) + ".");
                        return result;
                    }
                }
                RagdollPose drivePose = standingPose;
                if (referenceHandover.active()
                    && !referenceHandover.blendPose(standingPose, settings.mReferenceGait, drivePose))
                {
                    fail("The standing hand-over blend rejected the target pose at step " + std::to_string(step)
                        + ".");
                    return result;
                }
                if (gaitOutput && actuationDecision->mPoseSource == GaitPoseSource::MeasuredSwingLeg)
                {
                    const LegChain& swingLeg = gaitOutput->leftFootIsStance() ? *rightLeg : *leftLeg;
                    const std::array<std::size_t, 3> chain{ swingLeg.mHip, swingLeg.mKnee, swingLeg.mFoot };
                    if (!applyGaitMeasuredLegTargets(
                            drivePose.mBodyTransforms, pose.mBodyTransforms, *pelvisIndex, chain,
                            actuationDecision->mMeasuredPoseFrame))
                    {
                        fail("Measured load-bearing joint targets were rejected at step " + std::to_string(step) + ".");
                        return result;
                    }
                }
                if (gaitOutput && gaitOutput->hasStance() && gaitOutput->mStanceReady && swingFootTarget
                    && swingStartModel)
                {
                    const LegChain& swingLeg = gaitOutput->leftFootIsStance() ? *rightLeg : *leftLeg;
                    JPH::Vec3 placementModel = toModel(*swingFootTarget);
                    placementModel.SetZ(swingStartModel->GetZ());
                    const auto travelModel = ragdoll->getModelVector(horizontal(active.mRequestedHorizontalVelocity));
                    if (!travelModel)
                    {
                        fail("Gait direction conversion failed at step " + std::to_string(step) + ".");
                        return result;
                    }
                    const JPH::Vec3 bend = travelModel->IsNearZero() ? JPH::Vec3::sAxisY() : travelModel->Normalized();
                    SwingFootTrajectoryInput trajectoryInput;
                    trajectoryInput.mLiftOffPosition = *swingStartModel;
                    trajectoryInput.mPlacementPosition = placementModel;
                    trajectoryInput.mGait = *gaitOutput;
                    const auto footTrajectory
                        = calculateSwingFootTrajectoryState(trajectoryInput, profile.mActiveController.mGait);
                    if (!footTrajectory)
                    {
                        fail("Swing-foot trajectory was rejected at step " + std::to_string(step) + ".");
                        return result;
                    }
                    const JPH::Vec3& footTarget = footTrajectory->mPosition;
                    const TwoBoneLegTargetInput legInput{ swingLeg.mHip, swingLeg.mKnee, swingLeg.mFoot,
                        footTarget, bend, ragdoll->getHingeLegGeometry(swingLeg.mHip, swingLeg.mKnee, swingLeg.mFoot) };
                    const auto legPose = applySwingLegPose(drivePose.mBodyTransforms, basePose->mBodyTransforms,
                        pose.mBodyTransforms, *pelvisIndex, legInput, *actuationDecision);
                    if (!legPose)
                    {
                        fail("Swing-leg pose assembly was rejected at step " + std::to_string(step) + ".");
                        return result;
                    }
                    frame.mResolvedSwingFootTarget = legPose->mFrame.mTargetFootPosition;
                    if (legPose->mApplied)
                    {
                        frame.mSwingFootMotorTarget = footTarget;
                        frame.mSwingFootPoseTarget = legPose->mFootPosition;
                        RagdollPose worldHipPose = drivePose;
                        worldHipPose.mBodyTransforms[swingLeg.mHip].mRotation = legPose->mHipModelRotation;
                        swingHipWorldTarget = ragdoll->getWorldBodyRotation(worldHipPose, swingLeg.mHip);
                        if (!swingHipWorldTarget)
                        {
                            fail("Swing-hip world target was rejected at step " + std::to_string(step) + ".");
                            return result;
                        }
                        {
                            SwingFootControllerInput swingControlInput;
                            swingControlInput.mCurrentPosition = telemetry.mBodies[swingLeg.mFoot].mPosition;
                            swingControlInput.mTargetPosition = toWorld(footTarget);
                            swingControlInput.mCurrentVelocity
                                = telemetry.mBodies[swingLeg.mFoot].mMotion.mLinearVelocity;
                            swingControlInput.mTargetVelocity
                                = currentRootRotation * (footTrajectory->mLinearVelocity * settings.mUniformScale);
                            // The constrained thigh, calf, and foot accelerate
                            // together. Using only the small foot body's mass
                            // underestimates the force needed to unload the
                            // swing leg and leaves it pinned to the floor.
                            swingControlInput.mEffectiveMass = telemetry.mBodies[swingLeg.mHip].mMass
                                + telemetry.mBodies[swingLeg.mKnee].mMass + telemetry.mBodies[swingLeg.mFoot].mMass;
                            swingControlInput.mFootLifted = actuationDecision->mSwingFootInFlight;
                            frame.mSwingFootControl
                                = calculateSwingFootControl(swingControlInput, profile.mActiveController.mGait);
                            const bool swingAssistReady = !settings.mRequireStanceSupportForSwingAssist
                                || gaitOutput->mSwingFootLifted || gaitOutput->mStanceReady;
                            frame.mSwingAssistReady = swingAssistReady;
                            if (frame.mSwingFootControl && active.mSwingFootActuationEnabled && swingAssistReady)
                            {
                                if (settings.mSwingAssistAxes == ScenarioSwingAssistAxes::VerticalOnly)
                                {
                                    frame.mSwingFootControl->mForce.SetX(0.0f);
                                    frame.mSwingFootControl->mForce.SetY(0.0f);
                                }
                                if (!frame.mSwingFootControl->mForce.IsNearZero())
                                {
                                    const LegChain& stanceLeg
                                        = gaitOutput->leftFootIsStance() ? *leftLeg : *rightLeg;
                                    const std::size_t verticalReactionBody
                                        = actuationDecision->mSwingReactionThroughStanceFoot ? stanceLeg.mFoot : swingLeg.mHip;
                                    std::size_t horizontalReactionBody = stanceLeg.mFoot;
                                    if (!actuationDecision->mSwingReactionThroughStanceFoot)
                                    {
                                        switch (settings.mSwingHorizontalReaction)
                                        {
                                            case ScenarioSwingHorizontalReaction::SwingHip:
                                                horizontalReactionBody = swingLeg.mHip;
                                                break;
                                            case ScenarioSwingHorizontalReaction::Pelvis:
                                                horizontalReactionBody = *pelvisIndex;
                                                break;
                                            case ScenarioSwingHorizontalReaction::StanceHip:
                                                horizontalReactionBody = stanceLeg.mHip;
                                                break;
                                            case ScenarioSwingHorizontalReaction::StanceFoot:
                                                horizontalReactionBody = stanceLeg.mFoot;
                                                break;
                                        }
                                    }
                                    frame.mSwingFootVerticalReactionBody = verticalReactionBody;
                                    frame.mSwingFootHorizontalReactionBody = horizontalReactionBody;
                                    JPH::Vec3 verticalForce(0.0f, 0.0f, frame.mSwingFootControl->mForce.GetZ());
                                    JPH::Vec3 horizontalForce = frame.mSwingFootControl->mForce - verticalForce;
                                    bool applied = false;
                                    if (!verticalForce.IsNearZero())
                                    {
                                        const bool verticalApplied = ragdoll->applyBodyForcePair(swingLeg.mFoot,
                                            verticalReactionBody,
                                            verticalForce, profile.mActiveController.mGait.mMaximumSwingFootForce);
                                        if (!verticalApplied)
                                        {
                                            fail("Vertical swing-foot control was rejected at step "
                                                + std::to_string(step) + ".");
                                            return result;
                                        }
                                        applied = true;
                                    }
                                    if (!horizontalForce.IsNearZero())
                                    {
                                        const bool horizontalApplied = ragdoll->applyBodyForcePair(swingLeg.mFoot,
                                            horizontalReactionBody, horizontalForce,
                                            profile.mActiveController.mGait.mMaximumSwingFootHorizontalForce);
                                        applied = applied || horizontalApplied;
                                        if (!horizontalApplied)
                                        {
                                            fail("Horizontal swing-foot control was rejected at step "
                                                + std::to_string(step) + ".");
                                            return result;
                                        }
                                        if (settings.mCompensateSwingHorizontalReactionThroughStance
                                            && !actuationDecision->mSwingReactionThroughStanceFoot
                                            && !ragdoll->applyBodyForcePair(*pelvisIndex, stanceLeg.mFoot,
                                                horizontalForce,
                                                profile.mActiveController.mGait.mMaximumSwingFootHorizontalForce))
                                        {
                                            fail("Horizontal swing-reaction compensation was rejected at step "
                                                + std::to_string(step) + ".");
                                            return result;
                                        }
                                    }
                                    frame.mSwingFootControlApplied = applied;
                                    if (!applied)
                                    {
                                        fail("Swing-foot control was rejected at step " + std::to_string(step) + ".");
                                        return result;
                                    }
                                    const std::array<std::size_t, 1> verticalReactions{ verticalReactionBody };
                                    const std::array<std::size_t, 1> horizontalReactions{ horizontalReactionBody };
                                    frame.mAppliedSwingForcePairMoment
                                        = recordForcePair(frame, verticalReactions, verticalForce)
                                        + recordForcePair(frame, horizontalReactions, horizontalForce);
                                    if (settings.mCompensateSwingHorizontalReactionThroughStance
                                        && gaitOutput->mSwingFootLifted && !horizontalForce.IsNearZero())
                                    {
                                        const std::array<std::size_t, 1> stanceReaction{ stanceLeg.mFoot };
                                        frame.mAppliedSwingForcePairMoment
                                            += recordForcePair(frame, stanceReaction, horizontalForce);
                                    }
                                }
                            }
                        }
                    }
                }
                std::array<std::size_t, 2> disabledMotorBodies{};
                std::size_t disabledMotorCount = 0;
                std::span<const std::size_t> disabledMotors;
                if (gaitOutput && gaitOutput->hasStance())
                {
                    const LegChain& stanceLeg = gaitOutput->leftFootIsStance() ? *leftLeg : *rightLeg;
                    const LegChain& swingLeg = gaitOutput->leftFootIsStance() ? *rightLeg : *leftLeg;
                    const bool worldControl = settings.mUseWorldSwingHipControl && swingHipWorldTarget.has_value();
                    frame.mStanceHipMotorDisabled = settings.mReserveStanceHipMotor || worldControl;
                    frame.mSwingHipMotorDisabled = worldControl
                        || (settings.mDisableSwingHipMotor && gaitOutput->mStanceReady);
                    if (frame.mStanceHipMotorDisabled)
                    {
                        disabledMotorBodies[disabledMotorCount++] = stanceLeg.mHip;
                        frame.mDisabledStanceHipBody = stanceLeg.mHip;
                    }
                    if (frame.mSwingHipMotorDisabled)
                        disabledMotorBodies[disabledMotorCount++] = swingLeg.mHip;
                }
                disabledMotors = std::span<const std::size_t>(disabledMotorBodies.data(), disabledMotorCount);
                const auto setLegTorqueScale = [&](const LegChain& leg, float scale) {
                    for (const std::size_t bodyIndex : { leg.mHip, leg.mKnee, leg.mFoot })
                    {
                        if (bodyIndex >= profile.mBodies.size() || !profile.mBodies[bodyIndex].mConstraint
                            || !ragdoll->setMotorMaximumTorque(bodyIndex,
                                profile.mBodies[bodyIndex].mConstraint->mMotor.mMaximumTorque * scale
                                    * active.mStabilizationStrength))
                            return false;
                    }
                    return true;
                };
                // Restore both legs before assigning the current swing role;
                // otherwise a phase transition would leave the new stance leg
                // carrying the previous phase's raised torque limit.
                if (!setLegTorqueScale(*leftLeg, 1.0f) || !setLegTorqueScale(*rightLeg, 1.0f))
                {
                    fail("Leg motor torque limits could not be restored at step " + std::to_string(step) + ".");
                    return result;
                }
                if (gaitOutput && gaitOutput->hasStance()
                    && actuationDecision->mSwingLegMotorTorqueScale > 1.0f)
                {
                    const LegChain& swingLeg = gaitOutput->leftFootIsStance() ? *rightLeg : *leftLeg;
                    if (!setLegTorqueScale(swingLeg, actuationDecision->mSwingLegMotorTorqueScale))
                    {
                        fail("Swing-leg motor torque limit could not be applied at step " + std::to_string(step)
                            + ".");
                        return result;
                    }
                }
                for (const auto& arm : replayArms)
                {
                    if (!arm || !arm->mTarget) continue;
                    const auto target = ragdoll->getModelPosition(arm->mTarget->mWorldTarget);
                    const auto bend = ragdoll->getModelVector(arm->mTarget->mWorldBend);
                    if (!target || !bend) continue;
                    ArmTargetInput armInput;
                    armInput.mShoulderBodyIndex = arm->mBodies[1];
                    armInput.mElbowBodyIndex = arm->mBodies[2];
                    armInput.mHandBodyIndex = arm->mBodies[3];
                    armInput.mTargetHandPosition = *target;
                    armInput.mPreferredBendDirection = bend->IsNearZero() ? JPH::Vec3::sAxisZ() : bend->Normalized();
                    armInput.mWeight = arm->mWeight;
                    armInput.mHingeGeometry = ragdoll->getHingeLegGeometry(arm->mBodies[1], arm->mBodies[2], arm->mBodies[3]);
                    armInput.mShoulderGeometry = ragdoll->getSwingTwistJointGeometry(arm->mBodies[1]);
                    applyArmTargetPose(drivePose.mBodyTransforms, basePose->mBodyTransforms,
                        pose.mBodyTransforms, arm->mBodies[0], armInput);
                }
                // Apply only at the final ordinary pose input. The support
                // loads, world targets, disabled motors and rate limits below
                // retain exactly their production ownership.
                if (const auto residualPose = correctedProductionPose(drivePose))
                    drivePose = *residualPose;
                frame.mMotorsApplied = ragdoll->driveToPoseUsingMotors(drivePose, disabledMotors);
                if (!frame.mMotorsApplied)
                {
                    fail("Joint motors rejected the target pose at step " + std::to_string(step) + ".");
                    return result;
                }
                if (!recordDrivenMotorPose(drivePose))
                {
                    fail("Joint motor pose telemetry rejected at step " + std::to_string(step) + ".");
                    return result;
                }
                if (!active.mMotorTargetAngularVelocities.empty())
                {
                    for (std::size_t body = 0; body < bodyCount; ++body)
                    {
                        if (!profile.mBodies[body].mConstraint) continue;
                        if (!ragdoll->setMotorTargetAngularVelocity(body, active.mMotorTargetAngularVelocities[body]))
                        {
                            fail("Motor velocity reference rejected at step " + std::to_string(step));
                            return result;
                        }
                    }
                }
                if (useJointSupport)
                {
                    JointSupportOutput supportControl;
                    supportControl.mJointTorques.assign(bodyCount, JPH::Vec3::sZero());
                    // Native replay includes an uninitialized controller observation;
                    // standalone scenarios retain their authored startup actuation.
                    if (!replayResetSupportProbe
                        && (settings.mReplayBodyIds.empty() || observedSupportMotorEligible || referenceLocomotion)
                        && active.mTranslationActuationEnabled && supportEstimate)
                    {
                        const auto contacts = result.mFrames.empty() ? std::span<const GroundContact>{}
                            : std::span<const GroundContact>(result.mFrames.back().mContacts);
                        // After a stop the hand-over blend moves the standing
                        // support target from the center of mass at the
                        // hand-over to the mid-foot center.
                        const auto supportMotion = active.mSupportMotion ? active.mSupportMotion
                            : supportHandover.supportReference(supportEstimate->mCenter,
                                actuationDecisionInput.mMeasuredHorizontalVelocity, settings.mReferenceGait);
                        const auto control = calculateStandingJointSupport(telemetry, contacts,
                            feet, settings.mGravity, 0, profile.mActiveController, supportMotion);
                        if (!control)
                        {
                            fail("Joint support plan rejected at step " + std::to_string(step));
                            return result;
                        }
                        supportControl = *control;
                    }
                    frame.mJointSupport = supportControl;
                    for (std::size_t i = 1; i < bodyCount; ++i)
                        if (!ragdoll->driveMotorWithLoadTorque(i, supportControl.mJointTorques[i], settings.mFixedTimeStep))
                        {
                            fail("Support load motor rejected at step " + std::to_string(step));
                            return result;
                        }
                }
                if (gaitOutput && gaitOutput->hasStance() && !frame.mSwingHipMotorDisabled)
                {
                    const LegChain& swingLeg = gaitOutput->leftFootIsStance() ? *rightLeg : *leftLeg;
                    if (!ragdoll->limitMotorTargetStep(swingLeg.mHip,
                            profile.mActiveController.mGait.mMaximumSwingHipTargetSpeed * settings.mFixedTimeStep))
                    {
                        fail("Swing-hip constraint target rate limit was rejected at step " + std::to_string(step)
                            + ".");
                        return result;
                    }
                }
                if (gaitOutput && gaitOutput->hasStance())
                {
                    const LegChain& swingLeg = gaitOutput->leftFootIsStance() ? *rightLeg : *leftLeg;
                    const auto captureMotorTarget = [&](std::size_t bodyIndex,
                                                        std::optional<MotorTargetTelemetry>& destination,
                                                        const char* jointName) {
                        MotorTargetTelemetry telemetry;
                        if (!ragdoll->captureMotorTargetTelemetry(bodyIndex, telemetry))
                        {
                            fail(std::string("Swing ") + jointName
                                + " motor target could not be read at step " + std::to_string(step) + ".");
                            return false;
                        }
                        destination = std::move(telemetry);
                        return true;
                    };
                    if ((!frame.mSwingHipMotorDisabled
                            && !captureMotorTarget(swingLeg.mHip, frame.mSwingHipMotorTarget, "hip"))
                        || !captureMotorTarget(swingLeg.mKnee, frame.mSwingKneeMotorTarget, "knee")
                        || !captureMotorTarget(swingLeg.mFoot, frame.mSwingFootJointMotorTarget, "ankle"))
                        return result;
                    const LegChain& stanceLeg = gaitOutput->leftFootIsStance() ? *leftLeg : *rightLeg;
                    if ((actuationDecision->mReleaseStanceHipAxis && !frame.mStanceHipMotorDisabled
                            && !ragdoll->driveMotorWithFreeRotationAxis(stanceLeg.mHip,
                                actuationDecision->mFreeHipWorldAxis, settings.mFixedTimeStep))
                        || (actuationDecision->mReleaseSwingHipAxis && !frame.mSwingHipMotorDisabled
                            && !ragdoll->driveMotorWithFreeRotationAxis(swingLeg.mHip,
                                actuationDecision->mFreeHipWorldAxis, settings.mFixedTimeStep)))
                    {
                        fail("Loaded hip motor axis was rejected at step " + std::to_string(step) + ".");
                        return result;
                    }
                    frame.mStanceHipAxisReleased = actuationDecision->mReleaseStanceHipAxis
                        && !frame.mStanceHipMotorDisabled;
                    frame.mSwingHipAxisReleased = actuationDecision->mReleaseSwingHipAxis
                        && !frame.mSwingHipMotorDisabled;
                    for (const std::size_t foot : selectGaitWorldFootMotors(*actuationDecision,
                             supportEstimate.value_or(SupportEstimate{}), { *leftFootIndex, *rightFootIndex }))
                    {
                        const auto footRotation = ragdoll->getWorldBodyRotation(*basePose, foot);
                        if (!footRotation || !ragdoll->driveMotorToWorldOrientation(
                                foot, *footRotation, settings.mFixedTimeStep))
                        {
                            fail("Supported-foot world motor was rejected at step " + std::to_string(step) + ".");
                            return result;
                        }
                        if (foot == swingLeg.mFoot
                            && !captureMotorTarget(foot, frame.mSwingFootJointMotorTarget, "world ankle"))
                            return result;
                    }
                }
            }
            else
                ragdoll->disableMotors();

            if (!directMotorTracking && !referenceLocomotion && drive && active.mMotorsEnabled
                && settings.mUseWorldSwingHipControl && gaitOutput && gaitOutput->hasStance() && swingHipWorldTarget)
            {
                const LegChain& stanceLeg = gaitOutput->leftFootIsStance() ? *leftLeg : *rightLeg;
                const LegChain& swingLeg = gaitOutput->leftFootIsStance() ? *rightLeg : *leftLeg;
                const WorldOrientationControllerInput input{ telemetry.mBodies[swingLeg.mHip].mRotation,
                    *swingHipWorldTarget, telemetry.mBodies[swingLeg.mHip].mMotion.mAngularVelocity };
                const auto control = calculateSwingHipWorldControl(input, profile.mActiveController.mGait);
                if (!control
                    || !ragdoll->applyBodyTorquePair(swingLeg.mHip, stanceLeg.mHip, control->mTorque,
                        profile.mActiveController.mGait.mMaximumSwingHipWorldTorque))
                {
                    fail("World swing-hip control was rejected at step " + std::to_string(step) + ".");
                    return result;
                }
                frame.mSwingHipWorldControlApplied = true;
                frame.mSwingHipWorldTorqueSaturated = control->mSaturated;
                frame.mAppliedSwingHipWorldTorque = control->mTorque;
            }

            // The reference route discards the hip torque unless asked for
            // it; see ActiveControllerProfile::mReferenceHipStrategy.
            const bool hipStrategy = referenceLocomotion && profile.mActiveController.mReferenceHipStrategy;
            // Reference walking releases control during recovery. Native then
            // applies ordinary recovery assistance; selecting the reference
            // route must not suppress that assistance in the scenario adapter.
            const bool recoveryUpright = actuationDecisionInput.mRecovering && !useJointSupport;
            if (!directMotorTracking && drive && uprightOutput && !ragdoll->isRootKinematic()
                && (hipStrategy || recoveryUpright || (!referenceLocomotion && !useJointSupport)))
            {
                frame.mUprightActuationStage = uprightStage;
                UprightControllerOutput applied = *uprightOutput;
                applied.mPelvisForce *= active.mStabilizationStrength;
                applied.mPelvisTorque *= active.mStabilizationStrength;
                applied.mSpineTorque *= active.mStabilizationStrength;
                // Torque only on the reference route: joint support is
                // already driving the centre of mass.
                if (!active.mTranslationActuationEnabled || (hipStrategy && !recoveryUpright))
                    applied.mPelvisForce = JPH::Vec3::sZero();
                if (!active.mUprightTorqueEnabled)
                {
                    applied.mPelvisTorque = JPH::Vec3::sZero();
                    applied.mSpineTorque = JPH::Vec3::sZero();
                }
                frame.mAppliedUpright = applied;
                GaitActuationDecision routingDecision = *actuationDecision;
                if (!settings.mRouteGaitReactionThroughStanceLeg)
                {
                    routingDecision.mUseStanceVerticalReaction = false;
                    routingDecision.mUseStanceHorizontalReaction = false;
                }
                auto reactions = selectGaitReactionBodies(routingDecision,
                    supportEstimate.value_or(SupportEstimate{}), feet, leftLeg->mHip, rightLeg->mHip,
                    controllerOutput && controllerOutput->mMode == ActiveControllerMode::BlendBack);
                frame.mVerticalReactionBodies = std::move(reactions.mVertical);
                frame.mHorizontalReactionBodies = std::move(reactions.mHorizontal);
                // The legacy gait diagnostic may change walking routing, but
                // must not discard recovery's required internal reaction.
                frame.mAngularReactionBody = (settings.mRouteGaitReactionThroughStanceLeg
                    || routingDecision.mRequireInternalAngularReaction)
                    ? reactions.mAngular : std::nullopt;
                frame.mSecondaryAngularReactionBody = frame.mAngularReactionBody
                    ? reactions.mSecondaryAngular : std::nullopt;
                if (settings.mRouteGaitHorizontalReactionToStanceFoot
                    && routingDecision.mUseStanceHorizontalReaction)
                    frame.mHorizontalReactionBodies = {
                        gaitOutput->leftFootIsStance() ? leftLeg->mFoot : rightLeg->mFoot };
                if (!applied.mPelvisForce.IsNearZero() || !applied.mPelvisTorque.IsNearZero()
                    || !applied.mSpineTorque.IsNearZero())
                {
                    const bool groundContactRouting
                        = settings.mTranslationRouting == ScenarioTranslationRouting::GroundContactForce;
                    const bool useGroundContactForce = groundContactRouting && gaitOutput
                        && gaitOutput->mSwingFootLifted && !frame.mRequestedHorizontalVelocity.IsNearZero();
                    UprightControllerOutput bodyControl = applied;
                    if (useGroundContactForce)
                    {
                        bodyControl.mPelvisForce.SetX(0.0f);
                        bodyControl.mPelvisForce.SetY(0.0f);
                    }
                    frame.mUprightApplied = ragdoll->applyUprightControl(bodyControl, *pelvisIndex, *torsoIndex,
                        std::span<const std::size_t>(frame.mVerticalReactionBodies),
                        useGroundContactForce ? std::span<const std::size_t>()
                                              : std::span<const std::size_t>(frame.mHorizontalReactionBodies),
                        profile.mActiveController, frame.mAngularReactionBody, routingDecision.mLocomoting,
                        frame.mSecondaryAngularReactionBody);
                    if (!frame.mUprightApplied)
                    {
                        fail("Upright control was rejected at step " + std::to_string(step) + ".");
                        return result;
                    }
                    if (useGroundContactForce)
                    {
                        const JPH::Vec3 desiredGroundForce = horizontal(applied.mPelvisForce);
                        if (!desiredGroundForce.IsNearZero() && !result.mFrames.empty())
                        {
                            std::vector<std::pair<std::size_t, JPH::RVec3>> supportContacts;
                            for (const std::size_t bodyIndex : frame.mVerticalReactionBodies)
                            {
                                for (const GroundContact& contact : result.mFrames.back().mContacts)
                                {
                                    if (contact.mBodyIndex == bodyIndex && contact.mNormal.GetZ() > 0.0f)
                                        supportContacts.emplace_back(bodyIndex, contact.mPosition);
                                }
                            }
                            if (!supportContacts.empty())
                            {
                                JPH::Vec3 groundForce = desiredGroundForce;
                                JPH::Vec3 horizontalForce = groundForce;
                                const float horizontalLength = horizontalForce.Length();
                                const double groundHeight
                                    = std::accumulate(supportContacts.begin(), supportContacts.end(), 0.0,
                                          [](double sum, const std::pair<std::size_t, JPH::RVec3>& contact) {
                                              return sum + static_cast<double>(contact.second.GetZ());
                                          })
                                    / static_cast<double>(supportContacts.size());
                                const float centerOfMassHeight = static_cast<float>(
                                    static_cast<double>(telemetry.mCenterOfMass.GetZ()) - groundHeight);
                                JPH::RVec3 centerOfPressure(
                                    telemetry.mCenterOfMass.GetX(), telemetry.mCenterOfMass.GetY(), groundHeight);
                                const float normalLoad = telemetry.mTotalMass * settings.mGravity
                                    + std::max(0.0f, applied.mPelvisForce.GetZ());
                                if (horizontalLength > 1e-5f && normalLoad > 1e-5f && centerOfMassHeight > 1e-5f)
                                {
                                    const JPH::Vec3 direction = horizontalForce / horizontalLength;
                                    float minimumProjection = std::numeric_limits<float>::max();
                                    for (const auto& [_, position] : supportContacts)
                                        minimumProjection
                                            = std::min(minimumProjection, JPH::Vec3(position).Dot(direction));
                                    const float availableLever = std::max(
                                        0.0f, JPH::Vec3(telemetry.mCenterOfMass).Dot(direction) - minimumProjection);
                                    const float maximumHorizontalForce
                                        = normalLoad * availableLever / centerOfMassHeight;
                                    const float limitedLength = std::min(horizontalLength, maximumHorizontalForce);
                                    horizontalForce = direction * limitedLength;
                                    groundForce.SetX(horizontalForce.GetX());
                                    groundForce.SetY(horizontalForce.GetY());
                                    centerOfPressure -= direction * (limitedLength * centerOfMassHeight / normalLoad);
                                }
                                else
                                    groundForce = JPH::Vec3::sZero();
                                if (!groundForce.IsNearZero())
                                {
                                    const auto closest = std::min_element(supportContacts.begin(),
                                        supportContacts.end(), [&](const auto& left, const auto& right) {
                                            return (left.second - centerOfPressure).LengthSq()
                                                < (right.second - centerOfPressure).LengthSq();
                                        });
                                    bodies.AddForce(ragdoll->getBodyId(closest->first), groundForce, centerOfPressure,
                                        JPH::EActivation::Activate);
                                    frame.mAppliedGroundReactionMoment
                                        = JPH::Vec3(centerOfPressure - telemetry.mCenterOfMass).Cross(groundForce);
                                    frame.mAppliedGroundReactionForce = groundForce;
                                    frame.mAppliedNetForce += groundForce;
                                    frame.mAppliedPelvisForce += groundForce;
                                }
                            }
                        }
                        if (!frame.mVerticalReactionBodies.empty())
                            frame.mAppliedPelvisForce.SetZ(applied.mPelvisForce.GetZ());
                    }
                    else
                    {
                        frame.mAppliedPelvisForce = applied.mPelvisForce;
                        if (frame.mVerticalReactionBodies.empty())
                            frame.mAppliedPelvisForce.SetZ(0.0f);
                        if (frame.mHorizontalReactionBodies.empty())
                        {
                            frame.mAppliedPelvisForce.SetX(0.0f);
                            frame.mAppliedPelvisForce.SetY(0.0f);
                        }
                    }
                    frame.mAppliedPelvisTorque = applied.mPelvisTorque;
                    frame.mAppliedSpineTorque = applied.mSpineTorque;
                    if (!useGroundContactForce)
                    {
                        const JPH::Vec3 verticalForce(0.0f, 0.0f, frame.mAppliedPelvisForce.GetZ());
                        const JPH::Vec3 horizontalForce = frame.mAppliedPelvisForce - verticalForce;
                        frame.mAppliedUprightForcePairMoment
                            += recordForcePair(frame, frame.mVerticalReactionBodies, verticalForce);
                        frame.mAppliedUprightForcePairMoment
                            += recordForcePair(frame, frame.mHorizontalReactionBodies, horizontalForce);
                    }
                    else
                    {
                        const JPH::Vec3 verticalForce(0.0f, 0.0f, frame.mAppliedPelvisForce.GetZ());
                        frame.mAppliedUprightForcePairMoment
                            += recordForcePair(frame, frame.mVerticalReactionBodies, verticalForce);
                    }
                    frame.mAppliedTorqueSum += applied.mPelvisTorque + applied.mSpineTorque;
                    if (frame.mAngularReactionBody)
                        frame.mAppliedTorqueSum -= applied.mPelvisTorque + applied.mSpineTorque;
                }
            }

            if (frame.mMotorsApplied)
            {
                frame.mJointMotorTargets.resize(bodyCount);
                for (std::size_t body = 1; body < bodyCount; ++body)
                {
                    MotorTargetTelemetry target;
                    if (ragdoll->captureMotorTargetTelemetry(body, target))
                        frame.mJointMotorTargets[body] = target;
                }
            }

            const auto applyImpulse = [&](const ScenarioImpulse& impulse) {
                const JPH::BodyID id = ragdoll->getBodyId(impulse.mBodyIndex);
                if (impulse.mWorldPoint)
                    bodies.AddImpulse(id, impulse.mImpulse, *impulse.mWorldPoint);
                else
                    bodies.AddImpulse(id, impulse.mImpulse);
                ++frame.mImpulseCount;
            };
            while (impulseCursor < impulses.size() && impulses[impulseCursor].mStep <= step)
            {
                const ScenarioImpulse& impulse = impulses[impulseCursor];
                if (impulse.mStep == step)
                    applyImpulse(impulse);
                ++impulseCursor;
            }
            for (const ScenarioImpulse& impulse : active.mImpulses)
                if (impulse.mStep == step)
                    applyImpulse(impulse);
            active.mImpulses.clear();

            const bool observingSolver = settings.mObserveSolverImpulses && settings.mCollisionSteps == 1
                && !ragdoll->isRootKinematic()
                && ragdoll->beginSolverImpulseMeasurement(settings.mFixedTimeStep);
            // Keep the bounded replay command window through the first driven
            // recovery, so pre-solve targets can be compared with native logs.
            if (step < (settings.mReplayBodyIds.empty() ? 16 : 1800))
            {
                auto& targets = result.mStartupMotorTargets.emplace_back(bodyCount);
                for (std::size_t body = 1; body < bodyCount; ++body)
                    ragdoll->captureMotorTargetTelemetry(body, targets[body]);
                if (!settings.mReplayBodyIds.empty() && leftLeg && rightLeg && frame.mReferenceGait
                    && frame.mReferenceGait->mStage == ReferenceGaitStage::Swing
                    && RagdollCore::tuningText("OPENMW_RAGDOLL_LIMB_CLEARANCE_TRACE"))
                {
                    const auto& swing = referenceState.leftFootIsStance() ? *rightLeg : *leftLeg;
                    const auto& stance = referenceState.leftFootIsStance() ? *leftLeg : *rightLeg;
                    const std::array<std::size_t, 3> swingIds{ swing.mHip, swing.mKnee, swing.mFoot };
                    const std::array<std::size_t, 3> stanceIds{ stance.mHip, stance.mKnee, stance.mFoot };
                    const auto measured = [&](const auto& ids) {
                        std::array<LimbShapePose, 3> poses;
                        for (std::size_t i = 0; i < ids.size(); ++i)
                        {
                            const auto id = ragdoll->getBodyId(ids[i]);
                            poses[i] = { bodies.GetShape(id).GetPtr(), bodies.GetPosition(id), bodies.GetRotation(id) };
                        }
                        return poses;
                    };
                    const auto actual = measured(swingIds);
                    const auto support = measured(stanceIds);
                    auto commanded = actual;
                    const auto geometry = ragdoll->getHingeLegGeometry(swing.mHip, swing.mKnee, swing.mFoot);
                    const auto& hipTarget = targets[swing.mHip];
                    const auto& kneeTarget = targets[swing.mKnee];
                    const auto& ankleTarget = targets[swing.mFoot];
                    if (geometry && hipTarget.mChildWorldTarget && kneeTarget.mTargetHingeAngle && kneeTarget.mHingeMinimum
                        && ankleTarget.mChildWorldTarget)
                    {
                        // Hinge target is constraint-relative; geometry's reference
                        // pose may have a nonzero creation angle. Keep that offset.
                        const float referenceAngle = *kneeTarget.mHingeMinimum - geometry->mMinimumAngle;
                        const float bend = *kneeTarget.mTargetHingeAngle - referenceAngle;
                        commanded[0].mRotation = *hipTarget.mChildWorldTarget;
                        commanded[1].mPosition = commanded[0].mPosition
                            + JPH::RVec3(commanded[0].mRotation * geometry->mUpperOffset * settings.mUniformScale);
                        commanded[1].mRotation = (commanded[0].mRotation
                            * JPH::Quat::sRotation(geometry->mHingeAxis, bend) * geometry->mLowerRotation).Normalized();
                        commanded[2].mPosition = commanded[1].mPosition
                            + JPH::RVec3(commanded[1].mRotation
                                * (geometry->mLowerRotation.Conjugated() * geometry->mLowerOffset)
                                * settings.mUniformScale);
                        commanded[2].mRotation = ankleTarget.mWorldOrientation ? *ankleTarget.mChildWorldTarget
                            : (commanded[1].mRotation * actual[1].mRotation.Conjugated()
                                * *ankleTarget.mChildWorldTarget).Normalized();
                        if (step >= 582 && step <= 596 && RagdollCore::tuningText("OPENMW_RAGDOLL_LIMB_POSE_SEARCH"))
                        {
                            const auto hipLimits = ragdoll->getSwingTwistJointGeometry(swing.mHip);
                            const auto ankleLimits = ragdoll->getSwingTwistJointGeometry(swing.mFoot);
                            if (hipLimits && ankleLimits)
                            {
                                const auto shrink = [&](SwingTwistJointGeometry limits) {
                                    const float margin = settings.mReferenceGait.mFinalCommandFeasibilitySwingMargin;
                                    limits.mNormalHalfConeAngle = std::max(0.0f, limits.mNormalHalfConeAngle - margin);
                                    limits.mPlaneHalfConeAngle = std::max(0.0f, limits.mPlaneHalfConeAngle - margin);
                                    const float center = 0.5f * (limits.mTwistMinimum + limits.mTwistMaximum);
                                    limits.mTwistMinimum += margin;
                                    limits.mTwistMaximum -= margin;
                                    if (limits.mTwistMinimum > limits.mTwistMaximum)
                                        limits.mTwistMinimum = limits.mTwistMaximum = center;
                                    return limits;
                                };
                                LimbPoseSearchInput query;
                                for (std::size_t i = 0; i < 3; ++i) query.mShapes[i] = actual[i].mShape;
                                query.mHipPosition = actual[0].mPosition;
                                query.mParentRotation = bodies.GetRotation(ragdoll->getBodyId(*pelvisIndex));
                                query.mHipRotation = commanded[0].mRotation;
                                query.mFootRotation = commanded[2].mRotation;
                                query.mLeg = *geometry;
                                query.mHipLimits = shrink(*hipLimits);
                                query.mAnkleLimits = shrink(*ankleLimits);
                                query.mKneeAngleFromReference = bend;
                                query.mScale = settings.mUniformScale;
                                query.mMinimumWorldZ = settings.mGroundHeight;
                                if (RagdollCore::tuningText("OPENMW_RAGDOLL_A116_ONLINE"))
                                {
                                    // Default 2.5 matches the A116-A118 pilots. The env override is a
                                    // diagnostic knob for matched margin experiments only; it changes
                                    // behavior and is not an encoded/profiled setting.
                                    float clearance = 2.5f;
                                    if (const char* overrideClearance = RagdollCore::tuningText("OPENMW_RAGDOLL_A118_CLEARANCE"))
                                    {
                                        const float parsed = std::strtof(overrideClearance, nullptr);
                                        if (std::isfinite(parsed) && parsed >= 0.0f)
                                            clearance = parsed;
                                    }
                                    const float dt = settings.mFixedTimeStep;
                                    const auto advanceRotation = [&](JPH::QuatArg q, JPH::Vec3Arg w) {
                                        const float speed = w.Length();
                                        return speed > 1e-8f
                                            ? (JPH::Quat::sRotation(w / speed, speed * dt) * q).Normalized() : q;
                                    };
                                    const auto predicted = [&](std::size_t id, const LimbShapePose& pose) {
                                        const auto body = ragdoll->getBodyId(id);
                                        const auto q = advanceRotation(pose.mRotation, bodies.GetAngularVelocity(body));
                                        const auto center = bodies.GetCenterOfMassPosition(body);
                                        const auto localCenter = pose.mRotation.Conjugated() * JPH::Vec3(center - pose.mPosition);
                                        return LimbShapePose{ pose.mShape,
                                            center + JPH::RVec3(bodies.GetLinearVelocity(body) * dt - q * localCenter), q };
                                    };
                                    const bool prior = diagnosticOnlinePose && diagnosticOnlineStep
                                        && step == *diagnosticOnlineStep + 1 && diagnosticOnlineFoot == swing.mFoot;
                                    LimbPoseContinuity edge;
                                    edge.mMaximumFootTravel = 6;
                                    edge.mMaximumJointRotation = 0.35f;
                                    edge.mIntervals = 8;
                                    if (prior)
                                    {
                                        edge.mPrevious = *diagnosticOnlinePose;
                                        edge.mPreviousSupport = diagnosticOnlineSupport;
                                        edge.mPreviousParentRotation = diagnosticOnlineParent;
                                    }
                                    auto current = findClearLimbPose(query, support, clearance, 12, prior ? &edge : nullptr);
                                    const bool committing = prior && diagnosticOnlineNext
                                        && RagdollCore::tuningText("OPENMW_RAGDOLL_A118_COMMIT");
                                    int commitmentRejection = 0;
                                    if (committing)
                                    {
                                        auto committedQuery = query;
                                        committedQuery.mHipRotation = diagnosticOnlineNext->mPoses[0].mRotation;
                                        committedQuery.mFootRotation = diagnosticOnlineNext->mPoses[2].mRotation;
                                        committedQuery.mKneeAngleFromReference = diagnosticOnlineNext->mKneeAngleFromReference;
                                        // Zero displacement rejects a newly clamped/replaced target.
                                        current = findClearLimbPose(committedQuery, support, clearance, 0);
                                        if (!current) commitmentRejection = 1; // Current geometry/limits/floor query rejected.
                                        if (current && (!current->mPoses[0].mRotation.IsClose(committedQuery.mHipRotation, 1e-10f)
                                            || !current->mPoses[2].mRotation.IsClose(committedQuery.mFootRotation, 1e-10f)
                                            || std::abs(current->mKneeAngleFromReference - committedQuery.mKneeAngleFromReference) > 1e-6f
                                            || !isLimbTransitionClear(committedQuery, *current, support, clearance, edge)))
                                        { commitmentRejection = 2; current.reset(); } // Orientation/hinge/transition recheck.
                                        if (RagdollCore::tuningText("OPENMW_RAGDOLL_A118_REASONS"))
                                        {
                                            // Query-only subcondition decomposition. Reconstructs the exact
                                            // requested pose (no clamping) so the transition recheck can be
                                            // diagnosed even when the pose query itself rejected.
                                            const auto poseDiagnosis = diagnoseLimbPose(committedQuery, support, clearance);
                                            ClearLimbPose exact;
                                            exact.mKneeAngleFromReference = committedQuery.mKneeAngleFromReference;
                                            exact.mPoses[0] = { committedQuery.mShapes[0], committedQuery.mHipPosition,
                                                committedQuery.mHipRotation };
                                            const auto exactCalf = (committedQuery.mHipRotation
                                                * JPH::Quat::sRotation(committedQuery.mLeg.mHingeAxis,
                                                    committedQuery.mKneeAngleFromReference)
                                                * committedQuery.mLeg.mLowerRotation).Normalized();
                                            exact.mPoses[1] = { committedQuery.mShapes[1], committedQuery.mHipPosition
                                                + JPH::RVec3(committedQuery.mHipRotation
                                                    * committedQuery.mLeg.mUpperOffset * committedQuery.mScale), exactCalf };
                                            exact.mPoses[2] = { committedQuery.mShapes[2], exact.mPoses[1].mPosition
                                                + JPH::RVec3(exactCalf * (committedQuery.mLeg.mLowerRotation.Conjugated()
                                                    * committedQuery.mLeg.mLowerOffset) * committedQuery.mScale),
                                                committedQuery.mFootRotation };
                                            const auto transitionDiagnosis = diagnoseLimbTransition(
                                                committedQuery, exact, support, clearance, edge);
                                            std::cerr << "RAGDOLL_A118_REASON step=" << step
                                                << " rejection=" << commitmentRejection
                                                << " hip_clamp=" << poseDiagnosis.mHipClamped
                                                << " hip_delta=" << poseDiagnosis.mHipClampDelta
                                                << " knee_clamp=" << poseDiagnosis.mKneeClamped
                                                << " knee_delta=" << poseDiagnosis.mKneeClampDelta
                                                << " ankle_clamp=" << poseDiagnosis.mAnkleClamped
                                                << " ankle_delta=" << poseDiagnosis.mAnkleClampDelta
                                                << " exact_disp=" << poseDiagnosis.mExactFootDisplacement
                                                << " floor_clear=" << poseDiagnosis.mFloorClear
                                                << " floor_margin=" << poseDiagnosis.mFloorMargin
                                                << " support_clear=" << poseDiagnosis.mSupportClear
                                                << " support_gap=" << poseDiagnosis.mSupportGap
                                                << " transition_failure=" << static_cast<int>(transitionDiagnosis.mFailure)
                                                << " foot_travel=" << transitionDiagnosis.mFootTravel
                                                << " knee_change=" << transitionDiagnosis.mKneeChange
                                                << " hip_change=" << transitionDiagnosis.mHipChange
                                                << " ankle_change=" << transitionDiagnosis.mAnkleChange
                                                << " failing_sample=" << transitionDiagnosis.mFirstFailingSample
                                                << " sample_floor_margin=" << transitionDiagnosis.mSampleFloorMargin
                                                << " sample_support_gap=" << transitionDiagnosis.mSampleSupportGap
                                                << '\n';
                                        }
                                    }
                                    const auto previousEdge = edge;
                                    std::optional<ClearLimbPose> next;
                                    std::optional<LimbPoseRates> rates;
                                    const auto connectFuture = [&]()
                                    {
                                        auto future = query;
                                        future.mHipPosition = predicted(swing.mHip, actual[0]).mPosition;
                                        future.mParentRotation = advanceRotation(query.mParentRotation,
                                            bodies.GetAngularVelocity(ragdoll->getBodyId(*pelvisIndex)));
                                        future.mHipRotation = advanceRotation(query.mHipRotation, hipTarget.mTargetAngularVelocity);
                                        const float kneeRate = kneeTarget.mTargetAngularVelocity.Dot(
                                            actual[0].mRotation * geometry->mHingeAxis);
                                        future.mKneeAngleFromReference += kneeRate * dt;
                                        const auto futureCalf = (future.mHipRotation
                                            * JPH::Quat::sRotation(geometry->mHingeAxis, future.mKneeAngleFromReference)
                                            * geometry->mLowerRotation).Normalized();
                                        const auto relativeAnkle = (commanded[1].mRotation.Conjugated()
                                            * query.mFootRotation).Normalized();
                                        future.mFootRotation = (futureCalf * advanceRotation(relativeAnkle,
                                            actual[1].mRotation.Conjugated() * ankleTarget.mTargetAngularVelocity)).Normalized();
                                        std::array<LimbShapePose, 3> futureSupport;
                                        for (std::size_t i = 0; i < 3; ++i)
                                            futureSupport[i] = predicted(stanceIds[i], support[i]);
                                        edge.mPrevious = *current;
                                        edge.mPreviousSupport = support;
                                        edge.mPreviousParentRotation = query.mParentRotation;
                                        next = findClearLimbPose(future, futureSupport, clearance, 12, &edge);
                                        if (next) rates = calculateLimbPoseRates(*current, *next, dt);
                                    }
                                    ;
                                    if (current) connectFuture();
                                    std::size_t alternatives = 0;
                                    if (current && !next && !committing && RagdollCore::tuningText("OPENMW_RAGDOLL_A117_PAIRS"))
                                    {
                                        auto candidates = enumerateClearLimbPoses(query, support, clearance, 12);
                                        std::vector<std::size_t> order;
                                        for (std::size_t i = 0; i < candidates.size(); ++i) order.push_back(i);
                                        std::stable_sort(order.begin(), order.end(), [&](const auto x, const auto y) {
                                            return candidates[x].mFootDisplacement < candidates[y].mFootDisplacement;
                                        });
                                        for (const auto index : order)
                                        {
                                            const auto& candidate = candidates[index];
                                            if (prior && !isLimbTransitionClear(query, candidate, support, clearance, previousEdge))
                                                continue;
                                            ++alternatives;
                                            current = candidate;
                                            connectFuture();
                                            if (next) break;
                                        }
                                    }
                                    const double nominalDeviation = current
                                        ? (current->mPoses[2].mPosition - commanded[2].mPosition).Length() : -1;
                                    const bool applied = current && next && rates;
                                    if (applied)
                                    {
                                        const auto& pose = current->mPoses;
                                        if (!ragdoll->diagnosticOverrideMotorPose(swing.mHip,
                                                (query.mParentRotation.Conjugated() * pose[0].mRotation).Normalized(), 0,
                                                rates->mHipWorld)
                                            || !ragdoll->diagnosticOverrideMotorPose(swing.mKnee,
                                                (pose[0].mRotation.Conjugated() * pose[1].mRotation).Normalized(),
                                                current->mKneeAngleFromReference + referenceAngle,
                                                actual[0].mRotation * geometry->mHingeAxis * rates->mKnee)
                                            || !ragdoll->diagnosticOverrideMotorPose(swing.mFoot,
                                                (pose[1].mRotation.Conjugated() * pose[2].mRotation).Normalized(), 0,
                                                actual[1].mRotation * rates->mAnkleParent))
                                        { fail("A116 online command override rejected"); return result; }
                                        for (const auto id : swingIds) ragdoll->captureMotorTargetTelemetry(id, targets[id]);
                                        commanded = pose;
                                        diagnosticOnlinePose = current;
                                        diagnosticOnlineNext = next;
                                        diagnosticOnlineSupport = support;
                                        diagnosticOnlineParent = query.mParentRotation;
                                        diagnosticOnlineStep = step;
                                        diagnosticOnlineFoot = swing.mFoot;
                                    }
                                    else { diagnosticOnlinePose.reset(); diagnosticOnlineNext.reset(); }
                                    std::cerr << "RAGDOLL_A116_ONLINE step=" << step << " current=" << current.has_value()
                                        << " rejection=" << commitmentRejection
                                        << " nominal_deviation=" << nominalDeviation
                                        << " next=" << next.has_value() << " committing=" << committing << " alternatives=" << alternatives << " prior=" << prior << " applied=" << applied
                                        << " deviation=" << (current ? current->mFootDisplacement : -1) << '\n';
                                }
                                const bool continuous = RagdollCore::tuningText("OPENMW_RAGDOLL_LIMB_POSE_CONTINUITY");
                                if (!diagnosticLimbStep || step != *diagnosticLimbStep + 1
                                    || diagnosticLimbFoot != swing.mFoot)
                                {
                                    diagnosticLimbPose.reset();
                                    diagnosticLimbBlocked = false;
                                    diagnosticLimbRoutes.clear();
                                    diagnosticRouteBlocked = false;
                                }
                                LimbPoseContinuity continuity;
                                const bool seeded = continuous && diagnosticLimbPose.has_value();
                                if (seeded)
                                {
                                    continuity.mPrevious = *diagnosticLimbPose;
                                    continuity.mPreviousParentRotation = diagnosticLimbParent;
                                    continuity.mPreviousSupport = diagnosticLimbSupport;
                                    continuity.mMaximumFootTravel = 6.0f;
                                    continuity.mMaximumJointRotation = 0.35f;
                                    continuity.mIntervals = 8;
                                }
                                const auto choice = continuous && diagnosticLimbBlocked ? std::nullopt
                                    : findClearLimbPose(query, support, 1.0f, 12.0f, seeded ? &continuity : nullptr);
                                if (continuous && RagdollCore::tuningText("OPENMW_RAGDOLL_LIMB_POSE_GRAPH"))
                                {
                                    std::vector<DiagnosticLimbRoute> next;
                                    std::size_t edgesTested = 0;
                                    if (!diagnosticRouteBlocked)
                                    {
                                        const auto candidates = enumerateClearLimbPoses(query, support, 1.0f, 12.0f);
                                        // Cheapest path per current sample. Retain ALL reachable
                                        // samples: no beam cutoff or nearest-pose commitment.
                                        for (const auto& candidate : candidates)
                                        {
                                            DiagnosticLimbRoute route;
                                            bool reachable = diagnosticLimbRoutes.empty();
                                            for (const auto& previous : diagnosticLimbRoutes)
                                            {
                                                LimbPoseContinuity edge;
                                                edge.mPrevious = previous.mPath.back();
                                                edge.mPreviousParentRotation = diagnosticLimbParent;
                                                edge.mPreviousSupport = diagnosticLimbSupport;
                                                edge.mMaximumFootTravel = 6.0f;
                                                edge.mMaximumJointRotation = 0.35f;
                                                edge.mIntervals = 8;
                                                ++edgesTested;
                                                if (!isLimbTransitionClear(query, candidate, support, 1.0f, edge)) continue;
                                                route = previous;
                                                reachable = true;
                                                break; // Previous routes are sorted by cumulative cost.
                                            }
                                            if (!reachable) continue;
                                            route.mPath.push_back(candidate);
                                            route.mCost += candidate.mFootDisplacement * candidate.mFootDisplacement;
                                            next.push_back(std::move(route));
                                        }
                                    }
                                    std::stable_sort(next.begin(), next.end(), [](const auto& a, const auto& b) {
                                        return a.mCost < b.mCost;
                                    });
                                    diagnosticLimbRoutes = std::move(next);
                                    diagnosticRouteBlocked = diagnosticLimbRoutes.empty();
                                    std::cerr << "RAGDOLL_LIMB_POSE_GRAPH step=" << step
                                        << " reachable=" << diagnosticLimbRoutes.size() << " edges_tested=" << edgesTested;
                                    if (!diagnosticLimbRoutes.empty())
                                    {
                                        const auto& best = diagnosticLimbRoutes.front();
                                        std::cerr << " length=" << best.mPath.size() << " cost=" << best.mCost;
                                        for (std::size_t i = 0; i < best.mPath.size(); ++i)
                                        {
                                            const auto& point = best.mPath[i].mPoses[2].mPosition;
                                            std::cerr << " foot" << i << '=' << point.GetX() << ',' << point.GetY() << ',' << point.GetZ();
                                        }
                                    }
                                    std::cerr << '\n';
                                    if ((step == 591 || step == 596) && !diagnosticLimbRoutes.empty())
                                    {
                                        const auto& path = diagnosticLimbRoutes.front().mPath;
                                        for (std::size_t i = 0; i < path.size(); ++i)
                                        {
                                            const auto& candidate = path[i];
                                            if (i + 1 < path.size())
                                            {
                                                const auto rates = calculateLimbPoseRates(candidate, path[i + 1], settings.mFixedTimeStep);
                                                if (!rates) { fail("Limb route rate query rejected"); return result; }
                                                std::cerr << std::setprecision(9) << "RAGDOLL_LIMB_ROUTE_RATE end=" << step
                                                    << " step=" << step + 1 - path.size() + i
                                                    << " hip_world=" << rates->mHipWorld.GetX() << ','
                                                    << rates->mHipWorld.GetY() << ',' << rates->mHipWorld.GetZ()
                                                    << " knee=" << rates->mKnee
                                                    << " ankle_parent=" << rates->mAnkleParent.GetX() << ','
                                                    << rates->mAnkleParent.GetY() << ',' << rates->mAnkleParent.GetZ() << '\n';
                                            }
                                            std::cerr << std::setprecision(9) << "RAGDOLL_LIMB_ROUTE end=" << step
                                                << " step=" << step + 1 - path.size() + i << " swing_foot=" << swing.mFoot
                                                << " knee=" << candidate.mKneeAngleFromReference
                                                << " deviation=" << candidate.mFootDisplacement;
                                            for (std::size_t part = 0; part < 3; ++part)
                                            {
                                                const auto& shapePose = candidate.mPoses[part];
                                                const auto& q = shapePose.mRotation;
                                                const auto& p = shapePose.mPosition;
                                                std::cerr << " q" << part << '=' << q.GetX() << ',' << q.GetY()
                                                    << ',' << q.GetZ() << ',' << q.GetW()
                                                    << " p" << part << '=' << p.GetX() << ',' << p.GetY() << ',' << p.GetZ();
                                            }
                                            std::cerr << '\n';
                                        }
                                    }
                                }
                                if (continuous)
                                {
                                    diagnosticLimbBlocked = diagnosticLimbBlocked || !choice;
                                    diagnosticLimbPose = choice;
                                    diagnosticLimbSupport = support;
                                    diagnosticLimbParent = query.mParentRotation;
                                    diagnosticLimbStep = step;
                                    diagnosticLimbFoot = swing.mFoot;
                                }
                                std::cerr << std::setprecision(9) << "RAGDOLL_LIMB_POSE_SEARCH step=" << step
                                    << " found=" << choice.has_value()
                                    << " continuous=" << continuous << " seeded=" << seeded
                                    << " blocked=" << (continuous && diagnosticLimbBlocked);
                                if (choice)
                                {
                                    const auto recheck = measureLimbClearance(choice->mPoses, support, 1.0f);
                                    std::cerr << " clear=" << (recheck && recheck->mClear)
                                        << " displacement=" << choice->mFootDisplacement
                                        << " evaluated=" << choice->mEvaluatedCandidates
                                        << " foot=" << choice->mPoses[2].mPosition.GetX() << ','
                                        << choice->mPoses[2].mPosition.GetY() << ',' << choice->mPoses[2].mPosition.GetZ();
                                }
                                std::cerr << '\n';
                            }
                        }
                        for (std::size_t i = 0; i < 3; ++i)
                            for (std::size_t j = 0; j < 3; ++j)
                                for (const bool goal : { false, true })
                                {
                                    const auto& pose = goal ? commanded[i] : actual[i];
                                    const auto clearance = measureLimbClearance(
                                        std::span(&pose, 1), std::span(&support[j], 1), 1.0f);
                                    std::cerr << std::setprecision(9) << "RAGDOLL_LIMB_CLEARANCE step=" << step
                                        << " mode=" << (goal ? "command" : "actual")
                                        << " first=" << swingIds[i] << " second=" << stanceIds[j]
                                        << " valid=" << clearance.has_value();
                                    if (clearance)
                                    {
                                        std::cerr << " clear=" << clearance->mClear << " gap=";
                                        if (clearance->mMinimumReportedGap) std::cerr << *clearance->mMinimumReportedGap;
                                        else std::cerr << "outside_margin";
                                    }
                                    std::cerr << '\n';
                                }
                    }
                    else std::cerr << "RAGDOLL_LIMB_CLEARANCE step=" << step << " valid=0 reason=missing_target\n";
                }
                if (!settings.mReplayBodyIds.empty() && leftLeg && rightLeg
                    && RagdollCore::tuningText("OPENMW_RAGDOLL_INERTIA_TRACE"))
                    for (const auto* leg : { &*leftLeg, &*rightLeg })
                    {
                        const auto geometry = ragdoll->getSwingTwistJointGeometry(leg->mHip);
                        const auto joint = std::find_if(telemetry.mConstraints.begin(), telemetry.mConstraints.end(),
                            [&](const auto& value) { return value.mChildBodyIndex == leg->mHip; });
                        if (!geometry || joint == telemetry.mConstraints.end()) continue;
                        const auto parentInverse = bodies.GetInverseInertia(ragdoll->getBodyId(joint->mParentBodyIndex));
                        const auto childInverse = bodies.GetInverseInertia(ragdoll->getBodyId(leg->mHip));
                        const auto motorFrame = telemetry.mBodies[leg->mHip].mRotation * geometry->mChildFrame;
                        for (int axisIndex = 0; axisIndex < 3; ++axisIndex)
                        {
                            const auto axis = motorFrame * (axisIndex == 0 ? JPH::Vec3::sAxisX()
                                : axisIndex == 1 ? JPH::Vec3::sAxisY() : JPH::Vec3::sAxisZ());
                            const float inversePair = axis.Dot(parentInverse.Multiply3x3(axis)
                                + childInverse.Multiply3x3(axis));
                            double lockedLeg = 0;
                            for (const auto body : { leg->mHip, leg->mKnee, leg->mFoot })
                            {
                                const auto inverse = bodies.GetInverseInertia(ragdoll->getBodyId(body));
                                JPH::Mat44 inertia;
                                if (!inertia.SetInversed3x3(inverse))
                                { fail("Inertia observation singular"); return result; }
                                const auto& state = telemetry.mBodies[body];
                                const JPH::Vec3 radius(state.mCenterOfMass - joint->mPosition);
                                lockedLeg += axis.Dot(inertia.Multiply3x3(axis))
                                    + state.mMass * (radius.LengthSq() - std::pow(radius.Dot(axis), 2));
                            }
                            if (!(inversePair > 0) || !std::isfinite(lockedLeg))
                            { fail("Inertia observation invalid"); return result; }
                            std::cerr << std::setprecision(9)
                                      << "RAGDOLL_INERTIA schema=1 phase=pre_solve step=" << step
                                      << " hip=" << leg->mHip << " axis=" << axisIndex
                                      << " pair_inertia=" << 1.0f / inversePair
                                      << " locked_leg_inertia=" << lockedLeg
                                      << " spring_value=" << targets[leg->mHip].mSpring.mFrequencyOrStiffness
                                      << " stiffness_mode=" << targets[leg->mHip].mSpring.mStiffnessMode << '\n';
                        }
                    }
            }
            const auto updateError = physics.Update(
                settings.mFixedTimeStep, static_cast<int>(settings.mCollisionSteps), &allocator, &jobs);
            if (updateError != JPH::EPhysicsUpdateError::None)
            {
                fail("Jolt reported a physics update error at step " + std::to_string(step) + ".");
                return result;
            }

            if (observingSolver)
            {
                RagdollSolverImpulse impulse;
                if (!ragdoll->endSolverImpulseMeasurement(settings.mFixedTimeStep, impulse))
                {
                    fail("Solver impulse observation failed at step " + std::to_string(step) + ".");
                    return result;
                }
                frame.mSolverImpulse = impulse;
            }

            if (frame.mMotorsApplied && gaitOutput && gaitOutput->hasStance())
            {
                const LegChain& stanceLeg = gaitOutput->leftFootIsStance() ? *leftLeg : *rightLeg;
                const std::array<std::size_t, 3> stanceJoints{ stanceLeg.mHip, stanceLeg.mKnee, stanceLeg.mFoot };
                for (std::size_t joint = 0; joint < stanceJoints.size(); ++joint)
                {
                    if (!ragdoll->getMotorAngularImpulse(stanceJoints[joint], settings.mFixedTimeStep,
                            frame.mStanceLegMotorAngularImpulses[joint], frame.mStanceLegMotorPeakAxisUtilizations[joint]))
                    {
                        fail("Stance-leg motor impulse could not be read at step " + std::to_string(step) + ".");
                        return result;
                    }
                }
                const LegChain& swingLeg = gaitOutput->leftFootIsStance() ? *rightLeg : *leftLeg;
                frame.mSwingHipMotorBody = swingLeg.mHip;
                if (!ragdoll->getMotorAngularImpulse(swingLeg.mHip, settings.mFixedTimeStep,
                        frame.mSwingHipMotorAngularImpulse, frame.mSwingHipMotorPeakAxisUtilization))
                {
                    fail("Swing-hip motor impulse could not be read at step " + std::to_string(step) + ".");
                    return result;
                }
                if (!ragdoll->getMotorAngularImpulse(swingLeg.mKnee, settings.mFixedTimeStep,
                        frame.mSwingKneeMotorAngularImpulse, frame.mSwingKneeMotorPeakAxisUtilization)
                    || !ragdoll->getMotorAngularImpulse(swingLeg.mFoot, settings.mFixedTimeStep,
                        frame.mSwingFootJointMotorAngularImpulse,
                        frame.mSwingFootJointMotorPeakAxisUtilization))
                {
                    fail("Swing-leg motor impulse could not be read at step " + std::to_string(step) + ".");
                    return result;
                }
                if (settings.mCompensateSwingHipMotorImpulse)
                {
                    const float axes = profile.mBodies[swingLeg.mHip].mConstraint->mType == ConstraintType::SwingTwist
                        ? std::sqrt(3.0f)
                        : 1.0f;
                    const float maximumImpulse = profile.mBodies[swingLeg.mHip].mConstraint->mMotor.mMaximumTorque
                        * profile.mActiveController.mGait.mSwingLegMotorTorqueScale * axes
                        * settings.mFixedTimeStep;
                    frame.mAppliedStanceHipCompensationImpulse = -frame.mSwingHipMotorAngularImpulse;
                    if (!ragdoll->applyBodyAngularImpulsePair(
                            stanceLeg.mHip, *pelvisIndex, frame.mAppliedStanceHipCompensationImpulse, maximumImpulse))
                    {
                        fail("Stance-hip motor compensation was rejected at step " + std::to_string(step) + ".");
                        return result;
                    }
                }
            }

            if ((frame.mMotorsApplied && frame.mJointSupport) || (directMotorTracking && directMotorAxesCaptured))
            {
                frame.mJointSupportMotorAngularImpulses.assign(bodyCount, JPH::Vec3::sZero());
                frame.mJointSupportMotorUtilizations.assign(bodyCount, 0.0f);
                for (std::size_t i = 1; i < bodyCount; ++i)
                    if (!ragdoll->getMotorAngularImpulse(i, settings.mFixedTimeStep,
                            frame.mJointSupportMotorAngularImpulses[i], frame.mJointSupportMotorUtilizations[i]))
                    {
                        fail("Joint motor impulse unavailable at step " + std::to_string(step));
                        return result;
                    }
            }
            if (frame.mReferenceGait && frame.mJointSupportMotorAngularImpulses.size() == bodyCount)
            {
                frame.mReferenceGait->mStanceHipMotorTorque
                    = frame.mJointSupportMotorAngularImpulses[frame.mReferenceGait->mStanceHipBody] / settings.mFixedTimeStep;
                frame.mReferenceGait->mStanceAnkleMotorTorque
                    = frame.mJointSupportMotorAngularImpulses[frame.mReferenceGait->mStanceAnkleBody] / settings.mFixedTimeStep;
                for (std::size_t i = 0; i < bodyCount; ++i)
                {
                    referenceMotorTorques[i] = frame.mJointSupportMotorAngularImpulses[i] / settings.mFixedTimeStep;
                    if (frame.mJointSupportMotorUtilizations[i] > frame.mReferenceGait->mMaxMotorUtilization)
                    {
                        frame.mReferenceGait->mMaxMotorUtilization = frame.mJointSupportMotorUtilizations[i];
                        frame.mReferenceGait->mMaxMotorUtilizationBody = i;
                    }
                }
                referenceHistory = true;
            }
            if (!settings.mReplayReferenceFrames.empty())
            {
                auto reference = settings.mReplayReferenceFrames[step];
                if (settings.mReplayComputeHorizontalRoot && controllerOutput)
                {
                    const auto mode = controllerOutput->mMode;
                    const bool ownsRoot = (controllerOutput->mDriveJointMotors
                            && !active.mRequestedHorizontalVelocity.IsNearZero())
                        || mode == ActiveControllerMode::Falling || mode == ActiveControllerMode::Fallen
                        || mode == ActiveControllerMode::GettingUp || mode == ActiveControllerMode::BlendBack;
                    if (ownsRoot)
                    {
                        RagdollTelemetry solved;
                        const auto target = pelvisIndex ? ragdoll->getWorldBodyPosition(motorTarget, *pelvisIndex)
                                                        : std::nullopt;
                        if (!target || !ragdoll->captureTelemetry(solved) || *pelvisIndex >= solved.mBodies.size())
                        { fail("Replay horizontal root observation failed"); return result; }
                        const auto displacement = solved.mBodies[*pelvisIndex].mPosition - *target;
                        const auto oldRoot = ragdoll->getReferencePosition();
                        reference.mPosition.SetX(oldRoot.GetX() + displacement.GetX());
                        reference.mPosition.SetY(oldRoot.GetY() + displacement.GetY());
                    }
                }
                if (referenceSwingHistory)
                {
                    const auto translation = ragdoll->rebaseModelPosition(JPH::Vec3::sZero(), reference.mPosition, reference.mRotation);
                    if (!translation || !rebaseSwingTrajectoryHistory(referenceSwingHistory,
                            (reference.mRotation.Conjugated() * currentRootRotation).Normalized(), *translation))
                    { fail("Replay swing history rebase failed"); return result; }
                }
                for (auto* position : { &swingStartModel, &referenceSwingStartModel, &referenceLandingFootModel })
                    if (*position)
                    {
                        const auto rebased = ragdoll->rebaseModelPosition(**position, reference.mPosition, reference.mRotation);
                        if (!rebased) { fail("Replay cached point rebase failed"); return result; }
                        *position = rebased;
                    }
                if (!ragdoll->setReferenceTransform(reference.mPosition, reference.mRotation))
                { fail("Replay reference transform rejected"); return result; }
                currentRootPosition = reference.mPosition;
                currentRootRotation = reference.mRotation;
            }
            frame.mContacts = listener.takeSortedContacts(physics, settings.mReplayPreserveContactOrder);
            if (!ragdoll->captureTelemetry(telemetry) || telemetry.mBodies.size() != bodyCount
                || !ragdoll->captureModelPose(pose) || pose.mBodyTransforms.size() != bodyCount)
            {
                fail("Ragdoll telemetry could not be captured at step " + std::to_string(step) + ".");
                return result;
            }

            for (const auto& joint : telemetry.mConstraints)
                frame.mMaxAnchorSeparation = std::max(frame.mMaxAnchorSeparation, joint.mAnchorSeparation);
            // Optional replay observation of actual joint feasibility, distinct
            // from clamping a requested motor target before the solve.
            if (!settings.mReplayBodyIds.empty() && step < 1800
                && RagdollCore::tuningText("OPENMW_RAGDOLL_JOINT_LIMIT_TRACE"))
                for (const auto& joint : telemetry.mConstraints)
                {
                    const auto geometry = ragdoll->getSwingTwistJointGeometry(joint.mChildBodyIndex);
                    if (!geometry) continue;
                    const auto current = telemetry.mBodies[joint.mChildBodyIndex].mRotation;
                    const auto clamped = clampSwingTwistJointOrientation(*geometry,
                        telemetry.mBodies[joint.mParentBodyIndex].mRotation, current);
                    if (!clamped) { fail("Joint limit observation rejected"); return result; }
                    const auto delta = (*clamped * current.Conjugated()).Normalized();
                    const float excess = 2 * std::atan2(delta.GetXYZ().Length(), std::abs(delta.GetW()));
                    std::cerr << std::setprecision(9) << "RAGDOLL_JOINT_LIMIT schema=1 phase=post_solve step="
                              << step << " body=" << joint.mChildBodyIndex << " parent=" << joint.mParentBodyIndex
                              << " current_excess_rad=" << excess << '\n';
                }

            const std::optional<SupportEstimate> estimate
                = estimateFootSupport(telemetry.mCenterOfMass, frame.mContacts, feet, profile.mActiveController);
            if (!estimate)
            {
                fail("Support estimation rejected the observed contacts at step " + std::to_string(step) + ".");
                return result;
            }
            if (estimate->hasGroundContact() || ragdoll->isActive() || !supportEstimate
                || !supportEstimate->hasGroundContact())
                supportEstimate = *estimate;

            JPH::Vec3 centerOfMassVelocity = JPH::Vec3::sZero();
            for (const BodyTelemetry& body : telemetry.mBodies)
                centerOfMassVelocity += body.mMotion.mLinearVelocity * body.mMass;
            centerOfMassVelocity /= telemetry.mTotalMass;

            std::optional<SupportEstimate> stanceSupportEstimate;
            // Floating-base momentum/solver closure is undefined for an
            // externally driven root. Leave it unavailable, never zero-filled.
            if (!settings.mPinPelvis && !ragdoll->isRootKinematic())
            {
                RagdollMomentum momentum;
                if (!ragdoll->captureMomentum(momentum))
                {
                    fail("Centroidal momentum could not be captured at step " + std::to_string(step) + ".");
                    return result;
                }
                frame.mMomentum = momentum;
            }
            JPH::Vec3 observationRequest = active.mRequestedHorizontalVelocity;
            if (settings.mReplayObserveRestartRequest
                && useReferenceGait && !settings.mReferenceInputsAreFiltered
                && observationRequest.IsNearZero() && keyframeCursor < script.mKeyframes.size()
                && script.mKeyframes[keyframeCursor].mStep == step + 1
                && !script.mKeyframes[keyframeCursor].mRequestedHorizontalVelocity.IsNearZero())
            {
                auto observedSteering = referenceSteering;
                const bool planted = !referenceState.hasStance()
                    || (referenceState.mStage == ReferenceGaitStage::DoubleSupport
                        && supportEstimate->mLeftFootSupported && supportEstimate->mRightFootSupported);
                const bool ready = planted && (!referenceState.hasStance()
                    || horizontal(centerOfMassVelocity).Length() <= 2 * settings.mReferenceGait.mSteeringStopSpeed);
                observationRequest = observedSteering.step(
                    script.mKeyframes[keyframeCursor].mRequestedHorizontalVelocity,
                    settings.mFixedTimeStep, settings.mReferenceGait, ready, planted,
                    settings.mReferenceGaitPlayerInput, centerOfMassVelocity);
            }
            std::optional<JPH::RVec3> backwardRecoveryTarget;
            const bool referenceObservation
                = useReferenceGait && !horizontal(observationRequest).IsNearZero();
            if (!referenceObservation && gaitOutput && gaitOutput->hasStance())
            {
                const std::size_t stanceFoot = gaitOutput->leftFootIsStance() ? *leftFootIndex : *rightFootIndex;
                JPH::RVec3 stancePosition = telemetry.mBodies[stanceFoot].mPosition;
                stancePosition.SetZ(supportEstimate->mGroundCenter.GetZ());
                const auto stance = estimateGaitStanceSupport(gaitOutput->mPhase, telemetry.mCenterOfMass,
                    centerOfMassVelocity, observationRequest, supportEstimate->mCenterOfMassHeight,
                    settings.mGravity, frame.mContacts, feet, profile.mActiveController,
                    settings.mUseCapturePointForStanceReadiness,
                    settings.mEnableBackwardCaptureRecovery
                        ? std::optional<GaitCaptureStepInput>(GaitCaptureStepInput{ stancePosition, gaitOutput->mTimeInPhase })
                        : std::nullopt);
                if (!stance)
                {
                    fail("The shared stance observation was rejected at step " + std::to_string(step) + ".");
                    return result;
                }
                stanceSupportEstimate = stance->mSupport;
                frame.mStanceSupportsCenterOfMass = stance->mReady;
                backwardRecoveryTarget = stance->mBackwardRecoveryTarget;
            }

            const JPH::Vec3 locomotionRequest = horizontal(observationRequest);
            ActiveControllerInput input;
            input.mSupport = *supportEstimate;
            input.mRecoveryRequested = active.mRecoveryRequested || replayRecoveryRequested;
            input.mRecoveryAnimationFinished = active.mRecoveryAnimationFinished;
            input.mRootExternallyOwned = ragdoll->isRootKinematic();
            input.mRecoveryInterrupted = active.mRecoveryInterrupted
                || (active.mKinematicRecovery && recoveryMode && recoveryImpact);
            input.mPelvisUpDot = std::clamp(
                (telemetry.mBodies[*pelvisIndex].mRotation * pelvisLocalUp).Dot(JPH::Vec3::sAxisZ()), -1.0f, 1.0f);
            input.mTorsoUpDot = std::clamp(
                (telemetry.mBodies[*torsoIndex].mRotation * torsoLocalUp).Dot(JPH::Vec3::sAxisZ()), -1.0f, 1.0f);
            input.mTorsoFacingUpDot = std::clamp(
                (telemetry.mBodies[*torsoIndex].mRotation * torsoLocalForward).GetZ(), -1.0f, 1.0f);
            if (input.mSupport.hasGroundContact())
            {
                input.mPelvisHeight = std::max(0.0f,
                    static_cast<float>(static_cast<double>(telemetry.mBodies[*pelvisIndex].mCenterOfMass.GetZ())
                        - static_cast<double>(input.mSupport.mGroundCenter.GetZ())));
            }
            if (replayRecoveryHandoverPending)
            {
                referenceHandover.begin(pose, input.mPelvisHeight,
                    telemetry.mCenterOfMass, settings.mReferenceGait);
                replayRecoveryHandoverPending = false;
            }
            input.mCenterOfMassVelocity = centerOfMassVelocity;
            input.mTargetHorizontalVelocity = JPH::Vec3::sZero();

            if (ragdoll->isActive())
            {
                uprightInput.reset();
                uprightOutput.reset();
            }
            const bool recovering = controllerOutput
                && (controllerOutput->mMode == ActiveControllerMode::GettingUp
                    || controllerOutput->mMode == ActiveControllerMode::BlendBack);
            const bool locomotionRequested = !recovering && !locomotionRequest.IsNearZero();
            SettledPelvisHeightInput heightInput;
            heightInput.mSupport = input.mSupport;
            heightInput.mMode = controllerOutput ? controllerOutput->mMode : ActiveControllerMode::Standing;
            heightInput.mPelvisHeight = input.mPelvisHeight;
            heightInput.mPelvisVerticalVelocity
                = telemetry.mBodies[*pelvisIndex].mMotion.mLinearVelocity.Dot(JPH::Vec3::sAxisZ());
            heightInput.mLocomotionRequested = locomotionRequested;
            if (!pelvisHeightTracker.step(
                    heightInput, settings.mFixedTimeStep, profile.mActiveController, pelvisHeightOutput))
            {
                fail("The settled pelvis-height tracker rejected its observation at step "
                    + std::to_string(step) + ".");
                return result;
            }
            const std::optional<float>& targetPelvisHeight = pelvisHeightOutput.mTargetHeight;
            if (targetPelvisHeight)
                result.mTargetPelvisHeight = *targetPelvisHeight;
            const bool locomoting = locomotionRequested && pelvisHeightOutput.mSettled;
            const JPH::Vec3 requested = locomoting ? locomotionRequest : JPH::Vec3::sZero();
            input.mTargetHorizontalVelocity = requested;
            const bool preservingGaitOrientation
                = locomoting && controllerOutput && controllerOutput->mDriveJointMotors;
            // Native selects motor eligibility from the observed controller mode before
            // controller.step advances it. Keep that decision for the next
            // solve, including the first standing solve after BlendBack.
            const bool authoredRecoveryHeight = recovering
                && controllerOutput->mMode == ActiveControllerMode::GettingUp
                && active.mRecoveryPelvisHeightScale.has_value();
            const bool correctTranslation
                = input.mSupport.hasGroundContact() && (!recovering || authoredRecoveryHeight || controllerOutput->mRecoveryHeightAssist
                    || controllerOutput->mMode == ActiveControllerMode::BlendBack);
            GaitActuationDecisionInput controlDecisionInput;
            controlDecisionInput.mLocomotionRequested = locomotionRequested;
            controlDecisionInput.mPelvisHeightSettled = pelvisHeightOutput.mSettled;
            controlDecisionInput.mDriveJointMotors
                = controllerOutput && controllerOutput->mDriveJointMotors;
            controlDecisionInput.mRecovering = recovering;
            controlDecisionInput.mCorrectTranslation = correctTranslation;
            controlDecisionInput.mBothFeetSupported
                = input.mSupport.mLeftFootSupported && input.mSupport.mRightFootSupported;
            controlDecisionInput.mRequestedHorizontalVelocity = locomotionRequest;
            controlDecisionInput.mMeasuredHorizontalVelocity = centerOfMassVelocity;
            if (gaitOutput)
                controlDecisionInput.mGait = *gaitOutput;
            GaitControllerProfile controlProfile = profile.mActiveController.mGait;
            controlProfile.mPreLiftSwingUnloadFraction = preLiftSwingUnloadFraction;
            const auto controlDecision = calculateGaitActuationDecision(controlDecisionInput, controlProfile);
            if (!controlDecision)
            {
                fail("The gait horizontal-control decision was rejected at step " + std::to_string(step) + ".");
                return result;
            }
            observedSupportMotorEligible = controlDecisionInput.mDriveJointMotors && !recovering;
            // Native observes upright control even while the reference route
            // owns actuation. Keep that observation ready for recovery takeover.
            if (!directMotorTracking && targetPelvisHeight
                && (input.mSupport.hasSupport() || recovering || (locomoting && input.mSupport.hasGroundContact())
                    || preservingGaitOrientation))
            {
                UprightControllerInput upright;
                // Match native settling: stop lift-only unweighting and use
                // measured support-relative COM and height feedback.
                upright.mRecoveryUnweight = recovering && !authoredRecoveryHeight
                    && controllerOutput->mMode == ActiveControllerMode::GettingUp
                    && input.mSupport.hasGroundContact()
                    && (!controllerOutput->mRecoveryHeightAssist
                        || input.mPelvisUpDot < profile.mActiveController.mStandingMinimumUpDot);
                upright.mTargetPelvisHeight = correctTranslation ? *targetPelvisHeight
                    : input.mSupport.hasGroundContact()          ? input.mPelvisHeight
                                                                 : *targetPelvisHeight;
                if (authoredRecoveryHeight)
                    upright.mTargetPelvisHeight = *targetPelvisHeight * *active.mRecoveryPelvisHeightScale;
                if (correctTranslation && referenceHandover.active())
                    upright.mTargetPelvisHeight
                        = referenceHandover.blendPelvisHeight(*targetPelvisHeight, settings.mReferenceGait);
                upright.mPelvisHeight
                    = input.mSupport.hasGroundContact() ? input.mPelvisHeight : *targetPelvisHeight;
                if (correctTranslation)
                {
                    upright.mPelvisVerticalVelocity
                        = telemetry.mBodies[*pelvisIndex].mMotion.mLinearVelocity.Dot(JPH::Vec3::sAxisZ());
                }
                if (correctTranslation && input.mSupport.hasSupport())
                {
                    const JPH::Vec3 defaultOffset(telemetry.mCenterOfMass - input.mSupport.mCenter);
                    std::optional<JPH::Vec3> stanceOffset;
                    if (stanceSupportEstimate && stanceSupportEstimate->hasSupport())
                    {
                        const JPH::RVec3 target = stanceSupportEstimate->mClosestPoint
                            + 0.5 * (stanceSupportEstimate->mCenter - stanceSupportEstimate->mClosestPoint);
                        stanceOffset = JPH::Vec3(telemetry.mCenterOfMass - target);
                        stanceOffset->SetZ(0.0f);
                    }
                    const auto selectedOffset
                        = selectGaitCenterOfMassOffset(*controlDecision, requested, defaultOffset, stanceOffset);
                    if (!selectedOffset)
                    {
                        fail("The gait center-of-mass policy was rejected at step " + std::to_string(step) + ".");
                        return result;
                    }
                    upright.mCenterOfMassOffset = *selectedOffset;
                }
                if (correctTranslation)
                    upright.mCenterOfMassVelocity = centerOfMassVelocity;
                // Native gait input: the requested velocity is only tracked
                // while translation is being corrected from measured support.
                upright.mTargetHorizontalVelocity = controlDecision->mTargetHorizontalVelocity;
                upright.mPelvisUp = telemetry.mBodies[*pelvisIndex].mRotation * pelvisLocalUp;
                upright.mPelvisAngularVelocity = telemetry.mBodies[*pelvisIndex].mMotion.mAngularVelocity;
                upright.mTorsoUp = telemetry.mBodies[*torsoIndex].mRotation * torsoLocalUp;
                upright.mTorsoAngularVelocity = telemetry.mBodies[*torsoIndex].mMotion.mAngularVelocity;
                // Match native physical settling: only the active get-up
                // follows the target pose; BlendBack balances against world up.
                if (controllerOutput && controllerOutput->mMode == ActiveControllerMode::GettingUp)
                {
                    if (const auto targetPelvisRotation = ragdoll->getWorldBodyRotation(motorTarget, *pelvisIndex))
                        upright.mTargetUp = *targetPelvisRotation * pelvisLocalUp;
                }
                upright.mTotalMass = telemetry.mTotalMass;
                // Contact budgets are selected by the shared gait policy.
                upright.mSupportFriction = input.mSupport.mFriction.value_or(0);
                upright.mGravityMagnitude = settings.mGravity;
                upright.mCenterOfMassPosition = telemetry.mCenterOfMass;
                upright.mCenterOfMassHeight = input.mSupport.mCenterOfMassHeight;
                configureGaitSupportBudget(*controlDecision, correctTranslation, frame.mContacts, feet, upright);
                if (const auto correction = calculateUprightControl(upright, profile.mActiveController))
                {
                    uprightInput = upright;
                    uprightOutput = *correction;
                    uprightStage = controlDecision->mStage;
                }
            }

            ActiveControllerOutput output;
            if (!controller.step(input, settings.mFixedTimeStep, profile.mActiveController, output))
            {
                fail("The deterministic controller rejected its observation at step " + std::to_string(step) + ".");
                return result;
            }
            if (controllerOutput && controllerOutput->mMode == ActiveControllerMode::BlendBack
                && (output.mMode == ActiveControllerMode::Fallen || output.mMode == ActiveControllerMode::Falling))
            {
                active.mRecoveryRequested = false;
                active.mRecoveryAnimationFinished = false;
            }
            // The recorded native adapter finishes recovery between solves,
            // retaining measured legs before its next observed handover begins.
            if (!settings.mReplayRecoveryFinishEventsCaptured
                && !settings.mReplayBodyIds.empty() && controllerOutput
                && controllerOutput->mMode == ActiveControllerMode::BlendBack
                && output.mMode == ActiveControllerMode::Standing)
                replayRecoveryPosePending = true;
            controllerOutput = output;

            GaitControllerInput gaitInput;
            gaitInput.mEnabled = !directMotorTracking && !ragdoll->isRootKinematic()
                && locomoting && output.mDriveJointMotors;
            gaitInput.mLeftFootSupported = input.mSupport.mLeftFootSupported;
            gaitInput.mRightFootSupported = input.mSupport.mRightFootSupported;
            gaitInput.mStanceSupportsCenterOfMass = frame.mStanceSupportsCenterOfMass;
            gaitInput.mLeftFootHeight = static_cast<float>(telemetry.mBodies[*leftFootIndex].mCenterOfMass.GetZ());
            gaitInput.mRightFootHeight = static_cast<float>(telemetry.mBodies[*rightFootIndex].mCenterOfMass.GetZ());
            gaitInput.mLeftFootVerticalVelocity = telemetry.mBodies[*leftFootIndex].mMotion.mLinearVelocity.GetZ();
            gaitInput.mRightFootVerticalVelocity = telemetry.mBodies[*rightFootIndex].mMotion.mLinearVelocity.GetZ();
            if (gaitOutput && gaitOutput->hasStance() && swingFootTarget)
            {
                const std::size_t swingFoot = gaitOutput->leftFootIsStance() ? *rightFootIndex : *leftFootIndex;
                const std::size_t stanceFoot = gaitOutput->leftFootIsStance() ? *leftFootIndex : *rightFootIndex;
                // The latched target represents the body origin from the
                // model pose. Do not fold an authored shape COM offset into
                // the touchdown error.
                JPH::Vec3 placementError = JPH::Vec3(telemetry.mBodies[swingFoot].mPosition - *swingFootTarget);
                placementError.SetZ(0.0f);
                gaitInput.mSwingFootHorizontalPlacementError = placementError.Length();
                gaitInput.mSwingFootVerticalPlacementError = std::abs(static_cast<float>(
                    telemetry.mBodies[swingFoot].mPosition.GetZ()
                    - telemetry.mBodies[stanceFoot].mPosition.GetZ()));
                gaitInput.mSwingFootRelativeVerticalSpeed = std::abs(
                    telemetry.mBodies[swingFoot].mMotion.mLinearVelocity.GetZ()
                    - telemetry.mBodies[stanceFoot].mMotion.mLinearVelocity.GetZ());
            }
            GaitControllerOutput gaitStep;
            if (referenceObservation)
            {
                ReferenceGaitStateInput referenceInput;
                referenceInput.mEnabled = gaitInput.mEnabled && active.mMotorsEnabled
                    && output.mMode != ActiveControllerMode::GettingUp
                    && output.mMode != ActiveControllerMode::BlendBack;
                referenceInput.mLeftFootCenter = telemetry.mBodies[*leftFootIndex].mCenterOfMass;
                referenceInput.mRightFootCenter = telemetry.mBodies[*rightFootIndex].mCenterOfMass;
                referenceInput.mLeftFootSupported = gaitInput.mLeftFootSupported;
                referenceInput.mRightFootSupported = gaitInput.mRightFootSupported;
                referenceInput.mLeftFootHeight = gaitInput.mLeftFootHeight;
                referenceInput.mRightFootHeight = gaitInput.mRightFootHeight;
                referenceInput.mLeftFootVerticalVelocity = gaitInput.mLeftFootVerticalVelocity;
                referenceInput.mRightFootVerticalVelocity = gaitInput.mRightFootVerticalVelocity;
                const auto loadFraction = [&](std::size_t foot) {
                    double total = 0.0;
                    for (const GroundContact& contact : frame.mContacts)
                    {
                        if (contact.mBodyIndex != foot || !contact.hasSupport())
                            continue;
                        if (!contact.mNormalImpulse || !std::isfinite(*contact.mNormalImpulse)
                            || *contact.mNormalImpulse < 0.0f)
                            return -1.0f;
                        total += *contact.mNormalImpulse;
                    }
                    const double reference = static_cast<double>(telemetry.mTotalMass) * settings.mGravity
                        * settings.mFixedTimeStep;
                    return reference > 0.0 ? static_cast<float>(total / reference) : -1.0f;
                };
                referenceInput.mLeftFootLoadFraction = loadFraction(*leftFootIndex);
                referenceInput.mRightFootLoadFraction = loadFraction(*rightFootIndex);
                referenceInput.mCenterOfMass = telemetry.mCenterOfMass;
                referenceInput.mCenterOfMassVelocity = centerOfMassVelocity;
                referenceInput.mCenterOfMassHeight = input.mSupport.mCenterOfMassHeight;
                referenceInput.mGravity = settings.mGravity;
                referenceInput.mHeading = observationRequest;
                // i39 (delivery-comp-2026-09-11.txt): feedback for the
                // delivery-error observer. Unconditional bookkeeping; the
                // observer itself is env-gated inside the state machine,
                // so with the gate unset this changes nothing.
                referenceInput.mPlacementPredictedVelocity = referencePlacementPredicted;
                steeringObservedGaitState = referenceState;
                if (!referenceGait.step(referenceInput, settings.mFixedTimeStep, settings.mReferenceGait, referenceState))
                {
                    fail("The reference gait state machine rejected its observation at step " + std::to_string(step) + ".");
                    return result;
                }
                gaitStep = toGaitControllerOutput(referenceState, settings.mReferenceGait);
                gaitOutput = gaitStep;
                if (referenceState.hasStance() && referenceState.mStage == ReferenceGaitStage::Swing)
                {
                    const std::size_t swingFoot = referenceState.leftFootIsStance() ? *rightFootIndex : *leftFootIndex;
                    if (!referenceSwingStartModel)
                        referenceSwingStartModel = pose.mBodyTransforms[swingFoot].mTranslation;
                }
                else
                {
                    referenceSwingStartModel.reset();
                    referenceSwingHistory.reset();
                }
                if (referenceState.hasStance() && referenceState.mStage == ReferenceGaitStage::DoubleSupport
                    && referenceState.mEnteredFromTouchdown)
                {
                    const std::size_t landedFoot = referenceState.leftFootIsStance() ? *leftFootIndex : *rightFootIndex;
                    if (!referenceLandingFootModel)
                        referenceLandingFootModel = pose.mBodyTransforms[landedFoot].mTranslation;
                }
                else
                    referenceLandingFootModel.reset();
                if (!referenceState.hasStance())
                {
                    referencePlacement.reset();
                    previousStanceKneeTarget.reset();
                previousStanceHingeTarget.reset();
                    referenceHistory = false;
                }
                swingFootTarget = referencePlacement;
                backwardRecoveryStep = false;
            }
            else if (!gaitController.step(gaitInput, settings.mFixedTimeStep, profile.mActiveController.mGait, gaitStep))
            {
                fail("The gait phase controller rejected its observation at step " + std::to_string(step) + ".");
                return result;
            }
            if (!referenceObservation)
                gaitOutput = gaitStep;
            if (referenceObservation)
            {
                // Reference placement and swing latching were updated above.
            }
            else if (gaitStep.hasStance())
            {
                const std::size_t swingFoot = gaitStep.leftFootIsStance() ? *rightFootIndex : *leftFootIndex;
                if (gaitStep.mTransitioned)
                {
                    swingStartModel.reset();
                    swingFootTarget.reset();
                    backwardRecoveryStep = false;
                }
                if (!swingStartModel)
                    swingStartModel = pose.mBodyTransforms[swingFoot].mTranslation;
                // Replan while the swing foot remains planted and the
                // trajectory is vertical-only. Once measured lift-off begins
                // horizontal travel, retain that last placement until contact
                // advances the physics-owned stance. This incorporates the
                // weight-shift velocity without chasing a receding COM in air.
                if ((!swingFootTarget || !gaitStep.mSwingFootLifted) && input.mSupport.hasGroundContact()
                    && locomoting)
                {
                    const JPH::Vec3 forward = requested.Normalized();
                    const std::size_t stanceBodyIndex = gaitStep.leftFootIsStance() ? *leftFootIndex : *rightFootIndex;
                    const JPH::RVec3 footCenter = telemetry.mBodies[stanceBodyIndex].mCenterOfMass;
                    const JPH::RVec3 stanceFootPosition(
                        footCenter.GetX(), footCenter.GetY(), input.mSupport.mGroundCenter.GetZ());
                    SwingFootTargetInput swingInput;
                    swingInput.mPhase = gaitStep.mPhase;
                    swingInput.mCenterOfMassPosition = telemetry.mCenterOfMass;
                    swingInput.mCenterOfMassVelocity = centerOfMassVelocity;
                    swingInput.mTargetHorizontalVelocity = requested;
                    swingInput.mStanceFootPosition = stanceFootPosition;
                    swingInput.mForward = forward;
                    swingInput.mCenterOfMassHeight = input.mSupport.mCenterOfMassHeight;
                    swingInput.mGravity = settings.mGravity;
                    const auto placement = selectGaitSwingPlacement(gaitStep, swingInput, backwardRecoveryTarget,
                        { swingFootTarget, backwardRecoveryStep }, profile.mActiveController.mGait);
                    if (!placement)
                    {
                        fail("Gait placement policy rejected its observation at step " + std::to_string(step) + ".");
                        return result;
                    }
                    swingFootTarget = placement->mTarget;
                    backwardRecoveryStep = placement->mBackwardRecovery;
                }
            }
            else
            {
                swingStartModel.reset();
                swingFootTarget.reset();
                backwardRecoveryStep = false;
            }

            frame.mSupport = *supportEstimate;
            if (frame.mSolverImpulse && frame.mSupport.hasGroundContact() && !settings.mPinPelvis)
                frame.mSolverCenterOfPressure = estimateHorizontalPlaneCenterOfPressure(
                    *frame.mSolverImpulse, frame.mSupport.mGroundCenter.GetZ());
            frame.mControllerInput = input;
            frame.mController = output;
            frame.mGait = gaitStep;
            frame.mSwingFootTarget = swingFootTarget;
            frame.mBackwardRecoveryStep = backwardRecoveryStep;
            frame.mSwingFootHorizontalPlacementError = gaitInput.mSwingFootHorizontalPlacementError;
            frame.mSwingFootVerticalPlacementError = gaitInput.mSwingFootVerticalPlacementError;
            frame.mSwingFootRelativeVerticalSpeed = gaitInput.mSwingFootRelativeVerticalSpeed;
            frame.mUprightInput = uprightInput;
            frame.mUpright = uprightOutput;
            frame.mCenterOfMass = telemetry.mCenterOfMass;
            frame.mCenterOfMassVelocity = centerOfMassVelocity;
            frame.mPelvisUpDot = input.mPelvisUpDot;
            frame.mTorsoUpDot = input.mTorsoUpDot;
            frame.mPelvisHeight = input.mPelvisHeight;
            frame.mPelvisHeightTracker = pelvisHeightOutput;
            frame.mRagdollActive = ragdoll->isActive();
            frame.mAppliedNetActuationMoment
                = frame.mAppliedForcePairMoment + frame.mAppliedGroundReactionMoment + frame.mAppliedTorqueSum;
            frame.mBodies.reserve(bodyCount);
            for (std::size_t i = 0; i < bodyCount; ++i)
            {
                ScenarioBodyRecord body;
                body.mModelTransform = pose.mBodyTransforms[i];
                body.mWorldPosition = telemetry.mBodies[i].mPosition;
                body.mCenterOfMass = telemetry.mBodies[i].mCenterOfMass;
                body.mWorldRotation = telemetry.mBodies[i].mRotation;
                body.mLinearVelocity = telemetry.mBodies[i].mMotion.mLinearVelocity;
                body.mAngularVelocity = telemetry.mBodies[i].mMotion.mAngularVelocity;
                body.mMass = telemetry.mBodies[i].mMass;
                body.mActive = telemetry.mBodies[i].mActive;
                frame.mBodies.push_back(body);
            }
            frame.mSwingTrajectoryHistory = referenceSwingHistory;
            result.mFrames.push_back(std::move(frame));
        }
        return result;
    }

    CrowdScenarioResult runStandingCrowdScenario(
        const SkeletonDescriptor& skeleton, const RagdollProfile& profile, const CrowdScenarioSettings& settings)
    {
        CrowdScenarioResult result;
        result.mSettings = settings;
        result.mProfileHash = hashProfile(profile);
        const auto fail = [&](std::string message) { result.mErrors.push_back(std::move(message)); };
        const ScenarioSettings& actorSettings = settings.mActorSettings;

        if (actorSettings.mLocomotionController == ScenarioLocomotionController::DirectMotorTracking
            || actorSettings.mLocomotionController == ScenarioLocomotionController::ProductionResidual)
            fail("Learned residual controllers require a single-actor scenario with explicit input keyframes.");

        if (settings.mActorCount < 2 || settings.mActorCount > 32 || !isFinite(settings.mActorSpacing)
            || settings.mActorSpacing <= 0.0f || !isFinite(settings.mDurationSeconds)
            || settings.mDurationSeconds <= 0.0f || !actorSettings.mPowered)
        {
            fail("A crowd scenario requires 2-32 powered actors, positive finite spacing, and positive duration.");
        }
        if (!isFinite(actorSettings.mFixedTimeStep) || actorSettings.mFixedTimeStep <= 0.0f
            || actorSettings.mCollisionSteps == 0 || !isFinite(actorSettings.mGravity) || actorSettings.mGravity < 0.0f
            || !isFinite(actorSettings.mGroundHeight) || !isFinite(actorSettings.mGroundHalfExtent)
            || actorSettings.mGroundHalfExtent <= 0.0f || !isFinite(actorSettings.mGroundThickness)
            || actorSettings.mGroundThickness <= 0.0f || !isFinite(actorSettings.mGroundFriction)
            || actorSettings.mGroundFriction < 0.0f || !isFinite(actorSettings.mTotalMass)
            || actorSettings.mTotalMass <= 0.0f || !isFinite(actorSettings.mUniformScale)
            || actorSettings.mUniformScale <= 0.0f || !isFinite(actorSettings.mRootPosition)
            || !isFinite(actorSettings.mRootRotation) || !actorSettings.mRootRotation.IsNormalized(1e-3f))
        {
            fail("Crowd actor settings must be finite, positive where required, and use a normalized rotation.");
        }
        if (isFinite(settings.mDurationSeconds) && settings.mDurationSeconds > 0.0f
            && isFinite(actorSettings.mFixedTimeStep) && actorSettings.mFixedTimeStep > 0.0f)
        {
            const double steps
                = std::round(static_cast<double>(settings.mDurationSeconds) / actorSettings.mFixedTimeStep);
            if (steps < 1.0 || steps > 1000000.0)
                fail("Crowd duration must cover between one and one million fixed steps.");
            else
                result.mStepCount = static_cast<std::uint64_t>(steps);
        }
        if (profile.mBodies.empty())
            fail("A crowd scenario requires at least one ragdoll body.");
        if (!std::is_sorted(settings.mImpulses.begin(), settings.mImpulses.end(),
                [](const CrowdScenarioImpulse& left, const CrowdScenarioImpulse& right) {
                    return left.mStep < right.mStep;
                }))
            fail("Crowd impulses must be sorted by fixed step.");
        for (const CrowdScenarioImpulse& impulse : settings.mImpulses)
        {
            if (impulse.mStep >= result.mStepCount || impulse.mActorIndex >= settings.mActorCount
                || impulse.mBodyIndex >= profile.mBodies.size() || !isFinite(impulse.mImpulse)
                || (impulse.mWorldPoint && !isFinite(*impulse.mWorldPoint)))
            {
                fail("Crowd impulses must reference a valid step, actor, body, and finite values.");
                break;
            }
        }
        if (!result.mErrors.empty())
            return result;

        const auto findBody = [&](const std::string& name) -> std::optional<std::size_t> {
            for (std::size_t i = 0; i < profile.mBodies.size(); ++i)
            {
                if (profile.mBodies[i].mName == name)
                    return i;
            }
            return std::nullopt;
        };
        const std::optional<std::size_t> pelvisIndex = findBody(profile.mLandmarks.mPelvis);
        const std::optional<std::size_t> torsoIndex
            = profile.mLandmarks.mSpine.empty() ? std::nullopt : findBody(profile.mLandmarks.mSpine.back());
        const std::optional<std::size_t> leftFootIndex
            = profile.mLandmarks.mLeftFoot ? findBody(*profile.mLandmarks.mLeftFoot) : std::nullopt;
        const std::optional<std::size_t> rightFootIndex
            = profile.mLandmarks.mRightFoot ? findBody(*profile.mLandmarks.mRightFoot) : std::nullopt;
        if (!pelvisIndex || !torsoIndex || !leftFootIndex || !rightFootIndex || *pelvisIndex == *torsoIndex
            || *leftFootIndex == *rightFootIndex)
        {
            fail("Crowd profiles require pelvis, spine, left-foot, and right-foot landmarks.");
            return result;
        }
        const FootBodyIndices feet{ *leftFootIndex, *rightFootIndex };

        const std::size_t requiredBodies = 1 + settings.mActorCount * profile.mBodies.size();
        const JPH::uint maxBodies = static_cast<JPH::uint>(std::max<std::size_t>(256, requiredBodies));
        const JPH::uint maxBodyPairs = static_cast<JPH::uint>(std::max<std::size_t>(1024, requiredBodies * 16));
        const JPH::uint maxContactConstraints
            = static_cast<JPH::uint>(std::max<std::size_t>(1024, requiredBodies * 16));
        ScenarioBroadPhaseLayers broadPhaseLayers;
        ScenarioObjectVsBroadPhase objectVsBroadPhase;
        ScenarioObjectPairs objectPairs;
        CrowdContactListener listener(settings.mActorCount);
        JPH::PhysicsSystem physics;
        physics.Init(
            maxBodies, 0, maxBodyPairs, maxContactConstraints, broadPhaseLayers, objectVsBroadPhase, objectPairs);
        physics.SetGravity(JPH::Vec3(0.0f, 0.0f, -actorSettings.mGravity));
        physics.SetContactListener(&listener);
        configureScenarioPhysics(physics, actorSettings.mVelocitySolverSteps);
        JPH::TempAllocatorImpl allocator(16 * 1024 * 1024);
        JPH::JobSystemSingleThreaded jobs(JPH::cMaxPhysicsJobs);
        JPH::BodyInterface& bodies = physics.GetBodyInterface();

        JPH::Ref<JPH::Shape> groundShape = new JPH::BoxShape(JPH::Vec3(
            actorSettings.mGroundHalfExtent, actorSettings.mGroundHalfExtent, actorSettings.mGroundThickness * 0.5f));
        JPH::BodyCreationSettings groundSettings(groundShape,
            JPH::RVec3(
                0.0, 0.0, static_cast<double>(actorSettings.mGroundHeight) - actorSettings.mGroundThickness * 0.5),
            JPH::Quat::sIdentity(), JPH::EMotionType::Static, sGroundLayer);
        groundSettings.mFriction = actorSettings.mGroundFriction;
        groundSettings.mRestitution = 0.0f;
        const JPH::BodyID ground = bodies.CreateAndAddBody(groundSettings, JPH::EActivation::DontActivate);
        if (ground.IsInvalid())
        {
            fail("Jolt could not create the crowd ground plane.");
            return result;
        }

        struct ActorRuntime
        {
            std::unique_ptr<PassiveRagdoll> mRagdoll;
            ActiveRagdollController mController;
            SettledPelvisHeightTracker mPelvisHeightTracker;
            SettledPelvisHeightOutput mPelvisHeightOutput;
            std::optional<ActiveControllerOutput> mControllerOutput;
            std::optional<SupportEstimate> mSupport;
            std::optional<UprightControllerOutput> mUpright;
            std::optional<JointSupportOutput> mJointSupport;
            RagdollTelemetry mTelemetry;
            RagdollPose mPose;
            JPH::Vec3 mPelvisLocalUp = JPH::Vec3::sAxisZ();
            JPH::Vec3 mTorsoLocalUp = JPH::Vec3::sAxisZ();
            JPH::Vec3 mTorsoLocalForward = JPH::Vec3::sAxisY();
        };
        std::vector<std::unique_ptr<ActorRuntime>> actors;
        actors.reserve(settings.mActorCount);
        struct CrowdCleanup
        {
            JPH::BodyInterface& mBodies;
            JPH::BodyID mGround;
            std::vector<std::unique_ptr<ActorRuntime>>& mActors;
            ~CrowdCleanup()
            {
                mActors.clear();
                mBodies.RemoveBody(mGround);
                mBodies.DestroyBody(mGround);
            }
        } cleanup{ bodies, ground, actors };

        std::unordered_map<std::string, const SkeletonBone*> bones;
        for (const SkeletonBone& bone : skeleton.mBones)
            bones.emplace(bone.mName, &bone);
        RagdollPose motorTarget;
        motorTarget.mBodyTransforms.reserve(profile.mBodies.size());
        for (const BodyProfile& body : profile.mBodies)
        {
            const auto bone = bones.find(body.mBone);
            if (bone == bones.end())
            {
                fail("Crowd profile body '" + body.mName + "' references an absent skeleton bone.");
                return result;
            }
            motorTarget.mBodyTransforms.push_back(bone->second->mModelTransform);
        }

        for (std::size_t actorIndex = 0; actorIndex < settings.mActorCount; ++actorIndex)
        {
            const double centeredIndex
                = static_cast<double>(actorIndex) - 0.5 * static_cast<double>(settings.mActorCount - 1);
            BuildOptions buildOptions;
            buildOptions.mTotalMass = actorSettings.mTotalMass;
            buildOptions.mUseContinuousCollision = actorSettings.mUseContinuousCollision;
            buildOptions.mUniformScale = actorSettings.mUniformScale;
            buildOptions.mRootPosition
                = actorSettings.mRootPosition + JPH::RVec3(centeredIndex * settings.mActorSpacing, 0.0, 0.0);
            buildOptions.mRootRotation = actorSettings.mRootRotation;
            buildOptions.mObjectLayer = sRagdollLayer;
            PassiveRagdollCreateResult created = createPassiveRagdoll(physics, skeleton, profile, buildOptions, {},
                static_cast<JPH::CollisionGroup::GroupID>(actorIndex + 1), actorIndex + 1);
            if (!created.isValid())
            {
                for (const ValidationIssue& issue : created.mValidation.mIssues)
                    fail("actor " + std::to_string(actorIndex) + ": " + issue.mPath + ": " + issue.mMessage);
                if (result.mErrors.empty())
                    fail("Crowd actor " + std::to_string(actorIndex) + " could not be created.");
                return result;
            }

            auto actor = std::make_unique<ActorRuntime>();
            actor->mRagdoll = std::move(created.mRagdoll);
            if (!actor->mRagdoll->addToPhysicsSystem())
            {
                fail("Crowd actor " + std::to_string(actorIndex) + " could not enter the physics system.");
                return result;
            }
            listener.registerRagdoll(actorIndex, *actor->mRagdoll);
            if (!actor->mRagdoll->captureTelemetry(actor->mTelemetry)
                || actor->mTelemetry.mBodies.size() != profile.mBodies.size()
                || !actor->mRagdoll->captureModelPose(actor->mPose)
                || actor->mPose.mBodyTransforms.size() != profile.mBodies.size())
            {
                fail("Initial telemetry could not be captured for crowd actor " + std::to_string(actorIndex) + ".");
                return result;
            }
            actor->mPelvisLocalUp
                = actor->mTelemetry.mBodies[*pelvisIndex].mRotation.Conjugated() * JPH::Vec3::sAxisZ();
            actor->mTorsoLocalUp = actor->mTelemetry.mBodies[*torsoIndex].mRotation.Conjugated() * JPH::Vec3::sAxisZ();
            actor->mTorsoLocalForward = actor->mTelemetry.mBodies[*torsoIndex].mRotation.Conjugated()
                * (actorSettings.mRootRotation * JPH::Vec3::sAxisY());
            const float initialPelvisHeight = static_cast<float>(
                static_cast<double>(actor->mTelemetry.mBodies[*pelvisIndex].mCenterOfMass.GetZ())
                - actorSettings.mGroundHeight);
            if (!actor->mPelvisLocalUp.IsNormalized() || !actor->mTorsoLocalUp.IsNormalized()
                || !isFinite(initialPelvisHeight) || initialPelvisHeight < 0.0f)
            {
                fail("Crowd actor " + std::to_string(actorIndex) + " has an invalid initial upright pose.");
                return result;
            }
            actors.push_back(std::move(actor));
        }

        result.mFrames.reserve(static_cast<std::size_t>(result.mStepCount));
        std::size_t impulseCursor = 0;
        for (std::uint64_t step = 0; step < result.mStepCount; ++step)
        {
            CrowdScenarioFrame frame;
            frame.mStep = step;
            frame.mTime = static_cast<float>(step + 1) * actorSettings.mFixedTimeStep;
            frame.mActors.resize(settings.mActorCount);
            listener.beginStep();

            for (std::size_t actorIndex = 0; actorIndex < actors.size(); ++actorIndex)
            {
                ActorRuntime& actor = *actors[actorIndex];
                CrowdActorFrame& actorFrame = frame.mActors[actorIndex];
                const bool drive = !actor.mControllerOutput || actor.mControllerOutput->mDriveJointMotors;
                actor.mRagdoll->beginActuationStep(actorSettings.mUseJointSupportControl);
                if (drive)
                {
                    actorFrame.mMotorsApplied = actor.mRagdoll->driveToPoseUsingMotors(motorTarget);
                    if (!actorFrame.mMotorsApplied)
                    {
                        fail("Crowd actor " + std::to_string(actorIndex) + " rejected its motor pose at step "
                            + std::to_string(step) + ".");
                        return result;
                    }
                    if (actorSettings.mUseJointSupportControl)
                    {
                        for (std::size_t body = 1; body < profile.mBodies.size(); ++body)
                            if (!actor.mRagdoll->driveMotorWithLoadTorque(body,
                                    actor.mJointSupport ? actor.mJointSupport->mJointTorques[body] : JPH::Vec3::sZero(),
                                    actorSettings.mFixedTimeStep))
                            {
                                fail("Crowd joint support rejected actor " + std::to_string(actorIndex));
                                return result;
                            }
                        actorFrame.mJointSupportApplied = actor.mJointSupport && actor.mJointSupport->mSupported;
                    }
                }
                else
                    actor.mRagdoll->disableMotors();

                if (drive && actor.mUpright && actor.mSupport && !actorSettings.mUseJointSupportControl)
                {
                    std::array<std::size_t, 2> reactionBodies{};
                    std::size_t reactionCount = 0;
                    if (actor.mSupport->mLeftFootSupported)
                        reactionBodies[reactionCount++] = *leftFootIndex;
                    if (actor.mSupport->mRightFootSupported)
                        reactionBodies[reactionCount++] = *rightFootIndex;
                    if (!actor.mUpright->mPelvisForce.IsNearZero() || !actor.mUpright->mPelvisTorque.IsNearZero()
                        || !actor.mUpright->mSpineTorque.IsNearZero())
                    {
                        actorFrame.mUprightApplied = actor.mRagdoll->applyUprightControl(*actor.mUpright, *pelvisIndex,
                            *torsoIndex, std::span<const std::size_t>(reactionBodies.data(), reactionCount),
                            std::span<const std::size_t>(reactionBodies.data(), reactionCount),
                            profile.mActiveController);
                        if (!actorFrame.mUprightApplied)
                        {
                            fail("Crowd actor " + std::to_string(actorIndex) + " rejected upright control at step "
                                + std::to_string(step) + ".");
                            return result;
                        }
                    }
                }
            }

            while (impulseCursor < settings.mImpulses.size() && settings.mImpulses[impulseCursor].mStep <= step)
            {
                const CrowdScenarioImpulse& impulse = settings.mImpulses[impulseCursor];
                if (impulse.mStep == step)
                {
                    const JPH::BodyID id = actors[impulse.mActorIndex]->mRagdoll->getBodyId(impulse.mBodyIndex);
                    if (impulse.mWorldPoint)
                        bodies.AddImpulse(id, impulse.mImpulse, *impulse.mWorldPoint);
                    else
                        bodies.AddImpulse(id, impulse.mImpulse);
                    ++frame.mActors[impulse.mActorIndex].mImpulseCount;
                }
                ++impulseCursor;
            }

            if (physics.Update(
                    actorSettings.mFixedTimeStep, static_cast<int>(actorSettings.mCollisionSteps), &allocator, &jobs)
                != JPH::EPhysicsUpdateError::None)
            {
                fail("Jolt reported a crowd physics update error at step " + std::to_string(step) + ".");
                return result;
            }
            CrowdStepContacts contacts = listener.takeSortedContacts(physics);
            frame.mCrossActorContactCount = contacts.mCrossActorContactCount;

            for (std::size_t actorIndex = 0; actorIndex < actors.size(); ++actorIndex)
            {
                ActorRuntime& actor = *actors[actorIndex];
                CrowdActorFrame& actorFrame = frame.mActors[actorIndex];
                if (!actor.mRagdoll->captureTelemetry(actor.mTelemetry)
                    || actor.mTelemetry.mBodies.size() != profile.mBodies.size()
                    || !actor.mRagdoll->captureModelPose(actor.mPose)
                    || actor.mPose.mBodyTransforms.size() != profile.mBodies.size())
                {
                    fail("Telemetry could not be captured for crowd actor " + std::to_string(actorIndex) + " at step "
                        + std::to_string(step) + ".");
                    return result;
                }

                const std::optional<SupportEstimate> estimate = estimateFootSupport(actor.mTelemetry.mCenterOfMass,
                    contacts.mGroundContacts[actorIndex], feet, profile.mActiveController);
                if (!estimate)
                {
                    fail("Support estimation rejected crowd actor " + std::to_string(actorIndex) + " at step "
                        + std::to_string(step) + ".");
                    return result;
                }
                if (estimate->hasGroundContact() || actor.mRagdoll->isActive() || !actor.mSupport
                    || !actor.mSupport->hasGroundContact())
                    actor.mSupport = *estimate;

                JPH::Vec3 centerOfMassVelocity = JPH::Vec3::sZero();
                for (const BodyTelemetry& body : actor.mTelemetry.mBodies)
                {
                    centerOfMassVelocity += body.mMotion.mLinearVelocity * body.mMass;
                    actorFrame.mMaxBodySpeed
                        = std::max(actorFrame.mMaxBodySpeed, body.mMotion.mLinearVelocity.Length());
                }
                centerOfMassVelocity /= actor.mTelemetry.mTotalMass;
                ActiveControllerInput input;
                input.mSupport = *actor.mSupport;
                input.mPelvisUpDot = std::clamp(
                    (actor.mTelemetry.mBodies[*pelvisIndex].mRotation * actor.mPelvisLocalUp).Dot(JPH::Vec3::sAxisZ()),
                    -1.0f, 1.0f);
                input.mTorsoUpDot = std::clamp(
                    (actor.mTelemetry.mBodies[*torsoIndex].mRotation * actor.mTorsoLocalUp).Dot(JPH::Vec3::sAxisZ()),
                    -1.0f, 1.0f);
                input.mTorsoFacingUpDot = std::clamp(
                    (actor.mTelemetry.mBodies[*torsoIndex].mRotation * actor.mTorsoLocalForward).GetZ(), -1.0f, 1.0f);
                if (input.mSupport.hasGroundContact())
                {
                    input.mPelvisHeight = std::max(0.0f,
                        static_cast<float>(
                            static_cast<double>(actor.mTelemetry.mBodies[*pelvisIndex].mCenterOfMass.GetZ())
                            - static_cast<double>(input.mSupport.mGroundCenter.GetZ())));
                }
                input.mCenterOfMassVelocity = centerOfMassVelocity;

                SettledPelvisHeightInput heightInput;
                heightInput.mSupport = input.mSupport;
                heightInput.mMode = actor.mControllerOutput ? actor.mControllerOutput->mMode
                                                            : ActiveControllerMode::Standing;
                heightInput.mPelvisHeight = input.mPelvisHeight;
                heightInput.mPelvisVerticalVelocity
                    = actor.mTelemetry.mBodies[*pelvisIndex].mMotion.mLinearVelocity.Dot(JPH::Vec3::sAxisZ());
                if (!actor.mPelvisHeightTracker.step(
                        heightInput, actorSettings.mFixedTimeStep, profile.mActiveController,
                        actor.mPelvisHeightOutput))
                {
                    fail("The settled pelvis-height tracker rejected crowd actor " + std::to_string(actorIndex)
                        + " at step " + std::to_string(step) + ".");
                    return result;
                }

                ActiveControllerOutput output;
                if (!actor.mController.step(input, actorSettings.mFixedTimeStep, profile.mActiveController, output))
                {
                    fail("The controller rejected crowd actor " + std::to_string(actorIndex) + " at step "
                        + std::to_string(step) + ".");
                    return result;
                }
                actor.mControllerOutput = output;
                actor.mUpright.reset();
                actor.mJointSupport.reset();
                if (actorSettings.mUseJointSupportControl)
                {
                    // A released/fallen body has no actuator consuming this
                    // plan. Do not solve standing contact geometry for it.
                    if (output.mDriveJointMotors)
                    {
                        actor.mJointSupport = calculateStandingJointSupport(actor.mTelemetry,
                            contacts.mGroundContacts[actorIndex], feet, actorSettings.mGravity, 0,
                            profile.mActiveController);
                        if (!actor.mJointSupport)
                        {
                            fail("Crowd support plan rejected actor " + std::to_string(actorIndex)
                                + " at step " + std::to_string(step));
                            return result;
                        }
                    }
                }
                else if (input.mSupport.hasSupport() && actor.mPelvisHeightOutput.mTargetHeight)
                {
                    UprightControllerInput upright;
                    upright.mTargetPelvisHeight = *actor.mPelvisHeightOutput.mTargetHeight;
                    upright.mPelvisHeight = input.mPelvisHeight;
                    upright.mPelvisVerticalVelocity
                        = actor.mTelemetry.mBodies[*pelvisIndex].mMotion.mLinearVelocity.Dot(JPH::Vec3::sAxisZ());
                    upright.mCenterOfMassOffset = JPH::Vec3(actor.mTelemetry.mCenterOfMass - input.mSupport.mCenter);
                    upright.mCenterOfMassVelocity = centerOfMassVelocity;
                    upright.mPelvisUp = actor.mTelemetry.mBodies[*pelvisIndex].mRotation * actor.mPelvisLocalUp;
                    upright.mPelvisAngularVelocity = actor.mTelemetry.mBodies[*pelvisIndex].mMotion.mAngularVelocity;
                    upright.mTorsoUp = actor.mTelemetry.mBodies[*torsoIndex].mRotation * actor.mTorsoLocalUp;
                    upright.mTorsoAngularVelocity = actor.mTelemetry.mBodies[*torsoIndex].mMotion.mAngularVelocity;
                    upright.mTotalMass = actor.mTelemetry.mTotalMass;
                    actor.mUpright = calculateUprightControl(upright, profile.mActiveController);
                    if (!actor.mUpright)
                    {
                        fail("Upright control rejected crowd actor " + std::to_string(actorIndex) + " at step "
                            + std::to_string(step) + ".");
                        return result;
                    }
                }

                actorFrame.mSupport = *actor.mSupport;
                actorFrame.mMode = output.mMode;
                actorFrame.mCenterOfMass = actor.mTelemetry.mCenterOfMass;
                actorFrame.mCenterOfMassVelocity = centerOfMassVelocity;
                actorFrame.mPelvisUpDot = input.mPelvisUpDot;
            }
            result.mFrames.push_back(std::move(frame));
        }
        return result;
    }

    std::vector<std::byte> encodeCrowdScenarioTelemetry(const CrowdScenarioResult& result)
    {
        TelemetryEncoder encoder;
        for (const char c : { 'O', 'M', 'W', 'R', 'D', 'C', 'R', 'D' })
            encoder.u8(static_cast<std::uint8_t>(c));
        encoder.u32(8);
        const ScenarioSettings& actor = result.mSettings.mActorSettings;
        encoder.f32(actor.mFixedTimeStep);
        encoder.u32(actor.mCollisionSteps);
        encoder.u32(actor.mVelocitySolverSteps);
        encoder.boolean(actor.mUseContinuousCollision);
        encoder.f32(actor.mGravity);
        encoder.f32(actor.mGroundHeight);
        encoder.f32(actor.mGroundHalfExtent);
        encoder.f32(actor.mGroundThickness);
        encoder.f32(actor.mGroundFriction);
        encoder.f32(actor.mTotalMass);
        encoder.f32(actor.mUniformScale);
        encoder.rvec3(actor.mRootPosition);
        encoder.quat(actor.mRootRotation);
        encoder.boolean(actor.mPowered);
        encoder.boolean(actor.mRouteGaitReactionThroughStanceLeg);
        encoder.boolean(actor.mRouteGaitHorizontalReactionToStanceFoot);
        encoder.boolean(actor.mDisableSwingHipMotor);
        encoder.boolean(actor.mUseWorldSwingHipControl);
        encoder.boolean(actor.mUseJointSupportControl);
        encoder.u64(actor.mSeed);
        encoder.u64(result.mSettings.mActorCount);
        encoder.f32(result.mSettings.mActorSpacing);
        encoder.f32(result.mSettings.mDurationSeconds);
        encoder.u64(result.mSettings.mImpulses.size());
        for (const CrowdScenarioImpulse& impulse : result.mSettings.mImpulses)
        {
            encoder.u64(impulse.mStep);
            encoder.u64(impulse.mActorIndex);
            encoder.u64(impulse.mBodyIndex);
            encoder.vec3(impulse.mImpulse);
            encoder.boolean(impulse.mWorldPoint.has_value());
            if (impulse.mWorldPoint)
                encoder.rvec3(*impulse.mWorldPoint);
        }
        encoder.u64(result.mProfileHash);
        encoder.u64(result.mStepCount);
        encoder.u64(result.mErrors.size());
        for (const std::string& error : result.mErrors)
            encoder.text(error);
        encoder.u64(result.mFrames.size());
        for (const CrowdScenarioFrame& frame : result.mFrames)
        {
            encoder.u64(frame.mStep);
            encoder.f32(frame.mTime);
            encoder.u64(frame.mCrossActorContactCount);
            encoder.u64(frame.mActors.size());
            for (const CrowdActorFrame& actorFrame : frame.mActors)
            {
                encoder.support(actorFrame.mSupport);
                encoder.u32(static_cast<std::uint32_t>(actorFrame.mMode));
                encoder.rvec3(actorFrame.mCenterOfMass);
                encoder.vec3(actorFrame.mCenterOfMassVelocity);
                encoder.f32(actorFrame.mPelvisUpDot);
                encoder.f32(actorFrame.mMaxBodySpeed);
                encoder.u32(actorFrame.mImpulseCount);
                encoder.boolean(actorFrame.mMotorsApplied);
                encoder.boolean(actorFrame.mUprightApplied);
                encoder.boolean(actorFrame.mJointSupportApplied);
            }
        }
        return encoder.take();
    }

    std::uint64_t hashCrowdScenarioTelemetry(const CrowdScenarioResult& result)
    {
        return fnv1a(encodeCrowdScenarioTelemetry(result));
    }

    std::string formatCrowdScenarioTraceCsv(const CrowdScenarioResult& result)
    {
        std::ostringstream stream;
        stream.imbue(std::locale::classic());
        stream << std::setprecision(std::numeric_limits<float>::max_digits10);
        stream << "step,time,actor,mode,left_supported,right_supported,center_supported,ground_contacts,support_friction,"
                  "com_x,com_y,com_z,com_velocity_x,com_velocity_y,com_velocity_z,pelvis_up,max_body_speed,"
                  "impulse_count,motors_applied,upright_applied,cross_actor_contacts,joint_support_applied\n";
        for (const CrowdScenarioFrame& frame : result.mFrames)
        {
            for (std::size_t actorIndex = 0; actorIndex < frame.mActors.size(); ++actorIndex)
            {
                const CrowdActorFrame& actor = frame.mActors[actorIndex];
                stream << frame.mStep << ',' << frame.mTime << ',' << actorIndex << ','
                       << static_cast<std::uint32_t>(actor.mMode) << ',' << actor.mSupport.mLeftFootSupported << ','
                       << actor.mSupport.mRightFootSupported << ',' << actor.mSupport.mCenterOfMassSupported << ','
                       << actor.mSupport.mGroundContactCount << ',' << actor.mSupport.mFriction.value_or(-1) << ',' << actor.mCenterOfMass.GetX() << ','
                       << actor.mCenterOfMass.GetY() << ',' << actor.mCenterOfMass.GetZ() << ','
                       << actor.mCenterOfMassVelocity.GetX() << ',' << actor.mCenterOfMassVelocity.GetY() << ','
                       << actor.mCenterOfMassVelocity.GetZ() << ',' << actor.mPelvisUpDot << ','
                       << actor.mMaxBodySpeed << ',' << actor.mImpulseCount << ',' << actor.mMotorsApplied << ','
                       << actor.mUprightApplied << ',' << frame.mCrossActorContactCount << ','
                       << actor.mJointSupportApplied << '\n';
            }
        }
        return stream.str();
    }

    bool haveIdenticalCrowdScenarioTelemetry(const CrowdScenarioResult& expected, const CrowdScenarioResult& actual)
    {
        return encodeCrowdScenarioTelemetry(expected) == encodeCrowdScenarioTelemetry(actual);
    }

    std::vector<std::byte> encodeScenarioTelemetry(const ScenarioResult& result)
    {
        TelemetryEncoder encoder;
        for (const char c : { 'O', 'M', 'W', 'R', 'D', 'S', 'C', 'N' })
            encoder.u8(static_cast<std::uint8_t>(c));
        encoder.u32(sTelemetryVersion);

        const ScenarioSettings& settings = result.mSettings;
        encoder.f32(settings.mFixedTimeStep);
        encoder.u32(settings.mCollisionSteps);
        encoder.u32(settings.mVelocitySolverSteps);
        encoder.boolean(settings.mUseContinuousCollision);
        encoder.f32(settings.mGravity);
        encoder.f32(settings.mGroundHeight);
        encoder.f32(settings.mGroundHalfExtent);
        encoder.f32(settings.mGroundThickness);
        encoder.f32(settings.mGroundFriction);
        encoder.boolean(settings.mTriangleMeshGround);
        encoder.f32(settings.mTotalMass);
        encoder.f32(settings.mUniformScale);
        encoder.rvec3(settings.mRootPosition);
        encoder.quat(settings.mRootRotation);
        encoder.vec3(settings.mInitialActorMotion.mLinearVelocity);
        encoder.vec3(settings.mInitialActorMotion.mAngularVelocity);
        encoder.u64(settings.mReplayBodyIds.size());
        for (const auto id : settings.mReplayBodyIds) encoder.u64(id);
        encoder.boolean(settings.mReplayGroundId.has_value());
        if (settings.mReplayGroundId) encoder.u32(*settings.mReplayGroundId);
        encoder.u64(settings.mReplayGroundShape.size());
        for (const auto byte : settings.mReplayGroundShape) encoder.u8(std::to_integer<std::uint8_t>(byte));
        encoder.rvec3(settings.mReplayGroundPosition);
        encoder.quat(settings.mReplayGroundRotation);
        encoder.f32(settings.mReplayGroundRestitution);
        encoder.boolean(settings.mReferenceInputsAreFiltered);
        encoder.boolean(settings.mReplayPreserveContactOrder);
        // Former contact-root observation replay flag, always off.
        encoder.boolean(false);
        encoder.boolean(settings.mReplayNativeBraceLimits);
        encoder.boolean(settings.mReplayAutomaticRecoveryRequest);
        encoder.boolean(settings.mReplayBraceTargets);
        encoder.boolean(settings.mReplayComputeHorizontalRoot);
        encoder.boolean(settings.mReplayDiscardSupportOnReferenceReset);
        encoder.boolean(settings.mReplayObserveRestartRequest);
        encoder.boolean(settings.mReplayRecoveryFinishEventsCaptured);
        encoder.u64(settings.mReplayRecoveryFinishSteps.size());
        for (const auto step : settings.mReplayRecoveryFinishSteps) encoder.u64(step);
        encoder.u64(settings.mReplayRecoveryRequestSteps.size());
        for (const auto step : settings.mReplayRecoveryRequestSteps) encoder.u64(step);
        encoder.boolean(settings.mReferenceGait.mBrakeLateralTransfer);
        encoder.u64(settings.mReplayReferenceFrames.size());
        for (const auto& reference : settings.mReplayReferenceFrames)
        {
            encoder.rvec3(reference.mPosition);
            encoder.quat(reference.mRotation);
        }
        encoder.boolean(settings.mPowered);
        encoder.boolean(settings.mRouteGaitReactionThroughStanceLeg);
        encoder.boolean(settings.mRouteGaitHorizontalReactionToStanceFoot);
        encoder.u32(static_cast<std::uint32_t>(settings.mTranslationRouting));
        encoder.boolean(settings.mRequireStanceSupportForSwingAssist);
        encoder.boolean(settings.mUseCapturePointForStanceReadiness);
        encoder.boolean(settings.mEnableBackwardCaptureRecovery);
        encoder.boolean(settings.mObserveSolverImpulses);
        encoder.boolean(settings.mUseSolverActuation);
        encoder.boolean(settings.mUseJointSupportControl);
        encoder.u32(static_cast<std::uint32_t>(settings.mSwingAssistAxes));
        encoder.u32(static_cast<std::uint32_t>(settings.mSwingHorizontalReaction));
        encoder.boolean(settings.mCompensateSwingHorizontalReactionThroughStance);
        encoder.boolean(settings.mReserveStanceHipMotor);
        encoder.boolean(settings.mDisableSwingHipMotor);
        encoder.boolean(settings.mUseWorldSwingHipControl);
        encoder.boolean(settings.mCompensateSwingHipMotorImpulse);
        encoder.boolean(settings.mPinPelvis);
        encoder.boolean(settings.mPreLiftSwingUnloadFraction.has_value());
        if (settings.mPreLiftSwingUnloadFraction)
            encoder.f32(*settings.mPreLiftSwingUnloadFraction);
        encoder.u32(static_cast<std::uint32_t>(settings.mLocomotionController));
        encoder.boolean(settings.mReferenceGaitHoldSwingFoot);
        encoder.boolean(settings.mReferenceGaitGroundsPlacement);
        encoder.boolean(settings.mReferenceGaitCoordinatesLegPose);
        encoder.boolean(settings.mReferenceGaitPlayerInput);
        encoder.boolean(settings.mReferenceGaitMeasuredStopping);
        const ReferenceGaitProfile& reference = settings.mReferenceGait;
        if (reference.mContinuousSwingTrajectory)
            encoder.text("continuous-swing-v1");
        for (const float value : { reference.mDoubleSupportDuration, reference.mSwingDuration,
                 reference.mMinimumSwingDuration, reference.mContactConfirmationDuration,
                 reference.mMaximumStepDuration, reference.mSwingFootLift, reference.mTouchdownPressSpeed,
                 reference.mMaximumTouchdownPressDepth, reference.mLateralCaptureOffset, reference.mMinimumStepWidth,
                 reference.mMaximumStepWidth, reference.mMinimumForwardPlacement, reference.mMaximumStepReach,
                 reference.mVelocityGain, reference.mWeightTransferFrequency, reference.mWeightTransferDamping,
                 reference.mMaximumRequestedAcceleration, reference.mSwingLegFrequency, reference.mSwingLegDamping,
                 reference.mStanceLegFrequency, reference.mStanceLegDamping, reference.mStanceKneeFlexion,
                 reference.mPlacementLatchFraction, reference.mTrunkFrequency, reference.mTrunkDamping,
                 reference.mLoadedAnkleFrequency, reference.mLoadedAnkleDamping, reference.mTransferVelocityGain,
                 reference.mSwingReachFraction, reference.mTransferLoadFraction, reference.mSwingStartLoadFraction,
                 reference.mMaximumDoubleSupportDuration, reference.mForwardVelocityErrorGain,
                 reference.mSupportHullScale, reference.mLostLandingSwapDelay, reference.mLandingPressDepth,
                 reference.mSwingRiseEndFraction, reference.mSwingTravelEndFraction,
                 reference.mTouchdownPressStartFraction, reference.mStanceHipFrequency, reference.mStanceHipDamping,
                 reference.mSwingTravelStartFraction, reference.mSwingInertiaFeedForward,
                 reference.mSwingFeedForwardAccelerationLimit, reference.mSwingDescentEndFraction,
                 reference.mFinalCommandFeasibilityStanceMargin, reference.mFinalCommandFeasibilitySwingMargin,
                 reference.mStanceKneeEaseDuration, reference.mTerminalStancePush,
                 reference.mTerminalStancePushStartFraction, reference.mTerminalStancePushLateralLimit,
                 reference.mStanceAnklePushOff, reference.mStanceAnklePushOffStartFraction,
                 reference.mMaximumLateralRequestFraction })
            encoder.f32(value);
        encoder.boolean(reference.mStanceAnklePushOffTrailing);
        encoder.u64(hashReferenceGaitProfile(reference));
        encoder.boolean(reference.mPredictPlacement);
        encoder.boolean(reference.mRequireInitialSupportTransfer);
        encoder.boolean(reference.mCoordinateInitialTransfer);
        encoder.boolean(reference.mCoordinateWeightTransfers);
        encoder.boolean(reference.mLimitVelocityLead);
        encoder.f32(reference.mUnrestrictedRequestSpeed);
        encoder.f32(reference.mVelocityLeadAccelerationFraction);
        encoder.boolean(reference.mSwingVelocityFeedForward);
        encoder.boolean(reference.mStanceHipRateHold);
        encoder.u64(settings.mSeed);

        encoder.u64(result.mProfileHash);
        encoder.u64(result.mStepCount);
        encoder.u64(result.mBodyNames.size());
        for (const std::string& name : result.mBodyNames)
            encoder.text(name);
        encoder.u64(result.mLandmarks.mPelvis);
        encoder.u64(result.mLandmarks.mTorso);
        encoder.u64(result.mLandmarks.mLeftFoot);
        encoder.u64(result.mLandmarks.mRightFoot);
        encoder.f32(result.mTargetPelvisHeight);
        encoder.boolean(result.mKillingImpulse.has_value());
        if (result.mKillingImpulse)
        {
            encoder.u64(result.mKillingImpulse->mBodyIndex);
            encoder.vec3(result.mKillingImpulse->mImpulse);
            encoder.rvec3(result.mKillingImpulse->mWorldPoint);
        }

        encoder.u64(result.mFrames.size());
        for (const ScenarioFrame& frame : result.mFrames)
        {
            encoder.u64(frame.mStep);
            encoder.f32(frame.mTime);
            encoder.vec3(frame.mRequestedHorizontalVelocity);
            encoder.boolean(frame.mMotorsEnabled);
            encoder.boolean(frame.mTranslationActuationEnabled);
            encoder.boolean(frame.mSwingFootActuationEnabled);
            encoder.boolean(frame.mUprightTorqueEnabled);
            encoder.f32(frame.mStabilizationStrength);
            encoder.u32(frame.mImpulseCount);
            encoder.boolean(frame.mRootKinematic);
            encoder.boolean(frame.mKinematicLocomotion);
            encoder.u32(frame.mHybridStage);
            encoder.optionalFloat(frame.mRootReleaseVelocityError);
            encoder.boolean(frame.mKinematicRootTarget.has_value());
            if (frame.mKinematicRootTarget)
            {
                encoder.vec3(frame.mKinematicRootTarget->mTranslation);
                encoder.quat(frame.mKinematicRootTarget->mRotation);
            }
            encoder.u32(static_cast<std::uint32_t>(frame.mActuationStage));
            encoder.u32(static_cast<std::uint32_t>(frame.mUprightActuationStage));
            encoder.boolean(frame.mAppliedUpright.has_value());
            if (frame.mAppliedUpright)
                encoder.uprightOutput(*frame.mAppliedUpright);
            encoder.boolean(frame.mActuationGait.has_value());
            if (frame.mActuationGait)
                encoder.gait(*frame.mActuationGait);
            encoder.boolean(frame.mMotorsApplied);
            encoder.f32(frame.mMaxAnchorSeparation);
            encoder.u64(frame.mJointSupportMotorAngularImpulses.size());
            for (const auto& impulse : frame.mJointSupportMotorAngularImpulses)
                encoder.vec3(impulse);
            encoder.u64(frame.mJointSupportMotorUtilizations.size());
            for (const auto utilization : frame.mJointSupportMotorUtilizations)
                encoder.f32(utilization);
            encoder.boolean(frame.mJointSupport.has_value());
            if (frame.mJointSupport)
            {
                const auto& support = *frame.mJointSupport;
                encoder.boolean(support.mSupported);
                encoder.vec3(support.mAcceleration);
                encoder.rvec3(support.mCenterOfPressure);
                encoder.vec3(support.mRootForceResidual);
                encoder.vec3(support.mRootMomentResidual);
                encoder.u64(support.mJointTorques.size());
                for (const auto& torque : support.mJointTorques)
                    encoder.vec3(torque);
                encoder.u64(support.mContactForces.size());
                for (const auto& contact : support.mContactForces)
                {
                    encoder.u64(contact.mBodyIndex);
                    encoder.rvec3(contact.mPosition);
                    encoder.vec3(contact.mForce);
                }
            }
            encoder.boolean(frame.mStanceHipMotorDisabled);
            encoder.boolean(frame.mSwingHipMotorDisabled);
            encoder.boolean(frame.mStanceHipAxisReleased);
            encoder.boolean(frame.mSwingHipAxisReleased);
            encoder.boolean(frame.mSwingHipWorldControlApplied);
            encoder.boolean(frame.mSwingHipWorldTorqueSaturated);
            encoder.vec3(frame.mAppliedSwingHipWorldTorque);
            encoder.boolean(frame.mDisabledStanceHipBody.has_value());
            if (frame.mDisabledStanceHipBody)
                encoder.u64(*frame.mDisabledStanceHipBody);
            encoder.boolean(frame.mSwingHipMotorBody.has_value());
            if (frame.mSwingHipMotorBody)
                encoder.u64(*frame.mSwingHipMotorBody);
            encoder.vec3(frame.mSwingHipMotorAngularImpulse);
            encoder.f32(frame.mSwingHipMotorPeakAxisUtilization);
            encoder.vec3(frame.mSwingKneeMotorAngularImpulse);
            encoder.f32(frame.mSwingKneeMotorPeakAxisUtilization);
            encoder.vec3(frame.mSwingFootJointMotorAngularImpulse);
            encoder.f32(frame.mSwingFootJointMotorPeakAxisUtilization);
            for (std::size_t joint = 0; joint < frame.mStanceLegMotorAngularImpulses.size(); ++joint)
            {
                encoder.vec3(frame.mStanceLegMotorAngularImpulses[joint]);
                encoder.f32(frame.mStanceLegMotorPeakAxisUtilizations[joint]);
            }
            encoder.optionalMotorTarget(frame.mSwingHipMotorTarget);
            encoder.optionalMotorTarget(frame.mSwingKneeMotorTarget);
            encoder.optionalMotorTarget(frame.mSwingFootJointMotorTarget);
            encoder.u64(frame.mJointMotorTargets.size());
            for (const auto& target : frame.mJointMotorTargets)
                encoder.optionalMotorTarget(target);
            encoder.vec3(frame.mAppliedStanceHipCompensationImpulse);
            encoder.boolean(frame.mUprightApplied);
            encoder.boolean(frame.mSwingFootControlApplied);
            encoder.boolean(frame.mStanceSupportsCenterOfMass);
            encoder.f32(frame.mPreLiftSwingUnloadForce);
            encoder.boolean(frame.mSwingAssistReady);
            encoder.u64(frame.mSwingFootVerticalReactionBody);
            encoder.u64(frame.mSwingFootHorizontalReactionBody);
            encoder.boolean(frame.mSwingFootControl.has_value());
            if (frame.mSwingFootControl)
            {
                encoder.vec3(frame.mSwingFootControl->mForce);
                encoder.boolean(frame.mSwingFootControl->mSaturated);
            }
            encoder.indices(frame.mVerticalReactionBodies);
            encoder.indices(frame.mHorizontalReactionBodies);
            encoder.boolean(frame.mAngularReactionBody.has_value());
            if (frame.mAngularReactionBody)
                encoder.u64(*frame.mAngularReactionBody);
            encoder.boolean(frame.mSecondaryAngularReactionBody.has_value());
            if (frame.mSecondaryAngularReactionBody)
                encoder.u64(*frame.mSecondaryAngularReactionBody);
            encoder.vec3(frame.mAppliedPelvisForce);
            encoder.vec3(frame.mAppliedPelvisTorque);
            encoder.vec3(frame.mAppliedSpineTorque);
            encoder.vec3(frame.mAppliedNetForce);
            encoder.vec3(frame.mAppliedForcePairMoment);
            encoder.vec3(frame.mAppliedSwingForcePairMoment);
            encoder.vec3(frame.mAppliedUprightForcePairMoment);
            encoder.vec3(frame.mAppliedGroundReactionForce);
            encoder.vec3(frame.mAppliedGroundReactionMoment);
            encoder.vec3(frame.mAppliedTorqueSum);
            encoder.vec3(frame.mAppliedNetActuationMoment);
            encoder.optionalVec3(frame.mSwingFootMotorTarget);
            encoder.optionalVec3(frame.mResolvedSwingFootTarget);
            encoder.optionalVec3(frame.mSwingFootPoseTarget);
            encoder.boolean(frame.mReferenceGait.has_value());
            if (frame.mReferenceGait)
            {
                const ReferenceGaitFrameRecord& record = *frame.mReferenceGait;
                encoder.u32(static_cast<std::uint32_t>(record.mStage));
                encoder.f32(record.mTimeInStage);
                encoder.boolean(record.mSwingPoseApplied);
                encoder.optionalRVec3(record.mPlacement);
                encoder.vec3(record.mRequestedAcceleration);
                encoder.vec3(record.mPelvisOrientationError);
                encoder.vec3(record.mBranchReaction);
                encoder.vec3(record.mStanceHipTorque);
                encoder.f32(record.mLeftLoadFraction);
                encoder.f32(record.mRightLoadFraction);
                encoder.f32(record.mPlannedStanceFootLoad);
                encoder.f32(record.mMaxMotorUtilization);
                encoder.u64(record.mMaxMotorUtilizationBody);
                encoder.vec3(record.mStanceHipLoadTorque);
                encoder.vec3(record.mStanceHipMotorTorque);
                encoder.vec3(record.mStanceAnkleLoadTorque);
                encoder.vec3(record.mStanceAnkleMotorTorque);
                encoder.f32(record.mTransferVelocity);
                encoder.f32(record.mTransferReferenceVelocity);
                encoder.boolean(record.mSwingReachLimited);
                encoder.u32(record.mFeasibilityClampedCount);
                encoder.f32(record.mStanceKneeTargetJump);
                encoder.f32(record.mStanceKneeModelTargetJumpLegacy);
                encoder.boolean(record.mStanceKneeHingeTarget.has_value());
                if (record.mStanceKneeHingeTarget)
                    encoder.f32(*record.mStanceKneeHingeTarget);
                if (settings.mReferenceGaitCoordinatesLegPose)
                {
                    encoder.boolean(record.mCoordinatedPelvisTargetZ.has_value());
                    if (record.mCoordinatedPelvisTargetZ) encoder.f64(*record.mCoordinatedPelvisTargetZ);
                }
            }

            // The removed contact-root route's frame fields, encoded with
            // their absent values so existing scenario identities stay valid:
            // no input, not replayed, no output, no reference pose, owner
            // "production", load weights 1/0, no stance or touchdown body,
            // no anchors and no capture or hold.
            encoder.boolean(false);
            encoder.boolean(false);
            encoder.boolean(false);
            encoder.boolean(false);
            encoder.text("production");
            encoder.f32(1.0f);
            encoder.f32(0.0f);
            for (int absent = 0; absent < 6; ++absent)
                encoder.boolean(false);

            encoder.u64(frame.mContacts.size());
            for (const GroundContact& contact : frame.mContacts)
            {
                encoder.u64(contact.mBodyIndex);
                encoder.rvec3(contact.mPosition);
                encoder.vec3(contact.mNormal);
                encoder.f32(contact.mPenetrationDepth);
                encoder.boolean(contact.mNormalImpulse.has_value());
                if (contact.mNormalImpulse)
                    encoder.f32(*contact.mNormalImpulse);
                encoder.optionalFloat(contact.mFriction);
                encoder.optionalVec3(contact.mPointVelocity);
                encoder.optionalFloat(contact.mFrictionImpulseMagnitude);
            }
            encoder.support(frame.mSupport);

            const ActiveControllerInput& input = frame.mControllerInput;
            encoder.support(input.mSupport);
            encoder.f32(input.mPelvisUpDot);
            encoder.f32(input.mTorsoUpDot);
            encoder.f32(input.mTorsoFacingUpDot);
            encoder.f32(input.mPelvisHeight);
            encoder.vec3(input.mCenterOfMassVelocity);
            encoder.vec3(input.mTargetHorizontalVelocity);
            encoder.boolean(input.mRecoveryRequested);
            encoder.boolean(input.mActuationInterrupted);
            encoder.boolean(input.mRecoveryInterrupted);
            encoder.boolean(input.mRootExternallyOwned);
            encoder.boolean(input.mRecoveryAnimationFinished);

            const ActiveControllerOutput& controller = frame.mController;
            encoder.u32(static_cast<std::uint32_t>(controller.mMode));
            encoder.f32(controller.mTimeInMode);
            encoder.f32(controller.mFallingExitCandidateTime);
            encoder.boolean(controller.mDriveJointMotors);
            encoder.boolean(controller.mRecoveryHeightAssist);
            encoder.boolean(controller.mRecoveryDirection.has_value());
            if (controller.mRecoveryDirection)
                encoder.u32(static_cast<std::uint32_t>(*controller.mRecoveryDirection));
            encoder.gait(frame.mGait);
            encoder.optionalRVec3(frame.mSwingFootTarget);
            encoder.boolean(frame.mBackwardRecoveryStep);
            encoder.optionalRVec3(frame.mSolverCenterOfPressure);
            encoder.boolean(frame.mMomentum.has_value());
            if (frame.mMomentum)
            {
                encoder.f32(frame.mMomentum->mTotalMass);
                encoder.rvec3(frame.mMomentum->mCenterOfMass);
                encoder.vec3(frame.mMomentum->mLinearMomentum);
                encoder.vec3(frame.mMomentum->mAngularMomentum);
            }
            encoder.boolean(frame.mSolverImpulse.has_value());
            if (frame.mSolverImpulse)
            {
                encoder.rvec3(frame.mSolverImpulse->mReferenceCenterOfMass);
                encoder.vec3(frame.mSolverImpulse->mLinearImpulse);
                encoder.vec3(frame.mSolverImpulse->mAngularImpulse);
                encoder.vec3(frame.mSolverImpulse->mInternalAnchorImpulseMoment);
                encoder.boolean(frame.mSolverImpulse->mVelocityLimitTouched);
                encoder.boolean(frame.mSolverImpulse->mActivationChanged);
                encoder.boolean(frame.mSolverImpulse->mContinuousCollisionEnabled);
                encoder.boolean(frame.mSolverImpulse->mFreeVelocityLimitTouched);
                encoder.vec3(frame.mSolverImpulse->mFreeLinearLimitImpulse);
                encoder.vec3(frame.mSolverImpulse->mFreeAngularLimitImpulse);
            }
            encoder.f32(frame.mSwingFootHorizontalPlacementError);
            encoder.f32(frame.mSwingFootVerticalPlacementError);
            encoder.f32(frame.mSwingFootRelativeVerticalSpeed);

            encoder.boolean(frame.mUprightInput.has_value());
            if (frame.mUprightInput)
                encoder.uprightInput(*frame.mUprightInput);
            encoder.boolean(frame.mUpright.has_value());
            if (frame.mUpright)
                encoder.uprightOutput(*frame.mUpright);

            encoder.rvec3(frame.mCenterOfMass);
            encoder.vec3(frame.mCenterOfMassVelocity);
            encoder.f32(frame.mPelvisUpDot);
            encoder.f32(frame.mTorsoUpDot);
            encoder.f32(frame.mPelvisHeight);
            encoder.optionalFloat(frame.mPelvisHeightTracker.mTargetHeight);
            encoder.f32(frame.mPelvisHeightTracker.mCandidateTime);
            encoder.boolean(frame.mPelvisHeightTracker.mCandidateValid);
            encoder.boolean(frame.mPelvisHeightTracker.mSettled);
            encoder.boolean(frame.mPelvisHeightTracker.mFrozen);
            encoder.boolean(frame.mRagdollActive);
            encoder.boolean(frame.mSupportMotion.has_value());
            if (frame.mSupportMotion)
            {
                encoder.rvec3(frame.mSupportMotion->mPosition);
                encoder.vec3(frame.mSupportMotion->mVelocity);
                encoder.vec3(frame.mSupportMotion->mAcceleration);
                encoder.boolean(frame.mSupportMotion->mSupportFoot.has_value());
                if (frame.mSupportMotion->mSupportFoot)
                    encoder.u64(*frame.mSupportMotion->mSupportFoot);
            }

            encoder.u64(frame.mBodies.size());
            for (const ScenarioBodyRecord& body : frame.mBodies)
            {
                encoder.transform(body.mModelTransform);
                encoder.rvec3(body.mWorldPosition);
                encoder.rvec3(body.mCenterOfMass);
                encoder.quat(body.mWorldRotation);
                encoder.vec3(body.mLinearVelocity);
                encoder.vec3(body.mAngularVelocity);
                encoder.f32(body.mMass);
                encoder.boolean(body.mActive);
            }
        }

        encoder.u64(result.mErrors.size());
        for (const std::string& error : result.mErrors)
            encoder.text(error);
        return encoder.take();
    }

    std::string formatScenarioTraceCsv(const ScenarioResult& result)
    {
        const bool hasSupportMotion = std::any_of(result.mFrames.begin(), result.mFrames.end(),
            [](const ScenarioFrame& frame) { return frame.mSupportMotion.has_value(); });
        std::ostringstream stream;
        stream.imbue(std::locale::classic());
        stream << std::setprecision(std::numeric_limits<float>::max_digits10);
        stream << "step,time,gait_profile_hash,mode,gait_phase,gait_progress,gait_stance_ready,raw_stance_support,gait_transitioned,"
                  "gait_timed_out,swing_lifted,swing_touchdown_candidate,gait_contact_candidate_time,"
                  "gait_time_since_lift,swing_assist_ready,"
                  "swing_vertical_reaction_body,swing_horizontal_reaction_body,actuation_gait_phase,"
                  "actuation_gait_progress,actuation_stance_ready,"
                  "left_supported,right_supported,ground_contacts,support_friction,pre_lift_unload_force,pelvis_height,pelvis_vertical_velocity,"
                  "target_pelvis_height,pelvis_height_candidate_time,pelvis_height_candidate_valid,"
                  "pelvis_height_settled,pelvis_height_frozen,pelvis_up,com_x,com_y,com_z,"
                  "com_velocity_x,com_velocity_y,com_velocity_z,request_x,request_y,request_z,force_x,force_y,"
                  "force_z,upright_applied,magnitude_saturated,horizontal_saturated,support_moment_saturated,"
                  "vertical_saturated,swing_control_applied,swing_force_x,swing_force_y,swing_force_z,"
                  "swing_force_saturated,left_foot_x,left_foot_y,left_foot_z,left_foot_vz,right_foot_x,"
                  "right_foot_y,right_foot_z,right_foot_vz,"
                  "swing_placement_x,swing_placement_y,swing_placement_z,swing_placement_error,"
                  "swing_vertical_placement_error,swing_relative_vertical_speed,swing_motor_target_x,"
                  "swing_motor_target_y,swing_motor_target_z,resolved_swing_target_x,resolved_swing_target_y,"
                  "resolved_swing_target_z,swing_pose_target_x,swing_pose_target_y,swing_pose_target_z,"
                  "swing_hip_target_error,swing_knee_target_error,swing_ankle_target_error,"
                  "swing_hip_target_clamp,swing_ankle_target_clamp,"
                  "swing_hip_current_qx,swing_hip_current_qy,swing_hip_current_qz,swing_hip_current_qw,"
                  "swing_hip_requested_qx,swing_hip_requested_qy,swing_hip_requested_qz,swing_hip_requested_qw,"
                  "swing_hip_target_qx,swing_hip_target_qy,swing_hip_target_qz,swing_hip_target_qw,"
                  "swing_hip_normal_cone,swing_hip_plane_cone,swing_hip_twist_min,swing_hip_twist_max,"
                  "swing_hip_motor_utilization,swing_knee_motor_utilization,swing_ankle_motor_utilization,"
                  "swing_knee_current_angle,swing_knee_target_angle,swing_knee_minimum,swing_knee_maximum,"
                  "actuation_stage,upright_actuation_stage,stance_hip_axis_released,swing_hip_axis_released,"
                  "swing_ankle_world_motor,backward_recovery_step,solver_observed,solver_linear_impulse_x,"
                  "solver_linear_impulse_y,solver_linear_impulse_z,solver_angular_impulse_x,solver_angular_impulse_y,"
                  "solver_angular_impulse_z,solver_anchor_moment_x,solver_anchor_moment_y,solver_anchor_moment_z,"
                  "solver_velocity_limit,solver_activation_changed,solver_ccd_enabled,solver_cop_valid,solver_cop_x,solver_cop_y,solver_cop_z,"
                  "solver_reference_com_x,solver_reference_com_y,solver_reference_com_z,"
                  "stance_hip_impulse_x,stance_hip_impulse_y,stance_hip_impulse_z,stance_hip_utilization,"
                  "stance_knee_impulse_x,stance_knee_impulse_y,stance_knee_impulse_z,stance_knee_utilization,"
                  "stance_ankle_impulse_x,stance_ankle_impulse_y,stance_ankle_impulse_z,stance_ankle_utilization,"
                  "swing_hip_impulse_x,swing_hip_impulse_y,swing_hip_impulse_z,"
                  "swing_knee_impulse_x,swing_knee_impulse_y,swing_knee_impulse_z,"
                  "swing_ankle_impulse_x,swing_ankle_impulse_y,swing_ankle_impulse_z,"
                  "pelvis_torque_x,pelvis_torque_y,pelvis_torque_z,spine_torque_x,spine_torque_y,spine_torque_z,"
                  "centroidal_momentum_observed,centroidal_angular_momentum_x,"
                  "centroidal_angular_momentum_y,centroidal_angular_momentum_z,"
                  "free_velocity_limit,free_linear_limit_impulse_x,free_linear_limit_impulse_y,free_linear_limit_impulse_z,"
                  "free_angular_limit_impulse_x,free_angular_limit_impulse_y,free_angular_limit_impulse_z,"
                  "max_anchor_separation,joint_support_planned,joint_support_hull_delta_x,"
                  "joint_support_hull_delta_y,joint_support_friction_limited,joint_support_acceleration_x,"
                  "joint_support_acceleration_y,joint_support_acceleration_z,joint_support_root_force_x,"
                  "joint_support_root_force_y,joint_support_root_force_z,joint_support_root_moment_x,"
                  "joint_support_root_moment_y,joint_support_root_moment_z,"
                  "reference_stage,reference_time_in_stage,reference_swing_pose_applied,"
                  "reference_placement_x,reference_placement_y,reference_placement_z,"
                  "reference_requested_acceleration_x,reference_requested_acceleration_y,"
                  "reference_pelvis_error_x,reference_pelvis_error_y,reference_pelvis_error_z,"
                  "reference_branch_reaction_x,reference_branch_reaction_y,reference_branch_reaction_z,"
                  "reference_stance_hip_torque_x,reference_stance_hip_torque_y,reference_stance_hip_torque_z,"
                  "reference_left_load_fraction,reference_right_load_fraction,"
                  "reference_planned_stance_load,reference_max_motor_utilization,reference_max_motor_utilization_body,"
                  "reference_stance_hip_load_x,reference_stance_hip_load_y,reference_stance_hip_load_z,"
                  "reference_stance_hip_motor_x,reference_stance_hip_motor_y,reference_stance_hip_motor_z,"
                  "reference_stance_ankle_load_x,reference_stance_ankle_load_y,reference_stance_ankle_load_z,"
                  "reference_stance_ankle_motor_x,reference_stance_ankle_motor_y,reference_stance_ankle_motor_z,"
                  "reference_transfer_velocity,reference_transfer_reference_velocity,reference_swing_reach_limited,"
                  "reference_ff_hip_x,reference_ff_hip_y,reference_ff_hip_z,"
                  "reference_ff_knee_x,reference_ff_knee_y,reference_ff_knee_z,"
                  "reference_hip_rate_target_x,reference_hip_rate_target_y,reference_hip_rate_target_z,"
                  "reference_hip_rate_x,reference_hip_rate_y,reference_hip_rate_z,"
                  "reference_placement_com_x,reference_placement_com_y,"
                  "reference_placement_velocity_x,reference_placement_velocity_y,reference_feasibility_clamped,"
                  "reference_stance_knee_target_jump,reference_stance_knee_model_target_jump_legacy,reference_stance_knee_hinge_target,"
                  "pelvis_heading,left_foot_heading,right_foot_heading,left_foot_contacts,right_foot_contacts,"
                  "left_foot_roll,right_foot_roll,left_cop_x,left_cop_y,right_cop_x,right_cop_y,"
                  "left_contact_normal_impulse,right_contact_normal_impulse,"
                  "left_loaded_point_min_vz,left_loaded_point_max_vz,right_loaded_point_min_vz,right_loaded_point_max_vz";
        if (hasSupportMotion)
            stream << ",support_target_x,support_target_y,support_target_z,support_velocity_x,support_velocity_y,"
                      "support_velocity_z,support_acceleration_x,support_acceleration_y,support_acceleration_z,support_foot";
        if (result.mSettings.mReferenceGaitCoordinatesLegPose)
            stream << ",coordinated_pelvis_com_target_z";
        stream << ",reference_coordinated_pose_applied";
        stream << ",reference_stance_hip_body,reference_stance_hip_utilization"
                  ",reference_stance_ankle_body,reference_stance_ankle_utilization";
        stream << ",reference_emergency_step,reference_emergency_step_count";
        stream << ",left_loaded_point_max_horizontal_speed,right_loaded_point_max_horizontal_speed";
        stream << ",left_contact_peak_friction_ratio,right_contact_peak_friction_ratio";
        stream << ",contact_root_input_available,contact_root_input_replayed,contact_root_input_duration"
                  ",contact_root_input_locomotion_requested,contact_root_input_interrupted"
                  ",contact_root_input_left_supported,contact_root_input_right_supported"
                  ",contact_root_input_production_ready,contact_root_input_measured_velocity_x"
                  ",contact_root_input_measured_velocity_y,contact_root_input_measured_velocity_z"
                  ",contact_root_input_left_slip,contact_root_input_right_slip"
                  ",contact_root_input_left_load,contact_root_input_right_load"
                  ",contact_root_input_support_error,contact_root_input_incoming_available"
                  ",contact_root_input_incoming_error,contact_root_input_requested_direction_x"
                  ",contact_root_input_requested_direction_y,contact_root_input_requested_direction_z";
        stream << ",contact_root_available,contact_root_stance,contact_root_next_stance,contact_root_sample_index,contact_root_phase"
                  ",contact_root_envelope_weight,contact_root_locomotion_weight,contact_root_load_transfer_weight"
                  ",contact_root_handoff_weight,contact_root_confirmation_ticks"
                  ",contact_root_activation_confirmed,contact_root_translation_allowed"
                  ",contact_root_stance_changed,contact_root_support_reacquired"
                  ",contact_root_root_offset_x,contact_root_root_offset_y,contact_root_root_offset_z"
                  ",contact_root_root_displacement_x,contact_root_root_displacement_y,contact_root_root_displacement_z"
                  ",contact_root_root_velocity_x,contact_root_root_velocity_y,contact_root_root_velocity_z"
                  ",contact_root_nominal_progress_speed,contact_root_measured_progress_speed"
                  ",contact_root_governed_progress_speed,contact_root_selected_slip_speed"
                  ",contact_root_support_reference_error,contact_root_feasibility_hold_time"
                  ",contact_root_transition_confirmation_ticks,contact_root_support_feasible"
                  ",contact_root_progress_held,contact_root_touchdown_pending"
                  ",contact_root_incoming_anchor_error,contact_root_incoming_anchor_available"
                  ",contact_root_incoming_anchor_reachable,contact_root_incoming_anchor_qualified"
                  ",contact_root_left_load_fraction,contact_root_right_load_fraction"
                  ",contact_root_bilateral_load_weight,contact_root_unilateral_load_weight"
                  ",contact_root_load_target_stance,contact_root_incoming_load_confirmed"
                  ",contact_root_outgoing_foot_unloaded,contact_root_load_transition_pending"
                  ",contact_root_load_transition_age,contact_root_commit_confirmation_ticks"
                  ",contact_root_reentry_slip_settled_time,contact_root_reentry_eligible"
                  ",contact_root_fallback_reason"
                  ",contact_root_authority_owner,contact_root_production_load_weight"
                  ",contact_root_candidate_load_weight,contact_root_stance_body,contact_root_touchdown_body"
                  ",contact_root_outgoing_hold_active,contact_root_outgoing_hold_error_x"
                  ",contact_root_outgoing_hold_error_y,contact_root_outgoing_hold_speed"
                  ",contact_root_outgoing_hold_saturated,contact_root_touchdown_active"
                  ",contact_root_touchdown_error_x,contact_root_touchdown_error_y"
                  ",contact_root_touchdown_speed,contact_root_touchdown_saturated"
                  ",root_kinematic,kinematic_root_x,kinematic_root_y,kinematic_root_z"
                  ",kinematic_root_qx,kinematic_root_qy,kinematic_root_qz,kinematic_root_qw"
                  ",kinematic_locomotion,hybrid_stage,root_release_velocity_error";
        stream << '\n';
        for (const ScenarioFrame& frame : result.mFrames)
        {
            const UprightControllerOutput empty;
            const UprightControllerOutput& upright = frame.mUpright ? *frame.mUpright : empty;
            const JPH::RVec3 left = result.mLandmarks.mLeftFoot < frame.mBodies.size()
                ? frame.mBodies[result.mLandmarks.mLeftFoot].mCenterOfMass
                : JPH::RVec3::sZero();
            const JPH::RVec3 right = result.mLandmarks.mRightFoot < frame.mBodies.size()
                ? frame.mBodies[result.mLandmarks.mRightFoot].mCenterOfMass
                : JPH::RVec3::sZero();
            const JPH::Vec3 leftVelocity = result.mLandmarks.mLeftFoot < frame.mBodies.size()
                ? frame.mBodies[result.mLandmarks.mLeftFoot].mLinearVelocity
                : JPH::Vec3::sZero();
            const JPH::Vec3 rightVelocity = result.mLandmarks.mRightFoot < frame.mBodies.size()
                ? frame.mBodies[result.mLandmarks.mRightFoot].mLinearVelocity
                : JPH::Vec3::sZero();
            const SwingFootControllerOutput emptySwing;
            const SwingFootControllerOutput& swing = frame.mSwingFootControl ? *frame.mSwingFootControl : emptySwing;
            stream << frame.mStep << ',' << frame.mTime << ',' << hashReferenceGaitProfile(result.mSettings.mReferenceGait)
                   << ',' << static_cast<std::uint32_t>(frame.mController.mMode)
                   << ',' << static_cast<std::uint32_t>(frame.mGait.mPhase) << ',' << frame.mGait.mPhaseProgress
                   << ',' << frame.mGait.mStanceReady << ',' << frame.mStanceSupportsCenterOfMass << ','
                   << frame.mGait.mTransitioned << ',' << frame.mGait.mPhaseTimedOut << ','
                   << frame.mGait.mSwingFootLifted << ',' << frame.mGait.mSwingTouchdownCandidate << ','
                   << frame.mGait.mSwingContactCandidateTime << ','
                   << frame.mGait.mSwingTimeSinceLift << ',' << frame.mSwingAssistReady << ','
                   << frame.mSwingFootVerticalReactionBody << ',' << frame.mSwingFootHorizontalReactionBody;
            if (frame.mActuationGait)
                stream << ',' << static_cast<std::uint32_t>(frame.mActuationGait->mPhase) << ','
                       << frame.mActuationGait->mPhaseProgress << ',' << frame.mActuationGait->mStanceReady;
            else
                stream << ",,,";
            stream << ',' << frame.mSupport.mLeftFootSupported << ','
                   << frame.mSupport.mRightFootSupported << ',' << frame.mSupport.mGroundContactCount << ','
                   << frame.mSupport.mFriction.value_or(-1) << ','
                   << frame.mPreLiftSwingUnloadForce << ',' << frame.mPelvisHeight << ','
                   << frame.mBodies[result.mLandmarks.mPelvis].mLinearVelocity.GetZ() << ',';
            if (frame.mPelvisHeightTracker.mTargetHeight)
                stream << *frame.mPelvisHeightTracker.mTargetHeight;
            stream << ',' << frame.mPelvisHeightTracker.mCandidateTime << ','
                   << frame.mPelvisHeightTracker.mCandidateValid << ',' << frame.mPelvisHeightTracker.mSettled << ','
                   << frame.mPelvisHeightTracker.mFrozen << ','
                   << frame.mPelvisUpDot << ',' << frame.mCenterOfMass.GetX() << ','
                   << frame.mCenterOfMass.GetY() << ',' << frame.mCenterOfMass.GetZ() << ','
                   << frame.mCenterOfMassVelocity.GetX() << ',' << frame.mCenterOfMassVelocity.GetY() << ','
                   << frame.mCenterOfMassVelocity.GetZ() << ',' << frame.mRequestedHorizontalVelocity.GetX() << ','
                   << frame.mRequestedHorizontalVelocity.GetY() << ',' << frame.mRequestedHorizontalVelocity.GetZ()
                   << ',' << upright.mPelvisForce.GetX() << ',' << upright.mPelvisForce.GetY() << ','
                   << upright.mPelvisForce.GetZ() << ',' << frame.mUprightApplied << ','
                   << upright.mPelvisMagnitudeSaturated << ',' << upright.mPelvisHorizontalForceSaturated << ','
                   << upright.mPelvisSupportMomentSaturated << ',' << upright.mPelvisVerticalForceSaturated << ','
                   << frame.mSwingFootControlApplied << ',' << swing.mForce.GetX() << ',' << swing.mForce.GetY() << ','
                   << swing.mForce.GetZ() << ',' << swing.mSaturated << ',' << left.GetX() << ',' << left.GetY() << ','
                   << left.GetZ() << ',' << leftVelocity.GetZ() << ',' << right.GetX() << ',' << right.GetY() << ','
                   << right.GetZ() << ',' << rightVelocity.GetZ();
            if (frame.mSwingFootTarget)
            {
                stream << ',' << frame.mSwingFootTarget->GetX() << ',' << frame.mSwingFootTarget->GetY() << ','
                       << frame.mSwingFootTarget->GetZ();
            }
            else
                stream << ",,,";
            stream << ',' << frame.mSwingFootHorizontalPlacementError << ','
                   << frame.mSwingFootVerticalPlacementError << ',' << frame.mSwingFootRelativeVerticalSpeed;
            if (frame.mSwingFootMotorTarget)
            {
                stream << ',' << frame.mSwingFootMotorTarget->GetX() << ','
                       << frame.mSwingFootMotorTarget->GetY() << ',' << frame.mSwingFootMotorTarget->GetZ();
            }
            else
                stream << ",,,";
            if (frame.mResolvedSwingFootTarget)
            {
                stream << ',' << frame.mResolvedSwingFootTarget->GetX() << ','
                       << frame.mResolvedSwingFootTarget->GetY() << ',' << frame.mResolvedSwingFootTarget->GetZ();
            }
            else
                stream << ",,,";
            if (frame.mSwingFootPoseTarget)
            {
                stream << ',' << frame.mSwingFootPoseTarget->GetX() << ',' << frame.mSwingFootPoseTarget->GetY()
                       << ',' << frame.mSwingFootPoseTarget->GetZ();
            }
            else
                stream << ",,,";
            const auto error = [](const std::optional<MotorTargetTelemetry>& telemetry) {
                return telemetry ? telemetry->mAngularError : 0.0f;
            };
            const auto quaternion = [&stream](const std::optional<JPH::Quat>& value) {
                if (value)
                    stream << ',' << value->GetX() << ',' << value->GetY() << ',' << value->GetZ() << ','
                           << value->GetW();
                else
                    stream << ",,,,";
            };
            stream << ',' << error(frame.mSwingHipMotorTarget) << ',' << error(frame.mSwingKneeMotorTarget) << ','
                   << error(frame.mSwingFootJointMotorTarget) << ','
                   << (frame.mSwingHipMotorTarget ? frame.mSwingHipMotorTarget->mTargetClampDelta : 0.0f) << ','
                   << (frame.mSwingFootJointMotorTarget
                               ? frame.mSwingFootJointMotorTarget->mTargetClampDelta
                               : 0.0f);
            quaternion(frame.mSwingHipMotorTarget
                    ? frame.mSwingHipMotorTarget->mCurrentOrientation
                    : std::optional<JPH::Quat>{});
            quaternion(frame.mSwingHipMotorTarget
                    ? frame.mSwingHipMotorTarget->mRequestedOrientation
                    : std::optional<JPH::Quat>{});
            quaternion(frame.mSwingHipMotorTarget
                    ? frame.mSwingHipMotorTarget->mTargetOrientation
                    : std::optional<JPH::Quat>{});
            const auto optionalScalar = [&stream](const std::optional<float>& value) {
                if (value)
                    stream << ',' << *value;
                else
                    stream << ',';
            };
            optionalScalar(frame.mSwingHipMotorTarget
                    ? frame.mSwingHipMotorTarget->mNormalHalfConeAngle
                    : std::optional<float>{});
            optionalScalar(frame.mSwingHipMotorTarget
                    ? frame.mSwingHipMotorTarget->mPlaneHalfConeAngle
                    : std::optional<float>{});
            optionalScalar(frame.mSwingHipMotorTarget
                    ? frame.mSwingHipMotorTarget->mTwistMinimum
                    : std::optional<float>{});
            optionalScalar(frame.mSwingHipMotorTarget
                    ? frame.mSwingHipMotorTarget->mTwistMaximum
                    : std::optional<float>{});
            stream << ',' << frame.mSwingHipMotorPeakAxisUtilization << ','
                   << frame.mSwingKneeMotorPeakAxisUtilization << ','
                   << frame.mSwingFootJointMotorPeakAxisUtilization;
            if (frame.mSwingKneeMotorTarget && frame.mSwingKneeMotorTarget->mCurrentHingeAngle
                && frame.mSwingKneeMotorTarget->mTargetHingeAngle && frame.mSwingKneeMotorTarget->mHingeMinimum
                && frame.mSwingKneeMotorTarget->mHingeMaximum)
            {
                stream << ',' << *frame.mSwingKneeMotorTarget->mCurrentHingeAngle << ','
                       << *frame.mSwingKneeMotorTarget->mTargetHingeAngle << ','
                       << *frame.mSwingKneeMotorTarget->mHingeMinimum << ','
                       << *frame.mSwingKneeMotorTarget->mHingeMaximum;
            }
            else
                stream << ",,,,";
            stream << ',' << gaitActuationStageName(frame.mActuationStage) << ','
                   << gaitActuationStageName(frame.mUprightActuationStage) << ','
                   << frame.mStanceHipAxisReleased << ',' << frame.mSwingHipAxisReleased << ','
                   << (frame.mSwingFootJointMotorTarget && frame.mSwingFootJointMotorTarget->mWorldOrientation) << ','
                   << frame.mBackwardRecoveryStep << ',';
            const auto& impulse = frame.mSolverImpulse.value_or(RagdollSolverImpulse{});
            const auto& cop = frame.mSolverCenterOfPressure.value_or(JPH::RVec3::sZero());
            stream << frame.mSolverImpulse.has_value() << ',' << impulse.mLinearImpulse.GetX() << ','
                   << impulse.mLinearImpulse.GetY() << ',' << impulse.mLinearImpulse.GetZ() << ','
                   << impulse.mAngularImpulse.GetX() << ',' << impulse.mAngularImpulse.GetY() << ','
                   << impulse.mAngularImpulse.GetZ() << ',' << impulse.mInternalAnchorImpulseMoment.GetX() << ','
                   << impulse.mInternalAnchorImpulseMoment.GetY() << ',' << impulse.mInternalAnchorImpulseMoment.GetZ() << ','
                   << impulse.mVelocityLimitTouched << ',' << impulse.mActivationChanged << ','
                   << impulse.mContinuousCollisionEnabled << ','
                   << frame.mSolverCenterOfPressure.has_value() << ',' << cop.GetX() << ',' << cop.GetY() << ','
                   << cop.GetZ() << ',' << impulse.mReferenceCenterOfMass.GetX() << ','
                   << impulse.mReferenceCenterOfMass.GetY() << ',' << impulse.mReferenceCenterOfMass.GetZ();
            const auto vector = [&](JPH::Vec3Arg value) {
                stream << ',' << value.GetX() << ',' << value.GetY() << ',' << value.GetZ();
            };
            for (std::size_t joint = 0; joint < frame.mStanceLegMotorAngularImpulses.size(); ++joint)
            {
                vector(frame.mStanceLegMotorAngularImpulses[joint]);
                stream << ',' << frame.mStanceLegMotorPeakAxisUtilizations[joint];
            }
            vector(frame.mSwingHipMotorAngularImpulse);
            vector(frame.mSwingKneeMotorAngularImpulse);
            vector(frame.mSwingFootJointMotorAngularImpulse);
            vector(frame.mAppliedPelvisTorque);
            vector(frame.mAppliedSpineTorque);
            stream << ',' << frame.mMomentum.has_value();
            if (frame.mMomentum)
                vector(frame.mMomentum->mAngularMomentum);
            else
                stream << ",,,";
            stream << ',' << impulse.mFreeVelocityLimitTouched;
            vector(impulse.mFreeLinearLimitImpulse);
            vector(impulse.mFreeAngularLimitImpulse);
            stream << ',' << frame.mMaxAnchorSeparation << ','
                   << (frame.mJointSupport && frame.mJointSupport->mSupported);
            if (frame.mJointSupport && frame.mJointSupport->mHullProjectionDelta)
                stream << ',' << frame.mJointSupport->mHullProjectionDelta->GetX()
                       << ',' << frame.mJointSupport->mHullProjectionDelta->GetY()
                       << ',' << frame.mJointSupport->mFrictionLimited;
            else
                stream << ",,,";
            vector(frame.mJointSupport ? frame.mJointSupport->mAcceleration : JPH::Vec3::sZero());
            vector(frame.mJointSupport ? frame.mJointSupport->mRootForceResidual : JPH::Vec3::sZero());
            vector(frame.mJointSupport ? frame.mJointSupport->mRootMomentResidual : JPH::Vec3::sZero());
            if (frame.mReferenceGait)
            {
                const ReferenceGaitFrameRecord& record = *frame.mReferenceGait;
                stream << ',' << referenceGaitStageName(record.mStage) << ',' << record.mTimeInStage << ','
                       << record.mSwingPoseApplied;
                if (record.mPlacement)
                    stream << ',' << record.mPlacement->GetX() << ',' << record.mPlacement->GetY() << ','
                           << record.mPlacement->GetZ();
                else
                    stream << ",,,";
                stream << ',' << record.mRequestedAcceleration.GetX() << ',' << record.mRequestedAcceleration.GetY();
                vector(record.mPelvisOrientationError);
                vector(record.mBranchReaction);
                vector(record.mStanceHipTorque);
                stream << ',' << record.mLeftLoadFraction << ','
                       << record.mRightLoadFraction << ',' << record.mPlannedStanceFootLoad << ','
                       << record.mMaxMotorUtilization << ',' << record.mMaxMotorUtilizationBody;
                vector(record.mStanceHipLoadTorque);
                vector(record.mStanceHipMotorTorque);
                vector(record.mStanceAnkleLoadTorque);
                vector(record.mStanceAnkleMotorTorque);
                stream << ',' << record.mTransferVelocity << ',' << record.mTransferReferenceVelocity << ','
                       << record.mSwingReachLimited;
                vector(record.mSwingHipFeedForwardTorque);
                vector(record.mSwingKneeFeedForwardTorque);
                vector(record.mSwingHipRateTarget);
                vector(record.mSwingHipMeasuredRate);
                if (record.mPlacementCenterOfMass)
                    stream << ',' << record.mPlacementCenterOfMass->GetX() << ','
                           << record.mPlacementCenterOfMass->GetY();
                else
                    stream << ",,";
                stream << ',' << record.mPlacementVelocity.GetX() << ',' << record.mPlacementVelocity.GetY();
                stream << ',' << record.mFeasibilityClampedCount;
                stream << ',' << record.mStanceKneeTargetJump << ',' << record.mStanceKneeModelTargetJumpLegacy << ',';
                if (record.mStanceKneeHingeTarget)
                    stream << *record.mStanceKneeHingeTarget;
            }
            else
                stream << ",,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,";
            // World heading (yaw about +Z, zero along +Y) of the pelvis and
            // feet, and the contact count under each foot.
            const auto heading = [&](std::size_t body) {
                if (body >= frame.mBodies.size())
                    return 0.0f;
                const JPH::Vec3 facing = frame.mBodies[body].mWorldRotation * JPH::Vec3::sAxisY();
                return std::atan2(-facing.GetX(), facing.GetY());
            };
            stream << ',' << heading(result.mLandmarks.mPelvis) << ',' << heading(result.mLandmarks.mLeftFoot) << ','
                   << heading(result.mLandmarks.mRightFoot);
            for (const std::size_t foot : { result.mLandmarks.mLeftFoot, result.mLandmarks.mRightFoot })
                stream << ',' << std::count_if(frame.mContacts.begin(), frame.mContacts.end(),
                    [foot](const GroundContact& contact) { return contact.mBodyIndex == foot && contact.hasSupport(); });
            // Foot tilt from vertical (radians) and the impulse-weighted
            // center of pressure under each foot from the solved contacts.
            for (const std::size_t foot : { result.mLandmarks.mLeftFoot, result.mLandmarks.mRightFoot })
            {
                float roll = 0.0f;
                if (foot < frame.mBodies.size())
                {
                    const JPH::Vec3 up = frame.mBodies[foot].mWorldRotation * JPH::Vec3::sAxisZ();
                    roll = std::acos(std::clamp(up.GetZ(), -1.0f, 1.0f));
                }
                stream << ',' << roll;
            }
            for (const std::size_t foot : { result.mLandmarks.mLeftFoot, result.mLandmarks.mRightFoot })
            {
                double weight = 0.0, x = 0.0, y = 0.0;
                for (const auto& contact : frame.mContacts)
                {
                    if (contact.mBodyIndex != foot || !contact.mNormalImpulse || !std::isfinite(*contact.mNormalImpulse)
                        || *contact.mNormalImpulse <= 0.0f)
                        continue;
                    weight += *contact.mNormalImpulse;
                    x += *contact.mNormalImpulse * static_cast<double>(contact.mPosition.GetX());
                    y += *contact.mNormalImpulse * static_cast<double>(contact.mPosition.GetY());
                }
                stream << ',';
                if (weight > 0.0)
                    stream << x / weight;
                stream << ',';
                if (weight > 0.0)
                    stream << y / weight;
            }
            // Report solved contact loads, not a geometric support flag.
            // A missing/invalid impulse makes that foot's total unknown; a
            // foot with no contacts has a known zero total.
            for (const std::size_t foot : { result.mLandmarks.mLeftFoot, result.mLandmarks.mRightFoot })
            {
                double total = 0.0;
                bool known = true;
                for (const auto& contact : frame.mContacts)
                {
                    if (contact.mBodyIndex != foot)
                        continue;
                    if (!contact.mNormalImpulse || !std::isfinite(*contact.mNormalImpulse)
                        || *contact.mNormalImpulse < 0.0f)
                    {
                        known = false;
                        break;
                    }
                    total += *contact.mNormalImpulse;
                }
                stream << ',';
                if (known)
                    stream << total;
            }
            for (const std::size_t foot : { result.mLandmarks.mLeftFoot, result.mLandmarks.mRightFoot })
            {
                std::optional<float> minimum, maximum;
                bool known = true;
                for (const auto& contact : frame.mContacts)
                {
                    if (contact.mBodyIndex != foot)
                        continue;
                    if (!contact.mNormalImpulse || !std::isfinite(*contact.mNormalImpulse) || *contact.mNormalImpulse < 0)
                    {
                        known = false;
                        break;
                    }
                    if (*contact.mNormalImpulse == 0)
                        continue;
                    if (!contact.mPointVelocity || !isFinite(*contact.mPointVelocity))
                    {
                        known = false;
                        break;
                    }
                    const float velocity = contact.mPointVelocity->GetZ();
                    minimum = minimum ? std::min(*minimum, velocity) : velocity;
                    maximum = maximum ? std::max(*maximum, velocity) : velocity;
                }
                stream << ',';
                if (known && minimum)
                    stream << *minimum;
                stream << ',';
                if (known && maximum)
                    stream << *maximum;
            }
            if (hasSupportMotion)
            {
                if (frame.mSupportMotion)
                {
                    const auto& motion = *frame.mSupportMotion;
                    stream << ',' << motion.mPosition.GetX() << ',' << motion.mPosition.GetY() << ',' << motion.mPosition.GetZ();
                    vector(motion.mVelocity);
                    vector(motion.mAcceleration);
                    stream << ',';
                    if (motion.mSupportFoot)
                        stream << *motion.mSupportFoot;
                }
                else
                    stream << ",,,,,,,,,,";
            }
            if (result.mSettings.mReferenceGaitCoordinatesLegPose)
            {
                stream << ',';
                if (frame.mReferenceGait && frame.mReferenceGait->mCoordinatedPelvisTargetZ)
                    stream << *frame.mReferenceGait->mCoordinatedPelvisTargetZ;
            }
            stream << ',' << (frame.mReferenceGait && frame.mReferenceGait->mCoordinatedPelvisTargetZ.has_value());
            // Reference planning can switch stance after the preceding gait
            // observation. Index solved utilization by this command's body,
            // not the generic stance arrays populated from that observation.
            if (frame.mReferenceGait)
            {
                for (const auto body : { frame.mReferenceGait->mStanceHipBody,
                         frame.mReferenceGait->mStanceAnkleBody })
                {
                    stream << ',' << body << ',';
                    if (body < frame.mJointSupportMotorUtilizations.size())
                        stream << frame.mJointSupportMotorUtilizations[body];
                }
            }
            else stream << ",,,,";

            // These are the state that selected the applied plan, not the
            // post-solve phase. Unknown/inactive reference plans stay blank.
            stream << ',';
            if (frame.mReferenceGait)
                stream << frame.mReferenceGait->mEmergencyStep;
            stream << ',';
            if (frame.mReferenceGait)
                stream << frame.mReferenceGait->mEmergencyStepCount;
            // World material-point speed. It measures slip only for a static
            // support surface; moving-ground users must subtract its velocity.
            for (const std::size_t foot : { result.mLandmarks.mLeftFoot, result.mLandmarks.mRightFoot })
            {
                std::optional<float> maximum;
                bool known = true;
                for (const auto& contact : frame.mContacts)
                {
                    if (contact.mBodyIndex != foot)
                        continue;
                    if (!contact.mNormalImpulse || !std::isfinite(*contact.mNormalImpulse)
                        || *contact.mNormalImpulse < 0)
                    {
                        known = false;
                        break;
                    }
                    if (*contact.mNormalImpulse == 0)
                        continue;
                    if (!contact.mPointVelocity || !isFinite(*contact.mPointVelocity))
                    {
                        known = false;
                        break;
                    }
                    const auto velocity = *contact.mPointVelocity;
                    const float speed = std::hypot(velocity.GetX(), velocity.GetY());
                    maximum = maximum ? std::max(*maximum, speed) : speed;
                }
                stream << ',';
                if (known && maximum)
                    stream << *maximum;
            }
            // Ratio is per contact, never the magnitude of summed impulses.
            // A zero normal load has no defined ratio and is skipped.
            for (const std::size_t foot : { result.mLandmarks.mLeftFoot, result.mLandmarks.mRightFoot })
            {
                std::optional<float> peak;
                bool known = true;
                for (const auto& contact : frame.mContacts)
                {
                    if (contact.mBodyIndex != foot)
                        continue;
                    if (!contact.mNormalImpulse || !std::isfinite(*contact.mNormalImpulse)
                        || *contact.mNormalImpulse < 0)
                    {
                        known = false;
                        break;
                    }
                    if (*contact.mNormalImpulse == 0)
                        continue;
                    if (!contact.mFriction || !std::isfinite(*contact.mFriction) || *contact.mFriction <= 0
                        || !contact.mFrictionImpulseMagnitude || !std::isfinite(*contact.mFrictionImpulseMagnitude))
                    {
                        known = false;
                        break;
                    }
                    const float ratio = *contact.mFrictionImpulseMagnitude / (*contact.mFriction * *contact.mNormalImpulse);
                    peak = peak ? std::max(*peak, ratio) : ratio;
                }
                stream << ',';
                if (known && peak)
                    stream << *peak;
            }
            // Columns of the removed contact-root route, kept empty for
            // trace readers: input and output absent, production owner,
            // load weights 1/0, no stance/touchdown body, no hold or capture.
            stream << ",0,0" << std::string(19, ',') << ",0" << std::string(49, ',')
                   << ",production,1,0,," << ",0,,,,,0,,,,";
            stream << ',' << frame.mRootKinematic;
            if (frame.mKinematicRootTarget)
            {
                const auto& root = *frame.mKinematicRootTarget;
                stream << ',' << root.mTranslation.GetX() << ',' << root.mTranslation.GetY()
                       << ',' << root.mTranslation.GetZ() << ',' << root.mRotation.GetX()
                       << ',' << root.mRotation.GetY() << ',' << root.mRotation.GetZ()
                       << ',' << root.mRotation.GetW();
            }
            else stream << ",,,,,,,";
            stream << ',' << frame.mKinematicLocomotion << ',' << static_cast<unsigned>(frame.mHybridStage) << ',';
            if (frame.mRootReleaseVelocityError) stream << *frame.mRootReleaseVelocityError;
            stream << '\n';
        }
        return stream.str();
    }

    std::uint64_t hashScenarioTelemetry(const ScenarioResult& result)
    {
        return fnv1a(encodeScenarioTelemetry(result));
    }

    bool haveIdenticalScenarioTelemetry(const ScenarioResult& expected, const ScenarioResult& actual)
    {
        return encodeScenarioTelemetry(expected) == encodeScenarioTelemetry(actual);
    }

    std::vector<ScenarioGaitStageSummary> summarizeScenarioGaitStages(
        const ScenarioResult& result, bool uprightStages)
    {
        std::vector<ScenarioGaitStageSummary> summaries;
        for (const auto stage : { GaitActuationStage::Inactive, GaitActuationStage::Settling,
                 GaitActuationStage::AwaitingStance, GaitActuationStage::WeightTransfer,
                 GaitActuationStage::Unloading, GaitActuationStage::Swing, GaitActuationStage::Touchdown })
        {
            ScenarioGaitStageSummary summary;
            summary.mStage = stage;
            for (std::size_t i = 0; i < result.mFrames.size(); ++i)
            {
                const ScenarioFrame& frame = result.mFrames[i];
                if ((uprightStages ? frame.mUprightActuationStage : frame.mActuationStage) != stage)
                    continue;
                ++summary.mFrames;
                const JPH::Vec3 requested = horizontal(frame.mRequestedHorizontalVelocity);
                const float speed = requested.Length();
                summary.mMeanRequestedSpeed += speed;
                if (speed > 0.0f)
                {
                    const JPH::Vec3 direction = requested / speed;
                    summary.mMeanAchievedSpeed += frame.mCenterOfMassVelocity.Dot(direction);
                    if (frame.mUprightApplied && frame.mTranslationActuationEnabled
                        && frame.mAppliedPelvisForce.Dot(direction) < 0.0f)
                        ++summary.mBrakingFrames;
                }
                if (!frame.mSupport.hasGroundContact())
                    ++summary.mAirborneFrames;
                if (frame.mController.mMode == ActiveControllerMode::Falling
                    || frame.mController.mMode == ActiveControllerMode::Fallen)
                    ++summary.mFallingFrames;
                if (frame.mAppliedUpright && frame.mUprightApplied && frame.mTranslationActuationEnabled)
                {
                    if (frame.mAppliedUpright->mPelvisVerticalForceSaturated)
                        ++summary.mVerticalSaturationFrames;
                    if (frame.mAppliedUpright->mPelvisSupportMomentSaturated)
                        ++summary.mSupportMomentSaturationFrames;
                }
                if (frame.mMotorsApplied && frame.mSwingHipMotorPeakAxisUtilization >= 0.999f)
                    ++summary.mSwingHipSaturationFrames;
                if (frame.mMotorsApplied && frame.mSwingKneeMotorPeakAxisUtilization >= 0.999f)
                    ++summary.mSwingKneeSaturationFrames;
                if (frame.mMotorsApplied && frame.mSwingFootJointMotorPeakAxisUtilization >= 0.999f)
                    ++summary.mSwingAnkleSaturationFrames;
                if (frame.mSolverImpulse)
                {
                    const auto& impulse = *frame.mSolverImpulse;
                    ++summary.mSolverObservedFrames;
                    summary.mFreeVelocityLimitedFrames += impulse.mFreeVelocityLimitTouched;
                    summary.mSolverVelocityLimitedFrames += impulse.mVelocityLimitTouched;
                    summary.mMaxFreeLinearLimitImpulse = std::max(summary.mMaxFreeLinearLimitImpulse,
                        impulse.mFreeLinearLimitImpulse.Length());
                    summary.mMaxFreeAngularLimitImpulse = std::max(summary.mMaxFreeAngularLimitImpulse,
                        impulse.mFreeAngularLimitImpulse.Length());
                }
                summary.mMaxNetForce = std::max(summary.mMaxNetForce, frame.mAppliedNetForce.Length());
                summary.mMaxNetMoment = std::max(summary.mMaxNetMoment, frame.mAppliedNetActuationMoment.Length());
                if (i > 0)
                {
                    const ScenarioFrame& previous = result.mFrames[i - 1];
                    for (const bool left : { true, false })
                    {
                        const std::size_t foot = left ? result.mLandmarks.mLeftFoot : result.mLandmarks.mRightFoot;
                        const bool supported = left ? frame.mSupport.mLeftFootSupported
                                                    : frame.mSupport.mRightFootSupported;
                        const bool wasSupported = left ? previous.mSupport.mLeftFootSupported
                                                       : previous.mSupport.mRightFootSupported;
                        if (supported && wasSupported && foot < frame.mBodies.size() && foot < previous.mBodies.size())
                            summary.mMaxStanceSlide = std::max(summary.mMaxStanceSlide,
                                horizontal(JPH::Vec3(frame.mBodies[foot].mCenterOfMass
                                    - previous.mBodies[foot].mCenterOfMass)).Length());
                    }
                }
            }
            summary.mDuration = static_cast<float>(summary.mFrames) * result.mSettings.mFixedTimeStep;
            if (summary.mFrames > 0)
            {
                summary.mMeanRequestedSpeed /= static_cast<float>(summary.mFrames);
                summary.mMeanAchievedSpeed /= static_cast<float>(summary.mFrames);
            }
            summaries.push_back(summary);
        }
        return summaries;
    }

    std::string formatScenarioGaitStagesCsv(const ScenarioResult& result)
    {
        std::ostringstream stream;
        stream.imbue(std::locale::classic());
        stream << std::setprecision(std::numeric_limits<float>::max_digits10);
        stream << "policy,stage,frames,duration,requested_speed,achieved_speed,braking_frames,airborne_frames,"
                  "falling_frames,vertical_saturated,moment_saturated,swing_hip_saturated,"
                  "swing_knee_saturated,swing_ankle_saturated,max_net_force,"
                  "max_net_moment,max_stance_slide,solver_observed,free_velocity_limited,solver_velocity_limited,"
                  "max_free_linear_limit_impulse,max_free_angular_limit_impulse\n";
        for (const bool upright : { false, true })
            for (const ScenarioGaitStageSummary& stage : summarizeScenarioGaitStages(result, upright))
                stream << (upright ? "upright" : "motor") << ',' << gaitActuationStageName(stage.mStage) << ','
                       << stage.mFrames << ',' << stage.mDuration << ',' << stage.mMeanRequestedSpeed << ','
                       << stage.mMeanAchievedSpeed << ',' << stage.mBrakingFrames << ',' << stage.mAirborneFrames << ','
                       << stage.mFallingFrames << ',' << stage.mVerticalSaturationFrames << ','
                       << stage.mSupportMomentSaturationFrames << ',' << stage.mSwingHipSaturationFrames << ','
                       << stage.mSwingKneeSaturationFrames << ',' << stage.mSwingAnkleSaturationFrames << ','
                       << stage.mMaxNetForce << ',' << stage.mMaxNetMoment << ',' << stage.mMaxStanceSlide << ','
                       << stage.mSolverObservedFrames << ',' << stage.mFreeVelocityLimitedFrames << ','
                       << stage.mSolverVelocityLimitedFrames << ',' << stage.mMaxFreeLinearLimitImpulse << ','
                       << stage.mMaxFreeAngularLimitImpulse << '\n';
        return stream.str();
    }

    ScenarioInvariants summarizeScenarioInvariants(const ScenarioResult& result)
    {
        ScenarioInvariants invariants;
        invariants.mFrameCount = result.mFrames.size();
        for (const auto& frame : result.mFrames)
        {
            if (!frame.mSolverImpulse)
                continue;
            const auto& impulse = *frame.mSolverImpulse;
            ++invariants.mSolverObservedFrames;
            invariants.mFreeVelocityLimitedFrames += impulse.mFreeVelocityLimitTouched;
            invariants.mSolverVelocityLimitedFrames += impulse.mVelocityLimitTouched;
            invariants.mMaxFreeLinearLimitImpulse = std::max(invariants.mMaxFreeLinearLimitImpulse,
                impulse.mFreeLinearLimitImpulse.Length());
            invariants.mMaxFreeAngularLimitImpulse = std::max(invariants.mMaxFreeAngularLimitImpulse,
                impulse.mFreeAngularLimitImpulse.Length());
        }
        if (result.mFrames.empty())
            return invariants;

        const std::array<std::size_t, 2> feet{ result.mLandmarks.mLeftFoot, result.mLandmarks.mRightFoot };
        std::size_t consecutiveAirborne = 0;
        float requestedSpeedSum = 0.0f;
        float achievedSpeedSum = 0.0f;
        float finalAchievedSum = 0.0f;
        std::size_t finalFrames = 0;
        const float finalWindowStart = result.mFrames.back().mTime - 0.5f;
        std::optional<float> firstImpulseTime;
        std::vector<float> impulseTimes;
        std::optional<float> lastTransitionTime;
        std::optional<GaitPhase> lastPhase;
        float leftDurationSum = 0.0f;
        float rightDurationSum = 0.0f;
        std::size_t leftDurations = 0;
        std::size_t rightDurations = 0;
        bool previousTimedOut = false;

        for (std::size_t i = 0; i < result.mFrames.size(); ++i)
        {
            const ScenarioFrame& frame = result.mFrames[i];
            if (frame.mSupport.hasGroundContact())
                consecutiveAirborne = 0;
            else
            {
                ++invariants.mAirborneFrames;
                ++consecutiveAirborne;
                invariants.mMaxConsecutiveAirborneFrames
                    = std::max(invariants.mMaxConsecutiveAirborneFrames, consecutiveAirborne);
            }
            // Saturation is an actuation invariant. The controller may still
            // compute a dry-run correction while falling or fallen, but drive
            // is disabled in those modes and that output cannot saturate the
            // simulated ragdoll.
            if (frame.mUpright && frame.mUprightApplied)
            {
                if (frame.mUpright->mPelvisForceSaturated)
                    ++invariants.mPelvisForceSaturatedFrames;
                if (frame.mUpright->mPelvisMagnitudeSaturated)
                    ++invariants.mPelvisMagnitudeSaturatedFrames;
                if (frame.mUpright->mPelvisHorizontalForceSaturated)
                    ++invariants.mPelvisHorizontalForceSaturatedFrames;
                if (frame.mUpright->mPelvisVerticalForceSaturated)
                    ++invariants.mPelvisVerticalForceSaturatedFrames;
                if (frame.mUpright->mPelvisTorqueSaturated)
                    ++invariants.mPelvisTorqueSaturatedFrames;
                if (frame.mUpright->mSpineTorqueSaturated)
                    ++invariants.mSpineTorqueSaturatedFrames;
            }
            invariants.mMaxAppliedNetForce = std::max(invariants.mMaxAppliedNetForce, frame.mAppliedNetForce.Length());
            invariants.mMaxForcePairMoment
                = std::max(invariants.mMaxForcePairMoment, frame.mAppliedForcePairMoment.Length());
            invariants.mMaxSwingForcePairMoment
                = std::max(invariants.mMaxSwingForcePairMoment, frame.mAppliedSwingForcePairMoment.Length());
            invariants.mMaxUprightForcePairMoment
                = std::max(invariants.mMaxUprightForcePairMoment, frame.mAppliedUprightForcePairMoment.Length());
            invariants.mMaxGroundReactionForce
                = std::max(invariants.mMaxGroundReactionForce, frame.mAppliedGroundReactionForce.Length());
            invariants.mMaxGroundReactionMoment
                = std::max(invariants.mMaxGroundReactionMoment, frame.mAppliedGroundReactionMoment.Length());
            invariants.mMaxNetActuationMoment
                = std::max(invariants.mMaxNetActuationMoment, frame.mAppliedNetActuationMoment.Length());
            invariants.mMaxSwingHipMotorTorque = std::max(invariants.mMaxSwingHipMotorTorque,
                frame.mSwingHipMotorAngularImpulse.Length() / result.mSettings.mFixedTimeStep);
            invariants.mMaxSwingHipMotorPeakAxisUtilization
                = std::max(invariants.mMaxSwingHipMotorPeakAxisUtilization, frame.mSwingHipMotorPeakAxisUtilization);
            if (frame.mSwingHipMotorPeakAxisUtilization >= 0.999f)
                ++invariants.mSwingHipMotorSaturatedFrames;
            if (frame.mSwingHipWorldTorqueSaturated)
                ++invariants.mSwingHipWorldTorqueSaturatedFrames;
            invariants.mMaxSwingHipWorldTorque
                = std::max(invariants.mMaxSwingHipWorldTorque, frame.mAppliedSwingHipWorldTorque.Length());
            invariants.mMaxStanceHipCompensationTorque = std::max(invariants.mMaxStanceHipCompensationTorque,
                frame.mAppliedStanceHipCompensationImpulse.Length() / result.mSettings.mFixedTimeStep);
            if (i > 0)
            {
                const ScenarioFrame& previous = result.mFrames[i - 1];
                for (std::size_t side = 0; side < feet.size(); ++side)
                {
                    const bool wasSupported
                        = side == 0 ? previous.mSupport.mLeftFootSupported : previous.mSupport.mRightFootSupported;
                    const bool isSupported
                        = side == 0 ? frame.mSupport.mLeftFootSupported : frame.mSupport.mRightFootSupported;
                    if (!wasSupported || !isSupported || feet[side] >= frame.mBodies.size()
                        || feet[side] >= previous.mBodies.size())
                        continue;
                    const JPH::Vec3 travel = horizontal(JPH::Vec3(
                        frame.mBodies[feet[side]].mCenterOfMass - previous.mBodies[feet[side]].mCenterOfMass));
                    const float slide = travel.Length();
                    invariants.mMaxStanceFootSlide = std::max(invariants.mMaxStanceFootSlide, slide);
                    invariants.mTotalStanceFootSlide += slide;
                }
            }

            const JPH::Vec3 requested = horizontal(frame.mRequestedHorizontalVelocity);
            const float requestedSpeed = requested.Length();
            if (requestedSpeed > 1e-3f)
            {
                ++invariants.mRequestedFrames;
                requestedSpeedSum += requestedSpeed;
                const float achieved = horizontal(frame.mCenterOfMassVelocity).Dot(requested / requestedSpeed);
                achievedSpeedSum += achieved;
                invariants.mMaxTrackingError = std::max(
                    invariants.mMaxTrackingError, (horizontal(frame.mCenterOfMassVelocity) - requested).Length());
                if (frame.mTime > finalWindowStart)
                {
                    finalAchievedSum += achieved;
                    ++finalFrames;
                }
            }

            if (frame.mGait.mTransitioned && frame.mGait.hasStance())
            {
                if (lastTransitionTime && lastPhase)
                {
                    const float duration = frame.mTime - *lastTransitionTime;
                    if (*lastPhase == GaitPhase::LeftStance)
                    {
                        leftDurationSum += duration;
                        ++leftDurations;
                    }
                    else if (*lastPhase == GaitPhase::RightStance)
                    {
                        rightDurationSum += duration;
                        ++rightDurations;
                    }
                }
                if (frame.mGait.mPhase == GaitPhase::LeftStance)
                    ++invariants.mSteps.mLeftStanceCount;
                else
                    ++invariants.mSteps.mRightStanceCount;
                for (const float impulseTime : impulseTimes)
                {
                    if (frame.mTime > impulseTime && frame.mTime <= impulseTime + 1.0f)
                    {
                        ++invariants.mStepsWithinSecondAfterImpulse;
                        break;
                    }
                }
                lastTransitionTime = frame.mTime;
                lastPhase = frame.mGait.mPhase;
            }
            if (frame.mGait.mPhaseTimedOut && !previousTimedOut)
                ++invariants.mSteps.mTimedOutPhases;
            previousTimedOut = frame.mGait.mPhaseTimedOut;

            if (frame.mController.mMode == ActiveControllerMode::Falling)
                ++invariants.mFallingFrames;
            if (frame.mController.mMode == ActiveControllerMode::Fallen)
                ++invariants.mFallenFrames;
            invariants.mMinPelvisUpDot = std::min(invariants.mMinPelvisUpDot, frame.mPelvisUpDot);
            for (const ScenarioBodyRecord& body : frame.mBodies)
                invariants.mMaxBodySpeed = std::max(invariants.mMaxBodySpeed, body.mLinearVelocity.Length());

            if (frame.mImpulseCount > 0)
            {
                ++invariants.mImpulseFrames;
                impulseTimes.push_back(frame.mTime);
                if (!firstImpulseTime)
                    firstImpulseTime = frame.mTime;
            }
            if (firstImpulseTime && frame.mTime > *firstImpulseTime
                && (frame.mController.mMode == ActiveControllerMode::Falling
                    || frame.mController.mMode == ActiveControllerMode::Fallen))
                invariants.mFellAfterImpulse = true;
        }

        if (invariants.mRequestedFrames > 0)
        {
            invariants.mMeanRequestedSpeed = requestedSpeedSum / static_cast<float>(invariants.mRequestedFrames);
            invariants.mMeanAchievedSpeed = achievedSpeedSum / static_cast<float>(invariants.mRequestedFrames);
        }
        if (finalFrames > 0)
            invariants.mFinalHalfSecondAchievedSpeed = finalAchievedSum / static_cast<float>(finalFrames);
        if (leftDurations > 0)
            invariants.mSteps.mMeanLeftStanceDuration = leftDurationSum / static_cast<float>(leftDurations);
        if (rightDurations > 0)
            invariants.mSteps.mMeanRightStanceDuration = rightDurationSum / static_cast<float>(rightDurations);
        const float longer
            = std::max(invariants.mSteps.mMeanLeftStanceDuration, invariants.mSteps.mMeanRightStanceDuration);
        const float shorter
            = std::min(invariants.mSteps.mMeanLeftStanceDuration, invariants.mSteps.mMeanRightStanceDuration);
        invariants.mSteps.mDurationSymmetry = longer > 0.0f ? shorter / longer : 0.0f;
        return invariants;
    }

    ScenarioGaitMilestones summarizeScenarioGaitMilestones(const ScenarioResult& result)
    {
        ScenarioGaitMilestones milestones;
        bool lifted = false;
        bool haveClosestApproach = false;
        for (const ScenarioFrame& frame : result.mFrames)
        {
            if (!milestones.mStanceStartedStep && frame.mGait.hasStance())
                milestones.mStanceStartedStep = frame.mStep;
            if (!milestones.mStanceReadyStep && frame.mGait.mStanceReady)
                milestones.mStanceReadyStep = frame.mStep;
            if (!milestones.mLiftOffStep && frame.mGait.mSwingFootLifted)
                milestones.mLiftOffStep = frame.mStep;
            lifted = lifted || frame.mGait.mSwingFootLifted;
            if (lifted && frame.mGait.mSwingContactCandidateTime > 0.0f && !milestones.mContactCandidateStep)
                milestones.mContactCandidateStep = frame.mStep;
            if (lifted && frame.mGait.mTransitioned && frame.mGait.hasStance() && !frame.mGait.mPhaseTimedOut
                && !milestones.mStanceTransferStep)
                milestones.mStanceTransferStep = frame.mStep;
            if (frame.mGait.mPhaseTimedOut && !milestones.mPhaseTimeoutStep)
                milestones.mPhaseTimeoutStep = frame.mStep;
            if (lifted && frame.mSwingFootTarget
                && (!haveClosestApproach
                    || frame.mSwingFootHorizontalPlacementError < milestones.mClosestApproachError))
            {
                milestones.mClosestApproachStep = frame.mStep;
                milestones.mClosestApproachError = frame.mSwingFootHorizontalPlacementError;
                haveClosestApproach = true;
            }
        }
        return milestones;
    }

    std::vector<ScenarioGaitPhaseMilestones> summarizeScenarioGaitPhases(const ScenarioResult& result)
    {
        std::vector<ScenarioGaitPhaseMilestones> phases;
        bool continuingStance = false;
        for (const ScenarioFrame& frame : result.mFrames)
        {
            if (!frame.mGait.hasStance())
            {
                continuingStance = false;
                continue;
            }
            if (!continuingStance || frame.mGait.mPhase != phases.back().mPhase)
            {
                if (continuingStance && frame.mGait.mTransitioned)
                    phases.back().mStanceTransferStep = frame.mStep;
                ScenarioGaitPhaseMilestones phase;
                phase.mPhase = frame.mGait.mPhase;
                phase.mStanceStartedStep = frame.mStep;
                phases.push_back(phase);
            }
            continuingStance = true;

            ScenarioGaitPhaseMilestones& phase = phases.back();
            if (!phase.mStanceReadyStep && frame.mGait.mStanceReady)
                phase.mStanceReadyStep = frame.mStep;
            if (!phase.mLiftOffStep && frame.mGait.mSwingFootLifted)
                phase.mLiftOffStep = frame.mStep;
            if (phase.mLiftOffStep && !phase.mContactCandidateStep
                && frame.mGait.mSwingContactCandidateTime > 0.0f)
                phase.mContactCandidateStep = frame.mStep;
            if (!phase.mPhaseTimeoutStep && frame.mGait.mPhaseTimedOut)
                phase.mPhaseTimeoutStep = frame.mStep;
            if (phase.mLiftOffStep && frame.mSwingFootTarget
                && (!phase.mClosestApproachStep
                    || frame.mSwingFootHorizontalPlacementError < phase.mClosestApproachError))
            {
                phase.mClosestApproachStep = frame.mStep;
                phase.mClosestApproachError = frame.mSwingFootHorizontalPlacementError;
            }
        }
        return phases;
    }
}

#include "ragdolltestbindings.hpp"
#include <ragdollcore/tuning.hpp>
#include "../manualdiagnostics.hpp"
#include "../teststepscheduler.hpp"

#ifdef OPENMW_RAGDOLL_DEV_API

#include <atomic>
#include <cstdlib>
#include <charconv>
#include <string_view>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

#include <sol/sol.hpp>
#include <SDL.h>

#include <components/lua/luastate.hpp>
#include <components/lua/utilpackage.hpp>
#include <components/misc/finitevalues.hpp>
#include <components/ragdollprofile/skeletonadapter.hpp>

#include "context.hpp"
#include "luamanagerimp.hpp"
#include "object.hpp"

#include "../mwbase/environment.hpp"
#include "../mwbase/mechanicsmanager.hpp"
#include "../mwbase/world.hpp"
#include "../mwmechanics/combat.hpp"
#include "../mwmechanics/creaturestats.hpp"
#include "../mwinput/actions.hpp"
#include "../mwinput/inputmanagerimp.hpp"
#include "../mwphysics/physicssystem.hpp"
#include "../mwrender/animation.hpp"
#include "../mwrender/npcanimation.hpp"
#include "../mwrender/woundfx.hpp"
#include "../mwworld/class.hpp"
#include "../mwworld/worldimp.hpp"

namespace MWLua
{
    namespace
    {
        MWWorld::World& getWorld()
        {
            auto* world
                = dynamic_cast<MWWorld::World*>(static_cast<MWBase::World*>(MWBase::Environment::get().getWorld()));
            if (world == nullptr)
                throw std::runtime_error("Ragdoll test API requires the OpenMW world implementation");
            return *world;
        }

        MWPhysics::PhysicsSystem& getPhysicsSystem()
        {
            return getWorld().getPhysicsSystem();
        }

        sol::table balanceStateTable(sol::this_state thisState, const MWPhysics::PassiveRagdollBalanceState& state)
        {
            const auto availability = [](RagdollCore::BalanceSupportAvailability value) {
                switch (value)
                {
                    case RagdollCore::BalanceSupportAvailability::Supported: return "supported";
                    case RagdollCore::BalanceSupportAvailability::Unsupported: return "unsupported";
                    case RagdollCore::BalanceSupportAvailability::Unknown: return "unknown";
                }
                return "unknown";
            };
            const auto vector = [&](const auto& value) {
                // Lua numbers retain world-coordinate precision; these are
                // plain coordinate tables, not mutable physics vectors.
                sol::table result(thisState, sol::create);
                result["x"] = value.GetX();
                result["y"] = value.GetY();
                result["z"] = value.GetZ();
                return result;
            };
            const auto optional = [](sol::table& table, const char* name, const auto& value) {
                table[std::string(name) + "Known"] = value.has_value();
                if (value)
                    table[name] = *value;
            };
            const auto support = [&](const RagdollCore::BalanceSupportMeasurement& value) {
                sol::table result(thisState, sol::create);
                result["availability"] = availability(value.mAvailability);
                result["leftFoot"] = value.mLeftFootSupported;
                result["rightFoot"] = value.mRightFootSupported;
                optional(result, "horizontalDistance", value.mHorizontalDistance);
                optional(result, "minimumContactHeight", value.mMinimumContactHeight);
                optional(result, "maximumContactHeight", value.mMaximumContactHeight);
                if (value.mClosestPoint)
                    result["closestPoint"] = vector(*value.mClosestPoint);
                return result;
            };
            sol::table result(thisState, sol::create);
            result["schemaVersion"] = 2;
            result["actorSerial"] = std::to_string(state.mActorSerial);
            result["physicsStep"] = std::to_string(state.mPhysicsStep);
            result["lifecycleEpoch"] = std::to_string(state.mLifecycleEpoch);
            result["lastInvalidation"] = state.mLastInvalidation;
            sol::table intent(thisState, sol::create);
            intent["schemaVersion"] = 1;
            intent["availability"] = state.mIntent.mAvailability;
            const auto intentId = [&](const char* name, const auto& value) {
                intent[std::string(name) + "Known"] = value.has_value();
                if (value)
                    intent[name] = std::to_string(*value);
            };
            const auto intentVector = [&](const char* name, const auto& value) {
                intent[std::string(name) + "Known"] = value.has_value();
                if (value)
                    intent[name] = vector(*value);
            };
            intentId("captureSerial", state.mIntent.mCaptureSerial);
            intentId("captureActorSerial", state.mIntent.mCaptureActorSerial);
            intentId("captureLifecycleEpoch", state.mIntent.mCaptureLifecycleEpoch);
            optional(intent, "captureRenderFrame", state.mIntent.mCaptureRenderFrame);
            intentId("firstPossiblePhysicsStep", state.mIntent.mFirstPossiblePhysicsStep);
            intentId("consumedPhysicsStep", state.mIntent.mConsumedPhysicsStep);
            intentVector("queuedWorldVelocity", state.mIntent.mQueuedWorldVelocity);
            intentVector("processedRawHorizontalVelocity", state.mIntent.mProcessedRawHorizontalVelocity);
            intentVector("filteredCommandVelocity", state.mIntent.mFilteredCommandVelocity);
            optional(intent, "locomotionInterrupted", state.mIntent.mLocomotionInterrupted);
            result["intent"] = intent;
            result["actorRecord"] = state.mActorRecord;
            optional(result, "replaySeed", state.mReplaySeed);
            result["testStepMultiplier"] = state.mTestStepMultiplier;
            result["duration"] = state.mDuration;
            result["updateError"] = state.mUpdateError;
            result["powered"] = state.mPowered;
            result["rootKinematic"] = state.mRootKinematic;
            result["authorityEnabled"] = state.mAuthorityEnabled;
            result["authority"] = state.mAuthority;
            result["authorityApplied"] = state.mAuthorityApplied;
            result["authorityDemand"] = state.mAuthorityDemand;
            result["authorityBaseline"] = state.mAuthorityBaseline;
            result["authorityBaselineAvailable"] = state.mAuthorityBaselineAvailable;
            result["authorityDisturbance"] = state.mAuthorityDisturbance;
            result["authoritySupportLoss"] = state.mAuthoritySupportLoss;
            result["authorityUnsupportedTime"] = state.mAuthorityUnsupportedTime;
            result["authorityConsumedImpulse"] = state.mAuthorityConsumedImpulse;
            result["authorityConsumedReceiptCount"] = state.mAuthorityConsumedReceiptCount;
            result["authorityFirstConsumedReceipt"] = std::to_string(state.mAuthorityFirstConsumedReceipt);
            result["authorityLastConsumedReceipt"] = std::to_string(state.mAuthorityLastConsumedReceipt);
            result["authorityMissingPhysicsSteps"] = std::to_string(state.mAuthorityMissingPhysicsSteps);
            result["authorityStatus"] = state.mAuthorityStatus;
            result["authorityCorrection"] = state.mAuthorityCorrection;
            result["authorityCorrectedLegMask"] = state.mAuthorityCorrectedLegMask;
            result["route"] = state.mRoute;
            result["modeBeforeObservation"] = state.mModeBeforeObservation;
            result["rootReactionOffset"] = vector(state.mRootReactionOffset);
            result["rootReactionVelocity"] = vector(state.mRootReactionVelocity);
            result["gravity"] = vector(state.mGravity);
            result["weaponRecord"] = state.mWeaponRecord;
            result["weaponAttached"] = state.mWeaponAttached;
            if (state.mExcludedDetachedWeapon)
                result["excludedDetachedWeaponBody"] = *state.mExcludedDetachedWeapon + 1;
            result["massKnown"] = state.mMass.has_value();
            if (state.mMass)
            {
                result["totalMass"] = state.mMass->mTotalMass;
                result["centerOfMass"] = vector(state.mMass->mCenterOfMass);
                result["centerOfMassVelocity"] = vector(state.mMass->mCenterOfMassVelocity);
            }
            sol::table massBodies(thisState, sol::create);
            for (const auto& body : state.mMassBodies)
            {
                sol::table item(thisState, sol::create);
                item["bodyIndex"] = body.mBodyIndex + 1;
                item["bodyId"] = std::to_string(body.mBodyId.GetIndexAndSequenceNumber());
                item["mass"] = body.mMass;
                item["active"] = body.mActive;
                item["declaredKinematicRoot"] = body.mDeclaredKinematicRoot;
                item["centerOfMass"] = vector(body.mCenterOfMass);
                item["velocity"] = vector(body.mVelocity);
                massBodies.add(std::move(item));
            }
            result["massBodies"] = std::move(massBodies);
            result["support"] = support(state.mCenterOfMassSupport);
            result["supportNormalMinimumDot"] = state.mSupportNormalMinimumDot;
            result["capturePointApplicability"] = state.mCapturePointApplicability;
            result["capturePointKnown"] = state.mCapturePoint.has_value();
            optional(result, "supportPlaneHeight", state.mSupportPlaneHeight);
            result["planeNormalTolerance"] = state.mPlaneNormalTolerance;
            result["planeHeightTolerance"] = state.mPlaneHeightTolerance;
            if (state.mCapturePoint)
                result["capturePoint"] = vector(*state.mCapturePoint);
            result["capturePointSupport"] = support(state.mCapturePointSupport);
            sol::table feet(thisState, sol::create);
            sol::table knees(thisState, sol::create);
            for (std::size_t side = 0; side < 2; ++side)
            {
                const char* name = side == 0 ? "left" : "right";
                sol::table foot(thisState, sol::create);
                if (state.mFootBodyIndices[side])
                    foot["bodyIndex"] = *state.mFootBodyIndices[side] + 1;
                if (state.mFootBodyIds[side])
                    foot["bodyId"] = std::to_string(state.mFootBodyIds[side]->GetIndexAndSequenceNumber());
                foot["activeBeforeUpdate"] = state.mFootActiveBeforeUpdate[side];
                foot["geometryComplete"] = state.mFootGeometryComplete[side];
                foot["contactsComplete"] = state.mFootContactsComplete[side];
                foot["reason"] = state.mFootAvailabilityReason[side];
                const auto& slip = state.mFootSlip[side];
                foot["availability"] = availability(slip.mAvailability);
                optional(foot, "normalImpulse", state.mFootNormalImpulse[side]);
                optional(foot, "loadFraction", state.mFootLoadFraction[side]);
                optional(foot, "maximumTangentSpeed", slip.mMaximumTangentSpeed);
                optional(foot, "maximumTangentDistanceThisStep", slip.mMaximumTangentDistanceThisStep);
                optional(foot, "nativeReferenceOriginError", state.mReferenceFootError[side]);
                foot["composedDriveOriginErrorAvailability"] = state.mComposedFootAvailability[side];
                optional(foot, "composedDriveOriginError", state.mComposedFootError[side]);
                if (state.mComposedFootReference[side])
                    foot["composedDriveOriginReference"] = vector(*state.mComposedFootReference[side]);
                if (state.mActualFootOrigin[side])
                    foot["actualOrigin"] = vector(*state.mActualFootOrigin[side]);
                if (state.mComposedFootErrorVector[side])
                    foot["composedDriveOriginErrorVector"] = vector(*state.mComposedFootErrorVector[side]);
                feet[name] = std::move(foot);
                sol::table knee(thisState, sol::create);
                const auto& source = state.mKnees[side];
                if (source.mBodyIndex)
                    knee["bodyIndex"] = *source.mBodyIndex + 1;
                knee["motorAvailability"] = source.mMotorAvailability;
                knee["clipAvailability"] = source.mClipAvailability;
                knee["composedAvailability"] = source.mComposedAvailability;
                knee["animationAvailability"] = source.mAnimationAvailability;
                knee["composedSource"] = source.mComposedSource;
                optional(knee, "evaluatedAnimationAngle", source.mEvaluatedAnimationAngle);
                // Keep full integer identity as decimal text, as physicsStep
                // already does; no precision loss through Lua doubles.
                const auto serial = [&](const char* name, const auto& value) {
                    knee[std::string(name) + "Known"] = value.has_value();
                    if (value) knee[name] = std::to_string(*value);
                };
                serial("animationSampleSerial", source.mAnimationSampleSerial);
                serial("animationSampleNextPhysicsStep", source.mAnimationSampleNextPhysicsStep);
                serial("composedPoseSerial", source.mComposedPoseSerial);
                serial("composedPhysicsStep", source.mComposedPhysicsStep);
                serial("motorObservationPhysicsStep", source.mMotorObservationPhysicsStep);
                optional(knee, "composedAngle", source.mComposedAngle);
                optional(knee, "motorState", source.mMotorState);
                optional(knee, "finalToActualDelta", source.mFinalToActualDelta);
                optional(knee, "actualAngle", source.mActualAngle);
                optional(knee, "requestedMotorAngle", source.mRequestedMotorAngle);
                optional(knee, "finalMotorAngle", source.mFinalMotorAngle);
                optional(knee, "minimumAngle", source.mMinimumAngle);
                optional(knee, "maximumAngle", source.mMaximumAngle);
                optional(knee, "requestedToFinalDelta", source.mRequestedToFinalDelta);
                knees[name] = std::move(knee);
            }
            result["feet"] = std::move(feet);
            result["knees"] = std::move(knees);
            const auto segment = [&](const RagdollCore::BalanceSupportSegment& source) {
                sol::table item(thisState, sol::create);
                item["actorSerial"] = std::to_string(source.mGeneration.mActorSerial);
                item["lifecycleEpoch"] = std::to_string(source.mGeneration.mLifecycleEpoch);
                item["firstPhysicsStep"] = std::to_string(source.mFirstPhysicsStep);
                item["lastPhysicsStep"] = std::to_string(source.mLastPhysicsStep);
                item["onsetObserved"] = source.mOnsetObserved;
                switch (source.mEnd)
                {
                    case RagdollCore::BalanceSupportSegmentEnd::Open: item["end"] = "open"; break;
                    case RagdollCore::BalanceSupportSegmentEnd::Unsupported: item["end"] = "unsupported"; break;
                    case RagdollCore::BalanceSupportSegmentEnd::UnknownSupport: item["end"] = "unknown-support"; break;
                    case RagdollCore::BalanceSupportSegmentEnd::MissingPhysicsSteps: item["end"] = "missing-steps"; break;
                    case RagdollCore::BalanceSupportSegmentEnd::LifecycleInvalidated:
                        item["end"] = "lifecycle-invalidated"; break;
                }
                item["supportedSteps"] = std::to_string(source.mSupportedSteps);
                item["observedSupportDuration"] = source.mObservedSupportDuration;
                item["measuredSlipSteps"] = std::to_string(source.mMeasuredSlipSteps);
                item["observedSlipDuration"] = source.mObservedSlipDuration;
                optional(item, "observedMaximumTangentTravel", source.mObservedMaximumTangentTravel);
                item["slipComplete"] = source.mSlipComplete;
                item["supportEpisodeComplete"] = source.hasCompleteSupportEpisode();
                item["episodeTravelComplete"] = source.hasCompleteEpisodeTravel();
                return item;
            };
            sol::table episodes(thisState, sol::create);
            episodes["schemaVersion"] = 1;
            episodes["consumeStatus"] = state.mEpisodeConsumeStatus;
            episodes["missingSteps"] = std::to_string(state.mEpisodeMissingSteps);
            episodes["totalMissingSteps"] = std::to_string(state.mEpisodeTotalMissingSteps);
            episodes["travelMetric"] = "sum-of-per-step-maximum-tangent-distance";
            episodes["strideAxisSlideAvailability"] = "not-measured-by-scalar-slip-input";
            sol::table episodeFeet(thisState, sol::create);
            for (std::size_t side = 0; side < 2; ++side)
            {
                const auto& foot = state.mFootEpisodes[side];
                sol::table item(thisState, sol::create);
                item["availability"] = availability(foot.mAvailability);
                item["observedUnknownSteps"] = std::to_string(foot.mObservedUnknownSteps);
                item["observedUnknownDuration"] = foot.mObservedUnknownDuration;
                item["currentKnown"] = foot.mCurrentSegment.has_value();
                if (foot.mCurrentSegment)
                    item["current"] = segment(*foot.mCurrentSegment);
                item["closedKnown"] = state.mClosedSupportSegments[side].has_value();
                if (state.mClosedSupportSegments[side])
                    item["closed"] = segment(*state.mClosedSupportSegments[side]);
                item["invalidatedKnown"] = state.mInvalidatedSupportSegments[side].has_value();
                if (state.mInvalidatedSupportSegments[side])
                    item["invalidated"] = segment(*state.mInvalidatedSupportSegments[side]);
                episodeFeet[side == 0 ? "left" : "right"] = std::move(item);
            }
            episodes["feet"] = std::move(episodeFeet);
            result["episodes"] = std::move(episodes);

            sol::table contacts(thisState, sol::create);
            for (std::size_t index = 0; index < state.mContacts.size(); ++index)
            {
                const auto& point = state.mContacts[index];
                const auto& source = state.mContactSources[index];
                sol::table contact(thisState, sol::create);
                contact["selfBodyId"] = std::to_string(source.mSelfBodyId.GetIndexAndSequenceNumber());
                contact["surfaceBodyId"] = std::to_string(source.mSurfaceBodyId.GetIndexAndSequenceNumber());
                contact["subShape1"] = source.mSubShape1;
                contact["subShape2"] = source.mSubShape2;
                contact["manifoldPoint"] = source.mPointIndex;
                contact["firstBody"] = source.mFirstBody;
                optional(contact, "surfaceStatic", source.mSurfaceStatic);
                contact["surfacePointKnown"] = source.mSurfacePoint.has_value();
                if (source.mSurfacePoint)
                    contact["surfacePoint"] = vector(*source.mSurfacePoint);
                contact["bodyIndex"] = point.mContact.mBodyIndex + 1;
                contact["position"] = vector(point.mContact.mPosition);
                contact["normal"] = vector(point.mContact.mNormal);
                contact["penetrationDepth"] = point.mContact.mPenetrationDepth;
                optional(contact, "normalImpulse", point.mContact.mNormalImpulse);
                optional(contact, "frictionImpulse", point.mContact.mFrictionImpulseMagnitude);
                contact["footPointVelocityKnown"] = point.mContact.mPointVelocity.has_value();
                contact["surfacePointVelocityKnown"] = point.mSurfacePointVelocity.has_value();
                if (point.mContact.mPointVelocity)
                    contact["footPointVelocity"] = vector(*point.mContact.mPointVelocity);
                if (point.mSurfacePointVelocity)
                    contact["surfacePointVelocity"] = vector(*point.mSurfacePointVelocity);
                contacts.add(std::move(contact));
            }
            result["contacts"] = std::move(contacts);
            return result;
        }
        MWInput::InputManager& getRagdollTestInput()
        {
            MWBase::InputManager* baseInput = MWBase::Environment::get().getInputManager();
            auto* input = dynamic_cast<MWInput::InputManager*>(baseInput);
            if (input == nullptr || input->getRagdollTestWindow() == nullptr)
                throw std::runtime_error("Ragdoll mouse test requires the OpenMW window");
            return *input;
        }

        SDL_Window* getRagdollTestWindow()
        {
            return getRagdollTestInput().getRagdollTestWindow();
        }

        std::optional<std::string> persistReplayIfRequested(const MWPhysics::PassiveRagdollReplayResult& replay)
        {
            const char* outputDirectory = RagdollCore::tuningText("OPENMW_RAGDOLL_REPLAY_OUTPUT_DIR");
            if (outputDirectory == nullptr || *outputDirectory == '\0')
                return std::nullopt;
            if (replay.mData.size() > static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max()))
                throw std::runtime_error("Ragdoll replay is too large to persist");

            std::error_code error;
            const std::filesystem::path directory(outputDirectory);
            std::filesystem::create_directories(directory, error);
            if (error)
                throw std::runtime_error("Unable to create ragdoll replay output directory: " + error.message());

            const std::filesystem::path path = directory / ("ragdoll-" + std::to_string(replay.mSeed) + ".omwrdpl");
            std::ofstream stream(path, std::ios::binary | std::ios::trunc);
            if (!stream)
                throw std::runtime_error("Unable to open ragdoll replay output file");
            stream.write(
                reinterpret_cast<const char*>(replay.mData.data()), static_cast<std::streamsize>(replay.mData.size()));
            stream.close();
            if (!stream)
                throw std::runtime_error("Unable to write ragdoll replay output file");
            return path.string();
        }

        const MWWorld::Ptr& requireActor(const Object& object)
        {
            const MWWorld::Ptr& ptr = object.ptr();
            if (!ptr.getClass().isActor())
                throw std::runtime_error("Actor expected");
            return ptr;
        }

        // WOUNDFX-01 FX1: shader cuts the test API placed (main thread writes,
        // the Lua thread reads).
        std::atomic<int>& woundFxAdded()
        {
            static std::atomic<int> count{ 0 };
            return count;
        }

        // WOUNDFX-01 FX3: the kind the last test wound was drawn as (main
        // thread writes, the Lua thread reads): a WoundFxSet::Kind, 0 a
        // mesh-copy mark, -1 none.
        std::atomic<int>& woundFxLastKind()
        {
            static std::atomic<int> kind{ -1 };
            return kind;
        }
    }

    sol::table initManualObservationPackage(const Context& context)
    {
        sol::state_view lua = context.sol();
        sol::table api(lua, sol::create);
        api["manualDiagnostics"] = [](sol::this_state state, sol::optional<std::string> action,
            sol::optional<std::uint64_t> bookmark, sol::optional<std::string> reason) {
            auto& recorder = ManualDiagnostics::recorder();
            const bool noteSaved = action && *action == "note" && bookmark && reason
                && recorder.annotate(*bookmark, *reason);
            const auto snapshot = action && (*action == "bookmark" || *action == "cycle")
                ? recorder.control(*action == "bookmark") : recorder.status();
            sol::table result(state, sol::create);
            result["enabled"] = recorder.enabled();
            result["noteSaved"] = noteSaved;
            result["state"] = snapshot.state;
            result["tick"] = snapshot.frame.tick;
            result["actor"] = snapshot.frame.actor;
            result["mode"] = snapshot.mode;
            result["bookmark"] = snapshot.bookmark;
            result["bookmarkTick"] = snapshot.bookmarkTick;
            result["bytes"] = snapshot.bytes;
            result["leftError"] = snapshot.frame.handError[0];
            result["rightError"] = snapshot.frame.handError[1];
            result["leftGrip"] = snapshot.frame.grip[0];
            result["rightGrip"] = snapshot.frame.grip[1];
            float peak = -1;
            for (const auto& body : snapshot.frame.bodies) peak = std::max(peak, body.utilization);
            result["motor"] = peak;
            return result;
        };
        return LuaUtil::makeReadOnly(api);
    }

    sol::table initRagdollTestPackage(const Context& context)
    {
        sol::state_view lua = context.sol();
        sol::table api(lua, sol::create);
        api["manualDiagnostics"] = initManualObservationPackage(context)["manualDiagnostics"];
        api["injectObservationEditorInput"] = [context](int x, int y, sol::optional<unsigned int> character) {
            context.mLuaManager->addAction([x, y, character] {
                getRagdollTestInput().queueObservationEditorInput(x, y, character ? static_cast<int>(*character) : -1);
            }, "ObservationEditorTestInput");
        };
        api["populationTestConfig"] = [lua]() {
            const auto integer = [](const char* name, int fallback, int minimum, int maximum) {
                const char* value = RagdollCore::tuningText(name);
                if (!value) return fallback;
                const std::string_view text(value);
                int result = 0;
                const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
                if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || result < minimum || result > maximum)
                    throw std::runtime_error(std::string(name) + " is outside its registered bounds");
                return result;
            };
            sol::table result(lua, sol::create);
            result["n"] = integer("OPENMW_RAGDOLL_TEST_POPULATION_N", 8, 1, 8);
            result["spacing"] = integer("OPENMW_RAGDOLL_TEST_POPULATION_SPACING", 384, 128, 768);
            return result;
        };

        api["sampleAnimationCycle"] = [](sol::this_state thisState, const GObject& object,
                                           const std::string& group, unsigned sampleCount) {
            const auto& actor = requireActor(object);
            const auto* animation = getWorld().getAnimation(actor);
            if (!animation)
                throw std::runtime_error("Actor has no animation");
            const auto source = animation->getRagdollAnimationSource(group);
            if (!source)
                throw std::runtime_error("Actor has no complete source for animation group " + group);
            const auto cycle = getPhysicsSystem().sampleRagdollAnimationCycle(actor, *source->mKeyframes,
                group, source->mAccumulationBone, source->mAccumulationAxes, sampleCount);
            sol::table result(thisState, sol::create);
            result["valid"] = cycle.isValid();
            result["error"] = cycle.mError;
            result["startTime"] = cycle.mStartTime;
            result["endTime"] = cycle.mEndTime;
            result["maximumSeamTranslation"] = cycle.mMaximumSeamTranslation;
            result["minimumSeamRotationDot"] = cycle.mMinimumSeamRotationDot;
            result["accumulationBone"] = source->mAccumulationBone;
            sol::table frames(thisState, sol::create);
            for (const auto& pose : cycle.mFrames)
            {
                sol::table frame(thisState, sol::create);
                for (const auto& transform : pose)
                {
                    sol::table body(thisState, sol::create);
                    body.add(transform.mTranslation.GetX());
                    body.add(transform.mTranslation.GetY());
                    body.add(transform.mTranslation.GetZ());
                    body.add(transform.mRotation.GetX());
                    body.add(transform.mRotation.GetY());
                    body.add(transform.mRotation.GetZ());
                    body.add(transform.mRotation.GetW());
                    frame.add(std::move(body));
                }
                frames.add(std::move(frame));
            }
            result["frames"] = std::move(frames);
            return result;
        };

        api["getCombatState"] = [](sol::this_state thisState, const GObject& object) -> sol::object {
            const auto state = MWMechanics::getRagdollCombatState(object.ptr());
            if (!state)
                return sol::nil;
            sol::table result(thisState, sol::create);
            result["mode"] = RagdollCore::controllerModeName(state->mMode);
            result["timeInMode"] = state->mTimeInMode;
            result["physicsStep"] = state->mPhysicsStep;
            return result;
        };

        // Observe the actual scene attachment, independently of any physical
        // weapon proxy (which can exist before the draw animation attaches it).
        api["getWeaponVisibility"] = [](sol::this_state thisState, const GObject& object) {
            auto* animation = getWorld().getAnimation(requireActor(object));
            auto* npcAnimation = dynamic_cast<MWRender::NpcAnimation*>(animation);
            osg::Node* weapon = npcAnimation ? npcAnimation->getWeaponNode() : nullptr;
            osg::Group* root = animation ? animation->getObjectRoot() : nullptr;
            sol::table result(thisState, sol::create);
            result["animationAvailable"] = animation != nullptr;
            result["upperBodyReady"] = animation && animation->upperBodyReady();
            result["rootPresent"] = root != nullptr;
            result["weaponsShown"] = animation && animation->getWeaponsShown();
            result["weaponNodePresent"] = weapon != nullptr;
            result["weaponParentPathCount"] = weapon && root ? weapon->getParentalNodePaths(root).size() : 0;
            return result;
        };

        api["getBalanceState"] = [](sol::this_state thisState, const GObject& object) -> sol::object {
            const auto state = getPhysicsSystem().getPassiveRagdollBalanceState(requireActor(object));
            if (!state)
                return sol::nil;
            return balanceStateTable(thisState, *state);
        };

        api["getState"] = [](sol::this_state thisState, const GObject& object) -> sol::object {
            const MWWorld::Ptr& ptr = requireActor(object);
            const std::optional<MWPhysics::PassiveRagdollDebugState> state
                = getPhysicsSystem().getPassiveRagdollDebugState(ptr);
            if (!state)
                return sol::nil;

            sol::table result(thisState, sol::create);
            result["physicsStep"] = state->mPhysicsStep;
            const auto& nativeReaction = state->mReaction;
            sol::table reaction(thisState, sol::create);
            reaction["enabled"] = nativeReaction.mEnabled;
            reaction["behavior"] = nativeReaction.mBehavior;
            reaction["lastServedReceipt"] = nativeReaction.mLastServedReceipt;
            reaction["speedScale"] = nativeReaction.mSpeedScale;
            reaction["stabilizationStrength"] = nativeReaction.mStabilizationStrength;
            reaction["cradleLeft"] = nativeReaction.mCradleLeft;
            reaction["cradleRight"] = nativeReaction.mCradleRight;
            reaction["limpLeft"] = nativeReaction.mLimpLeft;
            reaction["limpRight"] = nativeReaction.mLimpRight;
            reaction["leftArmOwned"] = nativeReaction.mLeftArmOwned;
            reaction["rightArmOwned"] = nativeReaction.mRightArmOwned;
            reaction["recoveryBrace"] = nativeReaction.mRecoveryBrace;
            result["reaction"] = std::move(reaction);
            if (state->mBalance)
                result["balance"] = balanceStateTable(thisState, *state->mBalance);
            result["woundReceipts"] = state->mWoundReceiptCount;
            result["strikeHand"] = state->mStrikeRightHand ? "right" : "left";
            result["thrustInput"] = state->mThrustInput;
            result["playerPressCount"] = state->mPlayerPressCount;
            result["wristOutsideRight"] = state->mWristOutsideRight;
            result["blockStaggers"] = state->mBlockStaggers;
            result["dazeTime"] = state->mDazeTime;
            result["dazeDrop"] = state->mDazeDrop;
            result["dazeCount"] = state->mDazeCount;
            result["wristOutsideLeft"] = state->mWristOutsideLeft;
            result["armOwnerLeft"] = state->mArmOwnerLeft;
            result["ownerLegs"] = state->mOwnerLegs;
            result["ownerTrunk"] = state->mOwnerTrunk;
            result["ownerHead"] = state->mOwnerHead;
            result["armOwnerRight"] = state->mArmOwnerRight;
            if (state->mHscActive)
            {
                sol::table hsc(thisState, sol::create);
                hsc["el"] = state->mHscEl;
                hsc["bladeEl"] = state->mHscBladeEl;
                hsc["s"] = state->mChamberS;
                hsc["handError"] = state->mChamberHandError;
                hsc["wristError"] = state->mChamberWristError;
                hsc["grip"] = state->mHscGrip;
                hsc["line"] = state->mChamberLine;
                hsc["pole"] = state->mHscPole;
                hsc["blade"] = state->mHscBlade;
                hsc["stage"] = state->mHscStage;
                hsc["stanceX"] = state->mHscStanceX;
                hsc["stanceY"] = state->mHscStanceY;
                hsc["angle"] = state->mHscAngle;
                hsc["drivePhase"] = state->mHscDrivePhase;
                hsc["planeMiss"] = state->mHscPlaneMiss;
                hsc["snap"] = state->mHscSnap;
                hsc["chain"] = state->mHscChain;
                hsc["click"] = state->mHscAuto;
                hsc["riposte"] = state->mHscRiposte;
                hsc["flow"] = state->mHscFlow;
                hsc["sinceParryWon"] = state->mHscSinceParryWon;
                hsc["grip"] = state->mHscGripMode;
                hsc["jabs"] = state->mHscJabs;
                hsc["jab"] = state->mHscJab;
                hsc["route"] = state->mHscRoute;
                hsc["reseat"] = state->mHscReseat;
                hsc["reseats"] = state->mHscReseats;
                if (!state->mHscTrajectory.empty())
                {
                    sol::table trajectory(thisState, sol::create);
                    for (std::size_t i = 0; i < state->mHscTrajectory.size(); ++i)
                        trajectory[i + 1] = state->mHscTrajectory[i];
                    hsc["trajectory"] = std::move(trajectory);
                }
                result["hsc"] = std::move(hsc);
            }
            if (state->mTipActive)
            {
                sol::table tip(thisState, sol::create);
                tip["az"] = state->mTipAz;
                tip["gap"] = state->mTipGap;
                tip["el"] = state->mTipEl;
                tip["radius"] = state->mTipRadius;
                tip["bodyTwist"] = state->mTipBodyTwist;
                tip["bodyLean"] = state->mTipBodyLean;
                tip["command"] = state->mTipCommand;
                result["tip"] = std::move(tip);
            }
            result["attackPhase"] = state->mAttackPhase;
            result["attackId"] = std::to_string(state->mAttackId);
            result["attackStrength"] = state->mAttackStrength;
            result["attackType"] = state->mAttackType;
            result["woundMarks"] = state->mWoundMarkCount;
            result["lastWoundKind"] = state->mLastWoundKind;
            result["lastWoundRegion"] = state->mLastWoundRegion;
            result["lastWoundHealth"] = state->mLastWoundHealth;
            result["lastWoundFatigue"] = state->mLastWoundFatigue;
            result["localReactionAngle"] = state->mLocalReactionAngle;
            const auto& quality = state->mStepQuality;
            sol::table stepQuality(thisState, sol::create);
            stepQuality["steps"] = quality.mSteps;
            stepQuality["maximumAnchorSeparation"] = quality.mMaximumAnchorSeparation;
            stepQuality["maximumAnchorStep"] = quality.mMaximumAnchorStep;
            stepQuality["maximumAnchorParentBody"] = state->mBodies.at(quality.mMaximumAnchorParentBody).mName;
            stepQuality["maximumAnchorChildBody"] = state->mBodies.at(quality.mMaximumAnchorChildBody).mName;
            stepQuality["maximumHingeViolation"] = quality.mMaximumHingeViolation;
            result["stepQuality"] = std::move(stepQuality);
            sol::table bodies(thisState, sol::create);
            for (const MWPhysics::PassiveRagdollBodyState& body : state->mBodies)
            {
                sol::table item(thisState, sol::create);
                item["name"] = body.mName;
                item["position"] = body.mPosition;
                item["centerOfMass"] = body.mCenterOfMass;
                item["targetPosition"] = body.mTargetPosition;
                if (body.mTargetRotation)
                    item["targetRotation"] = LuaUtil::asTransform(*body.mTargetRotation);
                if (body.mMotorUtilization >= 0.0f)
                    item["motorUtilization"] = body.mMotorUtilization;
                item["renderPosition"] = body.mRenderPosition;
                item["physicsRenderPosition"] = body.mPhysicsRenderPosition;
                if (body.mSkeletonPosition)
                    item["skeletonPosition"] = *body.mSkeletonPosition;
                item["rotation"] = LuaUtil::asTransform(body.mRotation);
                item["linearVelocity"] = body.mLinearVelocity;
                item["angularVelocity"] = body.mAngularVelocity;
                item["mass"] = body.mMass;
                item["active"] = body.mActive;
                bodies.add(std::move(item));
            }
            result["bodies"] = std::move(bodies);
            sol::table constraints(thisState, sol::create);
            for (const MWPhysics::PassiveRagdollConstraintState& constraint : state->mConstraints)
            {
                sol::table item(thisState, sol::create);
                item["parentBody"] = constraint.mParentBody;
                item["childBody"] = constraint.mChildBody;
                item["anchorSeparation"] = constraint.mAnchorSeparation;
                if (constraint.mHingeAngle)
                {
                    item["hingeAngle"] = *constraint.mHingeAngle;
                    item["hingeMinimum"] = *constraint.mHingeMinimum;
                    item["hingeMaximum"] = *constraint.mHingeMaximum;
                }
                constraints.add(std::move(item));
            }
            result["constraints"] = std::move(constraints);
            sol::table contacts(thisState, sol::create);
            for (const auto& contact : state->mContacts)
            {
                sol::table item(thisState, sol::create);
                item["bodyIndex"] = contact.mBodyIndex + 1;
                item["position"] = contact.mPosition;
                item["normal"] = contact.mNormal;
                item["penetrationDepth"] = contact.mPenetrationDepth;
                if (contact.mNormalImpulse)
                    item["normalImpulse"] = *contact.mNormalImpulse;
                if (contact.mFriction)
                    item["friction"] = *contact.mFriction;
                contacts.add(std::move(item));
            }
            result["contacts"] = std::move(contacts);
            if (state->mSupport)
            {
                sol::table support(thisState, sol::create);
                support["leftFoot"] = state->mSupport->mLeftFootSupported;
                support["rightFoot"] = state->mSupport->mRightFootSupported;
                support["centerOfMassSupported"] = state->mSupport->mCenterOfMassSupported;
                support["footContactCount"] = state->mSupport->mFootContactCount;
                support["groundContactCount"] = state->mSupport->mGroundContactCount;
                support["center"] = state->mSupport->mCenter;
                support["groundCenter"] = state->mSupport->mGroundCenter;
                support["centerOfMassDistance"] = state->mSupport->mCenterOfMassDistance;
                support["centerOfMassHeight"] = state->mSupport->mCenterOfMassHeight;
                if (state->mSupport->mFriction)
                    support["friction"] = *state->mSupport->mFriction;
                result["support"] = std::move(support);
            }
            // What the arms are doing. Everything else in this controller
            // reports itself here; the arms did not, so a get-up could only
            // be judged by eye.
            {
                sol::table arms(thisState, sol::create);
                bool any = false;
                for (const bool rightArm : { false, true })
                {
                    const auto debug = getPhysicsSystem().getPassiveRagdollArmDebug(ptr, rightArm);
                    if (!debug)
                        continue;
                    any = true;
                    sol::table arm(thisState, sol::create);
                    arm["reachError"] = debug->mReachError;
                    arm["weight"] = debug->mWeight;
                    arm["applied"] = debug->mApplied;
                    arm["brace"] = debug->mBrace;
                    arm["targetWorld"] = debug->mTargetWorld;
                    arm["measuredErrorWorld"] = debug->mMeasuredErrorWorld;
                    arm["targetDistanceModel"] = debug->mTargetDistanceModel;
                    arm["minimumReachModel"] = debug->mMinimumReachModel;
                    arm["maximumReachModel"] = debug->mMaximumReachModel;
                    arm["reachWithoutShoulderModel"] = debug->mReachWithoutShoulderModel;
                    arms[rightArm ? "right" : "left"] = std::move(arm);
                }
                if (any)
                    result["arms"] = std::move(arms);
            }
            if (state->mController)
            {
                sol::table controller(thisState, sol::create);
                controller["mode"] = state->mController->mMode;
                if (state->mController->mRecoveryDirection)
                    controller["recoveryDirection"] = *state->mController->mRecoveryDirection;
                controller["timeInMode"] = state->mController->mTimeInMode;
                controller["fallingExitCandidateTime"] = state->mController->mFallingExitCandidateTime;
                controller["pelvisUpDot"] = state->mController->mPelvisUpDot;
                controller["targetPelvisUpDot"] = state->mController->mTargetPelvisUpDot;
                controller["torsoUpDot"] = state->mController->mTorsoUpDot;
                controller["torsoFacingUpDot"] = state->mController->mTorsoFacingUpDot;
                if (const auto heading = getPhysicsSystem().getPassiveRagdollHeadingError(ptr))
                    controller["headingError"] = *heading;
                controller["pelvisHeight"] = state->mController->mPelvisHeight;
                controller["centerOfMassSpeed"] = state->mController->mCenterOfMassSpeed;
                controller["velocityTrackingError"] = state->mController->mVelocityTrackingError;
                controller["rootExternallyOwned"] = state->mController->mRootExternallyOwned;
                if (state->mController->mOwnedRootVerticalVelocity)
                    controller["ownedRootVerticalVelocity"] = *state->mController->mOwnedRootVerticalVelocity;
                controller["centerOfMassVelocity"] = state->mController->mCenterOfMassVelocity;
                controller["targetHorizontalVelocity"] = state->mController->mTargetHorizontalVelocity;
                controller["rawTargetHorizontalVelocity"] = state->mController->mRawTargetHorizontalVelocity;
                controller["plannedHorizontalVelocity"] = state->mController->mPlannedHorizontalVelocity;
                controller["driveJointMotors"] = state->mController->mDriveJointMotors;
                controller["jointMotorsApplied"] = state->mController->mJointMotorsApplied;
                controller["jointSupportApplied"] = state->mController->mJointSupportControlApplied;
                controller["jointSupportContact"] = state->mController->mJointSupportContact;
                controller["jointSupportMotorUtilization"] = state->mController->mJointSupportMotorUtilization;
                controller["jointMotorPeakUtilization"] = state->mController->mJointMotorPeakUtilization;
                controller["recoveryRequested"] = state->mController->mRecoveryRequested;
                controller["requestHeadingTravel"] = state->mController->mRequestHeadingTravel;
                controller["commandHeadingTravel"] = state->mController->mCommandHeadingTravel;
                controller["referenceYawTravel"] = state->mController->mReferenceYawTravel;
                controller["basePoseTravel"] = state->mController->mBasePoseTravel;
                controller["referenceHeightTravel"] = state->mController->mReferenceHeightTravel;
                controller["referenceGroundTravel"] = state->mController->mReferenceGroundTravel;
                controller["fixedSteps"] = state->mController->mFixedSteps;
                controller["recoveryAnimationStarted"] = state->mController->mRecoveryAnimationStarted;
                controller["recoveryAnimationFinished"] = state->mController->mRecoveryAnimationFinished;
                controller["gaitPhase"] = state->mController->mGaitPhase;
                controller["gaitTimeInPhase"] = state->mController->mGaitTimeInPhase;
                controller["gaitPhaseProgress"] = state->mController->mGaitPhaseProgress;
                controller["gaitSwingTimeSinceLift"] = state->mController->mGaitSwingTimeSinceLift;
                controller["gaitSwingContactCandidateTime"] = state->mController->mGaitSwingContactCandidateTime;
                controller["gaitStanceReady"] = state->mController->mGaitStanceReady;
                controller["gaitStage"] = state->mController->mGaitStage;
                controller["gaitActuationStage"] = state->mController->mGaitActuationStage;
                controller["stanceHipAxisReleased"] = state->mController->mStanceHipAxisReleased;
                controller["swingHipAxisReleased"] = state->mController->mSwingHipAxisReleased;
                controller["swingAnkleWorldMotor"] = state->mController->mSwingAnkleWorldMotor;
                controller["uprightActuationStage"] = state->mController->mUprightActuationStage;
                controller["appliedPelvisForce"] = state->mController->mAppliedPelvisForce;
                controller["appliedVerticalForceSaturated"] = state->mController->mAppliedVerticalForceSaturated;
                controller["appliedSupportMomentSaturated"] = state->mController->mAppliedSupportMomentSaturated;
                controller["animationAssistedActuation"] = state->mController->mAnimationAssistedActuation;
                controller["locomotionInterrupted"] = state->mController->mLocomotionInterrupted;
                controller["gaitSwingFootLifted"] = state->mController->mGaitSwingFootLifted;
                controller["gaitSwingTouchdownCandidate"] = state->mController->mGaitSwingTouchdownCandidate;
                controller["gaitTransitioned"] = state->mController->mGaitTransitioned;
                controller["gaitPhaseTimedOut"] = state->mController->mGaitPhaseTimedOut;
                controller["backwardRecoveryStep"] = state->mController->mBackwardRecoveryStep;
                if (state->mController->mSwingFootTarget)
                    controller["swingFootTarget"] = *state->mController->mSwingFootTarget;
                if (state->mController->mSwingFootTrajectoryModel)
                    controller["swingFootTrajectoryModel"] = *state->mController->mSwingFootTrajectoryModel;
                if (state->mController->mSwingFootResolvedModel)
                    controller["swingFootResolvedModel"] = *state->mController->mSwingFootResolvedModel;
                if (state->mController->mSwingFootPoseModel)
                    controller["swingFootPoseModel"] = *state->mController->mSwingFootPoseModel;
                if (state->mController->mSwingFootControllerWorldTarget)
                {
                    controller["swingFootControllerWorldTarget"]
                        = *state->mController->mSwingFootControllerWorldTarget;
                }
                if (state->mController->mSwingHipMotorError)
                {
                    controller["swingHipMotorError"] = *state->mController->mSwingHipMotorError;
                    controller["swingHipMotorClamp"] = *state->mController->mSwingHipMotorClamp;
                }
                if (state->mController->mSwingKneeMotorError)
                {
                    controller["swingKneeMotorError"] = *state->mController->mSwingKneeMotorError;
                    controller["swingKneeMotorClamp"] = *state->mController->mSwingKneeMotorClamp;
                }
                if (state->mController->mSwingKneeCurrentAngle)
                {
                    controller["swingKneeCurrentAngle"] = *state->mController->mSwingKneeCurrentAngle;
                    controller["swingKneeTargetAngle"] = *state->mController->mSwingKneeTargetAngle;
                }
                if (state->mController->mSwingAnkleMotorError)
                {
                    controller["swingAnkleMotorError"] = *state->mController->mSwingAnkleMotorError;
                    controller["swingAnkleMotorClamp"] = *state->mController->mSwingAnkleMotorClamp;
                }
                if (state->mController->mSwingHipMotorUtilization)
                    controller["swingHipMotorUtilization"] = *state->mController->mSwingHipMotorUtilization;
                if (state->mController->mSwingKneeMotorUtilization)
                    controller["swingKneeMotorUtilization"] = *state->mController->mSwingKneeMotorUtilization;
                if (state->mController->mSwingAnkleMotorUtilization)
                    controller["swingAnkleMotorUtilization"] = *state->mController->mSwingAnkleMotorUtilization;
                if (state->mController->mSwingFootForce)
                {
                    controller["swingFootForce"] = *state->mController->mSwingFootForce;
                    controller["swingFootForceSaturated"] = state->mController->mSwingFootForceSaturated;
                    controller["swingFootControlApplied"] = state->mController->mSwingFootControlApplied;
                }
                controller["preLiftSwingUnloadForce"] = state->mController->mPreLiftSwingUnloadForce;
                controller["pelvisHeightCandidateTime"] = state->mController->mPelvisHeightCandidateTime;
                controller["pelvisHeightCandidateValid"] = state->mController->mPelvisHeightCandidateValid;
                controller["pelvisHeightSettled"] = state->mController->mPelvisHeightSettled;
                controller["pelvisHeightFrozen"] = state->mController->mPelvisHeightFrozen;
                if (state->mController->mTargetPelvisHeight)
                    controller["targetPelvisHeight"] = *state->mController->mTargetPelvisHeight;
                if (state->mController->mUprightTargetPelvisHeight)
                {
                    sol::table upright(thisState, sol::create);
                    upright["targetPelvisHeight"] = *state->mController->mUprightTargetPelvisHeight;
                    upright["recoveryUnweight"] = state->mController->mRecoveryUnweight;
                    upright["pelvisForce"] = state->mController->mPelvisForce;
                    upright["pelvisTorque"] = state->mController->mPelvisTorque;
                    upright["spineTorque"] = state->mController->mSpineTorque;
                    upright["pelvisForceSaturated"] = state->mController->mPelvisForceSaturated;
                    upright["pelvisMagnitudeSaturated"] = state->mController->mPelvisMagnitudeSaturated;
                    upright["pelvisHorizontalForceSaturated"]
                        = state->mController->mPelvisHorizontalForceSaturated;
                    upright["pelvisVerticalForceSaturated"] = state->mController->mPelvisVerticalForceSaturated;
                    upright["pelvisTorqueSaturated"] = state->mController->mPelvisTorqueSaturated;
                    upright["spineTorqueSaturated"] = state->mController->mSpineTorqueSaturated;
                    upright["applied"] = state->mController->mUprightControlApplied;
                    controller["upright"] = std::move(upright);
                }
                result["controller"] = std::move(controller);
            }
            result["centerOfMass"] = state->mCenterOfMass;
            result["referencePosition"] = state->mReferencePosition;
            if (state->mSoftRoot)
            {
                sol::table softRoot(thisState, sol::create);
                softRoot["force"] = state->mSoftRoot->mForce;
                softRoot["torque"] = state->mSoftRoot->mTorque;
                softRoot["positionError"] = state->mSoftRoot->mPositionError;
                softRoot["angleError"] = state->mSoftRoot->mAngleError;
                softRoot["strength"] = state->mSoftRoot->mStrength;
                softRoot["saturated"] = state->mSoftRoot->mSaturated;
                result["softRoot"] = std::move(softRoot);
            }
            if (state->mPhysicsActorPosition)
                result["physicsActorPosition"] = *state->mPhysicsActorPosition;
            result["totalMass"] = state->mTotalMass;
            result["renderInterpolationFactor"] = state->mRenderInterpolationFactor;
            result["renderPhysicsStep"] = state->mRenderPhysicsStep;
            result["modeTransitionFactor"] = state->mModeTransitionFactor;
            result["pendingImpact"] = state->mPendingImpact;
            result["inheritedAnimationMotion"] = state->mInheritedAnimationMotion;
            result["powered"] = state->mPowered;
            result["guardActive"] = state->mGuardActive;
            result["guardZone"] = state->mGuardZone;
            result["balanceRight"] = state->mBalanceRight;
            result["balanceForward"] = state->mBalanceForward;
            result["balanceTip"] = state->mBalanceTip;
            result["attackReady"] = state->mAttackReady;
            result["guardStartPhysicsStep"] = std::to_string(state->mGuardStartPhysicsStep);
            result["otherRagdollContacts"] = state->mOtherRagdollContacts;
            result["horizontalBodyWidth"] = state->mHorizontalBodyWidth;
            result["combatReactionCount"] = state->mCombatReactionCount;
            if (state->mWeapon)
            {
                const auto& attached = *state->mWeapon;
                sol::table weapon(thisState, sol::create);
                weapon["recordId"] = attached.mRecordId;
                weapon["bodyIndex"] = attached.mBodyIndex + 1;
                weapon["handBodyIndex"] = attached.mHandBodyIndex + 1;
                weapon["itemWeight"] = attached.mItemWeight;
                weapon["itemSpeed"] = attached.mItemSpeed;
                weapon["physicalMass"] = attached.mPhysicalMass;
                weapon["length"] = attached.mLength;
                weapon["gripSeparation"] = attached.mGripSeparation;
                weapon["gripAngularError"] = attached.mGripAngularError;
                weapon["twoHanded"] = attached.mTwoHanded;
                weapon["supportActive"] = attached.mSupportActive;
                weapon["supportHandBodyIndex"] = attached.mSupportHandBodyIndex + 1;
                if (attached.mSupportTarget)
                {
                    weapon["supportTarget"] = *attached.mSupportTarget;
                    weapon["supportCommandTarget"] = *attached.mSupportTarget;
                }
                weapon["supportPositionError"] = attached.mSupportPositionError;
                weapon["supportCommandPositionError"] = attached.mSupportPositionError;
                if (attached.mSupportPhysical)
                {
                    weapon["supportPhysicalTarget"] = attached.mSupportPhysical->mTarget;
                    weapon["supportPhysicalPositionError"] = attached.mSupportPhysical->mPositionError;
                    weapon["supportPhysicalPhysicsStep"] = attached.mSupportPhysical->mPhysicsStep;
                }
                weapon["supportRotationError"] = attached.mSupportRotationError;
                if (attached.mSupportGrip)
                {
                    const auto& value = *attached.mSupportGrip;
                    sol::table grip(thisState, sol::create);
                    grip["state"] = value.mState;
                    grip["reason"] = value.mReason;
                    grip["attached"] = value.mAttached;
                    grip["constraintActive"] = value.mConstraintActive;
                    grip["solvedObservation"] = value.mSolvedObservation;
                    grip["authoredGripReached"] = value.mAuthoredGripReached;
                    grip["observationPhysicsStep"] = value.mObservationPhysicsStep;
                    grip["lastImpulsePhysicsStep"] = value.mLastImpulsePhysicsStep;
                    grip["anchorSeparation"] = value.mAnchorSeparation;
                    grip["relativeAnchorSpeed"] = value.mRelativeAnchorSpeed;
                    grip["stableSeconds"] = value.mStableSeconds;
                    grip["captureDistance"] = value.mCaptureDistance;
                    grip["maximumRelativeSpeed"] = value.mMaximumRelativeSpeed;
                    grip["captureImpulse"] = value.mCaptureImpulse;
                    grip["availableImpulse"] = value.mAvailableImpulse;
                    grip["linearImpulse"] = value.mLinearImpulse;
                    grip["authoredWeight"] = value.mAuthoredWeight;
                    grip["authoredDistance"] = value.mAuthoredDistance;
                    grip["reachLimit"] = value.mReachLimit;
                    grip["authoredRemaining"] = value.mAuthoredRemaining;
                    grip["acquireCount"] = value.mAcquireCount;
                    grip["releaseCount"] = value.mReleaseCount;
                    grip["breakCount"] = value.mBreakCount;
                    grip["refusalCount"] = value.mRefusalCount;
                    if (value.mFirstLocalAnchor) grip["firstLocalAnchor"] = *value.mFirstLocalAnchor;
                    if (value.mSecondLocalAnchor) grip["secondLocalAnchor"] = *value.mSecondLocalAnchor;
                    if (value.mFirstWorldAnchor) grip["firstWorldAnchor"] = *value.mFirstWorldAnchor;
                    if (value.mSecondWorldAnchor) grip["secondWorldAnchor"] = *value.mSecondWorldAnchor;
                    if (value.mPreStepFirstWorldAnchor) grip["preStepFirstWorldAnchor"] = *value.mPreStepFirstWorldAnchor;
                    if (value.mPreStepSecondWorldAnchor) grip["preStepSecondWorldAnchor"] = *value.mPreStepSecondWorldAnchor;
                    if (value.mPreStepFirstCenter) grip["preStepFirstCenter"] = *value.mPreStepFirstCenter;
                    if (value.mPreStepSecondCenter) grip["preStepSecondCenter"] = *value.mPreStepSecondCenter;
                    grip["impulseOnSecond"] = value.mImpulseOnSecond;
                    grip["firstAngularImpulse"] = value.mFirstAngularImpulse;
                    grip["secondAngularImpulse"] = value.mSecondAngularImpulse;
                    grip["includedInTreeConstraints"] = false;
                    weapon["supportGrip"] = std::move(grip);
                }
                weapon["velocitySolverSteps"] = attached.mVelocitySolverSteps;
                weapon["contacts"] = attached.mContacts;
                weapon["groundContacts"] = attached.mGroundContacts;
                weapon["otherRagdollContacts"] = attached.mOtherRagdollContacts;
                weapon["contactEventCount"] = attached.mContactEventCount;
                weapon["strikeEventCount"] = attached.mStrikeEventCount;
                weapon["damageEventCount"] = attached.mDamageEventCount;
                weapon["lastImpactFraction"] = attached.mLastImpactFraction;
                weapon["lastAuthoredDamage"] = attached.mLastAuthoredDamage;
                weapon["lastFinalDamage"] = attached.mLastFinalDamage;
                weapon["lastDamageBlocked"] = attached.mLastDamageBlocked;
                weapon["defenseOutcomeCount"] = attached.mDefenseOutcomeCount;
                weapon["lastDefenseOutcome"] = attached.mLastDefenseOutcome;
                weapon["lastContactDisposition"] = attached.mLastContactDisposition;
                weapon["lastApproachAlignment"] = attached.mLastApproachAlignment;
                weapon["lastGuardFacingAlignment"] = attached.mLastGuardFacingAlignment;
                weapon["lastDefenseDamageScale"] = attached.mLastDefenseDamageScale;
                weapon["lastDefenseImpulseScale"] = attached.mLastDefenseImpulseScale;
                weapon["lastRedirectScale"] = attached.mLastRedirectScale;
                weapon["lastRedirectImpulse"] = attached.mLastRedirectImpulse;
                const auto copyContact = [&](const MWPhysics::PassiveRagdollWeaponState::ContactEvent& contact) {
                    sol::table event(thisState, sol::create);
                    event["contactIdentity"] = std::to_string(contact.mContactIdentity);
                    event["attackerActorIdentity"] = std::to_string(contact.mAttackerActorIdentity);
                    if (contact.mTargetActorIdentity)
                        event["targetActorIdentity"] = std::to_string(*contact.mTargetActorIdentity);
                    event["weaponRecordId"] = contact.mWeaponRecordId;
                    event["weaponBodyIdentity"] = std::to_string(contact.mWeaponBodyIdentity);
                    event["targetBodyIdentity"] = std::to_string(contact.mTargetBodyIdentity);
                    event["physicsStep"] = std::to_string(contact.mPhysicsStep);
                    event["swingIdentity"] = std::to_string(contact.mSwingIdentity);
                    event["swingActive"] = contact.mSwingActive;
                    event["point"] = contact.mPoint;
                    event["normal"] = contact.mNormal;
                    event["relativeNormalSpeed"] = contact.mRelativeNormalSpeed;
                    event["relativeSpeed"] = contact.mRelativeSpeed;
                    event["effectiveMass"] = contact.mEffectiveMass;
                    if (contact.mNormalImpulse) event["normalImpulse"] = *contact.mNormalImpulse;
                    return event;
                };
                if (attached.mContactWindows)
                {
                    sol::table rows(thisState, sol::create);
                    weapon["contactWindowsEvictedCount"] = std::to_string(attached.mContactWindowsEvictedCount);
                    for (const auto& sample : *attached.mContactWindows)
                    {
                        sol::table row(thisState, sol::create);
                        row["schemaVersion"] = 1;
                        row["actorSerial"] = std::to_string(sample.mActorSerial);
                        row["lifecycleEpoch"] = std::to_string(sample.mLifecycleEpoch);
                        row["weaponBodyIdentityKnown"] = sample.mWeaponBodyIdentity.has_value();
                        if (sample.mWeaponBodyIdentity)
                            row["weaponBodyIdentity"] = std::to_string(*sample.mWeaponBodyIdentity);
                        row["physicsStep"] = std::to_string(sample.mPhysicsStep);
                        row["attackId"] = std::to_string(sample.mAttackId);
                        row["swingIdentity"] = std::to_string(sample.mSwingIdentity);
                        row["phase"] = sample.mPhase;
                        row["nativeOwned"] = sample.mNativeOwned;
                        row["overflow"] = sample.mOverflow;
                        const auto copyEvents = [&](const auto& events) {
                            sol::table result(thisState, sol::create);
                            for (const auto& contact : events) result.add(copyContact(contact));
                            return result;
                        };
                        row["contacts"] = copyEvents(sample.mContacts);
                        row["strikes"] = copyEvents(sample.mStrikes);
                        row["dispatches"] = copyEvents(sample.mDispatches);
                        sol::table ids(thisState, sol::create);
                        for (const auto id : sample.mPhysicalHitIds) ids.add(std::to_string(id));
                        row["physicalHitIds"] = std::move(ids);
                        rows.add(std::move(row));
                    }
                    weapon["contactWindows"] = std::move(rows);
                }
                if (attached.mLastContactEvent)
                    weapon["lastContactEvent"] = copyContact(*attached.mLastContactEvent);
                if (attached.mLastStrikeEvent)
                    weapon["lastStrikeEvent"] = copyContact(*attached.mLastStrikeEvent);
                weapon["linearBreakImpulse"] = attached.mLinearBreakImpulse;
                weapon["angularBreakImpulse"] = attached.mAngularBreakImpulse;
                weapon["lastLinearImpulse"] = attached.mLastLinearImpulse;
                weapon["lastAngularImpulse"] = attached.mLastAngularImpulse;
                weapon["attached"] = attached.mAttached;
                weapon["constraintEnabled"] = attached.mConstraintEnabled;
                weapon["callbackArmed"] = attached.mCallbackArmed;
                weapon["bodyRetained"] = attached.mBodyRetained;
                weapon["releaseReason"] = attached.mReleaseReason;
                weapon["releaseRequests"] = attached.mReleaseRequests;
                weapon["releaseCount"] = attached.mReleaseCount;
                result["weapon"] = std::move(weapon);
            }
            if (state->mLastCombatReaction)
            {
                const auto& record = *state->mLastCombatReaction;
                sol::table reaction(thisState, sol::create);
                reaction["id"] = std::to_string(record.mCommand.mId);
                reaction["point"] = osg::Vec3f(record.mCommand.mWorldPoint.mX,
                    record.mCommand.mWorldPoint.mY, record.mCommand.mWorldPoint.mZ);
                reaction["impulse"] = osg::Vec3f(record.mCommand.mImpulse.mX,
                    record.mCommand.mImpulse.mY, record.mCommand.mImpulse.mZ);
                reaction["stunSeconds"] = record.mCommand.mStunSeconds;
                reaction["modeAtReceipt"] = RagdollCore::controllerModeName(record.mModeAtReceipt);
                reaction["physicsStep"] = record.mPhysicsStep;
                result["lastCombatReaction"] = std::move(reaction);
            }
            return sol::make_object(thisState, std::move(result));
        };

        api["getCounters"] = [lua] {
            const MWPhysics::PassiveRagdollDebugCounters counters = getPhysicsSystem().getPassiveRagdollDebugCounters();
            sol::table result(lua, sol::create);
            result["current"] = counters.mCurrent;
            result["pendingImpacts"] = counters.mPendingImpacts;
            result["created"] = counters.mCreated;
            result["removed"] = counters.mRemoved;
            result["evicted"] = counters.mEvicted;
            result["poseFailures"] = counters.mPoseFailures;
            result["motionSamples"] = counters.mMotionSamples;
            result["motionTransfers"] = counters.mMotionTransfers;
            result["weaponBodiesCreated"] = counters.mWeaponBodiesCreated;
            result["weaponBodiesDestroyed"] = counters.mWeaponBodiesDestroyed;
            result["weaponReleases"] = counters.mWeaponReleases;
            result["weaponCallbacksArmed"] = counters.mWeaponCallbacksArmed;
            result["weaponCallbacksDisarmed"] = counters.mWeaponCallbacksDisarmed;
            result["weaponDuplicateReleaseRequests"] = counters.mWeaponDuplicateReleaseRequests;
            return result;
        };

        api["prepareEquippedWeapon"] = [](const GObject& object) {
            return getPhysicsSystem().prepareEquippedRagdollWeapon(requireActor(object));
        };

        api["releaseWeapon"] = [](const GObject& object, std::string_view name) {
            const MWWorld::Ptr& ptr = requireActor(object);
            const auto reason = [&] {
                using Reason = MWPhysics::PassiveRagdollWeaponReleaseReason;
                if (name == "break") return Reason::Break;
                if (name == "drop") return Reason::Drop;
                if (name == "unequip") return Reason::Unequip;
                if (name == "death") return Reason::Death;
                if (name == "cleanup") return Reason::Cleanup;
                throw std::runtime_error("Unknown physical weapon release reason");
            }();
            return getPhysicsSystem().releasePassiveRagdollWeapon(ptr, reason);
        };

        api["getCameraTrackedPosition"] = [] { return getWorld().getCameraTrackedPosition(); };

        // Capture the rendered game frame through the ordinary screenshot
        // action. This also works on a private desktop without OS input/focus.
        // Spark burst at a world point (look check of the hit sparks).
        api["spawnSparks"] = [context](const osg::Vec3f& position, const osg::Vec3f& normal, float strength) {
            context.mLuaManager->addAction([position, normal, strength] {
                MWBase::Environment::get().getWorld()->spawnSparks(position, normal, strength);
            }, "RagdollTestSparks");
        };

        api["captureScreenshot"] = [context]() {
            context.mLuaManager->addAction([] {
                getRagdollTestInput().executeAction(MWInput::A_Screenshot);
            }, "CaptureRagdollTestScreenshot");
        };

        // Development-only input acceptance seam. Feed the ordinary SDL
        // queue so tests exercise bindings, mouse-axis conversion, native
        // hold/release ownership and the world adapter, not just driveSwing.
        api["prepareMouseInput"] = [context]() {
            context.mLuaManager->addAction([] {
                SDL_Window* window = getRagdollTestWindow();
                int width = 0, height = 0;
                SDL_GetWindowSize(window, &width, &height);
                if (SDL_GetKeyboardFocus() == window)
                    SDL_WarpMouseInWindow(window, width / 2, height / 2);
                // The unfocused fallback injects below InputWrapper, so it
                // neither activates the OS window nor moves the pointer.
            }, "PrepareRagdollTestMouseInput");
        };
        api["getMouseInputMode"] = [](sol::this_state thisState) {
            sol::table result(thisState, sol::create);
            SDL_Window* window = SDL_GetKeyboardFocus();
            result["keyboardFocus"] = window != nullptr;
            result["pointerInside"] = window && SDL_GetMouseFocus() == window;
            result["nativeRelative"] = SDL_GetRelativeMouseMode() == SDL_TRUE;
            result["syntheticReady"] = getRagdollTestWindow() != nullptr;
            return result;
        };

        api["injectMouseInput"] = [context](int deltaX, int deltaY, sol::optional<bool> held,
                                    sol::optional<int> mouseButton) {
            if (deltaX < -4096 || deltaX > 4096 || deltaY < -4096 || deltaY > 4096)
                throw std::runtime_error("Test mouse motion exceeds the bounded input range");
            const int buttonId = mouseButton.value_or(SDL_BUTTON_LEFT);
            if (buttonId != SDL_BUTTON_LEFT && buttonId != SDL_BUTTON_MIDDLE && buttonId != SDL_BUTTON_RIGHT)
                throw std::runtime_error("Unsupported test mouse button");
            context.mLuaManager->addAction([deltaX, deltaY, held, buttonId] {
                SDL_Window* window = getRagdollTestWindow();
                if (SDL_GetKeyboardFocus() != window)
                {
                    getRagdollTestInput().injectRagdollTestMouseInput(
                        deltaX, deltaY, held ? (*held ? 1 : 0) : -1, buttonId);
                    return;
                }
                int width = 0, height = 0;
                SDL_GetWindowSize(window, &width, &height);
                const auto push = [](SDL_Event& event) {
                    if (SDL_PushEvent(&event) != 1)
                        throw std::runtime_error("Unable to queue test mouse input");
                };
                if (held)
                {
                    SDL_Event button{};
                    button.type = *held ? SDL_MOUSEBUTTONDOWN : SDL_MOUSEBUTTONUP;
                    button.button.windowID = SDL_GetWindowID(window);
                    button.button.button = static_cast<Uint8>(buttonId);
                    button.button.state = *held ? SDL_PRESSED : SDL_RELEASED;
                    button.button.clicks = 1;
                    button.button.x = width / 2;
                    button.button.y = height / 2;
                    push(button);
                }
                SDL_Event motion{};
                motion.type = SDL_MOUSEMOTION;
                motion.motion.windowID = SDL_GetWindowID(window);
                motion.motion.x = width / 2;
                motion.motion.y = height / 2;
                motion.motion.xrel = deltaX;
                motion.motion.yrel = deltaY;
                push(motion);
            }, "InjectRagdollTestMouseInput");
        };

        api["testStepMultiplier"] = [] { return OMW::TestStepScheduler::requested(); };
        api["setTestStepActive"] = [](bool active, unsigned seed) {
            if (!OMW::TestStepScheduler::requested()) return;
            OMW::TestStepScheduler::replaySeed.store(active ? seed : 0);
            OMW::TestStepScheduler::active.store(active);
        };
        api["testReplaySeed"] = []() -> sol::optional<unsigned> {
            const unsigned seed = OMW::TestStepScheduler::replaySeed.load();
            if (!seed || !OMW::TestStepScheduler::requested()) return sol::nullopt;
            return seed;
        };

        api["startReplayRecording"] = [](const GObject& object, std::uint32_t seed,
                                          sol::optional<std::uint32_t> maximumFrames, sol::optional<bool> concurrent) {
            const std::optional<std::size_t> limit
                = maximumFrames ? std::optional<std::size_t>(*maximumFrames) : std::nullopt;
            if (!getPhysicsSystem().startPassiveRagdollReplay(requireActor(object), seed, limit, concurrent.value_or(false)))
                throw std::runtime_error("Unable to start ragdoll replay recording");
        };

        api["finishReplayRecording"] = [lua](sol::optional<GObject> object) -> sol::object {
            const std::optional<MWWorld::ConstPtr> actor = object
                ? std::optional<MWWorld::ConstPtr>(requireActor(*object)) : std::nullopt;
            const auto replay = getPhysicsSystem().finishPassiveRagdollReplay(actor ? &*actor : nullptr);
            if (!replay)
                return sol::nil;
            const auto outputPath = persistReplayIfRequested(*replay);
            sol::table result(lua, sol::create);
            result["version"] = replay->mVersion;
            result["seed"] = std::to_string(replay->mSeed);
            result["profileHash"] = std::to_string(replay->mProfileHash);
            result["fixedTimeStep"] = replay->mFixedTimeStep;
            result["workerThreadCount"] = replay->mWorkerThreadCount;
            result["collisionSteps"] = replay->mCollisionSteps;
            result["inputCount"] = replay->mInputCount;
            result["controllerStateCount"] = replay->mControllerStateCount;
            result["bodyCount"] = replay->mBodyCount;
            result["frameCount"] = replay->mFrameCount;
            result["firstFallingFrame"] = replay->mFirstFallingFrame;
            result["byteCount"] = replay->mData.size();
            result["data"] = std::string(reinterpret_cast<const char*>(replay->mData.data()), replay->mData.size());
            if (outputPath)
                result["outputPath"] = *outputPath;
            return sol::make_object(lua, std::move(result));
        };

        api["applyImpulse"] = [context](const GObject& object, int bodyIndex, const Misc::FiniteVec3f& impulse,
                                  sol::optional<Misc::FiniteVec3f> worldPoint) {
            const MWWorld::Ptr& ptr = requireActor(object);
            const auto state = getPhysicsSystem().getPassiveRagdollDebugState(ptr);
            if (!state)
                throw std::runtime_error("Actor does not own a passive ragdoll");
            if (bodyIndex < 1 || static_cast<std::size_t>(bodyIndex) > state->mBodies.size())
                throw std::runtime_error("Ragdoll body index is out of range");

            std::optional<osg::Vec3f> point;
            if (worldPoint)
                point = worldPoint->mValue;
            context.mLuaManager->addAction(
                [actor = Object(ptr), bodyIndex = static_cast<std::size_t>(bodyIndex - 1), impulse = impulse.mValue,
                    point] { getPhysicsSystem().applyPassiveRagdollImpulse(actor.ptr(), bodyIndex, impulse, point); },
                "ApplyRagdollTestImpulse");
        };

        api["applyCombatImpulse"] = [context](const GObject& object, const Misc::FiniteVec3f& impulse,
                                       const Misc::FiniteVec3f& worldPoint, sol::optional<std::uint32_t> reactionId) {
            const auto& ptr = requireActor(object);
            context.mLuaManager->addAction([actor = Object(ptr), impulse = impulse.mValue, point = worldPoint.mValue,
                                              reactionId] {
                auto& physics = getPhysicsSystem();
                const std::uint64_t id = reactionId ? *reactionId : physics.nextCombatReactionId();
                physics.applyCombatReactionAtPoint(actor.ptr(), id, impulse, point);
            }, "ApplyRagdollTestCombatImpulse");
        };

        // REACT-02: a blow on a named body through the blow reaction entry
        // point - direction into the victim (world), weapon speed u/s, the
        // mass the blow carries, and the damage it paid.
        api["applyBlow"] = [context](const GObject& object, std::string bodyName,
                               const Misc::FiniteVec3f& direction, float speed, float mass, float damage) {
            const auto& ptr = requireActor(object);
            context.mLuaManager->addAction([actor = Object(ptr), bodyName = std::move(bodyName),
                                              direction = direction.mValue, speed, mass, damage] {
                if (!getPhysicsSystem().applyTestBlow(actor.ptr(), bodyName, direction, speed, mass, damage))
                    throw std::runtime_error("applyBlow: no powered ragdoll, unknown body or bad numbers");
            }, "ApplyRagdollTestBlow");
        };

        // SHOVE-01: the actor shoves the body in front of it (as the player's
        // double right click); false when nobody is in reach.
        api["shove"] = [context](const GObject& object) {
            const auto& ptr = requireActor(object);
            context.mLuaManager->addAction(
                [actor = Object(ptr)] { getPhysicsSystem().shovePassiveRagdoll(actor.ptr()); }, "RagdollTestShove");
        };

        // Live tuning (TUNE-01): the same override the KINETIC page's
        // sliders set (without the saved file); applies at the next physics
        // step. nil drops it.
        api["setTuning"] = [](std::string name, sol::optional<std::string> value) {
            RagdollCore::setTuningOverride(name, value ? std::optional<std::string>(*value) : std::nullopt);
        };
        api["tuningText"] = [](std::string name) -> sol::optional<std::string> {
            if (const char* text = RagdollCore::tuningText(name))
                return std::string(text);
            return sol::nullopt;
        };

        // Melee push strength multiplier, for calibrating hit feel in the
        // manual range. Affects the impulse only, never damage.
        api["setMeleeImpulseScale"] = [](float scale) {
            if (!(scale >= 0.0f) || !std::isfinite(scale))
                throw std::runtime_error("Melee impulse scale must be a finite, non-negative number");
            MWMechanics::setMeleeRagdollImpulseScale(scale);
            return MWMechanics::getMeleeRagdollImpulseScale();
        };
        api["getMeleeImpulseScale"] = [] { return MWMechanics::getMeleeRagdollImpulseScale(); };
        // Physical strike reaction tuning: equivalent reaction impulse per
        // point of pre-blunt damage. 0 silences physical hit reactions; the
        // default 200 makes a clean 3-point hit stun, 6+ stagger and 9+ fall.
        api["reactionScale"] = [](sol::optional<float> value) {
            if (value)
            {
                if (!(*value >= 0.0f) || !std::isfinite(*value))
                    throw std::runtime_error("Reaction scale must be a finite, non-negative number");
                MWMechanics::setPhysicalReactionPerDamage(*value);
            }
            return MWMechanics::getPhysicalReactionPerDamage();
        };
        // Test-range normalization for bare-handed damage. The game formula
        // scales hand-to-hand damage with the skill value; a developer
        // character can carry a maxed skill, so a range can scale it down to
        // what a normally skilled fighter would deal.
        api["handDamageScale"] = [](sol::optional<float> value) {
            if (value)
            {
                if (!(*value >= 0.0f) || !std::isfinite(*value))
                    throw std::runtime_error("Hand damage scale must be a finite, non-negative number");
                MWMechanics::setPhysicalHandDamageScale(*value);
            }
            return MWMechanics::getPhysicalHandDamageScale();
        };
        // Whether a fallen actor gets up unaided. Diagnostics that watch a
        // fallen body for a fixed window turn it off.
        api["setAutomaticRecovery"] = [](bool automatic) {
            getPhysicsSystem().setAutomaticRagdollRecovery(automatic);
            return getPhysicsSystem().getAutomaticRagdollRecovery();
        };
        // Skip Morrowind's to-hit roll for the player, so a swing that
        // reaches the target connects. Testing aid; damage is unchanged.
        // The play hub asks before turning the player's hit roll off (user
        // 10-09: "we need to follow vanilla balance as much as possible"):
        // OPENMW_RAGDOLL_PLAYER_ROLL=1 keeps vanilla's roll for the player too,
        // as the NPCs have it. Tests keep their always-hit calibration.
        api["playerRollWanted"] = []() {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_PLAYER_ROLL");
            return text && std::atoi(text) != 0;
        };
        api["setPlayerAlwaysHits"] = [](bool alwaysHits) {
            MWMechanics::setPlayerAlwaysHits(alwaysHits);
            return MWMechanics::getPlayerAlwaysHits();
        };
        // Register a combat target outside the ordinary AI. The melee hit
        // test reads the attacker's combat target list (the player uses a
        // range scan instead), so scripted NPC duels need one entry per
        // duelist or every swing is discarded as a miss.
        api["setCombatTarget"] = [](const GObject& attacker, const GObject& target) {
            const MWWorld::Ptr& attackerPtr = requireActor(attacker);
            const MWWorld::Ptr& targetPtr = requireActor(target);
            if (attackerPtr.isEmpty() || targetPtr.isEmpty() || attackerPtr == targetPtr)
                throw std::runtime_error("setCombatTarget needs two distinct actors");
            MWBase::Environment::get().getMechanicsManager()->startCombat(
                attackerPtr, targetPtr, nullptr);
        };
        // RIPOSTE-02: the state a won parry leaves (the riposte window opens).
        api["markParryWon"] = [](const GObject& object) {
            const MWWorld::Ptr& ptr = requireActor(object);
            getPhysicsSystem().markRagdollParryWon(ptr);
        };
        // GRIP-05: the grip key for an actor; true when the grip changed.
        api["cycleGrip"] = [](const GObject& object) {
            return getPhysicsSystem().cycleRagdollGrip(requireActor(object));
        };
        api["getCombatTargets"] = [](sol::this_state thisState, const GObject& object) {
            const MWWorld::Ptr& ptr = requireActor(object);
            std::vector<MWWorld::Ptr> targets;
            ptr.getClass().getCreatureStats(ptr).getAiSequence().getCombatTargets(targets);
            sol::table result(thisState, sol::create);
            for (std::size_t i = 0; i < targets.size(); ++i)
                result[i + 1] = Object(targets[i]);
            return result;
        };

        api["grab"] = [context](const GObject& object, const Misc::FiniteVec3f& rayFrom,
                          const Misc::FiniteVec3f& rayDirection, float maxForce) {
            const MWWorld::Ptr& ptr = requireActor(object);
            if (!getPhysicsSystem().getPassiveRagdollDebugState(ptr))
                throw std::runtime_error("Actor does not own a passive ragdoll");
            context.mLuaManager->addAction(
                [actor = Object(ptr), from = rayFrom.mValue, direction = rayDirection.mValue, maxForce] {
                    getPhysicsSystem().grabPassiveRagdoll(actor.ptr(), from, direction, maxForce);
                },
                "GrabRagdollTest");
        };
        api["updateGrab"] = [context](const GObject& object, const Misc::FiniteVec3f& anchor) {
            const MWWorld::Ptr& ptr = requireActor(object);
            context.mLuaManager->addAction(
                [actor = Object(ptr), anchor = anchor.mValue] {
                    getPhysicsSystem().updatePassiveRagdollGrab(actor.ptr(), anchor);
                },
                "UpdateRagdollTestGrab");
        };
        api["releaseGrab"] = [context](const GObject& object) {
            const MWWorld::Ptr& ptr = requireActor(object);
            context.mLuaManager->addAction(
                [actor = Object(ptr)] { getPhysicsSystem().releasePassiveRagdollGrab(actor.ptr()); },
                "ReleaseRagdollTestGrab");
        };

        // Ask one hand to reach a world point. The blend rate is per second,
        // so a rate of 6 puts a full-weight arm in place in about a sixth of
        // a second. Repeating the call retargets without restarting the blend.
        api["setArmTarget"] = [context](const GObject& object, bool rightArm,
                                  const Misc::FiniteVec3f& worldTarget, float weight, float blendRate) {
            const MWWorld::Ptr& ptr = requireActor(object);
            if (!getPhysicsSystem().getPassiveRagdollDebugState(ptr))
                throw std::runtime_error("Actor does not own a passive ragdoll");
            if (!(weight >= 0.0f) || !(weight <= 1.0f))
                throw std::runtime_error("Arm target weight must be between 0 and 1");
            if (!(blendRate > 0.0f) || !std::isfinite(blendRate))
                throw std::runtime_error("Arm target blend rate must be a positive number");
            context.mLuaManager->addAction(
                [actor = Object(ptr), rightArm, target = worldTarget.mValue, weight, blendRate] {
                    getPhysicsSystem().setPassiveRagdollArmTarget(
                        actor.ptr(), rightArm, target, weight, blendRate);
                },
                "SetRagdollTestArmTarget");
        };
        api["releaseArmTarget"] = [context](const GObject& object, bool rightArm) {
            const MWWorld::Ptr& ptr = requireActor(object);
            context.mLuaManager->addAction(
                [actor = Object(ptr), rightArm] {
                    getPhysicsSystem().releasePassiveRagdollArmTarget(actor.ptr(), rightArm);
                },
                "ReleaseRagdollTestArmTarget");
        };
        api["setGuard"] = [context](const GObject& object, bool active,
                              sol::optional<Misc::FiniteVec3f> worldFacing) {
            const MWWorld::Ptr& ptr = requireActor(object);
            const std::optional<osg::Vec3f> facing = worldFacing
                ? std::optional<osg::Vec3f>(worldFacing->mValue) : std::nullopt;
            context.mLuaManager->addAction([actor = Object(ptr), active, facing] {
                if (!getPhysicsSystem().setPassiveRagdollGuard(actor.ptr(), active, facing))
                    throw std::runtime_error("Actor does not own a passive ragdoll");
            }, "SetRagdollGuard");
        };
        // The guard's zone (GUARD_ZONES): mouse counts right and up from the
        // middle (the player's come from the right-click guard).
        api["setGuardZone"] = [context](const GObject& object, float countsRight, float countsUp) {
            const MWWorld::Ptr& ptr = requireActor(object);
            context.mLuaManager->addAction([actor = Object(ptr), countsRight, countsUp] {
                if (!getPhysicsSystem().drivePassiveRagdollGuardZone(actor.ptr(), countsRight, countsUp, true))
                    throw std::runtime_error("Actor does not own a passive ragdoll");
            }, "SetRagdollGuardZone");
        };
        api["setWeaponFixtureYaw"] = [context](const GObject& object, float angle) {
            const MWWorld::Ptr& ptr = requireActor(object);
            if (!std::isfinite(angle))
                throw std::runtime_error("Weapon fixture yaw must be finite");
            context.mLuaManager->addAction(
                [actor = Object(ptr), angle] {
                    if (!getPhysicsSystem().setPassiveRagdollWeaponFixtureYaw(actor.ptr(), angle))
                        throw std::runtime_error("Actor has no attached physical weapon fixture");
                },
                "SetRagdollTestWeaponFixtureYaw");
        };
        // How far short of the request the arm actually reached, or nil when
        // that arm holds no request. A brace that has found the floor reads
        // near zero; one still waving reads the distance it is short by.
        api["getArmReach"] = [](const GObject& object, bool rightArm) {
            return getPhysicsSystem().getPassiveRagdollArmReach(requireActor(object), rightArm);
        };

        // Physical swing: while held, aim motion drives a point the
        // weapon hand chases. The basis vectors are the frame the motion is
        // drawn in - pass the camera's for a player-drawn arc. Returns the
        // speed the commanded point actually moved at.
        api["driveSwing"] = [context](const GObject& object, bool rightArm, const Misc::FiniteVec3f& aimRight,
                                const Misc::FiniteVec3f& aimForward, const Misc::FiniteVec3f& aimUp, float deltaRight,
                                float deltaUp, bool held, float duration) {
            const MWWorld::Ptr& ptr = requireActor(object);
            if (!getPhysicsSystem().getPassiveRagdollDebugState(ptr))
                throw std::runtime_error("Actor does not own a passive ragdoll");
            if (!(duration > 0.0f) || !std::isfinite(duration))
                throw std::runtime_error("Swing duration must be a positive number");
            context.mLuaManager->addAction(
                [actor = Object(ptr), rightArm, right = aimRight.mValue, forward = aimForward.mValue,
                    up = aimUp.mValue, deltaRight, deltaUp, held, duration] {
                    getPhysicsSystem().driveRagdollWeaponSwing(
                        actor.ptr(), rightArm, right, forward, up, deltaRight, deltaUp, held, duration);
                },
                "DriveRagdollTestSwing");
        };
        // How fast the commanded swing point is moving, or nil when nothing
        // is swinging. This is what a blow's force should be read from.
        // How much stronger the arm is while swinging than authored. The
        // authored motors hold a pose; they do not drive a weapon.
        api["setSwingStrength"] = [](float scale) {
            if (!(scale >= 1.0f) || !std::isfinite(scale))
                throw std::runtime_error("Swing strength must be at least 1");
            getPhysicsSystem().setRagdollSwingStrength(scale);
            return getPhysicsSystem().getRagdollSwingStrength();
        };
        api["getSwingStrength"] = [] { return getPhysicsSystem().getRagdollSwingStrength(); };

        // Localized limb damage ledger (development visual): damage taken per
        // anatomical region, the same numbers that drive the red tint.
        api["getLimbDamage"] = [lua](const GObject& object) {
            sol::table result(lua, sol::create);
            for (const auto& [region, damage] : getPhysicsSystem().getRagdollLimbDamage(requireActor(object)))
                result[region] = damage;
            return result;
        };
        api["clearLimbDamage"] = [] { getPhysicsSystem().clearRagdollLimbDamage(); };
        api["addLimbDamage"] = [](const GObject& object, std::string_view region, float damage) {
            getPhysicsSystem().addRagdollLimbDamage(requireActor(object), region, damage);
        };
        api["stageWoundReceipt"] = [](const GObject& actor, std::string_view body, int kind,
            const osg::Vec3f& point, const osg::Vec3f& normal) {
            return getPhysicsSystem().stageWoundReceipt(requireActor(actor), body, kind, point, normal);
        };
        // WOUNDFX-01 look check (OPENMW_RAGDOLL_WOUND_FX=1): one shader
        // wound on a region's bare skin. point: world, near the skin; normal:
        // out of the skin; direction: the stroke (a stab's edge axis); length
        // and halfWidth in game units (a bruise: halfWidth is its radius);
        // severity 0..1 (default 0.6); kind 'slash' (default; drawn as a chop
        // from severity 0.8), 'stab' (or 'puncture'), 'chop' or 'bruise' (or
        // 'blunt', 'dent') (bruise volumes only at OPENMW_RAGDOLL_WOUND_FX>=2).
        // The drawn kind follows the struck surface: at level 3 a stain on
        // clothing, a scratch or dent on armour (FX3).
        // Applied on the main thread at the next update; getWoundFxAdded
        // counts the wounds that took, getWoundFxLastKind names how the last
        // one was drawn.
        api["addWoundFx"] = [context](const GObject& object, std::string region, const Misc::FiniteVec3f& point,
                                const Misc::FiniteVec3f& normal, const Misc::FiniteVec3f& direction, float length,
                                float halfWidth, sol::optional<float> severity, sol::optional<std::string> kindName) {
            const MWWorld::Ptr& ptr = requireActor(object);
            const float strength = severity.value_or(0.6f);
            if (!(length > 0.0f) || !std::isfinite(length) || !(halfWidth > 0.0f) || !std::isfinite(halfWidth)
                || !std::isfinite(strength))
                throw std::runtime_error("Wound FX length and half width must be positive numbers");
            const std::string name = kindName.value_or("slash");
            int kind = 0;
            if (name == "bruise" || name == "blunt" || name == "dent")
                kind = 1;
            else if (name == "slash")
                kind = 2;
            else if (name == "stab" || name == "puncture")
                kind = 3;
            else if (name == "chop")
                kind = 4;
            else
                throw std::runtime_error("Wound FX kind must be slash, stab, chop or bruise (blunt), not " + name);
            context.mLuaManager->addAction(
                [actor = Object(ptr), region = std::move(region), point = point.mValue, normal = normal.mValue,
                    direction = direction.mValue, length, halfWidth, strength, kind] {
                    MWRender::Animation* animation
                        = MWBase::Environment::get().getWorld()->getAnimation(requireActor(actor));
                    const bool added = animation
                        && animation->addWoundFx(region, point, normal, direction, length, halfWidth, strength, kind);
                    woundFxLastKind() = added ? animation->getLastWoundFxKind() : -1;
                    if (added)
                        ++woundFxAdded();
                },
                "AddRagdollTestWoundFx");
        };
        api["getWoundFxAdded"] = [] { return woundFxAdded().load(); };
        // How the last addWoundFx wound was drawn: 'bruise', 'slash', 'stab',
        // 'chop' (skin), 'stain' (clothing), 'scratch' (armour: a scored line
        // or a dent), 'mark' (took as a mesh-copy mark), 'none'.
        api["getWoundFxLastKind"] = []() -> std::string {
            static const char* const names[]
                = { "mark", "bruise", "slash", "stab", "chop", "stain", "scratch" };
            const int kind = woundFxLastKind().load();
            return kind >= 0 && kind <= 6 ? names[kind] : "none";
        };
        // OPENMW_RAGDOLL_WOUND_FX: 0 off, 1 cut volumes, 2 bruise volumes too,
        // 3 clothing stains and armour scratches too, 4 blood (runs, blood
        // types, blood on weapons) too.
        api["getWoundFxLevel"] = [] { return MWRender::WoundFxSet::level(); };
        // WOUNDFX-01 FX4 look check (OPENMW_RAGDOLL_WOUND_FX=4): a blood mark
        // on the actor's held weapon, from..to along it (0 the grip, 1 the
        // tip), strength 0..1 (default 0.8), bloodType 0 red (default) or 1 a
        // skeleton's dust. Queued like a hit's (Animation::addWeaponBlood),
        // drawn at the actor's next animation update; getWeaponBloodFx counts
        // the marks drawn on its weapons so far.
        api["addWeaponBloodFx"] = [context](const GObject& object, float from, float to,
                                      sol::optional<float> strength, sol::optional<int> bloodType) {
            const MWWorld::Ptr& ptr = requireActor(object);
            const float amount = strength.value_or(0.8f);
            if (!std::isfinite(from) || !std::isfinite(to) || !std::isfinite(amount))
                throw std::runtime_error("Weapon blood range and strength must be numbers");
            const int type = bloodType.value_or(0);
            context.mLuaManager->addAction(
                [actor = Object(ptr), from, to, amount, type] {
                    if (MWRender::Animation* animation
                        = MWBase::Environment::get().getWorld()->getAnimation(requireActor(actor)))
                        animation->addWeaponBlood(from, to, amount, type);
                },
                "AddRagdollTestWeaponBloodFx");
        };
        api["getWeaponBloodFx"] = [](const GObject& object) -> std::size_t {
            const MWRender::Animation* animation = getWorld().getAnimation(requireActor(object));
            return animation ? animation->getWeaponBloodCount() : 0;
        };

        api["getSwingSpeed"] = [](const GObject& object) {
            return getPhysicsSystem().getRagdollWeaponSwingSpeed(requireActor(object));
        };

        // Turn the bracing arms on or off while the game is running, so a
        // get-up can be watched both ways without a rebuild.
        // Force the get-up animation: true face up, false face down, nil to
        // restore the measured choice.
        api["setRecoveryDirection"] = [](sol::optional<bool> faceUp) {
            getPhysicsSystem().setRagdollRecoveryDirection(
                faceUp ? std::optional<bool>(*faceUp) : std::nullopt);
        };

        api["setBracingArms"] = [](bool bracing) {
            getPhysicsSystem().setRagdollBracingArms(bracing);
            return getPhysicsSystem().getRagdollBracingArms();
        };
        api["getBracingArms"] = [] { return getPhysicsSystem().getRagdollBracingArms(); };

        api["setActive"] = [context](const GObject& object, bool active) {
            const MWWorld::Ptr& ptr = requireActor(object);
            if (!getPhysicsSystem().getPassiveRagdollDebugState(ptr))
                throw std::runtime_error("Actor does not own a passive ragdoll");
            context.mLuaManager->addAction(
                [actor = Object(ptr), active] { getPhysicsSystem().setPassiveRagdollActive(actor.ptr(), active); },
                "SetRagdollTestActive");
        };

        api["setPowered"] = [context](const GObject& object, bool powered) {
            const MWWorld::Ptr& ptr = requireActor(object);
            context.mLuaManager->addAction(
                [actor = Object(ptr), powered] {
                    const MWWorld::Ptr& actorPtr = requireActor(actor);
                    if (!getWorld().setActorPoweredRagdoll(actorPtr, powered))
                        throw std::runtime_error("Unable to change powered ragdoll state");
                },
                "SetPoweredRagdollTestState");
        };

        api["requestRecovery"] = [context](const GObject& object) {
            const MWWorld::Ptr& ptr = requireActor(object);
            context.mLuaManager->addAction(
                [actor = Object(ptr)] {
                    const MWWorld::Ptr& actorPtr = requireActor(actor);
                    if (!getWorld().requestActorPoweredRagdollRecovery(actorPtr))
                        throw std::runtime_error("Unable to request powered ragdoll recovery");
                },
                "RequestPoweredRagdollRecovery");
        };

        api["triggerDeath"] = [context](const GObject& object, sol::optional<sol::table> options) {
            const MWWorld::Ptr& ptr = requireActor(object);
            osg::Vec3f hitPoint = ptr.getRefData().getPosition().asVec3();
            osg::Vec3f direction(0.0f, 1.0f, 0.15f);
            float damage = 100.0f;
            if (options)
            {
                if (const auto value = options->get<sol::optional<Misc::FiniteVec3f>>("hitPoint"))
                    hitPoint = value->mValue;
                if (const auto value = options->get<sol::optional<Misc::FiniteVec3f>>("direction"))
                    direction = value->mValue;
                if (const auto value = options->get<sol::optional<Misc::FiniteFloat>>("damage"))
                    damage = value->mValue;
            }
            if (direction.length2() <= 1e-6f)
                throw std::runtime_error("Ragdoll test direction must be non-zero");
            if (damage <= 0.0f)
                throw std::runtime_error("Ragdoll test damage must be positive");

            context.mLuaManager->addAction(
                [actor = Object(ptr), hitPoint, direction, damage] {
                    const MWWorld::Ptr& actorPtr = requireActor(actor);
                    MWMechanics::CreatureStats& stats = actorPtr.getClass().getCreatureStats(actorPtr);
                    MWMechanics::DynamicStat<float> health = stats.getHealth();
                    health.setCurrent(0.0f);
                    stats.setHealth(health);
                    MWBase::Environment::get().getWorld()->recordActorKillingBlow(
                        actorPtr, hitPoint, direction, damage);
                },
                "TriggerRagdollTestDeath");
        };

        api["resurrect"] = [context](const GObject& object) {
            const MWWorld::Ptr& ptr = requireActor(object);
            context.mLuaManager->addAction(
                [actor = Object(ptr)] {
                    MWBase::Environment::get().getMechanicsManager()->resurrect(requireActor(actor));
                },
                "ResurrectRagdollTestActor");
        };

        return LuaUtil::makeReadOnly(api);
    }
}

#endif

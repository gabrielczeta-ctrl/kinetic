#include "mtphysics.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <stdexcept>
#include <thread>
#include <variant>

#include <Jolt/Core/JobSystem.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/RayCast.h>

#include <osg/Stats>

#include "components/debug/debuglog.hpp"
#include "components/misc/convert.hpp"
#include <components/misc/barrier.hpp>
#include <components/settings/values.hpp>

#include "../mwmechanics/actorutil.hpp"
#include "../mwmechanics/creaturestats.hpp"

#include "../mwrender/joltdebugdraw.hpp"

#include "../mwworld/class.hpp"

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"

#include "actor.hpp"
#include "joltfilters.hpp"
#include "movementsolver.hpp"
#include "object.hpp"
#include "physicssystem.hpp"

namespace MWPhysics
{
    namespace
    {
        template <class Mutex>
        std::optional<std::unique_lock<Mutex>> makeExclusiveLock(Mutex& mutex, LockingPolicy lockingPolicy)
        {
            if (lockingPolicy == LockingPolicy::NoLocks)
                return {};
            return std::unique_lock(mutex);
        }

        /// @brief A scoped lock that is either exclusive or inexistent depending on configuration
        template <class Mutex>
        class MaybeExclusiveLock
        {
        public:
            /// @param mutex a mutex
            /// @param threadCount decide wether the excluse lock will be taken
            explicit MaybeExclusiveLock(Mutex& mutex, LockingPolicy lockingPolicy)
                : mImpl(makeExclusiveLock(mutex, lockingPolicy))
            {
            }

        private:
            std::optional<std::unique_lock<Mutex>> mImpl;
        };

        template <class Mutex>
        std::optional<std::shared_lock<Mutex>> makeSharedLock(Mutex& mutex, LockingPolicy lockingPolicy)
        {
            if (lockingPolicy == LockingPolicy::NoLocks)
                return {};
            return std::shared_lock(mutex);
        }

        /// @brief A scoped lock that is either shared or inexistent depending on configuration
        template <class Mutex>
        class MaybeSharedLock
        {
        public:
            /// @param mutex a shared mutex
            /// @param threadCount decide wether the shared lock will be taken
            explicit MaybeSharedLock(Mutex& mutex, LockingPolicy lockingPolicy)
                : mImpl(makeSharedLock(mutex, lockingPolicy))
            {
            }

        private:
            std::optional<std::shared_lock<Mutex>> mImpl;
        };

        template <class Mutex>
        std::variant<std::monostate, std::unique_lock<Mutex>, std::shared_lock<Mutex>> makeLock(
            Mutex& mutex, LockingPolicy lockingPolicy)
        {
            switch (lockingPolicy)
            {
                case LockingPolicy::NoLocks:
                    return std::monostate{};
                case LockingPolicy::AllowSharedLocks:
                    return std::shared_lock(mutex);
            };

            throw std::runtime_error("Unsupported LockingPolicy: "
                + std::to_string(static_cast<std::underlying_type_t<LockingPolicy>>(lockingPolicy)));
        }

        /// @brief A scoped lock that is either shared, exclusive or inexistent depending on configuration
        template <class Mutex>
        class MaybeLock
        {
        public:
            /// @param mutex a shared mutex
            /// @param threadCount decide wether the lock will be shared, exclusive or inexistent
            explicit MaybeLock(Mutex& mutex, LockingPolicy lockingPolicy)
                : mImpl(makeLock(mutex, lockingPolicy))
            {
            }

        private:
            std::variant<std::monostate, std::unique_lock<Mutex>, std::shared_lock<Mutex>> mImpl;
        };
    }
}

// Actors simulation
namespace
{
    bool isUnderWater(const MWPhysics::ActorFrameData& actorData)
    {
        return actorData.mPosition.z() < actorData.mSwimLevel;
    }

    osg::Vec3f interpolateMovements(const MWPhysics::PtrHolder& ptr, float timeAccum, float physicsDt)
    {
        const float interpolationFactor = std::clamp(timeAccum / physicsDt, 0.0f, 1.0f);
        return ptr.getPosition() * interpolationFactor + ptr.getPreviousPosition() * (1.f - interpolationFactor);
    }

    using LockedActorSimulation
        = std::pair<std::shared_ptr<MWPhysics::Actor>, std::reference_wrapper<MWPhysics::ActorFrameData>>;

    namespace Visitors
    {
        template <class Impl, template <class> class Lock>
        struct WithLockedPtr
        {
            const Impl& mImpl;
            std::shared_mutex& mSimulationMutex;
            const MWPhysics::LockingPolicy mLockingPolicy;

            template <class Ptr, class FrameData>
            void operator()(MWPhysics::SimulationImpl<Ptr, FrameData>& sim) const
            {
                auto locked = sim.lock();
                if (!locked.has_value())
                    return;
                auto&& [ptr, frameData] = *std::move(locked);
                // Locked shared_ptr has to be destructed after releasing mSimulationMutex to avoid
                // possible deadlock. Ptr destructor also acquires mSimulationMutex.
                const std::pair arg(std::move(ptr), frameData);
                const Lock<std::shared_mutex> lock(mSimulationMutex, mLockingPolicy);
                mImpl(arg);
            }
        };

        struct InitPosition
        {
            const JPH::PhysicsSystem* mPhysicsSystem;
            void operator()(MWPhysics::ActorSimulation& sim) const
            {
                auto locked = sim.lock();
                if (!locked.has_value())
                    return;
                auto& [actor, frameDataRef] = *locked;
                auto& frameData = frameDataRef.get();
                frameData.mPosition = actor->applyOffsetChange();
                if (frameData.mWaterCollision && frameData.mPosition.z() < frameData.mWaterlevel
                    && actor->canMoveToWaterSurface(frameData.mWaterlevel, mPhysicsSystem))
                {
                    const auto offset = osg::Vec3f(0, 0, frameData.mWaterlevel - frameData.mPosition.z());
                    MWBase::Environment::get().getWorld()->moveObjectBy(actor->getPtr(), offset, false);
                    frameData.mPosition = actor->applyOffsetChange();
                }
                actor->updateCollisionObjectPosition();
                frameData.mOldHeight = frameData.mPosition.z();
                const auto rotation = actor->getPtr().getRefData().getPosition().asRotationVec3();
                frameData.mRotation = osg::Vec2f(rotation.x(), rotation.z());
                frameData.mInertia = actor->getInertialForce();
                frameData.mStuckFrames = actor->getStuckFrames();
                frameData.mLastStuckPosition = actor->getLastStuckPosition();
            }
        };

        struct PreStep
        {
            JPH::PhysicsSystem* mPhysicsSystem;
            void operator()(const LockedActorSimulation& sim) const
            {
                MWPhysics::MovementSolver::unstuck(sim.second, mPhysicsSystem);
            }
        };

        struct UpdatePosition
        {
            JPH::PhysicsSystem* mPhysicsSystem;
            void operator()(const LockedActorSimulation& sim) const
            {
                auto& [actor, frameDataRef] = sim;
                auto& frameData = frameDataRef.get();
                if (actor->setPosition(frameData.mPosition))
                {
                    frameData.mPosition = actor->getPosition(); // account for potential position change made by script
                    actor->updateCollisionObjectPosition();
                }
            }
        };

        struct Move
        {
            const float mPhysicsDt;
            const JPH::PhysicsSystem* mPhysicsSystem;
            const MWPhysics::WorldFrameData* mWorldFrameData;
            void operator()(const LockedActorSimulation& sim) const
            {
                assert(mWorldFrameData != nullptr);
                MWPhysics::MovementSolver::move(sim.second, mPhysicsDt, mPhysicsSystem, *mWorldFrameData);
            }
        };

        struct Sync
        {
            const bool mAdvanceSimulation;
            const float mTimeAccum;
            const float mPhysicsDt;
            const MWPhysics::PhysicsTaskScheduler* scheduler;
            void operator()(MWPhysics::ActorSimulation& sim) const
            {
                auto locked = sim.lock();
                if (!locked.has_value())
                    return;
                auto& [actor, frameDataRef] = *locked;
                auto& frameData = frameDataRef.get();
                auto ptr = actor->getPtr();

                MWMechanics::CreatureStats& stats = ptr.getClass().getCreatureStats(ptr);
                const float heightDiff = frameData.mPosition.z() - frameData.mOldHeight;
                const bool isStillOnGround = (mAdvanceSimulation && frameData.mWasOnGround && frameData.mIsOnGround);

                if (isStillOnGround || frameData.mFlying || isUnderWater(frameData) || frameData.mSlowFall < 1)
                    stats.land(ptr == MWMechanics::getPlayer() && (frameData.mFlying || isUnderWater(frameData)));
                else if (heightDiff < 0)
                    stats.addToFallHeight(-heightDiff);

                actor->setSimulationPosition(::interpolateMovements(*actor, mTimeAccum, mPhysicsDt));
                actor->setLastStuckPosition(frameData.mLastStuckPosition);
                actor->setStuckFrames(frameData.mStuckFrames);
                if (mAdvanceSimulation)
                {
                    MWWorld::Ptr standingOn;
                    auto* const ptrHolder
                        = static_cast<MWPhysics::PtrHolder*>(scheduler->getUserPointer(frameData.mStandingOn));
                    if (ptrHolder != nullptr)
                        standingOn = ptrHolder->getPtr();
                    actor->setStandingOnPtr(standingOn);
                    // the "on ground" state of an actor might have been updated by a traceDown, don't overwrite the
                    // change
                    if (actor->getOnGround() == frameData.mWasOnGround)
                        actor->setOnGround(frameData.mIsOnGround);
                    actor->setOnSlope(frameData.mIsOnSlope);
                    actor->setWalkingOnWater(frameData.mWalkingOnWater);
                    actor->setInertialForce(frameData.mInertia);
                }
            }
        };
    }
}

namespace MWPhysics
{
    PhysicsTaskScheduler::PhysicsTaskScheduler(float physicsDt, JPH::PhysicsSystem* physicsSystem,
        MWRender::JoltDebugDrawer* debugDrawer, JPH::JobSystem* jobSystem)
        : mPhysicsSystem(physicsSystem)
        , mJobSystem(jobSystem)
        , mDefaultPhysicsDt(physicsDt)
        , mPhysicsDt(physicsDt)
        , mTimeAccum(0.f)
        , mDebugDrawer(debugDrawer)
        , mLockingPolicy(jobSystem->GetMaxConcurrency() <= 1 ? LockingPolicy::NoLocks : LockingPolicy::AllowSharedLocks)
        , mNumThreads(static_cast<unsigned>(std::max(1, jobSystem->GetMaxConcurrency() - 1)))
        , mNumJobs(0)
        , mRemainingSteps(0)
        , mLOSCacheExpiry(Settings::physics().mLineofsightKeepInactiveCache)
        , mAdvanceSimulation(false)
        , mNextLOS(0)
        , mFrameNumber(0)
        , mTimer(osg::Timer::instance())
        , mPrevStepCount(1)
        , mBudget(physicsDt)
        , mAsyncBudget(0.0f)
        , mBudgetCursor(0)
        , mAsyncStartTime(0)
        , mTimeBegin(0)
        , mTimeEnd(0)
        , mFrameStart(0)
        , mSimulationBarrier(nullptr)
    {
        Log(Debug::Info) << "Using " << mNumThreads << " async physics threads";
        if (jobSystem->GetMaxConcurrency() <= 1)
            mLOSCacheExpiry = 0;
    }

    PhysicsTaskScheduler::~PhysicsTaskScheduler() {}

    std::tuple<unsigned, float> PhysicsTaskScheduler::calculateStepConfig(float timeAccum) const
    {
        int maxAllowedSteps = 2;
        int numSteps = static_cast<int>(timeAccum / mDefaultPhysicsDt);

        // adjust maximum step count based on whether we're likely physics bottlenecked or not
        // if maxAllowedSteps ends up higher than numSteps, we will not invoke delta time
        // if it ends up lower than numSteps, but greater than 1, we will run a number of true delta time physics steps
        // that we expect to be within budget if it ends up lower than numSteps and also 1, we will run a single delta
        // time physics step if we did not do this, and had a fixed step count limit, we would have an unnecessarily low
        // render framerate if we were only physics bottlenecked, and we would be unnecessarily invoking true delta time
        // if we were only render bottlenecked

        // get physics timing stats
        float budgetMeasurement = std::max(mBudget.get(), mAsyncBudget.get());
        // time spent per step in terms of the intended physics framerate
        budgetMeasurement /= mDefaultPhysicsDt;
        // ensure sane minimum value
        budgetMeasurement = std::max(0.00001f, budgetMeasurement);
        // we're spending almost or more than realtime per physics frame; limit to a single step
        if (budgetMeasurement > 0.95)
            maxAllowedSteps = 1;
        // physics is fairly cheap; limit based on expense
        if (budgetMeasurement < 0.5)
            maxAllowedSteps = static_cast<int>(std::ceil(1.0 / budgetMeasurement));
        // limit to a reasonable amount
        maxAllowedSteps = std::min(10, maxAllowedSteps);

        // fall back to delta time for this frame if fixed timestep physics would fall behind
        float actualDelta = mDefaultPhysicsDt;
        if (numSteps > maxAllowedSteps)
        {
            numSteps = maxAllowedSteps;
            // ensure that we do not simulate a frame ahead when doing delta time; this reduces stutter and latency
            // this causes interpolation to 100% use the most recent physics result when true delta time is happening
            // and we deliberately simulate up to exactly the timestamp that we want to render
            actualDelta = timeAccum / float(numSteps + 1);
            // actually: if this results in a per-step delta less than the target physics steptime, clamp it
            // this might reintroduce some stutter, but only comes into play in obscure cases
            // (because numSteps is originally based on mDefaultPhysicsDt, this won't cause us to overrun)
            actualDelta = std::max(actualDelta, mDefaultPhysicsDt);
        }

        return std::make_tuple(static_cast<unsigned>(numSteps), actualDelta);
    }

    void PhysicsTaskScheduler::applyQueuedMovements(float& timeAccum, std::vector<Simulation>& simulations,
        osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats)
    {
        // This function is called before Jolt physics step is taken, main thread
        assert(mSimulations != &simulations);
        prepareWork(timeAccum, simulations, frameStart, frameNumber, stats);
    }

    void PhysicsTaskScheduler::prepareWork(float& timeAccum, std::vector<Simulation>& simulations,
        osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats)
    {
        // This function run in the main thread to prepare data for job dispatch
        osg::Timer_t timeStart = mTimer->tick();

        // start by finishing previous background computation
        syncSimulation();

        if (mAdvanceSimulation)
            mAsyncBudget.update(mTimer->delta_s(mAsyncStartTime, mTimeEnd), mPrevStepCount, mBudgetCursor);
        updateStats(frameStart, frameNumber, stats);

        auto [numSteps, newDelta] = calculateStepConfig(timeAccum);
        timeAccum -= static_cast<float>(numSteps) * newDelta;

        // init
        const Visitors::InitPosition vis{ mPhysicsSystem };
        for (auto& sim : simulations)
        {
            std::visit(vis, sim);
        }

        mPrevStepCount = numSteps;
        mRemainingSteps = numSteps;
        mTimeAccum = timeAccum;
        mPhysicsDt = newDelta;
        mSimulations = &simulations;
        mAdvanceSimulation = (mRemainingSteps != 0);
        mNumJobs = static_cast<int>(mSimulations->size());
        mNextLOS.store(0, std::memory_order_relaxed);

        if (mAdvanceSimulation)
        {
            mWorldFrameData = std::make_unique<WorldFrameData>();
            mBudgetCursor += 1;

            // Resets simulation timers
            mAsyncStartTime = mTimer->tick();
            mBudget.update(mTimer->delta_s(timeStart, mTimer->tick()), 1, mBudgetCursor);

            // Dispatches jobs to be completed asynchronously (probably)
            // physics system must call syncSimulation to guarantee they have completed
            assert(mSimulationBarrier == nullptr);
            mSimulationBarrier = mJobSystem->CreateBarrier();
            JPH::JobHandle handle = mJobSystem->CreateJob(
                "MWSimulation", JPH::Color::sBlue, [this]() { doSimulation(); }, 0);
            mSimulationBarrier->AddJob(handle);
        }
    }

    void PhysicsTaskScheduler::syncSimulation()
    {
        waitForSimulationBarrier();

        if (mSimulations != nullptr)
        {
            // For each simulation, call the Sync method
            const Visitors::Sync vis{ mAdvanceSimulation, mTimeAccum, mPhysicsDt, this };
            for (auto& sim : *mSimulations)
                std::visit(vis, sim);

            mSimulations->clear();
            mSimulations = nullptr;
        }
    }

    void PhysicsTaskScheduler::waitForSimulationBarrier()
    {
        if (mSimulationBarrier)
        {
            mJobSystem->WaitForJobs(mSimulationBarrier);
            mJobSystem->DestroyBarrier(mSimulationBarrier);
            mSimulationBarrier = nullptr;
        }
    }

    void PhysicsTaskScheduler::resetSimulation(const ActorMap& actors)
    {
        waitForSimulationBarrier();

        mBudget.reset(mDefaultPhysicsDt);
        mAsyncBudget.reset(0.0f);
        if (mSimulations != nullptr)
        {
            mSimulations->clear();
            mSimulations = nullptr;
        }
        for (const auto& [_, actor] : actors)
        {
            actor->updatePosition();
            actor->updateCollisionObjectPosition();
        }
    }

    JPH::Body* PhysicsTaskScheduler::createPhysicsBody(JPH::BodyCreationSettings& settings)
    {
        JPH::BodyInterface& bodyInterface = mPhysicsSystem->GetBodyInterface();
        JPH::Body* body = bodyInterface.CreateBody(settings);
        return body;
    }

    void PhysicsTaskScheduler::removeCollisionObject(JPH::Body* joltBody)
    {
        mPhysicsSystem->GetBodyInterface().RemoveBody(joltBody->GetID());
    }

    void PhysicsTaskScheduler::destroyCollisionObject(JPH::Body* joltBody)
    {
        mPhysicsSystem->GetBodyInterface().DestroyBody(joltBody->GetID());
    }

    void PhysicsTaskScheduler::addCollisionObject(JPH::Body* joltBody, bool activate)
    {
        mPhysicsSystem->GetBodyInterface().AddBody(
            joltBody->GetID(), activate ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);
    }

    bool PhysicsTaskScheduler::getLineOfSight(
        const std::shared_ptr<Actor>& actor1, const std::shared_ptr<Actor>& actor2)
    {
        MaybeExclusiveLock lock(mLOSCacheMutex, mLockingPolicy);

        auto req = LOSRequest(actor1, actor2);
        auto result = std::find(mLOSCache.begin(), mLOSCache.end(), req);
        if (result == mLOSCache.end())
        {
            req.mResult = hasLineOfSight(actor1.get(), actor2.get());
            mLOSCache.push_back(req);
            return req.mResult;
        }
        result->mAge = 0;
        return result->mResult;
    }

    void PhysicsTaskScheduler::refreshLOSCache()
    {
        MaybeExclusiveLock lock(mLOSCacheMutex, mLockingPolicy);
        int job = 0;
        const int numLOS = static_cast<int>(mLOSCache.size());
        while ((job = mNextLOS.fetch_add(1, std::memory_order_relaxed)) < numLOS)
        {
            auto& req = mLOSCache[job];
            auto actorPtr1 = req.mActors[0].lock();
            auto actorPtr2 = req.mActors[1].lock();

            if (req.mAge++ > mLOSCacheExpiry || !actorPtr1 || !actorPtr2)
                req.mStale = true;
            else
                req.mResult = hasLineOfSight(actorPtr1.get(), actorPtr2.get());
        }
    }

    void PhysicsTaskScheduler::updatePtrAabb(const std::shared_ptr<PtrHolder>& ptr)
    {
        if (const auto actor = std::dynamic_pointer_cast<Actor>(ptr))
        {
            actor->updateCollisionObjectPosition();
        }
        else if (const auto object = std::dynamic_pointer_cast<Object>(ptr))
        {
            object->commitPositionChange();
        }
    }

    void PhysicsTaskScheduler::updateActorsPositions()
    {
        const Visitors::UpdatePosition impl{ mPhysicsSystem };
        const Visitors::WithLockedPtr<Visitors::UpdatePosition, MaybeExclusiveLock> vis{ impl, mSimulationMutex,
            mLockingPolicy };
        for (Simulation& sim : *mSimulations)
            std::visit(vis, sim);
    }

    bool PhysicsTaskScheduler::hasLineOfSight(const Actor* actor1, const Actor* actor2)
    {
        const float actor1EyeHeight = static_cast<float>(actor1->getHalfExtents().z() * 0.9);
        const float actor2EyeHeight = static_cast<float>(actor2->getHalfExtents().z() * 0.9);
        JPH::RVec3 from = Misc::Convert::toJolt<JPH::RVec3>(
            actor1->getCollisionObjectPosition() + osg::Vec3f(0.0f, 0.0f, actor1EyeHeight)); // eye level
        JPH::RVec3 to = Misc::Convert::toJolt<JPH::RVec3>(
            actor2->getCollisionObjectPosition() + osg::Vec3f(0.0f, 0.0f, actor2EyeHeight));

        auto diff = to - from;
        JPH::RRayCast ray(from, Misc::Convert::toJolt<JPH::Vec3>(diff));

        const std::uint32_t broadPhaseLayer = static_cast<std::uint32_t>(BroadPhaseLayers::WORLD.GetValue());
        if (broadPhaseLayer >= 32)
            return true;
        const std::uint32_t broadPhaseMask = 1u << broadPhaseLayer;
        MaskedBroadPhaseLayerFilter broadphaseLayerFilter(broadPhaseMask);
        MaskedObjectLayerFilter objectLayerFilter(Layers::WORLD | Layers::HEIGHTMAP | Layers::DOOR);

        JPH::RayCastResult ioHit;
        const bool didRayHit
            = mPhysicsSystem->GetNarrowPhaseQuery().CastRay(ray, ioHit, broadphaseLayerFilter, objectLayerFilter);
        return !didRayHit;
    }

    void PhysicsTaskScheduler::doSimulation()
    {
        while (mRemainingSteps)
        {
            // Before any jobs spawned for this sim step
            afterPreStep();

            const uint32_t jobCount = mNumJobs;
            const uint32_t groupSize = std::clamp((jobCount + mNumThreads - 1) / mNumThreads, 2u, 32u);
            const uint32_t groupCount = (jobCount + groupSize - 1) / groupSize;

            std::vector<JPH::JobHandle> handles;
            JPH::JobSystem::Barrier* barrier = mJobSystem->CreateBarrier();
            handles.reserve(groupCount);
            for (uint32_t groupIndex = 0; groupIndex < groupCount; ++groupIndex)
            {
                // For each group, generate one real job
                const auto& jobGroup = [this, jobCount, groupSize, groupIndex]() {
                    assert(mWorldFrameData != nullptr);
                    const Visitors::Move impl{ mPhysicsDt, mPhysicsSystem, mWorldFrameData.get() };
                    const Visitors::WithLockedPtr<Visitors::Move, MaybeLock> vis{ impl, mSimulationMutex,
                        mLockingPolicy };

                    // Calculate the current group's offset into the jobs:
                    const uint32_t groupJobOffset = groupIndex * groupSize;
                    const uint32_t groupJobEnd = std::min(groupJobOffset + groupSize, jobCount);

                    // Inside the group, loop through all job indices and execute job for each index:
                    for (uint32_t i = groupJobOffset; i < groupJobEnd; ++i)
                    {
                        std::visit(vis, (*mSimulations)[i]);
                    }
                };

                // Try to push a new job until it is pushed successfully:
                JPH::JobHandle handle = mJobSystem->CreateJob("MWSimulationStep", JPH::Color::sBlue, jobGroup, 0);
                handles.push_back(handle);
            }

            // Wait for step to complete
            barrier->AddJobs(handles.data(), static_cast<uint>(handles.size()));
            mJobSystem->WaitForJobs(barrier);
            mJobSystem->DestroyBarrier(barrier);

            // After jobs spawned for this sim step and have completed
            afterPostStep();
        }

        // All steps completed and jobs are no longer active
        refreshLOSCache();
        afterPostSim();
    }

    void PhysicsTaskScheduler::updateStats(osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats)
    {
        if (!stats.collectStats("engine"))
            return;
        if (mFrameNumber == frameNumber - 1)
        {
            stats.setAttribute(mFrameNumber, "physicsworker_time_begin", mTimer->delta_s(mFrameStart, mTimeBegin));
            stats.setAttribute(mFrameNumber, "physicsworker_time_taken", mTimer->delta_s(mTimeBegin, mTimeEnd));
            stats.setAttribute(mFrameNumber, "physicsworker_time_end", mTimer->delta_s(mFrameStart, mTimeEnd));
        }
        mFrameStart = frameStart;
        mTimeBegin = mTimer->tick();
        mFrameNumber = frameNumber;
    }

    void PhysicsTaskScheduler::debugDraw()
    {
        mDebugDrawer->step();
    }

    void* PhysicsTaskScheduler::getUserPointer(const JPH::BodyID bodyId) const
    {
        if (bodyId.IsInvalid())
        {
            return nullptr;
        }

        JPH::BodyLockRead lock(mPhysicsSystem->GetBodyLockInterface(), bodyId);
        if (lock.Succeeded())
        {
            const JPH::Body& body = lock.GetBody();
            return reinterpret_cast<void*>(static_cast<uintptr_t>(body.GetUserData()));
        }
        return nullptr;
    }

    void PhysicsTaskScheduler::releaseSharedStates()
    {
        waitForSimulationBarrier();

        if (mSimulations != nullptr)
        {
            mSimulations->clear();
            mSimulations = nullptr;
        }
        mUpdateAabb.clear();
    }

    void PhysicsTaskScheduler::afterPreStep()
    {
        if (!mRemainingSteps)
            return;
        const Visitors::PreStep impl{ mPhysicsSystem };
        const Visitors::WithLockedPtr<Visitors::PreStep, MaybeExclusiveLock> vis{ impl, mSimulationMutex,
            mLockingPolicy };
        for (auto& sim : *mSimulations)
            std::visit(vis, sim);
    }

    void PhysicsTaskScheduler::afterPostStep()
    {
        if (mRemainingSteps)
        {
            --mRemainingSteps;
            updateActorsPositions();
        }
    }

    void PhysicsTaskScheduler::afterPostSim()
    {
        {
            MaybeExclusiveLock lock(mLOSCacheMutex, mLockingPolicy);
            mLOSCache.erase(
                std::remove_if(mLOSCache.begin(), mLOSCache.end(), [](const LOSRequest& req) { return req.mStale; }),
                mLOSCache.end());
        }
        mTimeEnd = mTimer->tick();
    }
}

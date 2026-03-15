#ifndef OPENMW_MWPHYSICS_MTPHYSICS_H
#define OPENMW_MWPHYSICS_MTPHYSICS_H

#include <atomic>
#include <condition_variable>
#include <memory>
#include <optional>
#include <set>
#include <shared_mutex>
#include <thread>
#include <unordered_set>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <osg/Timer>

#include "components/misc/budgetmeasurement.hpp"
#include "physicssystem.hpp"
#include "ptrholder.hpp"

namespace JPH
{
    class JobSystem;
}

namespace Misc
{
    class Barrier;
}

namespace MWRender
{
    class JoltDebugDrawer;
}

namespace MWPhysics
{
    enum class LockingPolicy
    {
        NoLocks,
        AllowSharedLocks,
    };

    class PhysicsTaskScheduler
    {
    public:
        PhysicsTaskScheduler(float physicsDt, JPH::PhysicsSystem* physicsSystem, MWRender::JoltDebugDrawer* debugDrawer,
            JPH::JobSystem* jobSystem);
        ~PhysicsTaskScheduler();

        /// @brief move actors taking into account desired movements and collisions
        /// @param numSteps how much simulation step to run
        /// @param timeAccum accumulated time from previous run to interpolate movements
        /// @param actorsData per actor data needed to compute new positions
        /// @return new position of each actor
        void applyQueuedMovements(float& timeAccum, std::vector<Simulation>& simulations, osg::Timer_t frameStart,
            unsigned int frameNumber, osg::Stats& stats);

        void resetSimulation(const ActorMap& actors);
        void syncSimulation();

        void addCollisionObject(JPH::Body* joltBody, bool activate = false);
        void removeCollisionObject(JPH::Body* collisionObject);
        void destroyCollisionObject(JPH::Body* collisionObject);
        JPH::Body* createPhysicsBody(JPH::BodyCreationSettings& settings);

        std::shared_mutex& getSimulationMutex() { return mSimulationMutex; }

        void debugDraw();
        bool getLineOfSight(const std::shared_ptr<Actor>& actor1, const std::shared_ptr<Actor>& actor2);
        void* getUserPointer(const JPH::BodyID object) const;
        void releaseSharedStates(); // destroy all objects whose destructor can't be safely called from
                                    // ~PhysicsTaskScheduler()

        inline const JPH::BodyLockInterfaceLocking& getBodyLockInterface() const
        {
            return mPhysicsSystem->GetBodyLockInterface();
        }

        inline JPH::BodyInterface& getBodyInterface() { return mPhysicsSystem->GetBodyInterface(); }

    private:
        class WorkersSync;

        void waitForSimulationBarrier();
        void doSimulation();
        void updateActorsPositions();
        bool hasLineOfSight(const Actor* actor1, const Actor* actor2);
        void refreshLOSCache();
        void updatePtrAabb(const std::shared_ptr<PtrHolder>& ptr);
        void updateStats(osg::Timer_t frameStart, unsigned int frameNumber, osg::Stats& stats);
        std::tuple<unsigned, float> calculateStepConfig(float timeAccum) const;
        void afterPreStep();
        void afterPostStep();
        void afterPostSim();
        void prepareWork(float& timeAccum, std::vector<Simulation>& simulations, osg::Timer_t frameStart,
            unsigned int frameNumber, osg::Stats& stats);

        JPH::PhysicsSystem* mPhysicsSystem;
        JPH::JobSystem* mJobSystem;

        std::unique_ptr<WorldFrameData> mWorldFrameData;
        std::vector<Simulation>* mSimulations = nullptr;
        float mDefaultPhysicsDt;
        float mPhysicsDt;
        float mTimeAccum;
        MWRender::JoltDebugDrawer* mDebugDrawer;
        std::vector<LOSRequest> mLOSCache;
        std::set<std::weak_ptr<PtrHolder>, std::owner_less<std::weak_ptr<PtrHolder>>> mUpdateAabb;

        LockingPolicy mLockingPolicy;
        unsigned mNumThreads;
        int mNumJobs;
        unsigned mRemainingSteps;
        int mLOSCacheExpiry;
        bool mAdvanceSimulation;
        std::atomic<int> mNextLOS;

        mutable std::shared_mutex mSimulationMutex;
        mutable std::shared_mutex mLOSCacheMutex;

        unsigned int mFrameNumber;
        const osg::Timer* mTimer;

        unsigned mPrevStepCount;
        Misc::BudgetMeasurement mBudget;
        Misc::BudgetMeasurement mAsyncBudget;
        unsigned int mBudgetCursor;
        osg::Timer_t mAsyncStartTime;
        osg::Timer_t mTimeBegin;
        osg::Timer_t mTimeEnd;
        osg::Timer_t mFrameStart;
        bool mHasPendingTimingStats;

        JPH::JobSystem::Barrier* mSimulationBarrier;
    };

}
#endif

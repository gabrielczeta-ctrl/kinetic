#include <Jolt/Jolt.h>

#include <components/physicshelpers/aabb.hpp>

#include "commulativeaabb.hpp"

namespace DetourNavigator
{
    CommulativeAabb::CommulativeAabb(std::size_t lastChangeRevision, const JPH::AABox& aabb)
        : mLastChangeRevision(lastChangeRevision)
        , mAabb(aabb)
    {
    }

    bool CommulativeAabb::update(std::size_t lastChangeRevision, const JPH::AABox& aabb)
    {
        if (mLastChangeRevision != lastChangeRevision)
        {
            mLastChangeRevision = lastChangeRevision;
            mAabb = aabb;
            return true;
        }

        const JPH::AABox currentAabb = mAabb;
        mAabb.Encapsulate(aabb);

        return currentAabb != mAabb;
    }
}

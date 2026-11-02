#ifndef OPENMW_COMPONENTS_DETOURNAVIGATOR_COMMULATIVEAABB_H
#define OPENMW_COMPONENTS_DETOURNAVIGATOR_COMMULATIVEAABB_H

#include <cstddef>
#include <cstdint>

#include <Jolt/Geometry/AABox.h>
#include <Jolt/Jolt.h>

namespace DetourNavigator
{
    class CommulativeAabb
    {
    public:
        explicit CommulativeAabb(std::size_t lastChangeRevision, const JPH::AABox& aabb);

        bool update(std::size_t lastChangeRevision, const JPH::AABox& aabb);

    private:
        std::size_t mLastChangeRevision;
        JPH::AABox mAabb;
    };
}

#endif

#ifndef OPENMW_COMPONENTS_JOLTHELPERS_HEIGHTFIELD_H
#define OPENMW_COMPONENTS_JOLTHELPERS_HEIGHTFIELD_H

#include <osg/Vec3>

namespace PhysicsSystemHelpers
{
    inline osg::Vec3f getHeightfieldShift(int x, int y, int size, float minHeight, float maxHeight)
    {
        const float cellSize = static_cast<float>(size);
        return osg::Vec3f((static_cast<float>(x) + 0.5f) * cellSize, (static_cast<float>(y) + 0.5f) * cellSize,
            (maxHeight + minHeight) * 0.5f);
    }
}

#endif

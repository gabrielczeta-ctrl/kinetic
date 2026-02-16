#include "gettilespositions.hpp"
#include "settings.hpp"
#include "settingsutils.hpp"
#include "tilebounds.hpp"
#include "tileposition.hpp"

#include <components/misc/convert.hpp>

#include <osg/Matrixd>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

namespace DetourNavigator
{
    TilesPositionsRange makeTilesPositionsRange(
        const osg::Vec2f& aabbMin, const osg::Vec2f& aabbMax, const RecastSettings& settings)
    {
        osg::Vec2f min = toNavMeshCoordinates(settings, aabbMin);
        osg::Vec2f max = toNavMeshCoordinates(settings, aabbMax);

        const float border = getBorderSize(settings);
        min -= osg::Vec2f(border, border);
        max += osg::Vec2f(border, border);

        TilePosition minTile = getTilePosition(settings, min);
        TilePosition maxTile = getTilePosition(settings, max);

        if (minTile.x() > maxTile.x())
            std::swap(minTile.x(), maxTile.x());

        if (minTile.y() > maxTile.y())
            std::swap(minTile.y(), maxTile.y());

        return { minTile, maxTile + osg::Vec2i(1, 1) };
    }

    TilesPositionsRange makeTilesPositionsRange(
        const JPH::Shape& shape, const osg::Matrixd& transform, const RecastSettings& settings)
    {
        const TileBounds bounds = makeObjectTileBounds(shape, transform);
        return makeTilesPositionsRange(bounds.mMin, bounds.mMax, settings);
    }

    TilesPositionsRange makeTilesPositionsRange(const JPH::Shape& shape, const osg::Matrixd& transform,
        const TileBounds& bounds, const RecastSettings& settings)
    {
        if (const auto intersection = getIntersection(bounds, makeObjectTileBounds(shape, transform)))
            return makeTilesPositionsRange(intersection->mMin, intersection->mMax, settings);
        return {};
    }

    TilesPositionsRange makeTilesPositionsRange(
        const int cellSize, const osg::Vec3f& shift, const RecastSettings& settings)
    {
        const int halfCellSize = cellSize / 2;
        const auto halfCellSizeF = static_cast<osg::Vec3f::value_type>(halfCellSize);

        osg::Vec3f aabbMin = shift + osg::Vec3f(-halfCellSizeF, -halfCellSizeF, 0.0f);
        osg::Vec3f aabbMax = shift + osg::Vec3f(halfCellSizeF, halfCellSizeF, 0.0f);

        aabbMin.set(std::min(aabbMin.x(), aabbMax.x()), std::min(aabbMin.y(), aabbMax.y()), aabbMin.z());
        aabbMax.set(std::max(aabbMin.x(), aabbMax.x()), std::max(aabbMin.y(), aabbMax.y()), aabbMax.z());

        return makeTilesPositionsRange(Misc::Convert::toOsgXY(aabbMin), Misc::Convert::toOsgXY(aabbMax), settings);
    }

    TilesPositionsRange getIntersection(const TilesPositionsRange& a, const TilesPositionsRange& b) noexcept
    {
        const int beginX = std::max(a.mBegin.x(), b.mBegin.x());
        const int endX = std::min(a.mEnd.x(), b.mEnd.x());
        if (beginX > endX)
            return {};
        const int beginY = std::max(a.mBegin.y(), b.mBegin.y());
        const int endY = std::min(a.mEnd.y(), b.mEnd.y());
        if (beginY > endY)
            return {};
        return TilesPositionsRange{ TilePosition(beginX, beginY), TilePosition(endX, endY) };
    }

    TilesPositionsRange getUnion(const TilesPositionsRange& a, const TilesPositionsRange& b) noexcept
    {
        const int beginX = std::min(a.mBegin.x(), b.mBegin.x());
        const int endX = std::max(a.mEnd.x(), b.mEnd.x());
        const int beginY = std::min(a.mBegin.y(), b.mBegin.y());
        const int endY = std::max(a.mEnd.y(), b.mEnd.y());
        return TilesPositionsRange{ .mBegin = TilePosition(beginX, beginY), .mEnd = TilePosition(endX, endY) };
    }
}

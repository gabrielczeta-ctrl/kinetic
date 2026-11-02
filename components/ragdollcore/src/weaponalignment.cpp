#include <ragdollcore/weaponalignment.hpp>

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    namespace
    {
        bool finite(JPH::Vec3Arg value)
        {
            return std::isfinite(value.GetX()) && std::isfinite(value.GetY()) && std::isfinite(value.GetZ());
        }
    }

    std::optional<JPH::Quat> alignedStrikeHandRotation(const WeaponStrikeAxes& axes,
        JPH::Vec3Arg strikeDirection, JPH::Vec3Arg swingTangent)
    {
        if (!finite(strikeDirection) || strikeDirection.LengthSq() < 1e-8f)
            return std::nullopt;
        if (!finite(axes.mLongAxis) || axes.mLongAxis.LengthSq() < 1e-8f)
            return std::nullopt;

        const JPH::Vec3 longAxis = axes.mLongAxis.Normalized();
        JPH::Vec3 edge = finite(axes.mEdgeAxis) ? axes.mEdgeAxis : JPH::Vec3::sZero();
        edge -= longAxis * edge.Dot(longAxis);
        if (edge.LengthSq() < 1e-8f)
        {
            // No usable edge input: any stable perpendicular keeps the long
            // axis aligned without inventing a roll preference.
            edge = std::abs(longAxis.GetX()) < 0.9f
                ? longAxis.Cross(JPH::Vec3::sAxisX()) : longAxis.Cross(JPH::Vec3::sAxisY());
        }
        edge = edge.Normalized();

        const JPH::Vec3 forward = strikeDirection.Normalized();
        const JPH::Quat alignLong = JPH::Quat::sFromTo(longAxis, forward);
        const JPH::Vec3 rolledEdge = (alignLong * edge).Normalized();

        if (finite(swingTangent) && swingTangent.LengthSq() > 1e-8f)
        {
            JPH::Vec3 desiredEdge = swingTangent - forward * swingTangent.Dot(forward);
            if (desiredEdge.LengthSq() > 1e-8f)
            {
                desiredEdge = desiredEdge.Normalized();
                const float cosine = std::clamp(rolledEdge.Dot(desiredEdge), -1.0f, 1.0f);
                const float sine = forward.Dot(rolledEdge.Cross(desiredEdge));
                const float angle = std::atan2(sine, cosine);
                const JPH::Quat roll = JPH::Quat::sRotation(forward, angle);
                return std::optional((roll * alignLong).Normalized());
            }
        }
        return std::optional(alignLong.Normalized());
    }

    std::optional<JPH::Quat> continuousStrikeHandRotation(const WeaponStrikeAxes& axes,
        JPH::Vec3Arg strikeDirection, JPH::Vec3Arg swingTangent, JPH::QuatArg previous)
    {
        if (!finite(strikeDirection) || strikeDirection.LengthSq() < 1e-8f
            || !previous.IsNormalized() || previous.IsNaN())
            return std::nullopt;
        const auto forward = strikeDirection.Normalized();
        auto reference = previous * axes.mEdgeAxis;
        reference -= forward * reference.Dot(forward);
        auto tangent = swingTangent - forward * swingTangent.Dot(forward);
        if (!finite(tangent) || tangent.LengthSq() < 1e-8f)
            tangent = reference;
        else if (tangent.Dot(reference) < 0.0f)
            tangent = -tangent;
        return alignedStrikeHandRotation(axes, forward, tangent);
    }

    std::optional<WeaponShapeProfile> weaponShapeProfile(std::string_view record)
    {
        if (record == "iron claymore")
            return WeaponShapeProfile{ JPH::Vec3::sAxisZ(), JPH::Vec3::sAxisX(), 0.08f };
        if (record == "iron battle axe")
            return WeaponShapeProfile{ JPH::Vec3::sAxisZ(), JPH::Vec3::sAxisX(), 0.55f };
        if (record == "iron warhammer")
            return WeaponShapeProfile{ JPH::Vec3::sAxisZ(), JPH::Vec3::sAxisX(), 0.62f };
        return std::nullopt;
    }
}

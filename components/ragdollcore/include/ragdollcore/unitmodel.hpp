#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_UNITMODEL_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_UNITMODEL_H

namespace RagdollCore::UnitModel
{
    // The current Jolt port simulates in OpenMW world units. Keep every ragdoll
    // conversion at this boundary until a complete SI migration is validated.
    inline constexpr float UnitsPerMeter = 69.99125109f;
    inline constexpr float GravityMetersPerSecondSquared = 8.96f;
    inline constexpr float GravityUnitsPerSecondSquared = GravityMetersPerSecondSquared * UnitsPerMeter;
    inline constexpr float CellSizeUnits = 8192.0f;

    constexpr float fromMeters(float value)
    {
        return value * UnitsPerMeter;
    }

    constexpr float toMeters(float value)
    {
        return value / UnitsPerMeter;
    }
}

#endif

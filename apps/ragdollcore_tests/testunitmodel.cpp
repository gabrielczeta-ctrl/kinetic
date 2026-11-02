#include <ragdollcore/unitmodel.hpp>

#include <gtest/gtest.h>

namespace
{
    TEST(RagdollCoreUnitModelTest, convertsMetersAtTheCoreBoundary)
    {
        EXPECT_FLOAT_EQ(RagdollCore::UnitModel::fromMeters(1.0f), RagdollCore::UnitModel::UnitsPerMeter);
        EXPECT_FLOAT_EQ(RagdollCore::UnitModel::toMeters(RagdollCore::UnitModel::UnitsPerMeter), 1.0f);
    }

    TEST(RagdollCoreUnitModelTest, usesAuthoritativeOpenMWGravity)
    {
        EXPECT_FLOAT_EQ(RagdollCore::UnitModel::GravityMetersPerSecondSquared, 8.96f);
        EXPECT_FLOAT_EQ(RagdollCore::UnitModel::GravityUnitsPerSecondSquared,
            RagdollCore::UnitModel::GravityMetersPerSecondSquared * RagdollCore::UnitModel::UnitsPerMeter);
    }
}

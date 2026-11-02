# A9 diagnostic candidate. Absent environment preserves the pinned reducer.
insert_jolt_code("Jolt/Physics/Collision/ManifoldBetweenTwoFaces.cpp"
    "#include <Jolt/Jolt.h>"
    "#ifdef _MSC_VER\n#pragma warning(disable: 4996) // Read-only diagnostic environment.\n#endif\n#include <cstdlib>")
replace_jolt_code("Jolt/Physics/Collision/ManifoldBetweenTwoFaces.cpp"
    "float v = max(cMinDistanceSq, (projected[i] - point1v).LengthSq()) * penetration_depth_sq[i];"
    "float v = max(cMinDistanceSq, (projected[i] - point1v).LengthSq()) * (std::getenv(\"OPENMW_JOLT_GEOMETRIC_SECOND_CONTACT\") ? 1.0f : penetration_depth_sq[i]);")

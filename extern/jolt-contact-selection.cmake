# Read-only, opt-in contact selection diagnostic for the pinned Jolt source.
# Process-local update ordinals: use a single world and one worker for analysis.
insert_jolt_code("Jolt/Physics/PhysicsSystem.cpp" "#include <Jolt/Jolt.h>"
    "#ifdef _MSC_VER\n#pragma warning(disable: 4996) // getenv: read-only diagnostic environment, no concurrent mutation.\n#endif")
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/jolt-contact-selection.inc" contact_selection_helper)
insert_jolt_code("Jolt/Physics/PhysicsSystem.cpp" "JPH_NAMESPACE_BEGIN"
    "#include <atomic>\n#include <cstdio>\n#include <cstdlib>")
insert_jolt_code("Jolt/Physics/PhysicsSystem.cpp" "void PhysicsSystem::ProcessBodyPair("
    "${contact_selection_helper}")
insert_jolt_code("Jolt/Physics/PhysicsSystem.cpp" "JPH_NAMESPACE_BEGIN"
    "static std::atomic<unsigned long long> sContactSelectionUpdate{0};")
replace_jolt_code("Jolt/Physics/PhysicsSystem.cpp"
    "EPhysicsUpdateError PhysicsSystem::Update(float inDeltaTime, int inCollisionSteps, TempAllocator *inTempAllocator, JobSystem *inJobSystem)\n{"
    "EPhysicsUpdateError PhysicsSystem::Update(float inDeltaTime, int inCollisionSteps, TempAllocator *inTempAllocator, JobSystem *inJobSystem)\n{\n\tif (std::getenv(\"OPENMW_JOLT_CONTACT_SELECTION\")) ++sContactSelectionUpdate;")
insert_jolt_code("Jolt/Physics/PhysicsSystem.cpp"
    "\t// If the cache hasn't handled this body pair do actual collision detection"
    "\tTraceContactSelection(pair_handled ? \"cache_hit\" : \"cache_miss\", *body1, *body2, nullptr);")
insert_jolt_code("Jolt/Physics/PhysicsSystem.cpp"
    "\t\t\t\t// If we still have too many points, prune them now"
    "\t\t\t\tTraceContactSelection(\"before_reduce\", *body1, *body2, &manifold);")
# Remove the initial broad comment-anchor insertion (also matched the non-reduced collector).
set(selection_path "${JOLT_SOURCE_DIR}/Jolt/Physics/PhysicsSystem.cpp")
file(READ "${selection_path}" selection_content)
string(REPLACE "\t\t\t\tTraceContactSelection(\"after_reduce\", *body1, *body2, &manifold);\n" "" selection_content "${selection_content}")
file(WRITE "${selection_path}" "${selection_content}")
insert_jolt_code("Jolt/Physics/PhysicsSystem.cpp"
    "\t\t\t\tconstraint_created |= mContactManager.AddContactConstraint(ioContactAllocator, body_pair_handle, *body1, *body2, manifold);"
    "\t\t\t\tTraceContactSelection(\"after_reduce\", *body1, *body2, &manifold);")

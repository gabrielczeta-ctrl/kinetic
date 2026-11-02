# Read-only extension to pinned Jolt 5.5.0. Contact callbacks precede the solve;
# a negative penetration depth alone does not tell us whether support occurred.
# Keep this patch reproducible on clean restores and fail on an unknown source.
function(insert_jolt_code relative_path anchor addition)
    set(path "${JOLT_SOURCE_DIR}/${relative_path}")
    file(READ "${path}" content)
    string(FIND "${content}" "${addition}" applied)
    if(NOT applied EQUAL -1)
        return()
    endif()
    string(FIND "${content}" "${anchor}" found)
    if(found EQUAL -1)
        message(FATAL_ERROR "Jolt contact impulse patch anchor missing: ${relative_path}")
    endif()
    string(REPLACE "${anchor}" "${addition}\n${anchor}" content "${content}")
    file(WRITE "${path}" "${content}")
endfunction()
# Replace an exact source span; idempotent, fails on an unknown source.
function(replace_jolt_code relative_path before after)
    set(path "${JOLT_SOURCE_DIR}/${relative_path}")
    file(READ "${path}" content)
    string(FIND "${content}" "${after}" applied)
    if(NOT applied EQUAL -1)
        return()
    endif()
    string(FIND "${content}" "${before}" found)
    if(found EQUAL -1)
        message(FATAL_ERROR "Jolt patch anchor missing: ${relative_path}")
    endif()
    string(REPLACE "${before}" "${after}" content "${content}")
    file(WRITE "${path}" "${content}")
endfunction()

insert_jolt_code("Jolt/Physics/Constraints/ContactConstraintManager.h"
    "	/// Get the number of contact constraints that were found"
    [=[	/// Magnitude of the two solved tangential impulses, after Update.
	bool GetContactFrictionImpulse(const SubShapeIDPair &inPair, uint inPointIndex, float &outMagnitude) const;
]=])

insert_jolt_code("Jolt/Physics/Constraints/ContactConstraintManager.cpp"
    "bool ContactConstraintManager::WereBodiesInContact("
    [=[bool ContactConstraintManager::GetContactFrictionImpulse(const SubShapeIDPair &inPair, uint inPointIndex, float &outMagnitude) const
{
	const ManifoldCache &read_cache = mCache[mCacheWriteIdx ^ 1];
	const MKeyValue *kv = read_cache.Find(inPair, inPair.GetHash());
	if (kv == nullptr)
		return false;
	const CachedManifold &manifold = kv->GetValue();
	if ((manifold.mFlags & uint16(CachedManifold::EFlags::CCDContact)) != 0 || inPointIndex >= manifold.mNumContactPoints)
		return false;
	const auto &lambda = manifold.mContactPoints[inPointIndex].mFrictionLambda;
	outMagnitude = std::sqrt(lambda[0] * lambda[0] + lambda[1] * lambda[1]);
	return true;
}

]=])

insert_jolt_code("Jolt/Physics/PhysicsSystem.h"
    "	/// Check if 2 bodies were in contact during the last simulation step."
    [=[	/// Read solved tangential impulse magnitude using callback pair/point order.
	bool GetContactFrictionImpulse(const SubShapeIDPair &inPair, uint inPointIndex, float &outMagnitude) const
	{
		return mContactManager.GetContactFrictionImpulse(inPair, inPointIndex, outMagnitude);
	}

]=])

insert_jolt_code("Jolt/Physics/Constraints/ContactConstraintManager.h"
    "	/// Get the number of contact constraints that were found"
    [=[	/// Read the solved normal impulse for a callback's point index, after Update.
	/// The pair must use the same body/subshape order as the contact callback.
	/// Returns false for absent points and CCD-only callbacks (no cached impulse).
	bool GetContactNormalImpulse(const SubShapeIDPair &inPair, uint inPointIndex, float &outImpulse) const;
]=])

insert_jolt_code("Jolt/Physics/Constraints/ContactConstraintManager.cpp"
    "bool ContactConstraintManager::WereBodiesInContact("
    [=[bool ContactConstraintManager::GetContactNormalImpulse(const SubShapeIDPair &inPair, uint inPointIndex, float &outImpulse) const
{
	const ManifoldCache &read_cache = mCache[mCacheWriteIdx ^ 1];
	const MKeyValue *kv = read_cache.Find(inPair, inPair.GetHash());
	if (kv == nullptr)
		return false;
	const CachedManifold &manifold = kv->GetValue();
	if ((manifold.mFlags & uint16(CachedManifold::EFlags::CCDContact)) != 0 || inPointIndex >= manifold.mNumContactPoints)
		return false;
	outImpulse = manifold.mContactPoints[inPointIndex].mNonPenetrationLambda;
	return true;
}

]=])

insert_jolt_code("Jolt/Physics/PhysicsSystem.h"
    "	/// Check if 2 bodies were in contact during the last simulation step."
    [=[	/// Read a contact point's actual normal impulse after Update, never from a callback.
	/// Uses callback body/subshape order and point index. CCD-only contacts return false.
	bool GetContactNormalImpulse(const SubShapeIDPair &inPair, uint inPointIndex, float &outImpulse) const
	{
		return mContactManager.GetContactNormalImpulse(inPair, inPointIndex, outImpulse);
	}

]=])

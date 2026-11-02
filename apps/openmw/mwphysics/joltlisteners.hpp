#ifndef OPENMW_MWPHYSICS_JOLTLISTENERS_H
#define OPENMW_MWPHYSICS_JOLTLISTENERS_H

#include <components/debug/debuglog.hpp>
#include <components/misc/convert.hpp>

#include "ptrholder.hpp"

namespace MWPhysics
{
    // Note: this is called from a job so whatever you do here needs to be thread safe.
    class JoltContactListener : public JPH::ContactListener
    {
    public:
        JoltContactListener() {}

        // Allows you to ignore a contact before it is created (using layers to not make objects collide is cheaper!)
        virtual JPH::ValidateResult OnContactValidate(const JPH::Body& inBody1, const JPH::Body& inBody2,
            JPH::RVec3Arg inBaseOffset, const JPH::CollideShapeResult& inCollisionResult) override
        {
            bool canCollide = true;
            PtrHolder* ptr;

            ptr = Misc::Convert::toPointerFromUserData<PtrHolder>(inBody1.GetUserData());
            if (ptr)
                canCollide = ptr->onContactValidate(inBody2);

            if (canCollide)
            {
                ptr = Misc::Convert::toPointerFromUserData<PtrHolder>(inBody2.GetUserData());
                if (ptr)
                    canCollide = ptr->onContactValidate(inBody1);
            }

            if (canCollide)
            {
                return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
            }

            return JPH::ValidateResult::RejectAllContactsForThisBodyPair;
        }

        // NOTE: In the OnContactAdded callback all bodies are locked already
        virtual void OnContactAdded(const JPH::Body& inBody1, const JPH::Body& inBody2,
            const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings) override
        {
            PtrHolder* ptr1 = Misc::Convert::toPointerFromUserData<PtrHolder>(inBody1.GetUserData());
            if (ptr1)
                ptr1->onContactAdded(inBody2, inManifold, ioSettings);

            PtrHolder* ptr2 = Misc::Convert::toPointerFromUserData<PtrHolder>(inBody2.GetUserData());
            if (ptr2)
                ptr2->onContactAdded(inBody1, inManifold, ioSettings);
        }
    };
}

#endif

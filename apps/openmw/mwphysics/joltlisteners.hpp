#ifndef OPENMW_MWPHYSICS_JOLTLISTENERS_H
#define OPENMW_MWPHYSICS_JOLTLISTENERS_H

#include <algorithm>

#include <components/debug/debuglog.hpp>
#include <components/misc/convert.hpp>

#include <ragdollcore/contact.hpp>
#include <ragdollcore/tuning.hpp>

#include "joltlayers.hpp"
#include "ptrholder.hpp"

namespace MWPhysics
{
    // Note: this is called from a job so whatever you do here needs to be thread safe.
    class JoltContactListener : public JPH::ContactListener
    {
        static void observeContact(const JPH::Body& body1, const JPH::Body& body2,
            const JPH::ContactManifold& manifold, float friction)
        {
            PtrHolder* ptr1 = Misc::Convert::toPointerFromUserData<PtrHolder>(body1.GetUserData());
            PtrHolder* ptr2 = Misc::Convert::toPointerFromUserData<PtrHolder>(body2.GetUserData());
            if (ptr1 == nullptr && ptr2 == nullptr)
                return;

            if (ptr1)
                ptr1->onContactObserved(body1, body2, manifold, true, friction);
            if (ptr2)
                ptr2->onContactObserved(body2, body1, manifold, false, friction);
        }

        static void adjustContact(const JPH::Body& body1, const JPH::Body& body2,
            const JPH::ContactManifold& manifold, JPH::ContactSettings& ioSettings)
        {
            if (PtrHolder* ptr1 = Misc::Convert::toPointerFromUserData<PtrHolder>(body1.GetUserData()))
                ptr1->onContactSettingsSided(body1, body2, manifold, ioSettings, true);
            if (PtrHolder* ptr2 = Misc::Convert::toPointerFromUserData<PtrHolder>(body2.GetUserData()))
                ptr2->onContactSettingsSided(body2, body1, manifold, ioSettings, false);
        }

        // OPENMW_RAGDOLL_FLOOR_UNDERSIDE (1 = on, default off): a static's
        // contact that pushes a powered ragdoll's body down from below its
        // centre is rejected. A thin floor mesh has no inside: a foot pressed
        // more than half its height into it (the ship-hold walker, MOVE-01 W2,
        // 11-03) was pushed out through the bottom and walked on its calves
        // under the floor. Rejected, the leg lifts the foot back up through it.
        static bool floorUndersideRejected()
        {
            static const auto on = RagdollCore::tuned<bool>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_FLOOR_UNDERSIDE");
                return text && text[0] == '1';
            });
            return on.get();
        }

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
                canCollide = ptr->onContactValidatePair(inBody1, inBody2);

            if (canCollide)
            {
                ptr = Misc::Convert::toPointerFromUserData<PtrHolder>(inBody2.GetUserData());
                if (ptr)
                    canCollide = ptr->onContactValidatePair(inBody2, inBody1);
            }

            if (canCollide)
            {
                // Each hit (a mesh's triangle) is checked, not the pair's first.
                if ((inBody1.GetObjectLayer() == Layers::RAGDOLL || inBody2.GetObjectLayer() == Layers::RAGDOLL)
                    && (inBody1.IsStatic() || inBody2.IsStatic()) && floorUndersideRejected())
                    return RagdollCore::rejectsFloorUnderside(inBody1, inBody2, inBaseOffset, inCollisionResult)
                        ? JPH::ValidateResult::RejectContact
                        : JPH::ValidateResult::AcceptContact;
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

            adjustContact(inBody1, inBody2, inManifold, ioSettings);
            // A contact made a sensor passes through: it is not a touch.
            if (!ioSettings.mIsSensor)
                observeContact(inBody1, inBody2, inManifold, ioSettings.mCombinedFriction);
        }

        void OnContactPersisted(const JPH::Body& inBody1, const JPH::Body& inBody2,
            const JPH::ContactManifold& inManifold, JPH::ContactSettings& ioSettings) override
        {
            adjustContact(inBody1, inBody2, inManifold, ioSettings);
            if (!ioSettings.mIsSensor)
                observeContact(inBody1, inBody2, inManifold, ioSettings.mCombinedFriction);
        }
    };
}

#endif

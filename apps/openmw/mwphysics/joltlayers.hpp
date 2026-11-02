#ifndef OPENMW_MWPHYSICS_JOLTLAYERS_H
#define OPENMW_MWPHYSICS_JOLTLAYERS_H

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>

namespace MWPhysics
{
    // Layer that objects can be in, determines which other objects it can collide with
    // JPH::ObjectLayer is either uint16 or uint32 depending on JPH_OBJECT_LAYER_BITS
    namespace Layers
    {
        // Any static mesh (floors, walls, rocks etc)
        static constexpr JPH::ObjectLayer WORLD = 1 << 0;

        // Static geometry doors that animate
        static constexpr JPH::ObjectLayer DOOR = 1 << 1;

        // Any moving actors (player, NPCs, creatures)
        static constexpr JPH::ObjectLayer ACTOR = 1 << 2;

        // Static terrain collider
        static constexpr JPH::ObjectLayer HEIGHTMAP = 1 << 3;

        // A dynamic, moving projectile (magic bolt, arrows, bolts, throwing stars etc)
        static constexpr JPH::ObjectLayer PROJECTILE = 1 << 4;

        // Water body, detection primarily used for water walking
        static constexpr JPH::ObjectLayer WATER = 1 << 5;

        // Only camera collision checks pass for these objects
        static constexpr JPH::ObjectLayer CAMERA_ONLY = 1 << 6;

        // Only visual collision checks pass for these objects
        static constexpr JPH::ObjectLayer VISUAL_ONLY = 1 << 7;

        // Dynamic objects in the world, i.e sweet roll
        static constexpr JPH::ObjectLayer DYNAMIC_WORLD = 1 << 8;

        // Debris collides only with WORLD/HEIGHTMAP, useful for corpses and effects
        static constexpr JPH::ObjectLayer DEBRIS = 1 << 9;

        // Sensors only collide with DYNAMIC_WORLD objects
        static constexpr JPH::ObjectLayer SENSOR = 1 << 10;

        // A POWERED ragdoll: a body under active control that is expected to
        // hit things. Corpses stay on DEBRIS, which by design touches nothing
        // but the ground - correct for a body flopping in a corner, and the
        // reason a swung arm passed straight through its target. This layer
        // exists so two fighters can reach each other without every corpse in
        // the cell becoming a physics obstacle.
        static constexpr JPH::ObjectLayer RAGDOLL = 1 << 11;

        // The actor capsule of a POWERED kinematic-root body. It keeps the
        // ordinary world collision (stairs, slopes, walls) but does not collide
        // with other actor capsules, so powered bodies interact through their
        // physical limbs and can stumble into each other. Unpowered actors keep
        // the ordinary ACTOR capsule, and queries that look for actors include
        // this layer where they mean "a body that is present".
        static constexpr JPH::ObjectLayer POWERED_ACTOR = 1 << 12;
    }

    enum CollisionMask
    {
        CollisionMask_Default = Layers::WORLD | Layers::HEIGHTMAP | Layers::ACTOR | Layers::DOOR,
        CollisionMask_AnyPhysical
        = Layers::WORLD | Layers::HEIGHTMAP | Layers::ACTOR | Layers::DOOR | Layers::PROJECTILE | Layers::WATER
            | Layers::POWERED_ACTOR,
    };

    // Broadphase layers
    namespace BroadPhaseLayers
    {
        constexpr static JPH::BroadPhaseLayer WORLD{ 0 };
        constexpr static JPH::BroadPhaseLayer MOVING{ 1 };
        constexpr static JPH::BroadPhaseLayer DEBRIS{ 2 };
        constexpr static JPH::BroadPhaseLayer SENSOR{ 3 };
        constexpr static unsigned int NUM_LAYERS{ 4 };
    }

    // BroadPhaseLayerInterface implementation
    class JoltBPLayerInterface final : public JPH::BroadPhaseLayerInterface
    {
    public:
        JoltBPLayerInterface() {}

        virtual unsigned int GetNumBroadPhaseLayers() const override { return BroadPhaseLayers::NUM_LAYERS; }

        // Converts an object layer into broadphase layer
        virtual JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override
        {
            switch (inLayer)
            {
                case Layers::SENSOR:
                    return BroadPhaseLayers::SENSOR;
                case Layers::DEBRIS:
                    return BroadPhaseLayers::DEBRIS;
                case Layers::ACTOR:
                case Layers::PROJECTILE:
                case Layers::RAGDOLL:
                case Layers::POWERED_ACTOR:
                    return BroadPhaseLayers::MOVING;
            }
            return BroadPhaseLayers::WORLD;
        }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
        virtual const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override
        {
            switch ((JPH::BroadPhaseLayer::Type)inLayer)
            {
                // case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::WORLD:    return "WORLD";
                // case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::DYNAMIC_WORLD:        return "DYNAMIC_WORLD";
                // case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::DEBRIS:        return "DEBRIS";
                // case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::SENSOR:        return "SENSOR";
                // case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::UNUSED:        return "UNUSED";
                default:
                    return "INVALID";
            }
        }
#endif
    };

    // This class defines a ObjectVsBroadPhaseLayerFilter::ShouldCollide function that checks if an ObjectLayer collides
    // with objects that reside in a particular BroadPhaseLayer. ObjectLayers can collide with as many BroadPhaseLayers
    // as needed, so it is possible for a collision query to visit multiple broad phase trees.
    class JoltObjectVsBroadPhaseLayerFilter : public JPH::ObjectVsBroadPhaseLayerFilter
    {
    public:
        virtual bool ShouldCollide(JPH::ObjectLayer objectLayer, JPH::BroadPhaseLayer broadPhaseLayer) const override
        {
            switch (objectLayer)
            {
                case Layers::WORLD:
                    return broadPhaseLayer == BroadPhaseLayers::MOVING;
                case Layers::ACTOR:
                case Layers::PROJECTILE:
                case Layers::POWERED_ACTOR:
                    return broadPhaseLayer == BroadPhaseLayers::WORLD || broadPhaseLayer == BroadPhaseLayers::MOVING
                        || broadPhaseLayer == BroadPhaseLayers::SENSOR;
                case Layers::DOOR:
                    return broadPhaseLayer == BroadPhaseLayers::DEBRIS || broadPhaseLayer == BroadPhaseLayers::MOVING;
                case Layers::DEBRIS:
                    return broadPhaseLayer == BroadPhaseLayers::WORLD;
                case Layers::RAGDOLL:
                    return broadPhaseLayer == BroadPhaseLayers::WORLD || broadPhaseLayer == BroadPhaseLayers::MOVING;
                case Layers::SENSOR:
                    return broadPhaseLayer == BroadPhaseLayers::MOVING || broadPhaseLayer == BroadPhaseLayers::SENSOR;
                default:
                    return false;
            }
        }
    };

    // This class defines a ObjectLayerPairFilter::ShouldCollide function that checks if an ObjectLayer collides with
    // another ObjectLayer.
    class JoltObjectLayerPairFilter : public JPH::ObjectLayerPairFilter
    {
    public:
        virtual bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override
        {
            // NOTE: This doesn't filter against actor/object collison masks, for that we use body filters and/or
            // contact validation callbacks Rather this is a high level group->group check
            switch (inObject1)
            {
                // Static layers should collide with dynamic layers
                case Layers::DOOR:
                case Layers::WORLD:
                case Layers::WATER:
                case Layers::HEIGHTMAP:
                    return inObject2 == Layers::DYNAMIC_WORLD || inObject2 == Layers::ACTOR
                        || inObject2 == Layers::PROJECTILE || inObject2 == Layers::DEBRIS
                        || inObject2 == Layers::RAGDOLL || inObject2 == Layers::POWERED_ACTOR;

                // Any dynamic/moving layers should collide with all static geometry, sensors and other dynamics
                case Layers::DYNAMIC_WORLD:
                case Layers::PROJECTILE:
                case Layers::ACTOR:
                    return inObject2 == Layers::WORLD || inObject2 == Layers::HEIGHTMAP || inObject2 == Layers::DOOR
                        || inObject2 == Layers::WATER || inObject2 == Layers::ACTOR || inObject2 == Layers::PROJECTILE
                        || inObject2 == Layers::DYNAMIC_WORLD || inObject2 == Layers::SENSOR
                        || inObject2 == Layers::POWERED_ACTOR;

                // A powered actor's capsule: world collision and a mutual
                // capsule so two powered bodies cannot interpenetrate (the
                // limbs push through and the whole ragdoll spins when the
                // stand-ins overlap). It still does not collide with ordinary
                // actor capsules; projectiles hit it.
                case Layers::POWERED_ACTOR:
                    return inObject2 == Layers::WORLD || inObject2 == Layers::HEIGHTMAP
                        || inObject2 == Layers::DOOR || inObject2 == Layers::PROJECTILE
                        || inObject2 == Layers::POWERED_ACTOR;

                // Sensors should collide with other sensors, actors and projectiles (not dynamic objects)
                case Layers::SENSOR:
                    return inObject2 == Layers::SENSOR || inObject2 == Layers::PROJECTILE || inObject2 == Layers::ACTOR;

                // Debris layer should only collide with static world for performance
                case Layers::DEBRIS:
                    return inObject2 == Layers::WORLD || inObject2 == Layers::HEIGHTMAP;

                // A powered ragdoll stands on the world and can reach other
                // powered ragdolls. Deliberately NOT actor capsules: a capsule
                // is a gameplay approximation of a body that already has a
                // physical one here, so colliding with both would have a
                // fighter shoved by its opponent's stand-in as well as by the
                // opponent. Within one ragdoll the profile's self-collision
                // groups still apply, so this only opens body-to-body.
                case Layers::RAGDOLL:
                    return inObject2 == Layers::WORLD || inObject2 == Layers::HEIGHTMAP
                        || inObject2 == Layers::DOOR || inObject2 == Layers::RAGDOLL;

                default:
                    return false;
            }
        }
    };
}

#endif

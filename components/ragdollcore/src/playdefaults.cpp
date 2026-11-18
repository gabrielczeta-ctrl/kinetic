#include <ragdollcore/playdefaults.hpp>

namespace RagdollCore
{
    namespace
    {
        // KINETIC-1 as played (frozen 2026-11-02: tag kinetic-1, the
        // playthrough's ragdoll.cfg); sorted by name. Change a play value here,
        // not in a launcher.
        constexpr PlayDefault sPlayDefaults[] = {
            // ACRO-01: the landing roll's outcome and the get-up by
            // Acrobatics (the roll's own motion parts stay off); on 11-06
            // (user: "turn on the built-but-off features for a playthrough").
            { "OPENMW_RAGDOLL_ACRO_ROLL", "1" },

            { "OPENMW_RAGDOLL_AI_AIM", "0.45,1,30,0.3" },
            { "OPENMW_RAGDOLL_AI_LINES", "0.2,0.8" },
            { "OPENMW_RAGDOLL_AI_READ", "50,10,0.6" },
            { "OPENMW_RAGDOLL_AI_RIPOSTE", "0.6,0.3,0.9,0.5" },
            { "OPENMW_RAGDOLL_AI_STAMINA", "1.5,0.3" },
            { "OPENMW_RAGDOLL_AI_THREAT", "1" },

            { "OPENMW_RAGDOLL_BEARINGS_STUMBLE", "0.75" },

            { "OPENMW_RAGDOLL_BODY_LEADS", "1,1,0" },

            { "OPENMW_RAGDOLL_CAMERA_KICK", "1.5" },

            { "OPENMW_RAGDOLL_CAPSULE_STRIKE", "250" },

            { "OPENMW_RAGDOLL_CATCH_ON_HIT", "1" },

            { "OPENMW_RAGDOLL_CLASH_ARM", "0.5" },
            { "OPENMW_RAGDOLL_CLASH_SLIDE", "1,4,0.15,150,1" },
            { "OPENMW_RAGDOLL_CLASH_V2", "1" },
            { "OPENMW_RAGDOLL_CLASH_WEIGHT", "2,2.5,3" },

            { "OPENMW_RAGDOLL_CONNECT", "4,40,0.35,2,1" },

            { "OPENMW_RAGDOLL_CONTACT_OUTCOME", "1" },

            { "OPENMW_RAGDOLL_CORPSE_LOAD_POSE", "1" },

            { "OPENMW_RAGDOLL_CRAWL_DOWN", "50,260,0.4,1.0,0.3" },

            { "OPENMW_RAGDOLL_CREATURE_BIPEDS", "1" },

            { "OPENMW_RAGDOLL_CROWD", "1" },

            { "OPENMW_RAGDOLL_CUT_INTENT", "1" },

            { "OPENMW_RAGDOLL_DATA_FILES", "1" },

            { "OPENMW_RAGDOLL_DEATH_CARRY_TRUE_CAP", "1" },
            // A body killed on its feet dies powered for up to 1.4 s, weak,
            // falling along the blow, then goes limp (on 11-06, user).
            { "OPENMW_RAGDOLL_DEATH_PHYSICAL", "1,1.4,6000" },

            // The dodge (Left Alt), and a dodge roll at a run (on 11-06, user).
            { "OPENMW_RAGDOLL_DODGE", "1,40,20,0.22,0.3,200" },
            { "OPENMW_RAGDOLL_DROP_ROLL", "600,80,1100,3,0.35,0.3,50,24,1.4,1,0.466,0.4,0.85,380,150" },

            { "OPENMW_RAGDOLL_DUEL_COMPOSED", "1.0,0.3,0.7" },

            { "OPENMW_RAGDOLL_EDGE_TEETER", "45,8,0.3,1.3,2.2" },

            { "OPENMW_RAGDOLL_FALLING_TIMEOUT", "2.5,60" },

            { "OPENMW_RAGDOLL_FALL_GRAB", "0.4" },

            { "OPENMW_RAGDOLL_FEET_LEASH", "18,0.08,0.6" },

            { "OPENMW_RAGDOLL_FEINT", "4,0.25" },

            { "OPENMW_RAGDOLL_FIST_ALTERNATE", "1,0.12,0.05,0.2" },
            { "OPENMW_RAGDOLL_FIST_GUARD", "1" },
            { "OPENMW_RAGDOLL_FIST_POWER", "24,6" },

            { "OPENMW_RAGDOLL_FLOOR_UNDERSIDE", "1" },

            { "OPENMW_RAGDOLL_FOOTFALL", "1" },
            { "OPENMW_RAGDOLL_FOOTFALL_ARMED_STANCE_KNEE", "0.8" },
            // CROUCH-01: sneaking crouches on the knees (on 11-06, user;
            // known: a foot sank through the floor in about 1 case in 12).
            { "OPENMW_RAGDOLL_FOOTFALL_CROUCH_LEGS", "1,40,0,1" },
            { "OPENMW_RAGDOLL_FOOTFALL_FF", "0.75" },

            { "OPENMW_RAGDOLL_FOOTFALL_FOOT_LEVEL", "1" },
            { "OPENMW_RAGDOLL_FOOTFALL_STEP_UP", "0.5" },
            { "OPENMW_RAGDOLL_FOOTFALL_STOP_LEAD", "0" },
            { "OPENMW_RAGDOLL_FOOTFALL_TREAD_SNAP", "12" },
            { "OPENMW_RAGDOLL_FOOTFALL_TURN", "0.25,30" },

            { "OPENMW_RAGDOLL_FOOTFALL_WIDTH_BLEND", "0" },

            { "OPENMW_RAGDOLL_FP_CLEAR", "12" },
            { "OPENMW_RAGDOLL_FP_DISSOLVE", "5,11,0.8" },

            { "OPENMW_RAGDOLL_GLANCE_SHARE", "0.75" },

            { "OPENMW_RAGDOLL_GRIP_HALF", "0,0.55,0.8,0.5" },
            { "OPENMW_RAGDOLL_GRIP_MURDER", "0,0.65,0.85,0.8" },

            { "OPENMW_RAGDOLL_GUARD_BLOWS", "1" },
            { "OPENMW_RAGDOLL_GUARD_COVER", "20" },
            { "OPENMW_RAGDOLL_GUARD_HELD", "1" },
            { "OPENMW_RAGDOLL_GUARD_ZONES", "1" },

            { "OPENMW_RAGDOLL_HEAD_STABILIZE", "1" },

            { "OPENMW_RAGDOLL_HILT", "2" },

            { "OPENMW_RAGDOLL_HIT_REACT", "0,25,20,1.2,250" },

            { "OPENMW_RAGDOLL_HSC", "3" },
            { "OPENMW_RAGDOLL_HSC_AIM_POINT", "1" },
            { "OPENMW_RAGDOLL_HSC_AIM_REACH", "35" },
            { "OPENMW_RAGDOLL_HSC_CATCH_UP", "40" },
            { "OPENMW_RAGDOLL_HSC_FOLLOW_TURN", "18,0.6" },
            { "OPENMW_RAGDOLL_HSC_LOW", "0.9,100,0.6,0.2" },
            { "OPENMW_RAGDOLL_HSC_LUNGE", "12,14,0" },
            { "OPENMW_RAGDOLL_HSC_NPC", "1" },
            { "OPENMW_RAGDOLL_HSC_STANCE_RADIUS", "300" },
            { "OPENMW_RAGDOLL_HSC_STEER", "1500" },
            { "OPENMW_RAGDOLL_HSC_THRUST_AIM", "1" },

            { "OPENMW_RAGDOLL_JUMP_POSE", "0.25,0.65,0.15,0.45" },

            // Off for release 0.2 (user 11-17, after kicking in the playthrough:
            // "it is OP", "can we remove it"): 2 of 3 landed kicks felled an
            // unguarded foe for 10 fatigue. X does nothing. On = the first 1.
            { "OPENMW_RAGDOLL_KICK", "0,90,45,0.2,0.15,0.45,240,14,1.3,5,1.6,6" },

            { "OPENMW_RAGDOLL_LANDING_REACH", "0.25,0.15" },

            { "OPENMW_RAGDOLL_LEASE_COMPLIANCE", "1" },

            { "OPENMW_RAGDOLL_LEDGE_LANDING", "0.15" },

            { "OPENMW_RAGDOLL_LEGS_LEAD", "1,0,0.45,10" },

            { "OPENMW_RAGDOLL_LEGS_LEAD_GRIP", "0.6,0.25,150" },

            { "OPENMW_RAGDOLL_LIMB_DAMAGE_TINT", "0" },

            { "OPENMW_RAGDOLL_MANTLE", "36,110,0.25,0.6,0.3" },

            { "OPENMW_RAGDOLL_NPC_PARITY", "1" },
            { "OPENMW_RAGDOLL_NPC_STEADY", "0.35" },
            { "OPENMW_RAGDOLL_NPC_STEADY_ROOT", "1" },

            { "OPENMW_RAGDOLL_PGC", "1,1,1,1" },
            { "OPENMW_RAGDOLL_PGC_BRACE", "1" },
            { "OPENMW_RAGDOLL_PGC_LANDING_RELATIVE", "1" },
            { "OPENMW_RAGDOLL_PGC_STUCK", "0.75,0.8,2.0" },
            { "OPENMW_RAGDOLL_PGC_STUMBLE", "20,40,70,110,20,2.5" },

            { "OPENMW_RAGDOLL_PINNED_FOOT_FRICTION", "3" },

            { "OPENMW_RAGDOLL_PINNED_FOOT_FRICTION_NPC", "2" },

            { "OPENMW_RAGDOLL_POWER_NPCS", "1" },
            { "OPENMW_RAGDOLL_POWER_PLAYER", "1" },

            { "OPENMW_RAGDOLL_REACTION_V2_BALANCE", "22" },
            { "OPENMW_RAGDOLL_REACTION_V2_BEARINGS", "0.5,0.9,1,0.6,0.55,1.3" },
            { "OPENMW_RAGDOLL_REACTION_V2_COVER_HOLD", "0.8" },
            { "OPENMW_RAGDOLL_REACTION_V2_LEG", "0.3" },
            { "OPENMW_RAGDOLL_REACTION_V2_PUSH", "1.5,850" },
            { "OPENMW_RAGDOLL_REACTION_V2_VIS", "1.6" },

            { "OPENMW_RAGDOLL_RMB_GUARD", "1" },

            { "OPENMW_RAGDOLL_ROOT_WALK_SUPPORT", "0.6" },

            { "OPENMW_RAGDOLL_RUN_BOB", "0.3,5,250" },

            { "OPENMW_RAGDOLL_SCOPED_RECOVERY", "1" },

            { "OPENMW_RAGDOLL_SLOPE_STANCE", "22,40,14,0.25,0.5,1.8" },

            { "OPENMW_RAGDOLL_SMART_LANDING", "650,0.06,22,0.5,25" },

            { "OPENMW_RAGDOLL_SOFT_ROOT", "1" },
            { "OPENMW_RAGDOLL_SOFT_ROOT_INERTIA", "0" },

            { "OPENMW_RAGDOLL_STAGGER_GRIP", "1" },
            { "OPENMW_RAGDOLL_STAGGER_PLAN", "1,25,70,110,160,0.22,0.26,0.7,0.3,1.1,0.35,45" },
            { "OPENMW_RAGDOLL_STAGGER_SETTLE", "2" },

            { "OPENMW_RAGDOLL_STAGGER_UPRIGHT_HOLD", "0.94,30" },

            { "OPENMW_RAGDOLL_STEP01", "1,1,1.4,0.9,1,12,0.9,55,6,1,0.7,15,0" },

            { "OPENMW_RAGDOLL_SUPPORT_REACH", "0.6,40,35,10" },

            { "OPENMW_RAGDOLL_SWING_VOICE", "1.8,0.7,4" },

            { "OPENMW_RAGDOLL_WEAPON_KNOCK", "0.4,0.15" },
            { "OPENMW_RAGDOLL_WEAPON_TORQUE", "0.6,0.25" },

            { "OPENMW_RAGDOLL_WOUND_FX", "4" },
        };
    }

    std::span<const PlayDefault> playDefaults()
    {
        return sPlayDefaults;
    }
}

#include <ragdollcore/switchregistry.hpp>

#include <ragdollcore/playdefaults.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>

namespace RagdollCore
{
    namespace
    {
        // Sorted by name. Add a switch here when new code first reads it.
        constexpr SwitchSpec sSwitches[] = {
            { "OPENMW_RAGDOLL_K2_AIM_PREVIEW", SwitchKind::Experiment, SwitchArea::Weapon, "16", 0.0f, 40.0f,
                "foe radius: the cut's path on the reticle marks where the blade first meets the foe's body of that "
                "radius, or its closest pass (user 11-04: \"can't tell where it'll land\"), where K2_WEAPON runs the "
                "wheel; 0 = no mark. The wind-up shows no path (user 11-08)",
                "phase 1: the user's play decides" },
            { "OPENMW_RAGDOLL_K2_AI_ARRIVE", SwitchKind::Experiment, SwitchArea::Ai, "900,0", 0.0f, 5000.0f,
                "deceleration u/s^2, strike hold share: a duelling NPC brakes to stop at its mark instead of running through "
                "it, holds its gap at that share of its strike reach while its own attack runs, and never presses in "
                "inside half its reach - strike hold 0 = its attack presses in as before (the hold cost hits: NPCHIT-01 at 0.85 4/7, at 0.5 0-13 a run, off 8-13; user 11-06: \"the enemy npcs arent attacking\"); user bookmark 11-05: \"the enemy torpedos into the player as a charge ... if I "
                "dont move they dont hit me\"; the level-50 guard ran in at ~680 u/s and cut at a gap of 0-2 u); 0 = off",
                "the user's play decides (release 0.2)" },
            // Duel aggression (user 11-07 after the level-matched Skaal duel - 23
            // attacks in 105 s, 5 landed, 4 of 23 chained: "make them chain
            // attacks, step out less and also if the player is charging at them
            // make them be able to attack while walking back, or try to attack
            // when staggered"). DuelPressSettings, dueltactics.hpp.
            { "OPENMW_RAGDOLL_K2_AI_BACK_CUT", SwitchKind::Experiment, SwitchArea::Ai, "0.3,0.9", 0.0f, 1.0f,
                "chance at attack skill 0, at 100: a duelling NPC backing away from a charge (its backpedal, or its "
                "step-out with the foe closing faster than 180 u/s) strikes as the foe comes into reach, leading it by "
                "its strike's time, and keeps walking back through the attack - before, only a fighter with nerve over "
                "0.6 cut a charge off, standing; decided once per charge or step-out; 0,0 = off",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_AI_BEARINGS", SwitchKind::Experiment, SwitchArea::Ai, "0.35", 0.0f, 1.0f,
                "the most of its movement a duelling NPC uses while it gets its bearings after a blow, no run and "
                "no dodge burst (user bookmark 11-09: \"sling away ... then they go back and sling back\"; 0.5 s "
                "after the blow, 1.1 s before the bearings ended, the AI ran its capsule at the foe, and the body "
                "went 0 to 426 u/s on a curve); 0 = off",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_AI_PRESS", SwitchKind::Experiment, SwitchArea::Ai, "0.2,0.6,3", 0.0f, 6.0f,
                "chance at attack skill 0, at 100, attacks in a row: a duelling NPC whose attack is over with the foe "
                "still in reach (+30 u) attacks again at once instead of stepping out, up to that many attacks (fatigue "
                "0.4 and up); the vanilla attack delay is skipped; attacks 1 = off (one attack, then the step-out)",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_AI_SPACE", SwitchKind::Experiment, SwitchArea::Ai, "110,0.6,1.5,40", 0.0f, 1000.0f,
                "chase speed u/s, chase time s, patience x, margin u: how much room a duelling NPC gives - it "
                "chases only an opponent whose gap opens faster than the speed for the time (the tactics' 45 u/s "
                "for 0.25 s: any step back was run down), waits `patience` times as long at its measure before "
                "stepping in unprovoked (openings, cut-offs and ripostes are not slowed), and holds its measure "
                "`margin` beyond the longer reach (14). User 11-12: \"make it so the enemies walk less towards "
                "the player otherwise all fights would be holding back\"; that evening's session (1,029 phase "
                "changes): 39% of the NPCs' time stepping in, 25% chasing (92 chases), 16% at their measure. "
                "A scripted 60 s spear fight against a Hlaalu guard (the player stands, steps back, thrusts; 1 run each): as before (0, K2_AI_PRESS 0.2,0.7,3, K2_AI_STEP_OUT 0.3) the guard spent 66-76% of its time stepping in, 14-17% at its measure, the gap's median 73-75 u; with 110,0.6,2.5,40, press 0.1,0.3,2 and step-out 1.0 (the game's since 11-12): 49%, 24%, 98 u. The space values alone, or with the step-out only: 62-70%. Retuned 11-13 (user: \"the enemies should attack a bit more ... at least try to swing\"; with 2.5 and press 0.1,0.3,2, step-out 1.0 the NPCs' attacks in play fell from 22 to 15 a minute and their hits on the player from 6.7 to 2.4-4.3): 110,0.6,1.5,40 with press 0.2,0.6,3 and step-out 0.8 - the same scripted fight, 2 runs: 61-68% stepping in, the gap's median 78-82 u (p75 127-142; as before 92). Each value 0 = as before",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_AI_STAGGER_CUT", SwitchKind::Experiment, SwitchArea::Ai, "0.1,0.6,0.15", 0.0f, 2.0f,
                "chance at attack skill 0, at 100, delay s: a staggered duelling NPC with the foe in reach (+20 u) swings "
                "back after the delay - the reaction's hold on its weapon arm (no new attack while a stagger has it) "
                "lets go for that attack; decided once per stagger; 0,0 = off",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_AI_STEP_OUT", SwitchKind::Experiment, SwitchArea::Ai, "0.8", 0.0f, 3.0f,
                "seconds a duelling NPC backs off after its last strike (twice that at most, until it is back at its "
                "hold distance); 0 = the tactics' own 0.6",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_ARMED_HANDS", SwitchKind::Experiment, SwitchArea::Reaction, "1", 0.0f, 1.0f,
                "1 = a fighter with a weapon in hand strikes with the weapon only, never with the hand round its "
                "hilt: an AI or player strike made its weapon hand a fist striker even armed, so a sword hand "
                "brushing the other fighter dealt hand-to-hand blows on fatigue (11-07 play: a Hlaalu guard's "
                "\"fists\" took 19.6 fatigue from the exhausted player and knocked them out; user: \"I fall for "
                "some reason\"); 0 = the hand strikes too",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_ARM_STEP", SwitchKind::Experiment, SwitchArea::Legs, "1,0.5", 0.0f, 2.0f,
                "on, gain: the movement clip's loop takes as long as the legs' cycle and is pulled onto their phase "
                "(user 11-08: do the arms keep pace with the legs; session 025541, running: the clip's arm swing "
                "0.40 s, the legs' stride 0.82 s, the clip playing at speed / 222); on 0 = the clip's own rate",
                "the user's play" },
            { "OPENMW_RAGDOLL_K2_ARM_STRAIN", SwitchKind::Experiment, SwitchArea::Weapon, "0.5,1.5,0.2,0.6,0", 0.0f,
                10.0f,
                "per clash, hold s, recover/s, weaken, you: every clash a fighter parries strains its weapon arm by "
                "per clash x the clash's ratio (0.3-1.8), held that long, then eased off; the strained arm is weaker "
                "by weaken x strain (its drive, its guard in the clash table, its return from a knock) and at full "
                "strain it is spent - the guard drops and stays down until the strain is back to half (user 11-07: "
                "\"can we make it so they lose strength in the arm? a couple of clashes should open up their "
                "guard\"); you = the player's share (0 = NPCs only); per clash 0 = off (RagdollCore::ArmStrain)",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_BALANCE_RISK", SwitchKind::Experiment, SwitchArea::Hud, "1", 0.0f, 1.0f,
                "1 = the reticle's four wedges warn together when the body is about to go over: amber off balance, red "
                "and pulsing when going (ragdollcore balanceRisk: the capture point past the feet, Stumbling, Falling; "
                "user 11-08: the indicator says nothing about falling); 0 = as before",
                "the user's play" },
            { "OPENMW_RAGDOLL_K2_BALANCE_SIGIL", SwitchKind::Experiment, SwitchArea::Hud, "0", 0.0f, 1.0f,
                "1 = the balance sigil at the bottom of the screen (user 11-12): a bronze triangle of the three senses "
                "(eye, labyrinth, boot) around the stance seen from above - a moon per foot filled by the weight it "
                "carries, a sun at the centre of mass, a spear to the capture point's star, red once it leaves the "
                "feet (ragdollcore balanceSigil). 0 = no sigil: off since 11-17 after five days in the hub (user: "
                "\"we need to remove the balance graphic triangle from onscreen\"); its options left the KINETIC "
                "page with it. Unit-tested; drawn in the HUD-02 case's screenshots (640x480, one run)",
                "off by the user's play (11-17); kept, not deleted (user: \"Keep it off\")" },
            { "OPENMW_RAGDOLL_K2_BLOCK_NO_KNOCKOUT", SwitchKind::Experiment, SwitchArea::Reaction, "1", 0.0f, 1.0f,
                "1 = a block's fatigue cost (BLOCK_FATIGUE) stops at 0: a parry can exhaust its blocker but never "
                "knocks it out - every weapon parry is a block in KINETIC, and the cost went below zero (vanilla's "
                "shield rule), a knockout (user 11-07: \"I've fallen from getting parried from a dagger\"); 0 = "
                "below zero as vanilla",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_BLOW_PRESS", SwitchKind::Experiment, SwitchArea::Weapon, "1,0.15,8", 0.0f, 75.0f,
                "on, body share, mass cap kg: a blade that bites into flesh, or is stopped by it, presses on - no "
                "hit-stop, no throw-back; its contact drives the struck body away while the stroke carries "
                "through (plate struck blunt or flat still rebounds, an edge stuck in armour still sticks); and "
                "a blow carries `body share` of the attacker's mass by how much of its speed is the body moving "
                "along it (a jumping chop, a step into the cut), at most `cap` kg (user 11-05: \"the sword "
                "should push them away, but ... shouldnt bounce\"); 0 = the old hit-stop and weapon mass",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_BLOW_TRACE", SwitchKind::Debug, SwitchArea::Reaction, "", 0.0f, 1.0f,
                "set: for 3 s after a blow, the struck actor's capsule, pelvis, root target and asked speed "
                "(RAGDOLL_BLOW_TRACK); never changes behaviour",
                "" },
            { "OPENMW_RAGDOLL_K2_CALM", SwitchKind::Experiment, SwitchArea::Root, "0.9,0.2,4,1,14,0.15", 0.0f, 60.0f,
                "strength, ease down s, quiet s, ease up s, bump u, bump s: a calm NPC's body is held to the game's animation - "
                "the root at `strength` with its whole sideways hold and weight support, the legs on the clip instead "
                "of the stepping controller and its balance catches - and let go onto its own legs over `ease down` when "
                "it is bothered: in a fight, weapon or spell out, hit, shoved, staggering, falling, getting up, in the "
                "air, or its pelvis pushed `bump` u (plus a tenth of a second of its own travel) off the animation's for `bump s` "
                "(11-16 play: 14 one-step bumps on town walkers in 15 min, each only dipping the hold to 0.92; a bump "
                "now lasts and lets go all the way). Unbothered, upright and within `bump` of the animation for `quiet` s the hold "
                "returns over `ease up` (the root first, the legs from half way). A blow lets go at once. A body "
                "powered unbothered starts held. User 11-14: \"keeping it kinetic body with like 90% strength ... "
                "blended\"; in the 11 playthrough sessions of 11-05 to 11-13, 208 of 305 NPC body faults (stuck, legs "
                "failed, get-ups) were NPCs nobody fought or touched; strength 0 = every NPC body kinetic all the time",
                "the user's play of a town street, a hillside and a fight's end" },
            { "OPENMW_RAGDOLL_K2_CALM_DRAW", SwitchKind::Experiment, SwitchArea::Root, "1", 0.0f, 1.0f,
                "1 = a held NPC (K2_CALM) is drawn as the game's animation itself, blended to its physical body by the "
                "hold's level as it is let go (a blow lets the body go in one step; the picture follows over the ease "
                "down time). The physical body stays under it, held, for blows, shoves and the let-go. User 11-16 on "
                "play-2026-11-15-1, which drew the held physical body: \"weirder an goofier and not vanilla like at "
                "all\"; 0 = a held body is drawn from its physical body",
                "the user's play of a town street: calm NPCs look as in the original game, no pop at a let-go" },
            { "OPENMW_RAGDOLL_K2_CAMERA_STEADY", SwitchKind::Experiment, SwitchArea::Root, "1", 0.0f, 1.0f,
                "1 = the camera of a body on its feet takes the actor's share of the balance target from where the "
                "actor is drawn this frame, not from the position the last physics step read: under 60 fps a frame "
                "holds two steps now and then and the stepped position moves 0 or two steps of travel where the "
                "body moves one, which shook the whole body 3.8 u along its run against the camera (user 11-12, "
                "11-13: \"the model vibrates back an forth\"; every 6 steps at 54 fps, every 3 at 47); 0 = the "
                "stepped target",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_CAPSULE_LEASH", SwitchKind::Experiment, SwitchArea::Root, "45", 0.0f, 500.0f,
                "units: the capsule more than this far ahead of the physical pelvis is drawn back to it, so it waits for "
                "the body (user bookmark 11-05: \"slingshotting the model with the capsule at high speed\"; the capsule "
                "ran 547 u/s with the pelvis 100-110 u behind, then the body snapped up at ~770 u/s); 0 = off",
                "the user's play decides (release 0.2)" },
            { "OPENMW_RAGDOLL_K2_CLASH_OPEN", SwitchKind::Experiment, SwitchArea::Weapon, "25,0.6", 0.0f, 120.0f,
                "units, seconds: an unparried clash throws the struck side's hand that far up and away and keeps "
                "its block down that long (play-6 bookmark), where K2_WEAPON runs the wheel; 0 = off",
                "phase 1: the user's play decides" },
            { "OPENMW_RAGDOLL_K2_CORPSE_FRAME", SwitchKind::Experiment, SwitchArea::Lifecycle, "1", 0.0f, 1.0f,
                "a limp body is built and drawn in a frame that turns with the visible actor, as a powered body's "
                "is (user 11-17, the hand-off: \"it almost looks like it rotates 180 upon change\"; the corpse's "
                "frame took the capsule's rotation, identity for every humanoid, so bodies carried over at their "
                "true places were drawn turned by the actor's facing about the actor, half round for one facing "
                "south, and a corpse built from the skeleton stood in the physics turned the other way); "
                "0 = the capsule's rotation. RAGDOLL_DEATH_DRAWN body_off is the farthest a bone was drawn from "
                "its body",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_CUT_HIP_CAP", SwitchKind::Experiment, SwitchArea::Weapon, "0.6", 0.0f, 0.6f,
                "radians: in a cut and its follow-through the hips may turn this far over the planted legs "
                "(K2_HIP_TURN's share; 0.45 otherwise), eased at 1.5 rad/s, so a slash's follow-through turns "
                "the chest further past its end (user 11-05: \"bigger follow-through\"; it stopped ~57 deg "
                "against the wind-up's ~79), where K2_WEAPON runs the wheel; 0 = 0.45 throughout",
                "phase 1: the user's play decides" },
            { "OPENMW_RAGDOLL_K2_CUT_LEAD", SwitchKind::Experiment, SwitchArea::Weapon, "0.1", 0.0f, 0.3f,
                "seconds: in a cut the chest's turn is taken this far ahead on the cut's route, the hips leading "
                "it as before (user 11-05: \"rotate the chest when the chop/slash is being executed\"; the "
                "spine's motors trailed the turn 0.1-0.2 s, so it landed after the blade), where K2_WEAPON runs "
                "the wheel; 0 = the turn where the blade is",
                "phase 1: the user's play decides" },
            { "OPENMW_RAGDOLL_K2_CUT_REACH", SwitchKind::Experiment, SwitchArea::Weapon, "0.8,1000", 0.0f, 2000.0f,
                "share, units/s: a one-handed cut closes this share of the gap between the grip's distance "
                "from the shoulder and 0.95 of the arm's reach, most from a third of the cut to three "
                "quarters, and its elbow may open at this speed (ARM_ELBOW_SPEED's 250 otherwise) "
                "(bookmark 11-05: \"a left to right hard swing should extend the arm more with 1 handed "
                "sword\"; the sparksword's backhand struck with the elbow at 78 deg), not thrusts, fists, "
                "daggers or two-handed grips, where K2_WEAPON runs the wheel; 0 = the keys' reach and speed",
                "phase 1: the user's play decides" },
            { "OPENMW_RAGDOLL_K2_DAGGER_THRUST", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 1.0f,
                "the player's dagger thrust winds up in 0.04 s, strikes in 0.07, returns in 0.16 (/ the weapon's response) "
                "and a press chains from any stage at any skill (user bookmark 11-05: \"almost not even cap how fast it "
                "can be done\"), where K2_WEAPON runs the wheel; 0 = the short blade's pace",
                "the user's play decides (release 0.2)" },
            { "OPENMW_RAGDOLL_K2_DEATH_DRAW", SwitchKind::Experiment, SwitchArea::Lifecycle, "1", 0.0f, 1.0f,
                "on the frame a dying body goes limp its corpse is drawn in the pose the living body was drawn in "
                "(user 11-17: the ragdolls \"are not doing a continuous motion\" at the hand-off, \"it almost looks "
                "like it rotates 180\"; the swap runs after the frame's poses are written, the living body's "
                "binding put the death clip's pose back on the bones as it went and the corpse's wrote nothing "
                "until the next frame, so that one frame drew the animation); 0 = that frame from the animation. "
                "RAGDOLL_DEATH_SWAP logs how far the drawn pelvis moved and turned over the swap",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_DEATH_JOINTS", SwitchKind::Experiment, SwitchArea::Lifecycle, "1", 0.0f, 1.0f,
                "a corpse carried over from a living body takes that body's joint frames (its build skeleton, "
                "standing frames, anatomical elbows and knees) and is drawn from the physics at once (user 11-06: "
                "the angles snap at death; the corpse's joints came from the animation pose, 20-97 u off the body, "
                "and the solver yanked them into range, and the first 0.08 s blended from that pose); 0 = the "
                "animation pose's frames and the blend",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_DEATH_LIMITS", SwitchKind::Experiment, SwitchArea::Lifecycle, "0.5", 0.0f, 3.0f,
                "seconds: a joint the corpse takes over outside its own range (the living body's were opened to "
                "fight: a weapon hand's wrist to 2.0 rad against the corpse's 0.45/0.35, the shoulders for a "
                "swing) starts in a range opened to hold it, which closes to the authored one over that long; "
                "0 = the authored range from the first step, where the solver pulls the joint into it at once. "
                "RAGDOLL_DEATH_LIMITS logs how many joints were outside and the worst",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_DEATH_MOMENTUM", SwitchKind::Experiment, SwitchArea::Lifecycle, "1,0,0", 0.0f,
                1000.0f,
                "share, cap u/s, drag rate: the corpse keeps that share of each part's own motion and spin, the "
                "body's mean capped at `cap` (0 = none), the horizontal mean drained at `drag` per second (user "
                "11-06: real momentum at death; deaths moved at 52-374 u/s and all hit DEATH_CARRY_VELOCITY's "
                "15 u/s cap, then DEATH_DRAG's 3/s); share 0 = off, the old cap and drag",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_DESCENT", SwitchKind::Experiment, SwitchArea::Legs, "30,0.8,4", 0.0f, 60.0f,
                "most u, share, slack u: going down, the pelvis comes down by that share of how far the ground under "
                "the foot ahead (planted, or its landing) lies below the capsule's ground, past the slack (user "
                "11-14: \"their chest go ahead but the legs lag ... especially in slopes ... npcs running down "
                "slopes\"; PATH-09, a fighter charging down the Balmora hillside: the pelvis rode at the capsule's "
                "height with the next foothold 40-50 u lower, out of the leg's reach - no foot loaded on 85-95% of "
                "steps, the root carrying 0.8-0.9 of the weight, the feet trailing the pelvis 15-50 u); most 0 = off",
                "PATH-09, the slope and stairs cases and the user's play decide" },
            { "OPENMW_RAGDOLL_K2_DOWN_STILL", SwitchKind::Experiment, SwitchArea::Reaction, "1", 0.0f, 1.0f,
                "a powered NPC whose body is falling, lying or getting up takes no combat action (attack, spell, "
                "move) as a knocked-down one does: powered bodies clear Morrowind's knockdown, so a downed NPC "
                "could cast (bookmark 11-05); 0 = it acts",
                "phase 3: kept as the reaction path's rule, or removed" },
            { "OPENMW_RAGDOLL_K2_DRIVE_RESPONSE", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 3.0f,
                "cap: a one-handed weapon's hand drive takes the weapon's response (K2_WEAPON_PACE) up to this, "
                "while the wheel's timing keeps all of it (two-handers keep it all: their second hand steadies "
                "the grip). At the player's 1.5 a light one-hander's grip spring ran at 18 Hz (6 x 1.5 / 0.5), at "
                "the 60 Hz step's edge, and rang where its centre sat far out: VARIANT-01 11-07, the war axes "
                "glass/silver/dwarven 150-420 jolts and the steel/daedric daggers ~450 against 22-49 with 1 (and "
                "K2_HAND_EFFECTIVE_MASS), with the same cut speeds; at 1 for two-handers too the claymore's cuts "
                "fell to 22 deg/step from 34; 0 = no cap",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_EDGE_BITE", SwitchKind::Experiment, SwitchArea::Weapon, "0.6,0.1", 0.0f, 1.0f,
                "edge lead, armour stick seconds: a blade's or axe head's contact is a cut when the motion runs "
                "along the edge at least that share of the speed (the hull's normal said the flat: 25 of 30 axe "
                "hits on flesh were stops, 11-05 play), and an edge stopped by armour sticks that long instead of "
                "being thrown back (bookmark 11-05: \"a chopping sharp edge ... should very briefly get stuck\"); "
                "0,0 = off",
                "phase 1: the user's play decides" },
            { "OPENMW_RAGDOLL_K2_FIGHT_NIGHT", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 1.0f,
                "1 = FIST-05, Fight Night's punch control (user 11-07: \"something like fight night fistfighting\"): "
                "with bare fists the player's press is a flick - its release throws the punch its direction picks, "
                "either hand (up-left the jab, up-right the cross, sideways the hook from the other side, down-left "
                "and down-right the uppercuts, down the body shot; crouched, straights go to the body), its length "
                "the power (a haymaker winds up 2.5x, strikes 25% quicker, costs 3x the fatigue); a press without a "
                "drag throws the next straight of the jab-cross alternation. The left fist runs the right's lines "
                "mirrored (fistcontrol). 0 = K2_FISTS' steered drags",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_FISTS", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 1.0f,
                "bare fists: a held drag to the left throws the left fist and the right stays on guard, and the left "
                "punch goes where the crosshair points, stopping just through it (user 11-05: \"I can only move 1 hand and "
                "its impossible to aim and it bounces everywhere\"); 0 = the left only jabs on alternate clicks, at the "
                "foe's head",
                "the user's play decides (release 0.2)" },
            { "OPENMW_RAGDOLL_K2_FIST_AIM", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 1.0f,
                "on: player fist motor targets converge on the live camera ray during the strike, from the "
                "current shoulder even while moving; retains arm reach and contact-only damage. Both hands and "
                "all punch kinds; eases out in recovery, never steers a raised palm. 0 = previous aiming",
                "FistControlTest ray/reach checks; moving FIST proving case and player verdict required" },
            { "OPENMW_RAGDOLL_K2_FIST_CHARGE", SwitchKind::Experiment, SwitchArea::Weapon, "16,0.7,1", 0.0f, 40.0f,
                "units, radians, on: a held button's charge shows and counts (user 11-16 on play-2026-11-16-10: \"I "
                "wasnt able to charge the attacks in hand to hand\" - their session threw 7 charged punches of 59; a "
                "held straight sat 5 u from its guard and left no faster than a tap). The charged fist draws the "
                "first value further back along its punch's wind-up at the full charge and the trunk coils the "
                "second (the punching shoulder back); with the third 1 the charge is Morrowind's attack strength "
                "for the blow - the hit's quality is at least the share of the charge reached (DMG-04 reads the "
                "strength from the fist's speed at contact alone, and a charged fist is no faster). 0,0,0 = off",
                "FIST-09's charge lines (the draw, the shoulder, the fatigue a charged blow costs the dummy against a "
                "tap's); the user's play decides" },
            { "OPENMW_RAGDOLL_K2_FIST_DRIVE", SwitchKind::Experiment, SwitchArea::Weapon, "0,24", 0.0f, 60.0f,
                "Hz, g: a punching fist's drive - the spring that carries the hand to its commanded place, and "
                "its acceleration cap - for as long as the punch lasts (the right fist's wind-up and strike on the "
                "wheel, the left's whole punch, which has no drive and goes on the free arm's motors). The hands' "
                "drive is 4 Hz and 12 g (HSC_DRIVE_HZ, WEAPON_DRIVE_ACCEL). Built on the guess that the fists "
                "trailed their command (landed fists at a median 198 u/s in the user's 11-16 session, a real punch "
                "525-650): not shown. FIST-07, one run each, the hands' top speed in a tap at 10,24 against off: "
                "left straight 380 / 461 u/s, right straight 519 / 527, right at a strafe 553 / 703, a charged "
                "right 614 / 565. The fists already peak at a real punch's speed; what lands slowly is where on "
                "its way the fist meets the body. 0 = the hands' drive (off)",
                "not kept as a default; remove unless a contact-speed measure shows a gain" },
            { "OPENMW_RAGDOLL_K2_FIST_EASE", SwitchKind::Experiment, SwitchArea::Weapon, "0", 0.0f, 1.0f,
                "seconds: the left fist goes from where its punch left it to where it is held over this long, as "
                "the right fist's hold eases in on the wheel (0.12 s). The user's 11-17 play (\"after doing the "
                "attack the arms swing and fly around\"): with the button still down the left fist's command "
                "stepped 18-19 u in one tick at the end of every wound left hook, uppercut and backfist (the "
                "punch returns to its line's guard, the held place lies about the left shoulder), 4-5 u for the "
                "right. 0 = at once",
                "FistControlTest (the ease); FIST-12 (the left swings' path after the strike and settle time); the "
                "user's play decides. Off (0) from 11-17: built after the hub's 15:10 engine, never played; "
                "0.12 is the value to try" },
            { "OPENMW_RAGDOLL_K2_FIST_HUD", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 1.0f,
                "on: separate left/right held charge and selected punch cues; fists use neither the weapon's "
                "mouse trace nor its stale swing preview. 0 = previous weapon HUD",
                "FistSteerTest cue checks; HUD play check required" },
            { "OPENMW_RAGDOLL_K2_FIST_STANCE", SwitchKind::Experiment, SwitchArea::Weapon, "-40,1,1", -90.0f, 90.0f,
                "degrees, share, share: the player's fists keep a fighting stance on the move (user 11-16: \"when "
                "moving, the whole body moves right and sometimes the chest grabs an angle or the hips fall a bit, "
                "can we actually make a firmer stance? maybe even keep some sort of southpaw or orthodox\"). The "
                "shoulders' line stood 41 degrees left-shoulder-forward in the standing clip and 8 +-15 in the "
                "moving ones (their 11-17 session), so the stance was lost with the first step. With the fists up "
                "the chest's target is turned so the shoulders' line stays the first value from square to the view "
                "(negative = the left shoulder forward, orthodox; the guard's hand places are the left lead's) by "
                "the second value's share (the body's own line trims what is left, 3/s), and the clip's side tilt of "
                "that line is taken out by the third's. "
                "Punches turn the chest from there. 0,0,0 = off",
                "FIST-10 (the shoulders' line still, running and strafing, its spread); the user's play decides" },
            { "OPENMW_RAGDOLL_K2_FIST_STEER", SwitchKind::Experiment, SwitchArea::Weapon, "4,300,100,0", 0.0f, 2000.0f,
                "mode, counts, swing counts, release: the bare fists' control. The left button is the right fist, the right button the "
                "left; both buttons the guard, a tap of both the shove. "
                "4 = FIST-12, the weapons' charge (user 11-16 on mode 3 and FIST-11's bars: \"make it so fists can "
                "work with clicks but make the charging the same as the other weapons, but each direction is a "
                "different hit style\"): a click is mode 3's tap; a held button gives the mouse to that fist and "
                "winds it up the way the mouse goes - the punch whose wind-up lies that way, `counts` of travel to "
                "the full charge, shown on the weapons' wind-up reticle - and letting go throws it as hard as it "
                "was drawn. Time alone charges nothing. The swing (user 11-16: \"charge the fist and then swing it "
                "hard to the other direction\"): a wound fist is also thrown with the button still down by a stroke "
                "back against its draw, `swing counts` of travel within 0.1 s (the weapons' cut: 150 counts at 2,500 "
                "counts/s; a wound fist's draw may be 90); the hand winds up again once that punch is done, and "
                "slower travel back only lessens the draw, and it winds again only with the mouse at rest (the stroke's "
                "follow-through wound the opposite punch: the user's 11-17 strokes ran 1,300-2,700 counts). Release 0 "
                "(user 11-17: \"fist to fist works like any other weapon ... windup and move the mouse quickly to the "
                "other side to deliver the punch\"): letting go of a wound fist throws nothing, as a weapon's "
                "wind-up; 1 = letting go throws it too. Swing counts 0 = letting go only. "
                "3 = FIST-09, the crosshair picks the punch (user 11-16 on mode 2: \"its still really hard to aim and "
                "the movement keys choosing the attack.. I'm not so sure about it I think we need more control\"): "
                "taps and charged holds as mode 2, the mouse the view's, the punch by where the crosshair is on the "
                "foe - a tap the straight (low: the body shot), a charged punch the hook at the head, the straight "
                "at the chest, the uppercut low; and K2_FIST_STEP's step in. "
                "2 = FIST-07, the feet pick the punch (user 11-16 on mode 1: \"better than before but still pretty "
                "bad\", \"I kinda want to not use the weapon swinging style\"; research/FISTS-02): the mouse "
                "stays the view's. A tap throws the punch the movement keys pick - none the straight at the "
                "crosshair, a strafe that hand's hook, back the uppercut, forward the long straight - and a held "
                "button winds the same punch up over 0.6 s (the fist shown going out along its wind-up) for letting "
                "go to throw it harder (the full wind-up strikes 25% quicker, costs 3x the fatigue). "
                "1 = FIST-06, the steered fists (user 11-12: \"when I click and hold with fists nothing happens\"): "
                "held, the fist follows the mouse out along the wind-up that lies that way, `counts` of travel to "
                "the full wind-up, and the view is held; let go, it is thrown from there. "
                "0 = FIST-05's flicks (K2_FIGHT_NIGHT)",
                "the user's play decides; FistSteerTest (mode 4: no game case yet), FIST-06 (mode 1), FIST-07 (mode 2)" },
            { "OPENMW_RAGDOLL_K2_FIST_STEP", SwitchKind::Experiment, SwitchArea::Weapon, "40,42", 0.0f, 100.0f,
                "units, units: with K2_FIST_STEER 3 a punch thrown at a foe out of reach lunges at it - through the "
                "wind-up and the strike the hips go forward by the gap between the chests' distance and the second "
                "value, at most the first (either hand; the wheel's own step-in is 14 u and the right hand's only), "
                "and come back in the recovery; for a foe up to 20 u beyond that and within 35 degrees of the "
                "crosshair. And the foe assist (the punch's line turned onto a foe within 20-35 degrees of the "
                "crosshair) takes a foe that far instead of 40 u. The user's 11-16 fist sessions: punches were thrown "
                "with the foe a median 70-76 u off (a fist reaches about 40), 11-14% landed. 0 = off",
                "FIST-09's landing lines (taps at 50, 65 and 80 u, the step on and off); the user's play decides" },
            { "OPENMW_RAGDOLL_K2_FIST_TRUE", SwitchKind::Experiment, SwitchArea::Weapon, "0.96,0.06,0.25,0.75", 0.0f, 1.0f,
                "reach, hold s, return s, strike time x: the player's fists run true (user 11-16 on FIST-07: \"the fists feel wobbly and "
                "they are not straight/weighted at all\"; that session's right straights ran 2.7-3.8x their straight "
                "line, 23-28 u off it, 44-59 u behind their command, and hung 0.35 s at full stretch swinging +-7 u). "
                "Five things: a fist's commanded place stays within `reach` of the arm's length from its shoulder (the "
                "lines end past the arm's reach, FIST-01: the locked arm was pulled on at the drive's cap and swung on "
                "its end); the fist's drive carries the upper arm and forearm with the hand (it pushed the hand alone "
                "and the arm dragged behind); the left fist is driven as the right is, in its guard and its punches "
                "(it went on the arm's motors); the left straight goes from where the fist is straight at the crosshair "
                "and straight back (its mirrored line drew back 20 u to the shoulder and 13 u up, then out round a "
                "loop); and the right fist's strike takes `strike time` of the short weapons' (its route ends at the "
                "reach now), holds `hold` s at its end and returns in `return` s (0.35 and 0.5). 0 = off",
                "FIST-07's path lines (chord, path/chord, off the line, ring, behind its command); the user's play decides" },
            { "OPENMW_RAGDOLL_K2_FIST_WOUND", SwitchKind::Experiment, SwitchArea::Weapon, "0,0,0", 0.0f, 20.0f,
                "strike s, wind share, uppercut raise u: a hook or an uppercut thrown from a held fist (FIST-13; the "
                "user's 11-17 play: their strikes were commanded at 750-900 u/s over 0.10-0.12 s, the hand peaked at "
                "200-460 u/s and covered 40-70% of the strike, the right uppercut 9.5-12 u of 24-27). The strike takes "
                "at least `strike s` (never quicker than it was), the wind-up to the chamber `wind share` of a quick "
                "punch's (it was half: 0.07 s), and the uppercut's wind-up place is `raise` u above its -4 (read once, "
                "when the fist lines are made; nothing measured asks for it). Straights and body shots keep their "
                "times. Each value 0 = as it was",
                "FIST-12's swing lines (ahead, from_rest, path_out, settle) for the own-side and down swings, three "
                "runs each way; the user's play decides. Off (0,0,0) from 11-17: never run in the game; 0.18,1,0 "
                "is the value to try" },
            { "OPENMW_RAGDOLL_K2_FLICK_CHAIN", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 1.0f,
                "the button held, a finished cut or its return takes a drag away from the blade as the next wind-up at "
                "once, from where the blade is, by the press's skill rule (user bookmark 11-05: \"chop chop chop hack hack "
                "slash slash\"), where K2_WEAPON runs the wheel; 0 = the held drag waits for the guard",
                "the user's play decides (release 0.2)" },
            { "OPENMW_RAGDOLL_K2_FOREARM_ROLL", SwitchKind::Experiment, SwitchArea::Weapon, "1.3", 0.0f, 1.6f,
                "radians: while hand control holds a weapon both wrists roll this far each way (the forearm's turn; "
                "the rig's 0.3 is a corpse's), on HSC's wrist torque (user 11-06, spear and staff: \"a forearm roll "
                "joint\"; the elbow is a hinge, so a turned haft twisted the upper arm and the forearms crossed in "
                "9.5% of staff frames); 0 = the rig's",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_GAIT_CAP", SwitchKind::Experiment, SwitchArea::Root, "0,0", 0.0f, 5000.0f,
                "speed u/s, player: a physical body moves (its capsule and its movement clip) no faster than "
                "`speed` on foot (user 11-06, fast charges: the level-50 guard's capsule ran ~680 u/s and the "
                "root towed the body by the hips). PLAY-01 sprint (Speed and Athletics 100, 542 u/s) x3 each, "
                "off / 400 / 350 / 290: the legs carry 0-0.13 / 0-0.31 / 0.15-0.35 / 0.45-0.51 of the weight "
                "running, airborne 0.28-0.42 / 0.28-0.42 / 0.28-0.37 / 0.17-0.22, step length spread 35-55 / "
                "15-16 / 6-13 / 7-19%; player 1 caps the player too, 0 NPCs only; 0 = off. Off from "
                "play-2026-11-16-4 (user 11-16: \"can we uncap the npcs\"; it was 290,0 from 11-06)",
                "phase 2: until Stride's sprint gait carries Morrowind's speeds" },
            { "OPENMW_RAGDOLL_K2_GROUND_FOLLOW", SwitchKind::Experiment, SwitchArea::Legs, "60,10,0.4", 0.0f, 100.0f,
                "most u, slack u, descent s: the legs' ground lookups reach further down by how far the physical "
                "pelvis hangs under its target past the slack, plus how far the capsule comes down in `descent` s "
                "(PATH-09, running or walking down over 25 deg: the foothold a step ahead lay over 40 u under the "
                "capsule, outside the lookup, so landings were aimed and pinned at the capsule's height - no foot "
                "loaded on 75-92% of steps, pins 45+ u over their floor, and K2_DESCENT saw no drop to follow) (user 11-13: \"falling A LOT on slopes and uneven terrain ... stuck "
                "going up a hill\"; the lookup is one ray 30 u over to 40 u under the capsule, and with the body "
                "hanging 22-38 u under it on a climb the ground under the feet was outside it, so pins, landings and "
                "the balance rules' support test fell back to the capsule's height: a planted foot 46 u over its "
                "floor with no load, landings 27-53 u off, then the fall); 0 = the capsule's window only",
                "the slope, ramp and stairs cases and the user's play decide" },
            { "OPENMW_RAGDOLL_K2_GUARD_THREAT", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 1.0f,
                "1 = an NPC's block takes the zone the incoming blade comes from (above the head the high "
                "hanging block, a side that side, low the low block) instead of the middle zone only the "
                "player's mouse moves (CLASH-02 11-05: a raised NPC guard let 3 of 3 chops past), where "
                "K2_WEAPON runs the wheel; 0 = the middle zone",
                "phase 1: the user's play decides" },
            { "OPENMW_RAGDOLL_K2_HAFT_CHOP", SwitchKind::Experiment, SwitchArea::Weapon, "1,0", 0.0f, 1.0f,
                "spear, staff: 1 = the haft's chop (K2_HAFT_LINES) is wound up further: the weapon hand behind the "
                "weapon shoulder at the neck's height with its elbow out and up, the support hand above the brow, "
                "the shaft over the shoulder with its point down behind the back; it passes through the old "
                "chamber both ways, and a chain into it (from the point guard, from a cut) is blended in by that "
                "chamber (AttackLine mChainSpan) (user 11-12: \"dragging up should wind the spear more like the "
                "spear should go behind the character back\"). CHOP-02 iron spear, off / on: the chamber's shaft "
                "up and 15 deg back, 10-15 deg off its command / back and 35-45 deg down, 18-25 off after a second; "
                "the point's way down 129 / 181-185 u, its top speed 2,300 / 2,000-2,450 u/s, the wrist within 3 deg "
                "of its limit, the end 17-21 deg off either way (5 runs on). Tried on the way: the elbow down (the "
                "wrist 12 deg out, the shaft sagged 16 -> 54 deg off in a second and the cut came round the side); "
                "chained from the point guard over the whole wind-up (the short turn went round the side: the "
                "chest 97 deg round, the wrist 55 out in FLOW-01, the end 61 off). The staff's stays off: wound "
                "this far it chopped at 1,430-1,455 u/s against 1,830 and one of 3 ended 63 deg off. Open: with "
                "K2_SPEAR_GRIP the spear is still on its way back when a quick chop is cut (FLOW-01 iron spear, "
                "deg a step: grip 0 25-28, 12 10-26, 24 9-13; held, the same). 0 = the chamber above the head",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_HAFT_GRIP", SwitchKind::Experiment, SwitchArea::Weapon, "0,60,8", 0.0f, 400.0f,
                "share, units/s, minimum units: on a spear or staff under hand control the support hand's point "
                "slides along the haft that share of the way from the authored grip to the point nearest the "
                "support shoulder (within the arm's reach, never nearer the weapon hand than `minimum`) (user "
                "11-06: \"a left hand that slides along the haft\"; it stayed ~22 u behind the right hand and was "
                "carried over the right forearm); share 0 = the authored grip. Off since 11-07: until then the "
                "grip rejudge snapped every slid unit back (it never slid past 1 u) and re-seated the anchor each "
                "step - the steel spear's chop ran at 39% of its kind's; slid for real (the rejudge now leaves it "
                "alone) the steel spear still chopped at 53%, the hands tracked worse (spears' hand p50 3.0-4.5 "
                "against 2.5-3.0) and two variants were flagged; off, none (VARIANT-01, 2 rounds)",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_HAFT_GUARD", SwitchKind::Experiment, SwitchArea::Weapon, "1,1", 0.0f, 1.0f,
                "spear, staff: 1 = the haft's own guard (K2_HAFT_LINES): the weapon shoulder 20 deg forward, the "
                "support hand beside its own hip, the butt running back past it; HSC_HAFT_CLEAR does not turn it "
                "(user 11-12: \"the left hand stays grabbing the spear but it is behind the back, the right arm "
                "grabs it normally\", \"usually after a winded up attack or moving a lot/getting hit\"). The "
                "sword's guard runs a haft's butt back past the weapon hip through the body, so HSC_HAFT_CLEAR "
                "turned the weapon arm's command ~30 deg to push the butt out to the weapon side while the support "
                "arm was asked to hold the haft on its own side: the arms fought, the guard sat 6-10 deg off its "
                "command (the \"10-13 deg off, cause not found\" of 11-12) and could fall over to the weapon "
                "arm's - the shaft 32-35 deg off, the support hand 7-9 u on the weapon side, through any knock "
                "(POINT-02 steel spear after a held dragged thrust, 4 runs of 4; a point guard or a redraw ended "
                "it). On: every guard after every held attack 0-1 deg off, the support hand 20 u on its own side "
                "(POINT-02 iron and steel, 5 runs). It needs K2_HAFT_THRUST: with the sword's thrust the guard "
                "still fell over (steel 34-35 deg in 3 runs of 3, iron 58 once). Tried and not kept: letting the "
                "support hand go and take the weapon again in a wrong guard (it latched where it was; free for "
                "4 s, the weapon arm alone still held the shaft 27-30 deg off) and the hand's command capped at "
                "0.97 of the arm (4 of 4 fell over). 0 = the sword's guard",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_HAFT_LINES", SwitchKind::Experiment, SwitchArea::Weapon, "1,1", 0.0f, 1.0f,
                "spear, staff: 1 = the player's spear / staff swings on its own lines, authored by both hands "
                "(cutwheel haftLines): the hands stay in front of the trunk about 21 apart, the shaft turns between "
                "them (a staff's push and pull) and the chest's turn carries most of the arc - level sweeps both "
                "ways, the chop, the four diagonals; the thrust is the sword's. User 11-12: \"swinging makes the "
                "arms get in weird positions\". The sword's lines move the weapon hand round the body and leave "
                "the support hand, 21 behind it on the haft, where nobody chose: haftfit measures it past its "
                "arm's reach (chop 8 u, backhand 7, low left 10), inside the trunk (forehand 6) or across the "
                "weapon arm (backhand 9) on every line; the haft's own keep within 2 u on all seven. FLOW-01, off / "
                "on (1 run each): iron spear wrist past its limit p90 21 / 0 deg, jolts 42 / 13, hand p90 9.6 / "
                "7.1 u, top speed 33 / 34; wooden staff 10 / 0, 31 / 17, 9.1 / 6.7, 29 / 34 (snaps 5 / 9 and 6 / 14). 0 = the "
                "sword's lines (a spear: with K2_SPEAR_SWEEP's backhand)",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_HAFT_NPC", SwitchKind::Experiment, SwitchArea::Weapon, "1,1", 0.0f, 1.0f,
                "spear, staff: 1 = NPCs' spears / staffs are on the haft's lines too (K2_HAFT_LINES, _GUARD, "
                "_THRUST and _CHOP as the player's; they were the player's only) (user 11-12: \"we need to be "
                "careful with the enemy hands not mixing up after a swing then returning in a weird position\"). "
                "DUEL-W02 (two NPCs, 40 s; the support hand in the chest's frame while it holds, the weapon off its "
                "command), off / on: iron spear out of attacks the support hand on the weapon side 68% / 2% of "
                "the time, the weapon 45 / 4 deg off (p50; p90 91 / 17), in attacks 57% / 3% and 40 / 12; steel "
                "spear 87% / 0%, 37 / 3 and 41% / 8%, 19 / 9; wooden staff 68% / 16%, 38 / 5 (p90 51 / 43) and "
                "43% / 15%, 40 / 15 - its thrust is still the sword's (behind the chest 1-3% / 8-10%). The point's "
                "speed out of attacks p50 117 / 76 (iron spear), 96 / 77 (staff). Health the spears took off "
                "their foes in 2 duels each: longsword 2, 3 / 9, 4; staff 7, 11 / 8, 5; warhammer 0, 13 / 7, 14 "
                "(of 35). Not looked at: the other two-handers - a claymore's, a battle axe's and a warhammer's "
                "support hand reads on the weapon side 20-34% of the time out of attacks and the weapon 45-55 "
                "deg off its command (their hands are close; the measure may not fit them). 0 = the sword's",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_HAFT_PIVOT", SwitchKind::Experiment, SwitchArea::Weapon, "0", 0.0f, 1.0f,
                "share: a spear or staff held in both hands turns about the point between the hands - the weapon "
                "hand moves so that point stays where the guard's turn would put it (user 11-06: \"turning around "
                "the point between the hands\"; the wheel turned it about the weapon hand), where K2_WEAPON runs "
                "the wheel; 0 = about the weapon hand. Off since 11-06 night: it doubled the weapon hand's distance "
                "from its goal (steel spear FLOW-01 hand p50 8.5-9.6 on, 3.2-4.4 off; play 27 u; user: \"can't "
                "hold it properly\"); the staff gained nothing (2.7 either way)",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_HAFT_THRUST", SwitchKind::Experiment, SwitchArea::Weapon, "1,0", 0.0f, 1.0f,
                "spear, staff: 1 = the haft's dragged thrust (K2_HAFT_LINES) is its own: side-on, the weapon "
                "shoulder forward, the weapon hand left of the middle and the shaft level through it, drawn back "
                "13 u and driven 20 down its own line as the chest turns on; the support hand stays under its own "
                "shoulder. The sword's draws the weapon hand back to the weapon hip: the support hand, 21 behind "
                "it on the haft, is asked 18 u behind the chest on the weapon side (haftfit) and went there (user "
                "11-12: \"left hand keeps going behind the back\"; POINT-02: 9-11 u behind, 18-19 across), and "
                "the guard after it fell over (K2_HAFT_GUARD). THRUST-01 iron spear, off / on: the chamber's shaft "
                "54 / 2-11 deg off its command, the hand 11.4 / 1.1-1.6 u; the point's path 30 / 0-15 deg off the "
                "facing and 29 down / 5-11 up, ending 14 left / 2-4 off the eye's line; reach past the guard 50 / "
                "40-48 u; the point's top speed 858 / 433-667 u/s (the old one's was the swing in). FLOW-01 the "
                "thrust's hold: the hand 3-8 / 1.3-1.9 u off, the wrist 30-57 / 0-4 deg past its limit (the cut's "
                "16-68 either way). SPEAR-01's thrust band: 5 of 12 thrusts on (8 of 16 before): its bruises are "
                "contacts moving across the shaft on the foe's upper arm (WOUND_TRACE: 2-26 u/s along it of "
                "110-190), not looked into. The staff's stays the sword's (no point guard to chain from; not "
                "tried). 0 = the sword's",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_HAND_ASSIST", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 1.0f,
                "with K2_HAND_DRIVE: this share of the blade's turn as a torque on the weapon from outside the "
                "body (the old spring's way, rotation only), measured as a budget (K2-WEAPON-phase1.md)",
                "phase 1: decided with K2_HAND_DRIVE" },
            { "OPENMW_RAGDOLL_K2_HAND_DRIVE", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 10.0f,
                "the weapon carried by the arm's own joints (ragdollcore handdrive: the grip by shoulder and "
                "elbow torques, the blade by the wrist's motor) instead of the spring on the weapon body, where "
                "K2_WEAPON runs the wheel: above 0 = on at that x the arm's strength, 0 = the old spring "
                "(K2-WEAPON-phase1.md order step 4)",
                "phase 1: on and the weapon spring deleted, or removed" },
            { "OPENMW_RAGDOLL_K2_HAND_EFFECTIVE_MASS", SwitchKind::Experiment, SwitchArea::Weapon, "1,1", 0.0f, 1.0f,
                "grip, turn: on a one-handed weapon the hand drive pushes the mass the grip really carries, the hand "
                "and weapon as one rigid body (handdrive HandChain::mWeaponInertia; lighter there when the "
                "weapon's centre sits far from the grip), and sizes the blade's turn on their inertia about their "
                "centre, not the wrist's (turn 1). With K2_DRIVE_RESPONSE the light one-handers stop ringing at "
                "the 60 Hz step (VARIANT-01 11-07: steel and daedric daggers ~450 jolts, the dwarven war axe ~420, "
                "the iron dagger 90 -> all 22-49; the arm had flipped +-9.5 rad/s every step in the guard). Not "
                "two-handers (the iron spear's wrist 25 deg past its limit with it); 0 = the masses whole",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_HAND_POSTURE", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 1.0f,
                "with K2_HAND_DRIVE: 1 = the arm keeps the game's solved posture on its own motors and the drive "
                "adds only its gravity support and grip push (no spring on the weapon), 0 = the drive's joint "
                "torques and wrist motor (K2-WEAPON-phase1.md)",
                "phase 1: decided with K2_HAND_DRIVE" },
            { "OPENMW_RAGDOLL_K2_HEAD_DAMP", SwitchKind::Experiment, SwitchArea::Legs, "0.5", 0.0f, 1.0f,
                "the player's head, the share held to its pose's world orientation instead of following the chest "
                "(user 11-08: head movement damped to about 50%); 0 = off",
                "the user's play" },
            { "OPENMW_RAGDOLL_K2_HELD_NOT_BODY", SwitchKind::Experiment, SwitchArea::Reaction, "1", 0.0f, 1.0f,
                "1 = a blow that lands on the victim's own held weapon or shield is a meeting of arms, never a hit "
                "on the victim (RAGDOLL_HELD_NOT_BODY): such a body has no region, and the legacy fallback paid it "
                "in full (11-07 play: a guard's blow on the player's claymore, body 20, cost 19.6 fatigue at 0 left "
                "and knocked them out); 0 = paid as before",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_HELD_NOT_GROUND", SwitchKind::Experiment, SwitchArea::Reaction, "0", 0.0f, 1.0f,
                "1 = a contact of the body's own weapon or shield, or with another fighter's, never counts as "
                "ground: every contact with an upward normal fed the support estimate, so blades pressed together "
                "at chest height put the \"ground\" above the pelvis, the pelvis height read 0 and a fighter who "
                "slowed under 20 u/s was judged lying (Bracing -> Fallen, root let go, motors off). 11-16 play, "
                "7 of the player's 7 falls: upright (chest 78-103 u up in six), weapon contacts 86-135 u up on "
                "every step before the change, no decided fall logged, 4.5-6.4 s to stand again. "
                "RAGDOLL_FALLEN_FROM_FEET logs what the ground was read from; 0 = every contact counts. Off until the "
                "balance and duel suites have run with it on (contacts feed the support estimate: high risk)",
                "the suites, then the user's play" },
            { "OPENMW_RAGDOLL_K2_HELD_RECOVER", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 1.0f,
                "1 = a held cut that has reached its end recovers after its hold with the button still down "
                "(unless dragged on along the stroke: the run-on), and the mouse no longer drags a finished "
                "cut back along its path, where K2_WEAPON runs the wheel; 0 = it waits for the release (B14)",
                "phase 1: the user's play decides" },
            { "OPENMW_RAGDOLL_K2_HIP_TURN", SwitchKind::Experiment, SwitchArea::Weapon, "0.5", 0.0f, 1.0f,
                "the hips' share of the wheel's lead chest turn (0.45 rad at most): the root turns over the "
                "planted legs and the spine is asked only the rest (its joints twist 53 deg at most), where "
                "K2_WEAPON runs the wheel; 0 = the spine alone (play-6 bookmark, chest rotation)",
                "phase 1: the user's play decides" },
            { "OPENMW_RAGDOLL_K2_HOLD_AIM", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 1.0f,
                "an undecided attack press keeps the mouse off the view, and 24 counts of travel make it a drag at once, "
                "its wind-up from the way the mouse went (user bookmarks 11-05: \"holding down click lets the player start "
                "the swing from that direction\"; \"if you miss the click the camera moves a lot\"); 0 = the view turns "
                "for the press's first 0.16 s",
                "the user's play decides (release 0.2)" },
            { "OPENMW_RAGDOLL_K2_LEASH_GAIT", SwitchKind::Experiment, SwitchArea::Legs, "1", 0.0f, 1.0f,
                "the feet leash's own pull on the capsule (it waits for the feet on stairs and slopes) is no "
                "travel for the gait: the legs keep stepping at the capsule's own speed while it waits (11-07 "
                "trace, strafing up stairs: the pull read as -130..-214 u/s and 0 for 0.5 s, the gait settled "
                "with the feet 40-60 u behind the pelvis and the body fell over them); 0 = the pull counts",
                "the stairs cases and the user's play decide" },
            { "OPENMW_RAGDOLL_K2_LEFT_ARM_FRAMES", SwitchKind::Experiment, SwitchArea::Lifecycle, "1", 0.0f, 1.0f,
                "the left arm's joint frames come from the standing pose as the legs' and spine's do, not from "
                "the pose of the build (user bookmark 11-09: \"left arm still stiff with weapon sheathed\"; a rig "
                "rebuilt from the weapon's ready stance swung the sheathed left hand 19 u of 28 asked, the right "
                "31 of 33; built sheathed the left swung 32); 0 = the build pose",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_LEFT_FIST_HITS", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 1.0f,
                "the left fist's punch is a swing of its own for the strike selection (its own swing identity, the "
                "contact marked as swinging). Its contacts carried the wheel's swing state - idle between the right "
                "fist's punches - and a contact outside a swing is dropped: FIST-09's left taps met the head at "
                "80-170 u/s and none dealt anything; in the user's 11-16 fist sessions 31 of 136 left punches "
                "touched and about 8 dealt anything. 0 = as before",
                "FIST-09: the left taps land" },
            { "OPENMW_RAGDOLL_K2_LEFT_PUNCH", SwitchKind::Experiment, SwitchArea::Weapon, "0.6,1", 0.0f, 1.0f,
                "turn share, aim: FIST-05's left-hand punches turn the trunk by that share of their line's keyed "
                "turn (mirrored: the left shoulder goes in), and with aim 1 their line turns to where the crosshair "
                "points as the fist goes out (user 11-11: hand to hand \"unsatisfying\"; the left punch's keyed turn "
                "was computed and never applied, so half of all punches were the arm alone, and its line ignored "
                "the view); 0,0 = the arm alone on its line",
                "FIST-05 and the user's play decide" },
            { "OPENMW_RAGDOLL_K2_LOCKOUT", SwitchKind::Experiment, SwitchArea::Weapon, "0", 0.0f, 1.0f,
                "share of every attack/guard lockout - a clash's knockaway (0.4 s), broken guard and disarm "
                "(0.55 s), the bearings after a hit, and K2_CLASH_OPEN's open guard: 1 = as before, 0 = none "
                "(user 11-05: \"lets get rid of the lockout timers just for a test\"; the struck arm's real "
                "knock still has to come back)",
                "the user's play decides: kept at 0, or back to 1 and deleted" },
            { "OPENMW_RAGDOLL_K2_LODGE", SwitchKind::Experiment, SwitchArea::Weapon, "1,1,1", 0.0f, 1.0f,
                "stab, chop, pass (each 1 or 0): what a paid blow's weapon does in the body (ragdollcore lodge). "
                "Stab: a point goes in 0.6 u a point of damage (2-20 u) and is pinned on its own line for up to "
                "1.5 s - the body slides along the blade, not sideways, until it is drawn out. Chop: an edge "
                "that bites square-on (closing faster than it slides) lodges 1.5-7 u deep for up to 0.5 s. Pass: "
                "a drawn cut or a cleave goes through the body for 0.2 s and drags the part along the stroke "
                "(half the blade's sliding momentum, 900 kg u/s at most). The weapon goes through the body while "
                "held and until it is out; forces only, no joint (user 11-16: \"stab weapons physically enter "
                "the body ... pin the tip ... an equivalent for slashing\"); 0,0,0 = hit-stop and press as before. "
                "On in the hub from play-2026-11-16-10 at the user's word (\"pls add stab and pin\"), EXPERIMENTAL: the "
                "pin lets go in 0.05-0.15 s and lodged thrusts pay about half (LODGE-01); no suites run",
                "LODGE-01, LODGE-02 and the user's play decide" },
            { "OPENMW_RAGDOLL_K2_MIDDLE_PARRY", SwitchKind::Experiment, SwitchArea::Weapon, "0", 0.0f, 2.0f,
                "what the middle mouse button does: 0 = nothing (kept for the left-hand spell, SPELL-01; user "
                "11-10: \"middle mouse: cast spell\", after a day with the guard on it: \"tapping the middle mouse "
                "button is crazy lol needs a lot of force\"); 1 = it guards and parries as the right one does "
                "(hold = guard, its zone by the mouse; a tap = the timed parry), without the shove; 2 = it kicks. "
                "The kick is its own action at every value (A_PhysicalKick, X by default, in the controls menu)",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_MOMENTUM_WIND", SwitchKind::Experiment, SwitchArea::Weapon, "0.6", 0.0f, 2.0f,
                "share: the wind-up is up to 1 + this as fast with the body moving along the coming cut (full at 300 "
                "u/s, half credit for moving forward into any cut; user bookmark 11-05: \"the inertia helps it build "
                "up\"), where K2_WEAPON runs the wheel; 0 = off",
                "the user's play decides (release 0.2)" },
            { "OPENMW_RAGDOLL_K2_MOVE_ACCEL", SwitchKind::Experiment, SwitchArea::Root, "0", 0.0f, 20000.0f,
                "low, high u/s^2: how fast a held body's capsule may change its horizontal velocity, read from the "
                "actor's Speed (low at 0, high at 100) instead of active-body.cfg's capsule_acceleration (1400 for "
                "everyone); starts, stops and reversals quicken together (user 11-12: \"a bit less "
                "friction/inertia\", \"it takes so long for the player to start running\"; at 1400 the capsule "
                "reached 90% of a run 0.17-0.2 s after the key, the pelvis 0.28-0.43 s). PLAY-01 start stop, zigzag, "
                "reversal, run straight, run turn 180, run fast turn 4 at 2800,2800 against off, x3: the capsule at "
                "90% of a run in 0.08 s (0.15), stopped in 0.08 s (0.15-0.17); the body at 90% in 0.27-0.30 s "
                "either way (its horizontal hold sets that, not this bound) and stopped in 0.18-0.20 s (0.22); no "
                "fall, end lean alike; 0 = off",
                "the user's play; the body's own start lag is the hold's" },
            { "OPENMW_RAGDOLL_K2_NPC_COMBO", SwitchKind::Experiment, SwitchArea::Weapon, "0.6,0.4", 0.0f, 1.0f,
                "second, third: an NPC's attack chains into a second cut at this chance at skill 100 (x skill), "
                "and a second into a third, each on the NPC's next line with the momentum carried, where "
                "K2_WEAPON=2 runs NPCs on the wheel; 0 = one cut an attack (play-6 bookmark, attack patterns)",
                "phase 1: the user's play decides" },
            { "OPENMW_RAGDOLL_K2_NPC_REFALL", SwitchKind::Experiment, SwitchArea::Lifecycle, "2,60", 0.0f, 600.0f,
                "falls, seconds: an NPC out of combat that falls within 3 s of a get-up that many times inside the "
                "window goes on the vanilla capsule (as a stuck NPC does) until it has walked 300 u on (user 11-13: "
                "NPCs \"falling A LOT on slopes\"; one standing on a 42 deg cave slope got up 15 times in 14 "
                "minutes, down again 1.7 s after each); 0 = it keeps getting up",
                "a fallback: the steep-ground stand itself is open; the user's play decides" },
            { "OPENMW_RAGDOLL_K2_NPC_SHOVE", SwitchKind::Experiment, SwitchArea::Reaction, "1,0.35,0.12,4,0.3", 0.0f, 10.0f,
                "on, guard rate, close rate, cooldown s, least fatigue: SHOVE-02 (user 11-07: \"enemies should be "
                "able to shove the player as well\"), an NPC fighting the player shoves it with the player's own "
                "shove (SHOVE-01) - within reach (65 u) and facing it, between its own attacks, at the guard rate "
                "per second into a raised guard, the close rate closer than 50 u, x 0.6-1.5 by Strength, never "
                "within the cooldown or below the least fatigue (ragdollcore npcshove; RAGDOLL_NPC_SHOVE); 0 = off",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_NPC_START", SwitchKind::Experiment, SwitchArea::Ai, "350,6", 0.0f, 20000.0f,
                "u/s^2, rad/s: from a stop an NPC ramps up to its speed, and turns no faster than the second value "
                "(user 11-08: NPCs turn at once, then a startup stride that looks off; the trace: 0 to 130-174 u/s "
                "in 0.13 s); each 0 = off",
                "the user's play" },
            { "OPENMW_RAGDOLL_K2_PACE_FLOOR", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 1.0f,
                "share: a weapon is paced (the wheel's times and route, the hand drive's stiffness) at its weight but "
                "never below this share of the iron weapon of its kind (cutwheel pacedWeaponWeight): every weapon "
                "body is 1 kg, so a lighter blade's quicker command outran the body and swung slower (user 11-06: "
                "\"glass claymore control was pretty bad ... couldn't swing like steel claymore\"; FLOW-01: glass "
                "22-25 deg/step, steel 29-33); 0 = the weapon's own weight",
                "until real weapon masses (K2-WEAPON step 7)" },
            { "OPENMW_RAGDOLL_K2_PALM_PARRY", SwitchKind::Experiment, SwitchArea::Weapon, "1,0.22,1,12,30,0.9,1", 0.0f, 80.0f,
                "on, reaction s at no skill, longest palm out s at full skill, cover u, knock u, opening s, the "
                "reaction's factor in a wrong zone (1.5 as first built; 1 from 11-17, the user: \"its hard to parry... "
                "needs to be easier\" - a tap's flick picks a zone every time and 13 of their 34 palms waited the "
                "1.5x) (FIST-08; user "
                "11-16: \"I'm not sure if the blocking is working ... 'parrying' would be something like throwing a "
                "palm in that direction and that causing weapons to be deflected\"): the player's bare-handed guard "
                "(both buttons) against a weapon - the hands wait open where the guard zone puts them, the hand on "
                "an incoming stroke's side throws its palm at the foe's weapon (half way from its hand to its striking "
                "part) after the reaction time, and for the time it is out a blade that meets that arm or lands "
                "within `cover` of the palm is deflected: knocked aside, the cut ended, the foe open, vanilla block "
                "fatigue paid, a share of the blow still taken at low skill; a blow before or after it, or away "
                "from it, lands (under 200 u/s onto the raised hands it is still stopped). Hand-to-hand + Agility/5 + Luck/10 by fatigue (vanilla's block term) sets the "
                "times and shares; the zone the blow comes in halves the reaction (ragdollcore palmparry; "
                "RAGDOLL_PALM_PARRY; design docs/ragdoll/designs/FIST-08-palm-parry.md). FIST-08 (a longsword on the "
                "bare-fisted player, 30 s a phase, six runs): 1-5 blows a run deflected by a palm, 15 and 121 health "
                "lost against 175-319 with the hands down and 0-263 under the old guard; "
                "on 0 (the default until FIST-08 passes several runs) = the old guard (a blow within GUARD_COVER of a "
                "bare hand is blocked whole)",
                "FIST-08 passing several runs, then the user's play" },
            { "OPENMW_RAGDOLL_K2_PALM_TAP", SwitchKind::Experiment, SwitchArea::Weapon, "0.45,0.3,0.8", 0.0f, 2.0f,
                "window s, shove s, carry s (user 11-17: \"I'm not able to parry pls check why\", then \"Yes, tap parries\"): "
                "with bare fists a tap of both buttons (down under 0.2 s) is a timed palm parry - the guard stays up "
                "for `window` s from the press, the zone keeping the mouse through it as K2_TAP_PARRY_AIM does - and "
                "the shove is both buttons pressed again inside `shove` s of the last press (the weapons' double "
                "click). Both held is the guard as before; a single button pressed in the window ends it. The "
                "player's session of 11-17: 52 presses of both, median 0.127 s, 45 under 0.2 s - 21 shoved, 24 did "
                "nothing, the guard was down for 22 of the 24 blows that landed and up 28-30 ms for the other two, "
                "and all 8 palms thrown were taken back when the buttons came up 17-251 ms later (the foe's wind-up "
                "is 0.31 s, its cut lands 0.33-0.96 s after the wind-up begins). Carry (user 11-17 on the first tap "
                "build: \"damn its hard to parry... needs to be easier\"; 22 parries tried, 2 deflected, 5 stopped on "
                "the hands, 8 lost to the window ending 12-100 ms before the blade, 5 to the second tap's shove or "
                "a button of the pair let go and pressed again, 2 late): a tapped guard stays up to `carry` s past "
                "its window while it reads a blow or its palm is out; with a carry set a press of both while a "
                "blow is read is never the shove, and a button of the guard pressed again while the other is still "
                "down is both again; carry 0 = none of the three. Not measured in the game yet "
                "(FIST-08's tap phase); window 0 = a tap of both shoves as before (mousemanager.cpp; ragdollcore "
                "palmTapLeft, palmTapShoves)",
                "FIST-08's tap phase, then the user's play" },
            { "OPENMW_RAGDOLL_K2_PARRY_STEER", SwitchKind::Experiment, SwitchArea::Weapon, "12", 0.0f, 30.0f,
                "units: the player's block hand moves up to this far from its zone's place (0.4 of it at no weapon "
                "control, all at full), across the front, to where the foe's striking part (0.7 of hand to tip, read "
                "up to 0.25 s ahead on its velocity) will cross the guard, so the blade's strong half meets it (user "
                "11-12: \"add steering and tolerance zones\"; each zone was one fixed hand place and the blade only "
                "leaned half toward the foe's weapon as it is now); 0 = the zone's place",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_POINT_AIM", SwitchKind::Experiment, SwitchArea::Weapon, "0,1800,500", 0.0f, 100000.0f,
                "cone deg, mouse counts a radian, flick counts in 0.2 s. Off in play from 11-16 (user bookmark: "
                "\"remove the 'click and aim' from the spear, it is currently making winding up back the spear for a "
                "thrust harder as I need to move my mouse down more than I'd like\"; that session's wind-ups all began "
                "as flicks of 500-650 counts, and slower down-drags of 687 and 769 counts stayed in the point guard "
                "and wound nothing; SPEAR-02, slow down-drags of 480, 600 and 900 counts, iron and steel spear, off / "
                "on, 1 run: the thrust winds from the drag's first step, the point drawn back 22-36 u, the cut "
                "reaching 21-28 u past the guard / nothing wound, drawn back 0, 8-10 u), was 25,1800,500. Off, a "
                "click on the thrust line is the haft's thrust (K2_HAFT_THRUST). On: the player's spear (not a staff), the "
                "attack button held, is carried point-forward and the mouse moves the point - the rear hand stays, "
                "the shaft turns about it toward the aim's point (the crosshair's, a near foe's) offset by the "
                "mouse, within the cone, following it over 0.05 s; the release thrusts straight down the shaft "
                "(drawn back, then driven out, K2_POINT_POSE). A drag of the flick's net travel within 0.2 s, or "
                "pushed 300 counts past the cone's edge, winds up a swing on the wheel as before; a press let go "
                "within 0.2 s without a drag is the click it was (on the thrust line: this thrust, at the aim). "
                "User 11-12: \"holding click lets you control/aim the tip\"; the mouse moved the weapon hand's "
                "target among the wheel's lines, never the point. The command is trimmed until the real point is "
                "where it was asked (the held weapon settled 8 u right of it). First built with a speed rule for "
                "the swing (3200 counts/s) and 20,1200: in the user's 11-12 session the mouse's counts came in "
                "bursts (peaks of 4,000-38,000 counts/s on moves of 60-400 counts) and 26 of 29 presses left the "
                "point guard; the swings' drags were 600-2,400 counts, the aimed ones under 400 (\"needs better "
                "control for aiming\"). POINT-01 with the first build (1 run): the held point 0.04 rad off the "
                "middle, a 0.2 rad drag moves it 0.18-0.20 either way. First value 0 = off: a held drag winds up "
                "on the wheel",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_POINT_LEAD", SwitchKind::Experiment, SwitchArea::Weapon, "0.3,0.6", 0.0f, 1.0f,
                "lead, tip start: while a spear's attack is a thrust (the wheel's thrust line; an NPC spear's every "
                "attack), its contact in the tip zone (from that share of grip-to-tip out) is a puncture when the "
                "motion runs along the shaft toward the point at least `lead` of the speed, whatever the contact "
                "normal says (user 11-11: spears \"unsatisfying\"; the 11-09 spear session: 46 of 64 thrusts landed, "
                "36 of the 40 thrust hits read as shaft bruises and paid the 2-5 band at 0.6, 4.6 a hit against 24 "
                "for the four punctures and 25-36 for a claymore's; SPEAR-01: every thrust's contact normal lay "
                "across the shaft, the paid contact 0.63-0.98 of the way out moving 0.51-0.89 along it); 0 = the "
                "normal must face the point and the zone starts at 0.85",
                "SPEAR-01 and the user's play decide" },
            { "OPENMW_RAGDOLL_K2_POINT_POSE", SwitchKind::Experiment, SwitchArea::Weapon, "-9,18,-6,-35,6,20,21",
                -60.0f, 60.0f,
                "right, forward, up, twist deg, draw, reach, grip: K2_POINT_AIM's point guard - the weapon hand's "
                "place from the chest centre with the point straight ahead (units), the chest's turn (negative: "
                "the weapon shoulder forward), the thrust's draw back and reach along the shaft (units; the reach "
                "stops at 0.95 of the arm), and the rear hand's distance behind the weapon hand (the shaft turns "
                "about the rear hand's place). The body stands side-on with the spear on the support hand's "
                "side: with the arm 35 u long and the hands 21 apart, a level spear in front of the body put the "
                "rear hand in the belly (2,24,-12 turned -28: the real point 26 u above its command); side-on "
                "the hands track within 1-2 u. The thrust moves the hand along the shaft and nothing else (user "
                "11-12: \"the spear should go straight forward and it goes up forward and diagonally\": the "
                "first build ran the hand along the aim's line, 10 deg off the shaft, brought it 9 u back across "
                "the body for reach and let the trim go through the drive). The draw is 6: at 12 the rear hand met "
                "the hip and the point swung 12 u aside before the drive",
                "tuned with K2_POINT_AIM" },
            { "OPENMW_RAGDOLL_K2_PUSH_KICK", SwitchKind::Experiment, SwitchArea::Reaction, "1,0.1,0.25,4,2.5,320,0.2,200", 0.0f, 1000.0f,
                "on, body share at Hand-to-hand 0, added at 100, Strength cap, carry share, carry cap u/s, recoil "
                "share, felling carry u/s: the kick lands as the leg plus that share of "
                "the kicker's mass, at the foot's speed x Strength / 50 (up to the cap) plus the kicker's run-in (user "
                "bookmark 11-05: \"at very high strength a kick should be able to set them flying, strength + something "
                "else\"); 0 = KICK-01's push (14 kg, Strength capped at 1.4). The carry (user 11-09: a good heavy "
                "kick): the struck part's push is capped at a body part's speed, so a kick of "
                "any strength moved its target 45-50 u; the victim's whole body now takes `share` of the push's "
                "momentum as speed (the legs less toward the feet), up to the cap, an unguarded victim carried at "
                "the felling speed or more goes down, and the kicker's body takes `recoil` of it back. KICK-01, the "
                "target's travel: 45 u standing off; these values (carry 229 u/s, felled) 352-353 u (2 runs); not "
                "felled, carry 228-240 u/s, 116-119 u standing; felled at 274-289 u/s 413-445 u (3 runs each). "
                "The kicker's drift stays 6-8 u: its recoil does not show. Carry 0 = off",
                "the user's play decides (release 0.2)" },
            { "OPENMW_RAGDOLL_K2_REGRIP", SwitchKind::Experiment, SwitchArea::Weapon, "0.5", 0.0f, 10.0f,
                "seconds: a weapon whose grip joint broke under load (linear impulse over 10,000 or angular over "
                "250,000) while it is still the equipped weapon is gripped again that long after it let go - the "
                "body is rebuilt at the next quiet standing step, as for an equipment change. A released weapon was "
                "never re-gripped: the rebuild asked for a different weapon id, so sheathing and drawing did "
                "nothing. 11-16 play, a steel spear: the grip let go in a jam during a stagger (the hand 39 u from "
                "its goal), the engine fought with fists for 42 s through two sheathe-and-draws, the spear's model "
                "on the right hand bone, until the spear was taken off and put back on. A weapon dropped, "
                "unequipped or let go at death stays let go. RAGDOLL_WEAPON_RELEASE logs every release (reason, the "
                "grip's impulses against its limits), RAGDOLL_WEAPON_REGRIP each re-grip. REGRIP-01 (the dev call breaks a drawn weapon's grip, 1 run each): steel spear, steel staff and iron longsword held "
                "again 0.52 s after; with 0 none within 2.5 s (spear, staff); a spear let go as dropped stays let go "
                "both ways. 0 = off",
                "the user's play; then the break limit itself or a real disarm (the user's call)" },
            { "OPENMW_RAGDOLL_K2_ROOT_FIELD", SwitchKind::Experiment, SwitchArea::Root, "0,0", 0.0f, 5000.0f,
                "from, full u/s asked: the whole-body drive - at a run the soft root's horizontal pull acts on "
                "every part but a grounded foot by mass, like a field, instead of on the upper body that towed "
                "the legs through the hips; the same force, only where it acts; the share rises from 0 at `from` "
                "to 1 at `full` (user 11-07: an answer to fast charges without K2_GAIT_CAP); 0 = off",
                "phase 2: A/B against K2_GAIT_CAP, then the user's play" },
            { "OPENMW_RAGDOLL_K2_ROOT_TRACE", SwitchKind::Debug, SwitchArea::Root, "", 0.0f, 1.0f,
                "set: the root's strength terms (RAGDOLL_K2_ROOT) for the player every 0.5 s; never changes "
                "behaviour",
                "" },
            { "OPENMW_RAGDOLL_K2_SHOVE_ARMS", SwitchKind::Experiment, SwitchArea::Reaction, "1", 0.0f, 1.0f,
                "1 = a shove throws both arms forward (user 11-12: \"make it so shoving throws the arms forward\"): "
                "the weapon hand thrusts straight out from the shoulder (0.97 of the arm, the weapon upright before "
                "it, the chest leaning in as for a thrust; the planner's block with its own key, WheelInput::mShove) where it stayed on the wheel, the arm "
                "guarding beside the weapon joins (the double click's first click had raised that guard, which kept "
                "it), and a shove with nobody in reach still throws the arms out (no lunge, no fatigue, the cooldown); "
                "0 = only an unoccupied hand goes to a shoved chest",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_SIDE_STAIRS", SwitchKind::Experiment, SwitchArea::Legs, "1,0.3,0.05", 0.0f, 2.0f,
                "on, swing s, pause s: side-stepping stairs - travelling more sideways than forward with a riser "
                "within 48 u along the leading foot's way (or the feet on two treads), the gait clock stands aside "
                "and the feet take turns: the leading foot onto the next tread, deep enough for the trailing foot "
                "beside it, then the trailing foot there, each lifting `pause` after the other lands and swinging "
                "`swing` (11-07: with the clock every step fix left strafing up the test stairs at ~1 fall in 4); "
                "0 = off",
                "the stairs cases and the user's play decide" },
            { "OPENMW_RAGDOLL_K2_SIDE_TURN", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 3.0f,
                "a sideways cut turns the chest this much more than its line's keys (x cos^2 of the line's "
                "angle: a full forehand or backhand all of it, a diagonal half, a chop none), the hips leading "
                "the same; weapons only, where K2_WEAPON runs the wheel; 0 = the keys' turn (play-6 bookmark)",
                "phase 1: the user's play decides" },
            { "OPENMW_RAGDOLL_K2_SPEAR_GRIP", SwitchKind::Experiment, SwitchArea::Weapon, "24,80", 0.0f, 140.0f,
                "units, point: a spear is held `units` nearer its butt than the animation holds it (at the mesh's "
                "middle: an iron spear's point 55 u ahead of the weapon hand, a steel spear's 67), its point at "
                "most `point` ahead of the hand and at least 28 u of butt behind it: the weapon body, its grip and "
                "the drawn spear (getWeaponGripTurns) move through the hand toward the point; the support hand "
                "stays 21 behind the weapon hand (user 11-12: \"is it possible that we grab the spear from lower? "
                "like increasing the reach? that would also make it clip less into the player\"). Iron spear 23 u "
                "(the point 78 ahead, the butt 28 behind, 51 before), steel 12.7 (80 and 58, 70 before). Off / on: "
                "SPEAR-01's reach 92 / 116 u, THRUST-01's reach past the guard 41 / 50; POINT-01 the guard's point "
                "73 / 95 u ahead (steel 85 / 98), the thrust's bow 6.5 / 5 (steel 5.4 / 5; uncapped, its point 91 "
                "ahead, 10: a fail). The price is the lever: the weapon's centre is 23 u from the hand, so it "
                "turns slower - a quick chop is cut while the spear is still winding (K2_HAFT_CHOP), and the "
                "dragged thrust's wind-up holds the wrist 30-57 deg past its limit in 10 holds of 12 (grip 0: 2 of "
                "6; the cut's 45-70 either way). SPEAR-01's thrust band: 8 of 16 thrusts on, 11 of 16 off (it is "
                "bimodal; not told apart). Built with the body: a held spear takes it at its next draw. Not "
                "looked at: a spear drawn on the back by a sheathing mod. 0 = the animation's grip",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_SPEAR_SWEEP", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 1.0f,
                "1 = the player's spear has its own left-to-right sweep: chambered low across the front, rising to "
                "high on the right (user 11-07, 11-08: the sword's backhand feels flat on a spear); 0 = the sword's",
                "the user's play" },
            { "OPENMW_RAGDOLL_K2_SPEAR_THRUST", SwitchKind::Experiment, SwitchArea::Weapon, "1,0", 0.0f, 1.0f,
                "player, npc: 1 = a spear's thrust points at the aim: the blade is turned about the hand until "
                "its point lies on the aim's line (wheelaim pointThrust, from the wind-up's second half, at most "
                "45 deg; the hand's path is the sword's), and with no foe on the ray the crosshair point lies "
                "at the point's reach, not the blade's sweet spot (aimed from the chest at a point 72 u along a "
                "ray from the eye, a spear pointed 15 deg above the view). The sword's thrust ends with the "
                "blade 25 deg up (user 11-09 bookmarks: \"hit at the crosshair ... because my left arm was "
                "behind my back\"; 11-12: \"way too diagonally forward ... not like a horizontal piercing "
                "motion\"; that session's 63 thrusts: the commanded point 39 u above the crosshair's line). "
                "THRUST-01 iron spear, off / on (2 runs each): the point ends 30 u above and 7 left of the eye's "
                "level line / 4-5 below and 7 left; the shaft 6 / 10-11 deg off its command at the end; reach "
                "past the guard 34 / 41 u; the point's top speed 660 / 750-780 u/s; SPEAR-01 a thrust 7.9-11 / "
                "12.6-13.1 in 3 runs, 8.1 and 2.2 (a fail) in 2. Not fixed by it: the point still swings in "
                "from the high chamber (its path 27 deg sideways either way) - with both hands on, the support "
                "hand holds the shaft 62-72 deg off its command through the wind-up. Tried and not kept, 11-12 "
                "(THRUST-01, FLOW-01, SPEAR-01): the thrust's own keys along the left flank (the point 10 u off "
                "the line on a path 4-9 deg sideways, but the wind-up needs the weapon wrist at a right angle: "
                "in 2 holds of 4 the hand stood 23 u off, the wrist 16-41 deg past its limit) and on the right "
                "with the hands 12 u apart (16 u off, the low left cut's wrist 39-45 deg out); a second hand "
                "drive on the support arm (the left-to-right cut 26 -> 14 deg a step) and its mass on the grip "
                "spring (nothing); the haft slid through the support hand (21 u off); the support hand off the "
                "haft through the thrust (the wind-up 4 deg off, the end 21: a thrust took 1.3). The arm is 35 "
                "u from shoulder to grip, the hands 21 apart. 0 = the sword's thrust",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_SPEAR_THRUST_PACE", SwitchKind::Experiment, SwitchArea::Weapon, "0.1,0.14,0.3", 0.0f, 2.0f,
                "wind-up s, cut s, return s: the player's spear or staff on the thrust line winds up, strikes and "
                "returns in these times (/ the weapon's response; never slower than its weight's pace) and a press "
                "chains from any stage at any skill, as the dagger's thrust does at 0.04, 0.07, 0.16 (K2_DAGGER_THRUST). "
                "User bookmark 11-16: \"spear thrust windup should be much easier, almost like dagger (but the speed "
                "should be slower than dagger)\"; the iron spear's weight pace is about 0.43, 0.31, 0.6. Swings on "
                "the other lines keep the weight's pace; NPCs keep theirs. 0 = the weight's pace and chains from skill 50",
                "the user's play decides; THRUST-01 iron spear for the point's track" },
            { "OPENMW_RAGDOLL_K2_STAFF_AS_SPEAR", SwitchKind::Experiment, SwitchArea::Weapon, "1", 0.0f, 1.0f,
                "1 = a staff (BluntTwoWide) is held and driven as a spear, the player's and NPCs' (user 11-16: \"for "
                "release sake lets make it exactly the same as the spear only to have well the staff stats\"): the "
                "spear's lower grip (K2_SPEAR_GRIP), lines, chop wind-up, guard, dragged and standing click thrust, "
                "sweep and support hand - every `spear, staff` pair of the K2_HAFT_* switches reads its spear value. "
                "The staff's own stay: its record's damage, speed and reach, its weight's pace and blunt blows (no "
                "blood on the point, no lodging). Wooden staff, off / on, 1 run each (11-16): held 0 / 22.9 u nearer "
                "its butt, the guard's tip 66.5 / 84.5 u out; THRUST-01 the dragged thrust reaches 0.4 (it did not "
                "go in that run; the sword's thrust) / 36.0 u past the guard on its own line (4 up, 7 left), the "
                "hand 1.0 u and the shaft 1 deg off their command; CHOP-02 the point's way down 90 / 189 u (the "
                "iron spear's 181-185), top speed 1,924 / 2,056 u/s, the wrist within its limit both ways; FLOW-01 "
                "passes both ways, the support hand never lost. Not run: VARIANT-01's other staffs, NPC staffs, a "
                "duel. 0 = the staff's own grip at its middle, the sword's thrust and the old chop chamber",
                "THRUST-01, CHOP-02 and FLOW-01 wooden staff; VARIANT-01's staffs; the user's play decides" },
            { "OPENMW_RAGDOLL_K2_STAIR_STRAFE", SwitchKind::Experiment, SwitchArea::Legs, "0.5,8,0.8", 0.0f, 60.0f,
                "speed share, units, seconds: a powered body strafing (more than half its travel sideways) on "
                "stairs - both feet planted more than `units` apart in height in the last `seconds` - moves at "
                "`share` of its speed, the capsule and the movement clip both (11-07: strafing up the test stairs "
                "at 90 u/s each side step climbed 2-3 treads, 69 u a step, and the body fell over a foot caught "
                "on a riser about 1 run in 4 with every step fix); 1 or 0 = off. Alone (0.6) it still fell 6 of 24; "
                "with K2_SIDE_STAIRS's steps (0.5) 3 of 44 against ~1 in 4 (11-07)",
                "the stairs cases and the user's play decide" },
            { "OPENMW_RAGDOLL_K2_STRIDE_GRIP", SwitchKind::Experiment, SwitchArea::Legs, "0", 0.0f, 1.0f,
                "the legs-lead stance force's grip cap from the planted feet's contacts only (phase 2, MOVE-01 4.5): "
                "it took the lowest friction over both feet, so a swinging foot brushing the floor (its 0.05 grip, "
                "friction ~0.09) capped the planted foot's push as on ice (StrideSim sprint: the push when planted "
                "54 -> 281 u/s^2); 0 = the lowest over both feet",
                "phase 2: A/B in the game, then the user's play" },
            { "OPENMW_RAGDOLL_K2_STRIDE_LEAD_FIRST", SwitchKind::Experiment, SwitchArea::Legs, "0,25", 0.0f, 100.0f,
                "sideways share, spread units: Stride's lead foot (phase 2, MOVE-01 4.7) - moving mostly sideways "
                "with both feet down, a lift due to the trailing foot goes to the foot on the side of travel, unless "
                "it already stands `spread` further along (the game's walking strafe: the trailing foot stepped first, "
                "held by the no-cross guard, and the lead foot's late 89 u step dragged while the pelvis fell to "
                "31-42); 0 = off",
                "phase 2: A/B in the game, then the user's play" },
            { "OPENMW_RAGDOLL_K2_STRIDE_SIDE_RISE", SwitchKind::Experiment, SwitchArea::Legs, "20,0.5,0.3", 0.0f, 100.0f,
                "units, sideways share, lock: a sideways step climbs or descends a tread at a time - travelling more "
                "sideways than `share`, a landing more than `units` above or below the other foot's ground comes "
                "back toward it (the whole foot on its tread), never closer than the feet's gap (11-07 trace, "
                "strafing up stairs: the lead foot aimed 33-48 u up, its swing goal at the hip's height; the toe "
                "caught the riser, it landed 25-52 u short and the body fell over it); from `lock` of its phase "
                "a sideways swing keeps its tread (the last re-aim moved a landing a tread on, 33 u and a riser, "
                "and it landed 32 u short; 0 = no lock); 0 = off",
                "the stairs cases and the user's play decide" },
            { "OPENMW_RAGDOLL_K2_STRIDE_SWING", SwitchKind::Experiment, SwitchArea::Legs, "0", 0.0f, 1.5f,
                "share of the leg's length: Stride's swing (phase 2, MOVE-01 4.4) - the walking swing spring "
                "(PGC's, OPENMW_RAGDOLL_PGC third knob) pulls each foot to a landing held inside that share of the "
                "leg's reach from the hip where the hip will be at touchdown (StrideSim 11-06: pulled past its "
                "reach a strafing foot hung at full stretch and the stance knee buckled); 0 = the spring unclamped",
                "phase 2: A/B in the game, then the user's play" },
            { "OPENMW_RAGDOLL_K2_STRIDE_TURN", SwitchKind::Experiment, SwitchArea::Legs, "10,0.12", 0.0f, 60.0f,
                "units, seconds: Stride's turn (phase 2, MOVE-01 4.7) - a swing whose landing the no-cross guard "
                "holds back by more than `units` while the travel runs over 45 deg off the swing's own direction "
                "(a walk turned into a strafe) lands within `seconds`, so the foot on the side of travel steps next. "
                "StrideSim 11-06: the forward step in flight when a walk turned into a strafe ran its whole swing "
                "while the body coasted past the other foot (strafe sags 9 -> 3 of 20). Game, PLAY-01 unarmed "
                "course strafe x3 each: strafe-left pelvis median 62/51/51 -> 64/62/57, the root's sideways pull "
                "0.17-0.36 -> 0.07-0.14; the full course's walking measures unchanged; DUEL-W01 passes. 0 = off",
                "phase 2: the user's play (play-2026-11-06-6)" },
            { "OPENMW_RAGDOLL_K2_TAP_PARRY_AIM", SwitchKind::Experiment, SwitchArea::Weapon, "1,60", 0.0f, 150.0f,
                "on, flick counts. 1 = a tapped parry's zone takes the mouse until its window ends (0.35 s from the "
                "press), not only while the button is down, and the view does not turn meanwhile (user 11-10: \"tap "
                "ONCE! + direction - parry\"); 0 = the zone follows the mouse only while the button is held. Flick "
                "counts: in a press's first 0.35 s this much mouse travel picks a zone, where the guard asks 150 "
                "(user 11-12: \"tolerance zones\"; of 33 single parry taps on 11-11/12, 21 stayed under 150 in the "
                "window, 12 under 60); a held guard's first 0.35 s reads the same way; 0 or 150 = the guard's 150",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_THRUST_KEEP", SwitchKind::Experiment, SwitchArea::Weapon, "1.05", 0.0f, 1.6f,
                "radians: the player's spear or staff wound up on the thrust line thrusts for any stroke within this "
                "of straight ahead (user 11-06: \"a thrust wind-up that stays a thrust\"; a push over 22.5 deg off "
                "straight became a shaft swing, 2-3 of 13 thrust wind-ups in the 18:19 session), where K2_WEAPON runs "
                "the wheel; 0 = the stroke's own line",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_THRUST_PLANT", SwitchKind::Experiment, SwitchArea::Legs, "1", 0.0f, 1.0f,
                "1 = a standing thrust keeps its feet (user bookmark 11-16: \"when I thrust and it moves the player "
                "slightly forward it tries to do a front step ... thrusting would have the character planted\"): the "
                "thrust's own shift of the body (HSC_THRUST_LUNGE) is taken out of the body's place and speed before "
                "Footfall's standing rules (still clock, capture point, spot step) and the standing catch judge a "
                "step; a shove, a sag or the keys step as before. PLANT-01 (six standing thrusts, feet that left "
                "their place by 12 u; 2 runs off, 4 on): the longsword (middle button) 6 of 6 thrusts off, 0 of 24 "
                "on; the spear (a click) 2 of 6 off, 0 of 24 on; step commands logged per six thrusts 10-11 and 6-7 off, 0 on (the "
                "standing catch went first, a 15-40 u step forward and one back); the pelvis still goes 7-9 u forward "
                "and the reach is the same (26, 36 u). SPEAR-01 x3 each: all 24 thrusts land on and off, 8 on / 3 off "
                "of them paid 12-14 instead of 21-26 (six of the eight in one run; not settled); 0 = the lunge reads "
                "as the body leaving its stance",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_TIP_SIDE", SwitchKind::Experiment, SwitchArea::Weapon, "0.2", 0.0f, 1.0f,
                "share: a weapon whose mesh centre lies within this share of its half length from the grip (a spear "
                "or staff held at its middle) points its tip along the hand frame's + long axis, Morrowind's weapon "
                "convention (weaponalignment weaponTipSign), instead of toward the centre's side: the steel spear "
                "(centre 1.3 below the grip of a 68.8 half length) held its butt forward (user bookmark 11-06: \"the "
                "spear is pointing downwards\", the support arm wrecked); 0 = the centre's side",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_TURN_STEP", SwitchKind::Experiment, SwitchArea::Legs, "14,0.3,0.3", 0.0f, 20.0f,
                "lift units, window s, least swing s: the turn step (user bookmark 11-16 21:11: \"the ragdoll legs "
                "drag ... they just slide to catch up to the chest\"). For `window` after the command (over 30 u/s) "
                "points more than 30 deg off where it pointed 0.1 s before, or leaves rest, under 300 u/s: a gait "
                "swing that starts lifts at least a `lift`-high arc and lasts at least `least`, a swing in its first "
                "half takes the arc too, and K2_STRIDE_TURN may not cut a swing shorter than `least` in all. The "
                "11-16 duel (2,144 swings): within 0.6 s of a change a swing lasted 0.39 s and lifted 7.0 of 11.8 u, "
                "21% stayed under 3 u (sideways 3.3 of 7.9, 47%); steady 0.58 s, 34 of 41 u, 2% - the swing clock "
                "runs on time alone and the leg follows its goal 0.1 s late. PLAY-01 duel footwork (a key change "
                "every 0.15-0.35 s) x3 each way at 14,0.3,0.3: change swings under 3 u 34-39% -> 13-17% unarmed, "
                "19-48% -> 0-4% warhammer, sideways 29-50% -> 0%; crossed feet 0-0.3% -> 0-3.9% of moving steps, the "
                "unarmed pelvis's low 58-60 -> 50-52 u, one turn_s of 0.72 s in the six runs on; the planted foot's "
                "creep unchanged. Lift 0 = off",
                "the user's play decides (in the hub 11-17, EXPERIMENTAL)" },
            { "OPENMW_RAGDOLL_K2_TWO_HAND_REACH", SwitchKind::Experiment, SwitchArea::Weapon, "0.8", 0.2f, 1.0f,
                "the arm's share a two-handed grip reaches with when the wheel measures the step-in toward a foe "
                "past the sweet reach (HSC_LUNGE sizes it), where K2_WEAPON runs the wheel; 1 = the whole arm (B6, "
                "REACH-01)",
                "phase 1: the user's play decides" },
            { "OPENMW_RAGDOLL_K2_TWO_HAND_TURN", SwitchKind::Experiment, SwitchArea::Weapon, "1,0", 0.0f, 2.0f,
                "wide, close: the share of a two-hander's own inertia across its length that its blade's turn is "
                "sized on (handdrive mTurnInertia), for a wide grip (spear, staff) and for the others. Without it "
                "the turn's torque is sized on the weapon's mass at its centre about the wrist: a steel spear is "
                "gripped 1.3 u from its centre, so almost none was asked (user 11-09: the spear \"feels kinda "
                "bad\"; that session's spear pointed 65 deg off its plan in the sweep's wind-up, FLOW-01 steel "
                "spear 30-37 deg p50 against the claymore's 13). FLOW-01 off / on, the blade off its plan p50 and "
                "p90, deg: steel spear 30-37 and 88-94 / 19-21 and 37-62 (3 runs on), iron spear 35 and 79 / 22 "
                "and 36, wooden staff 37 and 120 / 19-20 and 58-68; the steel spear's cuts in deg a step, backhand "
                "11-21 / 29-34, chop 13-14 / 28-32, forehand 21-28 / 36-37 (the claymore's 28-35). THRUST-01 iron "
                "spear: the tip's peak 850 / 626-647 u/s, travel 89 / 78-82 (the off-line whip gone; to watch). "
                "Close grips at 1, one run each: claymore 13 / 12, warhammer 20 / 17, battle axe 23 / 14 - left "
                "at 0 until the user asks. 0 = the estimate",
                "the user's play decides" },
            { "OPENMW_RAGDOLL_K2_UNEVEN", SwitchKind::Experiment, SwitchArea::Root, "8,0.5,1,1,15,40", 0.0f, 90.0f,
                "slope deg, hold s, lift, lead, sag from u, sag full u: ground counts as uneven while the slope under the body is over that "
                "many degrees, the slope stance is up or the capsule climbs or descends over 25 u/s, and for `hold` "
                "s after. With lift 1 the root keeps its whole weight support there at a walk (ROOT_WALK_SUPPORT's "
                "0.6 was measured on the flat course; on a climb in the user's 11-13 session the legs' load fell "
                "0.81 -> 0.39 body weights, lift and legs together under one, the pelvis 22-38 u under its target); "
                "there too, as the pelvis hangs from `sag from` to `sag full` under its target the lift gets the held "
                "strength back (a walk's budgeted strength, 0.3, scales the weight support and the upward cap as "
                "well), the horizontal hold keeping its budget - the whole strength at once kept the pelvis up (16 u "
                "under on steep climbs, 39 without) by carrying the body: legs' load 0.48 -> 0.25, no foot loaded on "
                "61% of steps, the feet trailing over 30 u on 27% of a descent's steps against 13%; "
                "with lead 1 the legs-lead push stays off there for the hold (its gate had none: 772 of 1228 "
                "switches came within 0.3 s of the last); slope 0 = off",
                "the slope, ramp and stairs cases and the user's play decide" },
            { "OPENMW_RAGDOLL_K2_UPRIGHT_NOT_FALLEN", SwitchKind::Experiment, SwitchArea::Reaction, "1", 0.0f, 1.0f,
                "1 = an upright pelvis more than the lying height (30 u) over its lower foot is never called Fallen. "
                "The lying verdict reads the pelvis' height over the support estimate's ground, and contacts above "
                "the feet (a weapon, another fighter) put that ground at the hips: a slowing, upright fighter went "
                "Bracing -> Fallen, its motors off, and took 4-6 s to stand again (user bookmarks 11-16 \"why did I "
                "fall there\", \"lets check that fall too\": both such; RAGDOLL_FALLEN_FROM_FEET 15 times in that 7 "
                "minute session, the torso 0.97-1.0 upright, the pelvis read 0-29 u high). 0 = as before",
                "RAGDOLL_FALLEN_FROM_FEET rows in play (none with it on); DUEL and CLASH cases" },
            { "OPENMW_RAGDOLL_K2_WEAPON_BOX_INERTIA", SwitchKind::Experiment, SwitchArea::Weapon, "1,0.036,1", 0.0f, 1.0f,
                "on, roll floor, hafts: 1 = a weapon's exact-mesh body (its convex hull) takes the mass properties of a solid box over its "
                "bounds, keeping its exact collision: the hull's own inertia is lower and tilted off the long axis "
                "(glass claymore 744/758 and a cross term -36 against 1083/1130/0) and the arm's holding springs are "
                "sized from it (the hand's 104 Hz against 133). FLOW-01 backhand, 3-5 runs: glass 26-28 and steel "
                "27-28 deg/step, against 16-24 and 22-26 with the hull's own (user 11-06: the glass claymore's "
                "\"right to left just doesn't work\"; the iron claymore 17-29 -> 26-31, the longsword 23 -> 26). "
                "Hafts (spears, staffs) 1 = them too: without the roll floor the iron spear's chop bent the wrist "
                "21-24 deg past its limit; with it, VARIANT-01 flags 3 of 18 haft variants (borderline) against 11 "
                "with the hull's own. "
                "The roll floor holds the inertia about the blade at least that share of the inertia across it: "
                "a thin blade barely resisted a roll and rang in the solver, never settling (VARIANT-01 11-07: "
                "the dai-katanas at 30-50% of the iron claymore, the tantos' chop 75% of the dagger's; with "
                "0.036, the iron claymore's own share, 92-98%); 0 = the hull's own",
                "until real weapon masses (K2-WEAPON step 7)" },
            { "OPENMW_RAGDOLL_K2_WEAPON_PACE", SwitchKind::Tunable, SwitchArea::Weapon, "1.5,1", 0.5f, 3.0f,
                "player, NPCs: the weapon's response - x the attack speed (wind-up, cut and return times, the "
                "route's speed and acceleration) and x the hand drive's spring frequencies (their caps x its "
                "square), where K2_WEAPON runs the wheel (user 11-04: weapons \"50% more responsive\", drag "
                "faster); 1 = play-2026-11-04-3",
                "" },
            { "OPENMW_RAGDOLL_K2_WINDUP_RADIUS", SwitchKind::Experiment, SwitchArea::Weapon, "450", 0.0f, 1200.0f,
                "mouse counts: the player's stance circle, every weapon (inside it a held click moves the weapon among "
                "guards to aim; past it the wind-up commits; user 11-08: \"make the windup activation radius ... a bit "
                "larger\", to aim a spear at a charger); 0 = the wheel's 300",
                "the user's play" },
            { "OPENMW_RAGDOLL_K2_WOUND_HEAL", SwitchKind::Experiment, SwitchArea::Reaction, "1,1", 0.0f, 24.0f,
                "on, full hours: wounds heal (user 11-17: Restore Health and rest heal them, \"maintaining/repairing "
                "the armor fixes them\", \"do not make this healing system too complicated\"; ragdollcore woundheal). "
                "A body's wounds - the limb damage that slows an arm, the marks on skin, the stains on clothing - "
                "follow its health: the share of the missing health that comes back (a spell, a potion, an altar, "
                "rest) is the share of the remaining wounds that goes, the oldest marks first, none left at full "
                "health. The marks on worn armour follow that armour's condition the same way (a hammer, a smith). "
                "While the pool stays full, what is left (bruises from blows that cost fatigue only, a scratch on a "
                "piece that took no wear) fades over `full hours` of game time; 0 hours = it stays. Not saved. "
                "0 = wounds never heal (only the hub's reset cleared them). HEAL-01 (2 runs): four cuts and 40 limb "
                "damage at 20% health, 2 and 20.0 at 60%, none at full health; three marks on a half-worn cuirass "
                "stay through that and go at full condition",
                "0.2: the user's play decides" },
            { "OPENMW_RAGDOLL_NPC_BODIES", SwitchKind::Tunable, SwitchArea::Lifecycle, "10,2000", 0.0f, 8000.0f,
                "count, radius: at most this many NPC bodies at once (the player's own not counted), given to "
                "NPCs within the radius (units) nearest first and released past 1.75x it, the farthest first when "
                "the count is lowered (ragdollcore npcpopulation; options page slider, live; user 11-04: raise the "
                "old fixed 10 / 2000); 0 = no NPC bodies",
                "" },
            { "OPENMW_RAGDOLL_PLAYER_BLOOD", SwitchKind::Experiment, SwitchArea::Reaction, "1", 0.0f, 1.0f,
                "a blow that takes the player's health spawns Morrowind's blood effect at the hit, as it does on "
                "everyone else (user 11-12: \"if they hit the player blood comes out like vs an enemy\"; the "
                "built-in hit script skips the player unless its own option, off by default, is ticked); read by "
                "omw/combat/local.lua through openmw.kinetic's playerBleeds(); 0 = only that option decides",
                "the user's play decides" },
        };

        bool parseField(std::string_view text, float& value)
        {
            while (!text.empty() && text.front() == ' ')
                text.remove_prefix(1);
            while (!text.empty() && text.back() == ' ')
                text.remove_suffix(1);
            if (text.empty())
                return false;
            const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
            return result.ec == std::errc() && result.ptr == text.data() + text.size() && std::isfinite(value);
        }
    }

    std::span<const SwitchSpec> switchRegistry()
    {
        return sSwitches;
    }

    const SwitchSpec* findSwitch(std::string_view name)
    {
        const auto found = std::lower_bound(std::begin(sSwitches), std::end(sSwitches), name,
            [](const SwitchSpec& spec, std::string_view key) { return spec.mName < key; });
        return found != std::end(sSwitches) && found->mName == name ? &*found : nullptr;
    }

    std::vector<std::string> checkSwitchRegistry()
    {
        std::vector<std::string> problems;
        std::size_t tunables = 0;
        for (std::size_t i = 0; i < std::size(sSwitches); ++i)
        {
            const SwitchSpec& spec = sSwitches[i];
            const std::string name(spec.mName);
            if (!spec.mName.starts_with("OPENMW_RAGDOLL_") && !spec.mName.starts_with("OPENMW_KINETIC_"))
                problems.push_back(name + ": not an OPENMW_RAGDOLL_ or OPENMW_KINETIC_ switch");
            if (i > 0 && !(sSwitches[i - 1].mName < spec.mName))
                problems.push_back(name + ": not sorted, or a duplicate");
            if (spec.mReason.empty())
                problems.push_back(name + ": no reason");
            if (spec.mKind == SwitchKind::Experiment && spec.mExpires.empty())
                problems.push_back(name + ": an experiment with no expiry");
            if (spec.mKind == SwitchKind::Tunable)
                ++tunables;
            if (!(spec.mMin <= spec.mMax))
                problems.push_back(name + ": empty range");
            std::string_view rest = spec.mDefault;
            while (!rest.empty())
            {
                const std::size_t comma = rest.find(',');
                const std::string_view field = rest.substr(0, comma);
                float value = 0.0f;
                if (!parseField(field, value))
                    problems.push_back(name + ": default field '" + std::string(field) + "' is not a number");
                else if (value < spec.mMin || value > spec.mMax)
                    problems.push_back(name + ": default field '" + std::string(field) + "' out of range");
                if (comma == std::string_view::npos)
                    break;
                rest.remove_prefix(comma + 1);
            }
            for (const PlayDefault& play : playDefaults())
                if (play.mName == spec.mName)
                    problems.push_back(name + ": also in playDefaults()");
        }
        if (tunables > sTunableBudget)
            problems.push_back(std::to_string(tunables) + " tunables, over the budget of "
                + std::to_string(sTunableBudget));
        return problems;
    }

    std::string_view switchAreaName(SwitchArea area)
    {
        switch (area)
        {
            case SwitchArea::Pipeline:
                return "pipeline";
            case SwitchArea::Weapon:
                return "weapon";
            case SwitchArea::Legs:
                return "legs";
            case SwitchArea::Upright:
                return "upright";
            case SwitchArea::Reaction:
                return "reaction";
            case SwitchArea::Root:
                return "root";
            case SwitchArea::Lifecycle:
                return "lifecycle";
            case SwitchArea::Ai:
                return "ai";
            case SwitchArea::Hud:
                return "hud";
        }
        return "unknown";
    }
}

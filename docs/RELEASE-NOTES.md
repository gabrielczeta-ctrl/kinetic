# KINETIC TECH-DEMO RELEASE 0.1 (2026-11-02)

The first public release: a tech demo of physical combat in Morrowind.

Physical melee combat for Morrowind, built on OpenMW.

## What it is

Every fighter - you and the NPCs - is a physical body. Characters walk on
simulated legs, sway, step to keep their balance, stumble when struck, fall
and get up again. Your weapon is a physical object you steer with the mouse:
hold the left button and move to guard, wind up and cut along your stroke;
click for a full physical swing at the brass reticle. A blow lands only where
the weapon actually touches, and how fast and how squarely it lands decides
whether it bites, glances or stops. Morrowind's own rules still set every
number: your skill steadies the blade, Strength and Agility shape blows and
balance, armour protects the slot it covers, fatigue wears you down, and a
heavy staggering blow can knock a fighter down by the game's own knockdown
odds.

## Highlights

- Physical bodies for the player and every humanoid NPC, with real catch
  steps, stumbles, falls, drop rolls and get-ups.
- Mouse-steered weapons with weight and momentum; one-handers, two-handers
  (both hands on the grip), spears and staffs, daggers and fists.
- Spears swing like staffs and thrust with a lunge of the whole body; the
  middle button thrusts with any weapon.
- Blades cut with the edge, stab with the point and bruise with the flat;
  axes cut with the head; maces, staffs and fists bruise. Even a slow cut
  pays a little, with sound and blood.
- Skill shows: below weapon skill 50 the blade wanders and each attack waits
  for part of its recovery; from 50 attacks chain and carry their momentum.
- Guard with the right button (tap to parry), double right click to shove;
  attack straight out of a held guard.
- Combos: a cut that winds up where the last one ended flows on faster and
  for less fatigue; a click right after a parry you won is a quick riposte;
  bare fists go jab, cross, jab.
- Contact decides hits; wounds and blood where they landed; hit reactions
  from a flinch to a knockdown.
- NPCs fight with the same bodies and weapons: they read your guard, keep
  their distance, feint and pace their attacks.
- The brass reticle: aim, guard side, wind-up and cooldown at a glance; an
  optional swing-path trail (Options > Scripts > KINETIC > Swing path).
- Corpses fall where the body died and stay there; archers keep their bow
  animations; a blow to the head always cries out.
- Smooth at any frame rate.
- Your mods: `mods.cfg` lists your normal OpenMW setup, switched off until
  you enable each one (tested on the base game only - enable a few at a
  time).

## Install

1. Unpack the zip anywhere.
2. Run `KINETIC.cmd`; the first time it asks for your Morrowind "Data Files"
   folder (Morrowind GOTY recommended; Tribunal and Bloodmoon are used when
   present).
3. Options: in game, Options > Scripts > KINETIC.

The folder is self-contained (settings in `config\`, saves in `userdata\`)
and does not touch an existing OpenMW or Morrowind install. KINETIC is built
on OpenMW 0.52 (development): saves from OpenMW 0.51 load; saves made in
KINETIC do not load back in 0.51.

## Known limits

- Skeletons, dremora, golden saints and liches get the KINETIC body and
  fight like NPCs (new, experimental); other creatures are still animated.
- Swimming is vanilla; the body becomes physical again on land.
- A mouse drag steers the right fist only (clicks punch with both).
- Mods that change melee combat (attack, block, combat AI) will fight
  KINETIC.

## Licence and source

KINETIC is free software under the GNU GPL version 3, a modified OpenMW
(upstream 262d1ba0d6 with the Jolt physics merge request !3919). It is not an
official OpenMW release and is not endorsed by the OpenMW project. It
contains no Morrowind data. The package carries the licence, every
third-party component's notice and the source link (`SOURCE.txt`).

Made with AI assistance (Claude).

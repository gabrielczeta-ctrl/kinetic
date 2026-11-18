# KINETIC TECH-DEMO RELEASE 0.2

The second tech demo of physical combat in Morrowind: a faster, more
physical fight.

## New in 0.2

- **Faster, freer swings.** Weapons respond about 50% faster. A held click
  never turns the camera any more: move the mouse and the swing winds up
  from the way you moved. Keep the button held and drag away again to chain
  the next cut at once (chop, hack, slash). Daggers thrust almost as fast as
  you can click.
- **Weight behind the blow.** Moving into a cut winds it up faster, and a
  running or jumping blow carries part of your body's weight: it pushes
  harder and cuts deeper. The chest turns through sideways cuts, and
  one-handed cuts reach out with the arm.
- **Blades that land.** An edge bites, holds a moment or cleaves through; a
  blade that hits flesh presses on and shoves the body instead of bouncing
  off. Plate still rings back from a mace or the flat of a blade.
- **Clashes.** Blades slide past each other unless someone is guarding; a
  guarded clash is decided by grip, leverage, Strength and fatigue; a clash
  with no parry throws the struck hand aside. No more lockout timers after
  a clash or a hit.
- **The right button does all the defending.** Hold to guard (the block
  steers to meet the blade), tap to parry (a short flick picks its side),
  double-click to shove - the arms throw the shove. Middle mouse is free.
- **Spears and staffs.** Hold the button and the spear comes point-forward
  with the mouse moving its point; a click thrusts on the spot and keeps
  your feet planted; drag for the haft's own sweeps and chops (the chop
  winds up behind your back). A thrust pays the point's damage and leaves
  blood on the point. A staff now handles exactly like a spear, with its
  own damage, reach and blunt blows.
- **Bare fists, two buttons.** Left button = right fist, right button =
  left fist. Click to jab at the crosshair (aim low for the body); hold,
  wind up and stroke the mouse fast the other way for a hook, uppercut,
  overhand or cross; a punch lunges at a foe just out of reach. Both
  buttons guard; bare-handed, a hand throws its open palm at an incoming
  weapon and knocks it aside. Fists are the newest part and still changing.
- **Stab and pin, chop and slash (experimental).** A point goes into the
  body, a square edge lodges, a drawn cut passes through.
- **Smarter enemies.** NPCs attack in combos of two or three cuts, block
  where your cut is coming from, raise their blocks in time, keep their
  distance instead of crowding you, and brake instead of charging through
  you. Calm NPCs walk and idle as Morrowind draws them and turn physical
  when it matters.
- **Movement.** Steadier walking and running, feet that plant flat and
  step (not slide) when you change direction, a jump that leaps, slopes,
  stairs and rough ground that no longer trip the legs, and a body that no
  longer lags far behind at speed and snaps after it. The camera stays
  steady below 60 fps.
- **Deaths.** A body goes limp facing the way it fell, with no flicker of
  the death animation and no snapping wrist.
- **You bleed** where you are hit, **and you heal**: wounds close as your
  health comes back (a spell, a potion, rest), and repairing your armour
  takes the scratches and dents off it.
- **The reticle** shows where your swing will land while you wind it up,
  which way your weight is shifting, and (optionally) your mouse's path;
  its four wedges warn together when you are about to fall.
- **Bows and crossbows** take their ammunition from your pack by themselves.
- **Fixes.** Lockpicks and probes no longer freeze you; corpses stay where
  they died across loads; the Knockdowns option saves; downed foes can be
  reached.

The new combat rules are experiments, each behind its own switch; a line
`OPENMW_RAGDOLL_<NAME> = 0` in `ragdoll\ragdoll.cfg` turns one off (for
example `K2_LODGE` for stab and pin, `K2_TURN_STEP` for the turning step,
`K2_STAFF_AS_SPEAR` for the staff, `K2_PALM_PARRY` for the palm;
`OPENMW_RAGDOLL_K2_LOCKOUT = 1` brings the old lockouts back).

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
  steps, stumbles, falls and get-ups.
- Mouse-steered weapons with weight and momentum; one-handers, two-handers
  (both hands on the grip), spears and staffs, daggers and fists.
- Spears swing like staffs and thrust with a lunge of the whole body.
- Blades cut with the edge, stab with the point and bruise with the flat;
  axes cut with the head; maces, staffs and fists bruise. Even a slow cut
  pays a little, with sound and blood.
- Skill shows: below weapon skill 50 the blade wanders and each attack waits
  for part of its recovery; from 50 attacks chain and carry their momentum.
- Guard with the right button (tap to parry), double right click to shove;
  attack straight out of a held guard.
- Combos: a cut that winds up where the last one ended flows on faster and
  for less fatigue; a click right after a parry you won is a quick riposte.
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

- Weapon handling is about 90% complete.
- Bare fists need more work.
- Rolls are not ready: their animations are unfinished.
- Movement can still go wrong now and then, most of all walking up or down
  stairs diagonally.
- Skeletons, dremora, golden saints and liches get the KINETIC body and
  fight like NPCs (new, experimental); other creatures are still animated.
- Swimming is vanilla; the body becomes physical again on land.
- Mods that change melee combat (attack, block, combat AI) will fight
  KINETIC.

## Looking ahead

We are looking for a way to support dismemberment, and for ways to bring
more of the game under the same physics: the other creatures, rolls, better
fists. Other creators are invited to collaborate: open an issue on the
repository or write on the mod's page.

## Licence and source

KINETIC is free software under the GNU GPL version 3, a modified OpenMW
(upstream 262d1ba0d6 with the Jolt physics merge request !3919). It is not an
official OpenMW release and is not endorsed by the OpenMW project. It
contains no Morrowind data. The package carries the licence, every
third-party component's notice and the source link (`SOURCE.txt`).

Made with AI assistance (Claude).

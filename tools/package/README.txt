KINETIC TECH-DEMO RELEASE 0.2 - physical combat for Morrowind
=============================================================

Bodies and weapons are physical: you swing the weapon with the mouse, blows
land where they hit and push what they hit, bodies stumble, step, fall and
get up. NPCs fight by the same rules.

KINETIC is a modified OpenMW (free software, GPL-3.0). It is not an official
OpenMW release and is not endorsed by the OpenMW project. It contains no
Morrowind game data: you need your own copy of the game.

Requirements
- Windows 10/11, 64-bit.
- Morrowind (GOTY recommended): the "Data Files" folder with Morrowind.esm.
  Tribunal and Bloodmoon are used when present.

Start
- Run KINETIC.cmd. The first time it asks for your "Data Files" folder and
  remembers it (morrowind-data.txt; delete that file to choose again).
- This folder is self-contained: settings go to config\, saves and logs to
  userdata\. It does not touch an existing OpenMW or Morrowind install.
- Saves: KINETIC is built on a newer OpenMW (0.52 development) than the
  current release (0.51). Saves from 0.51 load in KINETIC; saves made in
  KINETIC do not load in 0.51.

Controls
- Left click: attack (a full physical swing). Hold the left button and move
  the mouse: the swing winds up from the way you moved and steers the
  weapon; a fast stroke is a committed swing. While the button is held the
  mouse never turns the view. Keep holding and drag away again to chain the
  next cut at once (chop, hack, slash).
- Moving into a cut winds it up faster and puts your weight behind it: a
  running or jumping blow pushes harder and cuts deeper.
- Spear and staff: hold the button and the point comes forward, the mouse
  moving it; a click thrusts on the spot; drag for the haft's sweeps and
  chops. A staff handles like a spear, with its own damage and blunt blows.
- Right button: hold = guard (the block steers to meet the blade), tap =
  parry (a short flick picks its side), double click = shove.
- Combos: a cut that winds up where the last one ended comes faster and
  cheaper; a click right after a parry you won is a quick riposte. Daggers
  thrust almost as fast as you can click.
- Bare fists: left button = right fist, right button = left fist. Click to
  jab at the crosshair (low for the body); hold, wind up and stroke the
  mouse fast the other way for a hook, uppercut, overhand or cross. Both
  buttons guard; bare-handed, a palm knocks an incoming weapon aside.
- The brass diamond at the centre is your aim; its bar refills when you can
  attack again.
- Your weapon skill, Strength, Agility and fatigue decide how steady the blade
  is, how hard a blow lands and how well you keep your feet, as in vanilla.
  A staggering blow can knock a body down by Morrowind's own knockdown odds
  (Options: Knockdowns - Vanilla, Rare or Off).

Mods
- mods.cfg (made on the first run) lists the data= and content= lines of your
  normal OpenMW setup, switched off with a #. Remove the # in front of a mod's
  data= and content= lines to load it. KINETIC was tested on the base game
  only: enable mods a few at a time. Mods that change melee combat (attack,
  block, combat AI) will fight KINETIC.

Options
- In game: Options > Scripts > KINETIC. Changes apply at the next start.
- Everything else: ragdoll\ragdoll.cfg (one "NAME = value" per line). The
  options page wins over that file; an environment variable wins over both.

Known limits
- Weapon handling is about 90% complete.
- Bare fists need more work.
- Rolls are not ready: their animations are unfinished.
- Movement can still go wrong now and then, most of all walking up or down
  stairs diagonally.
- Skeletons, dremora, golden saints and liches get the KINETIC body and
  fight like NPCs (new, experimental); other creatures are still animated.
- Swimming falls back to vanilla; it becomes physical again on land.
- New in 0.2 and still being tuned: bare fists (the newest part), stab and
  pin, the staff handled as a spear, no lockout after clashes, blades that
  press into bodies instead of bouncing. Each has a switch in
  ragdoll\ragdoll.cfg (see the release notes).

Licences: LICENSE.txt (GPL-3.0), THIRD-PARTY-NOTICES.txt. Source: SOURCE.txt.

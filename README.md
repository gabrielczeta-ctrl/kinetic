KINETIC
=======

**Physical melee combat for Morrowind, built on OpenMW.**

*Your body. Your blade. Every swing is real.*

KINETIC replaces Morrowind's dice-roll melee with physics. Every fighter, you
and every humanoid NPC, is a powered physical body that walks on simulated
legs, sways, steps to keep its balance, stumbles when struck, falls and gets
back up. Your weapon is a real object in your hands. You steer it with the
mouse, it has weight and momentum, and a blow lands only where the steel
actually touches.

It is still Morrowind. Your weapon skill decides how steady the blade is.
Strength and Agility shape your blows and your footing, armour protects the
slot it covers, fatigue wears you down, and a heavy blow can knock a fighter
flat by the game's own knockdown odds. Physics decides *whether and where* a
blow lands; Morrowind's formulas decide *how much it matters*.

Features
--------

- **Physical bodies for everyone.** Catch steps when pushed, stumbles, drop
  rolls off ledges, falls that are earned, get-ups that happen in the world.
- **A weapon you actually hold.** Hold the left button and move the mouse to
  guard, wind up and cut along your own stroke. Click for a full physical
  swing at the brass reticle. The middle button thrusts.
- **Contact decides everything.** The speed and angle at the point of contact
  decide whether a blow bites, glances or stops. Blades cut with the edge,
  stab with the point and bruise with the flat.
- **Skill feels like skill.** A novice's blade wanders and waits for its
  recovery. From weapon skill 50 attacks chain and carry their momentum into
  the next cut.
- **Combos.** A cut that winds up where the last one ended flows on faster
  and for less fatigue; a click right after a parry you won is a quick
  riposte; bare fists go jab, cross, jab.
- **Guard, parry, shove.** The right button guards (tap to parry). Two quick
  right clicks shove. You can attack straight out of a held guard.
- **Reactions with weight.** A flinch, a stagger with catch steps or a
  knockdown, decided by where the blow landed, how hard it was, and whether
  the body could still find its feet.
- **Wounds where they landed.** Cuts and punctures leave marks and blood; a
  wounded limb works worse.
- **NPCs fight by the same rules.** They use the same bodies and weapons. They
  read your guard, keep their distance, feint and pace their attacks.
- **Every melee weapon type.** One-handers, two-handers with both hands on
  the grip, spears with a lunging thrust, staffs, daggers and fists. Shields
  have their own physical body.
- **Smooth at any frame rate.** Physics steps at 60 Hz and the bodies are drawn
  smoothly between steps at 120 fps or uncapped.

Play
----

Download the release zip from
[Releases](https://github.com/gabrielczeta-ctrl/kinetic/releases) (or
Nexus Mods), unpack it anywhere and run `KINETIC.cmd`. You need
your own Morrowind (the "Data Files" folder; GOTY recommended). You don't need
an OpenMW install, and KINETIC doesn't touch an existing one. The package
README explains the controls, the options page (Options > Scripts > KINETIC),
loading your other mods (`mods.cfg`) and the known limits.

| Input | Action |
|---|---|
| Left click | Attack: a full physical swing at the reticle |
| Hold left + move | Steer the weapon; a fast stroke is a committed swing |
| Middle button | Thrust |
| Right button | Guard (tap to parry) |
| Two quick right clicks | Shove |

This is a tech demo (TECH-DEMO RELEASE 0.1): the bodies and the combat work,
and plenty is still rough. Skeletons, dremora, golden saints and liches get
the KINETIC body (new, experimental); other creatures are still animated,
swimming is vanilla, and a mouse drag steers the right fist only. Mods that
change melee combat (attack, block, combat AI) will conflict with KINETIC.

How it works
------------

- A 20-part powered ragdoll held to the character's capsule by a **soft
  root**, so the body can lag, sway and be pushed off its feet.
- **Footfall**, a gait controller that follows reference walk and run loops
  and places each step where the body needs it.
- **PGC**, a capture-point balance controller that plans 1-3 catch steps,
  landings, teetering and falls.
- A **hand controller** that plans guards, wind-ups, cuts and thrusts on a
  wheel around the body and drives the weapon as a physical object.
- A **contact system** that prices every blow with Morrowind's damage, hit
  roll, armour and knockdown formulas.

Every change is checked by scripted test cases (walking, balance, movement,
combat, duels) before it reaches a play build.

KINETIC is a modified OpenMW: upstream master 262d1ba0d6 (2026-08-31) with the
Jolt physics merge request (!3919, head 953ff1c32b). It is not an official
OpenMW release and is not endorsed by the OpenMW project.

Build
-----

Windows, Visual Studio 2022, OpenMW's dependency bundle (vcpkg); CMake fetches
Jolt, RecastNavigation and yaml-cpp.

    cmake -S . -B build-release -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake ^
        -DBUILD_OPENCS=OFF -DBUILD_LAUNCHER=OFF ^
        -DOPENMW_RAGDOLL_DEV_API=ON -DOPENMW_RAGDOLL_TEST_LUA=OFF
    cmake --build build-release --config Release --target openmw
    powershell -File tools/package-kinetic.ps1 -Release -Zip

The physics core is a library with its own tests (`ragdollcore-tests`).

Documentation
-------------

- [docs/RELEASE-NOTES.md](docs/RELEASE-NOTES.md) - what the release contains.

Licence: GPL-3.0, like OpenMW (see LICENSE). The release package adds the
third-party notices and the source link. Made with AI assistance (Claude).

---

OpenMW
======

OpenMW is an open-source open-world RPG game engine that supports playing Morrowind by Bethesda Softworks. You need to own the game for OpenMW to play Morrowind.

OpenMW also comes with OpenMW-CS, a replacement for Bethesda's Construction Set.

* Version: 0.52.0
* License: GPLv3 (see [LICENSE](https://gitlab.com/OpenMW/openmw/-/raw/master/LICENSE) for more information)
* Website: https://www.openmw.org
* IRC: #openmw on irc.libera.chat
* Discord: https://discord.gg/bWuqq2e


Font Licenses:
* DejaVuLGCSansMono.ttf: custom (see [files/data/fonts/DejaVuFontLicense.txt](https://gitlab.com/OpenMW/openmw/-/raw/master/files/data/fonts/DejaVuFontLicense.txt) for more information)
* DemonicLetters.ttf: SIL Open Font License (see [files/data/fonts/DemonicLettersFontLicense.txt](https://gitlab.com/OpenMW/openmw/-/raw/master/files/data/fonts/DemonicLettersFontLicense.txt) for more information)
* MysticCards.ttf: SIL Open Font License (see [files/data/fonts/MysticCardsFontLicense.txt](https://gitlab.com/OpenMW/openmw/-/raw/master/files/data/fonts/MysticCardsFontLicense.txt) for more information)

Current Status
--------------

The main quests in Morrowind, Tribunal and Bloodmoon are all completable. Some issues with side quests are to be expected (but rare). Check the [bug tracker](https://gitlab.com/OpenMW/openmw/-/issues/?milestone_title=openmw-1.0) for a list of issues we need to resolve before the "1.0" release. Even before the "1.0" release, however, OpenMW boasts some new [features](https://wiki.openmw.org/index.php?title=Features), such as improved graphics and user interfaces.

Pre-existing modifications created for the original Morrowind engine can be hit-and-miss. The OpenMW script compiler performs more thorough error-checking than Morrowind does, meaning that a mod created for Morrowind may not necessarily run in OpenMW. Some mods also rely on quirky behaviour or engine bugs in order to work. We are considering such compatibility issues on a case-by-case basis - in some cases adding a workaround to OpenMW may be feasible, in other cases fixing the mod will be the only option. If you know of any mods that work or don't work, feel free to add them to the [Mod status](https://wiki.openmw.org/index.php?title=Mod_status) wiki page.

Getting Started
---------------

* [Official forums](https://forum.openmw.org/)
* [Installation instructions](https://openmw.readthedocs.io/en/latest/manuals/installation/index.html)
* [Build from source](https://wiki.openmw.org/index.php?title=Development_Environment_Setup)
* [Testing the game](https://wiki.openmw.org/index.php?title=Testing)
* [How to contribute](https://wiki.openmw.org/index.php?title=Contribution_Wanted)
* [Report a bug](https://gitlab.com/OpenMW/openmw/issues) - read the [guidelines](https://wiki.openmw.org/index.php?title=Bug_Reporting_Guidelines) before submitting your first bug!
* [Known issues](https://gitlab.com/OpenMW/openmw/issues?label_name%5B%5D=Bug)

The data path
-------------

The data path tells OpenMW where to find your Morrowind files. If you run the launcher, OpenMW should be able to pick up the location of these files on its own, if both Morrowind and OpenMW are installed properly (installing Morrowind under WINE is considered a proper install).

Command line options
--------------------

    Syntax: openmw <options>
    Allowed options:
      --config arg                          additional config directories
      --replace arg                         settings where the values from the
                                            current source should replace those
                                            from lower-priority sources instead of
                                            being appended
      --user-data arg                       set user data directory (used for
                                            saves, screenshots, etc)
      --resources arg (=resources)          set resources directory
      --help                                print help message
      --version                             print version information and quit
      --data arg (=data)                    set data directories (later directories
                                            have higher priority)
      --data-local arg                      set local data directory (highest
                                            priority)
      --fallback-archive arg (=fallback-archive)
                                            set fallback BSA archives (later
                                            archives have higher priority)
      --start arg                           set initial cell
      --content arg                         content file(s): esm/esp, or
                                            omwgame/omwaddon/omwscripts
      --groundcover arg                     groundcover content file(s): esm/esp,
                                            or omwgame/omwaddon
      --no-sound [=arg(=1)] (=0)            disable all sounds
      --script-all [=arg(=1)] (=0)          compile all scripts (excluding dialogue
                                            scripts) at startup
      --script-all-dialogue [=arg(=1)] (=0) compile all dialogue scripts at startup
      --script-console [=arg(=1)] (=0)      enable console-only script
                                            functionality
      --script-run arg                      select a file containing a list of
                                            console commands that is executed on
                                            startup
      --script-warn [=arg(=1)] (=1)         handling of warnings when compiling
                                            scripts
                                            0 - ignore warnings
                                            1 - show warnings but consider script as
                                            correctly compiled anyway
                                            2 - treat warnings as errors
      --load-savegame arg                   load a save game file on game startup
                                            (specify an absolute filename or a
                                            filename relative to the current
                                            working directory)
      --skip-menu [=arg(=1)] (=0)           skip main menu on game startup
      --new-game [=arg(=1)] (=0)            run new game sequence (ignored if
                                            skip-menu=0)
      --encoding arg (=win1252)             Character encoding used in OpenMW game
                                            messages:

                                            win1250 - Central and Eastern European
                                            such as Polish, Czech, Slovak,
                                            Hungarian, Slovene, Bosnian, Croatian,
                                            Serbian (Latin script), Romanian and
                                            Albanian languages

                                            win1251 - Cyrillic alphabet such as
                                            Russian, Bulgarian, Serbian Cyrillic
                                            and other languages

                                            win1252 - Western European (Latin)
                                            alphabet, used by default
      --fallback arg                        fallback values
      --no-grab [=arg(=1)] (=0)             Don't grab mouse cursor
      --export-fonts [=arg(=1)] (=0)        Export Morrowind .fnt fonts to PNG
                                            image and XML file in current directory
      --activate-dist arg (=-1)             activation distance override
      --random-seed arg (=<impl defined>)   seed value for random number generator

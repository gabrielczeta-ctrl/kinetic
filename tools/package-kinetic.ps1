<#
Builds the KINETIC portable package (TECH-DEMO RELEASE 0.1): one folder that runs on its own
over Morrowind's Data Files.

  openmw.exe + DLLs + osgPlugins + resources   from the engine build and the
                                               pinned vcpkg bundle
  ragdoll\                                     profile, settings, gaits (play defaults
                                               appended to ragdoll.cfg)
  data\                                        kinetic.omwscripts (Options > Scripts > KINETIC)
  KINETIC.cmd, README.txt

usage: powershell -File tools/package-kinetic.ps1 [-Engine path\to\openmw.exe] [-Out dir] [-Zip]
         [-PlayLauncher Run-Game-MotionBricks.cmd] [-Record]
-PlayLauncher: the play defaults are that development launcher's switches
(its "set OPENMW_RAGDOLL_..." lines without file paths or traces), so the
package plays like the builds the user tests (10-30: a normal playthrough).
Rebuilding over an existing package keeps its saves and settings (config\,
userdata\, morrowind-data.txt, kinetic-switches.cfg).
#>
param(
    [string]$Engine = '',
    [string]$Out = '',
    [switch]$Zip,
    [switch]$Release,
    [string]$PlayLauncher = '',
    # Record every session and bookmark with B (development packages only).
    [switch]$Record,
    # Where the source of this build is published (SOURCE.txt). Default: the
    # repository's "public" remote (the GitHub fork of OpenMW).
    [string]$SourceUrl = '',
    # The published commit of this source (tools/public-branch.ps1): the public
    # branch drops the private development notes, so its commit differs from
    # HEAD while the shipped files are the same. Default: HEAD.
    [string]$SourceCommit = 'HEAD'
)
$version = 'TECH-DEMO RELEASE 0.1'
$ErrorActionPreference = 'Stop'
$source = Split-Path -Parent $PSScriptRoot
$root = Split-Path -Parent $source
if (!$Engine) {
    $Engine = if ($Release) { Join-Path $root 'build-release\Release\openmw.exe' }
        else { Join-Path $root 'build-footfall\RelWithDebInfo\openmw.exe' }
}
if ($Record -and $Release) { throw '-Record needs a development engine; the release build has no recorder.' }
$binary = Split-Path -Parent $Engine
$commit = (& git -C $source rev-parse --short=10 $SourceCommit).Trim()
if (!$Out) { $Out = Join-Path $root ("dist\KINETIC-tech-demo-0.1-" + $commit + $(if ($Release) { '' } else { '-dev' })) }
$manifest = Get-Content -Raw (Join-Path $source 'docs\ragdoll\version-manifest.json') | ConvertFrom-Json
$deps = Join-Path $root $manifest.dependencies.windows_bundle.relative_path
$depsBin = Join-Path $deps 'installed\x64-windows\bin'
if (!(Test-Path $Engine)) { throw "Engine not found: $Engine" }
if (!(Test-Path $depsBin)) { throw "Dependency bundle not found: $depsBin" }

# A rebuild keeps the player's saves and settings.
$kept = @('config', 'userdata', 'morrowind-data.txt', 'kinetic-switches.cfg', 'mods.cfg')
$stash = "$Out.keep"
# A stash left by an interrupted rebuild holds the saves: it is restored,
# never deleted (10-30: the folder was locked, the rebuild stopped with the
# saves stashed, and a rerun would have removed them).
if (Test-Path $Out) {
    New-Item -ItemType Directory -Force -Path $stash | Out-Null
    foreach ($name in $kept) {
        $path = Join-Path $Out $name
        if ((Test-Path $path) -and !(Test-Path (Join-Path $stash $name))) {
            Move-Item -LiteralPath $path -Destination (Join-Path $stash $name)
        }
    }
    # Its contents, not the folder: an Explorer window on it locks the folder.
    Get-ChildItem -LiteralPath $Out -Force | Where-Object { $kept -notcontains $_.Name } |
        Remove-Item -Recurse -Force
}
New-Item -ItemType Directory -Force -Path $Out | Out-Null
if (Test-Path $stash) {
    Get-ChildItem -LiteralPath $stash | Move-Item -Destination $Out
    Remove-Item -Recurse -Force $stash
}

# Engine, its resources and the runtime libraries.
Copy-Item -LiteralPath $Engine -Destination (Join-Path $Out 'openmw.exe')
Copy-Item -Recurse -LiteralPath (Join-Path $binary 'resources') -Destination $Out
foreach ($name in @('defaults.bin', 'defaults-cs.bin', 'gamecontrollerdb.txt')) {
    $path = Join-Path $binary $name
    if (Test-Path $path) { Copy-Item -LiteralPath $path -Destination $Out }
}
# The global config: resource paths and the vanilla fallback values, no user paths.
Copy-Item -LiteralPath (Join-Path $binary 'openmw.cfg.install') -Destination (Join-Path $Out 'openmw.cfg')
Get-ChildItem -LiteralPath $binary -Filter *.dll | Copy-Item -Destination $Out
Get-ChildItem -LiteralPath $depsBin -Filter *.dll | Where-Object { !(Test-Path (Join-Path $Out $_.Name)) } |
    Copy-Item -Destination $Out
Get-ChildItem -LiteralPath (Join-Path $depsBin 'Release') -Filter *.dll -ErrorAction SilentlyContinue |
    Where-Object { !(Test-Path (Join-Path $Out $_.Name)) } | Copy-Item -Destination $Out
$plugins = Get-ChildItem -LiteralPath $depsBin -Directory -Filter 'osgPlugins-*' | Select-Object -First 1
if ($plugins) { Copy-Item -Recurse -LiteralPath $plugins.FullName -Destination $Out }

# Ragdoll profile and settings; the play defaults (what the development
# launcher sets) go into the settings file, so the in-game options can change
# them (an environment variable would win over the options).
$ragdoll = Join-Path $Out 'ragdoll'
Copy-Item -Recurse -LiteralPath (Join-Path $source 'files\data\ragdoll') -Destination $ragdoll
# The debug ghost's reference loops are Mixamo-derived (release audit 10-06):
# never shipped. The game runs without them.
Remove-Item -LiteralPath (Join-Path $ragdoll 'reference-loops.txt') -ErrorAction SilentlyContinue
$defaults = @'

# --- KINETIC play defaults (package-kinetic.ps1) ------------------------------
OPENMW_RAGDOLL_POWER_PLAYER = 1
OPENMW_RAGDOLL_POWER_NPCS = 1
OPENMW_RAGDOLL_FOOTFALL = 1
OPENMW_RAGDOLL_CATCH_ON_HIT = 1
OPENMW_RAGDOLL_PGC = 1,1,1,1
OPENMW_RAGDOLL_PGC_BRACE = 1
'@
if ($PlayLauncher) {
    $lines = @('', "# --- play defaults from $(Split-Path -Leaf $PlayLauncher) (package-kinetic.ps1) ---")
    foreach ($line in Get-Content -LiteralPath $PlayLauncher) {
        if ($line.Trim() -match '^set "?(OPENMW_RAGDOLL_[A-Z0-9_]+)=([^"]*)"?\s*$' -and $Matches[2] -notmatch '%~dp0' `
            -and $Matches[1] -notmatch '_TRACE$') {
            $lines += "$($Matches[1]) = $($Matches[2])"
        }
    }
    # The first value of a key wins (loadRagdollSettingsFile), so these go
    # before the file's own entries, as the launcher's environment did.
    $cfg = Join-Path $ragdoll 'ragdoll.cfg'
    $text = ($lines -join "`n") + "`n`n" + (Get-Content -Raw -LiteralPath $cfg)
    [IO.File]::WriteAllText($cfg, $text, [Text.UTF8Encoding]::new($false))
}
Add-Content -LiteralPath (Join-Path $ragdoll 'ragdoll.cfg') -Value $defaults -Encoding UTF8

# The options page.
$data = Join-Path $Out 'data'
New-Item -ItemType Directory -Force -Path $data | Out-Null
$manual = Join-Path $source 'scripts\data\ragdoll_manual'
Copy-Item -LiteralPath (Join-Path $manual 'kinetic_settings.lua') -Destination $data
Copy-Item -LiteralPath (Join-Path $manual 'kinetic_settings_menu.lua') -Destination $data
# Stranded NPCs go home (build 59 bookmark 18).
Copy-Item -LiteralPath (Join-Path $manual 'kinetic_stranded.lua') -Destination $data
# Other combat mods (COMPAT-01): Mercy's melee fights go to KINETIC's duel AI.
Copy-Item -LiteralPath (Join-Path $manual 'kinetic_compat_actor.lua') -Destination $data
# The brass reticle (user 10-30: the crosshair in a normal playthrough): its
# feed, its player script, the reticle itself and its textures.
Copy-Item -LiteralPath (Join-Path $manual 'kinetic_hud_global.lua') -Destination $data
Copy-Item -LiteralPath (Join-Path $manual 'kinetic_hud_player.lua') -Destination $data
Copy-Item -LiteralPath (Join-Path $source 'scripts\data\ragdoll_hub\kinetic_reticle.lua') -Destination $data
New-Item -ItemType Directory -Force -Path (Join-Path $data 'textures') | Out-Null
Get-ChildItem -LiteralPath (Join-Path $manual 'textures') -Filter 'kinetic_brass_*.png' |
    Copy-Item -Destination (Join-Path $data 'textures')
Copy-Item -Recurse -LiteralPath (Join-Path $manual 'l10n') -Destination $data
$scripts = "MENU: kinetic_settings_menu.lua`nPLAYER: kinetic_settings.lua`nGLOBAL: kinetic_stranded.lua`nGLOBAL: kinetic_hud_global.lua`nPLAYER: kinetic_hud_player.lua`nNPC: kinetic_compat_actor.lua`n"
# -Record (user 10-31: bookmarks in the playthrough): every session is recorded
# into <workspace>\runtime-GAME-manual-playthrough-*, like the hub's, and B
# bookmarks a moment with a note. Needs a development engine (the recorder's Lua
# package is OPENMW_RAGDOLL_TEST_LUA only).
if ($Record) {
    Copy-Item -LiteralPath (Join-Path $manual 'kinetic_bookmark.lua') -Destination $data
    $scripts += "PLAYER: kinetic_bookmark.lua`n"
    [IO.File]::WriteAllText((Join-Path $Out 'recordings.txt'), "$root`r`n", [Text.Encoding]::ASCII)
}
else { Remove-Item -LiteralPath (Join-Path $Out 'recordings.txt') -ErrorAction SilentlyContinue }
[IO.File]::WriteAllText((Join-Path $data 'kinetic.omwscripts'), $scripts, [Text.UTF8Encoding]::new($false))

# cmd.exe needs CRLF (an LF-only batch file can miss its goto labels).
$launcher = (Get-Content -Raw (Join-Path $PSScriptRoot 'package\KINETIC.cmd')) -replace "`r?`n", "`r`n"
[IO.File]::WriteAllText((Join-Path $Out 'KINETIC.cmd'), $launcher, [Text.Encoding]::ASCII)
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'package\README.txt') -Destination $Out
[IO.File]::WriteAllText((Join-Path $Out 'VERSION.txt'), "KINETIC $version, source commit $commit`n", [Text.UTF8Encoding]::new($false))

# Licences (GPL-3.0, release audit 10-06): the licence, every third-party
# component's own copyright text (verbatim from the dependency bundle and the
# fetched sources, not retyped), and where this build's source is.
Copy-Item -LiteralPath (Join-Path $source 'LICENSE') -Destination (Join-Path $Out 'LICENSE.txt')
$notices = @('KINETIC third-party notices', '==========================', '',
    'KINETIC is free software under the GNU General Public License version 3 (LICENSE.txt).',
    'It is a modified version of OpenMW (https://openmw.org, GPL-3.0) that includes the OpenMW',
    'Jolt physics merge request 3919. KINETIC is not an official OpenMW release and is not',
    'endorsed by the OpenMW project. Morrowind, The Elder Scrolls and Bethesda are trademarks',
    'of ZeniMax Media Inc.; KINETIC contains no Morrowind game data.', '',
    'Source code: SOURCE.txt. The components below keep their own licences; their texts follow.', '')
$licenceFiles = @()
Get-ChildItem -LiteralPath (Join-Path $deps 'installed\x64-windows\share') -Directory | Sort-Object Name | ForEach-Object {
    $copyright = Join-Path $_.FullName 'copyright'
    if (Test-Path $copyright) { $licenceFiles += [pscustomobject]@{ Name = $_.Name; Path = $copyright } }
}
$fetched = [ordered]@{
    'JoltPhysics' = Join-Path $root 'build-footfall\extern\fetched\joltphysics\LICENSE'
    'RecastNavigation' = Join-Path $root 'upstream-clean\build-upstream\extern\fetched\recastnavigation\License.txt'
    'yaml-cpp' = Join-Path $root 'upstream-clean\build-upstream\extern\fetched\yaml-cpp\LICENSE'
}
foreach ($name in $fetched.Keys) {
    if (Test-Path $fetched[$name]) { $licenceFiles += [pscustomobject]@{ Name = $name; Path = $fetched[$name] } }
    else { Write-Warning "Licence text missing for $name ($($fetched[$name]))" }
}
Get-ChildItem -LiteralPath (Join-Path $source 'extern') -Recurse -Depth 2 -File |
    Where-Object { $_.Name -match '^(LICEN[CS]E|COPYING)' } | ForEach-Object {
        $licenceFiles += [pscustomobject]@{ Name = $_.Directory.Name; Path = $_.FullName }
    }
foreach ($file in $licenceFiles) {
    $notices += ('=' * 78), $file.Name, ('=' * 78), (Get-Content -Raw -LiteralPath $file.Path), ''
}
[IO.File]::WriteAllText((Join-Path $Out 'THIRD-PARTY-NOTICES.txt'), ($notices -join "`r`n"), [Text.UTF8Encoding]::new($false))
if (!$SourceUrl) { $SourceUrl = (& git -C $source config --get remote.public.url) -replace '\.git$', '' }
if (!$SourceUrl) { throw 'No source URL: pass -SourceUrl or add the "public" remote (the GitHub fork).' }
$tag = (& git -C $source tag --points-at $SourceCommit) | Select-Object -First 1
$sourceText = @("KINETIC $version source code (GPL-3.0)", '',
    "Repository: $SourceUrl", "Commit: $(& git -C $source rev-parse $SourceCommit)",
    "Tag: $(if ($tag) { $tag } else { '(none)' })", '',
    'The same source is available on request for three years from the release date.')
[IO.File]::WriteAllText((Join-Path $Out 'SOURCE.txt'), ($sourceText -join "`r`n") + "`r`n", [Text.UTF8Encoding]::new($false))

$size = (Get-ChildItem -Recurse -File $Out | Measure-Object Length -Sum).Sum / 1MB
Write-Output ("Package: {0} ({1:N0} MB)" -f $Out, $size)
if ($Zip) {
    $zipPath = "$Out.zip"
    if (Test-Path $zipPath) { Remove-Item -Force $zipPath }
    Compress-Archive -Path (Join-Path $Out '*') -DestinationPath $zipPath
    Write-Output "Zip: $zipPath"
}

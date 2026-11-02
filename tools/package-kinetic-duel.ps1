<#
Builds a KINETIC package with the NET-01 duel (1v1 between two games): the
ordinary package (package-kinetic.ps1) plus the duel scripts, the flat test
floor and KINETIC-Duel.cmd.

usage: powershell -File tools/package-kinetic-duel.ps1 [-Engine path\to\openmw.exe] [-Out dir] [-Zip]
         [-Release] [-PlayLauncher Run-Game-MotionBricks.cmd]
#>
param(
    [string]$Engine = '',
    [string]$Out = '',
    [switch]$Zip,
    [switch]$Release,
    [string]$PlayLauncher = ''
)
$ErrorActionPreference = 'Stop'
$source = Split-Path -Parent $PSScriptRoot
$root = Split-Path -Parent $source
$commit = (& git -C $source rev-parse --short=10 HEAD).Trim()
if (!$Out) { $Out = Join-Path $root ("dist\KINETIC-duel-" + $commit) }
$arguments = @{ Out = $Out }
if ($Engine) { $arguments.Engine = $Engine }
if ($Release) { $arguments.Release = $true }
if ($PlayLauncher) { $arguments.PlayLauncher = $PlayLauncher }
& (Join-Path $PSScriptRoot 'package-kinetic.ps1') @arguments

$data = Join-Path $Out 'data'
$duel = Join-Path $source 'scripts\data\kinetic_duel'
foreach ($name in @('kinetic_duel_global.lua', 'kinetic_duel_player.lua', 'kinetic_duel_puppet.lua')) {
    Copy-Item -LiteralPath (Join-Path $duel $name) -Destination $data
}
# The flat test floor: the plugin and its mesh.
Copy-Item -LiteralPath (Join-Path $source 'scripts\data\ragdoll_manual\ragdoll_test_area.omwaddon') -Destination $data
New-Item -ItemType Directory -Force -Path (Join-Path $data 'meshes') | Out-Null
Copy-Item -LiteralPath (Join-Path $source 'scripts\data\ragdoll_manual\meshes\ragdoll_test_floor.osgt') `
    -Destination (Join-Path $data 'meshes')
$scripts = Join-Path $data 'kinetic.omwscripts'
$text = [IO.File]::ReadAllText($scripts) + (Get-Content -Raw (Join-Path $duel 'kinetic_duel.omwscripts'))
[IO.File]::WriteAllText($scripts, ($text -replace "`r`n", "`n"), [Text.UTF8Encoding]::new($false))
$launcher = (Get-Content -Raw (Join-Path $PSScriptRoot 'package\KINETIC-Duel.cmd')) -replace "`r?`n", "`r`n"
[IO.File]::WriteAllText((Join-Path $Out 'KINETIC-Duel.cmd'), $launcher, [Text.Encoding]::ASCII)
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'package\DUEL-README.txt') -Destination $Out
Write-Output "Duel package: $Out"
if ($Zip) {
    $zipPath = "$Out.zip"
    if (Test-Path $zipPath) { Remove-Item -Force $zipPath }
    Compress-Archive -Path (Join-Path $Out '*') -DestinationPath $zipPath
    Write-Output "Zip: $zipPath"
}

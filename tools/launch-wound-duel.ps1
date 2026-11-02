param(
    [ValidateSet('iron dagger', 'iron longsword', 'fists')]
    [string] $Weapon = 'iron dagger'
)
$ErrorActionPreference = 'Stop'
$taskSource = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$taskWorkspace = [IO.Path]::GetFullPath((Join-Path $taskSource '..'))
$taskRuntime = Join-Path $taskWorkspace 'runtime-wound-duel'
New-Item -ItemType Directory -Path (Join-Path $taskRuntime 'data') -Force | Out-Null
Get-ChildItem -LiteralPath (Join-Path $taskSource 'scripts/data/ragdoll_wounds') -File | ForEach-Object {
    Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $taskRuntime 'data') -Force
}
[IO.File]::WriteAllText((Join-Path $taskRuntime 'data/wound_duel_loadout.lua'),
    "return { weapon = '$Weapon' }", [Text.UTF8Encoding]::new($false))
Remove-Item Env:GIT_CONFIG_COUNT,Env:GIT_CONFIG_KEY_0,Env:GIT_CONFIG_VALUE_0,Env:OPENMW_RAGDOLL_TEST_STEP_MULTIPLIER -ErrorAction SilentlyContinue
$taskLaunchPath = $env:PATH
Remove-Item Env:PATH
$env:Path = $taskLaunchPath
$env:OPENMW_RAGDOLL_LIMB_DAMAGE_TINT = '0'
# Manual logs carry what each injury was paid with and every raw contact's
# energy, so the contact work profile can be recalibrated from live play.
$env:OPENMW_RAGDOLL_AI_STRIKE_TRACE = '1'
$env:OPENMW_RAGDOLL_CONTACT_WORK_TRACE = '1'
& (Join-Path $PSScriptRoot 'ragdoll-dev.ps1') run -BuildDirectory build-play01 -RuntimeDirectory runtime-wound-duel `
    -ManualRagdollTest -RenderFps 60 -DataPath 'C:\Program Files (x86)\Steam\steamapps\common\Morrowind\Data Files' `
    *> (Join-Path $taskRuntime 'launcher.log')

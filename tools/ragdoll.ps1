[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet('configure', 'build', 'test', 'smoke', 'run')]
    [string] $Command = 'run',
    [ValidateSet(30, 60, 120)]
    [int] $RenderFps = 60,
    [ValidateRange(1, 64)]
    [int] $Jobs = 6,
    [string] $DataPath = 'C:\Program Files (x86)\Steam\steamapps\common\Morrowind\Data Files',
    [string] $TestFilter,
    [string] $TrackerTask
)

$ErrorActionPreference = 'Stop'
$source = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$workspace = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$sourceCommit = & git -C $source rev-parse HEAD
if ($LASTEXITCODE -ne 0) { throw 'Cannot identify canonical source revision.' }
$sourceBranch = & git -C $source branch --show-current
if ($sourceBranch -ne 'master') { throw 'Canonical commands require master. Use ragdoll-dev.ps1 in an isolated task checkout.' }
if (& git -C $source status --porcelain) {
    throw 'Canonical source has uncommitted changes. Commit reviewed work before building or running the canonical profile.'
}
$manifestPath = Join-Path $workspace 'build-master/canonical-build.json'
if ($Command -in @('run', 'smoke')) {
    if (!(Test-Path -LiteralPath $manifestPath -PathType Leaf)) {
        throw 'Canonical build provenance is missing. Run tools/ragdoll.ps1 build first.'
    }
    $built = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
    $enginePath = Join-Path $workspace 'build-master/RelWithDebInfo/openmw.exe'
    if ($built.source_commit -ne $sourceCommit -or
        (Get-FileHash -LiteralPath $enginePath -Algorithm SHA256).Hash -ne $built.openmw_sha256) {
        throw 'Canonical executable does not match this source revision. Run tools/ragdoll.ps1 build first.'
    }
}
$arguments = @{
    Command = $Command
    BuildDirectory = 'build-master'
    RuntimeDirectory = 'runtime-master'
    Jobs = $Jobs
}
if ($TrackerTask) { $arguments.TrackerTask = $TrackerTask }
if ($Command -in @('run', 'smoke')) {
    # Reproducible default profile, with no inherited experiment flags.
    Get-ChildItem Env: | Where-Object {
        $_.Name -like 'OPENMW_RAGDOLL_*' -or $_.Name -like 'OPENMW_JOLT_*'
    } | ForEach-Object { Remove-Item -LiteralPath ('Env:' + $_.Name) }
    $env:SDL_WINDOW_NO_ACTIVATION_WHEN_SHOWN = '1'
    $env:SDL_HINT_FORCE_RAISEWINDOW = '0'
    $arguments.DataPath = $DataPath
}
if ($Command -eq 'run') {
    $runtime = 'runtime-master-manual-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff')
    $arguments.RuntimeDirectory = $runtime
    $arguments.ManualRagdollTest = $true
    $arguments.RenderFps = $RenderFps
    $env:OPENMW_RAGDOLL_STEP_TRACE = '400000'
    $env:OPENMW_RAGDOLL_REPLAY_POSE_TRACE = '1'
    $env:OPENMW_RAGDOLL_REPLAY_SHAPE_DIR = Join-Path $workspace "$runtime/manual-replays/shapes"
}
if ($Command -eq 'smoke') {
    $arguments.SmokeRenderFps = $RenderFps
    if ($TestFilter) { $arguments.SmokeTestFilter = $TestFilter }
}
if ($Command -eq 'test' -and $TestFilter) { $arguments.GTestFilter = $TestFilter }
& (Join-Path $PSScriptRoot 'ragdoll-dev.ps1') @arguments
if ($Command -eq 'build') {
    $finishedCommit = & git -C $source rev-parse HEAD
    if ($finishedCommit -ne $sourceCommit -or (& git -C $source status --porcelain)) {
        throw 'Source changed during the build; this executable is not promoted. Rebuild from a clean committed revision.'
    }
    [ordered]@{
        source_commit = $sourceCommit
        source_branch = $sourceBranch
        completed_utc = [DateTimeOffset]::UtcNow.ToString('o')
        openmw_sha256 = (Get-FileHash -LiteralPath (Join-Path $workspace 'build-master/RelWithDebInfo/openmw.exe') -Algorithm SHA256).Hash
    } | ConvertTo-Json | Set-Content -LiteralPath $manifestPath -Encoding UTF8
}

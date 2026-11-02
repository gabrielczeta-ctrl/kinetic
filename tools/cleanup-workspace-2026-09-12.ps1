[CmdletBinding(SupportsShouldProcess)]
param([switch]$Execute)

$ErrorActionPreference = 'Stop'
$repo = (& git -C (Join-Path $PSScriptRoot '..') rev-parse --show-toplevel).Trim()
if (!$repo) { throw 'Cannot resolve repository root.' }
$workspace = [IO.Path]::GetFullPath((Join-Path $repo '..'))
$expectedRepo = Join-Path $workspace 'source-master'
if ([IO.Path]::GetFullPath($repo) -ne [IO.Path]::GetFullPath($expectedRepo)) {
    throw "Run the committed script from the canonical source-master checkout: $expectedRepo"
}

$names = @(
    'build-fable2',
    'build-jolt-opencs',
    'build-master-configure-check',
    'ragdoll-tracker-package-stage-v2',
    'ragdoll-tracker-package-stage-v3',
    'ragdoll-tracker-package-stage-v4',
    'ragdoll-tracker-package-stage-v5',
    'ragdoll-tracker-package-stage-v6',
    'ragdoll-tracker-package-stage-v7',
    'ragdoll-tracker-package-stage-v9',
    'ragdoll-tracker-site-v2.tar.gz',
    'ragdoll-tracker-site-v3.tar.gz',
    'ragdoll-tracker-site-v4.tar.gz',
    'ragdoll-tracker-site-v5.tar.gz',
    'ragdoll-tracker-site-v6.tar.gz',
    'ragdoll-tracker-site-v7.tar.gz',
    'ragdoll-tracker-site-v8.tar.gz',
    'ragdoll-tracker-stage-v8',
    'ragdoll-tracker-v9.tar.gz',
    'runtime-a97-manual',
    'runtime-combat-baseline-20260912',
    'runtime-continuation-20260912',
    'runtime-crowd',
    'runtime-horizon',
    'runtime-jolt-opencs',
    'runtime-kimi-feasibility',
    'runtime-liveaudit',
    'runtime-manual-20260912-074629',
    'runtime-master-configure-check',
    'runtime-matrix',
    'runtime-mouse-listener-120-20260912',
    'runtime-mouse-listener-30-20260912',
    'runtime-mouse-listener-60-20260912',
    'runtime-mouse-listener-mixed-20260912',
    'runtime-opus-friction',
    'runtime-opus-hip',
    'runtime-opus-preempt',
    'runtime-opus-search',
    'runtime-recovery-acquired-r01-20260912',
    'runtime-recovery-acquired-r02-20260912',
    'runtime-recovery-acquired-r03-20260912',
    'runtime-recovery-acquired-r04-20260912',
    'runtime-recovery-acquired-r05-20260912',
    'runtime-recovery-acquired-r06-20260912',
    'runtime-recovery-acquired-r07-20260912',
    'runtime-recovery-acquired-r08-20260912',
    'runtime-recovery-acquired-r09-20260912',
    'runtime-recovery-acquired-r10-20260912',
    'runtime-recovery-collapse-r01-20260912',
    'runtime-recovery-collapse-r02-20260912',
    'runtime-recovery-collapse-r03-20260912',
    'runtime-recovery-collapse-r04-20260912',
    'runtime-recovery-collapse-r05-20260912',
    'runtime-recovery-collapse-r06-20260912',
    'runtime-recovery-collapse-r07-20260912',
    'runtime-recovery-collapse-r08-20260912',
    'runtime-recovery-collapse-r09-20260912',
    'runtime-recovery-collapse-r10-20260912',
    'runtime-recovery-manual-20260912',
    'runtime-recovery-player-20260912',
    'runtime-recovery-player-r01-20260912',
    'runtime-recovery-player-r02-20260912',
    'runtime-recovery-player-r03-20260912',
    'runtime-recovery-player-r04-20260912',
    'runtime-recovery-player-r05-20260912',
    'runtime-recovery-player-r06-20260912',
    'runtime-recovery-player-r07-20260912',
    'runtime-recovery-player-r08-20260912',
    'runtime-recovery-player-r09-20260912',
    'runtime-recovery-player-r10-20260912',
    'runtime-recovery-player-start-20260912',
    'runtime-recovery-step-20260912'
)

if ($names.Count -ne 72 -or ($names | Sort-Object -Unique).Count -ne 72) {
    throw 'Cleanup allowlist must contain exactly 72 unique paths.'
}
if (Get-Process openmw* -ErrorAction SilentlyContinue) {
    throw 'OpenMW is running. Close it before cleanup.'
}

$targets = foreach ($name in $names) {
    $path = Join-Path $workspace $name
    if (!(Test-Path -LiteralPath $path)) { continue }
    $item = Get-Item -LiteralPath $path -Force
    $full = [IO.Path]::GetFullPath($item.FullName)
    if ([IO.Path]::GetDirectoryName($full) -ne $workspace) {
        throw "Target is not a direct workspace child: $full"
    }
    if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
        throw "Target is a reparse point: $full"
    }
    if ($item.PSIsContainer) {
        $nestedLinks = @(Get-ChildItem -LiteralPath $full -Recurse -Force -Attributes ReparsePoint -ErrorAction SilentlyContinue)
        if ($nestedLinks.Count) { throw "Target contains a reparse point: $full" }
    }
    $item
}

Write-Host "Verified cleanup allowlist: $($targets.Count) remaining path(s)."
$targets | ForEach-Object { Write-Host "  $($_.FullName)" }
if (!$Execute) {
    Write-Host 'Preview only. Add -Execute to delete exactly these paths.'
    return
}

foreach ($target in $targets) {
    if ($PSCmdlet.ShouldProcess($target.FullName, 'Delete verified cleanup candidate')) {
        if ($target.PSIsContainer) {
            Remove-Item -LiteralPath $target.FullName -Recurse -Force
        } else {
            Remove-Item -LiteralPath $target.FullName -Force
        }
    }
}
Write-Host "Deleted $($targets.Count) verified cleanup path(s)."

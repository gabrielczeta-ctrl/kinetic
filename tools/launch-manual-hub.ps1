[CmdletBinding()]
param(
    [ValidateSet(30, 60, 120)][int] $RenderFps = 60,
    [ValidateRange(1, 300)][int] $Seconds = 45,
    [ValidateRange(0, 4)][int] $CheckCircuits = 0,
    [string] $DataPath = 'C:\Program Files (x86)\Steam\steamapps\common\Morrowind\Data Files',
    # Restrict the circuit to one kind of scenario, e.g. the powered-opponent duels.
    [ValidateSet('', 'duel', 'player', 'dungeon')][string] $Only = '',
    # With -Only dungeon: the dungeon (cell name) whose door the player starts at.
    [string] $Dungeon = 'Addamasartus',
    # With -Only dungeon: strong (the hub's maxed character), level2 or level10.
    [ValidateSet('strong', 'level2', 'level10')][string] $Character = 'strong',
    # With -Only dungeon: print the NPCs of these dungeons ("A;B;C") at the start.
    [string] $Survey = '',
    # Fight length. 0 = default: 40 / the character's own health in the full tour; in the
    # duel-only circuit 150 / 250, because a warhammer or battle axe blow does 22-25 and
    # 40 health ended those duels in two hits.
    [ValidateRange(0, 100000)][int] $OpponentHealth = 0,
    [ValidateRange(0, 100000)][int] $PlayerHealth = 0,
    [switch] $PrepareOnly,
    # Where the manual range stands: the flat test floor (default) or a flat spot in
    # Balmora (varied terrain around it: slopes, stairs, the river bridges).
    [ValidateSet('floor', 'balmora')][string] $Area = 'floor',
    # Explicit manual diagnosis of the preserved PLAY-04 development build.
    # This does not change Run-Duels or the default accepted build binding.
    [switch] $GuidedDiagnostic,
    # The current development build (build/), unpinned, recorded. See Run-Game.cmd.
    [switch] $Game,
    # With -Game: a frozen copy of the development build to play (so builds made
    # while the user plays cannot replace it). Its commit is read from
    # "<exe>.commit" when that file exists.
    [string] $Engine = ''
)
$ErrorActionPreference = 'Stop'
function Get-TaskSha256([string] $Path) {
    $taskStream = [IO.File]::OpenRead($Path)
    $taskHasher = [Security.Cryptography.SHA256]::Create()
    try {
        return ([BitConverter]::ToString($taskHasher.ComputeHash($taskStream))).Replace('-', '').ToLowerInvariant()
    }
    finally {
        $taskHasher.Dispose()
        $taskStream.Dispose()
    }
}
$taskSource = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$taskWorkspace = [IO.Path]::GetFullPath((Join-Path $taskSource '..'))
if ($Game) { $GuidedDiagnostic = [switch]::new($true) }
$taskBuild = if ($Game) { 'build' } elseif ($GuidedDiagnostic) { 'build-pgcint' } else { 'build-play04' }
$taskEngine = Join-Path $taskWorkspace "$taskBuild/RelWithDebInfo/openmw.exe"
if ($Game -and $Engine) { $taskEngine = [IO.Path]::GetFullPath($Engine) }
# A game engine from another build directory (build-footfall/RelWithDebInfo/...) runs
# under that build profile, which must belong to this checkout.
if ($Game -and $Engine) { $taskBuild = Split-Path -Leaf (Split-Path -Parent (Split-Path -Parent $taskEngine)) }
$taskEngineHash = Get-TaskSha256 $taskEngine
$taskExpectedHash = if ($GuidedDiagnostic) { '80f109e084cc22b61fac61647ba0422652f99c873fd8d597cbeeee7befd30216' }
    else { '376c4b72939456c70347d5050c5af2d1500f9e016f2037da7af5a208decd1fd3' }
if (!$Game -and $taskEngineHash -ne $taskExpectedHash) {
    throw "$taskBuild changed; verify the new engine before updating this manual-hub binding."
}
$taskTintShader = Join-Path (Split-Path -Parent $taskEngine) 'resources/shaders/compatibility/objects.frag'
$taskTintShaderHash = Get-TaskSha256 $taskTintShader
$taskExpectedShader = if ($GuidedDiagnostic) { 'fbba836065497085ce046421f69b1e87fafca55fbc48692869da4cd78b96d19e' }
    else { '1af8cef9ac62163ac77b8acbc311ca812bb1826a67aee4207f107bce43c72f6c' }
if (!$Game -and $taskTintShaderHash -ne $taskExpectedShader) {
    throw 'The regional injury shader does not match the verified manual build.'
}
$taskDependencies = Join-Path $taskWorkspace 'upstream-clean/deps/vcpkg-x64-windows-2022-2026-02-24/installed/x64-windows/bin'
$taskGuiDll = Join-Path $taskDependencies 'Release/MyGUIEngine.dll'
if (!(Test-Path -LiteralPath $taskGuiDll -PathType Leaf)) { throw "Missing shared dependency: $taskGuiDll" }
# PowerShell can inherit PATH/Path aliases. Keep one spelling before the native
# helper extends it, so the engine receives the same verified DLL directories.
$taskLaunchPath = $env:PATH
Remove-Item Env:PATH -ErrorAction SilentlyContinue
$env:Path = "$taskDependencies/Release;$taskDependencies;$taskLaunchPath"
$taskRuntimePrefix = if ($Game) { 'runtime-GAME-manual-' } elseif ($GuidedDiagnostic) { 'runtime-PLAY-04-guided-manual-' } else { 'runtime-PLAY-05-manual-' }
$taskRuntime = Join-Path $taskWorkspace ($taskRuntimePrefix + [DateTimeOffset]::UtcNow.ToString('yyyyMMddTHHmmssfffZ'))
$taskData = Join-Path $taskRuntime 'data'
if ($GuidedDiagnostic) {
    $env:OPENMW_MANUAL_DIAGNOSTICS = Join-Path $taskRuntime 'manual-recording'
    # Charged release prototype (not in spec v0.2) for the user's play test.
    if (!$Game) { $env:OPENMW_RAGDOLL_TIP_CHARGE = '1' }
}
else {
    Remove-Item Env:OPENMW_MANUAL_DIAGNOSTICS -ErrorAction SilentlyContinue
}
New-Item -ItemType Directory -Path $taskData -Force | Out-Null
Copy-Item -Path (Join-Path $taskSource 'scripts/data/ragdoll_manual/*') -Destination $taskData -Recurse
Get-ChildItem -LiteralPath (Join-Path $taskSource 'scripts/data/ragdoll_hub') -File | ForEach-Object {
    Copy-Item -LiteralPath $_.FullName -Destination $taskData
}
Copy-Item -LiteralPath (Join-Path $taskSource 'scripts/data/ragdoll_manual/ragdoll_manual_player.lua') `
    -Destination (Join-Path $taskData 'manual_hub_player_base.lua')
$taskCheck = if ($CheckCircuits -gt 0) { ", checkCircuits = $CheckCircuits" } else { '' }
if ($Only) { $taskCheck += ", only = '$Only'" }
if ($Only -eq 'duel') {
    if ($OpponentHealth -eq 0) { $OpponentHealth = 150 }
    if ($PlayerHealth -eq 0) { $PlayerHealth = 250 }
}
if ($OpponentHealth -gt 0) { $taskCheck += ", opponentHealth = $OpponentHealth" }
if ($PlayerHealth -gt 0) { $taskCheck += ", playerHealth = $PlayerHealth" }
if ($Area -ne 'floor') { $taskCheck += ", area = '$Area'" }
if ($Only -eq 'dungeon') {
    if ($PlayerHealth -eq 0) { $PlayerHealth = 250; $taskCheck += ", playerHealth = 250" }
    $taskCheck += ", dungeon = '$($Dungeon.Replace("'", ''))'"
    $taskCheck += ", character = '$Character'"
    if ($Survey) { $taskCheck += ", survey = '$($Survey.Replace("'", ''))'" }
}
[IO.File]::WriteAllText((Join-Path $taskData 'manual_hub_config.lua'),
    "return { seconds = $Seconds$taskCheck }`n", [Text.UTF8Encoding]::new($false))
$taskGitPath = $taskSource.Replace('\', '/')
$taskCommit = (& git -c "safe.directory=$taskGitPath" -C $taskSource rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Cannot identify the manual-hub source.' }
$taskDirty = @(& git -c "safe.directory=$taskGitPath" -C $taskSource status --porcelain)
$taskFiles = @(Get-ChildItem -LiteralPath $taskData -File)
# The launcher's own choice wins (user 10-08: Run-Game-MotionBricks.cmd turns the
# whole-part red tint off); unset - the frozen alphas - keeps it on as before.
if (-not $env:OPENMW_RAGDOLL_LIMB_DAMAGE_TINT) { $env:OPENMW_RAGDOLL_LIMB_DAMAGE_TINT = '1' }
$taskFiles += @(Get-ChildItem -LiteralPath (Join-Path $taskSource 'scripts/data/ragdoll_manual') -File -Recurse)
$taskHashes = @{}
foreach ($taskFile in $taskFiles) {
    $taskHashes[$taskFile.FullName] = Get-TaskSha256 $taskFile.FullName
}
$taskEngineCommitFile = "$taskEngine.commit"
$taskEngineSource = if ($Game -and $Engine -and (Test-Path -LiteralPath $taskEngineCommitFile)) { (Get-Content -LiteralPath $taskEngineCommitFile -Raw).Trim() }
    elseif ($Game) { $taskCommit + $(if ($taskDirty.Count) { '-dirty' } else { '' }) } elseif ($GuidedDiagnostic) { 'dea99a5acc' }
    else { '325f4a2b5667149981b2562c4cee50360fed163b' }
@{ task = if ($GuidedDiagnostic) { 'PLAY-04' } else { 'PLAY-05' };
    purpose = if ($GuidedDiagnostic) { 'unqualified guided development build; manual diagnosis only' }
        elseif ($CheckCircuits -gt 0) { 'automated circuit wiring diagnostic; no manual acceptance' } else { 'manual observation only' };
    source_commit = $taskCommit;
    source_dirty = $taskDirty; engine_source = $taskEngineSource; engine_sha256 = $taskEngineHash;
    engine = $taskEngine; render_fps = $RenderFps; physics_fps = 60; script_sha256 = $taskHashes;
    reaction_layer_enabled = ($env:OPENMW_RAGDOLL_REACTION_LAYER -eq '1');
    reaction_layer_environment = $env:OPENMW_RAGDOLL_REACTION_LAYER;
    injury_shader_sha256 = $taskTintShaderHash;
    localized_damage_tint_enabled = ($env:OPENMW_RAGDOLL_LIMB_DAMAGE_TINT -eq '1');
    manual_recording = $GuidedDiagnostic.IsPresent;
    manual_recording_directory = $env:OPENMW_MANUAL_DIAGNOSTICS;
    manual_recording_schema = 1;
    scenarios = if ($Only) { "kind=$Only" } else { 23 }; seconds_per_scenario = $Seconds; check_circuits = $CheckCircuits; runtime = $taskRuntime
} | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $taskRuntime 'manual-manifest.json') -Encoding UTF8
Write-Output "Manual hub prepared: $taskRuntime"
if ($GuidedDiagnostic) {
    Write-Output 'PLAY-04 diagnostic only: known spear grip, held-contact and appearance failures. Run-Duels is unchanged.'
    Write-Output 'Quick click: quick attack. Hold LMB and drag: weapon control. Release ends the gesture.'
    Write-Output 'Always recording: B bookmarks a problem and opens a reason box. Click Save note or Skip. V cycles ghosts, targets, off. Recording continues with ghosts off.'
}
if ($PrepareOnly) { return }
Remove-Item Env:OPENMW_RAGDOLL_TEST_STEP_MULTIPLIER -ErrorAction SilentlyContinue
# Use the same checkout's helper so source and build ownership stay aligned.
try {
    & (Join-Path $taskSource 'tools/ragdoll-dev.ps1') run -BuildDirectory $taskBuild -RuntimeDirectory $taskRuntime `
        -EngineName (Split-Path -Leaf $taskEngine) `
        -ManualRagdollTest -StartCell 'OpenMW Ragdoll Test Floor' -RenderFps $RenderFps -DataPath $DataPath
}
finally {
    if ($GuidedDiagnostic -and (Test-Path -LiteralPath $env:OPENMW_MANUAL_DIAGNOSTICS)) {
        $taskPython = Get-Command python -ErrorAction SilentlyContinue
        if ($taskPython) {
            & $taskPython.Source (Join-Path $taskSource 'tools/play04/review_manual.py') $taskRuntime
            if ($LASTEXITCODE -ne 0) { Write-Warning 'Review reports a recording integrity issue; raw data is preserved.' }
            Write-Output "Bookmark viewer: $(Join-Path $env:OPENMW_MANUAL_DIAGNOSTICS 'review.html')"
            # The mouse against the commanded and physical blade (2026-10-04).
            $mouseView = Join-Path $env:OPENMW_MANUAL_DIAGNOSTICS 'mouse-view.html'
            & $taskPython.Source (Join-Path $taskSource 'tools/body/mouse_view.py') $env:OPENMW_MANUAL_DIAGNOSTICS --html $mouseView
            if ($LASTEXITCODE -eq 0) { Write-Output "Mouse viewer: $mouseView" }
            # The play metrics (test review 10-30): the session's movement and
            # combat numbers beside the previous builds' (play-metrics-history.jsonl).
            $metrics = Join-Path $taskRuntime 'play-metrics.txt'
            & $taskPython.Source (Join-Path $taskSource 'tools/body/play_metrics.py') $taskRuntime --bookmarks *> $metrics
            if ($LASTEXITCODE -eq 0) { Write-Output "Play metrics: $metrics" }
        }
        else { Write-Warning 'Python was unavailable; raw recording is preserved for later review.' }
    }
}

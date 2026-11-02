[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet('bootstrap', 'configure', 'build', 'test', 'run', 'manual-report', 'smoke', 'package', 'clean-generated')]
    [string] $Command = 'build',

    [switch] $WithOpenMWCS,
    [ValidateRange(1, 64)]
    [int] $Jobs = 4,
    # Supply both to use an isolated source/build/runtime profile. Relative
    # names resolve under the development root, not the caller's directory.
    [ValidateNotNullOrEmpty()]
    [string] $BuildDirectory,
    [ValidateNotNullOrEmpty()]
    [string] $RuntimeDirectory,
    # run: the engine executable name in the build's binary directory
    # (a frozen play copy, e.g. openmw-play.exe). Default openmw.exe.
    [string] $EngineName = 'openmw.exe',
    [string] $DataPath,
    [switch] $FreshStart,
    [switch] $EnablePassiveRagdolls,
    [switch] $ManualRagdollTest,
    [ValidateNotNullOrEmpty()]
    [string] $StartCell = 'Balmora',
    [string] $GTestFilter,
    [ValidateRange(1, 100)]
    [int] $GTestRepeat = 1,
    [switch] $GTestShuffle,
    [string] $SmokeTestFilter,
    [ValidatePattern('^test_[a-z0-9_]+$')]
    [string] $SmokeSuite = 'test_ragdoll',
    [ValidateRange(1, 16)]
    [int] $Parallel = 1,
    [string] $SmokeTimingReport,
    [string] $SmokeDataOverride,
    [string] $TrackerTask,
    [ValidateSet(30, 60, 120)]
    [int] $SmokeRenderFps = 60,
    # Manual run only: cap the render loop (framerate limit, vsync off) so
    # the F8 range can be played at 30/60/120 FPS. 0 leaves the settings alone.
    [ValidateSet(0, 30, 60, 120)]
    [int] $RenderFps = 0,
    [ValidateRange(0, 64)]
    [int] $SmokePhysicsThreads,
    [switch] $SmokeNativeMouse,
    [switch] $KeepRawOpenmwLog,
    [switch] $RecordVideo,
    [ValidateRange(1, 2048)]
    [int] $SmokeDiagnosticBudgetMiB = 128,
    [ValidateRange(1, 3600)]
    [int] $SmokeTimeoutSeconds = 300
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
# Disable reusable MSBuild worker nodes, and below wait on the exact CMake
# process rather than Start-Process -Wait's Windows process-tree semantics.
$env:MSBUILDDISABLENODEREUSE = '1'

$sourceRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$developmentRoot = [IO.Path]::GetFullPath((Join-Path $sourceRoot '..'))
$manifestPath = Join-Path $sourceRoot 'docs\ragdoll\version-manifest.json'
$manifest = Get-Content -Raw -LiteralPath $manifestPath | ConvertFrom-Json
$dependencyRoot = Join-Path $developmentRoot $manifest.dependencies.windows_bundle.relative_path
$qtRoot = Join-Path $developmentRoot $manifest.dependencies.qt_overlay.relative_path
$configuration = 'RelWithDebInfo'

function Assert-DevelopmentPath([string] $Path) {
    $fullPath = [IO.Path]::GetFullPath($Path)
    $prefix = $developmentRoot.TrimEnd('\') + '\'
    if (!$fullPath.StartsWith($prefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing path outside development root: $fullPath"
    }
    if ($fullPath.StartsWith('C:\Program Files\OpenMW 0.49.0', [StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing installed OpenMW path: $fullPath"
    }
    return $fullPath
}

function Resolve-ProfileDirectory([string] $OverridePath, [string] $DefaultName, [string] $Prefix) {
    $candidate = if (!$OverridePath) {
        Join-Path $developmentRoot $DefaultName
    } elseif ([IO.Path]::IsPathRooted($OverridePath)) {
        $OverridePath
    } else {
        Join-Path $developmentRoot $OverridePath
    }
    $resolved = (Assert-DevelopmentPath $candidate).TrimEnd('\', '/')
    # The cleanup command already operates on these roots. Keep configurable
    # roots confined to dedicated sibling build-/runtime- directories, so an
    # override cannot repurpose the source, dependency, or workspace root.
    if (![IO.Path]::GetDirectoryName($resolved).Equals($developmentRoot, [StringComparison]::OrdinalIgnoreCase) -or
        !(([IO.Path]::GetFileName($resolved).StartsWith($Prefix, [StringComparison]::OrdinalIgnoreCase)) -or
        ([IO.Path]::GetFileName($resolved) -eq $Prefix.TrimEnd('-')))) {
        throw "Profile directory must be a direct $Prefix directory under $developmentRoot`: $resolved"
    }
    if (Test-Path -LiteralPath $resolved) {
        $item = Get-Item -LiteralPath $resolved -Force
        if (!$item.PSIsContainer -or ($item.Attributes -band [IO.FileAttributes]::ReparsePoint)) {
            throw "Profile directory must be a regular directory, not a file or reparse point: $resolved"
        }
    }
    return $resolved
}

if ($PSBoundParameters.ContainsKey('BuildDirectory') -xor $PSBoundParameters.ContainsKey('RuntimeDirectory')) {
    throw 'Supply both -BuildDirectory and -RuntimeDirectory to keep the build and runtime profile isolated.'
}
$buildRoot = Resolve-ProfileDirectory $BuildDirectory $(if ($WithOpenMWCS) { 'build-jolt-opencs' } else { 'build-jolt' }) 'build-'
$runtimeRoot = Resolve-ProfileDirectory $RuntimeDirectory $(if ($WithOpenMWCS) { 'runtime-jolt-opencs' } else { 'runtime-jolt' }) 'runtime-'
$logRoot = Join-Path $runtimeRoot 'logs'
if ($BuildDirectory) {
    $cachePath = Join-Path $buildRoot 'CMakeCache.txt'
    if (Test-Path -LiteralPath $cachePath -PathType Leaf) {
        $sourceEntry = @(Select-String -LiteralPath $cachePath -Pattern '^CMAKE_HOME_DIRECTORY:INTERNAL=(.+)$')
        if (!$sourceEntry -or $sourceEntry.Count -ne 1) {
            throw "Cannot establish the source directory for this build profile: $cachePath"
        }
        $configuredSource = [IO.Path]::GetFullPath($sourceEntry[0].Matches[0].Groups[1].Value).TrimEnd('\', '/')
        if (!$configuredSource.Equals($sourceRoot, [StringComparison]::OrdinalIgnoreCase)) {
            throw "Build profile belongs to $configuredSource, but this helper belongs to $sourceRoot."
        }
    }
}

function Get-DevelopmentRelativePath([string] $Path) {
    $fullPath = Assert-DevelopmentPath $Path
    return $fullPath.Substring($developmentRoot.TrimEnd('\').Length + 1).Replace('\', '/')
}

function New-IsolatedDirectories {
    foreach ($path in @($runtimeRoot, $logRoot, (Join-Path $runtimeRoot 'config'),
            (Join-Path $runtimeRoot 'userdata'), (Join-Path $runtimeRoot 'data'))) {
        $safePath = Assert-DevelopmentPath $path
        New-Item -ItemType Directory -Force -Path $safePath | Out-Null
    }
}

function Invoke-Native([string] $FilePath, [string[]] $Arguments, [string] $LogName) {
    New-IsolatedDirectories
    $logPath = Join-Path $logRoot $LogName
    $quotedArguments = foreach ($argument in $Arguments) {
        if ($argument.Contains('"')) {
            throw "Native argument contains an unsupported quote: $argument"
        }
        if ($argument -match '\s') { '"' + $argument + '"' } else { $argument }
    }

    $startInfo = [Diagnostics.ProcessStartInfo]::new()
    $startInfo.FileName = $FilePath
    $startInfo.Arguments = $quotedArguments -join ' '
    $startInfo.UseShellExecute = $false
    $startInfo.CreateNoWindow = $true
    $startInfo.RedirectStandardOutput = $true
    $startInfo.RedirectStandardError = $true
    $process = [Diagnostics.Process]::new()
    $process.StartInfo = $startInfo
    if (!$process.Start()) {
        throw "Failed to start native command: $FilePath"
    }
    $stdoutTask = $process.StandardOutput.ReadToEndAsync()
    $stderrTask = $process.StandardError.ReadToEndAsync()
    $process.WaitForExit()
    $exitCode = $process.ExitCode
    $stdout = $stdoutTask.Result
    $stderr = $stderrTask.Result
    $combined = $stdout + $stderr
    [IO.File]::WriteAllText($logPath, $combined, [Text.UTF8Encoding]::new($false))
    if ($combined) {
        [Console]::Out.Write($combined)
    }
    if ($exitCode -ne 0) {
        throw "Command failed with exit code $exitCode. See $logPath"
    }
}

function Get-BinaryDirectory {
    $path = Join-Path $buildRoot $configuration
    if (!(Test-Path -LiteralPath $path -PathType Container)) {
        throw "Build output not found: $path. Run configure and build first."
    }
    return $path
}

function Set-IsolatedLocalConfig {
    New-IsolatedDirectories
    $binaryDirectory = Get-BinaryDirectory
    $configPath = Join-Path $binaryDirectory 'openmw.cfg'
    if (!(Test-Path -LiteralPath $configPath -PathType Leaf)) {
        throw "Generated local config not found: $configPath"
    }

    $userData = (Join-Path $runtimeRoot 'userdata').Replace('\', '/')
    $config = (Join-Path $runtimeRoot 'config').Replace('\', '/')
    $data = (Join-Path $runtimeRoot 'data').Replace('\', '/')
    $contents = Get-Content -Raw -LiteralPath $configPath
    $contents = [regex]::Replace($contents, '(?m)^data-local=.*$', "data-local=`"$data`"")
    $contents = [regex]::Replace($contents, '(?m)^user-data=.*$', "user-data=`"$userData`"")
    $contents = [regex]::Replace($contents, '(?m)^config=.*$', "config=`"$config`"")
    [IO.File]::WriteAllText($configPath, $contents, [Text.UTF8Encoding]::new($false))
}

function Set-RuntimePath {
    $yamlDirectory = Join-Path $buildRoot '_deps\yaml-cpp-build\RelWithDebInfo'
    $releaseDirectory = Join-Path $dependencyRoot 'installed\x64-windows\bin\Release'
    $binaryDirectory = Join-Path $dependencyRoot 'installed\x64-windows\bin'
    $env:Path = "$yamlDirectory;$releaseDirectory;$binaryDirectory;$env:Path"
}

function Get-MorrowindDataLayout {
    if (!$DataPath) {
        throw 'This command requires -DataPath pointing to legally owned Morrowind Data Files.'
    }

    $gameData = [IO.Path]::GetFullPath($DataPath)
    if (!(Test-Path -LiteralPath $gameData -PathType Container)) {
        throw "Morrowind data directory not found: $gameData"
    }

    $contentFiles = @('Morrowind.esm', 'Tribunal.esm', 'Bloodmoon.esm')
    $archiveFiles = @('Morrowind.bsa', 'Tribunal.bsa', 'Bloodmoon.bsa')
    foreach ($requiredFile in @($contentFiles + $archiveFiles)) {
        if (!(Test-Path -LiteralPath (Join-Path $gameData $requiredFile) -PathType Leaf)) {
            throw "$requiredFile not found under Morrowind data directory: $gameData. The development scenarios require the Game of the Year content set."
        }
    }

    $iniCandidates = @(
        (Join-Path $gameData 'Morrowind.ini'),
        (Join-Path (Split-Path -Parent $gameData) 'Morrowind.ini')
    )
    $morrowindIni = $iniCandidates | Where-Object { Test-Path -LiteralPath $_ -PathType Leaf } | Select-Object -First 1
    if (!$morrowindIni) {
        throw 'Morrowind.ini was not found in the data directory or its parent; fallback values are required.'
    }

    return [pscustomobject]@{
        DataPath = $gameData
        IniPath = $morrowindIni
        ContentFiles = $contentFiles
        ArchiveFiles = $archiveFiles
    }
}

function Set-DevelopmentRunSettings {
    $settingsPath = Assert-DevelopmentPath (Join-Path $runtimeRoot 'config\settings.cfg')
    $contents = if (Test-Path -LiteralPath $settingsPath -PathType Leaf) {
        Get-Content -Raw -LiteralPath $settingsPath
    } else {
        ''
    }
    $contents = [regex]::Replace(
        $contents,
        '(?mi)^\s*enable passive ragdolls\s*=.*(?:\r?\n|$)',
        '')
    $enabled = if ($EnablePassiveRagdolls -or $ManualRagdollTest) { 'true' } else { 'false' }
    $contents = [regex]::Replace($contents, '(?mi)^\s*(resolution x|resolution y|fullscreen|framerate limit|vsync mode)\s*=.*(?:\r?\n|$)', '')
    # Remove empty duplicate sections left by earlier launcher versions.
    $contents = [regex]::Replace($contents, '(?m)^\[([^\]\r\n]+)\]\s*(?=\[|\z)', '')
    $renderCap = if ($RenderFps -gt 0) { "framerate limit = $RenderFps`r`nvsync mode = 0`r`n" } else { '' }
    $prefix = "[Video]`r`nresolution x = 1280`r`nresolution y = 720`r`nfullscreen = false`r`n$renderCap`r`n[Game]`r`nenable passive ragdolls = $enabled`r`n`r`n"
    [IO.File]::WriteAllText($settingsPath, $prefix + $contents.TrimStart(), [Text.UTF8Encoding]::new($false))
}

function Invoke-Bootstrap {
    if (!(Test-Path -LiteralPath $dependencyRoot -PathType Container)) {
        throw "Pinned dependency bundle is missing: $dependencyRoot"
    }
    foreach ($tool in @('cmake', 'git')) {
        if (!(Get-Command $tool -ErrorAction SilentlyContinue)) {
            throw "Required tool is not available: $tool"
        }
    }

    $toolchain = Join-Path $dependencyRoot 'scripts\buildsystems\vcpkg.cmake'
    $compiler = 'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Tools\MSVC\14.44.35207\bin\Hostx64\x64\cl.exe'
    foreach ($requiredPath in @($toolchain, $compiler)) {
        if (!(Test-Path -LiteralPath $requiredPath -PathType Leaf)) {
            throw "Pinned build input is missing: $requiredPath"
        }
    }

    $upstream = & git -C $sourceRoot rev-parse "$($manifest.source.openmw_upstream)^{commit}"
    $joltMergeRequest = & git -C $sourceRoot rev-parse "$($manifest.source.jolt_merge_request)^{commit}"
    if ($upstream -ne $manifest.source.openmw_upstream -or
        $joltMergeRequest -ne $manifest.source.jolt_merge_request) {
        throw 'One or more pinned source revisions are unavailable.'
    }
    # The early ragdoll reference (another fork) is only in development clones.
    $ragdollReference = & git -C $sourceRoot rev-parse --verify --quiet "$($manifest.source.lihogloglo_reference)^{commit}"
    if (!$ragdollReference) { Write-Warning 'The ragdoll reference revision is not in this clone (not needed to build).' }

    $fixture = Join-Path $developmentRoot $manifest.test_inputs.component_fixture.relative_path
    if (Test-Path -LiteralPath $fixture -PathType Leaf) {
        $actualHash = (Get-FileHash -Algorithm SHA512 -LiteralPath $fixture).Hash.ToLowerInvariant()
        if ($actualHash -ne $manifest.test_inputs.component_fixture.sha512) {
            throw "Component fixture checksum mismatch: $fixture"
        }
    }

    Write-Host "Pinned source revisions and Windows dependency bundle are available."
}

function Invoke-Configure {
    Invoke-Bootstrap
    New-IsolatedDirectories
    New-Item -ItemType Directory -Force -Path (Assert-DevelopmentPath $buildRoot) | Out-Null

    $fixtureSource = Join-Path $developmentRoot $manifest.test_inputs.component_fixture.relative_path
    if (Test-Path -LiteralPath $fixtureSource -PathType Leaf) {
        $fixtureDirectory = Assert-DevelopmentPath (Join-Path $buildRoot 'apps\components_tests\data')
        New-Item -ItemType Directory -Force -Path $fixtureDirectory | Out-Null
        Copy-Item -LiteralPath $fixtureSource -Destination (Join-Path $fixtureDirectory 'template.omwgame') -Force
    }

    $toolchain = Join-Path $dependencyRoot 'scripts\buildsystems\vcpkg.cmake'
    $cleanFetchRoot = Join-Path $developmentRoot 'upstream-clean\build-upstream\extern\fetched'
    $arguments = @(
        '-S', $sourceRoot,
        '-B', $buildRoot,
        '-G', 'Visual Studio 17 2022',
        '-A', 'x64',
        "-DCMAKE_TOOLCHAIN_FILE=$toolchain",
        '-DVCPKG_TARGET_TRIPLET=x64-windows',
        '-DCMAKE_BUILD_TYPE=RelWithDebInfo',
        '-DBUILD_OPENMW=ON',
        '-DBUILD_NAVMESHTOOL=ON',
        '-DBUILD_PHYSICSOBJECTTOOL=ON',
        '-DBUILD_BENCHMARKS=OFF',
        '-DOPENMW_RAGDOLL_DEV_API=ON',
          '-DUSE_LUAJIT=ON',
          "-DLuaJit_INCLUDE_DIR=$(Join-Path $dependencyRoot 'installed/x64-windows/include/luajit')",
          "-DLuaJit_LIBRARY=$(Join-Path $dependencyRoot 'installed/x64-windows/lib/lua51.lib')",
          '-DOPENMW_USE_SYSTEM_YAML_CPP=OFF',
          '-DYAML_CPP_STATIC=ON',
          '-DOPENMW_USE_SYSTEM_GOOGLETEST=OFF',
          '-DOPENMW_USE_SYSTEM_RECASTNAVIGATION=OFF',
          '-DRECASTNAVIGATION_STATIC=ON',
          '-DOPENMW_USE_SYSTEM_SQLITE3=OFF',
          '-DCMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO=/debug /INCREMENTAL:NO',
        "-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=$(Join-Path $cleanFetchRoot 'googletest')",
        "-DFETCHCONTENT_SOURCE_DIR_RECASTNAVIGATION=$(Join-Path $cleanFetchRoot 'recastnavigation')",
        "-DFETCHCONTENT_SOURCE_DIR_SQLITE3=$(Join-Path $cleanFetchRoot 'sqlite3')",
        "-DFETCHCONTENT_SOURCE_DIR_YAML-CPP=$(Join-Path $cleanFetchRoot 'yaml-cpp')"
    )

    if ($WithOpenMWCS) {
        $qtPrefix = Join-Path $qtRoot 'installed\x64-windows'
        if (!(Test-Path -LiteralPath $qtPrefix -PathType Container)) {
            throw "Pinned Qt overlay is missing: $qtPrefix"
        }
        $arguments += @(
            '-DBUILD_OPENCS=ON',
            '-DBUILD_LAUNCHER=ON',
            '-DBUILD_WIZARD=ON',
            '-DBUILD_OPENMW_TESTS=OFF',
            "-DCMAKE_PREFIX_PATH=$qtPrefix",
            "-DQt6_DIR=$(Join-Path $qtPrefix 'share\Qt6')",
            '-DCMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO=/debug /INCREMENTAL:NO'
        )
    }
    else {
        $arguments += @(
            '-DBUILD_OPENCS=OFF',
            '-DBUILD_LAUNCHER=OFF',
            '-DBUILD_WIZARD=OFF',
            '-DBUILD_OPENMW_TESTS=ON',
            '-DBUILD_COMPONENTS_TESTS=ON'
        )
    }

    Invoke-Native 'cmake' $arguments 'configure.log'
    Set-IsolatedLocalConfig
}

function Invoke-Build {
    $targets = if ($WithOpenMWCS) {
        @()
    }
    else {
        @('--target', 'openmw', 'components-tests', 'ragdollcore-tests', 'openmw-tests', 'openmw-navmeshtool',
            'openmw-physicsobjecttool', 'openmw-iniimporter', 'ragdollcore-walksearch')
    }
    # Incremental .ilk caches exhausted this workspace during repeated live links.
    $arguments = @('--build', $buildRoot, '--config', $configuration, '--parallel', $Jobs) + $targets `
        + @('--', '/nr:false', '/p:LinkIncremental=false')
    Invoke-Native 'cmake' $arguments 'build.log'
}

function Invoke-Tests {
    $testTargets = @('ragdollcore-tests', 'components-tests', 'openmw-tests')
    $testBuildArguments = @('--build', $buildRoot, '--config', $configuration, '--parallel', $Jobs,
        '--target') + $testTargets + @('--', '/nr:false', '/p:LinkIncremental=false')
    Invoke-Native 'cmake' $testBuildArguments 'test-build.log'

    Set-IsolatedLocalConfig
    Set-RuntimePath
    $binaryDirectory = Get-BinaryDirectory
    $resultsRoot = Join-Path $runtimeRoot 'test-results'
    New-Item -ItemType Directory -Force -Path $resultsRoot | Out-Null

    $results = @()
    $failedTests = @()
    foreach ($test in $testTargets) {
        $executable = Join-Path $binaryDirectory "$test.exe"
        if (!(Test-Path -LiteralPath $executable -PathType Leaf)) {
            throw "Test executable not found: $executable"
        }
        $jsonPath = Join-Path $resultsRoot "$test.json"
        $started = [DateTimeOffset]::UtcNow
        $testArguments = @('--gtest_brief=1', "--gtest_output=json:$jsonPath")
        if ($GTestFilter) {
            $testArguments += "--gtest_filter=$GTestFilter"
        }
        if ($GTestRepeat -ne 1) {
            $testArguments += "--gtest_repeat=$GTestRepeat"
        }
        if ($GTestShuffle) {
            $testArguments += '--gtest_shuffle'
        }
        & $executable @testArguments 2>&1 | Tee-Object -FilePath (Join-Path $logRoot "$test.log")
        $exitCode = $LASTEXITCODE
        $results += [ordered]@{
            name = $test
            exit_code = $exitCode
            passed = ($exitCode -eq 0)
            started_utc = $started.ToString('o')
            result_file = Get-DevelopmentRelativePath $jsonPath
        }
        if ($exitCode -ne 0) {
            $failedTests += "$test ($exitCode)"
        }
    }

    # The executables above are run directly so GoogleTest can write one JSON
    # result per suite and accept a shared developer filter/repeat/shuffle. Do
    # not execute them a second time through CTest; instead verify that a normal
    # standalone CTest invocation has a complete, machine-readable catalog.
    $ctestLogPath = Join-Path $logRoot 'ctest-registration.json'
    $ctestOutput = & ctest --test-dir $buildRoot -C $configuration --show-only=json-v1 2>&1
    $ctestExitCode = $LASTEXITCODE
    $ctestText = $ctestOutput -join [Environment]::NewLine
    [IO.File]::WriteAllText($ctestLogPath, $ctestText, [Text.UTF8Encoding]::new($false))
    $expectedCtestTests = $testTargets
    $registeredCtestTests = @()
    if ($ctestExitCode -eq 0) {
        try {
            $ctestModel = $ctestText | ConvertFrom-Json
            $registeredCtestTests = @($ctestModel.tests | ForEach-Object { $_.name })
        }
        catch {
            $ctestExitCode = 1
            Write-Error "CTest registration output is not valid JSON: $ctestLogPath" -ErrorAction Continue
        }
    }
    $missingCtestTests = @($expectedCtestTests | Where-Object { $_ -notin $registeredCtestTests })
    if ($missingCtestTests.Count -ne 0) {
        $ctestExitCode = 1
        Write-Error "CTest registration is missing: $($missingCtestTests -join ', ')" -ErrorAction Continue
    }
    if ($ctestExitCode -eq 0) {
        Write-Output "CTest registered $($registeredCtestTests.Count) tests: $($registeredCtestTests -join ', ')"
    }
    $results += [ordered]@{
        name = 'ctest-registration'
        exit_code = $ctestExitCode
        passed = ($ctestExitCode -eq 0)
        registered = $registeredCtestTests.Count
    }
    $summary = [ordered]@{
        schema_version = 1
        generated_utc = [DateTimeOffset]::UtcNow.ToString('o')
        source_commit = (& git -C $sourceRoot rev-parse HEAD)
        configuration = $configuration
        with_openmw_cs = [bool]$WithOpenMWCS
        results = $results
        tracker = [ordered]@{
            producer = 'ragdoll-dev.ps1 test'
            total_checks = $results.Count
            failed_checks = @($results | Where-Object { !$_.passed }).Count
        }
    }
    $summaryPath = Join-Path $resultsRoot 'summary.json'
    $summary | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $summaryPath -Encoding utf8
    if ($ctestExitCode -ne 0) {
        $failedTests += "ctest-registration ($ctestExitCode)"
    }
    if ($TrackerTask) {
        & python (Join-Path $sourceRoot 'tools\tracker\write_result.py') --root $sourceRoot --task $TrackerTask --producer unit_tests_v1 --artifact $summaryPath
        if ($LASTEXITCODE -ne 0) { throw "Tracker result writer failed for $TrackerTask." }
    }
    if ($failedTests.Count -ne 0) {
        throw "Test command failed: $($failedTests -join ', '). All configured suites were still run; see $resultsRoot."
    }
}

function Export-ManualRagdollTraces {
    $engineLog = Join-Path $runtimeRoot 'config\openmw.log'
    if (!(Test-Path -LiteralPath $engineLog -PathType Leaf)) {
        Write-Warning "Manual trace extraction skipped because the engine log is missing: $engineLog"
        return
    }

    $marker = 'RAGDOLL_MANUAL_TRACE_JSON'
    $records = @()
    foreach ($line in Get-Content -LiteralPath $engineLog) {
        $markerIndex = $line.IndexOf($marker, [StringComparison]::Ordinal)
        if ($markerIndex -lt 0) {
            continue
        }
        $json = $line.Substring($markerIndex + $marker.Length).Trim()
        try {
            $parsed = $json | ConvertFrom-Json
        } catch {
            Write-Warning "Ignoring malformed manual ragdoll telemetry: $($_.Exception.Message)"
            continue
        }
        $records += [pscustomobject]@{ Json = $json; Parsed = $parsed }
    }
    if ($records.Count -eq 0) {
        Write-Host 'No bounded manual ragdoll traces were recorded in this run.'
        return
    }

    $traceRoot = Assert-DevelopmentPath (Join-Path $runtimeRoot 'manual-traces')
    New-Item -ItemType Directory -Force -Path $traceRoot | Out-Null
    $timestamp = [DateTimeOffset]::UtcNow.ToString('yyyyMMdd-HHmmssfff')
    foreach ($group in $records | Group-Object { $_.Parsed.trace_id }) {
        $sample = [string]$group.Group[0].Parsed.sample
        $safeSample = $sample -replace '[^A-Za-z0-9_-]', '_'
        $traceId = [int]$group.Name
        $tracePath = Join-Path $traceRoot ("$timestamp-$safeSample-{0:D3}.jsonl" -f $traceId)
        $jsonLines = [string[]]@($group.Group | ForEach-Object { $_.Json })
        [IO.File]::WriteAllLines($tracePath, $jsonLines, [Text.UTF8Encoding]::new($false))

        $summary = @($group.Group | Where-Object { $_.Parsed.kind -eq 'summary' } | Select-Object -Last 1)
        if ($summary.Count -eq 1) {
            $value = $summary[0].Parsed
            Write-Host ("Manual trace {0}: {1} frames, {2} transitions, {3} timeout frames, final mode {4}" -f `
                $traceId, $value.frames, $value.gaitTransitions, $value.gaitTimeoutFrames, $value.finalMode)
            Write-Host ("  observation unsupported={0} samples; arming unsupported={1}; first fall sample={2}; minimum pelvis up={3:N3}" -f `
                $value.unsupportedFrames, $value.armingUnsupportedFrames, $value.firstFallingFrame, $value.minimumPelvisUp)
            Write-Host ("  observation upward saturation={0} samples (longest {1}); max anchor separation={2:N3}" -f `
                $value.upwardSaturationFrames, $value.maximumConsecutiveUpwardSaturation, $value.maximumAnchorSeparation)
            if ($value.PSObject.Properties.Name -contains 'updateSamples') {
                Write-Host ("  presentation samples={0}; repeated physics steps={1}; skipped physics steps={2}" -f `
                    $value.updateSamples, $value.duplicatePhysicsSteps, $value.skippedPhysicsSteps)
                Write-Host ("  max gameplay root lag={0:N3}; prepared pelvis error={1:N3}; prepared pelvis jump={2:N3}" -f `
                    $value.maximumGameplayRootLag, $value.maximumRenderPelvisError, $value.maximumRenderPelvisJump)
            }
        } else {
            Write-Warning "Manual trace $traceId ended without a summary record."
        }
        Write-Host "Manual trace JSONL: $tracePath"
    }
}

function Invoke-Run {
    $layout = Get-MorrowindDataLayout
    Initialize-ReplayShapeDirectory
    Set-IsolatedLocalConfig
    Set-DevelopmentRunSettings
    Set-RuntimePath
    $binaryDirectory = Get-BinaryDirectory
    $arguments = @(
        '--replace=config',
        "--config=$(Join-Path $runtimeRoot 'config')",
        '--replace=user-data',
        "--user-data=$(Join-Path $runtimeRoot 'userdata')",
        "--data-local=$(Join-Path $runtimeRoot 'data')",
        "--resources=$(Join-Path $binaryDirectory 'resources')",
        '--replace=content',
        '--replace=fallback-archive',
        "--data=$($layout.DataPath)"
    )
    foreach ($archiveFile in $layout.ArchiveFiles) {
        $arguments += "--fallback-archive=$archiveFile"
    }
    foreach ($contentFile in $layout.ContentFiles) {
        $arguments += "--content=$contentFile"
    }
    if ($ManualRagdollTest) {
        # Keep the manual trace cadence comparable with the automated real-data
        # gate and independent of a developer's inherited environment.
        $env:OPENMW_PHYSICS_FPS = '60'
        # I.RagdollDev.recordWalk / recordPath persist their capture here so a
        # fall seen in play can be replayed offline (walksearch --replay-capture).
        $manualReplays = Join-Path $runtimeRoot 'manual-replays'
        New-Item -ItemType Directory -Force -Path $manualReplays | Out-Null
        $env:OPENMW_RAGDOLL_REPLAY_OUTPUT_DIR = $manualReplays
        $manualData = Join-Path $sourceRoot 'scripts\data\ragdoll_manual'
        $arguments += @(
            "--data=$manualData",
            '--content=ragdoll_test_area.omwaddon',
            '--content=ragdoll_manual.omwscripts',
            '--content=ragdoll_manual_autostart.omwscripts'
        )
    }
    if ($FreshStart -or $ManualRagdollTest) {
        $arguments += @(
            "--start=$StartCell",
            '--skip-menu=1',
            '--new-game=0'
        )
    }
    if ($ManualRagdollTest) {
        Write-Host 'The manual range relocates to an isolated flat-floor interior and spawns four named NPC samples.'
        Write-Host 'Wait for "RAGDOLL TEST RANGE READY", then press F8 for the clickable test controls.'
        Write-Host 'Samples: A=powered fall/recovery, B=passive death, C=experimental powered gait, D=physical defense.'
        Write-Host 'Choose "Start latest solver test (5 simulation seconds)"; the menu closes and the bounded test starts on C.'
        Write-Host 'F8 also offers "Stop experimental test and movement" and "Reset flat-floor range".'
        Write-Host 'Tests may run slowly. Check the applied/rejected counters; enabled alone does not mean the solver ran.'
        Write-Host 'Optional console comparison: enter luag, then I.RagdollDev.traceGait(10) or I.RagdollDev.traceStanding(10).'
        Write-Host 'Defense: F8 -> Prepare station D; hold attack and move the mouse; F7 raises the target guard.'
    }
    Invoke-Native (Join-Path $binaryDirectory $EngineName) $arguments 'openmw-run.log'
    if ($ManualRagdollTest) {
        Export-ManualRagdollTraces
    }
}

function Initialize-ReplayShapeDirectory {
    # The native contact callback writes snapshots but does not create this
    # directory. Prepare it before startup contacts, or the only ground record
    # is saved=0 and an otherwise complete recording cannot be replayed.
    if ($env:OPENMW_RAGDOLL_REPLAY_SHAPE_DIR) {
        $shapeDirectory = Assert-DevelopmentPath $env:OPENMW_RAGDOLL_REPLAY_SHAPE_DIR
        New-Item -ItemType Directory -Force -Path $shapeDirectory | Out-Null
        $env:OPENMW_RAGDOLL_REPLAY_SHAPE_DIR = $shapeDirectory
    }
}

function Invoke-Smoke {
    $layout = Get-MorrowindDataLayout
    Initialize-ReplayShapeDirectory

    Set-IsolatedLocalConfig
    Set-RuntimePath
    # The engine defaults to 60 Hz, but pin it explicitly so smoke results do
    # not inherit a developer-machine override.
    $env:OPENMW_PHYSICS_FPS = '60'
    $binaryDirectory = Get-BinaryDirectory
    $suite = Join-Path $sourceRoot "scripts\data\integration_tests\$SmokeSuite"
    if (!(Test-Path -LiteralPath $suite -PathType Container)) {
        throw "Smoke suite does not exist: $suite"
    }
    # Keep every trial's fixed-seed replay instead of overwriting the last run.
    $runId = [DateTimeOffset]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
    $runFamily = if ($Parallel -gt 1) { "runtime-smoke" } else { "smoke-flat-floor" }
    $workDirectory = Assert-DevelopmentPath (Join-Path $runtimeRoot "$runFamily\$runId")
    Write-Host "Smoke artifacts: $workDirectory"
    $replayOutputDirectory = Assert-DevelopmentPath (Join-Path $workDirectory 'test-results\replays')
    New-Item -ItemType Directory -Force -Path $replayOutputDirectory | Out-Null
    $env:OPENMW_RAGDOLL_REPLAY_OUTPUT_DIR = $replayOutputDirectory
    $arguments = @(
        (Join-Path $sourceRoot 'scripts\integration_tests.py'),
        $suite,
        '--omw', (Join-Path $binaryDirectory 'openmw.exe'),
        '--workdir', $workDirectory,
        '--config', $suite,
        '--data', (Join-Path $sourceRoot 'files\data'),
        '--data', (Join-Path $sourceRoot 'scripts\data\ragdoll_manual'),
        '--data', $layout.DataPath,
        '--ini', $layout.IniPath,
        '--suite_timeout', $SmokeTimeoutSeconds,
        '--diagnostic_budget_mib', $SmokeDiagnosticBudgetMiB,
        '--render_fps', $SmokeRenderFps
    )
    # integration_tests.py always writes a gzip archive before removing the
    # raw engine log. Keeping both copies made parallel smoke batches grow by
    # several gigabytes, so retain only the archive unless a debugging run
    # explicitly asks for the raw file.
    if (!$KeepRawOpenmwLog) {
        $arguments += '--discard_raw_openmw_log'
    }
    if ($SmokeDataOverride) {
        $override = Assert-DevelopmentPath $SmokeDataOverride
        if (!(Test-Path -LiteralPath $override -PathType Container)) {
            throw "Smoke data override directory does not exist: $override"
        }
        $arguments += @('--data', $override)
    }
    if ($SmokeNativeMouse) {
        $arguments += '--native_mouse'
    }
    if ($script:PSBoundParameters.ContainsKey('SmokePhysicsThreads')) {
        $arguments += @('--physics_threads', $SmokePhysicsThreads)
    }
    if ($SmokeTestFilter) {
        $arguments += @('--test_filter', $SmokeTestFilter)
    }
    if ($RecordVideo) {
        $arguments += @('--video_directory', (Join-Path $runtimeRoot 'progress-videos'))
    }
    if ($Parallel -gt 1) { $arguments += @('--parallel', $Parallel) }
    if ($SmokeTimingReport) { $arguments += @('--timing_report', ([IO.Path]::GetFullPath($SmokeTimingReport))) }
    Invoke-Native 'python' $arguments 'smoke.log'
    if ($TrackerTask) {
        $summaryPath = Join-Path $workDirectory 'result-summary.json'
        & python (Join-Path $sourceRoot 'tools\tracker\write_result.py') --root $sourceRoot --task $TrackerTask --producer smoke_v1 --artifact $summaryPath
        if ($LASTEXITCODE -ne 0) { throw "Tracker result writer failed for $TrackerTask." }
    }
}

function Invoke-Package {
    $packageRoot = Assert-DevelopmentPath (Join-Path $runtimeRoot 'package')
    New-Item -ItemType Directory -Force -Path $packageRoot | Out-Null
    Invoke-Native 'cmake' @('--install', $buildRoot, '--config', $configuration, '--prefix', $packageRoot) 'package.log'
}

function Invoke-CleanGenerated {
    foreach ($path in @($buildRoot, $runtimeRoot)) {
        $safePath = Assert-DevelopmentPath $path
        if (Test-Path -LiteralPath $safePath) {
            Remove-Item -LiteralPath $safePath -Recurse -Force
        }
    }
}

$liveMutex = $null
$ownsLiveMutex = $false
try {
    if ($Command -in @('run', 'smoke')) {
        # One coordinator owns the live engine lease. It may launch isolated
        # smoke shards; manual games and other coordinators remain excluded.
        $liveMutex = [Threading.Mutex]::new($false, 'Local\OpenMW-ragdoll-live')
        try { $ownsLiveMutex = $liveMutex.WaitOne(0) }
        catch [Threading.AbandonedMutexException] { $ownsLiveMutex = $true }
        if (!$ownsLiveMutex) {
            throw 'Another ragdoll run or smoke command is active. Finish that session before launching another.'
        }
        try {
            $games = @(Get-CimInstance Win32_Process -Filter "Name = 'openmw.exe' OR Name = 'openmw-play.exe'" -ErrorAction Stop | Where-Object {
                # Windows launches a second copy solely to monitor crashes.
                $_.CommandLine -notmatch '(?:^|\s)--crash-monitor(?:\s|$)'
            })
        }
        catch {
            # Restricted workers can deny CIM. Preserve the collision guard by
            # conservatively treating every visible OpenMW process as a game.
            $games = @(Get-Process openmw, openmw-play -ErrorAction SilentlyContinue | ForEach-Object {
                [pscustomobject]@{ ProcessId = $_.Id }
            })
        }
        if ($games.Count -gt 0) {
            throw "OpenMW is already running (PID $($games.ProcessId -join ', ')). Close that game before launching another."
        }
    }
    switch ($Command) {
        'bootstrap' { Invoke-Bootstrap }
        'configure' { Invoke-Configure }
        'build' { Invoke-Build }
        'test' { Invoke-Tests }
        'run' { Invoke-Run }
        'manual-report' { Export-ManualRagdollTraces }
        'smoke' { Invoke-Smoke }
        'package' { Invoke-Package }
        'clean-generated' { Invoke-CleanGenerated }
    }
}
finally {
    if ($ownsLiveMutex) { $liveMutex.ReleaseMutex() }
    if ($liveMutex) { $liveMutex.Dispose() }
}

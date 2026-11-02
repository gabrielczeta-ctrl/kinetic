@echo off
setlocal EnableExtensions
rem KINETIC - physical combat for Morrowind (OpenMW). Portable: settings, saves and
rem logs stay in this folder (config\, userdata\). Needs Morrowind's "Data Files".
set "KINETIC=%~dp0"
set "MWDATA="
if exist "%KINETIC%morrowind-data.txt" set /p MWDATA=<"%KINETIC%morrowind-data.txt"
if not defined MWDATA goto ask
if not exist "%MWDATA%\Morrowind.esm" goto ask
goto run
:ask
echo Where is Morrowind's "Data Files" folder? (the one with Morrowind.esm)
echo Example: C:\Program Files (x86)\Steam\steamapps\common\Morrowind\Data Files
set /p MWDATA=Path:
if not exist "%MWDATA%\Morrowind.esm" (
    echo Morrowind.esm was not found in "%MWDATA%".
    pause
    exit /b 1
)
> "%KINETIC%morrowind-data.txt" echo %MWDATA%
:run
rem The physical body and its tuning. In-game options: Options ^> Scripts ^> KINETIC.
set "OPENMW_RAGDOLL_SETTINGS=%KINETIC%ragdoll\ragdoll.cfg"
set "OPENMW_RAGDOLL_ACTIVE_BODY=%KINETIC%ragdoll\active-body.cfg"
set "OPENMW_KINETIC_SWITCHES=%KINETIC%kinetic-switches.cfg"
set "ARCHIVES=--fallback-archive=Morrowind.bsa"
set "CONTENT=--content=Morrowind.esm"
if exist "%MWDATA%\Tribunal.esm" (
    set "ARCHIVES=%ARCHIVES% --fallback-archive=Tribunal.bsa"
    set "CONTENT=%CONTENT% --content=Tribunal.esm"
)
if exist "%MWDATA%\Bloodmoon.esm" (
    set "ARCHIVES=%ARCHIVES% --fallback-archive=Bloodmoon.bsa"
    set "CONTENT=%CONTENT% --content=Bloodmoon.esm"
)
rem Your mods (mods.cfg: openmw.cfg's data= and content= lines, one per line, # = off).
rem The first run copies the ones in your OpenMW config there, switched off: mods made
rem for another OpenMW version or that change combat may not work with KINETIC.
set "OPENMWCFG=%USERPROFILE%\Documents\My Games\OpenMW\openmw.cfg"
if not exist "%KINETIC%mods.cfg" (
    echo # KINETIC mods: openmw.cfg data= and content= lines. Remove the # to load one.
    echo # Morrowind, Tribunal and Bloodmoon are loaded already.
    if exist "%OPENMWCFG%" for /f "usebackq tokens=1,* delims==" %%a in ("%OPENMWCFG%") do (
        if /i "%%a"=="data" echo #data=%%b
        if /i "%%a"=="content" echo #content=%%b
    )
) > "%KINETIC%mods.cfg"
set "MODS="
for /f "usebackq eol=# tokens=1,* delims==" %%a in ("%KINETIC%mods.cfg") do call :mod "%%a" "%%~b"
if not exist "%KINETIC%config" mkdir "%KINETIC%config"
if not exist "%KINETIC%userdata" mkdir "%KINETIC%userdata"
rem Bodies go limp on death (OpenMW's passive ragdolls, off by default; user 10-30:
rem "the enemy npcs are not turning into ragdolls on death").
findstr /c:"enable passive ragdolls" "%KINETIC%config\settings.cfg" >nul 2>&1 || (echo [Game]& echo enable passive ragdolls = true)>>"%KINETIC%config\settings.cfg"
rem A development package (package-kinetic.ps1 -Record) records each session like the
rem hub: recordings.txt names the folder; B bookmarks a moment with a note.
set "RECORDING="
if not exist "%KINETIC%recordings.txt" goto play
set /p RECORDINGS=<"%KINETIC%recordings.txt"
for /f %%t in ('powershell -NoProfile -Command "[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')"') do set "STAMP=%%t"
set "RECORDING=%RECORDINGS%\runtime-GAME-manual-playthrough-%STAMP%"
set "OPENMW_MANUAL_DIAGNOSTICS=%RECORDING%\manual-recording"
set OPENMW_RAGDOLL_FOOTFALL_TRACE=1
set OPENMW_RAGDOLL_DAMAGE_TRACE=1
set OPENMW_RAGDOLL_SWING_TRACE=1
echo Recording: %RECORDING%
echo B bookmarks a moment and opens a note box.
:play
"%KINETIC%openmw.exe" --replace=config "--config=%KINETIC%config" --replace=user-data "--user-data=%KINETIC%userdata" ^
    "--data-local=%KINETIC%userdata\data" "--resources=%KINETIC%resources" --replace=content --replace=fallback-archive ^
    "--data=%MWDATA%" %ARCHIVES% %CONTENT% %MODS% "--data=%KINETIC%data" --content=kinetic.omwscripts %*
set "EXITCODE=%errorlevel%"
rem openmw.log is rewritten on every launch: the recording keeps its own copy.
if defined RECORDING copy /y "%KINETIC%config\openmw.log" "%RECORDING%\openmw.log" >nul
if %EXITCODE% geq 1 pause
exit /b

:mod
if /i "%~1"=="data" if /i not "%~2"=="%MWDATA%" set MODS=%MODS% "--data=%~2"
if /i not "%~1"=="content" exit /b
for %%f in (Morrowind.esm Tribunal.esm Bloodmoon.esm) do if /i "%~2"=="%%f" exit /b
set MODS=%MODS% "--content=%~2"
exit /b

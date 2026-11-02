@echo off
setlocal EnableExtensions
rem KINETIC duel (NET-01): a 1v1 with a friend over the internet or a LAN.
rem One of you hosts, the other joins with the host's address. Over the internet
rem both install Tailscale (free) and use the host's Tailscale address.
set "KINETIC=%~dp0"
echo.
echo   KINETIC duel - 1v1 with a friend
echo.
echo   1  Host  (your friend joins you)
echo   2  Join  (you join your friend)
echo.
choice /c 12 /n /m "Host or join? [1/2] "
if errorlevel 2 goto join
set "OPENMW_KINETIC_DUEL=host:27015"
echo.
echo Hosting on UDP port 27015. Tell your friend your address:
echo   Tailscale: run "tailscale ip -4" (or see the Tailscale menu); LAN: run "ipconfig".
echo If Windows asks whether openmw.exe may use the network, allow it.
goto name
:join
echo.
set "ADDRESS="
set /p ADDRESS=Your friend's address (for example 100.101.102.103):
if not defined ADDRESS goto join
set "OPENMW_KINETIC_DUEL=%ADDRESS%"
:name
set "OPENMW_KINETIC_DUEL_NAME="
set /p OPENMW_KINETIC_DUEL_NAME=Your name:
if not defined OPENMW_KINETIC_DUEL_NAME set "OPENMW_KINETIC_DUEL_NAME=%USERNAME%"
call "%KINETIC%KINETIC.cmd" --content=ragdoll_test_area.omwaddon "--start=OpenMW Ragdoll Test Floor" --skip-menu=1 --new-game=0 %*

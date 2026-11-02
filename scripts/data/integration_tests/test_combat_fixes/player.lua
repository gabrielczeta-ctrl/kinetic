local core = require('openmw.core')
local camera = require('openmw.camera')
local interfaces = require('openmw.interfaces')
local self = require('openmw.self')
local testing = require('testing_util')
local types = require('openmw.types')
local util = require('openmw.util')

local pending
local pendingFrames = 0
local cameraReady = false
local handlers = {}
local inspectCamera, savedCamera
local aimPoint, aimFrames
local function updateContactAim()
    if not aimPoint then return end
    aimFrames = aimFrames + 1
    if aimFrames % 3 ~= 0 then return end
    local delta = aimPoint - camera.getPosition()
    local yaw = math.atan2(delta.x, delta.y)
    local pitch = math.atan2(-delta.z, math.sqrt(delta.x * delta.x + delta.y * delta.y))
    local yawError = (yaw - camera.getYaw() + math.pi) % (2 * math.pi) - math.pi
    local pitchError = pitch - camera.getPitch()
    local function pixels(value) return math.floor(math.max(-64, math.min(64, value * 256)) + 0.5) end
    local x, y = pixels(yawError), pixels(pitchError)
    local viewport = camera.worldToViewportVector(aimPoint)
    local screen = require('openmw.ui').screenSize()
    if math.abs(yawError) < 0.006 and math.abs(pitchError) < 0.006
        and math.abs(viewport.x - screen.x / 2) < 4 and math.abs(viewport.y - screen.y / 2) < 4 then
        print('PLAY04_AIM_READY point=' .. tostring(aimPoint)
            .. ' viewport=' .. tostring(viewport))
        core.sendGlobalEvent('fixAimReady', true)
        aimPoint = nil
    else
        core.sendGlobalEvent('fixAimMotion', { x = x, y = y })
    end
end
local function updateInspectCamera()
    if not inspectCamera then return end
    local pitch, yaw = inspectCamera.pitch, inspectCamera.yaw
    local forward = util.vector3(math.sin(yaw) * math.cos(pitch),
        math.cos(yaw) * math.cos(pitch), -math.sin(pitch))
    camera.setMode(camera.MODE.Static, true)
    camera.setStaticPosition(inspectCamera.center - forward * inspectCamera.distance)
    camera.setPitch(pitch)
    camera.setYaw(yaw)
    camera.setExtraPitch(0)
    camera.setExtraYaw(0)
end
handlers.fixInspectCamera = function(data)
    if data.center then
        if not savedCamera then
            savedCamera = { mode = camera.getMode(), pitch = camera.getPitch(), yaw = camera.getYaw() }
            -- Static mode retains the previous first-person visibility flag.
            -- Enter third person first so the whole actor is rendered.
            camera.setMode(camera.MODE.ThirdPerson, true)
        end
        inspectCamera = data
        updateInspectCamera()
    else
        inspectCamera = nil
        if savedCamera then
            camera.setMode(savedCamera.mode, true)
            camera.setPitch(savedCamera.pitch)
            camera.setYaw(savedCamera.yaw)
            camera.setExtraPitch(0)
            camera.setExtraYaw(0)
            savedCamera = nil
        end
    end
end
handlers.fixPoseProjection = function(data)
    print('PLAY04_PROJECT_CAMERA position=' .. tostring(camera.getPosition())
        .. ' yaw=' .. camera.getYaw() .. ' pitch=' .. camera.getPitch())
    for name, point in pairs(data) do
        print('PLAY04_PROJECT ' .. name .. '=' .. tostring(camera.worldToViewportVector(point)))
    end
end
handlers.fixPlayerStance = function(drawn)
    types.Actor.setStance(self, drawn and types.Actor.STANCE.Weapon or types.Actor.STANCE.Nothing)
end
handlers.fixAimAt = function(point)
    aimPoint, aimFrames = point, 0
end
-- GRIP-02: the first-person view looking down (radians, positive = down).
handlers.fixPlayerPitch = function(pitch) camera.setPitch(pitch) end
handlers.fixPlayerCamera = function(firstPerson)
    camera.setMode(firstPerson and camera.MODE.FirstPerson or camera.MODE.ThirdPerson)
    camera.setPitch(0)
    camera.setYaw(0)
    core.sendGlobalEvent('fixCameraReady', camera.getMode() == camera.MODE.FirstPerson)
end
for name, handler in pairs(testing.localEventHandlers) do handlers[name] = handler end
handlers.fixWoundCamera = function(data)
    if data.position then
        camera.setMode(camera.MODE.Static)
        camera.setStaticPosition(data.position)
        camera.setPitch(data.pitch or 0) -- positive looks down (WOUNDFX-01 lying shot)
        camera.setYaw(data.yaw or 0)
        camera.setFieldOfView(math.rad(45))
    else
        camera.setMode(camera.MODE.ThirdPerson)
        camera.setPreferredThirdPersonDistance(180)
        camera.setFieldOfView(math.rad(75))
    end
    cameraReady = true
end
-- KINETIC-01: the options page's switches file (openmw.kinetic).
handlers.fixKineticCheck = function()
    local kinetic = require('openmw.kinetic')
    local wrote = kinetic.setSwitch('OPENMW_RAGDOLL_HEAD_DAZE', '0')
    local refused = kinetic.setSwitch('OPENMW_RAGDOLL_BOGUS', '1')
    local unsafe = kinetic.setSwitch('OPENMW_RAGDOLL_SPARKS', 'x;y')
    local saved = kinetic.switches()['OPENMW_RAGDOLL_HEAD_DAZE'].saved
    local cleared = kinetic.setSwitch('OPENMW_RAGDOLL_HEAD_DAZE', nil)
    local after = kinetic.switches()['OPENMW_RAGDOLL_HEAD_DAZE'].saved
    core.sendGlobalEvent('fixKineticResult', { wrote = wrote, refused = refused, unsafe = unsafe, saved = saved,
        cleared = cleared, after = after })
end
-- SLIP-01: set every melee weapon skill.
handlers.fixSetWeaponSkill = function(value)
    for _, name in ipairs({ 'longblade', 'bluntweapon', 'axe', 'spear', 'shortblade', 'handtohand' }) do
        types.NPC.stats.skills[name](self).base = value
    end
end
-- SHIP-02: walk the navmesh route to `dest`, then these extra points.
local playerRoute, playerRouteIndex = nil, 1
handlers.fixPlayerRoute = function(data)
    local status, path = require('openmw.nearby').findPath(self.position, data.dest)
    local points = {}
    for _, p in ipairs(path or {}) do points[#points + 1] = p end
    for _, p in ipairs(data.extra or {}) do points[#points + 1] = p end
    -- Start from point `from` (SHIP-02: past the chargen name trigger).
    playerRouteIndex = math.max(1, math.min(data.from or 1, #points))
    playerRoute = #points > 0 and points or nil
    core.sendGlobalEvent('fixPlayerRouteStart', { points = #points, status = tostring(status), list = points })
end
handlers.fixPlayerRouteStop = function()
    playerRoute = nil
    self.controls.movement = 0
end
-- RIPOSTE-02: the player back to full health between bouts.
handlers.fixHealPlayer = function()
    local health = types.Actor.stats.dynamic.health(self)
    health.current = health.base
end
-- FATIGUE-01: set the player's fatigue to a share of its base.
handlers.fixSetFatigue = function(share)
    local fatigue = types.Actor.stats.dynamic.fatigue(self)
    fatigue.current = fatigue.base * share
end
-- SWIM-01: walk/swim forward (1) or stand (0).
local moveForward = 0
local lastSwimming = nil
handlers.fixPlayerMove = function(value)
    moveForward = value
    -- onUpdate writes the movement only while it is non-zero: stop it here.
    if value == 0 then self.controls.movement = 0 end
end
-- REPLAY-01 (2026-10-05): the user's recorded movement keys, turn and look
-- pitch, step by step (the flips came only in play, moving 79% of the time).
local replayControls = nil
handlers.fixPlayerControls = function(data) replayControls = data end
handlers.fixPlayerConfigure = function(data)
    camera.setMode(data.firstPerson and camera.MODE.FirstPerson or camera.MODE.ThirdPerson)
    camera.setPreferredThirdPersonDistance(180)
    camera.setPitch(data.pitch or (data.firstPerson and 0 or 0.15))
    camera.setYaw(0)
    cameraReady = true
    interfaces.Controls.overrideMovementControls(true)
    local equipment = {}
    if data.weapon then
        equipment[types.Actor.EQUIPMENT_SLOT.CarriedRight] = types.Actor.inventory(self):find(data.weapon)
    end
    -- RANGED-01 / SPELL-01: skilled enough that vanilla's rolls land.
    if data.ammo or data.spell or data.marksman then
        types.NPC.stats.skills.marksman(self).base = 100
        types.NPC.stats.skills.destruction(self).base = 100
    end
    -- RANGED-01 / SHIELD-01: ammunition and a shield.
    if data.ammo then
        equipment[types.Actor.EQUIPMENT_SLOT.Ammunition] = types.Actor.inventory(self):find(data.ammo)
    end
    if data.shield then
        equipment[types.Actor.EQUIPMENT_SLOT.CarriedLeft] = types.Actor.inventory(self):find(data.shield)
    end
    -- FPBODY-01: the player's own field of view (degrees, vertical), and
    -- clothes and armour by slot ({ [slot] = record id }).
    if data.fov then camera.setFieldOfView(math.rad(data.fov)) end
    for slot, id in pairs(data.wear or {}) do
        equipment[types.Actor.EQUIPMENT_SLOT[slot]] = types.Actor.inventory(self):find(id)
    end
    types.Actor.setEquipment(self, equipment)
    -- SPELL-01: a known, selected spell and the spell stance.
    if data.spell then
        types.Actor.spells(self):add(data.spell)
        types.Actor.setSelectedSpell(self, data.spell)
        local magicka = types.Actor.stats.dynamic.magicka(self)
        magicka.base = 500
        magicka.current = 500
        types.Actor.setStance(self, types.Actor.STANCE.Spell)
    elseif data.sheathed then
        types.Actor.setStance(self, types.Actor.STANCE.Nothing)
    else
        types.Actor.setStance(self, types.Actor.STANCE.Weapon)
    end
    pending = data
end
-- REPLAY-07: draw the weapon equipped sheathed
local closeMenus = false
handlers.fixCloseMenus = function(on) closeMenus = on end
handlers.fixPlayerDraw = function() types.Actor.setStance(self, types.Actor.STANCE.Weapon) end
return {
    eventHandlers = handlers,
    engineHandlers = {
        onFrame = function()
            -- SHIP-01: a scripted greeting or message box (the ship's chargen
            -- scripts) pauses the game; closed as soon as it opens.
            if closeMenus and interfaces.UI.getMode() ~= nil then
                print('SHIP01 closed menu ' .. tostring(interfaces.UI.getMode()))
                interfaces.UI.setMode()
            end
            updateInspectCamera()
            updateContactAim()
        end,
        onUpdate = function()
            if not cameraReady then
                camera.setMode(camera.MODE.ThirdPerson)
                camera.setPreferredThirdPersonDistance(180)
                camera.setPitch(0.15)
                camera.setYaw(0)
                cameraReady = true
            end
            if moveForward ~= 0 then self.controls.movement = moveForward end
            -- SHIP-02: the player walks a route (as stance.lua's ROUGH-01 walker).
            if playerRoute then
                local point = playerRoute[playerRouteIndex]
                while point and util.vector3(point.x - self.position.x, point.y - self.position.y, 0):length() < 45
                    and math.abs(point.z - self.position.z) < 60 do
                    playerRouteIndex = playerRouteIndex + 1
                    point = playerRoute[playerRouteIndex]
                end
                if not point then
                    playerRoute = nil
                    self.controls.movement = 0
                    core.sendGlobalEvent('fixPlayerRouteDone', true)
                else
                    local wanted = math.atan2(point.x - self.position.x, point.y - self.position.y)
                    local turn = (wanted - self.rotation:getYaw() + math.pi) % (2 * math.pi) - math.pi
                    self.controls.yawChange = math.max(-0.08, math.min(0.08, turn))
                    self.controls.movement = math.abs(turn) < 1.2 and 1 or 0
                end
            end
            if replayControls then
                self.controls.movement = replayControls.forward or 0
                self.controls.sideMovement = replayControls.side or 0
                self.controls.run = (replayControls.run or 0) > 0
                if replayControls.yaw then
                    self.controls.yawChange = replayControls.yaw
                    replayControls.yaw = nil
                end
                if replayControls.pitch then camera.setPitch(replayControls.pitch) end
                if replayControls.stop then replayControls = nil end
            end
            local swimming = types.Actor.isSwimming(self)
            if swimming ~= lastSwimming then
                lastSwimming = swimming
                core.sendGlobalEvent('fixSwimming', swimming)
            end
            if not pending then return end
            local equipped = types.Actor.getEquipment(self, types.Actor.EQUIPMENT_SLOT.CarriedRight)
            local spellReady = pending.spell and types.Actor.getStance(self) == types.Actor.STANCE.Spell
            if pending.spell and not spellReady then
                pendingFrames = (pendingFrames or 0) + 1
                if pendingFrames % 60 == 1 then
                    local selected = types.Actor.getSelectedSpell(self)
                    print(string.format('SPELL-01 stance=%s selected=%s', tostring(types.Actor.getStance(self)),
                        tostring(selected and selected.id)))
                    types.Actor.setStance(self, types.Actor.STANCE.Spell)
                end
            end
            local wantStance = pending.sheathed and types.Actor.STANCE.Nothing or types.Actor.STANCE.Weapon
            if spellReady or (types.Actor.getStance(self) == wantStance
                and ((not pending.weapon and not equipped)
                    or (equipped and equipped.recordId == pending.weapon))) then
                core.sendGlobalEvent('fixPlayerReady', { sequence = pending.sequence })
                pending = nil
            end
        end,
    },
}

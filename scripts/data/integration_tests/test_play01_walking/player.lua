local camera = require('openmw.camera')
local core = require('openmw.core')
local interfaces = require('openmw.interfaces')
local nearby = require('openmw.nearby')
local self = require('openmw.self')
local testing = require('testing_util')
local types = require('openmw.types')
local util = require('openmw.util')
local ui = require('openmw.ui')
local command

-- Film mode: a barcode at the top right shows the simulation time each frame
-- (two reference squares, white then black, then 24 bits of centiseconds,
-- most significant first). It is drawn in the same frame as the world, so
-- tools/body/film.py times every still by it - the screenshot key captures a
-- later frame and drops captures while one is pending, so no clock outside
-- the picture says which moment a still shows.
local BAR_BITS, BAR_PITCH, BAR_SIZE = 24, 10, 8
local barSquare = ui.texture { path = 'textures/ragdoll_hud_square.png' }
local barcode
local function showBarcode(simulationTime)
    if not barcode then
        local content = {}
        for k = 0, BAR_BITS + 1 do
            content[#content + 1] = { type = ui.TYPE.Image, props = { resource = barSquare,
                position = util.vector2(k * BAR_PITCH, 0), size = util.vector2(BAR_SIZE, BAR_SIZE),
                color = util.color.rgb(k == 0 and 1 or 0, k == 0 and 1 or 0, k == 0 and 1 or 0) } }
        end
        barcode = ui.create { layer = 'HUD', type = ui.TYPE.Widget, props = {
            relativePosition = util.vector2(1, 0), anchor = util.vector2(1, 0), position = util.vector2(-6, 6),
            size = util.vector2((BAR_BITS + 2) * BAR_PITCH, BAR_SIZE) }, content = ui.content(content) }
    end
    local value = math.floor(simulationTime * 100 + 0.5) % (2 ^ BAR_BITS)
    for k = 1, BAR_BITS do
        local bit = math.floor(value / 2 ^ (BAR_BITS - k)) % 2
        barcode.layout.content[k + 2].props.color = util.color.rgb(bit, bit, bit)
    end
    barcode:update()
end

local function prepareOrdinaryControls(weaponRecordId, sheathed, leftRecordId, firstPerson)
    testing.expect(interfaces.Controls ~= nil, 'Controls interface is missing')
    local equipment = types.Actor.getEquipment(self)
    equipment[types.Actor.EQUIPMENT_SLOT.CarriedRight]
        = weaponRecordId and types.Actor.inventory(self):find(weaponRecordId) or nil
    equipment[types.Actor.EQUIPMENT_SLOT.CarriedLeft]
        = leftRecordId and types.Actor.inventory(self):find(leftRecordId) or nil
    types.Actor.setEquipment(self, equipment)
    if weaponRecordId and not sheathed then types.Actor.setStance(self, types.Actor.STANCE.Weapon) end
    interfaces.Controls.overrideMovementControls(true)
    command = { segment = 'setup', movement = 0, sideMovement = 0, yawRate = 0 }
    camera.setMode(firstPerson and camera.MODE.FirstPerson or camera.MODE.ThirdPerson)
    camera.setYaw(self.rotation:getYaw())
    camera.setPreferredThirdPersonDistance(260)
    camera.setPitch(0.25)
    -- The flat floor, unless the case starts outside on terrain (Balmora)
    -- or in a Morrowind interior (RAMP-01).
    if self.cell.isExterior or self.cell.name ~= 'OpenMW Ragdoll Test Floor' then return end
    local hit = nearby.castRay(util.vector3(0, 0, 256), util.vector3(0, 0, -64), {
        collisionType = nearby.COLLISION_TYPE.World + nearby.COLLISION_TYPE.HeightMap,
    })
    testing.expect(hit.hit and hit.hitObject
        and hit.hitObject.recordId == 'openmw_ragdoll_test_floor', 'PLAY-01 floor is missing')
    testing.expectLessOrEqual(math.abs(hit.hitPos.z), 0.01, 'PLAY-01 floor is not at z=0')
end

testing.registerLocalTestStep('PLAY-01 prepare ordinary controls', function()
    prepareOrdinaryControls(nil)
end)

testing.registerLocalTestStep('PLAY-01 prepare ordinary controls with warhammer', function()
    testing.expect(types.Actor.inventory(self):find('iron warhammer') ~= nil,
        'PLAY-01 player has no warhammer')
    prepareOrdinaryControls('iron warhammer')
end)

testing.registerLocalTestStep('PLAY-01 prepare ordinary controls with fists raised', function()
    prepareOrdinaryControls(nil)
    types.Actor.setStance(self, types.Actor.STANCE.Weapon)
end)

testing.registerLocalTestStep('PLAY-01 prepare ordinary controls with dagger', function()
    testing.expect(types.Actor.inventory(self):find('iron dagger') ~= nil, 'PLAY-01 player has no dagger')
    prepareOrdinaryControls('iron dagger')
end)

testing.registerLocalTestStep('PLAY-01 prepare ordinary controls with claymore', function()
    testing.expect(types.Actor.inventory(self):find('iron claymore') ~= nil, 'PLAY-01 player has no claymore')
    prepareOrdinaryControls('iron claymore')
end)

testing.registerLocalTestStep('PLAY-01 prepare ordinary controls with longsword', function()
    testing.expect(types.Actor.inventory(self):find('iron longsword') ~= nil,
        'PLAY-01 player has no longsword')
    prepareOrdinaryControls('iron longsword')
end)

testing.registerLocalTestStep('PLAY-01 prepare ordinary controls with sheathed battle axe', function()
    testing.expect(types.Actor.inventory(self):find('iron battle axe') ~= nil,
        'PLAY-01 player has no battle axe')
    prepareOrdinaryControls('iron battle axe', true)
end)

-- TORCH-01 (user 10-31: "the left hand holding the torch shakes ... like it
-- has parkinsons" sheathed; drawn it "puts the torch very close to the chest").
testing.registerLocalTestStep('PLAY-01 prepare ordinary controls with torch', function()
    testing.expect(types.Actor.inventory(self):find('torch') ~= nil, 'PLAY-01 player has no torch')
    prepareOrdinaryControls('iron longsword', true, 'torch')
end)

testing.registerLocalTestStep('PLAY-01 prepare ordinary controls with drawn torch', function()
    testing.expect(types.Actor.inventory(self):find('torch') ~= nil, 'PLAY-01 player has no torch')
    prepareOrdinaryControls('iron longsword', false, 'torch')
end)

testing.registerLocalTestStep('PLAY-01 prepare ordinary controls with bare torch', function()
    testing.expect(types.Actor.inventory(self):find('torch') ~= nil, 'PLAY-01 player has no torch')
    prepareOrdinaryControls(nil, true, 'torch')
end)

testing.registerLocalTestStep('PLAY-01 prepare ordinary controls with first person torch', function()
    testing.expect(types.Actor.inventory(self):find('torch') ~= nil, 'PLAY-01 player has no torch')
    prepareOrdinaryControls('iron longsword', true, 'torch', true)
end)

testing.registerLocalTestStep('PLAY-01 prepare ordinary controls with claymore torch', function()
    testing.expect(types.Actor.inventory(self):find('torch') ~= nil, 'PLAY-01 player has no torch')
    prepareOrdinaryControls('iron claymore', true, 'torch')
end)

-- Fatigue below zero knocks Morrowind's actor out (its knockout clip lies
-- down until fatigue is back); nil restores it.
local ownAcrobatics
testing.localEventHandlers.play01Acrobatics = function(value)
    local acrobatics = types.NPC.stats.skills.acrobatics(self)
    ownAcrobatics = ownAcrobatics or acrobatics.base
    acrobatics.base = value or ownAcrobatics
    print(string.format('PLAY01_ACROBATICS base=%d', acrobatics.base))
end
testing.localEventHandlers.play01Fatigue = function(value)
    local fatigue = types.Actor.stats.dynamic.fatigue(self)
    if type(value) == 'table' then
        -- SAVE-01 tired cases: a share of the base.
        fatigue.current = fatigue.base * value.share
        return
    end
    fatigue.current = value or fatigue.base
end

-- Morrowind's own stagger, as its combat script applies it to a hit
-- that is not physical (creature bites, arrows, spells).
testing.localEventHandlers.play01Stagger = function(kind)
    if kind == 'knockdown' then
        types.Actor.setKnockedDown(self, true)
    else
        types.Actor.setHitRecovery(self, true)
    end
end

testing.localEventHandlers.play01Travel = function(value)
    if value.enabled == false then
        command = nil
    else
        command = value
    end
    if not command then
        self.controls.movement, self.controls.sideMovement, self.controls.yawChange = 0, 0, 0
        self.controls.run, self.controls.jump = false, false
        self.controls.sneak = false
        interfaces.Controls.overrideMovementControls(false)
    end
end

-- RAMP-01: the navmesh route an escort's AI travel follows (printed).
testing.localEventHandlers.play01Path = function(data)
    local status, path = nearby.findPath(data.from, data.to)
    local points = {}
    for _, p in ipairs(path or {}) do points[#points + 1] = string.format('%.0f,%.0f,%.0f', p.x, p.y, p.z) end
    print(string.format('PLAY01_PATH status=%s points=%s', tostring(status), table.concat(points, ' ')))
end
-- RAMP-01: the floor heights across the ramp (rows of y, columns of x).
testing.localEventHandlers.play01Ground = function(data)
    for _, y in ipairs(data.ys) do
        local row = {}
        for x = data.x0, data.x1, data.dx do
            local hit = nearby.castRay(util.vector3(x, y, 300), util.vector3(x, y, -300),
                { collisionType = nearby.COLLISION_TYPE.World + nearby.COLLISION_TYPE.HeightMap })
            row[#row + 1] = hit.hit and string.format('%4.0f', hit.hitPos.z) or ' ---'
        end
        print(string.format('PLAY01_GROUND y=%d x=%d..%d %s', y, data.x0, data.x1, table.concat(row, ' ')))
    end
end

return {
    engineHandlers = {
    -- A Morrowind interior's scripted greeting or message box (the prison
    -- ship's chargen, RAMP-01) pauses the game; closed as soon as it opens.
    onFrame = function()
        if not self.cell.isExterior and self.cell.name ~= 'OpenMW Ragdoll Test Floor'
            and interfaces.UI.getMode() ~= nil then
            print('PLAY01 closed menu ' .. tostring(interfaces.UI.getMode()))
            interfaces.UI.setMode()
        end
    end,
    onUpdate = function(dt)
        testing.updateLocal(dt)
        if not command then return end
        if command.film then showBarcode(core.getSimulationTime()) end
        -- Ordinary digital WASD values. The adapter owns speed selection,
        -- acceleration, travel direction and the independent facing request.
        self.controls.movement = command.movement or 0
        self.controls.sideMovement = command.sideMovement or 0
        self.controls.yawChange = (command.yawRate or 0) * dt
        self.controls.run, self.controls.jump = command.run == true, command.jump == true
        self.controls.sneak = command.sneak == true
        -- Visual review: a camera that tracks the player from the side
        -- (sideCamera = yaw offset from the facing, radians) or ahead.
        -- cameraDistance / cameraHeight / cameraPitch override the side
        -- camera (an overhead view: short distance, high, pitched down).
        if command.sideCamera then
            local yaw = self.rotation:getYaw() + command.sideCamera
            local toCamera = util.vector3(-math.sin(yaw), -math.cos(yaw), 0)
            camera.setMode(camera.MODE.Static)
            camera.setStaticPosition(self.position + toCamera * (command.cameraDistance or 230)
                + util.vector3(0, 0, command.cameraHeight or 70))
            camera.setYaw(yaw)
            camera.setPitch(command.cameraPitch or 0.12)
        elseif camera.getMode() == camera.MODE.Static then
            camera.setMode(camera.MODE.ThirdPerson)
        end
        if command.stance then
            local wanted = command.stance == 'weapon' and types.Actor.STANCE.Weapon or types.Actor.STANCE.Nothing
            if types.Actor.getStance(self) ~= wanted then types.Actor.setStance(self, wanted) end
        end
        core.sendGlobalEvent('play01InputSample', {
            segment = command.segment, simulation_time = core.getSimulationTime(),
            real_time = core.getRealTime(), dt = dt,
            movement = self.controls.movement, side_movement = self.controls.sideMovement,
            yaw_change = self.controls.yawChange, run = self.controls.run,
            jump = self.controls.jump,
            camera_yaw = camera.getYaw(), gameplay_yaw = self.rotation:getYaw(),
            camera_pitch = camera.getPitch(), camera_position = camera.getPosition(),
            camera_distance = camera.getThirdPersonDistance(),
            right_equipped = types.Actor.getEquipment(self, types.Actor.EQUIPMENT_SLOT.CarriedRight) ~= nil,
            left_equipped = types.Actor.getEquipment(self, types.Actor.EQUIPMENT_SLOT.CarriedLeft) ~= nil,
        })
    end },
    eventHandlers = testing.localEventHandlers,
}

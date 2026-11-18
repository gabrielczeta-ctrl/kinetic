-- The pilot's player: holds the controls the pilot's last action set
-- (pilot_global.lua sends them), every frame, as controls set once do not hold.
local camera = require('openmw.camera')
local core = require('openmw.core')
local interfaces = require('openmw.interfaces')
local nearby = require('openmw.nearby')
local self = require('openmw.self')
local testing = require('testing_util')
local types = require('openmw.types')
local util = require('openmw.util')

local enabled = false
local ready = false
local controls = { fwd = 0, side = 0, run = false, jump = false, face = false, turn = 0, steps = 1 }

local function nearestActor()
    local nearest, distance
    for _, actor in ipairs(nearby.actors) do
        if actor.id ~= self.id then
            local d = (actor.position - self.position):length()
            if not distance or d < distance then nearest, distance = actor, d end
        end
    end
    return nearest
end

local function configure(data)
    interfaces.Controls.overrideMovementControls(true)
    camera.setMode(camera.MODE.ThirdPerson)
    camera.setPreferredThirdPersonDistance(320)
    camera.setPitch(0.2)
    if data.weapon and data.weapon ~= 'none' then
        local weapon = types.Actor.inventory(self):find(data.weapon)
        testing.expect(weapon ~= nil, 'PILOT player has no ' .. data.weapon)
        types.Actor.setEquipment(self, { [types.Actor.EQUIPMENT_SLOT.CarriedRight] = weapon })
    end
    types.Actor.setStance(self, types.Actor.STANCE.Weapon)
    local health = types.Actor.stats.dynamic.health(self)
    health.base = data.health or health.base
    health.current = health.base
    enabled, ready = true, false
end

local function update()
    if not enabled then return end
    if not ready then
        if types.Actor.getStance(self) ~= types.Actor.STANCE.Weapon then return end
        ready = true
        core.sendGlobalEvent('pilotPlayerReady')
    end
    self.controls.movement = controls.fwd
    self.controls.sideMovement = controls.side
    self.controls.run = controls.run
    self.controls.jump = controls.jump
    local yaw = controls.turn / math.max(1, controls.steps)
    if controls.face then
        local target = nearestActor()
        if target then
            local direction = target.position - self.position
            if direction:length() > 0.001 then
                yaw = util.normalizeAngle(math.atan2(direction.x, direction.y) - self.rotation:getYaw())
            end
        end
    end
    self.controls.yawChange = yaw
end

local handlers = {}
for name, handler in pairs(testing.localEventHandlers) do handlers[name] = handler end
handlers.pilotPlayerConfigure = configure
handlers.pilotControls = function(data) controls = data end
handlers.pilotPlayerStop = function()
    enabled = false
    self.controls.movement = 0
    self.controls.sideMovement = 0
    self.controls.run = false
    self.controls.jump = false
    interfaces.Controls.overrideMovementControls(false)
end

return {
    engineHandlers = { onUpdate = update },
    eventHandlers = handlers,
}

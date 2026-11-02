local core = require('openmw.core')
local camera = require('openmw.camera')
local interfaces = require('openmw.interfaces')
local self = require('openmw.self')
local testing = require('testing_util')
local types = require('openmw.types')

local readyAt = 0
local function configure()
    camera.setMode(camera.MODE.ThirdPerson)
    readyAt = core.getSimulationTime() + 0.25
    interfaces.Controls.overrideMovementControls(true)
    local weapon = types.Actor.inventory(self):find('iron warhammer')
    testing.expect(weapon ~= nil, 'WPN-01 player has no equipped test weapon')
    types.Actor.setEquipment(self, { [types.Actor.EQUIPMENT_SLOT.CarriedRight] = weapon })
    types.Actor.setStance(self, types.Actor.STANCE.Weapon)
end

local function update()
    local weapon = types.Actor.getEquipment(self, types.Actor.EQUIPMENT_SLOT.CarriedRight)
    if weapon and types.Actor.getStance(self) == types.Actor.STANCE.Weapon
        and core.getSimulationTime() >= readyAt then
        core.sendGlobalEvent('weaponPlayerReady', { weapon = weapon.recordId })
        update = function() end
    end
end

local handlers = {}
for name, handler in pairs(testing.localEventHandlers) do handlers[name] = handler end
handlers.weaponPlayerConfigure = configure

return {
    engineHandlers = { onUpdate = function() update() end },
    eventHandlers = handlers,
}

local camera = require('openmw.camera')
local core = require('openmw.core')
local input = require('openmw.input')
local self = require('openmw.self')
local types = require('openmw.types')
local util = require('openmw.util')
local readyAt
return {
    engineHandlers = {
        onKeyPress = function(key)
            if key.code == input.KEY.F6 or key.code == input.KEY.F7 then
                core.sendGlobalEvent('woundDuelAction', { toggle = key.code == input.KEY.F7 })
            end
        end,
        onUpdate = function()
            if not readyAt or core.getSimulationTime() < readyAt then return end
            readyAt = nil
            core.sendGlobalEvent('woundDuelPlayerReady', {
                forward = camera.viewportToWorldVector(util.vector2(0.5, 0.5)),
            })
        end,
    },
    eventHandlers = {
        woundDuelPrepare = function(data)
            -- Expose the skin for this cosmetic review; items stay in inventory.
            local equipment = {}
            if not data.unarmed then
                equipment[types.Actor.EQUIPMENT_SLOT.CarriedRight] = types.Actor.inventory(self):find(data.weapon or 'iron dagger')
            end
            types.Actor.setEquipment(self, equipment)
            local health = types.Actor.stats.dynamic.health(self)
            health.base, health.current = 40, 40
            local fatigue = types.Actor.stats.dynamic.fatigue(self)
            fatigue.current = fatigue.base
            readyAt = core.getSimulationTime() + 0.5
        end,
    },
}

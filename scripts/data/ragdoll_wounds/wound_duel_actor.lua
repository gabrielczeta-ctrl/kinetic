local self = require('openmw.self')
local types = require('openmw.types')
local script = require('ragdoll_manual_duelist')
local configure = script.engineHandlers.onInit
local unarmed, ready, weapon = false, false, 'iron dagger'
script.engineHandlers.onInit = function(data)
    unarmed = data.unarmed
    weapon = data.weapon or 'iron dagger'
    configure(data)
    self:enableAI(false)
end
script.engineHandlers.onUpdate = function()
    if ready then return end
    local item = types.Actor.getEquipment(self, types.Actor.EQUIPMENT_SLOT.CarriedRight)
    if ((unarmed and not item) or (not unarmed and item and item.recordId == weapon))
        and types.Actor.getStance(self) == types.Actor.STANCE.Weapon then
        ready = true
    else
        configure({ unarmed = unarmed, weapon = weapon, health = 40 })
    end
end
script.eventHandlers.woundDuelFight = function(enabled) self:enableAI(enabled == true) end
return script

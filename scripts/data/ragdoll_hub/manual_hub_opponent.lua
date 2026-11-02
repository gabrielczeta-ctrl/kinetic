local self = require('openmw.self')
local types = require('openmw.types')
local items = require('manual_hub_items')
local script = require('ragdoll_manual_duelist')
local configure = script.engineHandlers.onInit
local config, ready
script.engineHandlers.onInit = function(data)
    config = data
    configure(data)
    self:enableAI(false)
end
script.engineHandlers.onUpdate = function()
    if ready then return end
    local inventory = types.Actor.inventory(self)
    local equipment = {}
    if config.weapon then equipment[types.Actor.EQUIPMENT_SLOT.CarriedRight] = inventory:find(config.weapon) end
    if config.armor then equipment[types.Actor.EQUIPMENT_SLOT.Cuirass] = inventory:find(config.armor) end
    -- Armor sets and shields (manual_hub_items), one item per slot.
    for _, id in ipairs(config.equip or {}) do
        local slot = items.slot(id)
        if slot then equipment[slot] = inventory:find(id) end
    end
    types.Actor.setEquipment(self, equipment)
    types.Actor.setStance(self, types.Actor.STANCE.Weapon)
    local item = types.Actor.getEquipment(self, types.Actor.EQUIPMENT_SLOT.CarriedRight)
    local armor = types.Actor.getEquipment(self, types.Actor.EQUIPMENT_SLOT.Cuirass)
    ready = types.Actor.getStance(self) == types.Actor.STANCE.Weapon
        and ((not config.weapon and not item) or (item and item.recordId == config.weapon))
        and (not config.armor or (armor and armor.recordId == config.armor))
end
script.eventHandlers.manualHubFight = function(enabled) self:enableAI(enabled == true) end
return script

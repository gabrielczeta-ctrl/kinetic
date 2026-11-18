local self = require('openmw.self')
local types = require('openmw.types')
local items = require('manual_hub_items')
local script = require('ragdoll_manual_duelist')
local configure = script.engineHandlers.onInit
local config, ready
-- A built duel's opponent (manual_hub_duel.lua): a tier replaces the game
-- record's level, attributes and skills; either way its best weapon skill
-- goes on the weapon it holds, it fights to the end (no fleeing, no alarm)
-- and its health is its own (or the tier's) times the duel's scale.
local function applyStats(stats)
    local skills = types.NPC.stats.skills
    local tier = stats.tier
    if tier then
        pcall(function() types.Actor.stats.level(self).current = tier.level end)
        for name, value in pairs(tier.attributes) do types.Actor.stats.attributes[name](self).base = value end
        for name, value in pairs(tier.skills) do
            if skills[name] then skills[name](self).base = value end
        end
        local a = tier.attributes
        local fatigue = types.Actor.stats.dynamic.fatigue(self)
        fatigue.base = a.strength + a.willpower + a.agility + a.endurance
        fatigue.current = fatigue.base
    end
    local best = 0
    for _, name in ipairs(stats.meleeSkills or {}) do best = math.max(best, skills[name](self).base) end
    if stats.skill and skills[stats.skill] then
        skills[stats.skill](self).base = math.max(best, skills[stats.skill](self).base)
    end
    for name, value in pairs({ fight = 100, flee = 0, alarm = 0 }) do
        pcall(function() types.Actor.stats.ai[name](self).base = value end)
    end
    local health = tier and tier.health
        or types.Actor.stats.dynamic.health(self).base * (stats.healthScale or 1)
    if not health or health < 1 then health = 100 end
    health = math.floor(health + 0.5)
    print(string.format('PLAY05_HUB DUELIST %s (%s) level %d %s %d block %d agility %d health %d',
        tostring(self.recordId), tier and 'tier' or 'own record', types.Actor.stats.level(self).current,
        tostring(stats.skill), stats.skill and skills[stats.skill](self).base or 0, skills.block(self).base,
        types.Actor.stats.attributes.agility(self).base, health))
    return health
end
script.engineHandlers.onInit = function(data)
    config = data
    if data and data.stats then data.health = applyStats(data.stats) end
    configure(data)
    self:enableAI(false)
end
script.engineHandlers.onUpdate = function()
    -- An update can come before the configure event sets `config`.
    if ready or not config then return end
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

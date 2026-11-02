-- NET-01 duel: the opponent's puppet. It stands still for the engine (no AI,
-- no input); its body replays the opponent's from the link. It holds the same
-- weapon, drawn, so its parts match the opponent's one for one.
local core = require('openmw.core')
local self = require('openmw.self')
local types = require('openmw.types')

local Actor = types.Actor
local WEAPON = 'steel longsword'
local stage = 0
local ticks = 0

local function onUpdate()
    self.controls.movement = 0
    self.controls.sideMovement = 0
    self.controls.use = 0
    if stage == 0 then
        self:enableAI(false)
        local weapon = Actor.inventory(self):find(WEAPON)
        if weapon then Actor.setEquipment(self, { [Actor.EQUIPMENT_SLOT.CarriedRight] = weapon }) end
        stage = 1
    elseif stage == 1 then
        Actor.setStance(self, Actor.STANCE.Weapon)
        ticks = ticks + 1
        -- The draw animation runs before the stance counts as drawn.
        if Actor.getStance(self) == Actor.STANCE.Weapon and ticks > 30 then
            stage = 2
            core.sendGlobalEvent('KineticDuelPuppetReady', { actor = self.object })
        end
    elseif Actor.getStance(self) ~= Actor.STANCE.Weapon then
        Actor.setStance(self, Actor.STANCE.Weapon)
    end
end

return {
    engineHandlers = { onUpdate = onUpdate },
    eventHandlers = {
        -- The opponent's health, for the health bar over the puppet.
        KineticDuelPuppetHealth = function(data)
            local health = Actor.stats.dynamic.health(self)
            if data.base then health.base = math.max(1, data.base) end
            if data.current then health.current = math.max(1, data.current) end
        end,
    },
}

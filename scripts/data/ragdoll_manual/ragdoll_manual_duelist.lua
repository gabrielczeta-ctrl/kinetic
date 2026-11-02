local core = require('openmw.core')
local self = require('openmw.self')
local types = require('openmw.types')

-- Manual-range duelist. Same shape as the integration suite's duelist.lua: it
-- equips its loadout, takes the weapon stance and then leaves the fight to the
-- ordinary AI combat packages. Nothing here toggles AI or writes controls, so
-- the physical strikes are driven exactly like the passing duel smoke.
--
-- Runtime-created actors do not get the built-in omw/combat/local.lua script,
-- whose Hit handler applies melee damage and spawns blood. Both halves are
-- reproduced here.

-- Copy of omw/combat/local.lua's blood effect, minus the player setting.
local function spawnBloodEffect(position)
    local bloodEffectModel = 'meshes/' .. core.getGMST(string.format('Blood_Model_%d', math.random(0, 2)))
    local record = self.object.type.record(self.object)
    local bloodTexture = core.getGMST(string.format('Blood_Texture_%d', record.bloodType))
    if not bloodTexture or bloodTexture == '' then
        bloodTexture = core.getGMST('Blood_Texture_0')
    end
    core.sendGlobalEvent('SpawnVfx', {
        model = bloodEffectModel,
        position = position,
        options = {
            mwMagicVfx = false,
            particleTextureOverride = bloodTexture,
            useAmbientLight = false,
        },
    })
end

local function configure(data)
    if not (data and data.unarmed) then
        local weapon = types.Actor.inventory(self):find((data and data.weapon) or 'iron dagger')
        if weapon then
            types.Actor.setEquipment(self, { [types.Actor.EQUIPMENT_SLOT.CarriedRight] = weapon })
        end
    else
        -- Empty table unequips every slot; the AI falls back to hand-to-hand.
        types.Actor.setEquipment(self, {})
    end
    types.Actor.setStance(self, types.Actor.STANCE.Weapon)
    -- Normal NPC pool by default so a dagger takes a real fight to finish;
    -- the range can ask for the quick (12) test value.
    local maxHealth = (data and data.health) or 40
    local health = types.Actor.stats.dynamic.health(self)
    health.base = maxHealth
    health.current = maxHealth
end

return {
    engineHandlers = {
        -- addScript passes the loadout as initialization data, so the fighter
        -- is armed on the same tick its script attaches.
        onInit = function(data) configure(data) end,
    },
    eventHandlers = {
        ragdollManualDuelistConfigure = configure,
        -- The built-in handler the runtime-created actor never received.
        Hit = function(data)
            if data and data.successful and data.damage and (data.damage.health or 0) > 0 then
                print(string.format('RAGDOLL_MANUAL_HIT actor=%s damage=%.2f strength=%.3f health_before=%.1f',
                    tostring(self.recordId), data.damage.health, data.strength or -1,
                    types.Actor.stats.dynamic.health(self).current))
                if not data.muteSound then
                    core.sound.playSound3d('Health Damage', self)
                end
                if data.hitPos then
                    spawnBloodEffect(data.hitPos)
                end
            end
            types.Actor._onHit(self, data)
        end,
    },
}

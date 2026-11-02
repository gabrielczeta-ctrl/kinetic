local core = require('openmw.core')
local types = require('openmw.types')
local self = require('openmw.self')

-- Minimal NPC duelist: equips a dagger, takes a weapon stance and then leaves
-- the fighting to the ordinary AI combat packages. Objects created at run
-- time do not receive the built-in omw/combat/local.lua script, whose Hit
-- handler applies melee damage through Actor._onHit; without that handler the
-- swings connect, compute damage and then apply nothing.

-- Copy of omw/combat/local.lua's blood effect, so a landed blow reads on
-- screen instead of only moving the health bar.
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
local retreating = nil

local function configure(data)
    if not (data and data.unarmed) then
        local weapon = types.Actor.inventory(self):find(data and data.weapon or 'iron dagger')
        if weapon then
            -- Only the weapon changes: a table with just the weapon stripped
            -- the stock guard's bonemold armour (every DMG-04 hit read
            -- unarmoured, 2026-09-29).
            local equipment = types.Actor.getEquipment(self)
            equipment[types.Actor.EQUIPMENT_SLOT.CarriedRight] = weapon
            -- SHIELD-01: a shield on the left arm.
            if data and data.shield then
                equipment[types.Actor.EQUIPMENT_SLOT.CarriedLeft] = types.Actor.inventory(self):find(data.shield)
            end
            -- RANGED-02: the archer's arrows.
            if data and data.ammo then
                equipment[types.Actor.EQUIPMENT_SLOT.Ammunition] = types.Actor.inventory(self):find(data.ammo)
            end
            types.Actor.setEquipment(self, equipment)
        end
        types.Actor.setStance(self, types.Actor.STANCE.Weapon)
    end
    -- DUEL-C01's foe: no AI of its own, so only the test's controls move it.
    if data and data.passive then
        self:enableAI(false)
        retreating = false
    end
    if data and data.fight then types.Actor.stats.ai.fight(self).base = data.fight end
    -- BODYPUSH-01: set the body's Strength and Endurance.
    if data and data.strength then types.Actor.stats.attributes.strength(self).base = data.strength end
    if data and data.endurance then types.Actor.stats.attributes.endurance(self).base = data.endurance end
    -- SKILL-01: the attacker's Long Blade skill.
    if data and data.longblade then types.NPC.stats.skills.longblade(self).base = data.longblade end
    if data and data.health then
        local health = types.Actor.stats.dynamic.health(self)
        health.base = data.health
        health.current = data.health
        return
    end
    -- DMG-04 measures real Morrowind stats: keep the record's health.
    if data and data.keepHealth then return end
    local health = types.Actor.stats.dynamic.health(self)
    -- Low enough that a duel resolves in a practical test window; the AI lands
    -- roughly one blow every few seconds and each committed strike carries a
    -- few points. Unarmed blows are lighter, so those duelists get less pool.
    local pool = (data and data.unarmed) and 8 or 12
    health.base = pool
    health.current = pool
end

local walking = false
local function update()
    if walking then
        self.controls.movement = 1
        self.controls.sideMovement = 0
        self.controls.run = false
        return
    end
    if retreating == nil then return end
    self.controls.movement = retreating and -1 or 0
    self.controls.sideMovement = 0
    self.controls.run = retreating
    self.controls.use = self.ATTACK_TYPE.NoAttack
end

return {
    engineHandlers = { onUpdate = update },
    eventHandlers = {
        duelDuelistConfigure = configure,
        -- BODYPUSH-01: walk straight ahead (true) or stand (false).
        duelDuelistWalk = function(value) walking = value == true end,
        -- DUEL-C01: back away from the foe (backwards, running) or stand;
        -- applied every frame (onUpdate), as controls set once do not hold.
        duelDuelistRetreat = function(data)
            retreating = data and data.on or false
        end,
        duelDuelistStop = function()
            self.controls.use = self.ATTACK_TYPE.NoAttack
            self.controls.movement = 0
            self.controls.sideMovement = 0
            self.controls.run = false
        end,
        Hit = function(data)
            if data and data.successful and data.damage and (data.damage.health or 0) > 0 then
                print(string.format('RAGDOLL_DUELIST_HIT actor=%s damage=%.2f health_before=%.1f',
                    tostring(self.recordId), data.damage.health,
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

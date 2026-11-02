local core = require('openmw.core')
local nearby = require('openmw.nearby')
local self = require('openmw.self')
local types = require('openmw.types')
local util = require('openmw.util')

-- Runtime-created actors do not get the built-in omw/combat/local.lua script.
-- Its Hit handler applies the damage the player's animation melee computes and
-- spawns the blood and damage sound; reproduce all three here.
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

local enabled = false
local attackPhase = 'idle'
local phaseDeadline = 0
local attackSequence = 0
local ready = false
local attacking = false
-- When set, the duelist fights the actor with this record id instead of the
-- player; the NPC-vs-NPC scenario configures both duelists that way.
local targetRecordId

local function targetActor()
    local nearest
    local distance
    for _, actor in ipairs(nearby.actors) do
        if actor.id ~= self.id then
            local wanted = (targetRecordId and actor.recordId == targetRecordId)
                or (not targetRecordId and types.Player.objectIsInstance(actor))
            if wanted then
                local candidateDistance = (actor.position - self.position):length()
                if not distance or candidateDistance < distance then
                    nearest, distance = actor, candidateDistance
                end
            end
        end
    end
    return nearest, distance
end

local function configure(data)
    self:enableAI(false)
    local weapon = types.Actor.inventory(self):find('iron dagger')
        or types.Actor.inventory(self):find('basic_dagger1h')
    if weapon then
        types.Actor.setEquipment(self, { [types.Actor.EQUIPMENT_SLOT.CarriedRight] = weapon })
    end
    types.Actor.setStance(self, types.Actor.STANCE.Weapon)
    local health = types.Actor.stats.dynamic.health(self)
    health.base = math.max(health.base, data and data.health or 10000)
    health.current = health.base
    targetRecordId = data and data.target or nil
    enabled = true
    ready = false
    attacking = false
    attackPhase = 'idle'
end

local function stop()
    enabled = false
    ready = false
    attacking = false
    attackPhase = 'idle'
    self.controls.use = self.ATTACK_TYPE.NoAttack
    self.controls.movement = 0
    self.controls.sideMovement = 0
    self.controls.run = false
end

local function update()
    if not enabled then return end
    if not ready then
        local weapon = types.Actor.getEquipment(self, types.Actor.EQUIPMENT_SLOT.CarriedRight)
        if weapon and types.Actor.getStance(self) == types.Actor.STANCE.Weapon then
            ready = true
            core.sendGlobalEvent('duelOpponentReady', { weapon = weapon.recordId })
        else
            return
        end
    end
    if not types.Actor.canMove(self) then
        attackPhase = 'idle'
        self.controls.use = self.ATTACK_TYPE.NoAttack
        self.controls.movement = 0
        return
    end
    local target, distance = targetActor()
    if not target then return end
    local direction = target.position - self.position
    if direction:length() > 0.001 then
        direction = direction:normalize()
        self.controls.yawChange
            = util.normalizeAngle(math.atan2(direction.x, direction.y) - self.rotation:getYaw())
    end
    self.controls.sideMovement = 0
    self.controls.run = false
    self.controls.movement = distance > 56 and 0.5 or 0
    if not attacking then
        attackPhase = 'idle'
        self.controls.use = self.ATTACK_TYPE.NoAttack
        return
    end
    local now = core.getSimulationTime()
    if attackPhase == 'idle' and distance <= 64 then
        attackPhase = 'charge'
        phaseDeadline = now + 0.35
        self.controls.use = self.ATTACK_TYPE.Any
    elseif attackPhase == 'charge' then
        self.controls.use = self.ATTACK_TYPE.Any
        if now >= phaseDeadline then
            self.controls.use = self.ATTACK_TYPE.NoAttack
            attackPhase = 'cooldown'
            phaseDeadline = now + 1.25
            attackSequence = attackSequence + 1
            core.sendGlobalEvent('duelOpponentStrikeReleased', { sequence = attackSequence })
        end
    elseif attackPhase == 'cooldown' and now >= phaseDeadline then
        attackPhase = 'idle'
    else
        self.controls.use = self.ATTACK_TYPE.NoAttack
    end
end

return {
    engineHandlers = { onInit = function() self:enableAI(false) end, onUpdate = update },
    eventHandlers = {
        duelOpponentConfigure = configure,
        duelOpponentAttackEnabled = function(data)
            attacking = data.enabled == true
            if not attacking then
                attackPhase = 'idle'
                self.controls.use = self.ATTACK_TYPE.NoAttack
            end
        end,
        duelOpponentStop = stop,
        Hit = function(data)
            if data and data.successful and data.damage and (data.damage.health or 0) > 0 then
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

local core = require('openmw.core')
local self = require('openmw.self')
local types = require('openmw.types')

local walking = false
-- While a duel runs, the AI owns movement and rotation: this script must not
-- rewrite the controls every frame or the fighter is pinned in place.
local dueling = false
-- Scripted path: an ordered list of segments { duration, movement,
-- sideMovement, yawRate } (seconds, forward fraction, side fraction, radians
-- per second). While a path is active it overrides the plain gait flag; the
-- last segment's controls persist until the path is cleared.
local path = nil
local pathIndex = 1
local pathElapsed = 0
local logFrames = 0

-- Copy of omw/combat/local.lua's blood effect. That built-in script cannot be
-- attached to a runtime-created actor, so the visible half of a hit has to be
-- reproduced here or a landed blow looks like nothing happened.
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

-- Diagnostic (2026-09-11): print the controls actually applied once a
-- second so a "strafe became a forward walk" report can be checked
-- against what this script wrote and the actor's yaw at the time.
local function logControls(tag)
    print(string.format('RAGDOLL_MANUAL_CONTROLS %s walking=%s path=%s index=%d move=%.2f side=%.2f yawChange=%.4f yaw=%.3f',
        tag, tostring(walking), path and #path or 'nil', pathIndex,
        self.controls.movement or 0, self.controls.sideMovement or 0, self.controls.yawChange or 0,
        self.rotation:getYaw()))
end

local function initialize()
    -- Every range sample stays idle until an explicit gait command. Inherited
    -- NPC AI can otherwise request movement during a standing-only trial.
    self:enableAI(false)
end

local function update(dt)
    if dueling then
        return
    end
    self.controls.jump = false
    self.controls.run = false
    if path then
        local segment = path[pathIndex]
        pathElapsed = pathElapsed + (dt or 0)
        while segment and pathElapsed >= (segment.duration or 0) and pathIndex < #path do
            pathElapsed = pathElapsed - (segment.duration or 0)
            pathIndex = pathIndex + 1
            segment = path[pathIndex]
        end
        if segment then
            self.controls.movement = segment.movement or 0
            self.controls.sideMovement = segment.sideMovement or 0
            self.controls.yawChange = (segment.yawRate or 0) * (dt or 0)
            logFrames = logFrames + 1
            if logFrames % 60 == 1 then logControls('path') end
            return
        end
    end
    -- Match the first live deterministic gait gate (roughly 60 world
    -- units/s); full-speed locomotion is not a validated milestone yet.
    self.controls.movement = walking and 0.4 or 0
    self.controls.sideMovement = 0
    self.controls.yawChange = 0
    if walking then
        logFrames = logFrames + 1
        if logFrames % 60 == 1 then logControls('walk') end
    end
end

return {
    engineHandlers = {
        onInit = initialize,
        onLoad = initialize,
        onUpdate = update,
    },
    eventHandlers = {
        ragdollManualDefenseTargetConfigure = function()
            local health = types.Actor.stats.dynamic.health(self)
            health.base = math.max(health.base, 1000)
            health.current = health.base
            core.sendGlobalEvent('ragdollManualDefenseTargetReady', { ready = true })
        end,
        setManualRagdollGait = function(enabled)
            walking = enabled == true
            if not walking then
                path = nil
            end
            logFrames = 0
            logControls('gait:' .. tostring(enabled))
        end,
        setManualRagdollPath = function(segments)
            if type(segments) == 'table' and #segments > 0 then
                path = segments
                pathIndex = 1
                pathElapsed = 0
            else
                path = nil
            end
            logControls('path-set')
        end,
        setManualRagdollDuel = function(enabled)
            dueling = enabled == true
            if dueling then
                path = nil
                walking = false
                self:enableAI(true)
            else
                self:enableAI(false)
            end
            logControls('duel:' .. tostring(dueling))
        end,
        -- Auto-duel loadout: one fighter is armed with a dagger, the other
        -- fights bare-handed. setEquipment and setStance are self-only
        -- bindings, which is why the global range delegates them here.
        setManualRagdollLoadout = function(data)
            local loadout = type(data) == 'table' and data.loadout or 'fist'
            if loadout == 'dagger' then
                local weapon = types.Actor.inventory(self):find('iron dagger')
                if weapon then
                    types.Actor.setEquipment(self, { [types.Actor.EQUIPMENT_SLOT.CarriedRight] = weapon })
                else
                    print('RagdollDev.autoDuel: dagger fighter has no iron dagger; fighting unarmed')
                end
            else
                -- An empty table unequips every slot; the AI then falls back to
                -- hand-to-hand.
                types.Actor.setEquipment(self, {})
            end
            -- The weapon stance is also the hand-to-hand ready pose.
            types.Actor.setStance(self, types.Actor.STANCE.Weapon)
            -- Short enough that a round resolves on screen, long enough to
            -- watch the physical blows land. The template's hand-to-hand
            -- damage is a couple of points a blow, so a large pool never ends.
            local health = types.Actor.stats.dynamic.health(self)
            health.base = 20
            health.current = 20
        end,
        -- Runtime-created actors do not get the built-in omw/combat/local.lua
        -- Hit handler, so melee damage would be computed and then dropped.
        -- The built-in handler also plays the damage sound and spawns blood;
        -- both are reproduced here so a landed blow reads on screen.
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

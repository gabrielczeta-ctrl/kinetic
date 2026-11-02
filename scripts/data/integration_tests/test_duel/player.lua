local camera = require('openmw.camera')
local core = require('openmw.core')
local interfaces = require('openmw.interfaces')
local nearby = require('openmw.nearby')
local self = require('openmw.self')
local testing = require('testing_util')
local types = require('openmw.types')
local util = require('openmw.util')

local enabled = false
local strikeSequence = 0
local strikePhase = 'idle'
local phaseDeadline = 0
local ready = false
local continuousAttack = false

local function opponent()
    local nearest
    local distance
    for _, actor in ipairs(nearby.actors) do
        if actor.id ~= self.id then
            local candidateDistance = (actor.position - self.position):length()
            if not distance or candidateDistance < distance then
                nearest, distance = actor, candidateDistance
            end
        end
    end
    return nearest, distance
end

local function configure(data)
    interfaces.Controls.overrideMovementControls(true)
    camera.setMode(camera.MODE.ThirdPerson)
    camera.setPreferredThirdPersonDistance(320)
    camera.setPitch(0.2)
    local weapon = (data and data.weapon and types.Actor.inventory(self):find(data.weapon))
        or types.Actor.inventory(self):find('iron warhammer')
        or types.Actor.inventory(self):find('iron longsword')
        or types.Actor.inventory(self):find('basic_dagger1h')
    testing.expect(weapon ~= nil, 'DUEL-A01 player has no test weapon')
    types.Actor.setEquipment(self, { [types.Actor.EQUIPMENT_SLOT.CarriedRight] = weapon })
    types.Actor.setStance(self, types.Actor.STANCE.Weapon)
    local health = types.Actor.stats.dynamic.health(self)
    health.base = math.max(health.base, 10000)
    health.current = health.base
    enabled = true
    ready = false
end

local function stop()
    enabled = false
    ready = false
    continuousAttack = false
    strikePhase = 'idle'
    self.controls.use = self.ATTACK_TYPE.NoAttack
    self.controls.movement = 0
    self.controls.sideMovement = 0
    self.controls.run = false
    interfaces.Controls.overrideMovementControls(false)
end

local function update()
    if not enabled then return end
    if not ready then
        local weapon = types.Actor.getEquipment(self, types.Actor.EQUIPMENT_SLOT.CarriedRight)
        if weapon and types.Actor.getStance(self) == types.Actor.STANCE.Weapon then
            ready = true
            core.sendGlobalEvent('duelPlayerReady', { weapon = weapon.recordId })
        else
            return
        end
    end
    local target, distance = opponent()
    if not target then return end
    local direction = target.position - self.position
    if direction:length() > 0.001 then
        direction = direction:normalize()
        self.controls.yawChange
            = util.normalizeAngle(math.atan2(direction.x, direction.y) - self.rotation:getYaw())
    end
    self.controls.sideMovement = 0
    self.controls.run = false
    self.controls.movement = distance > 95 and 0.35 or 0
    if continuousAttack and strikePhase == 'idle' and distance <= 100 then
        strikePhase = 'charge'
        phaseDeadline = core.getSimulationTime() + 0.65
        self.controls.use = self.ATTACK_TYPE.Any
    end
    if strikePhase == 'charge' then
        self.controls.use = self.ATTACK_TYPE.Any
        if core.getSimulationTime() >= phaseDeadline then
            self.controls.use = self.ATTACK_TYPE.NoAttack
            strikePhase = 'cooldown'
            phaseDeadline = core.getSimulationTime() + 0.85
            core.sendGlobalEvent('duelPlayerStrikeReleased', {
                sequence = strikeSequence,
                distance = distance,
                stance = types.Actor.getStance(self),
            })
        end
    elseif strikePhase == 'cooldown' and core.getSimulationTime() >= phaseDeadline then
        strikePhase = 'idle'
    else
        self.controls.use = self.ATTACK_TYPE.NoAttack
    end
end

local handlers = {}
for name, handler in pairs(testing.localEventHandlers) do handlers[name] = handler end
handlers.duelPlayerConfigure = configure
handlers.duelPlayerStrike = function(data)
    if not enabled or strikePhase ~= 'idle' then return end
    strikeSequence = data.sequence
    strikePhase = 'charge'
    phaseDeadline = core.getSimulationTime() + 0.65
    self.controls.use = self.ATTACK_TYPE.Any
end
handlers.duelPlayerAttackEnabled = function(data)
    continuousAttack = data.enabled == true
    strikeSequence = data.sequence or strikeSequence
    if not continuousAttack then
        strikePhase = 'idle'
        self.controls.use = self.ATTACK_TYPE.NoAttack
    end
end
handlers.duelPlayerStop = stop

-- NPCHIT-03: the player keeps moving like a fighter - facing the nearest
-- actor, it strafes left, right, steps back and in at a walk (1 s each).
local movePattern = false
local moveClock = 0
handlers.duelPlayerMovePattern = function(data)
    movePattern = data.enabled == true
    moveClock = 0
    interfaces.Controls.overrideMovementControls(movePattern)
    if not movePattern then
        self.controls.movement = 0
        self.controls.sideMovement = 0
    end
end
local function updateMovePattern(dt)
    if not movePattern then return end
    moveClock = moveClock + dt
    local target = opponent()
    if target then
        local direction = target.position - self.position
        if direction:length() > 0.001 then
            direction = direction:normalize()
            self.controls.yawChange
                = util.normalizeAngle(math.atan2(direction.x, direction.y) - self.rotation:getYaw())
        end
    end
    local step = math.floor(moveClock) % 4
    self.controls.run = false
    self.controls.sideMovement = (step == 0 and -0.6) or (step == 1 and 0.6) or 0
    self.controls.movement = (step == 2 and -0.6) or (step == 3 and 0.6) or 0
end

return {
    engineHandlers = { onUpdate = function(dt) update(dt); updateMovePattern(dt) end },
    eventHandlers = handlers,
}

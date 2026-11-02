local core = require('openmw.core')
local nearby = require('openmw.nearby')
local util = require('openmw.util')
local self = require('openmw.self')
local types = require('openmw.types')

-- A test dummy: no AI, and optionally a drawn dagger so its body stands in the
-- staggered weapon stance instead of the idle one.
local wantWeaponStance = false
local reported = false
local woundMode = 'normal'
local movement = 0
local running = false
-- REACT-02/03 (the leg give-out prices a blow by the share of health it
-- costs): a realistic health, topped up every frame so the dummy never dies.
local refillHealth = nil
-- REACT-04: the dummy fights with its own AI (an AI-disabled NPC ignores
-- the attack control); the script then leaves the controls alone.
local aiFights = false
-- ROUGH-01: a navmesh route walked by turning toward each point in turn.
local route, routeIndex = nil, 1
local function steer()
    if not route then return end
    local point = route[routeIndex]
    -- Reached when near on the floor and at about its height (SHIP-01: the
    -- top of a stair that doubles back is near its foot on the floor).
    while point and util.vector3(point.x - self.position.x, point.y - self.position.y, 0):length() < 45
        and math.abs(point.z - self.position.z) < 60 do
        routeIndex = routeIndex + 1
        point = route[routeIndex]
    end
    if not point then
        route, movement = nil, 0
        core.sendGlobalEvent('fixActorRouteDone', { id = self.id })
        return
    end
    local wanted = math.atan2(point.x - self.position.x, point.y - self.position.y)
    local turn = wanted - self.rotation:getYaw()
    turn = (turn + math.pi) % (2 * math.pi) - math.pi
    self.controls.yawChange = math.max(-0.08, math.min(0.08, turn))
    movement = math.abs(turn) < 1.2 and 1 or 0
end

local function configure(data)
    self:enableAI(false)
    -- true = an iron dagger; a record id = that weapon (spawnDummy puts it in
    -- the inventory; before 10-07 only the dagger was equipped and drawn, so a
    -- named weapon - CLASH-01's staff, REACT-02's guard - stayed in the pack).
    local named = data and type(data.weaponStance) == 'string' and data.weaponStance or nil
    wantWeaponStance = data and (data.weaponStance == true or named ~= nil)
    reported = false
    if wantWeaponStance then
        local weapon = types.Actor.inventory(self):find(named or 'iron dagger')
        if weapon then
            types.Actor.setEquipment(self, { [types.Actor.EQUIPMENT_SLOT.CarriedRight] = weapon })
        end
        types.Actor.setStance(self, types.Actor.STANCE.Weapon)
    else
        types.Actor.setStance(self, types.Actor.STANCE.Nothing)
    end
    local health = types.Actor.stats.dynamic.health(self)
    health.base = math.max(health.base, 10000)
    health.current = health.base
end

local function update()
    if refillHealth then
        local health = types.Actor.stats.dynamic.health(self)
        health.base = refillHealth
        health.current = refillHealth
    end
    if not aiFights then
        steer()
        self.controls.movement = movement
        self.controls.run = running
        self.controls.sideMovement = 0
        self.controls.use = self.ATTACK_TYPE.NoAttack
    end
    if reported then return end
    local stance = types.Actor.getStance(self)
    if (wantWeaponStance and stance == types.Actor.STANCE.Weapon)
        or (not wantWeaponStance and stance == types.Actor.STANCE.Nothing) then
        reported = true
        core.sendGlobalEvent('fixActorReady', { id = self.id, weaponStance = wantWeaponStance })
    end
end

return {
    engineHandlers = { onInit = function() self:enableAI(false) end, onUpdate = update },
    eventHandlers = {
        fixActorConfigure = configure,
        fixActorMove = function(value) movement = value end,
        fixActorRoute = function(dest)
            local status, path = nearby.findPath(self.position, dest)
            route, routeIndex = (path and #path > 0) and path or nil, 1
            local list = {}
            for _, p in ipairs(path or {}) do list[#list + 1] = string.format('%.0f,%.0f,%.0f', p.x, p.y, p.z) end
            core.sendGlobalEvent('fixActorRouteStart', { id = self.id, points = path and #path or 0,
                status = tostring(status), list = table.concat(list, ' ') })
        end,
        -- SHIP-01: walk these points in turn (past a navmesh route's end).
        fixActorPoints = function(points) route, routeIndex = points, 1 end,
        -- SHIP-01: the game's own AI travel (its pathing), as the chargen
        -- guard walks.
        fixActorTravel = function(dest)
            route = nil
            self:enableAI(true)
            require('openmw.interfaces').AI.startPackage({ type = 'Travel', destPosition = dest })
        end,
        fixActorRun = function(on) running = on end,
        fixActorHealth = function(value) refillHealth = value end,
        fixActorFight = function(on)
            aiFights = on
            self:enableAI(on)
        end,
        -- BITE-01: wear this cuirass (record id).
        fixActorCuirass = function(id)
            local item = types.Actor.inventory(self):find(id)
            if not item then return end
            local equipment = types.Actor.getEquipment(self)
            equipment[types.Actor.EQUIPMENT_SLOT.Cuirass] = item
            types.Actor.setEquipment(self, equipment)
        end,
        -- WOUNDFX-03: wear exactly these pieces ({ [slot name] = record id },
        -- e.g. { Shirt = 'common_shirt_01' }; they must be in the inventory),
        -- nothing else.
        fixActorWear = function(slots)
            local equipment = {}
            for slot, id in pairs(slots) do
                local index = types.Actor.EQUIPMENT_SLOT[slot]
                local item = types.Actor.inventory(self):find(id)
                if index and item then equipment[index] = item end
            end
            types.Actor.setEquipment(self, equipment)
        end,
        fixWoundMode = function(data)
            woundMode = data.mode
            types.Actor.setEquipment(self, {})
        end,
        Hit = function(data)
            if woundMode == 'cancel' then return end
            if woundMode == 'zero' then data.damage = { health = 0, fatigue = 0 } end
            types.Actor._onHit(self, data)
            if woundMode == 'duplicate' then types.Actor._onHit(self, data) end
        end,
    },
}

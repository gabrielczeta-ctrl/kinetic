-- PILOT: an outside process plays the player (tools/body/pilot.py).
--
-- One case, 'PILOT session'. The game posts an observation and waits, frozen,
-- for the pilot's next action (ragdoll.pilotExchange); an action holds for its
-- `steps` frames, then the next observation goes out. The player's weapon is
-- driven the way the user drives it: mouse motion and buttons through the
-- ordinary input path (ragdoll.injectMouseInput), movement through the
-- player's controls.
--
-- Action line (act.txt), space-separated key=value, every key optional:
--   steps=N        frames this action holds (default 6, 1..600)
--   fwd= side=     -1..1 movement;  run=0|1  jump=0|1
--   turn=RAD       yaw over the action;  face=0|1 keep facing the nearest actor
--   mdx= mdy=      mouse motion over the action, spread across its frames
--   lmb= rmb= mmb= 0|1 button held (kept until changed)
--   end=1          finish the session
-- The first line is the setup: weapon= foe= foeweapon= dist= foehealth=
--   health= passive=0|1 hits=0|1 (a + in a record id reads as a space).
local core = require('openmw.core')
local ragdoll = require('openmw.ragdolltest')
local testing = require('testing_util')
local testArea = require('ragdoll_test_area')
local types = require('openmw.types')
local util = require('openmw.util')
local world = require('openmw.world')

local playerReady = false

local function parse(line)
    local fields = {}
    for key, value in line:gmatch('(%w+)=(%S+)') do fields[key] = value end
    return fields
end

local function number(fields, key, default, low, high)
    local value = tonumber(fields[key]) or default
    return math.max(low, math.min(high, value))
end

local function encode(fields)
    local keys, parts = {}, {}
    for key in pairs(fields) do table.insert(keys, key) end
    table.sort(keys)
    for _, key in ipairs(keys) do
        local value = fields[key]
        local text
        if type(value) == 'number' then
            text = (value ~= value or value == math.huge or value == -math.huge) and 'null'
                or string.format('%.6g', value)
        elseif type(value) == 'boolean' then text = tostring(value)
        else text = '"' .. tostring(value):gsub('[%c\\"]', '_') .. '"' end
        table.insert(parts, '"' .. key .. '":' .. text)
    end
    return '{' .. table.concat(parts, ',') .. '}'
end

local function waitFor(description, predicate, timeout)
    local deadline = core.getSimulationTime() + (timeout or 10)
    repeat
        local value = predicate()
        if value then return value end
        coroutine.yield()
    until core.getSimulationTime() >= deadline
    error('PILOT timed out waiting for ' .. description)
end

local function addItem(actor, recordId)
    local inventory = types.Actor.inventory(actor)
    if not inventory:find(recordId) then
        world.createObject(recordId, 1):moveInto(inventory)
        coroutine.yield()
    end
    testing.expect(inventory:find(recordId) ~= nil, 'PILOT could not add ' .. recordId)
end

local function describe(prefix, actor, out)
    out[prefix .. 'x'], out[prefix .. 'y'], out[prefix .. 'z']
        = actor.position.x, actor.position.y, actor.position.z
    out[prefix .. 'yaw'] = actor.rotation:getYaw()
    out[prefix .. 'health'] = types.Actor.stats.dynamic.health(actor).current
    out[prefix .. 'fatigue'] = types.Actor.stats.dynamic.fatigue(actor).current
    local combat = ragdoll.getCombatState(actor)
    if combat then
        out[prefix .. 'mode'] = combat.mode
        out[prefix .. 'mode_time'] = combat.timeInMode
    end
    -- The stance the balance sigil draws (K2_BALANCE_SIGIL): feet, centre of
    -- mass, tipping point, each foot's load, daze; absent in the air.
    local okState, state = pcall(ragdoll.getState, actor)
    if okState and state and state.balanceSigil then
        for key, value in pairs(state.balanceSigil) do
            if type(value) == 'number' then out[prefix .. 'st_' .. key] = value end
        end
    end
    local ok, speed = pcall(ragdoll.getSwingSpeed, actor)
    if ok and type(speed) == 'number' then out[prefix .. 'swing_speed'] = speed end
end

local function observe(player, foe, step, started)
    local out = { step = step, t = core.getSimulationTime() - started }
    describe('p_', player, out)
    if foe and foe:isValid() then
        describe('f_', foe, out)
        local offset = foe.position - player.position
        out.dist = offset:length()
        -- 0 = the foe straight ahead, positive = to the player's right.
        out.bearing = util.normalizeAngle(math.atan2(offset.x, offset.y) - player.rotation:getYaw())
        out.f_dead = types.Actor.isDead(foe)
    end
    out.p_dead = types.Actor.isDead(player)
    return out
end

testing.registerGlobalTest('PILOT session', function()
    local foe
    local player = world.players[1]
    local ok, message = pcall(function()
        local setupLine = ragdoll.pilotExchange(encode({ hello = true }), 120000)
        testing.expect(setupLine ~= nil, 'PILOT no pilot answered (OPENMW_RAGDOLL_PILOT_DIR, tools/body/pilot.py)')
        local setup = parse(setupLine)
        local function record(key, default) return ((setup[key] or default):gsub('%+', ' ')) end
        local weapon = record('weapon', 'iron+longsword')
        local foeRecord = record('foe', 'hlaalu+guard')
        local foeWeapon = record('foeweapon', 'steel+longsword')

        ragdoll.setAutomaticRecovery(true)
        ragdoll.setPlayerAlwaysHits(setup.hits == '1')
        player:teleport(testArea.cell, testArea.player, { onGround = true })
        coroutine.yield()
        if weapon ~= 'none' then addItem(player, weapon) end
        playerReady = false
        player:sendEvent('pilotPlayerConfigure',
            { weapon = weapon, health = number(setup, 'health', 200, 1, 100000) })
        waitFor('player weapon stance', function() return playerReady end)
        ragdoll.setPowered(player, true)

        if foeRecord ~= 'none' then
            local created = world.createRecord(types.NPC.createRecordDraft({
                template = types.NPC.record(foeRecord), name = 'PILOT foe', isEssential = false,
                isPersistent = false, isRespawning = false,
            }))
            foe = world.createObject(created.id)
            if foeWeapon ~= 'none' then addItem(foe, foeWeapon) end
            foe:teleport(testArea.cell,
                testArea.player + util.vector3(0, number(setup, 'dist', 200, 60, 2000), 0), { onGround = true })
            foe:addScript('duelist.lua')
            coroutine.yield()
            foe:sendEvent('duelDuelistConfigure', {
                weapon = foeWeapon ~= 'none' and foeWeapon or nil, unarmed = foeWeapon == 'none',
                passive = setup.passive == '1', health = number(setup, 'foehealth', 200, 1, 100000),
            })
            waitFor('foe weapon stance', function()
                return types.Actor.getStance(foe) == types.Actor.STANCE.Weapon
            end)
            ragdoll.setPowered(foe, true)
            if setup.passive ~= '1' then ragdoll.setCombatTarget(foe, player) end
        end
        local settle = core.getSimulationTime() + 1.5
        while core.getSimulationTime() < settle do coroutine.yield() end
        ragdoll.prepareMouseInput()

        local started = core.getSimulationTime()
        local held = { lmb = false, rmb = false, mmb = false }
        local buttons = { lmb = 1, mmb = 2, rmb = 3 }
        local step = 0
        print('PILOT_BEGIN ' .. setupLine)
        while true do
            local line = ragdoll.pilotExchange(encode(observe(player, foe, step, started)), 300000)
            if not line then print('PILOT_END reason=silent'); break end
            local action = parse(line)
            if action['end'] == '1' then print('PILOT_END reason=pilot'); break end
            local steps = math.floor(number(action, 'steps', 6, 1, 600))
            player:sendEvent('pilotControls', {
                fwd = number(action, 'fwd', 0, -1, 1), side = number(action, 'side', 0, -1, 1),
                run = action.run == '1', jump = action.jump == '1', face = action.face == '1',
                turn = number(action, 'turn', 0, -6.3, 6.3), steps = steps,
            })
            for name, id in pairs(buttons) do
                local want = action[name]
                if want and (want == '1') ~= held[name] then
                    held[name] = want == '1'
                    ragdoll.injectMouseInput(0, 0, held[name], id)
                end
            end
            local dx = number(action, 'mdx', 0, -4096 * steps, 4096 * steps)
            local dy = number(action, 'mdy', 0, -4096 * steps, 4096 * steps)
            local sentX, sentY = 0, 0
            for frame = 1, steps do
                local x, y = math.floor(dx * frame / steps + 0.5), math.floor(dy * frame / steps + 0.5)
                if x ~= sentX or y ~= sentY then
                    ragdoll.injectMouseInput(x - sentX, y - sentY)
                    sentX, sentY = x, y
                end
                coroutine.yield()
            end
            step = step + 1
            if foe and foe:isValid() and setup.passive ~= '1' and step % 20 == 0 then
                ragdoll.setCombatTarget(foe, player)
            end
        end
        for name, id in pairs(buttons) do
            if held[name] then ragdoll.injectMouseInput(0, 0, false, id) end
        end
        ragdoll.pilotExchange(encode({ done = true, step = step }), 0)
    end)
    player:sendEvent('pilotPlayerStop')
    if foe and foe:isValid() then
        foe:sendEvent('duelDuelistStop')
        ragdoll.setPowered(foe, false)
        foe:remove()
    end
    ragdoll.setPlayerAlwaysHits(false)
    ragdoll.setPowered(player, false)
    if not ok then error(message) end
end)

local handlers = {}
for name, handler in pairs(testing.globalEventHandlers) do handlers[name] = handler end
handlers.pilotPlayerReady = function() playerReady = true end

return {
    engineHandlers = { onUpdate = testing.updateGlobal },
    eventHandlers = handlers,
}

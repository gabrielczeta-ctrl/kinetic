-- NET-01 duel (docs/ragdoll/designs/NET-01-duel.md): the 1v1 between two
-- games. This side's player fights on the flat test floor; the opponent is a
-- puppet NPC whose body replays the opponent's (openmw.kineticnet). Both
-- games run the same kit, so nothing but names, health and rounds is sent.
-- Inert unless the game was started with OPENMW_KINETIC_DUEL.
local core = require('openmw.core')
local net = require('openmw.kineticnet')
local types = require('openmw.types')
local util = require('openmw.util')
local world = require('openmw.world')

local ARENA_CELL = 'OpenMW Ragdoll Test Floor'
local CENTER = util.vector3(0, 0, 128)
-- The host starts south of the centre facing north, the joiner north facing south.
local SPOT = { host = util.vector3(0, -140, 128), join = util.vector3(0, 140, 128) }
-- The test bot (OPENMW_KINETIC_DUEL_BOT) starts both in reach.
if net.bot() then SPOT = { host = util.vector3(0, -45, 128), join = util.vector3(0, 45, 128) } end
local YAW = { host = 0, join = math.pi }
local WEAPON = 'steel longsword'
-- Seconds between a fighter going down and the next round.
local ROUND_PAUSE = 4

local placed = false
local session = 0
local puppet
local puppetReady = false
local puppetCheck = 0
local score = { mine = 0, theirs = 0 }
local resetAt
local roundOver = false

local function side(mine)
    local host = net.isHost()
    if mine then return host and 'host' or 'join' end
    return host and 'join' or 'host'
end

local function player()
    return world.players[1]
end

local function tellPlayer(event, data)
    local p = player()
    if p then p:sendEvent(event, data or {}) end
end

local function placePlayer()
    local p = player()
    local mine = side(true)
    p:teleport(ARENA_CELL, SPOT[mine], { rotation = util.transform.rotateZ(YAW[mine]), onGround = true })
    p:sendEvent('KineticDuelRestore', {})
end

local function giveKit(actor)
    local inventory = types.Actor.inventory(actor)
    if not inventory:find(WEAPON) then
        world.createObject(WEAPON, 1):moveInto(inventory)
    end
end

local function removePuppet()
    if puppet and puppet:isValid() then
        net.clearPuppet()
        puppet:remove()
    end
    puppet = nil
    puppetReady = false
end

local function spawnPuppet()
    removePuppet()
    local p = player()
    local record = world.createRecord(types.NPC.createRecordDraft({
        template = types.NPC.record(p),
        name = net.remoteName() ~= '' and net.remoteName() or 'Opponent',
        isEssential = false,
        isPersistent = false,
        isRespawning = false,
    }))
    puppet = world.createObject(record.id)
    giveKit(puppet)
    local theirs = side(false)
    puppet:teleport(ARENA_CELL, SPOT[theirs], { rotation = util.transform.rotateZ(YAW[theirs]), onGround = true })
    puppet:addScript('kinetic_duel_puppet.lua')
end

local function startRound()
    print(string.format('KINETIC_DUEL round start score=%d:%d', score.mine, score.theirs))
    roundOver = false
    resetAt = nil
    placePlayer()
    tellPlayer('KineticDuelScore', { mine = score.mine, theirs = score.theirs, roundOver = false })
end

local function endRound(iWon)
    if roundOver then return end
    roundOver = true
    if iWon then score.mine = score.mine + 1 else score.theirs = score.theirs + 1 end
    print(string.format('KINETIC_DUEL round over won=%s score=%d:%d', tostring(iWon), score.mine, score.theirs))
    resetAt = core.getRealTime() + ROUND_PAUSE
    tellPlayer('KineticDuelScore', { mine = score.mine, theirs = score.theirs, roundOver = true, won = iWon })
end

local function onMessage(text)
    local kind, a, b = text:match('^(%w+)|?([^|]*)|?([^|]*)$')
    if kind == 'hp' then
        tellPlayer('KineticDuelOpponentHealth', { current = tonumber(a) or 0, base = tonumber(b) or 0 })
        -- Stats change only in the actor's own (local) script.
        if puppet and puppet:isValid() then
            puppet:sendEvent('KineticDuelPuppetHealth', { current = tonumber(a), base = tonumber(b) })
        end
    elseif kind == 'down' then
        endRound(true)
    end
end

local function onUpdate()
    if net.state() == 'off' then return end
    local p = player()
    if not p then return end
    if not placed then
        placed = true
        giveKit(p)
        placePlayer()
        p:sendEvent('KineticDuelKit', { weapon = WEAPON })
    end
    if net.state() == 'connected' and net.session() ~= session then
        session = net.session()
        score = { mine = 0, theirs = 0 }
        spawnPuppet()
        startRound()
    end
    for _, text in ipairs(net.receive()) do onMessage(text) end
    -- The body can be rebuilt (a cell reload): make it the puppet again.
    if puppet and puppetReady and core.getRealTime() >= puppetCheck then
        puppetCheck = core.getRealTime() + 1
        if puppet:isValid() and not net.isPuppet(puppet) then net.setPuppet(puppet) end
    end
    if resetAt and core.getRealTime() >= resetAt then startRound() end
end

return {
    engineHandlers = { onUpdate = onUpdate },
    eventHandlers = {
        KineticDuelPuppetReady = function(data)
            if data.actor == puppet then
                puppetReady = true
                net.setPuppet(puppet)
            end
        end,
        -- This side's player is down: the opponent takes the round.
        KineticDuelDown = function()
            if roundOver or net.state() ~= 'connected' then return end
            net.send('down')
            endRound(false)
        end,
    },
}

local core = require('openmw.core')
local ragdoll = require('openmw.ragdolltest')
local testing = require('testing_util')
local testArea = require('ragdoll_test_area')
local types = require('openmw.types')
local util = require('openmw.util')
local world = require('openmw.world')

local opponentStrikeCount = 0
local playerStrikeReleaseCount = 0
local playerReady = false
local opponentReady = false
local activeOpponent
local activeReactionCycle
local reactionAttempt = 0

local function waitFor(description, predicate, timeout)
    timeout = timeout or 10
    local simulationDeadline = core.getSimulationTime() + timeout
    local realDeadline = core.getRealTime() + math.max(timeout * 4, 15)
    repeat
        local value = predicate()
        if value then return value end
        coroutine.yield()
    until core.getSimulationTime() >= simulationDeadline or core.getRealTime() >= realDeadline
    error('DUEL-A01 timed out waiting for ' .. description)
end

-- Runs eachFrame for a simulated duration, or until stop() is true.
local function waitSeconds(seconds, eachFrame, stop)
    local deadline = core.getSimulationTime() + seconds
    local realDeadline = core.getRealTime() + math.max(seconds * 4, 15)
    repeat
        if eachFrame then eachFrame() end
        if stop and stop() then return end
        coroutine.yield()
    until core.getSimulationTime() >= deadline or core.getRealTime() >= realDeadline
end

local function jsonString(value)
    local escapes = {
        ['"'] = '\\"', ['\\'] = '\\\\', ['\b'] = '\\b', ['\f'] = '\\f',
        ['\n'] = '\\n', ['\r'] = '\\r', ['\t'] = '\\t',
    }
    return '"' .. value:gsub('[%c\\"]', function(character)
        return escapes[character] or string.format('\\u%04x', string.byte(character))
    end) .. '"'
end

local function emitTelemetry(scenario, metrics)
    local fields = {
        '"schema_version":1',
        '"test_fixture":' .. jsonString(testArea.id),
        '"scenario":' .. jsonString(scenario),
    }
    local keys = {}
    for key in pairs(metrics) do table.insert(keys, key) end
    table.sort(keys)
    for _, key in ipairs(keys) do
        local value = metrics[key]
        local encoded
        if type(value) == 'number' then encoded = string.format('%.17g', value)
        elseif type(value) == 'boolean' then encoded = tostring(value)
        elseif type(value) == 'string' then encoded = jsonString(value)
        else error('Unsupported telemetry field type for ' .. key) end
        table.insert(fields, jsonString(key) .. ':' .. encoded)
    end
    print('RAGDOLL_TELEMETRY_JSON\t{' .. table.concat(fields, ',') .. '}')
end

local function addItem(actor, recordId)
    local inventory = types.Actor.inventory(actor)
    local item = inventory:find(recordId)
    if not item then
        world.createObject(recordId, 1):moveInto(inventory)
        coroutine.yield()
        item = inventory:find(recordId)
    end
    testing.expect(item ~= nil, 'DUEL-A01 could not add ' .. recordId)
end

local function setupOpponent()
    local player = world.players[1]
    player:teleport(testArea.cell, testArea.player, { onGround = true })
    waitFor('player relocation', function()
        local current = player.position
        local horizontal = util.vector3(
            current.x - testArea.player.x, current.y - testArea.player.y, 0)
        return player.cell.name == testArea.cell and horizontal:length() <= 2
    end)
    addItem(player, 'iron warhammer')
    local record = world.createRecord(types.NPC.createRecordDraft({
        template = types.NPC.record(player),
        name = 'DUEL-A01 recovery opponent',
        isEssential = false,
        isPersistent = false,
        isRespawning = false,
    }))
    local opponent = world.createObject(record.id)
    addItem(opponent, 'iron dagger')
    opponent:teleport(testArea.cell, testArea.player + util.vector3(0, 55, 0), { onGround = true })
    opponent:addScript('opponent.lua')
    coroutine.yield()
    player:sendEvent('duelPlayerConfigure')
    opponent:sendEvent('duelOpponentConfigure')
    waitFor('player weapon stance', function() return playerReady end)
    waitFor('opponent weapon stance', function() return opponentReady end)
    return opponent
end

local function waitForMode(opponent, description, wanted, timeout, seen)
    return waitFor(description, function()
        local state = ragdoll.getState(opponent)
        if not state or not state.controller then return false end
        seen[state.controller.mode] = true
        if wanted[state.controller.mode] then return state end
    end, timeout)
end

testing.registerGlobalTest('DUEL-A01 animated attacks recover and resume', function()
    local multiplier = ragdoll.testStepMultiplier()
    if multiplier > 0 then ragdoll.setTestStepActive(true, 904001) end
    local previousScale = ragdoll.getMeleeImpulseScale()
    local opponent
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        ragdoll.setPlayerAlwaysHits(true)
        -- A warhammer connects as a knock-down every time; the absorb band
        -- would otherwise swallow weak wind-up rolls and stall the cycle.
        ragdoll.setMeleeImpulseScale(2.5)
        opponent = setupOpponent()
        activeOpponent = opponent
        ragdoll.setPowered(opponent, true)
        waitForMode(opponent, 'initial powered standing', { standing = true }, 10, {})

        local cycleCount = 3
        local completed = 0
        local totalReactions = 0
        local maximumRecoverySeconds = 0
        for cycle = 1, cycleCount do
            opponent:sendEvent('duelOpponentAttackEnabled', { enabled = false })
            local initial = ragdoll.getState(opponent)
            testing.expect(initial and initial.controller.mode == 'standing',
                'DUEL-A01 cycle did not start standing')
            local reactionBefore = initial.combatReactionCount
            local seen = { standing = true }
            activeReactionCycle = cycle
            reactionAttempt = 0
            world.players[1]:sendEvent('duelPlayerAttackEnabled', { enabled = true, sequence = cycle })
            local reaction = waitFor('animated melee fall ' .. cycle, function()
                local state = ragdoll.getState(opponent)
                if state and state.controller then seen[state.controller.mode] = true end
                if state and state.combatReactionCount > reactionBefore
                    and (state.controller.mode == 'falling' or state.controller.mode == 'fallen') then
                    return state
                end
            end, 12)
            world.players[1]:sendEvent('duelPlayerAttackEnabled', { enabled = false })
            activeReactionCycle = nil
            totalReactions = totalReactions + reaction.combatReactionCount - reactionBefore

            waitForMode(opponent, 'settled fall ' .. cycle, { fallen = true }, 8, seen)
            local recoveryStarted = core.getSimulationTime()
            waitForMode(opponent, 'recovery completion ' .. cycle, { standing = true }, 15, seen)
            local recoverySeconds = core.getSimulationTime() - recoveryStarted
            maximumRecoverySeconds = math.max(maximumRecoverySeconds, recoverySeconds)

            local strikesBefore = opponentStrikeCount
            opponent:sendEvent('duelOpponentAttackEnabled', { enabled = true })
            waitFor('opponent combat resume ' .. cycle, function()
                return opponentStrikeCount > strikesBefore
            end, 5)
            opponent:sendEvent('duelOpponentAttackEnabled', { enabled = false })
            waitForMode(opponent, 'post-counterattack standing ' .. cycle,
                { standing = true }, 15, seen)
            testing.expect(seen.falling or seen.fallen, 'cycle never entered a physical fall')
            testing.expect(seen.fallen, 'cycle never settled as fallen')
            testing.expect(seen.getting_up, 'cycle skipped getting-up')
            testing.expect(seen.blend_back, 'cycle skipped blend-back')
            completed = completed + 1
            emitTelemetry('duel_cycle', {
                cycle = cycle,
                reactions = reaction.combatReactionCount - reactionBefore,
                recovery_seconds = recoverySeconds,
                opponent_resumed = opponentStrikeCount > strikesBefore,
                saw_falling = seen.falling == true,
                saw_fallen = seen.fallen == true,
                saw_getting_up = seen.getting_up == true,
                saw_blend_back = seen.blend_back == true,
            })
        end

        local final = ragdoll.getState(opponent)
        testing.expect(final and final.controller.mode == 'standing',
            'DUEL-A01 opponent did not finish standing')
        emitTelemetry('duel_summary', {
            cycles_requested = cycleCount,
            cycles_completed = completed,
            reactions = totalReactions,
            opponent_strikes = opponentStrikeCount,
            maximum_recovery_seconds = maximumRecoverySeconds,
            final_mode = final.controller.mode,
            soft_locks = 0,
            test_step_multiplier = multiplier,
            maximum_anchor_separation = final.stepQuality.maximumAnchorSeparation,
        })
        testing.expect(final.stepQuality.maximumAnchorSeparation < 1.5,
            string.format('DUEL-A01 joint anchors separated by %.3f units during fall/recovery (%s -> %s, step %s)',
                final.stepQuality.maximumAnchorSeparation, tostring(final.stepQuality.maximumAnchorParentBody),
                tostring(final.stepQuality.maximumAnchorChildBody), tostring(final.stepQuality.maximumAnchorStep)))
    end)

    if opponent and opponent:isValid() then
        opponent:sendEvent('duelOpponentStop')
        ragdoll.setPowered(opponent, false)
        opponent:remove()
    end
    activeOpponent = nil
    activeReactionCycle = nil
    world.players[1]:sendEvent('duelPlayerStop')
    ragdoll.setMeleeImpulseScale(previousScale)
    ragdoll.setPlayerAlwaysHits(false)
    if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
    if not ok then error(message) end
end)

testing.registerGlobalTest('DUEL-A02 npc duel to the death', function()
    local multiplier = ragdoll.testStepMultiplier()
    if multiplier > 0 then ragdoll.setTestStepActive(true, 906001) end
    local previousScale = ragdoll.getMeleeImpulseScale()
    local duelists = {}
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        ragdoll.setMeleeImpulseScale(1.5)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player, { onGround = true })
        waitFor('npc duel player relocation', function()
            local current = player.position
            local horizontal = util.vector3(
                current.x - testArea.player.x, current.y - testArea.player.y, 0)
            return player.cell.name == testArea.cell and horizontal:length() <= 2
        end)
        -- The player must not stand in the arena: both duelists swing at
        -- chest height and the melee detection would spend their blows on the
        -- spectator. Park the player out of reach; the population policy's
        -- acquire radius still covers the duelists.
        player:teleport(testArea.cell, testArea.player + util.vector3(0, -800, 0), { onGround = true })
        waitFor('npc duel spectator relocation', function()
            local current = player.position
            local horizontal = util.vector3(
                current.x - testArea.player.x, current.y - testArea.player.y, 0)
            return player.cell.name == testArea.cell and horizontal:length() >= 700
        end)
        local function makeDuelist(name, offset, armed)
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = types.NPC.record(player),
                name = name,
                isEssential = false,
                isPersistent = false,
                isRespawning = false,
            }))
            local actor = world.createObject(record.id)
            if armed then
                addItem(actor, 'iron dagger')
            end
            actor:teleport(testArea.cell, testArea.player + offset, { onGround = true })
            actor:addScript('duelist.lua')
            return actor, record.id
        end
        local duelistA, recordA = makeDuelist('DUEL-A02 duelist A', util.vector3(0, -45, 0), true)
        local duelistB, recordB = makeDuelist('DUEL-A02 duelist B', util.vector3(0, 45, 0), true)
        duelists = { duelistA, duelistB }
        coroutine.yield()
        duelistA:sendEvent('duelDuelistConfigure')
        duelistB:sendEvent('duelDuelistConfigure')
        -- The melee hit test discards swings at actors that are not in the
        -- attacker's combat target list; register both directions.
        ragdoll.setCombatTarget(duelistA, duelistB)
        ragdoll.setCombatTarget(duelistB, duelistA)
        local function weaponReady(actor)
            -- Armed duelists equip a dagger; unarmed ones only take the
            -- hand-to-hand ready stance. Either way the stance is the gate.
            return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon
        end
        waitFor('duelist A weapon stance', function() return weaponReady(duelistA) end)
        waitFor('duelist B weapon stance', function() return weaponReady(duelistB) end)
        ragdoll.setPowered(duelistA, true)
        ragdoll.setPowered(duelistB, true)
        waitForMode(duelistA, 'duelist A initial standing', { standing = true }, 10, {})
        waitForMode(duelistB, 'duelist B initial standing', { standing = true }, 10, {})

        local startTime = core.getSimulationTime()
        local dead, winner
        local lastTargetRefresh = 0
        waitFor('npc duel death', function()
            local healthA = types.Actor.stats.dynamic.health(duelistA).current
            local healthB = types.Actor.stats.dynamic.health(duelistB).current
            local now = core.getSimulationTime()
            -- The AI combat package can lapse (target lost across a
            -- knock-down or a long chase); keep both directions registered so
            -- the duel runs to a death instead of stalling.
            if now - lastTargetRefresh >= 2 then
                lastTargetRefresh = now
                ragdoll.setCombatTarget(duelistA, duelistB)
                ragdoll.setCombatTarget(duelistB, duelistA)
            end
            if healthA <= 0 or healthB <= 0 then
                dead = healthA <= 0 and duelistA or duelistB
                winner = dead == duelistA and duelistB or duelistA
                return true
            end
        end, 300)
        local duration = core.getSimulationTime() - startTime
        do
            local winnerState = ragdoll.getState(winner)
            local deadState = ragdoll.getState(dead)
            print(string.format(
                'DUEL-A02 death: winner=%s mode=%s health=%.1f | dead=%s powered=%s health=%.1f',
                tostring(winner.id),
                winnerState and winnerState.controller and winnerState.controller.mode or 'n/a',
                types.Actor.stats.dynamic.health(winner).current,
                tostring(dead.id),
                deadState and tostring(deadState.powered) or 'no-ragdoll',
                types.Actor.stats.dynamic.health(dead).current))
        end
        -- The corpse's first 1.5 s dead (RAGDOLL_DEATH_SLIDE: slide and
        -- depth under the floor) and 4 s (RAGDOLL_DEATH_DRAWN: the drawn body
        -- dragged by its actor) are logged before the case ends.
        waitSeconds(4.5)
        -- The survivor must be alive and on its feet. A wounded winner may
        -- brace or flee (ordinary morale) or recover from a knock-down first;
        -- all of those are standing states for gameplay purposes.
        local lastWinnerDebug = 0
        waitFor('npc duel winner recovered', function()
            local state = ragdoll.getState(winner)
            if not state or not state.controller then
                return false
            end
            local now = core.getSimulationTime()
            if now - lastWinnerDebug >= 4 then
                lastWinnerDebug = now
                print(string.format('DUEL-A02 winner mode=%s com=%.1f health=%.1f',
                    state.controller.mode, state.controller.centerOfMassSpeed or -1,
                    types.Actor.stats.dynamic.health(winner).current))
            end
            return state.controller.mode == 'standing' or state.controller.mode == 'bracing'
                or state.controller.mode == 'stumbling'
        end, 30)
        local deadState = ragdoll.getState(dead)
        testing.expect(deadState ~= nil, 'DUEL-A02 dead duelist lost its ragdoll')
        testing.expect(deadState.powered == false, 'DUEL-A02 dead duelist stayed powered')
        local winnerHealth = types.Actor.stats.dynamic.health(winner).current
        testing.expect(winnerHealth > 0, 'DUEL-A02 winner has no health left')
        emitTelemetry('npc_duel', {
            duration_seconds = duration,
            strikes = opponentStrikeCount,
            winner_health = winnerHealth,
            winner_standing = true,
            dead_ragdoll_powered = false,
        })
    end)

    for _, actor in ipairs(duelists) do
        if actor and actor:isValid() then
            actor:sendEvent('duelDuelistStop')
            ragdoll.setPowered(actor, false)
            actor:remove()
        end
    end
    ragdoll.setMeleeImpulseScale(previousScale)
    if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
    if not ok then error(message) end
end)

-- DMG-04 damage course: a stock Hlaalu guard (steel longsword, bonemold,
-- Strength 100, Long Blade 68) against a player-template fighter with an
-- iron claymore, both with their real health. Every landed blow is logged
-- (RAGDOLL_DAMAGE with OPENMW_RAGDOLL_DAMAGE_TRACE, RAGDOLL_DUELIST_HIT);
-- tools/body/damage_report.py compares them with vanilla Morrowind.
testing.registerGlobalTest('DMG-04 guard and claymore duel damage', function()
    local multiplier = ragdoll.testStepMultiplier()
    if multiplier > 0 then ragdoll.setTestStepActive(true, 906004) end
    local duelists = {}
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player + util.vector3(0, -800, 0), { onGround = true })
        waitFor('damage duel spectator relocation', function()
            local current = player.position
            local horizontal = util.vector3(
                current.x - testArea.player.x, current.y - testArea.player.y, 0)
            return player.cell.name == testArea.cell and horizontal:length() >= 700
        end)
        local function makeFighter(name, template, offset, weapon)
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = template,
                name = name,
                isEssential = false,
                isPersistent = false,
                isRespawning = false,
            }))
            local actor = world.createObject(record.id)
            if not types.Actor.inventory(actor):find(weapon) then addItem(actor, weapon) end
            actor:teleport(testArea.cell, testArea.player + offset, { onGround = true })
            actor:addScript('duelist.lua')
            return actor
        end
        local guard = makeFighter('DMG-04 guard', types.NPC.record('hlaalu guard'),
            util.vector3(0, -55, 0), 'steel longsword')
        local fighter = makeFighter('DMG-04 claymore', types.NPC.record(player),
            util.vector3(0, 55, 0), 'iron claymore')
        duelists = { guard, fighter }
        coroutine.yield()
        guard:sendEvent('duelDuelistConfigure', { weapon = 'steel longsword', keepHealth = true })
        fighter:sendEvent('duelDuelistConfigure', { weapon = 'iron claymore', keepHealth = true })
        ragdoll.setCombatTarget(guard, fighter)
        ragdoll.setCombatTarget(fighter, guard)
        for _, actor in ipairs(duelists) do
            waitFor('damage duel weapon stance', function()
                return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon
            end)
            ragdoll.setPowered(actor, true)
        end
        for _, actor in ipairs(duelists) do
            waitForMode(actor, 'damage duel initial standing', { standing = true }, 10, {})
        end
        for _, actor in ipairs(duelists) do
            local stats = types.NPC.stats
            print(string.format('DMG04_FIGHTER %s health=%.0f strength=%d agility=%d longblade=%d fatigue=%.0f',
                actor.recordId, types.Actor.stats.dynamic.health(actor).current,
                types.Actor.stats.attributes.strength(actor).modified, types.Actor.stats.attributes.agility(actor).modified,
                stats.skills.longblade(actor).modified, types.Actor.stats.dynamic.fatigue(actor).current))
        end
        local startTime = core.getSimulationTime()
        local lastTargetRefresh = 0
        waitSeconds(40, function()
            local now = core.getSimulationTime()
            if now - lastTargetRefresh >= 2 then
                lastTargetRefresh = now
                ragdoll.setCombatTarget(guard, fighter)
                ragdoll.setCombatTarget(fighter, guard)
            end
        end, function()
            return types.Actor.stats.dynamic.health(guard).current <= 0
                or types.Actor.stats.dynamic.health(fighter).current <= 0
        end)
        for _, actor in ipairs(duelists) do
            print(string.format('DMG04_END %s health=%.1f fatigue=%.1f after=%.1f s', actor.recordId,
                types.Actor.stats.dynamic.health(actor).current, types.Actor.stats.dynamic.fatigue(actor).current,
                core.getSimulationTime() - startTime))
        end
    end)
    for _, actor in ipairs(duelists) do
        if actor and actor:isValid() then
            actor:sendEvent('duelDuelistStop')
            ragdoll.setPowered(actor, false)
            actor:remove()
        end
    end
    if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
    if not ok then error(message) end
end)

-- NPCHIT-01 (user 10-06, build 62: "the NPCs can't hit me, I cycled through
-- all opponents and they didn't hit me once"): a hub-style opponent (the
-- player's own template, iron longsword) and then a Hlaalu guard (steel
-- longsword) each attack the standing, powered player for 30 s. The test
-- player applies no damage, so the blows are read from the log
-- (RAGDOLL_DAMAGE ... victim="player", tools/body/analysis/npchit01.py)
-- between NPCHIT01_BEGIN/END.
testing.registerGlobalTest('NPCHIT-01 opponents attack a standing player', function()
    local duelists = {}
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player, { onGround = true })
        coroutine.yield()
        ragdoll.setPowered(player, true)
        waitSeconds(1.5)
        for index, case in ipairs({ { 'NPCHIT-01 commoner', types.NPC.record(player), 'iron longsword' },
            { 'NPCHIT-01 guard', types.NPC.record('hlaalu guard'), 'steel longsword' } }) do
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = case[2], name = case[1], isEssential = false, isPersistent = false, isRespawning = false,
            }))
            local actor = world.createObject(record.id)
            if not types.Actor.inventory(actor):find(case[3]) then addItem(actor, case[3]) end
            actor:teleport(testArea.cell, testArea.player + util.vector3(0, 140, 0), { onGround = true })
            actor:addScript('duelist.lua')
            duelists[#duelists + 1] = actor
            coroutine.yield()
            actor:sendEvent('duelDuelistConfigure', { weapon = case[3], keepHealth = true, health = 5000 })
            waitFor('NPCHIT-01 weapon stance', function()
                return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon
            end)
            ragdoll.setPowered(actor, true)
            waitForMode(actor, 'NPCHIT-01 standing', { standing = true }, 10, {})
            ragdoll.setCombatTarget(actor, player)
            print(string.format('NPCHIT01_BEGIN opponent=%s weapon=%s', case[1], case[3]))
            local lastRefresh = core.getSimulationTime()
            waitSeconds(30, function()
                local now = core.getSimulationTime()
                if now - lastRefresh >= 2 then
                    lastRefresh = now
                    ragdoll.setCombatTarget(actor, player)
                end
            end)
            print(string.format('NPCHIT01_END opponent=%s', case[1]))
            actor:sendEvent('duelDuelistStop')
            ragdoll.setPowered(actor, false)
            actor:remove()
            duelists[#duelists] = nil
            waitSeconds(1)
        end
    end)
    for _, actor in ipairs(duelists) do
        if actor and actor:isValid() then
            actor:sendEvent('duelDuelistStop')
            ragdoll.setPowered(actor, false)
            actor:remove()
        end
    end
    ragdoll.setPowered(world.players[1], false)
    if not ok then error(message) end
end)

-- RANGED-02 (user 10-31: "an enemy using a bow wasn't playing the correct
-- animations"): a powered NPC archer (the player's template, long bow, 30
-- iron arrows) shoots at the standing, powered player from 600 units for
-- 20 s. Its bow is a body in its hand, which the melee controllers took for
-- a sword: the attack wheel held the right arm in a guard and the off-hand
-- pulled the bow hand (Morrowind's bow is in the left) down by the belt.
-- Passes when arrows leave and no melee owner (hand control, tip) holds the
-- right arm. Prints the arm owners' tally and how far the bow hand reached
-- out in front of its shoulder (the drawn bow's arm is straight).
testing.registerGlobalTest('RANGED-02 an npc archer shoots the player', function()
    local actor
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player, { onGround = true })
        coroutine.yield()
        ragdoll.setPowered(player, true)
        waitSeconds(1.5)
        local record = world.createRecord(types.NPC.createRecordDraft({
            template = types.NPC.record(player), name = 'RANGED-02 archer', isEssential = false,
            isPersistent = false, isRespawning = false,
        }))
        actor = world.createObject(record.id)
        addItem(actor, 'long bow')
        world.createObject('iron arrow', 30):moveInto(types.Actor.inventory(actor))
        actor:teleport(testArea.cell, testArea.player + util.vector3(0, 600, 0),
            { onGround = true, rotation = util.transform.rotateZ(math.pi) })
        actor:addScript('duelist.lua')
        coroutine.yield()
        actor:sendEvent('duelDuelistConfigure', { weapon = 'long bow', ammo = 'iron arrow', keepHealth = true,
            health = 5000 })
        waitFor('RANGED-02 weapon stance', function()
            return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon
        end)
        ragdoll.setPowered(actor, true)
        waitForMode(actor, 'RANGED-02 standing', { standing = true }, 10, {})
        ragdoll.setCombatTarget(actor, player)
        print('RANGED02_BEGIN')
        local owners = { left = {}, right = {} }
        local frames, melee, reachOut, reachSum, reachMax = 0, 0, 0, 0, 0
        local lastRefresh = core.getSimulationTime()
        waitSeconds(20, function()
            local now = core.getSimulationTime()
            if now - lastRefresh >= 2 then
                lastRefresh = now
                ragdoll.setCombatTarget(actor, player)
            end
            local state = ragdoll.getState(actor)
            if not (state and state.powered and state.bodies) then return end
            frames = frames + 1
            for side, owner in pairs({ left = state.armOwnerLeft or '?', right = state.armOwnerRight or '?' }) do
                owners[side][owner] = (owners[side][owner] or 0) + 1
            end
            if state.hsc or state.tip or state.armOwnerRight == 'hand-control' or state.armOwnerRight == 'tip' then
                melee = melee + 1
            end
            -- The bow hand's reach ahead of its shoulder, in arm lengths.
            local hand, shoulder, elbow
            for _, item in ipairs(state.bodies) do
                if item.name == 'left_hand' then hand = item.position
                elseif item.name == 'left_upper_arm' then shoulder = item.position
                elseif item.name == 'left_forearm' then elbow = item.position end
            end
            if hand and shoulder and elbow then
                local yaw = actor.rotation:getYaw()
                local forward = util.vector3(math.sin(yaw), math.cos(yaw), 0)
                local length = (elbow - shoulder):length() + (hand - elbow):length()
                local ahead = (hand - shoulder):dot(forward) / math.max(length, 1)
                reachSum = reachSum + ahead
                reachMax = math.max(reachMax, ahead)
                if ahead > 0.75 then reachOut = reachOut + 1 end
            end
        end)
        local left = types.Actor.inventory(actor):countOf('iron arrow')
        local function tally(map)
            local parts = {}
            for owner, count in pairs(map) do parts[#parts + 1] = owner .. '=' .. count end
            table.sort(parts)
            return table.concat(parts, ' ')
        end
        print(string.format('RANGED02_END frames=%d arrows_fired=%d melee_frames=%d bow_hand_out=%.2f '
            .. 'bow_hand_mean=%.2f bow_hand_max=%.2f', frames, 30 - left, melee, reachOut / math.max(frames, 1),
            reachSum / math.max(frames, 1), reachMax))
        print('RANGED02_OWNERS right: ' .. tally(owners.right) .. ' | left: ' .. tally(owners.left))
        testing.expect(frames > 0, 'the archer had no powered body')
        testing.expect(left < 30, 'the archer fired no arrow')
        testing.expect(melee == 0, string.format('a melee controller held the bow arm in %d of %d frames', melee,
            frames))
    end)
    if actor and actor:isValid() then
        actor:sendEvent('duelDuelistStop')
        ragdoll.setPowered(actor, false)
        actor:remove()
    end
    ragdoll.setPowered(world.players[1], false)
    if not ok then error(message) end
end)

-- DUEL-X01 (user 10-09: "it is very hard to hit when the weapons clash,
-- making fights too long"): the player (always-hit roll, as in the play hub)
-- attacks continuously while a Hlaalu guard (steel longsword, full AI: guard
-- reading, learning, riposte) fights back, 60 s, both kept alive. The blows,
-- clashes and clash-to-hit times are read from the log between
-- DUELX01_BEGIN/END (RAGDOLL_DAMAGE, RAGDOLL_CLASH).
testing.registerGlobalTest('DUEL-X01 player presses a guarding npc', function()
    local actor
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        ragdoll.setPlayerAlwaysHits(true)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player, { onGround = true })
        coroutine.yield()
        if not types.Actor.inventory(player):find('iron longsword') then addItem(player, 'iron longsword') end
        playerReady = false
        player:sendEvent('duelPlayerConfigure', { weapon = 'iron longsword' })
        waitFor('DUEL-X01 player weapon stance', function() return playerReady end)
        ragdoll.setPowered(player, true)
        waitSeconds(1.5)
        local record = world.createRecord(types.NPC.createRecordDraft({
            template = types.NPC.record('hlaalu guard'), name = 'DUEL-X01 guard', isEssential = false,
            isPersistent = false, isRespawning = false,
        }))
        actor = world.createObject(record.id)
        if not types.Actor.inventory(actor):find('steel longsword') then addItem(actor, 'steel longsword') end
        actor:teleport(testArea.cell, testArea.player + util.vector3(0, 140, 0), { onGround = true })
        actor:addScript('duelist.lua')
        coroutine.yield()
        actor:sendEvent('duelDuelistConfigure', { weapon = 'steel longsword', keepHealth = true, health = 5000 })
        waitFor('DUEL-X01 weapon stance', function()
            return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon
        end)
        ragdoll.setPowered(actor, true)
        waitForMode(actor, 'DUEL-X01 standing', { standing = true }, 10, {})
        ragdoll.setCombatTarget(actor, player)
        ragdoll.prepareMouseInput()
        print('DUELX01_BEGIN')
        -- The player's attacks are the mouse's (hand control): every 1.2 s in
        -- reach, a click chop or a dragged forehand cut in turn.
        local started = core.getSimulationTime()
        local lastRefresh = started
        local attack = 0
        local function drag(x, seconds)
            local from, sent = core.getSimulationTime(), 0
            waitSeconds(seconds, function()
                local f = math.min(1, (core.getSimulationTime() - from) / seconds)
                local v = math.floor(x * f)
                ragdoll.injectMouseInput(v - sent, 0)
                sent = v
            end)
            ragdoll.injectMouseInput(x - sent, 0)
        end
        while core.getSimulationTime() - started < 60 do
            local now = core.getSimulationTime()
            if now - lastRefresh >= 2 then
                lastRefresh = now
                ragdoll.setCombatTarget(actor, player)
            end
            if (actor.position - player.position):length() < 130 then
                attack = attack + 1
                if attack % 2 == 1 then
                    ragdoll.injectMouseInput(0, 0, true, 1)
                    waitSeconds(0.08)
                    ragdoll.injectMouseInput(0, 0, false, 1)
                    waitSeconds(1.1)
                else
                    ragdoll.injectMouseInput(0, 0, true, 1)
                    waitSeconds(0.2)
                    drag(1800, 0.35)
                    drag(-1200, 0.1)
                    waitSeconds(0.05)
                    ragdoll.injectMouseInput(0, 0, false, 1)
                    waitSeconds(0.5)
                end
            else
                coroutine.yield()
            end
        end
        ragdoll.injectMouseInput(0, 0, false)
        print('DUELX01_END attacks=' .. attack)
    end)
    world.players[1]:sendEvent('duelPlayerStop')
    if actor and actor:isValid() then
        actor:sendEvent('duelDuelistStop')
        ragdoll.setPowered(actor, false)
        actor:remove()
    end
    ragdoll.setPlayerAlwaysHits(false)
    ragdoll.setPowered(world.players[1], false)
    if not ok then error(message) end
end)

-- CROWD-01 (user 10-09: "better AI when fighting multiple enemies, they start
-- overlapping each other because they grab the same path to approach the
-- player"; chosen: all attack, spaced): three Hlaalu guards start in a line
-- on one side of the moving player and attack for 40 s. CROWD01 lines give
-- every NPC's and the player's position twice a second (overlap, spread).
testing.registerGlobalTest('CROWD-01 three npcs attack one player', function()
    local actors = {}
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player, { onGround = true })
        coroutine.yield()
        ragdoll.setPowered(player, true)
        waitSeconds(1.5)
        for index = 1, 3 do
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = types.NPC.record('hlaalu guard'), name = 'CROWD-01 guard ' .. index,
                isEssential = false, isPersistent = false, isRespawning = false,
            }))
            local actor = world.createObject(record.id)
            if not types.Actor.inventory(actor):find('steel longsword') then addItem(actor, 'steel longsword') end
            actor:teleport(testArea.cell, testArea.player + util.vector3(10 * index, 260 + 70 * index, 0), { onGround = true })
            actor:addScript('duelist.lua')
            actors[#actors + 1] = actor
            coroutine.yield()
            actor:sendEvent('duelDuelistConfigure', { weapon = 'steel longsword', keepHealth = true, health = 5000 })
            waitFor('CROWD-01 weapon stance', function()
                return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon
            end)
            ragdoll.setPowered(actor, true)
            waitForMode(actor, 'CROWD-01 standing', { standing = true }, 10, {})
        end
        for _, actor in ipairs(actors) do ragdoll.setCombatTarget(actor, player) end
        player:sendEvent('duelPlayerMovePattern', { enabled = true })
        print('CROWD01_BEGIN')
        local started = core.getSimulationTime()
        local lastRefresh, lastLog = started, started
        waitSeconds(40, function()
            local now = core.getSimulationTime()
            if now - lastRefresh >= 2 then
                lastRefresh = now
                for _, actor in ipairs(actors) do ragdoll.setCombatTarget(actor, player) end
            end
            if now - lastLog >= 0.5 then
                lastLog = now
                local parts = { string.format('CROWD01 t=%.1f p=%.0f,%.0f', now - started, player.position.x, player.position.y) }
                for i, actor in ipairs(actors) do
                    parts[#parts + 1] = string.format('a%d=%.0f,%.0f', i, actor.position.x, actor.position.y)
                end
                print(table.concat(parts, ' '))
            end
        end)
        print('CROWD01_END')
        player:sendEvent('duelPlayerMovePattern', { enabled = false })
    end)
    world.players[1]:sendEvent('duelPlayerMovePattern', { enabled = false })
    for _, actor in ipairs(actors) do
        if actor and actor:isValid() then
            actor:sendEvent('duelDuelistStop')
            ragdoll.setPowered(actor, false)
            actor:remove()
        end
    end
    ragdoll.setPowered(world.players[1], false)
    if not ok then error(message) end
end)

-- NPCHIT-03 (user 10-09: "the enemies should have a much better hit rate on
-- the player"): NPCHIT-01 against a player who keeps moving (strafe left,
-- right, back, in at a walk, 1 s each), the Hlaalu guard (steel longsword)
-- and a claymore commoner, 30 s each, framed by NPCHIT03_BEGIN/END.
-- tools/body/analysis/npcaim.py reads the cuts (OPENMW_RAGDOLL_AI_AIM_TRACE=1).
testing.registerGlobalTest('NPCHIT-03 opponents attack a moving player', function()
    local duelists = {}
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player, { onGround = true })
        coroutine.yield()
        ragdoll.setPowered(player, true)
        waitSeconds(1.5)
        for index, case in ipairs({ { 'NPCHIT-03 guard', types.NPC.record('hlaalu guard'), 'steel longsword' },
            { 'NPCHIT-03 claymore', types.NPC.record(player), 'iron claymore' } }) do
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = case[2], name = case[1], isEssential = false, isPersistent = false, isRespawning = false,
            }))
            local actor = world.createObject(record.id)
            if not types.Actor.inventory(actor):find(case[3]) then addItem(actor, case[3]) end
            actor:teleport(testArea.cell, testArea.player + util.vector3(0, 140, 0), { onGround = true })
            actor:addScript('duelist.lua')
            duelists[#duelists + 1] = actor
            coroutine.yield()
            actor:sendEvent('duelDuelistConfigure', { weapon = case[3], keepHealth = true, health = 5000 })
            waitFor('NPCHIT-03 weapon stance', function()
                return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon
            end)
            ragdoll.setPowered(actor, true)
            waitForMode(actor, 'NPCHIT-03 standing', { standing = true }, 10, {})
            ragdoll.setCombatTarget(actor, player)
            player:sendEvent('duelPlayerMovePattern', { enabled = true })
            print(string.format('NPCHIT03_BEGIN opponent=%s weapon=%s', case[1], case[3]))
            local lastRefresh = core.getSimulationTime()
            waitSeconds(30, function()
                local now = core.getSimulationTime()
                if now - lastRefresh >= 2 then
                    lastRefresh = now
                    ragdoll.setCombatTarget(actor, player)
                end
            end)
            print(string.format('NPCHIT03_END opponent=%s', case[1]))
            player:sendEvent('duelPlayerMovePattern', { enabled = false })
            actor:sendEvent('duelDuelistStop')
            ragdoll.setPowered(actor, false)
            actor:remove()
            duelists[#duelists] = nil
            waitSeconds(1)
        end
    end)
    world.players[1]:sendEvent('duelPlayerMovePattern', { enabled = false })
    for _, actor in ipairs(duelists) do
        if actor and actor:isValid() then
            actor:sendEvent('duelDuelistStop')
            ragdoll.setPowered(actor, false)
            actor:remove()
        end
    end
    ragdoll.setPowered(world.players[1], false)
    if not ok then error(message) end
end)

-- DUEL-W01 (user 2026-09-29 after spear duels: "apart from the opponent
-- swinging the weapon wildly"): a spear and a longsword NPC duel for 40 s.
-- Each weapon tip's speed is sampled every frame and split by whether its
-- owner is in an attack. Reports p50/p90/p99 speed and the share of the
-- non-attacking time the tip moved over 600 units/s (flailing).
testing.registerGlobalTest('DUEL-W01 npc weapon control spear and longsword', function()
    local multiplier = ragdoll.testStepMultiplier()
    if multiplier > 0 then ragdoll.setTestStepActive(true, 906011) end
    local duelists = {}
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player + util.vector3(0, -800, 0), { onGround = true })
        waitFor('weapon duel spectator relocation', function()
            local current = player.position
            local horizontal = util.vector3(
                current.x - testArea.player.x, current.y - testArea.player.y, 0)
            return player.cell.name == testArea.cell and horizontal:length() >= 700
        end)
        local function makeFighter(name, offset, weapon)
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = types.NPC.record(player),
                name = name,
                isEssential = false,
                isPersistent = false,
                isRespawning = false,
            }))
            local actor = world.createObject(record.id)
            if not types.Actor.inventory(actor):find(weapon) then addItem(actor, weapon) end
            actor:teleport(testArea.cell, testArea.player + offset, { onGround = true })
            actor:addScript('duelist.lua')
            return actor
        end
        local spear = makeFighter('DUEL-W01 spear', util.vector3(0, -60, 0), 'iron spear')
        local sword = makeFighter('DUEL-W01 longsword', util.vector3(0, 60, 0), 'iron longsword')
        duelists = { spear, sword }
        local weapons = { [spear.id] = 'iron spear', [sword.id] = 'iron longsword' }
        coroutine.yield()
        spear:sendEvent('duelDuelistConfigure', { weapon = 'iron spear', keepHealth = true })
        sword:sendEvent('duelDuelistConfigure', { weapon = 'iron longsword', keepHealth = true })
        ragdoll.setCombatTarget(spear, sword)
        ragdoll.setCombatTarget(sword, spear)
        for _, actor in ipairs(duelists) do
            waitFor('weapon duel stance', function()
                return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon
            end)
            ragdoll.setPowered(actor, true)
        end
        for _, actor in ipairs(duelists) do
            waitForMode(actor, 'weapon duel initial standing', { standing = true }, 10, {})
        end
        local samples, last = {}, {}
        for _, actor in ipairs(duelists) do samples[actor.id] = { idle = {}, attack = {} } end
        local startTime = core.getSimulationTime()
        local lastTargetRefresh = 0
        waitSeconds(40, function()
            local now = core.getSimulationTime()
            if now - lastTargetRefresh >= 2 then
                lastTargetRefresh = now
                ragdoll.setCombatTarget(spear, sword)
                ragdoll.setCombatTarget(sword, spear)
            end
            for _, actor in ipairs(duelists) do
                local state = ragdoll.getState(actor)
                if state and state.powered and state.weapon and state.weapon.bodyIndex then
                    local b = state.bodies[state.weapon.bodyIndex]
                    local tip = b.position + (b.rotation * util.vector3(0, 0, 1)) * (state.weapon.length * 0.5)
                    -- The tip relative to the chest: the fighters' own travel
                    -- is not the weapon flailing.
                    local chest
                    for _, item in ipairs(state.bodies) do if item.name == 'spine2' then chest = item.position end end
                    local rel = chest and (tip - chest) or tip
                    local previous = last[actor.id]
                    if previous and now > previous.t then
                        local speed = (rel - previous.rel):length() / (now - previous.t)
                        local attacking = (state.attackPhase and state.attackPhase ~= 'idle')
                            or ragdoll.getSwingSpeed(actor) ~= nil
                        table.insert(samples[actor.id][attacking and 'attack' or 'idle'], speed)
                    end
                    last[actor.id] = { rel = rel, t = now }
                end
            end
        end, function()
            return types.Actor.stats.dynamic.health(spear).current <= 0
                or types.Actor.stats.dynamic.health(sword).current <= 0
        end)
        local function pct(values, q)
            if #values == 0 then return -1 end
            table.sort(values)
            return values[math.max(1, math.ceil(q * #values))]
        end
        for _, actor in ipairs(duelists) do
            local s = samples[actor.id]
            local wild = 0
            for _, v in ipairs(s.idle) do if v > 600 then wild = wild + 1 end end
            print(string.format('DUEL-W01 %s idle n=%d p50=%.0f p90=%.0f p99=%.0f over600=%.2f | attack n=%d p50=%.0f p90=%.0f p99=%.0f | after=%.1f s',
                weapons[actor.id], #s.idle, pct(s.idle, 0.5), pct(s.idle, 0.9), pct(s.idle, 0.99),
                #s.idle > 0 and wild / #s.idle or -1, #s.attack, pct(s.attack, 0.5), pct(s.attack, 0.9),
                pct(s.attack, 0.99), core.getSimulationTime() - startTime))
        end
    end)
    for _, actor in ipairs(duelists) do
        if actor and actor:isValid() then
            actor:sendEvent('duelDuelistStop')
            ragdoll.setPowered(actor, false)
            actor:remove()
        end
    end
    if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
    if not ok then error(message) end
end)

-- DUEL-C01 (alpha 0.4 item 4, user 2026-09-30: "an opponent who retreats is
-- sprinted down and lunged at in reach"): a longsword duelist faces a foe
-- that stands for 3 s, then backs away running for 2.5 s, then stands. The
-- foe does not fight back. Reports the gap (units) when the retreat starts,
-- its largest value, 1 s after the retreat ends, the chaser's mean speed
-- during the retreat, and the health the foe lost from the retreat's start
-- to 2 s after it ended.
testing.registerGlobalTest('DUEL-C01 a retreating foe is chased down', function()
    local multiplier = ragdoll.testStepMultiplier()
    if multiplier > 0 then ragdoll.setTestStepActive(true, 906021) end
    local actors = {}
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player + util.vector3(0, -900, 0), { onGround = true })
        waitFor('chase spectator relocation', function()
            local current = player.position
            local horizontal = util.vector3(current.x - testArea.player.x, current.y - testArea.player.y, 0)
            return player.cell.name == testArea.cell and horizontal:length() >= 800
        end)
        local function make(name, offset, weapon, rotation)
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = types.NPC.record(player), name = name,
                isEssential = false, isPersistent = false, isRespawning = false,
            }))
            local actor = world.createObject(record.id)
            if weapon and not types.Actor.inventory(actor):find(weapon) then addItem(actor, weapon) end
            actor:teleport(testArea.cell, testArea.player + offset, { onGround = true, rotation = rotation })
            actor:addScript('duelist.lua')
            return actor
        end
        local chaser = make('DUEL-C01 chaser', util.vector3(0, -50, 0), 'iron longsword', util.transform.rotateZ(0))
        local foe = make('DUEL-C01 foe', util.vector3(0, 60, 0), 'iron dagger', util.transform.rotateZ(math.pi))
        actors = { chaser, foe }
        coroutine.yield()
        chaser:sendEvent('duelDuelistConfigure', { weapon = 'iron longsword', keepHealth = true })
        foe:sendEvent('duelDuelistConfigure', { weapon = 'iron dagger', health = 2000, passive = true })
        waitSeconds(0.2)
        ragdoll.setCombatTarget(chaser, foe)
        for _, actor in ipairs(actors) do
            waitFor('chase stance', function() return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon end)
            ragdoll.setPowered(actor, true)
        end
        for _, actor in ipairs(actors) do waitForMode(actor, 'chase standing', { standing = true }, 10, {}) end
        local function gap() return (chaser.position - foe.position):length() end
        waitSeconds(3, function() ragdoll.setCombatTarget(chaser, foe) end)
        local gapStart, gapMax = gap(), gap()
        local health0 = types.Actor.stats.dynamic.health(foe).current
        local travelled, lastPos = 0, chaser.position
        foe:sendEvent('duelDuelistRetreat', { on = true })
        waitSeconds(2.5, function()
            gapMax = math.max(gapMax, gap())
            travelled = travelled + (chaser.position - lastPos):length()
            lastPos = chaser.position
        end)
        foe:sendEvent('duelDuelistRetreat', { on = false })
        waitSeconds(1)
        local gapAfter = gap()
        waitSeconds(1)
        local lost = health0 - types.Actor.stats.dynamic.health(foe).current
        print(string.format('DUEL-C01 summary gap_start=%.0f gap_max=%.0f gap_after=%.0f chaser_speed=%.0f lost=%.1f',
            gapStart, gapMax, gapAfter, travelled / 2.5, lost))
    end)
    for _, actor in ipairs(actors) do
        if actor and actor:isValid() then
            actor:sendEvent('duelDuelistStop')
            ragdoll.setPowered(actor, false)
            actor:remove()
        end
    end
    if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
    if not ok then error(message) end
end)

-- PARRY-01 (alpha 0.4 item 5): two longsword fighters spar for 45 s with
-- deep health, so their blades meet often. The engine logs every clash
-- (RAGDOLL_PARRY_ROLES: both blades' speeds at the contact, which one was
-- the attack, the kicks); tools/body/analysis/parry.py scores them.
testing.registerGlobalTest('PARRY-01 longsword sparring clashes', function()
    local multiplier = ragdoll.testStepMultiplier()
    if multiplier > 0 then ragdoll.setTestStepActive(true, 906031) end
    local actors = {}
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player + util.vector3(0, -900, 0), { onGround = true })
        waitFor('parry spectator relocation', function()
            local current = player.position
            local horizontal = util.vector3(current.x - testArea.player.x, current.y - testArea.player.y, 0)
            return player.cell.name == testArea.cell and horizontal:length() >= 800
        end)
        local function make(name, offset)
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = types.NPC.record(player), name = name,
                isEssential = false, isPersistent = false, isRespawning = false,
            }))
            local actor = world.createObject(record.id)
            if not types.Actor.inventory(actor):find('iron longsword') then addItem(actor, 'iron longsword') end
            actor:teleport(testArea.cell, testArea.player + offset, { onGround = true })
            actor:addScript('duelist.lua')
            return actor
        end
        local a = make('PARRY-01 a', util.vector3(0, -60, 0))
        local b = make('PARRY-01 b', util.vector3(0, 60, 0))
        actors = { a, b }
        coroutine.yield()
        for _, actor in ipairs(actors) do actor:sendEvent('duelDuelistConfigure', { weapon = 'iron longsword', health = 600 }) end
        ragdoll.setCombatTarget(a, b)
        ragdoll.setCombatTarget(b, a)
        for _, actor in ipairs(actors) do
            waitFor('parry stance', function() return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon end)
            ragdoll.setPowered(actor, true)
        end
        for _, actor in ipairs(actors) do waitForMode(actor, 'parry standing', { standing = true }, 10, {}) end
        local last = 0
        waitSeconds(45, function()
            local now = core.getSimulationTime()
            if now - last >= 2 then
                last = now
                ragdoll.setCombatTarget(a, b)
                ragdoll.setCombatTarget(b, a)
            end
        end)
        print('PARRY-01 done')
    end)
    for _, actor in ipairs(actors) do
        if actor and actor:isValid() then
            actor:sendEvent('duelDuelistStop')
            ragdoll.setPowered(actor, false)
            actor:remove()
        end
    end
    if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
    if not ok then error(message) end
end)

-- ARMS-01 (user bookmark 7, 2026-10-02: "his arm is stuck behind his back"):
-- the PARRY-01 sparring with two-handers; the engine logs RAGDOLL_ARM_BEHIND
-- (and _END with its length) and RAGDOLL_ARM_ROUTE. The steel longsword is
-- the 2026-10-31 hub bookmark's ("the npc will try to hit with the sword
-- sticking out the left ribs").
for _, armsWeapon in ipairs({ 'iron claymore', 'iron warhammer', 'steel longsword' }) do
testing.registerGlobalTest('ARMS-01 sparring with ' .. armsWeapon, function()
    local multiplier = ragdoll.testStepMultiplier()
    if multiplier > 0 then ragdoll.setTestStepActive(true, 906031) end
    local actors = {}
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player + util.vector3(0, -900, 0), { onGround = true })
        waitFor('parry spectator relocation', function()
            local current = player.position
            local horizontal = util.vector3(current.x - testArea.player.x, current.y - testArea.player.y, 0)
            return player.cell.name == testArea.cell and horizontal:length() >= 800
        end)
        local function make(name, offset)
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = types.NPC.record(player), name = name,
                isEssential = false, isPersistent = false, isRespawning = false,
            }))
            local actor = world.createObject(record.id)
            if not types.Actor.inventory(actor):find(armsWeapon) then addItem(actor, armsWeapon) end
            actor:teleport(testArea.cell, testArea.player + offset, { onGround = true })
            actor:addScript('duelist.lua')
            return actor
        end
        local a = make('ARMS-01 a', util.vector3(0, -60, 0))
        local b = make('ARMS-01 b', util.vector3(0, 60, 0))
        actors = { a, b }
        coroutine.yield()
        for _, actor in ipairs(actors) do actor:sendEvent('duelDuelistConfigure', { weapon = armsWeapon, health = 600 }) end
        ragdoll.setCombatTarget(a, b)
        ragdoll.setCombatTarget(b, a)
        for _, actor in ipairs(actors) do
            waitFor('parry stance', function() return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon end)
            ragdoll.setPowered(actor, true)
        end
        for _, actor in ipairs(actors) do waitForMode(actor, 'parry standing', { standing = true }, 10, {}) end
        local last = 0
        waitSeconds(45, function()
            local now = core.getSimulationTime()
            if now - last >= 2 then
                last = now
                ragdoll.setCombatTarget(a, b)
                ragdoll.setCombatTarget(b, a)
            end
        end)
        print('ARMS-01 done ' .. armsWeapon)
    end)
    for _, actor in ipairs(actors) do
        if actor and actor:isValid() then
            actor:sendEvent('duelDuelistStop')
            ragdoll.setPowered(actor, false)
            actor:remove()
        end
    end
    if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
    if not ok then error(message) end
end)
end


-- SKILL-01 (Alpha 1.0 skill parity: "landed-and-bit rate per skill
-- 5/30/60/90 within 5 points of vanilla's chance"): per skill a 40 s
-- longsword duel, the attacker at that Long Blade skill, the defender at 50.
-- The engine logs every blow's roll (RAGDOLL_HIT_ROLL chance, bites);
-- tools/body/analysis/skill01.py compares the bite rate with the mean chance
-- per attacker. The test itself only checks that blows were rolled.
for _, skill in ipairs({ 5, 30, 60, 90 }) do
testing.registerGlobalTest('SKILL-01 longsword duel at Long Blade ' .. skill, function()
    local actors = {}
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player + util.vector3(0, -900, 0), { onGround = true })
        waitFor('skill spectator relocation', function()
            local current = player.position
            local horizontal = util.vector3(current.x - testArea.player.x, current.y - testArea.player.y, 0)
            return player.cell.name == testArea.cell and horizontal:length() >= 800
        end)
        local function make(name, offset)
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = types.NPC.record(player), name = name,
                isEssential = false, isPersistent = false, isRespawning = false,
            }))
            local actor = world.createObject(record.id)
            if not types.Actor.inventory(actor):find('iron longsword') then addItem(actor, 'iron longsword') end
            actor:teleport(testArea.cell, testArea.player + offset, { onGround = true })
            actor:addScript('duelist.lua')
            return actor
        end
        local a = make('SKILL-01 attacker ' .. skill, util.vector3(0, -60, 0))
        local b = make('SKILL-01 defender', util.vector3(0, 60, 0))
        actors = { a, b }
        coroutine.yield()
        a:sendEvent('duelDuelistConfigure', { weapon = 'iron longsword', health = 2000, longblade = skill })
        b:sendEvent('duelDuelistConfigure', { weapon = 'iron longsword', health = 2000, longblade = 50 })
        ragdoll.setCombatTarget(a, b)
        ragdoll.setCombatTarget(b, a)
        for _, actor in ipairs(actors) do
            waitFor('skill stance', function() return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon end)
            ragdoll.setPowered(actor, true)
        end
        for _, actor in ipairs(actors) do waitForMode(actor, 'skill standing', { standing = true }, 10, {}) end
        print(string.format('SKILL-01 attacker=%s skill=%d', a.recordId, skill))
        local last = 0
        waitSeconds(40, function()
            local now = core.getSimulationTime()
            if now - last >= 2 then
                last = now
                ragdoll.setCombatTarget(a, b)
                ragdoll.setCombatTarget(b, a)
            end
        end)
        print(string.format('SKILL-01 done skill=%d', skill))
    end)
    for _, actor in ipairs(actors) do
        if actor and actor:isValid() then
            actor:sendEvent('duelDuelistStop')
            ragdoll.setPowered(actor, false)
            actor:remove()
        end
    end
    if not ok then error(message) end
end)
end

-- SHIELD-01 (Alpha 1.0 "shields"): two longsword duellists spar for 45 s,
-- one carrying an iron shield. The engine logs the shield blocks
-- (RAGDOLL_PARRY kind=shield) and the blows that got through
-- (RAGDOLL_DAMAGE); OPENMW_RAGDOLL_SHIELD_BODY=0 is the arm-only baseline.
-- Passes when the shield carrier ends the fight with a shield body built.
-- SHIELD-03 (user 10-07: "A good hit with the hammer vs a shield should push
-- back and cause stagger"): the same with a warhammer against the shield.
for _, shieldCase in ipairs({ { 'SHIELD-01 a shield blocks longsword blows', 'iron longsword' },
    { 'SHIELD-03 a shield blocks warhammer blows', 'iron warhammer' } }) do
testing.registerGlobalTest(shieldCase[1], function()
    local attackWeapon = shieldCase[2]
    local multiplier = ragdoll.testStepMultiplier()
    if multiplier > 0 then ragdoll.setTestStepActive(true, 906041) end
    local actors = {}
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player + util.vector3(0, -900, 0), { onGround = true })
        waitFor('shield spectator relocation', function()
            local current = player.position
            local horizontal = util.vector3(current.x - testArea.player.x, current.y - testArea.player.y, 0)
            return player.cell.name == testArea.cell and horizontal:length() >= 800
        end)
        local function make(name, offset, shield)
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = types.NPC.record(player), name = name,
                isEssential = false, isPersistent = false, isRespawning = false,
            }))
            local actor = world.createObject(record.id)
            local weapon = shield and 'iron longsword' or attackWeapon
            if not types.Actor.inventory(actor):find(weapon) then addItem(actor, weapon) end
            if shield and not types.Actor.inventory(actor):find(shield) then addItem(actor, shield) end
            actor:teleport(testArea.cell, testArea.player + offset, { onGround = true })
            actor:addScript('duelist.lua')
            return actor
        end
        local a = make('SHIELD-01 shield', util.vector3(0, -60, 0), 'iron_shield')
        local b = make('SHIELD-01 sword', util.vector3(0, 60, 0))
        actors = { a, b }
        coroutine.yield()
        a:sendEvent('duelDuelistConfigure', { weapon = 'iron longsword', shield = 'iron_shield', health = 600 })
        b:sendEvent('duelDuelistConfigure', { weapon = attackWeapon, health = 600 })
        ragdoll.setCombatTarget(a, b)
        ragdoll.setCombatTarget(b, a)
        for _, actor in ipairs(actors) do
            waitFor('shield stance', function() return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon end)
            ragdoll.setPowered(actor, true)
        end
        for _, actor in ipairs(actors) do waitForMode(actor, 'shield standing', { standing = true }, 10, {}) end
        waitFor('shield equipped', function()
            return types.Actor.getEquipment(a)[types.Actor.EQUIPMENT_SLOT.CarriedLeft] ~= nil
        end, 5)
        local last = 0
        waitSeconds(45, function()
            local now = core.getSimulationTime()
            if now - last >= 2 then
                last = now
                ragdoll.setCombatTarget(a, b)
                ragdoll.setCombatTarget(b, a)
            end
        end)
        print(string.format('SHIELD-01 done health shield=%.0f sword=%.0f',
            types.Actor.stats.dynamic.health(a).current, types.Actor.stats.dynamic.health(b).current))
    end)
    for _, actor in ipairs(actors) do
        if actor and actor:isValid() then
            actor:sendEvent('duelDuelistStop')
            ragdoll.setPowered(actor, false)
            actor:remove()
        end
    end
    if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
    if not ok then error(message) end
end)
end

-- GUARD-01 (alpha 0.4 items 5-6): the player holds the guard (C) with a
-- weapon while a claymore duellist attacks for 30 s. Reports the player
-- weapon's angular speed (how far blows knock the guard: p50, p90, share of
-- frames over 6 rad/s); the engine logs the blows that got through
-- (RAGDOLL_DAMAGE victim=player) and the clashes (RAGDOLL_PARRY_ROLES).
testing.registerGlobalTest('GUARD-01 a held guard against a claymore', function()
    local multiplier = ragdoll.testStepMultiplier()
    if multiplier > 0 then ragdoll.setTestStepActive(true, 906041) end
    local attacker
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player, { onGround = true, rotation = util.transform.rotateZ(0) })
        waitFor('guard player relocation', function()
            local current = player.position
            local horizontal = util.vector3(current.x - testArea.player.x, current.y - testArea.player.y, 0)
            return player.cell.name == testArea.cell and horizontal:length() <= 2
        end)
        if not types.Actor.inventory(player):find('iron longsword') then addItem(player, 'iron longsword') end
        local record = world.createRecord(types.NPC.createRecordDraft({
            template = types.NPC.record(player), name = 'GUARD-01 attacker',
            isEssential = false, isPersistent = false, isRespawning = false,
        }))
        attacker = world.createObject(record.id)
        addItem(attacker, 'iron claymore')
        attacker:teleport(testArea.cell, testArea.player + util.vector3(0, 110, 0), { onGround = true, rotation = util.transform.rotateZ(math.pi) })
        attacker:addScript('duelist.lua')
        coroutine.yield()
        playerReady = false
        player:sendEvent('duelPlayerConfigure')
        attacker:sendEvent('duelDuelistConfigure', { weapon = 'iron claymore', health = 600 })
        waitFor('player weapon stance', function() return playerReady end)
        waitFor('attacker stance', function() return types.Actor.getStance(attacker) == types.Actor.STANCE.Weapon end)
        ragdoll.setPowered(player, true)
        ragdoll.setPowered(attacker, true)
        waitForMode(player, 'guard player standing', { standing = true }, 10, {})
        waitForMode(attacker, 'guard attacker standing', { standing = true }, 10, {})
        ragdoll.setCombatTarget(attacker, player)
        local speeds, last = {}, 0
        -- The attacker after each hard block (item 8): how far its pelvis
        -- moves in the next 0.6 s, and frames it spends bracing.
        local function body(state, name)
            for _, item in ipairs(state.bodies) do if item.name == name then return item end end
        end
        local staggers0 = (ragdoll.getState(attacker) or {}).blockStaggers or 0
        local pending, shoves, bracing = {}, {}, 0
        print('GUARD-01 start')
        waitSeconds(30, function()
            ragdoll.setGuard(player, true, util.vector3(0, 1, 0))
            local now = core.getSimulationTime()
            if now - last >= 2 then last = now ragdoll.setCombatTarget(attacker, player) end
            local s = ragdoll.getState(player)
            if s and s.weapon and s.weapon.bodyIndex then
                speeds[#speeds + 1] = s.bodies[s.weapon.bodyIndex].angularVelocity:length()
            end
            local a = ragdoll.getState(attacker)
            if a then
                local pelvis = body(a, 'pelvis').position
                if a.controller and a.controller.mode == 'bracing' then bracing = bracing + 1 end
                if (a.blockStaggers or 0) > staggers0 then
                    staggers0 = a.blockStaggers
                    pending[#pending + 1] = { t = now, from = pelvis, max = 0 }
                end
                for i = #pending, 1, -1 do
                    local p = pending[i]
                    p.max = math.max(p.max, (util.vector3(pelvis.x, pelvis.y, 0) - util.vector3(p.from.x, p.from.y, 0)):length())
                    if now - p.t >= 0.6 then shoves[#shoves + 1] = p.max table.remove(pending, i) end
                end
            end
        end)
        print('GUARD-01 end')
        table.sort(speeds)
        local over = 0
        for _, v in ipairs(speeds) do if v > 6 then over = over + 1 end end
        local shoveSum = 0
        for _, v in ipairs(shoves) do shoveSum = shoveSum + v end
        print(string.format('GUARD-01 summary frames=%d spin_p50=%.2f spin_p90=%.2f over6=%.3f staggers=%d shove=%.1f attacker_bracing=%d', #speeds,
            speeds[math.ceil(#speeds * 0.5)] or -1, speeds[math.ceil(#speeds * 0.9)] or -1, #speeds > 0 and over / #speeds or -1,
            #shoves, #shoves > 0 and shoveSum / #shoves or -1, bracing))
    end)
    ragdoll.setGuard(world.players[1], false)
    if attacker and attacker:isValid() then
        attacker:sendEvent('duelDuelistStop')
        ragdoll.setPowered(attacker, false)
        attacker:remove()
    end
    world.players[1]:sendEvent('duelPlayerStop')
    if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
    if not ok then error(message) end
end)

-- DUEL-C02 (user 2026-09-30 bookmark: "enemies should be able to hit the
-- player while he's down"): a passive foe is knocked down beside a longsword
-- duellist, once with Fight 90 and once with Fight 10. Reports the health
-- the foe lost in the 4 s after the knockdown.
for _, fight in ipairs({ 90, 10 }) do
    testing.registerGlobalTest('DUEL-C02 a downed foe is struck by fight ' .. fight, function()
        local multiplier = ragdoll.testStepMultiplier()
        if multiplier > 0 then ragdoll.setTestStepActive(true, 906050 + fight) end
        local actors = {}
        local ok, message = pcall(function()
            ragdoll.setAutomaticRecovery(true)
            local player = world.players[1]
            player:teleport(testArea.cell, testArea.player + util.vector3(0, -900, 0), { onGround = true })
            waitFor('downed spectator relocation', function()
                local current = player.position
                local horizontal = util.vector3(current.x - testArea.player.x, current.y - testArea.player.y, 0)
                return player.cell.name == testArea.cell and horizontal:length() >= 800
            end)
            local function make(name, offset, weapon, rotation)
                local record = world.createRecord(types.NPC.createRecordDraft({
                    template = types.NPC.record(player), name = name,
                    isEssential = false, isPersistent = false, isRespawning = false,
                }))
                local actor = world.createObject(record.id)
                if not types.Actor.inventory(actor):find(weapon) then addItem(actor, weapon) end
                actor:teleport(testArea.cell, testArea.player + offset, { onGround = true, rotation = rotation })
                actor:addScript('duelist.lua')
                return actor
            end
            local striker = make('DUEL-C02 striker', util.vector3(0, -50, 0), 'iron longsword', util.transform.rotateZ(0))
            local foe = make('DUEL-C02 foe', util.vector3(0, 60, 0), 'iron dagger', util.transform.rotateZ(math.pi))
            actors = { striker, foe }
            coroutine.yield()
            striker:sendEvent('duelDuelistConfigure', { weapon = 'iron longsword', keepHealth = true, fight = fight })
            foe:sendEvent('duelDuelistConfigure', { weapon = 'iron dagger', health = 2000, passive = true })
            waitSeconds(0.2)
            ragdoll.setCombatTarget(striker, foe)
            for _, actor in ipairs(actors) do
                waitFor('downed stance', function() return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon end)
                ragdoll.setPowered(actor, true)
            end
            for _, actor in ipairs(actors) do waitForMode(actor, 'downed standing', { standing = true }, 10, {}) end
            waitSeconds(2, function() ragdoll.setCombatTarget(striker, foe) end)
            ragdoll.applyCombatImpulse(foe, util.vector3(0, 7000, 0), foe.position + util.vector3(0, 0, 100))
            waitForMode(foe, 'foe down', { falling = true, fallen = true }, 3, {})
            local health0 = types.Actor.stats.dynamic.health(foe).current
            waitSeconds(4, function() ragdoll.setCombatTarget(striker, foe) end)
            print(string.format('DUEL-C02 fight=%d summary lost=%.1f', fight, health0 - types.Actor.stats.dynamic.health(foe).current))
        end)
        for _, actor in ipairs(actors) do
            if actor and actor:isValid() then
                actor:sendEvent('duelDuelistStop')
                ragdoll.setPowered(actor, false)
                actor:remove()
            end
        end
        if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
        if not ok then error(message) end
    end)
end

-- DUEL-L01 (user 2026-09-30 bookmark: "two characters bump and lock onto
-- each other as they do not strafe around"): two longsword fighters duel 40
-- s with deep health. Reports the share of time their pelvises are closer
-- than 40 units (pressed together) and how far the line between them turned
-- (degrees, summed: circling).
testing.registerGlobalTest('DUEL-L01 footwork longsword pair', function()
    local multiplier = ragdoll.testStepMultiplier()
    if multiplier > 0 then ragdoll.setTestStepActive(true, 906061) end
    local actors = {}
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player + util.vector3(0, -900, 0), { onGround = true })
        waitFor('footwork spectator relocation', function()
            local current = player.position
            local horizontal = util.vector3(current.x - testArea.player.x, current.y - testArea.player.y, 0)
            return player.cell.name == testArea.cell and horizontal:length() >= 800
        end)
        local function make(name, offset)
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = types.NPC.record(player), name = name,
                isEssential = false, isPersistent = false, isRespawning = false,
            }))
            local actor = world.createObject(record.id)
            if not types.Actor.inventory(actor):find('iron longsword') then addItem(actor, 'iron longsword') end
            actor:teleport(testArea.cell, testArea.player + offset, { onGround = true })
            actor:addScript('duelist.lua')
            return actor
        end
        local a = make('DUEL-L01 a', util.vector3(0, -60, 0))
        local b = make('DUEL-L01 b', util.vector3(0, 60, 0))
        actors = { a, b }
        coroutine.yield()
        for _, actor in ipairs(actors) do actor:sendEvent('duelDuelistConfigure', { weapon = 'iron longsword', health = 600 }) end
        ragdoll.setCombatTarget(a, b)
        ragdoll.setCombatTarget(b, a)
        for _, actor in ipairs(actors) do
            waitFor('footwork stance', function() return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon end)
            ragdoll.setPowered(actor, true)
        end
        for _, actor in ipairs(actors) do waitForMode(actor, 'footwork standing', { standing = true }, 10, {}) end
        local frames, pressed, turned, lastAngle, last = 0, 0, 0, nil, 0
        local function pelvis(actor)
            local s = ragdoll.getState(actor)
            if not s then return actor.position end
            for _, item in ipairs(s.bodies) do if item.name == 'pelvis' then return item.position end end
            return actor.position
        end
        waitSeconds(40, function()
            local now = core.getSimulationTime()
            if now - last >= 2 then
                last = now
                ragdoll.setCombatTarget(a, b)
                ragdoll.setCombatTarget(b, a)
            end
            local d = pelvis(b) - pelvis(a)
            local flat = util.vector3(d.x, d.y, 0)
            frames = frames + 1
            if flat:length() < 40 then pressed = pressed + 1 end
            local angle = math.deg(math.atan2(flat.x, flat.y))
            if lastAngle then
                local step = angle - lastAngle
                while step > 180 do step = step - 360 end
                while step < -180 do step = step + 360 end
                turned = turned + math.abs(step)
            end
            lastAngle = angle
        end)
        print(string.format('DUEL-L01 summary pressed=%.3f turned=%.0f frames=%d', pressed / frames, turned, frames))
    end)
    for _, actor in ipairs(actors) do
        if actor and actor:isValid() then
            actor:sendEvent('duelDuelistStop')
            ragdoll.setPowered(actor, false)
            actor:remove()
        end
    end
    if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
    if not ok then error(message) end
end)

-- BODYPUSH-01 (user bookmark 15, 2026-09-30: "strength or endurance should
-- define how much a ragdoll pushes another when they come into contact"): a
-- strong body (Strength/Endurance 100) walks into a weak one (30) for 3 s, then
-- the weak one into the strong one. Reports how far the pushed body moved.
testing.registerGlobalTest('BODYPUSH-01 the stronger body shoves the weaker', function()
    local actors = {}
    local ok, message = pcall(function()
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player + util.vector3(0, -900, 0), { onGround = true })
        waitFor('push spectator relocation', function()
            local current = player.position
            return player.cell.name == testArea.cell
                and util.vector3(current.x - testArea.player.x, current.y - testArea.player.y, 0):length() >= 800
        end)
        local function make(name, offset, yaw)
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = types.NPC.record(player), name = name,
                isEssential = false, isPersistent = false, isRespawning = false,
            }))
            local actor = world.createObject(record.id)
            actor:teleport(testArea.cell, testArea.player + offset,
                { onGround = true, rotation = util.transform.rotateZ(yaw) })
            actor:addScript('duelist.lua')
            return actor
        end
        local function pelvis(actor)
            local s = ragdoll.getState(actor)
            if s then for _, item in ipairs(s.bodies) do if item.name == 'pelvis' then return item.position end end end
            return actor.position
        end
        for _, case in ipairs({ { 'strong pushes weak', 100, 30 }, { 'weak pushes strong', 30, 100 } }) do
            local pusher = make('BODYPUSH-01 pusher', util.vector3(0, -45, 0), 0)
            local pushed = make('BODYPUSH-01 pushed', util.vector3(0, 20, 0), math.pi)
            actors = { pusher, pushed }
            coroutine.yield()
            pusher:sendEvent('duelDuelistConfigure', { passive = true, health = 600, strength = case[2], endurance = case[2] })
            pushed:sendEvent('duelDuelistConfigure', { passive = true, health = 600, strength = case[3], endurance = case[3] })
            waitSeconds(0.5)
            for _, actor in ipairs(actors) do
                ragdoll.setPowered(actor, true)
                waitForMode(actor, 'push standing', { standing = true }, 10, {})
            end
            waitSeconds(1)
            local start = pelvis(pushed)
            pusher:sendEvent('duelDuelistWalk', true)
            waitSeconds(3)
            pusher:sendEvent('duelDuelistWalk', false)
            local moved = pelvis(pushed) - start
            print(string.format('BODYPUSH-01 %s pushed_moved=%.1f', case[1], util.vector3(moved.x, moved.y, 0):length()))
            for _, actor in ipairs(actors) do ragdoll.setPowered(actor, false); actor:remove() end
            actors = {}
            waitSeconds(0.5)
        end
    end)
    for _, actor in ipairs(actors) do if actor and actor:isValid() then ragdoll.setPowered(actor, false); actor:remove() end end
    if not ok then error(message) end
end)

-- SOAK-01 (Alpha 1.0: "no crash in a 1-hour soak"): two pairs of powered
-- duelists fight with rotating weapons for SOAK_MINUTES of game time; a dead
-- fighter is replaced by a fresh one. Reports rounds and deaths every 5 minutes.
-- SOAK-02 (the dying shuffle, user bookmark 20): the same for 10 minutes,
-- each corpse left 2 s so the engine logs how far it slid
-- (RAGDOLL_DEATH_SLIDE, first 1.5 s dead).
for _, soak in ipairs({ { 'SOAK-01 an hour of powered duels', 50, 0 },
    { 'SOAK-02 ten minutes of deaths watched', 10, 2 } }) do
local SOAK_MINUTES, corpseSeconds = soak[2], soak[3]
testing.registerGlobalTest(soak[1], function()
    local fighters = {}
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player + util.vector3(0, -900, 0), { onGround = true })
        waitFor('soak spectator relocation', function()
            local current = player.position
            return player.cell.name == testArea.cell
                and util.vector3(current.x - testArea.player.x, current.y - testArea.player.y, 0):length() >= 800
        end)
        local weapons = { 'iron dagger', 'iron longsword', 'iron claymore', 'iron warhammer', 'iron spear', 'iron battle axe' }
        local spawned = 0
        local function spawn(slot)
            spawned = spawned + 1
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = types.NPC.record(player), name = 'SOAK-01 ' .. spawned,
                isEssential = false, isPersistent = false, isRespawning = false,
            }))
            local actor = world.createObject(record.id)
            addItem(actor, weapons[(spawned - 1) % #weapons + 1])
            local pair = math.floor((slot - 1) / 2)
            local side = (slot % 2 == 1) and -45 or 45
            actor:teleport(testArea.cell, testArea.player + util.vector3(pair * 300 - 150, side, 0), { onGround = true })
            actor:addScript('duelist.lua')
            return actor
        end
        for slot = 1, 4 do fighters[slot] = spawn(slot) end
        coroutine.yield()
        local function configure(slot)
            fighters[slot]:sendEvent('duelDuelistConfigure', { health = 60 })
        end
        for slot = 1, 4 do configure(slot) end
        local configured = {}
        local started = core.getSimulationTime()
        local deaths, lastReport, lastTargets = 0, started, 0
        local diedAt = {}
        while core.getSimulationTime() - started < SOAK_MINUTES * 60 do
            local now = core.getSimulationTime()
            for slot = 1, 4 do
                local actor = fighters[slot]
                if actor:isValid() and types.Actor.getStance(actor) == types.Actor.STANCE.Weapon and not configured[slot] then
                    ragdoll.setPowered(actor, true)
                    configured[slot] = true
                end
                if actor:isValid() and types.Actor.isDead(actor) and not diedAt[slot] then diedAt[slot] = now end
                if actor:isValid() and types.Actor.isDead(actor) and now - diedAt[slot] >= corpseSeconds then
                    diedAt[slot] = nil
                    deaths = deaths + 1
                    ragdoll.setPowered(actor, false)
                    actor:remove()
                    fighters[slot] = spawn(slot)
                    configured[slot] = false
                    coroutine.yield()
                    configure(slot)
                end
            end
            if now - lastTargets >= 2 then
                lastTargets = now
                for pair = 0, 1 do
                    local a, b = fighters[pair * 2 + 1], fighters[pair * 2 + 2]
                    if a:isValid() and b:isValid() then
                        ragdoll.setCombatTarget(a, b)
                        ragdoll.setCombatTarget(b, a)
                    end
                end
            end
            if now - lastReport >= 300 then
                lastReport = now
                print(string.format('SOAK-01 minute=%.0f deaths=%d spawned=%d', (now - started) / 60, deaths, spawned))
            end
            coroutine.yield()
        end
        print(string.format('SOAK-01 done minutes=%d deaths=%d spawned=%d', SOAK_MINUTES, deaths, spawned))
    end)
    for _, actor in ipairs(fighters) do
        if actor and actor:isValid() then ragdoll.setPowered(actor, false); actor:remove() end
    end
    if not ok then error(message) end
end)
end

testing.registerGlobalTest('DUEL-A03 unarmed npc duel to the death', function()
    local multiplier = ragdoll.testStepMultiplier()
    if multiplier > 0 then ragdoll.setTestStepActive(true, 907001) end
    local previousScale = ragdoll.getMeleeImpulseScale()
    local duelists = {}
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        ragdoll.setMeleeImpulseScale(1.5)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player, { onGround = true })
        waitFor('npc duel player relocation', function()
            local current = player.position
            local horizontal = util.vector3(
                current.x - testArea.player.x, current.y - testArea.player.y, 0)
            return player.cell.name == testArea.cell and horizontal:length() <= 2
        end)
        -- The player must not stand in the arena: both duelists swing at
        -- chest height and the melee detection would spend their blows on the
        -- spectator. Park the player out of reach; the population policy's
        -- acquire radius still covers the duelists.
        player:teleport(testArea.cell, testArea.player + util.vector3(0, -800, 0), { onGround = true })
        waitFor('npc duel spectator relocation', function()
            local current = player.position
            local horizontal = util.vector3(
                current.x - testArea.player.x, current.y - testArea.player.y, 0)
            return player.cell.name == testArea.cell and horizontal:length() >= 700
        end)
        local function makeDuelist(name, offset, armed)
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = types.NPC.record(player),
                name = name,
                isEssential = false,
                isPersistent = false,
                isRespawning = false,
            }))
            local actor = world.createObject(record.id)
            if armed then
                addItem(actor, 'iron dagger')
            end
            actor:teleport(testArea.cell, testArea.player + offset, { onGround = true })
            actor:addScript('duelist.lua')
            return actor, record.id
        end
        local duelistA, recordA = makeDuelist('DUEL-A03 duelist A', util.vector3(0, -45, 0), false)
        local duelistB, recordB = makeDuelist('DUEL-A03 duelist B', util.vector3(0, 45, 0), false)
        duelists = { duelistA, duelistB }
        coroutine.yield()
        duelistA:sendEvent('duelDuelistConfigure', { unarmed = true })
        duelistB:sendEvent('duelDuelistConfigure', { unarmed = true })
        -- The melee hit test discards swings at actors that are not in the
        -- attacker's combat target list; register both directions.
        ragdoll.setCombatTarget(duelistA, duelistB)
        ragdoll.setCombatTarget(duelistB, duelistA)
        local function weaponReady(actor)
            -- Armed duelists equip a dagger; unarmed ones only take the
            -- hand-to-hand ready stance. Either way the stance is the gate.
            return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon
        end
        waitFor('duelist A weapon stance', function() return weaponReady(duelistA) end)
        waitFor('duelist B weapon stance', function() return weaponReady(duelistB) end)
        ragdoll.setPowered(duelistA, true)
        ragdoll.setPowered(duelistB, true)
        waitForMode(duelistA, 'duelist A initial standing', { standing = true }, 10, {})
        waitForMode(duelistB, 'duelist B initial standing', { standing = true }, 10, {})

        local startTime = core.getSimulationTime()
        local dead, winner
        local lastTargetRefresh = 0
        waitFor('npc duel death', function()
            local healthA = types.Actor.stats.dynamic.health(duelistA).current
            local healthB = types.Actor.stats.dynamic.health(duelistB).current
            local now = core.getSimulationTime()
            -- The AI combat package can lapse (target lost across a
            -- knock-down or a long chase); keep both directions registered so
            -- the duel runs to a death instead of stalling.
            if now - lastTargetRefresh >= 2 then
                lastTargetRefresh = now
                ragdoll.setCombatTarget(duelistA, duelistB)
                ragdoll.setCombatTarget(duelistB, duelistA)
            end
            if healthA <= 0 or healthB <= 0 then
                dead = healthA <= 0 and duelistA or duelistB
                winner = dead == duelistA and duelistB or duelistA
                return true
            end
        end, 300)
        local duration = core.getSimulationTime() - startTime
        do
            local winnerState = ragdoll.getState(winner)
            local deadState = ragdoll.getState(dead)
            print(string.format(
                'DUEL-A03 death: winner=%s mode=%s health=%.1f | dead=%s powered=%s health=%.1f',
                tostring(winner.id),
                winnerState and winnerState.controller and winnerState.controller.mode or 'n/a',
                types.Actor.stats.dynamic.health(winner).current,
                tostring(dead.id),
                deadState and tostring(deadState.powered) or 'no-ragdoll',
                types.Actor.stats.dynamic.health(dead).current))
        end
        -- The corpse's first 1.5 s dead (RAGDOLL_DEATH_SLIDE: slide and
        -- depth under the floor) and 4 s (RAGDOLL_DEATH_DRAWN: the drawn body
        -- dragged by its actor) are logged before the case ends.
        waitSeconds(4.5)
        -- The survivor must be alive and on its feet. A wounded winner may
        -- brace or flee (ordinary morale) or recover from a knock-down first;
        -- all of those are standing states for gameplay purposes.
        local lastWinnerDebug = 0
        waitFor('npc duel winner recovered', function()
            local state = ragdoll.getState(winner)
            if not state or not state.controller then
                return false
            end
            local now = core.getSimulationTime()
            if now - lastWinnerDebug >= 4 then
                lastWinnerDebug = now
                print(string.format('DUEL-A03 winner mode=%s com=%.1f health=%.1f',
                    state.controller.mode, state.controller.centerOfMassSpeed or -1,
                    types.Actor.stats.dynamic.health(winner).current))
            end
            return state.controller.mode == 'standing' or state.controller.mode == 'bracing'
                or state.controller.mode == 'stumbling'
        end, 30)
        local deadState = ragdoll.getState(dead)
        testing.expect(deadState ~= nil, 'DUEL-A03 dead duelist lost its ragdoll')
        testing.expect(deadState.powered == false, 'DUEL-A03 dead duelist stayed powered')
        local winnerHealth = types.Actor.stats.dynamic.health(winner).current
        testing.expect(winnerHealth > 0, 'DUEL-A03 winner has no health left')
        emitTelemetry('npc_duel', {
            duration_seconds = duration,
            strikes = opponentStrikeCount,
            winner_health = winnerHealth,
            winner_standing = true,
            dead_ragdoll_powered = false,
        })
    end)

    for _, actor in ipairs(duelists) do
        if actor and actor:isValid() then
            actor:sendEvent('duelDuelistStop')
            ragdoll.setPowered(actor, false)
            actor:remove()
        end
    end
    ragdoll.setMeleeImpulseScale(previousScale)
    if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
    if not ok then error(message) end
end)


local handlers = {}
for name, handler in pairs(testing.globalEventHandlers) do handlers[name] = handler end
handlers.duelOpponentStrikeReleased = function()
    opponentStrikeCount = opponentStrikeCount + 1
end
handlers.duelPlayerReady = function(data)
    playerReady = true
    print('DUEL-A01 player ready with ' .. tostring(data.weapon))
end
handlers.duelOpponentReady = function(data)
    opponentReady = true
    print('DUEL-A01 opponent ready with ' .. tostring(data.weapon))
end
handlers.duelPlayerStrikeReleased = function(data)
    playerStrikeReleaseCount = playerStrikeReleaseCount + 1
    print(string.format('DUEL-A01 player released strike %s at distance %.3f stance %s',
        tostring(data.sequence), data.distance or -1, tostring(data.stance)))
    if activeOpponent and activeOpponent:isValid() and activeReactionCycle then
        local state = ragdoll.getState(activeOpponent)
        if state then
            local pelvis
            for _, body in ipairs(state.bodies) do
                if body.name == 'pelvis' then pelvis = body break end
            end
            if pelvis then
                reactionAttempt = reactionAttempt + 1
                local direction = activeOpponent.position - world.players[1].position
                direction = util.vector3(direction.x, direction.y, 0)
                if direction:length() < 0.001 then direction = util.vector3(0, 1, 0)
                else direction = direction:normalize() end
                ragdoll.applyCombatImpulse(activeOpponent,
                    direction * 6000 + util.vector3(0, 0, 350),
                    pelvis.centerOfMass, 6100 + activeReactionCycle * 10 + reactionAttempt)
            end
        end
    end
end

-- Plan phase 6: several powered fights at once. Four armed duels (eight
-- powered bodies) run side by side for 20 simulated seconds; every living
-- fighter must keep a powered ragdoll, and the cost is printed as real
-- seconds per simulated second.
testing.registerGlobalTest('SCALE-01 four armed duels at once', function()
    local multiplier = ragdoll.testStepMultiplier()
    if multiplier > 0 then ragdoll.setTestStepActive(true, 906101) end
    local fighters = {}
    local ok, message = pcall(function()
        ragdoll.setAutomaticRecovery(true)
        local player = world.players[1]
        player:teleport(testArea.cell, testArea.player + util.vector3(0, -900, 0), { onGround = true })
        waitFor('scale spectator relocation', function()
            return player.cell.name == testArea.cell
        end)
        local function make(name, offset)
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = types.NPC.record(player), name = name,
                isEssential = false, isPersistent = false, isRespawning = false,
            }))
            local actor = world.createObject(record.id)
            addItem(actor, 'iron dagger')
            actor:teleport(testArea.cell, testArea.player + offset, { onGround = true })
            actor:addScript('duelist.lua')
            return actor
        end
        local pairs_ = {}
        for i = 1, 4 do
            local x = (i - 2.5) * 220
            local a = make('SCALE-01 A' .. i, util.vector3(x, -45, 0))
            local b = make('SCALE-01 B' .. i, util.vector3(x, 45, 0))
            pairs_[i] = { a, b }
            fighters[#fighters + 1] = a
            fighters[#fighters + 1] = b
        end
        coroutine.yield()
        for _, pair in ipairs(pairs_) do
            pair[1]:sendEvent('duelDuelistConfigure')
            pair[2]:sendEvent('duelDuelistConfigure')
            ragdoll.setCombatTarget(pair[1], pair[2])
            ragdoll.setCombatTarget(pair[2], pair[1])
        end
        for _, actor in ipairs(fighters) do
            waitFor('scale fighter stance', function()
                return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon
            end)
            ragdoll.setPowered(actor, true)
        end
        for _, actor in ipairs(fighters) do
            waitForMode(actor, 'scale fighter standing', { standing = true }, 10, {})
        end
        local simStart, realStart = core.getSimulationTime(), core.getRealTime()
        local frames, lost, falls, lastRefresh = 0, 0, 0, 0
        local wasDown = {}
        waitFor('scale fight duration', function()
            frames = frames + 1
            local now = core.getSimulationTime()
            if now - lastRefresh >= 2 then
                lastRefresh = now
                for _, pair in ipairs(pairs_) do
                    ragdoll.setCombatTarget(pair[1], pair[2])
                    ragdoll.setCombatTarget(pair[2], pair[1])
                end
            end
            for _, actor in ipairs(fighters) do
                local alive = types.Actor.stats.dynamic.health(actor).current > 0
                local state = ragdoll.getState(actor)
                if alive and not (state and state.powered) then lost = lost + 1 end
                local down = state and state.controller
                    and (state.controller.mode == 'falling' or state.controller.mode == 'fallen')
                if down and not wasDown[actor.id] then falls = falls + 1 end
                wasDown[actor.id] = down
            end
            return now - simStart >= 20
        end, 40)
        local simElapsed = core.getSimulationTime() - simStart
        local realElapsed = core.getRealTime() - realStart
        local dead = 0
        for _, actor in ipairs(fighters) do
            if types.Actor.stats.dynamic.health(actor).current <= 0 then dead = dead + 1 end
        end
        print(string.format('SCALE-01 fighters=%d sim=%.1f real=%.1f real_per_sim=%.2f frames=%d frame_ms=%.1f lost_samples=%d falls=%d dead=%d multiplier=%d',
            #fighters, simElapsed, realElapsed, realElapsed / math.max(simElapsed, 1e-3), frames,
            1000 * realElapsed / math.max(frames, 1), lost, falls, dead, multiplier))
        testing.expect(lost == 0, 'SCALE-01 a living fighter lost its powered ragdoll')
        emitTelemetry('scale_four_duels', {
            fighters = #fighters, real_per_sim = realElapsed / math.max(simElapsed, 1e-3),
            frame_ms = 1000 * realElapsed / math.max(frames, 1), falls = falls, dead = dead,
        })
    end)
    for _, actor in ipairs(fighters) do
        if actor and actor:isValid() then
            actor:sendEvent('duelDuelistStop')
            ragdoll.setPowered(actor, false)
            actor:remove()
        end
    end
    if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
    if not ok then error(message) end
end)

-- Plan phase 6 in the real world: Balmora's streets with the population
-- policy (run with OPENMW_RAGDOLL_POWER_NPCS=1). Townspeople near the player
-- get powered bodies while their ordinary AI walks them around; watch a
-- minute for falls and broken bodies.
testing.registerGlobalTest('SCALE-02 powered townspeople in Balmora', function()
    local player = world.players[1]
    player:teleport('', util.vector3(-21800, -12500, 1200), { onGround = true })
    waitFor('Balmora arrival', function()
        return player.cell and player.cell.isExterior and player.position.x < -16000
    end, 20)
    local deadline = core.getSimulationTime() + 5
    waitFor('Balmora settle', function() return core.getSimulationTime() >= deadline end, 10)
    local simStart, realStart = core.getSimulationTime(), core.getRealTime()
    local frames, maxPowered, poweredSamples, falls, broken = 0, 0, 0, 0, 0
    local wasDown, seen = {}, {}
    local nextSample = simStart
    waitFor('Balmora minute', function()
        frames = frames + 1
        local now = core.getSimulationTime()
        if now >= nextSample then
            nextSample = now + 0.25
            local powered = 0
            for _, actor in ipairs(world.activeActors) do
                if actor ~= player and actor.type == types.NPC then
                    local state = ragdoll.getState(actor)
                    if state and state.powered then
                        powered = powered + 1
                        seen[actor.id] = true
                        for _, body in ipairs(state.bodies) do
                            local v = body.position
                            if v.x ~= v.x or v.y ~= v.y or v.z ~= v.z then broken = broken + 1 break end
                        end
                        local mode = state.controller and state.controller.mode
                        local down = mode == 'falling' or mode == 'fallen'
                        if down and not wasDown[actor.id] then
                            falls = falls + 1
                            print(string.format('SCALE-02 fall actor=%s record=%s t=%.1f', tostring(actor.id),
                                types.NPC.record(actor).id, now - simStart))
                        end
                        wasDown[actor.id] = down
                    end
                end
            end
            maxPowered = math.max(maxPowered, powered)
            poweredSamples = poweredSamples + powered
        end
        return now - simStart >= 60
    end, 120)
    local simElapsed = core.getSimulationTime() - simStart
    local realElapsed = core.getRealTime() - realStart
    local distinct = 0
    for _ in pairs(seen) do distinct = distinct + 1 end
    print(string.format('SCALE-02 cell=%s max_powered=%d distinct=%d mean_powered=%.1f falls=%d broken=%d frame_ms=%.1f real_per_sim=%.2f',
        tostring(player.cell and player.cell.name), maxPowered, distinct, poweredSamples / math.max(1, simElapsed * 4),
        falls, broken, 1000 * realElapsed / math.max(frames, 1), realElapsed / math.max(simElapsed, 1e-3)))
    testing.expect(broken == 0, 'SCALE-02 a powered townsperson had a non-finite body')
    emitTelemetry('scale_balmora', { max_powered = maxPowered, falls = falls, broken = broken,
        frame_ms = 1000 * realElapsed / math.max(frames, 1) })
end)

-- Play test 2026-09-28: "enemy guard can't get up" (Balmora). Every NPC
-- get-up case was on the flat test floor: here townspeople on Balmora's
-- streets are knocked down (a 10000 chest impulse, the player's heavy knockdown
-- case; guards divide it by their Agility toughness, up to 1.8) and must stand again within 12 s.
testing.registerGlobalTest('SCALE-03 knocked-down townspeople get up in Balmora', function()
    local player = world.players[1]
    player:teleport('', util.vector3(-21800, -12500, 1200), { onGround = true })
    waitFor('Balmora arrival', function()
        return player.cell and player.cell.isExterior and player.position.x < -16000
    end, 20)
    local settle = core.getSimulationTime() + 5
    waitFor('Balmora settle', function() return core.getSimulationTime() >= settle end, 10)
    local candidates = {}
    for _, actor in ipairs(world.activeActors) do
        if actor ~= player and actor.type == types.NPC and not types.Actor.isDead(actor) then
            local state = ragdoll.getState(actor)
            if state and state.powered and state.controller and state.controller.mode == 'standing' then
                candidates[#candidates + 1] = actor
            end
        end
    end
    table.sort(candidates, function(a, b)
        return (a.position - player.position):length() < (b.position - player.position):length()
    end)
    testing.expect(#candidates > 0, 'SCALE-03 no powered standing townsperson near the player')
    local tested, falls = 0, 0
    for index = 1, math.min(3, #candidates) do
        local actor = candidates[index]
        -- A townsperson can walk out of the powered range (its body is
        -- released) while the ones before it are tested.
        local state = ragdoll.getState(actor)
        local chest
        for _, body in ipairs(state and state.powered and state.bodies or {}) do
            if body.name == 'spine2' then chest = body break end
        end
        if chest then
            tested = tested + 1
            ragdoll.applyCombatImpulse(actor, actor.rotation * util.vector3(0, -10000, 0), chest.centerOfMass,
                20263000 + index)
            local seen, sequence, last = {}, {}, nil
            local start = core.getSimulationTime()
            local stood = false
            local realDeadline = core.getRealTime() + 48
            repeat
                coroutine.yield()
                local now = ragdoll.getState(actor)
                local mode = now and now.controller and now.controller.mode or 'none'
                seen[mode] = true
                if mode ~= last then
                    local pelvis
                    for _, body in ipairs(now and now.bodies or {}) do
                        if body.name == 'pelvis' then pelvis = body break end
                    end
                    sequence[#sequence + 1] = string.format('%s@%.1f(z%.0f,up%.2f)', mode,
                        core.getSimulationTime() - start, pelvis and pelvis.position.z - actor.position.z or 0,
                        now and now.controller and now.controller.torsoUpDot or -9)
                    last = mode
                end
                stood = (seen.falling or seen.fallen) and mode == 'standing'
                -- A tough NPC (Agility toughness, up to 1.8x) catches the blow:
                -- it never falls, and there is no get-up to check.
                local held = not (seen.falling or seen.fallen) and core.getSimulationTime() - start > 3
            until stood or held or core.getSimulationTime() - start > 12 or core.getRealTime() > realDeadline
            local fell = seen.falling or seen.fallen
            if fell then falls = falls + 1 end
            print(string.format('SCALE-03 actor=%s record=%s cell=%s up=%s after=%.1f modes=%s',
                tostring(actor.id), types.NPC.record(actor).id, tostring(actor.cell and actor.cell.name),
                tostring(stood), core.getSimulationTime() - start, table.concat(sequence, ' ')))
            testing.expect(stood or not fell, 'SCALE-03 townsperson ' .. types.NPC.record(actor).id .. ' did not get up')
        end
    end
    testing.expect(tested > 0, 'SCALE-03 no townsperson had a chest body')
    testing.expect(falls > 0, 'SCALE-03 no townsperson was knocked down')
end)

-- CREATURE-02 tier 1: a stock bipedal creature (the human skeleton and
-- animations) against an armed NPC, to a death. The creature is not powered
-- by the test: the population policy has to give it its body
-- (OPENMW_RAGDOLL_CREATURE_BIPEDS), then it fights through the ordinary AI.
for _, foe in ipairs({
    { id = 'skeleton', weapon = 'iron longsword', shield = 'iron_shield' },
    -- The AI takes its best weapon, so the stock random ones (the skeleton's
    -- iron, the dremora's excellent melee weapon) often win over the longsword.
    { id = 'dremora' },
}) do
    testing.registerGlobalTest('CREATURE-02 ' .. foe.id .. ' duels an npc', function()
        local multiplier = ragdoll.testStepMultiplier()
        if multiplier > 0 then ragdoll.setTestStepActive(true, 907001) end
        local actors = {}
        local ok, message = pcall(function()
            ragdoll.setAutomaticRecovery(true)
            local player = world.players[1]
            player:teleport(testArea.cell, testArea.player + util.vector3(0, -800, 0), { onGround = true })
            waitFor('creature duel spectator relocation', function()
                local current = player.position
                local horizontal = util.vector3(
                    current.x - testArea.player.x, current.y - testArea.player.y, 0)
                return player.cell.name == testArea.cell and horizontal:length() >= 700
            end)
            -- A trained fighter (the stock Hlaalu guard, Long Blade 68; the
            -- player's own template, Short Blade ~15, never got its dagger
            -- past the skeleton's shield in 23 s) with a silver blade: dremora
            -- and golden saints are immune to normal weapons, and with steel
            -- the AI put the sword away and fought with its fists.
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = types.NPC.record('hlaalu guard'),
                name = 'CREATURE-02 npc',
                isEssential = false,
                isPersistent = false,
                isRespawning = false,
            }))
            local npc = world.createObject(record.id)
            addItem(npc, 'silver longsword')
            npc:teleport(testArea.cell, testArea.player + util.vector3(0, -45, 0), { onGround = true })
            npc:addScript('duelist.lua')
            local creature = world.createObject(foe.id)
            if foe.weapon then addItem(creature, foe.weapon) end
            if foe.shield then addItem(creature, foe.shield) end
            creature:teleport(testArea.cell, testArea.player + util.vector3(0, 45, 0), { onGround = true })
            creature:addScript('duelist.lua')
            actors = { npc, creature }
            coroutine.yield()
            -- Pools the duel can spend on both sides: with its record's 38
            -- health a skeleton died in 2.5 s before it swung (1 run in 4).
            npc:sendEvent('duelDuelistConfigure', { weapon = 'silver longsword', health = 80 })
            creature:sendEvent('duelDuelistConfigure', { weapon = foe.weapon, shield = foe.shield, health = 120 })
            ragdoll.setCombatTarget(npc, creature)
            ragdoll.setCombatTarget(creature, npc)
            waitFor('npc weapon stance', function()
                return types.Actor.getStance(npc) == types.Actor.STANCE.Weapon
            end)
            waitFor('creature weapon stance', function()
                return types.Actor.getStance(creature) == types.Actor.STANCE.Weapon
            end)
            ragdoll.setPowered(npc, true)
            local powered = waitFor('creature powered by the population policy', function()
                local state = ragdoll.getState(creature)
                return state and state.powered and state
            end, 10)
            -- The 20 body parts, the weapon, and the shield if one is carried.
            testing.expectEqual(#powered.bodies, foe.shield and 22 or 21,
                'CREATURE-02 creature body has an unexpected body count')
            -- On its feet: the AI may open the fight at once (bracing, stumbling).
            waitForMode(creature, 'creature upright', { standing = true, bracing = true, stumbling = true }, 10, {})

            local startTime = core.getSimulationTime()
            waitFor('npc health pool', function() return types.Actor.stats.dynamic.health(npc).current == 80 end)
            local npcHealth = types.Actor.stats.dynamic.health(npc).current
            local creatureHealth = types.Actor.stats.dynamic.health(creature).current
            local dead, winner
            local lastTargetRefresh = 0
            -- Each side's weapon touching the other fighter (body, blade or
            -- shield): the counter restarts with every rebuilt body.
            local touched = { [npc.id] = false, [creature.id] = false }
            waitFor('creature duel death', function()
                local healthA = types.Actor.stats.dynamic.health(npc).current
                local healthB = types.Actor.stats.dynamic.health(creature).current
                local now = core.getSimulationTime()
                if now - lastTargetRefresh >= 2 then
                    lastTargetRefresh = now
                    ragdoll.setCombatTarget(npc, creature)
                    ragdoll.setCombatTarget(creature, npc)
                end
                for _, actor in ipairs({ npc, creature }) do
                    local state = ragdoll.getState(actor)
                    if state and state.weapon and (state.weapon.otherRagdollContacts or 0) > 0 then
                        touched[actor.id] = true
                    end
                end
                if healthA <= 0 or healthB <= 0 then
                    dead = healthA <= 0 and npc or creature
                    winner = dead == npc and creature or npc
                    return true
                end
            end, 300)
            local duration = core.getSimulationTime() - startTime
            local npcTaken = npcHealth - types.Actor.stats.dynamic.health(npc).current
            local creatureTaken = creatureHealth - types.Actor.stats.dynamic.health(creature).current
            print(string.format('CREATURE-02 %s: winner=%s after=%.1f npc_taken=%.1f creature_taken=%.1f',
                foe.id, tostring(winner.recordId), duration, npcTaken, creatureTaken))
            waitSeconds(4.5)
            local deadState = ragdoll.getState(dead)
            testing.expect(deadState ~= nil, 'CREATURE-02 the dead fighter lost its ragdoll')
            testing.expect(deadState.powered == false, 'CREATURE-02 the dead fighter stayed powered')
            -- Both fight with physical weapons. Who draws blood varies: the
            -- dremora's random weapon (spear to warhammer), the guard's shield.
            testing.expect(touched[creature.id], 'CREATURE-02 the ' .. foe.id .. "'s weapon never touched the npc")
            testing.expect(touched[npc.id], "CREATURE-02 the npc's weapon never touched the " .. foe.id)
            testing.expect(npcTaken > 0 or creatureTaken > 0, 'CREATURE-02 nobody was hurt')
            emitTelemetry('creature_duel', {
                creature = foe.id,
                duration_seconds = duration,
                npc_taken = npcTaken,
                creature_taken = creatureTaken,
                creature_won = winner == creature,
            })
        end)

        for _, actor in ipairs(actors) do
            if actor and actor:isValid() then
                actor:sendEvent('duelDuelistStop')
                ragdoll.setPowered(actor, false)
                actor:remove()
            end
        end
        if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
        if not ok then error(message) end
    end)
end

return {
    engineHandlers = { onUpdate = testing.updateGlobal },
    eventHandlers = handlers,
}

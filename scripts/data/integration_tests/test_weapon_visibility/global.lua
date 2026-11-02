-- Scene attachment regression, separate from the frozen 97 two-hand cases.
local core = require('openmw.core')
local I = require('openmw.interfaces')
local ragdoll = require('openmw.ragdolltest')
local testing = require('testing_util')
local types = require('openmw.types')
local util = require('openmw.util')
local world = require('openmw.world')
local sequence, ready, currentCase = 0, nil, ''
local function expect(value, message) testing.expect(value, 'WPN-05 visibility: ' .. message) end
local function emit(row)
    row.scenario, row.case = 'weapon_visibility', currentCase
    row.time = core.getSimulationTime()
    local fields = {}
    for key, value in pairs(row) do
        fields[#fields + 1] = string.format('%q:', key)
            .. (type(value) == 'string' and string.format('%q', value) or tostring(value))
    end
    print('RAGDOLL_TELEMETRY_JSON\t{' .. table.concat(fields, ',') .. '}')
end
local function waitFor(label, predicate, seconds)
    local deadline = core.getRealTime() + (seconds or 10)
    repeat
        local value = predicate()
        if value then return value end
        coroutine.yield()
    until core.getRealTime() >= deadline
    error('WPN-05 visibility timeout: ' .. label)
end
local function observe(seconds, callback)
    local started, deadline = core.getSimulationTime(), core.getRealTime() + 15
    repeat
        if callback then callback() end
        coroutine.yield()
        expect(core.getRealTime() < deadline, 'simulation stopped')
    until core.getSimulationTime() - started >= seconds
end
local function configure(player, data)
    sequence = sequence + 1
    data.sequence = sequence
    player:sendEvent('weaponVisibilityConfigure', data)
    waitFor('local camera/equipment/stance', function() return ready == sequence end)
end
local function visibility(player, label)
    expect(type(ragdoll.getWeaponVisibility) == 'function', 'live scene visibility API missing')
    local value = ragdoll.getWeaponVisibility(player)
    expect(value and value.animationAvailable and value.rootPresent, 'missing player animation/root')
    expect(type(value.weaponsShown) == 'boolean' and type(value.weaponNodePresent) == 'boolean',
        'missing visibility fields')
    emit({ kind = 'visibility', label = label, shown = value.weaponsShown,
        node_present = value.weaponNodePresent, parent_paths = value.weaponParentPathCount })
    return value
end
local function requireVisible(player, label)
    local value = visibility(player, label)
    expect(value.weaponsShown and value.weaponNodePresent and value.weaponParentPathCount == 1,
        label .. ' did not retain exactly one live weapon attachment')
end
local function waitVisible(player, label)
    waitFor(label, function()
        local value = visibility(player, label)
        return value.weaponsShown and value.weaponNodePresent and value.weaponParentPathCount == 1
    end)
    observe(0.25, function() requireVisible(player, label .. '_stable') end)
end
local function waitHidden(player, label)
    waitFor(label, function()
        local value = visibility(player, label)
        return not value.weaponNodePresent and not value.weaponsShown
    end)
    observe(0.25, function()
        local value = visibility(player, label .. '_stable')
        expect(not value.weaponNodePresent and not value.weaponsShown, 'hidden weapon reappeared')
    end)
end
local function setup(camera)
    local player = world.players[1]
    I.RagdollDev.autoDuel(false)
    I.RagdollDev.cleanup()
    ragdoll.injectMouseInput(0, 0, false)
    ragdoll.setPowered(player, false)
    for _, id in ipairs({ 'iron claymore', 'iron longsword' }) do
        expect(types.Weapon.record(id) ~= nil, 'original weapon record missing')
        if not types.Actor.inventory(player):find(id) then
            world.createObject(id, 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
    end
    player:teleport('OpenMW Ragdoll Test Floor', util.vector3(0, 0, 128),
        { onGround = true, rotation = util.transform.rotateZ(0) })
    configure(player, { camera = camera, equip = false, drawn = false })
    observe(0.75)
    waitHidden(player, 'initial_sheathed')
    return player
end
local function register(name, run)
    testing.registerGlobalTest(name, function()
        currentCase = name
        local ok, message = pcall(run)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.setPowered(world.players[1], false)
        observe(0.25)
        if not ok then error(message, 0) end
    end)
end
for _, from in ipairs({ 'first_person', 'third_person' }) do
    local to = from == 'first_person' and 'third_person' or 'first_person'
    register('WPN-05 visibility same-frame ' .. from .. ' to ' .. to, function()
        local player = setup(from)
        configure(player, { camera = to, equip = 'iron claymore', drawn = true })
        waitVisible(player, 'same_frame_draw')
        ragdoll.captureScreenshot()
        observe(0.25)
    end)
end
register('WPN-05 visibility ordinary draw and ready camera rebuild', function()
    local player = setup('third_person')
    configure(player, { equip = 'iron claymore', drawn = true })
    waitVisible(player, 'ordinary_draw')
    observe(1)
    configure(player, { camera = 'first_person' })
    waitVisible(player, 'ready_first_person')
    configure(player, { camera = 'third_person' })
    waitVisible(player, 'ready_third_person')
end)
register('WPN-05 visibility sheath redraw and empty hands', function()
    local player = setup('first_person')
    configure(player, { camera = 'third_person', equip = 'iron claymore', drawn = true })
    waitVisible(player, 'draw')
    configure(player, { camera = 'first_person', drawn = false })
    waitHidden(player, 'sheath')
    configure(player, { camera = 'third_person', drawn = true })
    waitVisible(player, 'redraw')
    configure(player, { camera = 'first_person', equip = false, drawn = true })
    waitFor('empty hands', function() return not visibility(player, 'empty_hands').weaponNodePresent end)
    observe(0.25, function() expect(not visibility(player, 'empty_hands_stable').weaponNodePresent,
        'unarmed rebuild restored a weapon mesh') end)
end)
register('WPN-05 visibility equipped replacement', function()
    local player = setup('third_person')
    configure(player, { equip = 'iron longsword', drawn = true })
    waitVisible(player, 'one_hand_draw')
    configure(player, { waitAnimationGone = 'weapononehand' })
    configure(player, { camera = 'first_person', equip = 'iron claymore', drawn = true })
    waitVisible(player, 'two_hand_replacement')
    ragdoll.captureScreenshot()
    observe(0.25)
end)
register('WPN-05 visibility replacement during unfinished draw', function()
    local player = setup('third_person')
    configure(player, { equip = 'iron longsword', drawn = true })
    waitVisible(player, 'one_hand_draw')
    -- Preserve the original failing timing: attachment is visible but the
    -- one-hand equip animation has not been awaited before the type switch.
    configure(player, { camera = 'first_person', equip = 'iron claymore', drawn = true })
    waitVisible(player, 'unfinished_draw_replacement')
    ragdoll.captureScreenshot()
    observe(0.25)
end)
register('WPN-05 visibility hybrid native charged attack', function()
    local player = setup('first_person')
    -- Reproduce the actual helper path from the failing claymore captures.
    I.RagdollDev.playerFists('iron claymore')
    waitFor('hybrid standing claymore', function()
        local state = ragdoll.getState(player)
        return state and state.powered and state.controller and state.controller.mode == 'standing'
            and state.weapon and state.weapon.recordId == 'iron claymore'
    end)
    waitVisible(player, 'hybrid_ready')
    local seen, captured, attackId = {}, {}, nil
    local function watch()
        requireVisible(player, 'hybrid_attack')
        local state = ragdoll.getState(player)
        expect(state and state.weapon and state.weapon.recordId == 'iron claymore', 'lost hybrid weapon')
        local phase = state.attackPhase
        seen[phase] = true
        if phase ~= 'idle' then
            attackId = attackId or state.attackId
            expect(state.attackId == attackId, 'one press changed attack identity')
        end
        if not captured[phase] then ragdoll.captureScreenshot(); captured[phase] = true end
        emit({ kind = 'attack_visibility', phase = phase, attack_id = state.attackId,
            step = state.physicsStep, strength = state.attackStrength })
        return phase
    end
    ragdoll.injectMouseInput(0, 0, true)
    observe(1.5 / types.Weapon.record('iron claymore').speed, function()
        local phase = watch()
        expect(phase ~= 'release' and phase ~= 'follow_through', 'held press released early')
    end)
    expect(seen.wind_up, 'native charge never started')
    ragdoll.injectMouseInput(0, 0, false)
    waitFor('visible release and recovery', function()
        local phase = watch()
        return seen.release and seen.follow_through and phase == 'idle'
    end)
    observe(0.5, function() expect(watch() == 'idle', 'attack restarted') end)
    emit({ kind = 'visibility_summary', complete = true, attack_id = attackId })
end)
local handlers = {}
for name, handler in pairs(testing.globalEventHandlers) do handlers[name] = handler end
handlers.weaponVisibilityReady = function(value) ready = value end
handlers.weaponVisibilityProgress = function(value) value.kind = 'configure_progress'; emit(value) end
return { engineHandlers = { onUpdate = testing.updateGlobal }, eventHandlers = handlers }

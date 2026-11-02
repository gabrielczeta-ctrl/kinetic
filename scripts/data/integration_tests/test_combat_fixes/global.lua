local core = require('openmw.core')
local ragdoll = require('openmw.ragdolltest')
local testing = require('testing_util')
local testArea = require('ragdoll_test_area')
local types = require('openmw.types')
local util = require('openmw.util')
local world = require('openmw.world')

-- Live checks for the combat fixes of 2026-09-18 on the kinematic-root route:
--   FIX-02 graded hit reactions: head and torso blows stagger, only the legs
--          (or one overwhelming blow) drop a held body, and a stagger does not
--          end in a fall when its stun runs out;
--   FIX-03 a ragdoll created in a weapon stance has the same leg bones as one
--          created in the idle pose (no averaged joint anchors);
--   FIX-05 holding the attack extends the player's arm toward the front;
--   FIX-07 the limb damage ledger behind the red body-part tint.

local readyActors = {}
local roughRoute, roughDone = nil, false -- ROUGH-01
local shipPlayerRoute = nil -- SHIP-02 (fixPlayerRouteStart)
-- Hand control's drag sizes (build 40, calibrated on the user's play: the
-- chamber 1500 counts past the 300-count stance circle, the cut 3000 counts,
-- a stroke runs on into the next wind-up 2500 counts past the cut's end).
HSC_WIND = 1800
HSC_CUT = 3000
HSC_RUN_ON = 2500
local playerReadySequence
local cameraReadyMode
local contactAimReady

local kineticResult
local function registerTest(name, seed, run)
    testing.registerGlobalTest(name, function()
        core.sendGlobalEvent('ragdollManualControl', { action = 'duel_auto_off' })
        core.sendGlobalEvent('ragdollManualControl', { action = 'duel_restage' })
        coroutine.yield()
        coroutine.yield()
        local accelerated = ragdoll.testStepMultiplier() > 0
        if accelerated then ragdoll.setTestStepActive(true, seed) end
        local ok, message = pcall(run)
        if accelerated then ragdoll.setTestStepActive(false, 0) end
        if not ok then error(message, 0) end
    end)
end

local function waitFor(description, predicate, timeout)
    timeout = timeout or 10
    local simulationDeadline = core.getSimulationTime() + timeout
    local realDeadline = core.getRealTime() + math.max(timeout * 4, 15)
    repeat
        local value = predicate()
        if value then return value end
        coroutine.yield()
    until core.getSimulationTime() >= simulationDeadline or core.getRealTime() >= realDeadline
    error('timed out waiting for ' .. description)
end

local function waitSeconds(seconds, eachFrame)
    local deadline = core.getSimulationTime() + seconds
    local realDeadline = core.getRealTime() + math.max(seconds * 4, 10)
    repeat
        if eachFrame then eachFrame() end
        coroutine.yield()
    until core.getSimulationTime() >= deadline or core.getRealTime() >= realDeadline
end

local function body(state, name)
    for _, item in ipairs(state.bodies) do
        if item.name == name then return item end
    end
    error('ragdoll has no body named ' .. name)
end

-- Put a victim's measured body point at a world XY. The point's offset from the
-- actor origin depends on facing (the torso leans forward), so a single
-- origin + (desired - point) teleport that also rotates the actor lands up to
-- ~24 units off when an earlier blow turned it. Rotate first, re-measure, then
-- correct.
local function placeVictimPoint(victim, measure, desired, rotation)
    local point = measure()
    victim:teleport(testArea.cell, victim.position + util.vector3(desired.x - point.x, desired.y - point.y, 0),
        { onGround = true, rotation = rotation })
    -- The body turns to a new facing at a bounded rate, so correct until the
    -- measured point stays put.
    for _ = 1, 6 do
        waitSeconds(0.4)
        point = measure()
        local error = util.vector3(desired.x - point.x, desired.y - point.y, 0)
        if error:length() < 2 then return end
        victim:teleport(testArea.cell, victim.position + error, { onGround = true, rotation = rotation })
    end
    print(string.format('FIX-08 placement did not converge: %s', tostring(desired - measure())))
end

local function placePlayer()
    local player = world.players[1]
    player:teleport(testArea.cell, testArea.player,
        { onGround = true, rotation = util.transform.rotateZ(0) })
    waitFor('player relocation', function()
        local current = player.position
        local horizontal = util.vector3(current.x - testArea.player.x, current.y - testArea.player.y, 0)
        return player.cell.name == testArea.cell and horizontal:length() <= 2
    end)
    return player
end

local function spawnDummy(name, offset, weaponStance)
    local player = world.players[1]
    local record = world.createRecord(types.NPC.createRecordDraft({
        template = types.NPC.record(player),
        name = name,
        isEssential = false,
        isPersistent = false,
        isRespawning = false,
    }))
    local actor = world.createObject(record.id)
    if weaponStance then
        -- A record name arms it with that weapon (CLASH-01: a staff).
        world.createObject(type(weaponStance) == 'string' and weaponStance or 'iron dagger', 1)
            :moveInto(types.Actor.inventory(actor))
    end
    actor:teleport(testArea.cell, testArea.player + offset, { onGround = true })
    actor:addScript('stance.lua')
    coroutine.yield()
    actor:sendEvent('fixActorConfigure', { weaponStance = weaponStance })
    waitFor(name .. ' stance', function() return readyActors[actor.id] ~= nil end, 10)
    return actor
end

local function powerAndStand(actor, label)
    ragdoll.setPowered(actor, true)
    return waitFor(label .. ' powered standing', function()
        local state = ragdoll.getState(actor)
        if state and state.powered and state.controller and state.controller.mode == 'standing' then
            return state
        end
    end, 12)
end

local function watchModes(actor, seconds, seen)
    waitSeconds(seconds, function()
        local state = ragdoll.getState(actor)
        if state and state.controller then seen[state.controller.mode] = true end
    end)
end

registerTest('FIX-02 head and torso blows stagger, a hard leg blow drops', 918001, function()
    placePlayer()
    ragdoll.setAutomaticRecovery(true)
    ragdoll.clearLimbDamage()
    local victim = spawnDummy('FIX-02 victim', util.vector3(60, 90, 0), false)
    local ok, message = pcall(function()
        powerAndStand(victim, 'FIX-02 victim')
        local push = util.vector3(0, 1, 0)
        local seen = {}
        -- The blows the duel of 2026-09-18 landed: 2100-2625 before the region
        -- scale, to the head and the chest. Before the fix every one of them
        -- released the root, at once or when its stun ran out.
        for blow = 1, 6 do
            local state = ragdoll.getState(victim)
            local struck = body(state, blow % 2 == 0 and 'head' or 'spine2')
            ragdoll.applyCombatImpulse(victim, push * 2600, struck.centerOfMass, 7100 + blow)
            watchModes(victim, 0.9, seen)
        end
        watchModes(victim, 1.5, seen)
        testing.expect(not seen.falling and not seen.fallen,
            'head and torso blows dropped a held body')

        local ledger = ragdoll.getLimbDamage(victim)
        testing.expect((ledger.head or 0) > 0, 'limb damage ledger missed the head blows')
        testing.expect((ledger.chest or 0) > 0, 'limb damage ledger missed the chest blows')
        testing.expect((ledger.left_calf or 0) == 0, 'limb damage ledger invented a calf blow')

        -- A light blow is absorbed and leaves the body standing once the
        -- stun has run out (the stun-expiry release).
        local light = ragdoll.getState(victim)
        ragdoll.applyCombatImpulse(victim, push * 700, body(light, 'spine1').centerOfMass, 7200)
        local lightSeen = {}
        watchModes(victim, 1.2, lightSeen)
        testing.expect(not lightSeen.falling and not lightSeen.fallen, 'an absorbed blow dropped the body')

        -- The legs are what fail: one hard blow to the calf is a knock-down.
        local standing = ragdoll.getState(victim)
        ragdoll.applyCombatImpulse(victim, push * 2400, body(standing, 'left_calf').centerOfMass, 7300)
        waitFor('knock-down from the leg blow', function()
            local state = ragdoll.getState(victim)
            return state and state.controller
                and (state.controller.mode == 'falling' or state.controller.mode == 'fallen')
        end, 4)

        ragdoll.clearLimbDamage()
        local cleared = ragdoll.getLimbDamage(victim)
        testing.expect(next(cleared) == nil, 'clearLimbDamage left entries behind')
    end)
    if victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)

registerTest('FIX-02 one overwhelming blow drops a held body anywhere', 918002, function()
    placePlayer()
    ragdoll.setAutomaticRecovery(true)
    local victim = spawnDummy('FIX-02 overwhelmed', util.vector3(60, 90, 0), false)
    local ok, message = pcall(function()
        powerAndStand(victim, 'FIX-02 overwhelmed')
        local state = ragdoll.getState(victim)
        ragdoll.applyCombatImpulse(victim, util.vector3(0, 1, 0) * 6500, body(state, 'spine2').centerOfMass, 7400)
        waitFor('knock-down from the overwhelming blow', function()
            local current = ragdoll.getState(victim)
            return current and current.controller
                and (current.controller.mode == 'falling' or current.controller.mode == 'fallen')
        end, 4)
    end)
    if victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)

registerTest('DAZE-01 head blows daze, chest blows do not', 918004, function()
    placePlayer()
    ragdoll.setAutomaticRecovery(true)
    local victim = spawnDummy('DAZE-01 victim', util.vector3(60, 90, 0), false)
    local ok, message = pcall(function()
        powerAndStand(victim, 'DAZE-01 victim')
        local push = util.vector3(0, 1, 0)
        for blow, part in ipairs({ 'spine2', 'head', 'head' }) do
            local before = ragdoll.getState(victim)
            local pelvis0 = body(before, 'pelvis').centerOfMass.z
            local count0 = before.dazeCount or 0
            ragdoll.applyCombatImpulse(victim, push * (blow == 3 and 1200 or 2600),
                body(before, part).centerOfMass, 7500 + blow)
            local seen, peakDaze, peakDrop, low = {}, 0, 0, 0
            waitSeconds(2.5, function()
                local state = ragdoll.getState(victim)
                if state then
                    peakDaze = math.max(peakDaze, state.dazeTime or 0)
                    peakDrop = math.max(peakDrop, state.dazeDrop or 0)
                    low = math.max(low, pelvis0 - body(state, 'pelvis').centerOfMass.z)
                    if state.controller then seen[state.controller.mode] = true end
                end
            end)
            local after = ragdoll.getState(victim)
            print(string.format('DAZE-01 blow=%d part=%s dazes=%d peak_daze=%.2f peak_sink=%.1f pelvis_low=%.1f fell=%s',
                blow, part, (after.dazeCount or 0) - count0, peakDaze, peakDrop, low,
                tostring(seen.falling or seen.fallen or false)))
            if part == 'head' then
                testing.expect((after.dazeCount or 0) > count0, 'a head blow did not daze')
            else
                testing.expect((after.dazeCount or 0) == count0, 'a chest blow dazed')
            end
            testing.expect(not seen.falling and not seen.fallen, 'the blow dropped the body')
        end
    end)
    if victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)

for fallIndex, fall in ipairs({
    { label = 'forward', push = util.vector3(0, 1, 0) },
    { label = 'backward', push = util.vector3(0, -1, 0) },
    { label = 'sideways', push = util.vector3(1, 0, 0) },
    { label = 'backward by a chest blow', push = util.vector3(0, -1, 0), part = 'spine2', impulse = 6500 },
    { label = 'forward by a chest blow', push = util.vector3(0, 1, 0), part = 'spine2', impulse = 6500 },
}) do
registerTest('FALL-01 a body knocked ' .. fall.label .. ' tries to catch the fall', 918005 + fallIndex * 7, function()
    local player = placePlayer()
    player:sendEvent('fixPlayerConfigure', { sequence = 380 + fallIndex, firstPerson = true })
    waitFor('player loadout', function() return playerReadySequence == 380 + fallIndex end)
    ragdoll.setAutomaticRecovery(false)
    local victim = spawnDummy('FALL-01 victim', util.vector3(60, 90, 0), false)
    local ok, message = pcall(function()
        powerAndStand(victim, 'FALL-01 victim')
        local start = ragdoll.getState(victim)
        local ground = math.min(body(start, 'left_foot').centerOfMass.z, body(start, 'right_foot').centerOfMass.z) - 3
        -- OPENMW_RAGDOLL_TEST_FALL_SCALE: the blows x this (CATCH-02 10-27:
        -- since the momentum reactions and PGC, build 85 caught all five at 1).
        local scale = tonumber(ragdoll.tuningText('OPENMW_RAGDOLL_TEST_FALL_SCALE') or '') or 1
        -- A blow's push alone rarely fells anyone now (injury does, RXN-02
        -- M5): first a cut to the left calf that gives the knee out
        -- (OPENMW_RAGDOLL_TEST_FALL_INJURY damage, 0 = none), then the push.
        local injury = tonumber(ragdoll.tuningText('OPENMW_RAGDOLL_TEST_FALL_INJURY') or '') or 30
        if injury > 0 then ragdoll.applyBlow(victim, 'left_calf', fall.push, 300, 1.5, injury) end
        -- OPENMW_RAGDOLL_TEST_FALL_RAW = N: instead a raw push of N x the
        -- blow on the struck part, bypassing the reaction's cap (a knock-down
        -- that PGC cannot step out of, for measuring the catch).
        local raw = tonumber(ragdoll.tuningText('OPENMW_RAGDOLL_TEST_FALL_RAW') or '') or 0
        -- OPENMW_RAGDOLL_TEST_FALL_YANK = "distance u, seconds, force": the
        -- chest is dragged that far along the fall (and a third of it down)
        -- over that time, then let go - a throw the balance cannot step out
        -- of (build 85 caught every blow above, CATCH-02 10-27).
        local yank = {}
        for v in (ragdoll.tuningText('OPENMW_RAGDOLL_TEST_FALL_YANK') or ''):gmatch('[^,]+') do yank[#yank + 1] = tonumber(v) end
        -- OPENMW_RAGDOLL_TEST_FALL_BLOW = "speed u/s, mass kg, damage": a
        -- weapon blow through the reaction controller (ragdoll.applyBlow, as
        -- STAGGER-01: heavy = 1600,2.4,35) instead of the combat impulse,
        -- which skips the controller and its stagger plan (STEP-01 10-29).
        local blowSpec = {}
        for v in (ragdoll.tuningText('OPENMW_RAGDOLL_TEST_FALL_BLOW') or ''):gmatch('[^,]+') do blowSpec[#blowSpec + 1] = tonumber(v) end
        if blowSpec[1] and blowSpec[1] > 0 then
            ragdoll.applyBlow(victim, fall.part or 'left_calf', fall.push, blowSpec[1] * scale, blowSpec[2] or 2.4,
                blowSpec[3] or 35)
        elseif yank[1] and yank[1] > 0 then
            local chest = body(start, 'spine2').centerOfMass
            local seconds, force = yank[2] or 0.35, yank[3] or 30000
            ragdoll.grab(victim, chest + util.vector3(0, 0, 100), util.vector3(0, 0, -1), force)
            local t1 = core.getSimulationTime()
            local dir = util.vector3(fall.push.x, fall.push.y, 0):normalize()
            waitSeconds(seconds, function()
                local share = math.min(1, (core.getSimulationTime() - t1) / seconds)
                ragdoll.updateGrab(victim, chest + (dir * yank[1] + util.vector3(0, 0, -yank[1] / 3)) * share)
            end)
            ragdoll.releaseGrab(victim)
        elseif raw > 0 then
            for index, b in ipairs(start.bodies) do
                if b.name == (fall.part or 'left_calf') then
                    ragdoll.applyImpulse(victim, index, fall.push * ((fall.impulse or 2600) * raw), b.centerOfMass)
                end
            end
        else
            ragdoll.applyCombatImpulse(victim, fall.push * ((fall.impulse or 2600) * scale), body(start, fall.part or 'left_calf').centerOfMass, 7600 + fallIndex)
        end
        local t0 = core.getSimulationTime()
        local fallAt, handAt, headAt, chestAt, headSpeed, braced, chestSpeed, headLanding = nil, nil, nil, nil, 0, 0, 0, 0
        local modes, headLow, pelvisLow = {}, 1e9, 1e9
        -- Steps against slide: each foot's lifts (clear of the floor by 4 u,
        -- then down again) and how far it moved while down; the pelvis's and
        -- the capsule's horizontal travel.
        local function flat(v) return util.vector3(v.x, v.y, 0) end
        local pelvis0, capsule0 = flat(body(start, 'pelvis').centerOfMass), flat(victim.position)
        local pelvisFar, capsuleFar = 0, 0
        local feet = {}
        for _, name in ipairs({ 'left_foot', 'right_foot' }) do
            feet[name] = { up = false, steps = 0, slide = 0, last = flat(body(start, name).centerOfMass) }
        end
        waitSeconds(3.0, function()
            local state = ragdoll.getState(victim)
            if not state then return end
            pelvisFar = math.max(pelvisFar, (flat(body(state, 'pelvis').centerOfMass) - pelvis0):length())
            capsuleFar = math.max(capsuleFar, (flat(victim.position) - capsule0):length())
            for name, f in pairs(feet) do
                local at = body(state, name).centerOfMass
                f.rest = f.rest or (at.z - ground)
                local height = at.z - ground - f.rest -- above its standing height
                local up = height > 4 or (f.up and height > 1.5)
                if f.up and not up then f.steps = f.steps + 1 end
                if not up and not f.up and height < 1.5 then f.slide = f.slide + (flat(at) - f.last):length() end
                -- Net slide: how far each flat contact ends from where it began.
                local onFloor = not up and height < 1.5
                if onFloor and not f.from then f.from = flat(at) end
                if f.from and not onFloor then f.net = (f.net or 0) + (f.lastFlat - f.from):length(); f.from = nil end
                if onFloor then f.lastFlat = flat(at) end
                f.up, f.last = up, flat(at)
            end
            local t = core.getSimulationTime() - t0
            local mode = state.controller and state.controller.mode
            if mode and not modes[mode] then modes[mode] = true; modes[#modes + 1] = mode end
            headLow = math.min(headLow, body(state, 'head').centerOfMass.z - ground)
            pelvisLow = math.min(pelvisLow, body(state, 'pelvis').centerOfMass.z - ground)
            if not fallAt and (mode == 'falling' or mode == 'fallen') then fallAt = t end
            local hand = math.min(body(state, 'left_hand').centerOfMass.z, body(state, 'right_hand').centerOfMass.z) - ground
            local head = body(state, 'head').centerOfMass.z - ground
            local chest = body(state, 'spine2').centerOfMass.z - ground
            if not handAt and hand < 6 then handAt = t end
            if not chestAt and chest < 14 then chestAt = t end
            if not headAt and head < 12 then
                headAt = t
                local hv = body(state, 'head').linearVelocity
                headLanding = hv and -hv.z or 0
            end
            local v = body(state, 'head').linearVelocity
            if v and not headAt then headSpeed = math.max(headSpeed, -v.z) end
            local cv = body(state, 'spine2').linearVelocity
            if cv and not chestAt then chestSpeed = math.max(chestSpeed, -cv.z) end
            local lh = body(state, 'left_hand')
            if mode == 'falling' and lh.motorUtilization then braced = braced + 1 end
        end)
        local function f(x) return x and string.format('%.2f', x) or '-' end
        print(string.format('FALL-01 dir=%s fall=%s hand=%s chest=%s head=%s hands_first=%s head_peak=%.0f head_at12=%.0f chest_peak=%.0f brace_frames=%d',
            fall.label, f(fallAt), f(handAt), f(chestAt), f(headAt),
            tostring(handAt ~= nil and (headAt == nil or handAt < headAt)), headSpeed, headLanding, chestSpeed, braced)
            .. string.format(' head_low=%.0f pelvis_low=%.0f modes=%s', headLow, pelvisLow, table.concat(modes, ','))
            .. string.format(' pelvis_travel=%.0f capsule_travel=%.0f steps=%d,%d foot_slide=%.0f,%.0f', pelvisFar,
                capsuleFar, feet.left_foot.steps, feet.right_foot.steps, feet.left_foot.slide, feet.right_foot.slide)
            .. (function()
                for _, f in pairs(feet) do
                    if f.from then f.net = (f.net or 0) + (f.lastFlat - f.from):length(); f.from = nil end
                end
                return string.format(' net_slide=%.0f,%.0f', feet.left_foot.net or 0, feet.right_foot.net or 0)
            end)())
        testing.expect(fallAt ~= nil, 'the blow did not knock the body down')
    end)
    if victim:isValid() then victim:remove() end
    ragdoll.setAutomaticRecovery(true)
    if not ok then error(message, 0) end
end)
end

-- SHOVE-01 (user 10-29: a double right click shoves): a dummy shoves another
-- standing in front of it. In reach, the hands go to its chest and it is
-- pushed off its spot (steps, or a fall); out of reach nothing happens.
-- OPENMW_RAGDOLL_TEST_SHOVES = shoves 1 s apart (default 1).
for shoveIndex, case in ipairs({
    { label = 'in reach', gap = 50, pushed = true },
    { label = 'out of reach', gap = 140, pushed = false },
}) do
registerTest('SHOVE-01 a shove ' .. case.label, 918400 + shoveIndex, function()
    placePlayer()
    ragdoll.setAutomaticRecovery(true)
    local shover = spawnDummy('SHOVE-01 shover', util.vector3(60, 60, 0), false)
    local target = spawnDummy('SHOVE-01 target', util.vector3(60, 60 + case.gap, 0), false)
    local ok, message = pcall(function()
        powerAndStand(shover, 'SHOVE-01 shover')
        powerAndStand(target, 'SHOVE-01 target')
        waitSeconds(0.5)
        local flat = function(v) return util.vector3(v.x, v.y, 0) end
        local t0, s0 = ragdoll.getState(target), ragdoll.getState(shover)
        local from = flat(body(t0, 'pelvis').centerOfMass)
        local shoverFrom = flat(body(s0, 'pelvis').centerOfMass)
        local shoves = tonumber(ragdoll.tuningText('OPENMW_RAGDOLL_TEST_SHOVES') or '') or 1
        local far, handNear, pelvisLow, modes = 0, math.huge, math.huge, {}
        local balanceTip, balanceWay = 0, '-' 
        -- How far the hands go out along the line to the target (from where
        -- they hung before the shove).
        local line = flat(body(t0, 'pelvis').centerOfMass) - shoverFrom
        line = line / math.max(line:length(), 1)
        local handOut = 0
        local handFrom = {}
        for _, hand in ipairs({ 'left_hand', 'right_hand' }) do handFrom[hand] = flat(body(s0, hand).centerOfMass) end
        local ground = math.min(body(t0, 'left_foot').centerOfMass.z, body(t0, 'right_foot').centerOfMass.z)
        -- OPENMW_RAGDOLL_TEST_SHOVE_CLICKS: rapid extra right clicks, 0.1 s apart.
        local clicks = tonumber(ragdoll.tuningText('OPENMW_RAGDOLL_TEST_SHOVE_CLICKS') or '') or 0
        local fatigue0 = types.Actor.stats.dynamic.fatigue(shover).current
        for _ = 1, shoves do
            ragdoll.shove(shover)
            for _ = 1, clicks do
                waitSeconds(0.1)
                ragdoll.shove(shover)
            end
            waitSeconds(1.0, function()
                local ts, ss = ragdoll.getState(target), ragdoll.getState(shover)
                if not ts or not ss then return end
                far = math.max(far, (flat(body(ts, 'pelvis').centerOfMass) - from):length())
                pelvisLow = math.min(pelvisLow, body(ts, 'pelvis').centerOfMass.z - ground)
                if ts.controller then modes[ts.controller.mode] = true end
                -- HUD-01: the balance reading of the shoved body.
                if (ts.balanceTip or 0) > balanceTip then
                    balanceTip, balanceWay = ts.balanceTip, string.format('%.2f,%.2f', ts.balanceRight or 0, ts.balanceForward or 0)
                end
                local chest = body(ts, 'spine2').centerOfMass
                for _, hand in ipairs({ 'left_hand', 'right_hand' }) do
                    handNear = math.min(handNear, (body(ss, hand).centerOfMass - chest):length())
                    handOut = math.max(handOut, (flat(body(ss, hand).centerOfMass) - handFrom[hand]):dot(line))
                end
            end)
        end
        waitSeconds(1.5, function()
            local ts = ragdoll.getState(target)
            if not ts then return end
            far = math.max(far, (flat(body(ts, 'pelvis').centerOfMass) - from):length())
            pelvisLow = math.min(pelvisLow, body(ts, 'pelvis').centerOfMass.z - ground)
            if ts.controller then modes[ts.controller.mode] = true end
        end)
        -- The lunge: how far the shover went toward the target.
        local recoil = (flat(body(ragdoll.getState(shover), 'pelvis').centerOfMass) - shoverFrom):dot(line)
        local spent = fatigue0 - types.Actor.stats.dynamic.fatigue(shover).current
        local seen = {}
        for mode in pairs(modes) do seen[#seen + 1] = mode end
        table.sort(seen)
        print(string.format('SHOVE-01 case=%s gap=%d shoves=%d target_travel=%.0f pelvis_low=%.0f hand_near=%.0f hand_out=%.0f lunge=%.0f fatigue=%.1f clicks=%d modes=%s balance_tip=%.2f balance_way=%s',
            case.label, case.gap, shoves, far, pelvisLow, handNear, handOut, recoil, spent, clicks, table.concat(seen, ','), balanceTip, balanceWay))
        if case.pushed then
            testing.expect(far >= 8, 'the shove did not move the target off its spot')
            testing.expect(handOut >= 12, 'the hands did not go out toward the target')
        else
            testing.expect(far < 8, 'a shove out of reach moved the target')
        end
    end)
    if shover:isValid() then shover:remove() end
    if target:isValid() then target:remove() end
    if not ok then error(message, 0) end
end)
end

-- BOUNCE-01 (user 10-29: "some hits make the enemies just bounce up"): a
-- heavy rising blow on the chest (the STAGGER-01 heavy blow, 1600 u/s 2.4 kg,
-- no damage) must not lift the body off its feet. Logged: how high the
-- lowest foot and the pelvis rose within 1.5 s.
for bounceIndex, case in ipairs({
    { label = 'mostly up', direction = util.vector3(0, 0.35, 1) },
    { label = 'up at 30 degrees', direction = util.vector3(0, 0.87, 0.5) },
}) do
registerTest('BOUNCE-01 a rising blow ' .. case.label, 918600 + bounceIndex, function()
    placePlayer()
    ragdoll.setAutomaticRecovery(true)
    local victim = spawnDummy('BOUNCE-01 victim', util.vector3(60, 90, 0), false)
    local ok, message = pcall(function()
        powerAndStand(victim, 'BOUNCE-01 victim')
        waitSeconds(0.5)
        local start = ragdoll.getState(victim)
        local feet0 = math.min(body(start, 'left_foot').centerOfMass.z, body(start, 'right_foot').centerOfMass.z)
        local pelvis0 = body(start, 'pelvis').centerOfMass.z
        ragdoll.applyBlow(victim, 'spine2', case.direction:normalize(), 1600, 2.4, 0)
        local footRise, pelvisRise = 0, 0
        waitSeconds(1.5, function()
            local st = ragdoll.getState(victim)
            if not st then return end
            footRise = math.max(footRise,
                math.min(body(st, 'left_foot').centerOfMass.z, body(st, 'right_foot').centerOfMass.z) - feet0)
            pelvisRise = math.max(pelvisRise, body(st, 'pelvis').centerOfMass.z - pelvis0)
        end)
        print(string.format('BOUNCE-01 case=%s foot_rise=%.1f pelvis_rise=%.1f', case.label, footRise, pelvisRise))
        testing.expect(footRise < 6, 'the blow lifted both feet off the ground')
    end)
    if victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)
end

-- BRACE-01 (user bookmark 2026-10-31 hub session, "NPC stuck in recovery": a
-- longsword duellist hit in the chest toppled, caught itself, and braced in
-- place for 17 s while its AI chased): an NPC with a longsword walks forward
-- (held walk command) and takes a chest blow across its path, no damage.
-- Logged: the walking speed before, whether it fell, how long until it had
-- walked 80 units on, its longest stop (under 20 u/s) and time braced after
-- the blow; the engine logs RAGDOLL_TOPPLE / _REHOLD / RAGDOLL_BRACE_REACQUIRE.
-- A blow (its push is capped) and a raw chest impulse of 41000 stagger it,
-- 50000 fells it; none takes the topple-and-rehold path (no scripted push
-- found one: 20000-70000 go from a stagger straight to a fall), so this
-- guards walking on after a blow, and ARMS-01 with the longsword logs the
-- rehold (RAGDOLL_BRACE_REACQUIRE) when a duel topples.
for braceIndex, speed in ipairs({ 2500, 'shove 41000', 'shove 50000' }) do
registerTest('BRACE-01 a walking longsword NPC hit in the chest at ' .. speed, 918980 + braceIndex, function()
    placePlayer()
    ragdoll.setAutomaticRecovery(true)
    local walker = spawnDummy('BRACE-01 walker', util.vector3(-450, 450, 0), 'steel longsword')
    local ok, message = pcall(function()
        -- The hub duellist's Agility (30): fewer catch steps, so it topples.
        pcall(function() types.Actor.stats.attributes.agility(walker).base = 30 end)
        powerAndStand(walker, 'BRACE-01 walker')
        local from, fromTime = walker.position, core.getSimulationTime()
        walker:sendEvent('fixActorMove', 1)
        waitSeconds(2.0)
        local blowAt = walker.position
        local path = util.vector3(blowAt.x - from.x, blowAt.y - from.y, 0)
        local before = path:length() / math.max(core.getSimulationTime() - fromTime, 0.1)
        local across = path:length() > 1 and util.vector3(-path.y, path.x, 0):normalize() or util.vector3(1, 0, 0)
        local shove = type(speed) == 'string' and tonumber(speed:match('%d+'))
        if shove then
            local state = ragdoll.getState(walker)
            local chest = 1
            for index, item in ipairs(state.bodies) do
                if item.name == 'spine2' then chest = index end
            end
            ragdoll.applyImpulse(walker, chest, (across + util.vector3(0, 0, 0.15)):normalize() * shove)
        else
            ragdoll.applyBlow(walker, 'spine2', (across + util.vector3(0, 0, 0.1)):normalize(), speed, 6, 0)
        end
        local started = core.getSimulationTime()
        local last, lastTime = walker.position, started
        local fell, onAgain, stop, longestStop, braced = false, nil, 0, 0, 0
        waitSeconds(6, function()
            local now = core.getSimulationTime()
            local dt = now - lastTime
            if dt <= 0 then return end
            local here = walker.position
            local step = util.vector3(here.x - last.x, here.y - last.y, 0):length()
            last, lastTime = here, now
            local state = ragdoll.getState(walker)
            local mode = state and state.controller and state.controller.mode or '-'
            if mode == 'falling' or mode == 'fallen' then fell = true end
            if mode == 'bracing' then braced = braced + dt end
            if not onAgain and util.vector3(here.x - blowAt.x, here.y - blowAt.y, 0):length() >= 80 then
                onAgain = now - started
            end
            if step / dt < 20 then
                stop = stop + dt
                longestStop = math.max(longestStop, stop)
            else
                stop = 0
            end
        end)
        walker:sendEvent('fixActorMove', 0)
        print(string.format('BRACE-01 blow=%s walk_before=%.0f fell=%s on_again=%s longest_stop=%.1f braced=%.1f',
            tostring(speed), before, tostring(fell), onAgain and string.format('%.1f', onAgain) or 'never', longestStop, braced))
        testing.expect(before > 40, 'the walker did not walk before the blow')
        testing.expect(fell or longestStop < 3, 'stood in place over 3 s after the blow without falling')
    end)
    if walker:isValid() then walker:remove() end
    if not ok then error(message, 0) end
end)
end

-- ROUGH-01 (user 10-29: "fighting on irregular terrain (in the dungeon) makes
-- it very hard to stay in a combat stance and causes sudden slow downs"): an
-- NPC with a longsword drawn walks the cave floor of Ashinabi (the dungeon the
-- user played) under the AI's travel package, from the entrance to the
-- farthest of its NPCs' spots (the cell's own NPCs and creatures disabled, the
-- player parked at the far end so the cell is active). Logged: the time, the
-- path, the speed and its dips (under 40 u/s for 0.3 s or more while still
-- travelling), the controller modes, the weapon stance kept, a fall.
-- OPENMW_RAGDOLL_TEST_ROUGH = "cell name" (default Ashinabi).
registerTest('ROUGH-01 an armed NPC walks a cave floor', 918500, function()
    local name = ragdoll.tuningText('OPENMW_RAGDOLL_TEST_ROUGH') or 'Ashinabi'
    local cell = world.getCellByName(name)
    local entrance
    for _, door in ipairs(cell:getAll(types.Door)) do
        if types.Door.isTeleport(door) then entrance = door.position end
    end
    testing.expect(entrance ~= nil, 'no entrance door in ' .. name)
    -- The cell's actors stand on real floor: start at the one nearest the
    -- entrance (the door's own spot is inside its mesh - the walker was put
    -- to swimming there), walk to the one farthest from it.
    local spots = {}
    for _, list in ipairs({ cell:getAll(types.NPC), cell:getAll(types.Creature) }) do
        for _, actor in ipairs(list) do
            spots[#spots + 1] = actor.position
            actor.enabled = false
        end
    end
    -- Dry spots only (Ashinabi is partly under water: a walker started in
    -- it was put to swimming).
    local water = cell.hasWater and cell.waterLevel or nil
    local dry = {}
    for _, spot in ipairs(spots) do
        if not water or spot.z > water + 20 then dry[#dry + 1] = spot end
    end
    print(string.format('ROUGH-01 setup water=%s spots=%d dry=%d', tostring(water), #spots, #dry))
    spots = dry
    table.sort(spots, function(a, b) return (a - entrance):length() < (b - entrance):length() end)
    entrance = spots[1]
    -- The farthest spot within a short walk (the far end of Ashinabi is
    -- 7500 u away, past water).
    local far, farDistance = spots[2], spots[2] and (spots[2] - entrance):length() or 0
    for _, spot in ipairs(spots) do
        local d = (spot - entrance):length()
        if d > farDistance and d <= 2500 then far, farDistance = spot, d end
    end
    print(string.format('ROUGH-01 setup walk=%.0f', farDistance))
    testing.expect(far ~= nil, 'no spot to walk to in ' .. name)
    -- The player beside the start (an actor far from the player is not
    -- animated, so it got no body to power), the walker a step ahead.
    local player = world.players[1]
    local ahead = (far - entrance):normalize()
    player:teleport(name, entrance - ahead * 80, { onGround = true })
    waitFor('player in ' .. name, function() return player.cell and player.cell.name == name end, 10)
    local record = world.createRecord(types.NPC.createRecordDraft({
        template = types.NPC.record(player), name = 'ROUGH-01 walker',
        isEssential = false, isPersistent = false, isRespawning = false,
    }))
    local walker = world.createObject(record.id)
    world.createObject('iron longsword', 1):moveInto(types.Actor.inventory(walker))
    walker:teleport(name, entrance + ahead * 40, { onGround = true })
    walker:addScript('stance.lua')
    coroutine.yield()
    walker:sendEvent('fixActorConfigure', { weaponStance = 'iron longsword' })
    waitFor('ROUGH-01 walker stance', function() return readyActors[walker.id] ~= nil end, 10)
    local ok, message = pcall(function()
        -- Upright on uneven floor reads as bracing too.
        ragdoll.setPowered(walker, true)
        local lastMode = '-'
        local okStand = pcall(waitFor, 'ROUGH-01 walker upright', function()
            local state = ragdoll.getState(walker)
            lastMode = state and state.controller and state.controller.mode or (state and 'no controller' or 'no state')
            return state and state.powered and (lastMode == 'standing' or lastMode == 'bracing')
        end, 12)
        testing.expect(okStand, 'the walker never stood up powered (mode ' .. tostring(lastMode) .. ')')
        local dest = far + (entrance - far):normalize() * 90
        roughRoute = nil
        walker:sendEvent('fixActorRoute', dest)
        waitFor('ROUGH-01 route', function() return roughRoute ~= nil end, 5)
        print(string.format('ROUGH-01 route points=%d status=%s', roughRoute.points, roughRoute.status))
        roughDone = false
        local started = core.getSimulationTime()
        local last, path, slow, dips, slowTime, drawnTime, total = walker.position, 0, 0, 0, 0, 0, 0
        local modes, fell, pelvisDrop = {}, false, 0
        local balanceMax, balanceShown = 0, 0
        local standingPelvis = nil
        local maxSeconds = 50
        local legs = 1
        waitSeconds(maxSeconds, function()
            -- A partial route: ask again from here, up to six legs.
            if roughDone and legs < 6 then
                roughDone, legs = false, legs + 1
                walker:sendEvent('fixActorRoute', dest)
            end
            if roughDone then return end
            local dt = core.getSimulationTime() - started - total
            if dt <= 0 then return end
            total = total + dt
            local now = walker.position
            local step = util.vector3(now.x - last.x, now.y - last.y, 0):length()
            path = path + step
            last = now
            local remaining = util.vector3(dest.x - now.x, dest.y - now.y, 0):length()
            local state = ragdoll.getState(walker)
            if state and state.controller then
                modes[state.controller.mode] = true
                if state.controller.mode == 'falling' or state.controller.mode == 'fallen' then fell = true end
                local pelvis = body(state, 'pelvis').centerOfMass.z - now.z
                standingPelvis = standingPelvis or pelvis
                pelvisDrop = math.max(pelvisDrop, standingPelvis - pelvis)
                -- HUD-01: how often walking alone would show a balance wedge.
                balanceMax = math.max(balanceMax, state.balanceTip or 0)
                if (state.balanceTip or 0) > 0.3 then balanceShown = balanceShown + dt end
            end
            if types.Actor.getStance(walker) == types.Actor.STANCE.Weapon then drawnTime = drawnTime + dt end
            if total > 1.5 and remaining > 150 then
                if step / dt < 40 then
                    slowTime = slowTime + dt
                    if slowTime >= 0.3 and slow == 0 then dips = dips + 1 end
                    slow = slowTime >= 0.3 and 1 or 0
                else
                    slowTime, slow = 0, 0
                end
            end
        end)
        local remaining = util.vector3(dest.x - walker.position.x, dest.y - walker.position.y, 0):length()
        local seen = {}
        for mode in pairs(modes) do seen[#seen + 1] = mode end
        table.sort(seen)
        print(string.format('ROUGH-01 cell=%s legs=%d straight=%.0f path=%.0f left=%.0f seconds=%.1f speed=%.0f dips=%d drawn=%.2f pelvis_drop=%.0f fell=%s modes=%s balance_max=%.2f balance_shown=%.1f',
            name, legs, (dest - entrance):length(), path, remaining, total, path / math.max(total, 0.1), dips,
            drawnTime / math.max(total, 0.1), pelvisDrop, tostring(fell), table.concat(seen, ','), balanceMax, balanceShown))
    end)
    if walker:isValid() then walker:remove() end
    if not ok then error(message, 0) end
end)

registerTest('KINETIC-01 the options page saves whitelisted switches', 918006, function()
    local player = placePlayer()
    kineticResult = nil
    player:sendEvent('fixKineticCheck')
    waitFor('kinetic result', function() return kineticResult ~= nil end, 5)
    local r = kineticResult
    print(string.format('KINETIC-01 wrote=%s refused=%s unsafe=%s saved=%s cleared=%s after=%s', tostring(r.wrote),
        tostring(r.refused), tostring(r.unsafe), tostring(r.saved), tostring(r.cleared), tostring(r.after)))
    testing.expect(r.wrote == true and r.saved == '0', 'a whitelisted switch was not saved')
    testing.expect(r.refused == false and r.unsafe == false, 'an unknown switch or unsafe value was accepted')
    testing.expect(r.cleared == true and r.after == '', 'clearing a switch left it saved')
end)

-- POSEMAP-01 (alpha 0.9 pose map, user 2026-10-02: drags should not make
-- the player manage elbows and wrists): with a held drag, steer the blade
-- slowly to each cell of a grid (azimuth x elevation, degrees), let it settle
-- and log the body's orientations there (x and y axes of each body, world)
-- plus the blade's actual direction and the wrist excess. tools/body/posemap.py
-- turns the log into the weapon class's pose map.
for mapIndex, weapon in ipairs({ 'iron claymore', 'iron longsword' }) do
    registerTest('POSEMAP-01 capture guard poses with ' .. weapon, 918600 + mapIndex, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            ragdoll.setPlayerAlwaysHits(true)
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 600 + mapIndex, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 600 + mapIndex end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local perDeg = 1500 / 180
            local names = { 'pelvis', 'spine', 'spine1', 'spine2', 'left_clavicle', 'left_upper_arm', 'left_forearm',
                'left_hand', 'right_clavicle', 'right_upper_arm', 'right_forearm', 'right_hand' }
            local function bladeDirection(s)
                local b = s.bodies[s.weapon.bodyIndex]
                local tip = b.position + (b.rotation * util.vector3(0, 0, 1)) * (s.weapon.length * 0.5)
                local d = tip - body(s, 'spine2').position
                return math.deg(math.atan2(d.x, d.y)), math.deg(math.atan2(d.z, math.sqrt(d.x * d.x + d.y * d.y)))
            end
            -- Steer the command toward (az, el) at about 1 degree per step.
            local function steer(az, el)
                local s = ragdoll.getState(player)
                local t = s.tip
                if not t then return end
                local dx, dy = az - math.deg(t.az), el - math.deg(t.el)
                local steps = math.max(math.abs(dx), math.abs(dy), 1)
                local step0 = tonumber(s.physicsStep)
                local sentX, sentY = 0, 0
                waitFor('steer', function()
                    local n = tonumber(ragdoll.getState(player).physicsStep) - step0
                    local f = math.min(1, n / steps)
                    local x, y = math.floor(dx * perDeg * f), math.floor(-dy * perDeg * f)
                    if x ~= sentX or y ~= sentY then ragdoll.injectMouseInput(x - sentX, y - sentY) end
                    sentX, sentY = x, y
                    return n >= steps + 6
                end, steps / 30 + 6)
            end
            waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            waitSeconds(0.6)
            local presses = ragdoll.getState(player).playerPressCount
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitFor('press registered', function() return ragdoll.getState(player).playerPressCount > presses end, 3)
            for _, el in ipairs({ -20, 10, 40, 70 }) do
                for _, az in ipairs({ -60, -30, 0, 30, 60 }) do
                    -- Twice: the command can lag the first pass at the edges.
                    steer(az, el)
                    steer(az, el)
                    waitSeconds(0.8)
                    local s = ragdoll.getState(player)
                    local bladeAz, bladeEl = bladeDirection(s)
                    local parts = {}
                    for _, name in ipairs(names) do
                        local b = body(s, name)
                        local x, y = b.rotation * util.vector3(1, 0, 0), b.rotation * util.vector3(0, 1, 0)
                        local p = b.position
                        parts[#parts + 1] = string.format('%s=%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f', name,
                            x.x, x.y, x.z, y.x, y.y, y.z, p.x, p.y, p.z)
                    end
                    local wb = s.bodies[s.weapon.bodyIndex]
                    local bladeDir = wb.rotation * util.vector3(0, 0, 1)
                    print(string.format('POSEMAP-01 weapon=%s az=%d el=%d cmd_az=%.1f cmd_el=%.1f blade_az=%.1f blade_el=%.1f blade_dir=%.4f,%.4f,%.4f twist=%.1f actor=%.2f,%.2f,%.2f out_r=%.1f out_l=%.1f %s',
                        weapon, az, el, math.deg(s.tip.az), math.deg(s.tip.el), bladeAz, bladeEl, bladeDir.x, bladeDir.y,
                        bladeDir.z, math.deg(s.tip.bodyTwist or 0), player.position.x, player.position.y, player.position.z,
                        s.wristOutsideRight or -1, s.wristOutsideLeft or -1, table.concat(parts, ' ')))
                end
            end
            ragdoll.injectMouseInput(0, 0, false, 1)
        end)
        ragdoll.setPlayerAlwaysHits(false)
        if not ok then error(message, 0) end
    end)
end

-- POSEMAP-02 (alpha 0.9): held drags that should feel smooth - a circle, a
-- figure eight, two diagonals and a wind-up (drag up and back) then a fast
-- downward stroke. Scores per weapon: blade jerk (p90 change of the blade's
-- angular step, degrees), wrist excess p90, elbow flips (the weapon elbow's
-- side changing sign), the command gap, and the wind-up's deepest blade-back
-- (the blade direction's backward share at the top: > 0 = pointing behind).
for smoothIndex, weapon in ipairs({ 'iron claymore', 'iron longsword' }) do
    registerTest('POSEMAP-02 smooth drags with ' .. weapon, 918610 + smoothIndex, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            ragdoll.setPlayerAlwaysHits(true)
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 610 + smoothIndex, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 610 + smoothIndex end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local perDeg = 1500 / 180
            local prevDir, prevStep, jerks, wrists, gaps = nil, nil, {}, {}, {}
            local elbowSide, flips, back, sideMax = nil, 0, -1, -180
            local function sample(windup)
                local s = ragdoll.getState(player)
                if not s or not s.weapon then return end
                local b = s.bodies[s.weapon.bodyIndex]
                local dir = (b.rotation * util.vector3(0, 0, 1)):normalize()
                if prevDir then
                    local step = math.deg(math.acos(math.max(-1, math.min(1, dir:dot(prevDir)))))
                    if prevStep then jerks[#jerks + 1] = math.abs(step - prevStep) end
                    prevStep = step
                end
                prevDir = dir
                wrists[#wrists + 1] = math.max(s.wristOutsideRight or 0, 0)
                if s.tip then gaps[#gaps + 1] = s.tip.gap or 0 end
                local sh, el, ha = body(s, 'right_upper_arm').position, body(s, 'right_forearm').position,
                    body(s, 'right_hand').position
                -- An elbow snap: the weapon elbow jumps over 6 units against
                -- the chest in one step (a flip of the arm's solution).
                local rel = el - body(s, 'spine2').position
                if elbowSide and (rel - elbowSide):length() > 6 then flips = flips + 1 end
                elbowSide = rel
                if windup then back = math.max(back, -dir.y) end
                if windup == 'side' then sideMax = math.max(sideMax, math.deg(math.atan2(dir.x, dir.y))) end
            end
            local function run(points, stepsPerLeg, windupLegs)
                local x0, y0 = 0, 0
                for leg, p in ipairs(points) do
                    local dx, dy = p[1] - x0, p[2] - y0
                    local steps = stepsPerLeg[leg] or stepsPerLeg[1]
                    local step0 = tonumber(ragdoll.getState(player).physicsStep)
                    local sentX, sentY = 0, 0
                    waitFor('leg', function()
                        local n = tonumber(ragdoll.getState(player).physicsStep) - step0
                        local f = math.min(1, n / steps)
                        local x, y = math.floor(dx * perDeg * f), math.floor(-dy * perDeg * f)
                        if x ~= sentX or y ~= sentY then ragdoll.injectMouseInput(x - sentX, y - sentY) end
                        sentX, sentY = x, y
                        sample(windupLegs and windupLegs[leg])
                        return n >= steps
                    end, steps / 30 + 6)
                    x0, y0 = p[1], p[2]
                end
            end
            local function press()
                waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                waitSeconds(0.6)
                local presses = ragdoll.getState(player).playerPressCount
                ragdoll.injectMouseInput(0, 0, true, 1)
                waitFor('press registered', function() return ragdoll.getState(player).playerPressCount > presses end, 3)
            end
            local function release()
                ragdoll.injectMouseInput(0, 0, false, 1)
                waitSeconds(1.0)
            end
            -- Circle: 8 legs around a 50-degree radius.
            press()
            local circle = {}
            for k = 0, 8 do
                local a = k * math.pi / 4
                circle[#circle + 1] = { 50 * math.sin(a), 50 * math.cos(a) - 50 }
            end
            run(circle, { 12 })
            release()
            -- Figure eight.
            press()
            run({ { 50, 30 }, { 0, 0 }, { -50, -30 }, { 0, 0 }, { 50, -30 }, { 0, 0 }, { -50, 30 }, { 0, 0 } }, { 14 })
            release()
            -- Diagonals: top right to bottom left and back.
            press()
            run({ { 50, 40 }, { -50, -40 }, { 50, 40 } }, { 20, 25, 25 })
            release()
            -- Wind-up: drag up and over (100 degrees) slowly, then strike down fast.
            press()
            run({ { 0, 100 }, { 0, -60 } }, { 60, 8 }, { true, false })
            release()
            -- Side wind-up: drag 160 degrees to the right slowly, then cut back fast.
            press()
            run({ { 160, 0 }, { -40, 0 } }, { 80, 8 }, { 'side', false })
            release()
            local function p90(t)
                if #t == 0 then return 0 end
                table.sort(t)
                return t[math.max(1, math.floor(0.9 * #t))]
            end
            local gapSum = 0
            for _, g in ipairs(gaps) do gapSum = gapSum + g end
            print(string.format('POSEMAP-02 weapon=%s jerk_p90=%.2f wrist_p90=%.1f elbow_snaps=%d gap_mean=%.1f windup_back=%.2f side_max=%.0f samples=%d',
                weapon, p90(jerks), p90(wrists), flips, #gaps > 0 and gapSum / #gaps or -1, back, sideMax, #jerks))
        end)
        ragdoll.setPlayerAlwaysHits(false)
        if not ok then error(message, 0) end
    end)
end

-- LOOK-01 (alpha 0.9, user 2026-10-02: the pose map "should follow where I
-- look, so we can aim up/down"): with a longsword drawn, look 30 degrees down,
-- level and 30 degrees up and read the guard's elevation; it must rise with
-- the look.
registerTest('LOOK-01 the guard follows the look', 918620, function()
    local player = placePlayer()
    local ok, message = pcall(function()
        if not types.Actor.inventory(player):find('iron longsword') then
            world.createObject('iron longsword', 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        player:sendEvent('fixPlayerConfigure', { weapon = 'iron longsword', sequence = 620, firstPerson = true })
        waitFor('player loadout', function() return playerReadySequence == 620 end)
        waitSeconds(1)
        powerAndStand(player, 'iron longsword')
        local results = {}
        for _, pitch in ipairs({ -30, 0, 30 }) do
            local eye = player.position + util.vector3(0, 0, 110)
            local target = eye + util.vector3(0, 200, 200 * math.tan(math.rad(pitch)))
            player:sendEvent('fixAimAt', target)
            waitSeconds(2.0)
            local s = ragdoll.getState(player)
            local el = s.tip and math.deg(s.tip.el) or -999
            results[#results + 1] = el
            print(string.format('LOOK-01 look=%d guard_el=%.1f actor_pitch=%.3f', pitch, el, player.rotation:getPitch()))
        end
        testing.expect(results[3] > results[1] + 10, 'the guard did not rise with the look')
    end)
    if not ok then error(message, 0) end
end)

registerTest('FIX-03 leg bones do not depend on the pose the ragdoll was created in', 918003, function()
    placePlayer()
    local idle = spawnDummy('FIX-03 idle', util.vector3(-70, 90, 0), false)
    local stance = spawnDummy('FIX-03 weapon stance', util.vector3(70, 90, 0), true)
    local ok, message = pcall(function()
        -- Let the weapon-stance clip settle into its staggered feet.
        waitSeconds(1.5)
        local idleState = powerAndStand(idle, 'FIX-03 idle')
        local stanceState = powerAndStand(stance, 'FIX-03 weapon stance')
        waitSeconds(0.25)
        idleState = ragdoll.getState(idle)
        stanceState = ragdoll.getState(stance)
        testing.expect(stanceState.weapon and stanceState.weapon.attached, 'stance dagger has no physical attachment')
        print(string.format('FIX-05 dagger proxy length %.3f grip separation %.3f',
            stanceState.weapon.length, stanceState.weapon.gripSeparation))
        testing.expect(stanceState.weapon.length > 5 and stanceState.weapon.length < 45,
            'iron dagger collision length does not match its short mesh')
        testing.expect(stanceState.weapon.gripSeparation < 1.5, 'dagger proxy is detached from the hand')
        local worst = 0
        local worstPair = ''
        for _, pair in ipairs({
            { 'left_thigh', 'left_calf' }, { 'left_calf', 'left_foot' },
            { 'right_thigh', 'right_calf' }, { 'right_calf', 'right_foot' },
        }) do
            local function boneOffset(state)
                local parent = body(state, pair[1])
                local child = body(state, pair[2])
                return parent.rotation:inverse() * (child.position - parent.position)
            end
            local difference = (boneOffset(idleState) - boneOffset(stanceState)):length()
            print(string.format('FIX-03 %s -> %s bone offset difference %.3f', pair[1], pair[2], difference))
            if difference > worst then worst, worstPair = difference, pair[1] .. ' -> ' .. pair[2] end
        end
        -- Physical joints give a little under load; the averaged anchors were
        -- off by five to ten units.
        testing.expect(worst < 1.5, string.format(
            'leg bone %s differs by %.2f units between an idle-created and a stance-created ragdoll',
            worstPair, worst))
    end)
    if idle:isValid() then idle:remove() end
    if stance:isValid() then stance:remove() end
    if not ok then error(message, 0) end
end)


-- PLAY-01 manual regression: exercise damage, not merely empty-space reach.
local contactLoadouts = {
    { weapon = false }, { weapon = 'iron dagger' }, { weapon = false, quick = true },
    { weapon = false, quick = true, guided = true },
}
for _, weapon in ipairs({ 'iron dagger', 'iron longsword', 'iron claymore', 'iron warhammer',
    'steel warhammer', 'wooden staff', 'iron spear', 'iron battle axe' }) do
    contactLoadouts[#contactLoadouts + 1] = { weapon = weapon, quick = true, guided = true, firstPerson = true }
end
for _, weapon in ipairs({ 'iron dagger', 'iron longsword', 'iron claymore', 'iron warhammer',
    'steel warhammer', 'wooden staff', 'iron spear', 'iron battle axe' }) do
    contactLoadouts[#contactLoadouts + 1] = { weapon = weapon, quick = true, guided = true, firstPerson = true, aimed = true }
end
for _, weapon in ipairs({ 'iron dagger', 'iron longsword', 'iron claymore', 'iron warhammer',
    'steel warhammer', 'wooden staff', 'iron spear', 'iron battle axe' }) do
    contactLoadouts[#contactLoadouts + 1] = { weapon = weapon, drag = true, guided = true, firstPerson = true, aimed = true }
end
-- 2b committed swings: the user's real drags run at about 27000 mouse
-- counts/s (manual recordings 2026-09-28, strokes.py); the drags above sweep
-- at about 860. Same wind-up, then a sweep at the user's speed.
for _, weapon in ipairs({ 'iron dagger', 'iron longsword', 'iron claymore', 'iron warhammer' }) do
    contactLoadouts[#contactLoadouts + 1] = { weapon = weapon, drag = true, fast = true, guided = true, firstPerson = true, aimed = true }
end
-- The one-click attack under hand control (2026-10-03): a left click
-- with no drag plays the controller's own attack (standing: the chop).
for _, weapon in ipairs({ 'iron longsword', 'iron claymore', 'iron warhammer', 'iron battle axe' }) do
    contactLoadouts[#contactLoadouts + 1] = { weapon = weapon, click = true, guided = true, firstPerson = true, aimed = true }
end
for index, loadout in ipairs(contactLoadouts) do
    local weapon = loadout.weapon
    local label = (loadout.click and ('click ' .. weapon))
        or (loadout.drag and ((loadout.fast and 'fast drag ' or 'held drag ') .. weapon)
        or loadout.guided and ('guided ' .. (weapon or 'fists')) or loadout.quick and 'quick fists' or weapon or 'fists')
        .. (loadout.firstPerson and ' first person' or '')
        .. (loadout.aimed and ' crosshair centered' or '')
    registerTest('FIX-08 repeated native player hits with ' .. label, 918020 + index, function()
        local player = placePlayer()
        local victim
        local savedReaction = ragdoll.reactionScale()
        local savedHandScale = ragdoll.handDamageScale()
        local ok, message = pcall(function()
            ragdoll.setPlayerAlwaysHits(true)
            ragdoll.handDamageScale(0.35)
            if weapon and not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', {
                weapon = weapon, sequence = 20 + index,
                firstPerson = loadout.firstPerson and not (loadout.quick or loadout.drag or loadout.click),
            })
            waitFor('player loadout', function() return playerReadySequence == 20 + index end)
            -- Contact and damage at full control ("always hits"), so at a
            -- master's skill too: below 50 a click in the claymore's hold
            -- waits for the return (HSC_SKILL, the real skill since 10-31)
            -- and its blow came after the attack's damage window (FIX-08
            -- guided claymore 1 of 3 at the test skill, 3 of 3 at 100).
            player:sendEvent('fixSetWeaponSkill', 100)
            waitSeconds(1)
            powerAndStand(player, label)
            if loadout.firstPerson and (loadout.quick or loadout.drag or loadout.click) then
                cameraReadyMode = nil
                player:sendEvent('fixPlayerCamera', true)
                waitFor('quick contact first-person camera', function() return cameraReadyMode == true end)
            end
            victim = spawnDummy('FIX-08 contact target', util.vector3(0, 200, 0), false)
            powerAndStand(victim, 'contact target')
            victim:teleport(testArea.cell, player.position + util.vector3(0, 35, 0),
                { onGround = true, rotation = util.transform.rotateZ(math.pi) })
            -- Isolate hit registration from knockback and repeated knockdown.
            ragdoll.reactionScale(0)
            local victimModes = {}
            waitSeconds(0.5, function()
                local vs = ragdoll.getState(victim)
                local m = vs and vs.controller and vs.controller.mode or 'none'
                if victimModes[#victimModes] ~= m then victimModes[#victimModes + 1] = m end
            end)
            local vs = ragdoll.getState(victim)
            print(string.format('FIX-08 target settled %s modes=%s chest_z=%.1f dist=%.1f', label,
                table.concat(victimModes, '>'), vs and (body(vs, 'spine2').position.z - victim.position.z) or -1,
                (victim.position - player.position):length()))
            ragdoll.prepareMouseInput()
            local function damageTotal()
                local total = 0
                for _, value in pairs(ragdoll.getLimbDamage(victim)) do total = total + value end
                return total
            end
            local function performContactInput(observe)
                local button = weapon and not (loadout.quick or loadout.drag or loadout.click) and 2 or 1
                local attackId = ragdoll.getState(player).attackId
                local ps = ragdoll.getState(player)
                print(string.format('FIX-08 click %s mode=%s phase=%s attackId=%s presses=%s', label,
                    ps.controller and ps.controller.mode or 'none', tostring(ps.attackPhase), tostring(ps.attackId),
                    tostring(ps.playerPressCount)))
                ragdoll.injectMouseInput(0, 0, true, button)
                waitSeconds(loadout.quick and 0.04 or loadout.click and 0.08 or loadout.drag and 0.25 or 0.45, observe)
                if loadout.drag then
                    -- A visible wind-up and a continuous cross-body sweep.
                    -- Integrate the same total mouse motion at every render FPS.
                    -- The drag starts from the raised guard (tip 36 degrees
                    -- up) and moves relative to it: bring the tip down to
                    -- chest height while winding up (~46 degrees, 380
                    -- counts), then sweep across. The old legs swept over
                    -- the victim's head.
                    -- The pull-down is measured from where the tip is to the
                    -- victim's chest (380 counts from the old fixed 36-degree
                    -- guard): a guard that follows the look (pose map, alpha
                    -- 0.9) already sits near the chest.
                    local down = 380
                    local ds = ragdoll.getState(player)
                    -- Hand control (OPENMW_RAGDOLL_HSC) moves the hand:
                    -- bring the blade it points down to the chest.
                    if ds and ds.hsc then
                        local pivot = (body(ds, 'left_upper_arm').position + body(ds, 'right_upper_arm').position) * 0.5
                        local chestNow = body(ragdoll.getState(victim), 'spine2').position
                        local flat = math.sqrt((chestNow.x - pivot.x) ^ 2 + (chestNow.y - pivot.y) ^ 2)
                        local chestEl = math.deg(math.atan2(chestNow.z - pivot.z, flat))
                        down = math.max(0, math.min(600, math.floor((math.deg(ds.hsc.bladeEl) - chestEl) * 1500 / 180)))
                    elseif ds and ds.tip then
                        local pivot = (body(ds, 'left_upper_arm').position + body(ds, 'right_upper_arm').position) * 0.5
                        local chestNow = body(ragdoll.getState(victim), 'spine2').position
                        local flat = math.sqrt((chestNow.x - pivot.x) ^ 2 + (chestNow.y - pivot.y) ^ 2)
                        local chestEl = math.deg(math.atan2(chestNow.z - pivot.z, flat))
                        down = math.max(0, math.min(600, math.floor((math.deg(ds.tip.el) - chestEl) * 1500 / 180)))
                    end
                    local legs = loadout.fast
                        and { { x = 600, y = down, seconds = 0.25 }, { x = -2400, y = 0, seconds = 0.1 } }
                        or { { x = 150, y = down, seconds = 0.25 }, { x = -300, y = 0, seconds = 0.35 } }
                    -- Under hand control (the cut wheel, 2026-10-03) a
                    -- drag attack is a wind-up and the cut back across: 600
                    -- counts right winds the forehand up, 1200 back cuts it
                    -- level across the chest. The legs above were the tip
                    -- controller's (a pull-down, then a 300-count sweep: a
                    -- quarter of a cut on the wheel).
                    if ds and ds.hsc then
                        legs = loadout.fast
                            and { { x = HSC_WIND, y = 0, seconds = 0.25 }, { x = -HSC_CUT, y = 0, seconds = 0.12 } }
                            or { { x = HSC_WIND, y = 0, seconds = 0.4 }, { x = -HSC_CUT, y = 0, seconds = 0.35 } }
                    end
                    for _, leg in ipairs(legs) do
                        local started, sentX, sentY = core.getSimulationTime(), 0, 0
                        waitSeconds(leg.seconds, function()
                            local fraction = math.min(1, (core.getSimulationTime() - started) / leg.seconds)
                            local x, y = math.floor(leg.x * fraction), math.floor(leg.y * fraction)
                            ragdoll.injectMouseInput(x - sentX, y - sentY)
                            sentX, sentY = x, y
                            if observe then observe() end
                            local state = ragdoll.getState(player)
                            testing.expect(state.attackId == attackId and state.attackPhase == 'idle',
                                'held drag launched an automatic attack')
                        end)
                        ragdoll.injectMouseInput(leg.x - sentX, leg.y - sentY)
                    end
                end
                local damageBeforeRelease = damageTotal()
                ragdoll.injectMouseInput(0, 0, false, button)
                return damageBeforeRelease
            end
            local previous = damageTotal()
            local injuringAttacks = 0
            local contactFailures = {}
            local function vitality()
                return types.Actor.stats.dynamic.health(victim).current
                    + types.Actor.stats.dynamic.fatigue(victim).current
            end
            for attack = 1, 3 do
                -- The powered bodies move during a blow. Restore a measured
                -- reachable fixture for each attempt; an actor-origin offset
                -- no longer guarantees contact after the first push apart.
                local p = ragdoll.getState(player)
                -- The tip controller strikes with the right hand; the old
                -- alternating left/right jab is gone.
                local side = 'right'
                local shoulder = body(p, side .. '_upper_arm').position
                local elbow = body(p, side .. '_forearm').position
                local hand = body(p, side .. '_hand').position
                local reach = (elbow - shoulder):length() + (hand - elbow):length()
                local chest = body(ragdoll.getState(victim), 'spine2').position
                -- Guided: the target sits 0.55 of the weapon's length past the
                -- hand, which assumes an end grip. A spear or staff is held near
                -- its middle (body 6-7 units from the hand, under 15% of the
                -- length), so that put the target at the very tip and the
                -- guided spear never struck (2026-09-29); those measure 0.55 of
                -- the real hand-to-tip distance. Other weapons are unchanged.
                local bladeReach = 0
                if weapon then
                    bladeReach = 18
                    if loadout.guided then
                        bladeReach = p.weapon.length * 0.55
                        local wb = p.bodies[p.weapon.bodyIndex]
                        local center = (wb.position - hand):length()
                        if center < 0.15 * p.weapon.length then
                            bladeReach = 0.55 * (center + p.weapon.length * 0.5)
                        end
                    end
                end
                local desired = shoulder + util.vector3(0, reach + bladeReach, 0)
                -- 14: at 20 the quick punch fell short (0 of 4); at 14 the
                -- target sometimes braces against the guard fist and falls
                -- during placement (quick fists passes about half the runs on
                -- every build, 2026-10-01 bisect).
                local fistClearance = 14
                if not weapon then
                    -- The fist guard is already most of a reach out: a chest
                    -- centred at full reach overlapped it and was shoved away
                    -- before the punch. Start just clear of the guard fist.
                    desired = util.vector3(desired.x, math.max(desired.y, hand.y + fistClearance), desired.z)
                end
                placeVictimPoint(victim, function() return body(ragdoll.getState(victim), 'spine2').position end,
                    desired, util.transform.rotateZ(math.pi))
                waitSeconds(0.5)
                -- The target can brace against the guard and fall during
                -- placement (half the fist runs, 2026-10-01: chest 7-62 u at
                -- the click). A blow at a body on the floor measures nothing:
                -- stand it up and place it again (at most twice).
                for retry = 1, 2 do
                    local vs = ragdoll.getState(victim)
                    if vs and vs.controller and vs.controller.mode == 'standing'
                        and body(vs, 'spine2').position.z - victim.position.z > 80 then break end
                    print(string.format('FIX-08 target fell during placement %s retry=%d', label, retry))
                    ragdoll.setPowered(victim, false)
                    coroutine.yield()
                    powerAndStand(victim, 'replaced target')
                    waitSeconds(0.5)
                    placeVictimPoint(victim, function() return body(ragdoll.getState(victim), 'spine2').position end,
                        desired, util.transform.rotateZ(math.pi))
                    waitSeconds(0.5)
                end
                if loadout.aimed then
                    contactAimReady = false
                    player:sendEvent('fixAimAt', body(ragdoll.getState(victim), 'spine2').position)
                    waitFor('crosshair centered on target chest', function() return contactAimReady end, 4)
                    ragdoll.captureScreenshot()
                end
                previous = damageTotal() -- setup contact is not an attack result
                local receiptsBefore = ragdoll.getState(victim).woundReceipts
                local damageEventsBefore = weapon and ragdoll.getState(player).weapon.damageEventCount or 0
                -- Fatigue regenerates between samples. Record an actual
                -- decrease during the hit, rather than comparing net stats
                -- after the whole hold/release window (0.8 s of regeneration
                -- can exceed a legitimate glancing punch).
                local lastVitality, observedStatLoss = vitality(), 0
                local function observeStatLoss()
                    local current = vitality()
                    observedStatLoss = observedStatLoss + math.max(0, lastVitality - current)
                    lastVitality = current
                end
                local nextSample = 0
                local lastContactStep = -1
                local function observeAttack()
                    observeStatLoss()
                    if core.getSimulationTime() < nextSample then return end
                    nextSample = core.getSimulationTime() + 0.1
                    local s = ragdoll.getState(player)
                    local t = ragdoll.getState(victim)
                    local strikeSide = weapon and 'right' or s.strikeHand
                    local h = body(s, strikeSide .. '_hand').position
                    local shoulder = body(s, strikeSide .. '_upper_arm').position
                    local chest = body(t, 'spine2').position
                    local arm = s.arms and s.arms[strikeSide]
                    for _, window in ipairs(s.weapon and s.weapon.contactWindows or {}) do
                        local windowStep = tonumber(window.physicsStep)
                        if windowStep > lastContactStep then
                            lastContactStep = windowStep
                            if #window.contacts > 0 or #window.strikes > 0 then
                                print(string.format('PLAY04_CONTACT_WINDOW weapon=%s attempt=%d step=%s phase=%s attack=%s contacts=%d strikes=%d',
                                    label, attack, window.physicsStep, window.phase, window.attackId,
                                    #window.contacts, #window.strikes))
                            end
                        end
                    end
                    local tipText = 'nil'
                    if s.weapon and s.weapon.bodyIndex then
                        local wb = s.bodies[s.weapon.bodyIndex]
                        tipText = tostring(wb.position + (wb.rotation * util.vector3(0, 0, 1)) * (s.weapon.length * 0.5))
                    end
                    print(string.format('FIX-08 pose %s side=%s swing=%s tip=%s hand=%s shoulder=%s chest=%s reach=%s target=%s',
                        label, strikeSide, tostring(ragdoll.getSwingSpeed(player)), tipText, tostring(h), tostring(shoulder),
                        tostring(chest), tostring(arm and arm.reachError), tostring(arm and arm.targetWorld)))
                end
                -- Force (alpha 0.8): how far the struck body's chest moves
                -- from where it stood, until the attacker has withdrawn.
                local chestBefore = body(ragdoll.getState(victim), 'spine2').position
                local chestMove, spotMove = 0, 0
                local partsBefore = {}
                do
                    local vs = ragdoll.getState(victim)
                    local pelvis = body(vs, 'pelvis').position
                    for _, b in ipairs(vs.bodies) do partsBefore[b.name] = b.position - pelvis end
                end
                local function observePush()
                    local vs = ragdoll.getState(victim)
                    if not vs then return end
                    chestMove = math.max(chestMove, (body(vs, 'spine2').position - chestBefore):length())
                    -- The struck spot: the upper body part that moved most
                    -- against the pelvis.
                    local pelvis = body(vs, 'pelvis').position
                    for _, b in ipairs(vs.bodies) do
                        local before = partsBefore[b.name]
                        if before and not b.name:find('thigh') and not b.name:find('calf') and not b.name:find('foot') then
                            spotMove = math.max(spotMove, ((b.position - pelvis) - before):length())
                        end
                    end
                end
                local damageBeforeRelease = performContactInput(function() observeAttack(); observePush() end)
                waitSeconds(0.35, function() observeAttack(); observePush() end)
                -- Repeated impacts need actual withdrawal, not just a second
                -- click while the same blade contact is still pinned in place.
                waitFor('repeat-strike withdrawal', function()
                    observeAttack()
                    observePush()
                    return ragdoll.getSwingSpeed(player) == nil
                end, 4)
                print(string.format('FIX-08 push %s attack=%d chest_move=%.1f spot_move=%.1f', label, attack, chestMove, spotMove))
                -- A committed stroke carries through the release by design
                -- (roadmap 2b, 2026-09-28): its blow can land after the
                -- button is up while the swing is still live. A drag counts
                -- what landed until the swing withdrew, not settling contact
                -- after it.
                local damageAtWithdrawal = damageTotal()
                waitSeconds(0.2, observeStatLoss)
                observeStatLoss()
                local total = damageTotal()
                local state = ragdoll.getState(player)
                local w = state.weapon
                if w then
                    local item = types.Actor.getEquipment(player, types.Actor.EQUIPMENT_SLOT.CarriedRight)
                    print(string.format('FIX-08 damage inputs authored=%.4f fraction=%.4f strength=%.2f condition=%.2f defense=%s disposition=%s',
                        w.lastAuthoredDamage, w.lastImpactFraction,
                        types.Actor.stats.attributes.strength(player).modified, types.Item.itemData(item).condition,
                        tostring(w.lastDefenseOutcome), tostring(w.lastContactDisposition)))
                    local strike = w.lastStrikeEvent
                    if strike then
                        print(string.format('FIX-08 committed strike swing=%s normal_speed=%.6f speed=%.6f damage_events=%d',
                            tostring(strike.swingIdentity), strike.relativeNormalSpeed, strike.relativeSpeed,
                            w.damageEventCount))
                    end
                end
                print(string.format('FIX-08 %s attack=%d damage=%.4f observed_stat_loss=%.4f attached=%s contacts=%d strikes=%d ground=%d last_target=%s health=%.3f fatigue=%.3f',
                    label, attack, total - previous, observedStatLoss, tostring(w and w.attached),
                    w and w.contactEventCount or -1, w and w.strikeEventCount or -1,
                    w and w.groundContacts or -1,
                    tostring(w and w.lastContactEvent and w.lastContactEvent.targetActorIdentity),
                    types.Actor.stats.dynamic.health(victim).current,
                    types.Actor.stats.dynamic.fatigue(victim).current))
                -- Contact work: a press whose contact is only a graze or a
                -- light touch earns nothing, so a scripted attack need not
                -- injure. One that does must debit stats and leave a receipt.
                local recordedInjury = total > previous
                local injured = recordedInjury
                if loadout.drag then
                    injured = damageAtWithdrawal > previous
                    print(string.format('PLAY04_DRAG_CONTACT %s attempt=%d damage_before_release=%.4f until_withdrawal=%.4f',
                        label, attack, damageBeforeRelease - previous, damageAtWithdrawal - previous))
                end
                if injured then injuringAttacks = injuringAttacks + 1 end
                local receipt = ragdoll.getState(victim)
                if recordedInjury then
                    testing.expect(observedStatLoss > 0, 'hit ledger changed without applying damage to victim stats')
                    -- HIT-02 v2: separate regions/contact episodes may both injure.
                    -- Exact per-ID dedup is checked by the duplicate receipt case.
                    -- DMG-01 currently classifies fists, iron dagger and iron
                    -- longsword only. Other records still owe a paid physical
                    -- strike and stat loss; wound-mark coverage is separate.
                    if not weapon or weapon == 'iron dagger' or weapon == 'iron longsword' then
                        testing.expect(receipt.woundReceipts > receiptsBefore, 'native contact produced no wound receipt')
                        testing.expect(receipt.lastWoundKind ~= 'none', 'native contact lost its wound classification')
                    end
                    if weapon then
                        testing.expect(w.damageEventCount > damageEventsBefore,
                            'weapon injury did not come from a physical contact damage event')
                    end
                    if not weapon then
                        testing.expect(receipt.lastWoundKind == 'bruise', 'a punch was classified as an open wound')
                    end
                else
                    testing.expect(observedStatLoss == 0 or not weapon, 'victim lost health without a recorded injury')
                end
                if w and recordedInjury then
                    -- This fixture has 30 strength, a nearly new weapon and
                    -- no resistance: its ordinary equipment factor is ~0.8.
                    -- The range override must not leave a second skill roll
                    -- reducing a real contact to six percent of that damage.
                    testing.expect(w.lastFinalDamage > w.lastAuthoredDamage * 0.75,
                        'test-cell connection override still reduced damage by weapon skill')
                end
                previous = total
            end
            print(string.format('HIT02_INJURING_ATTACKS %s count=%d of 3', label, injuringAttacks))
            if injuringAttacks ~= 3 then
                contactFailures[#contactFailures + 1] = 'not all three player ' .. label .. ' attacks damaged the target'
            end
            -- A held input is not an automatic sequence of punches. These
            -- bodies can still move and make fresh contacts while settling;
            -- v2 intentionally permits those injuries. Exact stationary,
            -- duplicate and chatter controls are in ContactDamageTest.
            local presses = ragdoll.getState(player).playerPressCount
            ragdoll.injectMouseInput(0, 0, true)
            waitSeconds(0.45)
            local heldDamage = damageTotal()
            waitSeconds(1.0)
            print(string.format('HIT02_HELD_MOTION %s additional_damage=%.4f', label, damageTotal() - heldDamage))
            testing.expect(ragdoll.getState(player).playerPressCount == presses + 1, 'held input started extra attacks')
            ragdoll.injectMouseInput(0, 0, false)
            -- A motionless hold released is Morrowind's hold-and-release
            -- attack: let it start and finish before measuring rest.
            local releaseStep = ragdoll.getState(player).physicsStep
            local started = pcall(waitFor, 'released attack starts', function() return ragdoll.getSwingSpeed(player) ~= nil end, 0.3)
            local startStep = ragdoll.getState(player).physicsStep
            waitFor('physical withdrawal completed', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            local endStep = ragdoll.getState(player).physicsStep
            waitSeconds(0.5)
            local restDamage = damageTotal()
            local function restSample()
                local s, v = ragdoll.getState(player), ragdoll.getState(victim)
                if not (s and s.weapon and s.weapon.bodyIndex and v) then return end
                local wb = s.bodies[s.weapon.bodyIndex]
                local tip = wb.position + (wb.rotation * util.vector3(0, 0, 1)) * (s.weapon.length * 0.5)
                print(string.format('HIT02_REST_SAMPLE step=%s tip=%s grip=%s head=%s chest=%s vmode=%s',
                    tostring(s.physicsStep), tostring(tip), tostring(wb.position), tostring(body(v, 'head').position),
                    tostring(body(v, 'spine2').position), tostring(v.controller and v.controller.mode)))
            end
            print(string.format('HIT02_REST_TIMELINE %s release=%s started=%s start=%s end=%s rest=%s', label,
                tostring(releaseStep), tostring(started), tostring(startStep), tostring(endStep),
                tostring(ragdoll.getState(player).physicsStep)))
            waitSeconds(0.5, restSample)
            if weapon and damageTotal() ~= restDamage then
                local strike = ragdoll.getState(player).weapon.lastStrikeEvent
                print(string.format('HIT02_AFTER_RELEASE damage=%.4f approach=%.6f speed=%.6f strike_step=%s swing=%s',
                    damageTotal() - restDamage, strike.relativeNormalSpeed, strike.relativeSpeed,
                    tostring(strike.physicsStep), tostring(ragdoll.getSwingSpeed(player))))
            end
            if damageTotal() ~= restDamage then
                contactFailures[#contactFailures + 1] = 'resting contact caused damage'
            end
            victim:teleport(testArea.cell, player.position + util.vector3(0, 200, 0), { onGround = true })
            waitSeconds(0.25)
            local missDamage = damageTotal()
            local missBaseline = missDamage
            local function observeMiss()
                local total = damageTotal()
                if total ~= missDamage then
                    local p, v = ragdoll.getState(player), ragdoll.getState(victim)
                    local w = p.weapon
                    print(string.format('PLAY04_MISS_INJURY weapon=%s damage=%.4f phase=%s step=%s hand=%s chest=%s player=%s victim=%s contacts=%d damage_events=%d',
                        label, total - missDamage, p.attackPhase, tostring(p.physicsStep),
                        tostring(body(p, 'right_hand').position), tostring(body(v, 'spine2').position),
                        tostring(player.position), tostring(victim.position),
                        w and w.otherRagdollContacts or -1, w and w.damageEventCount or -1))
                    for _, window in ipairs(w and w.contactWindows or {}) do
                        if #window.contacts > 0 or #window.strikes > 0 then
                            print(string.format('PLAY04_MISS_WINDOW step=%s phase=%s attack=%s contacts=%d strikes=%d',
                                window.physicsStep, window.phase, window.attackId, #window.contacts, #window.strikes))
                        end
                    end
                    missDamage = total
                end
            end
            print(string.format('PLAY04_MISS_SETUP weapon=%s damage_since_rest=%.4f hand=%s chest=%s',
                label, missDamage - restDamage, tostring(body(ragdoll.getState(player), 'right_hand').position),
                tostring(body(ragdoll.getState(victim), 'spine2').position)))
            performContactInput(observeMiss)
            waitSeconds(0.35, observeMiss)
            waitFor('miss attack withdrawal', function()
                observeMiss()
                return ragdoll.getSwingSpeed(player) == nil
            end, 4)
            observeMiss()
            if damageTotal() ~= missBaseline then
                contactFailures[#contactFailures + 1] = 'out-of-reach attack caused damage'
            end
            testing.expect(#contactFailures == 0, table.concat(contactFailures, '; '))
        end)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.reactionScale(savedReaction)
        ragdoll.injectMouseInput(0, 0, false, 2)
        ragdoll.handDamageScale(savedHandScale)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if victim and victim:isValid() then victim:remove() end
        if not ok then error(message, 0) end
    end)
end

-- STILLS-01 (handoff check, 2026-10-06: the user asked for stills before each
-- build): the player with a weapon, photographed from the side and the front
-- with a static camera - standing in guard, mid-cut, after the cut, after a
-- short run and its stop. The screenshots land in the run's userdata
-- screenshots folder in order; STILLS01 lines name them.
-- (10-30: the spear, staff, dagger and fists too - "holding it right now
-- clips the bottom of the weapon through the body".)
for index, weapon in ipairs({ 'iron warhammer', 'iron longsword', 'iron spear', 'wooden staff', 'iron dagger', 'fists' }) do
    registerTest('STILLS-01 stills with ' .. weapon, index <= 2 and 918330 + index or 919960 + index, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            local armed = weapon ~= 'fists'
            if armed and not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = armed and weapon or nil, sequence = 430 + index,
                firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 430 + index end)
            waitSeconds(1)
            powerAndStand(player, 'STILLS-01 player')
            ragdoll.prepareMouseInput()
            local shotIndex = 0
            local function shots(tag)
                for _, view in ipairs({ { 'side', util.vector3(-260, 0, 60), math.pi / 2 },
                    { 'front', util.vector3(0, 260, 60), math.pi } }) do
                    player:sendEvent('fixWoundCamera', { position = player.position + view[2], yaw = view[3] })
                    coroutine.yield()
                    coroutine.yield()
                    coroutine.yield()
                    shotIndex = shotIndex + 1
                    print(string.format('STILLS01 shot=%d weapon=%s moment=%s view=%s', shotIndex, weapon, tag, view[1]))
                    ragdoll.captureScreenshot()
                    waitSeconds(0.1)
                end
            end
            waitSeconds(1.5)
            shots('guard')
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitSeconds(0.25)
            local function drag(x, seconds, atHalf)
                local started, sent, took = core.getSimulationTime(), 0, false
                waitSeconds(seconds, function()
                    local f = math.min(1, (core.getSimulationTime() - started) / seconds)
                    local v = math.floor(x * f)
                    ragdoll.injectMouseInput(v - sent, 0)
                    sent = v
                end)
                ragdoll.injectMouseInput(x - sent, 0)
            end
            drag(HSC_WIND, 0.4)
            shots('wound up')
            drag(-1200, 0.1)
            waitSeconds(0.05)
            shots('mid cut')
            ragdoll.injectMouseInput(0, 0, false, 1)
            waitSeconds(1.5)
            shots('after cut')
            player:sendEvent('fixPlayerMove', 1)
            waitSeconds(2.0)
            shots('walking')
            player:sendEvent('fixPlayerMove', 0)
            waitSeconds(2.5)
            shots('stopped')
            -- The low stance (aim item 2): the look on the floor ahead, then a
            -- forehand drag (the stills' static camera leaves the cut the
            -- look's pitch).
            player:sendEvent('fixWoundCamera', {})
            waitSeconds(0.3)
            contactAimReady = false
            player:sendEvent('fixAimAt', player.position + util.vector3(0, 110, 5))
            pcall(waitFor, 'STILLS-01 look down', function() return contactAimReady end, 4)
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitSeconds(0.25)
            drag(HSC_WIND, 0.4)
            shots('low wound up')
            drag(-1200, 0.1)
            waitSeconds(0.05)
            shots('low mid cut')
            ragdoll.injectMouseInput(0, 0, false, 1)
            waitSeconds(1.5)
            shots('after low cut')
        end)
        player:sendEvent('fixPlayerMove', 0)
        player:sendEvent('fixWoundCamera', {})
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end)
end

-- WOUNDFX-05 (the elbow-bend look, 10-08; needs OPENMW_RAGDOLL_WOUND_FX>=1):
-- a cut on the player's own bare right forearm, photographed close in the
-- guard, wound up (the elbow folded by the wheel) and after the cut - the
-- wound must stay on the skin as the arm bends.
registerTest('WOUNDFX-05 a forearm cut through a wind-up', 918335, function()
    local weapon = 'iron longsword'
    local player = placePlayer()
    local ok, message = pcall(function()
        if not types.Actor.inventory(player):find(weapon) then
            world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 497, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 497 end)
        waitSeconds(1)
        powerAndStand(player, 'WOUNDFX-05 player')
        ragdoll.prepareMouseInput()
        waitSeconds(1.5)
        local state = ragdoll.getState(player)
        local forearm = body(state, 'right_forearm')
        local chest = body(state, 'spine2')
        local side = forearm.centerOfMass - chest.centerOfMass
        local normal = util.vector3(side.x, side.y, 0):normalize()
        local axis = (body(state, 'right_hand').position - forearm.position):normalize()
        local across = normal:cross(axis):normalize()
        local direction = (axis * 0.6 + across * 0.8):normalize()
        local point = forearm.centerOfMass + axis * 4 + normal * 5
        local before = ragdoll.getWoundFxAdded()
        ragdoll.addWoundFx(player, 'right_forearm', point, normal, direction, 6, 0.55, 0.7)
        waitSeconds(0.2)
        print(string.format('WOUNDFX05 placed took=%s', tostring(ragdoll.getWoundFxAdded() == before + 1)))
        local inverse = forearm.rotation:inverse()
        local localPoint = inverse * (forearm.centerOfMass + axis * 4 + normal * 2.5 - forearm.position)
        local localNormal = inverse * normal
        local shotIndex = 0
        local function shots(tag)
            local arm = body(ragdoll.getState(player), 'right_forearm')
            local at = arm.position + arm.rotation * localPoint
            local out = (arm.rotation * localNormal):normalize()
            for _, view in ipairs({ { 'close', 30, 0 }, { 'oblique', 42, 0.6 } }) do
                local swing = view[3]
                local dir = util.vector3(out.x * math.cos(swing) - out.y * math.sin(swing),
                    out.x * math.sin(swing) + out.y * math.cos(swing), out.z)
                local position = at + dir * view[2]
                local look = at - position
                local lookFlat = math.sqrt(look.x * look.x + look.y * look.y)
                local viewYaw = lookFlat > 0.05 and math.atan2(look.x, look.y) or 0
                local pitch = math.max(-1.45, math.min(1.45, math.atan2(-look.z, math.max(lookFlat, 0.001))))
                player:sendEvent('fixWoundCamera', { position = position, yaw = viewYaw, pitch = pitch })
                coroutine.yield()
                coroutine.yield()
                coroutine.yield()
                shotIndex = shotIndex + 1
                print(string.format('WOUNDFX05 shot=%d moment=%s view=%s', shotIndex, tag, view[1]))
                ragdoll.captureScreenshot()
                waitSeconds(0.05)
            end
        end
        shots('guard')
        local function drag(x, y, seconds)
            local started, sentX, sentY = core.getSimulationTime(), 0, 0
            waitSeconds(seconds, function()
                local f = math.min(1, (core.getSimulationTime() - started) / seconds)
                local vx, vy = math.floor(x * f), math.floor(y * f)
                ragdoll.injectMouseInput(vx - sentX, vy - sentY)
                sentX, sentY = vx, vy
            end)
            ragdoll.injectMouseInput(x - sentX, y - sentY)
        end
        ragdoll.injectMouseInput(0, 0, true, 1)
        waitSeconds(0.25)
        drag(0, -HSC_WIND, 0.4)
        shots('wound up over the head')
        ragdoll.injectMouseInput(0, 0, false, 1)
        waitSeconds(1.0)
        ragdoll.injectMouseInput(0, 0, true, 1)
        waitSeconds(0.25)
        drag(HSC_WIND, 0, 0.4)
        shots('wound up to the side')
        ragdoll.injectMouseInput(0, 0, false, 1)
        waitSeconds(1.5)
        shots('after')
    end)
    player:sendEvent('fixWoundCamera', {})
    ragdoll.injectMouseInput(0, 0, false, 1)
    ragdoll.setPowered(player, false)
    if not ok then error(message, 0) end
end)

-- WOUNDFX-01 (FX1 look check, needs OPENMW_RAGDOLL_WOUND_FX=1): one shader
-- cut on a bare right forearm, photographed close up while the elbow bends
-- through standing, guard, wind-up and lying. The cut is pinned in the
-- forearm body's frame, so each camera finds it again on the skin. Screenshots
-- land in order; WOUNDFX01 lines name them.
registerTest('WOUNDFX-01 a cut on a bare forearm', 918338, function()
    placePlayer()
    local victim = spawnDummy('WOUNDFX-01 dummy', util.vector3(0, 120, 0), false)
    local ok, message = pcall(function()
        victim:sendEvent('fixWoundMode', { mode = 'normal' }) -- strips the clothes: bare skin
        waitSeconds(0.5)
        powerAndStand(victim, 'WOUNDFX-01 dummy')
        waitSeconds(2) -- settle, and let the mesh be culled and skinned
        local state = ragdoll.getState(victim)
        local forearm = body(state, 'right_forearm')
        local chest = body(state, 'spine2')
        -- The outer side of the hanging forearm, a diagonal stroke across it.
        local side = forearm.centerOfMass - chest.centerOfMass
        local normal = util.vector3(side.x, side.y, 0):normalize()
        local axis = (body(state, 'right_hand').position - forearm.position):normalize()
        local across = normal:cross(axis):normalize()
        local direction = (axis * 0.6 + across * 0.8):normalize()
        -- Toward the wrist, clear of the elbow seam (first run: half the cut
        -- sat under the upper arm's mesh).
        local point = forearm.centerOfMass + axis * 6 + normal * 5
        local before = ragdoll.getWoundFxAdded()
        ragdoll.addWoundFx(victim, 'right_forearm', point, normal, direction, 6, 0.55, 0.7)
        waitSeconds(0.2)
        testing.expect(ragdoll.getWoundFxAdded() == before + 1,
            'no shader cut on the bare forearm (OPENMW_RAGDOLL_WOUND_FX=1 set? sleeves on?)')
        -- The cut's spot and outward normal in the forearm body's frame.
        local inverse = forearm.rotation:inverse()
        local localPoint = inverse * (forearm.centerOfMass + axis * 6 + normal * 2.5 - forearm.position)
        local localNormal = inverse * normal
        local shotIndex = 0
        local function shots(tag)
            local current = ragdoll.getState(victim)
            local arm = body(current, 'right_forearm')
            local at = arm.position + arm.rotation * localPoint
            local out = (arm.rotation * localNormal):normalize()
            local flat = math.sqrt(out.x * out.x + out.y * out.y)
            local yaw = flat > 0.05 and math.atan2(-out.x, -out.y) or 0
            for _, view in ipairs({ { 'close', 28, 0 }, { 'near', 45, 0 }, { 'oblique', 40, 0.6 } }) do
                -- The oblique view swings round the arm so the lips catch the light.
                local swing = view[3]
                local dir = util.vector3(out.x * math.cos(swing) - out.y * math.sin(swing),
                    out.x * math.sin(swing) + out.y * math.cos(swing), out.z)
                local position = at + dir * view[2]
                local look = at - position
                local lookFlat = math.sqrt(look.x * look.x + look.y * look.y)
                local viewYaw = lookFlat > 0.05 and math.atan2(look.x, look.y) or yaw
                local pitch = math.max(-1.45, math.min(1.45, math.atan2(-look.z, math.max(lookFlat, 0.001))))
                world.players[1]:sendEvent('fixWoundCamera', { position = position, yaw = viewYaw, pitch = pitch })
                coroutine.yield()
                coroutine.yield()
                coroutine.yield()
                shotIndex = shotIndex + 1
                print(string.format('WOUNDFX01 shot=%d moment=%s view=%s at=%s normal=%s', shotIndex, tag, view[1],
                    tostring(at), tostring(out)))
                ragdoll.captureScreenshot()
                waitSeconds(0.1)
            end
        end
        shots('standing')
        -- Guard: the right hand in front of the chest, the elbow bent.
        local forward = util.vector3(0, 1, 0)
        local up = util.vector3(0, 0, 1)
        local right = util.vector3(1, 0, 0)
        ragdoll.setArmTarget(victim, true, chest.centerOfMass + forward * 28 + up * 8, 1, 6)
        waitSeconds(1.2)
        shots('guard')
        -- Wind-up: the hand up beside the head, the elbow folded.
        local head = body(ragdoll.getState(victim), 'head').centerOfMass
        ragdoll.setArmTarget(victim, true, head + up * 22 + right * 10 - forward * 6, 1, 6)
        waitSeconds(1.2)
        shots('wind up')
        ragdoll.releaseArmTarget(victim, true)
        waitSeconds(0.5)
        -- Lying: knocked onto the back, no get-up.
        ragdoll.setAutomaticRecovery(false)
        local standing = ragdoll.getState(victim)
        ragdoll.applyCombatImpulse(victim, util.vector3(0, -1, 0) * 6500, body(standing, 'spine2').centerOfMass, 7338)
        waitSeconds(3.0)
        shots('lying')
        print(string.format('WOUNDFX01 done shots=%d added=%d', shotIndex, ragdoll.getWoundFxAdded() - before))
    end)
    world.players[1]:sendEvent('fixWoundCamera', {})
    ragdoll.setAutomaticRecovery(true)
    if victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)

-- WOUNDFX-02 (FX2 look check, needs OPENMW_RAGDOLL_WOUND_FX=1, or 2 for the
-- bruise): one shader wound of each kind on the stripped dummy's bare skin - a
-- slash on the right forearm, a stab in the chest, a chop on the right thigh
-- (severe enough for the bone hint), a bruise on the left forearm (at level 1
-- it is skipped: bruises keep the mesh-copy marks) - each photographed close
-- up fresh, and again three minutes later (the bruise turns from red to
-- purple, the blood dries). Each wound is pinned in its body's frame, so the
-- camera finds it again. Screenshots land in order; WOUNDFX02 lines name them.
registerTest('WOUNDFX-02 one wound of each kind on bare skin', 918339, function()
    placePlayer()
    local victim = spawnDummy('WOUNDFX-02 dummy', util.vector3(0, 120, 0), false)
    local ok, message = pcall(function()
        victim:sendEvent('fixWoundMode', { mode = 'normal' }) -- strips the clothes: bare skin
        waitSeconds(0.5)
        powerAndStand(victim, 'WOUNDFX-02 dummy')
        waitSeconds(2) -- settle, and let the meshes be culled and skinned
        local state = ragdoll.getState(victim)
        local chest = body(state, 'spine2')
        local up = util.vector3(0, 0, 1)
        local shoulders = body(state, 'right_upper_arm').position - body(state, 'left_upper_arm').position
        local lateral = util.vector3(shoulders.x, shoulders.y, 0):normalize() -- toward the dummy's right
        local forward = up:cross(lateral):normalize()
        -- A limb's outer side (away from the chest) and its axis down the limb.
        local function limb(name, toward)
            local part = body(state, name)
            local side = part.centerOfMass - chest.centerOfMass
            return part, util.vector3(side.x, side.y, 0):normalize(),
                (body(state, toward).position - part.position):normalize()
        end
        local wounds = {}
        do -- slash: WOUNDFX-01's cut, a diagonal stroke across the forearm
            local part, normal, axis = limb('right_forearm', 'right_hand')
            local across = normal:cross(axis):normalize()
            wounds[#wounds + 1] = { kind = 'slash', region = 'right_forearm', body = part.name,
                center = part.centerOfMass + axis * 6, normal = normal, out = 5, skin = 2.5,
                direction = (axis * 0.6 + across * 0.8):normalize(), length = 6, halfWidth = 0.55, severity = 0.6 }
        end
        do -- stab: left of the breastbone, the blade's edges a little off vertical
            wounds[#wounds + 1] = { kind = 'stab', region = 'chest', body = chest.name,
                center = chest.centerOfMass - lateral * 4 + up * 2, normal = forward, out = 8, skin = 7,
                direction = (up * 0.8 + lateral * 0.6):normalize(), length = 2.4, halfWidth = 0.35,
                severity = 0.7 }
        end
        do -- chop: the outer front of the thigh, across it
            local part = body(state, 'right_thigh')
            local axis = (body(state, 'right_calf').position - part.position):normalize()
            local normal = (forward + lateral * 0.6)
            normal = (normal - axis * normal:dot(axis)):normalize()
            wounds[#wounds + 1] = { kind = 'chop', region = 'right_thigh', body = part.name,
                center = part.centerOfMass, normal = normal, out = 6, skin = 4.5,
                direction = normal:cross(axis):normalize(), length = 8, halfWidth = 1.2, severity = 0.95 }
        end
        if ragdoll.getWoundFxLevel() >= 2 then -- bruise: the outer side of the left forearm
            -- Round the vanilla forearm/wrist seam: the blotch spills over both meshes.
            local part, normal, axis = limb('left_forearm', 'left_hand')
            wounds[#wounds + 1] = { kind = 'bruise', region = 'left_forearm', body = part.name,
                center = part.centerOfMass + axis * 4, normal = normal, out = 5, skin = 2.5,
                direction = axis, length = 1.0, halfWidth = 1.8, severity = 0.7 }
        else
            print('WOUNDFX02 bruise skipped: OPENMW_RAGDOLL_WOUND_FX=' .. ragdoll.getWoundFxLevel())
        end
        local before = ragdoll.getWoundFxAdded()
        for _, wound in ipairs(wounds) do
            local added = ragdoll.getWoundFxAdded()
            ragdoll.addWoundFx(victim, wound.region, wound.center + wound.normal * wound.out, wound.normal,
                wound.direction, wound.length, wound.halfWidth, wound.severity, wound.kind)
            waitSeconds(0.2)
            wound.took = ragdoll.getWoundFxAdded() == added + 1
            print(string.format('WOUNDFX02 placed kind=%s region=%s took=%s', wound.kind, wound.region,
                tostring(wound.took)))
            testing.expect(wound.took, 'no shader ' .. wound.kind .. ' on the bare ' .. wound.region
                .. ' (OPENMW_RAGDOLL_WOUND_FX=1 set? clothes on?)')
            -- The wound's spot on the skin and its outward normal in its body's
            -- frame (the snapshot the wound was aimed from).
            local part = body(state, wound.body)
            local inverse = part.rotation:inverse()
            wound.localPoint = inverse * (wound.center + wound.normal * wound.skin - part.position)
            wound.localNormal = inverse * wound.normal
        end
        local shotIndex = 0
        local function shots(tag)
            local current = ragdoll.getState(victim)
            for _, wound in ipairs(wounds) do
                local part = body(current, wound.body)
                local at = part.position + part.rotation * wound.localPoint
                local out = (part.rotation * wound.localNormal):normalize()
                local flat = math.sqrt(out.x * out.x + out.y * out.y)
                local yaw = flat > 0.05 and math.atan2(-out.x, -out.y) or 0
                for _, view in ipairs({ { 'close', 24, 0 }, { 'near', 40, 0 }, { 'oblique', 34, 0.6 } }) do
                    -- The oblique view swings round the wound so the relief catches the light.
                    local swing = view[3]
                    local dir = util.vector3(out.x * math.cos(swing) - out.y * math.sin(swing),
                        out.x * math.sin(swing) + out.y * math.cos(swing), out.z)
                    local position = at + dir * view[2]
                    local look = at - position
                    local lookFlat = math.sqrt(look.x * look.x + look.y * look.y)
                    local viewYaw = lookFlat > 0.05 and math.atan2(look.x, look.y) or yaw
                    local pitch = math.max(-1.45, math.min(1.45, math.atan2(-look.z, math.max(lookFlat, 0.001))))
                    world.players[1]:sendEvent('fixWoundCamera', { position = position, yaw = viewYaw, pitch = pitch })
                    coroutine.yield()
                    coroutine.yield()
                    coroutine.yield()
                    shotIndex = shotIndex + 1
                    print(string.format('WOUNDFX02 shot=%d moment=%s kind=%s region=%s view=%s at=%s normal=%s',
                        shotIndex, tag, wound.kind, wound.region, view[1], tostring(at), tostring(out)))
                    ragdoll.captureScreenshot()
                    waitSeconds(0.1)
                end
            end
        end
        shots('fresh')
        -- Three minutes of simulation time: the bruise's red has turned
        -- purple and the cuts' blood has dried darker.
        world.players[1]:sendEvent('fixWoundCamera', {})
        waitSeconds(180)
        shots('aged')
        print(string.format('WOUNDFX02 done shots=%d added=%d', shotIndex, ragdoll.getWoundFxAdded() - before))
    end)
    world.players[1]:sendEvent('fixWoundCamera', {})
    if victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)

-- WOUNDFX-03 (FX3 look check, needs OPENMW_RAGDOLL_WOUND_FX=3): wounds on
-- clothing and armour. One dummy in a common shirt and pants takes a slash
-- and a stab on the chest and a chop on the right thigh: each a blood stain
-- soaking out through the cloth with a slit along the cut. Another in an iron
-- cuirass and greaves takes a slash, a blunt blow and a stab on the cuirass
-- (a bright scored line with a dent beside it, a dished dent, a small dent
-- with a bright pit) and a chop on the greaves' shin: no blood. Each is
-- photographed close up fresh and again three minutes later (the stains dry
-- darker brown and matte; the scratches stay). A piece the test cannot equip
-- is skipped with a printed reason; each placed wound must be drawn as its
-- surface's kind (getWoundFxLastKind). Screenshots land in order; WOUNDFX03
-- lines name them.
registerTest('WOUNDFX-03 stains on clothing, scratches on armour', 918337, function()
    placePlayer()
    local victims = {}
    local ok, message = pcall(function()
        if ragdoll.getWoundFxLevel() < 3 then
            print('WOUNDFX03 skipped: OPENMW_RAGDOLL_WOUND_FX=' .. ragdoll.getWoundFxLevel() .. ', needs 3')
            testing.expect(false, 'WOUNDFX-03 needs OPENMW_RAGDOLL_WOUND_FX=3')
            return
        end
        -- Dress a dummy in exactly these pieces ({ slot name, record id });
        -- returns which slots hold their piece.
        local function dress(victim, label, pieces)
            local wear = {}
            for _, piece in ipairs(pieces) do
                local made = pcall(function()
                    world.createObject(piece[2], 1):moveInto(types.Actor.inventory(victim))
                end)
                if made then
                    wear[piece[1]] = piece[2]
                else
                    print(string.format('WOUNDFX03 %s: no record %s, %s skipped', label, piece[2], piece[1]))
                end
            end
            coroutine.yield()
            victim:sendEvent('fixActorWear', wear)
            waitSeconds(0.5)
            local worn = {}
            for _, piece in ipairs(pieces) do
                local item = types.Actor.getEquipment(victim, types.Actor.EQUIPMENT_SLOT[piece[1]])
                worn[piece[1]] = item ~= nil and item.recordId == piece[2]
                if wear[piece[1]] and not worn[piece[1]] then
                    print(string.format('WOUNDFX03 %s: could not equip %s in %s, skipped', label, piece[2], piece[1]))
                end
            end
            return worn
        end
        local clothed = spawnDummy('WOUNDFX-03 clothed dummy', util.vector3(-45, 120, 0), false)
        victims[#victims + 1] = clothed
        local armoured = spawnDummy('WOUNDFX-03 armoured dummy', util.vector3(45, 120, 0), false)
        victims[#victims + 1] = armoured
        local clothes = dress(clothed, 'clothed dummy',
            { { 'Shirt', 'common_shirt_01' }, { 'Pants', 'common_pants_01' } })
        local armour = dress(armoured, 'armoured dummy',
            { { 'Cuirass', 'iron_cuirass' }, { 'Greaves', 'iron_greaves' } })
        powerAndStand(clothed, 'WOUNDFX-03 clothed dummy')
        powerAndStand(armoured, 'WOUNDFX-03 armoured dummy')
        waitSeconds(2) -- settle, and let the new parts be culled and skinned
        local up = util.vector3(0, 0, 1)
        local wounds = {}
        -- One dummy's frame: its chest, toward its right, its front.
        local function frame(victim)
            local state = ragdoll.getState(victim)
            local shoulders = body(state, 'right_upper_arm').position - body(state, 'left_upper_arm').position
            local lateral = util.vector3(shoulders.x, shoulders.y, 0):normalize()
            return state, body(state, 'spine2'), lateral, up:cross(lateral):normalize()
        end
        do
            local state, chest, lateral, forward = frame(clothed)
            if clothes.Shirt then
                -- A slash down across the right breast, a stab left of the breastbone.
                wounds[#wounds + 1] = { victim = clothed, state = state, expect = 'stain', piece = 'shirt',
                    kind = 'slash', region = 'chest', body = chest.name,
                    center = chest.centerOfMass + lateral * 3 + up * 2, normal = forward, out = 9, skin = 7.5,
                    direction = (lateral * 0.8 - up * 0.6):normalize(), length = 7, halfWidth = 0.55,
                    severity = 0.65 }
                wounds[#wounds + 1] = { victim = clothed, state = state, expect = 'stain', piece = 'shirt',
                    kind = 'stab', region = 'chest', body = chest.name,
                    center = chest.centerOfMass - lateral * 4 - up * 2, normal = forward, out = 9, skin = 7.5,
                    direction = (up * 0.8 + lateral * 0.6):normalize(), length = 2.4, halfWidth = 0.35,
                    severity = 0.7 }
            end
            if clothes.Pants then
                -- WOUNDFX-02's chop across the outer front of the thigh.
                local part = body(state, 'right_thigh')
                local axis = (body(state, 'right_calf').position - part.position):normalize()
                local normal = (forward + lateral * 0.6)
                normal = (normal - axis * normal:dot(axis)):normalize()
                wounds[#wounds + 1] = { victim = clothed, state = state, expect = 'stain', piece = 'pants',
                    kind = 'chop', region = 'right_thigh', body = part.name,
                    center = part.centerOfMass, normal = normal, out = 7, skin = 5,
                    direction = normal:cross(axis):normalize(), length = 8, halfWidth = 1.2, severity = 0.95 }
            end
        end
        do
            local state, chest, lateral, forward = frame(armoured)
            if armour.Cuirass then
                -- A slash across the right breastplate, a blunt blow left of
                -- the middle, a stab low on the front.
                wounds[#wounds + 1] = { victim = armoured, state = state, expect = 'scratch', piece = 'cuirass',
                    kind = 'slash', region = 'chest', body = chest.name,
                    center = chest.centerOfMass + lateral * 3 + up * 2, normal = forward, out = 11, skin = 9,
                    direction = (lateral * 0.8 - up * 0.6):normalize(), length = 7, halfWidth = 0.55,
                    severity = 0.65 }
                wounds[#wounds + 1] = { victim = armoured, state = state, expect = 'scratch', piece = 'cuirass',
                    kind = 'blunt', region = 'chest', body = chest.name,
                    center = chest.centerOfMass - lateral * 4 + up * 1, normal = forward, out = 11, skin = 9,
                    direction = up, length = 1.0, halfWidth = 1.8, severity = 0.7 }
                wounds[#wounds + 1] = { victim = armoured, state = state, expect = 'scratch', piece = 'cuirass',
                    kind = 'stab', region = 'chest', body = chest.name,
                    center = chest.centerOfMass + lateral * 1 - up * 4, normal = forward, out = 11, skin = 9,
                    direction = (up * 0.8 + lateral * 0.6):normalize(), length = 2.4, halfWidth = 0.35,
                    severity = 0.7 }
            end
            if armour.Greaves then
                -- A chop across the shin (the greaves surely cover the calf).
                local part = body(state, 'right_calf')
                local axis = (body(state, 'right_foot').position - part.position):normalize()
                local normal = (forward + lateral * 0.3)
                normal = (normal - axis * normal:dot(axis)):normalize()
                wounds[#wounds + 1] = { victim = armoured, state = state, expect = 'scratch', piece = 'greaves',
                    kind = 'chop', region = 'right_calf', body = part.name,
                    center = part.centerOfMass, normal = normal, out = 6, skin = 4.5,
                    direction = normal:cross(axis):normalize(), length = 7, halfWidth = 1.0, severity = 0.9 }
            end
        end
        testing.expect(#wounds > 0, 'WOUNDFX-03 could dress neither dummy')
        local before = ragdoll.getWoundFxAdded()
        local placed = {}
        local failures = {} -- checked after the shots, so a miss still leaves pictures
        for _, wound in ipairs(wounds) do
            local added = ragdoll.getWoundFxAdded()
            ragdoll.addWoundFx(wound.victim, wound.region, wound.center + wound.normal * wound.out, wound.normal,
                wound.direction, wound.length, wound.halfWidth, wound.severity, wound.kind)
            waitSeconds(0.2)
            local drawn = ragdoll.getWoundFxLastKind()
            wound.took = ragdoll.getWoundFxAdded() == added + 1
            print(string.format('WOUNDFX03 placed kind=%s on=%s region=%s took=%s drawn=%s', wound.kind, wound.piece,
                wound.region, tostring(wound.took), drawn))
            if not (wound.took and drawn == wound.expect) then
                failures[#failures + 1] = string.format('the %s on the %s was drawn as %s, not a %s', wound.kind,
                    wound.piece, drawn, wound.expect)
            end
            if wound.took then
                -- The wound's spot on the surface and its outward normal in its
                -- body's frame (the snapshot the wound was aimed from).
                local part = body(wound.state, wound.body)
                local inverse = part.rotation:inverse()
                wound.localPoint = inverse * (wound.center + wound.normal * wound.skin - part.position)
                wound.localNormal = inverse * wound.normal
                placed[#placed + 1] = wound
            end
        end
        local shotIndex = 0
        local function shots(tag)
            for _, wound in ipairs(placed) do
                local part = body(ragdoll.getState(wound.victim), wound.body)
                local at = part.position + part.rotation * wound.localPoint
                local out = (part.rotation * wound.localNormal):normalize()
                local flat = math.sqrt(out.x * out.x + out.y * out.y)
                local yaw = flat > 0.05 and math.atan2(-out.x, -out.y) or 0
                for _, view in ipairs({ { 'close', 24, 0 }, { 'near', 40, 0 }, { 'oblique', 34, 0.6 } }) do
                    -- The oblique view swings round the wound so the relief
                    -- and the wet or metal highlight catch the light.
                    local swing = view[3]
                    local dir = util.vector3(out.x * math.cos(swing) - out.y * math.sin(swing),
                        out.x * math.sin(swing) + out.y * math.cos(swing), out.z)
                    local position = at + dir * view[2]
                    local look = at - position
                    local lookFlat = math.sqrt(look.x * look.x + look.y * look.y)
                    local viewYaw = lookFlat > 0.05 and math.atan2(look.x, look.y) or yaw
                    local pitch = math.max(-1.45, math.min(1.45, math.atan2(-look.z, math.max(lookFlat, 0.001))))
                    world.players[1]:sendEvent('fixWoundCamera', { position = position, yaw = viewYaw, pitch = pitch })
                    coroutine.yield()
                    coroutine.yield()
                    coroutine.yield()
                    shotIndex = shotIndex + 1
                    print(string.format('WOUNDFX03 shot=%d moment=%s kind=%s on=%s region=%s view=%s at=%s normal=%s',
                        shotIndex, tag, wound.kind, wound.piece, wound.region, view[1], tostring(at), tostring(out)))
                    ragdoll.captureScreenshot()
                    waitSeconds(0.1)
                end
            end
        end
        shots('fresh')
        -- Three minutes of simulation time: the stains have dried darker
        -- brown and matte; the scratches and dents are unchanged.
        world.players[1]:sendEvent('fixWoundCamera', {})
        waitSeconds(180)
        shots('aged')
        print(string.format('WOUNDFX03 done shots=%d added=%d', shotIndex, ragdoll.getWoundFxAdded() - before))
        testing.expect(#failures == 0, table.concat(failures, '; '))
    end)
    world.players[1]:sendEvent('fixWoundCamera', {})
    for _, victim in ipairs(victims) do
        if victim:isValid() then victim:remove() end
    end
    if not ok then error(message, 0) end
end)

-- WOUNDFX-06 (user 10-31, the release test: "I dont see the wounds" - the
-- foe was a Hlaalu guard, bonemold from helm to boots, and the log said
-- "WOUNDFX01 no shader recipe for Tri A_Bonemold_Bracer_W 0"): a stock
-- Hlaalu guard, powered and standing, takes a slash on each armoured region
-- (head, chest, pelvis, upper arm, forearm, thigh). WOUNDFX06 lines: per
-- region whether the wound was taken and how it was drawn
-- (getWoundFxLastKind), then a close shot of each.
registerTest('WOUNDFX-06 wounds on a Hlaalu guard in bonemold', 918339, function()
    placePlayer()
    local npc
    local ok, message = pcall(function()
        local record = world.createRecord(types.NPC.createRecordDraft({
            template = types.NPC.record('hlaalu guard'), name = 'WOUNDFX-06 guard',
            isEssential = false, isPersistent = false, isRespawning = false,
        }))
        npc = world.createObject(record.id)
        npc:teleport(testArea.cell, testArea.player + util.vector3(0, 120, 0),
            { onGround = true, rotation = util.transform.rotateZ(math.pi) })
        npc:addScript('stance.lua')
        coroutine.yield()
        npc:sendEvent('fixActorHealth', 5000)
        powerAndStand(npc, 'WOUNDFX-06 guard')
        waitSeconds(2)
        local worn = {}
        for slotName, slot in pairs(types.Actor.EQUIPMENT_SLOT) do
            local item = types.Actor.getEquipment(npc, slot)
            if item then worn[#worn + 1] = slotName .. '=' .. item.recordId end
        end
        table.sort(worn)
        print('WOUNDFX06 worn ' .. table.concat(worn, ' '))
        local up = util.vector3(0, 0, 1)
        local state = ragdoll.getState(npc)
        local shoulders = body(state, 'right_upper_arm').position - body(state, 'left_upper_arm').position
        local lateral = util.vector3(shoulders.x, shoulders.y, 0):normalize()
        local forward = up:cross(lateral):normalize()
        local targets = {
            { region = 'head', body = 'head' }, { region = 'chest', body = 'spine2' },
            { region = 'pelvis', body = 'pelvis' }, { region = 'right_upper_arm', body = 'right_upper_arm' },
            { region = 'right_forearm', body = 'right_forearm' }, { region = 'right_thigh', body = 'right_thigh' },
        }
        local placed = {}
        for _, t in ipairs(targets) do
            local part = body(state, t.body)
            local normal = (forward + lateral * 0.3):normalize()
            local added = ragdoll.getWoundFxAdded()
            ragdoll.addWoundFx(npc, t.region, part.centerOfMass + normal * 14, normal,
                (lateral * 0.8 - up * 0.6):normalize(), 6, 0.6, 0.8, 'slash')
            waitSeconds(0.2)
            local took = ragdoll.getWoundFxAdded() == added + 1
            print(string.format('WOUNDFX06 region=%s took=%s drawn=%s', t.region, tostring(took),
                tostring(ragdoll.getWoundFxLastKind())))
            if took then
                local inverse = part.rotation:inverse()
                placed[#placed + 1] = { region = t.region, body = t.body,
                    localPoint = inverse * (part.centerOfMass + normal * 6 - part.position), localNormal = inverse * normal }
            end
        end
        -- Close (30 u) and at fighting distance (110 u), after the blood has
        -- had 4 s to well out.
        waitSeconds(4)
        local shot = 0
        for _, w in ipairs(placed) do
            for _, distance in ipairs({ 30, 110 }) do
                local part = body(ragdoll.getState(npc), w.body)
                local at = part.position + part.rotation * w.localPoint
                local out = (part.rotation * w.localNormal):normalize()
                local position = at + out * distance
                local look = at - position
                local lookFlat = math.sqrt(look.x * look.x + look.y * look.y)
                world.players[1]:sendEvent('fixWoundCamera', { position = position,
                    yaw = math.atan2(look.x, look.y), pitch = math.atan2(-look.z, math.max(lookFlat, 0.001)) })
                coroutine.yield()
                coroutine.yield()
                coroutine.yield()
                shot = shot + 1
                print(string.format('WOUNDFX06 shot=%d region=%s distance=%d', shot, w.region, distance))
                ragdoll.captureScreenshot()
                waitSeconds(0.1)
            end
        end
    end)
    world.players[1]:sendEvent('fixWoundCamera', {})
    if npc and npc:isValid() then npc:remove() end
    if not ok then error(message, 0) end
end)

-- WOUNDFX-04 (FX4 look check, needs OPENMW_RAGDOLL_WOUND_FX=4): blood.
-- 1. Runs: the stripped dummy takes WOUNDFX-02's slash on the bare right
--    forearm and stab in the chest; each is photographed 1 s, 6 s and 60 s
--    after it was made - the blood runs downhill from it over the first
--    seconds (thin rivulets, wet), then stops, and by 60 s it is darker.
-- 2. Blood types: a dummy with skeleton blood (bloodType 1) takes the same
--    slash - a dry bone-white crack, nothing wet, no run - and one with Dwemer
--    blood (2) - drawn as a metal scratch. A 'skeleton' creature, when the
--    data has one, is tried too (creatures have no wound volumes: printed).
-- 3. The blade: an armed dummy's iron longsword gets a blood mark two thirds
--    of the way up and a lighter one near the guard; the blade is
--    photographed fresh and at 60 s (wet red, then dark).
-- Screenshots land in order; WOUNDFX04 lines name them.
registerTest('WOUNDFX-04 blood runs, blood types, blood on the blade', 918336, function()
    placePlayer()
    local actors = {}
    local ok, message = pcall(function()
        if ragdoll.getWoundFxLevel() < 4 then
            print('WOUNDFX04 skipped: OPENMW_RAGDOLL_WOUND_FX=' .. ragdoll.getWoundFxLevel() .. ', needs 4')
            testing.expect(false, 'WOUNDFX-04 needs OPENMW_RAGDOLL_WOUND_FX=4')
            return
        end
        local up = util.vector3(0, 0, 1)
        -- A dummy like spawnDummy's, of a given blood type.
        local function spawnBlooded(name, offset, bloodType)
            local player = world.players[1]
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = types.NPC.record(player),
                name = name,
                isEssential = false,
                isPersistent = false,
                isRespawning = false,
                bloodType = bloodType,
            }))
            local actor = world.createObject(record.id)
            actor:teleport(testArea.cell, testArea.player + offset, { onGround = true })
            actor:addScript('stance.lua')
            coroutine.yield()
            actor:sendEvent('fixActorConfigure', { weaponStance = false })
            waitFor(name .. ' stance', function() return readyActors[actor.id] ~= nil end, 10)
            return actor
        end
        -- A stripped, standing dummy and its frame: chest, toward its right, its front.
        local function prepare(actor, label)
            actors[#actors + 1] = actor
            actor:sendEvent('fixWoundMode', { mode = 'normal' }) -- strips the clothes: bare skin
            waitSeconds(0.5)
            powerAndStand(actor, label)
            waitSeconds(2) -- settle, and let the meshes be culled and skinned
            local state = ragdoll.getState(actor)
            local shoulders = body(state, 'right_upper_arm').position - body(state, 'left_upper_arm').position
            local lateral = util.vector3(shoulders.x, shoulders.y, 0):normalize()
            return state, body(state, 'spine2'), lateral, up:cross(lateral):normalize()
        end
        -- WOUNDFX-02's slash: a diagonal stroke across the outer forearm.
        local function forearmSlash(actor, state, chest)
            local part = body(state, 'right_forearm')
            local side = part.centerOfMass - chest.centerOfMass
            local normal = util.vector3(side.x, side.y, 0):normalize()
            local axis = (body(state, 'right_hand').position - part.position):normalize()
            local across = normal:cross(axis):normalize()
            return { victim = actor, state = state, kind = 'slash', region = 'right_forearm', body = part.name,
                center = part.centerOfMass + axis * 6, normal = normal, out = 5, skin = 2.5,
                direction = (axis * 0.6 + across * 0.8):normalize(), length = 6, halfWidth = 0.55, severity = 0.75 }
        end
        local placed = {}
        local failures = {} -- checked after the shots, so a miss still leaves pictures
        local function place(wound, expect)
            local added = ragdoll.getWoundFxAdded()
            ragdoll.addWoundFx(wound.victim, wound.region, wound.center + wound.normal * wound.out, wound.normal,
                wound.direction, wound.length, wound.halfWidth, wound.severity, wound.kind)
            waitSeconds(0.2)
            local drawn = ragdoll.getWoundFxLastKind()
            wound.took = ragdoll.getWoundFxAdded() == added + 1
            wound.born = core.getSimulationTime()
            print(string.format('WOUNDFX04 placed kind=%s on=%s region=%s took=%s drawn=%s', wound.kind,
                wound.label, wound.region, tostring(wound.took), drawn))
            if not (wound.took and drawn == expect) then
                failures[#failures + 1] = string.format('the %s on the %s was drawn as %s, not a %s', wound.kind,
                    wound.label, drawn, expect)
            end
            if wound.took then
                local part = body(wound.state, wound.body)
                local inverse = part.rotation:inverse()
                wound.localPoint = inverse * (wound.center + wound.normal * wound.skin - part.position)
                wound.localNormal = inverse * wound.normal
                placed[#placed + 1] = wound
            end
        end
        local shotIndex = 0
        -- Close, near and oblique views from out of the wound; `views` can
        -- narrow them.
        local function photograph(at, out, tag, label, views)
            local flat = math.sqrt(out.x * out.x + out.y * out.y)
            local yaw = flat > 0.05 and math.atan2(-out.x, -out.y) or 0
            for _, view in ipairs(views or { { 'close', 24, 0 }, { 'near', 40, 0 }, { 'oblique', 34, 0.6 } }) do
                local swing = view[3]
                local dir = util.vector3(out.x * math.cos(swing) - out.y * math.sin(swing),
                    out.x * math.sin(swing) + out.y * math.cos(swing), out.z)
                local position = at + dir * view[2]
                local look = at - position
                local lookFlat = math.sqrt(look.x * look.x + look.y * look.y)
                local viewYaw = lookFlat > 0.05 and math.atan2(look.x, look.y) or yaw
                local pitch = math.max(-1.45, math.min(1.45, math.atan2(-look.z, math.max(lookFlat, 0.001))))
                world.players[1]:sendEvent('fixWoundCamera', { position = position, yaw = viewYaw, pitch = pitch })
                coroutine.yield()
                coroutine.yield()
                coroutine.yield()
                shotIndex = shotIndex + 1
                print(string.format('WOUNDFX04 shot=%d moment=%s on=%s view=%s at=%s normal=%s', shotIndex, tag,
                    label, view[1], tostring(at), tostring(out)))
                ragdoll.captureScreenshot()
                waitSeconds(0.1)
            end
        end
        local function photographWound(wound, tag)
            local part = body(ragdoll.getState(wound.victim), wound.body)
            photograph(part.position + part.rotation * wound.localPoint,
                (part.rotation * wound.localNormal):normalize(), tag, wound.kind .. ' ' .. wound.label)
        end
        local function waitUntil(time)
            local left = time - core.getSimulationTime()
            if left > 0 then waitSeconds(left) end
        end

        -- 1. Runs on red-blooded bare skin.
        local red = spawnDummy('WOUNDFX-04 dummy', util.vector3(0, 120, 0), false)
        local redState, redChest, redLateral, redForward = prepare(red, 'WOUNDFX-04 dummy')
        local runs = {}
        runs[1] = forearmSlash(red, redState, redChest)
        runs[1].label = 'red forearm'
        runs[2] = { victim = red, state = redState, label = 'red chest', kind = 'stab', region = 'chest',
            body = redChest.name, center = redChest.centerOfMass - redLateral * 4 + up * 2, normal = redForward,
            out = 8, skin = 7, direction = (up * 0.8 + redLateral * 0.6):normalize(), length = 2.4,
            halfWidth = 0.4, severity = 0.8 }
        for _, wound in ipairs(runs) do place(wound, wound.kind) end
        for _, moment in ipairs({ { '1 s', 1 }, { '6 s', 6 }, { '60 s', 60 } }) do
            for _, wound in ipairs(runs) do
                if wound.took then
                    waitUntil(wound.born + moment[2])
                    photographWound(wound, moment[1])
                end
            end
            world.players[1]:sendEvent('fixWoundCamera', {})
        end

        -- 2. Blood types.
        local skeletal = spawnBlooded('WOUNDFX-04 skeleton-blooded dummy', util.vector3(-60, 120, 0), 1)
        local skeletalState, skeletalChest = prepare(skeletal, 'WOUNDFX-04 skeleton-blooded dummy')
        local dust = forearmSlash(skeletal, skeletalState, skeletalChest)
        dust.label = 'skeleton forearm'
        place(dust, 'slash')
        local dwemer = spawnBlooded('WOUNDFX-04 Dwemer-blooded dummy', util.vector3(60, 120, 0), 2)
        local dwemerState, dwemerChest = prepare(dwemer, 'WOUNDFX-04 Dwemer-blooded dummy')
        local metal = forearmSlash(dwemer, dwemerState, dwemerChest)
        metal.label = 'Dwemer forearm'
        place(metal, 'scratch')
        for _, wound in ipairs({ dust, metal }) do
            if wound.took then
                waitUntil(wound.born + 1)
                photographWound(wound, '1 s')
            end
        end
        world.players[1]:sendEvent('fixWoundCamera', {})
        do -- a 'skeleton' creature, if the data has one
            local made, creature = pcall(function() return world.createObject('skeleton', 1) end)
            if not made or not creature then
                print('WOUNDFX04 creature skipped: no skeleton record in the test data')
            else
                actors[#actors + 1] = creature
                creature:teleport(testArea.cell, testArea.player + util.vector3(0, 220, 0), { onGround = true })
                waitSeconds(1)
                local added = ragdoll.getWoundFxAdded()
                local toPlayer = world.players[1].position - creature.position
                local normal = util.vector3(toPlayer.x, toPlayer.y, 0):normalize()
                local took = pcall(function()
                    ragdoll.addWoundFx(creature, 'right_forearm', creature.position + up * 70 + normal * 12, normal,
                        up, 6, 0.55, 0.75, 'slash')
                end)
                waitSeconds(0.2)
                print(string.format('WOUNDFX04 creature skeleton: call=%s took=%s drawn=%s (creatures have no'
                    .. ' wound volumes: only NPC bodies draw them)', tostring(took),
                    tostring(ragdoll.getWoundFxAdded() == added + 1), ragdoll.getWoundFxLastKind()))
                creature:remove()
                actors[#actors] = nil -- (it was the last one added)
            end
        end

        -- 3. Blood on the blade (the other dummies out of the way).
        for _, actor in ipairs(actors) do
            if actor:isValid() then actor:remove() end
        end
        actors = {}
        local armed = spawnDummy('WOUNDFX-04 armed dummy', util.vector3(0, 200, 0), 'iron longsword')
        actors[#actors + 1] = armed
        powerAndStand(armed, 'WOUNDFX-04 armed dummy')
        local attached = waitFor('WOUNDFX-04 longsword attached', function()
            local state = ragdoll.getState(armed)
            if state and state.weapon and state.weapon.attached then return state end
        end, 10)
        waitSeconds(1)
        local marks = ragdoll.getWeaponBloodFx(armed)
        ragdoll.addWeaponBloodFx(armed, 0.55, 0.85, 0.9) -- where it went in, deep
        ragdoll.addWeaponBloodFx(armed, 0.25, 0.4, 0.4) -- a lighter one, through cloth
        waitSeconds(0.3)
        local drawnMarks = ragdoll.getWeaponBloodFx(armed) - marks
        local markBorn = core.getSimulationTime()
        print(string.format('WOUNDFX04 blade marks drawn=%d', drawnMarks))
        if drawnMarks ~= 2 then
            failures[#failures + 1] = string.format('%d of 2 blood marks drawn on the longsword', drawnMarks)
        end
        local function photographBlade(tag)
            local state = ragdoll.getState(armed)
            if not (state and state.weapon) then return end
            local blade = state.bodies[state.weapon.bodyIndex]
            if not blade then return end
            local center = blade.renderPosition or blade.position
            local axis = (blade.rotation * util.vector3(0, 0, 1)):normalize()
            -- From the side of the blade, level with its middle.
            local side = axis:cross(up)
            if side:length() < 0.2 then
                local toPlayer = world.players[1].position - center
                side = util.vector3(toPlayer.x, toPlayer.y, 0)
            end
            side = side:normalize()
            if (world.players[1].position - center):dot(side) < 0 then side = side * -1 end
            photograph(center + axis * (0.15 * state.weapon.length), side, tag, 'longsword',
                { { 'side', 45, 0 }, { 'close', 28, 0 }, { 'oblique', 38, 0.7 } })
        end
        waitUntil(markBorn + 1)
        photographBlade('1 s')
        waitUntil(markBorn + 60)
        photographBlade('60 s')
        print(string.format('WOUNDFX04 done shots=%d placed=%d blade=%d attached=%s', shotIndex, #placed,
            drawnMarks, tostring(attached ~= nil)))
        testing.expect(#failures == 0, table.concat(failures, '; '))
    end)
    world.players[1]:sendEvent('fixWoundCamera', {})
    for _, actor in ipairs(actors) do
        if actor:isValid() then actor:remove() end
    end
    if not ok then error(message, 0) end
end)

-- CREATURE-01 (ALPHA 1.0 gate: never checked with the attack wheel): the
-- player's forehand cuts at a rat and at a guar standing in front. Passes when
-- each takes damage and nothing errors (creatures have no powered body: the
-- blade strikes their collision shapes).
for index, creature in ipairs({ 'rat', 'guar' }) do
    registerTest('CREATURE-01 wheel cuts at a ' .. creature, 918380 + index, function()
        local weapon = 'iron longsword'
        local player = placePlayer()
        local victim
        local ok, message = pcall(function()
            ragdoll.setPlayerAlwaysHits(true)
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 480 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 480 + index end)
            waitSeconds(1)
            powerAndStand(player, 'CREATURE-01 player')
            cameraReadyMode = nil
            player:sendEvent('fixPlayerCamera', true)
            waitFor('CREATURE-01 first-person camera', function() return cameraReadyMode == true end)
            ragdoll.prepareMouseInput()
            victim = world.createObject(creature, 1)
            victim:teleport(testArea.cell, testArea.player + util.vector3(0, creature == 'rat' and 70 or 110, 0),
                { onGround = true, rotation = util.transform.rotateZ(math.pi) })
            -- The test config loads no Morrowind combat script (data-mw), so
            -- the dummies' own handler takes the Hit event (as for the NPCs).
            victim:addScript('stance.lua')
            waitSeconds(1.0)
            local health0 = types.Actor.stats.dynamic.health(victim).current
            for rep = 1, 3 do
                if types.Actor.isDead(victim) then break end
                contactAimReady = false
                player:sendEvent('fixAimAt', victim.position + util.vector3(0, 0, creature == 'rat' and 10 or 60))
                pcall(waitFor, 'CREATURE-01 look', function() return contactAimReady end, 4)
                ragdoll.injectMouseInput(0, 0, true, 1)
                waitSeconds(0.25)
                for _, leg in ipairs({ { x = HSC_WIND, seconds = 0.4 }, { x = -HSC_CUT, seconds = 0.2 } }) do
                    local started, sent = core.getSimulationTime(), 0
                    waitSeconds(leg.seconds, function()
                        local f = math.min(1, (core.getSimulationTime() - started) / leg.seconds)
                        local v = math.floor(leg.x * f)
                        ragdoll.injectMouseInput(v - sent, 0)
                        sent = v
                    end)
                    ragdoll.injectMouseInput(leg.x - sent, 0)
                end
                ragdoll.injectMouseInput(0, 0, false, 1)
                waitSeconds(1.0)
                print(string.format('CREATURE01 rep=%d health=%.1f of %.1f', rep,
                    types.Actor.stats.dynamic.health(victim).current, health0))
            end
            local lost = health0 - types.Actor.stats.dynamic.health(victim).current
            print(string.format('CREATURE01 creature=%s lost=%.1f dead=%s', creature, lost, tostring(types.Actor.isDead(victim))))
            testing.expect(lost > 0, 'CREATURE-01 the ' .. creature .. ' took no damage')
        end)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if victim and victim:isValid() then victim:remove() end
        if not ok then error(message, 0) end
    end)
end

-- REACT-01 (stage 7, hit reactions; video: "still too spongy"): forehand
-- cuts at a standing dummy, looking at its head and at its chest. Prints how
-- far its head and chest turned (degrees, largest over 0.6 s from just
-- before the cut) and whether it fell. Compare OPENMW_RAGDOLL_HIT_REACT=0.
registerTest('REACT-01 hit reactions of a struck dummy with iron longsword', 918370, function()
    local weapon = 'iron longsword'
    local player = placePlayer()
    local victim
    local ok, message = pcall(function()
        ragdoll.setPlayerAlwaysHits(true)
        if not types.Actor.inventory(player):find(weapon) then
            world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 470, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 470 end)
        waitSeconds(1)
        powerAndStand(player, 'REACT-01 player')
        cameraReadyMode = nil
        player:sendEvent('fixPlayerCamera', true)
        waitFor('REACT-01 first-person camera', function() return cameraReadyMode == true end)
        ragdoll.prepareMouseInput()
        local function forwardOf(state, name)
            local b = body(state, name)
            return b.rotation * util.vector3(0, 1, 0)
        end
        local function angle(a, b)
            local d = math.max(-1, math.min(1, a:normalize():dot(b:normalize())))
            return math.deg(math.acos(d))
        end
        for rep = 1, 3 do
            for _, part in ipairs({ 'head', 'spine2' }) do
                if not victim or not victim:isValid() or types.Actor.isDead(victim) or not ragdoll.getState(victim) then
                    if victim and victim:isValid() then victim:remove() coroutine.yield() end
                    victim = spawnDummy('REACT-01 target', util.vector3(0, 200, 0), false)
                    powerAndStand(victim, 'REACT-01 target')
                end
                local p = ragdoll.getState(player)
                local shoulder = body(p, 'right_upper_arm').position
                local elbow = body(p, 'right_forearm').position
                local hand = body(p, 'right_hand').position
                local reach = (elbow - shoulder):length() + (hand - elbow):length()
                local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
                placeVictimPoint(victim, function()
                    local vs = ragdoll.getState(victim)
                    return vs and body(vs, 'spine2').position or victim.position
                end, pivot + util.vector3(0, reach + p.weapon.length * 0.55, 0), util.transform.rotateZ(math.pi))
                waitSeconds(0.6)
                contactAimReady = false
                local vs0 = ragdoll.getState(victim)
                if vs0 then player:sendEvent('fixAimAt', body(vs0, part).position) end
                pcall(waitFor, 'REACT-01 look', function() return contactAimReady end, 4)
                local vs = ragdoll.getState(victim)
                if vs then
                    local head0, chest0 = forwardOf(vs, 'head'), forwardOf(vs, 'spine2')
                    local headTurn, chestTurn, fell = 0, 0, false
                    local function watch()
                        local st = ragdoll.getState(victim)
                        if not st then return end
                        headTurn = math.max(headTurn, angle(head0, forwardOf(st, 'head')))
                        chestTurn = math.max(chestTurn, angle(chest0, forwardOf(st, 'spine2')))
                        if st.controller and (st.controller.mode == 'falling' or st.controller.mode == 'fallen') then fell = true end
                    end
                    ragdoll.injectMouseInput(0, 0, true, 1)
                    if part == 'head' then
                        -- A click: the chop, which reaches the head (AIM-04).
                        waitSeconds(0.08, watch)
                    else
                        waitSeconds(0.25)
                        for _, leg in ipairs({ { x = HSC_WIND, seconds = 0.4 }, { x = -HSC_CUT, seconds = 0.2 } }) do
                            local started, sent = core.getSimulationTime(), 0
                            waitSeconds(leg.seconds, function()
                                local f = math.min(1, (core.getSimulationTime() - started) / leg.seconds)
                                local v = math.floor(leg.x * f)
                                ragdoll.injectMouseInput(v - sent, 0)
                                sent = v
                                watch()
                            end)
                            ragdoll.injectMouseInput(leg.x - sent, 0)
                        end
                    end
                    ragdoll.injectMouseInput(0, 0, false, 1)
                    waitSeconds(part == 'head' and 1.2 or 0.6, watch)
                    print(string.format('REACT01 part=%s rep=%d head_turn=%.1f chest_turn=%.1f fell=%s',
                        part, rep, headTurn, chestTurn, tostring(fell)))
                end
                pcall(waitFor, 'REACT-01 withdrawal', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                waitSeconds(1.0)
            end
        end
    end)
    ragdoll.injectMouseInput(0, 0, false)
    ragdoll.setPlayerAlwaysHits(false)
    ragdoll.setPowered(player, false)
    if victim and victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)

-- CLASH-01 (C1, build 59 bookmarks 10/13: "a chop with a giant claymore
-- should be slowed down by the staff but it should at least do some
-- damage"): a dummy holds a wooden staff in its block; three chops each with a
-- claymore (heavy) and a longsword. CLASH01 lines: damage dealt, blade clashes
-- (RAGDOLL_PARRY) and push-throughs (RAGDOLL_CLASH_WEIGHT) between
-- CLASH01_BEGIN/END. Compare OPENMW_RAGDOLL_CLASH_WEIGHT=0.
for index, weapon in ipairs({ 'iron claymore', 'iron longsword' }) do
    registerTest('CLASH-01 chops into a staff block with ' .. weapon, 918360 + index, function()
        local player = placePlayer()
        local victim
        local savedReaction = ragdoll.reactionScale()
        local ok, message = pcall(function()
            ragdoll.setPlayerAlwaysHits(true)
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 460 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 460 + index end)
            waitSeconds(1)
            powerAndStand(player, 'CLASH-01 player')
            cameraReadyMode = nil
            player:sendEvent('fixPlayerCamera', true)
            waitFor('CLASH-01 first-person camera', function() return cameraReadyMode == true end)
            ragdoll.reactionScale(0)
            ragdoll.prepareMouseInput()
            local function damageTotal()
                local total = 0
                if victim and victim:isValid() then
                    for _, value in pairs(ragdoll.getLimbDamage(victim)) do total = total + value end
                end
                return total
            end
            local function drag(y, seconds)
                local started, sent = core.getSimulationTime(), 0
                waitSeconds(seconds, function()
                    local f = math.min(1, (core.getSimulationTime() - started) / seconds)
                    local v = math.floor(y * f)
                    ragdoll.injectMouseInput(0, v - sent)
                    sent = v
                end)
                ragdoll.injectMouseInput(0, y - sent)
            end
            for rep = 1, 3 do
                if not victim or not victim:isValid() or types.Actor.isDead(victim) or not ragdoll.getState(victim) then
                    if victim and victim:isValid() then victim:remove() coroutine.yield() end
                    victim = spawnDummy('CLASH-01 staff', util.vector3(0, 200, 0), 'wooden staff')
                    powerAndStand(victim, 'CLASH-01 staff')
                end
                local p = ragdoll.getState(player)
                local shoulder = body(p, 'right_upper_arm').position
                local elbow = body(p, 'right_forearm').position
                local hand = body(p, 'right_hand').position
                local reach = (elbow - shoulder):length() + (hand - elbow):length()
                local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
                placeVictimPoint(victim, function()
                    local vs = ragdoll.getState(victim)
                    return vs and body(vs, 'spine2').position or victim.position
                end, pivot + util.vector3(0, reach + p.weapon.length * 0.55, 0), util.transform.rotateZ(math.pi))
                waitSeconds(0.6)
                local before = damageTotal()
                print(string.format('CLASH01_BEGIN weapon=%s rep=%d', weapon, rep))
                ragdoll.injectMouseInput(0, 0, true, 1)
                waitSeconds(0.25)
                drag(-HSC_WIND, 0.4)
                -- Raised as the cut starts: a guard blocks in its first 0.6 s.
                ragdoll.setGuard(victim, true)
                drag(HSC_CUT, 0.2)
                waitSeconds(0.3)
                ragdoll.injectMouseInput(0, 0, false, 1)
                waitSeconds(0.6)
                pcall(waitFor, 'CLASH-01 withdrawal', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                print(string.format('CLASH01_END weapon=%s rep=%d damage=%.2f', weapon, rep, damageTotal() - before))
                if victim and victim:isValid() then ragdoll.setGuard(victim, false) end
                waitSeconds(0.6)
            end
        end)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.reactionScale(savedReaction)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if victim and victim:isValid() then victim:remove() end
        if not ok then error(message, 0) end
    end)
end

-- REACT-02 (RXN-02: the reaction baseline before the controller rewrite): a
-- matrix of blows on a fresh dummy per cell - six parts (head, chest,
-- shoulder, upper arm, thigh, calf) x three strengths (light, solid, heavy) x
-- standing and walking, from the front, plus side blows to the head and
-- chest and no-blow controls. Each blow goes through the same reaction entry
-- point as a weapon blow (ragdoll.applyBlow). Per cell over 4 s: the largest
-- turn of the head, the chest and the struck part (degrees), the chest's
-- tilt from upright, how far the pelvis travelled, steps taken, a fall, and
-- the time until the body settled (tilt back, upright mode, held 0.3 s).
-- REACT02 lines (tools/body/analysis/react02.py); a side still at the peak.
-- REACT-05 (user 10-07: "the hit reactions ... are barely doing anything"):
-- REACT-02 measures unarmed dummies; in play the victim is armed and
-- fighting (the wheel and the guard hold its arms). The same measurements on
-- a dummy armed with an iron longsword that fights the player with its own
-- AI, struck 2 s into the fight (side column 'fight').
local function reactMatrix(fightOnly)
    return function()
    local player = placePlayer()
    local victim
    local ok, message = pcall(function()
        local strengths = {
            light = { speed = 300, mass = 1.6, damage = 5 },
            solid = { speed = 800, mass = 1.6, damage = 15 },
            heavy = { speed = 1600, mass = 2.4, damage = 35 },
        }
        local cells = {}
        if fightOnly then
            cells[#cells + 1] = { part = 'none', strength = 'none', moving = false, side = false, fight = true }
            for _, part in ipairs({ 'head', 'spine2', 'right_upper_arm', 'left_thigh' }) do
                for _, strength in ipairs({ 'solid', 'heavy' }) do
                    cells[#cells + 1] = { part = part, strength = strength, moving = false, side = false, fight = true }
                end
            end
        end
        for _, moving in ipairs(fightOnly and {} or { false, true }) do
            cells[#cells + 1] = { part = 'none', strength = 'none', moving = moving, side = false }
            for _, part in ipairs({ 'head', 'spine2', 'right_clavicle', 'right_upper_arm', 'left_thigh', 'left_calf' }) do
                for _, strength in ipairs({ 'light', 'solid', 'heavy' }) do
                    cells[#cells + 1] = { part = part, strength = strength, moving = moving, side = false }
                end
            end
        end
        for _, part in ipairs(fightOnly and {} or { 'head', 'spine2' }) do
            for _, strength in ipairs({ 'solid', 'heavy' }) do
                cells[#cells + 1] = { part = part, strength = strength, moving = false, side = true }
                -- A guarding victim (armed, guard raised): a braced body.
                cells[#cells + 1] = { part = part, strength = strength, moving = false, side = false, guard = true }
            end
        end
        local function axes(state, name)
            local b = body(state, name)
            return b.rotation * util.vector3(0, 1, 0), b.rotation * util.vector3(0, 0, 1)
        end
        local function angle(a, b)
            local d = math.max(-1, math.min(1, a:normalize():dot(b:normalize())))
            return math.deg(math.acos(d))
        end
        local function turn(f0, u0, f1, u1) return math.max(angle(f0, f1), angle(u0, u1)) end
        local down = { falling = true, fallen = true, gettingUp = true, getting_up = true }
        for index, cell in ipairs(cells) do
            if victim and victim:isValid() then victim:remove() coroutine.yield() end
            victim = spawnDummy('REACT-02 dummy', util.vector3(0, 320, 0),
                (cell.guard or cell.fight) and 'iron longsword' or false)
            victim:sendEvent('fixActorHealth', 100)
            victim:teleport(testArea.cell, testArea.player + util.vector3(0, 320, 0),
                { onGround = true, rotation = util.transform.rotateZ(math.pi) })
            coroutine.yield()
            powerAndStand(victim, 'REACT-02 dummy')
            waitSeconds(0.8)
            if cell.moving then
                victim:sendEvent('fixActorMove', 1)
                waitSeconds(1.5)
            end
            if cell.guard then
                ragdoll.setGuard(victim, true, util.vector3(0, -1, 0))
                waitSeconds(0.7)
            end
            if cell.fight then
                victim:sendEvent('fixActorFight', true)
                ragdoll.setCombatTarget(victim, player)
                waitSeconds(2.0)
            end
            local s0 = ragdoll.getState(victim)
            if s0 then
                local name = cell.part == 'none' and 'spine2' or cell.part
                local h0f, h0u = axes(s0, 'head')
                local c0f, c0u = axes(s0, 'spine2')
                local p0f, p0u = axes(s0, name)
                local pelvis0 = body(s0, 'pelvis').position
                -- Lean: the pelvis-to-chest axis against vertical.
                local function lean(st) return angle(body(st, 'spine2').position - body(st, 'pelvis').position, util.vector3(0, 0, 1)) end
                local tilt0 = lean(s0)
                -- The dummy faces -y: a front blow pushes +y, a side blow +x.
                local direction = cell.side and util.vector3(1, 0, 0) or util.vector3(0, 1, 0)
                if cell.fight then
                    -- From the player toward the dummy (it faces the player).
                    local away = victim.position - player.position
                    away = util.vector3(away.x, away.y, 0)
                    direction = away:length() > 1 and away:normalize() or direction
                end
                local headTurn, chestTurn, partTurn, tiltMax, travel, fell = 0, 0, 0, 0, 0, false
                local drop = 0 -- the pelvis's lowest point under where it stood (a knee giving out)
                local netX, netY = 0, 0
                local headSpin, partT, footOff, footLand = 0, nil, nil, nil
                local la0f, la0u = axes(s0, 'left_upper_arm')
                local ra0f, ra0u = axes(s0, 'right_upper_arm')
                local armRise = 0
                local partV0 = body(s0, name).linearVelocity
                local steps, footFast = 0, { false, false }
                local settle, calmSince, started = nil, nil, core.getSimulationTime()
                local shot = false
                if cell.part ~= 'none' then
                    local st = strengths[cell.strength]
                    ragdoll.applyBlow(victim, cell.part, direction, st.speed, st.mass, st.damage)
                end
                print(string.format('REACT02_BEGIN cell=%d part=%s strength=%s moving=%s side=%s guard=%s',
                    index, cell.part, cell.strength, tostring(cell.moving), tostring(cell.side), tostring(cell.guard == true)))
                waitSeconds(4.0, function()
                    local st = ragdoll.getState(victim)
                    if not st then return end
                    local now = core.getSimulationTime() - started
                    local hf, hu = axes(st, 'head')
                    local cf, cu = axes(st, 'spine2')
                    local pf, pu = axes(st, cell.part == 'none' and 'spine2' or cell.part)
                    headTurn = math.max(headTurn, turn(h0f, h0u, hf, hu))
                    chestTurn = math.max(chestTurn, turn(c0f, c0u, cf, cu))
                    partTurn = math.max(partTurn, turn(p0f, p0u, pf, pu))
                    local tilt = lean(st)
                    tiltMax = math.max(tiltMax, tilt)
                    local pelvis = body(st, 'pelvis').position
                    local moved = util.vector3(pelvis.x - pelvis0.x, pelvis.y - pelvis0.y, 0)
                    travel = math.max(travel, moved:length())
                    drop = math.max(drop, pelvis0.z - pelvis.z)
                    netX, netY = moved.x, moved.y
                    local laf, lau = axes(st, 'left_upper_arm')
                    local raf, rau = axes(st, 'right_upper_arm')
                    armRise = math.max(armRise, turn(la0f, la0u, laf, lau), turn(ra0f, ra0u, raf, rau))
                    local hw = body(st, 'head').angularVelocity
                    if hw then headSpin = math.max(headSpin, hw:length()) end
                    local pv = body(st, cell.part == 'none' and 'spine2' or cell.part).linearVelocity
                    if not partT and pv and partV0 and (pv - partV0):length() > 30 then partT = now end
                    local mode = st.controller and st.controller.mode or ''
                    if down[mode] then fell = true end
                    for i, foot in ipairs({ 'left_foot', 'right_foot' }) do
                        local v = body(st, foot).linearVelocity
                        local speed = v and util.vector3(v.x, v.y, 0):length() or 0
                        if not footFast[i] and speed > 80 then
                            footFast[i] = true
                            footOff = footOff or now
                        elseif footFast[i] and speed < 20 then
                            footFast[i] = false
                            steps = steps + 1
                            footLand = footLand or now
                        end
                    end
                    local calm = not down[mode] and tilt < tilt0 + 5
                    if calm then
                        calmSince = calmSince or now
                        if not settle and now - calmSince >= 0.3 and (now > 0.25 or cell.part == 'none') then settle = calmSince end
                    else
                        calmSince = nil
                        settle = nil
                    end
                    -- Stills: the impact (0.12 s), the reflex (0.30 s) and a
                    -- leg giving out (0.70 s, leg blows).
                    local legBlow = cell.part:find('thigh') or cell.part:find('calf') or cell.fight
                    local due = (shot == false and now >= 0.12 and 0.12) or (shot == 0.12 and now >= 0.3 and 0.3)
                        or (legBlow and shot == 0.3 and now >= 0.7 and 0.7) or nil
                    if due and cell.part ~= 'none' then
                        shot = due
                        world.players[1]:sendEvent('fixWoundCamera',
                            { position = victim.position + util.vector3(-260, 0, 60), yaw = math.pi / 2 })
                        print(string.format('REACT02 shot cell=%d part=%s strength=%s moving=%s side=%s t=%.2f', index,
                            cell.part, cell.strength, tostring(cell.moving), cell.fight and 'fight' or tostring(cell.side), due))
                        ragdoll.captureScreenshot()
                    end
                end)
                if cell.moving then victim:sendEvent('fixActorMove', 0) end
                if cell.fight and victim:isValid() then victim:sendEvent('fixActorFight', false) end
                if cell.guard and victim:isValid() then ragdoll.setGuard(victim, false) end
                -- Direction error: where the pelvis went against the blow (standing).
                local dirErr = 'none'
                if not cell.moving and cell.part ~= 'none' and math.sqrt(netX * netX + netY * netY) > 5 then
                    dirErr = string.format('%.0f', angle(util.vector3(netX, netY, 0), direction))
                end
                local function t(v) return v and string.format('%.3f', v) or 'none' end
                print(string.format('REACT02 cell=%d part=%s strength=%s moving=%s side=%s guard=%s fight=%s head_turn=%.1f chest_turn=%.1f '
                    .. 'part_turn=%.1f tilt_max=%.1f tilt0=%.1f travel=%.1f net_x=%.1f net_y=%.1f steps=%d fell=%s settle=%s '
                    .. 'head_spin=%.1f part_t=%s foot_off=%s foot_land=%s dir_err=%s arm_turn=%.1f drop=%.1f',
                    index, cell.part, cell.strength, tostring(cell.moving), tostring(cell.side), tostring(cell.guard == true),
                    tostring(cell.fight == true), headTurn, chestTurn, partTurn,
                    tiltMax, tilt0, travel, netX, netY, steps, tostring(fell), settle and string.format('%.2f', settle) or 'none',
                    headSpin, t(partT), t(footOff), t(footLand), dirErr, armRise, drop))
            end
        end
    end)
    world.players[1]:sendEvent('fixWoundCamera', {})
    if victim and victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
    end
end
registerTest('REACT-02 region and strength matrix of blows', 919210, reactMatrix(false))
registerTest('REACT-05 blows on an armed fighting dummy', 919260, reactMatrix(true))

-- CLICK-02 (build 63 play: a click chop met the foe 0.6 s after the click,
-- contact and no blow - the cut's intent ended 0.4 s after the button):
-- a click attack while the dummy stands out of reach, then the dummy is
-- moved into the blade's path partway through the cut (0.35, 0.5 and 0.65 s
-- after the click). CLICK02 lines: contacts and strikes per click. Compare
-- OPENMW_RAGDOLL_CUT_INTENT=1.
for index, weapon in ipairs({ 'iron claymore', 'iron longsword' }) do
    registerTest('CLICK-02 a click cut meets a foe late with ' .. weapon, 919220 + index, function()
        local player = placePlayer()
        local victim
        local ok, message = pcall(function()
            ragdoll.setPlayerAlwaysHits(true)
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 480 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 480 + index end)
            waitSeconds(1)
            powerAndStand(player, 'CLICK-02 player')
            ragdoll.prepareMouseInput()
            for rep = 1, 3 do
                for _, delay in ipairs({ 0.35, 0.5, 0.65 }) do
                    if not victim or not victim:isValid() or types.Actor.isDead(victim) or not ragdoll.getState(victim) then
                        if victim and victim:isValid() then victim:remove() coroutine.yield() end
                        victim = spawnDummy('CLICK-02 target', util.vector3(0, 400, 0), false)
                        powerAndStand(victim, 'CLICK-02 target')
                    end
                    pcall(waitFor, 'CLICK-02 guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                    local p = ragdoll.getState(player)
                    local shoulder = body(p, 'right_upper_arm').position
                    local elbow = body(p, 'right_forearm').position
                    local hand = body(p, 'right_hand').position
                    local reach = (elbow - shoulder):length() + (hand - elbow):length()
                    local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
                    local near = pivot + util.vector3(0, reach + p.weapon.length * 0.6, 0)
                    -- Out of reach while the click starts.
                    placeVictimPoint(victim, function()
                        local vs = ragdoll.getState(victim)
                        return vs and body(vs, 'spine2').position or victim.position
                    end, near + util.vector3(0, 220, 0), util.transform.rotateZ(math.pi))
                    waitSeconds(0.5)
                    local w0 = ragdoll.getState(player).weapon
                    local contacts0, strikes0 = w0.contactEventCount, w0.strikeEventCount
                    print(string.format('CLICK02_BEGIN weapon=%s delay=%.2f rep=%d', weapon, delay, rep))
                    ragdoll.injectMouseInput(0, 0, true, 1)
                    waitSeconds(0.08)
                    ragdoll.injectMouseInput(0, 0, false, 1)
                    waitSeconds(delay - 0.08)
                    -- Into the blade's path.
                    placeVictimPoint(victim, function()
                        local vs = ragdoll.getState(victim)
                        return vs and body(vs, 'spine2').position or victim.position
                    end, near, util.transform.rotateZ(math.pi))
                    waitSeconds(1.2)
                    local w = ragdoll.getState(player).weapon
                    print(string.format('CLICK02 weapon=%s delay=%.2f rep=%d contacts=%d strikes=%d', weapon, delay, rep,
                        w.contactEventCount - contacts0, w.strikeEventCount - strikes0))
                    waitSeconds(0.5)
                end
            end
        end)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if victim and victim:isValid() then victim:remove() end
        if not ok then error(message, 0) end
    end)
end

-- BITE-01 (RXN-02 M7: glance, bite, stop, cleave): forehand cuts at a
-- dummy in no cuirass, netch leather, imperial chain and iron, at a fast and
-- a slow drag, per sword. The engine logs RAGDOLL_CONTACT_OUTCOME (outcome,
-- energy, the slot's cut energy) with OPENMW_RAGDOLL_CONTACT_OUTCOME=1;
-- BITE01_BEGIN/END frame each cut with its damage.
for index, weapon in ipairs({ 'iron longsword', 'iron claymore' }) do
    registerTest('BITE-01 cuts into armour with ' .. weapon, 919230 + index, function()
        local player = placePlayer()
        local victim
        local ok, message = pcall(function()
            ragdoll.setPlayerAlwaysHits(true)
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 490 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 490 + index end)
            waitSeconds(1)
            powerAndStand(player, 'BITE-01 player')
            ragdoll.prepareMouseInput()
            local function damageTotal()
                local total = 0
                if victim and victim:isValid() then
                    for _, value in pairs(ragdoll.getLimbDamage(victim)) do total = total + value end
                end
                return total
            end
            local function drag(x, seconds)
                local started, sent = core.getSimulationTime(), 0
                waitSeconds(seconds, function()
                    local f = math.min(1, (core.getSimulationTime() - started) / seconds)
                    local v = math.floor(x * f)
                    ragdoll.injectMouseInput(v - sent, 0)
                    sent = v
                end)
                ragdoll.injectMouseInput(x - sent, 0)
            end
            for _, armour in ipairs({ 'none', 'netch_leather_cuirass', 'imperial_chain_cuirass', 'iron_cuirass' }) do
                if victim and victim:isValid() then victim:remove() coroutine.yield() end
                victim = spawnDummy('BITE-01 target', util.vector3(0, 200, 0), false)
                if armour ~= 'none' then
                    local made = pcall(function() world.createObject(armour, 1):moveInto(types.Actor.inventory(victim)) end)
                    if not made then print('BITE01 missing armour ' .. armour) end
                    coroutine.yield()
                    victim:sendEvent('fixActorCuirass', armour)
                end
                powerAndStand(victim, 'BITE-01 target')
                for _, speed in ipairs({ { 'fast', 0.2 }, { 'slow', 0.6 } }) do
                    local p = ragdoll.getState(player)
                    local shoulder = body(p, 'right_upper_arm').position
                    local elbow = body(p, 'right_forearm').position
                    local hand = body(p, 'right_hand').position
                    local reach = (elbow - shoulder):length() + (hand - elbow):length()
                    local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
                    placeVictimPoint(victim, function()
                        local vs = ragdoll.getState(victim)
                        return vs and body(vs, 'spine2').position or victim.position
                    end, pivot + util.vector3(0, reach + p.weapon.length * 0.55, 0), util.transform.rotateZ(math.pi))
                    waitSeconds(0.6)
                    local before = damageTotal()
                    print(string.format('BITE01_BEGIN weapon=%s armour=%s speed=%s', weapon, armour, speed[1]))
                    ragdoll.injectMouseInput(0, 0, true, 1)
                    waitSeconds(0.25)
                    drag(HSC_WIND, 0.4)
                    drag(-HSC_CUT, speed[2])
                    waitSeconds(0.3)
                    ragdoll.injectMouseInput(0, 0, false, 1)
                    waitSeconds(0.6)
                    pcall(waitFor, 'BITE-01 withdrawal', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                    print(string.format('BITE01_END weapon=%s armour=%s speed=%s damage=%.2f', weapon, armour, speed[1],
                        damageTotal() - before))
                    waitSeconds(0.5)
                end
            end
        end)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if victim and victim:isValid() then victim:remove() end
        if not ok then error(message, 0) end
    end)
end

-- REACT-03 (RXN-02, the robustness sweep after DReCon's cube test): a dummy
-- walks back and forth for 60 s taking a blow every 1.2-1.8 s - a random
-- part, strength (light/solid/heavy weights 3/2/1) and direction - through
-- the blow entry point. REACT03 lines: each blow, each fall, and the summary
-- (blows, falls, mean seconds between falls, steps per blow).
registerTest('REACT-03 random blows on a walking dummy', 919240, function()
    placePlayer()
    local victim
    local ok, message = pcall(function()
        math.randomseed(919240)
        local parts = { 'head', 'spine2', 'spine1', 'right_clavicle', 'left_upper_arm', 'right_forearm', 'left_thigh',
            'right_calf', 'pelvis' }
        local strengths = {
            { 'light', 300, 1.6, 5 }, { 'light', 300, 1.6, 5 }, { 'light', 300, 1.6, 5 },
            { 'solid', 800, 1.6, 15 }, { 'solid', 800, 1.6, 15 }, { 'heavy', 1600, 2.4, 35 },
        }
        local down = { falling = true, fallen = true, getting_up = true }
        local function fresh()
            if victim and victim:isValid() then victim:remove() coroutine.yield() end
            victim = spawnDummy('REACT-03 dummy', util.vector3(0, 300, 0), false)
            victim:sendEvent('fixActorHealth', 100)
            powerAndStand(victim, 'REACT-03 dummy')
        end
        fresh()
        local started = core.getSimulationTime()
        local nextBlow, blows, falls, wasDown, lastFall = 1.0, 0, 0, false, 0
        local walkFlip, direction = 0, 1
        waitSeconds(60, function()
            if not victim or not victim:isValid() then return end
            local now = core.getSimulationTime() - started
            if now >= walkFlip then
                direction = -direction
                victim:sendEvent('fixActorMove', 1)
                local yaw = direction > 0 and 0 or math.pi
                walkFlip = now + 3.0
                pcall(function() victim:teleport(testArea.cell, victim.position, { rotation = util.transform.rotateZ(yaw) }) end)
            end
            local st = ragdoll.getState(victim)
            local mode = st and st.controller and st.controller.mode or ''
            local isDown = down[mode] == true
            if isDown and not wasDown then
                falls = falls + 1
                print(string.format('REACT03 fall t=%.2f since_last=%.2f', now, now - lastFall))
                lastFall = now
            end
            wasDown = isDown
            if now >= nextBlow and not isDown then
                local part = parts[math.random(#parts)]
                local st2 = strengths[math.random(#strengths)]
                local angle = math.random() * 2 * math.pi
                ragdoll.applyBlow(victim, part, util.vector3(math.cos(angle), math.sin(angle), 0), st2[2], st2[3], st2[4])
                blows = blows + 1
                print(string.format('REACT03 blow t=%.2f part=%s strength=%s angle=%.0f', now, part, st2[1], math.deg(angle)))
                nextBlow = now + 1.2 + math.random() * 0.6
            end
        end)
        if victim and victim:isValid() then victim:sendEvent('fixActorMove', 0) end
        print(string.format('REACT03 summary blows=%d falls=%d mean_between=%.1f', blows, falls,
            falls > 0 and 60 / falls or 60))
    end)
    if victim and victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)

-- STEADY-02 (user 10-07: "the enemies still wiggle their heads and weapons
-- too much, especially when running"): an NPC armed with an iron longsword
-- chases the player from 1500 units (its AI runs it in), three times; then
-- walks. Per frame while it moves: the head's spin (rad/s, world) and the
-- weapon hand's speed in the NPC's own frame (its position and facing, so
-- the run itself does not count). STEADY02 lines: p50/p90 per gait.
registerTest('STEADY-02 an armed NPC running and walking', 919270, function()
    local player = placePlayer()
    local victim
    local ok, message = pcall(function()
        local function pct(list, q)
            if #list == 0 then return -1 end
            table.sort(list)
            return list[math.max(1, math.floor(#list * q + 0.5))]
        end
        local function sample(label, seconds, setup)
            local spins, hands = {}, {}
            local gripSamples, gripOn = 0, 0
            local prevLocal, prevTime
            local shotAt = core.getSimulationTime() + seconds * 0.6
            setup()
            waitSeconds(seconds, function()
                -- A side still of the two-hander mid-run (the left hand's grip).
                if label == 'run2h' and shotAt and core.getSimulationTime() >= shotAt and victim:isValid() then
                    shotAt = nil
                    world.players[1]:sendEvent('fixWoundCamera',
                        { position = victim.position + util.vector3(-220, 0, 70), yaw = math.pi / 2 })
                    print('STEADY02 shot')
                    ragdoll.captureScreenshot()
                end
                if not victim:isValid() then return end
                local st = ragdoll.getState(victim)
                if not st then return end
                local hw = body(st, 'head').angularVelocity
                local now = core.getSimulationTime()
                local yaw = victim.rotation:getYaw()
                local rel = body(st, 'right_hand').position - victim.position
                local c, s2 = math.cos(-yaw), math.sin(-yaw)
                local localHand = util.vector3(rel.x * c - rel.y * s2, rel.x * s2 + rel.y * c, rel.z)
                local speed = 0
                if prevLocal and now > prevTime then speed = (localHand - prevLocal):length() / (now - prevTime) end
                prevLocal, prevTime = localHand, now
                local v = victim.position
                local moving = st.controller and st.controller.mode ~= 'standing' or true
                if hw then spins[#spins + 1] = hw:length() end
                if st.weapon then
                    gripSamples = gripSamples + 1
                    if st.weapon.supportGrip and st.weapon.supportGrip.attached then gripOn = gripOn + 1 end
                end
                if speed > 0 then hands[#hands + 1] = speed end
            end)
            print(string.format('STEADY02 gait=%s head_spin_p50=%.2f head_spin_p90=%.2f hand_p50=%.1f hand_p90=%.1f grip=%.2f n=%d',
                label, pct(spins, 0.5), pct(spins, 0.9), pct(hands, 0.5), pct(hands, 0.9),
                gripSamples > 0 and gripOn / gripSamples or -1, #spins))
        end
        for rep = 1, 4 do
            -- Reps 3-4 with a two-hander (the left hand's grip, "grip" = share of
            -- samples with the support grip attached).
            local weapon = rep <= 2 and 'iron longsword' or 'iron claymore'
            if victim and victim:isValid() then victim:remove() coroutine.yield() end
            victim = spawnDummy('STEADY-02 runner', util.vector3(0, 1500, 0), weapon)
            victim:sendEvent('fixActorHealth', 100)
            powerAndStand(victim, 'STEADY-02 runner')
            waitSeconds(0.5)
            sample(rep <= 2 and 'run' or 'run2h', 3.0, function()
                victim:sendEvent('fixActorFight', true)
                ragdoll.setCombatTarget(victim, player)
            end)
            victim:sendEvent('fixActorFight', false)
        end
        if victim and victim:isValid() then victim:remove() coroutine.yield() end
        victim = spawnDummy('STEADY-02 walker', util.vector3(0, 600, 0), 'iron longsword')
        victim:sendEvent('fixActorHealth', 100)
        powerAndStand(victim, 'STEADY-02 walker')
        waitSeconds(0.5)
        sample('walk', 4.0, function() victim:sendEvent('fixActorMove', 1) end)
        victim:sendEvent('fixActorMove', 0)
        sample('stand', 3.0, function() end)
    end)
    world.players[1]:sendEvent('fixWoundCamera', {})
    if victim and victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)

-- REACT-04 (RXN-02 M6: the interrupt and hyper armour, never measured in the
-- game): a dummy armed with an iron longsword fights the player with its own
-- AI (its attacks on the wheel with HSC_NPC) and
-- takes a blow to the chest or head, light/solid/heavy, either early in its
-- wind-up or in its cut. Design: a wind-up breaks from a flinch up, a
-- committed cut only from a stagger. REACT04 rows: the stage when struck,
-- whether a cut followed a wind-up blow, the highest stage after, and the
-- RXN2 blow line (tier, interrupt) from the log (tools/body/analysis/react04.py).
-- FIST-01 (user 10-11: "use wheel for both dagger and fists"): bare fists on
-- the cut wheel (OPENMW_RAGDOLL_HSC_FISTS=1). The player punches a standing
-- dummy at fist reach: clicks (the straight), then a drag on each punch line
-- (wind-up one way, a fast stroke back = the punch). FIST01_BEGIN/END frame
-- each attempt; the blows are the RAGDOLL_IMPACT lines by the player between
-- them (tools: the session's connect_gz-style count).
registerTest('FIST-01 punches on the wheel', 919991, function()
    local player = placePlayer()
    local victim
    local attempts = 0
    local savedReaction = ragdoll.reactionScale()
    local ok, message = pcall(function()
        ragdoll.setPlayerAlwaysHits(true)
        player:sendEvent('fixPlayerConfigure', { sequence = 491, firstPerson = true })
        waitFor('player loadout', function() return playerReadySequence == 491 end)
        waitSeconds(1)
        powerAndStand(player, 'FIST-01 player')
        ragdoll.reactionScale(0)
        ragdoll.prepareMouseInput()
        local function drag(x, y, seconds)
            local started, sx, sy = core.getSimulationTime(), 0, 0
            waitSeconds(seconds, function()
                local f = math.min(1, (core.getSimulationTime() - started) / seconds)
                local vx, vy = math.floor(x * f), math.floor(y * f)
                ragdoll.injectMouseInput(vx - sx, vy - sy)
                sx, sy = vx, vy
            end)
            ragdoll.injectMouseInput(x - sx, y - sy)
        end
        victim = spawnDummy('FIST-01 target', util.vector3(0, 200, 0), false)
        victim:sendEvent('fixActorHealth', 1000)
        powerAndStand(victim, 'FIST-01 target')
        local function place()
            local p = ragdoll.getState(player)
            local shoulder = body(p, 'right_upper_arm').position
            local elbow = body(p, 'right_forearm').position
            local hand = body(p, 'right_hand').position
            local reach = (elbow - shoulder):length() + (hand - elbow):length()
            local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
            placeVictimPoint(victim, function()
                local vs = ragdoll.getState(victim)
                return vs and body(vs, 'spine2').position or victim.position
            end, pivot + util.vector3(0, reach + 8, 0), util.transform.rotateZ(math.pi))
            waitSeconds(0.5)
        end
        local attemptsList = {
            { 'click', 0, 0 }, { 'click', 0, 0 }, { 'click', 0, 0 },
            { 'cross', 0, -1 }, { 'hook', 1, 0 }, { 'overhand', 0, 1 },
            { 'uppercut', 1, -1 }, { 'body', -1, -1 }, { 'backfist', -1, 0 },
        }
        for _, attempt in ipairs(attemptsList) do
            if not victim:isValid() or types.Actor.isDead(victim) then break end
            place()
            attempts = attempts + 1
            print(string.format('FIST01_BEGIN kind=%s', attempt[1]))
            if attempt[1] == 'click' then
                ragdoll.injectMouseInput(0, 0, true, 1)
                ragdoll.injectMouseInput(0, 0, false, 1)
                waitSeconds(1.0)
            else
                ragdoll.injectMouseInput(0, 0, true, 1)
                waitSeconds(0.2)
                drag(attempt[2] * HSC_WIND, attempt[3] * HSC_WIND, 0.35)
                drag(-attempt[2] * HSC_CUT, -attempt[3] * HSC_CUT, 0.15)
                waitSeconds(0.25)
                ragdoll.injectMouseInput(0, 0, false, 1)
                waitSeconds(0.8)
            end
            print(string.format('FIST01_END kind=%s', attempt[1]))
            waitSeconds(0.3)
        end
    end)
    ragdoll.injectMouseInput(0, 0, false)
    ragdoll.reactionScale(savedReaction)
    ragdoll.setPlayerAlwaysHits(false)
    ragdoll.setPowered(player, false)
    if victim and victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)

-- PUNCH-01 (user 10-30: fists and daggers "hits dont connect because we cant
-- really aim the punch and the click makes the fist go forward but its not a
-- good range punch and it returns too quickly"): third person, the crosshair
-- put on a standing dummy's chest, head or belly, straight ahead, further out
-- and to either side, then one click. PUNCH01 rows: how far the hand got
-- along the aim (from the shoulders' middle; `reach` = the arm), how close
-- it came to the aimed point, when it was furthest out, how long it stayed
-- within 5 units of that, and the damage.
local punchPlacements = {
    { name = 'mid', part = 'spine2', depth = 8, side = 0 },
    { name = 'far', part = 'spine2', depth = 25, side = 0 },
    { name = 'left', part = 'spine2', depth = 8, side = -25 },
    { name = 'right', part = 'spine2', depth = 8, side = 25 },
    { name = 'head', part = 'head', depth = 8, side = 0 },
    { name = 'belly', part = 'spine', depth = 8, side = 0 },
}
for index, weapon in ipairs({ 'fists', 'iron dagger' }) do
    registerTest('PUNCH-01 aimed clicks with ' .. weapon, 919970 + index, function()
        local player = placePlayer()
        local victim
        local savedReaction = ragdoll.reactionScale()
        local ok, message = pcall(function()
            ragdoll.setPlayerAlwaysHits(true)
            local armed = weapon ~= 'fists'
            if armed and not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = armed and weapon or nil, sequence = 470 + index,
                firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 470 + index end)
            waitSeconds(1)
            powerAndStand(player, 'PUNCH-01 player')
            ragdoll.reactionScale(0)
            ragdoll.prepareMouseInput()
            victim = spawnDummy('PUNCH-01 target', util.vector3(0, 200, 0), false)
            victim:sendEvent('fixActorHealth', 1000)
            powerAndStand(victim, 'PUNCH-01 target')
            local function damageTotal()
                local total = 0
                for _, value in pairs(ragdoll.getLimbDamage(victim)) do total = total + value end
                return total
            end
            local hits, attempts = 0, 0
            for _, placement in ipairs(punchPlacements) do
                for _ = 1, 2 do
                    local p = ragdoll.getState(player)
                    local shoulder = body(p, 'right_upper_arm').position
                    local elbow = body(p, 'right_forearm').position
                    local hand = body(p, 'right_hand').position
                    local reach = (elbow - shoulder):length() + (hand - elbow):length()
                    local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
                    local d = reach + placement.depth
                    local a = math.rad(placement.side)
                    placeVictimPoint(victim, function()
                        local vs = ragdoll.getState(victim)
                        return vs and body(vs, 'spine2').position or victim.position
                    end, pivot + util.vector3(math.sin(a) * d, math.cos(a) * d, 0), util.transform.rotateZ(math.pi + a))
                    -- A dummy knocked far off is rebuilt (RAGDOLL_TELEPORT): wait
                    -- for its body, and skip a placement that did not hold.
                    local vs = waitFor('PUNCH-01 target body', function() return ragdoll.getState(victim) end, 5)
                    local aimed = body(vs, placement.part).position
                    local placedAt = (body(vs, 'spine2').position - pivot)
                    if math.abs(math.sqrt(placedAt.x ^ 2 + placedAt.y ^ 2) - d) > 20 then
                        print(string.format('PUNCH01 weapon=%s at=%s placement_failed=%.1f', weapon, placement.name,
                            math.sqrt(placedAt.x ^ 2 + placedAt.y ^ 2)))
                        goto continue
                    end
                    contactAimReady = false
                    player:sendEvent('fixAimAt', aimed)
                    pcall(waitFor, 'PUNCH-01 aim', function() return contactAimReady end, 4)
                    waitSeconds(0.3)
                    p = ragdoll.getState(player)
                    pivot = (body(p, 'left_upper_arm').position + body(p, 'right_upper_arm').position) * 0.5
                    vs = ragdoll.getState(victim)
                    aimed = body(vs, placement.part).position
                    local aim = (aimed - pivot):normalize()
                    local rest = (body(p, 'right_hand').position - pivot):dot(aim)
                    local before = damageTotal()
                    local started = core.getSimulationTime()
                    ragdoll.injectMouseInput(0, 0, true, 1)
                    waitSeconds(0.08)
                    ragdoll.injectMouseInput(0, 0, false, 1)
                    local most, mostAt, closest, samples = -1e9, 0, 1e9, {}
                    waitSeconds(1.4, function()
                        local s = ragdoll.getState(player)
                        local t = ragdoll.getState(victim)
                        if not s or not t then return end
                        local h = body(s, 'right_hand').position
                        local out = (h - pivot):dot(aim)
                        local now = core.getSimulationTime() - started
                        samples[#samples + 1] = { now, out }
                        if out > most then most, mostAt = out, now end
                        closest = math.min(closest, (h - body(t, placement.part).position):length())
                    end)
                    local held = 0
                    for i = 2, #samples do
                        if samples[i][2] >= most - 5 then held = held + (samples[i][1] - samples[i - 1][1]) end
                    end
                    local damage = damageTotal() - before
                    attempts = attempts + 1
                    if damage > 0 then hits = hits + 1 end
                    print(string.format('PUNCH01 weapon=%s at=%s reach=%.1f target=%.1f rest=%.1f out=%.1f '
                        .. 'gain=%.1f closest=%.1f peak_s=%.2f held_s=%.2f hit=%d damage=%.2f',
                        weapon, placement.name, reach, (aimed - pivot):length(), rest, most, most - rest, closest,
                        mostAt, held, damage > 0 and 1 or 0, damage))
                    waitSeconds(0.6)
                    ::continue::
                end
            end
            print(string.format('PUNCH01_TOTAL weapon=%s hits=%d of %d', weapon, hits, attempts))
        end)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.reactionScale(savedReaction)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if victim and victim:isValid() then victim:remove() end
        if not ok then error(message, 0) end
    end)
end

-- TUNE-01 (live tuning registry): the KINETIC page's override reaches a
-- cached switch on the next physics step. The same solid chest blow lands
-- with the hit push off (REACTION_V2_PUSH 0) and strong (3, cap 1500); the
-- chest must travel clearly further with the strong push, and dropping the
-- override must give the launcher's value back.
registerTest('TUNE-01 live tuning reaches the hit push', 919990, function()
    placePlayer()
    local victim
    local ok, message = pcall(function()
        local name = 'OPENMW_RAGDOLL_REACTION_V2_PUSH'
        local before = ragdoll.tuningText(name)
        local travel = {}
        for _, setting in ipairs({ '0,560', '3,1500' }) do
            if victim and victim:isValid() then victim:remove() coroutine.yield() end
            ragdoll.setTuning(name, setting)
            victim = spawnDummy('TUNE-01 dummy', util.vector3(0, 320, 0))
            victim:sendEvent('fixActorHealth', 100)
            local s0 = powerAndStand(victim, 'TUNE-01 dummy')
            waitSeconds(1.0)
            if ragdoll.tuningText(name) ~= setting then
                error('TUNE-01: the override did not apply: ' .. tostring(ragdoll.tuningText(name)))
            end
            s0 = ragdoll.getState(victim)
            local chest0 = body(s0, 'spine2').position
            ragdoll.applyBlow(victim, 'spine2', util.vector3(0, 1, 0), 800, 1.6, 5)
            local most = 0
            waitSeconds(0.6, function()
                local st = ragdoll.getState(victim)
                if not st then return end
                local chest = body(st, 'spine2').position
                local moved = util.vector3(chest.x - chest0.x, chest.y - chest0.y, 0)
                most = math.max(most, moved:length())
            end)
            travel[#travel + 1] = most
            print(string.format('TUNE01 push=%s chest_travel=%.1f', setting, most))
        end
        ragdoll.setTuning(name, nil)
        waitSeconds(0.2)
        if ragdoll.tuningText(name) ~= before then
            error('TUNE-01: dropping the override did not restore ' .. tostring(before))
        end
        if not (travel[2] > travel[1] + 4) then
            error(string.format('TUNE-01: strong push %.1f u is not clearly beyond no push %.1f u', travel[2], travel[1]))
        end
    end)
    ragdoll.setTuning('OPENMW_RAGDOLL_REACTION_V2_PUSH', nil)
    if victim and victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)

registerTest('REACT-04 blows on an attacking dummy', 919250, function()
    local player = placePlayer()
    local victim
    local ok, message = pcall(function()
        local strengths = { { 'light', 300, 1.6, 5 }, { 'solid', 800, 1.6, 15 }, { 'heavy', 1600, 2.4, 35 } }
        for _, part in ipairs({ 'spine2', 'head' }) do
            for _, phase in ipairs({ 'wind', 'cut' }) do
                for _, st in ipairs(strengths) do
                    if victim and victim:isValid() then victim:remove() coroutine.yield() end
                    victim = spawnDummy('REACT-04 dummy', util.vector3(0, 320, 0), 'iron longsword')
                    victim:sendEvent('fixActorHealth', 100)
                    powerAndStand(victim, 'REACT-04 dummy')
                    waitSeconds(1.0)
                    local function stage()
                        local vs = ragdoll.getState(victim)
                        return vs and vs.hsc and vs.hsc.stage or -1
                    end
                    victim:sendEvent('fixActorFight', true)
                    ragdoll.setCombatTarget(victim, player)
                    local want = phase == 'wind' and 1 or 2
                    local found = pcall(waitFor, 'REACT-04 stage', function()
                        return stage() == want
                    end, 10)
                    if phase == 'wind' then waitSeconds(0.08) end
                    local at = stage()
                    print(string.format('REACT04_BEGIN part=%s phase=%s strength=%s', part, phase, st[1]))
                    -- From the front: from the player toward the dummy.
                    local away = victim.position - player.position
                    away = util.vector3(away.x, away.y, 0)
                    away = away:length() > 1 and away:normalize() or util.vector3(0, 1, 0)
                    ragdoll.applyBlow(victim, part, away, st[2], st[3], st[4])
                    local maxAfter, cutAfter = at, false
                    waitSeconds(1.2, function()
                        local s2 = stage()
                        maxAfter = math.max(maxAfter, s2)
                        if s2 == 2 then cutAfter = true end
                    end)
                    print(string.format('REACT04 part=%s phase=%s strength=%s reached=%s stage_at_blow=%d cut_after=%s max_after=%d',
                        part, phase, st[1], tostring(found), at, tostring(cutAfter), maxAfter))
                    waitSeconds(0.8)
                end
            end
        end
    end)
    if victim and victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)

-- GUARDCUT-01 (build 63 play: 12 of 12 "no hit" bookmarks were cuts that got
-- past a raised guard to the thigh, calf, neck or chest and still counted as
-- blocked): a longsword dummy raises its guard as each cut starts; the player
-- cuts four lines at it (chop, forehand, rising cuts at both legs), twice
-- each. GUARDCUT01_BEGIN/END frame each cut with the damage dealt; the engine
-- logs RAGDOLL_DEFENSE (outcome, cover) and RAGDOLL_PARRY (blade clashes)
-- between them (tools/body/analysis/guardcut01.py). Compare
-- OPENMW_RAGDOLL_GUARD_COVER=0.
for index, weapon in ipairs({ 'iron claymore', 'iron longsword' }) do
    registerTest('GUARDCUT-01 cuts past a raised guard with ' .. weapon, 919200 + index, function()
        local player = placePlayer()
        local victim
        local savedReaction = ragdoll.reactionScale()
        local ok, message = pcall(function()
            ragdoll.setPlayerAlwaysHits(true)
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 470 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 470 + index end)
            waitSeconds(1)
            powerAndStand(player, 'GUARDCUT-01 player')
            cameraReadyMode = nil
            player:sendEvent('fixPlayerCamera', true)
            waitFor('GUARDCUT-01 first-person camera', function() return cameraReadyMode == true end)
            ragdoll.reactionScale(0)
            ragdoll.prepareMouseInput()
            local function damageTotal()
                local total = 0
                if victim and victim:isValid() then
                    for _, value in pairs(ragdoll.getLimbDamage(victim)) do total = total + value end
                end
                return total
            end
            local function drag(x, y, seconds)
                local started, sx, sy = core.getSimulationTime(), 0, 0
                waitSeconds(seconds, function()
                    local f = math.min(1, (core.getSimulationTime() - started) / seconds)
                    local vx, vy = math.floor(x * f), math.floor(y * f)
                    ragdoll.injectMouseInput(vx - sx, vy - sy)
                    sx, sy = vx, vy
                end)
                ragdoll.injectMouseInput(x - sx, y - sy)
            end
            local d = 0.7071
            local lines = {
                { 'chop', 0, -1 }, { 'forehand', 1, 0 },
                { 'rising-left', d, d }, { 'rising-right', -d, d },
            }
            for rep = 1, 2 do
                for _, line in ipairs(lines) do
                    if not victim or not victim:isValid() or types.Actor.isDead(victim) or not ragdoll.getState(victim) then
                        if victim and victim:isValid() then victim:remove() coroutine.yield() end
                        victim = spawnDummy('GUARDCUT-01 guard', util.vector3(0, 200, 0), 'iron longsword')
                        powerAndStand(victim, 'GUARDCUT-01 guard')
                    end
                    local p = ragdoll.getState(player)
                    local shoulder = body(p, 'right_upper_arm').position
                    local elbow = body(p, 'right_forearm').position
                    local hand = body(p, 'right_hand').position
                    local reach = (elbow - shoulder):length() + (hand - elbow):length()
                    local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
                    placeVictimPoint(victim, function()
                        local vs = ragdoll.getState(victim)
                        return vs and body(vs, 'spine2').position or victim.position
                    end, pivot + util.vector3(0, reach + p.weapon.length * 0.55, 0), util.transform.rotateZ(math.pi))
                    waitSeconds(0.6)
                    local before = damageTotal()
                    print(string.format('GUARDCUT01_BEGIN weapon=%s line=%s rep=%d', weapon, line[1], rep))
                    ragdoll.injectMouseInput(0, 0, true, 1)
                    waitSeconds(0.25)
                    drag(line[2] * HSC_WIND, line[3] * HSC_WIND, 0.4)
                    -- Raised as the cut starts, as a duellist does at a wind-up.
                    ragdoll.setGuard(victim, true)
                    drag(-line[2] * HSC_CUT, -line[3] * HSC_CUT, 0.2)
                    waitSeconds(0.3)
                    ragdoll.injectMouseInput(0, 0, false, 1)
                    waitSeconds(0.6)
                    pcall(waitFor, 'GUARDCUT-01 withdrawal', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                    print(string.format('GUARDCUT01_END weapon=%s line=%s rep=%d damage=%.2f', weapon, line[1], rep,
                        damageTotal() - before))
                    if victim and victim:isValid() then ragdoll.setGuard(victim, false) end
                    waitSeconds(0.6)
                end
            end
        end)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.reactionScale(savedReaction)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if victim and victim:isValid() then victim:remove() end
        if not ok then error(message, 0) end
    end)
end

-- FPV-01 (build 59 bookmark 15: "keep the first person view unblocked by the
-- player's own arms"): first-person screenshots in the guard, wound up and
-- mid-cut, per weapon (userdata/screenshots; FPV01 lines name each shot).
for index, weapon in ipairs({ 'iron longsword', 'iron claymore' }) do
    registerTest('FPV-01 first-person view with ' .. weapon, 918350 + index, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 450 + index, firstPerson = true })
            waitFor('player loadout', function() return playerReadySequence == 450 + index end)
            waitSeconds(1)
            powerAndStand(player, 'FPV-01 player')
            cameraReadyMode = nil
            player:sendEvent('fixPlayerCamera', true)
            waitFor('FPV-01 first-person camera', function() return cameraReadyMode == true end)
            ragdoll.prepareMouseInput()
            local shot = 0
            local function snap(tag)
                coroutine.yield()
                shot = shot + 1
                print(string.format('FPV01 shot=%d weapon=%s moment=%s', shot, weapon, tag))
                ragdoll.captureScreenshot()
                waitSeconds(0.1)
            end
            local function drag(x, seconds)
                local started, sent = core.getSimulationTime(), 0
                waitSeconds(seconds, function()
                    local f = math.min(1, (core.getSimulationTime() - started) / seconds)
                    local v = math.floor(x * f)
                    ragdoll.injectMouseInput(v - sent, 0)
                    sent = v
                end)
                ragdoll.injectMouseInput(x - sent, 0)
            end
            waitSeconds(1.5)
            snap('guard')
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitSeconds(0.25)
            drag(HSC_WIND, 0.4)
            snap('wound up')
            drag(-1200, 0.1)
            waitSeconds(0.05)
            snap('mid cut')
            ragdoll.injectMouseInput(0, 0, false, 1)
            waitSeconds(1.5)
            snap('after cut')
        end)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end)
end

-- CONNECT-01 (build 61 bookmarks: 12 of 20 "didn't connect"): two cases
-- from the recording, per weapon. "rest": the foe point-blank, the guard's
-- blade resting on it, then a full forehand cut. "chain": one hold, a
-- forehand cut dragged on into the backhand and cut back - two cuts that
-- should both strike. CONNECT01_BEGIN/END frame each attempt; the blows are
-- the RAGDOLL_IMPACT lines between them (tools/body/analysis/connect01.py).
-- Compare OPENMW_RAGDOLL_CONNECT=0.
for index, weapon in ipairs({ 'iron longsword', 'iron claymore', 'iron warhammer', 'iron dagger' }) do
    registerTest('CONNECT-01 resting blade and chained cuts with ' .. weapon, 918340 + index, function()
        local player = placePlayer()
        local victim
        local savedReaction = ragdoll.reactionScale()
        local ok, message = pcall(function()
            ragdoll.setPlayerAlwaysHits(true)
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 440 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 440 + index end)
            waitSeconds(1)
            powerAndStand(player, 'CONNECT-01 player')
            cameraReadyMode = nil
            player:sendEvent('fixPlayerCamera', true)
            waitFor('CONNECT-01 first-person camera', function() return cameraReadyMode == true end)
            ragdoll.reactionScale(0)
            ragdoll.prepareMouseInput()
            local function drag(x, seconds)
                local started, sent = core.getSimulationTime(), 0
                waitSeconds(seconds, function()
                    local f = math.min(1, (core.getSimulationTime() - started) / seconds)
                    local v = math.floor(x * f)
                    ragdoll.injectMouseInput(v - sent, 0)
                    sent = v
                end)
                ragdoll.injectMouseInput(x - sent, 0)
            end
            for rep = 1, 3 do
                for _, mode in ipairs({ 'rest', 'chain' }) do
                    -- A claymore kills the dummy in a few blows: a new one.
                    local function fresh()
                        return victim and victim:isValid() and not types.Actor.isDead(victim) and ragdoll.getState(victim)
                    end
                    if not fresh() then
                        if victim and victim:isValid() then victim:remove() coroutine.yield() end
                        victim = spawnDummy('CONNECT-01 target', util.vector3(0, 200, 0), false)
                        powerAndStand(victim, 'CONNECT-01 target')
                    end
                    local p = ragdoll.getState(player)
                    local shoulder = body(p, 'right_upper_arm').position
                    local elbow = body(p, 'right_forearm').position
                    local hand = body(p, 'right_hand').position
                    local reach = (elbow - shoulder):length() + (hand - elbow):length()
                    local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
                    -- Point-blank for "rest" (the guard's blade lies in the foe),
                    -- the blade's sweet spot for "chain".
                    local d = mode == 'rest' and (reach + p.weapon.length * 0.25) or (reach + p.weapon.length * 0.55)
                    placeVictimPoint(victim, function()
                        local vs = ragdoll.getState(victim)
                        return vs and body(vs, 'spine2').position or victim.position
                    end, pivot + util.vector3(0, d, 0), util.transform.rotateZ(math.pi))
                    waitSeconds(0.5)
                    print(string.format('CONNECT01_BEGIN weapon=%s mode=%s rep=%d', weapon, mode, rep))
                    ragdoll.injectMouseInput(0, 0, true, 1)
                    waitSeconds(0.25)
                    drag(HSC_WIND, 0.4)
                    drag(-HSC_CUT, 0.2)
                    if mode == 'chain' then
                        drag(-HSC_WIND, 0.4)
                        drag(HSC_CUT, 0.2)
                    end
                    waitSeconds(0.2)
                    ragdoll.injectMouseInput(0, 0, false, 1)
                    waitSeconds(0.6)
                    pcall(waitFor, 'CONNECT-01 withdrawal', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                    print(string.format('CONNECT01_END weapon=%s mode=%s rep=%d', weapon, mode, rep))
                    waitSeconds(0.6)
                end
            end
        end)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.reactionScale(savedReaction)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if victim and victim:isValid() then victim:remove() end
        if not ok then error(message, 0) end
    end)
end

-- CONNECT-02 (RXN-02 V2, 2026-10-07: "the target stays closer, and a next
-- cut that starts with the blade resting in the body can fail"; video
-- feedback's top complaint, point-blank misses): CONNECT-01 runs with the
-- reactions off and re-places the foe before each cut. Here the reactions
-- are on and the foe is placed once per series, point-blank (`near` blade
-- lengths past the arm's reach), then six alternating cuts follow without
-- re-placing it, a press per cut as in play. CONNECT02_BEGIN/END frame each cut with the
-- foe's distance; the blows are the RAGDOLL_IMPACT lines between them
-- (tools/body/analysis/connect02.py). Every cut should strike.
for index, weapon in ipairs({ 'iron longsword', 'iron claymore', 'iron dagger' }) do
    registerTest('CONNECT-02 point-blank cut series with ' .. weapon, 918350 + index, function()
        local player = placePlayer()
        local victim
        local ok, message = pcall(function()
            ragdoll.setPlayerAlwaysHits(true)
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 450 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 450 + index end)
            waitSeconds(1)
            powerAndStand(player, 'CONNECT-02 player')
            cameraReadyMode = nil
            player:sendEvent('fixPlayerCamera', true)
            waitFor('CONNECT-02 first-person camera', function() return cameraReadyMode == true end)
            ragdoll.prepareMouseInput()
            local function drag(x, seconds)
                local started, sent = core.getSimulationTime(), 0
                waitSeconds(seconds, function()
                    local f = math.min(1, (core.getSimulationTime() - started) / seconds)
                    local v = math.floor(x * f)
                    ragdoll.injectMouseInput(v - sent, 0)
                    sent = v
                end)
                ragdoll.injectMouseInput(x - sent, 0)
            end
            for series, near in ipairs({ 0.1, 0.25, 0.4 }) do
                if victim and victim:isValid() then victim:remove() coroutine.yield() end
                victim = spawnDummy('CONNECT-02 target', util.vector3(0, 200, 0), false)
                powerAndStand(victim, 'CONNECT-02 target')
                local p = ragdoll.getState(player)
                local shoulder = body(p, 'right_upper_arm').position
                local elbow = body(p, 'right_forearm').position
                local hand = body(p, 'right_hand').position
                local reach = (elbow - shoulder):length() + (hand - elbow):length()
                local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
                placeVictimPoint(victim, function()
                    local vs = ragdoll.getState(victim)
                    return vs and body(vs, 'spine2').position or victim.position
                end, pivot + util.vector3(0, reach + p.weapon.length * near, 0), util.transform.rotateZ(math.pi))
                waitSeconds(0.5)
                local side = 1
                for cut = 1, 6 do
                    if not victim:isValid() or types.Actor.isDead(victim) then break end
                    local ps, vs = ragdoll.getState(player), ragdoll.getState(victim)
                    local gap = (ps and vs) and (body(vs, 'spine2').position - body(ps, 'spine2').position):length() or -1
                    print(string.format('CONNECT02_BEGIN weapon=%s near=%.2f cut=%d gap=%.1f', weapon, near, cut, gap))
                    -- A press per cut, forehand and backhand in turn (the play
                    -- recordings: one press per cut; a held back-and-forth
                    -- drag stays one cut on the wheel).
                    ragdoll.injectMouseInput(0, 0, true, 1)
                    waitSeconds(0.1)
                    drag(side * HSC_WIND, 0.4)
                    drag(-side * HSC_CUT, 0.2)
                    waitSeconds(0.15)
                    ragdoll.injectMouseInput(0, 0, false, 1)
                    waitSeconds(0.25)
                    print(string.format('CONNECT02_END weapon=%s near=%.2f cut=%d', weapon, near, cut))
                    pcall(waitFor, 'CONNECT-02 withdrawal', function() return ragdoll.getSwingSpeed(player) == nil end, 3)
                    side = -side
                end
            end
        end)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if victim and victim:isValid() then victim:remove() end
        if not ok then error(message, 0) end
    end)
end

-- NPCHIT-02 (AI-02 items 4 and 6, 10-08): a Hlaalu guard (steel longsword,
-- AI combat) attacks the player, who holds the right-click guard in a zone -
-- none, high, left, low - 20 s each. NPCHIT02_BEGIN/END frame each zone; the
-- engine logs the NPC's line choice (RAGDOLL_AI2 what=line, AI_LINES) and its
-- cuts (RAGDOLL_AI_CUT, AI_AIM_TRACE), the player's defence (RAGDOLL_DEFENSE,
-- RAGDOLL_PARRY) and the damage taken (tools/body/analysis/npchit02.py).
registerTest('NPCHIT-02 an NPC attacks a guarding player', 919270, function()
    local weapon = 'iron longsword'
    local player = placePlayer()
    local attacker
    local RIGHT = 3
    local ok, message = pcall(function()
        if not types.Actor.inventory(player):find(weapon) then
            world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 480, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 480 end)
        waitSeconds(1)
        powerAndStand(player, 'NPCHIT-02 player')
        cameraReadyMode = nil
        player:sendEvent('fixPlayerCamera', true)
        waitFor('NPCHIT-02 first-person camera', function() return cameraReadyMode == true end)
        ragdoll.prepareMouseInput()
        local record = world.createRecord(types.NPC.createRecordDraft({
            template = types.NPC.record('hlaalu guard'), name = 'NPCHIT-02 guard',
            isEssential = false, isPersistent = false, isRespawning = false,
        }))
        attacker = world.createObject(record.id)
        if not types.Actor.inventory(attacker):find('steel longsword') then
            world.createObject('steel longsword', 1):moveInto(types.Actor.inventory(attacker))
        end
        attacker:teleport(testArea.cell, testArea.player + util.vector3(0, 160, 0), { onGround = true })
        attacker:addScript('stance.lua')
        coroutine.yield()
        attacker:sendEvent('fixActorConfigure', { weaponStance = 'steel longsword' })
        waitFor('NPCHIT-02 guard stance', function() return readyActors[attacker.id] ~= nil end, 10)
        attacker:sendEvent('fixActorHealth', 5000)
        powerAndStand(attacker, 'NPCHIT-02 guard')
        attacker:sendEvent('fixActorFight', true)
        ragdoll.setCombatTarget(attacker, player)
        for _, zone in ipairs({ { 'none', nil, nil }, { 'high', 0, -500 }, { 'left', -500, 0 }, { 'low', 0, 500 } }) do
            if zone[2] then
                ragdoll.injectMouseInput(0, 0, true, RIGHT)
                waitSeconds(0.1)
                ragdoll.injectMouseInput(zone[2], zone[3])
                waitSeconds(0.3)
            end
            local st = ragdoll.getState(player)
            print(string.format('NPCHIT02_BEGIN zone=%s read=%s active=%s', zone[1], tostring(st.guardZone),
                tostring(st.guardActive)))
            local lastRefresh = core.getSimulationTime()
            waitSeconds(20, function()
                local now = core.getSimulationTime()
                if now - lastRefresh >= 2 then
                    lastRefresh = now
                    ragdoll.setCombatTarget(attacker, player)
                end
                -- The player turns to face the NPC, as a guarding player does.
                local to = attacker.position - player.position
                local err = math.atan2(to.x, to.y) - player.rotation:getYaw()
                err = (err + math.pi) % (2 * math.pi) - math.pi
                player:sendEvent('fixPlayerControls', { forward = 0, side = 0, run = 0,
                    yaw = math.max(-0.08, math.min(0.08, err * 0.3)) })
            end)
            player:sendEvent('fixPlayerControls', nil)
            print(string.format('NPCHIT02_END zone=%s', zone[1]))
            if zone[2] then
                ragdoll.injectMouseInput(0, 0, false, RIGHT)
                waitSeconds(0.5)
            end
        end
    end)
    ragdoll.injectMouseInput(0, 0, false, RIGHT)
    ragdoll.setPowered(player, false)
    if attacker and attacker:isValid() then attacker:remove() end
    if not ok then error(message, 0) end
end)

-- AI02-CHARGE (AI-02 item 2, user 10-07: "they should walk backwards if the
-- player charges at them and they are feeling threatened"): the player runs at
-- an NPC in AI combat from 400 units and clicks an attack once 70 units away
-- (or after 3 s), six times, at a commoner (the player's template) and at a
-- Hlaalu guard. AI02_CHARGE_BEGIN/END frame each charge; the engine logs the
-- NPC's read (RAGDOLL_AI2 what=threat, AI_THREAT) and the blows
-- (RAGDOLL_DAMAGE; tools/body/analysis/charge.py).
registerTest('AI02-CHARGE the player charges an NPC', 919280, function()
    local weapon = 'iron longsword'
    local player = placePlayer()
    local npc
    local ok, message = pcall(function()
        if not types.Actor.inventory(player):find(weapon) then
            world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 490, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 490 end)
        waitSeconds(1)
        powerAndStand(player, 'AI02-CHARGE player')
        cameraReadyMode = nil
        player:sendEvent('fixPlayerCamera', true)
        waitFor('AI02-CHARGE first-person camera', function() return cameraReadyMode == true end)
        ragdoll.prepareMouseInput()
        for _, profile in ipairs({ { 'commoner', types.NPC.record(player), 'iron longsword' },
            { 'guard', types.NPC.record('hlaalu guard'), 'steel longsword' } }) do
            local record = world.createRecord(types.NPC.createRecordDraft({
                template = profile[2], name = 'AI02-CHARGE ' .. profile[1],
                isEssential = false, isPersistent = false, isRespawning = false,
            }))
            npc = world.createObject(record.id)
            if not types.Actor.inventory(npc):find(profile[3]) then
                world.createObject(profile[3], 1):moveInto(types.Actor.inventory(npc))
            end
            npc:teleport(testArea.cell, testArea.player + util.vector3(0, 400, 0), { onGround = true })
            npc:addScript('stance.lua')
            coroutine.yield()
            npc:sendEvent('fixActorConfigure', { weaponStance = profile[3] })
            waitFor('AI02-CHARGE npc stance', function() return readyActors[npc.id] ~= nil end, 10)
            npc:sendEvent('fixActorHealth', 5000)
            powerAndStand(npc, 'AI02-CHARGE npc')
            npc:sendEvent('fixActorFight', true)
            for n = 1, 6 do
                -- Apart again: the player back at the start, the NPC 400 ahead.
                player:sendEvent('fixPlayerControls', nil)
                player:sendEvent('fixPlayerMove', 0)
                -- Out of the fight while it is set up (it walked in otherwise and
                -- the charge began inside measure), back in as the player runs.
                npc:sendEvent('fixActorFight', false)
                ragdoll.setPowered(npc, false)
                coroutine.yield()
                npc:teleport(testArea.cell, player.position + util.vector3(0, 400, 0),
                    { onGround = true, rotation = util.transform.rotateZ(math.pi) })
                coroutine.yield()
                powerAndStand(npc, 'AI02-CHARGE npc')
                waitSeconds(0.5)
                print(string.format('AI02_CHARGE_BEGIN npc=%s n=%d distance=%.0f', profile[1], n,
                    (npc.position - player.position):length()))
                npc:sendEvent('fixActorFight', true)
                ragdoll.setCombatTarget(npc, player)
                player:sendEvent('fixPlayerControls', { forward = 1, run = 1 })
                local started = core.getSimulationTime()
                waitFor('AI02-CHARGE closing', function()
                    local flat = npc.position - player.position
                    return util.vector3(flat.x, flat.y, 0):length() < 70 or core.getSimulationTime() - started > 3
                end, 5)
                ragdoll.injectMouseInput(0, 0, true, 1)
                waitSeconds(0.1)
                ragdoll.injectMouseInput(0, 0, false, 1)
                waitSeconds(0.4)
                player:sendEvent('fixPlayerControls', nil)
                player:sendEvent('fixPlayerMove', 0)
                waitSeconds(1.5)
                print(string.format('AI02_CHARGE_END npc=%s n=%d', profile[1], n))
            end
            ragdoll.setPowered(npc, false)
            npc:remove()
            npc = nil
            waitSeconds(0.5)
        end
    end)
    player:sendEvent('fixPlayerControls', nil)
    player:sendEvent('fixPlayerMove', 0)
    ragdoll.injectMouseInput(0, 0, false, 1)
    ragdoll.setPowered(player, false)
    if npc and npc:isValid() then npc:remove() end
    if not ok then error(message, 0) end
end)

-- RIPOSTE-01 (AI-02 item 5, 10-08): the player cuts a Hlaalu guard in AI
-- combat again and again (overhead chops and forehands from the wheel, 24
-- strokes); its guard reads them (AI_READ) and a won parry (RAGDOLL_CLASH
-- outcome=knockaway parry=it) opens the riposte window (AI_RIPOSTE).
-- RIPOSTE01_BEGIN/END frame the run; tools/body/analysis/riposte.py counts
-- the knockaways it won and its blows on the player within 0.8 s after.
registerTest('RIPOSTE-01 an NPC answers its parry', 919290, function()
    local weapon = 'iron longsword'
    local player = placePlayer()
    local npc
    local ok, message = pcall(function()
        ragdoll.setPlayerAlwaysHits(true)
        if not types.Actor.inventory(player):find(weapon) then
            world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 495, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 495 end)
        waitSeconds(1)
        powerAndStand(player, 'RIPOSTE-01 player')
        cameraReadyMode = nil
        player:sendEvent('fixPlayerCamera', true)
        waitFor('RIPOSTE-01 first-person camera', function() return cameraReadyMode == true end)
        ragdoll.prepareMouseInput()
        local record = world.createRecord(types.NPC.createRecordDraft({
            template = types.NPC.record('hlaalu guard'), name = 'RIPOSTE-01 guard',
            isEssential = false, isPersistent = false, isRespawning = false,
        }))
        npc = world.createObject(record.id)
        if not types.Actor.inventory(npc):find('steel longsword') then
            world.createObject('steel longsword', 1):moveInto(types.Actor.inventory(npc))
        end
        npc:teleport(testArea.cell, testArea.player + util.vector3(0, 110, 0),
            { onGround = true, rotation = util.transform.rotateZ(math.pi) })
        npc:addScript('stance.lua')
        coroutine.yield()
        npc:sendEvent('fixActorConfigure', { weaponStance = 'steel longsword' })
        waitFor('RIPOSTE-01 guard stance', function() return readyActors[npc.id] ~= nil end, 10)
        npc:sendEvent('fixActorHealth', 5000)
        powerAndStand(npc, 'RIPOSTE-01 guard')
        npc:sendEvent('fixActorFight', true)
        ragdoll.setCombatTarget(npc, player)
        local function drag(x, y, seconds)
            local started, sentX, sentY = core.getSimulationTime(), 0, 0
            waitSeconds(seconds, function()
                local f = math.min(1, (core.getSimulationTime() - started) / seconds)
                local vx, vy = math.floor(x * f), math.floor(y * f)
                ragdoll.injectMouseInput(vx - sentX, vy - sentY)
                sentX, sentY = vx, vy
            end)
            ragdoll.injectMouseInput(x - sentX, y - sentY)
        end
        local function face()
            local to = npc.position - player.position
            local err = math.atan2(to.x, to.y) - player.rotation:getYaw()
            err = (err + math.pi) % (2 * math.pi) - math.pi
            player:sendEvent('fixPlayerControls', { forward = 0, side = 0, run = 0,
                yaw = math.max(-0.1, math.min(0.1, err * 0.5)) })
        end
        print('RIPOSTE01_BEGIN')
        for n = 1, 24 do
            ragdoll.setCombatTarget(npc, player)
            waitSeconds(0.4, face)
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitSeconds(0.15)
            if n % 2 == 1 then
                drag(0, -HSC_WIND, 0.35)
                drag(0, HSC_CUT, 0.2)
            else
                drag(HSC_WIND, 0, 0.35)
                drag(-HSC_CUT, 0, 0.2)
            end
            waitSeconds(0.2)
            ragdoll.injectMouseInput(0, 0, false, 1)
            waitSeconds(1.0, face)
        end
        print('RIPOSTE01_END')
    end)
    player:sendEvent('fixPlayerControls', nil)
    ragdoll.injectMouseInput(0, 0, false, 1)
    ragdoll.setPlayerAlwaysHits(false)
    ragdoll.setPowered(player, false)
    if npc and npc:isValid() then npc:remove() end
    if not ok then error(message, 0) end
end)

-- GUARD-02 (user 10-07, right-click guard: hold the right button to guard,
-- the mouse picks where - "how do we set a defensive stance to for example
-- parry an overhead chop?"; a quick tap is a timed parry). With
-- OPENMW_RAGDOLL_RMB_GUARD=1 and GUARD_ZONES=1.
-- Part 1, the player: the right button held, the mouse moved to each zone;
-- GUARD02 zone lines (guard up, the zone read, the hand against the chest in
-- the body frame) and a still per zone; a tap holds the guard a moment.
-- Part 2, zone vs line: a guarding longsword dummy in its high or low zone
-- takes the player's overhead chops; GUARD02_CHOP lines count the blade
-- contacts (RAGDOLL_PARRY kind=blade) and the body impacts between the
-- markers (tools/body/analysis/guard02.py). High should meet the chop.
registerTest('GUARD-02 right-click guard zones and parry tap', 918380, function()
    local weapon = 'iron longsword'
    local player = placePlayer()
    local victim
    local ok, message = pcall(function()
        ragdoll.setPlayerAlwaysHits(true)
        if not types.Actor.inventory(player):find(weapon) then
            world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 470, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 470 end)
        waitSeconds(1)
        powerAndStand(player, 'GUARD-02 player')
        cameraReadyMode = nil
        player:sendEvent('fixPlayerCamera', true)
        waitFor('GUARD-02 first-person camera', function() return cameraReadyMode == true end)
        ragdoll.prepareMouseInput()
        local RIGHT = 3
        local function measure(label)
            local st = ragdoll.getState(player)
            local hand, chest = body(st, 'right_hand').position, body(st, 'spine2').position
            local yaw = player.rotation:getYaw()
            local rel = hand - chest
            local c, sn = math.cos(-yaw), math.sin(-yaw)
            print(string.format('GUARD02 zone=%s read=%s active=%s hand_right=%.1f hand_fwd=%.1f hand_up=%.1f',
                label, tostring(st.guardZone), tostring(st.guardActive),
                rel.x * c - rel.y * sn, rel.x * sn + rel.y * c, rel.z))
        end
        for _, zone in ipairs({ { 'middle', 0, 0 }, { 'high', 0, -500 }, { 'left', -500, 0 }, { 'right', 500, 0 },
            { 'low', 0, 500 } }) do
            ragdoll.injectMouseInput(0, 0, true, RIGHT)
            waitSeconds(0.1)
            ragdoll.injectMouseInput(zone[2], zone[3])
            waitSeconds(0.5)
            measure(zone[1])
            ragdoll.captureScreenshot()
            waitSeconds(0.1)
            ragdoll.injectMouseInput(0, 0, false, RIGHT)
            waitSeconds(0.8)
        end
        local released = ragdoll.getState(player)
        print(string.format('GUARD02 released active=%s', tostring(released.guardActive)))
        -- The tap: a quick press and release holds the guard ~0.35 s.
        ragdoll.injectMouseInput(0, 0, true, RIGHT)
        waitSeconds(0.05)
        ragdoll.injectMouseInput(0, 0, false, RIGHT)
        waitSeconds(0.15)
        local during = ragdoll.getState(player).guardActive
        waitSeconds(0.5)
        local after = ragdoll.getState(player).guardActive
        print(string.format('GUARD02 tap during=%s after=%s', tostring(during), tostring(after)))
        -- Part 2: zone vs the overhead chop.
        -- The NPC guard AI lowers a guard nothing threatens: the test holds it.
        local holdGuard = nil
        local function keepGuard()
            if holdGuard and holdGuard:isValid() and not ragdoll.getState(holdGuard).guardActive then
                ragdoll.setGuard(holdGuard, true, util.vector3(0, -1, 0))
            end
        end
        local function drag(x, y, seconds)
            local started, sentX, sentY = core.getSimulationTime(), 0, 0
            waitSeconds(seconds, function()
                keepGuard()
                local f = math.min(1, (core.getSimulationTime() - started) / seconds)
                local vx, vy = math.floor(x * f), math.floor(y * f)
                ragdoll.injectMouseInput(vx - sentX, vy - sentY)
                sentX, sentY = vx, vy
            end)
            ragdoll.injectMouseInput(x - sentX, y - sentY)
        end
        for _, zone in ipairs({ { 'high', 500 }, { 'low', -500 } }) do
            for rep = 1, 3 do
                if victim and victim:isValid() then victim:remove() coroutine.yield() end
                victim = spawnDummy('GUARD-02 guard', util.vector3(0, 200, 0), weapon)
                victim:sendEvent('fixActorHealth', 100)
                powerAndStand(victim, 'GUARD-02 guard')
                local p = ragdoll.getState(player)
                local shoulder = body(p, 'right_upper_arm').position
                local reach = (body(p, 'right_forearm').position - shoulder):length()
                    + (body(p, 'right_hand').position - body(p, 'right_forearm').position):length()
                local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
                placeVictimPoint(victim, function()
                    local vs = ragdoll.getState(victim)
                    return vs and body(vs, 'spine2').position or victim.position
                end, pivot + util.vector3(0, reach + p.weapon.length * 0.55, 0), util.transform.rotateZ(math.pi))
                ragdoll.setGuard(victim, true, util.vector3(0, -1, 0))
                waitSeconds(0.1)
                ragdoll.setGuardZone(victim, 0, zone[2])
                holdGuard = victim
                waitSeconds(0.1, keepGuard)
                print(string.format('GUARD02_ZONE_SET zone=%s read=%s', zone[1], tostring(ragdoll.getState(victim).guardZone)))
                waitSeconds(0.5, keepGuard)
                if rep == 1 then
                    world.players[1]:sendEvent('fixWoundCamera',
                        { position = victim.position + util.vector3(-240, 0, 60), yaw = math.pi / 2 })
                    waitSeconds(0.1)
                    print(string.format('GUARD02 shot zone=%s guard=%s hsc=%s', zone[1],
                        tostring(ragdoll.getState(victim).guardActive), tostring(ragdoll.getState(victim).hsc ~= nil)))
                    ragdoll.captureScreenshot()
                    waitSeconds(0.1)
                    world.players[1]:sendEvent('fixWoundCamera', {})
                    cameraReadyMode = nil
                    player:sendEvent('fixPlayerCamera', true)
                    waitFor('GUARD-02 first-person camera', function() return cameraReadyMode == true end)
                end
                print(string.format('GUARD02_CHOP_BEGIN zone=%s rep=%d read=%s', zone[1], rep,
                    tostring(ragdoll.getState(victim).guardZone)))
                ragdoll.injectMouseInput(0, 0, true, 1)
                waitSeconds(0.2, keepGuard)
                drag(0, -HSC_WIND, 0.4)
                drag(0, HSC_CUT, 0.2)
                waitSeconds(0.3, keepGuard)
                holdGuard = nil
                ragdoll.injectMouseInput(0, 0, false, 1)
                print(string.format('GUARD02_CHOP_END zone=%s rep=%d', zone[1], rep))
                ragdoll.setGuard(victim, false)
                waitSeconds(0.8)
            end
        end
    end)
    ragdoll.injectMouseInput(0, 0, false, 3)
    ragdoll.injectMouseInput(0, 0, false)
    ragdoll.setPlayerAlwaysHits(false)
    ragdoll.setPowered(player, false)
    if victim and victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)

-- GUARD-03 (user 10-31: "when guard button is clicked and then the attack
-- button is clicked at the same time the guard stops and immediately goes
-- into attack/drag mode"): the right button held (the guard up 0.5 s, a
-- dummy in reach), then the left button: a click, or a held chop drag. The
-- right button stays down throughout. GUARD03 lines: the guard 0.1 s after
-- the press, when the cut began (s after the press, -1 = never) and the
-- blade's contacts and strikes in 1.5 s. Compare
-- OPENMW_RAGDOLL_GUARD_TO_ATTACK=0 (the guard wins).
for index, weapon in ipairs({ 'iron longsword', 'iron claymore' }) do
    registerTest('GUARD-03 attack out of a held guard with ' .. weapon, 919480 + index, function()
        local player = placePlayer()
        local victim
        local ok, message = pcall(function()
            ragdoll.setPlayerAlwaysHits(true)
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 900 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 900 + index end)
            waitSeconds(1)
            powerAndStand(player, 'GUARD-03 player')
            ragdoll.prepareMouseInput()
            local RIGHT = 3
            local function drag(x, y, seconds)
                local started, sentX, sentY = core.getSimulationTime(), 0, 0
                waitSeconds(seconds, function()
                    local f = math.min(1, (core.getSimulationTime() - started) / seconds)
                    local vx, vy = math.floor(x * f), math.floor(y * f)
                    ragdoll.injectMouseInput(vx - sentX, vy - sentY)
                    sentX, sentY = vx, vy
                end)
                ragdoll.injectMouseInput(x - sentX, y - sentY)
            end
            for rep = 1, 3 do
                for _, mode in ipairs({ 'click', 'drag' }) do
                    if not victim or not victim:isValid() or types.Actor.isDead(victim) or not ragdoll.getState(victim) then
                        if victim and victim:isValid() then victim:remove() coroutine.yield() end
                        victim = spawnDummy('GUARD-03 target', util.vector3(0, 200, 0), false)
                        powerAndStand(victim, 'GUARD-03 target')
                    end
                    pcall(waitFor, 'GUARD-03 guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                    local p = ragdoll.getState(player)
                    local shoulder = body(p, 'right_upper_arm').position
                    local reach = (body(p, 'right_forearm').position - shoulder):length()
                        + (body(p, 'right_hand').position - body(p, 'right_forearm').position):length()
                    local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
                    placeVictimPoint(victim, function()
                        local vs = ragdoll.getState(victim)
                        return vs and body(vs, 'spine2').position or victim.position
                    end, pivot + util.vector3(0, reach + p.weapon.length * 0.6, 0), util.transform.rotateZ(math.pi))
                    ragdoll.injectMouseInput(0, 0, true, RIGHT)
                    waitSeconds(0.5)
                    local guardBefore = ragdoll.getState(player).guardActive
                    local w0 = ragdoll.getState(player).weapon
                    local contacts0, strikes0 = w0.contactEventCount, w0.strikeEventCount
                    print(string.format('GUARD03_BEGIN weapon=%s mode=%s rep=%d guard=%s', weapon, mode, rep,
                        tostring(guardBefore)))
                    local pressed = core.getSimulationTime()
                    local cutAt, guardAfter = -1, nil
                    local function watch()
                        local st = ragdoll.getState(player)
                        local now = core.getSimulationTime() - pressed
                        if guardAfter == nil and now >= 0.1 then guardAfter = st.guardActive end
                        if cutAt < 0 and st.hsc and st.hsc.stage == 2 then cutAt = now end
                    end
                    ragdoll.injectMouseInput(0, 0, true, 1)
                    if mode == 'click' then
                        waitSeconds(0.08, watch)
                        ragdoll.injectMouseInput(0, 0, false, 1)
                        waitSeconds(1.42, watch)
                    else
                        waitSeconds(0.1, watch)
                        drag(0, -HSC_WIND, 0.4)
                        drag(0, HSC_CUT, 0.2)
                        waitSeconds(0.3, watch)
                        ragdoll.injectMouseInput(0, 0, false, 1)
                        waitSeconds(0.5, watch)
                    end
                    local w = ragdoll.getState(player).weapon
                    print(string.format('GUARD03 weapon=%s mode=%s rep=%d guard_before=%s guard_after=%s cut_at=%.2f contacts=%d strikes=%d',
                        weapon, mode, rep, tostring(guardBefore), tostring(guardAfter), cutAt,
                        w.contactEventCount - contacts0, w.strikeEventCount - strikes0))
                    ragdoll.injectMouseInput(0, 0, false, RIGHT)
                    waitSeconds(1.0)
                end
            end
        end)
        ragdoll.injectMouseInput(0, 0, false, 3)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if victim and victim:isValid() then victim:remove() end
        if not ok then error(message, 0) end
    end)
end

-- RECOVER-01 (user 10-31: "is attack recovery related to stats? ... make it
-- depend on the skill"): two clicks 0.45 s apart (the second in the first's
-- cut) at a dummy, at weapon skill 20, 45 and 90. Below 50 the second click
-- waits for the first cut and the return to the guard (longer at low
-- control) and then plays; from 50 it chains at once. RECOVER01 lines: cuts begun, the gap between the two cuts'
-- starts (s), the longest return (s) and the blade's strikes.
for index, weapon in ipairs({ 'iron longsword', 'iron claymore' }) do
    registerTest('RECOVER-01 two quick clicks by skill with ' .. weapon, 919490 + index, function()
        local player = placePlayer()
        local victim
        local ok, message = pcall(function()
            -- Not "always hits": that is full weapon control, and the return
            -- by control would not show.
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 910 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 910 + index end)
            waitSeconds(1)
            powerAndStand(player, 'RECOVER-01 player')
            ragdoll.prepareMouseInput()
            for _, skill in ipairs({ 20, 45, 90 }) do
                player:sendEvent('fixSetWeaponSkill', skill)
                waitSeconds(0.5)
                for rep = 1, 2 do
                    if not victim or not victim:isValid() or types.Actor.isDead(victim) or not ragdoll.getState(victim) then
                        if victim and victim:isValid() then victim:remove() coroutine.yield() end
                        victim = spawnDummy('RECOVER-01 target', util.vector3(0, 200, 0), false)
                        powerAndStand(victim, 'RECOVER-01 target')
                    end
                    pcall(waitFor, 'RECOVER-01 guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                    local p = ragdoll.getState(player)
                    local shoulder = body(p, 'right_upper_arm').position
                    local reach = (body(p, 'right_forearm').position - shoulder):length()
                        + (body(p, 'right_hand').position - body(p, 'right_forearm').position):length()
                    local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
                    placeVictimPoint(victim, function()
                        local vs = ragdoll.getState(victim)
                        return vs and body(vs, 'spine2').position or victim.position
                    end, pivot + util.vector3(0, reach + p.weapon.length * 0.6, 0), util.transform.rotateZ(math.pi))
                    waitSeconds(0.5)
                    local w0 = ragdoll.getState(player).weapon
                    local strikes0 = w0.strikeEventCount
                    local started = core.getSimulationTime()
                    local cuts, lastStage, recoverAt, longestReturn = {}, nil, nil, 0
                    local function watch()
                        local st = ragdoll.getState(player)
                        local stage = st.hsc and st.hsc.stage
                        local now = core.getSimulationTime() - started
                        if stage == 2 and lastStage ~= 2 then cuts[#cuts + 1] = now end
                        if stage == 3 and lastStage ~= 3 then recoverAt = now end
                        if stage ~= 3 and lastStage == 3 and recoverAt then
                            longestReturn = math.max(longestReturn, now - recoverAt)
                        end
                        lastStage = stage
                    end
                    for _ = 1, 2 do
                        ragdoll.injectMouseInput(0, 0, true, 1)
                        waitSeconds(0.08, watch)
                        ragdoll.injectMouseInput(0, 0, false, 1)
                        waitSeconds(0.37, watch)
                    end
                    waitSeconds(3.0, watch)
                    print(string.format('RECOVER01 weapon=%s skill=%d rep=%d cuts=%d gap=%.2f return=%.2f strikes=%d',
                        weapon, skill, rep, #cuts, #cuts >= 2 and cuts[2] - cuts[1] or -1, longestReturn,
                        ragdoll.getState(player).weapon.strikeEventCount - strikes0))
                    testing.expect(#cuts >= 2, 'RECOVER-01 the second click was lost at skill ' .. skill)
                end
            end
        end)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if victim and victim:isValid() then victim:remove() end
        if not ok then error(message, 0) end
    end)
end

-- COMBO-01 (user 10-31, momentum chains as combos; HSC_CHAIN_FLOW): at
-- weapon skill 90, a dummy in reach, three ways to follow a cut:
--   square  - one held stroke: a forehand that runs on into the backhand's
--             wind-up and cuts back (the follow-through feeds the new line);
--   reverse - a forehand, then a new press in its follow-through dragged back
--             the way it came (against the momentum);
--   clicks  - two quick chop clicks 0.45 s apart (a chop's follow-through
--             falls away from the next chop's chamber).
-- COMBO01 lines: cuts begun, the gap between their starts (s), the fatigue
-- the series cost and the blade's strikes; the engine's RAGDOLL_HSC_COMBO
-- lines give each chained cut's flow (0-1) and its fatigue share. With the
-- switch on, square should flow and pay less; reverse and clicks not.
for index, weapon in ipairs({ 'iron longsword', 'iron claymore' }) do
    registerTest('COMBO-01 chain flow with ' .. weapon, 919500 + index, function()
        local player = placePlayer()
        local victim
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 920 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 920 + index end)
            waitSeconds(1)
            powerAndStand(player, 'COMBO-01 player')
            player:sendEvent('fixSetWeaponSkill', 90)
            ragdoll.prepareMouseInput()
            local function drag(x, y, seconds, watch)
                local started, sentX, sentY = core.getSimulationTime(), 0, 0
                waitSeconds(seconds, function()
                    local f = math.min(1, (core.getSimulationTime() - started) / seconds)
                    local vx, vy = math.floor(x * f), math.floor(y * f)
                    ragdoll.injectMouseInput(vx - sentX, vy - sentY)
                    sentX, sentY = vx, vy
                    if watch then watch() end
                end)
                ragdoll.injectMouseInput(x - sentX, y - sentY)
            end
            for rep = 1, 3 do
                for _, mode in ipairs({ 'square', 'reverse', 'clicks' }) do
                    if not victim or not victim:isValid() or types.Actor.isDead(victim) or not ragdoll.getState(victim) then
                        if victim and victim:isValid() then victim:remove() coroutine.yield() end
                        victim = spawnDummy('COMBO-01 target', util.vector3(0, 200, 0), false)
                        powerAndStand(victim, 'COMBO-01 target')
                    end
                    pcall(waitFor, 'COMBO-01 guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                    local p = ragdoll.getState(player)
                    local shoulder = body(p, 'right_upper_arm').position
                    local reach = (body(p, 'right_forearm').position - shoulder):length()
                        + (body(p, 'right_hand').position - body(p, 'right_forearm').position):length()
                    local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
                    placeVictimPoint(victim, function()
                        local vs = ragdoll.getState(victim)
                        return vs and body(vs, 'spine2').position or victim.position
                    end, pivot + util.vector3(0, reach + p.weapon.length * 0.6, 0), util.transform.rotateZ(math.pi))
                    player:sendEvent('fixSetFatigue', 1.0)
                    waitSeconds(0.5)
                    local fatigue0 = types.Actor.stats.dynamic.fatigue(player).current
                    local strikes0 = ragdoll.getState(player).weapon.strikeEventCount
                    local started = core.getSimulationTime()
                    local cuts, lastStage = {}, nil
                    local function watch()
                        local st = ragdoll.getState(player)
                        local stage = st.hsc and st.hsc.stage
                        if stage == 2 and lastStage ~= 2 then cuts[#cuts + 1] = core.getSimulationTime() - started end
                        lastStage = stage
                    end
                    print(string.format('COMBO01_BEGIN weapon=%s mode=%s rep=%d', weapon, mode, rep))
                    if mode == 'square' then
                        ragdoll.injectMouseInput(0, 0, true, 1)
                        drag(HSC_WIND, 0, 0.35, watch)
                        waitSeconds(0.15, watch)
                        drag(-(HSC_CUT + HSC_RUN_ON + 600), 0, 0.6, watch)
                        drag(HSC_CUT, 0, 0.3, watch)
                        ragdoll.injectMouseInput(0, 0, false, 1)
                    elseif mode == 'reverse' then
                        ragdoll.injectMouseInput(0, 0, true, 1)
                        drag(HSC_WIND, 0, 0.35, watch)
                        waitSeconds(0.15, watch)
                        drag(-HSC_CUT, 0, 0.2, watch)
                        ragdoll.injectMouseInput(0, 0, false, 1)
                        waitSeconds(0.1, watch)
                        ragdoll.injectMouseInput(0, 0, true, 1)
                        drag(HSC_WIND, 0, 0.35, watch)
                        drag(-HSC_CUT, 0, 0.2, watch)
                        ragdoll.injectMouseInput(0, 0, false, 1)
                    else
                        for _ = 1, 2 do
                            ragdoll.injectMouseInput(0, 0, true, 1)
                            waitSeconds(0.08, watch)
                            ragdoll.injectMouseInput(0, 0, false, 1)
                            waitSeconds(0.37, watch)
                        end
                    end
                    waitSeconds(1.5, watch)
                    local spent = fatigue0 - types.Actor.stats.dynamic.fatigue(player).current
                    print(string.format('COMBO01 weapon=%s mode=%s rep=%d cuts=%d gap=%.2f fatigue=%.1f strikes=%d',
                        weapon, mode, rep, #cuts, #cuts >= 2 and cuts[2] - cuts[1] or -1, spent,
                        ragdoll.getState(player).weapon.strikeEventCount - strikes0))
                    testing.expect(#cuts >= 2, 'COMBO-01 ' .. mode .. ': the second cut never began')
                    print(string.format('COMBO01_END weapon=%s mode=%s rep=%d', weapon, mode, rep))
                end
            end
        end)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.setPowered(player, false)
        if victim and victim:isValid() then victim:remove() end
        if not ok then error(message, 0) end
    end)
end

-- RIPOSTE-02 (user 10-31, the player's riposte; HSC_RIPOSTE): the player
-- holds the guard 0.5 s at a dummy in reach, drops it and clicks 0.1 s later,
-- 8 times "control" and 8 times "riposte" (ragdoll.markParryWon just before
-- the guard drops: the state a won clash leaves - the clash path that sets it
-- is RIPOSTE-01's). RIPOSTE02 lines: the click's wind-up (s from the press to
-- the cut), whether it ran as a riposte, strikes. With the switch on, the
-- riposte's wind-up is about `wind share` of the control's; off, the same.
registerTest('RIPOSTE-02 the player answers a parry', 919510, function()
    local weapon = 'iron longsword'
    local player = placePlayer()
    local victim
    local RIGHT = 3
    local ok, message = pcall(function()
        if not types.Actor.inventory(player):find(weapon) then
            world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 930, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 930 end)
        waitSeconds(1)
        powerAndStand(player, 'RIPOSTE-02 player')
        player:sendEvent('fixSetWeaponSkill', 60)
        ragdoll.prepareMouseInput()
        for n = 1, 16 do
            local mode = n % 2 == 0 and 'riposte' or 'control'
            if not victim or not victim:isValid() or types.Actor.isDead(victim) or not ragdoll.getState(victim) then
                if victim and victim:isValid() then victim:remove() coroutine.yield() end
                victim = spawnDummy('RIPOSTE-02 target', util.vector3(0, 200, 0), false)
                powerAndStand(victim, 'RIPOSTE-02 target')
            end
            pcall(waitFor, 'RIPOSTE-02 guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            local p = ragdoll.getState(player)
            local shoulder = body(p, 'right_upper_arm').position
            local reach = (body(p, 'right_forearm').position - shoulder):length()
                + (body(p, 'right_hand').position - body(p, 'right_forearm').position):length()
            local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
            placeVictimPoint(victim, function()
                local vs = ragdoll.getState(victim)
                return vs and body(vs, 'spine2').position or victim.position
            end, pivot + util.vector3(0, reach + p.weapon.length * 0.6, 0), util.transform.rotateZ(math.pi))
            ragdoll.injectMouseInput(0, 0, true, RIGHT)
            waitSeconds(0.5)
            if mode == 'riposte' then ragdoll.markParryWon(player) end
            ragdoll.injectMouseInput(0, 0, false, RIGHT)
            waitSeconds(0.1)
            local strikes0 = ragdoll.getState(player).weapon.strikeEventCount
            local pressed = core.getSimulationTime()
            local cutAt, riposte = -1, false
            local function watch()
                local st = ragdoll.getState(player)
                if st.hsc and st.hsc.riposte then riposte = true end
                if cutAt < 0 and st.hsc and st.hsc.stage == 2 then cutAt = core.getSimulationTime() - pressed end
            end
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitSeconds(0.08, watch)
            ragdoll.injectMouseInput(0, 0, false, 1)
            waitSeconds(1.2, watch)
            print(string.format('RIPOSTE02 n=%d mode=%s cut_at=%.2f riposte=%s strikes=%d', n, mode, cutAt,
                tostring(riposte), ragdoll.getState(player).weapon.strikeEventCount - strikes0))
            testing.expect(cutAt >= 0, 'RIPOSTE-02 the click never cut (' .. mode .. ')')
            waitSeconds(0.8)
        end
    end)
    ragdoll.injectMouseInput(0, 0, false, RIGHT)
    ragdoll.injectMouseInput(0, 0, false, 1)
    ragdoll.setPowered(player, false)
    if victim and victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)

-- GRIP-05 (user 10-31, the half-sword grip; OPENMW_RAGDOLL_GRIP_HALF): at a
-- dummy in reach, 8 clicks in the normal grip, then the grip key
-- (ragdoll.cycleGrip): the support hand should take the blade about halfway
-- up (one-handers too) within a second, every click thrust, the grip hold
-- through 8 clicks; then the key again, back to normal. GRIP05 lines: the
-- grip mode, the support grip attached and when, the hands' spacing along the
-- weapon (units and share of its length), cuts, strikes, the support grip's
-- breaks and releases in the series.
for index, weapon in ipairs({ 'iron longsword', 'iron claymore' }) do
    registerTest('GRIP-05 half-sword grip with ' .. weapon, 919520 + index, function()
        local player = placePlayer()
        local victim
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 940 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 940 + index end)
            waitSeconds(1)
            powerAndStand(player, 'GRIP-05 player')
            player:sendEvent('fixSetWeaponSkill', 60)
            ragdoll.prepareMouseInput()
            local function spacing()
                local st = ragdoll.getState(player)
                local right, left = body(st, 'right_hand').position, body(st, 'left_hand').position
                return (left - right):length(), st.weapon.length
            end
            local function grip()
                local w = ragdoll.getState(player).weapon
                return w.supportGrip or {}
            end
            local function series(label)
                if not victim or not victim:isValid() or types.Actor.isDead(victim) or not ragdoll.getState(victim) then
                    if victim and victim:isValid() then victim:remove() coroutine.yield() end
                    victim = spawnDummy('GRIP-05 target', util.vector3(0, 200, 0), false)
                    powerAndStand(victim, 'GRIP-05 target')
                end
                local p = ragdoll.getState(player)
                local shoulder = body(p, 'right_upper_arm').position
                local reach = (body(p, 'right_forearm').position - shoulder):length()
                    + (body(p, 'right_hand').position - body(p, 'right_forearm').position):length()
                local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
                -- The murder-stroke strikes with the hilt, a short way past the
                -- hands: the foe stands closer than for the blade.
                local share = label == 'murder' and 0.15 or 0.4
                placeVictimPoint(victim, function()
                    local vs = ragdoll.getState(victim)
                    return vs and body(vs, 'spine2').position or victim.position
                end, pivot + util.vector3(0, reach + p.weapon.length * share, 0), util.transform.rotateZ(math.pi))
                waitSeconds(0.5)
                local g0 = grip()
                local breaks0, releases0 = g0.breakCount or 0, g0.releaseCount or 0
                local strikes0 = ragdoll.getState(player).weapon.strikeEventCount
                local cuts, lastStage, held, steps, gripModes = 0, nil, 0, 0, {}
                local function watch()
                    local st = ragdoll.getState(player)
                    local stage = st.hsc and st.hsc.stage
                    if stage == 2 and lastStage ~= 2 then cuts = cuts + 1 end
                    lastStage = stage
                    steps = steps + 1
                    if (st.weapon.supportGrip or {}).attached then held = held + 1 end
                    if st.hsc then gripModes[st.hsc.grip or -1] = true end
                end
                for _ = 1, 8 do
                    ragdoll.injectMouseInput(0, 0, true, 1)
                    waitSeconds(0.08, watch)
                    ragdoll.injectMouseInput(0, 0, false, 1)
                    waitSeconds(0.9, watch)
                end
                local g = grip()
                local d, length = spacing()
                local modes = {}
                for m in pairs(gripModes) do modes[#modes + 1] = tostring(m) end
                print(string.format('GRIP05 weapon=%s series=%s grip=%s held=%.2f spacing=%.1f share=%.2f cuts=%d strikes=%d breaks=%d releases=%d',
                    weapon, label, table.concat(modes, '/'), steps > 0 and held / steps or 0, d, d / math.max(1, length), cuts,
                    ragdoll.getState(player).weapon.strikeEventCount - strikes0,
                    (g.breakCount or 0) - breaks0, (g.releaseCount or 0) - releases0))
                return cuts
            end
            pcall(waitFor, 'GRIP-05 guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            series('normal')
            pcall(waitFor, 'GRIP-05 guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            local changed = ragdoll.cycleGrip(player)
            local t0 = core.getSimulationTime()
            local gotAt = -1
            local states = {}
            pcall(waitFor, 'GRIP-05 half-sword grip', function()
                local g = grip()
                local key = tostring(g.state) .. '/' .. tostring(g.reason)
                states[key] = (states[key] or 0) + 1
                if g.attached then gotAt = core.getSimulationTime() - t0 return true end
                return false
            end, 2)
            waitSeconds(0.5)
            local d, length = spacing()
            local seen = {}
            for k, n in pairs(states) do seen[#seen + 1] = k .. '=' .. n end
            print(string.format('GRIP05_CHANGE weapon=%s changed=%s attached_at=%.2f spacing=%.1f share=%.2f states=%s',
                weapon, tostring(changed), gotAt, d, d / math.max(1, length), table.concat(seen, ',')))
            testing.expect(changed, 'GRIP-05 the grip key did nothing')
            testing.expect(gotAt >= 0, 'GRIP-05 the support hand never took the blade')
            local cuts = series('half')
            testing.expect(cuts >= 6, 'GRIP-05 half-sword clicks did not thrust')
            -- The key again: the murder-stroke when its switch is on (the
            -- sword turned round, both hands on the blade, the hilt the
            -- hammer), else back to normal.
            pcall(waitFor, 'GRIP-05 guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            ragdoll.cycleGrip(player)
            waitSeconds(1.0)
            local st = ragdoll.getState(player)
            if st.hsc and st.hsc.grip == 2 then
                d, length = spacing()
                print(string.format('GRIP05_MURDER weapon=%s attached=%s spacing=%.1f share=%.2f weapon_attached=%s',
                    weapon, tostring(grip().attached), d, d / math.max(1, length), tostring(st.weapon.attached)))
                series('murder')
                testing.expect(ragdoll.getState(player).weapon.attached, 'GRIP-05 the murder-stroke dropped the weapon')
                pcall(waitFor, 'GRIP-05 guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                ragdoll.cycleGrip(player)
                waitSeconds(1.5)
                st = ragdoll.getState(player)
            end
            d, length = spacing()
            print(string.format('GRIP05_BACK weapon=%s grip=%s attached=%s spacing=%.1f weapon_attached=%s', weapon,
                tostring(st.hsc and st.hsc.grip), tostring(grip().attached), d, tostring(st.weapon.attached)))
            testing.expect(st.hsc == nil or st.hsc.grip == 0, 'GRIP-05 the grip did not come back to normal')
        end)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.setPowered(player, false)
        if victim and victim:isValid() then victim:remove() end
        if not ok then error(message, 0) end
    end)
end

-- FIST-02 (user 10-31, boxing combos; OPENMW_RAGDOLL_FIST_ALTERNATE): bare
-- fists at a dummy in reach, series of clicks 0.35 s apart (a 1-2-1-2), then
-- a pause and again. With the switch on the clicks alternate hands - the
-- first of a series the left jab, the next the right hand's wheel punch.
-- FIST02 lines per series: jabs and right punches thrown, the left hand's
-- furthest reach from its shoulder (share of the arm), the hands that struck
-- and the dummy's health lost.
registerTest('FIST-02 jab and cross', 919530, function()
    local player = placePlayer()
    local victim
    local savedReaction = ragdoll.reactionScale()
    local ok, message = pcall(function()
        player:sendEvent('fixPlayerConfigure', { sequence = 950, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 950 end)
        waitSeconds(1)
        powerAndStand(player, 'FIST-02 player')
        ragdoll.setPlayerAlwaysHits(true)
        ragdoll.reactionScale(0)
        ragdoll.prepareMouseInput()
        victim = spawnDummy('FIST-02 target', util.vector3(0, 200, 0), false)
        victim:sendEvent('fixActorHealth', 1000)
        powerAndStand(victim, 'FIST-02 target')
        for series = 1, 3 do
            local p = ragdoll.getState(player)
            local shoulder = body(p, 'right_upper_arm').position
            local reach = (body(p, 'right_forearm').position - shoulder):length()
                + (body(p, 'right_hand').position - body(p, 'right_forearm').position):length()
            local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
            placeVictimPoint(victim, function()
                local vs = ragdoll.getState(victim)
                return vs and body(vs, 'spine2').position or victim.position
            end, pivot + util.vector3(0, reach + 8, 0), util.transform.rotateZ(math.pi))
            waitSeconds(1.2)
            local st0 = ragdoll.getState(player)
            local jabs0 = st0.hsc and st0.hsc.jabs or 0
            -- Hand-to-hand hits fatigue (Morrowind), not health.
            local health0 = types.Actor.stats.dynamic.fatigue(victim).current
            local punches, lastStage, leftReach, hands = 0, nil, 0, {}
            local function watch()
                local st = ragdoll.getState(player)
                if not st then return end
                local stage = st.hsc and st.hsc.stage
                if stage == 2 and lastStage ~= 2 then punches = punches + 1 end
                lastStage = stage
                if st.strikeHand then hands[st.strikeHand] = true end
                local h, sh, fa = body(st, 'left_hand'), body(st, 'left_upper_arm'), body(st, 'left_forearm')
                local arm = (fa.position - sh.position):length() + (h.position - fa.position):length()
                leftReach = math.max(leftReach, (h.position - sh.position):length() / math.max(1, arm))
            end
            for _ = 1, 4 do
                ragdoll.injectMouseInput(0, 0, true, 1)
                waitSeconds(0.06, watch)
                ragdoll.injectMouseInput(0, 0, false, 1)
                waitSeconds(0.29, watch)
            end
            waitSeconds(1.0, watch)
            local st = ragdoll.getState(player)
            local jabs = (st.hsc and st.hsc.jabs or 0) - jabs0
            print(string.format('FIST02 series=%d jabs=%d right=%d left_reach=%.2f hands=%s fatigue_lost=%.1f', series, jabs,
                punches, leftReach, (hands.left and 'L' or '') .. (hands.right and 'R' or ''),
                health0 - types.Actor.stats.dynamic.fatigue(victim).current))
        end
    end)
    ragdoll.injectMouseInput(0, 0, false)
    ragdoll.setPlayerAlwaysHits(false)
    ragdoll.reactionScale(savedReaction)
    ragdoll.setPowered(player, false)
    if victim and victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)

-- RIPOSTE-03 (observation, not a gate): RIPOSTE-02 against a Hlaalu guard in
-- AI combat. The player holds the guard; when its parry wins (hsc.sinceParryWon
-- resets: a knockaway or a blow stopped on the guard) the guard drops and a
-- click follows at once; with no parry won in 3 s the click comes anyway.
-- RIPOSTE03 lines as RIPOSTE02's plus whether a parry was won. (10-31: a held
-- guard won no parry in 16 bouts at skill 60 - it was broken 3 times.)
registerTest('RIPOSTE-03 the player answers a live parry', 919511, function()
    local weapon = 'iron longsword'
    local player = placePlayer()
    local npc
    local RIGHT = 3
    local ok, message = pcall(function()
        if not types.Actor.inventory(player):find(weapon) then
            world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 930, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 930 end)
        waitSeconds(1)
        powerAndStand(player, 'RIPOSTE-03 player')
        player:sendEvent('fixSetWeaponSkill', 60)
        ragdoll.prepareMouseInput()
        local record = world.createRecord(types.NPC.createRecordDraft({
            template = types.NPC.record('hlaalu guard'), name = 'RIPOSTE-03 guard',
            isEssential = false, isPersistent = false, isRespawning = false,
        }))
        npc = world.createObject(record.id)
        if not types.Actor.inventory(npc):find('steel longsword') then
            world.createObject('steel longsword', 1):moveInto(types.Actor.inventory(npc))
        end
        npc:teleport(testArea.cell, testArea.player + util.vector3(0, 110, 0),
            { onGround = true, rotation = util.transform.rotateZ(math.pi) })
        npc:addScript('stance.lua')
        coroutine.yield()
        npc:sendEvent('fixActorConfigure', { weaponStance = 'steel longsword' })
        waitFor('RIPOSTE-03 guard stance', function() return readyActors[npc.id] ~= nil end, 10)
        npc:sendEvent('fixActorHealth', 5000)
        powerAndStand(npc, 'RIPOSTE-03 guard')
        npc:sendEvent('fixActorFight', true)
        local function face()
            local to = npc.position - player.position
            local err = math.atan2(to.x, to.y) - player.rotation:getYaw()
            err = (err + math.pi) % (2 * math.pi) - math.pi
            player:sendEvent('fixPlayerControls', { forward = 0, side = 0, run = 0,
                yaw = math.max(-0.1, math.min(0.1, err * 0.5)) })
        end
        print('RIPOSTE03_BEGIN')
        for n = 1, 16 do
            ragdoll.setCombatTarget(npc, player)
            player:sendEvent('fixHealPlayer')
            pcall(waitFor, 'RIPOSTE-03 guard', function() return ragdoll.getSwingSpeed(player) == nil end, 3)
            ragdoll.injectMouseInput(0, 0, true, RIGHT)
            waitSeconds(0.15, face)
            local won = false
            local waited = 0
            local step0 = core.getSimulationTime()
            waitFor('RIPOSTE-03 parry', function()
                face()
                local st = ragdoll.getState(player)
                won = st.hsc ~= nil and st.hsc.sinceParryWon ~= nil and st.hsc.sinceParryWon < 0.1
                waited = core.getSimulationTime() - step0
                return won or waited > 3
            end, 4)
            ragdoll.injectMouseInput(0, 0, false, RIGHT)
            local strikes0 = ragdoll.getState(player).weapon.strikeEventCount
            local pressed = core.getSimulationTime()
            local cutAt, riposte = -1, false
            local function watch()
                local st = ragdoll.getState(player)
                if st.hsc and st.hsc.riposte then riposte = true end
                if cutAt < 0 and st.hsc and st.hsc.stage == 2 then cutAt = core.getSimulationTime() - pressed end
            end
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitSeconds(0.08, watch)
            ragdoll.injectMouseInput(0, 0, false, 1)
            waitSeconds(1.2, watch)
            print(string.format('RIPOSTE03 n=%d won=%s waited=%.2f cut_at=%.2f riposte=%s strikes=%d',
                n, tostring(won), waited, cutAt, tostring(riposte),
                ragdoll.getState(player).weapon.strikeEventCount - strikes0))
            waitSeconds(0.6, face)
        end
        print('RIPOSTE03_END')
    end)
    player:sendEvent('fixPlayerControls', nil)
    ragdoll.injectMouseInput(0, 0, false, RIGHT)
    ragdoll.injectMouseInput(0, 0, false, 1)
    ragdoll.setPowered(player, false)
    if npc and npc:isValid() then npc:remove() end
    if not ok then error(message, 0) end
end)

-- CUT-01 (user bookmark 2, 2026-10-06: "if the cut has been commited it
-- should go all the way through, not stop mid-chop or mid-slash"): a full
-- forehand wind-up, then a cut dragged back only `cut` counts (a full cut is
-- HSC_CUT), held still 0.5 s, then released. Prints how far along the route
-- the cut got (hsc.s, 2 = the end) and the damage.
registerTest('CUT-01 short cut drags with iron longsword', 918320, function()
    local weapon = 'iron longsword'
    local player = placePlayer()
    local victim
    local savedReaction = ragdoll.reactionScale()
    local ok, message = pcall(function()
        ragdoll.setPlayerAlwaysHits(true)
        if not types.Actor.inventory(player):find(weapon) then
            world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 420, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 420 end)
        waitSeconds(1)
        powerAndStand(player, 'CUT-01 player')
        cameraReadyMode = nil
        player:sendEvent('fixPlayerCamera', true)
        waitFor('CUT-01 first-person camera', function() return cameraReadyMode == true end)
        victim = spawnDummy('CUT-01 target', util.vector3(0, 200, 0), false)
        powerAndStand(victim, 'CUT-01 target')
        ragdoll.reactionScale(0)
        ragdoll.prepareMouseInput()
        local function damageTotal()
            local total = 0
            for _, value in pairs(ragdoll.getLimbDamage(victim)) do total = total + value end
            return total
        end
        for _, cut in ipairs({ 700, 700, 1200, 1200, HSC_CUT }) do
            if types.Actor.isDead(victim) or not ragdoll.getState(victim) then
                victim:remove()
                coroutine.yield()
                victim = spawnDummy('CUT-01 target', util.vector3(0, 200, 0), false)
                powerAndStand(victim, 'CUT-01 new target')
            end
            local p = ragdoll.getState(player)
            local shoulder = body(p, 'right_upper_arm').position
            local elbow = body(p, 'right_forearm').position
            local hand = body(p, 'right_hand').position
            local reach = (elbow - shoulder):length() + (hand - elbow):length()
            local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
            placeVictimPoint(victim, function() return body(ragdoll.getState(victim), 'spine2').position end,
                pivot + util.vector3(0, reach + p.weapon.length * 0.55, 0), util.transform.rotateZ(math.pi))
            waitSeconds(0.3)
            local before = damageTotal()
            local maxS = 0
            local function watch()
                local st = ragdoll.getState(player)
                if st and st.hsc and st.hsc.s then maxS = math.max(maxS, st.hsc.s) end
            end
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitSeconds(0.25, watch)
            for _, leg in ipairs({ { x = HSC_WIND, seconds = 0.4 }, { x = -cut, seconds = 0.15 } }) do
                local started, sentX = core.getSimulationTime(), 0
                waitSeconds(leg.seconds, function()
                    local fraction = math.min(1, (core.getSimulationTime() - started) / leg.seconds)
                    local x = math.floor(leg.x * fraction)
                    ragdoll.injectMouseInput(x - sentX, 0)
                    sentX = x
                    watch()
                end)
                ragdoll.injectMouseInput(leg.x - sentX, 0)
            end
            waitSeconds(0.5, watch)
            ragdoll.injectMouseInput(0, 0, false, 1)
            waitSeconds(0.8, watch)
            pcall(waitFor, 'CUT-01 withdrawal', function() watch(); return ragdoll.getSwingSpeed(player) == nil end, 4)
            print(string.format('CUT01 cut=%d max_s=%.2f damage=%.3f', cut, maxS, damageTotal() - before))
            waitSeconds(0.6)
        end
    end)
    ragdoll.injectMouseInput(0, 0, false)
    ragdoll.reactionScale(savedReaction)
    ragdoll.setPlayerAlwaysHits(false)
    ragdoll.setPowered(player, false)
    if victim and victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)

-- STEER-01 (aim item 3, user 10-06: "steer during the cut"): a chop (drag
-- up, then down) that curves: straight down for the cut's first half, then
-- down and `side` counts right or left (a straight diagonal drag is only a
-- diagonal line: nothing moves across its own stroke). Prints where the blade
-- (the weapon body) crossed the chest's height going down and where it was
-- lowest, across (x), forward (y) and up (z) from the chest, in the player's
-- frame (facing +y). Compare OPENMW_RAGDOLL_HSC_STEER=0.
registerTest('STEER-01 steer a chop sideways with iron longsword', 918330, function()
    local weapon = 'iron longsword'
    local player = placePlayer()
    local ok, message = pcall(function()
        if not types.Actor.inventory(player):find(weapon) then
            world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 430, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 430 end)
        waitSeconds(1)
        powerAndStand(player, 'STEER-01 player')
        cameraReadyMode = nil
        player:sendEvent('fixPlayerCamera', true)
        waitFor('STEER-01 first-person camera', function() return cameraReadyMode == true end)
        ragdoll.prepareMouseInput()
        for rep = 1, 2 do
            for _, side in ipairs({ 0, 1200, -1200 }) do
                local p = ragdoll.getState(player)
                local pivot = (body(p, 'left_upper_arm').position + body(p, 'right_upper_arm').position) * 0.5
                local mid, low
                local function watch()
                    local st = ragdoll.getState(player)
                    if not (st and st.hsc and st.hsc.s and st.weapon) then return end
                    local w = st.bodies[st.weapon.bodyIndex].position - pivot
                    if st.hsc.s > 1.0 and w.z < 0 and not mid then mid = w end
                    if st.hsc.s > 1.0 and (not low or w.z < low.z) then low = w end
                end
                ragdoll.injectMouseInput(0, 0, true, 1)
                waitSeconds(0.25, watch)
                for _, leg in ipairs({ { x = 0, y = -HSC_WIND, seconds = 0.4 }, { x = 0, y = HSC_CUT / 2, seconds = 0.1 },
                    { x = side, y = HSC_CUT / 2, seconds = 0.15 } }) do
                    local started, sentX, sentY = core.getSimulationTime(), 0, 0
                    waitSeconds(leg.seconds, function()
                        local fraction = math.min(1, (core.getSimulationTime() - started) / leg.seconds)
                        local x, y = math.floor(leg.x * fraction), math.floor(leg.y * fraction)
                        ragdoll.injectMouseInput(x - sentX, y - sentY)
                        sentX, sentY = x, y
                        watch()
                    end)
                    ragdoll.injectMouseInput(leg.x - sentX, leg.y - sentY)
                end
                ragdoll.injectMouseInput(0, 0, false, 1)
                waitSeconds(0.6, watch)
                pcall(waitFor, 'STEER-01 withdrawal', function() watch(); return ragdoll.getSwingSpeed(player) == nil end, 4)
                print(string.format('STEER01 side=%d rep=%d cross=%.1f,%.1f,%.1f low=%.1f,%.1f,%.1f', side, rep,
                    mid and mid.x or 0, mid and mid.y or 0, mid and mid.z or 0, low and low.x or 0, low and low.y or 0,
                    low and low.z or 0))
                waitSeconds(0.8)
            end
        end
    end)
    ragdoll.injectMouseInput(0, 0, false)
    ragdoll.setPowered(player, false)
    if not ok then error(message, 0) end
end)

-- AIM-04 (aim up/down, user 10-05 build 57: "aim up/down (head, legs): more
-- control, maybe chest bending"): the look is put on the foe's head, chest,
-- thigh or calf, then a click or a forehand drag. AIM04_BEGIN/END lines frame
-- each attack; the RAGDOLL_IMPACT regions between them say what it struck
-- (tools/body/analysis/aim04.py). Compare OPENMW_RAGDOLL_HSC_AIM_ASSIST=0.
registerTest('AIM-04 look at head chest and legs with iron longsword', 918310, function()
    local weapon = 'iron longsword'
    local player = placePlayer()
    local victim
    local savedReaction = ragdoll.reactionScale()
    local ok, message = pcall(function()
        ragdoll.setPlayerAlwaysHits(true)
        if not types.Actor.inventory(player):find(weapon) then
            world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 410, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 410 end)
        waitSeconds(1)
        powerAndStand(player, 'AIM-04 player')
        cameraReadyMode = nil
        player:sendEvent('fixPlayerCamera', true)
        waitFor('AIM-04 first-person camera', function() return cameraReadyMode == true end)
        victim = spawnDummy('AIM-04 target', util.vector3(0, 200, 0), false)
        powerAndStand(victim, 'AIM-04 target')
        ragdoll.reactionScale(0)
        ragdoll.prepareMouseInput()
        for _, mode in ipairs({ 'click', 'drag' }) do
            for rep = 1, 2 do
                for _, part in ipairs({ 'head', 'spine2', 'left_thigh', 'left_calf' }) do
                    if types.Actor.isDead(victim) or not ragdoll.getState(victim) then
                        victim:remove()
                        coroutine.yield()
                        victim = spawnDummy('AIM-04 target', util.vector3(0, 200, 0), false)
                        powerAndStand(victim, 'AIM-04 new target')
                    end
                    local p = ragdoll.getState(player)
                    local shoulder = body(p, 'right_upper_arm').position
                    local elbow = body(p, 'right_forearm').position
                    local hand = body(p, 'right_hand').position
                    local reach = (elbow - shoulder):length() + (hand - elbow):length()
                    local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
                    local desired = pivot + util.vector3(0, reach + p.weapon.length * 0.55, 0)
                    placeVictimPoint(victim, function() return body(ragdoll.getState(victim), 'spine2').position end,
                        desired, util.transform.rotateZ(math.pi))
                    waitSeconds(0.3)
                    contactAimReady = false
                    player:sendEvent('fixAimAt', body(ragdoll.getState(victim), part).position)
                    pcall(waitFor, 'AIM-04 look', function() return contactAimReady end, 4)
                    print(string.format('AIM04_BEGIN mode=%s part=%s rep=%d', mode, part, rep))
                    ragdoll.injectMouseInput(0, 0, true, 1)
                    if mode == 'click' then
                        waitSeconds(0.08)
                    else
                        waitSeconds(0.25)
                        for _, leg in ipairs({ { x = HSC_WIND, y = 0, seconds = 0.4 }, { x = -HSC_CUT, y = 0, seconds = 0.2 } }) do
                            local started, sentX = core.getSimulationTime(), 0
                            waitSeconds(leg.seconds, function()
                                local fraction = math.min(1, (core.getSimulationTime() - started) / leg.seconds)
                                local x = math.floor(leg.x * fraction)
                                ragdoll.injectMouseInput(x - sentX, 0)
                                sentX = x
                            end)
                            ragdoll.injectMouseInput(leg.x - sentX, 0)
                        end
                    end
                    ragdoll.injectMouseInput(0, 0, false, 1)
                    waitSeconds(0.35)
                    pcall(waitFor, 'AIM-04 withdrawal', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                    print(string.format('AIM04_END mode=%s part=%s rep=%d', mode, part, rep))
                    waitSeconds(0.6)
                end
            end
        end
    end)
    ragdoll.injectMouseInput(0, 0, false)
    ragdoll.reactionScale(savedReaction)
    ragdoll.setPlayerAlwaysHits(false)
    ragdoll.setPowered(player, false)
    if victim and victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)

-- AIM-03 (whiffs, user 10-05 build 57: "chops whiff and side slashes don't
-- connect where they should"): a foe off the facing at the distances the user
-- missed from (play 10-05/06: half the misses passed within 35 units of the
-- foe's body). The target's chest is put at 0, +-15 and +-25 deg from the
-- player's facing, at the blade's sweet spot (reach + 0.55 of the blade) and
-- near its tip (0.8); each gets a click (the controller's own attack) and a
-- forehand drag. Measures what bit (AIM03 lines, tools/body/analysis/aim03.py);
-- compare OPENMW_RAGDOLL_HSC_AIM_ASSIST=0.
for index, weapon in ipairs({ 'iron longsword', 'iron claymore' }) do
    registerTest('AIM-03 off-centre foe with ' .. weapon, 918300 + index, function()
        local player = placePlayer()
        local victim
        local savedReaction = ragdoll.reactionScale()
        local ok, message = pcall(function()
            ragdoll.setPlayerAlwaysHits(true)
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 400 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 400 + index end)
            waitSeconds(1)
            powerAndStand(player, 'AIM-03 player')
            cameraReadyMode = nil
            player:sendEvent('fixPlayerCamera', true)
            waitFor('AIM-03 first-person camera', function() return cameraReadyMode == true end)
            victim = spawnDummy('AIM-03 target', util.vector3(0, 200, 0), false)
            powerAndStand(victim, 'AIM-03 target')
            ragdoll.reactionScale(0)
            ragdoll.prepareMouseInput()
            local function damageTotal()
                local total = 0
                for _, value in pairs(ragdoll.getLimbDamage(victim)) do total = total + value end
                return total
            end
            local hits, attacks = 0, 0
            for _, mode in ipairs({ 'click', 'drag' }) do
                for _, spot in ipairs({ 0.55, 0.8 }) do
                    for _, angle in ipairs({ 0, 15, -15, 25, -25 }) do
                        -- A claymore kills the dummy in about four blows: a new one.
                        if types.Actor.isDead(victim) or not ragdoll.getState(victim) then
                            victim:remove()
                            coroutine.yield()
                            victim = spawnDummy('AIM-03 target', util.vector3(0, 200, 0), false)
                            powerAndStand(victim, 'AIM-03 new target')
                        end
                        local p = ragdoll.getState(player)
                        local shoulder = body(p, 'right_upper_arm').position
                        local elbow = body(p, 'right_forearm').position
                        local hand = body(p, 'right_hand').position
                        local reach = (elbow - shoulder):length() + (hand - elbow):length()
                        local pivot = (body(p, 'left_upper_arm').position + shoulder) * 0.5
                        local d = reach + p.weapon.length * spot
                        local a = math.rad(angle)
                        local desired = pivot + util.vector3(math.sin(a) * d, math.cos(a) * d, 0)
                        placeVictimPoint(victim, function()
                            local vs = ragdoll.getState(victim)
                            return vs and body(vs, 'spine2').position or victim.position
                        end, desired, util.transform.rotateZ(math.pi + a))
                        waitSeconds(0.3)
                        local vs = ragdoll.getState(victim)
                        if not (vs and vs.controller and vs.controller.mode == 'standing') then
                            ragdoll.setPowered(victim, false)
                            coroutine.yield()
                            powerAndStand(victim, 'AIM-03 replaced target')
                            placeVictimPoint(victim, function()
                                local vs = ragdoll.getState(victim)
                                return vs and body(vs, 'spine2').position or victim.position
                            end, desired, util.transform.rotateZ(math.pi + a))
                        end
                        local before = damageTotal()
                        ragdoll.injectMouseInput(0, 0, true, 1)
                        if mode == 'click' then
                            waitSeconds(0.08)
                        else
                            waitSeconds(0.25)
                            for _, leg in ipairs({ { x = HSC_WIND, y = 0, seconds = 0.4 }, { x = -HSC_CUT, y = 0, seconds = 0.2 } }) do
                                local started, sentX = core.getSimulationTime(), 0
                                waitSeconds(leg.seconds, function()
                                    local fraction = math.min(1, (core.getSimulationTime() - started) / leg.seconds)
                                    local x = math.floor(leg.x * fraction)
                                    ragdoll.injectMouseInput(x - sentX, 0)
                                    sentX = x
                                end)
                                ragdoll.injectMouseInput(leg.x - sentX, 0)
                            end
                        end
                        ragdoll.injectMouseInput(0, 0, false, 1)
                        waitSeconds(0.35)
                        pcall(waitFor, 'AIM-03 withdrawal', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                        local damage = damageTotal() - before
                        attacks = attacks + 1
                        if damage > 0 then hits = hits + 1 end
                        local vsAfter = ragdoll.getState(victim)
                        print(string.format('AIM03 weapon=%s mode=%s spot=%.2f angle=%d hit=%d damage=%.3f dist=%.1f',
                            weapon, mode, spot, angle, damage > 0 and 1 or 0, damage,
                            vsAfter and (body(vsAfter, 'spine2').position - pivot):length() or -1))
                        waitSeconds(0.6)
                    end
                end
            end
            print(string.format('AIM03_TOTAL weapon=%s hits=%d of %d', weapon, hits, attacks))
            testing.expect(hits > 0, 'no off-centre attack bit at all')
        end)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.reactionScale(savedReaction)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if victim and victim:isValid() then victim:remove() end
        if not ok then error(message, 0) end
    end)
end

-- AIM-01 (aiming review 2026-09-30): single left clicks at a standing
-- dummy's chest with the crosshair on it: at the depth the blade's sweet spot
-- reaches (0.8 of the arm plus 3/4 of hand to tip), nearer, farther and to
-- either side. Reports how close the blade (from 0.4 of the way to the tip
-- out) came to the aimed point until its first contact, where along the
-- blade, and whether the blow struck. A chest is ~12 units deep, so a blow
-- that lands measures about that.
local aimPlacements = {
    { name = 'mid', depth = 0, side = 0 },
    { name = 'near', depth = -20, side = 0 },
    { name = 'far', depth = 15, side = 0 },
    { name = 'left', depth = 0, side = -25 },
    { name = 'right', depth = 0, side = 25 },
}
-- The user plays mostly in third person (aim analysis 2026-09-29): the
-- claymore and longsword also run with the camera behind.
local aimLoadouts = {}
for _, weapon in ipairs({ 'iron dagger', 'iron longsword', 'iron claymore', 'iron warhammer', 'iron battle axe' }) do
    aimLoadouts[#aimLoadouts + 1] = { weapon = weapon, firstPerson = true }
end
for _, weapon in ipairs({ 'iron longsword', 'iron claymore' }) do
    aimLoadouts[#aimLoadouts + 1] = { weapon = weapon, firstPerson = false }
end
for index, loadout in ipairs(aimLoadouts) do
    local weapon = loadout.weapon
    registerTest('AIM-01 aimed clicks with ' .. weapon .. (loadout.firstPerson and '' or ' third person'), 918300 + index, function()
        local player = placePlayer()
        local victim
        local savedReaction = ragdoll.reactionScale()
        local ok, message = pcall(function()
            ragdoll.setPlayerAlwaysHits(true)
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 300 + index, firstPerson = loadout.firstPerson })
            waitFor('player loadout', function() return playerReadySequence == 300 + index end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            victim = spawnDummy('AIM-01 target', util.vector3(0, 200, 0), false)
            powerAndStand(victim, 'aim target')
            ragdoll.reactionScale(0)
            ragdoll.prepareMouseInput()
            local landed, misses = 0, {}
            for _, placement in ipairs(aimPlacements) do
                waitFor('guard before the click', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                local p = ragdoll.getState(player)
                local shoulder = body(p, 'right_upper_arm').position
                local elbow = body(p, 'right_forearm').position
                local hand = body(p, 'right_hand').position
                local reach = (elbow - shoulder):length() + (hand - elbow):length()
                local wb = p.bodies[p.weapon.bodyIndex]
                local tip = wb.position + (wb.rotation * util.vector3(0, 0, 1)) * (p.weapon.length * 0.5)
                local bladeReach = (tip - hand):length()
                local desired = shoulder
                    + util.vector3(placement.side, 0.8 * reach + 0.75 * bladeReach + placement.depth, 0)
                placeVictimPoint(victim, function() return body(ragdoll.getState(victim), 'spine2').position end,
                    desired, util.transform.rotateZ(math.pi))
                waitSeconds(0.5)
                contactAimReady = false
                player:sendEvent('fixAimAt', body(ragdoll.getState(victim), 'spine2').position)
                waitFor('crosshair on the chest', function() return contactAimReady end, 4)
                local target = body(ragdoll.getState(victim), 'spine2').position
                local w0 = ragdoll.getState(player).weapon
                local contacts0, strikes0 = w0.contactEventCount, w0.strikeEventCount
                local best, bestAlong, contacted = math.huge, 0, false
                local function observe()
                    if contacted then return end
                    local s = ragdoll.getState(player)
                    local w = s.weapon
                    local b = s.bodies[w.bodyIndex]
                    local h = body(s, 'right_hand').position
                    local t = b.position + (b.rotation * util.vector3(0, 0, 1)) * (w.length * 0.5)
                    local axis = t - h
                    local length2 = axis:dot(axis)
                    if length2 > 1 then
                        local along = math.max(0.4, math.min(1, (target - h):dot(axis) / length2))
                        local miss = (h + axis * along - target):length()
                        if miss < best then best, bestAlong = miss, along end
                    end
                    if w.contactEventCount > contacts0 then contacted = true end
                end
                ragdoll.injectMouseInput(0, 0, true, 1)
                waitSeconds(0.1, observe)
                ragdoll.injectMouseInput(0, 0, false, 1)
                waitSeconds(1.2, observe)
                local w = ragdoll.getState(player).weapon
                local strikes = w.strikeEventCount - strikes0
                if strikes > 0 then landed = landed + 1 end
                misses[#misses + 1] = best
                print(string.format('AIM-01 %s%s place=%s miss=%.1f along=%.2f contacted=%s strikes=%d',
                    weapon, loadout.firstPerson and '' or ' third', placement.name, best, bestAlong, tostring(contacted), strikes))
            end
            table.sort(misses)
            print(string.format('AIM-01 %s%s summary landed=%d of %d median_miss=%.1f worst_miss=%.1f', weapon,
                loadout.firstPerson and '' or ' third', landed,
                #aimPlacements, misses[math.ceil(#misses / 2)], misses[#misses]))
            testing.expect(landed >= 3, string.format('%s landed %d of %d aimed clicks', weapon, landed, #aimPlacements))
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.reactionScale(savedReaction)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if victim and victim:isValid() then victim:remove() end
        if not ok then error(message, 0) end
    end)
end

-- AIM-02 (aiming review 2026-09-30): a drag moves the blade the mouse's way
-- from its first moment. In play (Release A recordings) drags from rest went
-- the mouse's way at cosine 0.53 with the claymore, 0.29 with the spear, and
-- about 0 when started right after a swing. Two rounds of four directions:
-- from rest, then each drag 0.15 s after the last one's release ("busy").
-- Reports, per drag, the cosine between the tip's travel on the screen plane
-- and the mouse's over the first 0.2 s.
for index, weapon in ipairs({ 'iron dagger', 'iron longsword', 'iron claymore', 'iron spear', 'wooden staff' }) do
    registerTest('AIM-02 drags start the mouse way with ' .. weapon, 918320 + index, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 320 + index, firstPerson = true })
            waitFor('player loadout', function() return playerReadySequence == 320 + index end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local function tipNow()
                local s = ragdoll.getState(player)
                local b = s.bodies[s.weapon.bodyIndex]
                return b.position + (b.rotation * util.vector3(0, 0, 1)) * (s.weapon.length * 0.5)
            end
            local totals = { rest = { 0, 0 }, busy = { 0, 0 } }
            -- Injected mouse y is down.
            local directions = { { name = 'right', x = 1, y = 0 }, { name = 'left', x = -1, y = 0 },
                { name = 'up', x = 0, y = -1 }, { name = 'down', x = 0, y = 1 } }
            for _, round in ipairs({ 'rest', 'busy' }) do
                if round == 'rest' then
                    -- nothing: each drag below waits for the guard
                end
                for _, drag in ipairs(directions) do
                    if round == 'rest' then
                        waitFor('guard before the drag', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                        waitSeconds(0.8)
                    else
                        waitSeconds(0.15)
                    end
                    -- Injected input reaches the physics about ten steps after it
                    -- is sent: start counting once the press has landed, send
                    -- the motion over 12 physics steps (0.2 s) and measure 2
                    -- steps after the last of it was sent (it lands 2 steps
                    -- after the press). Timed on Lua frames, the press lasted 2
                    -- physics steps and most of the motion came after release.
                    local presses = ragdoll.getState(player).playerPressCount
                    ragdoll.injectMouseInput(0, 0, true, 1)
                    waitFor('press registered', function() return ragdoll.getState(player).playerPressCount > presses end, 3)
                    local start = tipNow()
                    local counts, step0, sentX, sentY = 400, tonumber(ragdoll.getState(player).physicsStep), 0, 0
                    waitFor('drag steps', function()
                        local steps = tonumber(ragdoll.getState(player).physicsStep) - step0
                        local fraction = math.min(1, steps / 12)
                        local x, y = math.floor(drag.x * counts * fraction), math.floor(drag.y * counts * fraction)
                        ragdoll.injectMouseInput(x - sentX, y - sentY)
                        sentX, sentY = x, y
                        return steps >= 14
                    end, 5)
                    -- The player faces +y: the screen's right is +x, up is +z.
                    local moved = tipNow() - start
                    local screen = util.vector2(moved.x, moved.z)
                    local cosine = screen:length() > 0.5 and screen:dot(util.vector2(drag.x, -drag.y)) / screen:length() or 0
                    print(string.format('AIM-02 %s %s drag=%s cos=%.2f moved=%.1f', weapon, round, drag.name, cosine, moved:length()))
                    totals[round][1] = totals[round][1] + cosine
                    totals[round][2] = totals[round][2] + 1
                    ragdoll.injectMouseInput(0, 0, false, 1)
                    -- the release must land before the next press
                    waitFor('release registered', function()
                        return tonumber(ragdoll.getState(player).physicsStep) - step0 >= 16
                    end, 3)
                end
            end
            local rest = totals.rest[1] / totals.rest[2]
            local busy = totals.busy[1] / totals.busy[2]
            print(string.format('AIM-02 %s summary rest_cos=%.2f busy_cos=%.2f', weapon, rest, busy))
            testing.expect(rest >= 0.6, string.format('%s drags from rest started the mouse way %.2f', weapon, rest))
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end)
end

registerTest('FIX-08 equipping dagger after player activation attaches a strike surface', 918023, function()
    local player = placePlayer()
    local ok, message = pcall(function()
        player:sendEvent('fixPlayerConfigure', { sequence = 123 })
        waitFor('unarmed loadout', function() return playerReadySequence == 123 end)
        waitSeconds(0.5)
        powerAndStand(player, 'unarmed player')
        if not types.Actor.inventory(player):find('iron dagger') then
            world.createObject('iron dagger', 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        player:sendEvent('fixPlayerConfigure', { weapon = 'iron dagger', sequence = 124 })
        waitFor('dagger loadout', function() return playerReadySequence == 124 end)
        waitFor('live dagger attachment', function()
            local state = ragdoll.getState(player)
            return state and state.weapon and state.weapon.attached and state.weapon.recordId == 'iron dagger'
        end, 3)
    end)
    ragdoll.setPowered(player, false)
    if not ok then error(message, 0) end
end)

for index, weapon in ipairs({ false, 'iron dagger' }) do
    registerTest('FIX-09 manual player setup from first person with ' .. (weapon or 'fists'), 918030 + index, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            player:sendEvent('fixPlayerConfigure', { firstPerson = true, sequence = 130 + index })
            waitFor('first-person initial state', function() return playerReadySequence == 130 + index end)
            waitSeconds(0.5)
            core.sendGlobalEvent('ragdollManualControl', { action = weapon and 'player_dagger' or 'player_fists' })
            local state = waitFor('manual physical player setup', function()
                local current = ragdoll.getState(player)
                return current and current.powered and current.controller
                    and current.controller.mode == 'standing' and current
            end)
            testing.expect(weapon and state.weapon and state.weapon.attached or not weapon and not state.weapon,
                'manual player setup did not attach the requested contact surface')
        end)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end)
end

-- These are receipt/render/controller diagnostics, separately from FIX-08's
-- real native mouse contacts and the core dagger surface classifier tests.
for index, case in ipairs({
    { name = 'slash left forearm', kind = 2, region = 'left_forearm', health = 4, mode = 'normal' },
    { name = 'puncture right forearm', kind = 3, region = 'right_forearm', health = 4, mode = 'normal' },
    { name = 'fatigue punch bruise', kind = 1, region = 'left_upper_arm', fatigue = 20, mode = 'normal' },
    { name = 'face punch bruise', kind = 1, region = 'head', fatigue = 20, mode = 'normal', face = true },
    { name = 'zero damage no wound', kind = 2, region = 'left_forearm', health = 4, mode = 'zero' },
    { name = 'cancelled hit no wound', kind = 2, region = 'left_forearm', health = 4, mode = 'cancel' },
    { name = 'duplicate receipt one debit', kind = 3, region = 'right_forearm', health = 4, mode = 'duplicate' },
    { name = 'slash thin hand surface', kind = 2, region = 'left_hand', health = 4, mode = 'normal', offset = util.vector3(3, 0, 0) },
    { name = 'distant surface no mark', kind = 2, region = 'left_hand', health = 4, mode = 'normal', offset = util.vector3(50, 0, 0), project = false },
}) do
    registerTest('DMG-03 wounds ' .. case.name, 918100 + index, function()
        placePlayer()
        local victim = spawnDummy('DMG-03 wound target', util.vector3(40, 90, 0), false)
        local ok, message = pcall(function()
            victim:sendEvent('fixWoundMode', { mode = case.mode })
            waitSeconds(0.5)
            powerAndStand(victim, 'wound target')
            waitSeconds(2) -- settle activation motion and ensure the mesh has been culled/skinned
            local state = ragdoll.getState(victim)
            local struck = body(state, case.region)
            local parentName
            for _, joint in ipairs(state.constraints) do
                if joint.childBody == case.region then parentName = joint.parentBody end
            end
            local function relativeSpeed(current)
                local child = body(current, case.region).angularVelocity
                local parent = parentName and body(current, parentName).angularVelocity or util.vector3(0, 0, 0)
                return (child - parent):length()
            end
            local baselineSpeed = 0
            waitSeconds(0.25, function()
                baselineSpeed = math.max(baselineSpeed, relativeSpeed(ragdoll.getState(victim)))
            end)
            local normal = util.vector3(0, case.face and 1 or -1, 0)
            local point = body(ragdoll.getState(victim), case.region).centerOfMass + normal * 5 + (case.offset or util.vector3(0, 0, 0))
            world.players[1]:sendEvent('fixWoundCamera', { position = point + normal * 45, yaw = case.face and math.pi or 0 })
            waitSeconds(0.25)
            -- Camera setup advances physics. Stage against the current body,
            -- just as the real contact callback does, not the half-second-old pose.
            point = body(ragdoll.getState(victim), case.region).centerOfMass + normal * 5 + (case.offset or util.vector3(0, 0, 0))
            local id = ragdoll.stageWoundReceipt(victim, case.region, case.kind, point, normal)
            testing.expect(id > 0, 'failed to stage native receipt diagnostic')
            local before = types.Actor.stats.dynamic.health(victim).current
            victim:sendEvent('Hit', {
                physicalHitId = id, successful = true, sourceType = 'melee',
                hitPos = point, damage = { health = case.health or 0, fatigue = case.fatigue or 0 },
                ignoreStagger = true,
            })
            local peak, physicalSpeed = 0, 0
            waitSeconds(0.65, function()
                local current = ragdoll.getState(victim)
                testing.expect(current and current.powered, 'wound destroyed the powered rig')
                peak = math.max(peak, current.localReactionAngle)
                physicalSpeed = math.max(physicalSpeed, relativeSpeed(current))
                testing.expect(current.controller.mode ~= 'falling' and current.controller.mode ~= 'fallen',
                    'small local wound caused a fall')
            end)
            local after = ragdoll.getState(victim)
            local loss = before - types.Actor.stats.dynamic.health(victim).current
            local accepted = case.mode ~= 'zero' and case.mode ~= 'cancel'
            print(string.format('DMG-03_WOUND_OBSERVED case=%s marks=%d peak=%.4f baseline=%.4f response=%.4f',
                case.name, after.woundMarks, peak, baselineSpeed, physicalSpeed))
            testing.expect(after.woundReceipts == (case.mode == 'cancel' and 0 or 1), 'receipt not consumed exactly once')
            testing.expect(math.abs(loss - (accepted and (case.health or 0) or 0)) < 0.01, 'health debit differs from resolved hit')
            if accepted then
                testing.expect(after.woundMarks == (case.project == false and 0 or 1), 'wound projection did not respect the surface/distance control')
                -- (The old local reaction angle went with M8, build 70: V2's controller gives the part.)
                -- (A receipt moved no joint since M8: V2 reacts to the blow itself, REACT-02.)
                testing.expect(after.lastWoundRegion == case.region, 'reaction selected the wrong limb')
            else
                testing.expect(after.woundMarks == 0 and peak == 0, 'cancelled/zero hit left a wound or flinch')
            end
            testing.expect(after.localReactionAngle == 0, 'transient reaction failed to recover')
            print(string.format('DMG-03_WOUND case=%s receipts=%d marks=%d health_loss=%.3f peak_angle=%.4f baseline_speed=%.4f hit_speed=%.4f kind=%s region=%s',
                case.name, after.woundReceipts, after.woundMarks, loss, peak, baselineSpeed, physicalSpeed,
                after.lastWoundKind, after.lastWoundRegion))
            if accepted and case.project ~= false then
                -- Projection must keep working after the first overlay: a
                -- cloned drawable must not tag the source mesh as a wound.
                local secondPoint = body(after, case.region).centerOfMass + normal * 5 + util.vector3(1, 0, 0)
                local secondId = ragdoll.stageWoundReceipt(victim, case.region, case.kind, secondPoint, normal)
                victim:sendEvent('Hit', {
                    physicalHitId = secondId, successful = true, sourceType = 'melee',
                    hitPos = secondPoint, damage = { health = case.health or 0, fatigue = case.fatigue or 0 },
                    ignoreStagger = true,
                })
                waitSeconds(0.65)
                testing.expect(ragdoll.getState(victim).woundMarks == 2, 'second hit could not mark the same mesh')
            end
            waitSeconds(1) -- bounded close-up for the harness's optional video capture
        end)
        world.players[1]:sendEvent('fixWoundCamera', {})
        victim:remove()
        if not ok then error(message, 0) end
    end)
end

local function prepareContactPlayer(weapon, sequence, firstPerson)
    local player = placePlayer()
    if weapon and not types.Actor.inventory(player):find(weapon) then
        world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
        coroutine.yield()
    end
    -- Rig creation needs the complete third-person skeleton. The ordinary
    -- manual activation route has the same precondition (FIX-09 covers entry
    -- from first person). Switch view only after the physical rig is ready.
    player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = sequence })
    waitFor('contact player equipment', function() return playerReadySequence == sequence end)
    waitSeconds(0.5)
    powerAndStand(player, 'contact player')
    if firstPerson then
        cameraReadyMode = nil
        player:sendEvent('fixPlayerCamera', true)
        waitFor('first-person camera', function() return cameraReadyMode == true end)
    end
    ragdoll.prepareMouseInput()
    waitSeconds(0.5)
    return player
end


playerSwimming = false
-- TELEPORT-01 (play 2026-10-01: "can't go into interior cells" - a door
-- moved the player but not its powered body, which dragged the player back
-- to the exterior's XY inside the interior: into the void every frame): the
-- powered player is teleported out to Balmora, then back into the test floor
-- interior. Each time it must stand where it was put (no drift back, no
-- fall) and be powered again within 3 s.
registerTest('TELEPORT-01 a powered player goes through doors', 918397, function()
    local player = placePlayer()
    local ok, message = pcall(function()
        player:sendEvent('fixPlayerConfigure', { sequence = 397, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 397 end)
        powerAndStand(player, 'teleport')
        local function hop(label, cell, position)
            player:teleport(cell, position, { onGround = true })
            waitFor(label .. ' arrival', function()
                local d = player.position - position
                return math.sqrt(d.x * d.x + d.y * d.y) < 50
            end, 5)
            waitSeconds(0.5)
            local landed = player.position.z
            local lowest, farthest = math.huge, 0
            waitSeconds(3, function()
                if player.cell then
                    lowest = math.min(lowest, player.position.z - landed)
                    local d = player.position - position
                    farthest = math.max(farthest, math.sqrt(d.x * d.x + d.y * d.y))
                end
            end)
            local s = ragdoll.getState(player)
            print(string.format('TELEPORT-01 %s cell=%s drift=%.0f lowest=%.0f powered=%s', label,
                tostring(player.cell and player.cell.name), farthest, lowest, tostring(s and s.powered)))
            testing.expect(farthest < 100, label .. ': the player was dragged away from where it arrived')
            testing.expect(lowest > -300, label .. ': the player fell')
            testing.expect(s and s.powered, label .. ': the body was not powered again')
        end
        hop('to Balmora', '', util.vector3(-23000, -15400, 1100))
        hop('back inside', testArea.cell, testArea.player)
    end)
    ragdoll.setPowered(player, false)
    if not ok then error(message, 0) end
end)

-- TELEPORT-02 (play 10-30 bookmark "I fell down and I cant get up"): through
-- the Smuggler Den door into Ashinabi with a claymore out, the re-powered
-- body fell at the arrival and lay there. The player arrives at that door's
-- spot (and the test floor's for comparison); the controller must be
-- standing again within 6 s. TELEPORT02 rows: the modes seen, in order.
registerTest('TELEPORT-02 standing again after a door', 918398, function()
    local player = placePlayer()
    local ok, message = pcall(function()
        local weapon = 'steel claymore'
        if not types.Actor.inventory(player):find(weapon) then
            world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 398, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 398 end)
        powerAndStand(player, 'teleport')
        local failures = {}
        for _, hop in ipairs({ { 'Ashinabi door', 'Ashinabi', util.vector3(-7573, 1440, 432) },
            { 'test floor', testArea.cell, testArea.player }, { 'Ashinabi door again', 'Ashinabi', util.vector3(-7573, 1440, 432) } }) do
            player:teleport(hop[2], hop[3], { onGround = true })
            local modes, last, standingFor = {}, nil, 0
            waitSeconds(6, function()
                local s = ragdoll.getState(player)
                local m = s and s.powered and s.controller and s.controller.mode or (s and 'unpowered' or 'none')
                if m ~= last then modes[#modes + 1] = m last = m end
                standingFor = m == 'standing' and standingFor + 1 or 0
            end)
            local s = ragdoll.getState(player)
            local torso = s and s.bodies and body(s, 'spine2')
            print(string.format('TELEPORT02 at=%s cell=%s modes=%s standing_frames=%d chest_above_feet=%.0f',
                hop[1], tostring(player.cell and player.cell.name), table.concat(modes, '>'), standingFor,
                torso and (torso.position.z - player.position.z) or -1))
            if standingFor < 30 then failures[#failures + 1] = hop[1] .. ': not standing (' .. table.concat(modes, '>') .. ')' end
        end
        testing.expect(#failures == 0, table.concat(failures, '; '))
    end)
    ragdoll.setPowered(player, false)
    if not ok then error(message, 0) end
end)

-- SHIP-01 (user 10-30 playthrough: "the guard that greets you right after
-- Jiub wasnt able to climb the stairs"): in the Imperial Prison Ship a
-- powered NPC (the chargen guard's record, armed as it is) walks from the
-- guard's spot to the door up on deck along the navmesh route. SHIP01 rows:
-- the cell's NPCs and doors (once), then the walker's height and distance to
-- the door every second, and whether it got there (within 80 units, 30 s).
registerTest('SHIP-01 a powered NPC climbs the prison ship stairs', 918399, function()
    local player = placePlayer()
    local walker
    local ok, message = pcall(function()
        local name = 'Imperial Prison Ship'
        -- Character creation is over: the ship's chargen scripts otherwise
        -- greet the player near the guards, and the dialogue pauses the game.
        pcall(function() world.mwscript.getGlobalVariables(player).chargenstate = -1 end)
        player:sendEvent('fixCloseMenus', true)
        -- Spawned where the player is (an actor's script runs only in an
        -- active cell), then taken along to the ship.
        walker = spawnDummy('SHIP-01 walker', util.vector3(0, 200, 0), false)
        player:teleport(name, util.vector3(0, 0, 0), { onGround = true })
        waitFor('player on the ship', function() return player.cell and player.cell.name == name end, 10)
        waitSeconds(1)
        local guard, door
        for _, actor in ipairs(world.activeActors) do
            if actor ~= player and actor.cell and actor.cell.name == name then
                print(string.format('SHIP01 npc=%s at=%s', actor.recordId, tostring(actor.position)))
                if not guard and actor.recordId:find('guard') then guard = actor end
            end
        end
        for _, d in ipairs(player.cell:getAll(types.Door)) do
            local dest = types.Door.destCell(d)
            print(string.format('SHIP01 door at=%s to=%s', tostring(d.position), dest and dest.name or '-'))
            if dest and not dest.isInterior then door = d end
        end
        testing.expect(guard ~= nil and door ~= nil, 'SHIP-01: no guard or no outside door on the ship')
        local start = guard.position
        guard:teleport('', util.vector3(0, 0, -10000))
        player:teleport(name, start + util.vector3(-60, 0, 0), { onGround = true })
        waitSeconds(0.5)
        walker:teleport(name, start, { onGround = true })
        waitSeconds(0.5)
        powerAndStand(walker, 'SHIP-01 walker')
        roughRoute, roughDone = nil, false
        -- OPENMW_RAGDOLL_TEST_SHIP_AI=1: the game's AI travel walks it (as
        -- the chargen guard), else the navmesh route steered by the script.
        local aiTravel = ragdoll.tuningText('OPENMW_RAGDOLL_TEST_SHIP_AI') == '1'
        if aiTravel then
            walker:sendEvent('fixActorTravel', door.position)
            roughRoute = { points = 0, status = 'ai travel' }
        else
            walker:sendEvent('fixActorRoute', door.position)
        end
        waitFor('SHIP-01 route', function() return roughRoute ~= nil end, 5)
        print(string.format('SHIP01 route points=%d status=%s from=%s to=%s path=%s', roughRoute.points, roughRoute.status,
            tostring(start), tostring(door.position), tostring(roughRoute.list)))
        local started, last, best = core.getSimulationTime(), -1, 1e9
        -- OPENMW_RAGDOLL_TEST_SHIP_VANILLA=1: the walker unpowered (the
        -- vanilla capsule), for comparison.
        if ragdoll.tuningText('OPENMW_RAGDOLL_TEST_SHIP_VANILLA') == '1' then ragdoll.setPowered(walker, false) end
        local onward, highest = false, start.z
        waitSeconds(30, function()
            if not walker:isValid() then return end
            -- Past the route's end: straight on to the door.
            if roughDone and not onward and not aiTravel then
                onward = true
                walker:sendEvent('fixActorPoints', { door.position })
            end
            highest = math.max(highest, walker.position.z)
            local d = (walker.position - door.position):length()
            best = math.min(best, d)
            local now = core.getRealTime()
            if math.floor(now) ~= last then
                last = math.floor(now)
                local s = ragdoll.getState(walker)
                print(string.format('SHIP01 sim=%.2f at=%.0f,%.0f,%.0f to_door=%.0f mode=%s',
                    core.getSimulationTime() - started, walker.position.x, walker.position.y, walker.position.z, d,
                    s and s.controller and s.controller.mode or (s and 'unpowered' or 'none')))
            end
        end)
        print(string.format('SHIP01 reached=%s closest=%.0f highest=%.0f', tostring(best < 80), best, highest))
        -- The scripted walk ends against the wall where the stairs turn (a
        -- vanilla capsule stops there too, at z 198): the bar is the climb.
        testing.expect(highest >= start.z + 80, string.format('SHIP-01: the walker climbed only to %.0f', highest))
    end)
    player:sendEvent('fixCloseMenus', false)
    if walker and walker:isValid() then walker:remove() end
    if not ok then error(message, 0) end
end)

-- SHIP-02 (user 10-31, the release build: "its really hard to climb the
-- stairs in the beginning at the ship"): the powered player, unarmed as at
-- character creation, walks from the chargen guard's spot along the navmesh
-- route to the deck door (then straight to it), 30 s, at a walk; then the
-- same unpowered (the vanilla capsule) for comparison. SHIP02 rows every
-- second: height, distance to the door, controller mode; the summary: the
-- highest point, the closest to the door, when the climb (+80) was made,
-- seconds stalled (moving < 10 u/s while asked to walk) and the modes seen.
registerTest('SHIP-02 the player climbs the prison ship stairs', 919540, function()
    local player = placePlayer()
    local ok, message = pcall(function()
        local name = 'Imperial Prison Ship'
        -- Character creation over (the Morrowind global is CharGenState; a
        -- lower-case key did nothing and walking to the stairs opened the
        -- name menu, which left the game paused).
        local globals = world.mwscript.getGlobalVariables(player)
        pcall(function() globals.CharGenState = -1 end)
        print('SHIP02 CharGenState=' .. tostring(globals.CharGenState))
        player:sendEvent('fixCloseMenus', true)
        player:sendEvent('fixPlayerConfigure', { sequence = 960, firstPerson = false, sheathed = true })
        waitFor('player loadout', function() return playerReadySequence == 960 end)
        player:teleport(name, util.vector3(0, 0, 0), { onGround = true })
        waitFor('player on the ship', function() return player.cell and player.cell.name == name end, 10)
        waitSeconds(1)
        local guard, door
        for _, actor in ipairs(world.activeActors) do
            if actor ~= player and actor.cell and actor.cell.name == name and actor.recordId:find('guard') then
                guard = guard or actor
            end
        end
        for _, d in ipairs(player.cell:getAll(types.Door)) do
            local dest = types.Door.destCell(d)
            if dest and not dest.isInterior then door = d end
        end
        testing.expect(guard ~= nil and door ~= nil, 'SHIP-02: no guard or no outside door on the ship')
        local start = guard.position
        guard:teleport('', util.vector3(0, 0, -10000))
        -- The route from the guard's spot; the climb starts at its last point
        -- on the hold's floor (walking the hold to it trips the chargen name
        -- menu, which leaves the game paused).
        player:teleport(name, start, { onGround = true })
        waitSeconds(1)
        shipPlayerRoute = nil
        player:sendEvent('fixPlayerRoute', { dest = door.position })
        waitFor('SHIP-02 route', function() return shipPlayerRoute ~= nil end, 5)
        player:sendEvent('fixPlayerRouteStop')
        local foot, footIndex = start, 1
        for i, p in ipairs(shipPlayerRoute.list or {}) do
            print(string.format('SHIP02 route %d at=%.0f,%.0f,%.0f', i, p.x, p.y, p.z))
            if p.z < start.z + 20 then foot, footIndex = p, i end
        end
        for _, powered in ipairs({ true, false }) do
            player:teleport(name, foot, { onGround = true })
            waitSeconds(1)
            if powered then powerAndStand(player, 'SHIP-02 player') else ragdoll.setPowered(player, false) end
            waitSeconds(1)
            shipPlayerRoute = nil
            player:sendEvent('fixPlayerRoute', { dest = door.position, extra = { door.position }, from = footIndex + 1 })
            waitFor('SHIP-02 route', function() return shipPlayerRoute ~= nil end, 5)
            local began = core.getSimulationTime()
            local z0 = player.position.z
            local highest, best, climbedAt, stalled, last, lastPos = z0, 1e9, -1, 0, -1, player.position
            local modes = {}
            waitSeconds(30, function()
                local now = core.getSimulationTime() - began
                local p = player.position
                highest = math.max(highest, p.z)
                local d = (p - door.position):length()
                best = math.min(best, d)
                if climbedAt < 0 and p.z >= z0 + 80 then climbedAt = now end
                local s = ragdoll.getState(player)
                local mode = s and s.controller and s.controller.mode or (s and 'unpowered' or 'none')
                modes[mode] = (modes[mode] or 0) + 1
                if math.floor(now) ~= last then
                    if last >= 0 and (p - lastPos):length() < 10 and d > 80 then stalled = stalled + 1 end
                    last = math.floor(now)
                    lastPos = p
                    print(string.format('SHIP02 powered=%s t=%d at=%.0f,%.0f,%.0f to_door=%.0f mode=%s', tostring(powered),
                        last, p.x, p.y, p.z, d, mode))
                end
            end)
            player:sendEvent('fixPlayerRouteStop')
            local seen = {}
            for m, n in pairs(modes) do seen[#seen + 1] = m .. '=' .. n end
            print(string.format('SHIP02_SUMMARY powered=%s route=%s climbed_at=%.1f highest=%.0f closest=%.0f stalled_s=%d modes=%s',
                tostring(powered), tostring(shipPlayerRoute and shipPlayerRoute.points), climbedAt, highest - z0, best,
                stalled, table.concat(seen, ',')))
        end
    end)
    player:sendEvent('fixPlayerRouteStop')
    player:sendEvent('fixCloseMenus', false)
    ragdoll.setPowered(player, false)
    if not ok then error(message, 0) end
end)

-- SWIM-01 (Alpha 1.0: "swimming falls back to vanilla safely"): the powered
-- player is put into the Odai river by Balmora (first spot found swimming),
-- swims forward 5 s, climbs nothing. Reports the ragdoll state while swimming.
registerTest('SWIM-01 a powered player swims', 918395, function()
    local player = placePlayer()
    local ok, message = pcall(function()
        player:sendEvent('fixPlayerConfigure', { sequence = 395, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 395 end)
        powerAndStand(player, 'swim')
        local found = false
        for _, x in ipairs({ -21800, -21600, -21400, -22000, -21200, -22200, -21000 }) do
            for _, y in ipairs({ -12200, -12600, -11800, -13000 }) do
                player:teleport('', util.vector3(x, y, -60))
                waitSeconds(1.0)
                if playerSwimming then found = true break end
            end
            if found then break end
        end
        testing.expect(found, 'no water found by Balmora')
        local modes = {}
        player:sendEvent('fixPlayerMove', 1)
        waitSeconds(5, function()
            local s = ragdoll.getState(player)
            local m = s and s.controller and s.controller.mode or 'none'
            modes[m] = (modes[m] or 0) + 1
        end)
        player:sendEvent('fixPlayerMove', 0)
        local list = {}
        for m, n in pairs(modes) do list[#list + 1] = m .. '=' .. n end
        local s = ragdoll.getState(player)
        print(string.format('SWIM-01 swimming=%s powered=%s modes %s pos=%s', tostring(playerSwimming),
            tostring(s and s.powered), table.concat(list, ' '), tostring(player.position)))
        -- Swim fallback: unpowered in water, powered again on dry ground.
        local wet = s and s.powered
        placePlayer()
        waitSeconds(3)
        local dry = ragdoll.getState(player)
        print(string.format('SWIM-01 back on land powered=%s', tostring(dry and dry.powered)))
        testing.expect(not wet, 'the body stayed powered in water')
        testing.expect(dry and dry.powered, 'the body was not powered again on land')
    end)
    player:sendEvent('fixPlayerMove', 0)
    ragdoll.setPowered(player, false)
    if not ok then error(message, 0) end
end)

-- RANGED-01 (Alpha 1.0: "bows ... kept vanilla-aimed"): the powered player
-- draws a long bow, holds and releases three shots at a dummy 400 units
-- ahead (first person, crosshair level). Passes when the dummy takes damage.
-- The thrown variant (Alpha 1.0 "thrown weapons") throws iron throwing knives.
local rangedCases = {
    { label = 'shoots a bow', weapon = 'long bow', ammo = 'iron arrow', spent = 'iron arrow' },
    { label = 'throws knives', weapon = 'iron throwing knife', spent = 'iron throwing knife' },
}
for caseIndex, rangedCase in ipairs(rangedCases) do
for index, powered in ipairs({ true, false }) do
registerTest('RANGED-01 a ' .. (powered and 'powered' or 'vanilla') .. ' player ' .. rangedCase.label,
    918393 + index * 10 + (caseIndex - 1) * 1000, function()
    local player = placePlayer()
    local victim
    local ok, message = pcall(function()
        local inventory = types.Actor.inventory(player)
        local spentItem = inventory:find(rangedCase.spent)
        if spentItem then spentItem:remove() end
        if not inventory:find(rangedCase.weapon) then world.createObject(rangedCase.weapon, rangedCase.ammo and 1 or 30):moveInto(inventory) end
        if rangedCase.ammo then world.createObject(rangedCase.ammo, 30):moveInto(inventory) end
        coroutine.yield()
        ragdoll.setPlayerAlwaysHits(true)
        player:sendEvent('fixPlayerConfigure', { weapon = rangedCase.weapon, ammo = rangedCase.ammo, marksman = true,
            sequence = 393, firstPerson = true })
        waitFor('player loadout', function() return playerReadySequence == 393 end)
        waitSeconds(1)
        if powered then powerAndStand(player, 'bow') end
        victim = spawnDummy('RANGED-01 target', util.vector3(0, 300, 0), false)
        victim:teleport(testArea.cell, player.position + util.vector3(0, 300, 0),
            { onGround = true, rotation = util.transform.rotateZ(math.pi) })
        waitSeconds(1)
        local health = types.Actor.stats.dynamic.health(victim).current
        ragdoll.prepareMouseInput()
        for shot = 1, 3 do
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitSeconds(1.2)
            if shot == 1 then ragdoll.captureScreenshot() end
            ragdoll.injectMouseInput(0, 0, false, 1)
            waitSeconds(0.1)
            if shot == 1 then ragdoll.captureScreenshot() end
            waitSeconds(1.4)
        end
        local after = types.Actor.stats.dynamic.health(victim).current
        local state = ragdoll.getState(player)
        local left = types.Actor.inventory(player):countOf(rangedCase.spent)
        print(string.format('RANGED-01 %s health %.1f -> %.1f powered=%s left=%d', rangedCase.label, health, after,
            tostring(state and state.powered), left))
        testing.expect(left < 30, 'nothing was fired')
        testing.expect(after < health, 'no shot hit the target')
    end)
    ragdoll.injectMouseInput(0, 0, false, 1)
    ragdoll.setPlayerAlwaysHits(false)
    ragdoll.setPowered(player, false)
    if victim and victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)
end
end

-- SHIELD-02 (Alpha 1.0 "shields"): the powered player carries an iron
-- shield and a longsword (shield body on the left forearm), stands 3 s and
-- walks forward 4 s. Passes when the body never falls.
registerTest('SHIELD-02 a powered player walks with a shield', 918396, function()
    local player = placePlayer()
    local ok, message = pcall(function()
        local inventory = types.Actor.inventory(player)
        if not inventory:find('iron_shield') then world.createObject('iron_shield', 1):moveInto(inventory) end
        if not inventory:find('iron longsword') then world.createObject('iron longsword', 1):moveInto(inventory) end
        coroutine.yield()
        player:sendEvent('fixPlayerConfigure', { weapon = 'iron longsword', shield = 'iron_shield', sequence = 396, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 396 end)
        powerAndStand(player, 'shield')
        local modes = {}
        local count = function()
            local s = ragdoll.getState(player)
            local m = s and s.controller and s.controller.mode or 'none'
            modes[m] = (modes[m] or 0) + 1
        end
        waitSeconds(3, count)
        player:sendEvent('fixPlayerMove', 1)
        waitSeconds(4, count)
        player:sendEvent('fixPlayerMove', 0)
        waitSeconds(1, count)
        local list = {}
        for m, n in pairs(modes) do list[#list + 1] = m .. '=' .. n end
        local left = types.Actor.getEquipment(player)[types.Actor.EQUIPMENT_SLOT.CarriedLeft]
        print(string.format('SHIELD-02 modes %s shield=%s', table.concat(list, ' '), tostring(left and left.recordId)))
        testing.expect(left ~= nil, 'the shield is not equipped')
        testing.expect(not modes.falling and not modes.fallen, 'the body fell')
    end)
    player:sendEvent('fixPlayerMove', 0)
    ragdoll.setPowered(player, false)
    if not ok then error(message, 0) end
end)

-- SPELL-01 (Alpha 1.0: "spells cast from a physical body with vanilla
-- effects"): the powered player casts a fireball at a dummy 300 units ahead.
for index, powered in ipairs({ true, false }) do
registerTest('SPELL-01 a ' .. (powered and 'powered' or 'vanilla') .. ' player casts a spell', 918394 + index * 10, function()
    local player = placePlayer()
    local victim
    local ok, message = pcall(function()
        player:sendEvent('fixPlayerConfigure', { spell = 'fireball', sequence = 394, firstPerson = true })
        waitFor('player loadout', function() return playerReadySequence == 394 end)
        waitSeconds(1)
        if powered then powerAndStand(player, 'spell') end
        victim = spawnDummy('SPELL-01 target', util.vector3(0, 300, 0), false)
        victim:teleport(testArea.cell, player.position + util.vector3(0, 300, 0),
            { onGround = true, rotation = util.transform.rotateZ(math.pi) })
        waitSeconds(1)
        local health = types.Actor.stats.dynamic.health(victim).current
        local magicka = types.Actor.stats.dynamic.magicka(player).current
        ragdoll.prepareMouseInput()
        for cast = 1, 3 do
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitSeconds(0.3)
            ragdoll.injectMouseInput(0, 0, false, 1)
            waitSeconds(2.5)
        end
        local after = types.Actor.stats.dynamic.health(victim).current
        local spent = magicka - types.Actor.stats.dynamic.magicka(player).current
        local state = ragdoll.getState(player)
        print(string.format('SPELL-01 health %.1f -> %.1f magicka spent %.1f powered=%s', health, after, spent,
            tostring(state and state.powered)))
        testing.expect(spent > 0, 'no spell was cast')
        -- The bolts reach the target powered or not (OPENMW_RAGDOLL_PROJECTILE_TRACE:
        -- 3 MAGIC_BOLT_HIT on the dummy either way); the test dummy takes no
        -- fire damage even from the vanilla player, so health is reported only.
    end)
    ragdoll.injectMouseInput(0, 0, false, 1)
    ragdoll.setPowered(player, false)
    if victim and victim:isValid() then victim:remove() end
    if not ok then error(message, 0) end
end)
end

-- STANCE-01 (Alpha 1.0 stances): a drag released high, low and in the middle
-- (the low guard needs OPENMW_RAGDOLL_TIP_STANCES=2; the default 1 keeps middle);
-- the commanded tip elevation the weapon rests at afterwards (degrees).
registerTest('STANCE-01 the guard remembers where the drag ended', 918392, function()
    local player = placePlayer()
    local ok, message = pcall(function()
        local weapon = 'iron longsword'
        if not types.Actor.inventory(player):find(weapon) then
            world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        ragdoll.setPlayerAlwaysHits(true)
        player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 392, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 392 end)
        waitSeconds(1)
        powerAndStand(player, weapon)
        ragdoll.prepareMouseInput()
        local perDeg = 1500 / 180
        for _, case in ipairs({ { 'high', 50 }, { 'low', -55 }, { 'middle', -15 } }) do
            waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            waitSeconds(0.6)
            local presses = ragdoll.getState(player).playerPressCount
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitFor('press registered', function() return ragdoll.getState(player).playerPressCount > presses end, 3)
            local sent = 0
            local step0 = tonumber(ragdoll.getState(player).physicsStep)
            waitFor('drag ' .. case[1], function()
                local n = tonumber(ragdoll.getState(player).physicsStep) - step0
                local y = math.floor(-case[2] * perDeg * math.min(1, n / 40))
                if y ~= sent then ragdoll.injectMouseInput(0, y - sent) end
                sent = y
                return n >= 50
            end, 6)
            ragdoll.injectMouseInput(0, 0, false, 1)
            waitSeconds(1.5)
            local t = ragdoll.getState(player).tip
            print(string.format('STANCE-01 %s rest_el=%.1f rest_az=%.1f', case[1], t and math.deg(t.el) or -999,
                t and math.deg(t.az) or -999))
        end
    end)
    ragdoll.injectMouseInput(0, 0, false, 1)
    ragdoll.setPlayerAlwaysHits(false)
    ragdoll.setPowered(player, false)
    if not ok then error(message, 0) end
end)

-- DEATH-01 (user bookmark 20, 2026-09-30: "when dying enemies do some weird
-- shuffle, they don't just die, they have some sort of momentum"): powered
-- dummies standing and walking are killed by one heavy blow; the engine logs
-- RAGDOLL_DEATH_SLIDE (how far each body slid in its first 1.5 s dead).
registerTest('DEATH-01 bodies die where they stand', 918391, function()
    local player = placePlayer()
    for index = 1, 6 do
        local walking = index > 3
        local victim = spawnDummy('DEATH-01 ' .. index, util.vector3(-150 + 60 * index, 260, 0), false)
        powerAndStand(victim, 'death target')
        waitSeconds(1)
        if walking then
            victim:sendEvent('fixActorMove', 1)
            waitSeconds(1.2)
        end
        local state = ragdoll.getState(victim)
        print(string.format('DEATH-01 victim=%d before powered=%s mode=%s', index, tostring(state and state.powered),
            tostring(state and state.controller and state.controller.mode)))
        local point = body(state, 'spine2').centerOfMass
        victim:sendEvent('Hit', { successful = true, sourceType = 'melee', hitPos = point,
            damage = { health = 100000, fatigue = 0 } })
        waitSeconds(2.5)
        print(string.format('DEATH-01 victim=%d walking=%s dead=%s', index, tostring(walking),
            tostring(types.Actor.isDead(victim))))
        victim:remove()
    end
end)

-- SPARKS-01 (user 2026-09-30: the sparks "look like smoke"): bursts of
-- several strengths in front of the camera, screenshots during the flight.
registerTest('SPARKS-01 spark bursts on camera', 918390, function()
    local player = placePlayer()
    player:sendEvent('fixPlayerConfigure', { sequence = 390, firstPerson = true })
    waitFor('player loadout', function() return playerReadySequence == 390 end)
    waitSeconds(1)
    local yaw = player.rotation:getYaw()
    local forward = util.vector3(math.sin(yaw), math.cos(yaw), 0)
    for _, strength in ipairs({ 0.2, 0.6, 1.0 }) do
        local point = player.position + forward * 60 + util.vector3(0, 0, 100)
        ragdoll.spawnSparks(point, util.vector3(-forward.x, -forward.y, 0.3), strength)
        waitSeconds(0.04)
        ragdoll.captureScreenshot()
        waitSeconds(0.08)
        ragdoll.captureScreenshot()
        waitSeconds(0.15)
        ragdoll.captureScreenshot()
        waitSeconds(0.6)
    end
end)

-- EDGE-01 (alpha 0.6, user bookmarks 10-13 2026-09-30): a claymore dragged
-- to the edge of its reach "starts bouncing" and does "weird flips"; the user
-- asked for the body to turn at the limit. Drags far past the right edge and
-- holds, then far past the left edge and holds. Reports per edge: the blade's
-- azimuth wobble over the hold (std and range, degrees, chest frame), the
-- tip-to-command gap, the player's turn (degrees) and the wrists outside their
-- limits (90th percentile).
for index, weapon in ipairs({ 'iron claymore', 'iron longsword' }) do
    registerTest('EDGE-01 drag past the reach edge with ' .. weapon, 918380 + index, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            ragdoll.setPlayerAlwaysHits(true)
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 380 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 380 + index end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local per = {}
            local function sample(segment)
                local s = ragdoll.getState(player)
                if not s or not s.weapon then return end
                local b = s.bodies[s.weapon.bodyIndex]
                local tip = b.position + (b.rotation * util.vector3(0, 0, 1)) * (s.weapon.length * 0.5)
                local chestBody = body(s, 'spine2')
                local d = tip - chestBody.position
                local yaw = player.rotation:getYaw()
                local az = math.deg(math.atan2(d.x, d.y) - yaw)
                az = (az + 180) % 360 - 180
                local list = per[segment] or {}
                per[segment] = list
                list[#list + 1] = { az = az, gap = s.tip and s.tip.gap or -1, yaw = math.deg(yaw),
                    wrist = math.max(s.wristOutsideRight or 0, s.wristOutsideLeft or 0) }
            end
            local perDeg = 1500 / 180
            local function run(segment, degX, steps)
                local step0 = tonumber(ragdoll.getState(player).physicsStep)
                local sentX = 0
                waitFor(segment, function()
                    local n = tonumber(ragdoll.getState(player).physicsStep) - step0
                    local x = math.floor(degX * perDeg * math.min(1, n / steps))
                    if x ~= sentX then ragdoll.injectMouseInput(x - sentX, 0) end
                    sentX = x
                    sample(segment)
                    return n >= steps
                end, steps / 30 + 6)
            end
            local function hold(segment, seconds)
                waitSeconds(seconds, function() sample(segment) end)
            end
            waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            waitSeconds(0.6)
            local presses = ragdoll.getState(player).playerPressCount
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitFor('press registered', function() return ragdoll.getState(player).playerPressCount > presses end, 3)
            local yaw0 = math.deg(player.rotation:getYaw())
            run('to-right', 200, 200)
            hold('hold-right', 1.5)
            run('to-left', -400, 400)
            hold('hold-left', 1.5)
            ragdoll.injectMouseInput(0, 0, false, 1)
            hold('release', 1.0)
            -- The user's flip came on a release at the right edge.
            waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            waitSeconds(0.6)
            presses = ragdoll.getState(player).playerPressCount
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitFor('press registered', function() return ragdoll.getState(player).playerPressCount > presses end, 3)
            run('to-right-2', 110, 110)
            hold('hold-right-2', 0.8)
            ragdoll.injectMouseInput(0, 0, false, 1)
            hold('release-right', 1.0)
            for _, seg in ipairs({ 'hold-right', 'hold-left', 'release', 'hold-right-2', 'release-right' }) do
                local list = per[seg] or {}
                local n, sum, sq, lo, hi, gap, yaw = #list, 0, 0, math.huge, -math.huge, 0, 0
                local wr = {}
                for _, v in ipairs(list) do
                    sum = sum + v.az
                    sq = sq + v.az * v.az
                    lo, hi = math.min(lo, v.az), math.max(hi, v.az)
                    gap = gap + v.gap
                    wr[#wr + 1] = v.wrist
                    yaw = v.yaw
                end
                table.sort(wr)
                local mean = sum / math.max(n, 1)
                print(string.format('EDGE-01 %s %s n=%d az_mean=%.1f az_std=%.1f az_range=%.1f az=%.0f..%.0f gap=%.1f turn=%.1f wrist_p90=%.1f',
                    weapon, seg, n, mean, math.sqrt(math.max(0, sq / math.max(n, 1) - mean * mean)), hi - lo, lo, hi,
                    gap / math.max(n, 1), ((yaw - yaw0 + 180) % 360) - 180, wr[math.max(1, math.ceil(#wr * 0.9))] or -1))
            end
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end)
end

local handlers = {}
for name, handler in pairs(testing.globalEventHandlers) do handlers[name] = handler end
handlers.fixActorReady = function(data) readyActors[data.id] = data end
handlers.fixActorRouteStart = function(data) roughRoute = data end
handlers.fixPlayerRouteStart = function(data) shipPlayerRoute = data end -- SHIP-02
handlers.fixActorRouteDone = function() roughDone = true end
handlers.fixPlayerReady = function(data) playerReadySequence = data.sequence end
handlers.fixKineticResult = function(data) kineticResult = data end
handlers.fixCameraReady = function(value) cameraReadyMode = value end
handlers.fixAimReady = function(value) contactAimReady = value end
handlers.fixSwimming = function(value) playerSwimming = value end
-- WRIST-01 (alpha 0.4 item 1, 2026-09-30): the claymore's wrists broke at
-- the edges of its reach (user). Held drags sweep the command slowly across
-- the whole arc at three heights, then fast committed strokes each way.
-- Every frame: how far each wrist sits outside its joint limits (degrees),
-- the command's angles, the chest twist and the blade's measured azimuth.
-- Reports per weapon: wrist-outside mean, 90th percentile, max, share of
-- frames over 20 degrees, and the blade's azimuth range (reach kept).
for index, weapon in ipairs({ 'iron dagger', 'iron longsword', 'iron claymore', 'iron warhammer',
    'iron spear', 'wooden staff', 'iron battle axe' }) do
    registerTest('WRIST-01 wrists hold at the edges with ' .. weapon, 918340 + index, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            -- Full control: no skill aim wander in the sweeps.
            ragdoll.setPlayerAlwaysHits(true)
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 340 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 340 + index end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local samples = {}
            local function sample(segment)
                local s = ragdoll.getState(player)
                if not s or not s.weapon then return end
                local b = s.bodies[s.weapon.bodyIndex]
                local tip = b.position + (b.rotation * util.vector3(0, 0, 1)) * (s.weapon.length * 0.5)
                local chest = body(s, 'spine2').position
                local d = tip - chest
                local bladeAz = math.deg(math.atan2(d.x, d.y))
                local bladeEl = math.deg(math.atan2(d.z, math.sqrt(d.x * d.x + d.y * d.y)))
                local r, l = s.wristOutsideRight or -1, s.wristOutsideLeft or -1
                local t = s.tip
                -- Chest pitch: the chest's forward lean over the pelvis (degrees,
                -- positive forward; the player faces +y).
                local pelvis = body(s, 'pelvis').position
                local lean = math.deg(math.atan2(chest.y - pelvis.y, chest.z - pelvis.z))
                -- Elbow side (bookmarks 16-18, "kimura"): the forearm's offset
                -- from the upper-arm -> hand line, its right (+x) share; below
                -- -0.5 the elbow points in across the chest.
                local sh, el, ha = body(s, 'right_upper_arm').position, body(s, 'right_forearm').position,
                    body(s, 'right_hand').position
                local line = (ha - sh):normalize()
                local off = (el - sh) - line * (el - sh):dot(line)
                local elbowRight = off:length() > 0.5 and off:normalize().x or 0
                samples[#samples + 1] = { r = r, l = l, az = bladeAz, seg = segment, lean = lean, elbow = elbowRight }
                if #samples % 2 == 0 then
                    print(string.format('WRIST-01S %s seg=%s az=%.1f el=%.1f twist=%.1f gap=%.1f out_r=%.1f out_l=%.1f blade_az=%.1f blade_el=%.1f',
                        weapon, segment, t and math.deg(t.az) or 0, t and math.deg(t.el) or 0,
                        t and math.deg(t.bodyTwist) or 0, t and t.gap or -1, r, l, bladeAz, bladeEl))
                end
            end
            -- 1500 counts sweep 180 degrees; injected mouse y is down.
            local perDeg = 1500 / 180
            local function run(segment, degX, degY, steps)
                local step0 = tonumber(ragdoll.getState(player).physicsStep)
                local sentX, sentY = 0, 0
                waitFor(segment, function()
                    local n = tonumber(ragdoll.getState(player).physicsStep) - step0
                    local f = math.min(1, n / steps)
                    local x, y = math.floor(degX * perDeg * f), math.floor(-degY * perDeg * f)
                    if x ~= sentX or y ~= sentY then ragdoll.injectMouseInput(x - sentX, y - sentY) end
                    sentX, sentY = x, y
                    sample(segment)
                    return n >= steps + 12
                end, steps / 30 + 6)
            end
            local function press()
                waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                waitSeconds(0.6)
                local presses = ragdoll.getState(player).playerPressCount
                ragdoll.injectMouseInput(0, 0, true, 1)
                waitFor('press registered', function() return ragdoll.getState(player).playerPressCount > presses end, 3)
            end
            local function release()
                ragdoll.injectMouseInput(0, 0, false, 1)
                waitSeconds(0.8, function() sample('release') end)
            end
            -- Slow sweeps (1 degree per physics step) at the guard, high, low.
            for _, height in ipairs({ { 'mid', 0 }, { 'high', 45 }, { 'low', -40 }, { 'top', 70 } }) do
                press()
                run(height[1] .. '-rise', 0, height[2], math.abs(height[2]) + 1)
                run(height[1] .. '-right', 150, 0, 150)
                run(height[1] .. '-left', -300, 0, 300)
                run(height[1] .. '-back', 150, 0, 150)
                release()
            end
            -- Fast committed strokes: right to left and back, 8 steps each.
            press()
            run('wind-right', 110, 0, 40)
            run('fast-left', -220, 0, 8)
            run('fast-right', 220, 0, 8)
            release()
            local all, over, sum = {}, 0, 0
            local minAz, maxAz = math.huge, -math.huge
            for _, v in ipairs(samples) do
                local worst = math.max(v.r, v.l)
                all[#all + 1] = worst
                sum = sum + math.max(0, worst)
                if worst > 20 then over = over + 1 end
                minAz, maxAz = math.min(minAz, v.az), math.max(maxAz, v.az)
            end
            local leans = { high = { 0, 0 }, low = { 0, 0 } }
            for _, v in ipairs(samples) do
                for _, h in ipairs({ 'high', 'low' }) do
                    if v.seg and v.seg:sub(1, #h + 1) == h .. '-' then
                        leans[h][1] = leans[h][1] + v.lean
                        leans[h][2] = leans[h][2] + 1
                    end
                end
            end
            local elbowIn, elbowSum, held = 0, 0, 0
            for _, v in ipairs(samples) do
                if v.seg ~= 'release' then
                    held = held + 1
                    elbowSum = elbowSum + v.elbow
                    if v.elbow < -0.5 then elbowIn = elbowIn + 1 end
                end
            end
            print(string.format('WRIST-01 %s elbow mean_right=%.2f inward=%.3f', weapon,
                elbowSum / math.max(1, held), elbowIn / math.max(1, held)))
            local topIn, top = 0, 0
            for _, v in ipairs(samples) do
                if v.seg and v.seg:sub(1, 4) == 'top-' then
                    top = top + 1
                    if v.elbow < -0.5 then topIn = topIn + 1 end
                end
            end
            print(string.format('WRIST-01 %s elbow top inward=%.3f n=%d', weapon, topIn / math.max(1, top), top))
            print(string.format('WRIST-01 %s pitch lean_high=%.1f lean_low=%.1f', weapon,
                leans.high[1] / math.max(1, leans.high[2]), leans.low[1] / math.max(1, leans.low[2])))
            table.sort(all)
            local p90 = all[math.max(1, math.ceil(#all * 0.9))]
            print(string.format('WRIST-01 %s summary frames=%d mean=%.1f p90=%.1f max=%.1f over20=%.3f blade_az=%.0f..%.0f',
                weapon, #all, sum / #all, p90, all[#all], over / #all, minAz, maxAz))
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end)
end

-- ELBOW-01 (user bookmarks 16-18, 2026-09-30: "the elbow always pointing
-- inside", "kimura'd"): the play pose - third person with the camera pitched
-- well down (0.7 rad, as in the recording), a held longsword drag raised to
-- the top and swept right, left and back. Reports the share of frames whose
-- elbow points in across the chest (forearm offset from the upper-arm ->
-- hand line, right share below -0.5), overall and with the tip behind the
-- chest, and the tip's mean forward offset.
for index, variant in ipairs({ { 0.15, false, 90 }, { 0.7, false, 90 }, { 0.7, true, 90 }, { 0.15, false, 150 } }) do
    local pitch, walking, rise = variant[1], variant[2], variant[3]
    registerTest('ELBOW-01 held longsword drags, camera pitch ' .. pitch .. (walking and ' walking' or '')
        .. (rise > 90 and ' over the head' or ''), 918470 + index, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            local weapon = 'iron longsword'
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            ragdoll.setPlayerAlwaysHits(true)
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 470 + index, firstPerson = false, pitch = pitch })
            waitFor('player loadout', function() return playerReadySequence == 470 + index end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local n, inward, behind, behindIn, forwardSum = 0, 0, 0, 0, 0
            local function sample()
                local s = ragdoll.getState(player)
                if not s or not s.weapon then return end
                local b = s.bodies[s.weapon.bodyIndex]
                local tip = b.position + (b.rotation * util.vector3(0, 0, 1)) * (s.weapon.length * 0.5)
                local chest = body(s, 'spine2').position
                local sh, el, ha = body(s, 'right_upper_arm').position, body(s, 'right_forearm').position,
                    body(s, 'right_hand').position
                local line = (ha - sh):normalize()
                local off = (el - sh) - line * (el - sh):dot(line)
                local right = off:length() > 0.5 and off:normalize().x or 0
                n = n + 1
                forwardSum = forwardSum + (tip.y - chest.y)
                if right < -0.5 then inward = inward + 1 end
                if tip.y < chest.y then
                    behind = behind + 1
                    if right < -0.5 then behindIn = behindIn + 1 end
                end
            end
            local perDeg = 1500 / 180
            local function run(degX, degY, steps)
                local step0 = tonumber(ragdoll.getState(player).physicsStep)
                local sentX, sentY = 0, 0
                waitFor('elbow sweep', function()
                    local k = tonumber(ragdoll.getState(player).physicsStep) - step0
                    local f = math.min(1, k / steps)
                    local x, y = math.floor(degX * perDeg * f), math.floor(-degY * perDeg * f)
                    if x ~= sentX or y ~= sentY then ragdoll.injectMouseInput(x - sentX, y - sentY) end
                    sentX, sentY = x, y
                    sample()
                    return k >= steps + 12
                end, steps / 30 + 6)
            end
            waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            waitSeconds(0.6)
            ragdoll.injectMouseInput(0, 0, true, 1)
            if walking then player:sendEvent('fixPlayerMove', 1) end
            run(0, rise, rise + 1)
            run(150, 0, 150)
            run(-300, 0, 300)
            run(150, 0, 150)
            ragdoll.injectMouseInput(0, 0, false, 1)
            player:sendEvent('fixPlayerMove', 0)
            waitSeconds(0.5)
            -- The guard left where a drag ends (play: the elbow points in on
            -- 53% of guard frames with the tip back over the right shoulder).
            local guardN, guardIn = 0, 0
            for _, spot in ipairs({ { 110, 60 }, { 150, 40 }, { -60, 60 } }) do
                waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                waitSeconds(0.4)
                ragdoll.injectMouseInput(0, 0, true, 1)
                run(spot[1], spot[2], 60)
                ragdoll.injectMouseInput(0, 0, false, 1)
                local before, beforeIn = n, inward
                waitSeconds(2, sample)
                guardN, guardIn = guardN + (n - before), guardIn + (inward - beforeIn)
                local ts = ragdoll.getState(player).tip
                print(string.format('ELBOW-01 guard spot az=%d el=%d inward=%.3f tip_az=%.0f tip_el=%.0f', spot[1], spot[2],
                    (inward - beforeIn) / math.max(1, n - before), ts and math.deg(ts.az) or 0, ts and math.deg(ts.el) or 0))
                ragdoll.injectMouseInput(0, 0, true, 1)
                run(-spot[1], -spot[2], 60)
                ragdoll.injectMouseInput(0, 0, false, 1)
                waitSeconds(0.5)
            end
            print(string.format('ELBOW-01 guard n=%d inward=%.3f', guardN, guardIn / math.max(1, guardN)))
            print(string.format('ELBOW-01 rise=%d walking=%s pitch=%.2f n=%d inward=%.3f behind=%d behind_inward=%.3f tip_forward=%.1f',
                rise, tostring(walking), pitch, n, inward / math.max(1, n), behind, behindIn / math.max(1, behind), forwardSum / math.max(1, n)))
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        player:sendEvent('fixPlayerMove', 0)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end)
end

-- SHEATHE-01 (play 2026-10-01 "wtf happened to the left arm here?"): after
-- drags with a two-hander and a sheathe, the left hand stayed 50-65 u off its
-- animation target at shoulder height while standing (claymore p50 64, staff
-- 47). Drags (or none), sheathe, stand 3 s; prints each hand's distance to its
-- target and height under its shoulder.
for index, case in ipairs({ { 'iron claymore', true }, { 'iron claymore', false }, { 'wooden staff', true },
    { 'iron longsword', true }, { 'iron claymore', true, 10 }, { 'iron claymore', true, nil, true }, { 'wooden staff', true, nil, true } }) do
    local weapon, drags, tired, walk = case[1], case[2], case[3], case[4]
    registerTest('SHEATHE-01 hands rest after sheathing ' .. weapon .. (drags and ' after drags' or '')
        .. (tired and (' tired ' .. tired) or '') .. (walk and ' then walking' or ''),
        918480 + index, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 480 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 480 + index end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local perDeg = 1500 / 180
            local function run(degX, degY, steps)
                local step0 = tonumber(ragdoll.getState(player).physicsStep)
                local sentX, sentY = 0, 0
                waitFor('sheathe sweep', function()
                    local k = tonumber(ragdoll.getState(player).physicsStep) - step0
                    local f = math.min(1, k / steps)
                    local x, y = math.floor(degX * perDeg * f), math.floor(-degY * perDeg * f)
                    if x ~= sentX or y ~= sentY then ragdoll.injectMouseInput(x - sentX, y - sentY) end
                    sentX, sentY = x, y
                    return k >= steps + 12
                end, steps / 30 + 6)
            end
            waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            waitSeconds(0.6)
            if drags then
                for _, spot in ipairs({ { 120, 50 }, { -150, 20 }, { 60, -40 } }) do
                    ragdoll.injectMouseInput(0, 0, true, 1)
                    run(spot[1], spot[2], 40)
                    ragdoll.injectMouseInput(0, 0, false, 1)
                    waitSeconds(0.3)
                end
            end
            if tired then player:sendEvent('fixSetFatigue', tired / 200); waitSeconds(0.1) end
            local start = core.getSimulationTime()
            repeat
                player:sendEvent('fixPlayerStance', false)
                waitSeconds(0.05)
            until types.Actor.getStance(player) == types.Actor.STANCE.Nothing or core.getSimulationTime() - start > 2
            waitSeconds(1.5)
            if walk then
                player:sendEvent('fixPlayerMove', 1)
                waitSeconds(2.5)
                player:sendEvent('fixPlayerMove', 0)
                waitSeconds(1)
            end
            local n, sums = 0, { left_hand = { 0, 0, 0 }, right_hand = { 0, 0, 0 } }
            waitSeconds(1.5, function()
                local s = ragdoll.getState(player)
                if not s then return end
                n = n + 1
                for side, acc in pairs(sums) do
                    local hand = body(s, side)
                    local shoulder = body(s, side == 'left_hand' and 'left_upper_arm' or 'right_upper_arm')
                    local err = (hand.position - hand.targetPosition):length()
                    acc[1] = acc[1] + err
                    acc[2] = math.max(acc[2], err)
                    acc[3] = acc[3] + (hand.position.z - shoulder.position.z)
                end
            end)
            for side, acc in pairs(sums) do
                print(string.format('SHEATHE-01 weapon=%s drags=%s tired=%s %s err_mean=%.1f err_max=%.1f below_shoulder=%.1f',
                    weapon, tostring(drags), tostring(tired), side, acc[1] / math.max(1, n), acc[2], acc[3] / math.max(1, n)))
            end
            testing.expect(sums.left_hand[1] / math.max(1, n) < 25, 'left hand off its rest after sheathing')
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        player:sendEvent('fixPlayerMove', 0)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end)
end

-- FIST-01 (play 2026-10-01 "punches are still not working"): a quick click
-- with bare fists plays the vanilla punch clip; the clip throws the left fist
-- (reach 16 -> 35 u) while the physical left hand stayed in the guard (22 u).
-- Four clicks; per click the largest physical and target reach of each hand.
registerTest('FIST-01 quick clicks extend the punching fist', 918490, function()
    local player = placePlayer()
    local ok, message = pcall(function()
        player:sendEvent('fixPlayerConfigure', { sequence = 490, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 490 end)
        waitSeconds(1)
        powerAndStand(player, 'fists')
        ragdoll.prepareMouseInput()
        waitSeconds(0.8)
        local extended = 0
        local spins, headSpins, flips, lastSpin = {}, {}, 0, nil
        for click = 1, 4 do
            local best = { left = { 0, 0 }, right = { 0, 0 } }
            local hands = {}
            ragdoll.injectMouseInput(0, 0, true)
            waitSeconds(0.05)
            ragdoll.injectMouseInput(0, 0, false)
            waitSeconds(1.0, function()
                local st = ragdoll.getState(player)
                if not st then return end
                if st.strikeHand then hands[st.strikeHand] = true end
                -- Shake: torso and head spin, and flips of the torso's
                -- angular velocity between steps (play 2026-10-01 build 14
                -- "it shakes like crazy").
                local torso, head = body(st, 'spine2'), body(st, 'head')
                spins[#spins + 1] = torso.angularVelocity:length()
                headSpins[#headSpins + 1] = head.angularVelocity:length()
                if lastSpin and lastSpin:length() > 0.5 and torso.angularVelocity:length() > 0.5
                    and lastSpin:dot(torso.angularVelocity) < -0.3 * lastSpin:length() * torso.angularVelocity:length() then
                    flips = flips + 1
                end
                lastSpin = torso.angularVelocity
                for _, side in ipairs({ 'left', 'right' }) do
                    local h, sh = body(st, side .. '_hand'), body(st, side .. '_upper_arm')
                    local fa = body(st, side .. '_forearm')
                    local arm = (fa.position - sh.position):length() + (h.position - fa.position):length()
                    best[side][1] = math.max(best[side][1], (h.position - sh.position):length())
                    -- A target past the straight arm counts as the straight arm.
                    best[side][2] = math.max(best[side][2], math.min((h.targetPosition - sh.position):length(), arm))
                end
            end)
            local side = best.left[2] > best.right[2] and 'left' or 'right'
            if best[side][1] >= 0.85 * best[side][2] then extended = extended + 1 end
            print(string.format('FIST-01 click=%d punch=%s left phys=%.1f target=%.1f right phys=%.1f target=%.1f strike=%s',
                click, side, best.left[1], best.left[2], best.right[1], best.right[2],
                (hands.left and 'L' or '') .. (hands.right and 'R' or '')))
            waitSeconds(0.4)
        end
        table.sort(spins); table.sort(headSpins)
        print(string.format('FIST-01 extended=%d/4 torso_spin p50=%.2f p90=%.2f head_spin p50=%.2f p90=%.2f torso_flips=%d n=%d',
            extended, spins[math.floor(#spins / 2) + 1] or 0, spins[math.floor(#spins * 0.9) + 1] or 0,
            headSpins[math.floor(#headSpins / 2) + 1] or 0, headSpins[math.floor(#headSpins * 0.9) + 1] or 0, flips, #spins))
        testing.expect(extended >= 3, 'the punching fist did not follow the clip')
    end)
    ragdoll.injectMouseInput(0, 0, false)
    ragdoll.setPowered(player, false)
    if not ok then error(message, 0) end
end)

-- DRAG-01 (user bookmarks 2026-10-01): from the raised guard a side drag
-- turned into a chop, and a mouse circle over the top made the arm flip.
-- Part 1: from the guard, drag right then left (60 degrees of mouse each);
-- reports how far the tip went sideways vs down. Part 2: two mouse circles
-- (radius 40 degrees); reports the arm plane's largest turn per 0.1 s and
-- how many 0.1 s windows turned it more than 30 degrees.
for index, case in ipairs({ { 'iron claymore' }, { 'iron warhammer' }, { 'iron longsword' },
    { 'iron claymore', true }, { 'iron longsword', true } }) do
    local weapon, high = case[1], case[2]
    registerTest('DRAG-01 side drags and circles from the ' .. (high and 'high ' or '') .. 'guard with ' .. weapon,
        918500 + index, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            ragdoll.setPlayerAlwaysHits(true)
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 500 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 500 + index end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local perDeg = 1500 / 180
            local function tipLocal()
                local s = ragdoll.getState(player)
                local b = s.bodies[s.weapon.bodyIndex]
                local tip = b.position + (b.rotation * util.vector3(0, 0, 1)) * (s.weapon.length * 0.5)
                local chest = body(s, 'spine2').position
                local yaw = player.rotation:getYaw()
                local d = tip - chest
                local fwd = util.vector3(math.sin(yaw), math.cos(yaw), 0)
                local right = util.vector3(math.cos(yaw), -math.sin(yaw), 0)
                return util.vector3(d:dot(right), d:dot(fwd), d.z)
            end
            local planes, lastPlane, worst, flips = {}, nil, 0, 0
            local function armPlane()
                local s = ragdoll.getState(player)
                local sh, el, ha = body(s, 'right_upper_arm').position, body(s, 'right_forearm').position,
                    body(s, 'right_hand').position
                local n = (el - sh):cross(ha - el)
                return n:length() > 1 and n:normalize() or nil
            end
            local function run(points, steps)
                local step0 = tonumber(ragdoll.getState(player).physicsStep)
                local sentX, sentY, k0 = 0, 0, 0
                waitFor('drag path', function()
                    local k = tonumber(ragdoll.getState(player).physicsStep) - step0
                    local f = math.min(1, k / steps)
                    local x, y = points(f)
                    x, y = math.floor(x * perDeg), math.floor(-y * perDeg)
                    if x ~= sentX or y ~= sentY then ragdoll.injectMouseInput(x - sentX, y - sentY) end
                    sentX, sentY = x, y
                    if k - k0 >= 6 then
                        k0 = k
                        local p = armPlane()
                        if p and lastPlane then
                            local turn = math.deg(math.acos(math.max(-1, math.min(1, p:dot(lastPlane)))))
                            worst = math.max(worst, turn)
                            if turn > 30 then flips = flips + 1 end
                        end
                        lastPlane = p
                    end
                    return k >= steps + 12
                end, steps / 30 + 6)
            end
            waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            waitSeconds(0.8)
            if high then
                -- The high guard (stance): released with the tip high.
                ragdoll.injectMouseInput(0, 0, true, 1)
                run(function(f) return 0, 45 * f end, 45)
                ragdoll.injectMouseInput(0, 0, false, 1)
                waitSeconds(0.8)
            end
            local start = tipLocal()
            ragdoll.injectMouseInput(0, 0, true, 1)
            run(function(f) return 60 * f, 0 end, 60)
            local right = tipLocal()
            run(function(f) return -120 * f, 0 end, 120)
            local left = tipLocal()
            ragdoll.injectMouseInput(0, 0, false, 1)
            print(string.format('DRAG-01 weapon=%s high=%s side start=(%.0f,%.0f,%.0f) right=(%.0f,%.0f,%.0f) left=(%.0f,%.0f,%.0f) sideways=%.0f down=%.0f',
                weapon, tostring(high), start.x, start.y, start.z, right.x, right.y, right.z, left.x, left.y, left.z,
                right.x - left.x, start.z - math.min(right.z, left.z)))
            waitSeconds(0.6)
            waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            waitSeconds(0.5)
            worst, flips, lastPlane = 0, 0, nil
            ragdoll.injectMouseInput(0, 0, true, 1)
            run(function(f)
                local a = f * 4 * math.pi
                return 40 * math.sin(a), 40 * (1 - math.cos(a))
            end, 240)
            ragdoll.injectMouseInput(0, 0, false, 1)
            print(string.format('DRAG-01 weapon=%s high=%s circle worst_turn=%.0f flips=%d', weapon, tostring(high), worst, flips))
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end)
end

-- SPIN-01 (user bookmarks 2026-10-01 "claymore spins a full 360"): walking
-- back, a drag released and then a fast mouse turn (60 degrees in 0.1 s,
-- the facing ran 560 deg/s in play). Reports the blade's largest world turn
-- per step and the steps over 15 degrees (a whip).
for index, weapon in ipairs({ 'iron claymore', 'iron longsword' }) do
    registerTest('SPIN-01 fast turn after a drag with ' .. weapon, 918510 + index, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 510 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 510 + index end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local function bladeDir()
                local st = ragdoll.getState(player)
                local b = st.bodies[st.weapon.bodyIndex]
                return (b.rotation * util.vector3(0, 0, 1)):normalize()
            end
            local worst, whips = 0, 0
            for round = 1, 3 do
                waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                player:sendEvent('fixPlayerMove', -1)
                waitSeconds(0.5)
                ragdoll.injectMouseInput(0, 0, true, 1)
                for i = 1, 20 do ragdoll.injectMouseInput(15, -5); coroutine.yield() end
                ragdoll.injectMouseInput(0, 0, false, 1)
                coroutine.yield()
                local last = bladeDir()
                local step0 = tonumber(ragdoll.getState(player).physicsStep)
                local sent = 0
                waitFor('turn', function()
                    local k = tonumber(ragdoll.getState(player).physicsStep) - step0
                    local want = math.floor(math.min(1, k / 6) * 60 * 1500 / 180) * (round % 2 == 0 and -1 or 1)
                    if want ~= sent then ragdoll.injectMouseInput(want - sent, 0); sent = want end
                    local d = bladeDir()
                    local turn = math.deg(math.acos(math.max(-1, math.min(1, d:dot(last)))))
                    last = d
                    worst = math.max(worst, turn)
                    if turn > 15 then whips = whips + 1 end
                    return k >= 60
                end, 4)
                player:sendEvent('fixPlayerMove', 0)
                waitSeconds(0.8)
            end
            print(string.format('SPIN-01 weapon=%s worst_turn=%.0f whips=%d', weapon, worst, whips))
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        player:sendEvent('fixPlayerMove', 0)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end)
end

-- EDGE-02 (user bookmarks 17-18, 2026-10-01: "it starts spazzing at the
-- right limit"): drag far right (mouse 110 degrees) and hold at three
-- heights; after 0.5 s settling, 1 s of the blade's per-step turn (jitter)
-- and the tip's gap to its goal.
for index, weapon in ipairs({ 'iron claymore', 'iron longsword' }) do
    registerTest('EDGE-02 holding at the right limit with ' .. weapon, 918520 + index, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 520 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 520 + index end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local perDeg = 1500 / 180
            local function bladeDir()
                local st = ragdoll.getState(player)
                local b = st.bodies[st.weapon.bodyIndex]
                return (b.rotation * util.vector3(0, 0, 1)):normalize()
            end
            local function move(dx, dy, steps)
                local sx, sy = 0, 0
                for i = 1, steps do
                    local x = math.floor(dx * perDeg * i / steps); local y = math.floor(-dy * perDeg * i / steps)
                    ragdoll.injectMouseInput(x - sx, y - sy); sx, sy = x, y
                    coroutine.yield()
                end
            end
            for _, spot in ipairs({ { 0, 10 }, { 110, 30 }, { 110, 0 }, { 110, -30 } }) do
                local side, height = spot[1], spot[2]
                waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                waitSeconds(0.5)
                ragdoll.injectMouseInput(0, 0, true, 1)
                move(side, height - 20, 40)
                waitSeconds(0.5)
                local first = bladeDir()
                local last, turns, gaps, n = first, 0, 0, 0
                waitSeconds(1.0, function()
                    local d = bladeDir()
                    turns = turns + math.deg(math.acos(math.max(-1, math.min(1, d:dot(last)))))
                    last = d
                    local st = ragdoll.getState(player)
                    gaps = gaps + (st.tip and st.tip.gap or 0)
                    n = n + 1
                end)
                local net = math.deg(math.acos(math.max(-1, math.min(1, first:dot(last)))))
                print(string.format('EDGE-02 weapon=%s side=%d height=%d shake=%.2f gap=%.0f', weapon, side, height,
                    math.max(0, turns - net) / math.max(1, n), gaps / math.max(1, n)))
                ragdoll.injectMouseInput(0, 0, false, 1)
                waitSeconds(1.0)
            end
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end)
end

-- PUSH-01 (alpha 0.8 force): a standing powered dummy's head or chest takes
-- a sideways impulse of 1000 or 3000; how far the struck part moves
-- against the pelvis in 0.6 s, and whether the body falls.
registerTest('PUSH-01 standing dummy takes raw impulses', 918530, function()
    placePlayer()
    local dummy = spawnDummy('push probe', util.vector3(0, 200, 0), false)
    local ok, message = pcall(function()
        powerAndStand(dummy, 'push probe')
        waitSeconds(1)
        for _, part in ipairs({ 'head', 'spine2' }) do
            for _, size in ipairs({ 1000, 3000, 30000 }) do
                waitFor('standing', function()
                    local s = ragdoll.getState(dummy)
                    return s and s.controller and s.controller.mode == 'standing'
                end, 8)
                waitSeconds(1)
                local s = ragdoll.getState(dummy)
                local index
                for i, b in ipairs(s.bodies) do if b.name == part then index = i end end
                local before = body(s, part).position - body(s, 'pelvis').position
                ragdoll.applyImpulse(dummy, index, util.vector3(size, 0, 0))
                local move, fell = 0, false
                waitSeconds(0.6, function()
                    local st = ragdoll.getState(dummy)
                    if not st then return end
                    move = math.max(move, ((body(st, part).position - body(st, 'pelvis').position) - before):length())
                    if st.controller and st.controller.mode ~= 'standing' and st.controller.mode ~= 'bracing' then fell = true end
                end)
                print(string.format('PUSH-01 part=%s mass=%.2f impulse=%d move=%.1f fell=%s', part, s.bodies[index].mass,
                    size, move, tostring(fell)))
            end
        end
    end)
    ragdoll.setPowered(dummy, false)
    if not ok then error(message, 0) end
end)

-- SLIP-01 (user bookmark 2026-10-01, alpha 0.8 momentum): a hard drag right
-- to left that stops under the held button. The blade should run on past
-- where the mouse stopped and come back - more and slower at low skill.
-- Reports the blade's overshoot (degrees past its final resting azimuth) and
-- the time until it is back within 10 degrees.
for index, case in ipairs({ { 'iron warhammer', 10 }, { 'iron warhammer', 90 }, { 'iron claymore', 10 },
    { 'iron claymore', 90 } }) do
    local weapon, skill = case[1], case[2]
    registerTest('SLIP-01 hard stroke stops with ' .. weapon .. ' skill ' .. skill, 918540 + index, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 540 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 540 + index end)
            player:sendEvent('fixSetWeaponSkill', skill)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local function bladeAz()
                local st = ragdoll.getState(player)
                local b = st.bodies[st.weapon.bodyIndex]
                local d = b.rotation * util.vector3(0, 0, 1)
                local yaw = player.rotation:getYaw()
                local fwd = util.vector3(math.sin(yaw), math.cos(yaw), 0)
                local right = util.vector3(math.cos(yaw), -math.sin(yaw), 0)
                return math.deg(math.atan2(d:dot(right), d:dot(fwd)))
            end
            local overs, backs = {}, {}
            for round = 1, 2 do
                waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                waitSeconds(0.6)
                ragdoll.injectMouseInput(0, 0, true, 1)
                -- wind up right slowly, then a hard stroke left (user speed)
                local sx = 0
                for i = 1, 20 do local x = math.floor(600 * i / 20); ragdoll.injectMouseInput(x - sx, 0); sx = x; coroutine.yield() end
                waitSeconds(0.2)
                sx = 0
                for i = 1, 6 do local x = math.floor(-1600 * i / 6); ragdoll.injectMouseInput(x - sx, 0); sx = x; coroutine.yield() end
                local samples = {}
                local t0 = core.getSimulationTime()
                waitSeconds(1.6, function() samples[#samples + 1] = { core.getSimulationTime() - t0, bladeAz() } end)
                ragdoll.injectMouseInput(0, 0, false, 1)
                local final = samples[#samples][2]
                local over, back = 0, 0
                for _, s in ipairs(samples) do
                    over = math.max(over, final - s[2]) -- stroke went left (negative az)
                end
                for k = #samples, 1, -1 do
                    if math.abs(samples[k][2] - final) > 10 then back = samples[k][1]; break end
                end
                overs[#overs + 1], backs[#backs + 1] = over, back
                waitSeconds(0.8)
            end
            print(string.format('SLIP-01 weapon=%s skill=%d overshoot=%.0f,%.0f back_s=%.2f,%.2f', weapon, skill,
                overs[1], overs[2], backs[1], backs[2]))
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end)
end

-- SWING-02 (alpha 0.4 item 2, 2026-09-30): the turn is spread over hips,
-- spine and chest, and big swings shift the weight and step in. A held
-- slow drag parks the blade far right, then far left (the turn split:
-- pelvis and chest yaw from their rest); then 6 committed strokes each way.
-- Reports: pelvis and chest yaw at the parks, pelvis travel forward during
-- the strokes, the feet's largest move and moves over 8 units (steps), the
-- blade's peak speed per stroke, and whether the body left standing.
for index, weapon in ipairs({ 'iron longsword', 'iron claymore', 'iron warhammer' }) do
    registerTest('SWING-02 footwork with ' .. weapon, 918360 + index, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            ragdoll.setPlayerAlwaysHits(true)
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 360 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 360 + index end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            -- Yaw of a left-to-right body line (hips: thighs, chest: upper arms).
            local lines = { pelvis = { 'left_thigh', 'right_thigh' }, spine2 = { 'left_upper_arm', 'right_upper_arm' } }
            local function yawOf(s, name)
                local v = body(s, lines[name][2]).position - body(s, lines[name][1]).position
                return math.deg(math.atan2(v.x, v.y))
            end
            local function wrap(a) while a > 180 do a = a - 360 end while a < -180 do a = a + 360 end return a end
            local s0 = ragdoll.getState(player)
            local pelvis0, chest0 = yawOf(s0, 'pelvis'), yawOf(s0, 'spine2')
            local feet0 = { body(s0, 'left_foot').position, body(s0, 'right_foot').position }
            local pelvisPos0 = body(s0, 'pelvis').position
            local fell, footMax, steps, forwardMax = false, 0, 0, -math.huge
            local modes = {}
            local lastFeet = { feet0[1], feet0[2] }
            local peak, lastTip, lastT = 0, nil, nil
            local function sample()
                local s = ragdoll.getState(player)
                if not s then return end
                if s.controller then
                    modes[s.controller.mode] = true
                    if s.controller.mode == 'falling' or s.controller.mode == 'fallen' then fell = true end
                end
                for k, name in ipairs({ 'left_foot', 'right_foot' }) do
                    local p = body(s, name).position
                    footMax = math.max(footMax, (util.vector3(p.x, p.y, 0) - util.vector3(feet0[k].x, feet0[k].y, 0)):length())
                    if (util.vector3(p.x, p.y, 0) - util.vector3(lastFeet[k].x, lastFeet[k].y, 0)):length() > 8 then
                        steps = steps + 1
                        lastFeet[k] = p
                    end
                end
                forwardMax = math.max(forwardMax, body(s, 'pelvis').position.y - pelvisPos0.y)
                local b = s.bodies[s.weapon.bodyIndex]
                local tip = b.position + (b.rotation * util.vector3(0, 0, 1)) * (s.weapon.length * 0.5)
                local now = core.getSimulationTime()
                if lastTip and now > lastT then peak = math.max(peak, (tip - lastTip):length() / (now - lastT)) end
                lastTip, lastT = tip, now
            end
            local perDeg = 1500 / 180
            local function run(degX, steps_)
                local step0 = tonumber(ragdoll.getState(player).physicsStep)
                local sentX = 0
                waitFor('drag', function()
                    local n = tonumber(ragdoll.getState(player).physicsStep) - step0
                    local x = math.floor(degX * perDeg * math.min(1, n / steps_))
                    if x ~= sentX then ragdoll.injectMouseInput(x - sentX, 0) end
                    sentX = x
                    sample()
                    return n >= steps_ + 12
                end, steps_ / 30 + 6)
            end
            local function press()
                waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                waitSeconds(0.5)
                local presses = ragdoll.getState(player).playerPressCount
                ragdoll.injectMouseInput(0, 0, true, 1)
                waitFor('press registered', function() return ragdoll.getState(player).playerPressCount > presses end, 3)
            end
            press()
            local parks = {}
            for _, park in ipairs({ { 'right', 90 }, { 'left', -170 } }) do
                run(park[2], math.abs(park[2]))
                waitSeconds(0.6, sample)
                local s = ragdoll.getState(player)
                parks[#parks + 1] = string.format('%s pelvis=%.0f chest=%.0f', park[1],
                    wrap(yawOf(s, 'pelvis') - pelvis0), wrap(yawOf(s, 'spine2') - chest0))
            end
            run(80, 80)
            ragdoll.injectMouseInput(0, 0, false, 1)
            waitSeconds(0.8, sample)
            local peaks = {}
            for stroke = 1, 6 do
                press()
                run(stroke % 2 == 1 and 110 or -110, 30)
                peak = 0
                run(stroke % 2 == 1 and -220 or 220, 8)
                peaks[#peaks + 1] = peak
                ragdoll.injectMouseInput(0, 0, false, 1)
                waitSeconds(0.8, sample)
            end
            table.sort(peaks)
            local seen = {}
            for mode in pairs(modes) do seen[#seen + 1] = mode end
            table.sort(seen)
            print(string.format('SWING-02 %s summary %s | %s | forward=%.1f foot_max=%.1f steps=%d peak_p50=%.0f fell=%s modes=%s',
                weapon, parks[1], parks[2], forwardMax, footMax, steps, peaks[math.ceil(#peaks / 2)], tostring(fell),
                table.concat(seen, ',')))
            testing.expect(not fell, weapon .. ' fell during the swings')
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end)
end

-- WOUND-01 (alpha 0.4 item 7): wounded arms swing weaker and slower,
-- wounded legs limp. Arms: 6 committed strokes whole, then with 40 damage
-- on the weapon forearm (both forearms for the claymore): the blade's peak
-- speed per stroke (median). Legs: a dummy walks 4 s whole, then with 40
-- damage on its left leg: hip tilt (right thigh height - left, mean; a
-- limp drops the weak side) and walking speed.
for index, weapon in ipairs({ 'iron longsword', 'iron claymore' }) do
    registerTest('WOUND-01 wounded arm swings with ' .. weapon, 918370 + index, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            ragdoll.setPlayerAlwaysHits(true)
            ragdoll.clearLimbDamage()
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 370 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 370 + index end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local peak, lastTip, lastT = 0, nil, nil
            local function sample()
                local s = ragdoll.getState(player)
                if not s or not s.weapon then return end
                local b = s.bodies[s.weapon.bodyIndex]
                local tip = b.position + (b.rotation * util.vector3(0, 0, 1)) * (s.weapon.length * 0.5)
                local now = core.getSimulationTime()
                if lastTip and now > lastT then peak = math.max(peak, (tip - lastTip):length() / (now - lastT)) end
                lastTip, lastT = tip, now
            end
            local perDeg = 1500 / 180
            local function run(degX, steps_)
                local step0 = tonumber(ragdoll.getState(player).physicsStep)
                local sentX = 0
                waitFor('drag', function()
                    local n = tonumber(ragdoll.getState(player).physicsStep) - step0
                    local x = math.floor(degX * perDeg * math.min(1, n / steps_))
                    if x ~= sentX then ragdoll.injectMouseInput(x - sentX, 0) end
                    sentX = x
                    sample()
                    return n >= steps_ + 12
                end, steps_ / 30 + 6)
            end
            local function strokes()
                local peaks = {}
                for stroke = 1, 6 do
                    waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                    waitSeconds(0.5)
                    local presses = ragdoll.getState(player).playerPressCount
                    ragdoll.injectMouseInput(0, 0, true, 1)
                    waitFor('press registered', function() return ragdoll.getState(player).playerPressCount > presses end, 3)
                    run(stroke % 2 == 1 and 110 or -110, 30)
                    peak = 0
                    run(stroke % 2 == 1 and -220 or 220, 8)
                    peaks[#peaks + 1] = peak
                    ragdoll.injectMouseInput(0, 0, false, 1)
                    waitSeconds(0.8, sample)
                end
                table.sort(peaks)
                return peaks[math.ceil(#peaks / 2)]
            end
            local whole = strokes()
            ragdoll.addLimbDamage(player, 'right_forearm', 40)
            if weapon == 'iron claymore' then ragdoll.addLimbDamage(player, 'left_forearm', 40) end
            local hurt = strokes()
            print(string.format('WOUND-01 %s arm summary whole_peak=%.0f wounded_peak=%.0f ratio=%.2f', weapon, whole, hurt, hurt / whole))
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.clearLimbDamage()
        ragdoll.setPlayerAlwaysHits(false)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end)
end

registerTest('WOUND-01 wounded leg limps', 918375, function()
    placePlayer()
    local dummy
    local ok, message = pcall(function()
        ragdoll.clearLimbDamage()
        dummy = spawnDummy('WOUND-01 walker', util.vector3(0, 150, 0), false)
        powerAndStand(dummy, 'walker')
        local function walk()
            local tilts, start = {}, dummy.position
            local byStance = { left = { 0, 0 }, right = { 0, 0 } }
            dummy:sendEvent('fixActorMove', 1)
            waitSeconds(1)
            local from, t0 = dummy.position, core.getSimulationTime()
            waitSeconds(4, function()
                local s = ragdoll.getState(dummy)
                if s then
                    tilts[#tilts + 1] = body(s, 'right_thigh').position.z - body(s, 'left_thigh').position.z
                    -- the lower foot is the standing one
                    local side = body(s, 'left_foot').position.z < body(s, 'right_foot').position.z and 'left' or 'right'
                    byStance[side][1] = byStance[side][1] + body(s, 'pelvis').position.z
                    byStance[side][2] = byStance[side][2] + 1
                end
            end)
            local speed = (dummy.position - from):length() / (core.getSimulationTime() - t0)
            dummy:sendEvent('fixActorMove', 0)
            waitSeconds(1.5)
            local sum = 0
            for _, v in ipairs(tilts) do sum = sum + v end
            -- back to the start for the next walk
            dummy:teleport(testArea.cell, start, { onGround = true })
            waitSeconds(1.5)
            local drop = byStance.right[1] / math.max(1, byStance.right[2]) - byStance.left[1] / math.max(1, byStance.left[2])
            return sum / #tilts, speed, drop
        end
        local tiltWhole, speedWhole, dropWhole = walk()
        ragdoll.addLimbDamage(dummy, 'left_thigh', 20)
        ragdoll.addLimbDamage(dummy, 'left_calf', 20)
        local tiltHurt, speedHurt, dropHurt = walk()
        -- drop: pelvis height standing on the right minus on the left (a
        -- left limp makes it positive)
        print(string.format('WOUND-01 leg summary tilt_whole=%.2f tilt_wounded=%.2f speed_whole=%.0f speed_wounded=%.0f drop_whole=%.2f drop_wounded=%.2f',
            tiltWhole, tiltHurt, speedWhole, speedHurt, dropWhole, dropHurt))
    end)
    ragdoll.clearLimbDamage()
    if dummy and dummy:isValid() then
        ragdoll.setPowered(dummy, false)
        dummy:remove()
    end
    if not ok then error(message, 0) end
end)

-- FATIGUE-01 (alpha 0.4 item 9): drag strokes cost fatigue, tired bodies
-- swing weaker. A longsword player makes 8 slow drags (180 degrees each,
-- 60 steps, below the commit speed) and reports the fatigue they cost; then
-- 6 committed strokes rested and 6 at 10% fatigue: the blade's peak speed
-- per stroke (median).
registerTest('FATIGUE-01 drags cost fatigue and tired swings are weaker', 918380, function()
    local player = placePlayer()
    local weapon = 'iron longsword'
    local ok, message = pcall(function()
        if not types.Actor.inventory(player):find(weapon) then
            world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        ragdoll.setPlayerAlwaysHits(true)
        player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 380, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 380 end)
        waitSeconds(1)
        powerAndStand(player, weapon)
        ragdoll.prepareMouseInput()
        player:sendEvent('fixSetFatigue', 1)
        waitSeconds(0.5)
        local peak, lastTip, lastT = 0, nil, nil
        local function sample()
            local s = ragdoll.getState(player)
            if not s or not s.weapon then return end
            local b = s.bodies[s.weapon.bodyIndex]
            local tip = b.position + (b.rotation * util.vector3(0, 0, 1)) * (s.weapon.length * 0.5)
            local now = core.getSimulationTime()
            if lastTip and now > lastT then peak = math.max(peak, (tip - lastTip):length() / (now - lastT)) end
            lastTip, lastT = tip, now
        end
        local perDeg = 1500 / 180
        local function run(degX, steps_)
            local step0 = tonumber(ragdoll.getState(player).physicsStep)
            local sentX = 0
            waitFor('drag', function()
                local n = tonumber(ragdoll.getState(player).physicsStep) - step0
                local x = math.floor(degX * perDeg * math.min(1, n / steps_))
                if x ~= sentX then ragdoll.injectMouseInput(x - sentX, 0) end
                sentX = x
                sample()
                return n >= steps_ + 12
            end, steps_ / 30 + 6)
        end
        local function press()
            waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            waitSeconds(0.4)
            local presses = ragdoll.getState(player).playerPressCount
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitFor('press registered', function() return ragdoll.getState(player).playerPressCount > presses end, 3)
        end
        local f0 = types.Actor.stats.dynamic.fatigue(player).current
        press()
        for drag = 1, 8 do run(drag % 2 == 1 and 180 or -180, 60) end
        ragdoll.injectMouseInput(0, 0, false, 1)
        waitSeconds(0.5)
        local dragCost = f0 - types.Actor.stats.dynamic.fatigue(player).current
        local function strokes(share)
            local peaks = {}
            for stroke = 1, 6 do
                player:sendEvent('fixSetFatigue', share)
                press()
                run(stroke % 2 == 1 and 110 or -110, 30)
                peak = 0
                run(stroke % 2 == 1 and -220 or 220, 8)
                peaks[#peaks + 1] = peak
                ragdoll.injectMouseInput(0, 0, false, 1)
                waitSeconds(0.8, sample)
            end
            table.sort(peaks)
            return peaks[math.ceil(#peaks / 2)]
        end
        local rested = strokes(1)
        local tired = strokes(0.1)
        player:sendEvent('fixSetFatigue', 1)
        print(string.format('FATIGUE-01 summary drag_cost=%.1f base=%.0f rested_peak=%.0f tired_peak=%.0f ratio=%.2f',
            dragCost, types.Actor.stats.dynamic.fatigue(player).base, rested, tired, tired / rested))
    end)
    ragdoll.injectMouseInput(0, 0, false, 1)
    ragdoll.setPlayerAlwaysHits(false)
    ragdoll.setPowered(player, false)
    if not ok then error(message, 0) end
end)

-- CHAMBER-01 (user 2026-10-02): one continuous sequence on the chamber path
-- (since build 32 the attack library: a right drag is the forehand line; LINES-01
-- covers all four attacks)
-- (OPENMW_RAGDOLL_HSC=1 OPENMW_RAGDOLL_HSC_PATH=1; add OPENMW_RAGDOLL_HSC_PUPPET=1
-- for the no-lag diagnostic): forward guard -> slow drag right into the
-- right-shoulder chamber -> hold -> cut left across -> follow-through, then
-- slow reversals with pauses. Per segment: the blade's largest turn in one
-- step and the steps turning over 15 degrees (flips), the elbow's largest jump
-- against the chest and jumps over 4 units, the arm solve's hand and wrist
-- distance from what the path asked (p90/max), the wrist outside its limits
-- (p90), and at each hold the blade direction in the body frame (x right,
-- y forward, z up). Screenshots at the holds.
local function chamberSequence(tag, plan)
    return function()
    local weapon = 'iron longsword'
    local player = placePlayer()
    local ok, message = pcall(function()
        if not types.Actor.inventory(player):find(weapon) then
            world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
            coroutine.yield()
        end
        player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 720, firstPerson = false })
        waitFor('player loadout', function() return playerReadySequence == 720 end)
        waitSeconds(1)
        powerAndStand(player, weapon)
        ragdoll.prepareMouseInput()
        local stats = nil
        local prevBlade, prevElbow
        local function bodyFrame(v)
            local yaw = player.rotation:getYaw()
            local fx, fy = math.sin(yaw), math.cos(yaw)
            return util.vector3(v.x * fy - v.y * fx, v.x * fx + v.y * fy, v.z)
        end
        local function sample()
            local s = ragdoll.getState(player)
            if not s or not s.weapon or not stats then return end
            local b = s.bodies[s.weapon.bodyIndex]
            local blade = (b.rotation * util.vector3(0, 0, 1)):normalize()
            local elbow = body(s, 'right_forearm').position - body(s, 'spine2').position
            if prevBlade then
                local turn = math.deg(math.acos(math.max(-1, math.min(1, blade:dot(prevBlade)))))
                stats.turnMax = math.max(stats.turnMax, turn)
                stats.turnSum = stats.turnSum + turn
                stats.turnCount = stats.turnCount + 1
                if turn > 15 then stats.flips = stats.flips + 1 end
            end
            if prevElbow then
                local jump = (elbow - prevElbow):length()
                stats.elbowMax = math.max(stats.elbowMax, jump)
                if jump > 4 then stats.snaps = stats.snaps + 1 end
            end
            prevBlade, prevElbow = blade, elbow
            if s.hsc then
                if s.hsc.handError >= 0 then stats.hand[#stats.hand + 1] = s.hsc.handError end
                if s.hsc.wristError >= 0 then stats.wrist[#stats.wrist + 1] = s.hsc.wristError end
                stats.s = s.hsc.s
            end
            stats.outside[#stats.outside + 1] = math.max(s.wristOutsideRight or 0, 0)
            stats.blade = bodyFrame(blade)
            -- The chest's turn: the shoulder line against the body's right.
            local line = bodyFrame(body(s, 'right_upper_arm').position - body(s, 'left_upper_arm').position)
            stats.chest = math.deg(math.atan2(-line.y, line.x))
            -- The elbow against the right shoulder: out = along the shoulder
            -- line (+ outside), up = vertical (+ above the shoulder).
            local outward = util.vector3(line.x, line.y, 0):normalize()
            local e = bodyFrame(body(s, 'right_forearm').position - body(s, 'right_upper_arm').position)
            stats.elbowOut, stats.elbowUp = e:dot(outward), e.z
        end
        local function q(t, f)
            if #t == 0 then return -1 end
            table.sort(t)
            return t[math.max(1, math.floor(f * #t))]
        end
        local function segment(name, dx, seconds, shots)
            stats = { turnMax = 0, turnSum = 0, turnCount = 0, flips = 0, elbowMax = 0, snaps = 0, hand = {}, wrist = {}, outside = {}, s = -9 }
            local steps = math.max(1, math.floor(seconds * 60))
            local step0 = tonumber(ragdoll.getState(player).physicsStep)
            local sent, shot = 0, 0
            waitFor(name, function()
                local n = tonumber(ragdoll.getState(player).physicsStep) - step0
                local x = math.floor(dx * math.min(1, n / steps))
                if x ~= sent then ragdoll.injectMouseInput(x - sent, 0) end
                sent = x
                sample()
                if shots and shot < shots and n >= (shot + 1) * steps / (shots + 1) then
                    shot = shot + 1
                    ragdoll.captureScreenshot()
                end
                return n >= steps
            end, seconds + 6)
            print(string.format(tag .. ' %-14s s=%.2f turn_mean=%.2f turn_max=%.1f flips=%d elbow_max=%.1f snaps=%d hand_err p90=%.1f max=%.1f wrist_err p90=%.0f max=%.0f outside_p90=%.0f blade=(%.2f,%.2f,%.2f) chest=%.0f elbow_out=%.1f elbow_up=%.1f',
                name, stats.s, stats.turnSum / math.max(1, stats.turnCount), stats.turnMax, stats.flips, stats.elbowMax, stats.snaps, q(stats.hand, 0.9), q(stats.hand, 1),
                q(stats.wrist, 0.9), q(stats.wrist, 1), q(stats.outside, 0.9), stats.blade.x, stats.blade.y, stats.blade.z,
                stats.chest or 0, stats.elbowOut or 0, stats.elbowUp or 0))
        end
        -- Side (from the right) and front-quarter stills of a held pose.
        local function views(name)
            print(tag .. ' view ' .. name .. ' play')
            ragdoll.captureScreenshot()
            waitSeconds(0.05)
            local s = ragdoll.getState(player)
            local center = body(s, 'spine2').position
            local yaw = player.rotation:getYaw()
            for _, view in ipairs({ { name = 'side', yaw = yaw - math.pi / 2, pitch = 0.05, distance = 170 },
                { name = 'front_quarter', yaw = yaw - 3 * math.pi / 4, pitch = 0.1, distance = 170 } }) do
                view.center = center
                player:sendEvent('fixInspectCamera', view)
                waitSeconds(0.08)
                print(tag .. ' view ' .. name .. ' ' .. view.name)
                ragdoll.captureScreenshot()
                waitSeconds(0.05)
            end
            player:sendEvent('fixInspectCamera', {})
            waitSeconds(0.05)
        end
        waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
        waitSeconds(0.8)
        local presses = ragdoll.getState(player).playerPressCount
        ragdoll.injectMouseInput(0, 0, true, 1)
        waitFor('press registered', function() return ragdoll.getState(player).playerPressCount > presses end, 3)
        plan(segment, views)
        ragdoll.injectMouseInput(0, 0, false, 1)
    end)
    ragdoll.injectMouseInput(0, 0, false, 1)
    ragdoll.setPowered(player, false)
    if not ok then error(message, 0) end
    end
end

-- LINES-01 (design "KINETIC: Weapon Arm and Elbow Design", attack library):
-- each one-handed attack through its loop with iron longsword: guard -> the
-- wind-up drag (up chop, right forehand, left backhand, down thrust) ->
-- chamber hold -> the cut drag back -> follow-through hold -> release ->
-- recovery. Per segment: the physical elbow's angle from the key's pole
-- (about the shoulder-hand line), the hand's distance from the target grip,
-- blade turns over 15 deg in one step (flips), elbow jumps over 4 units in
-- one step against the chest (snaps), the wrist outside its limits and the
-- chest's turn. Pass bars at the holds: elbow within 15 deg, hand within 3
-- units, no flips or snaps, wrist inside its limits.
local function linesTest(tag, line, dirX, dirY, ring)
    return function()
        local weapon = 'iron longsword'
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 730 + line, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 730 + line end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local function bodyFrame(v)
                local yaw = player.rotation:getYaw()
                local fx, fy = math.sin(yaw), math.cos(yaw)
                return util.vector3(v.x * fy - v.y * fx, v.x * fx + v.y * fy, v.z)
            end
            local function q(t, f)
                if #t == 0 then return -1 end
                table.sort(t)
                return t[math.max(1, math.floor(f * #t))]
            end
            local prevBlade, prevElbow, stats
            local fastCut = false
            local failures = {}
            local function sample()
                local s = ragdoll.getState(player)
                if not s or not s.weapon or not stats then return end
                local b = s.bodies[s.weapon.bodyIndex]
                local blade = (b.rotation * util.vector3(0, 0, 1)):normalize()
                local sh = body(s, 'right_upper_arm').position
                local el = body(s, 'right_forearm').position
                local hand = body(s, 'right_hand').position
                local elbowChest = el - body(s, 'spine2').position
                if prevBlade then
                    local turn = math.deg(math.acos(math.max(-1, math.min(1, blade:dot(prevBlade)))))
                    stats.turnMax = math.max(stats.turnMax, turn)
                    if turn > 15 then stats.flips = stats.flips + 1 end
                end
                if prevElbow then
                    local jump = (elbowChest - prevElbow):length()
                    stats.elbowJump = math.max(stats.elbowJump, jump)
                    if jump > 4 then stats.snaps = stats.snaps + 1 end
                end
                prevBlade, prevElbow = blade, elbowChest
                stats.outside[#stats.outside + 1] = math.max(s.wristOutsideRight or 0, 0)
                local line = bodyFrame(body(s, 'right_upper_arm').position - body(s, 'left_upper_arm').position)
                stats.chest = math.deg(math.atan2(-line.y, line.x))
                if s.hsc then
                    stats.s = s.hsc.s
                    stats.line = s.hsc.line
                    if s.hsc.grip and s.hsc.grip:length() > 1 then
                        stats.hand[#stats.hand + 1] = (hand - s.hsc.grip):length()
                    end
                    local pole = s.hsc.pole
                    if pole and pole:length() > 0.5 then
                        local d = (hand - sh):normalize()
                        local e = el - sh
                        local ep = e - d * e:dot(d)
                        local pp = pole - d * pole:dot(d)
                        -- A nearly straight arm has no elbow direction to judge.
                        if ep:length() > 3 and pp:length() > 0.1 then
                            local c = ep:normalize():dot(pp:normalize())
                            stats.elbow[#stats.elbow + 1] = math.deg(math.acos(math.max(-1, math.min(1, c))))
                        end
                    end
                end
            end
            local function segment(name, dx, dy, seconds, hold)
                stats = { turnMax = 0, flips = 0, elbowJump = 0, snaps = 0, hand = {}, elbow = {}, outside = {}, s = -9, line = -2 }
                local steps = math.max(1, math.floor(seconds * 60))
                local step0 = tonumber(ragdoll.getState(player).physicsStep)
                local sx, sy = 0, 0
                waitFor(name, function()
                    local n = tonumber(ragdoll.getState(player).physicsStep) - step0
                    local f = math.min(1, n / steps)
                    local x, y = math.floor(dx * f), math.floor(dy * f)
                    if x ~= sx or y ~= sy then ragdoll.injectMouseInput(x - sx, y - sy) end
                    sx, sy = x, y
                    -- Holds are judged on their settled second half.
                    if not hold or n >= steps / 2 then sample() end
                    return n >= steps
                end, seconds + 6)
                local elbowP90, handP90 = q(stats.elbow, 0.9), q(stats.hand, 0.9)
                local outside = q(stats.outside, 0.9)
                print(string.format('%s %-14s line=%d s=%.2f elbow p50=%.0f p90=%.0f hand p90=%.1f turn_max=%.1f flips=%d elbow_jump=%.1f snaps=%d wrist_out_p90=%.0f chest=%.0f',
                    tag, name, stats.line, stats.s, q(stats.elbow, 0.5), elbowP90, handP90, stats.turnMax, stats.flips,
                    stats.elbowJump, stats.snaps, outside, stats.chest or 0))
                if hold then
                    if elbowP90 > 15 then failures[#failures + 1] = name .. ' elbow ' .. math.floor(elbowP90) end
                    if handP90 > 3 then failures[#failures + 1] = name .. ' hand ' .. string.format('%.1f', handP90) end
                    if outside > 1 then failures[#failures + 1] = name .. ' wrist ' .. math.floor(outside) end
                end
                if stats.flips > 0 and not fastCut then failures[#failures + 1] = name .. ' flips ' .. stats.flips end
                if stats.snaps > 0 then failures[#failures + 1] = name .. ' snaps ' .. stats.snaps end
            end
            local function views(name)
                ragdoll.captureScreenshot()
                waitSeconds(0.05)
                local s = ragdoll.getState(player)
                local center = body(s, 'spine2').position
                local yaw = player.rotation:getYaw()
                for _, view in ipairs({ { name = 'side', yaw = yaw - math.pi / 2, pitch = 0.05, distance = 170 },
                    { name = 'behind', yaw = yaw + math.pi, pitch = 0.15, distance = 170 } }) do
                    view.center = center
                    player:sendEvent('fixInspectCamera', view)
                    waitSeconds(0.08)
                    print(tag .. ' view ' .. name .. ' ' .. view.name)
                    ragdoll.captureScreenshot()
                    waitSeconds(0.05)
                end
                player:sendEvent('fixInspectCamera', {})
                waitSeconds(0.05)
            end
            waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            waitSeconds(0.8)
            local presses = ragdoll.getState(player).playerPressCount
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitFor('press registered', function() return ragdoll.getState(player).playerPressCount > presses end, 3)
            segment('guard', 0, 0, 0.5, true)
            segment('wind-up', 600 * dirX, 600 * dirY, 1.0)
            segment('chamber-hold', 0, 0, 0.8, true)
            views('chamber')
            if ring then
                -- RING-01: the chamber walked round the whole wheel, an eighth
                -- at a time (300 counts across the line each), holding at each
                -- line's chamber. Mouse y is down-positive.
                local theta = math.atan2(-dirY, dirX)
                for i = 1, 8 do
                    local dx, dy = -math.sin(theta) * 300, -math.cos(theta) * 300
                    segment('ring-' .. i, dx, dy, 0.5)
                    segment('ring-hold-' .. i, 0, 0, 0.4, true)
                    theta = theta + math.pi / 4
                end
                ragdoll.injectMouseInput(0, 0, false, 1)
                segment('release-hold', 0, 0, 0.9)
                segment('recover', 0, 0, 1.2)
                print(string.format('%s result %s %s', tag, #failures == 0 and 'PASS' or 'FAIL', table.concat(failures, '; ')))
                return
            end
            segment('cut', -1200 * dirX, -1200 * dirY, 1.0)
            segment('follow-hold', 0, 0, 0.8, true)
            views('follow-through')
            -- A fast cut back up the route and down again: no snaps (the
            -- blade turns faster than 15 deg a step by design here).
            segment('back-to-chamber', 1200 * dirX, 1200 * dirY, 1.0)
            segment('chamber-hold2', 0, 0, 0.6, true)
            fastCut = true
            segment('fast-cut', -1200 * dirX, -1200 * dirY, 0.3)
            fastCut = false
            segment('follow-hold2', 0, 0, 0.8, true)
            ragdoll.injectMouseInput(0, 0, false, 1)
            segment('release-hold', 0, 0, 0.9)
            segment('recover', 0, 0, 1.2)
            segment('guard-again', 0, 0, 0.5, true)
            print(string.format('%s result %s %s', tag, #failures == 0 and 'PASS' or 'FAIL', table.concat(failures, '; ')))
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end
end
-- Mouse y is down-positive: the chop's wind-up drag (up) is negative y.
registerTest('LINES-01 chop with iron longsword', 918730, linesTest('LINES-01 chop', 0, 0, -1))
registerTest('LINES-01 forehand with iron longsword', 918731, linesTest('LINES-01 forehand', 1, 1, 0))
registerTest('LINES-01 backhand with iron longsword', 918732, linesTest('LINES-01 backhand', 2, -1, 0))
registerTest('LINES-01 thrust with iron longsword', 918733, linesTest('LINES-01 thrust', 3, 0, 1))
-- The diagonals of the cut wheel (2026-10-03).
local d = math.sqrt(0.5)
registerTest('LINES-01 up-right with iron longsword', 918734, linesTest('LINES-01 up-right', 4, d, -d))
registerTest('LINES-01 up-left with iron longsword', 918735, linesTest('LINES-01 up-left', 5, -d, -d))
registerTest('LINES-01 down-right with iron longsword', 918736, linesTest('LINES-01 down-right', 6, d, d))
registerTest('LINES-01 down-left with iron longsword', 918737, linesTest('LINES-01 down-left', 7, -d, d))

-- FLOW-01 (goal 2026-10-03: "weapon movement smooth, seamless, fluid"): a
-- player's chained attacks with one weapon, at mixed speeds, with steering and
-- the returns to the guard, judged on the PHYSICAL weapon over the whole run.
-- Per step: the blade's angular velocity (deg/step) and its change (jolt,
-- deg/step^2: steady fast swings are fine, sudden changes are not), sudden
-- changes of the elbow's velocity against the chest (snaps, units/step^2), the hand's distance from the
-- commanded grip, the wrist outside its limits and, two-handed, whether the
-- support hand stays on the grip. Rows per stroke and a total.
DEBUG_FLOW = false
local function flowTest(tag, weapon, sequence)
    return function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 760 + sequence, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 760 + sequence end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local function q(t, f)
                if #t == 0 then return -1 end
                local c = {}
                for i, v in ipairs(t) do c[i] = v end
                table.sort(c)
                return c[math.max(1, math.floor(f * #c))]
            end
            local function newStats()
                return { jolt = {}, speed = {}, hand = {}, move = {}, outside = {}, snaps = 0, jolts = 0, support = {}, supportLost = 0,
                    profile = {}, extraPeaks = 0, cmdProfile = {}, cmdPeaks = 0, prominent = 0, tipProfile = {}, head = {}, chest = {} }
            end
            local total = newStats()
            local stats
            local prevBlade, prevOmega, prevElbow, prevElbowVelocity, prevStep, prevAttached, prevCmd, prevTip
            local guardHead, guardChest
            local function add(key, value)
                stats[key][#stats[key] + 1] = value
                total[key][#total[key] + 1] = value
            end
            local function sample(moving)
                local s = ragdoll.getState(player)
                if not s or not s.weapon or not stats then return end
                local step = tonumber(s.physicsStep)
                if prevStep and step == prevStep then return end
                local dsteps = prevStep and math.max(1, step - prevStep) or 1
                prevStep = step
                local b = s.bodies[s.weapon.bodyIndex]
                local blade = (b.rotation * util.vector3(0, 0, 1)):normalize()
                if prevBlade then
                    local axis = prevBlade:cross(blade)
                    local sine = axis:length()
                    local angle = math.deg(math.atan2(sine, prevBlade:dot(blade))) / dsteps
                    local omega = sine > 1e-6 and axis:normalize() * angle or util.vector3(0, 0, 0)
                    add('speed', angle)
                    stats.profile[#stats.profile + 1] = angle
                    if prevOmega then
                        local jolt = (omega - prevOmega):length()
                        add('jolt', jolt)
                        if jolt > 8 then
                            stats.jolts = stats.jolts + 1
                            total.jolts = total.jolts + 1
                        end
                    end
                    prevOmega = omega
                end
                prevBlade = blade
                -- The tip's speed (units/step): a thrust is a translation, its
                -- blade barely turns (1-2 deg a step), so its smoothness is
                -- judged on the tip.
                local tipNow = b.position + blade * (s.weapon.length * 0.5)
                if prevTip then stats.tipProfile[#stats.tipProfile + 1] = (tipNow - prevTip):length() / dsteps end
                prevTip = tipNow
                -- A snap is a sudden change of the elbow's velocity against the
                -- chest (units/step^2), not a fast elbow: a quick cut moves it
                -- 4-6 units a step smoothly.
                local elbowChest = body(s, 'right_forearm').position - body(s, 'spine2').position
                if prevElbow then
                    local velocity = (elbowChest - prevElbow) / dsteps
                    if prevElbowVelocity and (velocity - prevElbowVelocity):length() > 2.5 then
                        stats.snaps = stats.snaps + 1
                        total.snaps = total.snaps + 1
                    end
                    prevElbowVelocity = velocity
                end
                prevElbow = elbowChest
                add('outside', math.max(s.wristOutsideRight or 0, 0))
                -- The head against the chest: how far each turns from its guard
                -- orientation (the head keeps the eyes on the target while the
                -- chest turns with the swing).
                local function turnFrom(ref, q)
                    if not ref then return 0 end
                    -- The whole turn between the two orientations.
                    local angle = 0
                    for _, axis in ipairs({ util.vector3(1, 0, 0), util.vector3(0, 1, 0), util.vector3(0, 0, 1) }) do
                        local v1, v2 = q * axis, ref * axis
                        angle = math.max(angle, math.deg(math.acos(math.max(-1, math.min(1, v1:dot(v2))))))
                    end
                    return angle
                end
                -- Relative to the pelvis, so the body's own turning is left out.
                local pelvisInv = body(s, 'pelvis').rotation:inverse()
                local headQ, chestQ = pelvisInv * body(s, 'head').rotation, pelvisInv * body(s, 'spine2').rotation
                guardHead = guardHead or headQ
                guardChest = guardChest or chestQ
                if moving then
                    add('head', turnFrom(guardHead, headQ))
                    add('chest', turnFrom(guardChest, chestQ))
                end
                -- The commanded blade's own turn, to tell a rough command
                -- from a rough physical follow.
                if s.hsc and s.hsc.blade and s.hsc.blade:length() > 0.5 then
                    local c = s.hsc.blade:normalize()
                    if prevCmd then
                        stats.cmdProfile[#stats.cmdProfile + 1] = math.deg(math.acos(math.max(-1, math.min(1, prevCmd:dot(c))))) / dsteps
                        stats.sProfile = stats.sProfile or {}
                        stats.sProfile[#stats.sProfile + 1] = string.format('%.3f/%d/%.0f', s.hsc.s, s.hsc.stage, math.deg(s.hsc.angle))
                    end
                    prevCmd = c
                end
                if s.hsc and s.hsc.grip and s.hsc.grip:length() > 1 then
                    local e = (body(s, 'right_hand').position - s.hsc.grip):length()
                    add('hand', e)
                    if moving then add('move', e) end
                end
                if s.weapon.twoHanded then
                    local g = s.weapon.supportGrip
                    local attached = (g and g.attached) and true or false
                    add('support', attached and 1 or 0)
                    if prevAttached and not attached then
                        stats.supportLost = stats.supportLost + 1
                        total.supportLost = total.supportLost + 1
                    end
                    prevAttached = attached
                end
            end
            local function report(name, st)
                local supportShare = -1
                if #st.support > 0 then
                    local n = 0
                    for _, v in ipairs(st.support) do n = n + v end
                    supportShare = n / #st.support
                end
                print(string.format('%s %-16s peaks+=%d prominent+=%d cmd_peaks+=%d jolts=%d jolt_p99=%.1f jolt_max=%.1f speed_max=%.1f snaps=%d hand_p50=%.1f hand_p90=%.1f move_p90=%.1f wrist_out_p90=%.0f support=%.2f lost=%d',
                    tag, name, st.extraPeaks, st.prominent, st.cmdPeaks, st.jolts, q(st.jolt, 0.99), q(st.jolt, 1), q(st.speed, 1), st.snaps, q(st.hand, 0.5),
                    q(st.hand, 0.9), q(st.move, 0.9), q(st.outside, 0.9), supportShare, st.supportLost))
            end
            -- Movement units (Rohrer et al. 2002): peaks of the blade's speed
            -- over a stroke above 20% of its maximum, lightly smoothed (3
            -- steps). One peak is one smooth movement; every extra peak is a
            -- sub-movement - a catch-up, a ring, a jolt.
            -- Prominent extra peaks (2026-10-04): a peak counts only if the
            -- speed between it and the previous counted peak fell by at least
            -- 10% of the stroke's top speed - a wiggle on a plateau (a few
            -- percent, the route at its top speed) is not a sub-movement.
            local function prominentPeaks(profile)
                local n = #profile
                if n < 5 then return 0 end
                local smooth = {}
                for i = 1, n do
                    local a, b, c = profile[math.max(1, i - 1)], profile[i], profile[math.min(n, i + 1)]
                    smooth[i] = (a + b + c) / 3
                end
                local top = 0
                for i = 1, n do top = math.max(top, smooth[i]) end
                if top < 1 then return 0 end
                local peaks, lastPeak, valley = 0, nil, nil
                for i = 2, n - 1 do
                    if valley then valley = math.min(valley, smooth[i]) end
                    if smooth[i] > smooth[i - 1] and smooth[i] >= smooth[i + 1] and smooth[i] > 0.2 * top then
                        if not lastPeak then
                            peaks, lastPeak, valley = 1, smooth[i], smooth[i]
                        elseif math.min(lastPeak, smooth[i]) - valley >= 0.1 * top then
                            peaks, lastPeak, valley = peaks + 1, smooth[i], smooth[i]
                        elseif smooth[i] > lastPeak then
                            lastPeak, valley = smooth[i], smooth[i]
                        end
                    end
                end
                return math.max(0, peaks - 1)
            end
            local function extraPeaks(profile)
                local n = #profile
                if n < 5 then return 0 end
                local smooth = {}
                for i = 1, n do
                    local a, b, c = profile[math.max(1, i - 1)], profile[i], profile[math.min(n, i + 1)]
                    smooth[i] = (a + b + c) / 3
                end
                local top = 0
                for i = 1, n do top = math.max(top, smooth[i]) end
                if top < 1 then return 0 end
                local peaks = 0
                for i = 2, n - 1 do
                    if smooth[i] > smooth[i - 1] and smooth[i] >= smooth[i + 1] and smooth[i] > 0.2 * top then
                        peaks = peaks + 1
                    end
                end
                return math.max(0, peaks - 1)
            end
            local function move(name, dx, dy, seconds)
                stats = newStats()
                local steps = math.max(1, math.floor(seconds * 60))
                local step0 = tonumber(ragdoll.getState(player).physicsStep)
                local sx, sy = 0, 0
                waitFor(name, function()
                    local n = tonumber(ragdoll.getState(player).physicsStep) - step0
                    -- A minimum-jerk mouse stroke, as a hand moves a mouse.
                    local t = math.min(1, n / steps)
                    local f = t * t * t * (10 - 15 * t + 6 * t * t)
                    local x, y = math.floor(dx * f), math.floor(dy * f)
                    if x ~= sx or y ~= sy then ragdoll.injectMouseInput(x - sx, y - sy) end
                    sx, sy = x, y
                    sample(dx ~= 0 or dy ~= 0)
                    return n >= steps
                end, seconds + 6)
                -- Judged on the strokes only (wind-ups, steering, cuts).
                stats.extraPeaks = (dx ~= 0 or dy ~= 0) and extraPeaks(stats.profile) or 0
                total.extraPeaks = total.extraPeaks + stats.extraPeaks
                stats.prominent = (dx ~= 0 or dy ~= 0)
                        and prominentPeaks(name:find('thrust') and stats.tipProfile or stats.profile) or 0
                total.prominent = total.prominent + stats.prominent
                stats.cmdPeaks = (dx ~= 0 or dy ~= 0) and extraPeaks(stats.cmdProfile) or 0
                if DEBUG_FLOW and (dx ~= 0 or dy ~= 0) then
                    local t = {}
                    for i, v in ipairs(stats.cmdProfile) do t[#t + 1] = string.format('%.1f', v) end
                    local u = {}
                    for i, v in ipairs(stats.profile) do u[#u + 1] = string.format('%.1f', v) end
                    print(tag .. ' cmdprofile ' .. name .. ' ' .. table.concat(t, ' '))
                    print(tag .. ' physprofile ' .. name .. ' ' .. table.concat(u, ' '))
                    print(tag .. ' sprofile ' .. name .. ' ' .. table.concat(stats.sProfile or {}, ' '))
                end
                total.cmdPeaks = total.cmdPeaks + stats.cmdPeaks
                report(name, stats)
            end
            local function press()
                local presses = ragdoll.getState(player).playerPressCount
                ragdoll.injectMouseInput(0, 0, true, 1)
                waitFor('press', function() return ragdoll.getState(player).playerPressCount > presses end, 3)
            end
            local function release() ragdoll.injectMouseInput(0, 0, false, 1) end
            -- One attack: the wind-up drag, a short hold, the cut back across,
            -- the follow-through, the release and the way back to the guard.
            local function attack(name, dirX, dirY, wind, cut, steer)
                press()
                move(name .. '-wind', HSC_WIND * dirX, HSC_WIND * dirY, wind)
                local cx, cy = dirX, dirY
                if steer then
                    -- The stance circle (build 38): sideways from deep retracts
                    -- to the circle, then a drag out on the new diagonal winds
                    -- up again, and the cut comes back through the centre.
                    move(name .. '-steer', -dirY * (HSC_WIND - 300), dirX * (HSC_WIND - 300), 0.6)
                    local rx, ry = (dirX - dirY) * math.sqrt(0.5), (dirY + dirX) * math.sqrt(0.5)
                    move(name .. '-rewind', rx * (HSC_WIND - 300), ry * (HSC_WIND - 300), 0.4)
                    cx, cy = rx, ry
                end
                move(name .. '-hold', 0, 0, 0.25)
                move(name .. '-cut', -HSC_CUT * cx, -HSC_CUT * cy, cut)
                move(name .. '-follow', 0, 0, 0.3)
                release()
                move(name .. '-recover', 0, 0, 0.5)
            end
            local d = math.sqrt(0.5)
            waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            waitSeconds(0.8)
            move('guard', 0, 0, 0.5)
            attack('chop', 0, -1, 0.5, 0.45)
            attack('forehand', 1, 0, 0.4, 0.3)
            attack('backhand', -1, 0, 0.5, 0.45)
            attack('up-right', d, -d, 0.35, 0.25)
            attack('down-left', -d, d, 0.5, 0.35)
            attack('thrust', 0, 1, 0.4, 0.3)
            attack('steered', 0, -1, 0.5, 0.4, true)
            attack('down-right', d, d, 0.3, 0.25)
            move('guard-again', 0, 0, 0.8)
            report('total', total)
            print(string.format('%s total head_turn_p50=%.1f head_turn_p90=%.1f chest_turn_p50=%.1f chest_turn_p90=%.1f', tag,
                q(total.head, 0.5), q(total.head, 0.9), q(total.chest, 0.5), q(total.chest, 0.9)))
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end
end
registerTest('FLOW-01 iron longsword', 918750, flowTest('FLOW-01 longsword', 'iron longsword', 0))
registerTest('FLOW-01 iron claymore', 918751, flowTest('FLOW-01 claymore', 'iron claymore', 1))
registerTest('FLOW-01 iron warhammer', 918752, flowTest('FLOW-01 warhammer', 'iron warhammer', 2))
registerTest('FLOW-01 iron battle axe', 918753, flowTest('FLOW-01 battle axe', 'iron battle axe', 3))
registerTest('FLOW-01 iron spear', 918754, flowTest('FLOW-01 spear', 'iron spear', 4))
registerTest('FLOW-01 wooden staff', 918755, flowTest('FLOW-01 staff', 'wooden staff', 5))

-- THRUST-01 (user 10-31: "can we make it so the thrust is more evident? ...
-- thrusting with the claymore the sword doesnt travel that much"): from the
-- guard, standing, no foe, a dragged thrust (down, back up) and a clicked one
-- (the thrust button). Per thrust, along the facing the player had at the
-- guard: the tip's place at the guard, its least through the wind-up and its
-- most through the drive (travel = most - least, reach = most - guard), the
-- pelvis's most forward shift, the tip's top speed (u/s), the wrist outside
-- its limits and, two-handed, the support grip. FAIL: the support hand let
-- go, or the thrust drove less than 30 units.
local function thrustTest(tag, weapon, sequence)
    return function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 940 + sequence, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 940 + sequence end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local function q(t, f)
                if #t == 0 then return -1 end
                local c = {}
                for i, v in ipairs(t) do c[i] = v end
                table.sort(c)
                return c[math.max(1, math.floor(f * #c))]
            end
            local failures = {}
            local forward, origin, pelvis0, stats, prevTip, prevStep, prevAttached
            local function tipOf(s)
                local b = s.bodies[s.weapon.bodyIndex]
                return b.position + (b.rotation * util.vector3(0, 0, 1)):normalize() * (s.weapon.length * 0.5)
            end
            local function sample(phase)
                local s = ragdoll.getState(player)
                if not s or not s.weapon then return end
                local step = tonumber(s.physicsStep)
                if prevStep and step == prevStep then return end
                local dsteps = prevStep and math.max(1, step - prevStep) or 1
                prevStep = step
                local tip = tipOf(s)
                local along = (tip - origin):dot(forward)
                if phase == 'guard' then
                    stats.guard[#stats.guard + 1] = along
                elseif phase == 'wind' then
                    stats.least = math.min(stats.least, along)
                else
                    stats.most = math.max(stats.most, along)
                end
                stats.pelvis = math.max(stats.pelvis, (body(s, 'pelvis').position - pelvis0):dot(forward))
                if prevTip then stats.speed = math.max(stats.speed, (tip - prevTip):length() * 60 / dsteps) end
                prevTip = tip
                stats.outside[#stats.outside + 1] = math.max(s.wristOutsideRight or 0, 0)
                if s.weapon.twoHanded then
                    local g = s.weapon.supportGrip
                    local attached = (g and g.attached) and true or false
                    stats.support[#stats.support + 1] = attached and 1 or 0
                    if prevAttached and not attached then stats.lost = stats.lost + 1 end
                    prevAttached = attached
                end
            end
            local function run(seconds, phase, dx, dy)
                local steps = math.max(1, math.floor(seconds * 60))
                local step0 = tonumber(ragdoll.getState(player).physicsStep)
                local sx, sy = 0, 0
                waitFor(phase, function()
                    local n = tonumber(ragdoll.getState(player).physicsStep) - step0
                    if dx then
                        local t = math.min(1, n / steps)
                        local f = t * t * t * (10 - 15 * t + 6 * t * t)
                        local x, y = math.floor(dx * f), math.floor(dy * f)
                        if x ~= sx or y ~= sy then ragdoll.injectMouseInput(x - sx, y - sy) end
                        sx, sy = x, y
                    end
                    sample(phase)
                    return n >= steps
                end, seconds + 6)
            end
            local function begin()
                waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                waitSeconds(0.8)
                local s = ragdoll.getState(player)
                local yaw = player.rotation:getYaw()
                forward = util.vector3(math.sin(yaw), math.cos(yaw), 0)
                origin = body(s, 'pelvis').position
                pelvis0 = origin
                stats = { guard = {}, least = 1e9, most = -1e9, pelvis = 0, speed = 0, outside = {}, support = {}, lost = 0 }
                prevTip, prevStep, prevAttached = nil, nil, nil
                run(0.4, 'guard')
            end
            local function report(name)
                local guard = 0
                for _, v in ipairs(stats.guard) do guard = guard + v / #stats.guard end
                local support = -1
                if #stats.support > 0 then
                    support = 0
                    for _, v in ipairs(stats.support) do support = support + v / #stats.support end
                end
                local travel = stats.most - stats.least
                print(string.format('%s %-7s guard=%.1f least=%.1f most=%.1f travel=%.1f reach=%.1f pelvis=%.1f speed_max=%.0f wrist_out_p90=%.0f support=%.2f lost=%d',
                    tag, name, guard, stats.least, stats.most, travel, stats.most - guard, stats.pelvis, stats.speed,
                    q(stats.outside, 0.9), support, stats.lost))
                if stats.lost > 0 then failures[#failures + 1] = name .. ' support lost ' .. stats.lost end
                if travel < 30 then failures[#failures + 1] = string.format('%s travel %.1f', name, travel) end
            end
            -- Dragged: down to the chamber, a short hold, back up through the
            -- centre (the thrust), the follow-through, the release.
            begin()
            local presses = ragdoll.getState(player).playerPressCount
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitFor('press', function() return ragdoll.getState(player).playerPressCount > presses end, 3)
            run(0.45, 'wind', 0, HSC_WIND)
            run(0.25, 'wind', 0, 0)
            run(0.3, 'drive', 0, -HSC_CUT)
            run(0.4, 'drive', 0, 0)
            ragdoll.injectMouseInput(0, 0, false, 1)
            run(1.0, 'recover')
            report('dragged')
            -- Clicked: the thrust button pressed and let go; the wind-up and
            -- the drive play on their own.
            begin()
            ragdoll.injectMouseInput(0, 0, true, 2)
            coroutine.yield()
            coroutine.yield()
            ragdoll.injectMouseInput(0, 0, false, 2)
            local stage = 'wind'
            waitSeconds(1.6, function()
                local s = ragdoll.getState(player)
                if stage == 'wind' and s and s.hsc and s.hsc.stage == 2 then stage = 'drive' end
                sample(stage)
            end)
            report('clicked')
            print(string.format('%s result %s %s', tag, #failures == 0 and 'PASS' or 'FAIL', table.concat(failures, '; ')))
            if #failures > 0 then error(tag .. ' ' .. table.concat(failures, '; '), 0) end
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end
end
registerTest('THRUST-01 iron longsword', 918790, thrustTest('THRUST-01 longsword', 'iron longsword', 0))
registerTest('THRUST-01 iron claymore', 918791, thrustTest('THRUST-01 claymore', 'iron claymore', 1))
registerTest('THRUST-01 iron spear', 918792, thrustTest('THRUST-01 spear', 'iron spear', 2))
registerTest('THRUST-01 wooden staff', 918793, thrustTest('THRUST-01 staff', 'wooden staff', 3))

-- FLOW-02: pictures of a weapon's attacks (side and behind at the chamber and
-- the follow-through of the chop, forehand and thrust), for judging the arms -
-- the support arm above all - by eye. Prints the support grip's state.
local function viewsTest(tag, weapon, sequence)
    return function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 780 + sequence, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 780 + sequence end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local function drag(dx, dy, seconds)
                local steps = math.max(1, math.floor(seconds * 60))
                local step0 = tonumber(ragdoll.getState(player).physicsStep)
                local sx, sy = 0, 0
                waitFor('drag', function()
                    local n = tonumber(ragdoll.getState(player).physicsStep) - step0
                    local t = math.min(1, n / steps)
                    local f = t * t * t * (10 - 15 * t + 6 * t * t)
                    local x, y = math.floor(dx * f), math.floor(dy * f)
                    if x ~= sx or y ~= sy then ragdoll.injectMouseInput(x - sx, y - sy) end
                    sx, sy = x, y
                    return n >= steps
                end, seconds + 6)
            end
            local function views(name)
                waitSeconds(0.4)
                local s = ragdoll.getState(player)
                local g = s.weapon and s.weapon.supportGrip
                local left = body(s, 'left_hand').position
                local right = body(s, 'right_hand').position
                print(string.format('%s %s support=%s separation=%.2f hands=%.1f', tag, name,
                    tostring(g and g.attached), g and g.anchorSeparation or -1, (left - right):length()))
                local center = body(s, 'spine2').position
                local yaw = player.rotation:getYaw()
                for _, view in ipairs({ { name = 'side', yaw = yaw - math.pi / 2, pitch = 0.05, distance = 170 },
                    { name = 'behind', yaw = yaw + math.pi, pitch = 0.15, distance = 170 } }) do
                    view.center = center
                    player:sendEvent('fixInspectCamera', view)
                    waitSeconds(0.08)
                    print(tag .. ' view ' .. name .. ' ' .. view.name)
                    ragdoll.captureScreenshot()
                    waitSeconds(0.05)
                end
                player:sendEvent('fixInspectCamera', {})
                waitSeconds(0.05)
            end
            local function attack(name, dirX, dirY)
                local presses = ragdoll.getState(player).playerPressCount
                ragdoll.injectMouseInput(0, 0, true, 1)
                waitFor('press', function() return ragdoll.getState(player).playerPressCount > presses end, 3)
                drag(HSC_WIND * dirX, HSC_WIND * dirY, 0.5)
                views(name .. '-chamber')
                drag(-HSC_CUT * dirX, -HSC_CUT * dirY, 0.4)
                views(name .. '-follow')
                ragdoll.injectMouseInput(0, 0, false, 1)
                waitSeconds(1.8)
            end
            waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            waitSeconds(0.8)
            views('guard')
            attack('chop', 0, -1)
            attack('forehand', 1, 0)
            attack('thrust', 0, 1)
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end
end
registerTest('FLOW-02 views iron claymore', 918760, viewsTest('FLOW-02 claymore', 'iron claymore', 1))
registerTest('FLOW-02 views iron warhammer', 918761, viewsTest('FLOW-02 warhammer', 'iron warhammer', 2))
registerTest('FLOW-02 views iron battle axe', 918762, viewsTest('FLOW-02 battle axe', 'iron battle axe', 3))
registerTest('FLOW-02 views iron longsword', 918763, viewsTest('FLOW-02 longsword', 'iron longsword', 0))

-- HANDOFF-01 (consolidation, 2026-10-03): changes of arm owner (draw, guard
-- key during and after an attack, a click attack and a block) must not snap
-- the hand. Logs every owner change; per change, the largest change of the
-- weapon hand's velocity (units/step^2) within 10 steps, against the run's
-- median.
local function handoffTest(tag, weapon, sequence)
    return function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 800 + sequence, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 800 + sequence end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local rows, changes = {}, {}
            local prevStep, prevHand, prevVel, prevOwner = nil, nil, nil, nil
            local function sample(label)
                local s = ragdoll.getState(player)
                if not s then return end
                local step = tonumber(s.physicsStep)
                if prevStep and step == prevStep then return end
                prevStep = step
                local hand = body(s, 'right_hand').position - body(s, 'pelvis').position
                local jump = 0
                if prevHand then
                    local vel = hand - prevHand
                    if prevVel then jump = (vel - prevVel):length() end
                    prevVel = vel
                end
                prevHand = hand
                local owner = tostring(s.armOwnerRight) .. '/' .. tostring(s.armOwnerLeft) .. '/trunk:' .. tostring(s.ownerTrunk)
                    .. '/head:' .. tostring(s.ownerHead) .. '/legs:' .. tostring(s.ownerLegs)
                local g = s.weapon and s.weapon.supportGrip
                rows[#rows + 1] = { step = step, jump = jump, grip = g and (tostring(g.state) .. '/' .. tostring(g.reason)) or '-',
                    tip = s.tip and s.tip.command or '-' }
                if owner ~= prevOwner then
                    changes[#changes + 1] = { step = step, from = prevOwner or '-', to = owner, label = label }
                    prevOwner = owner
                end
            end
            local function hold(label, seconds) waitSeconds(seconds, function() sample(label) end) end
            local function drag(label, dx, dy, seconds)
                local steps = math.max(1, math.floor(seconds * 60))
                local step0 = tonumber(ragdoll.getState(player).physicsStep)
                local sx, sy = 0, 0
                waitFor(label, function()
                    local n = tonumber(ragdoll.getState(player).physicsStep) - step0
                    local t = math.min(1, n / steps)
                    local f = t * t * t * (10 - 15 * t + 6 * t * t)
                    local x, y = math.floor(dx * f), math.floor(dy * f)
                    if x ~= sx or y ~= sy then ragdoll.injectMouseInput(x - sx, y - sy) end
                    sx, sy = x, y
                    sample(label)
                    return n >= steps
                end, seconds + 6)
            end
            local function press() ragdoll.injectMouseInput(0, 0, true, 1) end
            local function release() ragdoll.injectMouseInput(0, 0, false, 1) end
            hold('guard', 1.0)
            player:sendEvent('fixPlayerStance', false)
            hold('sheathe', 2.0)
            player:sendEvent('fixPlayerStance', true)
            hold('draw', 2.5)
            press()
            drag('wind', 0, -HSC_WIND, 0.5)
            drag('cut', 0, HSC_CUT, 0.4)
            release()
            hold('follow', 0.2)
            ragdoll.setGuard(player, true)
            hold('block', 0.6)
            ragdoll.setGuard(player, false)
            hold('unblock', 1.5)
            press()
            hold('click', 0.05)
            release()
            hold('click-attack', 0.3)
            ragdoll.setGuard(player, true)
            hold('block-in-attack', 0.5)
            ragdoll.setGuard(player, false)
            hold('end', 2.0)
            local all = {}
            for _, r in ipairs(rows) do all[#all + 1] = r.jump end
            table.sort(all)
            local median = all[math.max(1, math.floor(#all / 2))] or 0
            local worst = 0
            for _, c in ipairs(changes) do
                local peak = 0
                for _, r in ipairs(rows) do
                    if math.abs(r.step - c.step) <= 10 then peak = math.max(peak, r.jump) end
                end
                worst = math.max(worst, peak)
                print(string.format('%s change step=%d %s -> %s during=%s hand_jump_max=%.2f', tag, c.step, c.from, c.to, c.label, peak))
                for _, r in ipairs(rows) do
                    if math.abs(r.step - c.step) <= 10 and r.jump == peak and peak > 1 then
                        print(string.format('%s   worst at %+d steps grip=%s tip=%s', tag, r.step - c.step, r.grip, tostring(r.tip)))
                    end
                end
            end
            print(string.format('%s total changes=%d worst_change_jump=%.2f run_p99=%.2f run_median=%.2f', tag, #changes, worst,
                all[math.max(1, math.floor(#all * 0.99))] or 0, median))
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.setGuard(player, false)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end
end
registerTest('HANDOFF-01 iron longsword', 918770, handoffTest('HANDOFF-01 longsword', 'iron longsword', 0))
registerTest('HANDOFF-01 iron claymore', 918771, handoffTest('HANDOFF-01 claymore', 'iron claymore', 1))
registerTest('HANDOFF-01 wooden staff', 918772, handoffTest('HANDOFF-01 staff', 'wooden staff', 2))

-- The weapon roadmap of 2026-10-04 (stance circle, mouse-driven cut, drive
-- phases, momentum chains):
--   STANCE-02 the stance circle: slow moves inside the radius change guards
--             and never cut, a released stance stays, past the radius commits
--             a wind-up, sideways from deep retracts first, a fast drag back
--             cuts, a quick flick across the circle is a snap cut;
--   ONSET-01  mouse-to-blade motion onset (ms) for steady drags from rest
--             (stance moves and a cut), bar 50 ms;
--   CHOP-01   a straight-down stroke is a vertical chop: the tip's and the
--             blade's sideways error from the chest's vertical plane while
--             the cut passes the front;
--   CHAIN-01  a backhand chained into the forehand's follow-through: the
--             blade's speed dip between the two cuts against their peaks.
DEBUG_ONSET = false
-- The arm's quality per physics step (goal 2026-10-05, tools/body/arm_quality.py
-- sums the ARMQ lines): in the guard, steps with the hand turned more than 45
-- deg from its command (blade and roll) and the hand's distance from its grip
-- goal; in wind-ups and cuts, steps turned more than 90; re-seats.
local function armTally(label)
    local t = { guard = 0, g45 = 0, rest = {}, attack = 0, a90 = 0, steps = 0, reseats = 0, last = nil,
        wind90 = 0, cut90 = 0, back = 0, b45 = 0, run90 = 0, held90 = 0, rest8 = 0, ends = 0, end90 = 0 }
    function t.add(hsc)
        if not hsc or not hsc.wristError or hsc.wristError < 0 then return end
        t.steps = t.steps + 1
        local r = hsc.reseats or 0
        if t.last and r > t.last then t.reseats = t.reseats + (r - t.last) end
        t.last = r
        -- a stance kept inside the circle (300 counts) is a guard too
        local r = math.sqrt((hsc.stanceX or 0) ^ 2 + (hsc.stanceY or 0) ^ 2)
        if hsc.stage == 0 or (hsc.stage == 1 and r <= 300) then
            t.guard = t.guard + 1
            if hsc.wristError > 45 then t.g45 = t.g45 + 1 end
            t.rest[#t.rest + 1] = hsc.handError
            if hsc.handError > 8 then t.rest8 = t.rest8 + 1 end
        elseif hsc.stage == 1 or hsc.stage == 2 then
            t.attack = t.attack + 1
            -- held at the cut's end (play holds the button after a drag)
            if hsc.stage == 2 and (hsc.s or 0) >= 1.99 then
                t.ends = t.ends + 1
                if hsc.wristError > 90 then t.end90 = t.end90 + 1 end
            end
            if hsc.wristError > 90 then
                t.a90 = t.a90 + 1
                if hsc.stage == 1 then t.wind90 = t.wind90 + 1 else t.cut90 = t.cut90 + 1 end
                -- held: still more than 90 deg off after 0.25 s (a flip or a
                -- stuck wrist; a heavy blade trailing a fast command catches up
                -- within 3-6 steps)
                t.run90 = t.run90 + 1
                if t.run90 == 15 then t.held90 = t.held90 + 15 elseif t.run90 > 15 then t.held90 = t.held90 + 1 end
            else
                t.run90 = 0
            end
        elseif hsc.stage == 3 then
            t.back = t.back + 1
            if hsc.wristError > 45 then t.b45 = t.b45 + 1 end
        end
    end
    function t.report()
        table.sort(t.rest)
        local p90 = #t.rest > 0 and t.rest[math.max(1, math.ceil(#t.rest * 0.9))] or -1
        print(string.format('ARMQ %s guard=%d g45=%d restP90=%.2f attack=%d a90=%d reseats=%d steps=%d wind90=%d cut90=%d return=%d return45=%d held90=%d rest8=%d ends=%d end90=%d',
            label, t.guard, t.g45, p90, t.attack, t.a90, t.reseats, t.steps, t.wind90, t.cut90, t.back, t.b45, t.held90, t.rest8, t.ends, t.end90))
    end
    return t
end

local function controlTest(tag, weapon, sequence, kind)
    return function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 820 + sequence,
                firstPerson = kind:match('fp$') ~= nil })
            waitFor('player loadout', function() return playerReadySequence == 820 + sequence end)
            -- The controls at a master's skill: below 50 a press waits for
            -- the guard instead of chaining (HSC_SKILL, the real skill).
            player:sendEvent('fixSetWeaponSkill', 100)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local failures = {}
            local function check(name, value)
                print(string.format('%s check %-40s %s', tag, name, value and 'ok' or 'FAIL'))
                if not value then failures[#failures + 1] = name end
            end
            local function frame()
                local yaw = player.rotation:getYaw()
                return util.vector3(math.cos(yaw), -math.sin(yaw), 0), util.vector3(math.sin(yaw), math.cos(yaw), 0)
            end
            -- The blade's tip, the blade and the chest, this physics step.
            local function sample()
                local s = ragdoll.getState(player)
                local b = s.bodies[s.weapon.bodyIndex]
                local blade = (b.rotation * util.vector3(0, 0, 1)):normalize()
                local chest = body(s, 'spine2').position
                local pelvis = body(s, 'pelvis').position
                return { step = tonumber(s.physicsStep), tip = b.position + blade * (s.weapon.length * 0.5) - chest,
                    blade = blade, hsc = s.hsc, chest = chest, pelvis = pelvis,
                    pelvisTip = b.position + blade * (s.weapon.length * 0.5) - pelvis,
                    hand = body(s, 'right_hand').position - pelvis }
            end
            local function nextStep(previous)
                local s
                waitFor('step', function()
                    s = sample()
                    return s.step ~= previous
                end, 2)
                return s
            end
            local function press()
                local presses = ragdoll.getState(player).playerPressCount
                ragdoll.injectMouseInput(0, 0, true, 1)
                waitFor('press', function() return ragdoll.getState(player).playerPressCount > presses end, 3)
            end
            local function release() ragdoll.injectMouseInput(0, 0, false, 1) end
            -- A minimum-jerk drag over `seconds`, calling each(sample) per step.
            local function drag(dx, dy, seconds, each)
                local steps = math.max(1, math.floor(seconds * 60))
                local first = sample()
                local step0, previous = first.step, first.step
                local sx, sy = 0, 0
                while true do
                    local s = nextStep(previous)
                    previous = s.step
                    local n = s.step - step0
                    local t = math.min(1, n / steps)
                    local f = t * t * t * (10 - 15 * t + 6 * t * t)
                    local x, y = math.floor(dx * f), math.floor(dy * f)
                    if x ~= sx or y ~= sy then ragdoll.injectMouseInput(x - sx, y - sy) end
                    sx, sy = x, y
                    if each then each(s) end
                    if n >= steps then return s end
                end
            end
            local function hold(seconds, each)
                local first = sample()
                local previous = first.step
                local deadline = first.step + math.floor(seconds * 60)
                while true do
                    local s = nextStep(previous)
                    previous = s.step
                    if each then each(s) end
                    if s.step >= deadline then return s end
                end
            end
            local function stance(s)
                return s.hsc and math.sqrt(s.hsc.stanceX ^ 2 + s.hsc.stanceY ^ 2) or 0
            end
            waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            waitSeconds(0.8)
            if kind == 'stance' then
                -- Mouse y counts are screen-down positive: up is negative.
                press()
                local cut = false
                local watch = function(s) if s.hsc and s.hsc.stage == 2 then cut = true end end
                drag(0, -220, 0.35, watch)
                local s = hold(0.2, watch)
                check('up stance inside the circle', s.hsc.stage == 1 and stance(s) < 300 and stance(s) > 150)
                drag(220, 220, 0.35, watch)
                drag(-440, 0, 0.5, watch)
                drag(220, 220, 0.35, watch)
                s = hold(0.2, watch)
                check('slow guard changes never cut', not cut)
                print(string.format('%s low stance x=%.0f y=%.0f stage=%d', tag, s.hsc.stanceX, s.hsc.stanceY, s.hsc.stage))
                release()
                s = hold(1.5)
                check('a released stance stays', s.hsc.stage == 1 and stance(s) > 150)
                press()
                drag(0, -(HSC_WIND + 400), 0.5, watch)
                s = hold(0.2)
                local deep = stance(s)
                print(string.format('%s committed stance x=%.0f y=%.0f', tag, s.hsc.stanceX, s.hsc.stanceY))
                check('past the radius commits a wind-up', s.hsc.stage == 1 and deep > 450)
                local angle0 = s.hsc.angle
                drag(150, 0, 0.4)
                s = hold(0.1)
                print(string.format('%s retract radius %.0f -> %.0f angle %.2f -> %.2f', tag, deep, stance(s), angle0, s.hsc.angle))
                check('sideways from deep retracts first', stance(s) < deep - 100 and math.abs(s.hsc.angle - angle0) < 0.05)
                drag(0, -150, 0.3)
                cut = false
                drag(0, 1200, 0.3, watch)
                check('a fast drag back cuts', cut)
                release()
                hold(2.0)
                press()
                drag(-250, 0, 0.35)
                s = hold(0.3)
                local snap = false
                drag(600, 0, 0.12, function(x) if x.hsc and x.hsc.stage == 2 and x.hsc.snap then snap = true end end)
                hold(0.4)
                check('a quick flick across the circle snaps', snap)
                release()
                hold(1.5)
            elseif kind == 'onset' then
                -- Steady drags from rest, 50 counts a step (3000 counts/s).
                local worst = 0
                local function onset(name, dx, dy, steps, rest)
                    local base = hold(rest or 0.7)
                    local jitter = 0
                    hold(0.15, function(s)
                        jitter = math.max(jitter, (s.tip - base.tip):length())
                        if DEBUG_ONSET then
                            print(string.format('%s debug %s step=%d s=%.2f stage=%d stance=%.0f,%.0f tip=%.1f,%.1f,%.1f owner=%s', tag, name, s.step,
                                s.hsc and s.hsc.s or -1, s.hsc and s.hsc.stage or -1, s.hsc and s.hsc.stanceX or 0, s.hsc and s.hsc.stanceY or 0,
                                s.tip.x, s.tip.y, s.tip.z, tostring(ragdoll.getState(player).armOwnerRight)))
                        end
                    end)
                    base = sample()
                    local threshold = math.max(1.0, 2 * jitter)
                    local first, moved
                    local previous = base.step
                    for i = 1, steps + 20 do
                        if i <= steps then ragdoll.injectMouseInput(dx, dy) end
                        local s = nextStep(previous)
                        previous = s.step
                        first = first or s.step
                        if not moved and (s.tip - base.tip):length() > threshold then moved = s.step end
                        if moved and i >= steps then break end
                    end
                    local ms = moved and (moved - first + 1) * 1000 / 60 or 999
                    worst = math.max(worst, ms)
                    local after = sample()
                    print(string.format('%s stroke %-14s onset_ms=%.0f threshold=%.1f jitter=%.2f s=%.3f->%.3f stance=%.0f,%.0f->%.0f,%.0f stage=%d moved=%.1f', tag, name, ms, threshold, jitter,
                        base.hsc and base.hsc.s or -1, after.hsc and after.hsc.s or -1, base.hsc and base.hsc.stanceX or 0, base.hsc and base.hsc.stanceY or 0,
                        after.hsc and after.hsc.stanceX or 0, after.hsc and after.hsc.stanceY or 0, after.hsc and after.hsc.stage or -1, (after.tip - base.tip):length()))
                end
                press()
                onset('stance-up', 0, -30, 8)
                onset('stance-right', 25, 25, 10)
                onset('stance-left', -30, 0, 16)
                onset('stance-centre', 25, 0, 5)
                onset('wind-up', 0, -60, 25)
                onset('cut', 0, 80, 15)
                release()
                hold(2.4)
                press()
                onset('flick', 0, -40, 5)
                onset('snap', 0, 100, 8)
                release()
                hold(1.5)
                print(string.format('%s total onset_max_ms=%.0f', tag, worst))
            elseif kind == 'chop' then
                local lateral, angle, miss, cmdBlade, cmdGrip, hand = {}, {}, {}, {}, {}, {}
                for i = 1, 3 do
                    press()
                    drag(0, -HSC_WIND, 0.45)
                    hold(0.25)
                    drag(0, HSC_CUT, 0.35, function(s)
                        -- Where the blade passes the front (pointing forward within
                        -- about 30 deg of level), whatever the route's timing.
                        local _, fwd = frame()
                        if s.hsc and s.hsc.stage == 2 and s.blade:dot(fwd) > 0.85 then
                            local right = frame()
                            local function deg(v) return math.abs(math.deg(math.asin(math.max(-1, math.min(1, v))))) end
                            lateral[#lateral + 1] = math.abs(s.pelvisTip:dot(right))
                            angle[#angle + 1] = deg(s.blade:dot(right))
                            miss[#miss + 1] = s.hsc.planeMiss
                            cmdBlade[#cmdBlade + 1] = deg(s.hsc.blade:normalize():dot(right))
                            cmdGrip[#cmdGrip + 1] = (s.hsc.grip - s.pelvis):dot(right)
                            hand[#hand + 1] = s.hand:dot(right)
                        end
                    end)
                    hold(0.3)
                    release()
                    hold(1.6)
                end
                local function q(t, f)
                    if #t == 0 then return -1 end
                    table.sort(t)
                    return t[math.max(1, math.floor(f * #t))]
                end
                print(string.format('%s samples=%d tip_lateral_p50=%.1f tip_lateral_p90=%.1f blade_lateral_p50=%.1f blade_lateral_p90=%.1f command_plane_miss_p90=%.2f',
                    tag, #lateral, q(lateral, 0.5), q(lateral, 0.9), q(angle, 0.5), q(angle, 0.9), q(miss, 0.9)))
                print(string.format('%s command_blade_lateral_p90=%.1f command_grip_x_p50=%.1f hand_x_p50=%.1f (from the pelvis, right +)',
                    tag, q(cmdBlade, 0.9), q(cmdGrip, 0.5), q(hand, 0.5)))
            elseif kind == 'chain' then
                -- One held stroke: a forehand cut that carries on left past its
                -- follow-through into the backhand's wind-up, then the backhand
                -- cut back right. carried = the blade's slowest speed from the
                -- forehand's peak to the turn of the mouse, against that peak.
                local speeds, prevBlade = {}, nil
                local function track(s)
                    if prevBlade then
                        speeds[#speeds + 1] = math.deg(math.acos(math.max(-1, math.min(1, prevBlade:dot(s.blade)))))
                    end
                    prevBlade = s.blade
                end
                press()
                drag(HSC_WIND, 0, 0.35)
                hold(0.15)
                local stages = {}
                drag(-(HSC_CUT + HSC_RUN_ON + 600), 0, 0.6, function(s)
                    track(s)
                    if s.hsc then stages[#stages + 1] = s.hsc.stage end
                end)
                local turn = #speeds
                drag(HSC_CUT, 0, 0.3, track)
                release()
                hold(1.5)
                local peak1, at1, peak2, trough = 0, 1, 0, 1e9
                for i = 1, turn do if speeds[i] > peak1 then peak1, at1 = speeds[i], i end end
                for i = turn + 1, #speeds do peak2 = math.max(peak2, speeds[i]) end
                for i = at1, math.max(at1, turn - 6) do trough = math.min(trough, speeds[i]) end
                local chained = false
                for i = 2, #stages do if stages[i - 1] == 2 and stages[i] == 1 then chained = true end end
                local t = {}
                for i = 1, #speeds do t[#t + 1] = string.format('%.0f%s', speeds[i], i == turn and '|' or '') end
                print(tag .. ' profile ' .. table.concat(t, ' '))
                print(tag .. ' stages ' .. table.concat(stages, ''))
                print(string.format('%s peak1=%.1f peak2=%.1f trough=%.2f carried=%.2f chained=%s', tag, peak1, peak2, trough,
                    trough / math.max(1e-3, peak1), tostring(chained)))
                check('the stroke chains into the next wind-up', chained)
            elseif kind == 'press-chain' then
                -- CHAIN-02: a forehand cut, pressed again as it is still moving
                -- (the button released and pressed in the cut's last fifth), the
                -- drag going on left into the backhand's wind-up, then the
                -- backhand cut. carried = the blade's slowest speed between the
                -- forehand's peak and the turn of the mouse, against that peak.
                local speeds, prevBlade = {}, nil
                local cmd, prevCmd, stages = {}, nil, {}
                local function track(s)
                    if prevBlade then
                        speeds[#speeds + 1] = math.deg(math.acos(math.max(-1, math.min(1, prevBlade:dot(s.blade)))))
                        local c = s.hsc and s.hsc.blade and s.hsc.blade:normalize()
                        cmd[#cmd + 1] = (c and prevCmd) and math.deg(math.acos(math.max(-1, math.min(1, prevCmd:dot(c))))) or 0
                        stages[#stages + 1] = s.hsc and s.hsc.stage or -1
                    end
                    prevBlade = s.blade
                    prevCmd = s.hsc and s.hsc.blade and s.hsc.blade:normalize() or prevCmd
                end
                press()
                drag(HSC_WIND, 0, 0.35)
                hold(0.15)
                -- One continuous minimum-jerk stroke left (the cut and on into
                -- the backhand's wind-up), the button released and pressed again
                -- without the mouse stopping once 80% of the cut is dragged.
                do
                    local total, seconds = HSC_CUT + HSC_WIND, 0.5
                    local steps = math.floor(seconds * 60)
                    local first = sample()
                    local step0, previous, sx, repressed = first.step, first.step, 0, false
                    while true do
                        local st = nextStep(previous)
                        previous = st.step
                        local n = st.step - step0
                        local t = math.min(1, n / steps)
                        local f = t * t * t * (10 - 15 * t + 6 * t * t)
                        local x = math.floor(-total * f)
                        if x ~= sx then ragdoll.injectMouseInput(x - sx, 0) end
                        sx = x
                        track(st)
                        if not repressed and -x >= HSC_CUT * 0.8 then
                            ragdoll.injectMouseInput(0, 0, false, 1)
                            ragdoll.injectMouseInput(0, 0, true, 1)
                            repressed = true
                        end
                        if n >= steps then break end
                    end
                end
                local turn = #speeds
                drag(HSC_CUT, 0, 0.3, track)
                release()
                hold(1.5)
                local peak1, at1, trough = 0, 1, 1e9
                for i = 1, turn do if speeds[i] > peak1 then peak1, at1 = speeds[i], i end end
                for i = at1, math.max(at1, turn - 6) do trough = math.min(trough, speeds[i]) end
                local t = {}
                for i = 1, #speeds do t[#t + 1] = string.format('%.0f%s', speeds[i], i == turn and '|' or '') end
                print(tag .. ' profile ' .. table.concat(t, ' '))
                local u = {}
                for k = 1, #cmd do u[#u + 1] = string.format('%.0f', cmd[k]) end
                print(tag .. ' command ' .. table.concat(u, ' '))
                print(tag .. ' stages ' .. table.concat(stages, ''))
                print(string.format('%s peak1=%.1f trough=%.2f carried=%.2f', tag, peak1, trough, trough / math.max(1e-3, peak1)))
            elseif kind:match('^replay') then
                -- REPLAY-01: the user's own input before a bookmark
                -- (tools/body/replay_cases.py), step by step. Counts the
                -- steps in the guard with the blade more than 45 deg off its
                -- command (stuck) and in an attack more than 90 off (spun),
                -- here and in the recording.
                local case = require('replay_inputs')[tonumber(kind:match('%d+'))]
                local function off(s)
                    local c = s.hsc and s.hsc.blade
                    if not c or c:length() < 1e-3 then return -1 end
                    return math.deg(math.acos(math.max(-1, math.min(1, s.blade:dot(c:normalize())))))
                end
                -- REPLAY-02: the same, exhausted (fatigue 0 throughout)
                local tired = kind:match('tired') ~= nil
                local leftHand = kind:match('left') ~= nil
                if kind:match('hubdraw') then
                    -- REPLAY-08: the play hub's own weapon preparation
                    -- (RagdollDev.playerFists: unpowered, equipped and drawn,
                    -- powered again a moment later), from the dagger, as in play
                    local dev = require('openmw.interfaces').RagdollDev
                    dev.playerFists('iron dagger')
                    waitSeconds(3)
                    dev.playerFists(case.weapon)
                    waitSeconds(0.5)
                    waitFor('hub draw powered', function()
                        local st = ragdoll.getState(player)
                        return st and st.powered and st.controller and st.controller.mode == 'standing'
                    end, 12)
                    waitSeconds(1.5)
                    waitFor('guard after the hub draw', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                    ragdoll.prepareMouseInput()
                end
                if kind:match('sheath') then
                    -- REPLAY-07: weapons switched while sheathed (the body rebuilt
                    -- with an empty relaxed hand), then drawn, as in play
                    if not types.Actor.inventory(player):find('iron dagger') then
                        world.createObject('iron dagger', 1):moveInto(types.Actor.inventory(player))
                        coroutine.yield()
                    end
                    for _, w in ipairs({ 'iron dagger', case.weapon }) do
                        local seq = 1300 + sequence * 2 + (w == case.weapon and 1 or 0)
                        player:sendEvent('fixPlayerConfigure', { weapon = w, sequence = seq, firstPerson = false,
                            sheathed = true })
                        waitFor('sheathed loadout', function() return playerReadySequence == seq end)
                        waitSeconds(2.5)
                    end
                    player:sendEvent('fixPlayerDraw')
                    waitSeconds(2.5)
                    waitFor('guard after the draw', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                    waitSeconds(1)
                end
                if kind:match('swap') then
                    -- REPLAY-05: as play began, the dagger drawn first, then this weapon
                    if not types.Actor.inventory(player):find('iron dagger') then
                        world.createObject('iron dagger', 1):moveInto(types.Actor.inventory(player))
                        coroutine.yield()
                    end
                    for _, w in ipairs({ 'iron dagger', case.weapon }) do
                        local seq = 900 + sequence * 2 + (w == case.weapon and 1 or 0)
                        player:sendEvent('fixPlayerConfigure', { weapon = w, sequence = seq, firstPerson = false })
                        waitFor('swap loadout', function() return playerReadySequence == seq end)
                        waitSeconds(2.5)
                    end
                    waitFor('guard after swap', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
                    waitSeconds(1)
                end
                local held, previous = false, sample().step
                do
                    local st0 = ragdoll.getState(player)
                    print(string.format('%s start strikeHand=%s armOwnerLeft=%s armOwnerRight=%s', tag,
                        tostring(st0.strikeHand), tostring(st0.armOwnerLeft), tostring(st0.armOwnerRight)))
                end
                local tally = armTally(kind:gsub('%d+', '') .. ' ' .. weapon .. ' ' .. case.name)
                local mine = { stuck = 0, spun = 0, wrist = 0, moved = 0, turned = 0 }
                local lastPos, lastYaw = player.position, player.rotation:getYaw()
                local theirs = { stuck = 0, spun = 0 }
                local line = {}
                for i, st in ipairs(case.steps) do
                    local dx, dy, button = st[1], st[2], st[3] == 1
                    -- moving: the recorded keys, turn and look pitch (REPLAY-01 since
                    -- 2026-10-05; the mouse up is then the turn, not injected)
                    local moving = st[7] ~= nil and not kind:match('still')
                    if moving then
                        player:sendEvent('fixPlayerControls', { side = st[7], forward = st[8], run = st[9],
                            yaw = st[10], pitch = st[11] })
                        if not button and not held then dx, dy = 0, 0 end
                    end
                    if tired and i % 10 == 1 then player:sendEvent('fixSetFatigue', 0) end
                    if leftHand then
                        -- REPLAY-04: the free left hand where play's third-person
                        -- guard held it (chest frame: 3 left, 31 ahead, 8 up)
                        local right, forward = frame()
                        local chest = body(ragdoll.getState(player), 'spine2').position
                        ragdoll.setArmTarget(player, false, chest - right * 3 + forward * 31 + util.vector3(0, 0, 8), 1, 3)
                    end
                    if button ~= held then
                        ragdoll.injectMouseInput(dx, dy, button, 1)
                        held = button
                    elseif dx ~= 0 or dy ~= 0 then
                        ragdoll.injectMouseInput(dx, dy)
                    end
                    local s = nextStep(previous)
                    previous = s.step
                    -- the hand's whole rotation from its command (blade and roll):
                    -- a hand rolled half a turn about the blade keeps the blade's
                    -- direction ("the wrist is backwards")
                    tally.add(s.hsc)
                    local o, stage = s.hsc and s.hsc.wristError or -1, s.hsc and s.hsc.stage or -1
                    -- counted as the recording's side is: button up / held
                    if not held and o > 45 then mine.stuck = mine.stuck + 1 end
                    if held and o > 90 then mine.spun = mine.spun + 1 end
                    mine.wrist = math.max(mine.wrist, ragdoll.getState(player).wristOutsideRight or 0)
                    mine.moved = mine.moved + (player.position - lastPos):length()
                    mine.turned = mine.turned + math.abs(player.rotation:getYaw() - lastYaw)
                    lastPos, lastYaw = player.position, player.rotation:getYaw()
                    local recorded = st[6] or st[4]
                    if recorded > 45 and not held then theirs.stuck = theirs.stuck + 1 end
                    if recorded > 90 and held then theirs.spun = theirs.spun + 1 end
                    if i % 30 == 0 or i == case.bookmarkStep then
                        line[#line + 1] = string.format('%d:%s%.0f/%d', i, i == case.bookmarkStep and 'B' or '', o, recorded)
                    end
                end
                ragdoll.injectMouseInput(0, 0, false, 1)
                player:sendEvent('fixPlayerControls', { side = 0, forward = 0, run = 0, stop = true })
                print(string.format('%s %s (%s): %s', tag, case.name, case.note, table.concat(line, ' ')))
                tally.report()
                print(string.format('%s %s summary stuck %d (recorded %d) spun %d (recorded %d) worstWrist %.0f moved %.0f turned %.1f',
                    tag, case.name, mine.stuck, theirs.stuck, mine.spun, theirs.spun, mine.wrist, mine.moved, mine.turned))
                check('no stuck guard (blade over 45 deg off for 0.5 s)', mine.stuck < 30)
            elseif kind == 'move' or kind == 'move-still' then
                -- MOVE-01 (goal 2026-10-05): guards, wind-ups and cuts while
                -- walking, strafing, backing off, running and turning (the
                -- user fought moving 79% of the time; no test moved).
                local tally = armTally((kind:match('still') and 'MOVE-02 ' or 'MOVE-01 ') .. weapon)
                local phase = nil
                local forward, side, run, turn = 0, 0, 0, 0
                local function each(s)
                    tally.add(s.hsc)
                    if phase then phase.add(s.hsc) end
                    if turn ~= 0 then player:sendEvent('fixPlayerControls', { side = side, forward = forward,
                        run = run, yaw = turn }) end
                end
                local function moveAs(f, sd, r, tr)
                    forward, side, run, turn = f, sd, r, tr
                    player:sendEvent('fixPlayerControls', { side = sd, forward = f, run = r })
                end
                local function stroke(wx, wy, cx, cy)
                    press()
                    drag(wx, wy, 0.4, each)
                    hold(0.15, each)
                    drag(cx, cy, 0.3, each)
                    release()
                    hold(1.5, each)
                end
                local modes = { { 1, 0, 0, 0 }, { -1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, -1, 0, 0 },
                    { 1, 0, 1, 0 }, { -1, 0, 1, 0 }, { 1, 0, 0, 0.02 }, { 0, 1, 0, -0.02 } }
                -- MOVE-02: the same strokes standing still (what moving adds)
                if kind:match('still') then modes = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } end
                for _, m in ipairs(modes) do
                    if phase then phase.report() end
                    phase = armTally(string.format('PHASE %s f%d s%d r%d t%.2f', weapon, m[1], m[2], m[3], m[4]))
                    moveAs(m[1], m[2], m[3], m[4])
                    hold(0.6, each)
                    stroke(HSC_WIND, 0, -HSC_CUT, 0) -- right, cut left
                    stroke(0, -HSC_WIND, 0, HSC_CUT) -- up, chop
                    stroke(-HSC_WIND, 0, HSC_CUT, 0) -- left, cut right
                    -- side to side (the user: "I can't swing side to side")
                    press()
                    drag(1200, 0, 0.2, each)
                    drag(-3000, 0, 0.3, each)
                    drag(3000, 0, 0.3, each)
                    release()
                    hold(0.8, each)
                end
                if phase then phase.report() end
                phase = nil
                moveAs(0, 0, 0, 0)
                player:sendEvent('fixPlayerControls', { side = 0, forward = 0, run = 0, stop = true })
                hold(1.0, each)
                tally.report()
            elseif kind == 'trap-kick' then
                -- TRAP-03: the arm knocked into a wrong configuration in the
                -- guard (the tip kicked down, back, left, right), then left
                -- alone. stuck = steps with the blade more than 45 deg off its
                -- command 1-3 s after the kick (the guard should be back).
                local function off(s)
                    local c = s.hsc and s.hsc.blade
                    if not c or c:length() < 1e-3 then return 0 end
                    return math.deg(math.acos(math.max(-1, math.min(1, s.blade:dot(c:normalize())))))
                end
                local right, forward = frame()
                local up = util.vector3(0, 0, 1)
                local stuckCases = 0
                for _, kick in ipairs({ { 'down', -up, 1 }, { 'down', -up, 2 }, { 'back', -forward, 1 },
                    { 'back+down', (-forward - up):normalize(), 2 }, { 'left', -right, 1 }, { 'left', -right, 2 },
                    { 'right+down', (right - up):normalize(), 2 }, { 'back+left', (-forward - right):normalize(), 2 } }) do
                    hold(1.0)
                    local st = ragdoll.getState(player)
                    local w = st.bodies[st.weapon.bodyIndex]
                    local blade = w.rotation * util.vector3(0, 0, 1)
                    local tip = w.position + blade * (st.weapon.length * 0.5)
                    ragdoll.applyImpulse(player, st.weapon.bodyIndex, kick[2] * (400 * kick[3]), tip)
                    local worst, stuck, at1, at3 = 0, 0, 0, 0
                    local n = 0
                    hold(3.0, function(s)
                        n = n + 1
                        local o = off(s)
                        worst = math.max(worst, o)
                        if n > 60 and o > 45 then stuck = stuck + 1 end
                        if n == 60 then at1 = o end
                        at3 = o
                    end)
                    local se = sample()
                    print(string.format('%s kick %-10s x%d worst=%.0f at1s=%.0f at3s=%.0f stuck=%d handError=%.1f', tag,
                        kick[1], kick[3], worst, at1, at3, stuck, se.hsc and se.hsc.handError or -1))
                    if stuck > 30 then stuckCases = stuckCases + 1 end
                end
                check('the guard comes back after every kick', stuckCases == 0)
            elseif kind == 'trap-stop' or kind == 'trap-wind' then
                -- TRAP-01/02 (user bookmarks 2026-10-04 build 47: "in guard but
                -- the wrist is backwards, sword pointing down", "I can't get it
                -- to normal"; claymore "wind up slowly to the right ... 1 spin").
                -- off = the physical blade's angle from the commanded one.
                local function off(s)
                    local c = s.hsc and s.hsc.blade
                    if not c or c:length() < 1e-3 then return 0 end
                    return math.deg(math.acos(math.max(-1, math.min(1, s.blade:dot(c:normalize())))))
                end
                local worst = { off = 0, wrist = 0 }
                local function watch(s)
                    worst.off = math.max(worst.off, off(s))
                    local w = ragdoll.getState(player).wristOutsideRight or 0
                    worst.wrist = math.max(worst.wrist, w)
                end
                local function settle(label)
                    -- the guard 1.5 s after the release: off and hand error
                    local s = hold(1.5, watch)
                    local stuck = 0
                    hold(1.0, function(x) watch(x); if off(x) > 45 then stuck = stuck + 1 end end)
                    s = sample()
                    print(string.format('%s %s guard off=%.0f handError=%.1f stuckSteps=%d worstOff=%.0f worstWrist=%.0f stage=%d',
                        tag, label, off(s), s.hsc and s.hsc.handError or -1, stuck, worst.off, worst.wrist,
                        s.hsc and s.hsc.stage or -1))
                    return off(s), stuck
                end
                local bad = 0
                if kind == 'trap-stop' then
                    for _, case in ipairs({ { 0.3, 0.25 }, { 0.45, 0.2 }, { 0.6, 0.15 }, { 0.45, 0.35 } }) do
                        worst = { off = 0, wrist = 0 }
                        press()
                        drag(HSC_WIND, 0, 0.4, watch)
                        hold(0.15, watch)
                        drag(-math.floor(HSC_CUT * case[1]), 0, case[2], watch)
                        hold(0.1, watch)
                        release()
                        local o, stuck = settle(string.format('stop %.2f in %.2fs', case[1], case[2]))
                        if o > 30 or stuck > 20 then bad = bad + 1 end
                    end
                else
                    for _, case in ipairs({ { 1, 0, 1.5 }, { 0.7, 0.7, 1.5 }, { 1, 0, 3.0 }, { 0.7, -0.7, 1.5 } }) do
                        worst = { off = 0, wrist = 0 }
                        press()
                        drag(math.floor(HSC_WIND * case[1]), -math.floor(HSC_WIND * case[2]), case[3], watch)
                        hold(0.5, watch)
                        print(string.format('%s wind %.1f,%.1f in %.1fs worstOff=%.0f worstWrist=%.0f', tag,
                            case[1], case[2], case[3], worst.off, worst.wrist))
                        if worst.off > 45 then bad = bad + 1 end
                        release()
                        local o, stuck = settle('after wind')
                        if o > 30 or stuck > 20 then bad = bad + 1 end
                    end
                end
                check('no stuck guard or blade more than 45 deg off', bad == 0)
            elseif kind == 'hud' then
                -- HUD-02 (run with --hub-player): the stance overlay fed as
                -- manual_hub.lua feeds it, through a wind-up right and a cut;
                -- screenshots in the wind-up, the cut and the follow-through.
                local shots = {}
                local function publish(s)
                    local st = ragdoll.getState(player)
                    local h = st.hsc
                    local w = st.bodies[st.weapon.bodyIndex]
                    player:sendEvent('manualHubAim', { tip = s.tip + s.chest, hud = h and {
                        stage = h.stage, x = h.stanceX, y = h.stanceY, angle = h.angle, line = h.line,
                        snap = h.snap, chain = h.chain, click = h.click, route = h.route,
                        bladeCommand = h.blade, blade = w.rotation * util.vector3(0, 0, 1) } or nil,
                        wristRight = st.wristOutsideRight, wristLeft = st.wristOutsideLeft })
                    local stage = h and h.stage or -1
                    if not shots[stage] and stage >= 1 and (stage ~= 1 or (h.route or 0) > 0.6) then
                        shots[stage] = true
                        ragdoll.captureScreenshot()
                        print(string.format('%s shot stage=%d x=%.0f y=%.0f route=%.2f line=%s', tag, stage,
                            h.stanceX, h.stanceY, h.route or -1, tostring(h.line)))
                    end
                end
                hold(0.5, publish)
                press()
                drag(HSC_WIND, 0, 0.5, publish)
                hold(0.4, publish)
                drag(-HSC_CUT, 0, 0.35, publish)
                release()
                hold(1.2, publish)
                check('overlay fed in the wind-up, cut and recovery', shots[1] and shots[2] and shots[3])
            end
            if #failures > 0 then error(tag .. ' failed: ' .. table.concat(failures, ', ')) end
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end
end
registerTest('STANCE-02 stance circle iron longsword', 918780, controlTest('STANCE-02 longsword', 'iron longsword', 0, 'stance'))
registerTest('STANCE-02 stance circle iron claymore', 918781, controlTest('STANCE-02 claymore', 'iron claymore', 1, 'stance'))
registerTest('ONSET-01 iron longsword', 918782, controlTest('ONSET-01 longsword', 'iron longsword', 2, 'onset'))
registerTest('ONSET-01 iron claymore', 918783, controlTest('ONSET-01 claymore', 'iron claymore', 3, 'onset'))
registerTest('CHOP-01 iron longsword', 918784, controlTest('CHOP-01 longsword', 'iron longsword', 4, 'chop'))
registerTest('CHOP-01 iron claymore', 918785, controlTest('CHOP-01 claymore', 'iron claymore', 5, 'chop'))
registerTest('CHAIN-01 iron longsword', 918786, controlTest('CHAIN-01 longsword', 'iron longsword', 6, 'chain'))
registerTest('CHAIN-01 iron claymore', 918787, controlTest('CHAIN-01 claymore', 'iron claymore', 7, 'chain'))
registerTest('CHAIN-02 press chain iron longsword', 918788, controlTest('CHAIN-02 longsword', 'iron longsword', 8, 'press-chain'))
registerTest('CHAIN-02 press chain iron claymore', 918789, controlTest('CHAIN-02 claymore', 'iron claymore', 9, 'press-chain'))
registerTest('HUD-02 stance overlay iron longsword', 918790, controlTest('HUD-02 longsword', 'iron longsword', 10, 'hud'))
registerTest('TRAP-01 stopped cut iron longsword', 918791, controlTest('TRAP-01 longsword', 'iron longsword', 11, 'trap-stop'))
registerTest('TRAP-01 stopped cut iron claymore', 918792, controlTest('TRAP-01 claymore', 'iron claymore', 12, 'trap-stop'))
registerTest('TRAP-02 slow wind-up iron claymore', 918793, controlTest('TRAP-02 claymore', 'iron claymore', 13, 'trap-wind'))
registerTest('TRAP-02 slow wind-up iron longsword', 918794, controlTest('TRAP-02 longsword', 'iron longsword', 14, 'trap-wind'))
registerTest('MOVE-01 moving swings iron longsword', 918797, controlTest('MOVE-01 longsword', 'iron longsword', 17, 'move'))
registerTest('MOVE-01 moving swings iron claymore', 918798, controlTest('MOVE-01 claymore', 'iron claymore', 18, 'move'))
registerTest('MOVE-02 standing swings iron longsword', 918799, controlTest('MOVE-02 longsword', 'iron longsword', 19, 'move-still'))
registerTest('MOVE-02 standing swings iron claymore', 918800, controlTest('MOVE-02 claymore', 'iron claymore', 20, 'move-still'))
registerTest('TRAP-03 kicked guard iron longsword', 918795, controlTest('TRAP-03 longsword', 'iron longsword', 15, 'trap-kick'))
registerTest('TRAP-03 kicked guard iron claymore', 918796, controlTest('TRAP-03 claymore', 'iron claymore', 16, 'trap-kick'))
do
    local ok, replays = pcall(require, 'replay_inputs')
    for index, case in ipairs(ok and replays or {}) do
        registerTest('REPLAY-01 ' .. case.name .. ' ' .. case.weapon, 918800 + index,
            controlTest('REPLAY-01', case.weapon, 20 + index, 'replay' .. index))
        registerTest('REPLAY-08 hub draw ' .. case.name .. ' ' .. case.weapon, 918970 + index,
            controlTest('REPLAY-08', case.weapon, 300 + index, 'replay' .. index .. 'hubdraw'))
        registerTest('REPLAY-07 switched sheathed ' .. case.name .. ' ' .. case.weapon, 918950 + index,
            controlTest('REPLAY-07', case.weapon, 260 + index, 'replay' .. index .. 'sheath'))
        registerTest('REPLAY-06 mouse only ' .. case.name .. ' ' .. case.weapon, 918930 + index,
            controlTest('REPLAY-06', case.weapon, 220 + index, 'replay' .. index .. 'still'))
        registerTest('REPLAY-02 tired ' .. case.name .. ' ' .. case.weapon, 918850 + index,
            controlTest('REPLAY-02', case.weapon, 60 + index, 'replay' .. index .. 'tired'))
        registerTest('REPLAY-03 first person ' .. case.name .. ' ' .. case.weapon, 918870 + index,
            controlTest('REPLAY-03', case.weapon, 100 + index, 'replay' .. index .. 'fp'))
        registerTest('REPLAY-04 left hand up ' .. case.name .. ' ' .. case.weapon, 918890 + index,
            controlTest('REPLAY-04', case.weapon, 140 + index, 'replay' .. index .. 'left'))
        registerTest('REPLAY-05 after a weapon swap ' .. case.name .. ' ' .. case.weapon, 918910 + index,
            controlTest('REPLAY-05', case.weapon, 180 + index, 'replay' .. index .. 'swap'))
    end
end

-- GRIP-01 (roadmap 2026-10-04, item 5): the two-handed grip holds through hand
-- control's hardest use - fast cuts, a snap cut, chained attacks, stance flicks,
-- a block - and re-grips quickly after a draw. Bars: the support grip is never
-- lost while the weapon is out; the draw's re-grip time is printed (target
-- about 0.2 s after hand control takes the arm).
local function gripTest(tag, weapon, sequence)
    return function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 840 + sequence, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 840 + sequence end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local prevStep, prevAttached, lostAt
            local losses, regrips, attachedSteps, steps = 0, {}, 0, 0
            local separation, leftOut = {}, {}
            local watching = true
            local function sample(label)
                local s = ragdoll.getState(player)
                if not s or not s.weapon then return end
                local step = tonumber(s.physicsStep)
                if prevStep and step == prevStep then return end
                prevStep = step
                local g = s.weapon.supportGrip
                local attached = (g and g.attached) and true or false
                if watching then
                    steps = steps + 1
                    if attached then attachedSteps = attachedSteps + 1 end
                    if prevAttached and not attached then
                        losses = losses + 1
                        lostAt = step
                        print(string.format('%s grip lost during %s (%s)', tag, label, tostring(g and g.reason)))
                    end
                    if attached and g.anchorSeparation then separation[#separation + 1] = g.anchorSeparation end
                    leftOut[#leftOut + 1] = math.max(0, s.wristOutsideLeft or 0)
                end
                if lostAt and attached and not prevAttached then
                    regrips[#regrips + 1] = (step - lostAt) / 60
                    lostAt = nil
                end
                prevAttached = attached
            end
            local function hold(label, seconds) waitSeconds(seconds, function() sample(label) end) end
            local function drag(label, dx, dy, seconds)
                local n0 = tonumber(ragdoll.getState(player).physicsStep)
                local steps = math.max(1, math.floor(seconds * 60))
                local sx, sy = 0, 0
                waitFor(label, function()
                    local n = tonumber(ragdoll.getState(player).physicsStep) - n0
                    local t = math.min(1, n / steps)
                    local f = t * t * t * (10 - 15 * t + 6 * t * t)
                    local x, y = math.floor(dx * f), math.floor(dy * f)
                    if x ~= sx or y ~= sy then ragdoll.injectMouseInput(x - sx, y - sy) end
                    sx, sy = x, y
                    sample(label)
                    return n >= steps
                end, seconds + 6)
            end
            local function press() ragdoll.injectMouseInput(0, 0, true, 1) end
            local function release() ragdoll.injectMouseInput(0, 0, false, 1) end
            hold('guard', 1.5)
            local d = math.sqrt(0.5)
            -- Fast cuts in four directions.
            for _, a in ipairs({ { 'chop', 0, -1 }, { 'forehand', 1, 0 }, { 'backhand', -1, 0 }, { 'up-left', -d, -d } }) do
                press()
                drag(a[1] .. '-wind', HSC_WIND * a[2], HSC_WIND * a[3], 0.3)
                drag(a[1] .. '-cut', -HSC_CUT * a[2], -HSC_CUT * a[3], 0.22)
                hold(a[1] .. '-follow', 0.2)
                release()
                hold(a[1] .. '-recover', 1.6)
            end
            -- A chain: forehand, then backhand pressed in the follow-through.
            press()
            drag('chain-wind', HSC_WIND, 0, 0.3)
            drag('chain-cut', -HSC_CUT, 0, 0.22)
            release()
            hold('chain-gap', 0.05)
            press()
            drag('chain2-wind', -HSC_WIND, 0, 0.3)
            drag('chain2-cut', HSC_CUT, 0, 0.22)
            release()
            hold('chain-recover', 1.6)
            -- Stance flicks and a snap cut.
            press()
            drag('stance-left', -250, 0, 0.3)
            drag('stance-up', 250, -250, 0.3)
            drag('stance-right', 250, 250, 0.3)
            drag('snap', -600, 0, 0.12)
            hold('snap-follow', 0.3)
            release()
            hold('snap-recover', 1.6)
            -- A block in an attack.
            press()
            drag('block-wind', 0, -HSC_WIND, 0.3)
            ragdoll.setGuard(player, true)
            hold('block', 0.5)
            ragdoll.setGuard(player, false)
            release()
            hold('block-recover', 1.5)
            watching = false
            -- Re-grip after a draw.
            player:sendEvent('fixPlayerStance', false)
            hold('sheathe', 2.0)
            local drawStep = tonumber(ragdoll.getState(player).physicsStep)
            player:sendEvent('fixPlayerStance', true)
            local hscStep, gripStep, lastGripKey
            waitSeconds(4, function()
                local s = ragdoll.getState(player)
                local step = tonumber(s.physicsStep)
                if not hscStep and s.hsc then hscStep = step end
                local g = s.weapon and s.weapon.supportGrip
                if hscStep and not gripStep and g and g.attached then gripStep = step end
                local key = g and (tostring(g.state) .. '/' .. tostring(g.reason)) or 'none'
                if key ~= lastGripKey and step - drawStep < 120 then
                    print(string.format('%s draw grip t=%.2f %s owner=%s hsc=%s', tag, (step - drawStep) / 60, key,
                        tostring(s.armOwnerLeft), tostring(s.hsc ~= nil)))
                    lastGripKey = key
                end
            end)
            table.sort(separation)
            table.sort(leftOut)
            local function q(t, f) return #t > 0 and t[math.max(1, math.floor(f * #t))] or -1 end
            print(string.format('%s total losses=%d attached=%.3f separation_p99=%.2f left_wrist_out_p90=%.0f regrip_max_s=%.2f draw_hsc_s=%.2f draw_regrip_s=%.2f',
                tag, losses, steps > 0 and attachedSteps / steps or -1, q(separation, 0.99), q(leftOut, 0.9),
                #regrips > 0 and math.max(table.unpack(regrips)) or 0,
                hscStep and (hscStep - drawStep) / 60 or -1, (gripStep and hscStep) and (gripStep - hscStep) / 60 or -1))
            if losses > 0 then error(tag .. ' lost the support grip ' .. losses .. ' times') end
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.setGuard(player, false)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end
end
registerTest('GRIP-01 iron claymore', 918790, gripTest('GRIP-01 claymore', 'iron claymore', 0))
registerTest('GRIP-01 iron warhammer', 918791, gripTest('GRIP-01 warhammer', 'iron warhammer', 1))
registerTest('GRIP-01 iron battle axe', 918792, gripTest('GRIP-01 battle axe', 'iron battle axe', 2))

-- STANCE-03: pictures of the stance circle (build 38-42) - the centre guard,
-- the compact guards up, right, left and down, a deep wind-up and a vertical
-- chop passing the front - from the side and from behind.
local function stanceViewsTest(tag, weapon, sequence)
    return function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 850 + sequence, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 850 + sequence end)
            waitSeconds(1)
            powerAndStand(player, weapon)
            ragdoll.prepareMouseInput()
            local function drag(dx, dy, seconds)
                local steps = math.max(1, math.floor(seconds * 60))
                local step0 = tonumber(ragdoll.getState(player).physicsStep)
                local sx, sy = 0, 0
                waitFor('drag', function()
                    local n = tonumber(ragdoll.getState(player).physicsStep) - step0
                    local t = math.min(1, n / steps)
                    local f = t * t * t * (10 - 15 * t + 6 * t * t)
                    local x, y = math.floor(dx * f), math.floor(dy * f)
                    if x ~= sx or y ~= sy then ragdoll.injectMouseInput(x - sx, y - sy) end
                    sx, sy = x, y
                    return n >= steps
                end, seconds + 6)
            end
            local function views(name, settle)
                waitSeconds(settle or 0.6)
                local s = ragdoll.getState(player)
                local center = body(s, 'spine2').position
                local yaw = player.rotation:getYaw()
                for _, view in ipairs({ { name = 'side', yaw = yaw - math.pi / 2, pitch = 0.05, distance = 170 },
                    { name = 'front', yaw = yaw, pitch = 0.1, distance = 170 } }) do
                    view.center = center
                    player:sendEvent('fixInspectCamera', view)
                    waitSeconds(0.08)
                    print(tag .. ' view ' .. name .. ' ' .. view.name)
                    ragdoll.captureScreenshot()
                    waitSeconds(0.05)
                end
                player:sendEvent('fixInspectCamera', {})
                waitSeconds(0.05)
            end
            waitFor('guard', function() return ragdoll.getSwingSpeed(player) == nil end, 4)
            waitSeconds(0.8)
            views('centre')
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitSeconds(0.1)
            drag(0, -250, 0.4) views('guard-up')
            drag(250, 250, 0.5) views('guard-right')
            drag(-500, 0, 0.6) views('guard-left')
            drag(250, 250, 0.5) views('guard-down')
            drag(0, -250, 0.4)
            drag(0, -HSC_WIND, 0.6) views('wind-up-deep')
            drag(0, HSC_CUT * 0.55, 0.25) views('chop-front', 0.0)
            ragdoll.injectMouseInput(0, 0, false, 1)
            waitSeconds(2)
        end)
        ragdoll.injectMouseInput(0, 0, false, 1)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end
end
registerTest('STANCE-03 views iron longsword', 918795, stanceViewsTest('STANCE-03 longsword', 'iron longsword', 0))
registerTest('STANCE-03 views iron claymore', 918796, stanceViewsTest('STANCE-03 claymore', 'iron claymore', 1))

-- RACE-01 (user 11-01: "check if the system works the same for argonians and
-- khajiit"): the beast races use their own skeleton (base_animkna) and
-- animations. The same dummy (the player's stats) as a human, an Argonian
-- and a Khajiit: assembly, the standing body (segment lengths, pelvis and
-- foot heights, modes), a 4 s walk and a 2500 chest blow; unarmed and with
-- a longsword. RACE01 lines carry the numbers to compare across races.
local function raceTemplate(beastRace)
    for _, record in ipairs(types.NPC.records) do
        local race = types.NPC.races.records[record.race]
        if record.isMale and race and (beastRace and record.race == beastRace or not beastRace and not race.isBeast)
            and record.head ~= '' and record.hair ~= '' then
            return record
        end
    end
end
for raceIndex, raceCase in ipairs({
    { 'human', nil, nil }, { 'argonian', 'argonian', nil }, { 'khajiit', 'khajiit', nil },
    { 'human longsword', nil, 'steel longsword' }, { 'argonian longsword', 'argonian', 'steel longsword' },
    { 'khajiit longsword', 'khajiit', 'steel longsword' },
}) do
registerTest('RACE-01 ' .. raceCase[1], 919100 + raceIndex, function()
    local tag, beastRace, weapon = raceCase[1], raceCase[2], raceCase[3]
    placePlayer()
    ragdoll.setAutomaticRecovery(true)
    local look = raceTemplate(beastRace)
    testing.expect(look ~= nil, 'no male ' .. tostring(beastRace or 'human') .. ' record with a head and hair')
    local player = world.players[1]
    local record = world.createRecord(types.NPC.createRecordDraft({
        template = types.NPC.record(player), name = 'RACE-01 ' .. tag, race = look.race, head = look.head,
        hair = look.hair, isMale = true, isEssential = false, isPersistent = false, isRespawning = false,
    }))
    local actor = world.createObject(record.id)
    if weapon then world.createObject(weapon, 1):moveInto(types.Actor.inventory(actor)) end
    actor:teleport(testArea.cell, testArea.player + util.vector3(-450, 450, 0), { onGround = true })
    actor:addScript('stance.lua')
    coroutine.yield()
    actor:sendEvent('fixActorConfigure', { weaponStance = weapon })
    waitFor(tag .. ' stance', function() return readyActors[actor.id] ~= nil end, 10)
    local ok, message = pcall(function()
        local state = powerAndStand(actor, 'RACE-01 ' .. tag)
        local names = {}
        for _, item in ipairs(state.bodies) do names[#names + 1] = item.name end
        print(string.format('RACE01 %s race=%s bodies=%d %s', tag, tostring(types.NPC.record(actor).race),
            #state.bodies, table.concat(names, ',')))
        -- Standing 3 s: modes, then the held body's shape.
        local seen = {}
        watchModes(actor, 3, seen)
        local modes = {}
        for mode in pairs(seen) do modes[#modes + 1] = mode end
        table.sort(modes)
        state = ragdoll.getState(actor)
        local ground = actor.position.z
        local function at(name) return body(state, name).position end
        local function span(a, b) return (at(a) - at(b)):length() end
        print(string.format('RACE01 %s stand modes=%s pelvis_z=%.1f head_z=%.1f lfoot_z=%.1f rfoot_z=%.1f '
            .. 'thigh=%.1f/%.1f calf=%.1f/%.1f foot_gap=%.1f pelvis_head=%.1f upperarm=%.1f forearm=%.1f',
            tag, table.concat(modes, '+'), at('pelvis').z - ground, at('head').z - ground,
            at('left_foot').z - ground, at('right_foot').z - ground,
            span('left_thigh', 'left_calf'), span('right_thigh', 'right_calf'),
            span('left_calf', 'left_foot'), span('right_calf', 'right_foot'),
            util.vector3(at('left_foot').x - at('right_foot').x, at('left_foot').y - at('right_foot').y, 0):length(),
            span('pelvis', 'head'), span('right_upper_arm', 'right_forearm'), span('right_forearm', 'right_hand')))
        testing.expect(seen.standing and not seen.falling and not seen.fallen, 'did not stand still for 3 s')
        -- Walk 4 s.
        local from, fromTime = actor.position, core.getSimulationTime()
        local walkSeen, lowFoot, highFoot = {}, math.huge, -math.huge
        actor:sendEvent('fixActorMove', 1)
        waitSeconds(4, function()
            local now = ragdoll.getState(actor)
            if not (now and now.controller) then return end
            walkSeen[now.controller.mode] = true
            local z = actor.position.z
            for _, name in ipairs({ 'left_foot', 'right_foot' }) do
                local foot = body(now, name).position.z - z
                lowFoot, highFoot = math.min(lowFoot, foot), math.max(highFoot, foot)
            end
        end)
        actor:sendEvent('fixActorMove', 0)
        local path = util.vector3(actor.position.x - from.x, actor.position.y - from.y, 0):length()
        local speed = path / math.max(core.getSimulationTime() - fromTime, 0.1)
        local walkModes = {}
        for mode in pairs(walkSeen) do walkModes[#walkModes + 1] = mode end
        table.sort(walkModes)
        print(string.format('RACE01 %s walk speed=%.0f modes=%s foot_z=%.1f..%.1f', tag, speed,
            table.concat(walkModes, '+'), lowFoot, highFoot))
        testing.expect(speed > 40, 'did not walk')
        testing.expect(not walkSeen.falling and not walkSeen.fallen, 'fell while walking')
        -- Settle, then a 2500 chest blow from the side.
        waitSeconds(1.5)
        local fell, recovered, blowSeen = false, nil, {}
        local started = core.getSimulationTime()
        ragdoll.applyBlow(actor, 'spine2', util.vector3(1, 0, 0.1):normalize(), 2500, 6, 0)
        waitSeconds(6, function()
            local now = ragdoll.getState(actor)
            local mode = now and now.controller and now.controller.mode or '-'
            blowSeen[mode] = true
            if mode == 'falling' or mode == 'fallen' then fell = true end
            if fell and not recovered and mode == 'standing' then recovered = core.getSimulationTime() - started end
        end)
        local blowModes = {}
        for mode in pairs(blowSeen) do blowModes[#blowModes + 1] = mode end
        table.sort(blowModes)
        print(string.format('RACE01 %s blow modes=%s fell=%s recovered=%s', tag, table.concat(blowModes, '+'),
            tostring(fell), recovered and string.format('%.1f', recovered) or '-'))
    end)
    if actor:isValid() then actor:remove() end
    if not ok then error(message, 0) end
end)
end

-- GRIP-03 (10-09, user: "in one handed weapons the left hand is always too
-- close to the face so it blocks the camera"; "dissolve the body"): the
-- whole case in first person - guard, walking, a cut - one shot level and
-- one looking down at each moment (GRIP03 lines name them). FPBODY-01 (10-31,
-- user: looking down "it's transparent and looks weird"): a third shot as
-- far down as the view goes (65 degrees), and a run dressed in a shirt,
-- trousers and iron armour at the player's field of view (83).
local GRIP03_DRESSED = { Shirt = 'common_shirt_01', Pants = 'common_pants_01', Cuirass = 'iron_cuirass',
    LeftPauldron = 'iron_pauldron_left', RightPauldron = 'iron_pauldron_right' }
for index, setup in ipairs({ { 'iron longsword' }, { 'iron claymore' }, { 'iron longsword', GRIP03_DRESSED } }) do
    local weapon, wear = setup[1], setup[2]
    registerTest('GRIP-03 first-person view of the hands with ' .. weapon .. (wear and ' dressed' or ''),
        918890 + index, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            for _, id in ipairs({ weapon, wear and wear.Shirt, wear and wear.Pants, wear and wear.Cuirass,
                wear and wear.LeftPauldron, wear and wear.RightPauldron }) do
                if not types.Actor.inventory(player):find(id) then
                    world.createObject(id, 1):moveInto(types.Actor.inventory(player))
                end
            end
            coroutine.yield()
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, wear = wear, sequence = 890 + index,
                firstPerson = true, fov = wear and 83 })
            waitFor('player loadout', function() return playerReadySequence == 890 + index end)
            waitSeconds(1)
            powerAndStand(player, 'GRIP-03 player')
            cameraReadyMode = nil
            player:sendEvent('fixPlayerCamera', true)
            waitFor('GRIP-03 first-person camera', function() return cameraReadyMode == true end)
            ragdoll.prepareMouseInput()
            local shot = 0
            local function snap(tag)
                for _, pitch in ipairs({ 0, 0.5, 1.13 }) do
                    player:sendEvent('fixPlayerPitch', pitch)
                    coroutine.yield()
                    coroutine.yield()
                    coroutine.yield()
                    shot = shot + 1
                    print(string.format('GRIP03 shot=%d weapon=%s moment=%s pitch=%.1f', shot, weapon, tag, pitch))
                    ragdoll.captureScreenshot()
                    waitSeconds(0.1)
                end
                player:sendEvent('fixPlayerPitch', 0)
            end
            local function drag(x, seconds)
                local started, sent = core.getSimulationTime(), 0
                waitSeconds(seconds, function()
                    local f = math.min(1, (core.getSimulationTime() - started) / seconds)
                    local v = math.floor(x * f)
                    ragdoll.injectMouseInput(v - sent, 0)
                    sent = v
                end)
                ragdoll.injectMouseInput(x - sent, 0)
            end
            waitSeconds(2.0)
            snap('guard')
            player:sendEvent('fixPlayerMove', 1)
            waitSeconds(2.0)
            snap('walking')
            player:sendEvent('fixPlayerMove', 0)
            waitSeconds(1.5)
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitSeconds(0.25)
            drag(HSC_WIND, 0.4)
            snap('wound up')
            drag(-1200, 0.1)
            waitSeconds(0.05)
            snap('mid cut')
            ragdoll.injectMouseInput(0, 0, false, 1)
            waitSeconds(1.5)
            snap('after cut')
        end)
        player:sendEvent('fixPlayerMove', 0)
        player:sendEvent('fixPlayerPitch', 0)
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end)
end

-- GRIP-02 (10-09, user: "the left hand does not attach properly to the weapon
-- handle" - standing, guard and swings; with one-handed weapons "the left
-- hand is always too close to the face so it blocks the camera"): close
-- stills of both hands (left side, front, from above) and first-person shots
-- in the guard, wound up, mid-cut, after the cut and walking. GRIP02 lines
-- name the shots; the palm against the handle line is in RAGDOLL_GRIP_LINE
-- (OPENMW_RAGDOLL_GRIP_LINE_TRACE=1), framed by GRIP02_MOMENT lines.
for index, weapon in ipairs({ 'iron claymore', 'iron warhammer', 'iron longsword' }) do
    registerTest('GRIP-02 hands on the handle with ' .. weapon, 918880 + index, function()
        local player = placePlayer()
        local ok, message = pcall(function()
            if not types.Actor.inventory(player):find(weapon) then
                world.createObject(weapon, 1):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            player:sendEvent('fixPlayerConfigure', { weapon = weapon, sequence = 880 + index, firstPerson = false })
            waitFor('player loadout', function() return playerReadySequence == 880 + index end)
            waitSeconds(1)
            powerAndStand(player, 'GRIP-02 player')
            ragdoll.prepareMouseInput()
            local shot = 0
            local function snap(tag, view)
                coroutine.yield()
                coroutine.yield()
                coroutine.yield()
                shot = shot + 1
                print(string.format('GRIP02 shot=%d weapon=%s moment=%s view=%s', shot, weapon, tag, view))
                ragdoll.captureScreenshot()
                waitSeconds(0.1)
            end
            local function shots(tag)
                print(string.format('GRIP02_MOMENT weapon=%s moment=%s t=%.2f', weapon, tag, core.getSimulationTime()))
                -- Close views aimed at the middle of the two hands: the left
                -- side, the front, above the front.
                local s = ragdoll.getState(player)
                local mid = player.position + util.vector3(0, 20, 95)
                if s and s.bodies and s.weapon and s.weapon.handBodyIndex and s.bodies[s.weapon.handBodyIndex] then
                    local left = body(s, 'left_hand')
                    local right = s.bodies[s.weapon.handBodyIndex].position
                    mid = left and (left.position + right) * 0.5 or right
                end
                for _, view in ipairs({ { 'left', util.vector3(-1, 0.3, 0.35) }, { 'front', util.vector3(0.15, 1, 0.3) },
                    { 'above', util.vector3(-0.3, 0.6, 1.0) } }) do
                    local eye = mid + view[2]:normalize() * 70
                    local d = mid - eye
                    player:sendEvent('fixWoundCamera', { position = eye, yaw = math.atan2(d.x, d.y),
                        pitch = math.atan2(-d.z, math.sqrt(d.x * d.x + d.y * d.y)) })
                    snap(tag, view[1])
                end
                cameraReadyMode = nil
                player:sendEvent('fixPlayerCamera', true)
                pcall(waitFor, 'GRIP-02 first person', function() return cameraReadyMode == true end, 2)
                snap(tag, 'first')
                player:sendEvent('fixPlayerPitch', 0.6)
                snap(tag, 'first down')
                player:sendEvent('fixPlayerPitch', 0)
            end
            local function drag(x, seconds)
                local started, sent = core.getSimulationTime(), 0
                waitSeconds(seconds, function()
                    local f = math.min(1, (core.getSimulationTime() - started) / seconds)
                    local v = math.floor(x * f)
                    ragdoll.injectMouseInput(v - sent, 0)
                    sent = v
                end)
                ragdoll.injectMouseInput(x - sent, 0)
            end
            waitSeconds(1.5)
            shots('guard')
            ragdoll.injectMouseInput(0, 0, true, 1)
            waitSeconds(0.25)
            drag(HSC_WIND, 0.4)
            shots('wound up')
            drag(-1200, 0.1)
            waitSeconds(0.05)
            shots('mid cut')
            ragdoll.injectMouseInput(0, 0, false, 1)
            waitSeconds(1.5)
            shots('after cut')
            player:sendEvent('fixPlayerMove', 1)
            waitSeconds(2.0)
            shots('walking')
            player:sendEvent('fixPlayerMove', 0)
            waitSeconds(1.0)
            print(string.format('GRIP02_MOMENT weapon=%s moment=end t=%.2f', weapon, core.getSimulationTime()))
        end)
        player:sendEvent('fixPlayerMove', 0)
        player:sendEvent('fixWoundCamera', {})
        ragdoll.injectMouseInput(0, 0, false)
        ragdoll.setPowered(player, false)
        if not ok then error(message, 0) end
    end)
end

handlers.fixAimMotion = function(value) ragdoll.injectMouseInput(value.x, value.y) end
return {
    engineHandlers = { onUpdate = testing.updateGlobal },
    eventHandlers = handlers,
}

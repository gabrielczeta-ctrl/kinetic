local core = require('openmw.core')
local ragdoll = require('openmw.ragdolltest')
local testing = require('testing_util')
local util = require('openmw.util')
local world = require('openmw.world')
local diagnostic = require('diagnostics')
local protocols = require('protocols')
local types = require('openmw.types')
local input
testing.globalEventHandlers.play01InputSample = function(value) input = value end

local function waitFor(description, predicate, timeout)
    local simulationDeadline, realDeadline = core.getSimulationTime() + timeout, core.getRealTime() + timeout * 5 + 10
    repeat
        local value = predicate()
        if value then return value end
        coroutine.yield()
    until core.getSimulationTime() >= simulationDeadline or core.getRealTime() >= realDeadline
    error('PLAY-01 timed out waiting for ' .. description)
end

local modes = {
    { name = 'unarmed', powered = true, prepare = 'PLAY-01 prepare ordinary controls' },
    { name = 'warhammer', powered = true, prepare = 'PLAY-01 prepare ordinary controls with warhammer',
        weapon = 'iron warhammer' },
    { name = 'vanilla', powered = false, prepare = 'PLAY-01 prepare ordinary controls' },
    -- Play test 2026-09-24: the user walked with a battle axe equipped but
    -- sheathed, which no case covered (tripping, crossed feet).
    { name = 'sheathed', powered = true, prepare = 'PLAY-01 prepare ordinary controls with sheathed battle axe',
        weapon = 'iron battle axe', protocols = {
            ['forward'] = true, ['backward'] = true, ['strafe left'] = true, ['strafe right'] = true,
            ['diagonal left'] = true, ['diagonal right'] = true, ['start stop'] = true,
            ['moving turn'] = true, ['reversal'] = true, ['RAMP-01 ship player up'] = true,
            ['rebuild mid-run'] = true, ['run draw'] = true,
        } },
    -- Test course loadouts (fists raised, dagger, claymore; longsword and
    -- warhammer are above).
    { name = 'fists', powered = true, prepare = 'PLAY-01 prepare ordinary controls with fists raised',
        protocols = { ['course'] = true, ['course long'] = true, ['course run'] = true,
            ['course balmora up'] = true, ['course balmora down'] = true } },
    { name = 'dagger', powered = true, prepare = 'PLAY-01 prepare ordinary controls with dagger',
        weapon = 'iron dagger', protocols = { ['course'] = true, ['course run'] = true } },
    { name = 'claymore', powered = true, prepare = 'PLAY-01 prepare ordinary controls with claymore',
        weapon = 'iron claymore', protocols = { ['course'] = true, ['course run'] = true,
            ['course balmora up'] = true, ['course balmora down'] = true, ['run diagonal'] = true,
            ['run zigzag'] = true, ['start stop'] = true } },
    -- A one-handed weapon with a free hand (look review only).
    { name = 'longsword', powered = true, prepare = 'PLAY-01 prepare ordinary controls with longsword',
        weapon = 'iron longsword', protocols = {
            ['look walk'] = true, ['look run'] = true, ['look stand'] = true, ['look attack'] = true,
            ['look strafe'] = true, ['course'] = true, ['course run'] = true,
        } },
    -- TORCH-01: a torch in the left hand, longsword sheathed / drawn.
    { name = 'torch', powered = true, prepare = 'PLAY-01 prepare ordinary controls with torch',
        weapon = 'iron longsword', left = 'torch',
        protocols = { ['no input'] = true, ['forward'] = true, ['start stop'] = true } },
    { name = 'torch bare', powered = true, prepare = 'PLAY-01 prepare ordinary controls with bare torch',
        left = 'torch', protocols = { ['no input'] = true, ['forward'] = true, ['start stop'] = true } },
    { name = 'torch fp', powered = true, prepare = 'PLAY-01 prepare ordinary controls with first person torch',
        weapon = 'iron longsword', left = 'torch',
        protocols = { ['no input'] = true, ['forward'] = true, ['start stop'] = true } },
    -- The playthrough character: a sheathed two-hander and a torch.
    { name = 'torch claymore', powered = true, prepare = 'PLAY-01 prepare ordinary controls with claymore torch',
        weapon = 'iron claymore', left = 'torch',
        protocols = { ['no input'] = true, ['forward'] = true, ['start stop'] = true, ['course run'] = true,
            ['course balmora up'] = true } },
    { name = 'torch drawn', powered = true, prepare = 'PLAY-01 prepare ordinary controls with drawn torch',
        weapon = 'iron longsword', left = 'torch',
        protocols = { ['no input'] = true, ['forward'] = true, ['start stop'] = true } },
}

local function addItem(actor, recordId)
    local inventory = types.Actor.inventory(actor)
    if not inventory:find(recordId) then
        world.createObject(recordId, 1):moveInto(inventory)
        coroutine.yield()
    end
    testing.expect(inventory:find(recordId) ~= nil, 'PLAY-01 could not add ' .. recordId)
end

for modeIndex, mode in ipairs(modes) do
    for protocolIndex, protocol in ipairs(protocols) do
        -- The vanilla reference has no physical bodies to receive impulses.
        -- Keep these protocols on both powered modes; they are not applicable
        -- to the animation-only control and must not be reported as passes.
        local requiresBody = false
        for _, segment in ipairs(protocol.segments) do
            requiresBody = requiresBody or segment.impactAt ~= nil or segment.disturbanceAt ~= nil or segment.blowAt ~= nil
                or segment.grab ~= nil
        end
        -- protocol.modes: the modes a case runs in (PGC-03 cases: unarmed only).
        requiresBody = requiresBody or protocol.poweredOnly == true
        if (mode.powered or not requiresBody) and (not mode.protocols or mode.protocols[protocol.name] or protocol.allModes)
            and (not protocol.modes or protocol.modes[mode.name]) then
        local animationMode = mode.name
        local name = 'PLAY-01 ' .. animationMode .. ' ' .. protocol.name
        testing.registerGlobalTest(name, function()
            local player = world.players[1]
            input = nil
            if protocol.exterior then
                -- Terrain (Balmora): an exterior start, onGround.
                local at = protocol.exterior
                player:teleport('', at, {
                    onGround = true, rotation = util.transform.rotateZ(protocol.initialYaw or 0),
                })
                waitFor('exterior relocation', function()
                    return player.cell.isExterior
                        and math.abs(player.position.x - at.x) < 1 and math.abs(player.position.y - at.y) < 1
                end, 10)
            else
            -- protocol.start: a place on the test floor's fixtures (PGC-03:
            -- the ledges, ramps and tall stairs; tools/make-ragdoll-test-features.py).
            -- protocol.interior = { cell, position }: a Morrowind interior (RAMP-01).
            local cellName = protocol.interior and protocol.interior[1] or 'OpenMW Ragdoll Test Floor'
            local at = protocol.interior and protocol.interior[2] or protocol.start or util.vector3(0, 0, 128)
            player:teleport(cellName, at, {
                onGround = true, rotation = util.transform.rotateZ(protocol.initialYaw or 0),
            })
            waitFor('interior relocation', function()
                return player.cell.name == cellName
                    and math.abs(player.position.x - at.x) < 1 and math.abs(player.position.y - at.y) < 1
            end, 5)
            -- Character creation over: the prison ship's chargen scripts
            -- otherwise open the name menu, which pauses the game.
            if protocol.interior then
                pcall(function() world.mwscript.getGlobalVariables(player).CharGenState = -1 end)
            end
            end
            if mode.weapon then addItem(player, mode.weapon) end
            if mode.left then addItem(player, mode.left) end
            -- SAVE-01 loaded cases: carried weight (unequipped), for the
            -- encumbrance the balance controller reads.
            if protocol.carry then
                world.createObject(protocol.carry[1], protocol.carry[2]):moveInto(types.Actor.inventory(player))
                coroutine.yield()
            end
            testing.runLocalTest(player, mode.prepare)
            if protocol.fatigueShare then
                player:sendEvent('play01Fatigue', { share = protocol.fatigueShare })
            end
            -- ACRO-01: protocol.acrobatics is the player's Acrobatics for the
            -- case; every other case gets the character's own back.
            player:sendEvent('play01Acrobatics', protocol.acrobatics or false)
            waitFor(animationMode .. ' idle input acknowledgement', function()
                return input and input.segment == 'setup'
                    and input.right_equipped == (mode.weapon ~= nil) and input.left_equipped == (mode.left ~= nil)
            end, 5)
            if mode.weapon then
                testing.expect(ragdoll.prepareEquippedWeapon(player),
                    'PLAY-01 could not select equipped ' .. mode.weapon)
            end
            -- Match the existing ordinary player's one-second animated spawn
            -- settling period. Subsequent powered startup is included in the
            -- trace and two-second idle segment, never hidden by a ready gate.
            -- Optional powered opponent standing in the walking path (user
            -- report 2026-09-23: bumping an opponent made it oscillate).
            local opponent
            if protocol.opponentAhead then
                local record = world.createRecord(types.NPC.createRecordDraft({
                    template = types.NPC.record(player), name = 'PLAY-01 bump opponent',
                    isEssential = false, isPersistent = false, isRespawning = false,
                }))
                opponent = world.createObject(record.id)
                opponent:teleport(protocol.exterior and '' or 'OpenMW Ragdoll Test Floor',
                    player.position + player.rotation * util.vector3(0, protocol.opponentAhead, 0),
                    { onGround = true, rotation = util.transform.rotateZ((protocol.initialYaw or 0) + math.pi) })
                coroutine.yield()
                ragdoll.setPowered(opponent, true)
                waitFor('bump opponent standing', function()
                    local state = ragdoll.getState(opponent)
                    return state and state.controller and state.controller.mode == 'standing'
                end, 10)
            end
            -- protocol.escort = { record id, start, destination }: a copy of
            -- an NPC of the cell (its record without its mwscript, which would
            -- give it orders of its own: the prison ship's chargen guard walks
            -- back to greet the player), put at the start, powered, travels to
            -- the destination by Morrowind's AI (RAMP-01: the guard from
            -- Jiub's side up the ramp to the deck). The original is put away.
            if protocol.escort then
                for _, actor in ipairs(world.activeActors) do
                    if actor.recordId == protocol.escort[1] and actor ~= player then
                        actor:teleport('', util.vector3(0, 0, -10000))
                    end
                end
                local record = world.createRecord(types.NPC.createRecordDraft({
                    template = protocol.escort[1] == 'player' and types.NPC.record(player)
                        or types.NPC.record(protocol.escort[1]), mwscript = '',
                    isEssential = false, isPersistent = false, isRespawning = false,
                }))
                opponent = world.createObject(record.id)
                opponent:teleport(player.cell.name, protocol.escort[2], { onGround = true })
                waitFor('escort at its start', function()
                    return (opponent.position - protocol.escort[2]):length() < 40
                end, 5)
                -- escort[4] == 'vanilla': left unpowered (the control).
                if protocol.escort[4] ~= 'vanilla' then
                    ragdoll.setPowered(opponent, true)
                    waitFor('escort standing', function()
                        local state = ragdoll.getState(opponent)
                        return state and state.controller and state.controller.mode == 'standing'
                    end, 10)
                else
                    ragdoll.setPowered(opponent, false)
                end
                -- 'door': the cell's door to an exterior (the ship's deck door).
                local destination = protocol.escort[3]
                if destination == 'door' then
                    for _, door in ipairs(player.cell:getAll(types.Door)) do
                        local leads = types.Door.destCell(door)
                        if leads and not leads.isInterior then destination = door.position end
                    end
                    testing.expect(destination ~= 'door', 'PLAY-01 no door to an exterior in the cell')
                end
                print(string.format('PLAY01_ESCORT from=%s to=%s', tostring(opponent.position), tostring(destination)))
                player:sendEvent('play01Path', { from = opponent.position, to = destination })
                player:sendEvent('play01Ground', { ys = { 1100, 1130, 1140, 1150, 1170, 1200, 1250, 1300, 1350 },
                    x0 = -60, x1 = 90, dx = 6 })
                opponent:sendEvent('StartAIPackage', { type = 'Travel', destPosition = destination })
            end
            local settle = core.getSimulationTime() + 1
            repeat coroutine.yield() until core.getSimulationTime() >= settle
            diagnostic.begin()
            diagnostic.write({ kind = 'anchor', test = name, animation_mode = animationMode,
                protocol = protocol, sampling = 'render-observed; no interpolation',
                stage_a_control = protocol.control,
                fixture = 'flat-floor-v1', start_position = player.position,
                ordinary_controls = true, running = false, camera = 'third person',
                common_seed = protocol.commonSeed,
                replay_seed = protocol.replaySeed or (20261400 + modeIndex * 100 + protocolIndex),
                units = 'OpenMW world units; seconds; radians',
                purpose = 'diagnostic capture completeness only; no gameplay acceptance verdict',
            })
            local started, previousStep, previousTime = core.getSimulationTime(), nil, nil
            local firstFailure, replayStarted, segmentsStarted = nil, false, 0
            local replaySeed = protocol.replaySeed or (20261400 + modeIndex * 100 + protocolIndex)
            local function observe(segment)
                local state = ragdoll.getState(player)
                local now, step = core.getSimulationTime(), state and state.physicsStep
                if state and not replayStarted then
                    ragdoll.startReplayRecording(player, replaySeed, 1800)
                    replayStarted = true
                end
                if step ~= nil and step == previousStep then return end
                if step == nil and now == previousTime then return end
                local controllerMode = state and state.controller and state.controller.mode
                local expectedPowered = mode.powered
                -- Knockdown cases fall on purpose and watch the get-up.
                local anomaly = expectedPowered and ((not state and now - started >= 0.5 and 'missing rig')
                    or (not protocol.allowFall and state and (controllerMode == 'falling' or controllerMode == 'fallen'
                    or controllerMode == 'getting_up' or controllerMode == 'blend_back') and controllerMode))
                if anomaly and not firstFailure then
                    firstFailure = { elapsed = now - started, physics_step = step,
                        segment = segment, observed_mode = anomaly }
                    diagnostic.write({ kind = 'first_failure', test = name, observation = firstFailure })
                    -- Zero ordinary movement immediately; observe one second
                    -- of settling context, then leave later commands unexecuted.
                    player:sendEvent('play01Travel', { segment = 'failure stop' })
                end
                local opponentState = opponent and opponent:isValid() and ragdoll.getState(opponent)
                local opponentSummary
                if opponentState and opponentState.bodies then
                    local feet = {}
                    for _, body in ipairs(opponentState.bodies) do
                        if body.name == 'left_foot' or body.name == 'right_foot' then feet[body.name] = body.position end
                    end
                    for _, body in ipairs(opponentState.bodies) do
                        if body.name == 'pelvis' then
                            opponentSummary = { position = body.position, angularVelocity = body.angularVelocity,
                                linearVelocity = body.linearVelocity,
                                mode = opponentState.controller and opponentState.controller.mode,
                                pelvisUpDot = opponentState.controller and opponentState.controller.pelvisUpDot,
                                torsoUpDot = opponentState.controller and opponentState.controller.torsoUpDot,
                                leftFoot = feet.left_foot, rightFoot = feet.right_foot }
                        end
                    end
                end
                diagnostic.write({ kind = 'frame', test = name, elapsed = now - started, opponent = opponentSummary,
                    opponent_position = opponent and opponent:isValid() and opponent.position or nil,
                    segment = segment, input = input, physics_step = step,
                    skipped_steps = step and previousStep and math.max(0, step - previousStep - 1) or 0,
                    player_position = player.position, gameplay_yaw = player.rotation:getYaw(),
                    rig_present = state ~= nil, state = diagnostic.state(state),
                })
                previousStep, previousTime = step, now
                return step
            end
            local ok, err = pcall(function()
                if mode.powered then ragdoll.setPowered(player, true) end
                if protocol.poweredLying then
                    -- The body is powered while the animation lies down, as
                    -- the manual hub re-powered a player whose killing blow
                    -- had also knocked it down: here Morrowind's knockout
                    -- (fatigue below zero) lays the unpowered actor down, the
                    -- body is powered 2.5 s into it, and fatigue comes back so
                    -- the clip gets up.
                    local wait = function(seconds)
                        local simulation, real = core.getSimulationTime() + seconds, core.getRealTime() + seconds * 5 + 10
                        repeat coroutine.yield()
                        until core.getSimulationTime() >= simulation or core.getRealTime() >= real
                    end
                    if mode.powered then ragdoll.setPowered(player, false) end
                    wait(0.5)
                    player:sendEvent('play01Fatigue', -20)
                    wait(2.5)
                    if mode.weapon then
                        testing.expect(ragdoll.prepareEquippedWeapon(player),
                            'PLAY-01 could not select equipped ' .. mode.weapon .. ' while knocked out')
                    end
                    ragdoll.setPowered(player, true)
                    wait(0.2)
                    player:sendEvent('play01Fatigue', nil)
                    diagnostic.write({ kind = 'powered_lying', test = name,
                        simulation_time = core.getSimulationTime() })
                end
                if ragdoll.testStepMultiplier() > 0 then
                    ragdoll.setTestStepActive(true, replaySeed)
                end
                -- Film mode (showcase clips, 10-11): OPENMW_RAGDOLL_TEST_FILM =
                -- side camera yaw offset in degrees (e.g. 90); every segment is
                -- filmed at 15 stills per simulated second from a camera that
                -- follows the player (tools/body/film.py joins them into clips).
                local filmText = ragdoll.tuningText and ragdoll.tuningText('OPENMW_RAGDOLL_TEST_FILM')
                local filmYaw = filmText and math.rad(tonumber(filmText) or 90)
                local filmStart, filmLast = core.getSimulationTime(), nil
                for index, segment in ipairs(protocol.segments) do
                    segmentsStarted = index
                    if filmYaw and segment.sideCamera == nil then
                        local filmed = {}
                        for k, v in pairs(segment) do filmed[k] = v end
                        filmed.sideCamera, filmed.cameraDistance, filmed.cameraHeight = filmYaw, 300, 60
                        filmed.film = true -- the player shows the time barcode (film.py)
                        segment = filmed
                    end
                    player:sendEvent('play01Travel', segment)
                    -- A click (look attack): press and release in one batch, as the
                    -- mouse does; the physics consumes it at the next fixed step.
                    if segment.click then
                        ragdoll.injectMouseInput(0, 0, true)
                        ragdoll.injectMouseInput(0, 0, false)
                    end
                    diagnostic.write({ kind = 'segment_request', test = name, index = index, command = segment })
                    local segmentStarted = core.getSimulationTime()
                    local deadline = core.getRealTime() + segment.duration * 5 + 10
                    local impactApplied, disturbanceApplied, staggerApplied, blowApplied = false, false, false, false
                    local grabStart
                    local segmentTicks = math.floor(segment.duration * 60 + 0.5)
                    local shotIndex = 1
                    local repowered = false
                    for segmentTick = 1, segmentTicks do
                        coroutine.yield()
                        -- segment.repowerAt: the body rebuilt this many seconds
                        -- in (BUILD_FRAMES; a door's rebuild mid-stride).
                        if segment.repowerAt and not repowered and mode.powered
                            and core.getSimulationTime() - segmentStarted >= segment.repowerAt then
                            repowered = true
                            ragdoll.setPowered(player, false)
                            ragdoll.setPowered(player, true)
                            diagnostic.write({ kind = 'repower', test = name, segment = segment.segment,
                                simulation_time = core.getSimulationTime() })
                        end
                        if filmYaw and (not filmLast or core.getSimulationTime() - filmLast >= 1 / 15 - 1e-4) then
                            filmLast = core.getSimulationTime()
                            ragdoll.captureScreenshot()
                            diagnostic.write({ kind = 'screenshot', test = name, segment = segment.segment,
                                at = filmLast - filmStart, view = 'film', real = core.getRealTime() })
                        end
                        -- Optional stills for visual review (OPENMW_RAGDOLL_SHOTS
                        -- env is not visible to Lua, so the protocol opts in).
                        local shots = segment.screenshots
                        if shots and shots[shotIndex]
                            and core.getSimulationTime() - segmentStarted >= shots[shotIndex] then
                            ragdoll.captureScreenshot()
                            diagnostic.write({ kind = 'screenshot', test = name, segment = segment.segment,
                                at = shots[shotIndex], view = segment.view })
                            shotIndex = shotIndex + 1
                        end
                        if segment.impactAt and not impactApplied
                            and core.getSimulationTime() - segmentStarted >= segment.impactAt then
                            -- impactTarget = 'opponent' knocks the opponent down instead.
                            local struck = segment.impactTarget == 'opponent' and opponent or player
                            local impactState = ragdoll.getState(struck)
                            local target = segment.impactBody or 'pelvis'
                            local pelvis
                            if impactState then
                                for _, body in ipairs(impactState.bodies) do
                                    if body.name == target then pelvis = body break end
                                end
                            end
                            testing.expect(pelvis ~= nil, 'PLAY-01 impact has no ' .. target .. ' body')
                            -- impactImpulse is in the player's frame (x right,
                            -- y forward); the default is the old +X world push.
                            local impulse = util.vector3(400, 0, 0)
                            if segment.impactImpulse then
                                local i = segment.impactImpulse
                                impulse = player.rotation * util.vector3(i[1], i[2], i[3] or 0)
                            end
                            diagnostic.write({ kind = 'impact', test = name, body = target, impulse = impulse })
                            ragdoll.applyCombatImpulse(struck, impulse, pelvis.centerOfMass,
                                20261500 + protocolIndex)
                            impactApplied = true
                        end
                        -- A blow through the reaction controller (STAGGER-01):
                        -- segment.blow = { part, { right, forward } direction in
                        -- the player's frame, weapon speed, mass, damage }.
                        if segment.blowAt and not blowApplied
                            and core.getSimulationTime() - segmentStarted >= segment.blowAt then
                            local b = segment.blow
                            local dir = player.rotation * util.vector3(b[2][1], b[2][2], 0)
                            diagnostic.write({ kind = 'impact', test = name, body = b[1], blow = true,
                                speed = b[3], mass = b[4] })
                            ragdoll.applyBlow(player, b[1], dir:normalize(), b[3], b[4], b[5])
                            blowApplied = true
                        end
                        if segment.staggerAt and not staggerApplied
                            and core.getSimulationTime() - segmentStarted >= segment.staggerAt then
                            player:sendEvent('play01Stagger', segment.stagger)
                            diagnostic.write({ kind = 'stagger', test = name, stagger = segment.stagger })
                            staggerApplied = true
                        end
                        if segment.disturbanceAt and not disturbanceApplied
                            and core.getSimulationTime() - segmentStarted >= segment.disturbanceAt then
                            local disturbanceState = ragdoll.getState(player)
                            local pelvis, pelvisIndex
                            if disturbanceState then
                                for bodyIndex, body in ipairs(disturbanceState.bodies) do
                                    if body.name == 'pelvis' then
                                        pelvis, pelvisIndex = body, bodyIndex
                                        break
                                    end
                                end
                            end
                            testing.expect(pelvis ~= nil, 'PLAY-01 heading disturbance has no pelvis body')
                            ragdoll.applyImpulse(player, pelvisIndex, util.vector3(500, 0, 0),
                                pelvis.centerOfMass + util.vector3(0, 20, 0))
                            disturbanceApplied = true
                        end
                        -- Drag the chest (segment.grab = { right, forward } in
                        -- the player's frame, reached over the first 60% of
                        -- the segment, released at its end).
                        if segment.grab then
                            if segmentTick == 1 then
                                local grabState = ragdoll.getState(player)
                                for _, body in ipairs(grabState and grabState.bodies or {}) do
                                    if body.name == (segment.grabBody or 'spine2') then
                                        grabStart = body.centerOfMass
                                        break
                                    end
                                end
                                testing.expect(grabStart ~= nil, 'PLAY-01 grab has no grab body')
                                ragdoll.grab(player, grabStart + util.vector3(0, 0, 100), util.vector3(0, 0, -1),
                                    segment.grabForce or 30000)
                                diagnostic.write({ kind = 'impact', test = name, body = segment.grabBody or 'spine2',
                                    grab = true })
                            end
                            local share = math.min(1, segmentTick / (0.6 * segmentTicks))
                            ragdoll.updateGrab(player, grabStart
                                + player.rotation * util.vector3(segment.grab[1], segment.grab[2], 0) * share)
                            if segmentTick == segmentTicks then
                                ragdoll.releaseGrab(player)
                            end
                        end
                        local observedStep = observe(segment.segment)
                        if segmentTick == 1 then
                            diagnostic.write({ kind = 'transition_marker', test = name, index = index,
                                segment = segment.segment, physics_step = observedStep,
                                simulation_time = core.getSimulationTime() })
                        end
                        if firstFailure and core.getSimulationTime() - started >= firstFailure.elapsed + 1 then break end
                        if core.getRealTime() >= deadline then error('PLAY-01 segment realtime timeout: ' .. segment.segment) end
                    end
                    if firstFailure then break end
                end
            end)
            if ragdoll.testStepMultiplier() > 0 then ragdoll.setTestStepActive(false, 0) end
            if replayStarted then
                local replay = ragdoll.finishReplayRecording()
                diagnostic.write({ kind = 'fixed_step_replay', test = name, seed = replaySeed,
                    present = replay ~= nil, frames = replay and replay.frameCount,
                    bytes = replay and replay.byteCount, output_path = replay and replay.outputPath,
                    fixed_time_step = replay and replay.fixedTimeStep, profile_hash = replay and replay.profileHash,
                })
            end
            diagnostic.write({ kind = 'capture_stop', test = name, elapsed = core.getSimulationTime() - started,
                segments_started = segmentsStarted, segments_planned = #protocol.segments,
                first_failure = firstFailure, capture_error = not ok and tostring(err) or nil,
                termination = not ok and 'capture error' or (firstFailure and 'first physical failure' or 'protocol complete'),
            })
            player:sendEvent('play01Travel', { enabled = false })
            if mode.powered then ragdoll.setPowered(player, false) end
            if opponent and opponent:isValid() then
                ragdoll.setPowered(opponent, false)
                opponent:remove()
            end
            coroutine.yield()
            if not ok then error(err) end
        end)
        end
    end
end

return {
    engineHandlers = { onUpdate = testing.updateGlobal },
    eventHandlers = testing.globalEventHandlers,
}

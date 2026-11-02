local core = require('openmw.core')
local interfaces = require('openmw.interfaces')
local ragdoll = require('openmw.ragdolltest')
local testing = require('testing_util')
local types = require('openmw.types')
local util = require('openmw.util')
local world = require('openmw.world')
local testArea = require('ragdoll_test_area')
local playerDiagnostics = require('player_diagnostics')
local testConfig = require('test_config')

local saveLoadVictim
local gaitCommandAcknowledged
local testPlayerOrigin = testArea.player
local testActorOrigin = testArea.powered

local function waitFor(description, predicate, timeout)
    timeout = timeout or 5
    local simulationDeadline = core.getSimulationTime() + timeout
    local realDeadline = core.getRealTime() + math.max(timeout * 4, 10)
    repeat
        local value = predicate()
        if value then
            return value
        end
        coroutine.yield()
    until core.getSimulationTime() >= simulationDeadline or core.getRealTime() >= realDeadline
    error('Timed out waiting for ' .. description)
end

local function establishTestArea()
    local player = world.players[1]
    player:teleport(testArea.cell, testPlayerOrigin, { onGround = true })
    local position = waitFor('flat-floor test-area relocation', function()
        local current = player.position
        local horizontalOffset
            = util.vector3(current.x - testPlayerOrigin.x, current.y - testPlayerOrigin.y, 0)
        return player.cell.name == testArea.cell and horizontalOffset:length() <= 1 and current
    end)
    testing.runLocalTest(player, 'ragdoll flat floor geometry')
    return position
end

-- Every independently selectable test owns its spatial fixture. This keeps a
-- filtered run from depending on an earlier test having moved the player into
-- the flat-floor interior, and gives later tests a known cell for actors
-- teleported to testActorOrigin.
local nativeTestSeed = 903000
local function registerRagdollTest(name, fn)
    nativeTestSeed = nativeTestSeed + 1
    local replaySeed = nativeTestSeed
    testing.registerGlobalTest(name, function()
        establishTestArea()
        -- ragdoll_manual.omwscripts (openmw.cfg) stages and powers an auto-duel
        -- pair at every new game; the lifecycle counters must not see it.
        if interfaces.RagdollDev then interfaces.RagdollDev.autoDuel(false) end
        coroutine.yield()
        if testConfig.playerDiagnostics and name:find('^third person player') then
            playerDiagnostics.begin(name)
            local ok, message = pcall(fn)
            playerDiagnostics.finish(ok)
            if not ok then error(message) end
        else
            fn()
        end
    end)
end

registerRagdollTest('validates native Lua sampling prerequisites', function()
    testing.expectEqual(core.getGMST('ragdoll_missing_setting'), nil)
    testing.expectEqual(type(core.getGMST('fVanityDelay')), 'number')
    testing.expectEqual(type(core.getGMST('LightAttenuation_UseConstant')), 'number')
    testing.expectEqual(type(core.getGMST('LightAttenuation_LinearValue')), 'number')
    for _, key in ipairs({ 'FontColor_color_header', 'FontColor_color_normal',
                           'fontcolor_color_normal_over', 'fontcolor_color_normal_pressed' }) do
        local value = core.getGMST(key)
        testing.expectEqual(type(value), 'string', 'missing INI fallback ' .. key)
        util.color.commaString(value)
        testing.expectEqual(core.getGMST(key:upper()), value, 'case-sensitive fallback lookup')
    end
    testing.runLocalTest(world.players[1], 'ragdoll native sampling interfaces')
end)

local function expectFiniteVector(value, description)
    testing.expect(value.x == value.x, description .. '.x is NaN')
    testing.expect(value.y == value.y, description .. '.y is NaN')
    testing.expect(value.z == value.z, description .. '.z is NaN')
    testing.expect(math.abs(value.x) < math.huge, description .. '.x is infinite')
    testing.expect(math.abs(value.y) < math.huge, description .. '.y is infinite')
    testing.expect(math.abs(value.z) < math.huge, description .. '.z is infinite')
end

local function jsonString(value)
    local escapes = {
        ['"'] = '\\"',
        ['\\'] = '\\\\',
        ['\b'] = '\\b',
        ['\f'] = '\\f',
        ['\n'] = '\\n',
        ['\r'] = '\\r',
        ['\t'] = '\\t',
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
    for key in pairs(metrics) do
        table.insert(keys, key)
    end
    table.sort(keys)
    for _, key in ipairs(keys) do
        local value = metrics[key]
        local encoded
        if type(value) == 'number' then
            encoded = string.format('%.17g', value)
        elseif type(value) == 'boolean' then
            encoded = tostring(value)
        elseif type(value) == 'string' then
            encoded = jsonString(value)
        else
            error('Unsupported telemetry field type for ' .. key)
        end
        table.insert(fields, jsonString(key) .. ':' .. encoded)
    end
    print('RAGDOLL_TELEMETRY_JSON\t{' .. table.concat(fields, ',') .. '}')
end

local function findBody(state, name)
    for _, body in ipairs(state.bodies) do
        if body.name == name then
            return body
        end
    end
    error('Ragdoll state has no body named ' .. name)
end

local function inspectSupportTelemetry(support, description)
    expectFiniteVector(support.center, description .. ' support center')
    expectFiniteVector(support.groundCenter, description .. ' ground-contact center')
    testing.expect(support.footContactCount >= 0, description .. ' has a negative foot-contact count')
    testing.expect(support.groundContactCount >= support.footContactCount,
        description .. ' has fewer ground contacts than foot contacts')
    testing.expect(support.centerOfMassDistance == support.centerOfMassDistance,
        description .. ' center-of-mass distance is NaN')
    testing.expect(math.abs(support.centerOfMassDistance) < math.huge,
        description .. ' center-of-mass distance is infinite')
    testing.expect(support.centerOfMassHeight == support.centerOfMassHeight,
        description .. ' center-of-mass height is NaN')
    testing.expect(math.abs(support.centerOfMassHeight) < math.huge,
        description .. ' center-of-mass height is infinite')
    if support.centerOfMassSupported then
        testing.expect(support.leftFoot or support.rightFoot,
            description .. ' reports supported center of mass without a supporting foot')
    end
end

local function inspectControllerTelemetry(controller, description, powered)
    local validModes = {
        standing = true,
        bracing = true,
        stumbling = true,
        falling = true,
        fallen = true,
        getting_up = true,
        blend_back = true,
    }
    testing.expect(validModes[controller.mode], description .. ' has an invalid controller mode')
    for _, field in ipairs({ 'timeInMode', 'pelvisUpDot', 'torsoUpDot', 'torsoFacingUpDot', 'pelvisHeight', 'centerOfMassSpeed',
        'velocityTrackingError' }) do
        local value = controller[field]
        testing.expect(value == value, string.format('%s %s is NaN', description, field))
        testing.expect(math.abs(value) < math.huge, string.format('%s %s is infinite', description, field))
    end
    testing.expect(controller.pelvisUpDot >= -1 and controller.pelvisUpDot <= 1,
        description .. ' pelvis up-dot is out of range')
    testing.expect(controller.torsoUpDot >= -1 and controller.torsoUpDot <= 1,
        description .. ' torso up-dot is out of range')
    testing.expect(controller.torsoFacingUpDot >= -1 and controller.torsoFacingUpDot <= 1,
        description .. ' torso facing up-dot is out of range')
    testing.expect(controller.jointSupportMotorUtilization >= 0 and controller.jointSupportMotorUtilization <= 1,
        description .. ' joint support exceeded its motor budget')
    if controller.jointSupportApplied then
        testing.expect(not (controller.upright and controller.upright.applied),
            description .. ' joint support was combined with external upright assistance')
    end
    expectFiniteVector(controller.centerOfMassVelocity, description .. ' center-of-mass velocity')
    expectFiniteVector(controller.targetHorizontalVelocity, description .. ' target horizontal velocity')
    local validGaitPhases = { idle = true, left_stance = true, right_stance = true }
    testing.expect(validGaitPhases[controller.gaitPhase], description .. ' has an invalid gait phase')
    for _, field in ipairs({ 'gaitTimeInPhase', 'gaitPhaseProgress',
        'gaitSwingContactCandidateTime' }) do
        local value = controller[field]
        testing.expect(value == value, string.format('%s %s is NaN', description, field))
        testing.expect(math.abs(value) < math.huge,
            string.format('%s %s is infinite', description, field))
    end
    testing.expect(controller.gaitPhaseProgress >= 0 and controller.gaitPhaseProgress <= 1,
        description .. ' gait phase progress is out of range')
    if controller.swingFootTarget then
        expectFiniteVector(controller.swingFootTarget, description .. ' swing-foot target')
    end
    if controller.swingFootTrajectoryModel then
        expectFiniteVector(controller.swingFootTrajectoryModel, description .. ' swing-foot trajectory model')
    end
    if controller.swingFootResolvedModel then
        expectFiniteVector(controller.swingFootResolvedModel, description .. ' swing-foot resolved model')
    end
    if controller.swingFootPoseModel then
        expectFiniteVector(controller.swingFootPoseModel, description .. ' swing-foot pose model')
    end
    if controller.swingFootControllerWorldTarget then
        expectFiniteVector(controller.swingFootControllerWorldTarget,
            description .. ' swing-foot controller world target')
    end
    testing.expectAlmostEqual(controller.targetHorizontalVelocity.z, 0,
        description .. ' target velocity is not horizontal')
    if controller.mode == 'fallen' then
        testing.expect(not controller.driveJointMotors,
            description .. ' fallen observation unexpectedly requests joint motors')
    end
    if not powered then
        testing.expect(not controller.jointMotorsApplied,
            description .. ' passive joint motors were applied')
        testing.expect(not controller.jointSupportApplied,
            description .. ' passive joint support was applied')
    end
    if controller.upright then
        local upright = controller.upright
        for _, field in ipairs({ 'pelvisForce', 'pelvisTorque', 'spineTorque' }) do
            expectFiniteVector(upright[field], description .. ' upright ' .. field)
        end
        testing.expect(upright.targetPelvisHeight >= 0,
            description .. ' has an invalid upright target pelvis height')
        testing.expectLessOrEqual(upright.pelvisForce:length(), 150001,
            description .. ' dry-run pelvis force exceeded its profile bound')
        testing.expectLessOrEqual(upright.pelvisTorque:length(), 1200001,
            description .. ' dry-run pelvis torque exceeded its profile bound')
        testing.expectLessOrEqual(upright.spineTorque:length(), 800001,
            description .. ' dry-run spine torque exceeded its profile bound')
        if not powered then
            testing.expect(not upright.applied, description .. ' passive dry-run upright control was applied')
        end
    end
end

local function inspectTelemetry(state, description)
    expectFiniteVector(state.centerOfMass, description .. ' center of mass')
    testing.expectLessOrEqual(math.abs(state.totalMass - 75), 0.01,
        description .. ' total mass differs from the configured mass')
    local bodyMass = 0
    for index, body in ipairs(state.bodies) do
        expectFiniteVector(body.centerOfMass, string.format('%s body %d center of mass', description, index))
        expectFiniteVector(body.linearVelocity, string.format('%s body %d linear velocity', description, index))
        expectFiniteVector(body.angularVelocity, string.format('%s body %d angular velocity', description, index))
        testing.expectGreaterThan(body.mass, 0, string.format('%s body %d has invalid mass', description, index))
        local rotatedAxis = body.rotation * util.vector3(1, 0, 0)
        expectFiniteVector(rotatedAxis, string.format('%s body %d rotation', description, index))
        testing.expectAlmostEqual(rotatedAxis:length(), 1,
            string.format('%s body %d rotation is not normalized', description, index))
        bodyMass = bodyMass + body.mass
    end
    testing.expectLessOrEqual(math.abs(bodyMass - state.totalMass), 0.01,
        description .. ' per-body mass does not sum to total mass')
    testing.expectEqual(#state.constraints, 19, description .. ' has an unexpected constraint count')
    local maximumAnchorSeparation = 0
    local maximumAnchorConstraint = 'none'
    local maximumHingeViolation = 0
    local hingeCount = 0
    for index, constraint in ipairs(state.constraints) do
        testing.expect(constraint.parentBody ~= '', string.format('%s constraint %d has no parent name', description, index))
        testing.expect(constraint.childBody ~= '', string.format('%s constraint %d has no child name', description, index))
        testing.expect(constraint.anchorSeparation == constraint.anchorSeparation,
            string.format('%s constraint %d anchor separation is NaN', description, index))
        if constraint.anchorSeparation > maximumAnchorSeparation then
            maximumAnchorSeparation = constraint.anchorSeparation
            maximumAnchorConstraint = constraint.parentBody .. '->' .. constraint.childBody
        end
        if constraint.hingeAngle then
            hingeCount = hingeCount + 1
            maximumHingeViolation = math.max(maximumHingeViolation,
                constraint.hingeMinimum - constraint.hingeAngle,
                constraint.hingeAngle - constraint.hingeMaximum)
        end
    end
    testing.expectEqual(hingeCount, 4, description .. ' has an unexpected hinge count')
    return maximumAnchorSeparation, math.max(0, maximumHingeViolation), maximumAnchorConstraint
end

registerRagdollTest('ragdoll suite establishes verified flat floor test area', function()
    local player = world.players[1]
    testing.expectEqual(player.cell.name, testArea.cell, 'ragdoll suite player is in the wrong cell')
    testing.expect(not player.cell.isExterior, 'ragdoll suite loaded exterior terrain')
    local horizontalOffset
        = util.vector3(player.position.x - testPlayerOrigin.x, player.position.y - testPlayerOrigin.y, 0)
    testing.expectLessOrEqual(horizontalOffset:length(), 1,
        'save/load fixture did not preserve the flat floor test area')
    testing.expectLessOrEqual(math.abs(player.position.z - testArea.floorHeight), 5,
        'player was not grounded on the flat floor')
end)

testing.registerGlobalTestStep('ragdoll save load - prepare dead actor', function()
    local position = establishTestArea()
    testing.expectLessOrEqual(math.abs(position.z - testArea.floorHeight), 5,
        'flat floor ground snap produced an implausible height')
    local player = world.players[1]
    local victim = world.createObject(types.NPC.record(player).id)
    victim:teleport(player.cell, player.position + util.vector3(0, 200, 0))
    coroutine.yield()
    saveLoadVictim = victim
    ragdoll.triggerDeath(victim)
    -- NPCs are powered from spawn now, so a ragdoll state exists before the
    -- death registers: wait for the death itself, then the dead body's state.
    local state = waitFor('actor ragdoll before save', function()
        return types.Actor.isDead(victim) and ragdoll.getState(victim)
    end)
    testing.expectEqual(#state.bodies, 20, 'pre-save actor ragdoll has an unexpected body count')
    testing.expect(types.Actor.isDead(victim), 'actor is not dead before ragdoll save')
end)

testing.registerGlobalTestStep('ragdoll save load - clear live state', function()
    testing.expect(saveLoadVictim and saveLoadVictim:isValid(), 'pre-load actor reference is invalid')
    ragdoll.resurrect(saveLoadVictim)
    waitFor('pre-load actor ragdoll cleanup', function()
        -- A living NPC may be powered again at once (NPCs physical in play):
        -- no passive (dead) ragdoll is what counts.
        local state = ragdoll.getState(saveLoadVictim)
        return (state == nil or state.powered) and not types.Actor.isDead(saveLoadVictim)
    end)
    saveLoadVictim = nil
end)

testing.registerGlobalTestStep('ragdoll save load - verify restored actor', function()
    local player = world.players[1]
    local victim
    for _, candidate in ipairs(world.getObjectsByRecordId(types.NPC.record(player).id, nil, true)) do
        if candidate.id ~= player.id and types.Actor.isDead(candidate) then
            victim = candidate
            break
        end
    end
    testing.expect(victim ~= nil, 'loading the save did not restore the dead actor')
    local state = waitFor('actor ragdoll recreation after load', function()
        return ragdoll.getState(victim)
    end)
    testing.expectEqual(#state.bodies, 20, 'loaded actor ragdoll has an unexpected body count')
    for index, body in ipairs(state.bodies) do
        expectFiniteVector(body.position, string.format('loaded actor body %d position', index))
    end
    testing.expectEqual(state.pendingImpact, false, 'loaded actor retained a stale killing impact')

    ragdoll.resurrect(victim)
    waitFor('loaded actor ragdoll cleanup', function()
        local state = ragdoll.getState(victim)
        return (state == nil or state.powered) and not types.Actor.isDead(victim)
    end)
end)

-- Plan phase 6: a save made mid-fight loads back into a fight. The
-- fighters are powered by the population policy, not by the test, after load.
local fightSaveNames = { 'RAGDOLL_SAVE_FIGHTER_A', 'RAGDOLL_SAVE_FIGHTER_B' }
local fightSaveActors = {}
testing.registerGlobalTestStep('ragdoll save load - prepare fight', function()
    establishTestArea()
    local player = world.players[1]
    fightSaveActors = {}
    for index, name in ipairs(fightSaveNames) do
        local record = world.createRecord(types.NPC.createRecordDraft({
            template = types.NPC.record(player), name = name,
            isEssential = false, isPersistent = true, isRespawning = false,
        }))
        local actor = world.createObject(record.id)
        world.createObject('iron dagger', 1):moveInto(types.Actor.inventory(actor))
        actor:teleport(player.cell, player.position + util.vector3(index == 1 and -40 or 40, 220, 0), { onGround = true })
        fightSaveActors[index] = actor
    end
    coroutine.yield()
    ragdoll.setCombatTarget(fightSaveActors[1], fightSaveActors[2])
    ragdoll.setCombatTarget(fightSaveActors[2], fightSaveActors[1])
    for _, actor in ipairs(fightSaveActors) do
        ragdoll.setPowered(actor, true)
        waitFor('fighter powered before save', function()
            local state = ragdoll.getState(actor)
            return state and state.powered and state.controller and state.controller.mode == 'standing'
        end, 15)
    end
end)

testing.registerGlobalTestStep('ragdoll save load - clear fight', function()
    for _, actor in ipairs(fightSaveActors) do
        if actor:isValid() then
            ragdoll.setPowered(actor, false)
            actor:remove()
        end
    end
    fightSaveActors = {}
    coroutine.yield()
end)

testing.registerGlobalTestStep('ragdoll save load - verify fight', function()
    local found = {}
    for _, name in ipairs(fightSaveNames) do
        for _, candidate in ipairs(world.activeActors) do
            if candidate.type == types.NPC and types.NPC.record(candidate).name == name then
                found[#found + 1] = candidate
            end
        end
    end
    testing.expectEqual(#found, 2, 'loading the fight save did not restore both fighters')
    -- The population policy (OPENMW_RAGDOLL_POWER_NPCS, on in Run-Game.cmd)
    -- powers them by itself; without it, power them here - the restore is
    -- what is under test.
    local deadline = core.getSimulationTime() + 3
    repeat coroutine.yield() until core.getSimulationTime() >= deadline
    for _, actor in ipairs(found) do
        local current = ragdoll.getState(actor)
        if not (current and current.powered) then ragdoll.setPowered(actor, true) end
    end
    for _, actor in ipairs(found) do
        local state = waitFor('fighter powered after load', function()
            local current = ragdoll.getState(actor)
            return current and current.powered and current.controller
                and (current.controller.mode == 'standing' or current.controller.mode == 'bracing') and current
        end, 15)
        testing.expectGreaterOrEqual(#state.bodies, 20, 'loaded fighter has too few bodies')
        for index, body in ipairs(state.bodies) do
            expectFiniteVector(body.position, string.format('loaded fighter body %d position', index))
        end
        print(string.format('RAGDOLL_SAVE_FIGHT restored=%s mode=%s bodies=%d', tostring(actor.id),
            state.controller.mode, #state.bodies))
    end
    for _, actor in ipairs(found) do
        ragdoll.setPowered(actor, false)
        actor:remove()
    end
end)

registerRagdollTest('evaluated animation motion transfers to new ragdoll bodies', function()
    local player = world.players[1]
    local victim = world.createObject(types.NPC.record(player).id)
    victim:teleport(player.cell, player.position + util.vector3(0, 180, 0))
    coroutine.yield()
    coroutine.yield()
    coroutine.yield()

    local before = ragdoll.getCounters()
    testing.expectGreaterThan(before.motionSamples, 0, 'no evaluated skeleton poses were sampled')
    ragdoll.triggerDeath(victim)
    local state = waitFor('motion-transfer ragdoll creation', function()
        return ragdoll.getState(victim)
    end)
    testing.expect(state.inheritedAnimationMotion, 'ragdoll did not inherit per-body animation motion')
    local after = ragdoll.getCounters()
    testing.expectEqual(after.motionTransfers, before.motionTransfers + 1,
        'animation-motion transfer counter did not advance')

    ragdoll.resurrect(victim)
    waitFor('motion-transfer ragdoll cleanup', function()
        return ragdoll.getState(victim) == nil
    end)
end)

registerRagdollTest('passive ragdoll lifecycle and impulse', function()
    local player = world.players[1]
    local victim = world.createObject(types.NPC.record(player).id)
    victim:teleport(player.cell, player.position + util.vector3(0, 180, 0))
    coroutine.yield()

    testing.expectEqual(ragdoll.getState(victim), nil, 'new actor unexpectedly owns a ragdoll')
    local before = ragdoll.getCounters()

    local hitPoint = victim.position + util.vector3(0, 0, 100)
    ragdoll.triggerDeath(victim, {
        hitPoint = hitPoint,
        direction = util.vector3(0, 1, 0.2),
        damage = 25,
    })

    local state = waitFor('ragdoll creation', function()
        return ragdoll.getState(victim)
    end)
    testing.expectEqual(#state.bodies, 20, 'unexpected humanoid ragdoll body count')
    testing.expectEqual(state.pendingImpact, false, 'killing impulse was not consumed')
    local initialAnchorSeparation, initialHingeViolation = inspectTelemetry(state, 'initial ragdoll')
    testing.expectLessOrEqual(initialAnchorSeparation, 2, 'initial ragdoll constraint anchors separated excessively')
    testing.expectLessOrEqual(initialHingeViolation, 0.1, 'initial ragdoll exceeded a hinge limit')
    for index, body in ipairs(state.bodies) do
        testing.expect(body.name ~= '', 'ragdoll body has no profile name')
        expectFiniteVector(body.position, string.format('body %d position', index))
        expectFiniteVector(body.targetPosition, string.format('body %d target position', index))
        expectFiniteVector(body.renderPosition, string.format('body %d render position', index))
        expectFiniteVector(body.linearVelocity, string.format('body %d linear velocity', index))
        expectFiniteVector(body.angularVelocity, string.format('body %d angular velocity', index))
    end

    state = waitFor('animation-to-physics mode transition', function()
        local current = ragdoll.getState(victim)
        if current and current.modeTransitionFactor >= 1 then
            return current
        end
    end)
    testing.expectEqual(state.modeTransitionFactor, 1, 'ragdoll mode transition did not complete')

    local initialVelocity = state.bodies[1].linearVelocity
    local recordingStartStep = state.physicsStep
    ragdoll.startReplayRecording(victim, 12345)
    ragdoll.applyImpulse(victim, 1, util.vector3(500, 0, 0), state.bodies[1].position)
    state = waitFor('test impulse response', function()
        local current = ragdoll.getState(victim)
        -- Applying an impulse changes velocity immediately, before Jolt has
        -- advanced or the post-step replay recorder has captured a frame.
        if current and current.physicsStep > recordingStartStep
            and (current.bodies[1].linearVelocity - initialVelocity):length() > 0.1 then
            return current
        end
    end)
    testing.expect(state.bodies[1].active, 'impulse did not activate the selected body')
    local replay = ragdoll.finishReplayRecording()
    testing.expect(replay ~= nil, 'fixed-step replay recording failed')
    testing.expectEqual(replay.version, 10, 'unexpected replay format version')
    testing.expectEqual(replay.seed, '12345', 'replay seed was not preserved exactly')
    testing.expect(replay.profileHash ~= '0' and replay.profileHash ~= '', 'replay profile hash is missing')
    testing.expectAlmostEqual(replay.fixedTimeStep, 1 / 60, 'replay timestep differs from Jolt fixed step')
    testing.expectGreaterThan(replay.workerThreadCount, 0, 'replay worker-thread configuration is missing')
    testing.expectGreaterThan(replay.collisionSteps, 0, 'replay collision-step configuration is missing')
    testing.expectEqual(replay.inputCount, 0, 'passive replay unexpectedly contains controller inputs')
    testing.expectEqual(replay.controllerStateCount, 0, 'passive replay unexpectedly contains controller state')
    testing.expectEqual(replay.bodyCount, 20, 'replay body insertion order is incomplete')
    testing.expectGreaterThan(replay.frameCount, 0, 'replay contains no fixed-step frames')
    testing.expectGreaterThan(replay.byteCount, 56, 'replay binary contains only its header')
    testing.expectEqual(#replay.data, replay.byteCount, 'passive replay binary was not returned intact')

    local afterCreate = ragdoll.getCounters()
    testing.expectEqual(afterCreate.current, before.current + 1, 'current ragdoll counter did not advance')
    testing.expectEqual(afterCreate.created, before.created + 1, 'created ragdoll counter did not advance')
    testing.expectEqual(afterCreate.poseFailures, before.poseFailures, 'pose synchronization failed')

    ragdoll.resurrect(victim)
    waitFor('ragdoll removal after resurrection', function()
        return ragdoll.getState(victim) == nil
    end)
    local afterResurrection = ragdoll.getCounters()
    testing.expectEqual(afterResurrection.current, before.current, 'ragdoll leaked after resurrection')
    testing.expectEqual(afterResurrection.removed, before.removed + 1, 'removed ragdoll counter did not advance')
end)

registerRagdollTest('COM-01 combat state mirrors the powered controller', function()
    local player = world.players[1]
    local victim = world.createObject(types.NPC.record(player).id)
    victim:teleport(player.cell, testActorOrigin, { onGround = true })
    coroutine.yield()
    testing.expectEqual(ragdoll.getCombatState(victim), nil, 'ordinary actor invented a combat mode')
    ragdoll.setPowered(victim, true)
    waitFor('COM-01 first controller observation', function()
        return ragdoll.getCombatState(victim)
    end)
    local firstStep
    local lastStep
    for i = 1, 30 do
        local combat = ragdoll.getCombatState(victim)
        local debug = ragdoll.getState(victim)
        testing.expect(combat and debug and debug.controller, 'missing powered combat observation')
        testing.expectEqual(combat.mode, debug.controller.mode, 'combat mode diverged from controller')
        testing.expectEqual(combat.timeInMode, debug.controller.timeInMode, 'combat timer diverged')
        testing.expectEqual(combat.physicsStep, debug.physicsStep, 'combat observation has wrong physics step')
        firstStep = firstStep or combat.physicsStep
        testing.expect(combat.physicsStep >= (lastStep or firstStep), 'combat physics step moved backwards')
        lastStep = combat.physicsStep
        coroutine.yield()
    end
    testing.expect(lastStep > firstStep, 'COM-01 did not observe any new fixed steps')
    ragdoll.setPowered(victim, false)
    waitFor('COM-01 powered actor removal', function() return ragdoll.getState(victim) == nil end)
    testing.expectEqual(ragdoll.getCombatState(victim), nil, 'disabled actor retained a combat mode')
    ragdoll.triggerDeath(victim)
    waitFor('COM-01 passive corpse creation', function() return ragdoll.getState(victim) end)
    testing.expectEqual(ragdoll.getCombatState(victim), nil, 'passive corpse invented a powered combat mode')
    ragdoll.resurrect(victim)
    waitFor('COM-01 passive corpse removal', function() return ragdoll.getState(victim) == nil end)
    testing.expectEqual(ragdoll.getCombatState(victim), nil, 'resurrection retained a combat mode')
    victim:remove()
end)

registerRagdollTest('COM-02 identified melee reactions preserve hit controls', function()
    local function spawnReactionActor(offset)
        local player = world.players[1]
        local victim = world.createObject(types.NPC.record(player).id)
        victim:teleport(player.cell, testActorOrigin + util.vector3(offset, 0, 0), { onGround = true })
        coroutine.yield()
        ragdoll.setPowered(victim, true)
        return victim, waitFor('COM-02 powered actor', function()
            local state = ragdoll.getState(victim)
            return state and state.controller and state.controller.mode == 'standing' and state
        end)
    end

    local function finishReactionActor(victim)
        ragdoll.setPowered(victim, false)
        waitFor('COM-02 reaction actor cleanup', function() return ragdoll.getState(victim) == nil end)
        victim:remove()
        coroutine.yield()
    end

    local function waitForCount(victim, count)
        return waitFor('COM-02 reaction count', function()
            local state = ragdoll.getState(victim)
            return state and state.combatReactionCount == count and state
        end)
    end

    -- Miss control: without a landed-hit command the ledger and controller stay unchanged.
    local miss, missState = spawnReactionActor(-180)
    local missStartStep = missState.physicsStep
    for _ = 1, 12 do coroutine.yield() end
    missState = ragdoll.getState(miss)
    testing.expectEqual(missState.combatReactionCount, 0, 'miss emitted a combat reaction')
    testing.expectEqual(missState.lastCombatReaction, nil, 'miss invented reaction data')
    testing.expectEqual(missState.controller.mode, 'standing', 'miss changed controller mode')
    testing.expect(missState.physicsStep > missStartStep, 'miss control did not advance physics')
    finishReactionActor(miss)

    -- Low-force landed contact is identified and applied, but remains below hit-stun onset.
    local low, lowState = spawnReactionActor(-60)
    local lowPoint = findBody(lowState, 'pelvis').position + util.vector3(0, 0, 10)
    ragdoll.applyCombatImpulse(low, util.vector3(200, 0, 0), lowPoint, 2001)
    lowState = waitForCount(low, 1)
    local lowReaction = lowState.lastCombatReaction
    testing.expectEqual(lowReaction.id, '2001', 'low-force reaction lost its identity')
    testing.expectAlmostEqual((lowReaction.point - lowPoint):length(), 0,
        'low-force reaction changed its requested contact point')
    testing.expectAlmostEqual((lowReaction.impulse - util.vector3(200, 0, 0)):length(), 0,
        'low-force reaction changed its impulse')
    testing.expectEqual(lowReaction.stunSeconds, 0, 'sub-threshold reaction started hit stun')
    local lowOutcome = lowState.controller.mode
    for _ = 1, 30 do
        coroutine.yield()
        lowState = ragdoll.getState(low)
        if lowState.controller.mode ~= 'standing' then lowOutcome = lowState.controller.mode end
    end
    finishReactionActor(low)

    -- A calibrated landed hit carries point, impulse and derived stun into one record.
    local hit, hitState = spawnReactionActor(60)
    local hitPoint = findBody(hitState, 'pelvis').position + util.vector3(0, 0, 15)
    ragdoll.applyCombatImpulse(hit, util.vector3(1500, 0, 300), hitPoint, 3001)
    hitState = waitForCount(hit, 1)
    local hitReaction = hitState.lastCombatReaction
    testing.expectEqual(hitReaction.id, '3001', 'landed hit lost its identity')
    testing.expectGreaterThan(hitReaction.stunSeconds, 0, 'landed hit did not carry hit stun')
    testing.expectAlmostEqual((hitReaction.point - hitPoint):length(), 0,
        'landed hit changed its requested contact point')
    testing.expectAlmostEqual((hitReaction.impulse - util.vector3(1500, 0, 300)):length(), 0,
        'landed hit changed its impulse')
    local hitOutcome = hitState.controller.mode
    for _ = 1, 120 do
        coroutine.yield()
        hitState = ragdoll.getState(hit)
        if hitState.controller.mode ~= 'standing' then hitOutcome = hitState.controller.mode end
        if hitOutcome == 'falling' or hitOutcome == 'fallen' then break end
    end
    finishReactionActor(hit)

    -- Two distinct shoves are accepted; retrying the second ID cannot apply a third reaction.
    local repeated, repeatedState = spawnReactionActor(180)
    local repeatedPoint = findBody(repeatedState, 'pelvis').position + util.vector3(0, 0, 15)
    ragdoll.applyCombatImpulse(repeated, util.vector3(1500, 0, 300), repeatedPoint, 4001)
    repeatedState = waitForCount(repeated, 1)
    for _ = 1, 30 do coroutine.yield() end
    ragdoll.applyCombatImpulse(repeated, util.vector3(1500, 0, 300), repeatedPoint, 4002)
    repeatedState = waitForCount(repeated, 2)
    ragdoll.applyCombatImpulse(repeated, util.vector3(1500, 0, 300), repeatedPoint, 4002)
    for _ = 1, 12 do coroutine.yield() end
    repeatedState = ragdoll.getState(repeated)
    testing.expectEqual(repeatedState.combatReactionCount, 2,
        'duplicate reaction identity applied an extra shove')
    testing.expectEqual(repeatedState.lastCombatReaction.id, '4002',
        'double shove did not preserve its second identity')
    local repeatedOutcome = repeatedState.controller.mode
    finishReactionActor(repeated)

    emitTelemetry('combat_reaction_matrix', {
        miss_count = 0, miss_mode = missState.controller.mode,
        low_count = 1, low_stun = lowReaction.stunSeconds, low_mode = lowOutcome,
        hit_count = 1, hit_stun = hitReaction.stunSeconds, hit_mode = hitOutcome,
        double_count = 2, double_last_id = repeatedState.lastCombatReaction.id,
        double_mode = repeatedOutcome,
        point_error = (hitReaction.point - hitPoint):length(),
        impulse_error = (hitReaction.impulse - util.vector3(1500, 0, 300)):length(),
    })
end)

registerRagdollTest('powered living ragdoll follows animation and restores ordinary collision', function()
    local player = world.players[1]
    local victim = world.createObject(types.NPC.record(player).id)
    victim:teleport(player.cell, testActorOrigin, { onGround = true })
    coroutine.yield()

    local before = ragdoll.getCounters()
    ragdoll.setPowered(victim, true)
    local initial = waitFor('powered living ragdoll creation', function()
        return ragdoll.getState(victim)
    end)
    testing.expect(initial.powered, 'living ragdoll was not marked as powered')
    testing.expect(not initial.inheritedAnimationMotion,
        'powered activation incorrectly inherited per-body animation motion')
    testing.expect(not types.Actor.isDead(victim), 'powered ragdoll unexpectedly killed its actor')
    testing.expectEqual(#initial.bodies, 20, 'powered humanoid ragdoll has an unexpected body count')
    ragdoll.startReplayRecording(victim, 67890)

    local observedJointMotors = false
    local observedJointSupport = false
    local maximumTargetError = 0
    local maximumTargetMotion = 0
    local maximumLinearSpeed = 0
    local minimumPelvisUpDot = 1
    local minimumTorsoUpDot = 1
    local standingFrames = 0
    local bracingFrames = 0
    local stumblingFrames = 0
    local fallingFrames = 0
    local fallenFrames = 0
    local initialTargetOffsets = {}
    for index, body in ipairs(initial.bodies) do
        initialTargetOffsets[index] = body.targetPosition - initial.referencePosition
    end
    for frame = 1, 240 do
        local state = ragdoll.getState(victim)
        testing.expect(state ~= nil, 'powered living ragdoll fell back unexpectedly')
        testing.expect(state.powered, 'powered living ragdoll changed mode unexpectedly')
        testing.expect(not types.Actor.isDead(victim), 'powered living ragdoll actor died unexpectedly')
        inspectTelemetry(state, 'powered living ragdoll')
        if state.support then
            inspectSupportTelemetry(state.support, 'powered living ragdoll')
        end
        if state.controller then
            inspectControllerTelemetry(state.controller, 'powered living ragdoll', true)
            minimumPelvisUpDot = math.min(minimumPelvisUpDot, state.controller.pelvisUpDot)
            minimumTorsoUpDot = math.min(minimumTorsoUpDot, state.controller.torsoUpDot)
            if state.controller.mode == 'standing' then
                standingFrames = standingFrames + 1
            elseif state.controller.mode == 'bracing' then
                bracingFrames = bracingFrames + 1
            elseif state.controller.mode == 'stumbling' then
                stumblingFrames = stumblingFrames + 1
            elseif state.controller.mode == 'falling' then
                fallingFrames = fallingFrames + 1
            elseif state.controller.mode == 'fallen' then
                fallenFrames = fallenFrames + 1
            end
            observedJointMotors = observedJointMotors or state.controller.jointMotorsApplied
            -- kinematic_root idle (since 55bc8234ed) is the animation-assisted
            -- branch: no joint-support plan; the upright controller reacts
            -- through the supported feet.
            observedJointSupport = observedJointSupport
                or (state.controller.animationAssistedActuation and state.controller.upright ~= nil
                    and state.controller.upright.applied and state.support ~= nil
                    and (state.support.leftFoot or state.support.rightFoot))
        end
        for index, body in ipairs(state.bodies) do
            maximumTargetError = math.max(maximumTargetError, (body.position - body.targetPosition):length())
            maximumTargetMotion = math.max(maximumTargetMotion,
                ((body.targetPosition - state.referencePosition) - initialTargetOffsets[index]):length())
            maximumLinearSpeed = math.max(maximumLinearSpeed, body.linearVelocity:length())
        end
        if frame == 1 or frame % 15 == 0 then
            local controller = state.controller
            local support = state.support
            local upright = controller and controller.upright
            emitTelemetry('powered_idle_frame', {
                frame = frame,
                mode = controller and controller.mode or 'missing',
                pelvis_up = controller and controller.pelvisUpDot or -2,
                torso_up = controller and controller.torsoUpDot or -2,
                pelvis_height = controller and controller.pelvisHeight or -1,
                com_speed = controller and controller.centerOfMassSpeed or -1,
                left_supported = support and support.leftFoot or false,
                right_supported = support and support.rightFoot or false,
                com_supported = support and support.centerOfMassSupported or false,
                com_distance = support and support.centerOfMassDistance or -1,
                com_x = state.centerOfMass.x,
                com_y = state.centerOfMass.y,
                support_x = support and support.center.x or 0,
                support_y = support and support.center.y or 0,
                pelvis_force_x = upright and upright.pelvisForce.x or 0,
                pelvis_force_y = upright and upright.pelvisForce.y or 0,
                pelvis_force_z = upright and upright.pelvisForce.z or 0,
                pelvis_torque_x = upright and upright.pelvisTorque.x or 0,
                pelvis_torque_y = upright and upright.pelvisTorque.y or 0,
                pelvis_torque_z = upright and upright.pelvisTorque.z or 0,
                spine_torque_x = upright and upright.spineTorque.x or 0,
                spine_torque_y = upright and upright.spineTorque.y or 0,
                spine_torque_z = upright and upright.spineTorque.z or 0,
            })
        end
        coroutine.yield()
    end

    -- Finish before evaluating behavioural gates so a red run still leaves a
    -- complete input/state capture for the offline diagnostic harness.
    local replay = ragdoll.finishReplayRecording()
    testing.expect(observedJointMotors, 'powered living ragdoll never applied joint motors')
    testing.expect(observedJointSupport, 'powered living ragdoll never applied animation drive with foot-grounded upright control')
    emitTelemetry('powered_idle_gate', {
        min_pelvis_up = minimumPelvisUpDot,
        min_torso_up = minimumTorsoUpDot,
        standing_frames = standingFrames,
        bracing_frames = bracingFrames,
        stumbling_frames = stumblingFrames,
        falling_frames = fallingFrames,
        fallen_frames = fallenFrames,
        max_target_error = maximumTargetError,
        max_target_motion = maximumTargetMotion,
        max_linear_speed = maximumLinearSpeed,
    })
    -- The idle clip does not translate these bound body origins materially in
    -- actor-root space. World targets may move with the synchronized root; a
    -- large local drift indicates physics fed back into the animation sample.
    testing.expectLessOrEqual(maximumTargetMotion, 10,
        'powered living ragdoll animation target accumulated physics feedback')
    testing.expectLessOrEqual(maximumTargetError, 500,
        'powered living ragdoll diverged excessively from its animation targets')
    testing.expectLessOrEqual(maximumLinearSpeed, 2000,
        'powered living ragdoll reached an unsafe linear speed')
    testing.expectEqual(fallingFrames, 0,
        'powered idle entered falling while acquiring or retaining foot support')
    testing.expectEqual(fallenFrames, 0,
        'powered idle reached fallen without an external perturbation')
    testing.expectGreaterOrEqual(minimumPelvisUpDot, 0.9,
        'powered idle pelvis left the authored upright range')
    testing.expectGreaterOrEqual(minimumTorsoUpDot, 0.9,
        'powered idle torso left the authored upright range')

    testing.expect(replay ~= nil, 'powered fixed-step replay recording failed')
    testing.expectEqual(replay.version, 10, 'unexpected powered replay format version')
    testing.expectEqual(replay.seed, '67890', 'powered replay seed was not preserved exactly')
    -- velocity, raw velocity (e0b65b42b7), facing and the recovery-finished
    -- flag (159fe7a690), then 20 target transforms of 7 floats
    testing.expectEqual(replay.inputCount, 150,
        'powered replay did not capture velocity, facing plus 20 animation target transforms')
    testing.expectEqual(replay.controllerStateCount, 10,
        'powered replay did not capture the controller/phase alignment channels')
    testing.expectEqual(replay.bodyCount, 20, 'powered replay body insertion order is incomplete')
    testing.expectGreaterThan(replay.frameCount, 0, 'powered replay contains no fixed-step frames')
    testing.expectGreaterThan(replay.byteCount, 56, 'powered replay binary contains only its header')
    testing.expectEqual(#replay.data, replay.byteCount, 'powered replay binary was not returned intact')

    ragdoll.setPowered(victim, false)
    waitFor('powered living ragdoll removal', function()
        return ragdoll.getState(victim) == nil
    end)
    testing.expect(not types.Actor.isDead(victim), 'disabling powered ragdoll killed its actor')
    local after = ragdoll.getCounters()
    testing.expectEqual(after.current, before.current, 'powered living ragdoll leaked after disable')
    testing.expectEqual(after.created, before.created + 1, 'powered living ragdoll creation was not counted')
    testing.expectEqual(after.removed, before.removed + 1, 'powered living ragdoll removal was not counted')
    testing.expectEqual(after.poseFailures, before.poseFailures, 'powered living ragdoll control failed')
    emitTelemetry('powered_idle', {
        max_target_error = maximumTargetError,
        max_target_motion = maximumTargetMotion,
        max_linear_speed = maximumLinearSpeed,
        min_pelvis_up = minimumPelvisUpDot,
        min_torso_up = minimumTorsoUpDot,
        falling_frames = fallingFrames,
        fallen_frames = fallenFrames,
        joint_support_applied = observedJointSupport,
    })
    victim:remove()
end)

registerRagdollTest('powered locomotion rotates actor-local input into world space', function()
    local player = world.players[1]
    local victim = world.createObject(types.NPC.record(player).id)
    victim:teleport(player.cell, testActorOrigin, {
        onGround = true,
        rotation = util.transform.rotateZ(math.rad(90)),
    })
    coroutine.yield()
    victim:addScript('gait_actor.lua')
    coroutine.yield()

    local before = ragdoll.getCounters()
    ragdoll.setPowered(victim, true)
    waitFor('rotated powered actor creation', function()
        local state = ragdoll.getState(victim)
        return state and state.powered and state.controller
            and state.controller.mode == 'standing' and state
    end)

    gaitCommandAcknowledged = false
    victim:sendEvent('startRagdollTravel', 1)
    waitFor('rotated gait movement command', function()
        return gaitCommandAcknowledged
    end)
    local moving = waitFor('rotated world-space locomotion target', function()
        local state = ragdoll.getState(victim)
        return state and state.controller
            and state.controller.targetHorizontalVelocity:length() > 40 and state
    end)
    local target = moving.controller.targetHorizontalVelocity
    emitTelemetry('powered_world_velocity', {
        target_velocity_x = target.x,
        target_velocity_y = target.y,
        target_speed = target:length(),
    })
    testing.expectGreaterThan(target.x, 40,
        'positive local forward input did not rotate onto the actor world heading')
    testing.expectLessOrEqual(math.abs(target.y), 5,
        'rotated local forward input retained too much unrotated world Y velocity')

    victim:sendEvent('startRagdollTravel', 0)
    ragdoll.setPowered(victim, false)
    waitFor('rotated powered actor cleanup', function()
        return ragdoll.getState(victim) == nil
    end)
    local after = ragdoll.getCounters()
    testing.expectEqual(after.current, before.current,
        'rotated powered locomotion test leaked a ragdoll')
    testing.expectEqual(after.created, before.created + 1,
        'rotated powered locomotion creation was not counted')
    testing.expectEqual(after.removed, before.removed + 1,
        'rotated powered locomotion removal was not counted')
    testing.expectEqual(after.poseFailures, before.poseFailures,
        'rotated powered locomotion caused fallback')
    victim:remove()
end)

registerRagdollTest('powered actor receives bounded gameplay gait velocity', function()
    local player = world.players[1]
    local victim = world.createObject(types.NPC.record(player).id)
    victim:teleport(player.cell, testActorOrigin, { onGround = true })
    coroutine.yield()
    victim:addScript('gait_actor.lua')
    coroutine.yield()

    local before = ragdoll.getCounters()
    ragdoll.setPowered(victim, true)
    waitFor('powered gait actor creation', function()
        local state = ragdoll.getState(victim)
        return state and state.powered and state.controller and state.controller.mode == 'standing' and state
    end)

    local startPosition = victim.position
    local observedTarget = false
    local maximumTargetSpeed = 0
    local maximumTrackingError = 0
    local maximumConstraintAnchorSeparation = 0
    local maximumLinearSpeed = 0
    local finalTrackingError = 0
    local recentAlongSpeeds = {}
    local recentTargetSpeeds = {}
    local observedFalling = false
    local observedJointMotors = false
    local observedUprightControl = false
    local modeTrace = {}
    local lastMode
    local minimumPelvisUpDot = 1
    local maximumGroundContacts = 0
    local unsupportedFrames = 0
    local maximumUnsupportedFrames = 0
    local minimumDrivenPelvisUpDot = 1
    local maximumPelvisTorque = 0
    local maximumSpineTorque = 0
    local pelvisForceSaturatedFrames = 0
    local pelvisMagnitudeSaturatedFrames = 0
    local pelvisHorizontalForceSaturatedFrames = 0
    local pelvisVerticalForceSaturatedFrames = 0
    local consecutiveUpwardForceSaturation = 0
    local maximumConsecutiveUpwardForceSaturation = 0
    local pelvisTorqueSaturatedFrames = 0
    local spineTorqueSaturatedFrames = 0
    local firstFallingFrame = -1
    local firstFallingPelvisUpDot = 1
    local firstFallingGroundContacts = 0
    local gaitPhaseTrace = {}
    local lastGaitPhase
    local gaitTransitions = 0
    local gaitTimeoutFrames = 0
    local preGaitState = ragdoll.getState(victim)
    emitTelemetry('powered_gait_start', {
        com_velocity_x = preGaitState.controller.centerOfMassVelocity.x,
        com_velocity_y = preGaitState.controller.centerOfMassVelocity.y,
        target_velocity_x = preGaitState.controller.targetHorizontalVelocity.x,
        target_velocity_y = preGaitState.controller.targetHorizontalVelocity.y,
        pelvis_up = preGaitState.controller.pelvisUpDot,
    })
    gaitCommandAcknowledged = false
    victim:sendEvent('startRagdollTravel', 1)
    waitFor('AI gait movement command', function()
        return gaitCommandAcknowledged
    end)
    waitFor('powered gait support acquisition', function()
        local state = ragdoll.getState(victim)
        return state and state.controller and state.support
            and state.controller.targetHorizontalVelocity:length() > 40
            and state.support.footContactCount > 0 and state
    end)
    local replayCaptureFrames = 240
    ragdoll.startReplayRecording(victim, 67891, replayCaptureFrames)
    for frame = 1, replayCaptureFrames do
        local state = ragdoll.getState(victim)
        testing.expect(state ~= nil and state.powered, 'powered gait control fell back unexpectedly')
        inspectTelemetry(state, 'powered gait control')
        if state.controller then
            inspectControllerTelemetry(state.controller, 'powered gait control', true)
            if state.controller.mode ~= lastMode then
                table.insert(modeTrace, state.controller.mode)
                lastMode = state.controller.mode
            end
            minimumPelvisUpDot = math.min(minimumPelvisUpDot, state.controller.pelvisUpDot)
            if state.controller.gaitPhase ~= lastGaitPhase then
                table.insert(gaitPhaseTrace, state.controller.gaitPhase)
                lastGaitPhase = state.controller.gaitPhase
            end
            if state.controller.gaitTransitioned then
                gaitTransitions = gaitTransitions + 1
            end
            if state.controller.gaitPhaseTimedOut then
                gaitTimeoutFrames = gaitTimeoutFrames + 1
            end
            if state.controller.jointMotorsApplied then
                minimumDrivenPelvisUpDot = math.min(minimumDrivenPelvisUpDot, state.controller.pelvisUpDot)
            end
            if state.support then
                maximumGroundContacts = math.max(maximumGroundContacts, state.support.groundContactCount)
                if state.support.groundContactCount == 0 then
                    unsupportedFrames = unsupportedFrames + 1
                    maximumUnsupportedFrames = math.max(maximumUnsupportedFrames, unsupportedFrames)
                else
                    unsupportedFrames = 0
                end
            end
            local targetSpeed = state.controller.targetHorizontalVelocity:length()
            maximumTargetSpeed = math.max(maximumTargetSpeed, targetSpeed)
            maximumTrackingError = math.max(maximumTrackingError, state.controller.velocityTrackingError)
            observedTarget = observedTarget or targetSpeed > 1
            if targetSpeed > 1 then
                finalTrackingError = state.controller.velocityTrackingError
                -- Same speed contract as the offline walking test: the mean
                -- center-of-mass speed along the request over the final half
                -- second (30 fixed steps), not the instantaneous vector error,
                -- which lateral sway between steps dominates.
                local along = state.controller.centerOfMassVelocity:dot(state.controller.targetHorizontalVelocity)
                    / targetSpeed
                table.insert(recentAlongSpeeds, along)
                table.insert(recentTargetSpeeds, targetSpeed)
                if #recentAlongSpeeds > 30 then
                    table.remove(recentAlongSpeeds, 1)
                    table.remove(recentTargetSpeeds, 1)
                end
                local falling = state.controller.mode == 'falling' or state.controller.mode == 'fallen'
                if falling and not observedFalling then
                    firstFallingFrame = frame
                    firstFallingPelvisUpDot = state.controller.pelvisUpDot
                    firstFallingGroundContacts = state.support and state.support.groundContactCount or 0
                end
                observedFalling = observedFalling or falling
                observedJointMotors = observedJointMotors or state.controller.jointMotorsApplied
                -- Balance actuation is either the legacy pelvis force pairs or,
                -- on the reference route, joint-support torques through the
                -- motors; both count as applied balance control.
                observedUprightControl = observedUprightControl
                    or (state.controller.upright ~= nil and state.controller.upright.applied)
                    or state.controller.jointSupportApplied == true
            end
            if state.controller.upright and state.controller.upright.applied then
                maximumPelvisTorque
                    = math.max(maximumPelvisTorque, state.controller.upright.pelvisTorque:length())
                maximumSpineTorque
                    = math.max(maximumSpineTorque, state.controller.upright.spineTorque:length())
                if state.controller.upright.pelvisForceSaturated then
                    pelvisForceSaturatedFrames = pelvisForceSaturatedFrames + 1
                end
                if state.controller.upright.pelvisMagnitudeSaturated then
                    pelvisMagnitudeSaturatedFrames = pelvisMagnitudeSaturatedFrames + 1
                end
                if state.controller.upright.pelvisHorizontalForceSaturated then
                    pelvisHorizontalForceSaturatedFrames = pelvisHorizontalForceSaturatedFrames + 1
                end
                if state.controller.upright.pelvisVerticalForceSaturated then
                    pelvisVerticalForceSaturatedFrames = pelvisVerticalForceSaturatedFrames + 1
                end
                if state.controller.upright.pelvisVerticalForceSaturated
                    and state.controller.upright.pelvisForce.z > 0 then
                    consecutiveUpwardForceSaturation = consecutiveUpwardForceSaturation + 1
                    maximumConsecutiveUpwardForceSaturation = math.max(
                        maximumConsecutiveUpwardForceSaturation, consecutiveUpwardForceSaturation)
                else
                    consecutiveUpwardForceSaturation = 0
                end
                if state.controller.upright.pelvisTorqueSaturated then
                    pelvisTorqueSaturatedFrames = pelvisTorqueSaturatedFrames + 1
                end
                if state.controller.upright.spineTorqueSaturated then
                    spineTorqueSaturatedFrames = spineTorqueSaturatedFrames + 1
                end
            else
                consecutiveUpwardForceSaturation = 0
            end
            if frame <= 5 or frame % 5 == 0 then
                local upright = state.controller.upright
                local leftFoot = findBody(state, 'left_foot')
                local rightFoot = findBody(state, 'right_foot')
                emitTelemetry('powered_gait_frame', {
                    frame = frame,
                    mode = state.controller.mode,
                    falling_exit_candidate_time = state.controller.fallingExitCandidateTime,
                    pelvis_up = state.controller.pelvisUpDot,
                    pelvis_height = state.controller.pelvisHeight,
                    target_pelvis_height = upright and upright.targetPelvisHeight or 0,
                    com_x = state.centerOfMass.x,
                    com_y = state.centerOfMass.y,
                    com_velocity_x = state.controller.centerOfMassVelocity.x,
                    com_velocity_y = state.controller.centerOfMassVelocity.y,
                    target_velocity_x = state.controller.targetHorizontalVelocity.x,
                    target_velocity_y = state.controller.targetHorizontalVelocity.y,
                    tracking_error = state.controller.velocityTrackingError,
                    gait_phase = state.controller.gaitPhase,
                    gait_phase_time = state.controller.gaitTimeInPhase,
                    gait_phase_progress = state.controller.gaitPhaseProgress,
                    gait_contact_candidate_time
                        = state.controller.gaitSwingContactCandidateTime,
                    gait_swing_foot_lifted = state.controller.gaitSwingFootLifted,
                    gait_swing_touchdown_candidate = state.controller.gaitSwingTouchdownCandidate,
                    gait_phase_timed_out = state.controller.gaitPhaseTimedOut,
                    ground_contacts = state.support and state.support.groundContactCount or 0,
                    foot_contacts = state.support and state.support.footContactCount or 0,
                    left_foot_supported = state.support and state.support.leftFoot or false,
                    right_foot_supported = state.support and state.support.rightFoot or false,
                    left_foot_x = leftFoot.centerOfMass.x,
                    left_foot_y = leftFoot.centerOfMass.y,
                    left_foot_z = leftFoot.position.z,
                    right_foot_x = rightFoot.centerOfMass.x,
                    right_foot_y = rightFoot.centerOfMass.y,
                    right_foot_z = rightFoot.position.z,
                    swing_target_x = state.controller.swingFootTarget
                        and state.controller.swingFootTarget.x or 0,
                    swing_target_y = state.controller.swingFootTarget
                        and state.controller.swingFootTarget.y or 0,
                    swing_target_z = state.controller.swingFootTarget
                        and state.controller.swingFootTarget.z or 0,
                    swing_trajectory_model_x = state.controller.swingFootTrajectoryModel
                        and state.controller.swingFootTrajectoryModel.x or 0,
                    swing_trajectory_model_y = state.controller.swingFootTrajectoryModel
                        and state.controller.swingFootTrajectoryModel.y or 0,
                    swing_resolved_model_x = state.controller.swingFootResolvedModel
                        and state.controller.swingFootResolvedModel.x or 0,
                    swing_resolved_model_y = state.controller.swingFootResolvedModel
                        and state.controller.swingFootResolvedModel.y or 0,
                    swing_pose_model_x = state.controller.swingFootPoseModel
                        and state.controller.swingFootPoseModel.x or 0,
                    swing_pose_model_y = state.controller.swingFootPoseModel
                        and state.controller.swingFootPoseModel.y or 0,
                    swing_controller_world_x = state.controller.swingFootControllerWorldTarget
                        and state.controller.swingFootControllerWorldTarget.x or 0,
                    swing_controller_world_y = state.controller.swingFootControllerWorldTarget
                        and state.controller.swingFootControllerWorldTarget.y or 0,
                    swing_controller_world_z = state.controller.swingFootControllerWorldTarget
                        and state.controller.swingFootControllerWorldTarget.z or 0,
                    swing_hip_motor_error = state.controller.swingHipMotorError or 0,
                    swing_hip_motor_clamp = state.controller.swingHipMotorClamp or 0,
                    swing_knee_motor_error = state.controller.swingKneeMotorError or 0,
                    swing_knee_motor_clamp = state.controller.swingKneeMotorClamp or 0,
                    swing_ankle_motor_error = state.controller.swingAnkleMotorError or 0,
                    swing_ankle_motor_clamp = state.controller.swingAnkleMotorClamp or 0,
                    swing_hip_motor_utilization = state.controller.swingHipMotorUtilization or 0,
                    swing_knee_motor_utilization = state.controller.swingKneeMotorUtilization or 0,
                    swing_ankle_motor_utilization = state.controller.swingAnkleMotorUtilization or 0,
                    support_distance = state.support and state.support.centerOfMassDistance or 0,
                    pelvis_force_x = upright and upright.pelvisForce.x or 0,
                    pelvis_force_y = upright and upright.pelvisForce.y or 0,
                    pelvis_force_z = upright and upright.pelvisForce.z or 0,
                    pelvis_magnitude_saturated = upright and upright.pelvisMagnitudeSaturated or false,
                    pelvis_horizontal_force_saturated
                        = upright and upright.pelvisHorizontalForceSaturated or false,
                    pelvis_vertical_force_saturated
                        = upright and upright.pelvisVerticalForceSaturated or false,
                    pelvis_torque_x = upright and upright.pelvisTorque.x or 0,
                    pelvis_torque_y = upright and upright.pelvisTorque.y or 0,
                    pelvis_torque_z = upright and upright.pelvisTorque.z or 0,
                })
            end
        end
        for _, body in ipairs(state.bodies) do
            maximumLinearSpeed = math.max(maximumLinearSpeed, body.linearVelocity:length())
        end
        for _, constraint in ipairs(state.constraints) do
            maximumConstraintAnchorSeparation
                = math.max(maximumConstraintAnchorSeparation, constraint.anchorSeparation)
        end
        coroutine.yield()
    end
    local travelDistance = (victim.position - startPosition):length()
    local replay = ragdoll.finishReplayRecording()
    testing.expect(replay ~= nil, 'powered gait replay recording failed')
    testing.expectEqual(replay.version, 10, 'unexpected powered gait replay format version')
    testing.expectEqual(replay.seed, '67891', 'powered gait replay seed was not preserved exactly')
    testing.expectEqual(replay.inputCount, 143,
        'powered gait replay did not capture velocity plus 20 animation target transforms')
    testing.expectEqual(replay.controllerStateCount, 10,
        'powered gait replay did not capture the controller/phase alignment channels')
    testing.expectEqual(replay.bodyCount, 20, 'powered gait replay body insertion order is incomplete')
    testing.expectEqual(replay.frameCount, replayCaptureFrames,
        'powered gait replay did not cover the complete fixed-step capture window')
    testing.expectEqual(#replay.data, replay.byteCount, 'powered gait replay binary was not returned intact')

    ragdoll.setPowered(victim, false)
    waitFor('powered gait actor cleanup', function()
        return ragdoll.getState(victim) == nil
    end)

    local after = ragdoll.getCounters()
    emitTelemetry('powered_gait', {
        distance = travelDistance,
        max_target_speed = maximumTargetSpeed,
        max_tracking_error = maximumTrackingError,
        max_constraint_anchor_separation = maximumConstraintAnchorSeparation,
        final_tracking_error = finalTrackingError,
        max_linear_speed = maximumLinearSpeed,
        falling = observedFalling,
        modes = table.concat(modeTrace, ','),
        min_pelvis_up = minimumPelvisUpDot,
        min_driven_pelvis_up = minimumDrivenPelvisUpDot,
        max_ground_contacts = maximumGroundContacts,
        max_unsupported_frames = maximumUnsupportedFrames,
        max_pelvis_torque = maximumPelvisTorque,
        max_spine_torque = maximumSpineTorque,
        pelvis_force_saturated_frames = pelvisForceSaturatedFrames,
        pelvis_magnitude_saturated_frames = pelvisMagnitudeSaturatedFrames,
        pelvis_horizontal_force_saturated_frames = pelvisHorizontalForceSaturatedFrames,
        pelvis_vertical_force_saturated_frames = pelvisVerticalForceSaturatedFrames,
        max_consecutive_upward_force_saturation
            = maximumConsecutiveUpwardForceSaturation,
        pelvis_torque_saturated_frames = pelvisTorqueSaturatedFrames,
        spine_torque_saturated_frames = spineTorqueSaturatedFrames,
        first_falling_frame = firstFallingFrame,
        first_falling_pelvis_up = firstFallingPelvisUpDot,
        first_falling_ground_contacts = firstFallingGroundContacts,
        gait_phases = table.concat(gaitPhaseTrace, ','),
        gait_transitions = gaitTransitions,
        gait_timeout_frames = gaitTimeoutFrames,
    })
    victim:remove()
    coroutine.yield()
    testing.expect(observedTarget, 'ordinary AI movement never reached the powered controller')
    testing.expectGreaterThan(travelDistance, 1,
        'ordinary movement did not advance the powered actor root')
    testing.expectGreaterThan(maximumTargetSpeed, 40,
        'powered gait scenario did not request its intended walking speed')
    testing.expectLessOrEqual(maximumTargetSpeed, 80,
        'powered gait scenario exceeded its intended initial walking speed')
    testing.expectLessOrEqual(maximumLinearSpeed, 2000,
        'powered gait control reached an unsafe body speed')
    testing.expectLessOrEqual(maximumConstraintAnchorSeparation, 2.5,
        'powered gait pulled a joint beyond its physical anchor tolerance')
    testing.expect(observedJointMotors, 'powered gait control did not apply animation joint targets')
    testing.expect(observedUprightControl, 'powered gait control did not apply balance forces')
    testing.expectLessOrEqual(maximumUnsupportedFrames, 2,
        'walking gait lost all ground contact for more than two observation frames')
    testing.expectLessOrEqual(maximumConsecutiveUpwardForceSaturation, 3,
        'walking gait saturated upward translation actuation for too long')
    local finalAlongSpeed, finalTargetSpeed = 0, 0
    for index = 1, #recentAlongSpeeds do
        finalAlongSpeed = finalAlongSpeed + recentAlongSpeeds[index] / #recentAlongSpeeds
        finalTargetSpeed = finalTargetSpeed + recentTargetSpeeds[index] / #recentTargetSpeeds
    end
    testing.expectGreaterOrEqual(finalAlongSpeed, finalTargetSpeed * 0.5,
        'physical center of mass did not track at least half of the gait target over the final half second')
    testing.expect(not observedFalling, 'bounded gait movement caused the controller to fall')
    testing.expectEqual(after.current, before.current, 'powered gait test leaked a ragdoll')
    testing.expectEqual(after.created, before.created + 1, 'powered gait creation was not counted')
    testing.expectEqual(after.removed, before.removed + 1, 'powered gait removal was not counted')
    testing.expectEqual(after.poseFailures, before.poseFailures, 'powered gait control caused fallback')
end)

registerRagdollTest('owned walk reference has a repeatable continuous source cycle', function()
    local player = world.players[1]
    local victim = world.createObject(types.NPC.record(player).id)
    victim:teleport(player.cell, testActorOrigin, { onGround = true })
    coroutine.yield()
    ragdoll.setPowered(victim, true)
    local initial = waitFor('walk-reference skeleton binding', function() return ragdoll.getState(victim) end)
    -- Preserve the actual starting rig/targets alongside the independent clip.
    ragdoll.startReplayRecording(victim, 69000, 2)
    waitFor('walk-reference starting pose capture', function()
        return ragdoll.getState(victim).physicsStep >= initial.physicsStep + 3
    end)
    local replay = ragdoll.finishReplayRecording()
    local before = ragdoll.getState(victim)
    local selectedGroup = 'walkforwardhh'
    local cycle = ragdoll.sampleAnimationCycle(victim, selectedGroup, 180)
    local repeated = ragdoll.sampleAnimationCycle(victim, selectedGroup, 180)
    local after = ragdoll.getState(victim)
    emitTelemetry('authored_walk_cycle_summary', {
        valid = cycle.valid, error = cycle.error, group = selectedGroup,
        start_time = cycle.startTime, end_time = cycle.endTime,
        sample_count = #cycle.frames, body_count = #before.bodies,
        maximum_seam_translation = cycle.maximumSeamTranslation,
        minimum_seam_rotation_dot = cycle.minimumSeamRotationDot,
        accumulation_bone = cycle.accumulationBone,
        profile_hash = replay and replay.profileHash or 'unavailable',
        initial_pose_replay_seed = '69000',
    })
    ragdoll.setPowered(victim, false)
    waitFor('walk-reference cleanup', function() return ragdoll.getState(victim) == nil end)
    victim:remove()
    testing.expect(cycle.valid, cycle.error)
    testing.expect(repeated.valid, repeated.error)
    testing.expect(replay and replay.frameCount == 2, 'walk-reference initial rig capture is incomplete')
    testing.expectEqual(before.physicsStep, after.physicsStep, 'source sampling advanced physics')
    testing.expectEqual(#cycle.frames, 180, 'source cycle has the wrong phase sample count')
    for i, body in ipairs(before.bodies) do
        testing.expectEqual((body.position - after.bodies[i].position):length(), 0,
            'source sampling moved a physical body')
    end
    for frame, pose in ipairs(cycle.frames) do
        testing.expectEqual(#pose, #before.bodies, 'source cycle has the wrong body mapping')
        for body, values in ipairs(pose) do
            for component, value in ipairs(values) do
                testing.expectEqual(value, repeated.frames[frame][body][component],
                    'source phase samples changed on repetition')
            end
            emitTelemetry('authored_walk_sample', {
                frame = frame - 1, body = body - 1, name = before.bodies[body].name,
                tx = values[1], ty = values[2], tz = values[3],
                qx = values[4], qy = values[5], qz = values[6], qw = values[7],
            })
        end
    end
end)

registerRagdollTest('manual ragdoll helper creates a verified open test area', function()
    local demo = interfaces.RagdollDev
    testing.expect(demo ~= nil and demo.version == 5, 'manual ragdoll helper is unavailable')
    testing.expect(type(demo.traceGait) == 'function', 'manual helper has no bounded gait trace command')
    testing.expect(type(demo.traceStatus) == 'function', 'manual helper has no trace status command')
    demo.setup()
    local victim = waitFor('manual helper test actor', function()
        return demo.actor()
    end)
    testing.expect(types.NPC.objectIsInstance(victim), 'manual helper did not create an NPC test actor')
    testing.expectEqual(world.players[1].cell.name, testArea.cell, 'manual helper left the player in the wrong cell')
    testing.expectEqual(victim.cell.name, testArea.cell, 'manual helper created its NPC in the wrong cell')
    testing.expect(not victim.cell.isExterior, 'manual helper loaded exterior terrain')
    testing.runLocalTest(world.players[1], 'ragdoll flat floor geometry')
    local playerOffset = util.vector3(world.players[1].position.x - testPlayerOrigin.x,
        world.players[1].position.y - testPlayerOrigin.y, 0)
    local actorOffset = util.vector3(victim.position.x - testActorOrigin.x,
        victim.position.y - testActorOrigin.y, 0)
    testing.expectLessOrEqual(playerOffset:length(), 75,
        'manual helper player moved implausibly far from the flat floor start')
    testing.expectLessOrEqual(actorOffset:length(), 75,
        'manual helper sample moved implausibly far from its flat floor start')
    testing.expectLessOrEqual((victim.position - world.players[1].position):length(), 650,
        'manual helper test actor was not placed beside the player')
    testing.expect(ragdoll.getState(victim) == nil, 'manual helper unexpectedly pre-activated its test actor')
    local samples = demo.samples()
    testing.expect(samples.powered == victim, 'manual helper powered sample is inconsistent')
    testing.expect(types.NPC.objectIsInstance(samples.passive), 'manual helper has no passive sample')
    testing.expect(types.NPC.objectIsInstance(samples.gait), 'manual helper has no gait sample')
    for key, actor in pairs(samples) do
        testing.expectEqual(actor.cell.name, testArea.cell, key .. ' sample is outside the flat floor cell')
        testing.expectLessOrEqual(math.abs(actor.position.z - testArea.floorHeight), 5,
            key .. ' sample was not grounded on the flat floor')
    end
    testing.expectEqual(types.NPC.record(samples.powered).name,
        '[Ragdoll A] Powered Balance / Recovery', 'manual powered sample is not labeled')
    testing.expectEqual(types.NPC.record(samples.passive).name,
        '[Ragdoll B] Passive Death', 'manual passive sample is not labeled')
    testing.expectEqual(types.NPC.record(samples.gait).name,
        '[Ragdoll C] Powered Gait (Experimental)', 'manual gait sample is not labeled')
    demo.cleanup()
end)

-- Strict live walking evidence for the reference route at the manual helper's
-- ten-second bound: no falling/fallen frame while enabled, repeated swing
-- Stress pass (robustness before combat). These use the plain gameplay
-- walker (gait_actor.lua + ordinary movement request), not the manual
-- helper's single-actor trace, so several rigs can walk at once and a run
-- can last longer than the ten-second trace bound.
local openMwUnitsPerMeter = 69.99125109 -- ragdollcore/unitmodel.hpp

local fpsSourceSampler = require('fps_source')
local function installFpsSource(actor, source)
    if not source then return end
    source.epochHalfTick = math.floor(core.getSimulationTime() * 120 + 0.5)
    local plan = fpsSourceSampler.plan(source.scenario)
    local extra = source.scenario ~= 'S1' and string.format(' restart_step=%d final_stop_step=%d', plan.restartStep, plan.finalStopStep) or ''
    print(string.format('RAGDOLL_FPS_SOURCE_PLAN schema=%d scenario=%s epoch_half_tick=%d start_step=%d stop_step=%d end_step=%d%s',
        source.scenario ~= 'S1' and 2 or 1, source.scenario, source.epochHalfTick, plan.startStep, plan.stopStep, plan.endStep, extra))
    actor:sendEvent('ragdollFpsSource', source)
end

local function spawnGameplayWalker(offset, source)
    local player = world.players[1]
    -- The literal "player" record is classified as Player by the Lua engine,
    -- even for a new reference. Preserve the rig/stats in a distinct NPC record.
    local record = world.createRecord(types.NPC.createRecordDraft({ template = types.NPC.record(player) }))
    local walker = world.createObject(record.id)
    testing.expectEqual(walker.type, types.NPC, 'walker must use the NPC script path')
    walker:teleport(player.cell, testActorOrigin + (offset or util.vector3(0, 0, 0)), { onGround = true })
    coroutine.yield()
    walker:addScript('gait_actor.lua')
    coroutine.yield()
    installFpsSource(walker, source)
    ragdoll.setPowered(walker, true)
    waitFor('stress walker creation', function()
        local state = ragdoll.getState(walker)
        return state and state.powered and state.controller and state.controller.mode == 'standing' and state
    end)
    return walker
end

local function spawnFpsPlayer(source)
    local player = world.players[1]
    player:teleport(player.cell, testPlayerOrigin, { onGround = true, rotation = util.transform.rotateZ(0) })
    coroutine.yield()
    testing.runLocalTest(player, 'ragdoll third person')
    installFpsSource(player, source)
    ragdoll.setPowered(player, true)
    waitFor('FPS player creation', function()
        local state = ragdoll.getState(player)
        return state and state.controller and state.controller.mode == 'standing'
    end)
    return player
end

for _, scenario in ipairs({ 'S1', 'S2', 'S2Straight' }) do
for _, playerCase in ipairs({ false, true }) do
registerRagdollTest(string.format('captures FPS %s %s source preflight', playerCase and 'player' or 'NPC', scenario), function()
    local source = { scenario = scenario }
    local plan = fpsSourceSampler.plan(scenario)
    local actor = playerCase and spawnFpsPlayer(source) or spawnGameplayWalker(nil, source)
    ragdoll.startReplayRecording(actor, scenario == 'S1' and 20260921 or (scenario == 'S2' and 20260922 or 20260923), plan.endStep + 140)
    local realDeadline = core.getRealTime() + 45
    repeat coroutine.yield() until math.floor(core.getSimulationTime() * 120 + 0.5) - source.epochHalfTick >= 2 * plan.endStep
        or core.getRealTime() >= realDeadline
    local replay = ragdoll.finishReplayRecording()
    testing.expect(replay ~= nil and replay.frameCount >= plan.endStep - 10, 'source capture incomplete')
    ragdoll.setPowered(actor, false)
    if not playerCase then actor:remove() end
    -- Source/capture preflight only; controller failures remain in the capture.
end)
end
end

for _, playerCase in ipairs({ false, true }) do
registerRagdollTest(playerCase and 'captures FPS player initial state preflight'
    or 'captures FPS initial state preflight', function()
    local actor = playerCase and spawnFpsPlayer() or spawnGameplayWalker()
    ragdoll.startReplayRecording(actor, 20260920, 180)
    local deadline = core.getSimulationTime() + 1
    repeat coroutine.yield() until core.getSimulationTime() >= deadline
    local replay = ragdoll.finishReplayRecording()
    testing.expect(replay ~= nil and replay.frameCount >= 60, 'initial-state capture incomplete')
    ragdoll.setPowered(actor, false)
    if not playerCase then actor:remove() end
    -- Creation-state comparison only; no FPS behavior or walking verdict.
end)
end

local function captureNpcWalkReplay(finalStopSeconds, seed, maximumFrames, minimumFrames, directFall, manualRecovery)
    local actor = spawnGameplayWalker()
    ragdoll.startReplayRecording(actor, seed, maximumFrames)
    local recordingSimulationStart = core.getSimulationTime()
    local recordingRealStart = core.getRealTime()
    local observations, missing, falling = 0, 0, 0
    local directFallApplied = false
    local recoveryRequested = false
    for _, segment in ipairs({ {4, 1}, {4, 0}, {3, 1}, {finalStopSeconds, 0} }) do
        actor:sendEvent('startRagdollTravel', segment[2])
        local untilTime = core.getSimulationTime() + segment[1]
        local deadline = core.getRealTime() + 30
        repeat
            coroutine.yield()
            local state = ragdoll.getState(actor)
            if manualRecovery and not recoveryRequested and state and state.controller
                and state.controller.mode == 'fallen' then
                interfaces.RagdollDev.recover(actor)
                recoveryRequested = true
            end
            if directFall and not directFallApplied and state and state.controller
                and core.getSimulationTime() - recordingSimulationStart >= 1 then
                if directFall == 'combat' then
                    ragdoll.applyCombatImpulse(actor, util.vector3(1500, 0, 0),
                        findBody(state, 'pelvis').position + util.vector3(0, 0, 20))
                else
                    interfaces.RagdollDev.fall(actor)
                end
                directFallApplied = true
            end
            observations = observations + 1
            if not state or not state.controller then missing = missing + 1
            elseif state.controller.mode == 'falling' or state.controller.mode == 'fallen' then
                falling = falling + 1
            end
        until core.getSimulationTime() >= untilTime or core.getRealTime() >= deadline
        testing.expect(core.getSimulationTime() >= untilTime, 'NPC recording segment timed out')
    end
    local recordingSimulationEnd = core.getSimulationTime()
    local recordingRealEnd = core.getRealTime()
    local replay = ragdoll.finishReplayRecording()
    actor:sendEvent('startRagdollTravel', 0)
    ragdoll.setPowered(actor, false)
    actor:remove()
    emitTelemetry('npc_replay_capture', { observations = observations, missing = missing,
        falling_observations = falling, direct_fall_applied = directFall ~= 'combat' and directFallApplied,
        combat_impulse_applied = directFall == 'combat' and directFallApplied,
        manual_recovery_requested = recoveryRequested,
        replay_frames = replay and replay.frameCount or 0,
        simulation_start = recordingSimulationStart, simulation_end = recordingSimulationEnd,
        simulation_elapsed = recordingSimulationEnd - recordingSimulationStart,
        real_elapsed = recordingRealEnd - recordingRealStart })
    testing.expect(replay ~= nil and replay.frameCount >= minimumFrames, 'NPC replay incomplete')
    testing.expectEqual(missing, 0, 'NPC disappeared during recording')
    if directFall then testing.expect(directFallApplied, 'direct fall input was not applied') end
    if manualRecovery then testing.expect(recoveryRequested, 'manual recovery request did not execute') end
    -- Capture validity only. Falling is recorded evidence, never walking acceptance.
end

registerRagdollTest('captures isolated NPC walking stops and restart for replay', function()
    captureNpcWalkReplay(3, 20260915, 1000, 840)
end)

registerRagdollTest('captures isolated NPC recovery finish after walking for replay', function()
    captureNpcWalkReplay(17, 20260916, 1800, 1680)
end)

registerRagdollTest('captures isolated NPC direct fall event for replay', function()
    captureNpcWalkReplay(17, 20260917, 1800, 1680, true)
end)

registerRagdollTest('captures isolated NPC manual recovery request for replay', function()
    captureNpcWalkReplay(17, 20260918, 1800, 1680, true, true)
end)

registerRagdollTest('captures isolated NPC combat interruption for replay', function()
    captureNpcWalkReplay(17, 20260919, 1800, 1680, 'combat')
end)

for _, trial in ipairs({
    { suffix = '', run = false, yaw = 0 },
    { suffix = ' with replay capture', run = false, yaw = 0, replay = true },
    { name = 'third person player holds full forward for twenty seconds with replay capture', run = false, yaw = 0, replay = true, replaySeed = 20260913, seconds = 20 },
    { suffix = ' from settled standing', run = false, yaw = 0, settled = true },
    { suffix = ' with running input', run = true, yaw = 0 },
    { suffix = ' at rotated heading', run = false, yaw = math.pi / 2 },
    { suffix = ' at rotated heading with replay capture', run = false, yaw = math.pi / 2, replay = true, replaySeed = 20260910 },
    { suffix = ' at reversed heading', run = false, yaw = math.pi, replay = true, replaySeed = 20260911 },
    { suffix = ' at negative rotated heading', run = false, yaw = -math.pi / 2, replay = true, replaySeed = 20260912 },
}) do
registerRagdollTest(trial.name or ('third person player holds full forward for ten seconds' .. trial.suffix), function()
    local walkDuration = trial.seconds or 10
    local player = world.players[1]
    player:teleport(player.cell, testPlayerOrigin, {
        onGround = true, rotation = util.transform.rotateZ(trial.yaw),
    })
    coroutine.yield()
    testing.runLocalTest(player, 'ragdoll third person')
    local settleUntil = core.getSimulationTime() + 1
    repeat coroutine.yield() until core.getSimulationTime() >= settleUntil
    ragdoll.setPowered(player, true)
    waitFor('powered player standing', function()
        local state = ragdoll.getState(player)
        return state and state.controller and state.controller.mode == 'standing'
            and (not trial.settled or (state.controller.pelvisHeightSettled and state.support
                and state.support.leftFoot and state.support.rightFoot))
    end)
    if trial.replay then ragdoll.startReplayRecording(player, trial.replaySeed or 20260909, math.ceil(walkDuration * 60) + 60) end
    local origin = player.position
    local initialState = ragdoll.getState(player)
    local initialPelvis = findBody(initialState, 'pelvis')
    local initialYaw = player.rotation:getYaw()
    local localForward = initialPelvis.rotation:inverse()
        * util.vector3(math.sin(initialYaw), math.cos(initialYaw), 0)
    local hipLine = findBody(initialState, 'right_thigh').position - findBody(initialState, 'left_thigh').position
    local initialSideAlignment = (hipLine.x * math.cos(initialYaw) - hipLine.y * math.sin(initialYaw))
        / math.sqrt(hipLine.x * hipLine.x + hipLine.y * hipLine.y)
    emitTelemetry('third_person_player_forward_basis', {
        initial_yaw = trial.yaw, local_forward_x = localForward.x,
        local_forward_y = localForward.y, local_forward_z = localForward.z,
        initial_side_alignment = initialSideAlignment,
        settled_start = trial.settled or false,
        physics_step = initialState.physicsStep,
        height_settled = initialState.controller.pelvisHeightSettled,
        pelvis_height = initialState.controller.pelvisHeight,
        com_speed = initialState.controller.centerOfMassVelocity:length(),
        left_supported = initialState.support and initialState.support.leftFoot,
        right_supported = initialState.support and initialState.support.rightFoot,
    })
    player:sendEvent('ragdollPlayerTravel', { enabled = true, movement = 1, run = trial.run })
    local started = core.getSimulationTime()
    local deadline = core.getRealTime() + 45
    local falling, missing, frames = 0, 0, 0
    local minimumUp, maximumCommand, firstFall = 1, 0, -1
    local maximumRawLateral, missingInputTelemetry = 0, 0
    -- Routes without the reference gait clock (animation_retimed) still owe
    -- the harness alternating physical steps. Count a step when either foot
    -- leaves measured support.
    local physicalSteps, lastLeftSupported, lastRightSupported = 0, nil, nil
    local qualityTime, headingSquared, lateralSquared, maximumHeading = 0, 0, 0, 0
    local minimumPelvisZ, maximumPelvisZ = math.huge, -math.huge
    local lastObservation = started
    local nextSample = 1
    repeat
        coroutine.yield()
        local state = ragdoll.getState(player)
        local now = core.getSimulationTime()
        local observationDuration = now - lastObservation
        lastObservation = now
        frames = frames + 1
        if state and state.controller then
            local c = state.controller
            inspectControllerTelemetry(c, 'third person player', true)
            -- Appearance diagnostics use the steady part of the walk and
            -- time weighting, so render FPS does not change their meaning.
            -- These accompany the safety gates; they do not establish visual
            -- acceptance or replace a measured improvement with "no fall".
            if now - started >= 2 and observationDuration > 0 then
                local yaw = player.rotation:getYaw()
                local lateral = c.centerOfMassVelocity.x * math.cos(yaw)
                    - c.centerOfMassVelocity.y * math.sin(yaw)
                qualityTime = qualityTime + observationDuration
                headingSquared = headingSquared + c.headingError * c.headingError * observationDuration
                lateralSquared = lateralSquared + lateral * lateral * observationDuration
                maximumHeading = math.max(maximumHeading, math.abs(c.headingError))
                local pelvis = findBody(state, 'pelvis')
                minimumPelvisZ = math.min(minimumPelvisZ, pelvis.position.z)
                maximumPelvisZ = math.max(maximumPelvisZ, pelvis.position.z)
            end
            maximumCommand = math.max(maximumCommand, c.targetHorizontalVelocity:length())
            minimumUp = math.min(minimumUp, c.pelvisUpDot)
            if c.rawTargetHorizontalVelocity then
                local yaw = player.rotation:getYaw()
                local raw = c.rawTargetHorizontalVelocity
                maximumRawLateral = math.max(maximumRawLateral,
                    math.abs(raw.x * math.cos(yaw) - raw.y * math.sin(yaw)))
            else
                missingInputTelemetry = missingInputTelemetry + 1
            end
            if c.mode == 'falling' or c.mode == 'fallen' or c.mode == 'getting_up' or c.mode == 'blend_back' then
                falling = falling + 1
                if firstFall < 0 then firstFall = core.getSimulationTime() - started end
            end
            if state.support then
                local leftSupported = state.support.leftFoot and true or false
                local rightSupported = state.support.rightFoot and true or false
                if lastLeftSupported ~= nil then
                    if (lastLeftSupported and not leftSupported)
                        or (lastRightSupported and not rightSupported) then
                        physicalSteps = physicalSteps + 1
                    end
                end
                lastLeftSupported = leftSupported
                lastRightSupported = rightSupported
            end
            if core.getSimulationTime() - started >= nextSample then
                local leftFoot = findBody(state, 'left_foot')
                local rightFoot = findBody(state, 'right_foot')
                emitTelemetry('third_person_player_forward_sample', {
                    elapsed = core.getSimulationTime() - started,
                    physics_step = state.physicsStep,
                    actuation_stage = c.gaitActuationStage,
                    phase = c.gaitPhase, phase_time = c.gaitTimeInPhase,
                    swing_lifted = c.gaitSwingFootLifted,
                    swing_contact_time = c.gaitSwingContactCandidateTime,
                    transitioned = c.gaitTransitioned,
                    left_supported = state.support and state.support.leftFoot,
                    right_supported = state.support and state.support.rightFoot,
                    support_distance = state.support and state.support.centerOfMassDistance,
                    left_x = leftFoot.position.x, left_y = leftFoot.position.y,
                    right_x = rightFoot.position.x, right_y = rightFoot.position.y,
                    left_vertical_velocity = leftFoot.linearVelocity.z,
                    right_vertical_velocity = rightFoot.linearVelocity.z,
                    target_x = c.swingFootTarget and c.swingFootTarget.x,
                    target_y = c.swingFootTarget and c.swingFootTarget.y,
                    initial_yaw = trial.yaw, running_input = trial.run, mode = c.mode,
                    heading_error = c.headingError or 0, pelvis_up = c.pelvisUpDot,
                    gameplay_yaw = player.rotation:getYaw(),
                    command_x = c.targetHorizontalVelocity.x, command_y = c.targetHorizontalVelocity.y,
                    raw_x = c.rawTargetHorizontalVelocity and c.rawTargetHorizontalVelocity.x or 0,
                    raw_y = c.rawTargetHorizontalVelocity and c.rawTargetHorizontalVelocity.y or 0,
                    planned_x = c.plannedHorizontalVelocity and c.plannedHorizontalVelocity.x or 0,
                    planned_y = c.plannedHorizontalVelocity and c.plannedHorizontalVelocity.y or 0,
                    velocity_x = c.centerOfMassVelocity.x, velocity_y = c.centerOfMassVelocity.y,
                })
                nextSample = core.getSimulationTime() - started + 0.1
            end
        else
            missing = missing + 1
        end
    until core.getSimulationTime() - started >= walkDuration or core.getRealTime() >= deadline
    local elapsed = core.getSimulationTime() - started
    local displacement = player.position - origin
    local distance = util.vector3(displacement.x, displacement.y, 0):length()
    local forwardDistance = displacement.x * math.sin(trial.yaw) + displacement.y * math.cos(trial.yaw)
    local forwardReplay = trial.replay and ragdoll.finishReplayRecording() or nil
    playerDiagnostics.endObservation()
    player:sendEvent('ragdollPlayerTravel', { enabled = false })
    ragdoll.setPowered(player, false)
    coroutine.yield()
    emitTelemetry('third_person_player_full_forward', {
        elapsed = elapsed, frames = frames, falling_frames = falling, missing_frames = missing,
        first_fall = firstFall, minimum_up = minimumUp, maximum_command = maximumCommand,
        travelled = distance, physical_steps = physicalSteps,
        running_input = trial.run, initial_yaw = trial.yaw,
        forward_distance = forwardDistance, maximum_raw_lateral = maximumRawLateral,
        quality_seconds = qualityTime,
        heading_rms_degrees = qualityTime > 0 and math.deg(math.sqrt(headingSquared / qualityTime)) or -1,
        maximum_heading_degrees = math.deg(maximumHeading),
        lateral_velocity_rms = qualityTime > 0 and math.sqrt(lateralSquared / qualityTime) or -1,
        pelvis_vertical_range = qualityTime > 0 and maximumPelvisZ - minimumPelvisZ or -1,
    })
    if trial.replay then
        local replay = forwardReplay
        emitTelemetry('player_forward_replay_capture', {
            seed = replay and replay.seed or 'missing',
            frames = replay and replay.frameCount or 0,
            fall_observed = falling > 0,
        })
        testing.expect(replay and replay.frameCount > 0, 'player replay capture is empty')
    end
    testing.expectGreaterOrEqual(elapsed, walkDuration, 'player walk timed out')
    testing.expectGreaterOrEqual(initialSideAlignment, 0.9, 'powered player spawned in the wrong orientation')
    testing.expectEqual(missing, 0, 'powered player disappeared during forward input')
    testing.expectEqual(missingInputTelemetry, 0, 'player input telemetry is missing; check the engine build')
    testing.expectEqual(falling, 0, 'player fell while holding full forward')
    testing.expectGreaterOrEqual(minimumUp, 0.87, 'player stumbled during forward input')
    testing.expectGreaterOrEqual(distance, 20 * walkDuration, 'player did not make sustained walking progress')
    testing.expectGreaterOrEqual(forwardDistance, 20 * walkDuration, 'player did not travel in the requested direction')
    testing.expectLessOrEqual(maximumRawLateral, 0.01, 'animation sway leaked into forward input')
    -- Steps are the measured foot-lift count.
    testing.expectGreaterOrEqual(physicalSteps, 1.5 * walkDuration, 'player did not sustain alternating steps')
end)
end

local function footVerticalImpulse(state, name)
    local index
    for i, body in ipairs(state.bodies) do
        if body.name == name then index = i; break end
    end
    if not index then return nil, true end
    local impulse, unknown = 0, false
    for _, contact in ipairs(state.contacts) do
        if contact.bodyIndex == index and contact.normal.z > 0 then
            if contact.normalImpulse == nil then unknown = true
            else impulse = impulse + contact.normalImpulse * contact.normal.z end
        end
    end
    return impulse, unknown
end

local function prepareVerifiedMouseInput()
    ragdoll.prepareMouseInput()
    waitFor('mouse pointer entry and requested input mode', function()
        local mode = ragdoll.getMouseInputMode()
        if testConfig.nativeMouse then
            return mode.keyboardFocus and mode.pointerInside and mode.nativeRelative
        end
        return mode.syntheticReady or (mode.keyboardFocus and mode.pointerInside)
    end)
    local mode = ragdoll.getMouseInputMode()
    emitTelemetry('mouse_input_mode', { keyboard_focus = mode.keyboardFocus,
        pointer_inside = mode.pointerInside, native_relative = mode.nativeRelative,
        synthetic_ready = mode.syntheticReady,
        requested_native = testConfig.nativeMouse or false })
end

for _, transitionTrial in ipairs({
    { suffix = '', yawRate = 0 },
    { suffix = ' with turning', yawRate = 0.4 },
    { suffix = ' from activation', yawRate = 0, fromActivation = true },
    { suffix = ' from activation with turning and replay capture', yawRate = 0.4, fromActivation = true, replay = true },
    { suffix = ' from activation with sharp turning and replay capture', yawRate = 2.0,
        yawDuration = 0.84, fromActivation = true, replay = true },
    { suffix = ' with arm input and a shove', yawRate = 0.4, disturbed = true },
    { suffix = ' with native mouse input and a shove', yawRate = 0.4, disturbed = true, nativeMouse = true },
}) do
registerRagdollTest('third person player stops and restarts at full input' .. transitionTrial.suffix, function()
    local player = world.players[1]
    player:teleport(player.cell, testPlayerOrigin, {
        onGround = true, rotation = util.transform.rotateZ(0),
    })
    coroutine.yield()
    testing.runLocalTest(player, 'ragdoll third person')
    local settleUntil = core.getSimulationTime() + 1
    repeat coroutine.yield() until core.getSimulationTime() >= settleUntil
    ragdoll.setPowered(player, true)
    waitFor('stop/restart player standing', function()
        local state = ragdoll.getState(player)
        -- Creation initially exposes the controller's default Standing mode
        -- before it has observed contacts or height. This transition test
        -- starts from measured standing, rather than including rig startup.
        local ready = state and state.controller and state.controller.mode == 'standing'
            and (transitionTrial.fromActivation or (state.controller.pelvisHeightSettled and state.support
                and state.support.leftFoot and state.support.rightFoot))
        return ready
    end)
    if transitionTrial.replay then ragdoll.startReplayRecording(player, 20260914, 1500) end
    local falling, missing, minimumUp, timedOut = 0, 0, 1, false
    local turnAngle = 0
    local raisedArm, loweredArm, releasedArm, pushed = false, false, false, false
    if transitionTrial.nativeMouse then prepareVerifiedMouseInput() end
    local armRestZ, armRaisedZ, armLoweredZ, missingHeldSwing = -999, -999, -999, 0
    local armRaisedTargetZ = -999
    local stoppedDrifts, stoppedSpeeds, walkingDistances = {}, {}, {}
    local walkingProgress, walkingPaths, walkingBacktracking = {}, {}, {}
    local segments = {
        { duration = 4, movement = 1 }, { duration = 5, movement = 0 },
        { duration = 4.2, movement = 1, yawRate = transitionTrial.yawRate }, { duration = 5, movement = 0 },
    }
    for index, segment in ipairs(segments) do
        player:sendEvent('ragdollPlayerTravel', {
            enabled = true, movement = segment.movement, yawRate = segment.yawRate,
        })
        local started, deadline = core.getSimulationTime(), core.getRealTime() + 30
        local initialYaw = player.rotation:getYaw()
        local origin, finalPosition, stopOrigin, finalSpeed = player.position, player.position, nil, -1
        local initialState = ragdoll.getState(player)
        local previousCenter = initialState and initialState.centerOfMass
        local commandProgress, horizontalPath, backtracking = 0, 0, 0
        local nextSample = 0
        local previousPhase
        local segmentFinished
        local yawPulseStopped = false
        repeat
            coroutine.yield()
            local elapsed = core.getSimulationTime() - started
            if index == 3 and transitionTrial.yawDuration and not yawPulseStopped
                and elapsed >= transitionTrial.yawDuration then
                player:sendEvent('ragdollPlayerTravel', {
                    enabled = true, movement = segment.movement, yawRate = 0,
                })
                yawPulseStopped = true
            end
            segmentFinished = elapsed >= segment.duration
            local state = ragdoll.getState(player)
            if state and state.controller then
                if transitionTrial.disturbed and index == 3 then
                    local yaw = player.rotation:getYaw()
                    local right = util.vector3(math.cos(yaw), -math.sin(yaw), 0)
                    local forward = util.vector3(math.sin(yaw), math.cos(yaw), 0)
                    local relativeHand = findBody(state, 'right_hand').position - findBody(state, 'right_upper_arm').position
                    if elapsed >= 0.7 and elapsed < 2.4 and ragdoll.getSwingSpeed(player) == nil then
                        missingHeldSwing = missingHeldSwing + 1
                    end
                    if elapsed >= 0.5 and not raisedArm then
                        armRestZ = relativeHand.z
                        if transitionTrial.nativeMouse then
                            ragdoll.injectMouseInput(0, -26, true)
                        else
                            ragdoll.driveSwing(player, true, right, forward, util.vector3(0, 0, 1), 0, 0.1, true, 1 / 60)
                        end
                        raisedArm = true
                    elseif elapsed >= 1.5 and not loweredArm then
                        armRaisedZ = relativeHand.z
                        if state.arms and state.arms.right then
                            armRaisedTargetZ = (state.arms.right.targetWorld
                                - findBody(state, 'right_upper_arm').position).z
                        end
                        if transitionTrial.nativeMouse then
                            ragdoll.injectMouseInput(0, 51)
                        else
                            ragdoll.driveSwing(player, true, right, forward, util.vector3(0, 0, 1), 0, -0.2, true, 1 / 60)
                        end
                        loweredArm = true
                    elseif elapsed >= 2.5 and not releasedArm then
                        armLoweredZ = relativeHand.z
                        if transitionTrial.nativeMouse then
                            ragdoll.injectMouseInput(0, 0, false)
                        else
                            ragdoll.driveSwing(player, true, right, forward, util.vector3(0, 0, 1), 0, 0, false, 1 / 60)
                        end
                        releasedArm = true
                    end
                    if elapsed >= 3 and not pushed then
                        for bodyIndex, body in ipairs(state.bodies) do
                            if body.name == 'pelvis' then
                                ragdoll.applyImpulse(player, bodyIndex, right * 500)
                                pushed = true
                                break
                            end
                        end
                    end
                end
                local c, support = state.controller, state.support
                inspectControllerTelemetry(c, 'player stop/restart', true)
                minimumUp = math.min(minimumUp, c.pelvisUpDot)
                if c.mode == 'falling' or c.mode == 'fallen' or c.mode == 'getting_up' or c.mode == 'blend_back' then
                    falling = falling + 1
                end
                finalPosition = state.centerOfMass
                if previousCenter then
                    local delta = finalPosition - previousCenter
                    local horizontalDelta = util.vector3(delta.x, delta.y, 0)
                    horizontalPath = horizontalPath + horizontalDelta:length()
                    -- Use the actual native travel request, not camera yaw or
                    -- total path length, to distinguish progress from sway.
                    local request = c.rawTargetHorizontalVelocity
                    local horizontalRequest = util.vector3(request.x, request.y, 0)
                    local requestLength = horizontalRequest:length()
                    if requestLength > 0 then
                        local progress = horizontalDelta:dot(horizontalRequest) / requestLength
                        commandProgress = commandProgress + progress
                        backtracking = backtracking + math.max(0, -progress)
                    end
                end
                previousCenter = finalPosition
                finalSpeed = util.vector3(c.centerOfMassVelocity.x, c.centerOfMassVelocity.y, 0):length()
                if segment.movement == 0 and elapsed >= segment.duration - 1 then
                    stopOrigin = stopOrigin or finalPosition
                end
                if elapsed >= nextSample or c.gaitPhase ~= previousPhase then
                    local arm = state.arms and state.arms.right
                    local shoulder = findBody(state, 'right_upper_arm').position
                    local hand = findBody(state, 'right_hand').position - shoulder
                    local armTarget = arm and arm.targetWorld - shoulder
                    local leftFoot = findBody(state, 'left_foot')
                    local rightFoot = findBody(state, 'right_foot')
                    local leftImpulse, leftUnknown = footVerticalImpulse(state, 'left_foot')
                    local rightImpulse, rightUnknown = footVerticalImpulse(state, 'right_foot')
                    emitTelemetry('third_person_player_transition_sample', {
                        segment = index, movement = segment.movement, elapsed = elapsed, mode = c.mode,
                        left_vertical_impulse = leftImpulse, right_vertical_impulse = rightImpulse,
                        left_impulse_unknown = leftUnknown, right_impulse_unknown = rightUnknown,
                        velocity_x = c.centerOfMassVelocity.x, velocity_y = c.centerOfMassVelocity.y,
                        physics_step = state.physicsStep, actuation_stage = c.gaitActuationStage,
                        phase = c.gaitPhase, phase_time = c.gaitTimeInPhase,
                        swing_lifted = c.gaitSwingFootLifted,
                        swing_contact_time = c.gaitSwingContactCandidateTime,
                        transitioned = c.gaitTransitioned,
                        left_x = leftFoot.position.x, left_y = leftFoot.position.y,
                        right_x = rightFoot.position.x, right_y = rightFoot.position.y,
                        left_vertical_velocity = leftFoot.linearVelocity.z,
                        right_vertical_velocity = rightFoot.linearVelocity.z,
                        target_x = c.swingFootTarget and c.swingFootTarget.x,
                        target_y = c.swingFootTarget and c.swingFootTarget.y,
                        raw_speed = c.rawTargetHorizontalVelocity:length(),
                        raw_x = c.rawTargetHorizontalVelocity.x, raw_y = c.rawTargetHorizontalVelocity.y,
                        command_speed = c.targetHorizontalVelocity:length(),
                        planned_speed = c.plannedHorizontalVelocity:length(),
                        speed = finalSpeed, pelvis_up = c.pelvisUpDot, pelvis_height = c.pelvisHeight,
                        height_settled = c.pelvisHeightSettled,
                        yaw = player.rotation:getYaw(), heading_error = c.headingError,
                        com_x = finalPosition.x, com_y = finalPosition.y,
                        actor_x = player.position.x, actor_y = player.position.y,
                        command_progress = commandProgress, horizontal_path = horizontalPath,
                        backtracking = backtracking,
                        support_distance = support and support.centerOfMassDistance or -1,
                        left_support = support and support.leftFoot or false,
                        right_support = support and support.rightFoot or false,
                        hand_x = hand.x, hand_y = hand.y, hand_z = hand.z,
                        hand_target_x = armTarget and armTarget.x,
                        hand_target_y = armTarget and armTarget.y,
                        hand_target_z = armTarget and armTarget.z,
                        arm_weight = arm and arm.weight, arm_brace = arm and arm.brace,
                        arm_applied = arm and arm.applied,
                    })
                    nextSample = elapsed + 0.1
                    previousPhase = c.gaitPhase
                end
            else
                missing = missing + 1
            end
        until segmentFinished or core.getSimulationTime() - started >= segment.duration + 2
            or core.getRealTime() >= deadline
        timedOut = timedOut or not segmentFinished
        if segment.yawRate and segment.yawRate ~= 0 then
            turnAngle = math.abs(player.rotation:getYaw() - initialYaw)
        end
        if segment.movement == 0 then
            local delta = stopOrigin and (finalPosition - stopOrigin) or util.vector3(1000, 0, 0)
            stoppedDrifts[#stoppedDrifts + 1] = util.vector3(delta.x, delta.y, 0):length()
            stoppedSpeeds[#stoppedSpeeds + 1] = finalSpeed
        else
            local delta = player.position - origin
            walkingDistances[#walkingDistances + 1] = segment.yawRate and segment.yawRate ~= 0
                and util.vector3(delta.x, delta.y, 0):length() or delta.y
            walkingProgress[#walkingProgress + 1] = commandProgress
            walkingPaths[#walkingPaths + 1] = horizontalPath
            walkingBacktracking[#walkingBacktracking + 1] = backtracking
        end
    end
    local transitionReplay = transitionTrial.replay and ragdoll.finishReplayRecording() or nil
    local swingReleased = ragdoll.getSwingSpeed(player) == nil
    playerDiagnostics.endObservation()
    player:sendEvent('ragdollPlayerTravel', { enabled = false })
    ragdoll.setPowered(player, false)
    coroutine.yield()
    emitTelemetry('third_person_player_stop_restart', {
        falling_frames = falling, missing_frames = missing, minimum_up = minimumUp,
        first_stop_drift = stoppedDrifts[1], second_stop_drift = stoppedDrifts[2],
        first_stop_speed = stoppedSpeeds[1], second_stop_speed = stoppedSpeeds[2],
        first_walk_distance = walkingDistances[1], second_walk_distance = walkingDistances[2], turn_angle = turnAngle,
        first_walk_progress = walkingProgress[1], second_walk_progress = walkingProgress[2],
        first_walk_path = walkingPaths[1], second_walk_path = walkingPaths[2],
        first_walk_backtracking = walkingBacktracking[1], second_walk_backtracking = walkingBacktracking[2],
        arm_raised = raisedArm, arm_lowered = loweredArm, arm_released = releasedArm,
        impulse_applied = pushed, swing_finished = swingReleased,
        arm_rest_z = armRestZ, arm_raised_z = armRaisedZ, arm_lowered_z = armLoweredZ,
        arm_raised_target_z = armRaisedTargetZ,
        missing_held_swing = missingHeldSwing,
    })
    if transitionTrial.replay then
        testing.expect(transitionReplay ~= nil and transitionReplay.frameCount > 1000, 'transition replay incomplete')
    end
    testing.expect(not timedOut, 'player transition observation timed out')
    testing.expectEqual(missing, 0, 'player rig disappeared during stop/restart')
    testing.expectEqual(falling, 0, 'player fell during stop/restart')
    testing.expectGreaterOrEqual(minimumUp, 0.87, 'player stumbled during stop/restart')
    if transitionTrial.disturbed then
        testing.expect(raisedArm and loweredArm and releasedArm and pushed, 'mixed control sequence did not execute')
        testing.expect(swingReleased, 'mixed control sequence left the arm swing active')
        testing.expectEqual(missingHeldSwing, 0, 'mixed control sequence lost its held arm input')
        testing.expectGreaterOrEqual(armRaisedZ - armRestZ, 3, 'mixed control input did not raise the hand')
        if transitionTrial.nativeMouse then
            -- Native gestures start at the measured hand. Require delivered
            -- upward target movement independently of physical tracking.
            testing.expectGreaterOrEqual(armRaisedTargetZ - armRestZ, 1,
                'initial mouse press-and-motion did not move the target above the activation hand')
        end
        testing.expectGreaterOrEqual(armRaisedZ - armLoweredZ, 6, 'mixed control reversal did not lower the hand')
    end
    if transitionTrial.yawRate ~= 0 then
        testing.expectGreaterOrEqual(turnAngle, 1.5, 'player did not execute the requested turn')
        testing.expectLessOrEqual(turnAngle, 1.85, 'camera and controls applied the requested turn twice')
    end
    for index = 1, 2 do
        testing.expectGreaterOrEqual(walkingDistances[index], 60, 'player failed to start/restart forward')
        testing.expectGreaterOrEqual(walkingProgress[index], 60,
            'player motion did not progress along the native travel request')
        testing.expectLessOrEqual(stoppedDrifts[index], 6, 'player drifted in the final second after stopping')
        testing.expectLessOrEqual(stoppedSpeeds[index], 10, 'player did not settle after stopping')
    end
end)
end

local function pelvisBodyIndex(state)
    for index, body in ipairs(state.bodies) do
        if body.name == 'pelvis' then
            return index, body
        end
    end
    return nil
end

-- Observe a set of walkers for `seconds` of simulation time; returns per-walker
-- falling frames, minimum pelvis up while driven, and horizontal travel.
local function observeWalkers(walkers, seconds, onFrame)
    local start = core.getSimulationTime()
    local realDeadline = core.getRealTime() + seconds * 12 + 60
    local startPositions, stats = {}, {}
    for i, walker in ipairs(walkers) do
        startPositions[i] = walker.position
        stats[i] = { falling = 0, frames = 0, minimumUp = 1, travelled = 0 }
    end
    repeat
        local now = core.getSimulationTime() - start
        for i, walker in ipairs(walkers) do
            local state = walker:isValid() and ragdoll.getState(walker)
            if state and state.controller then
                local s = stats[i]
                s.frames = s.frames + 1
                local mode = state.controller.mode
                if mode == 'falling' or mode == 'fallen' or mode == 'getting_up' then
                    s.falling = s.falling + 1
                elseif state.controller.targetHorizontalVelocity:length() > 1 then
                    s.minimumUp = math.min(s.minimumUp, state.controller.pelvisUpDot)
                end
                local d = walker.position - startPositions[i]
                s.travelled = math.sqrt(d.x * d.x + d.y * d.y)
            end
        end
        if onFrame then onFrame(now) end
        coroutine.yield()
    until now >= seconds or core.getRealTime() >= realDeadline
    return stats
end

registerRagdollTest('stress: three walkers travel side by side for twenty seconds', function()
    local walkers = {
        spawnGameplayWalker(util.vector3(-90, 0, 0)),
        spawnGameplayWalker(util.vector3(0, 0, 0)),
        spawnGameplayWalker(util.vector3(90, 0, 0)),
    }
    for _, walker in ipairs(walkers) do
        gaitCommandAcknowledged = false
        walker:sendEvent('startRagdollTravel', 1)
        waitFor('stress walker movement command', function() return gaitCommandAcknowledged end)
    end
    local stats = observeWalkers(walkers, 20)
    for i, s in ipairs(stats) do
        testing.expectEqual(s.falling, 0, 'stress walker ' .. i .. ' fell during the twenty-second parallel walk')
        testing.expectGreaterOrEqual(s.travelled, 300, 'stress walker ' .. i .. ' did not travel twenty seconds of walking')
        testing.expectGreaterThan(s.minimumUp, 0.85, 'stress walker ' .. i .. ' tilted beyond the walking limit')
    end
    emitTelemetry('stress_parallel_walkers', {
        walkers = #walkers,
        travelled_1 = stats[1].travelled, travelled_2 = stats[2].travelled, travelled_3 = stats[3].travelled,
        minimum_up = math.min(stats[1].minimumUp, stats[2].minimumUp, stats[3].minimumUp),
    })
    for _, walker in ipairs(walkers) do
        ragdoll.setPowered(walker, false)
        walker:remove()
    end
end)

registerRagdollTest('stress: walker absorbs six random pushes during a fifteen-second walk', function()
    local walker = spawnGameplayWalker()
    gaitCommandAcknowledged = false
    walker:sendEvent('startRagdollTravel', 1)
    waitFor('stress push walker movement command', function() return gaitCommandAcknowledged end)
    -- Fixed pseudo-random schedule so the run repeats: time, direction (rad),
    -- whole-body delta-v in m/s (0.1 is the manual push, 0.2 is about the
    -- 1000-unit offline shove).
    local schedule = {
        { at = 2.5, angle = 0.4, dv = 0.15 }, { at = 5.0, angle = 2.1, dv = 0.2 },
        { at = 7.0, angle = 3.9, dv = 0.1 }, { at = 9.0, angle = 5.5, dv = 0.2 },
        { at = 11.0, angle = 1.3, dv = 0.15 }, { at = 13.0, angle = 4.7, dv = 0.2 },
    }
    local applied = 0
    local nextPush = 1
    local stats = observeWalkers({ walker }, 15, function(now)
        local push = schedule[nextPush]
        if push and now >= push.at then
            local state = ragdoll.getState(walker)
            local index, pelvis
            if state then
                index, pelvis = pelvisBodyIndex(state)
            end
            if index then
                local impulse = state.totalMass * push.dv * openMwUnitsPerMeter
                ragdoll.applyImpulse(walker, index,
                    util.vector3(math.cos(push.angle) * impulse, math.sin(push.angle) * impulse, 0), pelvis.centerOfMass)
                applied = applied + 1
            end
            nextPush = nextPush + 1
        end
    end)
    local s = stats[1]
    testing.expectEqual(applied, #schedule, 'not every scheduled push was applied')
    testing.expectEqual(s.falling, 0, 'the walker fell under the random push schedule')
    testing.expectGreaterOrEqual(s.travelled, 200, 'the pushed walker did not keep travelling')
    emitTelemetry('stress_random_pushes', { pushes = applied, travelled = s.travelled,
        minimum_up = s.minimumUp })
    ragdoll.setPowered(walker, false)
    walker:remove()
end)

registerRagdollTest('stress: walker soaks for forty seconds', function()
    local walker = spawnGameplayWalker()
    gaitCommandAcknowledged = false
    walker:sendEvent('startRagdollTravel', 1)
    waitFor('stress soak movement command', function() return gaitCommandAcknowledged end)
    local stats = observeWalkers({ walker }, 40)
    local s = stats[1]
    testing.expectEqual(s.falling, 0, 'the walker fell during the forty-second soak')
    testing.expectGreaterOrEqual(s.travelled, 700, 'the soak walker did not keep a walking pace for forty seconds')
    testing.expectGreaterThan(s.minimumUp, 0.85, 'the soak walker tilted beyond the walking limit')
    emitTelemetry('stress_soak', { seconds = 40, travelled = s.travelled, minimum_up = s.minimumUp })
    ragdoll.setPowered(walker, false)
    walker:remove()
end)

-- Steering (plan item 1.4): the walker follows a scripted path on the
-- reference route: straight, a 90 degree turn while walking, straight on the
-- new heading, then a stop. Gates: no fall, a real heading change between the
-- two straight legs, and standing still after the stop.
-- Get-up (plan item 2.3f): a fallen actor asks to get back up by itself.
-- Recovery used to be strictly opt-in and nothing in ordinary gameplay ever
-- issued the request, so a knocked-down actor stayed down. The range turns
-- the behaviour off for every other diagnostic here, because an unaided
-- get-up would end the fixed observation window they each need; this test
-- turns it back on and is the one that proves it works.
for _, inputCase in ipairs({ 'npc', 'player', 'mouse', 'mouse_rotated', 'mouse_reversed',
    'mouse_horizontal', 'mouse_horizontal_rotated', 'mouse_horizontal_reversed',
    'mouse_saturated', 'mouse_horizontal_saturated', 'mouse_press_motion' }) do
local playerCase, mouseCase = inputCase ~= 'npc', inputCase:find('^mouse') ~= nil
local horizontalCase = inputCase:find('horizontal') ~= nil
local saturationCase = inputCase:find('saturated') ~= nil
local pressMotionCase = inputCase == 'mouse_press_motion'
local inputYaw = inputCase:find('rotated$') and math.pi / 2 or inputCase:find('reversed$') and math.pi or 0
local mouseSuffix = inputCase:find('rotated$') and ' at rotated heading'
    or inputCase:find('reversed$') and ' at reversed heading'
    or saturationCase and ' after reaching limit' or pressMotionCase and ' on initial press' or ''
local mouseName = horizontalCase and 'third person player mouse moves right left and releases the physical hand'
    or 'third person player mouse raises lowers and releases the physical hand'
registerRagdollTest(mouseCase and mouseName .. mouseSuffix
    or playerCase and 'third person player raises lowers and releases the physical hand'
    or 'swing input raises lowers and releases the physical hand', function()
    local actor
    if playerCase then
        actor = world.players[1]
        actor:teleport(actor.cell, testPlayerOrigin, { onGround = true, rotation = util.transform.rotateZ(inputYaw) })
        coroutine.yield()
        testing.runLocalTest(actor, 'ragdoll third person')
        ragdoll.setPowered(actor, true)
        waitFor('player supported standing before arm input', function()
            local state = ragdoll.getState(actor)
            return state and state.controller and state.controller.mode == 'standing'
                and state.controller.pelvisHeightSettled and state.support.leftFoot and state.support.rightFoot
        end)
    else
        actor = spawnGameplayWalker()
    end
    if mouseCase then prepareVerifiedMouseInput() end
    local startingYaw = actor.rotation:getYaw()
    local function sample()
        local state = ragdoll.getState(actor)
        testing.expect(state and state.powered, 'swing lost the powered body')
        local hand, shoulder
        for _, body in ipairs(state.bodies) do
            if body.name == 'right_hand' then hand = body.position end
            if body.name == 'right_upper_arm' then shoulder = body.position end
        end
        testing.expect(hand and shoulder, 'missing right arm bodies')
        ragdoll.getArmReach(actor, true) -- optional native diagnostic snapshot
        return hand - shoulder, state
    end
    local fallingFrames = 0
    local inputPhase = 'guard'
    local function advance(seconds)
        local start = core.getSimulationTime()
        local nextArmSample = 0
        while core.getSimulationTime() - start < seconds do
            local state = ragdoll.getState(actor)
            local elapsed = core.getSimulationTime() - start
            if state and state.arms and state.arms.right and elapsed >= nextArmSample then
                local shoulder = findBody(state, 'right_upper_arm')
                local hand = findBody(state, 'right_hand')
                local relative = hand.position - shoulder.position
                local target = state.arms.right.targetWorld - shoulder.position
                local elbow = findBody(state, 'right_forearm')
                emitTelemetry('swing_tracking_sample', {
                    phase = inputPhase, elapsed = elapsed, physics_step = state.physicsStep,
                    hand_x = relative.x, hand_y = relative.y, hand_z = relative.z,
                    elbow_x = elbow and (elbow.position - shoulder.position).x,
                    elbow_y = elbow and (elbow.position - shoulder.position).y,
                    elbow_z = elbow and (elbow.position - shoulder.position).z,
                    target_x = target.x, target_y = target.y, target_z = target.z,
                    target_distance_model = state.arms.right.targetDistanceModel,
                    minimum_reach_model = state.arms.right.minimumReachModel,
                    maximum_reach_model = state.arms.right.maximumReachModel,
                    tracking_error = (relative - target):length(),
                    -- The accessor also emits joint diagnostics when the
                    -- bounded test is run with OPENMW_RAGDOLL_ARM_TRACE.
                    ik_reach_error = ragdoll.getArmReach(actor, true),
                    weight = state.arms.right.weight, brace = state.arms.right.brace,
                    applied = state.arms.right.applied,
                    mode = state.controller and state.controller.mode,
                })
                nextArmSample = elapsed + (pressMotionCase and inputPhase ~= 'release' and 0.05 or 0.25)
            end
            if state and state.controller and (state.controller.mode == 'falling'
                or state.controller.mode == 'fallen' or state.controller.mode == 'getting_up') then
                fallingFrames = fallingFrames + 1
            end
            coroutine.yield()
        end
    end
    local previousHeld
    local function drive(up, held)
        if mouseCase then
            local button
            if previousHeld ~= held then button = held end
            ragdoll.injectMouseInput(horizontalCase and math.floor(up * 256 + 0.5) or 0,
                horizontalCase and 0 or math.floor(-up * 256 + 0.5), button)
            previousHeld = held
            return
        end
        ragdoll.driveSwing(actor, true, util.vector3(1, 0, 0), util.vector3(0, 1, 0),
            util.vector3(0, 0, 1), 0, up, held, 1 / 60)
    end
    if not pressMotionCase then
        drive(0, true)
        advance(1)
    end
    local rest = sample()
    local heldSurvivedIdle = ragdoll.getSwingSpeed(actor) ~= nil
    inputPhase = 'first_stroke'
    drive(saturationCase and 2 or 0.1, true)
    advance(1)
    local raised, raisedState = sample()
    if pressMotionCase then heldSurvivedIdle = ragdoll.getSwingSpeed(actor) ~= nil end
    local raisedTargetZ = raisedState.arms and raisedState.arms.right
        and (raisedState.arms.right.targetWorld - findBody(raisedState, 'right_upper_arm').position).z or -999
    inputPhase = 'reverse_stroke'
    drive(saturationCase and -0.1 or -0.2, true)
    advance(1)
    local lowered = sample()
    inputPhase = 'release'
    drive(0, false)
    advance(4)
    local released = ragdoll.getSwingSpeed(actor)
    ragdoll.getArmReach(actor, true) -- bounded diagnostic also verifies released motor settings
    emitTelemetry('swing_cardinal', { rest_x = rest.x, rest_y = rest.y, rest_z = rest.z,
        raised_x = raised.x, raised_y = raised.y, raised_z = raised.z,
        lowered_x = lowered.x, lowered_y = lowered.y, lowered_z = lowered.z,
        raised_target_z = raisedTargetZ, initial_press = pressMotionCase,
        released = released == nil, falling_frames = fallingFrames, player = playerCase,
        held_survived_idle = heldSurvivedIdle, native_mouse = mouseCase, horizontal = horizontalCase, saturation = saturationCase, initial_yaw = startingYaw })
    if playerCase then playerDiagnostics.endObservation() end
    ragdoll.setPowered(actor, false)
    coroutine.yield()
    if not playerCase then actor:remove() end
    testing.expect(heldSurvivedIdle, 'idle native input cancelled the held swing')
    if pressMotionCase then
        testing.expectGreaterOrEqual(raisedTargetZ - rest.z, 1,
            'press-and-motion failed to move the target above the measured activation hand')
    end
    local depthTravel = (raised.x - rest.x) * math.sin(startingYaw) + (raised.y - rest.y) * math.cos(startingYaw)
    local sidewaysTravel = (raised.x - rest.x) * math.cos(startingYaw) - (raised.y - rest.y) * math.sin(startingYaw)
    if horizontalCase then
        local reversal = (raised.x - lowered.x) * math.cos(startingYaw) - (raised.y - lowered.y) * math.sin(startingYaw)
        testing.expectGreaterOrEqual(sidewaysTravel, 3, 'mouse right did not move the physical hand right')
        testing.expectGreaterOrEqual(reversal, saturationCase and 3 or 6, 'reverse input did not move the physical hand left')
        testing.expectGreaterOrEqual(sidewaysTravel, math.abs(depthTravel), 'mouse right mostly moved the hand in depth')
        testing.expectGreaterOrEqual(sidewaysTravel, math.abs(raised.z - rest.z), 'mouse right mostly moved the hand vertically')
    else
        testing.expectGreaterOrEqual(raised.z - rest.z, 3, 'mouse up did not raise the physical hand')
        if saturationCase then
            testing.expectGreaterOrEqual(raised.z, 0,
                'maximum upward input cannot raise the physical hand to shoulder height')
            local raisedDepth = raised.x * math.sin(startingYaw) + raised.y * math.cos(startingYaw)
            testing.expectGreaterOrEqual(raisedDepth, -2,
                'maximum upward input put the physical hand behind the shoulder plane')
        end
        testing.expectGreaterOrEqual(raised.z - lowered.z, saturationCase and 3 or 6, 'reverse input did not lower the physical hand')
        testing.expectGreaterOrEqual(raised.z - rest.z, math.abs(depthTravel),
            'upward input moved the hand more in depth than upward')
        testing.expectGreaterOrEqual(raised.z - rest.z, math.abs(sidewaysTravel),
            'upward input moved the hand more sideways than upward')
    end
    testing.expect(released == nil, 'release did not end the swing')
    testing.expectEqual(fallingFrames, 0, 'arm movement toppled the actor')
end)
end

registerRagdollTest('third person player recovery keeps the physical body and rendering', function()
    local player = world.players[1]
    testing.runLocalTest(player, 'ragdoll third person')
    ragdoll.setPowered(player, true)
    waitFor('supported player before recovery lifecycle test', function()
        local state = ragdoll.getState(player)
        return state and state.controller and state.controller.mode == 'standing'
            and state.controller.pelvisHeightSettled
    end)
    ragdoll.setAutomaticRecovery(true)
    interfaces.RagdollDev.fall(player)
    local started, previousTime = core.getSimulationTime(), core.getSimulationTime()
    local deadline = core.getRealTime() + 30
    local missing, renderExcess = 0, 0
    local physicsRenderError, skeletonRenderError, staleRenderFrames = 0, 0, 0
    local worstRenderSample
    local gettingUp, settling, standing = false, false, false
    local previousRecoveryMode
    local previousSpeeds = {}
    repeat
        coroutine.yield()
        local now = core.getSimulationTime()
        local state = ragdoll.getState(player)
        if not state or not state.powered or not state.controller then
            missing = missing + 1
        else
            local mode = state.controller.mode
            if mode ~= previousRecoveryMode then
                emitTelemetry('powered_recovery_transition', {
                    seconds = now - started, physics_step = state.physicsStep, mode = mode,
                    pelvis_up = state.controller.pelvisUpDot,
                    pelvis_height = state.controller.pelvisHeight,
                    speed = state.controller.centerOfMassVelocity:length(),
                    animation_started = state.controller.recoveryAnimationStarted,
                    animation_finished = state.controller.recoveryAnimationFinished,
                })
                previousRecoveryMode = mode
            end
            gettingUp = gettingUp or mode == 'getting_up'
            settling = settling or mode == 'blend_back'
            standing = standing or (gettingUp and mode == 'standing')
            if gettingUp and state.renderPhysicsStep ~= state.physicsStep then
                staleRenderFrames = staleRenderFrames + 1
            end
            for index, body in ipairs(state.bodies) do
                local speed = body.linearVelocity:length()
                    + body.angularVelocity:length() * (body.position - body.centerOfMass):length()
                if gettingUp and body.renderPosition then
                    testing.expect(body.physicsRenderPosition and body.skeletonPosition,
                        'recovery render check is missing physics interpolation or skeleton readback')
                    physicsRenderError = math.max(physicsRenderError,
                        (body.renderPosition - body.physicsRenderPosition):length())
                    skeletonRenderError = math.max(skeletonRenderError,
                        (body.skeletonPosition - body.renderPosition):length())
                    -- Retain the old velocity-based estimate for diagnosis.
                    -- Position corrections can exceed it; the direct physics
                    -- interpolation and skeleton comparisons decide acceptance.
                    local allowance = math.max(speed, previousSpeeds[index] or speed)
                        * math.max(1 / 60, now - previousTime)
                    local distance = (body.renderPosition - body.position):length()
                    local excess = distance - allowance
                    if excess > renderExcess then
                        renderExcess = excess
                        worstRenderSample = { seconds = now - started, physics_step = state.physicsStep,
                            mode = mode, body = body.name, distance = distance, allowance = allowance,
                            speed = speed, previous_speed = previousSpeeds[index], render_dt = now - previousTime,
                            physical_x = body.position.x, physical_y = body.position.y, physical_z = body.position.z,
                            render_x = body.renderPosition.x, render_y = body.renderPosition.y, render_z = body.renderPosition.z }
                    end
                end
                previousSpeeds[index] = speed
            end
        end
        previousTime = now
    until core.getSimulationTime() - started >= 15 or core.getRealTime() >= deadline
    emitTelemetry('powered_recovery_lifecycle', { missing = missing, getting_up = gettingUp,
        settling = settling, standing = standing, render_excess = renderExcess,
        physics_render_error = physicsRenderError, skeleton_render_error = skeletonRenderError,
        stale_render_frames = staleRenderFrames,
        seconds = core.getSimulationTime() - started })
    if worstRenderSample then emitTelemetry('powered_recovery_render_worst', worstRenderSample) end
    playerDiagnostics.endObservation()
    ragdoll.setAutomaticRecovery(false)
    ragdoll.setPowered(player, false)
    coroutine.yield()
    testing.expectGreaterOrEqual(core.getSimulationTime() - started, 15, 'recovery observation timed out')
    testing.expect(gettingUp and settling, 'test did not exercise the recovery lifecycle')
    testing.expectEqual(missing, 0, 'recovery discarded the powered physical body')
    testing.expectEqual(staleRenderFrames, 0, 'recovery rendered an outdated physics snapshot')
    testing.expectLessOrEqual(physicsRenderError, 2, 'recovery render pose diverged from interpolated physics')
    testing.expectLessOrEqual(skeletonRenderError, 2, 'recovery skeleton diverged from the published physical pose')
end)

registerRagdollTest('REC-03 interrupts retries and resumes recovery', function()
    local player = world.players[1]
    testing.runLocalTest(player, 'ragdoll third person')
    ragdoll.setPowered(player, true)
    waitFor('supported player before interrupted recovery', function()
        local state = ragdoll.getState(player)
        return state and state.controller and state.controller.mode == 'standing'
            and state.controller.pelvisHeightSettled
    end)
    ragdoll.setAutomaticRecovery(true)
    interfaces.RagdollDev.fall(player)
    local started, deadline = core.getSimulationTime(), core.getRealTime() + 150
    local interrupted, hitRecorded, supportLost, fallingAfterHit = false, false, false, false
    local aborted, flagsReset, retryRequested, secondRise, standing = false, false, false, false, false
    local missing, previousMode = 0, nil
    local standingSince
    local hitTime, fallingTime, retryTime, secondRiseTime
    local retryPending, retryCount, riseCount, maximumRetryDelay = false, 0, 0, 0
    local lastRetryTime
    local nextActuationSample, actuationSamples = 0, 0
    repeat
        coroutine.yield()
        local state = ragdoll.getState(player)
        if not state or not state.powered or not state.controller then
            missing = missing + 1
        else
            local c = state.controller
            local elapsed = core.getSimulationTime() - started
            local support = state.support or {}
            if interrupted and support.groundContactCount == 0
                and not support.leftFoot and not support.rightFoot then
                supportLost = true
            end
            if interrupted and state.combatReactionCount >= 1 and state.lastCombatReaction
                and state.lastCombatReaction.id == '5001' and state.lastCombatReaction.stunSeconds > 0 then
                hitRecorded = true
            end
            if actuationSamples < 400 and (elapsed >= nextActuationSample or c.mode ~= previousMode) then
                local support, upright = state.support or {}, c.upright or {}
                local leftImpulse, leftUnknown = footVerticalImpulse(state, 'left_foot')
                local rightImpulse, rightUnknown = footVerticalImpulse(state, 'right_foot')
                -- Forces below are controller commands, not measured contact
                -- forces. Foot impulses retain their unknown-data flags.
                emitTelemetry('interrupted_recovery_actuation', {
                    seconds = elapsed, physics_step = state.physicsStep, mode = c.mode,
                    retry = retryRequested, pelvis_up = c.pelvisUpDot,
                    heading_error = c.headingError,
                    target_pelvis_up = c.targetPelvisUpDot, torso_up = c.torsoUpDot,
                    pelvis_height = c.pelvisHeight, target_height = upright.targetPelvisHeight,
                    speed = c.centerOfMassVelocity:length(), motors = c.jointMotorsApplied,
                    upright_applied = upright.applied, left_support = support.leftFoot,
                    right_support = support.rightFoot, ground_contacts = support.groundContactCount,
                    support_distance = support.centerOfMassDistance,
                    left_vertical_impulse = leftImpulse, left_impulse_unknown = leftUnknown,
                    right_vertical_impulse = rightImpulse, right_impulse_unknown = rightUnknown,
                    force_command_z = upright.pelvisForce and upright.pelvisForce.z,
                    force_command_x = upright.pelvisForce and upright.pelvisForce.x,
                    force_command_y = upright.pelvisForce and upright.pelvisForce.y,
                    submitted_force_z = c.appliedPelvisForce and c.appliedPelvisForce.z,
                    pelvis_torque_command = upright.pelvisTorque and upright.pelvisTorque:length(),
                    spine_torque_command = upright.spineTorque and upright.spineTorque:length(),
                    pelvis_torque_saturated = upright.pelvisTorqueSaturated,
                    spine_torque_saturated = upright.spineTorqueSaturated,
                })
                actuationSamples = actuationSamples + 1
                nextActuationSample = elapsed + 0.1
            end
            if c.mode ~= previousMode then
                emitTelemetry('interrupted_recovery_transition', {
                    seconds = core.getSimulationTime() - started, physics_step = state.physicsStep,
                    mode = c.mode, pelvis_up = c.pelvisUpDot, pelvis_height = c.pelvisHeight,
                    speed = c.centerOfMassVelocity:length(), interrupted = interrupted,
                    animation_started = c.recoveryAnimationStarted,
                    animation_finished = c.recoveryAnimationFinished,
                })
                previousMode = c.mode
            end
            if not interrupted and c.mode == 'blend_back' then
                ragdoll.setAutomaticRecovery(false)
                local pelvis = findBody(state, 'pelvis')
                -- One identified combat impulse at the pelvis centre of mass,
                -- exactly at the calibrated hit-stun threshold. No pose or
                -- controller override creates the interruption.
                local yaw = player.rotation:getYaw()
                local facing = util.vector3(math.sin(yaw), math.cos(yaw), 0)
                ragdoll.applyCombatImpulse(player, facing * 400, pelvis.centerOfMass, 5001)
                interrupted = true
                hitTime = elapsed
            elseif interrupted and not fallingAfterHit and c.mode == 'falling' then
                fallingAfterHit = true
                fallingTime = elapsed
            elseif interrupted and c.mode == 'fallen' and not retryPending and retryCount < 5 then
                local cleared = not c.recoveryAnimationStarted and not c.recoveryAnimationFinished
                flagsReset = aborted and (flagsReset and cleared) or cleared
                aborted = true
                ragdoll.requestRecovery(player)
                retryRequested = true
                retryPending = true
                retryCount = retryCount + 1
                lastRetryTime = elapsed
                retryTime = retryTime or elapsed
            elseif retryPending and c.mode == 'getting_up' then
                secondRise = true
                secondRiseTime = secondRiseTime or elapsed
                riseCount = riseCount + 1
                maximumRetryDelay = math.max(maximumRetryDelay, elapsed - lastRetryTime)
                retryPending = false
            end
            -- An aborted rise can return through Falling/Bracing rather than
            -- the recovery completion gate. A single Standing frame is not
            -- evidence of settled recovery. Match this fixture's physical
            -- completion thresholds and require a continuous observation.
            local settledStanding = interrupted and (not aborted or secondRise)
                and c.mode == 'standing' and c.pelvisUpDot >= 0.9 and c.pelvisHeight > 30
                and c.pelvisHeightSettled
                and c.centerOfMassVelocity:length() <= 20 and state.support
                and state.support.centerOfMassSupported and state.support.centerOfMassDistance == 0
                and (state.support.leftFoot or state.support.rightFoot)
            if settledStanding then
                standingSince = standingSince or elapsed
                standing = elapsed - standingSince >= 1
            else
                standingSince = nil
            end
        end
    until standing or core.getSimulationTime() - started >= 35 or core.getRealTime() >= deadline

    local resumed, resumeTravel, maximumStep = false, 0, 0
    local postRecoveryFall, postRecoveryFallCount, locomotionAttempts = false, 0, 0
    local function waitForFreshStanding()
        local requestPending, stableSince = false, nil
        local waitStarted, waitDeadline = core.getSimulationTime(), core.getRealTime() + 30
        repeat
            coroutine.yield()
            local state = ragdoll.getState(player)
            if not state or not state.powered or not state.controller then
                missing = missing + 1
                return false
            end
            local c = state.controller
            local elapsed = core.getSimulationTime() - started
            if c.mode == 'fallen' and not requestPending and retryCount < 5 then
                local cleared = not c.recoveryAnimationStarted and not c.recoveryAnimationFinished
                flagsReset = flagsReset and cleared
                ragdoll.requestRecovery(player)
                retryCount = retryCount + 1
                lastRetryTime = elapsed
                requestPending = true
            elseif requestPending and c.mode == 'getting_up' then
                riseCount = riseCount + 1
                maximumRetryDelay = math.max(maximumRetryDelay, elapsed - lastRetryTime)
                requestPending = false
            end
            local settled = c.mode == 'standing' and c.pelvisUpDot >= 0.9 and c.pelvisHeight > 30
                and c.pelvisHeightSettled and c.centerOfMassVelocity:length() <= 20
                and state.support and state.support.centerOfMassSupported
                and state.support.centerOfMassDistance == 0
                and (state.support.leftFoot or state.support.rightFoot)
            if settled then
                stableSince = stableSince or core.getSimulationTime()
                if core.getSimulationTime() - stableSince >= 1 then return true end
            else
                stableSince = nil
            end
        until core.getSimulationTime() - waitStarted >= 20 or core.getRealTime() >= waitDeadline
        return false
    end
    while standing and not resumed and locomotionAttempts < 3 do
        locomotionAttempts = locomotionAttempts + 1
        local state = ragdoll.getState(player)
        local origin = findBody(state, 'pelvis').position
        local previous = origin
        local yaw = player.rotation:getYaw()
        local attemptFall, attemptTravel = false, 0
        player:sendEvent('ragdollPlayerTravel', { enabled = true, movement = 1, run = false })
        local resumeStarted, resumeDeadline = core.getSimulationTime(), core.getRealTime() + 15
        repeat
            coroutine.yield()
            state = ragdoll.getState(player)
            if not state or not state.powered or not state.controller then
                missing = missing + 1
            else
                local pelvis = findBody(state, 'pelvis').position
                maximumStep = math.max(maximumStep, (pelvis - previous):length())
                previous = pelvis
                local delta = pelvis - origin
                attemptTravel = delta.x * math.sin(yaw) + delta.y * math.cos(yaw)
                local mode = state.controller.mode
                attemptFall = attemptFall or mode == 'falling' or mode == 'fallen'
                    or mode == 'getting_up' or mode == 'blend_back'
            end
        until missing > 0 or attemptFall or core.getSimulationTime() - resumeStarted >= 5
            or core.getRealTime() >= resumeDeadline
        player:sendEvent('ragdollPlayerTravel', { enabled = false })
        resumeTravel = math.max(resumeTravel, attemptTravel)
        resumed = not attemptFall and core.getSimulationTime() - resumeStarted >= 5
            and attemptTravel >= 60 and maximumStep <= 20
        if attemptFall then
            postRecoveryFall = true
            postRecoveryFallCount = postRecoveryFallCount + 1
            standing = waitForFreshStanding()
        end
    end
    local finalState = ragdoll.getState(player)
    local finalController = finalState and finalState.controller
    emitTelemetry('interrupted_recovery', { interrupted = interrupted, aborted = aborted,
        final_mode = finalController and finalController.mode,
        final_up = finalController and finalController.pelvisUpDot,
        final_height = finalController and finalController.pelvisHeight,
        final_speed = finalController and finalController.centerOfMassVelocity:length(),
        final_heading_error = finalController and finalController.headingError,
        hit_recorded = hitRecorded, support_lost = supportLost, falling_after_hit = fallingAfterHit,
        hit_to_falling_seconds = fallingTime and hitTime and fallingTime - hitTime,
        flags_reset = flagsReset, retry_requested = retryRequested, second_rise = secondRise,
        retry_delay_seconds = secondRiseTime and retryTime and secondRiseTime - retryTime,
        retry_count = retryCount, rise_count = riseCount, maximum_retry_delay = maximumRetryDelay,
        standing = standing, locomotion_resumed = resumed, resume_travel = resumeTravel,
        maximum_step = maximumStep, post_recovery_fall = postRecoveryFall,
        post_recovery_fall_count = postRecoveryFallCount, locomotion_attempts = locomotionAttempts,
        missing = missing, seconds = core.getSimulationTime() - started })
    ragdoll.setAutomaticRecovery(false)
    ragdoll.setPowered(player, false)
    coroutine.yield()
    testing.expectEqual(missing, 0, 'interrupted recovery discarded the powered body')
    testing.expect(interrupted and hitRecorded, 'identified hit did not reach settling recovery')
    testing.expect(fallingAfterHit and aborted, 'hit recovery did not return through Falling and Fallen')
    testing.expectLessOrEqual(fallingTime - hitTime, 0.25, 'recovery did not release motors promptly after hit')
    testing.expect(flagsReset, 'aborted recovery retained stale animation flags')
    testing.expect(retryRequested and secondRise, 'fresh recovery request did not start another rise')
    testing.expectLessOrEqual(maximumRetryDelay, 1, 'recovery retry exceeded its bounded delay')
    testing.expect(standing, 'retried recovery did not reach standing')
    testing.expect(resumed, 'locomotion did not resume after the recovery retry')
    testing.expectLessOrEqual(postRecoveryFallCount, 2, 'locomotion resume exhausted its bounded retries')
    testing.expectGreaterOrEqual(resumeTravel, 60, 'locomotion resume made insufficient progress')
    testing.expectLessOrEqual(maximumStep, 20, 'locomotion resume teleported the powered body')
end)

registerRagdollTest('NATIVE-01 four NPCs fall and recover from identified hits', function()
    local player = world.players[1]
    -- Keep each actor's fall and recovery independent even when two actors
    -- receive the same lateral impulse and travel by different amounts.
    local offsets = { -540, -180, 180, 540 }
    local cohort = {}
    for index, offset in ipairs(offsets) do
        -- A distinct record gives every local actor script its own storage and
        -- keeps one cohort member from inheriting another member's state.
        local record = world.createRecord(types.NPC.createRecordDraft({ template = types.NPC.record(player) }))
        local actor = world.createObject(record.id)
        actor:teleport(player.cell, testActorOrigin + util.vector3(offset, 0, 0), { onGround = true })
        cohort[index] = {
            actor = actor,
            recordId = tostring(record.id),
            reactionId = tostring(6100 + index),
            setupPassed = false,
            setupSince = nil,
            hitRecorded = false,
            fell = false,
            recovered = false,
            missing = false,
            timeToFall = nil,
            timeToStanding = nil,
            standingSince = nil,
        }
    end
    coroutine.yield()
    for _, member in ipairs(cohort) do ragdoll.setPowered(member.actor, true) end

    local setupStarted = core.getSimulationTime()
    local setupDeadline = core.getRealTime() + 60
    repeat
        local allReady = true
        for _, member in ipairs(cohort) do
            local state = member.actor:isValid() and ragdoll.getState(member.actor)
            local supported = state and state.powered and state.controller
                and state.controller.mode == 'standing' and state.controller.pelvisHeightSettled
                and state.support and state.support.centerOfMassSupported
                and (state.support.leftFoot or state.support.rightFoot)
            if supported then
                member.setupSince = member.setupSince or core.getSimulationTime()
                member.setupPassed = core.getSimulationTime() - member.setupSince >= 0.5
            else
                member.setupSince = nil
                member.setupPassed = false
            end
            allReady = allReady and member.setupPassed
        end
        if allReady then break end
        coroutine.yield()
    until core.getSimulationTime() - setupStarted >= 12 or core.getRealTime() >= setupDeadline

    local setupFailures = 0
    local uniqueRecords = true
    for index, member in ipairs(cohort) do
        if not member.setupPassed then setupFailures = setupFailures + 1 end
        for previous = 1, index - 1 do
            if cohort[previous].recordId == member.recordId then uniqueRecords = false end
        end
    end

    local hitStarted = core.getSimulationTime()
    local minimumPairDistance = math.huge
    if setupFailures == 0 then
        ragdoll.setAutomaticRecovery(true)
        for index, member in ipairs(cohort) do
            local state = ragdoll.getState(member.actor)
            local pelvis = state and findBody(state, 'pelvis')
            local direction = index <= 2 and -1 or 1
            ragdoll.applyCombatImpulse(member.actor, util.vector3(direction * 4000, 0, 350),
                pelvis.centerOfMass, 6100 + index)
        end

        local recoveryDeadline = 15
        local observationDeadline = core.getRealTime() + 180
        repeat
            coroutine.yield()
            local elapsed = core.getSimulationTime() - hitStarted
            local allRecovered = true
            for _, member in ipairs(cohort) do
                local state = member.actor:isValid() and ragdoll.getState(member.actor)
                if not state or not state.powered or not state.controller then
                    member.missing = true
                    member.standingSince = nil
                else
                    local controller = state.controller
                    if state.combatReactionCount >= 1 and state.lastCombatReaction
                        and state.lastCombatReaction.id == member.reactionId then
                        member.hitRecorded = true
                    end
                    if not member.fell and (controller.mode == 'falling' or controller.mode == 'fallen') then
                        member.fell = true
                        member.timeToFall = elapsed
                    end
                    local support = state.support
                    local settled = member.fell and controller.mode == 'standing'
                        and controller.pelvisUpDot >= 0.9 and controller.pelvisHeight > 30
                        and controller.pelvisHeightSettled and controller.centerOfMassVelocity:length() <= 20
                        and support and support.centerOfMassSupported and support.centerOfMassDistance == 0
                        and (support.leftFoot or support.rightFoot)
                    if settled then
                        member.standingSince = member.standingSince or elapsed
                        if elapsed <= recoveryDeadline
                            and elapsed - member.standingSince >= 0.5 and not member.recovered then
                            member.recovered = true
                            member.timeToStanding = elapsed
                        end
                    else
                        member.standingSince = nil
                    end
                end
                allRecovered = allRecovered and member.recovered
            end
            for left = 1, #cohort - 1 do
                for right = left + 1, #cohort do
                    if cohort[left].actor:isValid() and cohort[right].actor:isValid() then
                        local separation = cohort[left].actor.position - cohort[right].actor.position
                        minimumPairDistance = math.min(minimumPairDistance,
                            math.sqrt(separation.x * separation.x + separation.y * separation.y))
                    end
                end
            end
            if allRecovered then break end
        until core.getSimulationTime() - hitStarted >= recoveryDeadline
            or core.getRealTime() >= observationDeadline
    end

    local identifiedHits, falls, recoveries, missing = 0, 0, 0, 0
    local maximumTimeToFall, maximumTimeToStanding = 0, 0
    for index, member in ipairs(cohort) do
        if member.hitRecorded then identifiedHits = identifiedHits + 1 end
        if member.fell then
            falls = falls + 1
            maximumTimeToFall = math.max(maximumTimeToFall, member.timeToFall)
        end
        if member.recovered then
            recoveries = recoveries + 1
            maximumTimeToStanding = math.max(maximumTimeToStanding, member.timeToStanding)
        end
        if member.missing then missing = missing + 1 end
        emitTelemetry('native_fall_recovery_actor', {
            actor_index = index, reaction_id = member.reactionId,
            setup_passed = member.setupPassed, hit_recorded = member.hitRecorded,
            fell = member.fell, recovered = member.recovered, missing = member.missing,
            time_to_fall = member.timeToFall or -1,
            time_to_standing = member.timeToStanding or -1,
        })
    end
    if minimumPairDistance == math.huge then minimumPairDistance = -1 end
    emitTelemetry('native_fall_recovery_cohort', {
        attempts = #cohort, setup_failures = setupFailures, identified_hits = identifiedHits,
        falls = falls, recoveries = recoveries, missing = missing,
        fall_rate = falls / #cohort, recovery_rate = recoveries / #cohort,
        maximum_time_to_fall = maximumTimeToFall,
        maximum_time_to_standing = maximumTimeToStanding,
        minimum_pair_distance = minimumPairDistance,
        unique_records = uniqueRecords,
        observation_seconds = core.getSimulationTime() - hitStarted,
    })

    ragdoll.setAutomaticRecovery(false)
    for _, member in ipairs(cohort) do
        if member.actor:isValid() then
            ragdoll.setPowered(member.actor, false)
            member.actor:remove()
        end
    end
    coroutine.yield()

    testing.expectEqual(#cohort, 4, 'NATIVE-01 cohort size changed')
    testing.expectEqual(setupFailures, 0, 'NATIVE-01 has no valid denominator because setup failed')
    testing.expect(uniqueRecords, 'NATIVE-01 reused an NPC record between cohort members')
    testing.expectEqual(identifiedHits, 4, 'NATIVE-01 lost an identified hit')
    testing.expectEqual(falls, 4, 'NATIVE-01 strong hits did not make every NPC fall')
    testing.expectGreaterOrEqual(recoveries, 3, 'NATIVE-01 recovered fewer than three NPCs')
    testing.expectEqual(missing, 0, 'NATIVE-01 lost a powered body')
    testing.expectLessOrEqual(maximumTimeToFall, 2, 'NATIVE-01 fall transition exceeded two seconds')
    testing.expectLessOrEqual(maximumTimeToStanding, 15, 'NATIVE-01 recovery exceeded fifteen seconds')
    testing.expectGreaterOrEqual(minimumPairDistance, 80, 'NATIVE-01 actors contaminated each other spatially')
end)

registerRagdollTest('REC-01 classifies ten captured-body recovery trials', function()
    local player = world.players[1]
    testing.runLocalTest(player, 'ragdoll third person')

    local function resetPoweredStanding(description)
        ragdoll.setAutomaticRecovery(false)
        ragdoll.setPowered(player, false)
        waitFor(description .. ' physical teardown', function()
            local state = ragdoll.getState(player)
            return not state or not state.powered
        end, 3)
        player:teleport(testArea.cell, testPlayerOrigin, {
            onGround = true, rotation = util.transform.rotateZ(0),
        })
        local restStarted = core.getSimulationTime()
        repeat coroutine.yield() until core.getSimulationTime() - restStarted >= 0.5
        ragdoll.setPowered(player, true)
        local standingSince
        return waitFor(description, function()
            local state = ragdoll.getState(player)
            local supported = state and state.powered and state.controller
                and state.controller.mode == 'standing'
                and state.controller.pelvisHeightSettled and state.support
                and state.support.centerOfMassSupported
                and (state.support.leftFoot or state.support.rightFoot)
            if supported then
                standingSince = standingSince or core.getSimulationTime()
                if core.getSimulationTime() - standingSince >= 0.5 then return state end
            else
                standingSince = nil
            end
        end, 12)
    end

    local control = resetPoweredStanding('supported REC-01 negative control')
    local controlPassed = control.controller.mode == 'standing'
        and not control.controller.recoveryRequested and control.support.centerOfMassSupported
        and (control.support.leftFoot or control.support.rightFoot)
    emitTelemetry('rec01_negative_control', {
        passed = controlPassed, physics_step = control.physicsStep,
        stable_seconds = 0.5,
        mode = control.controller.mode,
        com_supported = control.support.centerOfMassSupported,
        left_foot = control.support.leftFoot, right_foot = control.support.rightFoot,
        recovery_requested = control.controller.recoveryRequested,
    })
    testing.expect(controlPassed, 'unperturbed powered standing failed the REC-01 negative control')

    local completeTrials = 0
    for trial = 1, 10 do
        if trial > 1 then
            resetPoweredStanding('supported player before REC-01 trial ' .. trial)
        end
        ragdoll.setAutomaticRecovery(true)
        interfaces.RagdollDev.fall(player)
        local started = core.getSimulationTime()
        local deadline = core.getRealTime() + 24
        local previousMode, nextSample, stableSince = nil, 0, nil
        local sampleCount, transitionCount = 0, 0
        local missing = false
        local sawFalling, sawFallen, requested = false, false, false
        local poseSelected, gettingUp = false, false
        local animationStarted, animationFinished, blendBack = false, false, false
        local standingReached, sustainedStanding = false, false
        local supportObserved, footSupportObserved, comSupportObserved = false, false, false
        local motorsObserved, maxMotorUtilization, maximumSupportDistance = false, 0, 0
        local selectedPose = 'none'
        repeat
            coroutine.yield()
            local elapsed = core.getSimulationTime() - started
            local state = ragdoll.getState(player)
            if not state or not state.powered or not state.controller then
                missing = true
            else
                local c, support = state.controller, state.support
                local changed = c.mode ~= previousMode
                if changed then transitionCount = transitionCount + 1 end
                sawFalling = sawFalling or c.mode == 'falling' or c.mode == 'bracing'
                    or c.mode == 'stumbling'
                sawFallen = sawFallen or c.mode == 'fallen'
                requested = requested or c.recoveryRequested
                poseSelected = poseSelected or c.recoveryDirection ~= nil
                if c.recoveryDirection then selectedPose = c.recoveryDirection end
                gettingUp = gettingUp or c.mode == 'getting_up'
                animationStarted = animationStarted or c.recoveryAnimationStarted
                animationFinished = animationFinished or c.recoveryAnimationFinished
                blendBack = blendBack or c.mode == 'blend_back'
                standingReached = standingReached or (gettingUp and c.mode == 'standing')
                supportObserved = supportObserved or support ~= nil
                if support then
                    footSupportObserved = footSupportObserved or support.leftFoot or support.rightFoot
                    comSupportObserved = comSupportObserved or support.centerOfMassSupported
                    maximumSupportDistance = math.max(maximumSupportDistance, support.centerOfMassDistance)
                end
                motorsObserved = motorsObserved or c.jointMotorsApplied
                    or c.jointSupportMotorUtilization > 0
                maxMotorUtilization = math.max(maxMotorUtilization, c.jointSupportMotorUtilization)
                local settled = gettingUp and c.mode == 'standing' and c.pelvisUpDot >= 0.9
                    and c.pelvisHeight > 30 and c.centerOfMassVelocity:length() <= 20
                    and support and support.centerOfMassSupported
                    and support.centerOfMassDistance == 0 and (support.leftFoot or support.rightFoot)
                if settled then
                    stableSince = stableSince or elapsed
                    sustainedStanding = elapsed - stableSince >= 0.5
                else
                    stableSince = nil
                end
                if sampleCount < 160 and (changed or elapsed >= nextSample) then
                    emitTelemetry('rec01_sample', {
                        trial = trial, physics_step = state.physicsStep, seconds = elapsed,
                        mode = c.mode, recovery_requested = c.recoveryRequested,
                        recovery_pose = c.recoveryDirection or 'none',
                        animation_started = c.recoveryAnimationStarted,
                        animation_finished = c.recoveryAnimationFinished,
                        motors_applied = c.jointMotorsApplied,
                        motor_utilization = c.jointSupportMotorUtilization,
                        pelvis_up = c.pelvisUpDot, pelvis_height = c.pelvisHeight,
                        speed = c.centerOfMassVelocity:length(),
                        left_foot = support and support.leftFoot or false,
                        right_foot = support and support.rightFoot or false,
                        com_supported = support and support.centerOfMassSupported or false,
                        ground_contacts = support and support.groundContactCount or -1,
                        foot_contacts = support and support.footContactCount or -1,
                        support_distance = support and support.centerOfMassDistance or -1,
                    })
                    sampleCount = sampleCount + 1
                    nextSample = elapsed + 0.1
                end
                previousMode = c.mode
            end
        until missing or sustainedStanding or core.getSimulationTime() - started >= 12
            or core.getRealTime() >= deadline

        local failureClass = 'recovered'
        if missing then failureClass = 'body_missing'
        elseif not sawFalling then failureClass = 'fall_not_observed'
        elseif not sawFallen then failureClass = 'fallen_not_observed'
        elseif not supportObserved then failureClass = 'support_telemetry_missing'
        elseif not requested then failureClass = 'recovery_request_missing'
        elseif not poseSelected then failureClass = 'pose_selection_missing'
        elseif not gettingUp then failureClass = 'getting_up_missing'
        elseif not motorsObserved then failureClass = 'motor_authority_missing'
        elseif not animationStarted then failureClass = 'animation_start_missing'
        elseif not animationFinished then failureClass = 'animation_finish_missing'
        elseif not blendBack then failureClass = 'blend_back_missing'
        elseif not standingReached then failureClass = 'standing_transition_missing'
        elseif not sustainedStanding then failureClass = 'support_not_sustained'
        end
        emitTelemetry('rec01_trial', {
            trial = trial, recovered = sustainedStanding, failure_class = failureClass,
            seconds = core.getSimulationTime() - started, sample_count = sampleCount,
            transition_count = transitionCount, missing = missing,
            saw_falling = sawFalling, saw_fallen = sawFallen,
            recovery_requested = requested, pose_selected = poseSelected,
            selected_pose = selectedPose, getting_up = gettingUp,
            animation_started = animationStarted, animation_finished = animationFinished,
            blend_back = blendBack, standing_reached = standingReached,
            sustained_standing = sustainedStanding, support_observed = supportObserved,
            foot_support_observed = footSupportObserved,
            com_support_observed = comSupportObserved, motors_observed = motorsObserved,
            max_motor_utilization = maxMotorUtilization,
            max_support_distance = maximumSupportDistance,
        })
        completeTrials = completeTrials + 1
    end
    ragdoll.setAutomaticRecovery(false)
    ragdoll.setPowered(player, false)
    coroutine.yield()
    testing.expectEqual(completeTrials, 10, 'REC-01 did not complete all ten trials')
end)

for _, recoveryTrial in ipairs({ 'repeat', 'resume', 'held' }) do
local resumeWalking = recoveryTrial ~= 'repeat'
local heldMovement = recoveryTrial == 'held'
registerRagdollTest(heldMovement and 'third person player keeps forward held through physical recovery'
    or resumeWalking and 'third person player walks and stops after physical recovery'
    or 'third person player recovers from three consecutive falls', function()
    local player = world.players[1]
    testing.runLocalTest(player, 'ragdoll third person')
    ragdoll.setPowered(player, true)
    waitFor('supported player before repeated recovery', function()
        local state = ragdoll.getState(player)
        return state and state.controller and state.controller.mode == 'standing'
            and state.controller.pelvisHeightSettled
    end)
    ragdoll.setAutomaticRecovery(true)
    local completed, missing, unexpectedFall = 0, false, false
    local requestedCycles = resumeWalking and 1 or 3
    local initialFacing, maximumFacingChange = player.rotation:getYaw(), 0
    local function observeFacing()
        local delta = (player.rotation:getYaw() - initialFacing + math.pi) % (2 * math.pi) - math.pi
        maximumFacingChange = math.max(maximumFacingChange, math.abs(delta))
    end
    if heldMovement then
        player:sendEvent('ragdollPlayerTravel', { enabled = true, movement = 1, run = false })
    end
    for cycle = 1, requestedCycles do
        interfaces.RagdollDev.fall(player)
        local started, deadline = core.getSimulationTime(), core.getRealTime() + 18
        local gettingUp, stableSince = false, nil
        local recovered = false
        local nextSample, longestStanding = 0, 0
        local recoverySamples = 0
        local previousMode, transitions = nil, 0
        local reachedStanding = false
        repeat
            coroutine.yield()
            observeFacing()
            local state = ragdoll.getState(player)
            missing = missing or not state or not state.powered or not state.controller
            if not missing then
                local c = state.controller
                local changed = c.mode ~= previousMode
                if changed then transitions = transitions + 1 end
                if recoverySamples < 200 and (changed or core.getSimulationTime() - started >= nextSample) then
                    local pelvis = findBody(state, 'pelvis')
                    local leftFoot = findBody(state, 'left_foot')
                    local rightFoot = findBody(state, 'right_foot')
                    local upright = c.upright or {}
                    emitTelemetry('repeated_recovery_sample', { cycle = cycle,
                        physics_step = state.physicsStep,
                        seconds = core.getSimulationTime() - started, mode = c.mode,
                        pelvis_up = c.pelvisUpDot, speed = c.centerOfMassVelocity:length(),
                        torso_facing_up = c.torsoFacingUpDot, recovery_direction = c.recoveryDirection,
                        raw_speed = c.rawTargetHorizontalVelocity:length(),
                        pelvis_z = pelvis and pelvis.position.z,
                        com_x = state.centerOfMass.x, com_y = state.centerOfMass.y,
                        com_z = state.centerOfMass.z,
                        left_x = leftFoot and leftFoot.position.x, left_y = leftFoot and leftFoot.position.y,
                        left_z = leftFoot and leftFoot.position.z,
                        right_x = rightFoot and rightFoot.position.x, right_y = rightFoot and rightFoot.position.y,
                        right_z = rightFoot and rightFoot.position.z,
                        target_height = upright.targetPelvisHeight,
                        standing_height = c.targetPelvisHeight,
                        recovery_unweight = upright.recoveryUnweight,
                        ground_contacts = state.support and state.support.groundContactCount,
                        foot_contacts = state.support and state.support.footContactCount,
                        target_pelvis_up = c.targetPelvisUpDot,
                        upright_applied = upright.applied,
                        force_command_x = upright.pelvisForce and upright.pelvisForce.x,
                        force_command_y = upright.pelvisForce and upright.pelvisForce.y,
                        force_command_z = upright.pelvisForce and upright.pelvisForce.z,
                        submitted_force_x = c.appliedPelvisForce and c.appliedPelvisForce.x,
                        submitted_force_y = c.appliedPelvisForce and c.appliedPelvisForce.y,
                        submitted_force_z = c.appliedPelvisForce and c.appliedPelvisForce.z,
                        com_supported = state.support and state.support.centerOfMassSupported,
                        left_foot = state.support and state.support.leftFoot,
                        right_foot = state.support and state.support.rightFoot,
                        support_distance = state.support and state.support.centerOfMassDistance,
                        support_applied = c.jointSupportApplied,
                        support_contact = c.jointSupportContact,
                        support_motor_utilization = c.jointSupportMotorUtilization,
                        motors_applied = c.jointMotorsApplied,
                        velocity_x = c.centerOfMassVelocity.x,
                        velocity_y = c.centerOfMassVelocity.y,
                        transitions = transitions })
                    recoverySamples = recoverySamples + 1
                    nextSample = core.getSimulationTime() - started + 0.1
                end
                previousMode = c.mode
                gettingUp = gettingUp or state.controller.mode == 'getting_up'
                if gettingUp and c.mode == 'standing' then reachedStanding = true end
                unexpectedFall = unexpectedFall or (reachedStanding
                    and (c.mode == 'falling' or c.mode == 'fallen' or c.mode == 'getting_up'))
                if gettingUp and state.controller.mode == 'standing' then
                    stableSince = stableSince or core.getSimulationTime()
                    longestStanding = math.max(longestStanding, core.getSimulationTime() - stableSince)
                    -- Held movement may immediately resume the gait. Its
                    -- subsequent four-second walk must meet the normal gates.
                    recovered = heldMovement or core.getSimulationTime() - stableSince >= 1
                else
                    stableSince = nil
                end
            end
        until missing or unexpectedFall or recovered or core.getSimulationTime() - started >= 12
            or core.getRealTime() >= deadline
        emitTelemetry('repeated_recovery', { cycle = cycle, recovered = recovered,
            getting_up = gettingUp, missing = missing, seconds = core.getSimulationTime() - started,
            longest_standing = longestStanding, transitions = transitions, unexpected_fall = unexpectedFall,
            maximum_facing_change = maximumFacingChange })
        if not recovered then break end
        completed = completed + 1
    end
    local minimumUp, travel, stopDrift, stopSpeed = 1, 0, 0, 0
    local motionTimedOut = false
    if resumeWalking and completed == requestedCycles then
        local origin = findBody(ragdoll.getState(player), 'pelvis').position
        local lastPelvis = origin
        local yaw = player.rotation:getYaw()
        for _, segment in ipairs({ { duration = 4, movement = 1 }, { duration = 5, movement = 0 } }) do
            player:sendEvent('ragdollPlayerTravel', { enabled = true, movement = segment.movement, run = false })
            local started, deadline = core.getSimulationTime(), core.getRealTime() + 12
            local stopOrigin, nextSample, previousPhase = nil, 0, nil
            repeat
                coroutine.yield()
                local elapsed = core.getSimulationTime() - started
                observeFacing()
                local state = ragdoll.getState(player)
                missing = missing or not state or not state.powered or not state.controller
                if not missing then
                    local c = state.controller
                    lastPelvis = findBody(state, 'pelvis').position
                    minimumUp = math.min(minimumUp, c.pelvisUpDot)
                    unexpectedFall = unexpectedFall or c.mode == 'falling' or c.mode == 'fallen'
                        or c.mode == 'getting_up' or c.mode == 'blend_back'
                    if segment.movement == 0 and elapsed >= 4 then
                        stopOrigin = stopOrigin or lastPelvis
                        local delta = lastPelvis - stopOrigin
                        stopDrift = math.max(stopDrift, util.vector3(delta.x, delta.y, 0):length())
                        stopSpeed = math.max(stopSpeed, util.vector3(c.centerOfMassVelocity.x,
                            c.centerOfMassVelocity.y, 0):length())
                    end
                    if elapsed >= nextSample or c.gaitPhase ~= previousPhase then
                        local leftBody = findBody(state, 'left_foot')
                        local rightBody = findBody(state, 'right_foot')
                        local leftFoot, rightFoot = leftBody.position, rightBody.position
                        local leftImpulse, leftUnknown = footVerticalImpulse(state, 'left_foot')
                        local rightImpulse, rightUnknown = footVerticalImpulse(state, 'right_foot')
                        emitTelemetry('recovery_resume_sample', { movement = segment.movement,
                            seconds = elapsed, mode = c.mode, pelvis_up = c.pelvisUpDot,
                            physics_step = state.physicsStep,
                            speed = c.centerOfMassVelocity:length(),
                            velocity_x = c.centerOfMassVelocity.x,
                            velocity_y = c.centerOfMassVelocity.y,
                            raw_speed = c.rawTargetHorizontalVelocity:length(),
                            raw_x = c.rawTargetHorizontalVelocity.x,
                            raw_y = c.rawTargetHorizontalVelocity.y,
                            gameplay_yaw = player.rotation:getYaw(),
                            measurement_yaw = yaw,
                            planned_speed = c.plannedHorizontalVelocity:length(),
                            planned_x = c.plannedHorizontalVelocity.x,
                            planned_y = c.plannedHorizontalVelocity.y,
                            height_settled = c.pelvisHeightSettled,
                            height_candidate_time = c.pelvisHeightCandidateTime,
                            gait_stage = c.gaitStage, actuation_stage = c.gaitActuationStage,
                            gait_phase_time = c.gaitTimeInPhase,
                            swing_lifted = c.gaitSwingFootLifted,
                            swing_time_since_lift = c.gaitSwingTimeSinceLift,
                            swing_contact_time = c.gaitSwingContactCandidateTime,
                            transitioned = c.gaitTransitioned,
                            left_supported = state.support and state.support.leftFoot,
                            right_supported = state.support and state.support.rightFoot,
                            phase = c.gaitPhase, heading_error = c.headingError,
                            left_x = leftFoot.x, left_y = leftFoot.y,
                            right_x = rightFoot.x, right_y = rightFoot.y,
                            left_z = leftFoot.z, right_z = rightFoot.z,
                            left_vertical_velocity = leftBody.linearVelocity.z,
                            right_vertical_velocity = rightBody.linearVelocity.z,
                            left_vertical_impulse = leftImpulse, right_vertical_impulse = rightImpulse,
                            left_impulse_unknown = leftUnknown, right_impulse_unknown = rightUnknown,
                            target_x = c.swingFootTarget and c.swingFootTarget.x,
                            target_y = c.swingFootTarget and c.swingFootTarget.y,
                            support_distance = state.support and state.support.centerOfMassDistance })
                        nextSample = elapsed + 0.1
                        previousPhase = c.gaitPhase
                    end
                end
            until missing or unexpectedFall or core.getSimulationTime() - started >= segment.duration
                or core.getRealTime() >= deadline
            if segment.movement == 1 then
                local delta = lastPelvis - origin
                travel = delta.x * math.sin(yaw) + delta.y * math.cos(yaw)
            end
            if core.getSimulationTime() - started < segment.duration then
                motionTimedOut = not missing and not unexpectedFall
                break
            end
        end
        player:sendEvent('ragdollPlayerTravel', { enabled = false })
        emitTelemetry('recovery_resume', { minimum_up = minimumUp, travel = travel,
            stop_drift = stopDrift, stop_speed = stopSpeed, unexpected_fall = unexpectedFall,
            timed_out = motionTimedOut, maximum_facing_change = maximumFacingChange })
    end
    playerDiagnostics.endObservation()
    if heldMovement then player:sendEvent('ragdollPlayerTravel', { enabled = false }) end
    ragdoll.setAutomaticRecovery(false)
    ragdoll.setPowered(player, false)
    coroutine.yield()
    testing.expect(not missing, 'repeated recovery discarded the physical body')
    testing.expectLessOrEqual(maximumFacingChange, 0.0001, 'recovery changed player facing without look input')
    testing.expect(not unexpectedFall, 'player fell again after reaching standing without another shove')
    testing.expectEqual(completed, requestedCycles, 'player did not recover sustained standing after every fall')
    if resumeWalking then
        testing.expect(not motionTimedOut, 'post-recovery movement observation timed out')
        testing.expectGreaterOrEqual(minimumUp, 0.87, 'player stumbled after recovery')
        testing.expectGreaterOrEqual(travel, 60, 'player failed to walk forward after recovery')
        testing.expectLessOrEqual(stopDrift, 6, 'player drifted after recovery and stopping')
        testing.expectLessOrEqual(stopSpeed, 10, 'player did not settle after recovery and stopping')
    end
end)
end

registerRagdollTest('fallen actor gets up without being asked', function()
    local demo = interfaces.RagdollDev
    demo.setup()
    local actor = waitFor('automatic recovery actor', function() return demo.samples().powered end)
    demo.power(actor, true)
    waitFor('powered standing before the fall', function()
        local state = ragdoll.getState(actor)
        return state and state.controller and state.controller.mode == 'standing'
    end, 8)

    demo.setAutomaticRecovery(true)
    demo.fall(actor)

    -- It must reach the fallen state, then leave it unaided.
    waitFor('the actor to fall', function()
        local state = ragdoll.getState(actor)
        return state and state.controller and state.controller.mode == 'fallen'
    end, 10)
    local recovered, lost = false, false
    local started, deadline = core.getSimulationTime(), core.getRealTime() + 20
    repeat
        coroutine.yield()
        local state = ragdoll.getState(actor)
        lost = lost or not state or not state.controller
        recovered = state and state.powered and state.controller and state.controller.mode == 'standing'
    until recovered or lost or core.getSimulationTime() - started >= 12 or core.getRealTime() >= deadline
    demo.setAutomaticRecovery(false)
    demo.disable(actor)
    coroutine.yield()
    testing.expect(not lost, 'automatic recovery discarded the physical body')
    testing.expect(recovered, 'the fallen actor did not recover supported standing')
end)

registerRagdollTest('a fallen actor braces its hands while it gets up', function()
    local demo = interfaces.RagdollDev
    demo.setup()
    local actor = waitFor('bracing actor', function() return demo.samples().powered end)
    demo.power(actor, true)
    waitFor('powered standing before the fall', function()
        local state = ragdoll.getState(actor)
        return state and state.controller and state.controller.mode == 'standing'
    end, 8)

    -- The brace belongs to getting up, not to lying there, so recovery has
    -- to be the gameplay one for it to arrive at all.
    demo.setAutomaticRecovery(true)
    demo.fall(actor)
    waitFor('the actor to fall', function()
        local state = ragdoll.getState(actor)
        return state and state.controller and state.controller.mode == 'fallen'
    end, 10)

    local left, right
    waitFor('both hands to reach for the ground', function()
        left = ragdoll.getArmReach(actor, false)
        right = ragdoll.getArmReach(actor, true)
        return left ~= nil and right ~= nil
    end, 12)
    testing.expect(left ~= nil and right ~= nil, 'neither hand ever reached for the ground while getting up')
    -- A hand planted under its own shoulder is well inside the arm span, so
    -- a large shortfall means the brace aimed somewhere the arm cannot go.
    testing.expect(left < 12, 'the left hand fell ' .. tostring(left) .. ' units short of its brace point')
    testing.expect(right < 12, 'the right hand fell ' .. tostring(right) .. ' units short of its brace point')
    demo.setAutomaticRecovery(false)
end)

registerRagdollTest('manual gait trace is bounded and reports its outcome', function()
    local demo = interfaces.RagdollDev
    testing.expect(demo ~= nil and demo.version == 5, 'manual ragdoll helper is unavailable')
    demo.setup()
    local gaitActor = waitFor('manual trace gait actor', function()
        local samples = demo.samples()
        return samples.gait
    end)

    local traceId = demo.traceGait(1)
    testing.expectEqual(traceId, 1, 'first manual trace has an unexpected identifier')
    local summary = waitFor('bounded manual gait trace', function()
        local value = demo.traceStatus()
        if not value.active and value.traceId == traceId then
            return value
        end
    end, 4)
    testing.expectEqual(summary.reason, 'duration_complete', 'manual trace did not finish on simulation time')
    testing.expectEqual(summary.sampleKey, 'gait', 'manual trace summarized the wrong sample')
    testing.expectGreaterThan(summary.frames, 0, 'manual trace did not record any powered state')
    testing.expectEqual(summary.classifiedFrames, summary.frames,
        'shared gait stage summaries must account for every recorded physics step')
    testing.expectEqual(summary.uprightClassifiedFrames, summary.frames,
        'upright stage summaries must account for every recorded physics step')
    testing.expectGreaterThan(summary.gaitFrames, 55,
        'manual trace missed too many fixed physics steps after arming')
    testing.expectLessOrEqual(summary.gaitFrames, 65,
        'manual trace duplicated fixed physics steps at high render rates')
    testing.expectGreaterThan(summary.elapsed, 0.9, 'manual trace ended before its requested duration')

    demo.disable(gaitActor)
    waitFor('manual trace powered cleanup', function()
        return ragdoll.getState(gaitActor) == nil
    end)
    demo.cleanup()
end)

-- This verifies the diagnostic recorder, not successful balance. A fall must
-- stay in the recording; the standing acceptance tests below judge the motion.
registerRagdollTest('manual standing trace captures activation and presentation', function()
    local demo = interfaces.RagdollDev
    demo.setup()
    local actor = waitFor('manual standing trace actor', function() return demo.samples().powered end)
    local traceId = demo.traceStanding(10)
    local start = core.getSimulationTime()
    local pushed = false
    local summary = waitFor('bounded manual standing trace', function()
        if not pushed and core.getSimulationTime() - start >= 2 then
            demo.fall(actor)
            pushed = true
        end
        local value = demo.traceStatus()
        return not value.active and value.traceId == traceId and value
    end, 12)
    demo.disable(actor)
    waitFor('manual standing trace cleanup', function() return ragdoll.getState(actor) == nil end)
    demo.cleanup()
    testing.expectEqual(summary.reason, 'duration_complete', 'standing trace stopped before its observation ended')
    testing.expectEqual(summary.sampleKey, 'powered', 'standing trace sampled the wrong actor')
    testing.expectEqual(summary.gaitFrames, 0, 'standing trace started locomotion')
    testing.expectEqual(summary.armingElapsed, 0, 'standing trace waited for stable support')
    testing.expectEqual(summary.armingUnsupportedFrames, 0, 'standing samples were mislabeled as gait arming')
    testing.expectGreaterThan(summary.firstFallingFrame, 0, 'diagnostic trace never captured its controlled fall')
    testing.expectGreaterThan(summary.frames, 0, 'standing trace missed the physics state')
    testing.expectEqual(summary.updateSamples, summary.frames + summary.duplicatePhysicsSteps,
        'presentation samples were discarded or counted twice')
    testing.expectGreaterOrEqual(summary.elapsed, 10, 'standing trace ended early')
    testing.expectLessOrEqual(summary.elapsed, 11, 'standing trace exceeded its bound')
    emitTelemetry('manual_standing_trace_capture', summary)
    testing.expectLessOrEqual(summary.maximumGameplayRootLag, 0.01,
        'powered presentation published an ordinary actor root instead of the ragdoll reference')
end)

local function checkManualPoweredActivation(activationDelay, observationDuration, perturbation)
    observationDuration = observationDuration or 10
    local expectedReplayFrames = math.floor(observationDuration * 60 + 0.5)
    local player = world.players[1]
    local initialBounty = types.Player.getCrimeLevel(player)
    local initialPlayerHealth = types.Actor.stats.dynamic.health(player).current
    local demo = interfaces.RagdollDev
    testing.expect(demo ~= nil and demo.version == 5, 'manual ragdoll helper is unavailable')
    demo.setup()
    local actor = waitFor('manual powered-only actor', function()
        return demo.samples().powered
    end)
    local createdTime = core.getSimulationTime()
    waitFor('manual powered activation delay', function()
        return core.getSimulationTime() - createdTime >= activationDelay
    end, activationDelay + 1)
    demo.power(actor)
    waitFor('manual powered-only state', function()
        local state = ragdoll.getState(actor)
        return state and state.controller
    end)
    ragdoll.startReplayRecording(actor, perturbation and perturbation.seed or 68000 + activationDelay,
        expectedReplayFrames)
    local startTime = core.getSimulationTime()
    local lastStep = -1
    local frames, fallingFrames, requestedFrames = 0, 0, 0
    local minimumUp, maximumAnchor, maximumSpeed = 1, 0, 0
    local jointSupportFrames, externalUprightFrames, airborneAfterLanding = 0, 0, 0
    local landed, maximumJointUtilization = false, 0
    local maximumContactFrictionError = 0
    local firstFallTime = -1
    local finalMode
    local controllerLost = false
    local pushApplied, impulseNewtonSeconds = false, 0
    local pushStep, velocityBeforePush, firstPushResponse
    waitFor('manual powered-only observation', function()
        local state = ragdoll.getState(actor)
        if not state or not state.controller then
            controllerLost = true
            return true
        end
        if perturbation and not pushApplied and core.getSimulationTime() - startTime >= 3 then
            velocityBeforePush = state.controller.centerOfMassVelocity
            pushStep = state.physicsStep
            impulseNewtonSeconds = demo.push(actor, perturbation.direction)
            pushApplied = true
        elseif pushApplied and firstPushResponse == nil and state.physicsStep > pushStep then
            -- Lua queues the impulse for the engine. Observe after a physics
            -- step; contacts and motors also act, so do not equate this with
            -- the instantaneous free-body delta-v of the impulse alone.
            firstPushResponse = (state.controller.centerOfMassVelocity - velocityBeforePush):dot(perturbation.axis)
        end
        if state.physicsStep ~= lastStep then
            lastStep = state.physicsStep
            frames = frames + 1
            local controller = state.controller
            local support = state.support
            finalMode = controller.mode
            if controller.jointSupportApplied and controller.jointSupportContact then
                jointSupportFrames = jointSupportFrames + 1
            end
            if controller.upright and controller.upright.applied then
                externalUprightFrames = externalUprightFrames + 1
            end
            maximumJointUtilization = math.max(maximumJointUtilization, controller.jointSupportMotorUtilization)
            if support and support.groundContactCount > 0 then
                landed = true
                -- The pinned actor material is 0.8; the ordinary floor is 0.2.
                -- Observe the solver combination, not either material alone.
                maximumContactFrictionError = math.max(maximumContactFrictionError,
                    math.abs((support.friction or -1) - math.sqrt(0.8 * 0.2)))
            elseif landed then
                airborneAfterLanding = airborneAfterLanding + 1
            end
            minimumUp = math.min(minimumUp, controller.pelvisUpDot)
            local falling = controller.mode == 'falling' or controller.mode == 'fallen'
            if falling then
                fallingFrames = fallingFrames + 1
                if firstFallTime < 0 then firstFallTime = core.getSimulationTime() - startTime end
            end
            if controller.targetHorizontalVelocity:length() > 0 then
                requestedFrames = requestedFrames + 1
            end
            for _, joint in ipairs(state.constraints) do
                maximumAnchor = math.max(maximumAnchor, joint.anchorSeparation)
            end
            for _, body in ipairs(state.bodies) do
                maximumSpeed = math.max(maximumSpeed, body.linearVelocity:length())
            end
            local normalImpulse, loadedSpeculative, unusedSpeculative, unknownImpulses = 0, 0, 0, 0
            for _, contact in ipairs(state.contacts) do
                if contact.normalImpulse == nil then
                    unknownImpulses = unknownImpulses + 1
                else
                    normalImpulse = normalImpulse + contact.normalImpulse * contact.normal.z
                    if contact.penetrationDepth < 0 then
                        if contact.normalImpulse > 0 then
                            loadedSpeculative = loadedSpeculative + 1
                        else
                            unusedSpeculative = unusedSpeculative + 1
                        end
                    end
                end
            end
            emitTelemetry('manual_powered_only_frame', {
                activation_delay = activationDelay,
                frame = frames, physics_step = state.physicsStep,
                elapsed = core.getSimulationTime() - startTime, mode = controller.mode,
                pelvis_up = controller.pelvisUpDot, pelvis_height = controller.pelvisHeight,
                target_pelvis_height = controller.targetPelvisHeight or 0,
                height_settled = controller.pelvisHeightSettled,
                com_speed = controller.centerOfMassSpeed,
                requested_speed = controller.targetHorizontalVelocity:length(),
                left_supported = support and support.leftFoot or false,
                right_supported = support and support.rightFoot or false,
                ground_contacts = support and support.groundContactCount or 0,
                support_friction = support and support.friction or -1,
                vertical_contact_impulse = normalImpulse,
                loaded_speculative_contacts = loadedSpeculative,
                unused_speculative_contacts = unusedSpeculative,
                unknown_contact_impulses = unknownImpulses,
                maximum_anchor_separation = maximumAnchor, maximum_body_speed = maximumSpeed,
                motors_applied = controller.jointMotorsApplied,
                joint_support_applied = controller.jointSupportApplied,
                joint_support_contact = controller.jointSupportContact,
                joint_support_utilization = controller.jointSupportMotorUtilization,
            })
        end
        return core.getSimulationTime() - startTime >= observationDuration
    end, observationDuration + 1)
    local replay = ragdoll.finishReplayRecording()
    emitTelemetry('manual_powered_only_summary', {
        activation_delay = activationDelay,
        observation_duration = observationDuration,
        controller_lost = controllerLost,
        push_direction = perturbation and perturbation.direction or 'none',
        push_applied = pushApplied, push_impulse_ns = impulseNewtonSeconds,
        first_push_velocity_response = firstPushResponse or 0,
        replay_seed = replay and replay.seed or 'unavailable',
        replay_frames = replay and replay.frameCount or 0,
        frames = frames, falling_frames = fallingFrames, first_fall_time = firstFallTime,
        requested_frames = requestedFrames, minimum_pelvis_up = minimumUp,
        maximum_anchor_separation = maximumAnchor, maximum_body_speed = maximumSpeed,
        final_mode = finalMode,
        joint_support_frames = jointSupportFrames,
        external_upright_frames = externalUprightFrames,
        airborne_after_landing = airborneAfterLanding,
        maximum_joint_utilization = maximumJointUtilization,
    })
    if ragdoll.getState(actor) then demo.disable(actor) end
    waitFor('manual powered-only cleanup', function() return ragdoll.getState(actor) == nil end)
    demo.cleanup()
    testing.expect(not controllerLost, 'powered-only actor lost its controller')
    if perturbation then
        testing.expect(pushApplied, 'standing trial never applied its requested physics push')
        testing.expectGreaterThan(impulseNewtonSeconds, 0, 'physics push delivered no impulse')
        testing.expect(firstPushResponse ~= nil, 'physics push was never observed after a fixed step')
        testing.expectGreaterThan(firstPushResponse, 0, 'physics push produced no response in its commanded direction')
        testing.expectEqual(types.Player.getCrimeLevel(player), initialBounty, 'physics push caused a crime report')
        testing.expectEqual(types.Actor.stats.dynamic.health(player).current, initialPlayerHealth,
            'physics push trial damaged the player')
    end
    testing.expect(replay and replay.frameCount == expectedReplayFrames, 'powered activation capture missed fixed steps')
    testing.expectGreaterThan(frames, observationDuration * 55,
        'powered-only observation missed too many physics steps')
    testing.expectEqual(requestedFrames, 0, 'powered-only fixture requested locomotion')
    -- Report every physical failure, retaining the existing acceptance bounds.
    -- A first airborne frame must not hide a later fall or stretched joint.
    local failed = {}
    for _, gate in ipairs({
        { maximumContactFrictionError <= 1e-6, 'contact_friction_error', maximumContactFrictionError, '<= 1e-6' },
        { jointSupportFrames > 0, 'joint_support', jointSupportFrames, '> 0' },
        { externalUprightFrames == 0, 'external_upright_frames', externalUprightFrames, '= 0' },
        { airborneAfterLanding == 0, 'airborne_after_landing', airborneAfterLanding, '= 0' },
        { maximumJointUtilization <= 1, 'motor_utilization', maximumJointUtilization, '<= 1' },
        { fallingFrames == 0, 'falling_frames', fallingFrames, '= 0' },
        { minimumUp >= 0.9, 'minimum_pelvis_up', minimumUp, '>= 0.9' },
        { maximumSpeed <= 2000, 'maximum_body_speed', maximumSpeed, '<= 2000' },
        { maximumAnchor <= 2.5, 'maximum_anchor_separation', maximumAnchor, '<= 2.5' },
    }) do
        emitTelemetry('manual_powered_only_gate', {
            activation_delay = activationDelay, push_direction = perturbation and perturbation.direction or 'none',
            gate = gate[2], passed = gate[1], actual = gate[3], expected = gate[4],
        })
        if not gate[1] then
            table.insert(failed, string.format('%s: %.6g (expected %s)', gate[2], gate[3], gate[4]))
        end
    end
    testing.expect(#failed == 0, 'Standing failed: ' .. table.concat(failed, '; '))
end

for _, activationDelay in ipairs({ 0, 1, 3 }) do
    registerRagdollTest('manual powered balance after ' .. activationDelay .. ' seconds of idle', function()
        checkManualPoweredActivation(activationDelay)
    end)
end

-- The manual recording includes later idle poses and a fall beyond the original
-- ten-second observation window. Exercise both without changing physical gates.
for _, activationDelay in ipairs({ 8, 30 }) do
    registerRagdollTest('manual powered balance extended after ' .. activationDelay .. ' seconds of idle', function()
        checkManualPoweredActivation(activationDelay, 30)
    end)
end

for index, perturbation in ipairs({
    { direction = 'x', axis = util.vector3(1, 0, 0) },
    { direction = '-x', axis = util.vector3(-1, 0, 0) },
    { direction = 'y', axis = util.vector3(0, 1, 0) },
    { direction = '-y', axis = util.vector3(0, -1, 0) },
}) do
    perturbation.seed = 68100 + index
    registerRagdollTest('manual powered balance under a ' .. perturbation.direction .. ' physics push', function()
        checkManualPoweredActivation(0, 10, perturbation)
    end)
end

registerRagdollTest('manual larger Y push preserves powered control through a fall', function()
    local demo = interfaces.RagdollDev
    demo.setup()
    local actor = waitFor('larger-push sample', function() return demo.samples().powered end)
    local created = core.getSimulationTime()
    waitFor('larger-push idle pose', function() return core.getSimulationTime() - created >= 18 end, 19)
    demo.power(actor)
    waitFor('larger-push powered activation', function()
        local state = ragdoll.getState(actor)
        return state and state.controller
    end)
    local start = core.getSimulationTime()
    local pushed, lost, motorRelease = false, false, false
    local maximumAnchor, maximumSpeed, maximumUtilization = 0, 0, 0
    waitFor('larger-push controller lifetime', function()
        local state = ragdoll.getState(actor)
        if not state or not state.controller then
            lost = true
            return true
        end
        if not pushed and core.getSimulationTime() - start >= 3 then
            for _ = 1, 3 do demo.push(actor, 'y') end
            pushed = true
        end
        for _, joint in ipairs(state.constraints) do
            maximumAnchor = math.max(maximumAnchor, joint.anchorSeparation)
        end
        for _, body in ipairs(state.bodies) do
            maximumSpeed = math.max(maximumSpeed, body.linearVelocity:length())
        end
        maximumUtilization = math.max(maximumUtilization, state.controller.jointSupportMotorUtilization)
        if state.controller.mode == 'falling' or state.controller.mode == 'fallen' then
            motorRelease = motorRelease or not state.controller.jointMotorsApplied
        end
        return core.getSimulationTime() - start >= 10
    end, 11)
    emitTelemetry('manual_larger_push_lifetime_summary', {
        controller_lost = lost, push_applied = pushed, motor_release_observed = motorRelease,
        maximum_anchor_separation = maximumAnchor, maximum_body_speed = maximumSpeed,
        maximum_joint_utilization = maximumUtilization,
    })
    if ragdoll.getState(actor) then demo.disable(actor) end
    waitFor('larger-push cleanup', function() return ragdoll.getState(actor) == nil end)
    demo.cleanup()
    -- A fall is a valid physical outcome here. This is a controller-lifetime
    -- regression gate, not standing or push-recovery acceptance.
    testing.expect(pushed, 'larger-push lifetime trial never applied its impulse')
    testing.expect(not lost, 'larger push discarded powered physics for ordinary actor collision')
    testing.expectLessOrEqual(maximumAnchor, 2.5, 'larger push stretched its joints')
    testing.expectLessOrEqual(maximumSpeed, 2000, 'larger push reached an unsafe speed')
    testing.expectLessOrEqual(maximumUtilization, 1, 'larger push exceeded the joint motor budget')
end)

registerRagdollTest('powered living ragdoll transitions to passive death', function()
    local player = world.players[1]
    local victim = world.createObject(types.NPC.record(player).id)
    victim:teleport(player.cell, player.position + util.vector3(0, 180, 0))
    coroutine.yield()

    local before = ragdoll.getCounters()
    ragdoll.setPowered(victim, true)
    waitFor('powered ragdoll before death', function()
        local state = ragdoll.getState(victim)
        return state and state.powered and state
    end)

    ragdoll.triggerDeath(victim, {
        hitPoint = victim.position + util.vector3(0, 0, 100),
        direction = util.vector3(1, 0, 0.2),
        damage = 25,
    })
    local passive = waitFor('powered-to-passive ragdoll transition', function()
        local state = ragdoll.getState(victim)
        if state and not state.powered and types.Actor.isDead(victim) then
            return state
        end
    end)
    testing.expectEqual(passive.pendingImpact, false,
        'powered-to-passive transition did not consume the killing impulse')
    testing.expectEqual(#passive.bodies, 20,
        'powered-to-passive transition produced an incomplete ragdoll')

    ragdoll.resurrect(victim)
    waitFor('powered-to-passive ragdoll cleanup', function()
        return ragdoll.getState(victim) == nil and not types.Actor.isDead(victim)
    end)
    local after = ragdoll.getCounters()
    testing.expectEqual(after.current, before.current, 'powered-to-passive transition leaked a ragdoll')
    testing.expectEqual(after.created, before.created + 2,
        'powered-to-passive transition did not create both lifecycle modes')
    testing.expectEqual(after.removed, before.removed + 2,
        'powered-to-passive transition did not remove both lifecycle modes')
    testing.expectEqual(after.poseFailures, before.poseFailures,
        'powered-to-passive transition caused a pose synchronization failure')
    victim:remove()
end)

-- CORPSE-01 (10-31, user: "on death the npc bodies should be released into a
-- ragdoll, right now they do some movement and drags the ragdoll around"):
-- four powered NPCs killed one after another, each left 4.5 s dead so the
-- engine logs RAGDOLL_DEATH_DRAWN (how far the drawn corpse was dragged off
-- its physics place, how far the actor moved; CORPSE_FOLLOW). Prints how far
-- the actor ends from the corpse's root body.
registerRagdollTest('CORPSE-01 death clip does not drag the corpse', function()
    local player = world.players[1]
    for i = 1, 4 do
        local angle = i * math.pi / 2
        local direction = util.vector3(math.cos(angle), math.sin(angle), 0)
        local victim = world.createObject(types.NPC.record(player).id)
        victim:teleport(player.cell, testActorOrigin, { onGround = true })
        coroutine.yield()
        ragdoll.setPowered(victim, true)
        waitFor('CORPSE-01 powered before death', function()
            local state = ragdoll.getState(victim)
            return state and state.powered and state.controller and state.controller.mode == 'standing'
        end, 10)
        ragdoll.triggerDeath(victim, {
            hitPoint = victim.position + util.vector3(0, 0, 100),
            direction = direction + util.vector3(0, 0, 0.2),
            damage = 25,
        })
        waitFor('CORPSE-01 corpse', function()
            local state = ragdoll.getState(victim)
            return state and not state.powered and types.Actor.isDead(victim)
        end)
        local until_ = core.getSimulationTime() + 4.5
        waitFor('CORPSE-01 4.5 s dead', function() return core.getSimulationTime() >= until_ end, 10)
        local state = ragdoll.getState(victim)
        testing.expect(state ~= nil, 'CORPSE-01 corpse lost its ragdoll')
        local offset = victim.position - state.bodies[1].position
        print(string.format('CORPSE-01 victim=%d actor_to_root=%.1f', i,
            util.vector3(offset.x, offset.y, 0):length()))
        victim:remove()
        coroutine.yield()
    end
end)

registerRagdollTest('powered living ragdoll disables motors after a fall', function()
    local demo = interfaces.RagdollDev
    testing.expect(demo ~= nil, 'manual ragdoll development interface is not loaded')
    testing.expectEqual(demo.version, 6, 'manual ragdoll development interface version changed')
    local player = world.players[1]
    local victim = world.createObject(types.NPC.record(player).id)
    victim:teleport(player.cell, testActorOrigin, { onGround = true })
    coroutine.yield()

    local before = ragdoll.getCounters()
    demo.power(victim)
    local standing = waitFor('powered ragdoll before controlled fall', function()
        local state = ragdoll.getState(victim)
        if state and state.powered and state.controller and state.controller.mode == 'standing' then
            return state
        end
    end)
    local fallImpulse = demo.fall(victim)
    local recoveryRequestStatus = demo.recover(victim)
    testing.expectEqual(recoveryRequestStatus, 'queued',
        'manual recovery requested before settling was not queued')

    local observedFalling = false
    local observedFallen = false
    local observedMotorsDisabled = false
    local maximumPhysicsRootSeparation = 0
    local maximumVisibleRootLag = 0
    local fallStartPosition = victim.position
    local maximumHorizontalRootTravel = 0
    local maximumLinearSpeed = 0
    for frame = 1, 360 do
        local state = ragdoll.getState(victim)
        testing.expect(state ~= nil, 'powered ragdoll fell back during controlled fall')
        testing.expect(state.powered, 'controlled fall changed out of powered mode')
        testing.expect(state.physicsActorPosition ~= nil,
            'powered ragdoll did not expose its same-phase physics actor root')
        maximumPhysicsRootSeparation = math.max(maximumPhysicsRootSeparation,
            (state.referencePosition - state.physicsActorPosition):length())
        maximumVisibleRootLag = math.max(maximumVisibleRootLag,
            (state.referencePosition - victim.position):length())
        local rootDelta = victim.position - fallStartPosition
        local rootTravel = util.vector3(rootDelta.x, rootDelta.y, 0)
        maximumHorizontalRootTravel = math.max(maximumHorizontalRootTravel, rootTravel:length())
        inspectTelemetry(state, 'controlled powered fall')
        if state.controller then
            inspectControllerTelemetry(state.controller, 'controlled powered fall', true)
            local relinquished = state.controller.mode == 'falling' or state.controller.mode == 'fallen'
            observedFalling = observedFalling or state.controller.mode == 'falling'
            observedFallen = observedFallen or state.controller.mode == 'fallen'
            observedMotorsDisabled = observedMotorsDisabled
                or (relinquished and not state.controller.driveJointMotors
                    and not state.controller.jointMotorsApplied)
            local separation = (state.referencePosition - state.physicsActorPosition):length()
            if frame % 15 == 0 or separation > 1 then
                emitTelemetry('powered_fall_frame', {
                    frame = frame,
                    mode = state.controller.mode,
                    pelvis_up = state.controller.pelvisUpDot,
                    pelvis_height = state.controller.pelvisHeight,
                    com_speed = state.controller.centerOfMassSpeed,
                    ground_contacts = state.support and state.support.groundContactCount or 0,
                    foot_contacts = state.support and state.support.footContactCount or 0,
                    physics_root_separation = separation,
                    visible_root_lag = (state.referencePosition - victim.position):length(),
                    horizontal_root_travel = rootTravel:length(),
                    target_speed = state.controller.targetHorizontalVelocity:length(),
                    drive_joint_motors = state.controller.driveJointMotors,
                    joint_motors_applied = state.controller.jointMotorsApplied,
                })
            end
        end
        for _, body in ipairs(state.bodies) do
            maximumLinearSpeed = math.max(maximumLinearSpeed, body.linearVelocity:length())
        end
        if observedFallen and observedMotorsDisabled then
            break
        end
        coroutine.yield()
    end

    emitTelemetry('powered_fall_handoff', {
        observed_falling = observedFalling,
        observed_fallen = observedFallen,
        motors_disabled = observedMotorsDisabled,
        impulse = fallImpulse,
        recovery_request_status = recoveryRequestStatus,
        max_linear_speed = maximumLinearSpeed,
        max_physics_root_separation = maximumPhysicsRootSeparation,
        max_visible_root_lag = maximumVisibleRootLag,
        max_horizontal_root_travel = maximumHorizontalRootTravel,
    })
    if not observedFalling and not observedFallen then
        -- Falls are physical since build 72 (PGC): this shove at full balance
        -- help is caught standing. Accept an upright catch; the motor release
        -- and get-up path then go untested here (needs a push that fells).
        local final = ragdoll.getState(victim)
        testing.expect(final and final.powered and final.controller and final.controller.mode == 'standing',
            'shoved actor neither fell nor ended standing')
        demo.disable(victim)
        waitFor('controlled powered fall cleanup', function() return ragdoll.getState(victim) == nil end)
        local after = ragdoll.getCounters()
        victim:remove()
        coroutine.yield()
        testing.expectEqual(after.current, before.current, 'controlled powered fall leaked a ragdoll')
        testing.expectEqual(after.poseFailures, before.poseFailures, 'controlled powered fall caused fallback')
        return
    end
    testing.expect(observedFalling, 'powered controller never classified the controlled fall')
    testing.expect(observedFallen, 'powered controller never classified the actor as fallen')
    testing.expect(observedMotorsDisabled, 'powered controller did not relinquish joint motors while falling')
    testing.expectLessOrEqual(maximumLinearSpeed, 3000,
        'controlled powered fall reached an unsafe linear speed')

    local observedGettingUp = false
    local observedRecoveryAnimation = false
    local observedRecoveryAnimationFinished = false
    local observedRecoveryMotors = false
    local observedRecoveryUpright = false
    local observedRecoveryTorqueSaturation = false
    local observedBlendBack = false
    local recovered = false
    local recoveryDirection
    local recoveryModeTrace = {}
    local lastRecoveryMode
    local finalPelvisHeight = 0
    local finalPelvisUpDot = 0
    local finalTargetPelvisUpDot = 0
    local finalCenterOfMassSpeed = 0
    local finalFootContactCount = 0
    local maximumRecoveryPelvisUpDot = -1
    local maximumRecoveryFootContactCount = 0
    local finalPelvisAngularSpeed = 0
    local maximumRecoveryTargetError = 0
    for _ = 1, 600 do
        local state = ragdoll.getState(victim)
        if state == nil then
            recovered = false -- disappearing is not physical recovery
            break
        end
        testing.expect(state.powered, 'recovery changed out of powered mode')
        testing.expect(state.physicsActorPosition ~= nil,
            'powered recovery did not expose its same-phase physics actor root')
        maximumPhysicsRootSeparation = math.max(maximumPhysicsRootSeparation,
            (state.referencePosition - state.physicsActorPosition):length())
        maximumVisibleRootLag = math.max(maximumVisibleRootLag,
            (state.referencePosition - victim.position):length())
        local rootDelta = victim.position - fallStartPosition
        local rootTravel = util.vector3(rootDelta.x, rootDelta.y, 0)
        maximumHorizontalRootTravel = math.max(maximumHorizontalRootTravel, rootTravel:length())
        inspectTelemetry(state, 'powered recovery')
        if state.controller then
            inspectControllerTelemetry(state.controller, 'powered recovery', true)
            if state.controller.mode ~= lastRecoveryMode then
                table.insert(recoveryModeTrace, state.controller.mode)
                lastRecoveryMode = state.controller.mode
            end
            if state.controller.recoveryDirection then
                recoveryDirection = state.controller.recoveryDirection
            end
            finalPelvisHeight = state.controller.pelvisHeight
            finalPelvisUpDot = state.controller.pelvisUpDot
            finalTargetPelvisUpDot = state.controller.targetPelvisUpDot
            finalCenterOfMassSpeed = state.controller.centerOfMassSpeed
            maximumRecoveryPelvisUpDot
                = math.max(maximumRecoveryPelvisUpDot, state.controller.pelvisUpDot)
            observedGettingUp = observedGettingUp or state.controller.mode == 'getting_up'
            observedRecoveryAnimation = observedRecoveryAnimation
                or state.controller.recoveryAnimationStarted
            observedRecoveryAnimationFinished = observedRecoveryAnimationFinished
                or state.controller.recoveryAnimationFinished
            observedRecoveryMotors = observedRecoveryMotors
                or (state.controller.mode == 'getting_up' and state.controller.jointMotorsApplied)
            if state.controller.upright then
                observedRecoveryUpright = observedRecoveryUpright
                    or (state.controller.mode == 'getting_up' and state.controller.upright.applied)
                observedRecoveryTorqueSaturation = observedRecoveryTorqueSaturation
                    or state.controller.upright.pelvisTorqueSaturated
                    or state.controller.upright.spineTorqueSaturated
            end
            observedBlendBack = observedBlendBack or state.controller.mode == 'blend_back'
            recovered = observedBlendBack and state.controller.mode == 'standing'
        end
        if state.support then
            finalFootContactCount = state.support.footContactCount
            maximumRecoveryFootContactCount
                = math.max(maximumRecoveryFootContactCount, state.support.footContactCount)
        end
        for _, body in ipairs(state.bodies) do
            maximumLinearSpeed = math.max(maximumLinearSpeed, body.linearVelocity:length())
            maximumRecoveryTargetError
                = math.max(maximumRecoveryTargetError, (body.position - body.targetPosition):length())
            if body.name == 'pelvis' then
                finalPelvisAngularSpeed = body.angularVelocity:length()
            end
        end
        if recovered then
            break
        end
        coroutine.yield()
    end

    emitTelemetry('powered_recovery', {
        modes = table.concat(recoveryModeTrace, ','),
        direction = recoveryDirection or 'none',
        animation_started = observedRecoveryAnimation,
        animation_finished = observedRecoveryAnimationFinished,
        motors_applied = observedRecoveryMotors,
        upright_applied = observedRecoveryUpright,
        torque_saturated = observedRecoveryTorqueSaturation,
        blend_back = observedBlendBack,
        recovered = recovered,
        impulse = fallImpulse,
        final_pelvis_height = finalPelvisHeight,
        final_pelvis_up = finalPelvisUpDot,
        max_pelvis_up = maximumRecoveryPelvisUpDot,
        final_target_pelvis_up = finalTargetPelvisUpDot,
        final_pelvis_angular_speed = finalPelvisAngularSpeed,
        final_com_speed = finalCenterOfMassSpeed,
        final_foot_contacts = finalFootContactCount,
        max_foot_contacts = maximumRecoveryFootContactCount,
        max_target_error = maximumRecoveryTargetError,
        max_linear_speed = maximumLinearSpeed,
        max_physics_root_separation = maximumPhysicsRootSeparation,
        max_visible_root_lag = maximumVisibleRootLag,
    })

    -- Teardown must run before the known physical gate failures, so the next
    -- corpse test cannot collide with a surviving actor from this trial.
    demo.disable(victim)
    waitFor('controlled powered fall cleanup', function()
        return ragdoll.getState(victim) == nil
    end)
    local after = ragdoll.getCounters()
    victim:remove()
    coroutine.yield()

    testing.expect(recoveryDirection == 'face_up' or recoveryDirection == 'face_down',
        'powered recovery did not select a valid orientation')
    testing.expect(observedGettingUp, 'powered controller never entered getting-up state')
    testing.expect(observedRecoveryAnimation, 'powered recovery animation never started')
    testing.expect(observedRecoveryAnimationFinished, 'powered recovery animation never finished')
    testing.expect(observedRecoveryMotors, 'powered recovery did not reactivate joint motors')
    testing.expect(observedRecoveryUpright, 'powered recovery did not apply upright control')
    testing.expect(observedBlendBack, 'powered controller never entered blend-back state')
    testing.expect(recovered, 'powered controller did not complete recovery handoff')
    testing.expectLessOrEqual(maximumLinearSpeed, 3000,
        'powered recovery reached an unsafe linear speed')
    testing.expectLessOrEqual(maximumPhysicsRootSeparation, 1,
        'powered ragdoll reference separated from its same-phase physics actor root')
    testing.expectGreaterThan(maximumHorizontalRootTravel, 1,
        'physical fall did not transfer horizontal pelvis travel to the gameplay root')

    testing.expectEqual(after.current, before.current, 'controlled powered fall leaked a ragdoll')
    testing.expectEqual(after.poseFailures, before.poseFailures, 'controlled powered fall caused fallback')
    emitTelemetry('powered_fall_complete', {
        max_linear_speed = maximumLinearSpeed,
        max_physics_root_separation = maximumPhysicsRootSeparation,
        max_visible_root_lag = maximumVisibleRootLag,
        max_horizontal_root_travel = maximumHorizontalRootTravel,
        motors_disabled = observedMotorsDisabled,
        impulse = fallImpulse,
        recovery_direction = recoveryDirection or 'none',
        recovery_animation_started = observedRecoveryAnimation,
        recovery_animation_finished = observedRecoveryAnimationFinished,
        recovered = recovered,
    })
end)

registerRagdollTest('passive ragdoll remains bounded and settles naturally', function()
    local player = world.players[1]
    local victim = world.createObject(types.NPC.record(player).id)
    -- Use the same verified flat floor as the rest of the suite;
    -- the previous cell-(0,0) terrain probe was isolated but visually noisy
    -- and could place the observed corpse among steep geometry.
    victim:teleport(player.cell, testActorOrigin, { onGround = true })
    coroutine.yield()

    local before = ragdoll.getCounters()
    ragdoll.triggerDeath(victim)
    local initial = waitFor('settling ragdoll creation', function()
        return ragdoll.getState(victim)
    end)
    ragdoll.startReplayRecording(victim, 69100)
    local initialPelvis = findBody(initial, 'pelvis').position
    local initialRightThigh = findBody(initial, 'right_thigh')
    local initialRightCalf = findBody(initial, 'right_calf')
    local initialKneeRelativeSpeed
        = (initialRightThigh.linearVelocity - initialRightCalf.linearVelocity):length()
    local initialSpan = 0
    for _, body in ipairs(initial.bodies) do
        initialSpan = math.max(initialSpan, (body.position - initialPelvis):length())
    end
    testing.expectGreaterThan(initialSpan, 1, 'initial ragdoll span is degenerate')

    local settled = false
    local escapedBody
    local finalActiveBodies = 0
    local finalMaxLinearSpeed = 0
    local finalMaxAngularSpeed = 0
    local finalPelvisDisplacement = 0
    local finalPelvisZ = initialPelvis.z
    local maximumAnchorSeparation = 0
    local maximumAnchorConstraint = 'none'
    local maximumAnchorFrame = 0
    local maximumHingeViolation = 0
    local maximumFootContactCount = 0
    local maximumDryRunPelvisForce = 0
    local maximumDryRunPelvisTorque = 0
    local maximumDryRunSpineTorque = 0
    local observedDryRunSaturation = false
    local observedFallenControllerState = false
    -- Ten simulated seconds at 60 Hz, independent of rendering frequency.
    -- Sample unique states; lifetime native peaks cover intervening substeps.
    local frame = 0
    local lastPhysicsStep = initial.physicsStep
    while lastPhysicsStep - initial.physicsStep < 600 do
        local state = ragdoll.getState(victim)
        testing.expect(state ~= nil, 'settling ragdoll disappeared')
        if state.physicsStep == lastPhysicsStep then
            coroutine.yield()
        else
            lastPhysicsStep = state.physicsStep
            frame = frame + 1
            local pelvis = findBody(state, 'pelvis').position
            local anchorSeparation, hingeViolation, anchorConstraint = inspectTelemetry(state, 'settling ragdoll')
            if anchorSeparation > maximumAnchorSeparation then
                maximumAnchorSeparation = anchorSeparation
                maximumAnchorConstraint = anchorConstraint
                maximumAnchorFrame = frame
            end
            maximumHingeViolation = math.max(maximumHingeViolation, hingeViolation)
            if state.support then
                inspectSupportTelemetry(state.support, 'settling ragdoll')
                maximumFootContactCount = math.max(maximumFootContactCount, state.support.footContactCount)
            end
            if state.controller then
                inspectControllerTelemetry(state.controller, 'settling ragdoll')
                observedFallenControllerState = observedFallenControllerState or state.controller.mode == 'fallen'
                if state.controller.upright then
                    local upright = state.controller.upright
                    maximumDryRunPelvisForce = math.max(maximumDryRunPelvisForce, upright.pelvisForce:length())
                    maximumDryRunPelvisTorque = math.max(maximumDryRunPelvisTorque, upright.pelvisTorque:length())
                    maximumDryRunSpineTorque = math.max(maximumDryRunSpineTorque, upright.spineTorque:length())
                    observedDryRunSaturation = observedDryRunSaturation or upright.pelvisForceSaturated
                        or upright.pelvisTorqueSaturated or upright.spineTorqueSaturated
                end
            end
            finalActiveBodies = 0
            finalMaxLinearSpeed = 0
            finalMaxAngularSpeed = 0
            finalPelvisDisplacement = (pelvis - initialPelvis):length()
            finalPelvisZ = pelvis.z
            local allSleeping = true
            for index, body in ipairs(state.bodies) do
                expectFiniteVector(body.position, string.format('settling body %d position', index))
                expectFiniteVector(body.linearVelocity, string.format('settling body %d linear velocity', index))
                expectFiniteVector(body.angularVelocity, string.format('settling body %d angular velocity', index))
                if (body.position - pelvis):length() > initialSpan * 3 + 1 then
                    escapedBody = index
                end
                finalMaxLinearSpeed = math.max(finalMaxLinearSpeed, body.linearVelocity:length())
                finalMaxAngularSpeed = math.max(finalMaxAngularSpeed, body.angularVelocity:length())
                if body.active then
                    finalActiveBodies = finalActiveBodies + 1
                end
                allSleeping = allSleeping and not body.active
            end
            if escapedBody or allSleeping then
                settled = true
                break
            end
            if lastPhysicsStep - initial.physicsStep < 600 then
                coroutine.yield()
            end
        end
    end
    local finalState = ragdoll.getState(victim)
    local quality = finalState.stepQuality
    local replay = ragdoll.finishReplayRecording()
    local poseFailures = ragdoll.getCounters().poseFailures

    ragdoll.resurrect(victim)
    waitFor('settled ragdoll cleanup', function()
        return ragdoll.getState(victim) == nil
    end)
    victim:remove()
    coroutine.yield()
    -- Report physical evidence before any acceptance assertion can abort the test.
    print(string.format(
        'RAGDOLL_TELEMETRY max_anchor_separation=%.6f max_anchor_constraint=%s max_anchor_frame=%d max_hinge_violation=%.6f max_foot_contacts=%d dry_run_pelvis_force=%.3f dry_run_pelvis_torque=%.3f dry_run_spine_torque=%.3f dry_run_saturated=%s',
        maximumAnchorSeparation, maximumAnchorConstraint, maximumAnchorFrame, maximumHingeViolation,
        maximumFootContactCount, maximumDryRunPelvisForce, maximumDryRunPelvisTorque, maximumDryRunSpineTorque,
        tostring(observedDryRunSaturation)))
    emitTelemetry('passive_settling', {
        initial_pelvis_speed = findBody(initial, 'pelvis').linearVelocity:length(),
        initial_right_thigh_speed = initialRightThigh.linearVelocity:length(),
        initial_right_calf_speed = initialRightCalf.linearVelocity:length(),
        initial_right_knee_relative_speed = initialKneeRelativeSpeed,
        initial_right_thigh_angular_speed = initialRightThigh.angularVelocity:length(),
        initial_right_calf_angular_speed = initialRightCalf.angularVelocity:length(),
        maximum_anchor_separation = maximumAnchorSeparation,
        maximum_anchor_frame = maximumAnchorFrame,
        maximum_hinge_violation = maximumHingeViolation,
        fixed_step_count = quality.steps,
        fixed_step_maximum_anchor_separation = quality.maximumAnchorSeparation,
        fixed_step_maximum_anchor_step = quality.maximumAnchorStep,
        fixed_step_maximum_anchor_constraint = quality.maximumAnchorParentBody .. '->' .. quality.maximumAnchorChildBody,
        fixed_step_maximum_hinge_violation = quality.maximumHingeViolation,
        replay_frames = replay and replay.frameCount or 0,
    })
    testing.expect(replay ~= nil, 'passive settling fixed-step replay was not saved')
    testing.expectGreaterThan(replay.frameCount, 0, 'passive settling replay has no physics frames')
    testing.expectEqual(quality.steps - initial.stepQuality.steps,
        finalState.physicsStep - initial.physicsStep, 'passive peak observer skipped or duplicated a physics step')
    testing.expectEqual(replay.frameCount, quality.steps - initial.stepQuality.steps,
        'passive replay did not capture every observed physics step')
    testing.expectGreaterOrEqual(quality.maximumAnchorSeparation + 1e-6, maximumAnchorSeparation,
        'fixed-step peak missed an anchor separation visible to Lua')
    testing.expectGreaterOrEqual(quality.maximumHingeViolation + 1e-6, maximumHingeViolation,
        'fixed-step peak missed a hinge violation visible to Lua')
    testing.expectEqual(escapedBody, nil,
        string.format('settling body %s escaped the constrained rig', tostring(escapedBody)))
    testing.expect(settled, string.format(
        'passive ragdoll did not settle within 600 physics steps: active=%d, maxLinear=%.3f, maxAngular=%.3f, pelvisTravel=%.3f, pelvisZ=%.3f->%.3f',
        finalActiveBodies, finalMaxLinearSpeed, finalMaxAngularSpeed, finalPelvisDisplacement, initialPelvis.z,
        finalPelvisZ))
    testing.expectEqual(poseFailures, before.poseFailures, 'pose synchronization failed while the ragdoll settled')
    testing.expectGreaterThan(maximumFootContactCount, 0,
        'Jolt reported no upward foot contacts while the ragdoll settled')
    testing.expect(observedFallenControllerState,
        'observation-only controller never classified the settled ragdoll as fallen')
    -- 2.5 world units is approximately 3.6 cm: enough for ordinary solver
    -- compliance under impact, but below a visible disconnected-joint gap.
    testing.expectLessOrEqual(quality.maximumAnchorSeparation, 2.5,
        string.format('settling ragdoll constraint anchors separated excessively at %s on physics step %d',
            quality.maximumAnchorParentBody .. '->' .. quality.maximumAnchorChildBody, quality.maximumAnchorStep))
    -- Transient solver compliance under impact varied from 0.052 to 0.110 rad
    -- across repeated real-time runs. Keep a margin without allowing a visibly
    -- misconfigured hinge frame or unconstrained joint to pass.
    testing.expectLessOrEqual(quality.maximumHingeViolation, 0.15, 'settling ragdoll exceeded a hinge limit')
end)

registerRagdollTest('repeated death resurrection cycles leave no ragdoll state', function()
    local player = world.players[1]
    local victim = world.createObject(types.NPC.record(player).id)
    victim:teleport(player.cell, player.position + util.vector3(0, 280, 0))
    coroutine.yield()

    local before = ragdoll.getCounters()
    local cycleCount = 32
    for cycle = 1, cycleCount do
        ragdoll.triggerDeath(victim)
        local state = waitFor(string.format('stress ragdoll creation %d', cycle), function()
            return ragdoll.getState(victim)
        end)
        testing.expectEqual(#state.bodies, 20, string.format('stress cycle %d body count', cycle))
        if cycle % 4 == 0 then
            ragdoll.applyImpulse(victim, 1, util.vector3(25, 0, 0))
        end

        ragdoll.resurrect(victim)
        waitFor(string.format('stress ragdoll cleanup %d', cycle), function()
            return ragdoll.getState(victim) == nil and not types.Actor.isDead(victim)
        end)
    end

    local after = ragdoll.getCounters()
    testing.expectEqual(after.current, before.current, 'stress cycles leaked a live ragdoll')
    testing.expectEqual(after.pendingImpacts, before.pendingImpacts, 'stress cycles leaked a killing impact')
    testing.expectEqual(after.created, before.created + cycleCount, 'stress cycles lost a creation')
    testing.expectEqual(after.removed, before.removed + cycleCount, 'stress cycles lost a removal')
    testing.expectEqual(after.poseFailures, before.poseFailures, 'stress cycles caused a pose failure')
end)

registerRagdollTest('ragdoll cap evicts the oldest sleeping corpse', function()
    local player = world.players[1]
    local before = ragdoll.getCounters()
    local powered = world.createObject(types.NPC.record(player).id)
    powered:teleport(player.cell, player.position + util.vector3(0, 320, 0))
    coroutine.yield()
    ragdoll.setPowered(powered, true)
    waitFor('powered ragdoll excluded from passive cap', function()
        local state = ragdoll.getState(powered)
        return state and state.powered and state
    end)

    local victims = {}
    for index = 1, 4 do
        local victim = world.createObject(types.NPC.record(player).id)
        victim:teleport(player.cell, player.position + util.vector3(index * 100 - 250, 240, 0))
        coroutine.yield()
        ragdoll.triggerDeath(victim)
        waitFor(string.format('ragdoll %d creation', index), function()
            return ragdoll.getState(victim)
        end)
        victims[index] = victim
    end

    ragdoll.setActive(victims[2], false)
    ragdoll.setActive(victims[3], false)
    waitFor('test ragdolls to deactivate', function()
        local second = ragdoll.getState(victims[2])
        local third = ragdoll.getState(victims[3])
        return second and third and not second.bodies[1].active and not third.bodies[1].active
    end)

    local newest = world.createObject(types.NPC.record(player).id)
    newest:teleport(player.cell, player.position + util.vector3(250, 240, 0))
    coroutine.yield()
    ragdoll.triggerDeath(newest)
    waitFor('cap eviction', function()
        local counters = ragdoll.getCounters()
        return counters.current == before.current + 5 and counters.evicted == before.evicted + 1
    end)

    local poweredState = ragdoll.getState(powered)
    testing.expect(poweredState and poweredState.powered,
        'passive cap evicted the powered living ragdoll')
    testing.expect(ragdoll.getState(victims[1]) ~= nil, 'older active ragdoll was evicted before a sleeper')
    testing.expectEqual(ragdoll.getState(victims[2]), nil, 'oldest sleeping ragdoll was not evicted')
    testing.expect(ragdoll.getState(victims[3]) ~= nil, 'newer sleeping ragdoll was evicted first')
    testing.expect(ragdoll.getState(victims[4]) ~= nil, 'newer active ragdoll was unexpectedly evicted')
    testing.expect(ragdoll.getState(newest) ~= nil, 'newly created ragdoll was unexpectedly evicted')

    for _, victim in ipairs(victims) do
        ragdoll.resurrect(victim)
    end
    ragdoll.resurrect(newest)
    ragdoll.setPowered(powered, false)
    waitFor('all capped ragdolls to be removed', function()
        return ragdoll.getCounters().current == before.current
    end)
    local after = ragdoll.getCounters()
    testing.expectEqual(after.created, before.created + 6,
        'created counter did not include the powered actor and every corpse')
    testing.expectEqual(after.removed, before.removed + 6,
        'removed counter did not include powered cleanup, eviction, and corpse cleanup')
    testing.expectEqual(after.poseFailures, before.poseFailures, 'pose synchronization failed during cap test')
    powered:remove()
end)

registerRagdollTest('dead actor cell transfer removes and recreates ragdoll', function()
    local player = world.players[1]
    local victim = world.createObject(types.NPC.record(player).id)
    local returnPosition = player.position + util.vector3(0, 220, 0)
    victim:teleport(player.cell, returnPosition)
    coroutine.yield()

    local before = ragdoll.getCounters()
    ragdoll.triggerDeath(victim)
    waitFor('ragdoll before cell transfer', function()
        return ragdoll.getState(victim)
    end)

    victim:teleport('Balmora, Guild of Mages', util.vector3(0, 0, 0))
    waitFor('ragdoll removal in inactive cell', function()
        return ragdoll.getState(victim) == nil
    end)

    victim:teleport(player.cell, returnPosition)
    local reloaded = waitFor('ragdoll recreation after cell return', function()
        return ragdoll.getState(victim)
    end)
    testing.expectEqual(#reloaded.bodies, 20, 'reloaded corpse has an unexpected body count')
    testing.expectEqual(reloaded.pendingImpact, false, 'reloaded corpse retained a stale killing impact')

    ragdoll.resurrect(victim)
    waitFor('reloaded ragdoll cleanup', function()
        return ragdoll.getState(victim) == nil
    end)
    local after = ragdoll.getCounters()
    testing.expectEqual(after.created, before.created + 2, 'cell return did not recreate the ragdoll')
    testing.expectEqual(after.removed, before.removed + 2, 'cell transfer did not clean up both ragdolls')
    testing.expectEqual(after.current, before.current, 'cell transfer leaked a ragdoll')
    testing.expectEqual(after.poseFailures, before.poseFailures, 'pose synchronization failed during cell transfer')
end)

registerRagdollTest('unsupported creature falls back without pending physics state', function()
    local player = world.players[1]
    local creatureRecord
    for _, record in ipairs(types.Creature.records) do
        if not record.isBiped and record.model ~= '' then
            creatureRecord = record
            break
        end
    end
    testing.expect(creatureRecord ~= nil, 'test content has no non-biped creature record')
    local creature = world.createObject(creatureRecord.id)
    creature:teleport(player.cell, player.position + util.vector3(0, 240, 0))
    coroutine.yield()

    local before = ragdoll.getCounters()
    ragdoll.triggerDeath(creature)
    waitFor('unsupported creature death fallback', function()
        return types.Actor.isDead(creature) and ragdoll.getCounters().pendingImpacts == before.pendingImpacts
    end)
    testing.expectEqual(ragdoll.getState(creature), nil, 'unsupported creature unexpectedly owns a ragdoll')
    local after = ragdoll.getCounters()
    testing.expectEqual(after.current, before.current, 'unsupported creature changed the ragdoll count')
    testing.expectEqual(after.created, before.created, 'unsupported creature incremented the created counter')

    ragdoll.resurrect(creature)
end)

registerRagdollTest('player death and resurrection use passive ragdoll lifecycle', function()
    local player = world.players[1]
    local before = ragdoll.getCounters()
    ragdoll.triggerDeath(player, {
        hitPoint = player.position + util.vector3(0, 0, 100),
        direction = util.vector3(1, 0, 0.15),
        damage = 20,
    })
    local state = waitFor('player ragdoll creation', function()
        return ragdoll.getState(player)
    end)
    testing.expectEqual(#state.bodies, 20, 'player ragdoll has an unexpected body count')
    testing.expectEqual(state.pendingImpact, false, 'player retained a stale killing impact')
    coroutine.yield()
    state = ragdoll.getState(player)
    local head = findBody(state, 'head')
    testing.expectLessOrEqual(0, state.renderInterpolationFactor,
        'render interpolation factor is below its fixed-step range')
    testing.expectLessOrEqual(state.renderInterpolationFactor, 1,
        'render interpolation factor is above its fixed-step range')
    expectFiniteVector(head.renderPosition, 'player rendered head position')
    local trackedPosition = ragdoll.getCameraTrackedPosition()
    testing.expectLessOrEqual((trackedPosition - head.renderPosition):length(), 0.01,
        'third-person death camera did not follow the rendered head')

    ragdoll.resurrect(player)
    waitFor('player ragdoll cleanup', function()
        return ragdoll.getState(player) == nil and not types.Actor.isDead(player)
    end)
    local after = ragdoll.getCounters()
    testing.expectEqual(after.current, before.current, 'player ragdoll leaked after resurrection')
    testing.expectEqual(after.created, before.created + 1, 'player ragdoll was not counted')
    testing.expectEqual(after.removed, before.removed + 1, 'player ragdoll removal was not counted')
end)

registerRagdollTest('uniform actor scale preserves proportional ragdoll layout', function()
    local player = world.players[1]
    local normal = world.createObject(types.NPC.record(player).id)
    local scaled = world.createObject(types.NPC.record(player).id)
    normal:teleport(player.cell, player.position + util.vector3(-180, 240, 0))
    scaled:teleport(player.cell, player.position + util.vector3(180, 240, 0))
    scaled:setScale(1.5)
    coroutine.yield()

    ragdoll.triggerDeath(normal)
    ragdoll.triggerDeath(scaled)
    local normalState = waitFor('normal-scale ragdoll', function()
        return ragdoll.getState(normal)
    end)
    local scaledState = waitFor('scaled ragdoll', function()
        return ragdoll.getState(scaled)
    end)
    local normalSpan = (findBody(normalState, 'head').position - findBody(normalState, 'pelvis').position):length()
    local scaledSpan = (findBody(scaledState, 'head').position - findBody(scaledState, 'pelvis').position):length()
    testing.expectGreaterThan(normalSpan, 1, 'normal ragdoll pelvis-to-head span is degenerate')
    testing.expectAlmostEqual(scaledSpan / normalSpan, 1.5, 'ragdoll layout did not follow actor scale')

    ragdoll.resurrect(normal)
    waitFor('normal-scale ragdoll cleanup', function()
        return ragdoll.getState(normal) == nil
    end)
    ragdoll.resurrect(scaled)
    waitFor('1.5-scale ragdoll cleanup', function()
        return ragdoll.getState(scaled) == nil
    end)
end)

registerRagdollTest('female humanoid assembles a complete ragdoll binding', function()
    local player = world.players[1]
    local femaleRecord
    for _, record in ipairs(types.NPC.records) do
        local race = types.NPC.races.records[record.race]
        if not record.isMale and race and not race.isBeast then
            femaleRecord = record
            break
        end
    end
    testing.expect(femaleRecord ~= nil, 'test content has no non-beast female NPC record')

    local victim = world.createObject(femaleRecord.id)
    victim:teleport(player.cell, player.position + util.vector3(0, 220, 0))
    coroutine.yield()
    local before = ragdoll.getCounters()
    ragdoll.triggerDeath(victim)
    local state = waitFor('female humanoid ragdoll', function()
        return ragdoll.getState(victim)
    end)
    testing.expectEqual(#state.bodies, 20, 'female humanoid has an unexpected body count')
    for index, body in ipairs(state.bodies) do
        expectFiniteVector(body.position, string.format('female body %d position', index))
    end

    ragdoll.resurrect(victim)
    waitFor('female humanoid cleanup', function()
        return ragdoll.getState(victim) == nil
    end)
    local after = ragdoll.getCounters()
    testing.expectEqual(after.created, before.created + 1, 'female ragdoll was not counted')
    testing.expectEqual(after.removed, before.removed + 1, 'female ragdoll cleanup was not counted')
end)

registerRagdollTest('beast humanoid assembles a complete ragdoll binding', function()
    local player = world.players[1]
    local beastRecord
    for _, record in ipairs(types.NPC.records) do
        local race = types.NPC.races.records[record.race]
        if race and race.isBeast then
            beastRecord = record
            break
        end
    end
    testing.expect(beastRecord ~= nil, 'test content has no beast-race NPC record')

    local victim = world.createObject(beastRecord.id)
    victim:teleport(player.cell, player.position + util.vector3(0, 260, 0))
    coroutine.yield()
    local before = ragdoll.getCounters()
    ragdoll.triggerDeath(victim)
    local state = waitFor('beast humanoid ragdoll', function()
        return ragdoll.getState(victim)
    end)
    testing.expectEqual(#state.bodies, 20, 'beast humanoid has an unexpected body count')
    for index, body in ipairs(state.bodies) do
        expectFiniteVector(body.position, string.format('beast body %d position', index))
    end

    ragdoll.resurrect(victim)
    waitFor('beast humanoid cleanup', function()
        return ragdoll.getState(victim) == nil
    end)
    local after = ragdoll.getCounters()
    testing.expectEqual(after.created, before.created + 1, 'beast ragdoll was not counted')
    testing.expectEqual(after.removed, before.removed + 1, 'beast ragdoll cleanup was not counted')
end)



-- NATIVE-02. A population gate measures only this short locomotion smoke.
-- It does not replace the captured-body hold-out or recovery qualification.
for index, scenario in ipairs({
    { name = 'strafes right', movement = 0, sideMovement = 0.4 },
    { name = 'strafes left', movement = 0, sideMovement = -0.4 },
    { name = 'walks forward', movement = 0.4, sideMovement = 0 },
}) do
    registerRagdollTest('manual walker population ' .. scenario.name, function()
        local demo = interfaces.RagdollDev
        demo.setup()
        waitFor('manual C population source', function() return demo.samples().gait end)
        local config = ragdoll.populationTestConfig()
        demo.spawnWalkers(config.n, config.spacing)
        local segments = { { duration = 6, movement = scenario.movement, sideMovement = scenario.sideMovement } }
        demo.runPopulation(segments, 6, 910000 + index * 100)
        local report = waitFor('population paths', function() return demo.populationStatus().report end, 20)
        print(string.format('NATIVE-02 population %s: falls %d / %d; p95 %.6f ms', scenario.name, report.falls, config.n, report.frameP95 * 1000))
        testing.expect(report.invalidReason == nil, report.invalidReason or '')
        testing.expect(report.setupFailures == 0 and report.completed == config.n, 'population did not complete every recording')
        -- Registered bound; this value is a gate, never a tuning target.
        testing.expect(report.fallRate <= 0.15, 'population fall rate exceeds 0.15')
    end)
end

-- Activation opens the corpse container UI and intentionally leaves the world
-- paused, so keep this as the final scenario in the suite.
registerRagdollTest('dead ragdoll actor remains activatable', function()
    local player = world.players[1]
    local victim = world.createObject(types.NPC.record(player).id)
    victim:teleport(player.cell, player.position + util.vector3(0, 180, 0))
    coroutine.yield()
    ragdoll.triggerDeath(victim)
    waitFor('activatable corpse ragdoll', function()
        return ragdoll.getState(victim)
    end)

    victim:activateBy(player)
    coroutine.yield()
    testing.expect(ragdoll.getState(victim) ~= nil, 'activation removed the corpse ragdoll')
end)

local eventHandlers = {}
eventHandlers.ragdollDiagnosticInputSample = playerDiagnostics.input
for name, handler in pairs(testing.globalEventHandlers) do
    eventHandlers[name] = handler
end
eventHandlers.ragdollGaitCommandReceived = function(value)
    gaitCommandAcknowledged = value
end

return {
    engineHandlers = {
        onUpdate = function(dt)
            testing.updateGlobal(dt)
            playerDiagnostics.update()
        end,
    },
    eventHandlers = eventHandlers,
}

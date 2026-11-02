local core = require('openmw.core')
local ragdoll = require('openmw.ragdolltest')
local testing = require('testing_util')
local testArea = require('ragdoll_test_area')
local types = require('openmw.types')
local util = require('openmw.util')
local world = require('openmw.world')

local playerReady

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

local function waitFor(description, predicate, timeout)
    local simulationDeadline = core.getSimulationTime() + (timeout or 10)
    local realDeadline = core.getRealTime() + math.max((timeout or 10) * 4, 15)
    repeat
        local value = predicate()
        if value then return value end
        coroutine.yield()
    until core.getSimulationTime() >= simulationDeadline or core.getRealTime() >= realDeadline
    error('WPN-01 timed out waiting for ' .. description)
end

local function addItem(actor, recordId)
    local inventory = types.Actor.inventory(actor)
    if not inventory:find(recordId) then
        world.createObject(recordId, 1):moveInto(inventory)
        coroutine.yield()
    end
    testing.expect(inventory:find(recordId) ~= nil, 'WPN-01 could not add ' .. recordId)
end

local function waitSeconds(seconds, observe)
    local deadline = core.getSimulationTime() + seconds
    repeat
        if observe then observe() end
        coroutine.yield()
    until core.getSimulationTime() >= deadline
end

local function moveWeaponYaw(player, from, to, duration, hold, observe)
    local started = core.getSimulationTime()
    repeat
        local t = math.min((core.getSimulationTime() - started) / duration, 1)
        local smooth = t * t * (3 - 2 * t)
        ragdoll.setWeaponFixtureYaw(player, from + (to - from) * smooth)
        if observe then observe() end
        coroutine.yield()
    until core.getSimulationTime() - started >= duration
    ragdoll.setWeaponFixtureYaw(player, to)
    waitSeconds(hold, observe)
end

local function findBody(state, name)
    for _, body in ipairs(state.bodies) do
        if body.name == name then return body end
    end
    error('WPN-01 missing body ' .. name)
end

local function weaponYaw(state)
    local direction = findBody(state, 'equipped_weapon').rotation * util.vector3(0, 1, 0)
    return math.atan2(-direction.x, direction.y)
end

testing.registerGlobalTest('WPN-01 live equipped weapon hold yaw reversal stop impact miss', function()
    local multiplier = ragdoll.testStepMultiplier()
    if multiplier > 0 then ragdoll.setTestStepActive(true, 905001) end
    local player = world.players[1]
    player:teleport(testArea.cell, testArea.player,
        { onGround = true, rotation = util.transform.rotateZ(0) })
    waitFor('player relocation', function()
        local delta = player.position - testArea.player
        return player.cell.name == testArea.cell
            and util.vector3(delta.x, delta.y, 0):length() <= 2
    end)
    addItem(player, 'iron warhammer')
    player:sendEvent('weaponPlayerConfigure')
    waitFor('equipped weapon stance', function() return playerReady end)
    testing.expect(ragdoll.prepareEquippedWeapon(player), 'WPN-01 could not select equipped weapon')
    ragdoll.setPowered(player, true)
    local state = waitFor('physical weapon attachment', function()
        local current = ragdoll.getState(player)
        if current and current.weapon and current.controller
            and current.controller.mode == 'standing' then return current end
    end)
    local startupGripSeparation = 0
    local startupGripAngularError = 0
    local start = core.getSimulationTime()
    repeat
        state = ragdoll.getState(player)
        startupGripSeparation = math.max(startupGripSeparation, state.weapon.gripSeparation)
        startupGripAngularError = math.max(startupGripAngularError, state.weapon.gripAngularError)
        coroutine.yield()
    until core.getSimulationTime() - start >= 2

    start = core.getSimulationTime()
    local maximumGripSeparation = 0
    local maximumGripAngularError = 0
    repeat
        state = ragdoll.getState(player)
        testing.expect(state and state.weapon, 'WPN-01 lost its physical weapon')
        maximumGripSeparation = math.max(maximumGripSeparation, state.weapon.gripSeparation)
        maximumGripAngularError = math.max(maximumGripAngularError, state.weapon.gripAngularError)
        coroutine.yield()
    until core.getSimulationTime() - start >= 2
    local gripConstraint
    for _, constraint in ipairs(state.constraints) do
        if constraint.childBody == 'equipped_weapon' then gripConstraint = constraint break end
    end
    local restWeapon = findBody(state, 'equipped_weapon')
    local motionMaximumGripSeparation = 0
    local motionMaximumGripAngularError = 0
    local motionMaximumGroundContacts = 0
    local motionNonStandingFrames = 0
    local function observeMotion()
        local current = ragdoll.getState(player)
        if not current or not current.weapon or not current.controller then return end
        motionMaximumGripSeparation = math.max(
            motionMaximumGripSeparation, current.weapon.gripSeparation)
        motionMaximumGripAngularError = math.max(
            motionMaximumGripAngularError, current.weapon.gripAngularError)
        motionMaximumGroundContacts = math.max(
            motionMaximumGroundContacts, current.weapon.groundContacts)
        if current.controller.mode ~= 'standing' then
            motionNonStandingFrames = motionNonStandingFrames + 1
        end
    end
    -- This is the same smooth one-second +/-0.35 rad shoulder-subtree yaw as
    -- the qualified offline fixture, now applied to a captured native pose.
    moveWeaponYaw(player, 0, 0.35, 1, 0.5, observeMotion)
    local left = ragdoll.getState(player)
    moveWeaponYaw(player, 0.35, -0.35, 1, 0.5, observeMotion)
    local right = ragdoll.getState(player)
    moveWeaponYaw(player, -0.35, 0, 1, 0, observeMotion)
    local stopped = waitFor('physical weapon stop', function()
        local current = ragdoll.getState(player)
        if not current or not current.weapon then return false end
        observeMotion()
        local speed = findBody(current, 'equipped_weapon').angularVelocity:length()
        if speed <= 0.2
            and math.abs(util.normalizeAngle(weaponYaw(current) - weaponYaw(state))) <= 0.15 then
            return current
        end
    end, 6)
    print(string.format('WPN-01 diagnostic startup_gap=%.9f startup_angle=%.9f grip_max=%.9f grip_final=%.9f angle_max=%.9f angle_final=%.9f anchor_final=%.9f yaw_rest=%.9f yaw_left=%.9f yaw_right=%.9f yaw_stop=%.9f speed_stop=%.9f travel_left=%.9f travel_right=%.9f',
        startupGripSeparation, startupGripAngularError, maximumGripSeparation,
        state.weapon.gripSeparation, maximumGripAngularError, state.weapon.gripAngularError,
        gripConstraint and gripConstraint.anchorSeparation or -1, weaponYaw(state), weaponYaw(left),
        weaponYaw(right), weaponYaw(stopped), findBody(stopped, 'equipped_weapon').angularVelocity:length(),
        (findBody(left, 'equipped_weapon').position - restWeapon.position):length(),
        (findBody(right, 'equipped_weapon').position - findBody(left, 'equipped_weapon').position):length()))
    testing.expectEqual(state.weapon.recordId, 'iron warhammer')
    testing.expectEqualWithDelta(state.weapon.physicalMass, 1, 1e-4,
        'WPN-01 physical proxy mass changed')
    testing.expectEqualWithDelta(state.totalMass, 76, 1e-3,
        'WPN-01 articulated mass changed')
    testing.expectEqual(state.weapon.velocitySolverSteps, 40)
    testing.expectLessOrEqual(maximumGripSeparation, 1)
    testing.expectLessOrEqual(maximumGripAngularError, 0.03)
    testing.expectEqual(state.weapon.groundContacts, 0)
    testing.expectEqual(state.controller.mode, 'standing')
    testing.expectGreaterOrEqual(math.abs(util.normalizeAngle(weaponYaw(left) - weaponYaw(right))), 0.4,
        'WPN-01 physical weapon did not reverse its yaw')
    testing.expectGreaterOrEqual((findBody(left, 'equipped_weapon').position - restWeapon.position):length(), 5,
        'WPN-01 physical weapon did not move during the first yaw')
    testing.expectGreaterOrEqual((findBody(right, 'equipped_weapon').position
        - findBody(left, 'equipped_weapon').position):length(), 10,
        'WPN-01 physical weapon did not move during reversal')
    testing.expectLessOrEqual(findBody(stopped, 'equipped_weapon').angularVelocity:length(), 0.2,
        'WPN-01 physical weapon did not stop')
    testing.expectLessOrEqual(math.abs(util.normalizeAngle(weaponYaw(stopped) - weaponYaw(state))), 0.15,
        'WPN-01 physical weapon did not return to its rest yaw')
    testing.expectEqual(motionNonStandingFrames, 0,
        'WPN-01 actor left standing mode during weapon motion')
    testing.expectLessOrEqual(motionMaximumGripSeparation, 1,
        'WPN-01 grip separated during continuous weapon motion')
    testing.expectLessOrEqual(motionMaximumGripAngularError, 0.03,
        'WPN-01 grip rotated during continuous weapon motion')
    testing.expectEqual(motionMaximumGroundContacts, 0,
        'WPN-01 weapon touched the floor during motion')
    for _, phase in ipairs({ left, right, stopped }) do
        testing.expect(phase.controller.mode == 'standing', 'WPN-01 actor fell during weapon motion')
        testing.expectLessOrEqual(phase.weapon.gripSeparation, 1,
            'WPN-01 grip separated during weapon motion')
        testing.expectLessOrEqual(phase.weapon.gripAngularError, 0.03,
            'WPN-01 grip rotated during weapon motion')
        testing.expectEqual(phase.weapon.groundContacts, 0)
    end

    -- The qualified motion above is the registered miss control. Measure a
    -- wider target-only arc before spawning the target, then begin the impact
    -- at the opposite endpoint. This keeps an actor-sized target clear of the
    -- resting bar and makes every later contact attributable to the stroke.
    testing.expectEqual(stopped.weapon.otherRagdollContacts, 0,
        'WPN-01 miss control touched another actor')
    moveWeaponYaw(player, 0, 0.7, 1, 0.5)
    local impactEndpoint = ragdoll.getState(player)
    local endpointWeapon = findBody(impactEndpoint, 'equipped_weapon')
    local impactPoint = endpointWeapon.position
    moveWeaponYaw(player, 0.7, -0.7, 1.1, 0.5)
    local targetRecord = world.createRecord(types.NPC.createRecordDraft({
        template = types.NPC.record(player),
        name = 'WPN-01 physical impact target',
        isEssential = false,
        isPersistent = false,
        isRespawning = false,
    }))
    local radial = util.vector3(impactPoint.x - player.position.x, impactPoint.y - player.position.y, 0)
    radial = radial / radial:length()
    local targetRotation = util.transform.rotateZ(math.atan2(radial.x, -radial.y) + math.pi)
    local desiredImpactBodyPosition = impactPoint + radial * 1.5
    local safeOrigin = util.vector3(impactPoint.x, impactPoint.y, testArea.player.z) + radial * 150
    local calibrationTarget = world.createObject(targetRecord.id)
    calibrationTarget:teleport(testArea.cell, safeOrigin,
        { onGround = true, rotation = targetRotation })
    coroutine.yield()
    ragdoll.setPowered(calibrationTarget, true)
    local calibrationState = waitFor('impact target offset calibration', function()
        local current = ragdoll.getState(calibrationTarget)
        if current and current.controller and current.controller.mode == 'standing' then return current end
    end)
    waitSeconds(0.25)
    calibrationState = ragdoll.getState(calibrationTarget)
    local impactBodyName
    local impactBodyOffset
    local inwardProjection = math.huge
    for _, body in ipairs(calibrationState.bodies) do
        if math.abs(body.position.z - impactPoint.z) <= 15 then
            local offset = body.position - safeOrigin
            local projection = offset.x * radial.x + offset.y * radial.y
            if projection < inwardProjection then
                impactBodyName = body.name
                impactBodyOffset = offset
                inwardProjection = projection
            end
        end
    end
    testing.expect(impactBodyName ~= nil,
        'WPN-01 target has no physical body at weapon height')
    local correctedOrigin = util.vector3(
        desiredImpactBodyPosition.x - impactBodyOffset.x,
        desiredImpactBodyPosition.y - impactBodyOffset.y,
        testArea.player.z)
    ragdoll.setPowered(calibrationTarget, false)
    waitFor('impact target calibration cleanup', function()
        return not ragdoll.getState(calibrationTarget)
    end)
    calibrationTarget:remove()
    coroutine.yield()

    local target = world.createObject(targetRecord.id)
    target:teleport(testArea.cell, correctedOrigin,
        { onGround = true, rotation = targetRotation })
    coroutine.yield()
    ragdoll.setPowered(target, true)
    local alignedTarget = waitFor('aligned powered impact target', function()
        local current = ragdoll.getState(target)
        if current and current.controller and current.controller.mode == 'standing' then return current end
    end)
    waitSeconds(0.5)
    alignedTarget = ragdoll.getState(target)
    local impactBody = findBody(alignedTarget, impactBodyName)
    local alignedBody = impactBody
    local horizontalAlignment = util.vector3(
        alignedBody.position.x - desiredImpactBodyPosition.x,
        alignedBody.position.y - desiredImpactBodyPosition.y, 0):length()
    testing.expectLessOrEqual(horizontalAlignment, 12,
        'WPN-01 impact target did not align with the measured weapon endpoint')
    local contactBefore = ragdoll.getState(player).weapon.otherRagdollContacts
    testing.expectEqual(contactBefore, 0, 'WPN-01 target overlapped the resting weapon')
    local maximumTargetSpeed = 0
    local impactMaximumGripSeparation = 0
    local impactMaximumGripAngularError = 0
    local impactNonStandingFrames = 0
    local minimumTargetBodyDistance = math.huge
    local minimumSelectedBodyDistance = math.huge
    local function observeImpact()
        local wielder = ragdoll.getState(player)
        if wielder and wielder.weapon and wielder.controller then
            impactMaximumGripSeparation = math.max(
                impactMaximumGripSeparation, wielder.weapon.gripSeparation)
            impactMaximumGripAngularError = math.max(
                impactMaximumGripAngularError, wielder.weapon.gripAngularError)
            if wielder.controller.mode ~= 'standing' then
                impactNonStandingFrames = impactNonStandingFrames + 1
            end
        end
        local targetState = target:isValid() and ragdoll.getState(target) or nil
        if targetState and wielder then
            local weaponBody = findBody(wielder, 'equipped_weapon')
            for _, body in ipairs(targetState.bodies) do
                maximumTargetSpeed = math.max(maximumTargetSpeed, body.linearVelocity:length())
                local distance = (body.position - weaponBody.position):length()
                minimumTargetBodyDistance = math.min(minimumTargetBodyDistance, distance)
                if body.name == impactBody.name then
                    minimumSelectedBodyDistance = math.min(minimumSelectedBodyDistance, distance)
                end
            end
        end
        return wielder
    end
    local impactStarted = core.getSimulationTime()
    local previousYaw = -0.7
    local contacted = false
    local targetRemoved = false
    repeat
        local t = math.min((core.getSimulationTime() - impactStarted) / 4, 1)
        local smooth = t * t * (3 - 2 * t)
        local yaw = -0.7 + 1.4 * smooth
        ragdoll.setWeaponFixtureYaw(player, yaw)
        coroutine.yield()
        local wielder = observeImpact()
        if wielder and wielder.weapon.otherRagdollContacts > contactBefore then
            contacted = true
            ragdoll.setPowered(target, false)
            waitFor('impact target cleanup', function() return not ragdoll.getState(target) end)
            targetRemoved = true
            break
        end
        previousYaw = yaw
    until core.getSimulationTime() - impactStarted >= 4
    ragdoll.setWeaponFixtureYaw(player, previousYaw)
    waitSeconds(0.5, observeImpact)
    local impact = ragdoll.getState(player)
    local contactEvent = impact and impact.weapon and impact.weapon.lastContactEvent
    print(string.format('WPN-01 target alignment=%.9f min_body_distance=%.9f min_selected_distance=%.9f contacts=%d target_speed=%.9f impact_grip=%.9f impact_angle=%.9f mode=%s',
        horizontalAlignment, minimumTargetBodyDistance, minimumSelectedBodyDistance,
        impact.weapon.otherRagdollContacts - contactBefore, maximumTargetSpeed,
        impactMaximumGripSeparation, impactMaximumGripAngularError, impact.controller.mode))
    testing.expect(contacted and impact.weapon.otherRagdollContacts > contactBefore,
        'WPN-01 physical weapon did not contact the target')
    testing.expect(contactEvent ~= nil,
        'HIT-01 physical contact did not cross the game-thread event boundary')
    testing.expect(contactEvent and contactEvent.targetActorIdentity ~= nil,
        'HIT-01 contact did not retain the target actor identity')
    testing.expect(contactEvent and contactEvent.attackerActorIdentity
        ~= contactEvent.targetActorIdentity,
        'HIT-01 contact confused attacker and target ownership')
    testing.expect(contactEvent and contactEvent.weaponRecordId == 'iron warhammer',
        'HIT-01 contact did not retain weapon identity')
    testing.expect(contactEvent and contactEvent.weaponBodyIdentity
        ~= contactEvent.targetBodyIdentity,
        'HIT-01 contact confused weapon and target bodies')
    testing.expect(contactEvent and tonumber(contactEvent.contactIdentity) > 0,
        'HIT-01 contact identity is not monotonic')
    testing.expect(contactEvent and tonumber(contactEvent.physicsStep) >= 0,
        'HIT-01 contact did not retain its physics step')
    testing.expect(contactEvent and contactEvent.swingIdentity == '0'
        and contactEvent.swingActive == false,
        'HIT-01 fixture contact did not identify its resting swing lifetime')
    testing.expect(contactEvent and contactEvent.point:length() > 0,
        'HIT-01 contact point is missing')
    testing.expect(contactEvent and math.abs(contactEvent.normal:length() - 1) <= 0.01,
        'HIT-01 contact normal is not normalized')
    testing.expect(contactEvent and contactEvent.relativeNormalSpeed
        == contactEvent.relativeNormalSpeed,
        'HIT-01 relative normal speed is not finite')
    testing.expect(contactEvent and contactEvent.effectiveMass > 0
        and contactEvent.effectiveMass == contactEvent.effectiveMass,
        'HIT-03 contact-axis effective mass is missing')
    testing.expectEqual(impact.weapon.strikeEventCount, 0,
        'HIT-02 resting fixture contact became a qualified strike')
    testing.expectGreaterThan(maximumTargetSpeed, 1,
        'WPN-01 target did not receive a physical impact')
    testing.expectEqual(impact.controller.mode, 'standing',
        'WPN-01 wielder fell during physical impact')
    testing.expectLessOrEqual(impact.weapon.gripSeparation, 1,
        'WPN-01 grip separated during physical impact')
    testing.expectLessOrEqual(impact.weapon.gripAngularError, 0.03,
        'WPN-01 grip rotated during physical impact')
    testing.expectLessOrEqual(impactMaximumGripSeparation, 1,
        'WPN-01 grip separated during the impact stroke')
    testing.expectLessOrEqual(impactMaximumGripAngularError, 0.03,
        'WPN-01 grip rotated during the impact stroke')
    testing.expectEqual(impactNonStandingFrames, 0,
        'WPN-01 wielder left standing mode during the impact stroke')
    print(string.format('WPN-01 impact contacts=%d target_speed=%.9f grip=%.9f angle=%.9f',
        impact.weapon.otherRagdollContacts, maximumTargetSpeed,
        impact.weapon.gripSeparation, impact.weapon.gripAngularError))
    emitTelemetry('weapon_hold', {
        equipped_record = state.weapon.recordId,
        item_weight = state.weapon.itemWeight,
        physical_mass = state.weapon.physicalMass,
        articulated_mass = state.totalMass,
        velocity_solver_steps = state.weapon.velocitySolverSteps,
        startup_grip_separation = startupGripSeparation,
        startup_grip_angular_error = startupGripAngularError,
        maximum_grip_separation = maximumGripSeparation,
        maximum_grip_angular_error = maximumGripAngularError,
        ground_contacts = state.weapon.groundContacts,
        final_mode = state.controller.mode,
    })
    emitTelemetry('weapon_motion', {
        rest_yaw = weaponYaw(state),
        left_yaw = weaponYaw(left),
        right_yaw = weaponYaw(right),
        stopped_yaw = weaponYaw(stopped),
        reversal_yaw = math.abs(util.normalizeAngle(weaponYaw(left) - weaponYaw(right))),
        left_travel = (findBody(left, 'equipped_weapon').position - restWeapon.position):length(),
        reversal_travel = (findBody(right, 'equipped_weapon').position
            - findBody(left, 'equipped_weapon').position):length(),
        stopped_angular_speed = findBody(stopped, 'equipped_weapon').angularVelocity:length(),
        maximum_grip_separation = motionMaximumGripSeparation,
        maximum_grip_angular_error = motionMaximumGripAngularError,
        ground_contacts = motionMaximumGroundContacts,
        non_standing_frames = motionNonStandingFrames,
        miss_other_ragdoll_contacts = stopped.weapon.otherRagdollContacts,
    })
    emitTelemetry('weapon_impact', {
        weapon_contacts = impact.weapon.otherRagdollContacts - contactBefore,
        target_maximum_body_speed = maximumTargetSpeed,
        grip_separation = impact.weapon.gripSeparation,
        grip_angular_error = impact.weapon.gripAngularError,
        maximum_grip_separation = impactMaximumGripSeparation,
        maximum_grip_angular_error = impactMaximumGripAngularError,
        non_standing_frames = impactNonStandingFrames,
        wielder_mode = impact.controller.mode,
        contact_event_count = impact.weapon.contactEventCount,
        strike_event_count = impact.weapon.strikeEventCount,
        contact_identity = contactEvent and contactEvent.contactIdentity or '',
        attacker_actor_identity = contactEvent and contactEvent.attackerActorIdentity or '',
        target_actor_identity = contactEvent and contactEvent.targetActorIdentity or '',
        weapon_record_id = contactEvent and contactEvent.weaponRecordId or '',
        weapon_body_identity = contactEvent and contactEvent.weaponBodyIdentity or '',
        target_body_identity = contactEvent and contactEvent.targetBodyIdentity or '',
        contact_physics_step = contactEvent and contactEvent.physicsStep or '',
        swing_identity = contactEvent and contactEvent.swingIdentity or '',
        swing_active = contactEvent and contactEvent.swingActive or false,
        contact_point_length = contactEvent and contactEvent.point:length() or -1,
        contact_normal_length = contactEvent and contactEvent.normal:length() or -1,
        relative_normal_speed = contactEvent and contactEvent.relativeNormalSpeed or 0,
        effective_mass = contactEvent and contactEvent.effectiveMass or 0,
    })
    if not targetRemoved and target:isValid() then
        ragdoll.setPowered(target, false)
        target:remove()
    end

    -- Reuse the calibrated target placement and this same native process for
    -- HIT-02/HIT-03. A real swing lifetime arms the contact filter while the
    -- already-qualified fixture supplies one repeatable physical stroke.
    moveWeaponYaw(player, previousYaw, -0.7, 1, 0.25)
    previousYaw = -0.7
    local damageTarget = target
    damageTarget:teleport(testArea.cell, correctedOrigin,
        { onGround = true, rotation = targetRotation })
    coroutine.yield()
    ragdoll.setPowered(damageTarget, true)
    waitFor('HIT-03 powered damage target', function()
        local current = ragdoll.getState(damageTarget)
        if current and current.controller and current.controller.mode == 'standing' then return current end
    end)
    waitSeconds(0.5)
    local damageAlignmentState = ragdoll.getState(damageTarget)
    local damageImpactBody = findBody(damageAlignmentState, impactBodyName)
    local damageOrigin = util.vector3(
        correctedOrigin.x + desiredImpactBodyPosition.x - damageImpactBody.position.x,
        correctedOrigin.y + desiredImpactBodyPosition.y - damageImpactBody.position.y,
        correctedOrigin.z)
    ragdoll.setPowered(damageTarget, false)
    waitFor('HIT-03 target realignment cleanup', function()
        return not ragdoll.getState(damageTarget)
    end)
    damageTarget:teleport(testArea.cell, damageOrigin,
        { onGround = true, rotation = targetRotation })
    coroutine.yield()
    ragdoll.setPowered(damageTarget, true)
    local realignedDamageTarget = waitFor('HIT-03 realigned powered damage target', function()
        local current = ragdoll.getState(damageTarget)
        if current and current.controller and current.controller.mode == 'standing' then return current end
    end)
    waitSeconds(0.5)
    realignedDamageTarget = ragdoll.getState(damageTarget)
    local realignedDamageBody = findBody(realignedDamageTarget, impactBodyName)
    local damageAlignment = util.vector3(
        realignedDamageBody.position.x - desiredImpactBodyPosition.x,
        realignedDamageBody.position.y - desiredImpactBodyPosition.y, 0):length()
    testing.expectLessOrEqual(damageAlignment, 12,
        'HIT-03 damage target did not align with the qualified weapon path')
    local beforeDamageState = ragdoll.getState(player)
    local strikeBefore = beforeDamageState.weapon.strikeEventCount
    local damageBefore = beforeDamageState.weapon.damageEventCount
    local healthBefore = types.Actor.stats.dynamic.health(damageTarget).current
    ragdoll.driveSwing(player, true, util.vector3(1, 0, 0), util.vector3(0, 1, 0),
        util.vector3(0, 0, 1), 0, 0.1, true, 1 / 60)
    waitFor('HIT-03 swing lifetime activation', function()
        if ragdoll.getSwingSpeed(player) ~= nil then return true end
    end)
    local damageStarted = core.getSimulationTime()
    local qualified
    local damageMinimumDistance = math.huge
    local damageLastYaw = 0
    local currentFixtureYaw = -0.7
    local damageTargetMode = ''
    repeat
        local t = math.min((core.getSimulationTime() - damageStarted) / 4, 1)
        local smooth = t * t * (3 - 2 * t)
        currentFixtureYaw = -0.7 + 1.4 * smooth
        ragdoll.setWeaponFixtureYaw(player, currentFixtureYaw)
        coroutine.yield()
        local current = ragdoll.getState(player)
        local currentTarget = ragdoll.getState(damageTarget)
        if current and currentTarget then
            damageLastYaw = weaponYaw(current)
            damageTargetMode = currentTarget.controller.mode
            local weaponBody = findBody(current, 'equipped_weapon')
            for _, body in ipairs(currentTarget.bodies) do
                damageMinimumDistance = math.min(
                    damageMinimumDistance, (body.position - weaponBody.position):length())
            end
        end
        if current and current.weapon.damageEventCount > damageBefore then qualified = current break end
    until core.getSimulationTime() - damageStarted >= 4
    if not qualified then
        local diagnostic = ragdoll.getState(player)
        local last = diagnostic and diagnostic.weapon and diagnostic.weapon.lastContactEvent
        print(string.format('HIT-03 diagnostic raw=%d strike=%d damage=%d active=%s swing=%s speed=%.9f mass=%.9f min_distance=%.9f yaw=%.9f target_mode=%s',
            diagnostic and diagnostic.weapon.contactEventCount or -1,
            diagnostic and diagnostic.weapon.strikeEventCount or -1,
            diagnostic and diagnostic.weapon.damageEventCount or -1,
            tostring(last and last.swingActive or false), last and last.swingIdentity or '',
            last and last.relativeNormalSpeed or -1, last and last.effectiveMass or -1,
            damageMinimumDistance, damageLastYaw, damageTargetMode))
    end
    ragdoll.driveSwing(player, true, util.vector3(1, 0, 0), util.vector3(0, 1, 0),
        util.vector3(0, 0, 1), 0, 0, false, 1 / 60)
    testing.expect(qualified ~= nil, 'HIT-03 physical strike did not enter the RPG damage pipeline')
    local healthAfter = waitFor('HIT-03 target health damage', function()
        local value = types.Actor.stats.dynamic.health(damageTarget).current
        if value < healthBefore then return value end
    end)
    waitSeconds(0.25)
    local damageState = ragdoll.getState(player)
    local strikeEvent = damageState.weapon.lastStrikeEvent
    testing.expectEqual(damageState.weapon.strikeEventCount - strikeBefore, 1,
        'HIT-02 physical stroke emitted more than one qualified strike')
    testing.expectEqual(damageState.weapon.damageEventCount - damageBefore, 1,
        'HIT-03 physical stroke applied damage more than once')
    testing.expect(strikeEvent and strikeEvent.swingActive
        and tonumber(strikeEvent.swingIdentity) > 0,
        'HIT-02 qualified strike lost its swing lifetime')
    testing.expect(strikeEvent and strikeEvent.effectiveMass > 0,
        'HIT-03 qualified strike lost effective mass')
    testing.expectGreaterThan(damageState.weapon.lastImpactFraction, 0,
        'HIT-03 measured impact fraction was zero')
    testing.expectLessOrEqual(damageState.weapon.lastImpactFraction, 1,
        'HIT-03 measured impact fraction exceeded its bound')
    testing.expectGreaterThan(damageState.weapon.lastAuthoredDamage, 0,
        'HIT-03 authored weapon damage was not selected')
    testing.expectGreaterThan(damageState.weapon.lastFinalDamage, 0,
        'HIT-03 RPG modifiers removed all target damage')
    emitTelemetry('physical_weapon_damage', {
        strike_event_delta = damageState.weapon.strikeEventCount - strikeBefore,
        damage_event_delta = damageState.weapon.damageEventCount - damageBefore,
        health_damage = healthBefore - healthAfter,
        impact_fraction = damageState.weapon.lastImpactFraction,
        authored_damage = damageState.weapon.lastAuthoredDamage,
        final_damage = damageState.weapon.lastFinalDamage,
        blocked = damageState.weapon.lastDamageBlocked,
        relative_normal_speed = strikeEvent and strikeEvent.relativeNormalSpeed or 0,
        effective_mass = strikeEvent and strikeEvent.effectiveMass or 0,
        swing_identity = strikeEvent and strikeEvent.swingIdentity or '',
        swing_active = strikeEvent and strikeEvent.swingActive or false,
        minimum_body_distance = damageMinimumDistance,
        defense_outcome = damageState.weapon.lastDefenseOutcome,
        contact_disposition = damageState.weapon.lastContactDisposition,
        approach_alignment = damageState.weapon.lastApproachAlignment,
    })

    ragdoll.setPowered(damageTarget, false)
    waitFor('HIT-03 damage target cleanup', function() return not ragdoll.getState(damageTarget) end)
    damageTarget:remove()
    ragdoll.setPowered(player, false)
    if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
end)

local function runPhysicalDefenseScenario(label, expectedOutcome, guardAtSeconds, raiseImmediately, seed)
    local multiplier = ragdoll.testStepMultiplier()
    if multiplier > 0 then ragdoll.setTestStepActive(true, seed) end
    local player = world.players[1]
    player:teleport(testArea.cell, testArea.player,
        { onGround = true, rotation = util.transform.rotateZ(0) })
    waitFor(label .. ' player relocation', function()
        local delta = player.position - testArea.player
        return player.cell.name == testArea.cell
            and util.vector3(delta.x, delta.y, 0):length() <= 2
    end)
    addItem(player, 'iron warhammer')
    playerReady = nil
    player:sendEvent('weaponPlayerConfigure')
    waitFor(label .. ' equipped weapon stance', function() return playerReady end)
    testing.expect(ragdoll.prepareEquippedWeapon(player), label .. ' could not select equipped weapon')
    ragdoll.setPowered(player, true)
    waitFor(label .. ' physical weapon attachment', function()
        local current = ragdoll.getState(player)
        if current and current.weapon and current.weapon.attached
            and current.controller and current.controller.mode == 'standing' then return current end
    end)
    waitSeconds(0.5)

    moveWeaponYaw(player, 0, 0.7, 1, 0.25)
    local endpoint = ragdoll.getState(player)
    local endpointPosition = findBody(endpoint, 'equipped_weapon').position
    moveWeaponYaw(player, 0.7, -0.7, 1, 0.25)

    local targetRecord = world.createRecord(types.NPC.createRecordDraft({
        template = types.NPC.record(player),
        name = 'DEF-01 isolated ' .. label .. ' target',
        isEssential = false,
        isPersistent = false,
        isRespawning = false,
    }))
    local radial = endpointPosition - player.position
    radial = util.vector3(radial.x, radial.y, 0)
    radial = radial / radial:length()
    local desiredBodyPosition = endpointPosition + radial * 1.5
    local safeOrigin = util.vector3(endpointPosition.x, endpointPosition.y, testArea.player.z) + radial * 150
    local targetRotation = util.transform.rotateZ(math.atan2(radial.x, -radial.y) + math.pi)
    local calibrationTarget = world.createObject(targetRecord.id)
    calibrationTarget:teleport(testArea.cell, safeOrigin,
        { onGround = true, rotation = targetRotation })
    coroutine.yield()
    ragdoll.setPowered(calibrationTarget, true)
    local calibration = waitFor(label .. ' calibration target', function()
        local current = ragdoll.getState(calibrationTarget)
        if current and current.controller and current.controller.mode == 'standing' then return current end
    end)
    waitSeconds(0.25)
    calibration = ragdoll.getState(calibrationTarget)
    local impactBodyName
    local impactBodyOffset
    local inwardProjection = math.huge
    for _, body in ipairs(calibration.bodies) do
        if math.abs(body.position.z - endpointPosition.z) <= 15 then
            local offset = body.position - safeOrigin
            local projection = offset.x * radial.x + offset.y * radial.y
            if projection < inwardProjection then
                impactBodyName = body.name
                impactBodyOffset = offset
                inwardProjection = projection
            end
        end
    end
    testing.expect(impactBodyName ~= nil, label .. ' has no target body at weapon height')
    local targetOrigin = util.vector3(
        desiredBodyPosition.x - impactBodyOffset.x,
        desiredBodyPosition.y - impactBodyOffset.y,
        testArea.player.z)
    ragdoll.setPowered(calibrationTarget, false)
    waitFor(label .. ' calibration cleanup', function() return not ragdoll.getState(calibrationTarget) end)
    calibrationTarget:remove()
    coroutine.yield()

    local target = world.createObject(targetRecord.id)
    target:teleport(testArea.cell, targetOrigin,
        { onGround = true, rotation = targetRotation })
    coroutine.yield()
    ragdoll.setPowered(target, true)
    local ready = waitFor(label .. ' powered target', function()
        local current = ragdoll.getState(target)
        if current and current.controller and current.controller.mode == 'standing' then return current end
    end)
    waitSeconds(0.25)
    ready = ragdoll.getState(target)
    local actualBody = findBody(ready, impactBodyName)
    targetOrigin = util.vector3(
        targetOrigin.x + desiredBodyPosition.x - actualBody.position.x,
        targetOrigin.y + desiredBodyPosition.y - actualBody.position.y,
        targetOrigin.z)
    ragdoll.setPowered(target, false)
    waitFor(label .. ' target realignment cleanup', function() return not ragdoll.getState(target) end)
    target:teleport(testArea.cell, targetOrigin,
        { onGround = true, rotation = targetRotation })
    coroutine.yield()
    ragdoll.setPowered(target, true)
    ready = waitFor(label .. ' realigned target', function()
        local current = ragdoll.getState(target)
        if current and current.controller and current.controller.mode == 'standing' then return current end
    end)
    waitSeconds(0.25)
    ready = ragdoll.getState(target)
    actualBody = findBody(ready, impactBodyName)
    local alignment = util.vector3(actualBody.position.x - desiredBodyPosition.x,
        actualBody.position.y - desiredBodyPosition.y, 0):length()
    testing.expectLessOrEqual(alignment, 12, label .. ' target did not align with the weapon path')

    local before = ragdoll.getState(player)
    local outcomeBefore = before.weapon.defenseOutcomeCount
    local damageBefore = before.weapon.damageEventCount
    local contactBeforeDefense = before.weapon.contactEventCount
    local strikeBeforeDefense = before.weapon.strikeEventCount
    local healthBefore = types.Actor.stats.dynamic.health(target).current
    local guardFacing = player.position - target.position
    guardFacing = util.vector3(guardFacing.x, guardFacing.y, 0)
    guardFacing = guardFacing / guardFacing:length()
    print(string.format('DEF-01 isolated %s setup player=%s target=%s', label,
        tostring(player.position), tostring(target.position)))
    if raiseImmediately then
        ragdoll.setGuard(target, true, guardFacing)
        waitFor(label .. ' guard activation', function()
            local current = ragdoll.getState(target)
            return current and current.guardActive
        end)
    end

    ragdoll.driveSwing(player, true, util.vector3(1, 0, 0), util.vector3(0, 1, 0),
        util.vector3(0, 0, 1), 0, 0.1, true, 1 / 60)
    waitFor(label .. ' swing lifetime activation', function()
        if ragdoll.getSwingSpeed(player) ~= nil then return true end
    end)
    local started = core.getSimulationTime()
    local guardRaised = raiseImmediately
    local triggerDistance = -1
    local result
    local guardAge = -1
    local minimumDistance = math.huge
    repeat
        local elapsed = core.getSimulationTime() - started
        local t = math.min(elapsed / 4, 1)
        local smooth = t * t * (3 - 2 * t)
        ragdoll.setWeaponFixtureYaw(player, -0.7 + 1.4 * smooth)
        coroutine.yield()
        local current = ragdoll.getState(player)
        local defender = ragdoll.getState(target)
        if current and defender then
            local weaponPosition = findBody(current, 'equipped_weapon').position
            local distance = math.huge
            for _, body in ipairs(defender.bodies) do
                distance = math.min(distance, (weaponPosition - body.position):length())
            end
            minimumDistance = math.min(minimumDistance, distance)
            if not guardRaised and elapsed >= guardAtSeconds then
                triggerDistance = distance
                ragdoll.setGuard(target, true, guardFacing)
                guardRaised = true
            end
            if current.weapon.defenseOutcomeCount > outcomeBefore then
                print(string.format('DEF-01 isolated %s at_outcome guard=%s guard_step=%s strike_step=%s now=%s delta=%d elapsed=%.3f',
                    label, tostring(defender.guardActive), tostring(defender.guardStartPhysicsStep),
                    tostring(current.weapon.lastStrikeEvent and current.weapon.lastStrikeEvent.physicsStep),
                    tostring(current.physicsStep), current.weapon.defenseOutcomeCount - outcomeBefore, elapsed))
                result = current
                if defender.guardActive then
                    guardAge = tonumber(current.weapon.lastStrikeEvent.physicsStep)
                        - tonumber(defender.guardStartPhysicsStep)
                end
                break
            end
        end
    until core.getSimulationTime() - started >= 4.5
    if not result then
        local diagnostic = ragdoll.getState(player)
        local last = diagnostic.weapon.lastContactEvent
        local lastStrike = diagnostic.weapon.lastStrikeEvent
        print(string.format('DEF-01 isolated %s missing outcome raw_delta=%d strike_delta=%d min_distance=%.9f guard=%s trigger=%.9f last_active=%s last_target=%s strike_target=%s strike_normal=%.9f strike_total=%.9f',
            label, diagnostic.weapon.contactEventCount - contactBeforeDefense,
            diagnostic.weapon.strikeEventCount - strikeBeforeDefense, minimumDistance,
            tostring(ragdoll.getState(target).guardActive), triggerDistance,
            tostring(last and last.swingActive or false), last and last.targetActorIdentity or '',
            lastStrike and lastStrike.targetActorIdentity or '',
            lastStrike and lastStrike.relativeNormalSpeed or -1,
            lastStrike and lastStrike.relativeSpeed or -1))
    end
    testing.expect(result ~= nil, label .. ' did not emit a defense outcome')
    ragdoll.driveSwing(player, true, util.vector3(1, 0, 0), util.vector3(0, 1, 0),
        util.vector3(0, 0, 1), 0, 0, false, 1 / 60)
    waitSeconds(0.1)
    result = ragdoll.getState(player)
    local healthAfter = types.Actor.stats.dynamic.health(target).current
    print(string.format('DEF-01 isolated %s positions player=%s target=%s guard=%s', label,
        tostring(player.position), tostring(target.position), tostring(guardFacing)))
    print(string.format('DEF-01 isolated %s outcome=%s age=%d trigger=%.9f facing=%.9f redirect=%.9f',
        label, result.weapon.lastDefenseOutcome, guardAge, triggerDistance,
        result.weapon.lastGuardFacingAlignment, result.weapon.lastRedirectImpulse))
    testing.expectEqual(result.weapon.defenseOutcomeCount - outcomeBefore, 1,
        label .. ' emitted more than one defense outcome')
    testing.expectEqual(result.weapon.damageEventCount - damageBefore, 1,
        label .. ' entered combat logic more than once')
    testing.expectEqual(result.weapon.lastDefenseOutcome, expectedOutcome,
        label .. ' produced the wrong outcome')
    if expectedOutcome == 'block' or expectedOutcome == 'parry' then
        testing.expect(result.weapon.lastDamageBlocked, label .. ' did not block RPG damage')
        testing.expectEqualWithDelta(healthAfter, healthBefore, 1e-5, label .. ' changed target health')
        testing.expectGreaterThan(result.weapon.lastRedirectImpulse, 0,
            label .. ' did not redirect physical impulse')
    else
        testing.expect(not result.weapon.lastDamageBlocked, label .. ' was unexpectedly blocked')
        testing.expect(healthAfter < healthBefore, label .. ' did not damage the target')
    end
    emitTelemetry('physical_defense_' .. label, {
        outcome = result.weapon.lastDefenseOutcome,
        disposition = result.weapon.lastContactDisposition,
        outcome_event_delta = result.weapon.defenseOutcomeCount - outcomeBefore,
        damage_event_delta = result.weapon.damageEventCount - damageBefore,
        health_damage = healthBefore - healthAfter,
        blocked = result.weapon.lastDamageBlocked,
        redirect_impulse = result.weapon.lastRedirectImpulse,
        redirect_scale = result.weapon.lastRedirectScale,
        guard_age_steps = guardAge,
        guard_trigger_distance = triggerDistance,
        guard_facing_alignment = result.weapon.lastGuardFacingAlignment,
        target_alignment = alignment,
    })
    ragdoll.setPowered(target, false)
    target:remove()
    ragdoll.setPowered(player, false)
    if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
end

testing.registerGlobalTest('DEF-01 physical block', function()
    runPhysicalDefenseScenario('block', 'block', 3.65, false, 905101)
end)

testing.registerGlobalTest('DEF-01 timed parry', function()
    runPhysicalDefenseScenario('parry', 'parry', 4.05, false, 905102)
end)

testing.registerGlobalTest('DEF-01 late parry', function()
    runPhysicalDefenseScenario('late', 'hit', 0, true, 905103)
end)

local function weaponCounterDelta(after, before, key)
    return after[key] - before[key]
end

local function beginWeaponReleaseScenario(seed)
    local multiplier = ragdoll.testStepMultiplier()
    if multiplier > 0 then ragdoll.setTestStepActive(true, seed) end
    local player = world.players[1]
    player:teleport(testArea.cell, testArea.player, { onGround = true })
    waitFor('WPN-02 player relocation', function()
        local delta = player.position - testArea.player
        return player.cell.name == testArea.cell
            and util.vector3(delta.x, delta.y, 0):length() <= 2
    end)
    addItem(player, 'iron warhammer')
    playerReady = nil
    player:sendEvent('weaponPlayerConfigure')
    waitFor('WPN-02 equipped weapon stance', function() return playerReady end)
    local before = ragdoll.getCounters()
    testing.expect(ragdoll.prepareEquippedWeapon(player), 'WPN-02 could not select equipped weapon')
    ragdoll.setPowered(player, true)
    local state = waitFor('WPN-02 physical weapon attachment', function()
        local current = ragdoll.getState(player)
        if current and current.weapon and current.weapon.attached
            and current.controller and current.controller.mode == 'standing' then return current end
    end)
    testing.expect(state.weapon.constraintEnabled, 'WPN-02 grip did not start enabled')
    testing.expect(state.weapon.callbackArmed, 'WPN-02 callback did not start armed')
    testing.expectGreaterThan(state.weapon.linearBreakImpulse, 0)
    testing.expectGreaterThan(state.weapon.angularBreakImpulse, 0)
    return player, before, state, multiplier
end

local function finishWeaponReleaseScenario(reason, player, before, multiplier, released)
    local contacts = released and released.weapon.contacts or 0
    local stable = true
    if released then
        waitSeconds(0.25, function()
            local current = ragdoll.getState(player)
            if current and current.weapon and current.weapon.contacts ~= contacts then stable = false end
        end)
    end
    ragdoll.setPowered(player, false)
    waitFor('WPN-02 weapon body cleanup', function() return not ragdoll.getState(player) end)
    local after = ragdoll.getCounters()
    local expectedDuplicates = reason == 'cleanup' and 0 or 1
    testing.expectEqual(weaponCounterDelta(after, before, 'weaponBodiesCreated'), 1)
    testing.expectEqual(weaponCounterDelta(after, before, 'weaponBodiesDestroyed'), 1)
    testing.expectEqual(weaponCounterDelta(after, before, 'weaponReleases'), 1)
    testing.expectEqual(weaponCounterDelta(after, before, 'weaponCallbacksArmed'), 1)
    testing.expectEqual(weaponCounterDelta(after, before, 'weaponCallbacksDisarmed'), 1)
    testing.expectEqual(weaponCounterDelta(after, before, 'weaponDuplicateReleaseRequests'), expectedDuplicates)
    testing.expect(stable, 'WPN-02 released callback continued counting contacts')
    emitTelemetry('weapon_release_' .. reason, {
        release_reason = reason,
        release_count = weaponCounterDelta(after, before, 'weaponReleases'),
        bodies_created = weaponCounterDelta(after, before, 'weaponBodiesCreated'),
        bodies_destroyed = weaponCounterDelta(after, before, 'weaponBodiesDestroyed'),
        callbacks_armed = weaponCounterDelta(after, before, 'weaponCallbacksArmed'),
        callbacks_disarmed = weaponCounterDelta(after, before, 'weaponCallbacksDisarmed'),
        duplicate_release_requests = weaponCounterDelta(
            after, before, 'weaponDuplicateReleaseRequests'),
        constraint_disabled = released == nil or not released.weapon.constraintEnabled,
        callback_inactive = released == nil or not released.weapon.callbackArmed,
        body_retained_until_cleanup = released == nil or released.weapon.bodyRetained,
        contacts_stable_after_release = stable,
        last_linear_impulse = released and released.weapon.lastLinearImpulse or 0,
        last_angular_impulse = released and released.weapon.lastAngularImpulse or 0,
        linear_break_impulse = released and released.weapon.linearBreakImpulse or 0,
        angular_break_impulse = released and released.weapon.angularBreakImpulse or 0,
    })
    if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
end

local function registerExplicitWeaponRelease(reason, seed)
    testing.registerGlobalTest('WPN-02 ' .. reason .. ' releases weapon exactly once', function()
        local player, before, _, multiplier = beginWeaponReleaseScenario(seed)
        testing.expect(ragdoll.releaseWeapon(player, reason),
            'WPN-02 explicit release request was rejected')
        local released = waitFor('WPN-02 ' .. reason .. ' release', function()
            local current = ragdoll.getState(player)
            if current and current.weapon and current.weapon.releaseReason == reason then return current end
        end)
        testing.expectEqual(released.weapon.releaseCount, 1)
        testing.expectEqual(released.weapon.releaseRequests, 1)
        testing.expectEqual(released.weapon.attached, false)
        testing.expectEqual(released.weapon.constraintEnabled, false)
        testing.expectEqual(released.weapon.callbackArmed, false)
        testing.expectEqual(released.weapon.bodyRetained, true)
        testing.expectEqual(released.weapon.velocitySolverSteps, 0)
        finishWeaponReleaseScenario(reason, player, before, multiplier, released)
    end)
end

testing.registerGlobalTest('WPN-02 break releases weapon exactly once', function()
    local player, before, attached, multiplier = beginWeaponReleaseScenario(905021)
    ragdoll.applyImpulse(player, attached.weapon.bodyIndex, util.vector3(50000, 0, 0))
    local released = waitFor('WPN-02 finite grip break', function()
        local current = ragdoll.getState(player)
        if current and current.weapon and current.weapon.releaseReason == 'break' then return current end
    end)
    testing.expect(released.weapon.lastLinearImpulse > released.weapon.linearBreakImpulse
        or released.weapon.lastAngularImpulse > released.weapon.angularBreakImpulse,
        'WPN-02 break did not cross a finite grip limit')
    testing.expectEqual(released.weapon.releaseCount, 1)
    testing.expectEqual(released.weapon.releaseRequests, 1)
    testing.expectEqual(released.weapon.attached, false)
    testing.expectEqual(released.weapon.constraintEnabled, false)
    testing.expectEqual(released.weapon.callbackArmed, false)
    testing.expectEqual(released.weapon.bodyRetained, true)
    finishWeaponReleaseScenario('break', player, before, multiplier, released)
end)

registerExplicitWeaponRelease('drop', 905022)
registerExplicitWeaponRelease('unequip', 905023)
registerExplicitWeaponRelease('death', 905024)

testing.registerGlobalTest('WPN-02 cleanup releases weapon exactly once', function()
    local player, before, _, multiplier = beginWeaponReleaseScenario(905025)
    ragdoll.setPowered(player, false)
    waitFor('WPN-02 direct cleanup', function() return not ragdoll.getState(player) end)
    local after = ragdoll.getCounters()
    testing.expectEqual(weaponCounterDelta(after, before, 'weaponBodiesCreated'), 1)
    testing.expectEqual(weaponCounterDelta(after, before, 'weaponBodiesDestroyed'), 1)
    testing.expectEqual(weaponCounterDelta(after, before, 'weaponReleases'), 1)
    testing.expectEqual(weaponCounterDelta(after, before, 'weaponCallbacksArmed'), 1)
    testing.expectEqual(weaponCounterDelta(after, before, 'weaponCallbacksDisarmed'), 1)
    testing.expectEqual(weaponCounterDelta(after, before, 'weaponDuplicateReleaseRequests'), 0)
    emitTelemetry('weapon_release_cleanup', {
        release_reason = 'cleanup',
        release_count = 1,
        bodies_created = 1,
        bodies_destroyed = 1,
        callbacks_armed = 1,
        callbacks_disarmed = 1,
        duplicate_release_requests = 0,
        constraint_disabled = true,
        callback_inactive = true,
        body_retained_until_cleanup = true,
        contacts_stable_after_release = true,
    })
    if multiplier > 0 then ragdoll.setTestStepActive(false, 0) end
end)

local handlers = {}
for name, handler in pairs(testing.globalEventHandlers) do handlers[name] = handler end
handlers.weaponPlayerReady = function(data) playerReady = data.weapon end

return {
    engineHandlers = { onUpdate = testing.updateGlobal },
    eventHandlers = handlers,
}

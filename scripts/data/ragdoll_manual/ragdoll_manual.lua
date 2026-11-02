local core = require('openmw.core')
local ragdoll = require('openmw.ragdolltest')
local types = require('openmw.types')
local util = require('openmw.util')
local world = require('openmw.world')
local testArea = require('ragdoll_test_area')
-- The manual launcher can stand the range in Balmora (-Area balmora): the old
-- exterior range spot by the town's central pathgrid (flat ground, with slopes,
-- stairs and the river bridges around it). Same relative spacing.
do
    local ok, hubConfig = pcall(require, 'manual_hub_config')
    if ok and type(hubConfig) == 'table' and hubConfig.area == 'balmora' then
        local origin = util.vector3(-25741, -11821, 1200)
        testArea = {
            id = 'balmora-v1', cell = '', floorHeight = nil,
            player = origin,
            powered = util.vector3(-26042, -11656, 1200),
            passive = util.vector3(-25375, -11812, 1200),
            gait = util.vector3(-25255, -11524, 1200),
            defense = origin + util.vector3(0, -80, 0),
        }
    end
end

local setupStage = -1
local setupReadyTime = 0
local setupSettleDuration = 5 / 60
local openMwUnitsPerMeter = 69.99125109
local controlledFallVelocityChange = 2.0 * openMwUnitsPerMeter
local traceArmingDuration = 0.25
local traceArmingTimeout = 5
local playerTestOrigin = testArea.player
local sampleDefinitions = {
    powered = {
        label = '[Ragdoll A] Powered Balance / Recovery',
        position = testArea.powered,
    },
    passive = {
        label = '[Ragdoll B] Passive Death',
        position = testArea.passive,
    },
    gait = {
        label = '[Ragdoll C] Powered Gait (Experimental)',
        position = testArea.gait,
    },
    defense = {
        label = '[Ragdoll D] Physical Defense Target',
        position = testArea.defense,
    },
}
local sampleOrder = { 'powered', 'passive', 'gait', 'defense' }
local sampleRecords = {}
local samples = {}
-- Manual grab (G in the range): the sample nearest the camera ray is held
-- in front of the camera by a capped spring force; strength in force units
-- (the rig weighs about 82000).
local grab = nil
local grabStrength = 40000

local function rayDistance(position, ray)
    local offset = position - ray.from
    local along = offset:dot(ray.direction)
    if along < 0 then
        return math.huge, along
    end
    return (offset - ray.direction * along):length(), along
end

local function manualGrab(data)
    local ray = data.ray
    if data.action == 'release' or (data.action == 'grab' and grab) then
        if grab then
            ragdoll.releaseGrab(grab.actor)
            grab = nil
        end
        if data.action == 'release' then
            return
        end
    end
    if data.action == 'grab' then
        local best, bestDistance, bestAlong = nil, 90, 0
        for _, actor in pairs(samples) do
            if actor:isValid() and ragdoll.getState(actor) then
                local distance, along = rayDistance(actor.position + util.vector3(0, 0, 60), ray)
                if distance < bestDistance then
                    best, bestDistance, bestAlong = actor, distance, along
                end
            end
        end
        if not best then
            return
        end
        ragdoll.grab(best, ray.from, ray.direction, grabStrength)
        grab = { actor = best, distance = math.max(60, math.min(160, bestAlong)) }
    elseif data.action == 'update' and grab then
        if not grab.actor:isValid() then
            grab = nil
            return
        end
        ragdoll.updateGrab(grab.actor, ray.from + ray.direction * grab.distance)
    end
end
local pendingSamples = {}
local pendingRecovery
local activeTrace
local population
local lastTraceSummary
local traceSequence = 0
local defenseSession
local defenseSwing
local prepareDefense
local updateDefense
local updateAutoDuel
local pendingDefenseSetup = false
-- Whether accelerated test stepping (OPENMW_RAGDOLL_TEST_STEP_MULTIPLIER) is
-- currently engaged in this session.
local testStepActive = false

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

local function emitTraceRecord(kind, trace, metrics)
    local fields = {
        '"schema_version":5',
        '"test_fixture":' .. jsonString(testArea.id),
        '"kind":' .. jsonString(kind),
        '"trace_id":' .. tostring(trace.id),
        '"sample":' .. jsonString(trace.sample),
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
            if value ~= value then
                encoded = jsonString('nan')
            elseif value == math.huge then
                encoded = jsonString('infinity')
            elseif value == -math.huge then
                encoded = jsonString('-infinity')
            else
                encoded = string.format('%.17g', value)
            end
        elseif type(value) == 'boolean' then
            encoded = tostring(value)
        elseif type(value) == 'string' then
            encoded = jsonString(value)
        else
            error('Unsupported manual trace field type for ' .. key)
        end
        table.insert(fields, jsonString(key) .. ':' .. encoded)
    end
    print('RAGDOLL_MANUAL_TRACE_JSON\t{' .. table.concat(fields, ',') .. '}')
end

local function findBody(current, name)
    for _, body in ipairs(current.bodies) do
        if body.name == name then
            return body
        end
    end
    return nil
end

local function finishTrace(reason, current)
    local trace = activeTrace
    if not trace then
        return nil
    end
    activeTrace = nil
    if trace.recording then
        local ok, replay = pcall(ragdoll.finishReplayRecording)
        if ok and replay then
            print(string.format('Replay recorded: seed %s, %s frames%s', tostring(replay.seed),
                tostring(replay.frameCount or '?'), replay.outputPath and (', saved to ' .. replay.outputPath) or
                ' (set OPENMW_RAGDOLL_REPLAY_OUTPUT_DIR to persist)'))
        else
            print('Replay recording did not finish: ' .. tostring(replay))
        end
    end
    if trace.stopGait and trace.actor:isValid() then
        trace.actor:sendEvent('setManualRagdollGait', false)
    end

    local summary = {
        traceId = trace.id,
        sampleKey = trace.sample,
        reason = reason,
        elapsed = math.max(0, core.getSimulationTime() - trace.startTime),
        armingElapsed = trace.stopGait and (trace.gaitStartTime and trace.gaitStartTime - trace.startTime
            or math.max(0, core.getSimulationTime() - trace.startTime)) or 0,
        gaitElapsed = trace.gaitStartTime
            and math.max(0, core.getSimulationTime() - trace.gaitStartTime) or 0,
        frames = trace.frames,
        updateSamples = trace.updateSamples,
        duplicatePhysicsSteps = trace.duplicatePhysicsSteps,
        skippedPhysicsSteps = trace.skippedPhysicsSteps,
        maximumGameplayRootLag = trace.maximumGameplayRootLag,
        maximumRenderPelvisError = trace.maximumRenderPelvisError,
        maximumRenderPelvisJump = trace.maximumRenderPelvisJump,
        firstMode = trace.firstMode or 'unavailable',
        finalMode = current and current.controller and current.controller.mode or trace.lastMode or 'unavailable',
        gaitTransitions = trace.gaitTransitions,
        gaitTimeoutFrames = trace.gaitTimeoutFrames,
        gaitFrames = trace.gaitFrames,
        unsupportedFrames = trace.unsupportedFrames,
        armingUnsupportedFrames = trace.armingUnsupportedFrames,
        upwardSaturationFrames = trace.upwardSaturationFrames,
        armingUpwardSaturationFrames = trace.armingUpwardSaturationFrames,
        maximumConsecutiveUpwardSaturation = trace.maximumConsecutiveUpwardSaturation,
        maximumAnchorSeparation = trace.maximumAnchorSeparation,
        maximumBodySpeed = trace.maximumBodySpeed,
        maximumRootSeparation = trace.maximumRootSeparation,
        minimumPelvisUp = trace.minimumPelvisUp < math.huge and trace.minimumPelvisUp or 0,
        maximumTrackingError = trace.maximumTrackingError,
        finalTrackingError = current and current.controller
            and current.controller.velocityTrackingError or trace.finalTrackingError,
        firstFallingFrame = trace.firstFallingFrame or -1,
    }
    lastTraceSummary = summary
    summary.classifiedFrames = 0
    summary.uprightClassifiedFrames = 0
    for _, stage in ipairs({ 'inactive', 'settling', 'awaiting_stance', 'weight_transfer',
        'unloading', 'swing', 'touchdown' }) do
        local metrics = trace.stages[stage] or { frames = 0, achievedSpeedSum = 0,
            requestedSpeedSum = 0, unsupportedFrames = 0, fallingFrames = 0, swingHipSaturatedFrames = 0,
            swingKneeSaturatedFrames = 0, swingAnkleSaturatedFrames = 0 }
        summary.classifiedFrames = summary.classifiedFrames + metrics.frames
        emitTraceRecord('stage_summary', trace, {
            policy = 'motor', gait_stage = stage, frames = metrics.frames,
            mean_achieved_speed = metrics.frames > 0 and metrics.achievedSpeedSum / metrics.frames or 0,
            mean_requested_speed = metrics.frames > 0 and metrics.requestedSpeedSum / metrics.frames or 0,
            unsupported_frames = metrics.unsupportedFrames, falling_frames = metrics.fallingFrames,
            swing_hip_saturated_frames = metrics.swingHipSaturatedFrames,
            swing_knee_saturated_frames = metrics.swingKneeSaturatedFrames,
            swing_ankle_saturated_frames = metrics.swingAnkleSaturatedFrames,
        })
        local forces = trace.uprightStages[stage] or { frames = 0, brakingFrames = 0,
            verticalSaturatedFrames = 0, momentSaturatedFrames = 0 }
        summary.uprightClassifiedFrames = summary.uprightClassifiedFrames + forces.frames
        emitTraceRecord('stage_summary', trace, {
            policy = 'upright', gait_stage = stage, frames = forces.frames,
            braking_frames = forces.brakingFrames, vertical_saturated_frames = forces.verticalSaturatedFrames,
            moment_saturated_frames = forces.momentSaturatedFrames,
        })
    end
    emitTraceRecord('summary', trace, summary)
    print(string.format(
        'Manual ragdoll trace %d complete: %d physics samples, %d transitions, %d timeout frames, final mode %s',
        trace.id, trace.frames, trace.gaitTransitions, trace.gaitTimeoutFrames, summary.finalMode))
    return summary
end

local function updateTrace()
    local trace = activeTrace
    if not trace then
        return
    end
    if not trace.actor:isValid() then
        finishTrace('actor_invalid')
        return
    end

    local now = core.getSimulationTime()
    local current = ragdoll.getState(trace.actor)
    if not current or not current.controller then
        if now - trace.startTime >= traceArmingTimeout then
            finishTrace('state_unavailable')
        end
        return
    end
    -- Sample presentation on every Lua update, including updates with no new
    -- physics step. Fixed-step de-duplication otherwise hides interpolation
    -- flicker. These are prepared render positions, not measured screen pixels.
    local pelvis = findBody(current, 'pelvis')
    local gameplayRoot = trace.actor.position
    local gameplayRootLag = (gameplayRoot - current.referencePosition):length()
    local renderPelvisError = pelvis and (pelvis.renderPosition - pelvis.position):length() or 0
    local renderPelvisJump = pelvis and trace.previousRenderPelvis
        and (pelvis.renderPosition - trace.previousRenderPelvis):length() or 0
    local stepDelta = trace.lastPhysicsStep >= 0 and current.physicsStep - trace.lastPhysicsStep or 0
    trace.updateSamples = trace.updateSamples + 1
    trace.maximumGameplayRootLag = math.max(trace.maximumGameplayRootLag, gameplayRootLag)
    trace.maximumRenderPelvisError = math.max(trace.maximumRenderPelvisError, renderPelvisError)
    trace.maximumRenderPelvisJump = math.max(trace.maximumRenderPelvisJump, renderPelvisJump)
    emitTraceRecord('presentation', trace, {
        elapsed = now - trace.startTime,
        sample_dt = trace.previousSampleTime and now - trace.previousSampleTime or 0,
        update_sample = trace.updateSamples, physics_step = current.physicsStep, step_delta = stepDelta,
        mode = current.controller.mode, render_interpolation = current.renderInterpolationFactor,
        gameplay_root_x = gameplayRoot.x, gameplay_root_y = gameplayRoot.y, gameplay_root_z = gameplayRoot.z,
        reference_x = current.referencePosition.x, reference_y = current.referencePosition.y,
        reference_z = current.referencePosition.z, gameplay_root_lag = gameplayRootLag,
        pelvis_x = pelvis and pelvis.position.x or 0, pelvis_y = pelvis and pelvis.position.y or 0,
        pelvis_z = pelvis and pelvis.position.z or 0,
        render_pelvis_x = pelvis and pelvis.renderPosition.x or 0,
        render_pelvis_y = pelvis and pelvis.renderPosition.y or 0,
        render_pelvis_z = pelvis and pelvis.renderPosition.z or 0,
        render_pelvis_error = renderPelvisError, render_pelvis_jump = renderPelvisJump,
    })
    trace.previousRenderPelvis = pelvis and pelvis.renderPosition
    trace.previousSampleTime = now
    if current.physicsStep == trace.lastPhysicsStep then
        trace.duplicatePhysicsSteps = trace.duplicatePhysicsSteps + 1
        return
    end
    trace.skippedPhysicsSteps = trace.skippedPhysicsSteps + math.max(0, stepDelta - 1)
    trace.lastPhysicsStep = current.physicsStep

    trace.frames = trace.frames + 1
    local controller = current.controller
    local gaitActive = trace.gaitStartTime ~= nil
    if gaitActive then
        trace.gaitFrames = trace.gaitFrames + 1
    end
    local support = current.support
    local upright = controller.upright
    local leftFoot = findBody(current, 'left_foot')
    local rightFoot = findBody(current, 'right_foot')
    local maximumAnchorSeparation = 0
    for _, constraint in ipairs(current.constraints) do
        maximumAnchorSeparation = math.max(maximumAnchorSeparation, constraint.anchorSeparation)
    end
    local maximumBodySpeed = 0
    for _, body in ipairs(current.bodies) do
        maximumBodySpeed = math.max(maximumBodySpeed, body.linearVelocity:length())
    end
    local rootSeparation = current.physicsActorPosition
        and (current.physicsActorPosition - current.referencePosition):length() or 0

    local stage = controller.gaitActuationStage
    local metrics = trace.stages[stage]
    if not metrics then
        metrics = { frames = 0, achievedSpeedSum = 0, requestedSpeedSum = 0,
            unsupportedFrames = 0, fallingFrames = 0, swingHipSaturatedFrames = 0,
            swingKneeSaturatedFrames = 0, swingAnkleSaturatedFrames = 0 }
        trace.stages[stage] = metrics
    end
    metrics.frames = metrics.frames + 1
    local requested = controller.targetHorizontalVelocity
    local speed = math.sqrt(requested.x * requested.x + requested.y * requested.y)
    metrics.requestedSpeedSum = metrics.requestedSpeedSum + speed
    if speed > 0 then
        metrics.achievedSpeedSum = metrics.achievedSpeedSum
            + (controller.centerOfMassVelocity.x * requested.x
                + controller.centerOfMassVelocity.y * requested.y) / speed
    end
    if not support or support.groundContactCount == 0 then
        metrics.unsupportedFrames = metrics.unsupportedFrames + 1
    end
    if controller.mode == 'falling' or controller.mode == 'fallen' then
        metrics.fallingFrames = metrics.fallingFrames + 1
    end
    if controller.jointMotorsApplied and (controller.swingHipMotorUtilization or 0) >= 0.999 then
        metrics.swingHipSaturatedFrames = metrics.swingHipSaturatedFrames + 1
    end
    if controller.jointMotorsApplied and (controller.swingKneeMotorUtilization or 0) >= 0.999 then
        metrics.swingKneeSaturatedFrames = metrics.swingKneeSaturatedFrames + 1
    end
    if controller.jointMotorsApplied and (controller.swingAnkleMotorUtilization or 0) >= 0.999 then
        metrics.swingAnkleSaturatedFrames = metrics.swingAnkleSaturatedFrames + 1
    end
    local forceStage = controller.uprightActuationStage
    local forces = trace.uprightStages[forceStage]
    if not forces then
        forces = { frames = 0, brakingFrames = 0, verticalSaturatedFrames = 0, momentSaturatedFrames = 0 }
        trace.uprightStages[forceStage] = forces
    end
    forces.frames = forces.frames + 1
    if upright and upright.applied then
        local applied = controller.appliedPelvisForce
        if applied.x * requested.x + applied.y * requested.y < 0 then
            forces.brakingFrames = forces.brakingFrames + 1
        end
        if controller.appliedVerticalForceSaturated then
            forces.verticalSaturatedFrames = forces.verticalSaturatedFrames + 1
        end
        if controller.appliedSupportMomentSaturated then
            forces.momentSaturatedFrames = forces.momentSaturatedFrames + 1
        end
    end

    trace.firstMode = trace.firstMode or controller.mode
    trace.lastMode = controller.mode
    trace.finalTrackingError = controller.velocityTrackingError
    trace.maximumAnchorSeparation = math.max(trace.maximumAnchorSeparation, maximumAnchorSeparation)
    trace.maximumBodySpeed = math.max(trace.maximumBodySpeed, maximumBodySpeed)
    trace.maximumRootSeparation = math.max(trace.maximumRootSeparation, rootSeparation)
    trace.minimumPelvisUp = math.min(trace.minimumPelvisUp, controller.pelvisUpDot)
    trace.maximumTrackingError = math.max(trace.maximumTrackingError, controller.velocityTrackingError)
    if controller.gaitTransitioned then
        trace.gaitTransitions = trace.gaitTransitions + 1
    end
    if controller.gaitPhaseTimedOut then
        trace.gaitTimeoutFrames = trace.gaitTimeoutFrames + 1
    end
    if not support or support.groundContactCount == 0 then
        if gaitActive or not trace.stopGait then
            trace.unsupportedFrames = trace.unsupportedFrames + 1
        else
            trace.armingUnsupportedFrames = trace.armingUnsupportedFrames + 1
        end
    end
    if upright and upright.applied
        and upright.pelvisVerticalForceSaturated and upright.pelvisForce.z > 0 then
        if gaitActive or not trace.stopGait then
            trace.upwardSaturationFrames = trace.upwardSaturationFrames + 1
            trace.consecutiveUpwardSaturation = trace.consecutiveUpwardSaturation + 1
            trace.maximumConsecutiveUpwardSaturation = math.max(
                trace.maximumConsecutiveUpwardSaturation, trace.consecutiveUpwardSaturation)
        else
            trace.armingUpwardSaturationFrames = trace.armingUpwardSaturationFrames + 1
        end
    else
        trace.consecutiveUpwardSaturation = 0
    end
    if not trace.firstFallingFrame and (controller.mode == 'falling' or controller.mode == 'fallen') then
        trace.firstFallingFrame = trace.frames
    end

    emitTraceRecord('frame', trace, {
        elapsed = now - trace.startTime,
        gait_elapsed = trace.gaitStartTime and now - trace.gaitStartTime or -1,
        physics_step = current.physicsStep,
        stage = trace.stopGait and (trace.gaitStartTime and 'gait' or 'arming') or 'standing',
        gait_stage = controller.gaitStage,
        actuation_stage = controller.gaitActuationStage,
        stance_hip_axis_released = controller.stanceHipAxisReleased,
        swing_hip_axis_released = controller.swingHipAxisReleased,
        swing_ankle_world_motor = controller.swingAnkleWorldMotor,
        backward_recovery_step = controller.backwardRecoveryStep,
        upright_actuation_stage = controller.uprightActuationStage,
        applied_pelvis_force_x = controller.appliedPelvisForce.x,
        applied_pelvis_force_y = controller.appliedPelvisForce.y,
        applied_pelvis_force_z = controller.appliedPelvisForce.z,
        applied_vertical_saturated = controller.appliedVerticalForceSaturated,
        applied_moment_saturated = controller.appliedSupportMomentSaturated,
        frame = trace.frames,
        mode = controller.mode,
        -- Which get-up the controller committed to, and the torso reading it
        -- chose from. A body on its side reads near zero and the choice is a
        -- coin flip, so both have to be in the trace to tell a bad get-up
        -- from an unlucky one.
        recovery_direction = controller.recoveryDirection or 'none',
        left_arm_reach = current.arms and current.arms.left and current.arms.left.reachError or -1,
        right_arm_reach = current.arms and current.arms.right and current.arms.right.reachError or -1,
        left_arm_weight = current.arms and current.arms.left and current.arms.left.weight or 0,
        right_arm_weight = current.arms and current.arms.right and current.arms.right.weight or 0,
        left_arm_brace = (current.arms and current.arms.left and current.arms.left.brace) or false,
        right_arm_brace = (current.arms and current.arms.right and current.arms.right.brace) or false,
        torso_up = controller.torsoUpDot,
        heading_error = controller.headingError or 0,
        pelvis_up = controller.pelvisUpDot,
        pelvis_height = controller.pelvisHeight,
        com_x = current.centerOfMass.x,
        com_y = current.centerOfMass.y,
        com_z = current.centerOfMass.z,
        com_velocity_x = controller.centerOfMassVelocity.x,
        com_velocity_y = controller.centerOfMassVelocity.y,
        target_velocity_x = controller.targetHorizontalVelocity.x,
        target_velocity_y = controller.targetHorizontalVelocity.y,
        observation_real_time = core.getRealTime(),
        observation_simulation_time = now,
        gameplay_yaw = trace.actor.rotation:getYaw(),
        raw_travel_x = controller.rawTargetHorizontalVelocity and controller.rawTargetHorizontalVelocity.x or 0,
        raw_travel_y = controller.rawTargetHorizontalVelocity and controller.rawTargetHorizontalVelocity.y or 0,
        raw_travel_known = controller.rawTargetHorizontalVelocity ~= nil,
        tracking_error = controller.velocityTrackingError,
        gait_phase = controller.gaitPhase,
        gait_phase_time = controller.gaitTimeInPhase,
        gait_phase_progress = controller.gaitPhaseProgress,
        stance_ready = controller.gaitStanceReady,
        gait_transitioned = controller.gaitTransitioned,
        gait_timed_out = controller.gaitPhaseTimedOut,
        swing_foot_lifted = controller.gaitSwingFootLifted,
        swing_time_since_lift = controller.gaitSwingTimeSinceLift,
        swing_touchdown_candidate = controller.gaitSwingTouchdownCandidate,
        swing_contact_candidate_time = controller.gaitSwingContactCandidateTime,
        ground_contacts = support and support.groundContactCount or 0,
        foot_contacts = support and support.footContactCount or 0,
        left_foot_supported = support and support.leftFoot or false,
        right_foot_supported = support and support.rightFoot or false,
        com_supported = support and support.centerOfMassSupported or false,
        support_distance = support and support.centerOfMassDistance or 0,
        support_height = support and support.centerOfMassHeight or 0,
        left_foot_x = leftFoot and leftFoot.position.x or 0,
        left_foot_y = leftFoot and leftFoot.position.y or 0,
        left_foot_z = leftFoot and leftFoot.position.z or 0,
        left_foot_velocity_z = leftFoot and leftFoot.linearVelocity.z or 0,
        right_foot_x = rightFoot and rightFoot.position.x or 0,
        right_foot_y = rightFoot and rightFoot.position.y or 0,
        right_foot_z = rightFoot and rightFoot.position.z or 0,
        right_foot_velocity_z = rightFoot and rightFoot.linearVelocity.z or 0,
        swing_target_x = controller.swingFootTarget and controller.swingFootTarget.x or 0,
        swing_target_y = controller.swingFootTarget and controller.swingFootTarget.y or 0,
        swing_target_z = controller.swingFootTarget and controller.swingFootTarget.z or 0,
        swing_controller_world_x = controller.swingFootControllerWorldTarget
            and controller.swingFootControllerWorldTarget.x or 0,
        swing_controller_world_y = controller.swingFootControllerWorldTarget
            and controller.swingFootControllerWorldTarget.y or 0,
        swing_controller_world_z = controller.swingFootControllerWorldTarget
            and controller.swingFootControllerWorldTarget.z or 0,
        swing_hip_motor_utilization = controller.swingHipMotorUtilization or 0,
        swing_hip_motor_error = controller.swingHipMotorError or 0,
        swing_hip_motor_clamp = controller.swingHipMotorClamp or 0,
        swing_knee_motor_utilization = controller.swingKneeMotorUtilization or 0,
        swing_knee_motor_error = controller.swingKneeMotorError or 0,
        swing_knee_motor_clamp = controller.swingKneeMotorClamp or 0,
        swing_knee_current_angle = controller.swingKneeCurrentAngle or 0,
        swing_knee_target_angle = controller.swingKneeTargetAngle or 0,
        swing_ankle_motor_utilization = controller.swingAnkleMotorUtilization or 0,
        swing_ankle_motor_error = controller.swingAnkleMotorError or 0,
        swing_ankle_motor_clamp = controller.swingAnkleMotorClamp or 0,
        swing_force_x = controller.swingFootForce and controller.swingFootForce.x or 0,
        swing_force_y = controller.swingFootForce and controller.swingFootForce.y or 0,
        swing_force_z = controller.swingFootForce and controller.swingFootForce.z or 0,
        swing_force_saturated = controller.swingFootForceSaturated or false,
        swing_force_applied = controller.swingFootControlApplied or false,
        pre_lift_swing_unload_force = controller.preLiftSwingUnloadForce or 0,
        target_pelvis_height = controller.targetPelvisHeight or 0,
        pelvis_height_candidate_time = controller.pelvisHeightCandidateTime or 0,
        pelvis_height_candidate_valid = controller.pelvisHeightCandidateValid or false,
        pelvis_height_settled = controller.pelvisHeightSettled or false,
        pelvis_height_frozen = controller.pelvisHeightFrozen or false,
        pelvis_velocity_z = pelvis and pelvis.linearVelocity.z or 0,
        pelvis_force_x = upright and upright.pelvisForce.x or 0,
        pelvis_force_y = upright and upright.pelvisForce.y or 0,
        pelvis_force_z = upright and upright.pelvisForce.z or 0,
        upright_applied = upright and upright.applied or false,
        joint_support_applied = controller.jointSupportApplied,
        joint_support_contact = controller.jointSupportContact,
        joint_support_utilization = controller.jointSupportMotorUtilization,
        pelvis_magnitude_saturated = upright and upright.pelvisMagnitudeSaturated or false,
        pelvis_horizontal_saturated = upright and upright.pelvisHorizontalForceSaturated or false,
        pelvis_vertical_saturated = upright and upright.pelvisVerticalForceSaturated or false,
        maximum_anchor_separation = maximumAnchorSeparation,
        maximum_body_speed = maximumBodySpeed,
        root_separation = rootSeparation,
        render_interpolation = current.renderInterpolationFactor,
    })

    if not trace.stopGait then
        local durationStart = trace.movementDuration and trace.gaitStartTime or trace.startTime
        if durationStart and now - durationStart >= trace.duration then
            finishTrace('duration_complete', current)
        end
    elseif not trace.gaitStartTime then
        local ready = controller.mode == 'standing' and support and support.leftFoot
            and support.rightFoot and support.centerOfMassSupported and controller.pelvisHeightSettled
            and controller.centerOfMassSpeed <= 10
        if ready then
            trace.stableSince = trace.stableSince or now
            if now - trace.stableSince >= traceArmingDuration then
                trace.gaitStartTime = now
                trace.consecutiveUpwardSaturation = 0
                trace.actor:sendEvent('setManualRagdollGait', true)
                emitTraceRecord('gait_start', trace, { arming_elapsed = now - trace.startTime })
                print(string.format('Manual gait trace %d armed after %.3f seconds; gait started',
                    trace.id, now - trace.startTime))
            end
        else
            trace.stableSince = nil
        end
        if not trace.gaitStartTime and now - trace.startTime >= traceArmingTimeout then
            finishTrace('arming_timeout', current)
        end
    elseif now - trace.gaitStartTime >= trace.duration then
        finishTrace('duration_complete', current)
    end
end

-- Weapons used by the manual room and its continuous observation circuit.
-- Missing records are skipped so a different install still works.
local testWeapons = { 'iron dagger', 'iron longsword', 'iron claymore', 'iron warhammer',
    'steel warhammer', 'wooden staff', 'iron spear', 'iron battle axe' }

local function giveTestWeapons(player)
    local inventory = types.Actor.inventory(player)
    for _, recordId in ipairs(testWeapons) do
        local ok, existing = pcall(function() return inventory:countOf(recordId) end)
        if ok and existing == 0 then
            local created = pcall(function()
                world.createObject(recordId, 1):moveInto(inventory)
            end)
            if not created then
                print('Ragdoll range: weapon record "' .. recordId .. '" is unavailable in this install')
            end
        end
    end
end

local function removeActors(actors)
    for _, actor in pairs(actors) do
        if actor and actor:isValid() then
            actor:remove()
        end
    end
end

local function resolveActor(value)
    if value == 'player' then
        return world.players[1]
    end
    if type(value) == 'string' then
        local actor = samples[value]
        if not actor or not actor:isValid() then
            error('Ragdoll sample "' .. value .. '" is unavailable; wait for the test range to be ready', 3)
        end
        return actor
    end
    return value
end

local function ensureSampleRecords(player)
    local template = types.NPC.record(player)
    for _, key in ipairs(sampleOrder) do
        if not sampleRecords[key] then
            local definition = sampleDefinitions[key]
            local draft = types.NPC.createRecordDraft({
                template = template,
                name = definition.label,
                isEssential = false,
                isPersistent = false,
                isRespawning = false,
            })
            sampleRecords[key] = world.createRecord(draft)
        end
    end
end

local function beginSetup(prepareDefenseAfterSetup)
    pendingDefenseSetup = prepareDefenseAfterSetup == true
    local player = world.players[1]
    if player and player:isValid() then ragdoll.setPowered(player, false) end
    if population then population.reset() end
    finishTrace('range_reset')
    pendingRecovery = nil
    defenseSession = nil
    defenseSwing = nil
    removeActors(samples)
    removeActors(pendingSamples)
    samples = {}
    pendingSamples = {}
    world.players[1]:teleport(testArea.cell, playerTestOrigin,
        { onGround = true, rotation = util.transform.rotateZ(0) })
    giveTestWeapons(world.players[1])
    -- Morrowind decides whether a swing connects with a dice roll before any
    -- geometry is considered, so physical impacts cannot be calibrated by
    -- hand while it is on. The range turns it off for the player - unless the
    -- launcher asks for vanilla's roll on both sides (OPENMW_RAGDOLL_PLAYER_ROLL=1).
    ragdoll.setPlayerAlwaysHits(not (ragdoll.playerRollWanted and ragdoll.playerRollWanted()))
    -- The duelists are cloned from the developer character, whose
    -- hand-to-hand skill is maxed. Normalize bare-handed damage to what a
    -- normally skilled fighter deals (the weapon formulas do not scale with
    -- skill the same way), and use the graded reaction default.
    ragdoll.handDamageScale(0.35)
    -- 72 since DMG-04 (2026-09-28): every blow now pays its full band, so
    -- a full claymore hit staggers and only repeated ones floor.
    ragdoll.reactionScale(72)
    -- Accelerated test stepping (OPENMW_RAGDOLL_TEST_STEP_MULTIPLIER): the
    -- duel then resolves in a fraction of the wall-clock time when requested.
    local stepMultiplier = ragdoll.testStepMultiplier()
    if stepMultiplier > 0 then
        ragdoll.setTestStepActive(true, 424242)
        testStepActive = true
        print(string.format('Ragdoll range: accelerated test stepping k=%d active', stepMultiplier))
    end
    -- Every bounded trace below observes a controlled state for a fixed
    -- window, which an unaided get-up would end early. Tests that want the
    -- gameplay behaviour turn it back on themselves.
    ragdoll.setAutomaticRecovery(false)
    print('Ragdoll range: player melee connects on geometry (to-hit roll skipped)')
    setupStage = 1
    setupReadyTime = core.getSimulationTime() + setupSettleDuration
end

local function updateSetup()
    updateAutoDuel()
    if pendingRecovery then
        if not pendingRecovery:isValid() then
            pendingRecovery = nil
            print('Ragdoll recovery queue cancelled because the actor is no longer valid')
        else
            local current = ragdoll.getState(pendingRecovery)
            if not current or not current.powered then
                pendingRecovery = nil
                print('Ragdoll recovery queue cancelled because powered mode ended')
            elseif current.controller and current.controller.mode == 'fallen' then
                ragdoll.requestRecovery(pendingRecovery)
                pendingRecovery = nil
                print('Authored powered-ragdoll recovery requested from the fallen state')
            elseif current.controller and (current.controller.mode == 'getting_up'
                or current.controller.mode == 'blend_back') then
                pendingRecovery = nil
            end
        end
    end

    if setupStage == 1 then
        if core.getSimulationTime() >= setupReadyTime then
            local player = world.players[1]
            ensureSampleRecords(player)
            for _, key in ipairs(sampleOrder) do
                local definition = sampleDefinitions[key]
                local actor = world.createObject(sampleRecords[key].id)
                actor:teleport(testArea.cell, definition.position, { onGround = true })
                actor:addScript('ragdoll_manual_actor.lua')
                pendingSamples[key] = actor
            end
            setupStage = 2
            setupReadyTime = core.getSimulationTime() + setupSettleDuration
        end
    elseif setupStage == 2 then
        if core.getSimulationTime() >= setupReadyTime then
            samples = pendingSamples
            pendingSamples = {}
            setupStage = 3
            print('=== RAGDOLL TEST RANGE READY ===')
            print('[A] powered  - standing balance, controlled fall, and recovery')
            print('[B] passive  - death impulse, collision, settling, and resurrection')
            print('[C] gait     - experimental powered walking and balance tracking')
            print('[D] defense  - player physical weapon hit, block, parry, and late-guard checks')
            print('F8 opens the manual ragdoll controls.')
            print('Use I.RagdollDev.help() for the exact command sequence')
            if pendingDefenseSetup and prepareDefense then
                pendingDefenseSetup = false
                prepareDefense()
            end
        end
    end
    updateTrace()
    if population then population.update() end
    if updateDefense then updateDefense() end
end

local function setup()
    beginSetup(false)
end

local function actor()
    return samples.powered
end

local function sample(name)
    return resolveActor(name)
end

local function getSamples()
    return {
        powered = samples.powered,
        passive = samples.passive,
        gait = samples.gait,
        defense = samples.defense,
    }
end

local function cleanup()
    if population then population.reset() end
    finishTrace('cleanup')
    pendingRecovery = nil
    removeActors(samples)
    removeActors(pendingSamples)
    samples = {}
    pendingSamples = {}
    setupStage = -1
end

local function state(actor)
    return ragdoll.getState(resolveActor(actor))
end

local function power(actor)
    ragdoll.setPowered(resolveActor(actor), true)
end

local function poweredPelvis(actor)
    actor = resolveActor(actor)
    local current = state(actor)
    if not current or not current.powered then
        error('Power the selected living actor before applying a physics impulse', 3)
    end
    for index, body in ipairs(current.bodies) do
        if body.name == 'pelvis' then
            return actor, current, index, body
        end
    end
    error('The selected ragdoll has no pelvis', 3)
end

local function fall(actor)
    local actor, current, bodyIndex, pelvis = poweredPelvis(actor)
    -- Define the perturbation as a mass-independent change in velocity. The
    -- 2 m/s shove is physically interpretable and scales with actor mass.
    local impulseMagnitude = current.totalMass * controlledFallVelocityChange
    ragdoll.applyImpulse(actor, bodyIndex, util.vector3(impulseMagnitude, 0, 0),
        pelvis.position + util.vector3(0, 0, 20))
    return impulseMagnitude
end

local pushDirections = {
    x = util.vector3(1, 0, 0), ['-x'] = util.vector3(-1, 0, 0),
    y = util.vector3(0, 1, 0), ['-y'] = util.vector3(0, -1, 0),
}

local function push(actor, direction)
    direction = direction or 'x'
    local axis = pushDirections[direction]
    if not axis then error('Push direction must be x, -x, y or -y (world axes)', 2) end
    local actor, current, bodyIndex, pelvis = poweredPelvis(actor)
    -- A known external perturbation: equivalent whole-body delta-v of 0.1 m/s,
    -- matching the existing 7.5 N s offline case for the 75 kg humanoid.
    -- Apply it at the pelvis COM without calling combat or crime mechanics.
    local impulseNewtonSeconds = current.totalMass * 0.1
    ragdoll.applyImpulse(actor, bodyIndex, axis * (impulseNewtonSeconds * openMwUnitsPerMeter), pelvis.centerOfMass)
    print(string.format('Manual physics push: %.6g N s along %s; equivalent whole-body delta-v 0.1 m/s',
        impulseNewtonSeconds, direction))
    return impulseNewtonSeconds
end

local function recover(actor)
    actor = resolveActor(actor)
    local current = ragdoll.getState(actor)
    if not current or not current.powered or not current.controller then
        error('The selected actor is not in powered ragdoll mode', 2)
    end
    if current.controller.mode == 'fallen' then
        ragdoll.requestRecovery(actor)
        print('Authored powered-ragdoll recovery requested')
        return 'requested'
    end
    if current.controller.mode == 'getting_up' or current.controller.mode == 'blend_back' then
        print('Authored powered-ragdoll recovery is already in progress')
        return 'already_recovering'
    end
    pendingRecovery = actor
    print('Recovery queued; it will start only after the controller reaches the fallen state')
    return 'queued'
end

local function disable(actor)
    ragdoll.setPowered(resolveActor(actor), false)
end

local function kill(actor)
    actor = resolveActor(actor)
    ragdoll.triggerDeath(actor, {
        hitPoint = actor.position + util.vector3(0, 0, 100),
        direction = util.vector3(1, 0, 0.15),
        damage = 100,
    })
end

local function resurrect(actor)
    ragdoll.resurrect(resolveActor(actor))
end

local function gait(enabled)
    local actor = resolveActor('gait')
    actor:sendEvent('setManualRagdollGait', enabled ~= false)
end

local function startTrace(sampleKey, duration, startGait)
    duration = duration or 10
    if type(duration) ~= 'number' or duration < 1 or duration > 60 then
        error('Manual ragdoll trace duration must be between 1 and 60 simulation seconds', 2)
    end
    if activeTrace or (population and population.status().running) then
        error('A manual ragdoll trace or population is already active', 2)
    end

    local actor = resolveActor(sampleKey)
    ragdoll.setPowered(actor, true)
    traceSequence = traceSequence + 1
    local now = core.getSimulationTime()
    activeTrace = {
        id = traceSequence,
        sample = sampleKey,
        actor = actor,
        duration = duration,
        startTime = now,
        lastPhysicsStep = -1,
        stopGait = startGait,
        frames = 0,
        updateSamples = 0,
        duplicatePhysicsSteps = 0,
        skippedPhysicsSteps = 0,
        maximumGameplayRootLag = 0,
        maximumRenderPelvisError = 0,
        maximumRenderPelvisJump = 0,
        stages = {},
        uprightStages = {},
        gaitTransitions = 0,
        gaitTimeoutFrames = 0,
        gaitFrames = 0,
        unsupportedFrames = 0,
        armingUnsupportedFrames = 0,
        upwardSaturationFrames = 0,
        armingUpwardSaturationFrames = 0,
        consecutiveUpwardSaturation = 0,
        maximumConsecutiveUpwardSaturation = 0,
        maximumAnchorSeparation = 0,
        maximumBodySpeed = 0,
        maximumRootSeparation = 0,
        minimumPelvisUp = math.huge,
        maximumTrackingError = 0,
        finalTrackingError = 0,
    }
    emitTraceRecord('begin', activeTrace, { duration = duration })
    return traceSequence
end

local function traceStanding(duration)
    local id = startTrace('powered', duration, false)
    print(string.format('Manual standing trace %d started immediately; activation and falls are included', id))
    return id
end

local function traceGait(duration)
    local id = startTrace('gait', duration, true)
    print(string.format(
        'Manual gait trace %d is waiting for stable support, then will record %.1f gait seconds',
        id, activeTrace.duration))
    return id
end

local function stopTrace()
    return finishTrace('requested')
end

local function traceStatus()
    if activeTrace then
        return {
            active = true,
            traceId = activeTrace.id,
            sample = activeTrace.sample,
            elapsed = math.max(0, core.getSimulationTime() - activeTrace.startTime),
            stage = activeTrace.stopGait and (activeTrace.gaitStartTime and 'gait' or 'arming') or 'standing',
            gaitElapsed = activeTrace.gaitStartTime
                and math.max(0, core.getSimulationTime() - activeTrace.gaitStartTime) or 0,
            duration = activeTrace.duration,
            frames = activeTrace.frames,
        }
    end
    if lastTraceSummary then
        local result = { active = false }
        for key, value in pairs(lastTraceSummary) do
            result[key] = value
        end
        return result
    end
    return { active = false }
end

local function publishDefenseStatus(text, announce)
    local player = world.players[1]
    if player then
        player:sendEvent('ragdollManualDefenseStatus', {
            text = text,
            announce = announce == true,
        })
    end
    if announce then print(text) end
end

local function defenseActors()
    local player = world.players[1]
    local defender = samples.defense
    if not player or not player:isValid() then error('The manual range has no player', 3) end
    if not defender or not defender:isValid() then
        error('Ragdoll sample "defense" is unavailable; reset the range and wait for READY', 3)
    end
    return player, defender
end

local function defenseFacing(player, defender)
    local facing = player.position - defender.position
    facing = util.vector3(facing.x, facing.y, 0)
    if facing:length() < 0.001 then return util.vector3(0, -1, 0) end
    return facing / facing:length()
end

local function defenseState(announce)
    local player, defender = defenseActors()
    local attacker = ragdoll.getState(player)
    local target = ragdoll.getState(defender)
    local weapon = attacker and attacker.weapon or nil
    local health = types.Actor.stats.dynamic.health(defender).current
    local result = {
        phase = defenseSession and defenseSession.phase or 'not_prepared',
        targetHealth = health,
        targetGuardActive = target and target.guardActive or false,
        targetGuardStartPhysicsStep = target and target.guardStartPhysicsStep or 0,
        outcomeCount = weapon and weapon.defenseOutcomeCount or 0,
        outcome = weapon and weapon.lastDefenseOutcome or 'none',
        disposition = weapon and weapon.lastContactDisposition or 'invalid',
        blocked = weapon and weapon.lastDamageBlocked or false,
        finalDamage = weapon and weapon.lastFinalDamage or 0,
        redirectImpulse = weapon and weapon.lastRedirectImpulse or 0,
        guardFacingAlignment = weapon and weapon.lastGuardFacingAlignment or -1,
        approachAlignment = weapon and weapon.lastApproachAlignment or 0,
    }
    if announce then
        local text = string.format(
            'Defense D: %s | outcome %s / %s | blocked %s | damage %.3f | redirect %.3f | health %.1f | guard %s',
            result.phase, result.outcome, result.disposition, tostring(result.blocked), result.finalDamage,
            result.redirectImpulse, result.targetHealth, tostring(result.targetGuardActive))
        publishDefenseStatus(text, true)
    end
    return result
end

local function raiseDefenseGuard(delay)
    local player, defender = defenseActors()
    if not defenseSession or defenseSession.phase ~= 'ready' then
        error('Run I.RagdollDev.defensePrepare() and wait for "Defense D: READY" first', 3)
    end
    delay = delay == nil and 0 or math.max(0, delay)
    defenseSession.guardAt = core.getSimulationTime() + delay
    ragdoll.setGuard(defender, false, defenseFacing(player, defender))
    local text = delay > 0
        and string.format('Defense D: guard scheduled in %.2f s; begin the swing now', delay)
        or 'Defense D: guard raised NOW; contact <=0.15 s parries, <=0.60 s blocks, later hits'
    publishDefenseStatus(text, true)
    return text
end

local function openDefenseGuard()
    local player, defender = defenseActors()
    if not defenseSession or defenseSession.phase ~= 'ready' then
        error('Run I.RagdollDev.defensePrepare() and wait for "Defense D: READY" first', 3)
    end
    if defenseSession then defenseSession.guardAt = nil end
    ragdoll.setGuard(defender, false, defenseFacing(player, defender))
    local text = 'Defense D: guard open; the next valid physical contact should hit'
    publishDefenseStatus(text, true)
    return text
end

local function startDefenseSwing()
    local player = defenseActors()
    if not defenseSession or defenseSession.phase ~= 'ready' then
        error('Run I.RagdollDev.defensePrepare() and wait for "Defense D: READY" first', 3)
    end
    if defenseSwing then return 'physical sweep already in progress' end
    defenseSwing = {
        player = player,
        startTime = core.getSimulationTime(),
    }
    publishDefenseStatus('Defense D: physical sweep started; contact alone decides whether it hits', true)
    return 'physical sweep started'
end

local function updateDefenseSwing()
    local swing = defenseSwing
    if not swing then return end
    if not swing.player:isValid() then
        defenseSwing = nil
        return
    end
    local elapsed = core.getSimulationTime() - swing.startTime
    local held = elapsed < 2.0
    local fixtureYaw
    if elapsed < 0.5 then
        local t = elapsed / 0.5
        local smooth = t * t * (3 - 2 * t)
        fixtureYaw = -0.7 * smooth
    elseif held then
        local t = math.min((elapsed - 0.5) / 1.5, 1)
        local smooth = t * t * (3 - 2 * t)
        fixtureYaw = -0.7 + 1.4 * smooth
    else
        fixtureYaw = 0
    end
    local playerYaw = swing.player.rotation:getYaw()
    local forward = util.vector3(math.sin(playerYaw), math.cos(playerYaw), 0)
    local right = util.vector3(math.cos(playerYaw), -math.sin(playerYaw), 0)
    ragdoll.driveSwing(swing.player, true, right, forward, util.vector3(0, 0, 1),
        0, 0, held, 1 / 60)
    ragdoll.setWeaponFixtureYaw(swing.player, fixtureYaw)
    if not held then
        defenseSwing = nil
        publishDefenseStatus('Defense D: physical sweep released; use the result counter to distinguish hit from miss', false)
    end
end

-- Prepare fists or the selected weapon, then let the player's script confirm
-- third-person camera, equipment and stance before powering the full body.
local function preparePlayerFists(weapon)
    local player = world.players[1]
    if not player or not player:isValid() then return 'no player' end
    ragdoll.setAutomaticRecovery(true)
    ragdoll.setPowered(player, false)
    giveTestWeapons(player)
    player:sendEvent('ragdollManualFistsConfigure', { weapon = weapon })
    local text = 'Preparing player ' .. (weapon or 'fists') .. ' in third-person...'
    publishDefenseStatus(text, true)
    return text
end

prepareDefense = function()
    if setupStage ~= 3 then error('Wait for RAGDOLL TEST RANGE READY before preparing defense D', 3) end
    local player, defender = defenseActors()
    ragdoll.setAutomaticRecovery(true)
    defenseSession = {
        phase = 'equipping',
        player = player,
        defender = defender,
        playerReady = false,
        targetReady = false,
        guardAt = nil,
        deadline = core.getRealTime() + 15,
        lastOutcomeCount = 0,
        lastHealth = types.Actor.stats.dynamic.health(defender).current,
    }
    player:sendEvent('ragdollManualDefenseConfigure')
    defender:sendEvent('ragdollManualDefenseTargetConfigure')
    publishDefenseStatus('Defense D: equipping warhammer and enabling both physical bodies...', true)
    return 'preparing; wait for the READY message'
end

local function resetDefense()
    local player = world.players[1]
    if player and player:isValid() then ragdoll.setPowered(player, false) end
    beginSetup(true)
    publishDefenseStatus('Defense D: rebuilding the flat-floor station...', true)
    return 'resetting; wait for the READY message'
end

updateDefense = function()
    updateDefenseSwing()
    local session = defenseSession
    if not session then return end
    if not session.player:isValid() or not session.defender:isValid() then
        defenseSession = nil
        return
    end
    if core.getRealTime() >= session.deadline and session.phase ~= 'ready' then
        publishDefenseStatus('Defense D: setup timed out; run defenseReset() and inspect the console log', true)
        session.phase = 'failed'
        return
    end
    if session.phase == 'equipping' and session.playerReady and session.targetReady then
        if ragdoll.prepareEquippedWeapon(session.player) then
            ragdoll.setPowered(session.player, true)
            ragdoll.setPowered(session.defender, true)
            session.phase = 'arming'
        end
    elseif session.phase == 'arming' then
        local attacker = ragdoll.getState(session.player)
        local target = ragdoll.getState(session.defender)
        if attacker and attacker.weapon and attacker.weapon.attached
            and attacker.controller and attacker.controller.mode == 'standing'
            and attacker.controller.pelvisHeightSettled
            and attacker.support and attacker.support.leftFoot and attacker.support.rightFoot
            and target and target.controller and target.controller.mode == 'standing'
            and target.controller.pelvisHeightSettled
            and target.support and target.support.leftFoot and target.support.rightFoot then
            session.phase = 'ready'
            session.lastOutcomeCount = attacker.weapon.defenseOutcomeCount
            session.lastHealth = types.Actor.stats.dynamic.health(session.defender).current
            session.deadline = math.huge
            publishDefenseStatus(
                'Defense D: READY. WASD walks; F6 makes a physical sweep; hold attack + mouse draws one. F7 guards.', true)
        end
    elseif session.phase == 'ready' then
        if session.guardAt and core.getSimulationTime() >= session.guardAt then
            session.guardAt = nil
            ragdoll.setGuard(session.defender, true, defenseFacing(session.player, session.defender))
            publishDefenseStatus(
                'Defense D: guard raised NOW; contact <=0.15 s parries, <=0.60 s blocks, later hits', true)
        end
        local attacker = ragdoll.getState(session.player)
        local weapon = attacker and attacker.weapon or nil
        if weapon and weapon.defenseOutcomeCount > session.lastOutcomeCount then
            session.lastOutcomeCount = weapon.defenseOutcomeCount
            session.lastHealth = types.Actor.stats.dynamic.health(session.defender).current
            defenseState(true)
        end
    end
end

local function help()
    print('=== MANUAL RAGDOLL CHECKLIST ===')
    print('A / powered: I.RagdollDev.power("powered")')
    print('A / trace:   I.RagdollDev.traceStanding(10) -- powers A and records activation/fall')
    print('A / push:    I.RagdollDev.push("powered", "x") -- 0.1 m/s; directions x, -x, y, -y')
    print('A / fall:    I.RagdollDev.fall("powered")')
    print('A / recover: I.RagdollDev.recover("powered")')
    print('              returns "requested", "queued", or "already_recovering"')
    print('B / death:   I.RagdollDev.kill("passive")')
    print('B / reset:   I.RagdollDev.resurrect("passive")')
    print('C / enable:  I.RagdollDev.power("gait")')
    print('C / walk:    I.RagdollDev.gait(true)')
    print('C / stop:    I.RagdollDev.gait(false)')
    print('C / trace:   I.RagdollDev.traceGait(10)')
    print('NEW / F8:    clickable manual ragdoll menu (no console needed)')
    print('NEW / grab:  G grabs the ragdoll body under the crosshair; I.RagdollDev.grabStrength(120000) to haul')
    print('NEW / hits:  I.RagdollDev.setMeleeImpulseScale(0.6) -- melee push multiplier, 1.0 default, damage unchanged')
    print('NEW / hits:  I.RagdollDev.setPlayerAlwaysHits(false) -- restore Morrowind to-hit dice (range skips them)')
    print('NEW / swing: I.RagdollDev.playerRagdoll(true) -- arm YOUR body, then hold attack and drag the mouse')
    print('NEW / swing: I.RagdollDev.playerRagdoll(false) -- hand your body back to animation')
    print('NEW / swing: I.RagdollDev.swingStrength(12) -- arm strength while swinging, 6 default')
    print('NEW / stuck: I.RagdollDev.standUp() -- get up, or bail out to animation if you cannot')
    print('D / prepare: I.RagdollDev.defensePrepare() -- equips player + powers the defense target')
    print('Fist/ punch: I.RagdollDev.playerFists() -- power YOUR body, fists up, mouse draws the punch')
    print('D / open:    I.RagdollDev.defenseOpen() -- next valid contact should be a hit')
    print('D / guard:   I.RagdollDev.defenseGuard(0) -- or press F7 just before weapon contact')
    print('D / timed:   I.RagdollDev.defenseGuard(2) -- two-second lead while you begin a swing')
    print('D / status:  view(I.RagdollDev.defenseStatus()) -- outcome, damage, redirect, target health')
    print('D / reset:   I.RagdollDev.defenseReset() -- fresh target, health, weapon, and counters')
    print('NEW / getup: I.RagdollDev.getUpTest("powered", true) -- knock down and rise WITH bracing arms')
    print('NEW / getup: I.RagdollDev.getUpTest("powered", false) -- same again WITHOUT them, to compare')
    print('NEW / arms:  I.RagdollDev.armTarget("powered", true, 100, 200, 40) -- reach one hand at x,y,z')
    print('NEW / arms:  I.RagdollDev.armTarget("powered", true) -- let it go again')
    print('NEW / arms:  I.RagdollDev.armReach("powered", true) -- how far short the hand fell, nil if not reaching')
    print('Trace status:view(I.RagdollDev.traceStatus())')
    print('Inspect:     view(I.RagdollDev.state("powered"))')
    print('Duel:        I.RagdollDev.duel() -- two nearest NPCs fight each other')
    print('Duel:        I.RagdollDev.autoDuel(true) -- restage the FIST vs DAGGER pair now')
    print('Duel:        I.RagdollDev.duelLoadout("fist"|"dagger"|"mixed")')
    print('Duel:        I.RagdollDev.duelHealth(40) -- normal pool; 12 for quick rounds')
    print('Hits:        I.RagdollDev.reactionScale(120) -- reaction impulse per damage point')
    print('Limb damage: view(I.RagdollDev.limbDamage()) -- damage per body region (red tint); I.RagdollDev.clearLimbDamage()')
    print('Range:       I.RagdollDev.stepToggle() -- k=1/k=4 when the env enables stepping')
    print('Reset range: I.RagdollDev.setup()')
end

-- Auto duel: while enabled the range keeps exactly one duel running. Two
-- fighters are spawned in front of the player, powered, told to let their AI
-- own movement (setManualRagdollDuel) and registered as each other's combat
-- targets. Every round pairs one unarmed brawler with one dagger fighter, so
-- the hand-strike contact path and the weapon contact path are both exercised.
-- When one dies both bodies are removed and a fresh pair is staged, so an
-- unattended range neither runs dry nor fills with corpses. The harness
-- samples are left alone: their traces stay usable.
local autoDuelEnabled = true
local autoDuelLast = -100
local autoDuelDiagnosticLast = -100
local autoDuelSpawned = {}
local autoDuelLoadouts = {}
local autoDuelWeaponRetry = {}
local autoDuelLoadoutMode = 'mixed'
local autoDuelFighterHealth = 40

local function spawnFighter(loadout)
    local player = world.players[1]
    if not player or not player:isValid() then return nil end
    ensureSampleRecords(player)
    local actor = world.createObject(sampleRecords.gait.id)
    -- The duelist script is the integration duelist's behavior: no AI toggling,
    -- no control writes, ordinary AI combat packages own the fight.
    actor:addScript('ragdoll_manual_duelist.lua', {
        unarmed = loadout ~= 'dagger',
        health = autoDuelFighterHealth,
    })
    if loadout == 'dagger' then
        local inventory = types.Actor.inventory(actor)
        if not inventory:find('iron dagger') then
            world.createObject('iron dagger', 1):moveInto(inventory)
        end
    end
    return actor
end

local function removeIfValid(actor)
    if actor and actor:isValid() and actor.count > 0 then
        actor:remove()
    end
end

local function fighterStatusText(actor)
    local state = ragdoll.getState(actor)
    local weapon = types.Actor.getEquipment(actor, types.Actor.EQUIPMENT_SLOT.CarriedRight)
    local ok, targets = pcall(ragdoll.getCombatTargets, actor)
    local targetCount = ok and targets and #targets or -1
    return string.format(
        '%s(%s) powered=%s targets=%d health=%.1f stance=%s weapon=%s ragdoll_contacts=%d weapon_contacts=%d',
        autoDuelLoadouts[actor.id] or 'unknown', tostring(actor.id),
        tostring(state and state.powered == true), targetCount,
        types.Actor.stats.dynamic.health(actor).current,
        tostring(types.Actor.getStance(actor)), weapon and weapon.recordId or 'none',
        state and state.otherRagdollContacts or -1,
        state and state.weapon and state.weapon.contacts or -1)
end

local function duelStatusText()
    local alive = {}
    for _, actor in ipairs(world.activeActors) do
        alive[actor.id] = actor
    end
    local parts = {}
    for _, id in ipairs(autoDuelSpawned) do
        local actor = alive[id]
        if actor and actor:isValid() then
            parts[#parts + 1] = fighterStatusText(actor)
        end
    end
    if #parts == 0 then
        return string.format('Duel: no fighters staged (loadout %s, health %d)',
            autoDuelLoadoutMode, autoDuelFighterHealth)
    end
    return 'Duel: ' .. table.concat(parts, ' | ')
end

local function publishDuelStatus(announce)
    local text = duelStatusText()
    local player = world.players[1]
    if player then
        player:sendEvent('ragdollManualDuelStatus', { text = text, announce = announce == true })
    end
    if announce then print(text) end
    return text
end

local function restageDuel(label)
    local alive = {}
    for _, actor in ipairs(world.activeActors) do
        alive[actor.id] = actor
    end
    for _, id in ipairs(autoDuelSpawned) do
        removeIfValid(alive[id])
    end
    autoDuelSpawned = {}
    autoDuelLoadouts = {}
    autoDuelWeaponRetry = {}
    autoDuelLast = -100
    local text = label and ('Duel restaged: ' .. label) or 'Duel restaged'
    print('RagdollDev.autoDuel: ' .. text)
    return text
end

local function stepToggle()
    local multiplier = ragdoll.testStepMultiplier()
    if multiplier <= 0 then
        return 'Test stepping not enabled; launch with OPENMW_RAGDOLL_TEST_STEP_MULTIPLIER=4'
    end
    testStepActive = not testStepActive
    ragdoll.setTestStepActive(testStepActive, 424242)
    return string.format('Test stepping k=%d %s', multiplier, testStepActive and 'on' or 'off')
end

updateAutoDuel = function()
    if not autoDuelEnabled then return end
    local now = core.getSimulationTime()
    if now - autoDuelLast < 1.0 then return end
    autoDuelLast = now
    local player = world.players[1]
    if not player or not player:isValid() then return end
    local alive = {}
    for _, actor in ipairs(world.activeActors) do
        alive[actor.id] = actor
    end
    local fighters = {}
    for _, id in ipairs(autoDuelSpawned) do
        local actor = alive[id]
        if actor and actor:isValid() and types.Actor.stats.dynamic.health(actor).current > 0 then
            fighters[#fighters + 1] = actor
        else
            removeIfValid(actor)
            autoDuelLoadouts[id] = nil
        end
    end
    if #fighters == 2 then
        -- Keep the round running. Powering waits until the loadout is applied
        -- and arms the prepared physical weapon, then combat targets only once
        -- both are powered.
        ragdoll.setAutomaticRecovery(true)
        local powered = true
        for _, fighter in ipairs(fighters) do
            local state = ragdoll.getState(fighter)
            local loadout = autoDuelLoadouts[fighter.id] or 'fist'
            local weapon = types.Actor.getEquipment(fighter, types.Actor.EQUIPMENT_SLOT.CarriedRight)
            local stanceReady = types.Actor.getStance(fighter) == types.Actor.STANCE.Weapon
            if not stanceReady then
                powered = false
            elseif state and state.powered then
                -- A dagger fighter powered before its equipment action landed
                -- has no physical weapon: rebuild the body once so the prepared
                -- weapon becomes the strike surface.
                if loadout == 'dagger' and weapon and not state.weapon
                    and not autoDuelWeaponRetry[fighter.id] then
                    autoDuelWeaponRetry[fighter.id] = true
                    ragdoll.setPowered(fighter, false)
                    ragdoll.prepareEquippedWeapon(fighter)
                    ragdoll.setPowered(fighter, true)
                    powered = false
                end
            else
                if loadout == 'dagger' then
                    ragdoll.prepareEquippedWeapon(fighter)
                end
                ragdoll.setPowered(fighter, true)
                powered = false
            end
        end
        if powered then
            ragdoll.setCombatTarget(fighters[1], fighters[2])
            ragdoll.setCombatTarget(fighters[2], fighters[1])
        end
        -- While a round runs, publish the duel status to the HUD and the log.
        if now - (autoDuelDiagnosticLast or -100) >= 5 then
            autoDuelDiagnosticLast = now
            local text = duelStatusText()
            print(text)
            player:sendEvent('ragdollManualDuelStatus', { text = text, announce = false })
        end
        return
    end
    -- A round ended (or has not started): clear the leftovers and stage the
    -- next pair in front of the player.
    for _, id in ipairs(autoDuelSpawned) do
        removeIfValid(alive[id])
    end
    autoDuelSpawned = {}
    autoDuelLoadouts = {}
    autoDuelWeaponRetry = {}
    local facing = player.rotation:getYaw()
    local forward = util.vector3(math.sin(facing), math.cos(facing), 0)
    local right = util.vector3(forward.y, -forward.x, 0)
    for index = 1, 2 do
        local loadout
        if autoDuelLoadoutMode == 'fist' then
            loadout = 'fist'
        elseif autoDuelLoadoutMode == 'dagger' then
            loadout = 'dagger'
        else
            loadout = index == 1 and 'fist' or 'dagger'
        end
        local actor = spawnFighter(loadout)
        if actor then
            local lateral = index == 1 and 55 or -55
            actor:teleport(player.cell, player.position + forward * 90 + right * lateral, { onGround = true })
            autoDuelSpawned[#autoDuelSpawned + 1] = actor.id
            autoDuelLoadouts[actor.id] = loadout
            print(string.format('RagdollDev.autoDuel: staged %s fighter %s',
                loadout == 'dagger' and 'DAGGER' or 'FIST', tostring(actor.id)))
        end
    end
end

-- Powers the two nearest living NPCs and makes them fight each other. The
-- ordinary combat target list is registered in both directions because the
-- melee hit test only considers an attacker's registered targets.
local function duelNearby()
    local player = world.players[1]
    local candidates = {}
    for _, actor in ipairs(world.activeActors) do
        if actor:isValid() and not types.Player.objectIsInstance(actor) then
            local health = types.Actor.stats.dynamic.health(actor)
            local distance = (actor.position - player.position):length()
            if health.current > 0 and distance <= 2000 then
                candidates[#candidates + 1] = { actor = actor, distance = distance }
            end
        end
    end
    table.sort(candidates, function(a, b) return a.distance < b.distance end)
    if #candidates < 2 then
        print('RagdollDev.duel: need two living NPCs within 2000 units')
        return false
    end
    local attacker, defender = candidates[1].actor, candidates[2].actor
    ragdoll.setPowered(attacker, true)
    ragdoll.setPowered(defender, true)
    attacker:sendEvent('setManualRagdollDuel', true)
    defender:sendEvent('setManualRagdollDuel', true)
    ragdoll.setCombatTarget(attacker, defender)
    ragdoll.setCombatTarget(defender, attacker)
    print(string.format('RagdollDev.duel: %s vs %s, %.0f units apart',
        attacker.recordId, defender.recordId, (attacker.position - defender.position):length()))
    return true
end

population = require('ragdoll_manual_population')(function(offset, first)
    if first then
        removeActors(samples)
        samples = {}
    end
    local actor = world.createObject(sampleRecords.gait.id)
    actor:teleport(testArea.cell, testArea.gait + util.vector3(offset, 0, 0), { onGround = true })
    actor:addScript('ragdoll_manual_actor.lua')
    return actor
end, emitTraceRecord)

return {
    engineHandlers = {
        onUpdate = updateSetup,
    },
    eventHandlers = {
        ragdollManualGrab = function(data)
            local ok, err = pcall(manualGrab, data)
            if not ok then
                print('Manual ragdoll grab: ' .. tostring(err))
            end
        end,
        ragdollManualControl = function(data)
            local ok, result = pcall(function()
                if data.action == 'reset' then
                    return setup()
                elseif data.action == 'original_trace' then
                    return traceGait(5)
                elseif data.action == 'defense_prepare' then
                    return prepareDefense()
                elseif data.action == 'player_fists' then
                    return preparePlayerFists()
                elseif data.action == 'player_dagger' then
                    return preparePlayerFists('iron dagger')
                elseif data.action == 'defense_open' then
                    return openDefenseGuard()
                elseif data.action == 'defense_guard_now' then
                    return raiseDefenseGuard(0)
                elseif data.action == 'defense_guard_2s' then
                    return raiseDefenseGuard(2)
                elseif data.action == 'defense_status' then
                    return defenseState(true)
                elseif data.action == 'defense_swing' then
                    return startDefenseSwing()
                elseif data.action == 'defense_reset' then
                    return resetDefense()
                elseif data.action == 'duel_restage' then
                    return restageDuel()
                elseif data.action == 'duel_loadout_fist' then
                    autoDuelLoadoutMode = 'fist'
                    return restageDuel('fists only')
                elseif data.action == 'duel_loadout_dagger' then
                    autoDuelLoadoutMode = 'dagger'
                    return restageDuel('daggers only')
                elseif data.action == 'duel_loadout_mixed' then
                    autoDuelLoadoutMode = 'mixed'
                    return restageDuel('mixed')
                elseif data.action == 'duel_health_normal' then
                    autoDuelFighterHealth = 40
                    return restageDuel('health 40')
                elseif data.action == 'duel_health_quick' then
                    autoDuelFighterHealth = 12
                    return restageDuel('health 12')
                elseif data.action == 'duel_auto_on' then
                    autoDuelEnabled = true
                    autoDuelLast = -100
                    return 'Auto-duel on'
                elseif data.action == 'duel_auto_off' then
                    autoDuelEnabled = false
                    return 'Auto-duel off'
                elseif data.action == 'duel_status' then
                    return publishDuelStatus(true)
                elseif data.action == 'player_release' then
                    local player = world.players[1]
                    if player and player:isValid() then
                        ragdoll.setPowered(player, false)
                    end
                    return 'Player body released to animation'
                elseif data.action == 'step_toggle' then
                    return stepToggle()
                elseif data.action == 'reaction_light' then
                    ragdoll.reactionScale(36)
                    return string.format('Stagger scale %.0f (light)', ragdoll.reactionScale())
                elseif data.action == 'reaction_default' then
                    ragdoll.reactionScale(72)
                    return string.format('Stagger scale %.0f (default)', ragdoll.reactionScale())
                elseif data.action == 'reaction_strong' then
                    ragdoll.reactionScale(144)
                    return string.format('Stagger scale %.0f (strong)', ragdoll.reactionScale())
                else
                    error('Unknown manual ragdoll action: ' .. tostring(data.action))
                end
            end)
            if not ok then
                print('Manual ragdoll control: ' .. tostring(result))
                local player = world.players[1]
                if player then
                    player:sendEvent('ragdollManualExperimentalStatus',
                        { text = tostring(result), announce = true })
                end
            end
        end,
        ragdollManualPlayerCombatReady = function(data)
            local player = world.players[1]
            if not player or not player:isValid() then return end
            ragdoll.setPowered(player, true)
            publishDefenseStatus(data.weapon and ('Player ' .. data.weapon
                .. ': click for an attack; hold and drag to steer. Release between attacks.')
                or 'Player fists: click for a jab; hold to alternate punches; release to stop.', true)
        end,
        ragdollManualDefensePlayerReady = function(data)
            if not defenseSession or defenseSession.phase ~= 'equipping' then return end
            if data.ready then
                defenseSession.playerReady = true
                publishDefenseStatus('Defense D: equipped ' .. tostring(data.weapon) .. '; attaching physics...', true)
            else
                defenseSession.phase = 'failed'
                publishDefenseStatus('Defense D: setup failed: ' .. tostring(data.reason), true)
            end
        end,
        ragdollManualDefenseTargetReady = function(data)
            if not defenseSession or defenseSession.phase ~= 'equipping' then return end
            if data.ready then
                defenseSession.targetReady = true
            else
                defenseSession.phase = 'failed'
                publishDefenseStatus('Defense D: target setup failed: ' .. tostring(data.reason), true)
            end
        end,
    },
    interfaceName = 'RagdollDev',
    interface = {
        version = 6,
        setup = setup,
        actor = actor,
        sample = sample,
        samples = getSamples,
        cleanup = cleanup,
        state = state,
        power = power,
        push = push,
        fall = fall,
        recover = recover,
        disable = disable,
        kill = kill,
        resurrect = resurrect,
        gait = gait,
        spawnWalkers = function(n, spacing)
            if setupStage ~= 3 or activeTrace then error('Wait for an idle manual range before spawning walkers') end
            return population.spawn(n, spacing)
        end,
        duel = duelNearby,
        autoDuel = function(enabled)
            autoDuelEnabled = enabled ~= false
            autoDuelLast = -100
            if not autoDuelEnabled then
                for _, actor in ipairs(world.activeActors) do
                    if actor:isValid() and not types.Player.objectIsInstance(actor) then
                        actor:sendEvent('setManualRagdollDuel', false)
                    end
                end
                for _, id in ipairs(autoDuelSpawned) do
                    for _, actor in ipairs(world.activeActors) do
                        if actor.id == id then
                            removeIfValid(actor)
                        end
                    end
                end
                autoDuelSpawned = {}
            end
            return autoDuelEnabled
        end,
        runPopulation = population.run,
        populationStatus = population.status,
        traceGait = traceGait,
        traceStanding = traceStanding,
        tracePlayer = function(duration)
            local id = startTrace('player', duration or 30, false)
            local current = ragdoll.getState(world.players[1])
            local marker = string.format('VIDEO_SYNC trace=%d real=%.6f sim=%.6f physics=%s',
                id, core.getRealTime(), core.getSimulationTime(), tostring(current and current.physicsStep))
            print(marker)
            return marker
        end,
        stopTrace = stopTrace,
        traceStatus = traceStatus,
        defensePrepare = prepareDefense,
        playerFists = preparePlayerFists,
        -- Duel loadout and health. Changing either restages the current pair so
        -- the next round is the requested matchup.
        duelLoadout = function(mode)
            if mode == 'fist' or mode == 'dagger' or mode == 'mixed' then
                autoDuelLoadoutMode = mode
                return restageDuel(mode)
            end
            return autoDuelLoadoutMode
        end,
        duelHealth = function(value)
            if value == nil then
                return autoDuelFighterHealth
            end
            autoDuelFighterHealth = math.max(1, value)
            return restageDuel('health ' .. tostring(autoDuelFighterHealth))
        end,
        duelStatus = function() return publishDuelStatus(true) end,
        playerRelease = function()
            local player = world.players[1]
            if player and player:isValid() then
                ragdoll.setPowered(player, false)
            end
            return 'released'
        end,
        stepToggle = stepToggle,
        -- Physical strike reaction strength: reaction impulse per point of
        -- pre-blunt damage. nil reads the current value.
        reactionScale = function(value)
            if value ~= nil then
                ragdoll.reactionScale(value)
            end
            return ragdoll.reactionScale()
        end,
        defenseOpen = openDefenseGuard,
        defenseGuard = raiseDefenseGuard,
        defenseStatus = function() return defenseState(false) end,
        defenseSwing = startDefenseSwing,
        defenseReset = resetDefense,
        help = help,
        -- Grab strength for the G key in force units (default 40000, about
        -- half the rig's weight); larger hauls the body, smaller only tugs.
        grabStrength = function(value)
            grabStrength = value
            return grabStrength
        end,
        -- Melee push strength multiplier (default 1.0). Scales the impulse
        -- only, never damage, so the hit bands can be tuned by feel.
        setMeleeImpulseScale = function(value)
            return ragdoll.setMeleeImpulseScale(value)
        end,
        getMeleeImpulseScale = function()
            return ragdoll.getMeleeImpulseScale()
        end,
        -- Skip Morrowind's to-hit roll for the player so swings connect on
        -- geometry. The range enables this during setup.
        setPlayerAlwaysHits = function(enabled)
            return ragdoll.setPlayerAlwaysHits(enabled)
        end,
        -- Reach one hand at a world point, or release it by passing no
        -- point. Weight and blend rate are optional: full weight, and in
        -- place in about a sixth of a second. The point may be out of reach,
        -- which is how a brace is aimed at a floor it has not found yet.
        armTarget = function(actor, rightArm, position, weight, blendRate, z)
            local current = resolveActor(actor)
            if not current or not current:isValid() then
                return 'missing'
            end
            -- Three plain numbers work as well as a vector, because reaching
            -- for util.vector3 in the console is what stopped this being
            -- driven by hand at all.
            if type(position) == 'number' then
                position, weight, blendRate = util.vector3(position, weight, blendRate), z, nil
            end
            if position == nil then
                ragdoll.releaseArmTarget(current, rightArm and true or false)
                return 'released'
            end
            ragdoll.setArmTarget(
                current, rightArm and true or false, position, weight or 1.0, blendRate or 6.0)
            return 'reaching'
        end,
        -- Distance between the last requested point and the point the hand
        -- actually reached, or nil when that arm is not reaching.
        armReach = function(actor, rightArm)
            local current = resolveActor(actor)
            if not current or not current:isValid() then
                return nil
            end
            return ragdoll.getArmReach(current, rightArm and true or false)
        end,
        -- Plant the hands while getting up, or do not. Switchable live so a
        -- get-up can be watched both ways in one session; comparing across
        -- separate runs of a physical system is not a comparison.
        bracingArms = function(enabled)
            if enabled == nil then
                return ragdoll.getBracingArms()
            end
            return ragdoll.setBracingArms(enabled and true or false)
        end,
        -- Knock the actor down and let it get back up, with the brace in
        -- whichever state is asked for. Run it twice, once each way.
        -- Arm the PLAYER's own body as a powered ragdoll. Holding attack
        -- then drives the weapon hand with the mouse instead of the camera
        -- (plan item 2.3d). Third person only: first person still renders
        -- Morrowind's separate .1st rig, so the arms it shows are not the
        -- arms that move (that is plan item 2.3e, not done).
        playerRagdoll = function(enabled)
            local player = world.players[1]
            if not player or not player:isValid() then
                return 'no player'
            end
            local arming = enabled == nil or (enabled and true or false)
            if arming then
                -- The range switches automatic recovery off so a bounded
                -- trace can watch a body stay down. That is a diagnostic
                -- choice for the sample actors; applied to the PLAYER it
                -- means falling over ends the session, because nothing will
                -- ever ask the body to get back up.
                ragdoll.setAutomaticRecovery(true)
            end
            ragdoll.setPowered(player, arming)
            if not arming then
                return 'released'
            end
            return 'player armed (auto get-up ON); hold attack and drag the mouse. playerRagdoll(false) to release'
        end,
        -- How hard the arm swings, as a multiple of the authored motors
        -- (default 6). The authored values hold a pose; they do not drive a
        -- weapon, so this is the knob for "the arm is too weak".
        swingStrength = function(scale)
            if scale == nil then
                return ragdoll.getSwingStrength()
            end
            return ragdoll.setSwingStrength(scale)
        end,
        -- Localized limb damage: the damage each body region of an actor has
        -- taken, which is what the red body-part tint shows. With no argument
        -- it reads the player; clearLimbDamage() wipes every actor's tint.
        limbDamage = function(actor)
            local target = actor or world.players[1]
            if not target or not target:isValid() then
                return 'no actor'
            end
            return ragdoll.getLimbDamage(target)
        end,
        clearLimbDamage = function()
            ragdoll.clearLimbDamage()
            return 'limb damage tint cleared'
        end,
        -- Get up, or failing that hand the body straight back to animation.
        -- Being stuck on the floor with no way out is not a state a test
        -- environment should be able to put you in.
        standUp = function()
            local player = world.players[1]
            if not player or not player:isValid() then
                return 'no player'
            end
            ragdoll.setAutomaticRecovery(true)
            local state = ragdoll.getState(player)
            if state and state.controller and state.controller.mode == 'fallen' then
                ragdoll.requestRecovery(player)
                return 'recovery requested'
            end
            ragdoll.setPowered(player, false)
            return 'handed the body back to animation'
        end,
        -- Force which get-up plays: true face up, false face down, nil for
        -- the measured choice. A fallen body rests on its side, where that
        -- measurement sits on its own boundary, so the two are worth watching
        -- against each other before the choice is changed.
        recoveryDirection = function(faceUp)
            ragdoll.setRecoveryDirection(faceUp)
            return faceUp == nil and 'measured' or (faceUp and 'face_up' or 'face_down')
        end,
        getUpTest = function(actor, bracing, duration)
            local current = resolveActor(actor)
            if not current or not current:isValid() then
                return 'missing'
            end
            -- Everything that can fail happens before the trace starts. The
            -- first version started recording and then threw on an unpowered
            -- actor, which left a trace running and blocked every later call.
            local state = ragdoll.getState(current)
            if not state or not state.powered then
                power(current)
                return 'powered it; wait for it to stand, then run getUpTest again'
            end
            if not state.controller or state.controller.mode ~= 'standing' then
                return 'not standing yet (' .. tostring(state.controller and state.controller.mode)
                    .. '); wait, then run getUpTest again'
            end
            -- A previous run may still be recording; take it over rather than
            -- refusing, since the point is to fire this repeatedly.
            if activeTrace then
                stopTrace()
            end
            ragdoll.setBracingArms(bracing == nil or bracing and true or false)
            ragdoll.setAutomaticRecovery(true)
            -- Record it. Two get-ups watched by eye are an impression; two
            -- traces are a comparison, and the trace carries the recovery
            -- direction that decides which animation was even chosen.
            local id = traceStanding(duration or 8)
            fall(current)
            return ('trace %d: knocked down with bracing arms %s')
                :format(id, ragdoll.getBracingArms() and 'ON' or 'OFF')
        end,
        -- Whether a fallen actor gets up unaided (on in gameplay). The
        -- diagnostics below switch it off so a fallen body can be observed
        -- for a fixed window.
        setAutomaticRecovery = function(enabled)
            return ragdoll.setAutomaticRecovery(enabled)
        end,
    },
}

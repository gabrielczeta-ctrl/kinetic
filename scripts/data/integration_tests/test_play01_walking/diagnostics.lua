local core = require('openmw.core')
local util = require('openmw.util')
local M = {}
local recordId, emittedBytes = 0, 0
local function quote(value)
    return '"' .. value:gsub('[%z\1-\31\\"]', function(c)
        return string.format('\\u%04x', string.byte(c))
    end) .. '"'
end
local function encode(value)
    local kind = type(value)
    if kind == 'nil' then return 'null' end
    if kind == 'boolean' then return tostring(value) end
    if kind == 'number' then
        if value ~= value or math.abs(value) == math.huge then return quote('nonfinite') end
        return string.format('%.17g', value)
    end
    if kind == 'string' then return quote(value) end
    if kind == 'userdata' then
        local ok, x, y, z = pcall(function() return value.x, value.y, value.z end)
        if ok and type(x) == 'number' then
            return '[' .. encode(x) .. ',' .. encode(y) .. ',' .. encode(z) .. ']'
        end
        return encode({ x = value * util.vector3(1, 0, 0),
            y = value * util.vector3(0, 1, 0), z = value * util.vector3(0, 0, 1) })
    end
    if kind == 'table' then
        local keys, fields = {}, {}
        for key in pairs(value) do keys[#keys + 1] = key end
        table.sort(keys, function(a, b) return tostring(a) < tostring(b) end)
        for _, key in ipairs(keys) do fields[#fields + 1] = quote(tostring(key)) .. ':' .. encode(value[key]) end
        return '{' .. table.concat(fields, ',') .. '}'
    end
    error('Unsupported PLAY-01 diagnostic type ' .. kind)
end

function M.begin() recordId, emittedBytes = 0, 0 end
function M.write(row)
    row.schema_version, row.tracker_task = 1, 'PLAY-01'
    row.simulation_time, row.real_time = core.getSimulationTime(), core.getRealTime()
    local payload = encode(row)
    emittedBytes = emittedBytes + #payload
    -- 20 MiB was enough for the flat 13 s cases. The staircase cases walk a
    -- long diagonal approach and a full up/back flight, so the budget covers
    -- them without changing what is recorded.
    -- 80 MiB since the thigh, calf and hand bodies were added (2026-09-25).
    if emittedBytes > 200 * 1024 * 1024 then error('PLAY-01 per-case diagnostic byte budget exhausted') end
    recordId = recordId + 1
    local count = math.ceil(#payload / 2000)
    for part = 1, count do
        print(string.format('RAGDOLL_DIAGNOSTIC_CHUNK\t%d\t%d\t%d\t%s',
            recordId, part, count, payload:sub((part - 1) * 2000 + 1, part * 2000)))
    end
end

-- Every body (the behaviour viewer, tools/viewer, draws the whole
-- skeleton): thighs, calves and hands also give hip and knee target vs
-- actual at the end of swing and the carried hand heights (walk carry).
local bodyNames = { pelvis = true, spine = true, spine1 = true, spine2 = true, neck = true,
    left_foot = true, right_foot = true, left_thigh = true, right_thigh = true,
    left_calf = true, right_calf = true, left_hand = true, right_hand = true,
    head = true, left_forearm = true, right_forearm = true, left_clavicle = true, right_clavicle = true,
    left_upper_arm = true, right_upper_arm = true }
local controllerFields = {
    'mode', 'timeInMode', 'fixedSteps', 'pelvisUpDot', 'torsoUpDot', 'pelvisHeight',
    'pelvisHeightSettled', 'centerOfMassSpeed', 'velocityTrackingError',
    'centerOfMassVelocity', 'targetHorizontalVelocity', 'rawTargetHorizontalVelocity',
    'plannedHorizontalVelocity', 'headingError', 'gaitPhase', 'gaitTimeInPhase',
    'gaitPhaseProgress', 'gaitActuationStage', 'gaitSwingFootLifted',
    'gaitSwingTouchdownCandidate', 'gaitTransitioned', 'gaitPhaseTimedOut',
    'gaitSwingTimeSinceLift', 'gaitSwingContactCandidateTime', 'swingFootTarget',
    'jointSupportApplied', 'jointSupportContact', 'jointSupportMotorUtilization',
    'jointMotorPeakUtilization', 'targetPelvisHeight',
    'appliedPelvisForce', 'appliedVerticalForceSaturated', 'appliedSupportMomentSaturated',
    'animationAssistedActuation', 'locomotionInterrupted', 'externalPelvisAssistApplied',
    'coherentReferenceAvailable', 'coherentReferencePhase', 'coherentStance',
    'coherentHandoffWeight', 'coherentTranslationAllowed', 'coherentAnchorBody',
    'contactRootReferenceAvailable', 'contactRootSampleIndex',
    'contactRootEnvelopeWeight', 'contactRootLocomotionWeight',
    'contactRootLoadTransferWeight', 'contactRootConfirmationTicks',
    'contactRootActivationConfirmed', 'contactRootAuthorityOwner',
    'contactRootProductionReferenceActive', 'contactRootHandoffWeight',
    'contactRootProductionLoadWeight', 'contactRootCandidateLoadWeight',
    'contactRootDualActuation', 'contactRootOffset', 'contactRootDisplacement',
    'contactRootVelocity', 'contactRootStanceChanged', 'contactRootSupportReacquired',
    'contactRootNominalProgressSpeed', 'contactRootMeasuredProgressSpeed',
    'contactRootGovernedProgressSpeed', 'contactRootSelectedSlipSpeed',
    'contactRootSupportReferenceError', 'contactRootFeasibilityHoldTime',
    'contactRootTransitionConfirmationTicks', 'contactRootNextStance',
    'contactRootSupportFeasible', 'contactRootProgressHeld',
    'contactRootTouchdownPending', 'contactRootLeftFootLoadFraction',
    'contactRootRightFootLoadFraction', 'contactRootBilateralLoadWeight',
    'contactRootUnilateralLoadWeight', 'contactRootLoadTargetStance',
    'contactRootIncomingLoadConfirmed', 'contactRootOutgoingFootUnloaded',
    'contactRootLoadTransitionPending', 'contactRootLoadTransitionAge',
    'contactRootIncomingAnchorAvailable', 'contactRootIncomingAnchorHorizontalError',
    'contactRootIncomingAnchorReachable', 'contactRootCommitConfirmationTicks',
    'contactRootIncomingAnchorQualified',
    'contactRootReentrySlipSettledTime', 'contactRootReentryEligible',
    'contactRootFallbackReason',
    'touchdownCaptureActive', 'touchdownCaptureBody',
    'touchdownCaptureAnchorPosition', 'touchdownCaptureMeasuredContactPosition',
    'touchdownCaptureMeasuredFootOrigin', 'touchdownCaptureTargetFootOrigin',
    'touchdownCaptureTargetToMeasuredDisplacement',
    'touchdownCaptureTangentialSpeed', 'touchdownCaptureAnchorError',
    'touchdownCaptureVelocityCorrection', 'touchdownCaptureRequestedCorrection',
    'touchdownCaptureAppliedCorrection', 'touchdownCaptureSaturated',
    'touchdownCaptureIkApplied', 'touchdownCaptureAge',
    'outgoingHoldActive', 'outgoingHoldBody', 'outgoingHoldAnchorPosition',
    'outgoingHoldMeasuredContactPosition', 'outgoingHoldMeasuredFootOrigin',
    'outgoingHoldTargetFootOrigin', 'outgoingHoldTargetToMeasuredDisplacement',
    'outgoingHoldTangentialSpeed', 'outgoingHoldAnchorError',
    'outgoingHoldVelocityCorrection', 'outgoingHoldRequestedCorrection',
    'outgoingHoldAppliedCorrection', 'outgoingHoldSaturated',
    'outgoingHoldIkApplied', 'outgoingHoldAge', 'outgoingHoldPhase',
    'coherentMeasuredContactPosition', 'coherentDesiredContactPosition',
    'coherentContactError', 'coherentEndpointCorrection',
    'coherentEndpointCorrectionSaturated', 'coherentPositionIkApplied',
    'coherentLoadRoutingApplied', 'coherentSupportReferencePosition',
    'coherentSupportReferenceVelocity', 'coherentSupportReferenceError',
    'coherentSupportRequestedAcceleration', 'coherentSupportReferenceLeashed',
    'coherentSupportPlannedContactForce', 'coherentSupportPlannedContactMoment',
    'coherentRequestedLoadTorques', 'coherentAppliedMotorTorques',
    'coherentMotorUtilizations',
    'externalPelvisAssistSaturated', 'requestedPelvisAssistForce',
    'requestedPelvisAssistTiltTorque', 'requestedPelvisAssistYawTorque',
    'appliedPelvisAssistForce', 'appliedPelvisAssistTorque',
    'externalPelvisAssistTorqueSaturated',
    'assistedHeadingAvailable', 'physicalYaw', 'desiredYaw', 'referenceYaw',
    'referenceYawRate', 'referenceYawAcceleration', 'assistedHeadingError', 'physicalYawRate',
    'uprightActuationStage', 'recoveryRequested', 'requestHeadingTravel', 'commandHeadingTravel',
    'swingFootForce', 'swingFootForceSaturated', 'swingFootControlApplied', 'preLiftSwingUnloadForce',
    'swingHipMotorUtilization', 'swingKneeMotorUtilization', 'swingAnkleMotorUtilization',
    'swingHipMotorError', 'swingKneeMotorError', 'swingAnkleMotorError', 'upright',
}
function M.state(state)
    if not state then return {} end
    local selected = { powered = state.powered, centerOfMass = state.centerOfMass,
        support = state.support, experimentalWalking = state.experimentalWalking, softRoot = state.softRoot,
        contacts = state.contacts, stepQuality = state.stepQuality, bodies = {}, controller = {},
        body_names = {}, weapon = state.weapon }
    for index, body in ipairs(state.bodies) do
        selected.body_names[index] = body.name
        if bodyNames[body.name] then selected.bodies[body.name] = body end
    end
    for _, key in ipairs(controllerFields) do selected.controller[key] = state.controller and state.controller[key] end
    return selected
end
return M

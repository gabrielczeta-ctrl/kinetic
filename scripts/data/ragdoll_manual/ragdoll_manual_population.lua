-- NATIVE-02: independent recorders, one simultaneous movement cohort.
local core = require('openmw.core')
local ragdoll = require('openmw.ragdolltest')
local util = require('openmw.util')

return function(createWalker, emit)
    local walkers = {}
    local batch
    local sequence = 1000000
    local M = {}

    local function finish(reason)
        if not batch or batch.finished then return end
        batch.finished = true
        batch.reason = reason
        batch.falls = 0
        batch.setupFailures = 0
        batch.completed = 0
        batch.elapsed = batch.startTime and core.getSimulationTime() - batch.startTime or 0
        for _, walker in ipairs(walkers) do
            walker.summary = { index = walker.index, seed = walker.seed, frames = 0,
                first_falling_frame = -1, replay_path = '', replay_fixed_dt = 0,
                profile_hash = '', setup_failure = not walker.recording, completed = false }
            local row = walker.summary
            if walker.recording then
                local ok, replay = pcall(ragdoll.finishReplayRecording, walker.actor)
                walker.recording = false
                if ok and replay and replay.outputPath then
                    row.frames = replay.frameCount
                    row.first_falling_frame = replay.firstFallingFrame
                    row.replay_path = replay.outputPath
                    row.replay_fixed_dt = replay.fixedTimeStep
                    row.profile_hash = replay.profileHash
                    row.completed = reason == 'duration_complete' and replay.frameCount >= math.floor(batch.duration * 60)
                else
                    row.setup_failure = true
                    batch.invalidReason = batch.invalidReason or 'missing per-walker replay'
                end
            end
            if walker.actor:isValid() then
                walker.actor:sendEvent('setManualRagdollGait', false)
            end
            if row.first_falling_frame >= 0 then batch.falls = batch.falls + 1 end
            if row.setup_failure then batch.setupFailures = batch.setupFailures + 1 end
            if row.completed then batch.completed = batch.completed + 1 end
            emit('population_walker_summary', walker, row)
        end
        batch.fallRate = batch.falls / #walkers
        table.sort(batch.frameTimes)
        batch.frameP95 = #batch.frameTimes > 0 and batch.frameTimes[math.ceil(#batch.frameTimes * 0.95)] or math.huge
        emit('population_summary', batch, {
            actors_requested = #walkers, actors_completed = batch.completed,
            setup_failures = batch.setupFailures, falls = batch.falls, fall_rate = batch.fallRate,
            frame_p95_ms = batch.frameP95 * 1000, frame_samples = #batch.frameTimes,
            elapsed = batch.elapsed, duration = batch.duration, seed = batch.seed,
            spacing = batch.spacing, minimum_separation = batch.minimumSeparation,
            minimum_separation_widths = batch.minimumSeparationWidths,
            other_ragdoll_contacts = batch.otherContacts, reason = reason,
            invalid_reason = batch.invalidReason or '', clock = 'real', warmup_seconds = 0.5,
        })
    end

    function M.reset()
        finish('range_reset')
        for _, walker in ipairs(walkers) do
            if walker.actor:isValid() then walker.actor:remove() end
        end
        walkers = {}
        batch = nil
    end

    function M.spawn(n, spacing)
        if type(n) ~= 'number' or n % 1 ~= 0 or n < 1 or n > 8 then error('Walker count must be an integer from 1 to 8') end
        spacing = spacing or 384
        if type(spacing) ~= 'number' or spacing ~= spacing or spacing < 128 or spacing > 768 then
            error('Walker spacing must be between 128 and 768 world units')
        end
        M.reset()
        for index = 1, n do
            sequence = sequence + 1
            walkers[index] = { actor = createWalker((index - 1 - (n - 1) / 2) * spacing, index == 1),
                index = index, id = sequence, sample = 'population-' .. index,
                spacing = spacing, createdTime = core.getSimulationTime() }
        end
        local result = {}
        for _, walker in ipairs(walkers) do result[#result + 1] = walker.actor end
        return result
    end

    function M.run(segments, duration, seed)
        if #walkers == 0 then error('Call spawnWalkers before runPopulation') end
        if batch and not batch.finished then error('A population is already running') end
        if type(duration) ~= 'number' or duration ~= duration or duration < 1 or duration > 10 then error('Duration must be 1 to 10 movement seconds') end
        if type(seed) ~= 'number' or seed % 1 ~= 0 or seed < 0 or seed + #walkers >= 4294967296 then error('Population seed must fit uint32') end
        if type(segments) ~= 'table' or #segments == 0 then error('Population needs path segments') end
        sequence = sequence + 1
        batch = { id = sequence, sample = 'population', seed = seed, duration = duration,
            segments = segments, phase = 'spawn', requestedTime = core.getSimulationTime(),
            frameTimes = {}, spacing = walkers[1].spacing, minimumSeparation = 1e9,
            minimumSeparationWidths = 1e9, otherContacts = 0 }
        for _, walker in ipairs(walkers) do walker.seed = seed + walker.index end
        return batch.id
    end

    function M.status()
        return { running = batch ~= nil and not batch.finished, report = batch and batch.finished and batch or nil, walkers = walkers }
    end

    function M.update()
        if not batch or batch.finished then return end
        local now, real = core.getSimulationTime(), core.getRealTime()
        if now - batch.requestedTime > batch.duration + 10 then
            batch.invalidReason = 'population arming timeout'
            finish('arming_timeout')
            return
        end
        if batch.phase == 'spawn' then
            if now - batch.requestedTime < 5 / 60 then return end
            for _, walker in ipairs(walkers) do
                if not walker.actor:isValid() then
                    batch.invalidReason = 'actor invalid during setup'
                    finish('actor_invalid')
                    return
                end
                ragdoll.setPowered(walker.actor, true)
                walker.actor:sendEvent('setManualRagdollGait', false)
            end
            batch.phase = 'arming'
            return
        end
        local states, ready, observable = {}, true, true
        for index, walker in ipairs(walkers) do
            local state = walker.actor:isValid() and ragdoll.getState(walker.actor) or nil
            states[index] = state
            observable = observable and state and state.controller
            ready = ready and state and state.controller
        end
        if batch.phase == 'arming' then
            if not ready then return end
            for index, walker in ipairs(walkers) do
                local state = states[index]
                walker.bodyWidth = state.horizontalBodyWidth
                walker.contactBaseline = state.otherRagdollContacts
                if not walker.bodyWidth or walker.bodyWidth <= 0 or walker.contactBaseline == nil then
                    batch.invalidReason = 'missing physical spacing/contact telemetry'
                    finish('invalid_telemetry')
                    return
                end
                walker.actor:sendEvent('setManualRagdollGait', true)
                walker.actor:sendEvent('setManualRagdollPath', batch.segments)
                local ok, errorText = pcall(ragdoll.startReplayRecording, walker.actor, walker.seed,
                    math.floor(batch.duration * 60 + 120), true)
                if not ok then
                    batch.invalidReason = 'recording failed: ' .. tostring(errorText)
                    finish('recording_failed')
                    return
                end
                walker.recording = true
                walker.firstPhysicsStep = state.physicsStep
                emit('population_walker_begin', walker, { index = index, seed = walker.seed,
                    duration = batch.duration, body_width = walker.bodyWidth,
                    x = walker.actor.position.x, y = walker.actor.position.y })
            end
            batch.phase = 'running'
            batch.startTime = now
            batch.previousReal = real
            return
        end
        if not observable then
            batch.invalidReason = 'actor or controller disappeared'
            finish('actor_invalid')
            return
        end
        local sampleDt = real - batch.previousReal
        batch.previousReal = real
        if now - batch.startTime >= 0.5 and sampleDt > 0 then batch.frameTimes[#batch.frameTimes + 1] = sampleDt end
        emit('population_presentation', batch, { sample_dt = sampleDt, elapsed = now - batch.startTime,
            clock = 'real', physics_step = states[1].physicsStep })
        for index, walker in ipairs(walkers) do
            local state = states[index]
            if math.abs(state.referencePosition.x) > 3896 or math.abs(state.referencePosition.y) > 3896 then
                batch.invalidReason = 'walker left the clear floor bounds'
                finish('floor_bounds')
                return
            end
            batch.otherContacts = math.max(batch.otherContacts, state.otherRagdollContacts - walker.contactBaseline)
            emit('population_walker_frame', walker, { index = index, seed = walker.seed,
                physics_step = state.physicsStep, mode = state.controller.mode,
                elapsed = now - batch.startTime, x = state.referencePosition.x, y = state.referencePosition.y })
            for otherIndex = 1, index - 1 do
                local delta = state.referencePosition - states[otherIndex].referencePosition
                local separation = util.vector3(delta.x, delta.y, 0):length()
                batch.minimumSeparation = math.min(batch.minimumSeparation, separation)
                batch.minimumSeparationWidths = math.min(batch.minimumSeparationWidths,
                    separation / math.max(walker.bodyWidth, walkers[otherIndex].bodyWidth))
            end
        end
        if batch.otherContacts > 0 or batch.minimumSeparationWidths < 2 then
            batch.invalidReason = 'walkers interacted or separation below two body widths'
            finish('interaction')
        elseif now - batch.startTime >= batch.duration then
            finish('duration_complete')
        end
    end

    return M
end

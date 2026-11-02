-- Bounded observations of the real player adapter. These are render-observed
-- physics snapshots: skipped physics serials are reported, never interpolated.
local core = require('openmw.core')
local util = require('openmw.util')
local ragdoll = require('openmw.ragdolltest')
local world = require('openmw.world')
local M = {}
local active, samples, first, last, bytes, emitted, previousStep, input, replayStarted, replayCompleted, observing
local MAX_BYTES, MAX_EMITTED = 8 * 1024 * 1024, 16 * 1024 * 1024
local recordId = 0
local replaySeed = 70100

local function writeRecord(record)
    -- The engine log transport splits long lines near 4 KiB. Explicit
    -- framing keeps every chunk below that limit and allows loss detection.
    recordId = recordId + 1
    local count = math.ceil(#record / 2000)
    for part = 1, count do
        print(string.format('RAGDOLL_DIAGNOSTIC_CHUNK\t%d\t%d\t%d\t%s',
            recordId, part, count, record:sub((part - 1) * 2000 + 1, part * 2000)))
    end
end

local function quote(s)
    return '"' .. s:gsub('[%z\1-\31\\"]', function(c)
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
        if ok and type(x) == 'number' then return '[' .. encode(x) .. ',' .. encode(y) .. ',' .. encode(z) .. ']' end
        local rotated, axes = pcall(function()
            return { x = value * util.vector3(1, 0, 0), y = value * util.vector3(0, 1, 0), z = value * util.vector3(0, 0, 1) }
        end)
        return rotated and encode(axes) or quote('unsupported userdata')
    end
    if kind == 'table' then
        local parts, keys = {}, {}
        for key in pairs(value) do keys[#keys + 1] = key end
        table.sort(keys, function(a, b) return tostring(a) < tostring(b) end)
        for _, key in ipairs(keys) do parts[#parts + 1] = quote(tostring(key)) .. ':' .. encode(value[key]) end
        return '{' .. table.concat(parts, ',') .. '}'
    end
    return quote('unsupported ' .. kind)
end

function M.flush(reason)
    if not active then return end
    writeRecord(encode({ kind = 'window', test = active,
        reason = reason, buffered_bytes = bytes, emitted_bytes = emitted,
        buffered_samples = last - first + 1,
        real_time = core.getRealTime(), simulation_time = core.getSimulationTime() }))
    for index = first, last do
        local sample = samples[index]
        if sample and emitted + #sample <= MAX_EMITTED then
            writeRecord(sample)
            emitted = emitted + #sample
        end
    end
    samples, first, last, bytes = {}, 1, 0, 0
end

function M.begin(name)
    -- The test fixture reloads Lua between cases, so a module-local counter
    -- would reset and overwrite every sibling replay with the same seed.
    replaySeed = 70100
    for index = 1, #name do replaySeed = (replaySeed * 131 + name:byte(index)) % 2147483647 end
    active, samples, first, last, bytes, emitted, previousStep, input = name, {}, 1, 0, 0, 0, nil, nil
    replayStarted = false
    replayCompleted = false
    observing = true
    world.players[1]:sendEvent('ragdollDiagnosticInput', { enabled = true })
    writeRecord(encode({ kind = 'anchor', test = name,
        real_time = core.getRealTime(), simulation_time = core.getSimulationTime(),
        sampling = 'render-observed physics steps', maximum_buffer_bytes = MAX_BYTES,
        maximum_emitted_bytes = MAX_EMITTED }))
end

function M.input(command) input = command end

function M.update()
    if not active or not observing then return end
    local player = world.players[1]
    local state = ragdoll.getState(player)
    if state and not replayStarted and not replayCompleted then
        ragdoll.startReplayRecording(player, replaySeed, 1800)
        replayStarted = true
    end
    local step = state and state.physicsStep
    if step ~= nil and step == previousStep then return end
    if not state and not previousStep then return end
    local sample = encode({ kind = 'frame', test = active, real_time = core.getRealTime(),
        simulation_time = core.getSimulationTime(), physics_step = step,
        skipped_steps = step and previousStep and math.max(0, step - previousStep - 1) or 0,
        player_position = player.position, gameplay_yaw = player.rotation:getYaw(),
        input = input, rig_present = state ~= nil, state = state })
    previousStep = step
    last = last + 1
    samples[last], bytes = sample, bytes + #sample
    while first <= last and (last - first + 1 > 360 or bytes > MAX_BYTES) do
        bytes = bytes - #samples[first]
        samples[first], first = nil, first + 1
    end
    local mode = state and state.controller and state.controller.mode
    if emitted == 0 and (not state or mode == 'falling' or mode == 'fallen' or mode == 'getting_up') then
        M.finishReplay()
        M.flush('first_failure')
    end
end

function M.finishReplay()
    if replayStarted then
        local replay = ragdoll.finishReplayRecording()
        writeRecord(encode({ kind = 'fixed_step_replay', test = active, present = replay ~= nil,
            seed = replaySeed,
            frames = replay and replay.frameCount, bytes = replay and replay.byteCount,
            profile_hash = replay and replay.profileHash, fixed_time_step = replay and replay.fixedTimeStep,
            output_path = replay and replay.outputPath }))
        replayStarted, replayCompleted = false, true
    end
end

function M.endObservation()
    -- The test deliberately deletes its rig next. Do not misreport that
    -- cleanup as an unexpected loss of the player body.
    observing = false
    M.finishReplay()
end

function M.finish(ok)
    writeRecord(encode({ kind = 'capture_stop', test = active, real_time = core.getRealTime() }))
    M.finishReplay()
    M.flush(ok and 'test_complete' or 'test_failed')
    world.players[1]:sendEvent('ragdollDiagnosticInput', { enabled = false })
    active = nil
end

return M

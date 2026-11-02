-- Authored timelines, sampled by the ordinary native local onUpdate path.
local core = require('openmw.core')
local M = {}

function M.plan(scenario)
    if scenario == 'S1' then return { startStep = 60, stopStep = 660, endStep = 960 } end
    assert((scenario == 'S2' or scenario == 'S2Straight'), 'unsupported FPS source scenario')
    return { startStep = 60, stopStep = 300, restartStep = 540, finalStopStep = 792, endStep = 1092 }
end

function M.sample(source, player)
    local plan = M.plan(source.scenario)
    local tick = math.floor(core.getSimulationTime() * 120 + 0.5) - source.epochHalfTick
    local movement = tick >= 2 * plan.startStep and tick < 2 * plan.stopStep and 1 or 0
    local yawChange = 0
    if source.scenario ~= 'S1' then
        if tick >= 2 * plan.restartStep and tick < 2 * plan.finalStopStep then movement = 1 end
        local function yaw(t) return (source.scenario == 'S2' and 2 or 0) * math.min(0.84, math.max(0, (t - 1080) / 120)) end
        yawChange = yaw(tick) - yaw(source.previousHalfTick or 0)
        source.previousHalfTick = tick
        print(string.format('RAGDOLL_FPS_SOURCE schema=2 scenario=%s player=%d epoch_half_tick=%d half_tick=%d movement=%d yaw_change=%.17g authored_yaw=%.17g',
            source.scenario, player and 1 or 0, source.epochHalfTick, tick, movement, yawChange, yaw(tick)))
    else
        print(string.format('RAGDOLL_FPS_SOURCE schema=1 scenario=S1 player=%d epoch_half_tick=%d half_tick=%d movement=%d yaw_change=0',
            player and 1 or 0, source.epochHalfTick, tick, movement))
    end
    return movement, yawChange
end

return M

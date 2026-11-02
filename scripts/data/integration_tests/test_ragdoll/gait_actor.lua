local core = require('openmw.core')
local self = require('openmw.self')

local movement
local fpsSource
local sourceSampler = require('fps_source')

local function update()
    local yawChange = 0
    if fpsSource then movement, yawChange = sourceSampler.sample(fpsSource, false) end
    if movement ~= nil then
        self.controls.jump = false
        self.controls.run = false
        self.controls.movement = movement
        self.controls.sideMovement = 0
        self.controls.yawChange = yawChange
    end
end

return {
    engineHandlers = {
        onUpdate = update,
    },
    eventHandlers = {
        ragdollFpsSource = function(source)
            self:enableAI(false)
            fpsSource = source
        end,
        startRagdollTravel = function(value)
            self:enableAI(false)
            -- The first deterministic walker is tuned and gated at roughly
            -- 60 world units/s. Higher requested speeds belong in the later
            -- scenario matrix rather than the basic physical-walk gate.
            movement = value * 0.4
            core.sendGlobalEvent('ragdollGaitCommandReceived', true)
        end,
    },
}

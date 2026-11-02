local testing = require('testing_util')
local core = require('openmw.core')
local nearby = require('openmw.nearby')
local self = require('openmw.self')
local util = require('openmw.util')
local testArea = require('ragdoll_test_area')
local camera = require('openmw.camera')
local interfaces = require('openmw.interfaces')
local travel
local previousCameraMode
local fpsSource
local sourceSampler = require('fps_source')
testing.localEventHandlers.ragdollFpsSource = function(source)
    interfaces.Controls.overrideMovementControls(true)
    fpsSource = source
end

testing.registerLocalTestStep('ragdoll native sampling interfaces', function()
    testing.expect(interfaces.Camera ~= nil, 'native Camera script did not initialize')
    testing.expect(interfaces.Controls ~= nil, 'native Controls script did not initialize')
    testing.expect(interfaces.MWUI ~= nil, 'native MWUI script did not initialize')
end)

testing.registerLocalTestStep('ragdoll third person', function()
    previousCameraMode = camera.getMode()
    camera.setMode(camera.MODE.ThirdPerson)
    camera.setYaw(self.rotation:getYaw())
    camera.setPreferredThirdPersonDistance(260)
    camera.setPitch(0.25)
end)

local diagnosticInput = false
testing.localEventHandlers.ragdollDiagnosticInput = function(command) diagnosticInput = command.enabled end
testing.localEventHandlers.ragdollPlayerTravel = function(command)
    if command.enabled then
        interfaces.Controls.overrideMovementControls(true)
        previousCameraMode = previousCameraMode or camera.getMode()
        camera.setMode(camera.MODE.ThirdPerson)
        travel = command
    else
        interfaces.Controls.overrideMovementControls(false)
        self.controls.movement = 0
        self.controls.sideMovement = 0
        self.controls.run = false
        travel = nil
        if previousCameraMode then camera.setMode(previousCameraMode) end
        previousCameraMode = nil
    end
end

-- Invoked by every global scenario after relocation, including filtered runs.
-- Query physical collision, not just the render mesh or the spawn's pathgrid.
testing.registerLocalTestStep('ragdoll flat floor geometry', function()
    testing.expectEqual(self.cell.name, testArea.cell, 'flat floor validation ran in the wrong cell')
    local options = {
        collisionType = nearby.COLLISION_TYPE.World + nearby.COLLISION_TYPE.HeightMap
            + nearby.COLLISION_TYPE.Door,
    }
    -- West of x = -1300 the floor carries the walking suite's fixtures
    -- (PGC-03, 10-11: platforms, ramps, a wall, tall stairs); this suite's
    -- area is the flat part east of it.
    for x = -1000, 3500, 1500 do
        for y = -3500, 3500, 1750 do
            local hit = nearby.castRay(util.vector3(x, y, 256), util.vector3(x, y, -64), options)
            testing.expect(hit.hit, string.format('missing floor at (%d, %d)', x, y))
            testing.expect(hit.hitObject ~= nil and hit.hitObject.recordId == testArea.floorRecord,
                'baseline ray hit geometry other than the test floor')
            testing.expectLessOrEqual(math.abs(hit.hitPos.z - testArea.floorHeight), 0.01,
                'baseline floor is not level at z=0')
            testing.expectGreaterOrEqual(hit.hitNormal.z, 0.99999, 'baseline floor normal is not vertical')
        end
    end
    for z = 1, 201, 100 do
        for offset = -3500, 3500, 1750 do
            testing.expect(not nearby.castRay(util.vector3(-1300, offset, z),
                util.vector3(3500, offset, z), options).hit, 'obstacle across the flat floor X corridor')
            if offset > -1300 then
                testing.expect(not nearby.castRay(util.vector3(offset, -3500, z),
                    util.vector3(offset, 3500, z), options).hit, 'obstacle across the flat floor Y corridor')
            end
        end
    end
end)

return {
    engineHandlers = {
        onUpdate = function(dt)
            testing.updateLocal(dt)
            if fpsSource then
                self.controls.movement, self.controls.yawChange = sourceSampler.sample(fpsSource, true)
                self.controls.sideMovement = 0
                self.controls.run = false
                self.controls.jump = false
            end
            if travel then
                self.controls.movement = travel.movement
                self.controls.sideMovement = 0
                self.controls.run = travel.run == true
                self.controls.jump = false
                self.controls.yawChange = (travel.yawRate or 0) * dt
            end
            if diagnosticInput then
                core.sendGlobalEvent('ragdollDiagnosticInputSample', {
                    real_time = core.getRealTime(), simulation_time = core.getSimulationTime(),
                    movement = self.controls.movement, side_movement = self.controls.sideMovement,
                    run = self.controls.run, yaw_change = self.controls.yawChange,
                    use = self.controls.use,
                    camera_yaw = camera.getYaw(), gameplay_yaw = self.rotation:getYaw(),
                    camera_pitch = camera.getPitch(), camera_distance = camera.getThirdPersonDistance(),
                    camera_position = camera.getPosition(),
                })
            end
        end,
    },
    eventHandlers = testing.localEventHandlers,
}

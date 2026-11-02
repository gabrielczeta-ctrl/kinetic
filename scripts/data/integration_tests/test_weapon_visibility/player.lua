local camera = require('openmw.camera')
local animation = require('openmw.animation')
local core = require('openmw.core')
local I = require('openmw.interfaces')
local self = require('openmw.self')
local testing = require('testing_util')
local types = require('openmw.types')
local pending
local handlers = {}
for name, handler in pairs(testing.localEventHandlers) do handlers[name] = handler end
handlers.weaponVisibilityConfigure = function(data)
    -- Keep camera, equipment and stance in the same event. Splitting these
    -- actions across settled frames would hide the reported lifecycle race.
    if data.camera then
        camera.setMode(data.camera == 'first_person' and camera.MODE.FirstPerson or camera.MODE.ThirdPerson)
        camera.setPreferredThirdPersonDistance(150)
        camera.setPitch(0)
        camera.setYaw(0)
    end
    if data.equip ~= nil then
        local equipment = types.Actor.getEquipment(self)
        equipment[types.Actor.EQUIPMENT_SLOT.CarriedRight] = data.equip
            and assert(types.Actor.inventory(self):find(data.equip)) or nil
        types.Actor.setEquipment(self, equipment)
    end
    if data.drawn ~= nil then
        types.Actor.setStance(self, data.drawn and types.Actor.STANCE.Weapon or types.Actor.STANCE.Nothing)
    end
    pending = data
end
return {
    eventHandlers = handlers,
    engineHandlers = { onUpdate = function()
        types.Player.setControlSwitch(self, types.Player.CONTROL_SWITCH.VanityMode, false)
        types.Player.setControlSwitch(self, types.Player.CONTROL_SWITCH.Controls, true)
        types.Player.setControlSwitch(self, types.Player.CONTROL_SWITCH.Fighting, true)
        I.Controls.overrideMovementControls(true)
        self.controls.movement, self.controls.sideMovement = 0, 0
        self.controls.run, self.controls.jump = false, false
        if not pending then return end
        local item = types.Actor.getEquipment(self, types.Actor.EQUIPMENT_SLOT.CarriedRight)
        if core.getSimulationTime() >= (pending.nextReport or 0) then
            core.sendGlobalEvent('weaponVisibilityProgress', {
                sequence = pending.sequence, camera = camera.getMode(),
                queued_camera = camera.getQueuedMode() or -1,
                equipped = item and item.recordId or '', stance = types.Actor.getStance(self),
                one_hand_completion = animation.getCompletion(self, 'weapononehand') or -1,
                two_hand_completion = animation.getCompletion(self, 'weapontwohand') or -1,
            })
            pending.nextReport = core.getSimulationTime() + 0.25
        end
        -- Completion=nil means the native controller removed this group's
        -- state, not merely that its attach text key has already fired.
        if pending.waitAnimationGone and animation.getCompletion(self, pending.waitAnimationGone) ~= nil then return end
        if pending.equip ~= nil and ((pending.equip and (not item or item.recordId ~= pending.equip))
            or (not pending.equip and item)) then return end
        if pending.drawn ~= nil then
            local wanted = pending.drawn and types.Actor.STANCE.Weapon or types.Actor.STANCE.Nothing
            if types.Actor.getStance(self) ~= wanted then types.Actor.setStance(self, wanted); return end
        end
        if pending.camera then
            local wanted = pending.camera == 'first_person' and camera.MODE.FirstPerson or camera.MODE.ThirdPerson
            if camera.getMode() ~= wanted then
                if camera.getQueuedMode() ~= wanted then camera.setMode(wanted) end
                return
            end
        end
        core.sendGlobalEvent('weaponVisibilityReady', pending.sequence)
        pending = nil
    end },
}

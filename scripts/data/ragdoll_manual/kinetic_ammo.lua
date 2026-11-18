-- Ammunition for a drawn bow or crossbow (user 11-14: "player cant use
-- bows"). Morrowind fires nothing while the ammunition slot is empty or
-- holds the other kind, and says nothing about it: the player looted a long
-- bow and 40 chitin arrows, drew the bow and clicked some 60 times over 70 s
-- (playthrough of 11-13 night). With the arrows in the slot the same save
-- shoots. So while a bow or crossbow is drawn and the slot has no fitting
-- ammunition, the fitting stack in the pack goes in: the kind last used with
-- that weapon type if any is left, else the largest stack. A stack that runs
-- out is followed by the next the same way. With none in the pack, an attack
-- press says so.
local core = require('openmw.core')
local I = require('openmw.interfaces')
local input = require('openmw.input')
local self = require('openmw.self')
local types = require('openmw.types')
local ui = require('openmw.ui')

local CHECK_EVERY = 0.25
local MESSAGE_EVERY = 3

local Weapon = types.Weapon
local AMMO = {
    [Weapon.TYPE.MarksmanBow] = { type = Weapon.TYPE.Arrow, none = 'noArrows' },
    [Weapon.TYPE.MarksmanCrossbow] = { type = Weapon.TYPE.Bolt, none = 'noBolts' },
}
local SLOT = types.Actor.EQUIPMENT_SLOT

local l10n = core.l10n('Kinetic')
local lastUsed = {} -- ammunition type -> record id
local sinceCheck = CHECK_EVERY
local sinceMessage = MESSAGE_EVERY
local missing = nil -- the l10n key to show on an attack press
local useHeld = false

local function wanted()
    if types.Actor.getStance(self) ~= types.Actor.STANCE.Weapon then return nil end
    local weapon = types.Actor.getEquipment(self, SLOT.CarriedRight)
    if not weapon or not Weapon.objectIsInstance(weapon) then return nil end
    return AMMO[Weapon.record(weapon).type]
end

local function check()
    missing = nil
    local want = wanted()
    if not want then return end
    local ammo = types.Actor.getEquipment(self, SLOT.Ammunition)
    if ammo and Weapon.objectIsInstance(ammo) and Weapon.record(ammo).type == want.type then
        lastUsed[want.type] = ammo.recordId
        return
    end
    local best
    for _, item in ipairs(types.Actor.inventory(self):getAll(Weapon)) do
        if Weapon.record(item).type == want.type then
            if item.recordId == lastUsed[want.type] then
                best = item
                break
            end
            if not best or item.count > best.count then best = item end
        end
    end
    if not best then
        missing = want.none
        return
    end
    local equipment = types.Actor.getEquipment(self)
    equipment[SLOT.Ammunition] = best
    types.Actor.setEquipment(self, equipment)
    print('KINETIC_AMMO equipped ' .. best.recordId .. ' x' .. best.count)
end

return {
    engineHandlers = {
        onUpdate = function(dt)
            sinceCheck = sinceCheck + dt
            sinceMessage = sinceMessage + dt
            if sinceCheck >= CHECK_EVERY then
                sinceCheck = 0
                check()
            end
            local held = input.getBooleanActionValue('Use') and I.UI.getMode() == nil
            if held and not useHeld and missing and sinceMessage >= MESSAGE_EVERY then
                sinceMessage = 0
                ui.showMessage(l10n(missing))
            end
            useHeld = held
        end,
        onSave = function() return { lastUsed = lastUsed } end,
        onLoad = function(data) lastUsed = data and data.lastUsed or {} end,
    },
}

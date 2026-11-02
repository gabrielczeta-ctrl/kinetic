-- Other combat mods on KINETIC's bodies (COMPAT-01, user 10-31). Each part
-- does nothing unless its mod is loaded.
--
-- Mercy: Combat AI Overhaul (MaxYari) drives NPCs in combat from Lua with the
-- engine AI switched off, and KINETIC's duel AI (lines, guard, feints, pacing)
-- is part of the engine AI: under Mercy a powered NPC swings physically but
-- picks its blows like a vanilla clicker. So a melee fight goes to KINETIC:
-- while Mercy says FIGHT and the NPC is powered, in combat and holding a
-- melee weapon (or fists), Mercy is switched off for it (its own interface,
-- I.MercyCAO.setEnabled). Everything else stays Mercy's: warnings and standing
-- ground, retreat, surrender, hiding, spellcasters and marksmen, and any NPC
-- KINETIC is not powering (swimming, walking free of a snag). Its partial
-- switches (combatEnabled/locomotionEnabled) do not fit: with both off Mercy
-- still holds the NPC, standing still facing its enemy.
-- Mercy rolls surrender and retreat from the hits it sees in its own update,
-- which does not run while it is off; a hit that leaves the NPC below
-- LOOK_BELOW of its health hands it back for LOOK_FRAMES frames to roll. Mercy
-- reads the damage since its last update, so each look sees that one hit.
local I = require('openmw.interfaces')
local self = require('openmw.self')
local types = require('openmw.types')
local kinetic = require('openmw.kinetic')

local CHECK_EVERY = 0.2
local LOOK_BELOW = 0.5
local LOOK_FRAMES = 2

local RANGED = {
    [types.Weapon.TYPE.MarksmanBow] = true,
    [types.Weapon.TYPE.MarksmanCrossbow] = true,
    [types.Weapon.TYPE.MarksmanThrown] = true,
    [types.Weapon.TYPE.Arrow] = true,
    [types.Weapon.TYPE.Bolt] = true,
}

local function meleeStance()
    if types.Actor.getStance(self) ~= types.Actor.STANCE.Weapon then return false end
    local weapon = types.Actor.getEquipment(self, types.Actor.EQUIPMENT_SLOT.CarriedRight)
    if not weapon or not types.Weapon.objectIsInstance(weapon) then return true end
    return not RANGED[types.Weapon.record(weapon).type]
end

local function inCombat()
    local package = I.AI and I.AI.getActivePackage()
    return package ~= nil and package.type == 'Combat'
end

local sinceCheck = CHECK_EVERY
local mercyOff = false
local lookFrames = 0
local lastHealth

local function log(what)
    print(string.format('KINETIC_COMPAT mercy=%s npc=%s', what, tostring(self.recordId)))
end

local function setMercy(on, why)
    I.MercyCAO.setEnabled(on)
    mercyOff = not on
    log((on and 'on ' or 'off ') .. why)
end

local function updateMercy(dt)
    local mercy = I.MercyCAO
    if not mercy or not mercy.setEnabled then return end

    local health = types.Actor.stats.dynamic.health(self)
    local hit = lastHealth ~= nil and health.current < lastHealth
    lastHealth = health.current

    if lookFrames > 0 then
        lookFrames = lookFrames - 1
        if lookFrames > 0 then return end
    else
        sinceCheck = sinceCheck + dt
        -- A hit while low is looked at the frame it shows, not on the next check
        local low = mercyOff and hit and health.base > 0 and health.current / health.base < LOOK_BELOW
        if low and not types.Actor.isDead(self) then
            setMercy(true, 'look')
            lookFrames = LOOK_FRAMES
            return
        end
        if sinceCheck < CHECK_EVERY then return end
    end
    sinceCheck = 0

    local kineticFight = kinetic.isPowered(self) and inCombat() and meleeStance()
        and not types.Actor.isDead(self)
    if mercyOff then
        if not kineticFight then setMercy(true, 'release') end
    elseif kineticFight and mercy.state and mercy.state.combatState == 'FIGHT' then
        setMercy(false, 'melee')
    end
end

return {
    engineHandlers = {
        onUpdate = function(dt)
            if dt <= 0 then return end
            updateMercy(dt)
        end,
        -- Mercy's switch is not saved: it starts on after a load, and so does this
        onLoad = function() mercyOff = false end,
    },
}

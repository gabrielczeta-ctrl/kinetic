-- WPN-04 manual duel: iron daggers, fists, or iron longswords with contact-work wounds.
local core = require('openmw.core')
local I = require('openmw.interfaces')
local ragdoll = require('openmw.ragdolltest')
local types = require('openmw.types')
local util = require('openmw.util')
local world = require('openmw.world')
local phase, opponent, record, deadline = 'init', nil, nil, 0
local loadouts = {
    { weapon = 'iron dagger', label = 'Iron daggers', hint = 'Attack + drag slashes; middle mouse stabs (hold to extend).' },
    { label = 'Fists', hint = 'Click attack to alternate punches; drag to shape them.' },
    { weapon = 'iron longsword', label = 'Iron longswords', hint = 'Attack + drag cuts; middle mouse thrusts. Edge slashes, tip punctures, flat/guard bruises.' },
}
local selection, lastHud = 1, -1
local requested = require('wound_duel_loadout').weapon
for index, loadout in ipairs(loadouts) do
    if (loadout.weapon or 'fists') == requested then selection = index end
end
local function loadout() return loadouts[selection] end
local function unarmed() return loadout().weapon == nil end
local function label() return loadout().label end
local function announce(text, loud)
    local player = world.players[1]
    if player then player:sendEvent('ragdollManualDuelStatus', { text = text, announce = loud == true }) end
    if loud then print('DMG03_DUEL ' .. text) end
end
local function reset()
    local player = world.players[1]
    if types.Actor.isDead(player) then
        announce('Round over. Relaunch Run-Wound-Duel.cmd for a fresh player.', true)
        return
    end
    I.RagdollDev.autoDuel(false)
    if opponent and opponent:isValid() and opponent.count > 0 then
        ragdoll.setPowered(opponent, false)
        opponent:remove()
    end
    opponent = nil
    ragdoll.clearLimbDamage()
    I.RagdollDev.playerFists(loadout().weapon)
    player:sendEvent('woundDuelPrepare', { unarmed = unarmed(), weapon = loadout().weapon })
    phase, deadline = 'player', core.getSimulationTime() + 15
    announce(label() .. ': preparing one opponent...', true)
end
local function equipped(actor)
    local item = types.Actor.getEquipment(actor, types.Actor.EQUIPMENT_SLOT.CarriedRight)
    return types.Actor.getStance(actor) == types.Actor.STANCE.Weapon
        and ((unarmed() and not item) or (not unarmed() and item and item.recordId == loadout().weapon))
end
local function standing(actor)
    local state = ragdoll.getState(actor)
    return state and state.powered and state.controller and state.controller.mode == 'standing'
        and ((unarmed() and not state.weapon) or (not unarmed() and state.weapon and state.weapon.attached))
end
return {
    engineHandlers = {
        onUpdate = function()
            local helper, player = I.RagdollDev, world.players[1]
            if not helper or not player then return end
            local now = core.getSimulationTime()
            if phase == 'init' then
                helper.autoDuel(false)
                helper.setup()
                phase, deadline = 'range', now + 15
            elseif phase == 'range' and helper.samples().defense then
                helper.cleanup()
                reset()
            elseif phase == 'equipment' and equipped(opponent) then
                if not unarmed() then ragdoll.prepareEquippedWeapon(opponent) end
                ragdoll.setPowered(opponent, true)
                phase = 'bodies'
            elseif phase == 'bodies' and standing(player) and standing(opponent) then
                phase = 'ready'
                announce(label() .. ' READY. F6 starts; F7 cycles dagger/fists/longsword. '
                    .. loadout().hint, true)
            elseif phase == 'countdown' and now >= deadline then
                opponent:sendEvent('woundDuelFight', true)
                ragdoll.setCombatTarget(opponent, player)
                phase = 'fighting'
                announce(label() .. ': FIGHT! F6 resets; F7 switches loadout.', true)
            elseif phase == 'fighting' then
                if types.Actor.isDead(opponent) or types.Actor.isDead(player) then
                    opponent:sendEvent('woundDuelFight', false)
                    phase = 'finished'
                    announce(types.Actor.isDead(player) and 'You lost. Relaunch Run-Wound-Duel.cmd.'
                        or 'You won! F6 resets; F7 switches loadout.', true)
                elseif now - lastHud >= 0.5 then
                    lastHud = now
                    local state = ragdoll.getState(opponent)
                    local hit = state and state.lastWoundKind ~= 'none'
                        and (' | Last: ' .. state.lastWoundKind .. ' ' .. state.lastWoundRegion:gsub('_', ' ')) or ''
                    announce(string.format('%s | You %.1f HP | NPC %.1f HP / %.0f fatigue%s | F6 reset, F7 loadout',
                        label(), types.Actor.stats.dynamic.health(player).current,
                        types.Actor.stats.dynamic.health(opponent).current,
                        types.Actor.stats.dynamic.fatigue(opponent).current, hit))
                end
            end
            if (phase == 'range' or phase == 'player' or phase == 'equipment' or phase == 'bodies') and now > deadline then
                phase = 'failed'
                announce('Wound test setup timed out. F6 retries; inspect the log.', true)
            end
        end,
    },
    eventHandlers = {
        woundDuelPlayerReady = function(data)
            if phase ~= 'player' then return end
            local player = world.players[1]
            if not record then
                record = world.createRecord(types.NPC.createRecordDraft({
                    template = types.NPC.record(player), name = 'Wound Test Opponent',
                    isEssential = false, isPersistent = false, isRespawning = false,
                }))
            end
            opponent = world.createObject(record.id)
            if not unarmed() then world.createObject(loadout().weapon, 1):moveInto(types.Actor.inventory(opponent)) end
            opponent:addScript('wound_duel_actor.lua', { unarmed = unarmed(), weapon = loadout().weapon, health = 40 })
            local forward = util.vector3(data.forward.x, data.forward.y, 0)
            if forward:length() < 0.1 then forward = util.vector3(0, 1, 0) end
            forward = forward / forward:length()
            opponent:teleport(player.cell, player.position + forward * 140,
                { onGround = true, rotation = util.transform.rotateZ(player.rotation:getYaw() + math.pi) })
            phase = 'equipment'
        end,
        woundDuelAction = function(data)
            if data.toggle and (phase == 'ready' or phase == 'fighting' or phase == 'finished' or phase == 'failed') then
                selection = selection % #loadouts + 1
                reset()
            elseif phase == 'ready' then
                phase, deadline = 'countdown', core.getSimulationTime() + 3
                announce('Starting in 3 seconds...', true)
            elseif phase == 'fighting' or phase == 'finished' or phase == 'failed' then reset() end
        end,
    },
}

-- NET-01 duel, the player's side: the kit and stats both games share, a
-- round ends at 1 health instead of death, and the duel bar at the top.
local async = require('openmw.async')
local core = require('openmw.core')
local I = require('openmw.interfaces')
local net = require('openmw.kineticnet')
local self = require('openmw.self')
local types = require('openmw.types')
local ui = require('openmw.ui')
local util = require('openmw.util')

local Actor = types.Actor
local NPC = types.NPC

-- The same fighter on both sides (a fair duel needs no stat exchange).
local STATS = {
    attributes = { strength = 50, agility = 50, endurance = 50, speed = 50, luck = 40, willpower = 40 },
    skills = { longblade = 50, block = 35, unarmored = 30, athletics = 35, axe = 50, bluntweapon = 50,
        shortblade = 50, spear = 50, handtohand = 40 },
    health = 120,
    fatigue = 300,
}

local score = { mine = 0, theirs = 0 }
local opponentHealth
local banner = ''
local bannerUntil = 0
local roundOver = false
local downSent = false
local healthSent = 0
local element
local botNext = 0
local botRelease = false

local function applyStats()
    for name, value in pairs(STATS.attributes) do
        NPC.stats.attributes[name](self).base = value
    end
    for name, value in pairs(STATS.skills) do
        NPC.stats.skills[name](self).base = value
    end
    local health = Actor.stats.dynamic.health(self)
    health.base = STATS.health
    local fatigue = Actor.stats.dynamic.fatigue(self)
    fatigue.base = STATS.fatigue
end

local function restore()
    applyStats()
    for _, name in ipairs({ 'health', 'fatigue', 'magicka' }) do
        local stat = Actor.stats.dynamic[name](self)
        stat.current = stat.base
    end
    downSent = false
    roundOver = false
end

local function status()
    local state = net.state()
    if state == 'waiting' then
        return string.format('KINETIC duel - waiting for an opponent on %s', net.endpoint())
    elseif state == 'connecting' then
        return string.format('KINETIC duel - connecting to %s ...', net.endpoint())
    elseif state == 'lost' then
        return 'KINETIC duel - the opponent is gone' .. (net.isHost() and ' (waiting for them to return)'
            or ' (reconnecting)')
    elseif state == 'connected' then
        local ping = net.ping()
        local line = string.format('%s %d : %d %s', net.localName(), score.mine, score.theirs, net.remoteName())
        if opponentHealth then
            line = line .. string.format('   opponent %d/%d', math.floor(opponentHealth.current + 0.5),
                math.floor(opponentHealth.base + 0.5))
        end
        if ping >= 0 then line = line .. string.format('   ping %d ms', math.floor(ping + 0.5)) end
        return line
    end
    local problem = net.lastError()
    return 'KINETIC duel - not connected' .. (problem ~= '' and (': ' .. problem) or '')
end

local function draw()
    local text = status()
    if banner ~= '' and core.getRealTime() < bannerUntil then text = text .. '\n' .. banner end
    if element then
        element.layout.props.text = text
        element:update()
        return
    end
    element = ui.create({
        layer = 'HUD',
        type = ui.TYPE.Text,
        props = {
            relativePosition = util.vector2(0.5, 0.02),
            anchor = util.vector2(0.5, 0),
            text = text,
            textSize = 20,
            textColor = util.color.rgb(0.95, 0.85, 0.55),
            textShadow = true,
            textAlignH = ui.ALIGNMENT.Center,
            multiline = true,
        },
    })
end

-- A round ends at 1 health: the duel never kills (OpenMW's death screen
-- would end the game). Runs before the built-in handler that pays damage.
I.Combat.addOnHitHandler(function(attack)
    if net.state() == 'off' or not attack.damage or not attack.damage.health then return end
    local health = Actor.stats.dynamic.health(self)
    if roundOver then
        attack.damage.health = 0
        return
    end
    attack.damage.health = math.min(attack.damage.health, math.max(0, health.current - 1))
end)

local function onUpdate()
    if net.state() == 'off' then return end
    draw()
    if net.state() ~= 'connected' then return end
    local health = Actor.stats.dynamic.health(self)
    if not roundOver and not downSent and health.current <= 1.5 then
        downSent = true
        core.sendGlobalEvent('KineticDuelDown', {})
    end
    if core.getRealTime() >= healthSent then
        healthSent = core.getRealTime() + 0.25
        net.send(string.format('hp|%.1f|%.1f', health.current, health.base))
    end
    -- The test bot: a click every two seconds with the weapon drawn.
    if net.bot() and not roundOver and Actor.getStance(self) == Actor.STANCE.Weapon then
        if botRelease then
            net.click(false)
            botRelease = false
        elseif core.getRealTime() >= botNext then
            botNext = core.getRealTime() + 2
            net.click(true)
            botRelease = true
        end
    end
end

return {
    engineHandlers = { onUpdate = onUpdate },
    eventHandlers = {
        KineticDuelKit = function(data)
            restore()
            local weapon = Actor.inventory(self):find(data.weapon)
            if weapon then
                Actor.setEquipment(self, { [Actor.EQUIPMENT_SLOT.CarriedRight] = weapon })
                -- The stance waits a frame for the equipment.
                async:newUnsavableSimulationTimer(0.2, function()
                    Actor.setStance(self, Actor.STANCE.Weapon)
                end)
            end
        end,
        KineticDuelRestore = restore,
        KineticDuelOpponentHealth = function(data) opponentHealth = data end,
        KineticDuelScore = function(data)
            score.mine, score.theirs = data.mine, data.theirs
            roundOver = data.roundOver
            if data.roundOver then
                banner = data.won and 'You win the round' or 'You lose the round'
                bannerUntil = core.getRealTime() + 4
            else
                banner = 'Fight!'
                bannerUntil = core.getRealTime() + 1.5
            end
        end,
    },
}

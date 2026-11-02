local core = require('openmw.core')
local I = require('openmw.interfaces')
local ragdoll = require('openmw.ragdolltest')
local types = require('openmw.types')
local util = require('openmw.util')
local world = require('openmw.world')
local cases = require('manual_hub_cases')
local config = require('manual_hub_config')
local status = require('manual_hub_status')
local items = require('manual_hub_items')
local index, circuit, phase = 1, 1, 'init'
-- A restricted circuit (launcher -Only) is for playing, not for a timed tour:
-- it never advances by itself and a duel waits for F10 before the opponent fights.
-- The automated wiring check still has to tour on its timer.
local manualStart = config.only ~= nil and not config.checkCircuits
local paused, remaining, deadline, lastHud, nextAction = manualStart, 0, 0, -1, 0
local fighting = false
-- Why the duel was restaged, shown with the next 'waits' line so a restage
-- after a kill does not read as a timeout.
local outcome = ''
local resetAt = 0
local opponent, record, note, actions, aiMode = nil, nil, '', 0, 1
-- Duel: a killed opponent stays as a corpse (the last few) and a new duelist
-- comes in, the player as they are (user 10-05: "killing the duel opponent
-- spawns another duelist instead of restarting everything").
local corpses, kills, respawning = {}, 0, false
local maxCorpses = 3
-- The foe now in the duel: its weapon, armor pieces and cuirass. The case's
-- own at the start; each next duelist takes the next distinct foe of the duel
-- cases (user 10-05: "make them cycle between the available duelists with the
-- different weapons").
local foe, foeCycle, foeIndex = nil, nil, 0
local function player() return world.players[1] end
-- Self-check auto attacks (see autoClick).
local autoClickAt, autoClickDown, autoShot, autoClicks = 0, false, false, 0
local function case() return cases[index] end
-- A scenario's equipment, resolved once from the game records: the player's
-- and the foe's weapon (a missing material falls back to iron), and the
-- armor/shield ids each side wears.
local loadouts = {}
local function loadout(c)
    c = c or case()
    if loadouts[c.id] then return loadouts[c.id] end
    local L = { playerEquip = {}, foeEquip = {}, notes = {} }
    local function weapon(id)
        if not id then return nil end
        local fallback = id:gsub('^%a+ ', 'iron ')
        local resolved = items.weapon(id, items.weapon(fallback, 'iron longsword'))
        if resolved ~= id then L.notes[#L.notes + 1] = id .. ' missing, using ' .. resolved end
        return resolved
    end
    L.player = weapon(c.playerWeapon or c.weapon)
    L.foe = weapon(c.foeWeapon or c.weapon)
    if c.playerEnchanted then
        local enchanted = items.enchantedWeapon(types.Weapon.TYPE.LongBladeOneHand)
        if enchanted then L.player = enchanted else L.notes[#L.notes + 1] = 'no enchanted longsword found' end
    end
    local function add(list, ids, what)
        if #ids == 0 then L.notes[#L.notes + 1] = 'no ' .. what .. ' found' end
        for _, id in ipairs(ids) do list[#list + 1] = id end
    end
    if c.playerArmor then add(L.playerEquip, items.armorSet(c.playerArmor), c.playerArmor .. ' armor') end
    if c.foeArmor then add(L.foeEquip, items.armorSet(c.foeArmor), c.foeArmor .. ' armor') end
    if c.playerShield then add(L.playerEquip, { items.shield(c.playerShield) }, c.playerShield .. ' shield') end
    if c.foeShield then add(L.foeEquip, { items.shield(c.foeShield) }, c.foeShield .. ' shield') end
    L.text = string.format('You: %s%s | Foe: %s%s', L.player or 'fists',
        #L.playerEquip > 0 and (' + ' .. #L.playerEquip .. ' armor pieces') or '',
        L.foe or 'fists', #L.foeEquip > 0 and (' + ' .. #L.foeEquip .. ' armor pieces') or '')
    if #L.notes > 0 then L.text = L.text .. ' (' .. table.concat(L.notes, '; ') .. ')' end
    print('PLAY05_HUB_LOADOUT ' .. c.id .. ' ' .. L.text .. ' player=[' .. table.concat(L.playerEquip, ',')
        .. '] foe=[' .. table.concat(L.foeEquip, ',') .. ']')
    loadouts[c.id] = L
    return L
end
-- Items this hub handed the player for a scenario, taken back at the next.
local givenToPlayer = {}
local function takeBackPlayerItems()
    local inventory = types.Actor.inventory(player())
    for _, id in ipairs(givenToPlayer) do
        for _, item in ipairs(inventory:findAll(id)) do item:remove() end
    end
    givenToPlayer = {}
end
local function givePlayer(id)
    if not id then return end
    local inventory = types.Actor.inventory(player())
    if inventory:countOf(id) == 0 then
        world.createObject(id, 1):moveInto(inventory)
        givenToPlayer[#givenToPlayer + 1] = id
    end
end
local function state(actor) return actor and actor:isValid() and ragdoll.getState(actor) end
local function standing(actor)
    local s = state(actor)
    return s and s.controller and s.controller.mode == 'standing'
end
local function publish(loud)
    local text = string.format('Manual circuit %d | %d/%d %s | %s | %s\n%s\n%s',
        circuit, index, #cases, case().title, phase,
        paused and 'AUTO-ADVANCE PAUSED' or string.format('%.0fs', math.max(0, remaining)), case().hint, note)
    local enemy, detail = '', ''
    if opponent and opponent:isValid() then
        enemy, detail = status.enemy(state(opponent), types.Actor.stats.dynamic.health(opponent).current,
            types.Actor.stats.dynamic.health(player()).current)
    end
    local short = string.format('%d/%d %s | %s', index, #cases, case().title,
        phase == 'ready' and case().kind == 'duel' and (fighting and 'fighting' or 'F10 to start') or phase)
    -- Keep setup failures visible too; routine fight statistics have dedicated
    -- short lines instead of one long paragraph across the aiming view.
    if enemy == '' or phase == 'failed' then short = short .. '\n' .. note end
    player():sendEvent('manualHubStatus', { text = text, short = short, announce = loud == true,
        compact = true, enemy = enemy, detail = detail })
    if loud then print('PLAY05_HUB ' .. text:gsub('\n', ' | ')) end
end
local function removeOpponent()
    if opponent and opponent:isValid() then
        opponent:sendEvent('manualHubFight', false)
        ragdoll.setPowered(opponent, false)
        opponent:remove()
    end
    opponent = nil
end
-- The room gives the player every test weapon (about 170 weight); with a few of
-- the heavy two-handers that is over most characters' capacity and the player
-- cannot move. Keep only what this scenario uses. This has to run after the
-- room's player setup: playerFists() hands every missing test weapon back, so
-- lightening before it (the first version) left the player overencumbered.
local hubWeapons = { 'iron dagger', 'iron longsword', 'iron claymore', 'iron warhammer',
    'steel warhammer', 'wooden staff', 'iron spear', 'iron battle axe' }
local function lightenPlayer(keep)
    local inventory = types.Actor.inventory(player())
    for _, id in ipairs(hubWeapons) do
        if id ~= keep then
            for _, item in ipairs(inventory:findAll(id)) do item:remove() end
        end
    end
end
-- Evidence for the log: which test weapons the player still carries in play.
local loadLogged = true
local function logPlayerLoad()
    local inventory, names, weight = types.Actor.inventory(player()), {}, 0
    for _, id in ipairs(hubWeapons) do
        local count = inventory:countOf(id)
        if count > 0 then
            names[#names + 1] = id
            weight = weight + count * types.Weapon.record(id).weight
        end
    end
    local health = types.Actor.stats.dynamic.health(player())
    print(string.format('PLAY05_HUB_LOAD case=%s weapons=[%s] weapon_weight=%.0f player_health=%.0f/%.0f', case().id,
        table.concat(names, ', '), weight, health.current, health.base))
end
local function startFight()
    if fighting or not opponent or not opponent:isValid() then return end
    fighting = true
    opponent:sendEvent('manualHubFight', true)
    ragdoll.setCombatTarget(opponent, player())
    note = ''
end
local function foeKey(f)
    return (f.weapon or 'fists') .. '|' .. table.concat(f.equip, ',') .. '|' .. (f.armor or '')
end
local function foeOf(c)
    local L = loadout(c)
    return { weapon = L.foe, equip = L.foeEquip, armor = c.armor, title = c.title }
end
local function nextFoe()
    if not foeCycle then
        foeCycle = {}
        local seen = {}
        for _, c in ipairs(cases) do
            if c.kind == 'duel' then
                local f = foeOf(c)
                local key = foeKey(f)
                if not seen[key] then
                    seen[key] = true
                    foeCycle[#foeCycle + 1] = f
                end
            end
        end
    end
    foeIndex = foeIndex % #foeCycle + 1
    return foeCycle[foeIndex]
end
local function clearCorpses()
    for _, body in ipairs(corpses) do
        if body:isValid() then
            ragdoll.setPowered(body, false)
            body:remove()
        end
    end
    corpses = {}
end
local function begin()
    clearCorpses()
    kills, respawning = 0, false
    foe = nil
    fighting, loadLogged = false, false
    autoShot, autoClicks = false, 0
    I.RagdollDev.autoDuel(false)
    removeOpponent()
    if player() then takeBackPlayerItems() end
    player():sendEvent('manualHubReset', { health = config.playerHealth })
    if types.Actor.isDead(player()) then ragdoll.resurrect(player()) end
    ragdoll.clearLimbDamage()
    ragdoll.setRecoveryDirection(nil)
    I.RagdollDev.playerRelease()
    I.RagdollDev.cleanup()
    -- Destruction, equip and animation changes are deferred engine actions.
    -- Drain them before asking the next scenario to create/power new bodies.
    resetAt = core.getSimulationTime() + 0.5
    phase, deadline, remaining, actions, note = 'resetting', core.getRealTime() + 20, config.seconds, 0, ''
    publish(true)
end
local function advance()
    index = index % #cases + 1
    if index == 1 then
        circuit = circuit + 1
        print('PLAY05_HUB circuit complete; observations are not a pass verdict')
        if config.checkCircuits and circuit > config.checkCircuits then core.quit(); return end
    end
    begin()
end
local function ready()
    phase, nextAction = 'ready', core.getSimulationTime() + 2
    publish(true)
end
local function spawnOpponent()
    if not record then
        record = world.createRecord(types.NPC.createRecordDraft({ template = types.NPC.record(player()),
            name = 'Manual circuit opponent', isEssential = false, isPersistent = false, isRespawning = false }))
    end
    opponent = world.createObject(record.id)
    foe = foe or foeOf(case())
    for _, id in pairs({ weapon = foe.weapon, armor = foe.armor }) do
        world.createObject(id, 1):moveInto(types.Actor.inventory(opponent))
    end
    for _, id in ipairs(foe.equip) do world.createObject(id, 1):moveInto(types.Actor.inventory(opponent)) end
    opponent:addScript('manual_hub_opponent.lua', { weapon = foe.weapon, unarmed = not foe.weapon,
        armor = foe.armor, equip = foe.equip, health = config.opponentHealth or 40 })
    if respawning then
        -- The next duelist comes in ahead of the player, facing them.
        local yaw = player().rotation:getYaw()
        local ahead = util.vector3(math.sin(yaw), math.cos(yaw), 0) * 200
        opponent:teleport(player().cell, player().position + ahead,
            { onGround = true, rotation = util.transform.rotateZ(yaw + math.pi) })
    else
        -- Fixed, separated placement in the one test cell; no camera/input steering.
        -- Relative to the player (the range may stand in Balmora, not at the origin).
        opponent:teleport(player().cell, player().position + util.vector3(0, -180, 0),
            { onGround = true, rotation = util.transform.rotateZ(math.pi) })
    end
    phase = 'opponent'
end
local function action()
    local h, c = I.RagdollDev, case()
    if phase ~= 'ready' then return end
    actions = actions + 1
    if c.kind == 'duel' then
        startFight()
    elseif c.kind == 'balance' then
        if standing(h.sample('powered')) then h.push('powered', ({ 'x', '-x', 'y', '-y' })[(actions - 1) % 4 + 1]) end
    elseif c.kind == 'recovery' then
        if standing(h.sample('powered')) then h.fall('powered'); h.recover('powered') end
    elseif c.kind == 'passive' then
        local actor = h.sample('passive')
        if types.Actor.isDead(actor) then h.resurrect(actor) else h.kill(actor) end
    elseif c.kind == 'guard' then
        local facing = player().position - opponent.position
        if facing:length() > 0 then facing = facing / facing:length() end
        ragdoll.setGuard(opponent, true, facing)
        note = 'Guard raised. Watch the actual contact result below.'
    elseif c.id == 'release' then
        ragdoll.releaseWeapon(player(), 'unequip')
        note = 'Physical grip released; F7 reattaches through a fresh setup.'
    elseif c.kind == 'ai' then
        aiMode = aiMode % 3 + 1
        h.duelLoadout(({ 'fist', 'dagger', 'mixed' })[aiMode])
    end
    publish(true)
end
-- Self-check only (-CheckCircuits): in a running duel the player clicks an
-- attack every 1.2 s at the opponent, so unattended runs also cover blows on
-- the opponent (RXN on hub opponents, 2026-10-01).
local function beam(data)
    if not data.target or not data.target:isValid() or not data.point then return end
    local ok, err = pcall(function()
        ragdoll.applyCombatImpulse(data.target, data.direction * data.strength, data.point)
    end)
    ragdoll.spawnSparks(data.point, -data.direction, 900)
    print(string.format('PLAY05_BEAM target=%s strength=%d applied=%s %s', tostring(data.target.recordId),
        data.strength, tostring(ok), ok and '' or tostring(err)))
end
local function autoClick(now)
    if not config.checkCircuits or not fighting or not opponent or not opponent:isValid() then return end
    if autoClickDown then
        ragdoll.injectMouseInput(0, 0, false)
        autoClickDown = false
    elseif now >= autoClickAt then
        -- One screenshot per duel, mid-swing (userdata/screenshots).
        if not autoShot and now > autoClickAt + 0.0 and autoClicks == 2 then
            ragdoll.captureScreenshot(); autoShot = true
        end
        autoClicks = autoClicks + 1
        -- One heavy beam at the opponent's chest per duel (self-check of K).
        if autoClicks == 4 then
            local to = opponent.position - player().position
            local dir = util.vector3(to.x, to.y, 0):normalize()
            beam({ target = opponent, point = opponent.position + util.vector3(0, 0, 100), direction = dir,
                strength = 30000 })
        end
        local to = opponent.position - player().position
        player():teleport(player().cell, player().position,
            { rotation = util.transform.rotateZ(math.atan2(to.x, to.y)) })
        ragdoll.injectMouseInput(0, 0, true)
        autoClickDown, autoClickAt = true, now + 1.2
    end
end
-- DUNGEON (user 10-10: "spawn me in front of a dungeon so I can clear 'real'
-- opponents"; launcher -Only dungeon): no circuit and no hub opponent. The
-- player is put before the door of a real dungeon (config.dungeon, by default
-- Addamasartus, the smugglers' cave by Seyda Neen: armed humans), powered,
-- with a longsword, a claymore and a warhammer (switch in the inventory) and
-- the launcher's health; the dungeon's own NPCs are powered as they come near.
local dungeonPhase, dungeonAt = 'find', 0
local dungeonWeapons = { 'iron longsword', 'iron claymore', 'iron warhammer' }
-- Characters (user 10-10: "add the level 2 warrior option and give me a level
-- 10 dungeon option"): -Character strong (the hub's maxed character, no hit
-- roll - as before), level2 or level10 (a warrior of that level in vanilla
-- terms, with vanilla's hit roll).
local characters = {
    level2 = { level = 2, health = 60, material = 'iron',
        weapons = { 'iron longsword', 'iron claymore', 'iron warhammer' },
        attributes = { strength = 55, intelligence = 30, willpower = 40, agility = 45, speed = 40,
            endurance = 55, personality = 30, luck = 40 },
        skills = { longblade = 40, bluntweapon = 35, axe = 35, block = 35, heavyarmor = 30, mediumarmor = 30,
            athletics = 30, armorer = 25, spear = 20, shortblade = 20, handtohand = 20 } },
    level10 = { level = 10, health = 130, material = 'steel',
        weapons = { 'steel longsword', 'steel claymore', 'steel warhammer' },
        attributes = { strength = 75, intelligence = 35, willpower = 50, agility = 55, speed = 50,
            endurance = 70, personality = 35, luck = 45 },
        skills = { longblade = 65, bluntweapon = 55, axe = 55, block = 50, heavyarmor = 55, mediumarmor = 40,
            athletics = 45, armorer = 40, spear = 30, shortblade = 30, handtohand = 30 } },
}
local character = characters[config.character or '']
if character then
    for i, id in ipairs(character.weapons) do
        character.weapons[i] = items.weapon(id, dungeonWeapons[i])
    end
    dungeonWeapons = character.weapons
end
-- The exterior spot just outside the dungeon: its own exit door's
-- destination (in front of the entrance, facing out).
local function dungeonExit(name)
    local ok, cell = pcall(world.getCellByName, name)
    if not ok or not cell then return nil end
    for _, door in ipairs(cell:getAll(types.Door)) do
        if types.Door.isTeleport(door) then
            local dest = types.Door.destCell(door)
            if dest and dest.isExterior then
                return dest, types.Door.destPosition(door), types.Door.destRotation(door)
            end
        end
    end
end
-- -Survey "A;B;C": per dungeon, its first cell's NPCs and creatures (count,
-- levels, health) - to pick dungeons by level from the game data.
local function survey(list)
    for name in string.gmatch(list, '[^;]+') do
        name = name:gsub('_', ' ')
        local ok, cell = pcall(world.getCellByName, name)
        if not ok or not cell then
            print('PLAY05_HUB SURVEY ' .. name .. ' missing')
        else
            local function describe(kind, objects)
                local levels, health, names = {}, 0, {}
                for _, o in ipairs(objects) do
                    if not types.Actor.isDead(o) then
                        levels[#levels + 1] = types.Actor.stats.level(o).current
                        health = health + types.Actor.stats.dynamic.health(o).base
                        names[#names + 1] = (o.type.record(o).name or '?')
                    end
                end
                table.sort(levels)
                return string.format('%s %d levels [%s] health avg %.0f (%s)', kind, #levels,
                    table.concat(levels, ','), #levels > 0 and health / #levels or 0,
                    table.concat(names, ', '):sub(1, 160))
            end
            print('PLAY05_HUB SURVEY ' .. name .. ' | ' .. describe('NPC', cell:getAll(types.NPC)) .. ' | '
                .. describe('creature', cell:getAll(types.Creature)))
        end
    end
end
local function dungeonDoor(name)
    local wanted = name:lower()
    for x = -6, 6 do
        for y = -12, 6 do
            local ok, cell = pcall(world.getExteriorCell, x, y)
            if ok and cell then
                for _, door in ipairs(cell:getAll(types.Door)) do
                    if types.Door.isTeleport(door) then
                        local dest = types.Door.destCell(door)
                        if dest and dest.name and dest.name:lower() == wanted then return door end
                    end
                end
            end
        end
    end
end
local function updateDungeon(now)
    local name = config.dungeon or 'Addamasartus'
    if dungeonPhase == 'find' then
        -- The range's own background duel (two duelists by the player) and
        -- its samples stay off: the dungeon's NPCs are the fight.
        I.RagdollDev.autoDuel(false)
        I.RagdollDev.cleanup()
        if config.survey then survey(config.survey) end
        local exitCell, exitPosition, exitRotation = dungeonExit(name)
        local door = not exitCell and dungeonDoor(name)
        if exitCell then
            -- The exit faces out of the dungeon: turned round, the entrance is ahead.
            player():teleport(exitCell, exitPosition, { onGround = true,
                rotation = exitRotation * util.transform.rotateZ(math.pi) })
            print(string.format('PLAY05_HUB DUNGEON outside %s at %.0f,%.0f', name, exitPosition.x, exitPosition.y))
        elseif door then
            -- In front of the door: out along its facing, on the ground.
            local forward = door.rotation:apply(util.vector3(0, 1, 0))
            local spot = door.position + util.vector3(forward.x, forward.y, 0):normalize() * 220
            player():teleport(door.cell, spot, { onGround = true,
                rotation = util.transform.rotateZ(math.atan2(-forward.x, -forward.y)) })
            print(string.format('PLAY05_HUB DUNGEON door of %s at %.0f,%.0f', name, door.position.x, door.position.y))
        else
            player():teleport(name, util.vector3(0, 0, 0), { onGround = true })
            print('PLAY05_HUB DUNGEON no exterior door found for ' .. name .. '; put inside')
        end
        dungeonPhase, dungeonAt = 'arm', now + 1.5
    elseif dungeonPhase == 'arm' and now >= dungeonAt then
        if character then
            -- The hub character's own test weapons go; the preset's come.
            for _, id in ipairs(dungeonWeapons) do givePlayer(id) end
        end
        player():sendEvent('manualHubReset', { health = character and character.health or config.playerHealth })
        if character then player():sendEvent('manualHubCharacter', character) end
        ragdoll.setAutomaticRecovery(true)
        -- A preset character rolls to hit like in vanilla; the strong one as the hub.
        ragdoll.setPlayerAlwaysHits(not character
            and not (ragdoll.playerRollWanted and ragdoll.playerRollWanted()))
        I.RagdollDev.playerFists(dungeonWeapons[1])
        print('PLAY05_HUB DUNGEON character ' .. (config.character or 'strong'))
        dungeonPhase, dungeonAt = 'lighten', now + 1.0
    elseif dungeonPhase == 'lighten' and now >= dungeonAt then
        -- playerFists hands over every test weapon: keep the three.
        local inventory = types.Actor.inventory(player())
        for _, id in ipairs(hubWeapons) do
            local keep = false
            for _, k in ipairs(dungeonWeapons) do keep = keep or k == id end
            if not keep then
                for _, item in ipairs(inventory:findAll(id)) do item:remove() end
            end
        end
        for _, id in ipairs(dungeonWeapons) do givePlayer(id) end
        if character then
            local armor = items.armorSet(character.material, false)
            for _, id in ipairs(armor) do givePlayer(id) end
            player():sendEvent('manualHubEquip', { ids = armor, weapon = dungeonWeapons[1] })
        end
        dungeonPhase = 'ready'
        player():sendEvent('manualHubStatus', { text = 'DUNGEON: ' .. name .. ' ahead. Clear it.',
            short = 'DUNGEON: ' .. name .. ' - clear it (weapons in the inventory, I)', announce = true,
            compact = true, enemy = '', detail = '' })
        print(string.format('PLAY05_HUB DUNGEON ready: %s player at %.0f,%.0f,%.0f', name,
            player().position.x, player().position.y, player().position.z))
        dungeonAt = now + 3
    elseif dungeonPhase == 'ready' and dungeonAt > 0 and now >= dungeonAt then
        dungeonAt = 0
        if config.checkCircuits and ragdoll.captureScreenshot then ragdoll.captureScreenshot() end
    end
end
-- Aim indicator feed (every mode, the dungeon too): the tip controller's
-- goal and the real tip, hand control's stance and the guard for the stance
-- panel and the reticle.
local function feedAim()
    local ps = state(player())
    if ps then
        local tip, blade
        if ps.weapon and ps.weapon.bodyIndex and ps.bodies[ps.weapon.bodyIndex] then
            local b = ps.bodies[ps.weapon.bodyIndex]
            blade = b.rotation * util.vector3(0, 0, 1)
            tip = b.position + blade * (ps.weapon.length * 0.5)
        else
            for _, b in ipairs(ps.bodies) do
                if b.name == 'right_hand' then tip = b.position end
            end
        end
        -- The reticle (kinetic_reticle.lua): what hand control read
        -- from the mouse, and the commanded and physical blade.
        local hud
        local hsc = ps.hsc
        if hsc then
            hud = { stage = hsc.stage, x = hsc.stanceX, y = hsc.stanceY, angle = hsc.angle, line = hsc.line,
                snap = hsc.snap, chain = hsc.chain, click = hsc.click, route = hsc.route,
                reseat = hsc.reseat, reseats = hsc.reseats,
                bladeCommand = hsc.blade, blade = blade, trajectory = hsc.trajectory }
        end
        player():sendEvent('manualHubAim', { command = ps.tip and ps.tip.command or nil, tip = tip, hud = hud,
            wristRight = ps.wristOutsideRight, wristLeft = ps.wristOutsideLeft,
            guard = ps.guardActive, guardZone = ps.guardZone,
            balance = { x = ps.balanceRight or 0, y = ps.balanceForward or 0, tip = ps.balanceTip or 0 },
            ready = ps.attackReady or 1 })
    end
end
local function update(dt)
    if not I.RagdollDev or not player() then return end
    feedAim()
    if config.only == 'dungeon' then return updateDungeon(core.getSimulationTime()) end
    local h, c, now = I.RagdollDev, case(), core.getSimulationTime()
    autoClick(now)
    if phase == 'init' then begin(); return end
    if phase == 'resetting' and now >= resetAt then
        h.setup(); phase = 'setup'
    elseif phase == 'setup' and h.samples().defense then
        -- setup() disables automatic recovery for its diagnostic traces.
        ragdoll.setAutomaticRecovery(true)
        -- setup() stocks the player too; scenarios without a player setup end here.
        lightenPlayer(loadout(c).player)
        if c.kind == 'balance' or c.kind == 'recovery' then
            ragdoll.setRecoveryDirection(c.kind == 'recovery' and circuit % 2 == 1 or nil)
            if c.kind == 'recovery' and circuit % 2 == 0 then ragdoll.setRecoveryDirection(false) end
            h.power('powered'); ready()
        elseif c.kind == 'passive' then h.kill('passive'); ready()
        elseif c.kind == 'ai' then
            h.duelLoadout(({ 'fist', 'dagger', 'mixed' })[aiMode]); h.autoDuel(true); ready()
        else
            -- Keep the four demonstrations out of the player's swing lane.
            h.cleanup()
            local L = loadout(c)
            givePlayer(L.player)
            for _, id in ipairs(L.playerEquip) do givePlayer(id) end
            h.playerFists(L.player)
            if #L.playerEquip > 0 then player():sendEvent('manualHubEquip', { ids = L.playerEquip, weapon = L.player }) end
            phase = 'player'
        end
    elseif phase == 'player' then
        local s = state(player())
        if s and s.powered and (not loadout(c).player or (s.weapon and s.weapon.attached)) then
            lightenPlayer(loadout(c).player)
            if c.kind == 'target' or c.kind == 'guard' or c.kind == 'duel' then spawnOpponent() else ready() end
        end
    elseif phase == 'opponent' then
        local item = types.Actor.getEquipment(opponent, types.Actor.EQUIPMENT_SLOT.CarriedRight)
        local armor = types.Actor.getEquipment(opponent, types.Actor.EQUIPMENT_SLOT.Cuirass)
        if types.Actor.getStance(opponent) == types.Actor.STANCE.Weapon
            and ((not foe.weapon and not item) or (item and item.recordId == foe.weapon))
            and (not foe.armor or (armor and armor.recordId == foe.armor)) then
            if foe.weapon then ragdoll.prepareEquippedWeapon(opponent) end
            ragdoll.setPowered(opponent, true)
            phase = 'bodies'
        end
    elseif phase == 'bodies' and state(opponent) then
        local again = respawning
        if c.kind == 'duel' and (not manualStart or again) then startFight() end
        respawning = false
        ready()
        if c.kind == 'duel' and manualStart and not again then
            note = outcome .. loadout(c).text .. '\nOpponent waits. F10 starts the fight.'
            outcome = ''
            publish(false)
        end
    elseif phase == 'ready' then
        if not loadLogged and now >= nextAction - 1 then loadLogged = true; logPlayerLoad() end
        if not paused then remaining = remaining - dt end
        if remaining <= 0 then advance(); return end
        if types.Actor.isDead(player()) then
            note = 'Player died; restaging this observation.'; publish(true)
            outcome = 'You died. '; begin(); return
        end
        if opponent and types.Actor.isDead(opponent) and c.kind == 'duel' then
            kills = kills + 1
            print('PLAY05_HUB opponent died; next duelist (' .. kills .. ' killed)')
            outcome = 'Opponent killed (' .. kills .. '). Next one coming. '
            opponent:sendEvent('manualHubFight', false)
            corpses[#corpses + 1] = opponent
            while #corpses > maxCorpses do
                local oldest = table.remove(corpses, 1)
                if oldest:isValid() then
                    ragdoll.setPowered(oldest, false)
                    oldest:remove()
                end
            end
            opponent, fighting, respawning = nil, false, true
            deadline = core.getRealTime() + 20
            local current = foeKey(foe)
            repeat foe = nextFoe() until foeKey(foe) ~= current or #foeCycle < 2
            outcome = outcome .. 'Foe: ' .. (foe.weapon or 'fists') .. ' (' .. foe.title .. '). '
            print('PLAY05_HUB next duelist ' .. (foe.weapon or 'fists') .. ' [' .. table.concat(foe.equip, ',') .. ']')
            spawnOpponent()
            note = outcome
            publish(true)
            return
        end
        if opponent and types.Actor.isDead(opponent) then
            -- Restage without consuming the remaining observation window.
            print('PLAY05_HUB opponent died; restaging ' .. c.id)
            outcome = 'Opponent killed. '
            local saved = remaining; begin(); remaining = saved; return
        end
        if now >= nextAction then
            if c.kind == 'balance' or c.kind == 'recovery' then action() end
            nextAction = now + (c.kind == 'recovery' and 15 or 7)
        end
        if opponent and (fighting or c.kind ~= 'duel') then
            local s, p = state(opponent), state(player())
            local w = p and p.weapon
            note = string.format('%sYou HP %.0f | Opponent HP %.1f | fatigue %.0f | wound %s %s | defense %s | damage %.2f',
                kills > 0 and ('Killed ' .. kills .. ' | ') or '', types.Actor.stats.dynamic.health(player()).current,
                types.Actor.stats.dynamic.health(opponent).current, types.Actor.stats.dynamic.fatigue(opponent).current,
                s and s.lastWoundKind or 'none', s and s.lastWoundRegion or '',
                w and w.lastDefenseOutcome or 'none', w and w.lastFinalDamage or 0)
        end
    end
    if phase ~= 'ready' and phase ~= 'failed' and core.getRealTime() > deadline then
        note = 'Setup timed out in ' .. phase .. '. F7 retries; F6 skips. This is not a passed observation.'
        phase, paused = 'failed', true
        publish(true)
        if config.checkCircuits then core.quit() end
    end
    if now - lastHud >= 0.1 then lastHud = now; publish(false) end
end
return {
    engineHandlers = { onUpdate = function(dt)
        local ok, err = pcall(update, dt)
        if not ok then
            phase, paused, note = 'failed', true, tostring(err)
            print('PLAY05_HUB ERROR ' .. note)
            if player() then publish(true) end
            if config.checkCircuits then core.quit() end
        end
    end },
    eventHandlers = {
    -- Force beam (user bookmark 21, 2026-10-01: "spells that shoot a targeted
    -- laser at the target point, with variations for push strengths"): a
    -- combat impulse along the look ray at the struck point, with sparks.
    manualHubBeam = function(data) beam(data) end,
    manualHubAction = function(data)
        if config.only == 'dungeon' then return end
        outcome = ''
        if data.action == 'next' then advance()
        elseif data.action == 'repeat' then begin()
        elseif data.action == 'pause' then paused = not paused; publish(true)
        elseif data.action == 'action' then action()
        elseif data.action == 'jump' and cases[data.index] then index = data.index; begin() end
    end,
    },
}

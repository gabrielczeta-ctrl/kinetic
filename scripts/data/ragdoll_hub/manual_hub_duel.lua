-- The duel builder (F8 in the duel hub, user 11-05: "a duel menu, like select
-- your enemy duelist and select the weapons, so I can test both low level and
-- high level play"). Pure data shared by the global hub (manual_hub.lua) and
-- the player's menu (ragdoll_manual_player.lua); no world access here.
local duel = {}

-- Opponents are real game NPC records: the hub spawns a copy of the record
-- (its race, face, stats and name) without its own gear, then gives it the
-- chosen loadout. Guard-class records fight as Warriors so the copy never
-- starts vanilla's arrest dialogue (their stats are fixed in the record, so
-- the class changes nothing else).
duel.opponents = {
    { id = 'rostlogi', label = 'Rostlogi, commoner (level 2)',
      about = 'A Nord commoner: autocalculated Commoner stats, untrained.' },
    { id = 'a shady smuggler', label = 'Shady Smuggler (level 3)',
      about = 'A generic smuggler (bandit): autocalculated Smuggler stats.' },
    { id = 'hlaalu guard', label = 'Hlaalu Guard (level 20)', class = 'Warrior',
      about = "Balmora's guard: Long Blade 68, Block 64, 210 health." },
    { id = 'todd', label = "Todd's Super Tester Guy (level 35)", class = 'Warrior',
      about = "The original developers' test fighter (only in their test cell): Long Blade 91, Block 86, 304 health." },
    { id = 'skaal_guard', label = 'Skaal Honor Guard (level 50)', class = 'Warrior',
      about = 'A Solstheim village guard (Bloodmoon): every weapon skill and Block 100, 394 health.' },
}

-- Weapon families: the game record is '<material> <family>' (falling back to
-- steel, then iron, when the material has no such weapon); staff in iron is
-- the wooden staff. skill is the NPC skill the weapon trains; a two-handed
-- weapon takes no shield.
duel.weapons = {
    { key = 'longsword', label = 'Longsword', family = 'longsword', skill = 'longblade' },
    { key = 'claymore', twoHanded = true, label = 'Claymore (two-handed)', family = 'claymore', skill = 'longblade' },
    { key = 'waraxe', label = 'War axe', family = 'war axe', skill = 'axe' },
    { key = 'battleaxe', twoHanded = true, label = 'Battle axe (two-handed)', family = 'battle axe', skill = 'axe' },
    { key = 'mace', label = 'Mace', family = 'mace', skill = 'bluntweapon' },
    { key = 'warhammer', twoHanded = true, label = 'Warhammer (two-handed)', family = 'warhammer', skill = 'bluntweapon' },
    { key = 'dagger', label = 'Dagger', family = 'dagger', skill = 'shortblade' },
    { key = 'spear', twoHanded = true, label = 'Spear', family = 'spear', skill = 'spear' },
    { key = 'staff', twoHanded = true, label = 'Staff', family = 'staff', skill = 'bluntweapon' },
    { key = 'fists', label = 'Fists', skill = 'handtohand' },
}
duel.materials = { 'iron', 'steel', 'glass', 'ebony', 'daedric' }
duel.shields = {
    { key = 'none', label = 'none' }, { key = 'both', label = 'both' },
    { key = 'player', label = 'you only' }, { key = 'foe', label = 'opponent only' },
}
duel.armors = {
    { key = 'none', label = 'none' }, { key = 'both', label = 'both (weapon material)' },
    { key = 'player', label = 'you only' }, { key = 'foe', label = 'opponent only' },
}
duel.healths = {
    { key = 1, label = 'game values' }, { key = 2, label = 'long fights (x2)' },
    { key = 0.5, label = 'short fights (x0.5)' },
}

-- Skill tiers. low, mid and high are the hub's level 2, level 10 and level 30
-- (veteran) warriors; max is the hub's default character (100s, no hit roll
-- for the player). The tier's best weapon skill is put on the chosen weapon,
-- so a spear or fists duel is as skilled as a longsword one.
duel.tiers = {
    low = { label = 'low (level 2)', level = 2, health = 60,
        attributes = { strength = 55, intelligence = 30, willpower = 40, agility = 45, speed = 40,
            endurance = 55, personality = 30, luck = 40 },
        skills = { longblade = 40, bluntweapon = 35, axe = 35, block = 35, heavyarmor = 30, mediumarmor = 30,
            athletics = 30, armorer = 25, spear = 20, shortblade = 20, handtohand = 20 } },
    mid = { label = 'mid (level 10)', level = 10, health = 130,
        attributes = { strength = 75, intelligence = 35, willpower = 50, agility = 55, speed = 50,
            endurance = 70, personality = 35, luck = 45 },
        skills = { longblade = 65, bluntweapon = 55, axe = 55, block = 50, heavyarmor = 55, mediumarmor = 40,
            athletics = 45, armorer = 40, spear = 30, shortblade = 30, handtohand = 30 } },
    high = { label = 'high (level 30)', level = 30, health = 320,
        attributes = { strength = 100, intelligence = 40, willpower = 70, agility = 90, speed = 80,
            endurance = 100, personality = 40, luck = 60 },
        skills = { longblade = 90, bluntweapon = 80, axe = 85, block = 80, heavyarmor = 85, mediumarmor = 60,
            athletics = 70, armorer = 60, spear = 50, shortblade = 50, handtohand = 50 } },
    -- Speed and Athletics are left as they are: at 100 the vanilla run
    -- outpaces the tuned gait (the hub's own maxStats keeps them too).
    max = { label = 'max (100s)', level = 50, health = 250,
        attributes = { strength = 100, intelligence = 100, willpower = 100, agility = 100,
            endurance = 100, personality = 100, luck = 100 },
        skills = { longblade = 100, bluntweapon = 100, axe = 100, block = 100, heavyarmor = 100, mediumarmor = 100,
            lightarmor = 100, armorer = 100, spear = 100, shortblade = 100, handtohand = 100, marksman = 100 } },
}
duel.playerTiers = { 'low', 'mid', 'high', 'max' }
-- 'own' keeps the opponent's game record (its best weapon skill moved to the
-- chosen weapon); the others replace its stats with the tier.
duel.foeTiers = { 'own', 'low', 'mid', 'high', 'max' }
duel.meleeSkills = { 'longblade', 'shortblade', 'bluntweapon', 'axe', 'spear', 'handtohand' }

-- The menu's starting selection (indices into the lists above, tiers by key).
function duel.defaults()
    return { opponent = 3, foeTier = 'own', playerTier = 'max', weapon = 1, foeWeapon = 0, material = 2,
        shield = 1, armor = 1, health = 1 }
end

function duel.weaponOf(selection, foe)
    local index = foe and selection.foeWeapon ~= 0 and selection.foeWeapon or selection.weapon
    return duel.weapons[index] or duel.weapons[1]
end

-- One line per setting, for the menu and the HUD.
function duel.describe(s)
    local foe = duel.opponents[s.opponent] or duel.opponents[1]
    local mine, theirs = duel.weaponOf(s, false), duel.weaponOf(s, true)
    local foeTier = s.foeTier == 'own' and 'own record' or duel.tiers[s.foeTier].label
    local function name(w) return w.family and (duel.materials[s.material] .. ' ' .. w.label:lower()) or 'fists' end
    return string.format('%s, %s | you %s | %s%s', foe.label, foeTier, duel.tiers[s.playerTier].label,
        name(mine), theirs ~= mine and (' vs ' .. name(theirs)) or '')
end

-- A launcher spec ("opponent=skaal_guard,weapon=claymore,material=ebony,
-- you=high,foe=own,shield=both,armor=none,health=2") as a selection.
function duel.parse(spec)
    local s = duel.defaults()
    local function find(list, value, field)
        for i, item in ipairs(list) do
            if (field and item[field] or item) == value then return i end
        end
    end
    for key, value in string.gmatch(spec or '', '([%w_]+)=([^,]+)') do
        value = value:gsub('_', ' ')
        if key == 'opponent' then s.opponent = find(duel.opponents, value, 'id')
            or find(duel.opponents, (value:gsub(' ', '_')), 'id') or s.opponent
        elseif key == 'weapon' then s.weapon = find(duel.weapons, value, 'key') or s.weapon
        elseif key == 'foeweapon' then s.foeWeapon = find(duel.weapons, value, 'key') or 0
        elseif key == 'material' then s.material = find(duel.materials, value) or s.material
        elseif key == 'you' and duel.tiers[value] then s.playerTier = value
        elseif key == 'foe' and (value == 'own' or duel.tiers[value]) then s.foeTier = value
        elseif key == 'shield' then s.shield = find(duel.shields, value, 'key') or s.shield
        elseif key == 'armor' then s.armor = find(duel.armors, value, 'key') or s.armor
        elseif key == 'health' then s.health = find(duel.healths, tonumber(value), 'key') or s.health
        end
    end
    return s
end

return duel

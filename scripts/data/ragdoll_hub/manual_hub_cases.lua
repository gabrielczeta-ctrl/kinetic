-- PLAY-05: manual observation prompts, not assertions or acceptance results.
local cases = {
    { id = 'movement', title = 'Unarmed movement', kind = 'player',
      hint = 'Walk, stop, turn, strafe, reverse and punch while moving. Watch feet and body heading.' },
    { id = 'sword', title = 'One-handed longsword handling', kind = 'player', weapon = 'iron longsword',
      hint = 'Walk and turn. Quick-click for one native attack; hold through the threshold, then drag to aim the physical blade.' },
    { id = 'claymore', title = 'Two-handed claymore handling', kind = 'player', weapon = 'iron claymore',
      hint = 'Compare quick-click animation with held mouse-drag control. Check both hands, blade aim, return and movement.' },
    { id = 'hammer', title = 'Warhammer movement and handling', kind = 'player', weapon = 'iron warhammer',
      hint = 'Quick-click, then hold and drag. Check slower authored speed, two-hand grip, recovery, frozen feet and trips.' },
    { id = 'staff', title = 'Two-handed staff handling', kind = 'player', weapon = 'wooden staff',
      hint = 'Quick-click, then hold and drag. Check the original faster speed, long shape, both grips and clean recovery.' },
    { id = 'spear', title = 'Two-handed spear handling', kind = 'player', weapon = 'iron spear',
      hint = 'Use mouse drag to aim the long point. Check reach, both hands, camera ownership after the threshold and misses.' },
    { id = 'battle-axe', title = 'Two-handed battle axe handling', kind = 'player', weapon = 'iron battle axe',
      hint = 'Compare native quick attacks with aimed physical drags. Check edge path, grip, locomotion and recovery.' },
    { id = 'balance', title = 'Standing and disturbance', kind = 'balance',
      hint = 'Watch sample A receive small alternating pushes. F10 repeats a push. Watch feet, slipping and settling.' },
    { id = 'recovery', title = 'Fall and complete get-up', kind = 'recovery',
      hint = 'Watch A fall, brace and stand. F10 repeats when standing. Face-up/down alternate each circuit.' },
    { id = 'passive', title = 'Death, settling, grab and resurrection', kind = 'passive',
      hint = 'Sample B falls. G grabs/releases the body under the crosshair. F10 resurrects or kills B again.' },
    { id = 'contact', title = 'Weapon contact and wounds', kind = 'target', weapon = 'iron longsword',
      hint = 'Approach the idle target. Compare edge, tip, flat and glancing contacts, misses and repeated draws.' },
    { id = 'guard', title = 'Timed guard, block and parry', kind = 'guard', weapon = 'iron warhammer',
      hint = 'F10 raises the guard: <=0.15s parries, <=0.60s blocks, later contact hits. Use your own swing.' },
    { id = 'dagger-duel', title = 'Dagger duel', kind = 'duel', weapon = 'iron dagger',
      hint = 'Fight the NPC: drag to cut; middle mouse to stab. Check reactions, wounds and hit location.' },
    { id = 'fist-duel', title = 'Unarmed duel', kind = 'duel',
      hint = 'Fists on the wheel: click = cross; hold and drag to pick a punch (down cross, up overhand, right hook, low diagonals uppercut and body), then flick back to throw it.' },
    { id = 'sword-duel', title = 'Longsword duel', kind = 'duel', weapon = 'iron longsword',
      hint = 'Check approach, spacing, cuts, thrusts, flat contacts and recovery while both fighters move.' },
    { id = 'armor', title = 'Existing armor and condition rules', kind = 'duel', weapon = 'iron longsword', armor = 'iron_cuirass',
      hint = 'Same opponent with an iron cuirass. Compare covered/exposed hits; inspect equipment condition in inventory.' },
    -- WPN-06: powered opponents with the two-handed families. The opponent carries
    -- the same weapon; its support hand follows the animation (only the player has
    -- the passive second grip).
    { id = 'claymore-duel', title = 'Claymore duel', kind = 'duel', weapon = 'iron claymore',
      hint = 'Both fighters hold a claymore. Quick-click to strike, hold and drag to aim. Watch both grips, reach and reactions.' },
    { id = 'staff-duel', title = 'Staff duel', kind = 'duel', weapon = 'wooden staff',
      hint = 'Fast, long and light. Check spacing, the wide grip under contact and recovery after a blocked swing.' },
    { id = 'spear-duel', title = 'Spear duel', kind = 'duel', weapon = 'iron spear',
      hint = 'Thrusts at reach. Check the point lands where aimed and the support hand stays on the shaft.' },
    { id = 'hammer-duel', title = 'Warhammer duel', kind = 'duel', weapon = 'iron warhammer',
      hint = 'Slow heavy blows. Check staggers, knockdowns and whether either fighter loses the weapon.' },
    { id = 'axe-duel', title = 'Battle axe duel', kind = 'duel', weapon = 'iron battle axe',
      hint = 'Edge path and follow-through against a moving opponent. Check wounds and hit location.' },
    -- Equipment variety (2026-10-01, user: "different armors? weapons?
    -- enchanted weapons?"). Armor sets, shields and the enchanted weapon are
    -- resolved from the game's records by material (manual_hub_items.lua);
    -- a weapon that does not exist falls back to iron (the HUD says so).
    { id = 'steel-duel', title = 'Steel longswords, foe in steel armor', kind = 'duel',
      weapon = 'steel longsword', foeArmor = 'steel',
      hint = 'Full steel set on the foe. Compare blows on plate and on the gaps; watch damage and armor condition.' },
    { id = 'glass-duel', title = 'Glass longswords, foe in glass armor', kind = 'duel',
      weapon = 'glass longsword', foeArmor = 'glass',
      hint = 'Light, sharp glass against glass armor. Watch damage per blow and how fast fights end.' },
    { id = 'daedric-duel', title = 'Daedric longswords, foe in daedric armor', kind = 'duel',
      weapon = 'daedric longsword', foeArmor = 'daedric',
      hint = 'The heaviest blades and plate. Check swing weight, staggers and armor soaking blows.' },
    { id = 'leather-duel', title = 'Foe in light netch leather', kind = 'duel',
      weapon = 'iron longsword', foeArmor = 'netch_leather',
      hint = 'Light armor: blows should bite more than on plate. Compare with the steel scenario.' },
    { id = 'shield-duel', title = 'Sword and shield, both sides', kind = 'duel',
      weapon = 'iron longsword', playerShield = 'iron', foeShield = 'iron',
      hint = 'Both carry an iron shield. C guards; blows on the plate bounce. Check the left arm carrying the weight.' },
    { id = 'enchanted-duel', title = 'Enchanted longsword (on strike)', kind = 'duel',
      weapon = 'iron longsword', playerEnchanted = 'longblade', foeArmor = 'iron',
      hint = 'Your blade casts its enchantment on each physical hit. Watch the spell effect and charge in inventory.' },
    { id = 'armored-player', title = 'You in iron armor vs warhammer', kind = 'duel',
      weapon = 'iron longsword', foeWeapon = 'iron warhammer', playerArmor = 'iron',
      hint = 'Your body carries a full iron set. Check movement, balance and the blows your armor takes.' },
    { id = 'dagger-vs-shield', title = 'Dagger vs sword and shield', kind = 'duel',
      weapon = 'iron dagger', foeWeapon = 'iron longsword', foeShield = 'steel',
      hint = 'Fast short blade against a shield. Find the gaps around the plate; stab with middle mouse.' },
    { id = 'claymore-vs-dagger', title = 'Claymore vs dagger', kind = 'duel',
      weapon = 'iron claymore', foeWeapon = 'iron dagger',
      hint = 'Reach against speed. Keep the dagger out with long swings; watch recovery when it closes in.' },
    { id = 'fists-vs-armor', title = 'Fists vs an armored brawler', kind = 'duel', foeArmor = 'iron',
      hint = 'Both unarmed, the foe in iron plate. Punches on plate should do little; aim for the head.' },
    { id = 'release', title = 'Weapon release and reattachment', kind = 'player', weapon = 'iron longsword',
      hint = 'F10 releases the physical grip. F7 restages and reattaches. Check lingering contacts or stale hand control.' },
    { id = 'ai', title = 'Repeating AI duels', kind = 'ai',
      hint = 'Observe the pair approach, attack, react and restage. F10 cycles fists / daggers / mixed.' },
}
-- A launcher may restrict the circuit to one kind of scenario (only = 'duel').
-- Filtered here because the circuit and the F8 menu both index this list.
local config = require('manual_hub_config')
if config.only then
    local selected = {}
    for _, item in ipairs(cases) do
        if item.kind == config.only then selected[#selected + 1] = item end
    end
    if #selected > 0 then return selected end
end
return cases

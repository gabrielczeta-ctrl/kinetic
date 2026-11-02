-- Manual hub equipment: armor sets by material, shields and enchanted weapons,
-- resolved from the loaded game records so a scenario never names an id that
-- does not exist. Global script helper (world/types/core).
local core = require('openmw.core')
local types = require('openmw.types')

local items = {}

local function armorRecords()
    local ok, records = pcall(function() return types.Armor.records end)
    return ok and records or {}
end

local function weaponRecords()
    local ok, records = pcall(function() return types.Weapon.records end)
    return ok and records or {}
end

-- One piece per armor type whose id starts with the material ("iron",
-- "steel", "glass", "daedric", "netch_leather" ...), shield excluded unless
-- asked. Sorted by id so the same set comes back every time.
function items.armorSet(material, withShield)
    local T = types.Armor.TYPE
    local wanted = { [T.Helmet] = true, [T.Cuirass] = true, [T.LPauldron] = true, [T.RPauldron] = true,
        [T.Greaves] = true, [T.Boots] = true, [T.LGauntlet] = true, [T.RGauntlet] = true }
    if withShield then wanted[T.Shield] = true end
    local chosen = {}
    local prefix = material:lower()
    for _, record in ipairs(armorRecords()) do
        local id = record.id:lower()
        if wanted[record.type] and id:sub(1, #prefix) == prefix and not id:find('_uni') then
            local current = chosen[record.type]
            if not current or id < current then chosen[record.type] = id end
        end
    end
    local list = {}
    for _, id in pairs(chosen) do list[#list + 1] = id end
    table.sort(list)
    return list
end

-- A shield of the material, or nil.
function items.shield(material)
    local prefix = material:lower()
    local best
    for _, record in ipairs(armorRecords()) do
        local id = record.id:lower()
        if record.type == types.Armor.TYPE.Shield and id:sub(1, #prefix) == prefix and (not best or id < best) then
            best = id
        end
    end
    return best
end

-- The id if the weapon record exists, else the fallback.
function items.weapon(id, fallback)
    local ok, record = pcall(types.Weapon.record, id)
    if ok and record then return id end
    return fallback
end

-- An on-strike enchanted weapon of the type (types.Weapon.TYPE.*), the
-- cheapest by value so it is a common one, not an artifact.
function items.enchantedWeapon(weaponType)
    local onStrike = core.magic.ENCHANTMENT_TYPE.CastOnStrike
    local best, bestValue
    for _, record in ipairs(weaponRecords()) do
        if record.type == weaponType and record.enchant and record.enchant ~= '' then
            local ok, enchantment = pcall(function() return core.magic.enchantments.records[record.enchant] end)
            if ok and enchantment and enchantment.type == onStrike
                and (not bestValue or record.value < bestValue) then
                best, bestValue = record.id, record.value
            end
        end
    end
    return best
end

-- The equipment slot an item goes to, or nil for anything else.
function items.slot(id)
    local S = types.Actor.EQUIPMENT_SLOT
    local ok, armor = pcall(types.Armor.record, id)
    if ok and armor then
        local T = types.Armor.TYPE
        return ({ [T.Helmet] = S.Helmet, [T.Cuirass] = S.Cuirass, [T.LPauldron] = S.LeftPauldron,
            [T.RPauldron] = S.RightPauldron, [T.Greaves] = S.Greaves, [T.Boots] = S.Boots,
            [T.LGauntlet] = S.LeftGauntlet, [T.RGauntlet] = S.RightGauntlet, [T.Shield] = S.CarriedLeft,
            [T.LBracer] = S.LeftGauntlet, [T.RBracer] = S.RightGauntlet })[armor.type]
    end
    local okWeapon, weapon = pcall(types.Weapon.record, id)
    if okWeapon and weapon then return S.CarriedRight end
    return nil
end

return items

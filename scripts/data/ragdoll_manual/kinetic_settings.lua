-- KINETIC options page (Beta 1.0 settings menu).
-- Switches: the main on/off choices, saved to the KINETIC switches file
-- through openmw.kinetic and applied at the next start. "Default" leaves the
-- switch to the build (or to the launcher's environment, which wins).
-- Live tuning: sliders for the feel numbers. They apply as soon as the game
-- runs again (close the menu) and are saved to kinetic-tuning.cfg, which wins
-- over the launcher at the next start. Reset (the arrow at the right) goes
-- back to the launcher's value.
local async = require('openmw.async')
local I = require('openmw.interfaces')
local kinetic = require('openmw.kinetic')
local storage = require('openmw.storage')

local L10N = 'Kinetic'
local GROUP = 'SettingsKinetic'

-- key -> { env name, { item -> value } }; item names are l10n keys.
local switches = {
    rmbGuard = { 'OPENMW_RAGDOLL_RMB_GUARD', { On = '1', Off = '0' } },
    fallProtect = { 'OPENMW_RAGDOLL_FALL_PROTECT', { On = '1', Off = '0' } },
    balance = { 'OPENMW_RAGDOLL_PGC', { On = '1,1,1,1', Off = '0,0,0,0' } },
    headDaze = { 'OPENMW_RAGDOLL_HEAD_DAZE', { On = '1.2,1.8,12,1', Off = '0' } },
    hitPush = { 'OPENMW_RAGDOLL_REACTION_V2_VIS', { Light = '1', Normal = '1.6', Strong = '2.2' } },
    parryKnock = { 'OPENMW_RAGDOLL_PARRY_KNOCK', { On = '35,0.7,300', Off = '0' } },
    windUp = { 'OPENMW_RAGDOLL_TIP_CHAMBER', { On = '0.68', Off = '0' } },
    sparks = { 'OPENMW_RAGDOLL_SPARKS', { On = '1', Off = '0' } },
    cameraKick = { 'OPENMW_RAGDOLL_CAMERA_KICK', { Off = '0', Small = '1.5', Large = '4' } },
    knockdowns = { 'OPENMW_RAGDOLL_KNOCKDOWN', { Vanilla = '1', Rare = '0.5', Off = '0' } },
}
local order = { 'rmbGuard', 'fallProtect', 'balance', 'headDaze', 'hitPush', 'parryKnock', 'windUp', 'sparks',
    'cameraKick', 'knockdowns' }

local function itemsOf(key)
    local items = { 'Default' }
    local names = {}
    for name in pairs(switches[key][2]) do names[#names + 1] = name end
    table.sort(names)
    for _, name in ipairs(names) do items[#items + 1] = name end
    return items
end

-- The saved file is the truth; the page shows what it holds.
local saved = kinetic.switches()
local function savedItem(key)
    local env, values = switches[key][1], switches[key][2]
    local value = saved[env] and saved[env].saved or ''
    for name, v in pairs(values) do
        if v == value then return name end
    end
    return 'Default'
end

-- ---------------------------------------------------------------------------

I.Settings.registerPage({
    key = 'Kinetic',
    l10n = L10N,
    name = 'pageName',
    description = 'pageDescription',
})

local settings = {}
for _, key in ipairs(order) do
    settings[#settings + 1] = {
        key = key,
        renderer = 'select',
        name = key .. 'Name',
        description = key .. 'Description',
        default = 'Default',
        argument = { l10n = L10N, items = itemsOf(key) },
    }
end
I.Settings.registerGroup({
    key = GROUP,
    page = 'Kinetic',
    l10n = L10N,
    name = 'groupName',
    description = 'groupDescription',
    permanentStorage = true,
    settings = settings,
})

local section = storage.playerSection(GROUP)
-- Start from the file (a fresh profile, or a file edited by hand).
for _, key in ipairs(order) do
    local item = savedItem(key)
    if section:get(key) ~= item then section:set(key, item) end
end

section:subscribe(async:callback(function(_, key)
    if not key or not switches[key] then return end
    local item = section:get(key)
    local value = item ~= 'Default' and switches[key][2][item] or nil
    kinetic.setSwitch(switches[key][1], value)
end))

-- ---------------------------------------------------------------------------
-- Live tuning groups: one per tunable group, one slider per component.

local function parse(text, components)
    local values = {}
    local i = 1
    for part in string.gmatch(text or '', '[^,]+') do
        values[i] = tonumber(part)
        i = i + 1
    end
    local result = {}
    for c, component in ipairs(components) do
        result[c] = values[c] or component.fallback
    end
    return result
end

local function format(value, argument)
    local step = argument.step or 0.01
    local decimals = 0
    while step < 1 - 1e-6 and decimals < 3 do
        step = step * 10
        decimals = decimals + 1
    end
    return string.format('%.' .. decimals .. 'f', value)
end

local function compose(values, components)
    local parts = {}
    for c, component in ipairs(components) do
        parts[c] = format(values[c], component)
    end
    return table.concat(parts, ',')
end

local tunables = kinetic.tunables()
local groups, groupOrder = {}, {}
local byKey = {}
for t, tunable in ipairs(tunables) do
    if not groups[tunable.group] then
        groups[tunable.group] = {}
        groupOrder[#groupOrder + 1] = tunable.group
    end
    local current = parse(tunable.current, tunable.components)
    local base = parse(tunable.base, tunable.components)
    for c, component in ipairs(tunable.components) do
        local key = string.format('t%02d_%d', t, c)
        local label = tunable.name
        if #tunable.components > 1 then label = label .. ': ' .. component.label end
        byKey[key] = { tunable = tunable, component = c }
        table.insert(groups[tunable.group], {
            key = key,
            renderer = 'kineticSlider',
            name = label,
            description = c == 1 and tunable.description or '',
            default = base[c],
            argument = {
                min = component.min, max = component.max, step = component.step, base = base[c],
            },
            current = current[c],
        })
    end
end

local LIVE_PREFIX = 'SettingsKineticLive'
local liveSections = {}
for g, name in ipairs(groupOrder) do
    local groupKey = LIVE_PREFIX .. g
    local list = {}
    for _, setting in ipairs(groups[name]) do
        list[#list + 1] = {
            key = setting.key, renderer = setting.renderer, name = setting.name,
            description = setting.description, default = setting.default, argument = setting.argument,
        }
    end
    I.Settings.registerGroup({
        key = groupKey,
        page = 'Kinetic',
        -- Names are plain text: an l10n context with no entries shows the key.
        l10n = 'KineticLive',
        name = 'Live: ' .. name,
        description = 'Applies when you close the menu. Saved, and wins over the launcher at the next start.',
        permanentStorage = false,
        settings = list,
    })
    local liveSection = storage.playerSection(groupKey)
    for _, setting in ipairs(groups[name]) do
        if liveSection:get(setting.key) ~= setting.current then liveSection:set(setting.key, setting.current) end
    end
    liveSections[#liveSections + 1] = liveSection
    liveSection:subscribe(async:callback(function(_, key)
        local entry = key and byKey[key]
        if not entry then return end
        local tunable = entry.tunable
        -- Gather every component of this tunable (they may sit in this group only).
        local values, base = {}, parse(tunable.base, tunable.components)
        local same = true
        for c, component in ipairs(tunable.components) do
            local componentKey
            for k, e in pairs(byKey) do
                if e.tunable == tunable and e.component == c then componentKey = k end
            end
            values[c] = tonumber(liveSection:get(componentKey)) or base[c]
            if math.abs(values[c] - base[c]) > 1e-4 then same = false end
        end
        kinetic.setLive(tunable.env, (not same) and compose(values, tunable.components) or nil)
    end))
end

-- ---------------------------------------------------------------------------
-- The reticle (HUD-01, user 10-29): its opacity and size, read by the
-- reticle every frame (kinetic_reticle.lua), saved with the profile.
I.Settings.registerGroup({
    key = 'SettingsKineticHud',
    page = 'Kinetic',
    l10n = L10N,
    name = 'hudGroupName',
    description = 'hudGroupDescription',
    permanentStorage = true,
    settings = {
        { key = 'reticleOpacity', renderer = 'kineticSlider', name = 'reticleOpacityName',
            description = 'reticleOpacityDescription', default = 1,
            argument = { min = 0.1, max = 1, step = 0.05, base = 1 } },
        { key = 'reticleSize', renderer = 'kineticSlider', name = 'reticleSizeName',
            description = 'reticleSizeDescription', default = 1,
            argument = { min = 0.5, max = 2.5, step = 0.05, base = 1 } },
        -- TRAJ-01 (user 10-31): the committed attack's path drawn on screen.
        { key = 'reticleTrajectory', renderer = 'checkbox', name = 'reticleTrajectoryName',
            description = 'reticleTrajectoryDescription', default = true },
    },
})

return {}

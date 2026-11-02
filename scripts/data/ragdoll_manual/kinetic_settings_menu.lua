-- KINETIC options page: the live-tuning slider renderer. Settings
-- renderers are registered from a menu script (the page renders in the menu
-- context); kinetic_settings.lua (player) registers the groups that use it.
local async = require('openmw.async')
local I = require('openmw.interfaces')
local ui = require('openmw.ui')
local util = require('openmw.util')

-- ---------------------------------------------------------------------------
-- The slider renderer: [<] [=====|------] value [>] [reset]. A press on the
-- bar sets the value where it was pressed; the arrows step it.

local whiteTexture = ui.texture { path = 'white' }
local leftArrow = ui.texture { path = 'textures/omw_menu_scroll_left.dds' }
local rightArrow = ui.texture { path = 'textures/omw_menu_scroll_right.dds' }
local BAR_WIDTH, BAR_HEIGHT = 160, 12

local function snap(value, argument)
    local step = argument.step or 0
    if step > 0 then value = argument.min + math.floor((value - argument.min) / step + 0.5) * step end
    return util.clamp(value, argument.min, argument.max)
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

I.Settings.registerRenderer('kineticSlider', function(value, set, argument)
    value = tonumber(value) or argument.min
    local share = (argument.max > argument.min) and (value - argument.min) / (argument.max - argument.min) or 0
    share = util.clamp(share, 0, 1)
    local function arrow(texture, direction)
        return {
            type = ui.TYPE.Image,
            props = { resource = texture, size = util.vector2(12, 12) },
            events = {
                mouseClick = async:callback(function()
                    set(snap(value + direction * (argument.step or 0.01), argument))
                end),
            },
        }
    end
    local bar = {
        type = ui.TYPE.Widget,
        props = { size = util.vector2(BAR_WIDTH, BAR_HEIGHT) },
        content = ui.content {
            {
                type = ui.TYPE.Image,
                props = { resource = whiteTexture, relativeSize = util.vector2(1, 1),
                    color = util.color.rgb(0.25, 0.22, 0.18) },
            },
            {
                type = ui.TYPE.Image,
                props = { resource = whiteTexture, relativeSize = util.vector2(share, 1),
                    color = util.color.rgb(0.79, 0.65, 0.38) },
            },
        },
        events = {
            mousePress = async:callback(function(e)
                if e.button ~= 1 then return end
                local x = util.clamp(e.offset.x / BAR_WIDTH, 0, 1)
                set(snap(argument.min + x * (argument.max - argument.min), argument))
            end),
        },
    }
    local changed = argument.base ~= nil and math.abs(value - argument.base) > 1e-4
    return {
        type = ui.TYPE.Flex,
        props = { horizontal = true, arrange = ui.ALIGNMENT.Center },
        content = ui.content {
            arrow(leftArrow, -1),
            { template = I.MWUI.templates.interval },
            { template = I.MWUI.templates.box, content = ui.content { bar } },
            { template = I.MWUI.templates.interval },
            arrow(rightArrow, 1),
            { template = I.MWUI.templates.interval },
            {
                template = I.MWUI.templates.textNormal,
                props = { text = format(value, argument), size = util.vector2(56, 0) },
            },
            {
                template = I.MWUI.templates.textNormal,
                props = {
                    text = changed and '  reset' or '',
                    textColor = util.color.rgb(0.6, 0.8, 1.0),
                },
                events = changed and {
                    mouseClick = async:callback(function() set(argument.base) end),
                } or nil,
            },
        },
    }
end)


return {}

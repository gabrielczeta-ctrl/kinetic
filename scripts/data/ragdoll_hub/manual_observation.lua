-- Display samples are 10 Hz; the native recorder retains every physics tick.
local ok, native = pcall(require, 'openmw.manualobservation')
if not ok or not native.manualDiagnostics then return { update = function() end, key = function() return false end } end
local ui = require('openmw.ui')
local util = require('openmw.util')
local input = require('openmw.input')
local I = require('openmw.interfaces')
local async = require('openmw.async')
local editor, note, marked, confirmation
local white = ui.texture { path = 'white' }
local panel, elapsed, previousTick = nil, 0, nil
local fontSize = 13
local histories = { {}, {}, {} }
local colors = { util.color.rgb(1, 0.8, 0.1), util.color.rgb(0.5, 1, 0.3), util.color.rgb(1, 0.4, 0.3) }
local function text(value, y, size)
    return { type = ui.TYPE.Text, props = { position = util.vector2(8, y), text = value,
        textSize = size or fontSize, textColor = util.color.rgb(1, 1, 1) } }
end
local function rect(x, y, width, height, color)
    return { type = ui.TYPE.Image, props = { position = util.vector2(x, y), size = util.vector2(width, height),
        resource = white, color = color } }
end
local function noteButton(label, x, width, callback)
    -- No focus callback that tries to update a widget while its modal is destroyed.
    return { template = I.MWUI.templates.boxSolid,
        props = { position = util.vector2(x, 186), size = util.vector2(width, 34) },
        content = ui.content { text(label, 7, 14) },
        events = { mouseClick = async:callback(callback) } }
end
local function update(dt, force)
    elapsed = elapsed + dt
    if elapsed < 0.1 and not force then return end
    elapsed = 0
    local s = native.manualDiagnostics()
    if not s.enabled then return end
    local compact = ui.screenSize().x < 900
    local width = compact and 245 or 350
    local chartWidth = width - 16
    fontSize = compact and 10 or 13
    local values = { math.max(s.leftError, s.rightError), s.motor, math.max(s.leftGrip, s.rightGrip) }
    if previousTick ~= s.tick then
        previousTick = s.tick
        for i, value in ipairs(values) do
            local h = histories[i]; h[#h + 1] = value
            if #h > 50 then table.remove(h, 1) end
        end
    end
    local children = {
        text(string.format('REC %s | tick %d', s.state, s.tick), 6),
        text('B bookmark + note | V vanilla / targets / reference / off', 22),
        text(confirmation or (s.bookmark > 0 and string.format('Bookmark %d saved: tick %d', s.bookmark, s.bookmarkTick)
            or 'Press B when motion looks wrong'), 38),
    }
    local height = 65
    if s.mode > 0 then
        children[#children + 1] = text('Cyan animation | purple command', 60)
        children[#children + 1] = text('Physical: green-red load; gray N/A', 76)
        children[#children + 1] = text('Yellow target | history ~5 seconds', 92)
        local names, limits = { 'Hand error (units)', 'Motor load (1 = budget)', 'Grip gap (units)' }, { 30, 1, 3 }
        for i, h in ipairs(histories) do
            local y = 110 + (i - 1) * 50
            children[#children + 1] = text(names[i] .. ': ' .. (values[i] < 0 and 'N/A' or string.format('%.2f', values[i])), y)
            children[#children + 1] = rect(8, y + 17, chartWidth, 25, util.color.rgb(0.13, 0.13, 0.16))
            for j, value in ipairs(h) do
                if value >= 0 then
                    local bar = math.max(1, 25 * math.min(value / limits[i], 1))
                    children[#children + 1] = rect(8 + (50 - #h + j - 1) * chartWidth / 50,
                        y + 42 - bar, chartWidth / 50 - 1, bar, colors[i])
                end
            end
        end
        children[#children + 1] = text('Scales 30 / 1 / 3; higher values clip', 258)
        height = 278
    end
    local layout = { layer = 'HUD', template = I.MWUI.templates.boxSolid,
        props = { position = util.vector2(math.max(8, ui.screenSize().x - width - 12), 72),
            size = util.vector2(width, height) }, content = ui.content(children) }
    if panel then panel.layout = layout; panel:update() else panel = ui.create(layout) end
end
local function closeEditor()
    if editor then editor:destroy(); editor = nil; I.UI.removeMode('Interface') end
end
local function saveNote()
    if not editor then return end
    local saved = native.manualDiagnostics('note', marked.bookmark, note)
    if not saved.noteSaved then
        ui.showMessage('Note not saved: use at most 8192 bytes; recorder must be running.')
        return
    end
    confirmation = string.format('Note saved: %d / tick %d', marked.bookmark, marked.bookmarkTick)
    closeEditor()
    update(0, true)
end
local function openEditor()
    marked = native.manualDiagnostics('bookmark')
    if marked.state ~= 'recording' then return end
    note, confirmation = '', nil
    local width = math.min(560, ui.screenSize().x - 40)
    I.UI.addMode('Interface', { windows = {} })
    editor = ui.create { layer = 'Windows', template = I.MWUI.templates.boxSolid,
        props = { position = util.vector2((ui.screenSize().x - width) / 2, 100), size = util.vector2(width, 240) },
        content = ui.content {
            text(string.format('Bookmark %d - tick %d', marked.bookmark, marked.bookmarkTick), 10, 16),
            text('Click below and describe what looked or felt wrong.', 36, 13),
            { template = I.MWUI.templates.boxSolid,
                props = { position = util.vector2(8, 62), size = util.vector2(width - 16, 110) },
                content = ui.content {{ template = I.MWUI.templates.textEditBox,
                    props = { position = util.vector2(6, 6), size = util.vector2(width - 28, 98), text = '' },
                    events = { textChanged = async:callback(function(value) note = value end) } }} },
            noteButton('Save note', 8, 150, saveNote),
            noteButton('Skip / Escape', 180, 170, closeEditor),
        } }
end
return {
    update = update,
    isEditing = function() return editor ~= nil end,
    modeChanged = function(mode)
        if editor and mode ~= 'Interface' then editor:destroy(); editor = nil end
    end,
    key = function(key)
        if editor then
            if key.code == input.KEY.Escape then closeEditor() end
            return true
        end
        if not native.manualDiagnostics().enabled then return false end
        if key.code == input.KEY.B then openEditor()
        elseif key.code == input.KEY.V then native.manualDiagnostics('cycle')
        else return false end
        update(0, true)
        return true
    end,
}

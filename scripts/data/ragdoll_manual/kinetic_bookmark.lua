-- B bookmarks a moment of a recorded playthrough (a development package built
-- with package-kinetic.ps1 -Record: KINETIC.cmd sets OPENMW_MANUAL_DIAGNOSTICS).
-- The hub's recorder and note box (ragdoll_hub/manual_observation.lua) without
-- its panel, so a normal game keeps a clean screen. An engine without the
-- recorder (the release build) or a launch without a recording: B does nothing.
local ok, native = pcall(require, 'openmw.manualobservation')
if not ok or not native.manualDiagnostics then return {} end
local ui = require('openmw.ui')
local util = require('openmw.util')
local input = require('openmw.input')
local I = require('openmw.interfaces')
local async = require('openmw.async')
local editor, note, marked
local function text(value, y, size)
    return { type = ui.TYPE.Text, props = { position = util.vector2(8, y), text = value,
        textSize = size or 13, textColor = util.color.rgb(1, 1, 1) } }
end
local function noteButton(label, x, width, callback)
    return { template = I.MWUI.templates.boxSolid,
        props = { position = util.vector2(x, 186), size = util.vector2(width, 34) },
        content = ui.content { text(label, 7, 14) },
        events = { mouseClick = async:callback(callback) } }
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
    closeEditor()
    ui.showMessage(string.format('Bookmark %d saved', marked.bookmark))
end
local function openEditor()
    marked = native.manualDiagnostics('bookmark')
    if marked.state ~= 'recording' then
        -- The recorder stops at 1 GiB (about an hour of play): a new launch records again.
        ui.showMessage('Bookmark not saved: ' .. marked.state .. '. Restart KINETIC.cmd to record again.')
        return
    end
    note = ''
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
    engineHandlers = {
        onKeyPress = function(key)
            if editor then
                if key.code == input.KEY.Escape then closeEditor() end
                return
            end
            -- Inventory, console, dialogue and other modes keep their normal keys.
            if key.code ~= input.KEY.B or I.UI.getMode() ~= nil then return end
            if native.manualDiagnostics().enabled then openEditor() end
        end,
    },
    eventHandlers = {
        UiModeChanged = function(data)
            if editor and data.newMode ~= 'Interface' then editor:destroy(); editor = nil end
        end,
    },
}

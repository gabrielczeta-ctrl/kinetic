-- Reuse ordinary equipment/camera setup, grab handling and the existing HUD.
local script = require('manual_hub_player_base')
local core = require('openmw.core')
local input = require('openmw.input')
local I = require('openmw.interfaces')
local self = require('openmw.self')
local types = require('openmw.types')
local ui = require('openmw.ui')
local util = require('openmw.util')
local newButton = require('scripts.omw.input.button')
local cases = require('manual_hub_cases')
local observation = require('manual_observation')
-- This observation room can idle while automatic restaging continues. Vanity
-- orbit would replace ThirdPerson and prevent the body-capture setup handshake.
local baseUpdate = script.engineHandlers.onUpdate
-- The game's HUD toggle hides the debug text too (user 2026-10-01). The
-- engine hides only its own HUD windows, not Lua layers, and some panels
-- rebuild their layout, so the visibility is enforced every frame.
local function followHudVisibility()
    local visible = I.UI.isHudVisible()
    for _, layer in ipairs({ 'RagdollManualStatus', 'HUD' }) do
        local ok, elements = pcall(ui.getElements, layer)
        if ok and elements then
            for _, element in ipairs(elements) do
                local props = element.layout.props
                if props and (props.visible ~= false) ~= visible then
                    props.visible = visible
                    element:update()
                end
            end
        end
    end
end
-- The demo plays the ideal fighter (user 2026-10-02: "can you make the demo
-- have full/highest stats? so we try the 'ideal' always"): the combat
-- attributes and skills at 100, set once. Not Speed, Athletics or Acrobatics:
-- at 100 the vanilla run was far faster than the tuned gait ("why do I run so
-- fast?").
local maxStatsDone = false
local function maxStats()
    if maxStatsDone then return end
    maxStatsDone = true
    for _, name in ipairs({ 'strength', 'intelligence', 'willpower', 'agility', 'endurance', 'personality',
        'luck' }) do
        types.Actor.stats.attributes[name](self).base = 100
    end
    for _, name in ipairs({ 'longblade', 'shortblade', 'bluntweapon', 'axe', 'spear', 'handtohand', 'block',
        'heavyarmor', 'mediumarmor', 'lightarmor', 'marksman' }) do
        types.NPC.stats.skills[name](self).base = 100
    end
end
script.engineHandlers.onUpdate = function(dt)
    maxStats()
    types.Player.setControlSwitch(self, types.Player.CONTROL_SWITCH.VanityMode, false)
    baseUpdate(dt)
    observation.update(dt)
    followHudVisibility()
end
local baseKey = script.engineHandlers.onKeyPress
local menu
local function close()
    if menu then menu:destroy(); menu = nil; I.UI.removeMode('Interface') end
end
local function dispatch(action, index)
    close()
    core.sendGlobalEvent('manualHubAction', { action = action, index = index })
end
local function open()
    if menu then close(); return end
    if I.UI.getMode() ~= nil then return end
    local buttons = {
        { template = I.MWUI.templates.textHeader, props = { text = 'Manual test circuit' } },
        { template = I.MWUI.templates.textParagraph, props = { size = util.vector2(570, 0),
            text = 'One cell, repeating observations. Pausing stops automatic advance; you can keep playing. '
                .. 'These are development checks, not passed tests. Quick-click uses the native attack; hold then drag aims the physical weapon. Enchantments remain deferred.' } },
    }
    for _, entry in ipairs({ { 'Next (F6)', 'next' }, { 'Repeat / reset (F7)', 'repeat' },
        { 'Pause / resume advance (F9)', 'pause' }, { 'Scenario action (F10)', 'action' } }) do
        local action = entry[2]
        buttons[#buttons + 1] = newButton { text = entry[1], minimumSize = util.vector2(570, 22),
            callback = function() dispatch(action) end }
    end
    for index, c in ipairs(cases) do
        local selected = index
        buttons[#buttons + 1] = newButton { text = string.format('%02d  %s', index, c.title),
            minimumSize = util.vector2(570, 22), callback = function() dispatch('jump', selected) end }
    end
    buttons[#buttons + 1] = newButton { text = 'Close (F8)', minimumSize = util.vector2(570, 22), callback = close }
    I.UI.addMode('Interface', { windows = {} })
    menu = ui.create { layer = 'Windows', template = I.MWUI.templates.boxSolid,
        props = { position = util.vector2(math.max(10, (ui.screenSize().x - 600) / 2), 10) },
        content = ui.content { { template = I.MWUI.templates.padding,
            content = ui.content { { type = ui.TYPE.Flex, content = ui.content(buttons) } } } } }
end
-- H changes detail level; the enemy status always remains visible.
-- Announcements go to the log; none of these modes creates pop-up messages.
local displays = { 'compact', 'full' }
local display, lastStatus = nil, nil
local setStatus = script.eventHandlers.ragdollManualDuelStatus
-- The legacy arena can publish during restaging. Once the hub owns the HUD,
-- those messages must not replace the current opponent's status.
script.eventHandlers.ragdollManualDuelStatus = function(data)
    if not lastStatus then setStatus(data) end
end
local function showStatus()
    if not lastStatus then return end
    local mode = display or (lastStatus.compact and 'compact' or 'full')
    local text = ''
    if mode == 'full' then
        text = lastStatus.short .. '\n' .. lastStatus.enemy .. '\n' .. lastStatus.detail .. '\n' .. lastStatus.text
    elseif mode == 'compact' then
        text = lastStatus.short .. (lastStatus.enemy ~= '' and ('\n' .. lastStatus.enemy) or '')
    end
    setStatus({ announce = lastStatus.announce and mode == 'full', text = text,
        heading = 'F6 next | F7 reset | F8 menu | F10 action | H details | G grab' })
    lastStatus.announce = false
end
script.engineHandlers.onKeyPress = function(key)
    if observation.isEditing and observation.isEditing() then observation.key(key); return end
    if key.code == input.KEY.F8 then open(); return end
    if menu then if key.code == input.KEY.Escape then close() end; return end
    -- Inventory, console, dialogue and other modes keep their normal keys.
    if I.UI.getMode() ~= nil then return end
    if observation.key(key) then return end
    if key.code == input.KEY.H then
        local current = display or (lastStatus and lastStatus.compact and 'compact' or 'full')
        for position, name in ipairs(displays) do
            if name == current then display = displays[position % #displays + 1]; break end
        end
        showStatus()
        return
    end
    local actions = { [input.KEY.F6] = 'next', [input.KEY.F7] = 'repeat',
        [input.KEY.F9] = 'pause', [input.KEY.F10] = 'action' }
    if actions[key.code] then dispatch(actions[key.code]) else baseKey(key) end
end
script.eventHandlers.manualHubStatus = function(data)
    lastStatus = { text = data.text, short = data.short or data.text, compact = data.compact == true,
        announce = data.announce == true, enemy = data.enemy or '', detail = data.detail or '' }
    showStatus()
end
-- Scenario armor and shields for the player (manual_hub_items resolves ids).
local items = require('manual_hub_items')
script.eventHandlers.manualHubEquip = function(data)
    local equipment = types.Actor.getEquipment(self)
    local inventory = types.Actor.inventory(self)
    for _, id in ipairs(data.ids or {}) do
        local slot = items.slot(id)
        if slot then equipment[slot] = inventory:find(id) end
    end
    -- Same frame as the weapon setup, whose equip is not applied yet: the
    -- weapon goes into this table too, or it would be unequipped again.
    if data.weapon then equipment[types.Actor.EQUIPMENT_SLOT.CarriedRight] = inventory:find(data.weapon) end
    types.Actor.setEquipment(self, equipment)
end
-- Dungeon run characters (manual_hub.lua): a warrior of a given level.
script.eventHandlers.manualHubCharacter = function(data)
    types.Actor.stats.level(self).current = data.level
    for name, value in pairs(data.attributes or {}) do
        local stat = types.Actor.stats.attributes[name]
        if stat then stat(self).base = value end
    end
    for name, value in pairs(data.skills or {}) do
        local stat = types.NPC.stats.skills[name]
        if stat then stat(self).base = value end
    end
    local a = data.attributes or {}
    local health = types.Actor.stats.dynamic.health(self)
    health.base, health.current = data.health, data.health
    local fatigue = types.Actor.stats.dynamic.fatigue(self)
    fatigue.base = (a.strength or 50) + (a.willpower or 50) + (a.agility or 50) + (a.endurance or 50)
    fatigue.current = fatigue.base
    print(string.format('PLAY05_HUB DUNGEON player level %d health %d fatigue %d', data.level, data.health, fatigue.base))
end
script.eventHandlers.manualHubReset = function(data)
    script.eventHandlers.manualHubCancel()
    close()
    types.Actor.setStance(self, types.Actor.STANCE.Nothing)
    local health, fatigue = types.Actor.stats.dynamic.health(self), types.Actor.stats.dynamic.fatigue(self)
    -- A launcher may give the player a longer fight than the character's own health.
    if data and data.health then health.base = data.health end
    health.current, fatigue.current = health.base, fatigue.base
end
local baseMode = script.eventHandlers.UiModeChanged
script.eventHandlers.UiModeChanged = function(data)
    if observation.modeChanged then observation.modeChanged(data.newMode) end
    if menu and data.newMode ~= 'Interface' then menu:destroy(); menu = nil end
    baseMode(data)
end
local camera = require('openmw.camera')
-- Force beam (bookmark 21): K fires along the look ray, L cycles strength.
local nearby = require('openmw.nearby')
local beamStrengths = { { 3000, 'light (a sword cut)' }, { 10000, 'medium (a hammer blow)' }, { 30000, 'heavy (knocks down)' } }
local beamLevel = 2
local function fireBeam()
    local from = camera.getPosition()
    local dir = camera.viewportToWorldVector(util.vector2(0.5, 0.5)):normalize()
    -- The ray stops on the physical ragdoll bodies, which carry no object for
    -- Lua: pick the actor whose body line passes nearest the crosshair ray.
    local best, bestPoint, bestDistance = nil, nil, 60
    for _, actor in ipairs(nearby.actors) do
        if actor ~= self.object and not types.Actor.isDead(actor) then
            for _, height in ipairs({ 40, 80, 110, 125 }) do
                local p = actor.position + util.vector3(0, 0, height)
                local along = (p - from):dot(dir)
                if along > 0 and along < 3000 then
                    local closest = from + dir * along
                    local d = (p - closest):length()
                    if d < bestDistance then best, bestPoint, bestDistance = actor, closest, d end
                end
            end
        end
    end
    if not best then
        ui.showMessage('Beam: no actor in the crosshair')
        return
    end
    core.sendGlobalEvent('manualHubBeam', { target = best, point = bestPoint, direction = dir,
        strength = beamStrengths[beamLevel][1] })
end
local baseKeyBeam = script.engineHandlers.onKeyPress
script.engineHandlers.onKeyPress = function(key)
    if key.code == input.KEY.K and I.UI.getMode() == nil then fireBeam(); return end
    if key.code == input.KEY.L and I.UI.getMode() == nil then
        beamLevel = beamLevel % #beamStrengths + 1
        ui.showMessage('Beam strength: ' .. beamStrengths[beamLevel][2])
        return
    end
    if baseKeyBeam then baseKeyBeam(key) end
end

-- Aim indicator (user 2026-10-01 bookmark 8): a ring where the mouse is
-- steering the weapon tip (the tip controller's goal) and a dot on the real
-- tip. Faint at rest, bright while the attack button is held.
local aim = { ring = nil, dot = nil, command = nil, tip = nil }
local function aimElement(texture, size, color)
    return ui.create { layer = 'HUD', type = ui.TYPE.Image,
        props = { resource = ui.texture { path = texture }, size = util.vector2(size, size),
            anchor = util.vector2(0.5, 0.5), color = color, alpha = 0, visible = true } }
end
local function placeAim(element, point, size, alpha)
    local props = element.layout.props
    local screen = point and camera.worldToViewportVector(point)
    local inView = screen and screen.z > 1 and screen.x == screen.x
    local wantAlpha = inView and alpha or 0
    if inView then props.position = util.vector2(screen.x, screen.y) end
    props.size = util.vector2(size, size)
    props.alpha = wantAlpha
    element:update()
end
local reticle = require('kinetic_reticle')
script.eventHandlers.manualHubAim = function(data)
    aim.command, aim.tip = data.command, data.tip
    reticle.feed(data)
end
local baseFrame = script.engineHandlers.onFrame
script.engineHandlers.onFrame = function(dt)
    if baseFrame then baseFrame(dt) end
    if not aim.ring then
        aim.ring = aimElement('textures/ragdoll_aim_ring.png', 36, util.color.rgb(1, 1, 1))
        aim.dot = aimElement('textures/ragdoll_aim_dot.png', 12, util.color.rgb(1, 0.55, 0.15))
    end
    local held = input.isMouseButtonPressed(1)
    placeAim(aim.ring, aim.command, held and 44 or 30, held and 0.9 or 0.3)
    -- The dot on the real tip is off (user bookmark 7, 2026-10-06: "lets also
    -- get rid of the orange dot at the tip"); the ring stays.
    placeAim(aim.dot, nil, 10, 0)
    -- The stance panel at the right is gone (user 10-29: "remove the
    -- indicator on the right"); the reticle shows its information.
    reticle.frame(dt)
end
return script

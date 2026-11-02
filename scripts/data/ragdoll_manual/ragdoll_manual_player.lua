local camera = require('openmw.camera')
local core = require('openmw.core')
local input = require('openmw.input')
local I = require('openmw.interfaces')
local self = require('openmw.self')
local types = require('openmw.types')
local ui = require('openmw.ui')
local util = require('openmw.util')
local newButton = require('scripts.omw.input.button')

local panel
local panelDuelStatus
local hud
local ownsInterfaceMode = false
local grabbing = false
local pendingCombatSetup

local function cameraRay()
    return { from = camera.getPosition(), direction = camera.viewportToWorldVector(util.vector2(0.5, 0.5)) }
end

local duelText = 'Duel: waiting for the flat-floor range.'
local hudHeading = 'F8: manual menu   G: grab / release'
local hudWidth

local function closeMenu()
    if panel then
        panel:destroy()
        panel = nil
        panelDuelStatus = nil
    end
    if ownsInterfaceMode then
        ownsInterfaceMode = false
        I.UI.removeMode('Interface')
    end
end

local function dispatch(action)
    -- Close the paused menu before the global command. Tests use simulation
    -- time and must be able to progress after a button is pressed.
    closeMenu()
    core.sendGlobalEvent('ragdollManualControl', { action = action })
end

local function hudText()
    return hudHeading .. '\n' .. duelText
end

local function refreshHud()
    if hud then
        hud.layout.props.text = hudText()
        hud:update()
    end
end

local function button(text, action)
    return newButton {
        text = text,
        minimumSize = util.vector2(610, 28),
        callback = function() dispatch(action) end,
    }
end

local function openMenu()
    if panel then
        closeMenu()
        return
    end
    if I.UI.getMode() ~= nil then
        print('Close the current menu, then press F8 for ragdoll controls.')
        return
    end
    I.UI.addMode('Interface', { windows = {} })
    ownsInterfaceMode = true
    panelDuelStatus = {
        template = I.MWUI.templates.textParagraph,
        props = { text = duelText, size = util.vector2(620, 0) },
    }
    panel = ui.create {
        layer = 'Windows',
        template = I.MWUI.templates.boxSolid,
        props = { position = util.vector2(math.max(16, (ui.screenSize().x - 650) / 2), 70) },
        content = ui.content {
            {
                template = I.MWUI.templates.padding,
                content = ui.content {
                    {
                        type = ui.TYPE.Flex,
                        content = ui.content {
                            { template = I.MWUI.templates.textHeader,
                                props = { text = 'Ragdoll duels' } },
                            { template = I.MWUI.templates.textParagraph,
                                props = { size = util.vector2(620, 0),
                                    text = 'AI fighters are staged in front of you and powered automatically. '
                                        .. 'Loadout and health changes restage the pair.' } },
                            panelDuelStatus,
                            button('Restage duel now', 'duel_restage'),
                            button('Loadout: Fists only', 'duel_loadout_fist'),
                            button('Loadout: Daggers only', 'duel_loadout_dagger'),
                            button('Loadout: Mixed fist vs dagger', 'duel_loadout_mixed'),
                            button('Fighter health: Normal (40)', 'duel_health_normal'),
                            button('Fighter health: Quick (12)', 'duel_health_quick'),
                            button('Auto-duel: start / keep running', 'duel_auto_on'),
                            button('Auto-duel: stop', 'duel_auto_off'),
                            { template = I.MWUI.templates.textHeader,
                                props = { text = 'Player' } },
                            button('Arm player fists (physical punches)', 'player_fists'),
                            button('Arm player iron dagger (physical stabs)', 'player_dagger'),
                            button('Release player body to animation', 'player_release'),
                            { template = I.MWUI.templates.textHeader,
                                props = { text = 'Hit tuning' } },
                            button('Stagger scale: light (x0.5)', 'reaction_light'),
                            button('Stagger scale: default (x1)', 'reaction_default'),
                            button('Stagger scale: strong (x2)', 'reaction_strong'),
                            { template = I.MWUI.templates.textHeader,
                                props = { text = 'Range' } },
                            button('Test stepping: toggle k=1 / k=4', 'step_toggle'),
                            button('Report duel status', 'duel_status'),
                            button('Reset flat-floor range', 'reset'),
                            newButton { text = 'Close / resume (F8)', minimumSize = util.vector2(610, 28),
                                callback = closeMenu },
                        },
                    },
                },
            },
        },
    }
end

local function updateHud()
    local width = math.max(1, math.min(760, ui.screenSize().x - 36))
    if not hud then
        if not ui.layers.indexOf('RagdollManualStatus') then
            ui.layers.insertAfter('HUD', 'RagdollManualStatus', { interactive = false })
        end
        hud = ui.create {
            layer = 'RagdollManualStatus',
            template = I.MWUI.templates.textParagraph,
            props = { position = util.vector2(18, 18), size = util.vector2(width, 0), textSize = 15,
                text = hudText() },
        }
        hudWidth = width
    elseif width ~= hudWidth then
        hudWidth = width
        hud.layout.props.size = util.vector2(width, 0)
        hud:update()
    end
end

return {
    engineHandlers = {
        onUpdate = function()
            if not pendingCombatSetup then return end
            -- Let the deferred camera rebuild and equipment actions complete
            -- before the global script snapshots the full-body skeleton.
            if core.getSimulationTime() < pendingCombatSetup.readyAt then return end
            local item = types.Actor.getEquipment(self, types.Actor.EQUIPMENT_SLOT.CarriedRight)
            local weapon = pendingCombatSetup.weapon
            -- Unequipping the previous weapon can lower the stance in a later
            -- engine action. Reassert it after the requested equipment lands.
            if ((weapon and item and item.recordId == weapon) or (not weapon and not item))
                and types.Actor.getStance(self) ~= types.Actor.STANCE.Weapon then
                types.Actor.setStance(self, types.Actor.STANCE.Weapon)
                return
            end
            if core.getSimulationTime() >= (pendingCombatSetup.reportAt or math.huge) then
                print('MANUAL_COMBAT_SETUP waiting weapon=' .. tostring(weapon) .. ' equipped='
                    .. tostring(item and item.recordId) .. ' stance=' .. tostring(types.Actor.getStance(self))
                    .. ' camera=' .. tostring(camera.getMode()) .. ' dead=' .. tostring(types.Actor.isDead(self)))
                pendingCombatSetup.reportAt = core.getSimulationTime() + 3
            end
            if camera.getMode() ~= camera.MODE.ThirdPerson
                or types.Actor.getStance(self) ~= types.Actor.STANCE.Weapon
                or (weapon and (not item or item.recordId ~= weapon)) or (not weapon and item) then return end
            core.sendGlobalEvent('ragdollManualPlayerCombatReady', { weapon = weapon })
            pendingCombatSetup = nil
        end,
        onFrame = function()
            updateHud()
            if grabbing then
                core.sendGlobalEvent('ragdollManualGrab', { action = 'update', ray = cameraRay() })
            end
        end,
        onKeyPress = function(key)
            if key.code == input.KEY.F8 then
                openMenu()
            elseif key.code == input.KEY.Escape and panel then
                closeMenu()
            elseif key.code == input.KEY.G and not panel then
                -- G grabs the ragdoll body under the crosshair and holds it
                -- in front of the camera until G is pressed again.
                grabbing = not grabbing
                core.sendGlobalEvent('ragdollManualGrab',
                    { action = grabbing and 'grab' or 'release', ray = cameraRay() })
            end
        end,
    },
    eventHandlers = {
        manualHubCancel = function()
            pendingCombatSetup = nil
            grabbing = false
            core.sendGlobalEvent('ragdollManualGrab', { action = 'release' })
        end,
        ragdollManualDuelStatus = function(data)
            duelText = data.text
            hudHeading = data.heading or hudHeading
            refreshHud()
            if panel then
                panelDuelStatus.props.text = duelText
                panel:update()
            end
            if data.announce then
                print(duelText)
            end
        end,
        ragdollManualExperimentalStatus = function(data)
            -- Keep diagnostic announcements in the log so they cannot cover
            -- the view during play. The top-left duel HUD remains separate.
            if data.announce then
                print(data.text)
            end
        end,
        ragdollManualDefenseStatus = function(data)
            if data.announce then
                print(data.text)
            end
        end,
        ragdollManualFistsConfigure = function(data)
            camera.setMode(camera.MODE.ThirdPerson)
            camera.setPreferredThirdPersonDistance(260)
            local equipment = types.Actor.getEquipment(self)
            local weapon = data and data.weapon
            equipment[types.Actor.EQUIPMENT_SLOT.CarriedRight] = weapon and types.Actor.inventory(self):find(weapon) or nil
            types.Actor.setEquipment(self, equipment)
            types.Actor.setStance(self, types.Actor.STANCE.Weapon)
            pendingCombatSetup = { weapon = weapon, readyAt = core.getSimulationTime() + 0.25,
                reportAt = core.getSimulationTime() + 3 }
        end,
        UiModeChanged = function(data)
            if panel and data.newMode ~= 'Interface' then
                closeMenu()
            end
        end,
    },
}

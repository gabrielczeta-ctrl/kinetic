local testing = require('testing_util')
local menu = require('openmw.menu')

-- Start each independently selected global scenario from a clean world. The
-- global fixture then relocates the player to the isolated flat-floor interior.
-- This also guarantees that an assertion failure cannot leak actors or a
-- moved player into the following scenario.
testing.setSetupGlobalTest(function()
    menu.newGame({ bypass = true })
    coroutine.yield()
end)

local function findSave(description)
    for directory, saves in pairs(menu.getAllSaves()) do
        for slotName, info in pairs(saves) do
            if info.description == description then
                return directory, slotName
            end
        end
    end
end

testing.registerMenuTest('dead actor save load recreates passive ragdoll', function()
    local description = 'ragdoll dead actor round trip'
    while true do
        local staleDirectory, staleSlot = findSave(description)
        if not staleDirectory then
            break
        end
        menu.deleteGame(staleDirectory, staleSlot)
    end
    testing.runGlobalTest('ragdoll save load - prepare dead actor')

    menu.saveGame(description)
    coroutine.yield()
    local directory, slotName = findSave(description)
    testing.expect(directory ~= nil and slotName ~= nil, 'ragdoll save was not listed')

    testing.runGlobalTest('ragdoll save load - clear live state')
    menu.loadGame(directory, slotName)
    coroutine.yield()
    testing.runGlobalTest('ragdoll save load - verify restored actor')

    menu.deleteGame(directory, slotName)
    local remainingDirectory = findSave(description)
    testing.expectEqual(remainingDirectory, nil, 'ragdoll test save was not deleted')
end)

testing.registerMenuTest('fight save load restores powered fighters', function()
    local description = 'ragdoll fight round trip'
    while true do
        local staleDirectory, staleSlot = findSave(description)
        if not staleDirectory then break end
        menu.deleteGame(staleDirectory, staleSlot)
    end
    testing.runGlobalTest('ragdoll save load - prepare fight')
    menu.saveGame(description)
    coroutine.yield()
    local directory, slotName = findSave(description)
    testing.expect(directory ~= nil and slotName ~= nil, 'fight save was not listed')
    testing.runGlobalTest('ragdoll save load - clear fight')
    menu.loadGame(directory, slotName)
    coroutine.yield()
    testing.runGlobalTest('ragdoll save load - verify fight')
    menu.deleteGame(directory, slotName)
end)

return {
    engineHandlers = {
        onFrame = testing.makeUpdateMenu(),
    },
    eventHandlers = testing.menuEventHandlers,
}

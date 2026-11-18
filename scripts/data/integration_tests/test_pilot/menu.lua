local menu = require('openmw.menu')
local testing = require('testing_util')

testing.setSetupGlobalTest(function()
    menu.newGame({ bypass = true })
    coroutine.yield()
end)

return {
    engineHandlers = { onFrame = testing.makeUpdateMenu() },
    eventHandlers = testing.menuEventHandlers,
}

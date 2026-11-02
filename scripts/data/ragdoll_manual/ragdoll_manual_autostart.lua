local interfaces = require('openmw.interfaces')

local started = false

local function update()
    if started then
        return
    end

    local helper = interfaces.RagdollDev
    if helper then
        -- Manual launches must never inherit the bypass character's generic
        -- start marker. It can be inside architecture or unsuitable terrain,
        -- which makes every visual physics result ambiguous.
        helper.setup()
        started = true
    end
end

return {
    engineHandlers = {
        onUpdate = update,
    },
}

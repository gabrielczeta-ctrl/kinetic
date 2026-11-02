-- The brass reticle in a normal game (the package; the hub has its own in
-- ragdoll_manual_player.lua): fed by kinetic_hud_global.lua on development
-- engines, else read here from openmw.kinetic's hud() (release engines have
-- no ragdoll state package: the reticle never showed in a release, user
-- 10-31). With neither the engine's crosshair is left alone.
local reticle = require('kinetic_reticle')

local okKinetic, kinetic = pcall(require, 'openmw.kinetic')
local hudQuery = okKinetic and kinetic and kinetic.hud or nil

local fed = false -- the global feed is running
local polled = false -- logged once: the reticle reads openmw.kinetic

return {
    eventHandlers = {
        kineticHudAim = function(data)
            fed = true
            reticle.feed(data)
        end,
    },
    engineHandlers = {
        onFrame = function(dt)
            if not fed and hudQuery then
                local data = hudQuery()
                if data then
                    if not polled then
                        polled = true
                        print('KINETIC reticle: fed from openmw.kinetic')
                    end
                    reticle.feed(data)
                    reticle.frame(dt)
                end
                return
            end
            if fed then reticle.frame(dt) end
        end,
    },
}

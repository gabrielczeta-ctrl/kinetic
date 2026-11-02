-- Stranded NPCs (build 59 bookmark 18: "I pushed 4-5 NPCs off the bridges in
-- Balmora into the water and they can't climb back up"). Physical pushes
-- put NPCs where Morrowind's walls and banks keep them: a canal, a ditch,
-- the foot of a ledge. An NPC that has been swimming for SWIM_SECONDS, or
-- that has stood LOW_DROP units or more below its starting spot for
-- LOW_SECONDS, goes back to its starting spot once the player is not
-- looking at it (or is far away). Exteriors only: an interior's starting
-- spot may be in another cell.
local types = require('openmw.types')
local util = require('openmw.util')
local world = require('openmw.world')

local SWIM_SECONDS = 20
local LOW_SECONDS = 30
local LOW_DROP = 300
local UNSEEN_DISTANCE = 1500
local CHECK_EVERY = 1.0

local stranded = {}
local sinceCheck = 0

local function seenByPlayer(player, actor)
    local offset = actor.position - player.position
    local distance = offset:length()
    if distance > UNSEEN_DISTANCE then return false end
    if distance < 1 then return true end
    local forward = player.rotation * util.vector3(0, 1, 0)
    return forward:dot(offset / distance) > 0.2
end

local function check(dt)
    local player = world.players[1]
    if not player then return end
    local present = {}
    for _, actor in ipairs(world.activeActors) do
        if actor ~= player and types.NPC.objectIsInstance(actor) and not types.Actor.isDead(actor)
            and actor.cell and actor.cell.isExterior then
            local home = actor.startingPosition
            local key = actor.id
            present[key] = true
            -- (types.Actor.isSwimming takes local objects only: in water here is
            -- 50 units or more under the cell's water level.)
            local cell = actor.cell
            local swimming = cell.hasWater and cell.waterLevel ~= nil and actor.position.z < cell.waterLevel - 50
            local low = home and (home.z - actor.position.z) >= LOW_DROP
                and (util.vector2(home.x, home.y) - util.vector2(actor.position.x, actor.position.y)):length() < 4000
            local state = stranded[key] or { swim = 0, low = 0 }
            state.swim = swimming and state.swim + dt or 0
            state.low = low and state.low + dt or 0
            stranded[key] = state
            if home and (state.swim >= SWIM_SECONDS or state.low >= LOW_SECONDS) and not seenByPlayer(player, actor) then
                print(string.format('KINETIC_STRANDED npc=%s swim=%.0f low=%.0f home=%s', tostring(actor.recordId),
                    state.swim, state.low, tostring(home)))
                actor:teleport(actor.cell, home)
                stranded[key] = nil
            end
        end
    end
    for key in pairs(stranded) do
        if not present[key] then stranded[key] = nil end
    end
end

return {
    engineHandlers = {
        onUpdate = function(dt)
            sinceCheck = sinceCheck + dt
            if sinceCheck >= CHECK_EVERY then
                check(sinceCheck)
                sinceCheck = 0
            end
        end,
    },
}

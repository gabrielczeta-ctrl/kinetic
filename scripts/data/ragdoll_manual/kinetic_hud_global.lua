-- The brass reticle's feed outside the dev hub (user 10-30: "can you enable
-- the crosshair for my normal playthrough"): each frame the player's hand
-- control, guard, balance and attack readiness go to kinetic_hud_player.lua,
-- as manual_hub.lua's feedAim does in the hub. Needs the ragdoll state
-- package (development builds); without it the HUD stays off and the
-- engine's crosshair stays.
local world = require('openmw.world')

local ok, ragdoll = pcall(require, 'openmw.ragdolltest')
if not ok then ragdoll = nil end

local function update()
    if not ragdoll then return end
    local player = world.players[1]
    if not (player and player:isValid()) then return end
    local ps = ragdoll.getState(player)
    if not ps then return end
    local hud
    local hsc = ps.hsc
    if hsc then
        hud = { stage = hsc.stage, x = hsc.stanceX, y = hsc.stanceY, angle = hsc.angle, line = hsc.line,
            trajectory = hsc.trajectory }
    end
    player:sendEvent('kineticHudAim', { hud = hud, guard = ps.guardActive, guardZone = ps.guardZone,
        balance = { x = ps.balanceRight or 0, y = ps.balanceForward or 0, tip = ps.balanceTip or 0 },
        ready = ps.attackReady or 1 })
end

return { engineHandlers = { onUpdate = update } }

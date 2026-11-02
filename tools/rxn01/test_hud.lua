-- Run from the source root with LuaJIT. Exercise the actual hub presenter and
-- key handler; engine-owned UI creation is checked separately in native captures.
package.path = 'scripts/data/ragdoll_hub/?.lua;' .. package.path
local status = require('manual_hub_status')
local r = { enabled = true, behavior = 'arm_cradle', cradleLeft = true,
    limpRight = true, speedScale = 0.8, stabilizationStrength = 1, lastServedReceipt = 2,
    rightArmOwned = true }
local state = { controller = { mode = 'standing' }, reaction = r,
    lastWoundKind = 'bruise', lastWoundRegion = 'left_forearm', lastWoundHealth = 4, lastWoundFatigue = 0 }
local enemy, detail = status.enemy(state, 146, 250)
assert(enemy:find('Controller: standing', 1, true))
assert(enemy:find('left arm cradle + right limp', 1, true))
assert(enemy:find('speed limit 80%', 1, true))
assert(detail:find('right held by weapon / aim', 1, true))
r.enabled = false
assert(status.enemy(state, 146, 250):find('RXN: OFF', 1, true))
r.enabled, r.behavior, r.cradleLeft, r.limpRight, r.speedScale = true, 'none', false, false, 1
assert(status.enemy(state, 146, 250):find('no active reaction', 1, true))
assert(status.enemy(nil, 0, 250) == 'Enemy: preparing')

local shown, writes, mode = nil, 0, nil
local base = {
    engineHandlers = { onUpdate = function() end, onKeyPress = function() end },
    eventHandlers = {
        ragdollManualDuelStatus = function(data) shown = data; writes = writes + 1 end,
        UiModeChanged = function() end,
    },
}
local keys = { H = 1, F8 = 2, Escape = 3, F6 = 4, F7 = 5, F9 = 6, F10 = 7 }
package.preload.manual_hub_player_base = function() return base end
package.preload['openmw.core'] = function() return {} end
package.preload['openmw.input'] = function() return { KEY = keys } end
package.preload['openmw.interfaces'] = function() return { UI = { getMode = function() return mode end } } end
package.preload['openmw.self'] = function() return {} end
package.preload['openmw.types'] = function() return {} end
package.preload['openmw.ui'] = function() return {} end
package.preload['openmw.util'] = function() return {} end
package.preload['scripts.omw.input.button'] = function() return function() end end
package.preload.manual_hub_cases = function() return {} end
local hub = require('ragdoll_manual_player')
hub.eventHandlers.manualHubStatus({ text = 'Full scenario description', short = 'Dagger duel',
    compact = true, enemy = enemy, detail = detail })
assert(shown.text:find(enemy, 1, true))
assert(not shown.text:find('Full scenario description', 1, true))
for i = 1, 12 do
    hub.engineHandlers.onKeyPress({ code = keys.H })
    assert(shown.text:find(enemy, 1, true), 'H hid the enemy state')
    assert((shown.text:find('Full scenario description', 1, true) ~= nil) == (i % 2 == 1))
end
local previous = writes
hub.eventHandlers.ragdollManualDuelStatus({ text = 'Legacy duel message' })
assert(writes == previous, 'legacy arena replaced the hub HUD')
mode = 'Console'
hub.engineHandlers.onKeyPress({ code = keys.H })
assert(writes == previous, 'console typing changed the HUD')
print('RXN-01 HUD: native-state formatting, OFF/idle, simultaneous injury, 12 H cycles and legacy ownership passed')

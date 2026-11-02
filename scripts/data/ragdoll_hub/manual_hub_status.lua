-- Presentation only: every reaction/ownership value comes from the native
-- fixed-step snapshot, not a guess based on the last wound.
local function words(value)
    return (value or 'none'):gsub('_', ' ')
end
local function side(left, right)
    return left and (right and 'both' or 'left') or 'right'
end
local function enemy(state, health, playerHealth)
    if not state then return 'Enemy: preparing', '' end
    local lines = { string.format('You HP %.0f | Enemy HP %.0f | Controller: %s',
        playerHealth, health, words(state.controller and state.controller.mode or 'passive')) }
    local r = state.reaction
    if not r then
        lines[#lines + 1] = 'RXN: telemetry unavailable'
    elseif not r.enabled then
        lines[#lines + 1] = 'RXN: OFF'
    else
        local active = {}
        if r.behavior ~= 'none' and r.behavior ~= 'arm_cradle' and r.behavior ~= 'limp' then
            active[#active + 1] = words(r.behavior)
        end
        if r.cradleLeft or r.cradleRight then
            active[#active + 1] = side(r.cradleLeft, r.cradleRight) .. ' arm cradle'
        end
        if r.limpLeft or r.limpRight then
            active[#active + 1] = side(r.limpLeft, r.limpRight) .. ' limp'
        end
        lines[#lines + 1] = 'RXN: ' .. (#active > 0 and table.concat(active, ' + ') or 'ready / no active reaction')
            .. (r.speedScale < 0.999 and string.format(' | speed limit %.0f%%', r.speedScale * 100) or '')
    end
    lines[#lines + 1] = state.lastWoundRegion and state.lastWoundRegion ~= '' and state.lastWoundRegion ~= 'unknown'
        and string.format('Last injury: %s / %s | %.1f HP, %.1f fatigue',
            words(state.lastWoundRegion), words(state.lastWoundKind), state.lastWoundHealth, state.lastWoundFatigue)
        or 'Last injury: none'
    local detail = ''
    if r and r.enabled then
        local owned = r.leftArmOwned or r.rightArmOwned
        detail = string.format('RXN receipt %d | motor budget %.0f%% | Arm priority: %s%s',
            r.lastServedReceipt, r.stabilizationStrength * 100,
            owned and (side(r.leftArmOwned, r.rightArmOwned) .. ' held by weapon / aim') or 'free',
            r.recoveryBrace and ' | recovery / brace priority' or '')
    end
    return table.concat(lines, '\n'), detail
end
return { enemy = enemy }

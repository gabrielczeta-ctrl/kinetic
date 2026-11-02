-- The brass reticle (user 10-29: "use the crosshair as an indicator of weapon
-- being held or blocking ... the information seen in the mouse visual
-- indicator in the right, but with a good design and in the center"; style
-- C, a Dwemer diamond). It replaces the crosshair:
--   sheathed      a small diamond point
--   weapon out    the diamond's four corners open around it, a point where the
--                 mouse stance is (hand control's stance point)
--   wind-up       teal chevrons out along the cut's line, one per third of the
--                 full wind-up
--   cut           the diamond snaps in, red
--   recover       it opens again
--   guard         the guarded side of the diamond drawn solid (high: both top
--                 edges, left: both left, middle: all four); white for the
--                 parry window after the raise
--   shove         the diamond closes in and opens (a double right click)
--   balance       a solid brass wedge grows out of the corner the weight tips
--                 toward (top: forward, right: right, ...), gold, warming to
--                 amber near the edge (user 10-29: weighted corner, not circles,
--                 never like the attack cues - those are teal outline chevrons
--                 and the red snap of a cut)
--   cooldown      a thin brass bar under the diamond while a new attack cannot
--                 start: a clash lockout or the bearings after a hit (filling as
--                 it runs out), a stagger holding the weapon arm (empty), below
--                 weapon skill 50 the return to the guard; a white flash when it
--                 can (user 10-30: "the reticle shows the cooldown to attack
--                 again"; the engine's attackReadiness, HUD_READY)
--   swing path    once an attack is let go (a click's wind-up, any cut) teal dots
--                 on the screen where the weapon's tip is driven through the cut,
--                 brightening toward its end, kept 0.7 s after the cut (user
--                 10-31: "upon release the crosshair draws the player swing
--                 trajectory so the player gets better feedback on how the
--                 aiming works"; the engine's commanded tip, TRAJ-01). The KINETIC
--                 page's "Swing path" turns it off.
local camera = require('openmw.camera')
local input = require('openmw.input')
local self = require('openmw.self')
local storage = require('openmw.storage')
local types = require('openmw.types')
local ui = require('openmw.ui')
local util = require('openmw.util')

local M = {}
local GOLD = util.color.rgb(0.84, 0.70, 0.41)
local RED = util.color.rgb(0.94, 0.47, 0.35)
local TEAL = util.color.rgb(0.43, 0.84, 0.80)
local WHITE = util.color.rgb(1, 0.97, 0.90)
local AMBER = util.color.rgb(0.98, 0.62, 0.22)
-- corner: its texture, its way out (x right, y up on screen = forward)
local WEIGHTS = { { 'top', 0, 1 }, { 'right', 1, 0 }, { 'bottom', 0, -1 }, { 'left', -1, 0 } }
local FULL = 1800 -- stance counts of a full wind-up (HSC_STANCE_RADIUS + COMMIT_COUNTS)
local OPEN, CLOSED, SHOVED = 15, 9, 10 -- diamond radius, pixels
local PARRY = 0.35 -- the parry window after a raise, s
local EDGES = { high = { 'tl', 'tr' }, left = { 'tl', 'bl' }, right = { 'tr', 'br' }, low = { 'bl', 'br' },
    middle = { 'tl', 'tr', 'br', 'bl' } }
local TRAIL = 24 -- swing path dots (the engine sends up to 32 points; every point drawn when fewer)
local TRAIL_KEEP = 0.7 -- s the last path stays after the attack's cut, fading

local root = nil
-- Opacity and size from the KINETIC page (kinetic_settings.lua).
local hudSettings = storage.playerSection('SettingsKineticHud')
local function setting(key, low, high)
    return math.max(low, math.min(high, tonumber(hudSettings:get(key)) or 1))
end
local data, guard, zone, balance = nil, false, 'middle', nil
local ready, readyAt = 1, -10
local guardSince, now, radius, lastPress, shoveAt = -10, 0, OPEN, -10, -10
local rmbWas = false
local eased = {}
local path, pathAt = nil, -10 -- the last swing path (world points) and when it last came

local function texture(name) return ui.texture { path = 'textures/' .. name .. '.png' } end
local PIECES = { 'corners', 'centre', 'stance', 'edge_tl', 'edge_tr', 'edge_br', 'edge_bl', 'chev1', 'chev2', 'chev3',
    'weight_top', 'weight_right', 'weight_bottom', 'weight_left', 'cooldown' }
local CHEVRONS = {}
for k = 0, 7 do CHEVRONS[k] = texture('kinetic_brass_chevron_' .. k) end
local function image(name, tex, size)
    return { name = name, type = ui.TYPE.Image, props = { resource = texture(tex), size = util.vector2(size, size),
        relativePosition = util.vector2(0.5, 0.5), anchor = util.vector2(0.5, 0.5), color = GOLD, alpha = 0 } }
end

local function create()
    local content = { image('corners', 'kinetic_brass_corners', 36), image('centre', 'kinetic_brass_centre', 8),
        image('stance', 'kinetic_brass_centre', 6) }
    for _, e in ipairs({ 'tl', 'tr', 'br', 'bl' }) do content[#content + 1] = image('edge_' .. e, 'kinetic_brass_edge_' .. e, 36) end
    for i = 1, 3 do content[#content + 1] = image('chev' .. i, 'kinetic_brass_chevron_0', 14) end
    for _, w in ipairs(WEIGHTS) do content[#content + 1] = image('weight_' .. w[1], 'kinetic_brass_weight_' .. w[1], 12) end
    content[#content + 1] = image('cooldown', 'kinetic_brass_bar', 8)
    for i = 1, TRAIL do content[#content + 1] = image('path' .. i, 'kinetic_brass_centre', 5) end
    root = ui.create { layer = 'HUD', type = ui.TYPE.Widget,
        props = { relativeSize = util.vector2(1, 1) }, content = ui.content(content) }
end

-- The stance overlay's feed (manualHubAim): hand control's stance, the guard.
function M.feed(event)
    data = event.hud
    local up = event.guard and true or false
    if up and not guard then guardSince = now end
    guard, zone = up, event.guardZone or 'middle'
    balance = event.balance
    if data and data.trajectory and #data.trajectory >= 2 then path, pathAt = data.trajectory, now end
    local was = ready
    ready = tonumber(event.ready) or 1
    if ready >= 1 and was < 1 then readyAt = now end
end

local function place(props, x, y)
    props.position = util.vector2(x, -y)
end

function M.frame(dt)
    now = now + dt
    if not root then create() end
    local c = root.layout.content
    -- The engine's crosshair gives way to this one (the camera script shows
    -- it again every frame, this runs after it).
    camera.showCrosshair(false)
    local drawn = types.Actor.getStance(self) == types.Actor.STANCE.Weapon
    -- A double right click with the weapon out is a shove (mousemanager.cpp).
    local rmb = input.isMouseButtonPressed(3)
    if rmb and not rmbWas and drawn then
        if now - lastPress < 0.3 then shoveAt, lastPress = now, -10 else lastPress = now end
    end
    rmbWas = rmb
    local stage = data and data.stage or 0
    local target = OPEN
    if stage == 2 then target = CLOSED end
    local sinceShove = now - shoveAt
    if sinceShove < 0.35 then target = SHOVED end
    -- Snaps in fast, opens slower.
    local rate = target < radius and 160 or 60
    radius = radius + math.max(-rate * dt, math.min(rate * dt, target - radius))
    -- The page's size scales every piece and distance; its opacity every alpha.
    local scale, opacity = setting('reticleSize', 0.5, 2.5), setting('reticleOpacity', 0.1, 1)
    local r = radius * scale
    local size = r * 64 / 27 -- the diamond's corners sit 27 px from the centre of the 64 px texture
    c.centre.props.size = util.vector2(8 * scale, 8 * scale)
    c.stance.props.size = util.vector2(6 * scale, 6 * scale)
    local colour = (stage == 2) and RED or GOLD
    c.centre.props.alpha = drawn and 0.8 or 0.65
    c.centre.props.color = colour
    c.corners.props.size = util.vector2(size, size)
    c.corners.props.color = colour
    c.corners.props.alpha = drawn and 0.85 or 0
    -- the stance point, scaled so a full wind-up sits on the corners
    if drawn and data and data.x then
        local k, edge = r * 0.9 / FULL, 24 * scale
        place(c.stance.props, math.max(-edge, math.min(edge, data.x * k)), math.max(-edge, math.min(edge, data.y * k)))
        c.stance.props.alpha = 0.75
        c.stance.props.color = colour
    else
        c.stance.props.alpha = 0
    end
    -- the guard edges
    local lit = {}
    if drawn and guard then for _, e in ipairs(EDGES[zone] or EDGES.middle) do lit[e] = true end end
    local parry = now - guardSince < PARRY
    for _, e in ipairs({ 'tl', 'tr', 'br', 'bl' }) do
        local props = c['edge_' .. e].props
        props.size = util.vector2(size, size)
        props.alpha = lit[e] and (parry and 1 or 0.9) or 0
        props.color = parry and WHITE or GOLD
    end
    -- the wind-up chevrons along the cut's line
    local charge = 0
    if drawn and data and stage == 1 and data.angle then
        charge = math.sqrt((data.x or 0) ^ 2 + (data.y or 0) ^ 2) / FULL
    end
    local k = data and data.angle and (math.floor(data.angle / (math.pi / 4) + 0.5) % 8) or 0
    for i = 1, 3 do
        local props = c['chev' .. i].props
        if charge > 0 then
            props.resource = CHEVRONS[k]
            props.size = util.vector2(14 * scale, 14 * scale)
            local d = r + (6 + (i - 1) * 7) * scale
            place(props, math.cos(data.angle) * d, math.sin(data.angle) * d)
            props.color = TEAL
            props.alpha = charge >= (i - 1) / 3 + 0.05 and 0.9 or 0.18
        else
            props.alpha = 0
        end
    end
    -- the balance wedges, eased so a single frame's jolt does not flash
    local tip = balance and balance.tip or 0
    for _, w in ipairs(WEIGHTS) do
        local props = c['weight_' .. w[1]].props
        local share = balance and math.max(0, (balance.x or 0) * w[2] + (balance.y or 0) * w[3]) * tip or 0
        local key = 'eased_' .. w[1]
        eased[key] = (eased[key] or 0) + (share - (eased[key] or 0)) * math.min(1, dt * 12)
        -- Shown from a quarter of the way to the edge (plain walking peaks
        -- near 0.25, ROUGH-01), full at the edge.
        local k = (eased[key] - 0.25) / 0.75
        if k > 0.02 then
            local s = (8 + 12 * k) * scale
            props.size = util.vector2(s, s)
            local d = r + s * 0.25
            place(props, w[2] * d, w[3] * d)
            props.color = util.color.rgb(GOLD.r + (AMBER.r - GOLD.r) * k, GOLD.g + (AMBER.g - GOLD.g) * k,
                GOLD.b + (AMBER.b - GOLD.b) * k)
            props.alpha = 0.35 + 0.6 * k
        else
            props.alpha = 0
        end
    end
    -- the cooldown bar: under the diamond, as wide as the diamond when ready
    do
        local props = c.cooldown.props
        local flash = now - readyAt < 0.2
        if drawn and (ready < 1 or flash) then
            local width = 2 * r * (flash and 1 or ready)
            props.size = util.vector2(math.max(1, width), 6 * scale)
            place(props, 0, -(r + 7 * scale))
            props.color = flash and WHITE or GOLD
            props.alpha = flash and 0.95 or 0.55
        else
            props.alpha = 0
        end
    end
    -- the swing path: the dots spread evenly over the engine's points, each
    -- put where it shows on screen now (the camera moves on as the cut runs)
    do
        local show = drawn and path ~= nil and hudSettings:get('reticleTrajectory') ~= false
        local fade = 1
        if show then
            -- The engine sends the path every frame while the attack is
            -- committed; after that the last one fades.
            local age = now - pathAt
            if age > 0.1 then fade = 1 - (age - 0.1) / TRAIL_KEEP end
            if fade <= 0 then path, show = nil, false end
        end
        local count = show and #path or 0
        local screen, eye, ahead
        if count >= 2 then
            screen = ui.screenSize()
            eye = camera.getPosition()
            ahead = camera.viewportToWorldVector(util.vector2(0.5, 0.5)):normalize()
        end
        for i = 1, TRAIL do
            local props = c['path' .. i].props
            local p
            if count >= 2 then
                local f = 1 + (i - 1) / (TRAIL - 1) * (count - 1)
                local a = math.min(count - 1, math.floor(f))
                p = path[a] + (path[a + 1] - path[a]) * (f - a)
            end
            if p and (p - eye):dot(ahead) > 5 then
                local v = camera.worldToViewportVector(p)
                local along = (i - 1) / (TRAIL - 1)
                local s = (3 + 3 * along) * scale
                props.relativePosition = util.vector2(v.x / screen.x, v.y / screen.y)
                props.position = util.vector2(0, 0)
                props.size = util.vector2(s, s)
                props.color = TEAL
                props.alpha = (0.25 + 0.6 * along) * fade * opacity
            else
                props.alpha = 0
            end
        end
    end
    for _, name in ipairs(PIECES) do
        local props = c[name].props
        props.alpha = props.alpha * opacity
    end
    root:update()
end

return M

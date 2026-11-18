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
--                 and the red snap of a cut). With the weight arrow on, the
--                 arrow shows it instead.
--   weight arrow  under the diamond, a brass arrow from a pivot point the way
--                 the weight is shifting over the feet (up: forward, right:
--                 right), as long as the shift is near the tipping edge, gold
--                 warming to amber past the comfortable stance; hidden in the
--                 air (user 11-03 B21: "the direction towards which the weight
--                 is being shifted over, like the overall body vector, and be
--                 under the crosshair"; the engine's capture point off the
--                 feet, hudWeightShift). The KINETIC page's "Weight arrow"
--                 turns it off and brings the wedges back.
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
--                 page's "Swing path" turns it off. With K2_AIM_PREVIEW a dragged
--                 wind-up shows it too, and a mark sits where the blade will first
--                 meet the foe (user 11-04: "can't tell where it'll land"): a teal
--                 diamond when the cut goes well into the body, an amber diamond
--                 when it only grazes (it may glance off), a dim amber dot at the
--                 closest pass when it falls short or wide. REACH-01, 54 clicks:
--                 8+ units in struck 27 of 27, grazes 5 of 15, passes 0 of 12.
--   mouse trace   while the attack button is held with the weapon out, the mouse's
--                 own path drawn from the reticle's centre (where the press began),
--                 rounded off: a faint gold trail for the wind-up, the cut a teal
--                 slash that swells and thins along its length, no text (user
--                 11-06); kept 1.5 s after the button comes up (user 11-05: "I swing horizontally in a perfect
--                 straight line ... the slash comes too high or too low"). The cut's
--                 line is read from the stroke's first ~30 ms; drift across the
--                 stroke after that steers the cut up or down (1500 counts a
--                 radian). Raw mouse counts (= the engine's at camera sensitivity
--                 1), fitted inside a 110 px circle. The KINETIC page's "Mouse
--                 trace" turns it off.
local camera = require('openmw.camera')
local input = require('openmw.input')
local self = require('openmw.self')
local storage = require('openmw.storage')
local types = require('openmw.types')
local ui = require('openmw.ui')
local util = require('openmw.util')

local sigil = require('kinetic_sigil') -- the balance sigil (K2_BALANCE_SIGIL), fed and drawn with the reticle

local M = {}
local GOLD = util.color.rgb(0.84, 0.70, 0.41)
local RED = util.color.rgb(0.94, 0.47, 0.35)
local TEAL = util.color.rgb(0.43, 0.84, 0.80)
local WHITE = util.color.rgb(1, 0.97, 0.90)
local AMBER = util.color.rgb(0.98, 0.62, 0.22)
local RED = util.color.rgb(0.92, 0.18, 0.12)
-- corner: its texture, its way out (x right, y up on screen = forward)
local WEIGHTS = { { 'top', 0, 1 }, { 'right', 1, 0 }, { 'bottom', 0, -1 }, { 'left', -1, 0 } }
local FULL = 1800 -- stance counts of a full wind-up (HSC_STANCE_RADIUS + COMMIT_COUNTS)
local OPEN, CLOSED, SHOVED = 15, 9, 10 -- diamond radius, pixels
local PARRY = 0.35 -- the parry window after a raise, s
local EDGES = { high = { 'tl', 'tr' }, left = { 'tl', 'bl' }, right = { 'tr', 'br' }, low = { 'bl', 'br' },
    middle = { 'tl', 'tr', 'br', 'bl' } }
local TRAIL = 24 -- swing path dots (the engine sends up to 32 points; every point drawn when fewer)
local TRAIL_KEEP = 0.7 -- s the last path stays after the attack's cut, fading
local ARROW_GAP, ARROW_LONG, ARROW_DOTS = 34, 32, 5 -- pivot below the diamond's edge, longest shaft (px), shaft dots
local ARROW_FROM = 0.12 -- share of the tipping edge before the arrow shows

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
local shift = util.vector2(0, 0) -- the weight arrow's eased shift, share of the tipping edge
local path, pathAt = nil, -10 -- the last swing path (world points) and when it last came
local mark, markHit, markGap = nil, false, 0 -- where that path meets the foe (world), a hit, its depth
local GRAZE = 8 -- units into the body below which a hit only grazes
-- The mouse trace (one table: LuaJIT allows a function 60 upvalues): the held
-- press's mouse path in counts (x right, y up; each point {x, y, view}, view:
-- made before the press was a drag), where the cut began in it, the cut's line
-- (radians, the engine's angle: the wind-up's side) and its kind.
local T = {
    DOTS = 48, -- trace dots
    SCALE = 0.04, FIT = 110, -- px per count, the farthest a point is drawn (px)
    KEEP = 1.5, FADE = 0.5, -- s kept after the release, of which the last fading
    CLAIM = 0, -- s after the press the mouse still turns the view (K2_HOLD_AIM: none; 0.16 before)
    points = nil, at = -10, held = false, pressAt = -10, x = 0, y = 0,
    cut = nil, angle = nil, line = nil,
}

local function texture(name) return ui.texture { path = 'textures/' .. name .. '.png' } end
local PIECES = { 'corners', 'centre', 'stance', 'edge_tl', 'edge_tr', 'edge_br', 'edge_bl', 'chev1', 'chev2', 'chev3',
    'weight_top', 'weight_right', 'weight_bottom', 'weight_left', 'cooldown', 'arrow_pivot', 'arrow_shadow', 'arrow_head',
    'mark' }
for i = 1, ARROW_DOTS do PIECES[#PIECES + 1] = 'arrow_dot' .. i end
local CHEVRONS = {}
for k = 0, 7 do CHEVRONS[k] = texture('kinetic_brass_chevron_' .. k) end
local MARK_HIT, MARK_SHORT = texture('kinetic_brass_corners'), texture('kinetic_brass_centre')
local ARROWS = {} -- solid heads, k * 22.5 degrees counter-clockwise from right
for k = 0, 15 do ARROWS[k] = texture('kinetic_brass_arrow_' .. k) end
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
    content[#content + 1] = image('arrow_pivot', 'kinetic_brass_centre', 7)
    for i = 1, ARROW_DOTS do content[#content + 1] = image('arrow_dot' .. i, 'kinetic_brass_centre', 4) end
    -- a dark copy of the head under it, so it reads on a bright shirt or sky
    content[#content + 1] = image('arrow_shadow', 'kinetic_brass_arrow_0', 18)
    content[#content + 1] = image('arrow_head', 'kinetic_brass_arrow_0', 18)
    for _, hand in ipairs({ 'left', 'right' }) do
        content[#content + 1] = image('fist_' .. hand, 'kinetic_brass_bar', 8)
        content[#content + 1] = { name = 'fist_' .. hand .. '_label', type = ui.TYPE.Text,
            props = { relativePosition = util.vector2(0.5, 0.5), anchor = util.vector2(0.5, 0.5),
                text = '', textSize = 14, textColor = GOLD, alpha = 0 } }
    end
    for i = 1, TRAIL do content[#content + 1] = image('path' .. i, 'kinetic_brass_centre', 5) end
    content[#content + 1] = image('mark', 'kinetic_brass_corners', 16)
    for i = 1, T.DOTS do content[#content + 1] = image('trace' .. i, 'kinetic_brass_centre', 4) end
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
    sigil.feed(balance)
    if data and data.trajectory and #data.trajectory >= 2 then
        path, pathAt, mark, markHit = data.trajectory, now, data.mark, data.markHit and true or false
        markGap = tonumber(data.markGap) or 0
    end
    local was = ready
    ready = tonumber(event.ready) or 1
    if ready >= 1 and was < 1 then readyAt = now end
end

local function place(props, x, y)
    props.position = util.vector2(x, -y)
end

-- The mouse trace: recorded while the attack button is held (not guarding),
-- drawn from the reticle's centre.
local function drawTrace(c, drawn, stage, rmb, scale, opacity)
    local held = drawn and data ~= nil and input.isMouseButtonPressed(1) and not rmb
    -- FIST-12: the right button is the left fist, wound up by the mouse too.
    if drawn and data and data.fistDrag then held = input.isMouseButtonPressed(1) ~= input.isMouseButtonPressed(3) end
    if held and not T.held then
        T.points, T.x, T.y, T.cut, T.angle, T.line = { { 0, 0, true } }, 0, 0, nil, nil, nil
        T.pressAt = now
    end
    T.held = held
    if held then
        T.at = now
        local mx, my = input.getMouseMoveX() or 0, -(input.getMouseMoveY() or 0)
        T.x, T.y = T.x + mx, T.y + my
        local last = T.points[#T.points]
        -- Before the press is a drag the mouse still turns the view (and
        -- with it the crosshair the cut is aimed through).
        local view = now - T.pressAt < T.CLAIM
        if (T.x - last[1]) ^ 2 + (T.y - last[2]) ^ 2 >= 16 or (last[3] and not view) then
            T.points[#T.points + 1] = { T.x, T.y, view }
        end
        if stage == 2 and data.angle then
            T.cut = T.cut or #T.points
            T.angle, T.line = data.angle, data.line
        end
    end
    local age = now - T.at
    local show = drawn and T.points ~= nil and #T.points >= 2 and hudSettings:get('reticleMouseTrace') ~= false
        and age < T.KEEP
    if T.points and age >= T.KEEP and not T.held then T.points = nil end
    -- arc length and the farthest point, in counts
    local total, far, along, dragged = 0, 0, { 0 }, 0
    if show then
        for i = 2, #T.points do
            local step = math.sqrt((T.points[i][1] - T.points[i - 1][1]) ^ 2 + (T.points[i][2] - T.points[i - 1][2]) ^ 2)
            total = total + step
            if not T.points[i][3] then dragged = dragged + step end
            along[i] = total
            far = math.max(far, math.sqrt(T.points[i][1] ^ 2 + T.points[i][2] ^ 2))
        end
        -- A click (under the engine's 40 counts of drag) leaves no trace.
        if dragged < 40 then show = false end
    end
    local fade = math.max(0, math.min(1, (T.KEEP - age) / T.FADE)) * opacity
    local k = T.SCALE * scale
    if show and far * k > T.FIT * scale then k = T.FIT * scale / far end
    -- The stroke, drawn smooth (user 11-06: "remove the text ... a smoother
    -- line than the one that draws the exact mouse movement"): the mouse's
    -- path rounded off (Chaikin, three passes), the wind-up a faint gold
    -- trail, the cut a teal slash that swells and thins along its length.
    -- The reticle's chevrons still show the wind-up's way.
    local function smooth(from, to)
        local pts = {}
        for i = from, to do pts[#pts + 1] = { T.points[i][1], T.points[i][2] } end
        for _ = 1, 3 do
            if #pts < 3 then break end
            local out = { pts[1] }
            for i = 1, #pts - 1 do
                local a, b = pts[i], pts[i + 1]
                out[#out + 1] = { 0.75 * a[1] + 0.25 * b[1], 0.75 * a[2] + 0.25 * b[2] }
                out[#out + 1] = { 0.25 * a[1] + 0.75 * b[1], 0.25 * a[2] + 0.75 * b[2] }
            end
            out[#out + 1] = pts[#pts]
            pts = out
        end
        local lengths, sum = { 0 }, 0
        for i = 2, #pts do
            sum = sum + math.sqrt((pts[i][1] - pts[i - 1][1]) ^ 2 + (pts[i][2] - pts[i - 1][2]) ^ 2)
            lengths[i] = sum
        end
        return pts, lengths, sum
    end
    local function at(pts, lengths, sum, u)
        local s = sum * u
        local i = 2
        while i < #pts and lengths[i] < s do i = i + 1 end
        local a, b = pts[i - 1], pts[i] or pts[i - 1]
        local span = (lengths[i] or s) - lengths[i - 1]
        local f = span > 0 and (s - lengths[i - 1]) / span or 0
        return a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f
    end
    local cutFrom = show and T.cut ~= nil and T.cut < #T.points and T.cut or nil
    local windDots = cutFrom and 14 or T.DOTS
    local wPts, wLen, wSum, cPts, cLen, cSum
    if show then
        wPts, wLen, wSum = smooth(1, cutFrom or #T.points)
        if cutFrom then cPts, cLen, cSum = smooth(cutFrom, #T.points) end
    end
    for i = 1, T.DOTS do
        local props = c['trace' .. i].props
        if show and i <= windDots then
            local x, y = at(wPts, wLen, wSum, (i - 1) / math.max(1, windDots - 1))
            place(props, x * k, y * k)
            props.size = util.vector2(3 * scale, 3 * scale)
            props.color = GOLD
            props.alpha = 0.35 * fade
        elseif show and cutFrom and cSum > 0 then
            local u = (i - windDots - 1) / math.max(1, T.DOTS - windDots - 1)
            local x, y = at(cPts, cLen, cSum, u)
            place(props, x * k, y * k)
            local swell = math.sin(math.pi * math.min(1, math.max(0, u)))
            local size = (2 + 5 * swell) * scale
            props.size = util.vector2(size, size)
            props.color = TEAL
            props.alpha = (0.35 + 0.6 * swell) * fade
        else
            props.alpha = 0
        end
    end
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
    if rmb and not rmbWas and drawn and not (data and data.fists) then
        if now - lastPress < 0.3 then shoveAt, lastPress = now, -10 else lastPress = now end
    end
    rmbWas = rmb
    -- FIST-12 (fistDrag): the fists wind up by the mouse and show the
    -- weapons' wind-up (the stance point, the chevrons, the stroke).
    local fists = drawn and data and data.fists and not data.fistDrag
    local stage = data and data.stage or 0
    if fists then path, mark = nil, nil end
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
    if drawn and data and data.x and not fists then
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
    if drawn and data and stage == 1 and data.angle and not fists then
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
    local arrowOn = hudSettings:get('reticleWeightArrow') ~= false
    -- the balance wedges, eased so a single frame's jolt does not flash
    local tip = (balance and not arrowOn) and balance.tip or 0
    -- K2_BALANCE_RISK: all four wedges together are the about-to-fall warning,
    -- amber when off balance, red and pulsing when going over.
    local risk = balance and balance.risk or 0
    eased.risk = (eased.risk or 0) + (risk - (eased.risk or 0)) * math.min(1, dt * (risk > (eased.risk or 0) and 18 or 5))
    eased.riskClock = (eased.riskClock or 0) + dt
    local warning = eased.risk > 0.02
    for _, w in ipairs(WEIGHTS) do
        local props = c['weight_' .. w[1]].props
        local share = balance and math.max(0, (balance.x or 0) * w[2] + (balance.y or 0) * w[3]) * tip or 0
        local key = 'eased_' .. w[1]
        eased[key] = (eased[key] or 0) + (share - (eased[key] or 0)) * math.min(1, dt * 12)
        -- Shown from a quarter of the way to the edge (plain walking peaks
        -- near 0.25, ROUGH-01), full at the edge.
        local k = (eased[key] - 0.25) / 0.75
        if warning then
            local level = eased.risk
            local pulse = level > 0.7 and (0.5 + 0.5 * math.sin(eased.riskClock * 14)) or 1
            local s = (10 + 14 * level) * scale
            props.size = util.vector2(s, s)
            local d = r + 2 * scale + s * 0.25
            place(props, w[2] * d, w[3] * d)
            props.color = util.color.rgb(AMBER.r + (RED.r - AMBER.r) * level, AMBER.g + (RED.g - AMBER.g) * level,
                AMBER.b + (RED.b - AMBER.b) * level)
            props.alpha = (0.45 + 0.5 * level) * (0.55 + 0.45 * pulse)
        elseif k > 0.02 then
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
    -- the weight arrow: eased so a step's jolt reads as a lean, not a flicker
    do
        local edge = balance and balance.edge or 0
        local want = util.vector2(0, 0)
        if arrowOn and edge > 0 then
            want = util.vector2(balance.shiftX or 0, balance.shiftY or 0) / edge
            if want:length() > 1 then want = want:normalize() end
        end
        shift = shift + (want - shift) * math.min(1, dt * 10)
        local amount = shift:length()
        local k = math.max(0, math.min(1, (amount - ARROW_FROM) / (1 - ARROW_FROM)))
        local pivotY = -(r + ARROW_GAP * scale)
        local pieces = { c.arrow_pivot.props, c.arrow_head.props }
        local shadow = c.arrow_shadow.props
        for i = 1, ARROW_DOTS do pieces[#pieces + 1] = c['arrow_dot' .. i].props end
        if k > 0.02 then
            -- gold, warming to amber past the comfortable stance (35 of the
            -- edge's units beyond it, hudBalance)
            local warm = math.max(0, math.min(1, (amount - 0.45) / 0.55))
            local colour = util.color.rgb(GOLD.r + (AMBER.r - GOLD.r) * warm, GOLD.g + (AMBER.g - GOLD.g) * warm,
                GOLD.b + (AMBER.b - GOLD.b) * warm)
            local way = shift / amount
            local long = (6 + ARROW_LONG * amount) * scale
            local alpha = 0.3 + 0.65 * k
            place(c.arrow_pivot.props, 0, pivotY)
            c.arrow_pivot.props.size = util.vector2(7 * scale, 7 * scale)
            for i = 1, ARROW_DOTS do
                local props = c['arrow_dot' .. i].props
                local at = long * i / (ARROW_DOTS + 1)
                place(props, way.x * at, pivotY + way.y * at)
                props.size = util.vector2(4 * scale, 4 * scale)
            end
            local head = c.arrow_head.props
            local angle = math.atan2(way.y, way.x)
            head.resource = ARROWS[math.floor(angle / (math.pi / 8) + 0.5) % 16]
            head.size = util.vector2((16 + 6 * k) * scale, (16 + 6 * k) * scale)
            place(head, way.x * long, pivotY + way.y * long)
            for _, props in ipairs(pieces) do
                props.color = colour
                props.alpha = alpha
            end
            shadow.resource = head.resource
            shadow.size = util.vector2(head.size.x + 4 * scale, head.size.y + 4 * scale)
            place(shadow, way.x * long + scale, pivotY + way.y * long - scale)
            shadow.color = util.color.rgb(0.08, 0.06, 0.04)
            shadow.alpha = 0.55 * alpha
        else
            for _, props in ipairs(pieces) do props.alpha = 0 end
            shadow.alpha = 0
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
        -- the mark: where the blade will first meet the foe (a hit), or its
        -- closest pass (short or wide)
        local props = c.mark.props
        if count >= 2 and mark and (mark - eye):dot(ahead) > 5 then
            local v = camera.worldToViewportVector(mark)
            props.relativePosition = util.vector2(v.x / screen.x, v.y / screen.y)
            props.position = util.vector2(0, 0)
            if markHit then
                local sure = markGap <= -GRAZE
                props.resource = MARK_HIT
                props.size = util.vector2((sure and 16 or 14) * scale, (sure and 16 or 14) * scale)
                props.color = sure and TEAL or AMBER
                props.alpha = (sure and 0.95 or 0.8) * fade
            else
                props.resource = MARK_SHORT
                props.size = util.vector2(7 * scale, 7 * scale)
                props.color = AMBER
                props.alpha = 0.7 * fade
            end
        else
            props.alpha = 0
            if not path then mark = nil end
        end
    end
    -- Both fist controls get their own charge feedback; RMB is a punch,
    -- not a guard, here. These values come from the actual charge controller.
    local names = { [0] = 'Straight', [1] = 'Hook', [2] = 'Uppercut', [3] = 'Body' }
    for side, hand in ipairs({ 'left', 'right' }) do
        local value = fists and tonumber(data[hand .. 'Charge']) or -1
        local show = fists and not guard and value ~= nil and value >= 0
        local bar, label = c['fist_' .. hand].props, c['fist_' .. hand .. '_label'].props
        local x = (side == 1 and -85 or 85) * scale
        bar.alpha, label.alpha = show and opacity or 0, show and opacity or 0
        if show then
            value = math.max(0, math.min(1, value))
            bar.size = util.vector2(math.max(3, 64 * value) * scale, 5 * scale)
            place(bar, x, -30 * scale)
            bar.color = value >= 0.99 and WHITE or TEAL
            place(label, x, -48 * scale)
            label.textSize = 14 * scale
            label.text = (side == 1 and 'L: ' or 'R: ') .. (names[data[hand .. 'Punch']] or 'Straight')
            label.textColor = value >= 0.99 and WHITE or GOLD
        end
    end
    -- A view movement is not a fist wind-up, so never draw it as a mouse stroke.
    if fists then T.points, T.held = nil, false end
    drawTrace(c, drawn and not fists, stage,
        (rmb and not (data and data.fistDrag)) or input.isMouseButtonPressed(2), scale, opacity)
    for _, name in ipairs(PIECES) do
        local props = c[name].props
        props.alpha = props.alpha * opacity
    end
    sigil.frame(dt, opacity)
    root:update()
end

return M

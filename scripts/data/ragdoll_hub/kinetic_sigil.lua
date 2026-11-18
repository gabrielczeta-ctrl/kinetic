-- The balance sigil (K2_BALANCE_SIGIL, user 11-12: "some sort of balance
-- indicator that tells the player the foot weight distribution, the momentum,
-- maybe center of mass vectors ... old alchemical artifact style ... black and
-- gold ... the same style as the lines in the border for the menu"). A small
-- black panel at the bottom of the screen, framed like a Morrowind window:
--   triangle   the three senses that keep a body up, a tile at each corner:
--              the eye (top, dark silver), the labyrinth (the inner ear, left,
--              gold), the boot (footing, right, bronze). A tile dims and gets
--              a red border while its sense is impaired: the eye by a Blind
--              effect, the labyrinth by a blow's stun or a stagger, the boot
--              by fatigue under a quarter. Display only: none of them
--              changes the body's balance.
--   moons      the feet seen from above (up: forward), each filled by the
--              share of the weight it carries; a lifted foot is an empty ring
--   sun        the centre of mass over the ground
--   spear      from the sun toward the star: the body's momentum
--   star       the capture point, where a step would have to go to stop;
--              silver inside the dotted bound round the feet, red outside
--              it (step or fall), pulsing when the body is going over
-- Fed with the reticle (kinetic_reticle.lua: balance.sigil from the engine's
-- RagdollCore::balanceSigil, balance.risk). OFF since 11-17 (the user, after
-- play: off the screen): the switch's default is 0, so the engine sends no
-- stance and nothing is created. Its two options (the 'balanceSigil'
-- checkbox and 'balanceSigilSize', still read below; a profile that saw
-- them keeps their stored values) left the KINETIC page with it. The
-- reticle's opacity applies. Textures: tools/body/sigil_textures.py.
local self = require('openmw.self')
local storage = require('openmw.storage')
local types = require('openmw.types')
local ui = require('openmw.ui')
local util = require('openmw.util')

local M = {}
local hudSettings = storage.playerSection('SettingsKineticHud')

local PLAIN = util.color.rgb(1, 1, 1)
local GOLD = util.color.rgb(0.83, 0.69, 0.35)
local SUN = util.color.rgb(0.94, 0.84, 0.53)
local SILVER = util.color.rgb(0.64, 0.66, 0.69)
local BRONZE = util.color.rgb(0.71, 0.49, 0.26)
local RED = util.color.rgb(0.78, 0.28, 0.16)

-- The design's own units: a 280 square (sigil_textures.py's frame).
local DESIGN, PIXELS = 280, 150 -- and its size on screen at size 1
local CENTRE = util.vector2(140, 172) -- the feet's middle
local SENSES = { eye = util.vector2(140, 44), maze = util.vector2(44, 226), boot = util.vector2(236, 226) }
local PER_UNIT = 1.3 -- design units per game unit
local FAR = 48 -- the farthest anything is drawn from the feet's middle
local PAD = 12 -- game units round the feet inside which the capture point is safe
local BOUND, SPEAR = 20, 3 -- dots of the bound, of the spear's shaft

local root = nil
local sigil, risk = nil, 0
local eased = { shown = 0, left = util.vector2(-10, 0), right = util.vector2(10, 0), com = util.vector2(0, 0),
    tip = util.vector2(0, 0), leftLoad = 0.5, rightLoad = 0.5, out = 0, eye = 0, maze = 0, boot = 0, clock = 0 }

local function texture(name) return ui.texture { path = 'textures/kinetic_brass_' .. name .. '.png' } end
local MOONS = {}
for k = 0, 8 do MOONS[k] = texture('sigil_moon_' .. k) end
local ARROWS = {} -- solid heads, k * 22.5 degrees counter-clockwise from right
for k = 0, 15 do ARROWS[k] = texture('arrow_' .. k) end

local function image(name, tex, colour)
    return { name = name, type = ui.TYPE.Image, props = { resource = texture(tex), size = util.vector2(1, 1),
        anchor = util.vector2(0.5, 0.5), color = colour or PLAIN, alpha = 0 } }
end

local function create()
    local content = { image('frame', 'sigil_frame') }
    for i = 1, BOUND do content[#content + 1] = image('bound' .. i, 'centre', BRONZE) end
    for _, name in ipairs({ 'eye', 'maze', 'boot' }) do
        content[#content + 1] = image(name, 'sigil_' .. name)
        content[#content + 1] = image(name .. '_alert', 'sigil_alert', RED)
    end
    content[#content + 1] = image('moon_left', 'sigil_moon_4', GOLD)
    content[#content + 1] = image('moon_right', 'sigil_moon_4', GOLD)
    for i = 1, SPEAR do content[#content + 1] = image('spear' .. i, 'centre', SILVER) end
    content[#content + 1] = image('spear_head', 'arrow_0', SILVER)
    content[#content + 1] = image('sun', 'sigil_sun', SUN)
    content[#content + 1] = image('star', 'sigil_star', SILVER)
    root = ui.create { layer = 'HUD', type = ui.TYPE.Widget,
        props = { size = util.vector2(PIXELS, PIXELS), relativePosition = util.vector2(0.74, 1),
            anchor = util.vector2(0.5, 1), position = util.vector2(0, -12) },
        content = ui.content(content) }
end

function M.feed(balance)
    sigil = balance and balance.sigil or nil
    risk = balance and balance.risk or 0
end

local function clamp01(v) return math.max(0, math.min(1, v)) end
local function ease(key, want, rate, dt)
    eased[key] = eased[key] + (want - eased[key]) * math.min(1, dt * rate)
    return eased[key]
end
-- A point of the stance (game units; x right, y forward) in design units
-- from the feet's middle (y down), held within `far`.
local function stance(x, y, far)
    local v = util.vector2(x * PER_UNIT, -y * PER_UNIT)
    local length = v:length()
    if length > far then v = v * (far / length) end
    return v
end

-- How impaired each sense is, 0..1 (display only).
local function senses()
    local eye, boot = 0, 0
    pcall(function()
        local blind = types.Actor.activeEffects(self):getEffect('blind')
        eye = clamp01((blind and blind.magnitude or 0) / 100)
    end)
    pcall(function()
        local fatigue = types.Actor.stats.dynamic.fatigue(self)
        if fatigue.base > 0 then boot = clamp01((0.25 - fatigue.current / fatigue.base) / 0.25) end
    end)
    return eye, sigil and sigil.daze or 0, boot
end

function M.frame(dt, opacity)
    local on = hudSettings:get('balanceSigil') ~= false
    if not root then
        if not (on and sigil) then return end
        create()
    end
    local c = root.layout.content
    local size = math.max(0.5, math.min(2.5, tonumber(hudSettings:get('balanceSigilSize')) or 1))
    local side = PIXELS * size
    local f = side / DESIGN
    root.layout.props.size = util.vector2(side, side)
    -- The panel stays while the engine sends a stance; in the air (no
    -- stance) its contents fade and the panel dims.
    local shown = ease('shown', (on and sigil) and 1 or 0, 6, dt)
    local alpha = (on and 1 or 0) * opacity
    local function put(name, at, pixels, a, colour)
        local props = c[name].props
        props.position = at * f
        props.size = util.vector2(pixels * f, pixels * f)
        props.alpha = a * alpha
        if colour then props.color = colour end
        return props
    end
    put('frame', util.vector2(DESIGN / 2, DESIGN / 2), DESIGN, 0.45 + 0.55 * shown)

    -- the senses
    local eye, maze, boot = senses()
    local impaired = { eye = ease('eye', eye, 8, dt), maze = ease('maze', maze, 8, dt), boot = ease('boot', boot, 8, dt) }
    for name, at in pairs(SENSES) do
        local level = impaired[name]
        put(name, at, 52, (0.45 + 0.55 * shown) * (1 - 0.6 * level))
        put(name .. '_alert', at, 52, level)
    end

    -- the stance
    if sigil then
        local rate = 14
        eased.left = eased.left + (stance(sigil.leftX, sigil.leftY, 40) - eased.left) * math.min(1, dt * rate)
        eased.right = eased.right + (stance(sigil.rightX, sigil.rightY, 40) - eased.right) * math.min(1, dt * rate)
        eased.com = eased.com + (stance(sigil.comX, sigil.comY, FAR) - eased.com) * math.min(1, dt * rate)
        eased.tip = eased.tip + (stance(sigil.tipX, sigil.tipY, FAR) - eased.tip) * math.min(1, dt * rate)
        ease('leftLoad', sigil.leftLoad, 10, dt)
        ease('rightLoad', sigil.rightLoad, 10, dt)
        -- outside the bound: the capture point farther than PAD from the
        -- line between the feet
        local lx, ly, rx, ry = sigil.leftX, sigil.leftY, sigil.rightX, sigil.rightY
        local ax, ay = rx - lx, ry - ly
        local span2 = ax * ax + ay * ay
        local t = span2 > 1e-3 and clamp01(((sigil.tipX - lx) * ax + (sigil.tipY - ly) * ay) / span2) or 0.5
        local gap = math.sqrt((sigil.tipX - (lx + ax * t)) ^ 2 + (sigil.tipY - (ly + ay * t)) ^ 2)
        ease('out', gap > PAD and 1 or 0, 10, dt)
    end
    eased.clock = eased.clock + dt
    local feet = { { 'moon_left', eased.left, eased.leftLoad }, { 'moon_right', eased.right, eased.rightLoad } }
    for _, foot in ipairs(feet) do
        local phase = math.floor(clamp01(foot[3]) * 8 + 0.5)
        local props = put(foot[1], CENTRE + foot[2], 22, shown * (phase == 0 and 0.45 or 0.95))
        props.resource = MOONS[phase]
    end
    -- the bound: dots round the two feet, PAD game units out
    do
        local axis = eased.right - eased.left
        local length = axis:length()
        local a = length > 0.5 and axis / length or util.vector2(1, 0)
        local n = util.vector2(-a.y, a.x)
        local pad = PAD * PER_UNIT + 4
        for i = 1, BOUND do
            local angle = 2 * math.pi * (i - 0.5) / BOUND
            local cs, sn = math.cos(angle), math.sin(angle)
            local from = cs >= 0 and eased.right or eased.left
            put('bound' .. i, CENTRE + from + (a * cs + n * sn) * pad, 2.6, shown * (0.35 + 0.4 * eased.out),
                util.color.rgb(BRONZE.r + (RED.r - BRONZE.r) * eased.out, BRONZE.g + (RED.g - BRONZE.g) * eased.out,
                    BRONZE.b + (RED.b - BRONZE.b) * eased.out))
        end
    end
    -- the spear and the star
    do
        local shaft = eased.tip - eased.com
        local long = shaft:length()
        local show = long > 7 and shown or 0
        local way = long > 0.01 and shaft / long or util.vector2(0, -1)
        for i = 1, SPEAR do
            put('spear' .. i, CENTRE + eased.com + way * (4 + (long - 14) * i / (SPEAR + 1)), 2.8, show * 0.85)
        end
        local head = put('spear_head', CENTRE + eased.com + way * math.max(4, long - 9), 9, show * 0.9)
        head.resource = ARROWS[math.floor(math.atan2(-way.y, way.x) / (math.pi / 8) + 0.5) % 16]
        local pulse = risk > 0.7 and (0.6 + 0.4 * math.sin(eased.clock * 14)) or 1
        local out = eased.out
        put('star', CENTRE + eased.tip, 16 + 4 * out, shown * (0.75 + 0.25 * out) * pulse,
            util.color.rgb(SILVER.r + (RED.r - SILVER.r) * out, SILVER.g + (RED.g - SILVER.g) * out,
                SILVER.b + (RED.b - SILVER.b) * out))
        put('sun', CENTRE + eased.com, 13, shown)
    end
    root:update()
end

return M

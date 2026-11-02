local util = require('openmw.util')
-- Fresh newGame per case. Both animation modes receive these exact controls.
local function segment(name, duration, movement, side, yaw, jump)
    return { segment = name, duration = duration, movement = movement or 0,
        sideMovement = side or 0, yawRate = yaw or 0, jump = jump or false }
end
local function sneak(name, duration, movement, side, yaw)
    local value = segment(name, duration, movement, side, yaw)
    value.sneak = true
    return value
end
local function run(name, duration, movement, side, yaw)
    local value = segment(name, duration, movement, side, yaw)
    value.run = true
    return value
end
local function straight(name, movement, side)
    return { name = name, segments = {
        segment('idle', 2), segment('travel', 8, movement, side), segment('stop', 3),
    } }
end
local function heading(name, initialYaw, segments, control)
    return { name = name, initialYaw = initialYaw, segments = segments, control = control }
end
local quickQuarter = math.pi * 2
local quickHalf = math.pi * 4
local protocols = {
    { name = 'no input', segments = { segment('idle', 12) } },
    straight('forward', 1, 0), straight('backward', -1, 0),
    straight('strafe left', 0, -1), straight('strafe right', 0, 1),
    straight('diagonal left', 1, -1), straight('diagonal right', 1, 1),
    { name = 'start stop', segments = {
        segment('idle', 2), segment('start', 4, 1), segment('stop', 3),
        segment('restart', 4, 1), segment('stop again', 3),
    } },
    -- Turn before the staircase so this stays a flat-floor turn case; the
    -- stairs-turn behaviour is covered by `stairs up and back` and the side
    -- drop by `stairs drop off`.
    { name = 'moving turn', segments = {
        segment('idle', 2), segment('forward', 1.5, 1),
        segment('quarter turn', 2, 1, 0, math.pi / 4),
        segment('new heading', 2, 1), segment('stop', 3),
    } },
    { name = 'reversal', segments = {
        segment('idle', 2), segment('forward', 4, 1),
        segment('backward', 4, -1), segment('stop', 3),
    } },
    -- The staircase sits at y=400 on the forward path and rises 128 units over
    -- 256 units of travel before returning to the floor. The case walks up and
    -- over, stops, walks back over it and stops.
    -- The staircase sits off the corridor at (875, 875) with its ascent along
    -- +X. Diagonal right reaches its foot, then the strafe walks up and back.
    { name = 'stairs up and back', segments = {
        segment('idle', 2), segment('diagonal', 9.5, 1, 1),
        segment('climb', 5, 0, 1), segment('descend', 5, 0, -1),
        segment('return', 9.5, -1, -1), segment('stop', 2),
    } },
    -- Play test 2026-09-24 M6: drawing or sheathing while walking broke the
    -- leg animation. Walk, draw while walking, walk, sheathe while walking.
    { name = 'forward draw sheathe', segments = {
        segment('idle', 2), segment('walk', 2, 1),
        { segment = 'draw', duration = 2, movement = 1, sideMovement = 0, yawRate = 0, jump = false, stance = 'weapon' },
        { segment = 'armed', duration = 2, movement = 1, sideMovement = 0, yawRate = 0, jump = false, stance = 'weapon' },
        { segment = 'sheathe', duration = 2, movement = 1, sideMovement = 0, yawRate = 0, jump = false, stance = 'nothing' },
        segment('stop', 2),
    } },
    -- Play test 2026-09-24 N9: running with the hammer looked stiff until the
    -- weapon was sheathed and drawn again. Run as spawned, then redraw.
    { name = 'run redraw', segments = {
        segment('idle', 2),
        { segment = 'run spawned', duration = 4, movement = 1, sideMovement = 0, yawRate = 0, jump = false, run = true },
        { segment = 'run sheathed', duration = 2, movement = 1, sideMovement = 0, yawRate = 0, jump = false, run = true, stance = 'nothing' },
        { segment = 'run drawn', duration = 2, movement = 1, sideMovement = 0, yawRate = 0, jump = false, run = true, stance = 'weapon' },
        { segment = 'run redrawn', duration = 4, movement = 1, sideMovement = 0, yawRate = 0, jump = false, run = true, stance = 'weapon' },
        segment('stop', 2),
    } },
    -- Play test 2026-09-24 R2: "after landing the jump the character does a
    -- small hop". Standing jump, so steps do not mask the landing.
    { name = 'jump in place', segments = {
        segment('idle', 2), segment('jump', 0.2, 0, 0, 0, true), segment('land', 2),
    } },
    { name = 'jump forward', segments = {
        segment('idle', 2), segment('forward', 0.8, 1),
        segment('jump', 0.2, 1, 0, 0, true), segment('land', 1.2, 1),
        segment('stop', 2),
    } },
    -- Reach the staircase, climb part way, then walk off its side (128-unit
    -- drop). The landing speed is above the registered threshold, so the root
    -- must hand over to the passive ragdoll and recovery.
    { name = 'stairs drop off', segments = {
        segment('idle', 2), segment('diagonal', 9.5, 1, 1),
        segment('climb', 3, 0, 1), segment('step off', 2.5, 1, 0), segment('stop', 10),
    } },
    -- STAIRS-01 (user bookmark 9, 2026-09-30: "running up stairs too
    -- quickly causes some weird drag"): the stairs-drop-off approach, then the
    -- climb at a run and on over the top.
    { name = 'STAIRS-01 run up', segments = {
        segment('idle', 2),
        { segment = 'diagonal', duration = 4.8, movement = 1, sideMovement = 1, yawRate = 0, jump = false, run = true },
        { segment = 'climb', duration = 2.5, movement = 0, sideMovement = 1, yawRate = 0, jump = false, run = true },
        segment('stop', 3),
    } },
    -- RUNBACK-01 (user 2026-09-30: "something about running back makes the
    -- legs start dragging, like they both start skating and the torso flies
    -- around"; play bookmarks: both feet pinned while running backwards).
    { name = 'RUNBACK-01 run backward', segments = {
        segment('idle', 2),
        { segment = 'run back', duration = 6, movement = -1, sideMovement = 0, yawRate = 0, jump = false, run = true },
        segment('stop', 3),
    } },
    { name = 'RUNBACK-01 run back diagonal', segments = {
        segment('idle', 2),
        { segment = 'run back', duration = 6, movement = -1, sideMovement = -1, yawRate = 0, jump = false, run = true },
        segment('stop', 3),
    } },
    { name = 'RUNBACK-01 run forward', segments = {
        segment('idle', 2),
        { segment = 'run fwd', duration = 6, movement = 1, sideMovement = 0, yawRate = 0, jump = false, run = true },
        segment('stop', 3),
    } },
    -- SNEAK-01 (alpha 0.6 item 6, crouch): stand, crouch in place, sneak
    -- forward and sideways, stop crouched, stand up.
    { name = 'SNEAK-01 sneak', segments = {
        segment('idle', 2),
        { segment = 'crouch', duration = 2, movement = 0, sideMovement = 0, yawRate = 0, jump = false, sneak = true },
        { segment = 'sneak fwd', duration = 5, movement = 1, sideMovement = 0, yawRate = 0, jump = false, sneak = true },
        { segment = 'sneak side', duration = 3, movement = 0, sideMovement = 1, yawRate = 0, jump = false, sneak = true },
        { segment = 'crouch stop', duration = 2, movement = 0, sideMovement = 0, yawRate = 0, jump = false, sneak = true },
        segment('stand', 2),
    } },
    -- CROUCH-01 (11-01, docs/ragdoll/designs/CROUCH-01-physical-crouch.md):
    -- crouching in place, turning crouched, sneaking each way and the
    -- transitions in and out of it standing and on the move.
    -- tools/body/crouch_report.py measures them against the clip.
    { name = 'CROUCH-01 crouch in place', segments = {
        segment('idle', 2), sneak('crouch', 4), segment('stand', 3),
    } },
    { name = 'CROUCH-01 crouch turn', segments = {
        segment('idle', 2), sneak('crouch', 2),
        sneak('turn left', 3, 0, 0, math.pi / 2), sneak('turn right', 3, 0, 0, -math.pi / 2),
        sneak('crouch stop', 2), segment('stand', 2),
    } },
    { name = 'CROUCH-01 sneak forward', segments = {
        segment('idle', 2), sneak('crouch', 2), sneak('sneak', 6, 1), sneak('crouch stop', 3), segment('stand', 2),
    } },
    { name = 'CROUCH-01 sneak backward', segments = {
        segment('idle', 2), sneak('crouch', 2), sneak('sneak', 5, -1), sneak('crouch stop', 3), segment('stand', 2),
    } },
    { name = 'CROUCH-01 sneak strafe', segments = {
        segment('idle', 2), sneak('crouch', 2), sneak('sneak', 5, 0, 1), sneak('crouch stop', 3), segment('stand', 2),
    } },
    { name = 'CROUCH-01 sneak diagonal', segments = {
        segment('idle', 2), sneak('crouch', 2), sneak('sneak', 5, 1, -1), sneak('crouch stop', 3), segment('stand', 2),
    } },
    { name = 'CROUCH-01 sneak turning', segments = {
        segment('idle', 2), sneak('crouch', 2), sneak('sneak', 2, 1), sneak('sneak turn', 4, 1, 0, math.pi / 4),
        sneak('crouch stop', 3), segment('stand', 2),
    } },
    { name = 'CROUCH-01 walk into sneak', segments = {
        segment('idle', 2), segment('walk', 3, 1), sneak('sneak', 4, 1), segment('walk again', 3, 1),
        segment('stop', 3),
    } },
    { name = 'CROUCH-01 run into sneak', segments = {
        segment('idle', 2), run('run', 3, 1), sneak('sneak', 4, 1), segment('stop', 3),
    } },
    -- JUMP-01 (alpha 0.5, user 2026-09-30: "create a test for jumping off a
    -- ledge"): climb the staircase part way (about 64 units up, or to the
    -- 128-unit top), then jump off its side moving forward; land and stand.
    -- A body that lands on its feet should not fall.
    { name = 'JUMP-01 ledge jump low', segments = {
        segment('idle', 2), segment('diagonal', 9.5, 1, 1),
        segment('climb', 1.5, 0, 1), segment('jump off', 0.2, 1, 0, 0, true),
        segment('flight', 1.3, 1, 0), segment('stop', 6),
    } },
    { name = 'JUMP-01 ledge jump high', segments = {
        segment('idle', 2), segment('diagonal', 9.5, 1, 1),
        segment('climb', 3, 0, 1), segment('jump off', 0.2, 1, 0, 0, true),
        segment('flight', 1.3, 1, 0), segment('stop', 6),
    } },
    { name = 'JUMP-01 ledge jump high running', segments = {
        segment('idle', 2), segment('diagonal', 9.5, 1, 1),
        segment('climb', 3, 0, 1),
        { segment = 'run off', duration = 0.4, movement = 1, sideMovement = 0, yawRate = 0, jump = false, run = true },
        { segment = 'jump off', duration = 0.2, movement = 1, sideMovement = 0, yawRate = 0, jump = true, run = true },
        { segment = 'flight', duration = 1.3, movement = 1, sideMovement = 0, yawRate = 0, jump = false, run = true },
        segment('stop', 6),
    } },
    heading('heading screen idle zero', 0, { segment('idle', 12) }),
    heading('heading screen idle plus 90', math.pi / 2, { segment('idle', 12) }),
    heading('heading screen quarter left', 0, {
        segment('idle', 2), segment('turn', 0.25, 0, 0, quickQuarter), segment('settle', 3),
    }),
    heading('heading screen quarter right', 0, {
        segment('idle', 2), segment('turn', 0.25, 0, 0, -quickQuarter), segment('settle', 3),
    }),
    heading('heading screen half exact tie', 0, {
        segment('idle', 2), segment('turn', 0.25, 0, 0, quickHalf), segment('settle', 3),
    }),
    heading('heading screen wrap plus 179 to minus 179', math.rad(179), {
        segment('idle', 2), segment('turn', 0.25, 0, 0, math.rad(8)), segment('settle', 3),
    }),
    heading('heading screen straight forward fixed facing', 0, {
        segment('idle', 2), segment('forward', 8, 1), segment('stop', 3),
    }),
    heading('heading screen strafe left turn right', 0, {
        segment('idle', 2), segment('strafe', 2, 0, -1),
        segment('turn', 0.25, 0, -1, -quickQuarter), segment('new heading', 4, 0, -1), segment('stop', 3),
    }),
    heading('heading screen backpedal turn left', 0, {
        segment('idle', 2), segment('backpedal', 2, -1),
        segment('turn', 0.25, -1, 0, quickQuarter), segment('new heading', 4, -1), segment('stop', 3),
    }),
    heading('heading screen mid turn impact release reacquire', 0, {
        segment('idle', 2), segment('forward', 2, 1),
        { segment = 'turn and impact', duration = 0.25, movement = 1,
            sideMovement = 0, yawRate = quickQuarter, impactAt = 0.1 },
        segment('reacquire', 4, 1), segment('stop', 3),
    }),
    heading('heading control yaw off authority', 0, {
        segment('idle', 2), segment('turn', 0.25, 0, 0, quickQuarter), segment('settle', 3),
    }, 'maximum_animation_assist_yaw_torque=0'),
    heading('heading control damping off settling', 0, {
        segment('idle', 2), segment('turn', 0.25, 0, 0, quickQuarter),
        { segment = 'yaw disturbance', duration = 1, disturbanceAt = 0.1 }, segment('settle', 3),
    }, 'animation_assist_yaw_damping=0'),
}
-- Push sweep: chest (spine2) hits from the front, back and side while
-- standing and while walking. Bands: stun 400, stagger 1250, fall 1750.
-- 4000 and 5000 (2026-09-29) sit between the pushes the body always catches
-- and the 6000 knockdown: the topple zone, where a fall is allowed but a
-- body that stays up must end upright.
local dirs = { { 'front', 0, -1 }, { 'back', 0, 1 }, { 'side', 1, 0 } }
for _, strength in ipairs({ 600, 1200, 2000, 3000, 4000, 5000 }) do
    for _, d in ipairs(dirs) do
        local impulse = { d[2] * strength, d[3] * strength }
        protocols[#protocols + 1] = { name = 'push stand ' .. d[1] .. ' ' .. strength, allowFall = strength > 3000,
            segments = {
            segment('idle', 2),
            { segment = 'push', duration = 3, movement = 0, sideMovement = 0, yawRate = 0, jump = false,
                impactAt = 0.1, impactBody = 'spine2', impactImpulse = impulse,
                screenshots = strength == 3000 and { 0.05, 0.2, 0.35, 0.5, 0.8, 1.3 } or nil },
            segment('settle', strength > 3000 and 8 or 3),
        } }
        protocols[#protocols + 1] = { name = 'push walk ' .. d[1] .. ' ' .. strength, allowFall = strength > 3000,
            segments = {
            segment('idle', 1), segment('forward', 2, 1),
            { segment = 'push', duration = 3, movement = 1, sideMovement = 0, yawRate = 0, jump = false,
                impactAt = 0.1, impactBody = 'spine2', impactImpulse = impulse },
            segment('stop', strength > 3000 and 8 or 3),
        } }
        -- Hit while running (user bookmarks 2026-09-30: "after getting hit
        -- while running the legs stay straight").
        if strength == 2000 or strength == 3000 then
            protocols[#protocols + 1] = { name = 'push run ' .. d[1] .. ' ' .. strength, allowFall = true,
                segments = {
                segment('idle', 1),
                { segment = 'forward', duration = 2, movement = 1, sideMovement = 0, yawRate = 0, jump = false, run = true },
                { segment = 'push', duration = 3, movement = 1, sideMovement = 0, yawRate = 0, jump = false, run = true,
                    impactAt = 0.1, impactBody = 'spine2', impactImpulse = impulse },
                segment('stop', 3),
            } }
        end
    end
end
-- STOPPUSH-01 (alpha 0.6, PGC session 2026-09-30): pushes 0.3 and 0.8 s
-- after stopping a walk - offline PGC's weak spot (35/120 falls, all from the
-- catch releasing on an "uncatchable" verdict).
for _, strength in ipairs({ 2000, 3000 }) do
    for _, d in ipairs(dirs) do
        for _, delay in ipairs({ 0.3, 0.8 }) do
            protocols[#protocols + 1] = { name = 'STOPPUSH-01 ' .. d[1] .. ' ' .. strength .. ' at ' .. delay,
                allowFall = true, segments = {
                segment('idle', 1), segment('forward', 2, 1),
                { segment = 'push', duration = 3, movement = 0, sideMovement = 0, yawRate = 0, jump = false,
                    impactAt = delay, impactBody = 'spine2', impactImpulse = { d[2] * strength, d[3] * strength } },
                segment('settle', 3),
            } }
        end
    end
end
-- NEARPUSH-01 (2026-10-05, scoped recovery port): the pushes the legs decide.
-- A hit leaves the balance helper 1 - impulse / 5000 of its strength, so 5000
-- falls in 0.1-0.5 s whatever the legs do and 4000 always stands; between,
-- the steps decide. Chest (spine2) hits from 8 directions (0 = from the
-- front, 90 = from the left side), standing and walking.
for _, strength in ipairs({ 4400, 4700, 8000, 12000, 16000 }) do
    for angle = 0, 315, 45 do
        local a = math.rad(angle)
        local impulse = { math.sin(a) * strength, -math.cos(a) * strength }
        for _, moving in ipairs({ false, true }) do
            protocols[#protocols + 1] = { name = 'near push ' .. (moving and 'walk ' or 'stand ') .. angle .. ' ' .. strength,
                allowFall = true, segments = {
                segment('idle', moving and 1 or 2), moving and segment('forward', 2, 1) or segment('idle', 0.1),
                { segment = 'push', duration = 3, movement = moving and 1 or 0, sideMovement = 0, yawRate = 0, jump = false,
                    impactAt = 0.1, impactBody = 'spine2', impactImpulse = impulse },
                segment(moving and 'stop' or 'settle', 4),
            } }
        end
    end
end
-- SAVE-01 (PGC robustness, 2026-10-09): pushes in the movements no catch was
-- built or measured for - walking backward, strafing, running backward,
-- turning on the spot, two hits in a row, a hit on landing, a hit right after
-- a running stop, uphill and downhill. Impulses are in the player's frame
-- (x right, y forward): 'front' pushes back, 'back' pushes forward.
local function move(name, duration, movement, side, run, yaw, jump)
    return { segment = name, duration = duration, movement = movement or 0, sideMovement = side or 0,
        yawRate = yaw or 0, jump = jump or false, run = run or false }
end
local function hit(value, at, impulse)
    value.impactAt, value.impactBody, value.impactImpulse = at, 'spine2', impulse
    return value
end
local saves = {
    { 'backpedal front 3000', { move('idle', 1), move('back', 2, -1),
        hit(move('push', 3, -1), 0.1, { 0, -3000 }), move('stop', 3) } },
    { 'backpedal back 3000', { move('idle', 1), move('back', 2, -1),
        hit(move('push', 3, -1), 0.1, { 0, 3000 }), move('stop', 3) } },
    { 'strafe with 3000', { move('idle', 1), move('strafe', 2, 0, 1),
        hit(move('push', 3, 0, 1), 0.1, { 3000, 0 }), move('stop', 3) } },
    { 'strafe against 3000', { move('idle', 1), move('strafe', 2, 0, 1),
        hit(move('push', 3, 0, 1), 0.1, { -3000, 0 }), move('stop', 3) } },
    { 'run back front 3000', { move('idle', 1), move('run back', 2, -1, 0, true),
        hit(move('push', 3, -1, 0, true), 0.1, { 0, -3000 }), move('stop', 3) } },
    { 'run side 4000', { move('idle', 1), move('run', 2, 1, 0, true),
        hit(move('push', 3, 1, 0, true), 0.1, { 4000, 0 }), move('stop', 3) } },
    { 'turning side 3000', { move('idle', 2),
        hit(move('push', 3, 0, 0, false, math.pi), 0.3, { 3000, 0 }), move('settle', 3) } },
    { 'double walk front 2500', { move('idle', 1), move('walk', 2, 1),
        hit(move('push', 0.4, 1), 0.1, { 0, -2500 }), hit(move('push again', 3, 1), 0.05, { 0, -2500 }),
        move('stop', 3) } },
    { 'double stand side front 2500', { move('idle', 2),
        hit(move('push', 0.5), 0.1, { 2500, 0 }), hit(move('push again', 3), 0.05, { 0, -2500 }),
        move('settle', 3) } },
    { 'landing front 2500', { move('idle', 2), move('forward', 0.8, 1), move('jump', 0.2, 1, 0, false, 0, true),
        hit(move('push', 3, 1), 0.45, { 0, -2500 }), move('stop', 3) } },
    { 'run stop back 3000', { move('idle', 1), move('run', 3, 1, 0, true),
        hit(move('push', 3), 0.3, { 0, 3000 }), move('settle', 3) } },
}
for _, s in ipairs(saves) do
    protocols[#protocols + 1] = { name = 'SAVE-01 push ' .. s[1], allowFall = true, segments = s[2] }
end
-- Tired (fatigue at 15% of base) and loaded (180 weight of iron cuirasses
-- carried, near a fresh character's capacity) versions of two pushes.
for _, state in ipairs({ { 'tired', { fatigueShare = 0.15 } }, { 'loaded', { carry = { 'iron_cuirass', 6 } } } }) do
    for _, s in ipairs({
        { 'stand front 3000', { move('idle', 2), hit(move('push', 3), 0.1, { 0, -3000 }), move('settle', 3) } },
        { 'walk side 3000', { move('idle', 1), move('walk', 2, 1), hit(move('push', 3, 1), 0.1, { 3000, 0 }),
            move('stop', 3) } },
    }) do
        local p = { name = 'SAVE-01 push ' .. s[1] .. ' ' .. state[1], allowFall = true, segments = s[2] }
        for k, v in pairs(state[2]) do p[k] = v end
        protocols[#protocols + 1] = p
    end
end
for _, slope in ipairs({ { 'uphill', 1.026, util.vector3(-17258, -11291, 600), { 0, -3000 } },
    { 'downhill', 1.026 - math.pi, util.vector3(-15743, -10374, 1400), { 0, 3000 } } }) do
    protocols[#protocols + 1] = { name = 'SAVE-01 push walk ' .. slope[1] .. ' 3000', initialYaw = slope[2],
        exterior = slope[3], allowFall = true, segments = {
        move('idle', 2), move('walk', 2.5, 1), hit(move('push', 3, 1), 0.1, slope[4]), move('stop', 3),
    } }
end
-- DROP-01 (user bookmark 16, 2026-10-06: "stuck leaning after jumping off
-- stairs"): the run that did it, east down Balmora's steep bank (380 units
-- down over 1000 forward in under 3 s). It landed hard, the root was
-- released, and the body stayed Stumbling for 12 s, leaning on one foot.
protocols[#protocols + 1] = { name = 'DROP-01 balmora bank run off', exterior = util.vector3(-23700, -12595, 922),
    initialYaw = math.pi / 2, allowFall = true, segments = {
    segment('idle', 1),
    { segment = 'run off', duration = 5, movement = 1, sideMovement = 0, yawRate = 0, jump = false, run = true },
    segment('stop', 8),
} }
-- Knockdown and get-up (play test 2026-09-24 R5: "the stand up animation
-- is crazy" - the torso was thrown far above standing height).
for _, strength in ipairs({ 6000, 10000 }) do
    protocols[#protocols + 1] = { name = 'knockdown ' .. strength, allowFall = true, segments = {
        segment('idle', 2),
        { segment = 'push', duration = 1, movement = 0, sideMovement = 0, yawRate = 0, jump = false,
            impactAt = 0.1, impactBody = 'spine2', impactImpulse = { 0, -strength } },
        segment('recover', 12),
    } }
end
-- KNOCKRUN-01 (user bookmarks 2026-09-30 "stuck in a pose", "legs just don't
-- work"): knocked down, get up, then run at once. In play the body stayed in
-- Bracing for 5-11 s with both feet pinned (or both flagged swinging) while
-- running. Run forward, then back-diagonal like the recording.
for _, wait in ipairs({ 3.5, 4.5, 5.5 }) do
    protocols[#protocols + 1] = { name = 'KNOCKRUN-01 early run ' .. wait, allowFall = true, segments = {
        segment('idle', 2),
        { segment = 'push', duration = 1, movement = 0, sideMovement = 0, yawRate = 0, jump = false,
            impactAt = 0.1, impactBody = 'spine2', impactImpulse = { 0, -6000 } },
        segment('recover', wait),
        { segment = 'run', duration = 6, movement = -1, sideMovement = 1, yawRate = 0, jump = false, run = true },
        segment('stop', 2),
    } }
end
protocols[#protocols + 1] = { name = 'KNOCKRUN-01 get up and run', allowFall = true, segments = {
    segment('idle', 2),
    { segment = 'push', duration = 1, movement = 0, sideMovement = 0, yawRate = 0, jump = false,
        impactAt = 0.1, impactBody = 'spine2', impactImpulse = { 0, -6000 } },
    segment('recover', 7),
    { segment = 'run', duration = 6, movement = 1, sideMovement = 0, yawRate = 0, jump = false, run = true },
    { segment = 'run back', duration = 4, movement = -1, sideMovement = 1, yawRate = 0, jump = false, run = true },
    segment('stop', 2),
} }
-- Knockdown outdoors in Balmora (play test 2026-09-28: "I can't get up",
-- "enemy guard can't get up" - both in town, every other knockdown case is on
-- the flat test floor): on the street where the player's get-up stalled, and
-- across the hillside the terrain course climbs.
for _, place in ipairs({
    { 'street', util.vector3(-22386, -14865, 520), 2.1 },
    { 'hill', util.vector3(-17000, -11100, 700), 1.026 - math.pi / 2 },
}) do
    protocols[#protocols + 1] = { name = 'knockdown balmora ' .. place[1], exterior = place[2],
        initialYaw = place[3], allowFall = true, segments = {
        segment('idle', 2),
        { segment = 'push', duration = 1, movement = 0, sideMovement = 0, yawRate = 0, jump = false,
            impactAt = 0.1, impactBody = 'spine2', impactImpulse = { 0, -6000 } },
        segment('recover', 12),
    } }
end
-- ICARIAN-01 (user 11-01: the Scroll of Icarian Flight, Fortify Acrobatics
-- 1000 for 7 s, "I jump and only move a little forward"): a running jump on
-- the Balmora hillside at Acrobatics 1000 (vanilla: ~2700 u/s up and as much
-- forward) and 300. The capsule's forward travel and the body's distance from
-- it through the flight; RAGDOLL_JUMP_LAUNCH logs the launch.
for _, acro in ipairs({ 1000, 300 }) do
    protocols[#protocols + 1] = { name = 'ICARIAN-01 run jump acro ' .. acro, acrobatics = acro,
        exterior = util.vector3(-17000, -11100, 700), initialYaw = 1.026 - math.pi / 2, allowFall = true, segments = {
        segment('idle', 2), run('run', 1.0, 1),
        { segment = 'jump', duration = 0.2, movement = 1, sideMovement = 0, yawRate = 0, jump = true, run = true },
        -- Acrobatics 1000 flies ~14 s (peak ~20800 units up): the stop
        -- covers the landing.
        run('flight', 10, 1), segment('stop', acro >= 1000 and 12 or 4),
    } }
end
-- Knockdown on the Balmora stairs the play test fought down (2026-09-29:
-- "the enemy fell in the stairs and clipped into the ground"): facing down
-- the stairs and knocked back onto them, and facing up and knocked down them.
-- Stills while down; OPENMW_RAGDOLL_SINK_TRACE logs bodies under the floor.
for _, place in ipairs({ { 'stairs back', math.pi / 2 }, { 'stairs down', -math.pi / 2 } }) do
    protocols[#protocols + 1] = { name = 'knockdown balmora ' .. place[1],
        exterior = util.vector3(-23760, -10550, 960), initialYaw = place[2], allowFall = true, segments = {
        segment('idle', 2),
        { segment = 'push', duration = 1, movement = 0, sideMovement = 0, yawRate = 0, jump = false,
            impactAt = 0.1, impactBody = 'spine2', impactImpulse = { 0, -6000 }, screenshots = { 0.6 } },
        { segment = 'recover', duration = 12, movement = 0, sideMovement = 0, yawRate = 0, jump = false,
            screenshots = { 0.5, 1.5, 3 } },
    } }
end
-- The same stairs, an opponent standing a step down the stairs knocked
-- down them by a blow from the player's side (the bookmark's enemy was an
-- NPC). OPENMW_RAGDOLL_SINK_TRACE shows its bodies against the floor.
protocols[#protocols + 1] = { name = 'balmora stairs npc falls', exterior = util.vector3(-23760, -10550, 960),
    initialYaw = math.pi / 2, opponentAhead = 70, allowFall = true, segments = {
    segment('idle', 2),
    { segment = 'push', duration = 1, movement = 0, sideMovement = 0, yawRate = 0, jump = false,
        impactAt = 0.1, impactBody = 'spine2', impactImpulse = { 0, 8000 }, impactTarget = 'opponent',
        screenshots = { 0.6 } },
    { segment = 'recover', duration = 12, movement = 0, sideMovement = 0, yawRate = 0, jump = false,
        screenshots = { 0.5, 1.5, 3, 6 } },
} }
-- Morrowind's stagger on a powered body (user 2026-09-29: "get rid of the
-- vanilla attack reaction animations"): its knockdown and flinch clips must
-- not play through the physical body; the clip's pelvis stays up.
for _, kind in ipairs({ 'knockdown', 'flinch' }) do
    protocols[#protocols + 1] = { name = 'vanilla ' .. kind, segments = {
        segment('idle', 2),
        { segment = 'stagger', duration = 5, movement = 0, sideMovement = 0, yawRate = 0, jump = false,
            staggerAt = 0.1, stagger = kind },
    } }
end
-- Powered while lying down (play test 2026-09-29: "I cant get up" - the
-- manual hub re-powered a dead player while its animation still lay in
-- Morrowind's knockdown clip; the body then read upright as tipped and every
-- get-up after that ended in a fall, for 7 minutes). The runner powers the
-- body 2.5 s into a knockout clip and then gives fatigue back, so the clip
-- gets up: the body must end up standing and stay up.
protocols[#protocols + 1] = { name = 'powered lying', poweredLying = true, allowFall = true, segments = {
    segment('idle', 14),
} }
-- Standing off the anchor (play test 2026-09-28: "stuck in smooth criminal
-- lean" - after a fall the body stood 25 units from the actor, leaning 29
-- degrees, for seconds): the pelvis is dragged 30 units aside and let go; the
-- body must stand upright again, stepping or with the anchor yielding.
for _, dir in ipairs({ { 'side', 30, 0 }, { 'front', 0, 30 } }) do
    protocols[#protocols + 1] = { name = 'stand displaced ' .. dir[1], segments = {
        segment('idle', 2),
        { segment = 'drag', duration = 2.5, movement = 0, sideMovement = 0, yawRate = 0, jump = false,
            grab = { dir[2], dir[3] } },
        segment('let go', 6),
    } }
end
-- Spinning while moving (user report 2026-09-23: "pendulum swings in circles").
local spinShots = {}
for i = 0, 11 do spinShots[#spinShots + 1] = 0.1 + i * 0.12 end
protocols[#protocols + 1] = { name = 'spin run', segments = {
    segment('idle', 1), segment('forward', 1.5, 1),
    { segment = 'spin', duration = 2, movement = 1, sideMovement = 0, yawRate = quickQuarter / 2, jump = false,
        screenshots = spinShots },
    segment('stop', 2),
} }
-- Walk into a powered opponent (user report: bump -> opponent oscillates).
protocols[#protocols + 1] = { name = 'bump opponent', opponentAhead = 90, segments = {
    segment('idle', 1), segment('walk into', 3, 1), segment('push against', 2, 1), segment('stop', 3),
} }
protocols[#protocols + 1] = { name = 'bump opponent strafe', opponentAhead = 45, segments = {
    segment('idle', 1), segment('brush past', 3, 0, 1), segment('back', 3, 0, -1), segment('stop', 3),
} }
-- Run into the opponent, keep pushing, then let go (user session 2026-09-23:
-- capsule kept moving at 250 u/s with no input after contact).
local function run(name, duration, movement, side, yaw)
    local s = segment(name, duration, movement, side, yaw)
    s.run = true
    return s
end
protocols[#protocols + 1] = { name = 'bump opponent run', opponentAhead = 200, segments = {
    segment('idle', 1), run('run into', 2.5, 1), run('push against', 2, 1), segment('let go', 4),
} }
protocols[#protocols + 1] = { name = 'bump opponent run circle', opponentAhead = 120, segments = {
    segment('idle', 1), run('run into', 1.5, 1), run('circle', 3, 1, 0, math.pi / 2), segment('let go', 4),
} }
protocols[#protocols + 1] = { name = 'run straight', segments = {
    segment('idle', 1), run('run', 4, 1), segment('stop', 3),
} }
protocols[#protocols + 1] = { name = 'run turn', segments = {
    segment('idle', 1), run('run', 1.5, 1), run('turn', 2, 1, 0, math.pi / 2), run('run', 1, 1), segment('stop', 3),
} }
protocols[#protocols + 1] = { name = 'run strafe', segments = {
    segment('idle', 1), run('strafe', 3, 0, 1), run('diagonal', 2, 1, -1), segment('stop', 3),
} }
-- Flat running strafes both ways (STEPPING audit 11-01: the gait table's
-- run to the right led 0.048 leg lengths against 0.28 to the left; the
-- user: "hold run and hold right after rotating a bit ... both legs stay
-- to the left and the whole body to the right ... floating on a lean").
protocols[#protocols + 1] = { name = 'run strafe both', segments = {
    segment('idle', 1), run('strafe right', 3, 0, 1), run('strafe left', 3, 0, -1), segment('stop', 3),
} }
-- BUILD_FRAMES (user 11-01 bookmarks 5-11, "my right foot is always
-- forward" after running out of a door): the body is rebuilt 0.9 s into a
-- run, mid-stride, as a door's rebuild does (segment.repowerAt = seconds
-- into the segment), then stops and stands.
do
    local mid = run('run', 2.5, 1)
    mid.repowerAt = 0.9
    protocols[#protocols + 1] = { name = 'rebuild mid-run', segments = {
        segment('idle', 1), run('run in', 1, 1), mid, segment('stand', 5),
    } }
    -- the same without the rebuild (the baseline)
    protocols[#protocols + 1] = { name = 'run then stand', segments = {
        segment('idle', 1), run('run in', 1, 1), run('run', 2.5, 1), segment('stand', 5),
    } }
end
-- TURN_LEASH (user 11-01 bookmarks 14-15: a running 180 left the body
-- 56-71 u behind the capsule, then it snapped back): two running half
-- turns in 0.5 s each, both ways.
protocols[#protocols + 1] = { name = 'run turn 180', segments = {
    segment('idle', 1), run('run', 2, 1), run('turn left', 0.5, 1, 0, 6.28), run('run back', 2, 1),
    run('turn right', 0.5, 1, 0, -6.28), run('run again', 2, 1), segment('stop', 3),
} }
-- Drawing the weapon while running (user 11-02 bookmark: "if I weild the
-- weapon while running the running animation gets stuck"); sheathed mode.
do
    local drawing = run('run drawing', 3, 1)
    drawing.stance = 'weapon'
    protocols[#protocols + 1] = { name = 'run draw', segments = {
        segment('idle', 1), run('run', 2, 1), drawing, run('run drawn', 2, 1), segment('stop', 3),
    } }
end
protocols[#protocols + 1] = { name = 'run strafe turning', segments = {
    segment('idle', 1), run('turn first', 0.8, 1, 0, 0.8), run('strafe right', 3, 0, 1),
    run('strafe right turning', 2.5, 0, 1, 0.6), segment('stop', 3),
} }
-- Running with the direction changing every 0.7 s, backward diagonals
-- included - the inputs around the play test's "legs are stuck" bookmark
-- (2026-09-28: torso tipped 52-64 degrees, the helper let go, the body
-- stood released).
protocols[#protocols + 1] = { name = 'run zigzag', segments = {
    segment('idle', 1), run('zig', 0.7, 1, -1), run('zag', 0.7, 1, 1), run('back right', 0.7, -1, 1),
    run('back left', 0.7, -1, -1), run('zig again', 0.7, 1, -1), run('right', 0.7, 0, 1),
    run('zag again', 0.7, 1, 1), segment('stop', 3),
} }
-- Running diagonally, both ways (play test 2026-09-28: "legs are stuck",
-- fell while running diagonally).
protocols[#protocols + 1] = { name = 'run diagonal', segments = {
    segment('idle', 1), run('diagonal left', 3, 1, -1), run('diagonal right', 3, 1, 1), segment('stop', 3),
} }
-- Turning hard while running (user report: falls by itself).
for _, rate in ipairs({ 2, 3, 4 }) do
    protocols[#protocols + 1] = { name = 'run fast turn ' .. rate, segments = {
        segment('idle', 1), run('run', 1.5, 1), run('turn', 1, 1, 0, math.pi * rate),
        run('turn back', 1, 1, 0, -math.pi * rate), run('run', 1, 1), segment('stop', 2),
    } }
end
-- Visual review (LOOK): a side camera tracks the player; one still every
-- 1/15 s for 1.6 s of steady travel, then a stop. Screenshots land in the
-- shard's userdata/screenshots; tools/body/look_sheet.py tiles them.
local function look(name, movement, side, running, cameraYaw)
    local shots = {}
    for i = 0, 23 do shots[#shots + 1] = 0.05 + i / 15 end
    local function seg(label, duration, m, s, extra)
        local value = segment(label, duration, m, s)
        value.run = running
        value.sideCamera = cameraYaw or math.pi / 2
        for k, v in pairs(extra or {}) do value[k] = v end
        return value
    end
    -- Four views of the steady travel: profile (24 stills), then front,
    -- back and overhead (5 each). The segments share the name 'steady'.
    local few = { 0.1, 0.3, 0.5, 0.7, 0.9 }
    local profile = cameraYaw or math.pi / 2
    protocols[#protocols + 1] = { name = 'look ' .. name, segments = {
        seg('idle', 1), seg('travel', 2.5, movement, side),
        seg('steady', 1.7, movement, side, { screenshots = shots }),
        seg('steady', 1.0, movement, side, { screenshots = few, sideCamera = math.pi, view = 'front' }),
        seg('steady', 1.0, movement, side, { screenshots = few, sideCamera = 0, view = 'back' }),
        seg('steady', 1.0, movement, side, { screenshots = few, sideCamera = profile, cameraDistance = 30,
            cameraHeight = 230, cameraPitch = 1.40, view = 'top' }),
        seg('stop', 1.5, 0, 0, { screenshots = { 0.1, 0.3, 0.5, 0.8, 1.2 } }),
    } }
end
look('walk', 1, 0, false)
look('run', 1, 0, true)
look('strafe', 0, 1, false, 0)
look('backpedal', -1, 0, false)
look('run strafe', 0, 1, true, 0)
look('stand', 0, 0, false)
-- CROUCH-01 look: profile stills going down, crouched, sneaking forward,
-- front and back views, stopping and standing up (vanilla: the clip).
do
    local shots = {}
    for i = 0, 11 do shots[#shots + 1] = 0.05 + i / 7.5 end
    local few = { 0.1, 0.4, 0.7 }
    local function view(label, duration, m, extra, standing)
        local value = standing and segment(label, duration, m) or sneak(label, duration, m)
        value.sideCamera = math.pi / 2
        for k, v in pairs(extra or {}) do value[k] = v end
        return value
    end
    protocols[#protocols + 1] = { name = 'look crouch', segments = {
        view('idle', 1, 0, nil, true),
        view('crouch', 2, 0, { screenshots = { 0.1, 0.25, 0.5, 1.0, 1.8 } }),
        view('crouch', 1, 0, { screenshots = few, sideCamera = math.pi, view = 'front' }),
        view('travel', 2, 1),
        view('steady', 1.6, 1, { screenshots = shots }),
        view('steady', 0.9, 1, { screenshots = few, sideCamera = math.pi, view = 'front' }),
        view('steady', 0.9, 1, { screenshots = few, sideCamera = 0, view = 'back' }),
        view('stop', 2, 0, { screenshots = { 0.2, 0.5, 1.0, 1.8 } }),
        view('stand', 2, 0, { screenshots = { 0.1, 0.3, 0.6, 1.2, 1.9 } }, true),
    } }
end
-- Turning and moving around: the feet must follow a changing heading
-- ("walking and running turning is kinda bad", "moving around kinda breaks
-- the animation", 2026-09-26).
local function lookSequence(name, running, parts, leadIn)
    local shots = {}
    for i = 0, 23 do shots[#shots + 1] = 0.05 + i / 15 end
    local segments = { segment('idle', 1) }
    segments[1].sideCamera = math.pi / 2
    if leadIn then
        local lead = segment('travel', leadIn, 1, 0)
        lead.run = running
        lead.sideCamera = math.pi / 2
        segments[#segments + 1] = lead
    end
    for index, part in ipairs(parts) do
        local value = segment(index == 1 and 'steady' or 'travel', part[1], part[2], part[3])
        value.run = running
        value.yawRate = part[4]
        value.sideCamera = math.pi / 2
        if index == 1 then value.screenshots = shots end
        segments[#segments + 1] = value
    end
    local stop = segment('stop', 1.5, 0, 0)
    stop.sideCamera = math.pi / 2
    segments[#segments + 1] = stop
    protocols[#protocols + 1] = { name = 'look ' .. name, segments = segments }
end
-- Knocked down and getting up (look): the fall and the whole recovery in
-- profile stills, then the standing result from the front and above.
do
    local profile, front, top = {}, {}, {}
    for i = 0, 31 do profile[#profile + 1] = 0.1 + i * 0.25 end
    local function view(name, duration, extra)
        local value = segment(name, duration, 0, 0)
        value.sideCamera = math.pi / 2
        for k, v in pairs(extra or {}) do value[k] = v end
        return value
    end
    protocols[#protocols + 1] = { name = 'look knockdown', allowFall = true, segments = {
        view('idle', 1.5),
        view('push', 0.6, { impactAt = 0.1, impactBody = 'spine2', impactImpulse = { 0, -8000 } }),
        view('recover', 8.1, { screenshots = profile }),
        view('recover', 1.0, { screenshots = { 0.2, 0.6 }, sideCamera = math.pi, view = 'front' }),
        view('recover', 1.0, { screenshots = { 0.2, 0.6 }, cameraDistance = 30, cameraHeight = 230,
            cameraPitch = 1.40, view = 'top' }),
    } }
end
-- A one-click attack from the guard (look): the swing and how it ends, in
-- profile stills, then a second click from the front and from above ("ends
-- of swings look natural", 2026-09-26).
-- The click aims where the camera looks, so the camera is behind the player
-- (as in play) for the click itself.
do
    local shots, few = {}, { 0.1, 0.3, 0.5, 0.7, 0.9, 1.2 }
    for i = 0, 23 do shots[#shots + 1] = 0.02 + i / 15 end
    local function view(name, duration, extra)
        local value = segment(name, duration, 0, 0)
        value.sideCamera = math.pi / 2
        value.stance = 'weapon'
        for k, v in pairs(extra or {}) do value[k] = v end
        return value
    end
    protocols[#protocols + 1] = { name = 'look attack', segments = {
        view('idle', 2.0),
        view('steady', 0.1, { click = true, sideCamera = 0 }),
        view('steady', 1.7, { screenshots = shots }),
        view('steady', 0.8),
        view('steady', 0.1, { click = true, sideCamera = 0 }),
        view('steady', 1.5, { screenshots = few, sideCamera = math.pi, view = 'front' }),
        view('steady', 0.8, { sideCamera = math.pi }),
        view('steady', 0.1, { click = true, sideCamera = 0 }),
        view('steady', 1.5, { screenshots = few, cameraDistance = 30, cameraHeight = 230, cameraPitch = 1.40,
            view = 'top' }),
    } }
end
-- The test course (2026-09-27): the same scripted inputs every run, scored
-- against the animation by tools/body/course_report.py. Segment names start
-- with "c " and say what the body should be doing.
do
    local function part(name, duration, movement, side, running, yaw)
        local value = segment('c ' .. name, duration, movement, side, yaw or 0)
        value.run = running or false
        return value
    end
    local function course()
        return {
            part('stand', 2.0, 0, 0),
            part('walk', 4.0, 1, 0),
            part('run', 4.0, 1, 0, true),
            part('walk again', 3.0, 1, 0),
            part('strafe left', 3.0, 0, -1),
            part('strafe right', 3.0, 0, 1),
            part('backpedal', 3.0, -1, 0),
            part('walk to run', 1.5, 1, 0),
            part('run again', 3.0, 1, 0, true),
            part('run strafe left', 2.5, 0, -1, true),
            part('run strafe right', 2.5, 0, 1, true),
            part('run turn', 3.0, 1, 0, true, 1.2),
            part('run to walk', 3.0, 1, 0),
            part('stop', 2.0, 0, 0),
        }
    end
    protocols[#protocols + 1] = { name = 'course', segments = course() }
    -- Running (milestone 2): long straight runs, so a score has ~50 steps
    -- behind it (a 4 s segment's ~6 steps moved it by +-5 between identical
    -- runs). The floor ends ~4200 units ahead: at 12 s + 8 s the runner
    -- went off it during 'run again' and fell into the void.
    protocols[#protocols + 1] = { name = 'course run', segments = {
        part('stand', 2.0, 0, 0),
        part('walk', 2.0, 1, 0),
        part('run', 9.0, 1, 0, true),
        part('walk again', 3.0, 1, 0),
        part('run again', 5.0, 1, 0, true),
        part('stop', 2.0, 0, 0),
    } }
    -- Terrain (2026-09-28): a steady Balmora hillside from the user's play
    -- route (ground ~160 -> ~950 over ~1800 units), walked and run up, and
    -- down from the top. The play test there showed the feet stepping on a
    -- flat plane (the foot rays missed the terrain layer).
    local function hill(direction, yaw, at)
        -- allowFall: the route crosses real ledges (drops of 60-370 units).
        protocols[#protocols + 1] = { name = 'course balmora ' .. direction, initialYaw = yaw,
            exterior = at, allowFall = true, segments = {
            part('stand', 2.0, 0, 0),
            part('walk', 5.0, 1, 0),
            part('run', 4.0, 1, 0, true),
            part('walk again', 2.0, 1, 0),
            part('stop', 2.0, 0, 0),
        } }
    end
    hill('up', 1.026, util.vector3(-17258, -11291, 600))
    hill('down', 1.026 - math.pi, util.vector3(-15743, -10374, 1400))
    -- Three laps without a reset: a long session, where a drift builds up
    -- (the user's play test sank further run after run, 2026-09-27).
    local long = {}
    for lap = 1, 3 do
        for _, value in ipairs(course()) do
            if lap > 1 or value.segment ~= 'c stand' then value.segment = value.segment .. ' ' .. lap end
            -- Standing between laps, a moderate shove (a stagger, not a fall).
            if lap > 1 and value.segment == 'c stand ' .. lap then
                value.impactAt, value.impactBody, value.impactImpulse = 0.5, 'spine2', { 0, -2500 }
                value.duration = 3.0
            end
            long[#long + 1] = value
        end
    end
    protocols[#protocols + 1] = { name = 'course long', segments = long }
end
-- PGC-03 behaviour cases (2026-10-11 night; fixtures from
-- tools/make-ragdoll-test-features.py, -X half of the test floor). Unarmed
-- only. Facing: yaw 0 = +Y, pi/2 = +X, -pi/2 = -X. tools/body/behaviour_report.py
-- reports steps, foot skid, upright slide, falls and time until steady from
-- the case's `anchor` segment (or the blow); OPENMW_RAGDOLL_TEST_FILM films them.
do
    local function seg(name, duration, movement, side, run, jump)
        return { segment = name, duration = duration, movement = movement or 0, sideMovement = side or 0,
            yawRate = 0, jump = jump or false, run = run or false }
    end
    local unarmed = { unarmed = true }
    local function add(p)
        p.modes, p.poweredOnly, p.allowFall = unarmed, true, true
        protocols[#protocols + 1] = p
    end
    -- STAGGER-01: blows through the reaction controller (REACT-02 strengths)
    -- on the chest, standing, walking and running toward the blow (an
    -- approaching fighter), from the front; heavy from the side too.
    local strengths = { light = { 300, 1.6, 5 }, solid = { 800, 1.6, 15 }, heavy = { 1600, 2.4, 35 } }
    local function blow(value, at, direction, strength)
        local st = strengths[strength]
        value.blowAt, value.blow = at, { 'spine2', direction, st[1], st[2], st[3] }
        return value
    end
    for _, motion in ipairs({ { 'stand', 0, false }, { 'walk', 1, false }, { 'approach', 1, true } }) do
        for _, strength in ipairs({ 'light', 'solid', 'heavy' }) do
            add({ name = 'STAGGER-01 ' .. motion[1] .. ' front ' .. strength, anchor = 'hit', segments = {
                seg('idle', 1.5), seg('move', 1.5, motion[2], 0, motion[3]),
                blow(seg('hit', 4, motion[2], 0, motion[3]), 0.05, { 0, -1 }, strength), seg('settle', 2),
            } })
        end
    end
    for _, motion in ipairs({ { 'stand', 0 }, { 'walk', 1 } }) do
        add({ name = 'STAGGER-01 ' .. motion[1] .. ' side heavy', anchor = 'hit', segments = {
            seg('idle', 1.5), seg('move', 1.5, motion[2]),
            blow(seg('hit', 4, motion[2]), 0.05, { 1, 0 }, 'heavy'), seg('settle', 2),
        } })
    end
    -- LEDGE-01: walk and run off 40/80/120-unit platforms (centred x = -3400 /
    -- -2700 / -2000, 600 long along Y): forward from the centre leaves the
    -- +Y edge after 300 units.
    for _, ledge in ipairs({ { 40, -3400 }, { 80, -2700 }, { 120, -2000 } }) do
        for _, gait in ipairs({ { 'walk', false, 3.5 }, { 'run', true, 1.6 } }) do
            add({ name = 'LEDGE-01 ' .. gait[1] .. ' off ' .. ledge[1], anchor = 'off',
                start = util.vector3(ledge[2], 0, ledge[1] + 64), initialYaw = 0, segments = {
                seg('idle', 1.5), seg('off', gait[3], 1, 0, gait[2]), seg('on', 1.0, 1, 0, gait[2]), seg('stop', 4),
            } })
        end
    end
    -- STAIRJUMP-01: the tall stairs (top at 192, 7 steps of 24 down along -X
    -- from x = -3200, y = -1800): walk down them, jump down them walking and
    -- running from the top's edge.
    local stairs = util.vector3(-3000, -1800, 256)
    add({ name = 'STAIRJUMP-01 walk down', anchor = 'down', start = stairs, initialYaw = -math.pi / 2, segments = {
        seg('idle', 1.5), seg('down', 4.5, 1), seg('stop', 3) } })
    add({ name = 'STAIRJUMP-01 jump walking', anchor = 'jump', start = stairs, initialYaw = -math.pi / 2, segments = {
        seg('idle', 1.5), seg('approach', 1.4, 1), seg('jump', 0.2, 1, 0, false, true), seg('flight', 1.5, 1),
        seg('stop', 4) } })
    add({ name = 'STAIRJUMP-01 jump running', anchor = 'jump', start = stairs, initialYaw = -math.pi / 2, segments = {
        seg('idle', 1.5), seg('approach', 0.55, 1, 0, true), seg('jump', 0.2, 1, 0, true, true),
        seg('flight', 1.5, 1, 0, true), seg('stop', 4) } })
    -- The same stairs strafed down (facing +Y, stepping left along -X): the
    -- strafe on 24-unit risers, where a landing a tread below the capsule's
    -- ground window missed the probe (PGC-03 C9b).
    add({ name = 'STAIRJUMP-01 strafe down', anchor = 'down', start = util.vector3(-2950, -1800, 256), initialYaw = 0,
        segments = { seg('idle', 1.5), seg('down', 4.5, 0, -1), seg('stop', 3) } })
    -- TEETER-01 (D11): stop short of the 120-unit platform's +Y edge (300
    -- from the centre) with momentum toward it, and a light push toward it
    -- from 40 units away. The body should stay on top (report: over).
    add({ name = 'TEETER-01 walk stop short', anchor = 'stop', start = util.vector3(-2000, 0, 184), initialYaw = 0,
        segments = { seg('idle', 1.0), seg('walk', 2.3, 1), seg('stop', 4) } })
    add({ name = 'TEETER-01 run stop short', anchor = 'stop', start = util.vector3(-2000, -40, 184), initialYaw = 0,
        segments = { seg('idle', 1.0), seg('run', 0.95, 1, 0, true), seg('stop', 4) } })
    add({ name = 'TEETER-01 push near edge', anchor = 'push', start = util.vector3(-2000, 270, 184), initialYaw = 0,
        segments = { seg('idle', 1.5), blow(seg('push', 4), 0.05, { 0, 1 }, 'solid'), seg('settle', 2) } })
    -- WALL-01 (D13): a solid push toward a wall 40 units in front (the wall
    -- along Y at x = -1500, facing +X): a hand should catch it.
    add({ name = 'WALL-01 push toward wall', anchor = 'push', start = util.vector3(-1545, 0, 64),
        initialYaw = math.pi / 2, segments = { seg('idle', 1.5), blow(seg('push', 4), 0.05, { 0, 1 }, 'heavy'),
        seg('settle', 2) } })
    -- MANTLE-01 (E14): walk into the 40/80/120 platforms' -Y side (y = -300)
    -- from 80 units away and keep walking: 40 and 80 should be climbed onto
    -- (report: rise ~ the height), 120 is above chest height and stays below.
    for _, ledge in ipairs({ { 40, -3400 }, { 80, -2700 }, { 120, -2000 } }) do
        add({ name = 'MANTLE-01 walk into ' .. ledge[1], anchor = 'climb', start = util.vector3(ledge[2], -380, 64),
            initialYaw = 0, segments = { seg('idle', 1.0), seg('climb', 3.5, 1), seg('stop', 3) } })
    end
    -- WALL-01 ledge: a heavy push toward the 80-unit platform's side from 45
    -- units away - its top is at hip height; a hand should catch the top.
    add({ name = 'WALL-01 push toward ledge', anchor = 'push', start = util.vector3(-2700, -345, 64),
        initialYaw = 0, segments = { seg('idle', 1.5), blow(seg('push', 4), 0.05, { 0, 1 }, 'heavy'),
        seg('settle', 2) } })
    -- CRAWL-01 (E15): sneak toward the 40/80/120 platforms' +Y edge (100
    -- units ahead) and keep sneaking: 80 and 120 are crawled down (hands on
    -- the edge, lowered; report: rise ~ -height, no fall), 40 is under the
    -- crawl's 50 and is stepped down.
    for _, ledge in ipairs({ { 40, -3400 }, { 80, -2700 }, { 120, -2000 } }) do
        add({ name = 'CRAWL-01 sneak off ' .. ledge[1], anchor = 'crawl',
            start = util.vector3(ledge[2], 200, ledge[1] + 64), initialYaw = 0, segments = {
                seg('idle', 1.0),
                { segment = 'crawl', duration = 5, movement = 1, sideMovement = 0, yawRate = 0, jump = false, sneak = true },
                { segment = 'stop', duration = 3, movement = 0, sideMovement = 0, yawRate = 0, jump = false, sneak = true },
            } })
    end
    -- DROP-01 (C8b, the drop roll): run, walk and strafe off the 300/500/800
    -- towers (centred x = -3400 / -2700 / -2000, y = -2650) facing -Y, the
    -- edge 350 units ahead. 500 and 800 land past the hard-landing release
    -- (a roll with OPENMW_RAGDOLL_DROP_ROLL; 800 rolls onto the back at low
    -- Acrobatics); 300 stays under the roll's threshold, and the strafe is
    -- not travelling along the facing - both keep today's landing.
    -- behaviour_report.py: inv, rolled, headv, jolt, travel, roll.
    for _, tower in ipairs({ { 300, -3400 }, { 500, -2700 }, { 800, -2000 } }) do
        add({ name = 'DROP-01 run off ' .. tower[1], anchor = 'off',
            start = util.vector3(tower[2], -2500, tower[1] + 64), initialYaw = math.pi, segments = {
                seg('idle', 1.5), seg('off', 1.6, 1, 0, true), seg('on', 1.0, 1, 0, true), seg('stop', 6),
            } })
    end
    add({ name = 'DROP-01 walk off 500', anchor = 'off', start = util.vector3(-2700, -2500, 564),
        initialYaw = math.pi, segments = { seg('idle', 1.5), seg('off', 3.6, 1), seg('on', 1.0, 1), seg('stop', 6) } })
    -- Facing +X, strafing right (toward -Y) off the same edge.
    add({ name = 'DROP-01 strafe off 500', anchor = 'off', start = util.vector3(-2700, -2500, 564),
        initialYaw = math.pi / 2, segments = {
            seg('idle', 1.5), seg('off', 1.8, 0, 1, true), seg('on', 1.0, 0, 1, true), seg('stop', 6),
        } })
    -- ACRO-01 (the roll and the get-up by Acrobatics) at Acrobatics 10 and
    -- 90: the run off the 500 and 800 towers and the walk off 500
    -- (Morrowind's landing knocks Acrobatics 10 down from ~545 u, 90 never
    -- here; 10 s stop so a failed roll's get-up ends in the case); the run
    -- off the 120 ledge (the running roll of play: a ~390 u/s touchdown that
    -- keeps the run); the get-up from lying (the powered-lying setup: a 6000
    -- push is caught by the legs). behaviour_report.py: the drop and get-up
    -- tables. Named apart from 'DROP-01 run off N' (the drop stage).
    for _, acro in ipairs({ 10, 90 }) do
        for _, tower in ipairs({ { 500, -2700 }, { 800, -2000 } }) do
            add({ name = 'DROP-01 acro ' .. acro .. ' run off ' .. tower[1], acrobatics = acro, anchor = 'off',
                start = util.vector3(tower[2], -2500, tower[1] + 64), initialYaw = math.pi, segments = {
                    seg('idle', 1.5), seg('off', 1.6, 1, 0, true), seg('on', 1.0, 1, 0, true), seg('stop', 10),
                } })
        end
        add({ name = 'DROP-01 acro ' .. acro .. ' walk off 500', acrobatics = acro, anchor = 'off',
            start = util.vector3(-2700, -2500, 564), initialYaw = math.pi, segments = {
                seg('idle', 1.5), seg('off', 3.6, 1), seg('on', 1.0, 1), seg('stop', 10) } })
        add({ name = 'DROP-01 acro ' .. acro .. ' run off ledge 120', acrobatics = acro, anchor = 'off',
            start = util.vector3(-2000, 0, 184), initialYaw = 0, segments = {
                seg('idle', 1.5), seg('off', 1.6, 1, 0, true), seg('on', 1.5, 1, 0, true), seg('stop', 4),
            } })
        add({ name = 'ACRO-01 lying acro ' .. acro, acrobatics = acro, poweredLying = true, segments = {
            seg('stand', 12) } })
    end
    -- SLOPE-01: the 30/40/50-degree ramps (rising along +X from x = -3600 to
    -- a 200-unit plateau; rows y = 1200 / 2000 / 2800): stand on the middle of
    -- the ramp facing down, walk down it, and a push down the slope.
    for _, ramp in ipairs({ { 30, 1200 }, { 40, 2000 }, { 50, 2800 } }) do
        local run = 200 / math.tan(math.rad(ramp[1]))
        local mid = util.vector3(-3600 + run * 0.5, ramp[2], 100 + 64)
        local down = -math.pi / 2
        -- From the first frame: on the 50-degree ramp the capsule slides off
        -- at once (steeper than Morrowind's walkable 46).
        add({ name = 'SLOPE-01 stand ' .. ramp[1], anchor = 'stand', start = mid, initialYaw = down, segments = {
            seg('stand', 6.5) } })
        add({ name = 'SLOPE-01 walk down ' .. ramp[1], anchor = 'down', start = mid, initialYaw = down, segments = {
            seg('idle', 0.5), seg('down', 2.5, 1), seg('stop', 3) } })
        add({ name = 'SLOPE-01 push ' .. ramp[1], anchor = 'push', start = mid, initialYaw = down, segments = {
            seg('idle', 1.5), blow(seg('push', 4), 0.05, { 0, 1 }, 'solid'), seg('settle', 2) } })
        -- Up the ramp from the floor before it (user 10-29: "walking uphill
        -- gets buggy"); the walkable ramps only.
        if ramp[1] < 46 then
            local foot = util.vector3(-3600 - 120, ramp[2], 64)
            add({ name = 'SLOPE-01 walk up ' .. ramp[1], anchor = 'up', start = foot, initialYaw = math.pi / 2,
                segments = { seg('idle', 1), seg('up', 4.5, 1), seg('stop', 2) } })
            add({ name = 'SLOPE-01 run up ' .. ramp[1], anchor = 'up', start = foot, initialYaw = math.pi / 2,
                segments = { seg('idle', 1), seg('up', 2.5, 1, 0, true), seg('stop', 2) } })
        end
    end
end
lookSequence('turn', false, { { 4.5, 1, 0, 1.2 } }, 2.5)
lookSequence('run turn', true, { { 4.5, 1, 0, 1.5 } }, 2.5)
lookSequence('around', false, { { 1.6, 1, 0, 0 }, { 1.2, 0, 1, 0 }, { 1.2, -1, 0, 0 }, { 1.2, 0, -1, 0 }, { 1.2, 1, 1, 0 } })
-- IDLE-01 (user 10-31: "when standing without moving sometimes the models do
-- small steps to stabilize themselves nonstop", more on the player): standing
-- after a walk, standing while the mouse looks around, standing on terrain.
-- Every mode (allModes). hand_jitter.py counts the steps per segment.
do
    local function seg(name, duration, movement, side, yaw)
        return { segment = name, duration = duration, movement = movement or 0, sideMovement = side or 0,
            yawRate = yaw or 0, jump = false }
    end
    protocols[#protocols + 1] = { name = 'IDLE-01 after walk', allModes = true, segments = {
        seg('idle', 1), seg('walk', 2, 1), seg('stand', 10) } }
    protocols[#protocols + 1] = { name = 'IDLE-01 look around', allModes = true, segments = {
        seg('idle', 2), seg('look left', 1.5, 0, 0, 0.6), seg('look right', 3, 0, 0, -0.6),
        seg('look back', 1.5, 0, 0, 0.6), seg('stand', 6) } }
    protocols[#protocols + 1] = { name = 'IDLE-01 hillside', allModes = true, initialYaw = 1.026,
        exterior = util.vector3(-17258, -11291, 600), allowFall = true, segments = {
        seg('idle', 2), seg('stand', 10) } }
end
-- RAMP-01 (user 11-01: "at the beginning of the game the guard that comes
-- to you after jiub walks very weirdly and also cant get on the ramp to go to
-- the next floor, also I cant walk up the ramp either"): the Imperial Prison
-- Ship's ramp from the hold to the deck (about 40 degrees, y 1135-1395, z -104
-- to 110). 'player up' / 'player run up': the player walks / runs up it from
-- where the play session stood waiting. 'guard up': the cell's own guard,
-- powered, travels by Morrowind's AI from his start by Jiub to the deck past
-- the ramp's top, where the play session's guard turned (AI travel ignores
-- height: a deck spot above the hold floor, or the deck door, is "reached"
-- from below and a vanilla guard turns back too); the player stays in the hold. 'guard up vanilla': the same unpowered (run it with
-- OPENMW_RAGDOLL_POWER_NPCS=0, which otherwise powers him again). Chargen is switched off (its name menu
-- pauses the game).
do
    local ship = 'Imperial Prison Ship'
    protocols[#protocols + 1] = { name = 'RAMP-01 ship player up', interior = { ship, util.vector3(30, 1000, -104) },
        allowFall = true, segments = { move('idle', 1), move('up', 10, 1), move('stop', 3) } }
    -- Running (the play session's first try: braced at y 1215 and slid back
    -- 60 units while running; walking on, it went straight up).
    protocols[#protocols + 1] = { name = 'RAMP-01 ship player run up', interior = { ship, util.vector3(30, 900, -104) },
        allowFall = true, segments = { move('idle', 1), move('up', 6, 1, 0, true), move('stop', 3) } }
    protocols[#protocols + 1] = { name = 'RAMP-01 ship guard up', interior = { ship, util.vector3(61, -135, -104) },
        escort = { 'chargen boat guard 2', util.vector3(88, -66, -104), util.vector3(-110, 1360, 110) }, allowFall = true,
        segments = { move('wait', 60) } }
    -- An NPC made from the player's record (no armour, the player's race)
    -- on the same walk: tells a guard-only cause from an NPC-wide one.
    protocols[#protocols + 1] = { name = 'RAMP-01 ship walker up', interior = { ship, util.vector3(61, -135, -104) },
        escort = { 'player', util.vector3(88, -66, -104), util.vector3(-110, 1360, 110) }, allowFall = true,
        segments = { move('wait', 60) } }
    -- The same with the player (and camera) beside the ramp's foot, facing it.
    protocols[#protocols + 1] = { name = 'RAMP-01 ship walker up near', interior = { ship, util.vector3(-45, 1080, -104) },
        initialYaw = 0.5, escort = { 'player', util.vector3(88, -66, -104), util.vector3(-110, 1360, 110) },
        allowFall = true, segments = { move('wait', 60) } }
    protocols[#protocols + 1] = { name = 'RAMP-01 ship guard up vanilla', interior = { ship, util.vector3(61, -135, -104) },
        escort = { 'chargen boat guard 2', util.vector3(88, -66, -104), util.vector3(-110, 1360, 110), 'vanilla' }, allowFall = true,
        segments = { move('wait', 60) } }
end
return protocols

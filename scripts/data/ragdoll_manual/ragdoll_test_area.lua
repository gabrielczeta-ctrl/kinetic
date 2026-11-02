local util = require('openmw.util')

-- One spatial fixture for automated and manual tests. The content plugin owns
-- the static floor so cell reload and save/load use ordinary object lifecycle.
-- Relative XY spacing matches the former Balmora range; only the floor changes.
return {
    id = 'flat-floor-v1',
    cell = 'OpenMW Ragdoll Test Floor',
    floorRecord = 'openmw_ragdoll_test_floor',
    floorHeight = 0,
    player = util.vector3(0, 0, 128),
    powered = util.vector3(-301, 165, 128),
    passive = util.vector3(366, 9, 128),
    gait = util.vector3(486, 297, 128),
    -- Yaw zero faces -Y in the gameplay movement adapter. Keep D in front of
    -- the player and clear of the resting bar; the old +Y position put the
    -- target behind the camera and made forward movement increase the gap.
    defense = util.vector3(0, -80, 128),
}

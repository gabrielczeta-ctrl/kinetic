#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_PLAYDEFAULTS_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_PLAYDEFAULTS_HPP

#include <span>
#include <string_view>

namespace RagdollCore
{
    // The switch values the game plays with (KINETIC-2 step 2). Before, the
    // play values lived in the development launcher's `set` lines, which the
    // packager copied into the release's ragdoll.cfg, while most code
    // fallbacks kept features off: the code did not say what the game was.
    // The engine applies this table last (PhysicsSystem::loadRagdollSettingsFile),
    // so the environment, the options page and ragdoll.cfg all still win;
    // OPENMW_RAGDOLL_PLAY_DEFAULTS=0 skips it (the scripted tests, which keep
    // the code's own fallbacks unless they ask for play). Offline tools and
    // unit tests never apply it.
    //
    // Pure: no Jolt or engine types.
    struct PlayDefault
    {
        std::string_view mName;
        std::string_view mValue;
    };

    std::span<const PlayDefault> playDefaults();
}

#endif

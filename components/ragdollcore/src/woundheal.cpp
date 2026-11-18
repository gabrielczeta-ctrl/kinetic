#include <ragdollcore/woundheal.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace RagdollCore
{
    WoundHealSettings parseWoundHealSettings(const char* text)
    {
        WoundHealSettings settings;
        if (!text)
            return settings;
        float on = 1.0f;
        float hours = settings.mFullHours;
        const int read = std::sscanf(text, "%f,%f", &on, &hours);
        if (read >= 1 && std::isfinite(on))
            settings.mOn = on != 0.0f;
        if (read >= 2 && std::isfinite(hours) && hours >= 0.0f)
            settings.mFullHours = hours;
        return settings;
    }

    float woundHealShare(WoundHealState& state, float pool, float poolMax, double gameHours,
        const WoundHealSettings& settings)
    {
        if (!settings.mOn || !std::isfinite(pool) || !std::isfinite(poolMax) || !(poolMax > 0.0f)
            || !std::isfinite(gameHours))
        {
            state = {};
            return 0.0f;
        }
        const bool first = state.mLastPool < 0.0f;
        const float last = std::min(state.mLastPool, poolMax);
        const double hours = gameHours - state.mLastHours;
        state.mLastPool = std::max(pool, 0.0f);
        state.mLastHours = gameHours;
        if (first || !(pool > 0.0f))
            return 0.0f;
        float share = 0.0f;
        if (pool > last)
        {
            const float missing = poolMax - last;
            share = missing > 0.0f ? (pool - last) / missing : 1.0f;
        }
        else if (pool >= poolMax && settings.mFullHours > 0.0f && hours > 0.0)
            share = static_cast<float>(hours / settings.mFullHours);
        return std::clamp(share, 0.0f, 1.0f);
    }

    int woundMarksHealed(WoundHealState& state, int marks, float share)
    {
        if (marks <= 0)
        {
            state.mCarry = 0.0f;
            return 0;
        }
        if (!(share > 0.0f))
            return 0;
        if (share >= 1.0f)
        {
            state.mCarry = 0.0f;
            return marks;
        }
        state.mCarry += static_cast<float>(marks) * share;
        const int healed = std::min(marks, static_cast<int>(std::floor(state.mCarry + 1e-4f)));
        state.mCarry = std::max(0.0f, state.mCarry - static_cast<float>(healed));
        return healed;
    }
}

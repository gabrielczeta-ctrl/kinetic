#include <ragdollcore/calmhold.hpp>

#include <ragdollcore/tuning.hpp>

#include <algorithm>
#include <array>

namespace RagdollCore
{
    CalmHoldSettings calmHoldSettings()
    {
        CalmHoldSettings s;
        std::array<float, 6> v{ s.mStrength, s.mEaseDown, s.mQuiet, s.mEaseUp, s.mBump, s.mBumpFor };
        tuningList("OPENMW_RAGDOLL_K2_CALM", v);
        s.mStrength = std::clamp(v[0], 0.0f, 1.0f);
        s.mEaseDown = std::max(v[1], 0.0f);
        s.mQuiet = std::max(v[2], 0.0f);
        s.mEaseUp = std::max(v[3], 0.0f);
        s.mBump = std::max(v[4], 0.0f);
        s.mBumpFor = std::max(v[5], 0.0f);
        return s;
    }

    CalmHold updateCalmHold(const CalmHoldInput& input, const CalmHoldSettings& settings, CalmHoldState& state, float dt)
    {
        CalmHold out;
        if (!(settings.mStrength > 0.0f) || !input.mAllowed)
        {
            state = {};
            state.mStarted = true;
            return out;
        }
        if (!state.mStarted)
        {
            state.mStarted = true;
            if (!input.mBothered && input.mUpright)
            {
                state.mLevel = 1.0f;
                state.mQuietFor = settings.mQuiet;
            }
        }
        // A bump counts only on a body the hold has (the kinetic body is
        // off the animation by design) and only when it lasts.
        const bool off = settings.mBump > 0.0f && input.mOffTarget > settings.mBump;
        state.mOffFor = off && state.mLevel >= 1.0f ? state.mOffFor + dt : 0.0f;
        out.mBumped = !input.mBothered && off && state.mLevel >= 1.0f && state.mOffFor >= settings.mBumpFor;
        if (out.mBumped)
            state.mLetGo = true;
        const bool bothered = input.mBothered || state.mLetGo;
        if (bothered)
        {
            state.mQuietFor = 0.0f;
            state.mLevel = settings.mEaseDown > 0.0f ? std::max(0.0f, state.mLevel - dt / settings.mEaseDown) : 0.0f;
            if (!(state.mLevel > 0.0f))
                state.mLetGo = false;
        }
        else
        {
            state.mQuietFor = input.mUpright ? state.mQuietFor + dt : 0.0f;
            // Taken back only from near the animation: a body the hold
            // would have to drag there is bumped again at once.
            if (state.mQuietFor >= settings.mQuiet && (!off || state.mLevel > 0.0f))
                state.mLevel = settings.mEaseUp > 0.0f ? std::min(1.0f, state.mLevel + dt / settings.mEaseUp) : 1.0f;
        }
        out.mLevel = state.mLevel;
        state.mDraw = state.mLevel >= state.mDraw || !(settings.mEaseDown > 0.0f)
            ? state.mLevel
            : std::max(state.mLevel, state.mDraw - dt / settings.mEaseDown);
        out.mDraw = state.mDraw;
        // Let go, the legs step at once from where the feet are; taken back,
        // the root holds first and the legs follow the clip from half way.
        out.mLegsOnClip = !bothered && state.mLevel > 0.5f;
        return out;
    }

    void breakCalmHold(CalmHoldState& state)
    {
        state.mLevel = 0.0f;
        state.mQuietFor = 0.0f;
        state.mOffFor = 0.0f;
        state.mLetGo = false;
    }
}

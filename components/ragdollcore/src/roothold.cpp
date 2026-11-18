#include <ragdollcore/roothold.hpp>

#include <algorithm>

namespace RagdollCore
{
    namespace
    {
        bool is(const std::optional<ActiveControllerMode>& mode, ActiveControllerMode which)
        {
            return mode && *mode == which;
        }
    }

    RootRetake judgeRootRetake(const RootHoldInput& input, const RootHoldSettings& settings, RootHoldState& state, float dt)
    {
        using M = ActiveControllerMode;
        RootRetake r;
        // Driven modes keep the root kinematic: getting up and blending back
        // too (their clips carry the pelvis path; released, the get-up ended
        // lying). Falling and fallen stay released so a real fall is physical.
        r.mDriven = is(input.mMode, M::Standing) || is(input.mMode, M::Bracing) || is(input.mMode, M::Stumbling)
            || is(input.mMode, M::GettingUp) || is(input.mMode, M::BlendBack);
        // Re-taking the root after physics owned it needs a short settled
        // standing (the "levitating in ragdoll mode" lock after repeated hits).
        const bool standingSlow = is(input.mMode, M::Standing) && input.mKnown && input.mCenterOfMassSpeed < 30.0f;
        state.mReacquireTimer = standingSlow ? state.mReacquireTimer + dt : 0.0f;
        const bool bracedUpright = settings.mBraceReacquire > 0.0f && state.mPhysicsOwned
            && is(input.mMode, M::Bracing) && input.mKnown && input.mTorsoUpDot > 0.85f
            && input.mCenterOfMassSpeed < 60.0f;
        state.mBracedTime = bracedUpright ? state.mBracedTime + dt : 0.0f;
        const bool bracedReacquire = settings.mBraceReacquire > 0.0f && state.mBracedTime >= settings.mBraceReacquire;
        r.mReacquired = state.mReacquireTimer >= 0.25f || bracedReacquire
            || (settings.mBraceReacquire > 0.0f && state.mReheld);
        r.mBraceOrRehold = r.mReacquired && state.mPhysicsOwned && (bracedReacquire || state.mReheld)
            && !state.mLandingReleased && !state.mToppled;
        r.mByRehold = state.mReheld;
        state.mReheld = false;
        // Recovery re-takes the root at once: the get-up clip is the only
        // thing that raises the body, and it must own the root from its first
        // frame; it also ends a hard landing's release.
        r.mRecoveryOwns = is(input.mMode, M::GettingUp) || is(input.mMode, M::BlendBack);
        if (r.mRecoveryOwns && settings.mGetUpEndsLanding)
            state.mLandingReleased = false;
        return r;
    }

    ToppleEvents judgeTopple(const RootHoldInput& input, bool recoveryOwns, const RootHoldSettings& settings,
        RootHoldState& state, float dt)
    {
        using M = ActiveControllerMode;
        ToppleEvents events;
        const bool tipped = input.mRootKinematic && input.mKnown && input.mTorsoUpDot < settings.mToppleUpDot
            && !input.mRollOwns;
        const float upNow = input.mKnown ? input.mTorsoUpDot : 1.0f;
        const bool stillTipping = !settings.mToppleRising || upNow <= state.mToppleLastUp + 1e-4f;
        state.mToppleLastUp = upNow;
        state.mFallDecidedTime += dt;
        state.mToppleTime = tipped ? state.mToppleTime + (stillTipping ? dt : 0.0f) : 0.0f;
        // A lean past the line that rises too slowly to matter still falls
        // after 1 s (review 10-08: no stuck lean).
        state.mTippedFor = tipped ? state.mTippedFor + dt : 0.0f;
        if (state.mTippedFor >= 1.0f)
            state.mToppleTime = std::max(state.mToppleTime, 0.15f);
        if (state.mToppleTime >= 0.15f && !state.mToppled && !recoveryOwns)
        {
            state.mToppled = true;
            events.mToppled = true;
        }
        if (recoveryOwns)
            state.mToppled = false;
        const bool standingMode
            = is(input.mMode, M::Standing) || is(input.mMode, M::Bracing) || is(input.mMode, M::Stumbling);
        if (state.mToppled && settings.mRehold[1] > 0.0f && standingMode && input.mKnown
            && input.mTorsoUpDot > settings.mRehold[0] && state.mFallDecidedTime > 1.5f)
            state.mToppledStandTime += dt;
        else
            state.mToppledStandTime = 0.0f;
        if (state.mToppledStandTime > settings.mRehold[1])
        {
            state.mToppled = false;
            state.mToppledStandTime = 0.0f;
            state.mReheld = true;
            events.mReheld = true;
        }
        return events;
    }

    bool rootHeld(const RootRetake& retake, const RootHoldState& state, bool routeMatches, bool powered,
        bool interrupted)
    {
        return routeMatches && powered && !interrupted && retake.mDriven && !state.mToppled
            && (retake.mRecoveryOwns
                || (!state.mLandingReleased && (!state.mPhysicsOwned || retake.mReacquired)));
    }
}

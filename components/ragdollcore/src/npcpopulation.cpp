#include <ragdollcore/npcpopulation.hpp>

#include <ragdollcore/tuning.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace RagdollCore
{
    NpcPopulationLimits npcPopulationLimits()
    {
        std::array<float, 2> values{ 10.0f, 2000.0f };
        tuningList("OPENMW_RAGDOLL_NPC_BODIES", values);
        NpcPopulationLimits limits;
        const float count = std::isfinite(values[0]) ? std::clamp(values[0], 0.0f, 100.0f) : 10.0f;
        limits.mMaximum = static_cast<std::size_t>(std::lround(count));
        limits.mAcquireRadius = std::isfinite(values[1]) ? std::clamp(values[1], 0.0f, 8000.0f) : 2000.0f;
        limits.mReleaseRadius = limits.mAcquireRadius * sReleaseFactor;
        return limits;
    }

    NpcPopulationPlan planNpcPopulation(
        std::span<const NpcPopulationCandidate> candidates, const NpcPopulationLimits& limits)
    {
        NpcPopulationPlan plan;
        std::vector<const NpcPopulationCandidate*> kept; // powered, judged, staying
        std::size_t held = 0;
        std::vector<const NpcPopulationCandidate*> waiting; // unpowered, in reach
        for (const NpcPopulationCandidate& candidate : candidates)
        {
            if (candidate.mPowered)
            {
                if (candidate.mHeld)
                    ++held;
                else if (!candidate.mEligible || candidate.mDistance > limits.mReleaseRadius)
                    plan.mRelease.push_back(candidate.mId);
                else if (candidate.mPinned)
                    ++held;
                else
                    kept.push_back(&candidate);
            }
            else if (candidate.mEligible && candidate.mDistance <= limits.mAcquireRadius)
                waiting.push_back(&candidate);
        }
        const auto nearer = [](const NpcPopulationCandidate* a, const NpcPopulationCandidate* b) {
            return a->mDistance < b->mDistance || (a->mDistance == b->mDistance && a->mId < b->mId);
        };
        std::sort(kept.begin(), kept.end(), nearer);
        std::sort(waiting.begin(), waiting.end(), nearer);

        // Over the count (lowered on the options page): the farthest go.
        const std::size_t room = limits.mMaximum > held ? limits.mMaximum - held : 0;
        while (kept.size() > room)
        {
            plan.mRelease.push_back(kept.back()->mId);
            kept.pop_back();
        }
        plan.mFreeSlots = room - kept.size();

        // Full: the nearest newcomer takes the farthest body when clearly nearer.
        if (plan.mFreeSlots == 0 && !kept.empty() && !waiting.empty()
            && waiting.front()->mDistance + sSwapMargin < kept.back()->mDistance)
        {
            plan.mRelease.push_back(kept.back()->mId);
            kept.pop_back();
            plan.mFreeSlots = 1;
        }

        if (plan.mFreeSlots > 0)
        {
            plan.mAcquire.reserve(waiting.size());
            for (const NpcPopulationCandidate* candidate : waiting)
                plan.mAcquire.push_back(candidate->mId);
        }
        return plan;
    }
}

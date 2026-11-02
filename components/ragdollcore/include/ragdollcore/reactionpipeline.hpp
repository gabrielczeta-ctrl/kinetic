#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_REACTIONPIPELINE_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_REACTIONPIPELINE_H

// RXN-01 offline consumer: composes a resolved contact/receipt (the contract)
// with the balance authority and the behavior policy, and produces the bounded
// pose/limit requests the walker and the motor composer consume. This is the
// shared seam: the same pipeline runs in the offline tester and, later, in the
// native adapter through an integration note.

#include <ragdollcore/reactionauthority.hpp>
#include <ragdollcore/behavior.hpp>
#include <ragdollcore/reactioncontract.hpp>

#include <array>
#include <compare>
#include <cstddef>
#include <set>
#include <span>

namespace RagdollCore
{
    struct ReactionPipelineProfile
    {
        // Master switch. Disabled until the balance sequence and this contract
        // are registered; zero keeps the sampled clip and the full budget.
        bool mEnabled = false;
        BehaviorProfile mBehavior;
        ReactionAuthorityProfile mBalance;
    };

    struct ReactionPipelineState
    {
        struct ServedContact
        {
            std::uint64_t mAttackerActorIdentity = 0;
            std::uint64_t mTargetActorIdentity = 0;
            std::uint64_t mTargetRegistryGeneration = 0;
            std::uint64_t mContactId = 0;

            auto operator<=>(const ServedContact&) const = default;
        };

        BehaviorState mBehavior;
        struct LimbWound
        {
            float mSeverity = 0.0f;
            bool mCapable = true;
            bool mKnown = false;
        };
        // Left/right arm, left/right leg; last explicit receipt-driven state.
        // Missing receipts do not heal injuries. reset() ends their lifetime.
        std::array<LimbWound, 4> mWounds;
        ReactionAuthorityState mBalance;
        ReactionDisturbanceLatch mLatch;
        std::uint64_t mLastFrameStep = 0;
        std::uint64_t mLastServedReceiptId = 0;
        // State is actor-lifetime scoped and reset on removal/generation
        // replacement. Keep every identity during that lifetime so a delayed
        // A->B->...->A manifold cannot become fresh merely by aging out.
        std::set<ServedContact> mServedContacts;
        bool mFrameSeeded = false;
        void reset() { *this = ReactionPipelineState{}; }
    };

    // One fixed step. Deterministic rules:
    //  - disabled: the clip and the full budget are untouched;
    //  - invalid input: ignored, like an idle frame;
    //  - duplicate or out-of-order frames: no new events, timers still decay;
    //  - a duplicate contact id is consumed at most once; a receipt with an id
    //    not newer than the last served one is stale and ignored;
    //  - a fresh receipt on an already served contact is still processed.
    ReactionOutputs stepReactionPipeline(ReactionPipelineState& state,
        const ReactionContractInput& input, const ReactionPipelineProfile& profile);

    struct ReactionPoseTargets
    {
        std::size_t mSpine = static_cast<std::size_t>(-1);
        std::size_t mNeck = static_cast<std::size_t>(-1);
        std::size_t mLeftUpperArm = static_cast<std::size_t>(-1);
        std::size_t mRightUpperArm = static_cast<std::size_t>(-1);
        std::size_t mSpine1 = static_cast<std::size_t>(-1);
        std::size_t mSpine2 = static_cast<std::size_t>(-1);
        std::size_t mHead = static_cast<std::size_t>(-1);
        std::size_t mLeftForearm = static_cast<std::size_t>(-1);
        std::size_t mRightForearm = static_cast<std::size_t>(-1);
        std::size_t mLeftHand = static_cast<std::size_t>(-1);
        std::size_t mRightHand = static_cast<std::size_t>(-1);
        std::vector<int> mParents;
        float mExpressionScale = 1.0f;
    };

    // Resolve the complete authored hierarchy for anatomical pose composition.
    // Missing landmarks remain invalid and are skipped. Parents must precede
    // children; malformed hierarchies are rejected by applyReactionPose.
    ReactionPoseTargets makeReactionPoseTargets(const RagdollProfile& profile);

    // Anatomical model-space rotations propagate through the full branch, so
    // children do not counter-rotate against a changed parent. The root is never
    // edited; these are motor requests, subject to the authored joint limits.
    // Empty hierarchy retains the four-landmark compatibility probe.
    bool applyReactionPose(std::span<const Transform> pose, const ReactionOutputs& outputs,
        const ReactionPoseTargets& targets, std::span<Transform> out);
}

#endif

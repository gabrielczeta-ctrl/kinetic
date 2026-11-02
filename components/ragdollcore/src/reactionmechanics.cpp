#include <ragdollcore/reactionmechanics.hpp>

namespace RagdollCore
{
    bool composeReactionTargets(std::span<const Transform> clip, std::span<const Transform> balance,
        const ReactionOutputs& outputs, const ReactionPoseTargets& targets, std::span<Transform> out,
        const ReactionComposeProfile& profile)
    {
        if (clip.size() != balance.size() || out.size() != clip.size())
            return false;
        // 1. The clip is the base.
        std::copy(clip.begin(), clip.end(), out.begin());
        // 2. Bounded reaction pose offsets ride on top of the clip.
        const bool reactionApplied = applyReactionPose(clip, outputs, targets, out);
        // 3. The balance correction is capped by the pose-correction authority
        //    relative to the reaction-modified target, so a guard pose is never
        //    silently overwritten by the balance layer.
        const float authority = std::isfinite(outputs.mPoseCorrectionAuthority)
            ? std::clamp(outputs.mPoseCorrectionAuthority, 0.0f, 1.0f) : 0.0f;
        blendReactionBalancePose(out, balance, authority, profile.mAuthority, out);
        return reactionApplied || authority > 0.0f;
    }
}

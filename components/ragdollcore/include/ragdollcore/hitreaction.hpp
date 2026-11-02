#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_HITREACTION_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_HITREACTION_H

#include "referencegait.hpp"

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    enum class HitReactionBand
    {
        None,
        Absorb,
        Stagger,
        Fall,
    };

    /// Banding of landed hits on a powered body whose root is animation-held.
    ///
    /// The band is the only thing that may take such a body off its feet: the
    /// held root keeps it upright whatever the limbs measure, so a knock-down
    /// is a decision, not an observation. Three quantities decide it:
    ///  - the strongest single hit and the decaying sum of all hits, which
    ///    grade absorb against stagger;
    ///  - the same two restricted to regions that can drop a body (legs and
    ///    pelvis), which alone can reach the fall threshold. A flurry to the
    ///    head and chest therefore staggers however long it lasts, and a graze
    ///    on the shin does not convert it into a knock-down;
    ///  - an overwhelming single blow, which drops the body wherever it lands.
    struct HitReactionState
    {
        /// A single hit at or above this multiple of the fall impulse is a
        /// knock-down on any region.
        static constexpr float sOverwhelmingFallFactor = 3.0f;
        /// Decay rate of the sums (1/s) once the stun has ended.
        static constexpr float sAccumulatorDecayRate = 1.5f;

        float mPeakImpulse = 0.0f;
        float mAccumulator = 0.0f;
        float mKnockdownPeakImpulse = 0.0f;
        float mKnockdownAccumulator = 0.0f;

        void reset() { *this = HitReactionState{}; }

        /// Register a landed hit. The caller has already started or extended
        /// the stun; a hit outside any stun is a graze and is not recorded.
        void registerHit(float impulseMagnitude, bool regionCanKnockDown)
        {
            if (!std::isfinite(impulseMagnitude) || impulseMagnitude <= 0.0f)
                return;
            mPeakImpulse = std::max(mPeakImpulse, impulseMagnitude);
            mAccumulator += impulseMagnitude;
            if (regionCanKnockDown)
            {
                mKnockdownPeakImpulse = std::max(mKnockdownPeakImpulse, impulseMagnitude);
                mKnockdownAccumulator += impulseMagnitude;
            }
        }

        /// Advance by one frame. The peaks belong to the current stun and are
        /// cleared when it ends; the sums then decay so that only hits landed
        /// in quick succession add up.
        void decay(float duration, bool stunActive)
        {
            if (stunActive || !std::isfinite(duration) || duration <= 0.0f)
                return;
            mPeakImpulse = 0.0f;
            mKnockdownPeakImpulse = 0.0f;
            const float keep = std::max(0.0f, 1.0f - sAccumulatorDecayRate * duration);
            mAccumulator *= keep;
            mKnockdownAccumulator *= keep;
        }

        HitReactionBand band(bool stunActive, const ReferenceGaitProfile& gait) const
        {
            if (!stunActive)
                return HitReactionBand::None;
            const float knockdown = std::max(mKnockdownPeakImpulse, mKnockdownAccumulator);
            if (knockdown >= gait.mHitStunFallImpulse
                || mPeakImpulse >= gait.mHitStunFallImpulse * sOverwhelmingFallFactor)
                return HitReactionBand::Fall;
            if (std::max(mPeakImpulse, mAccumulator) >= gait.mHitStunStaggerImpulse)
                return HitReactionBand::Stagger;
            return HitReactionBand::Absorb;
        }
    };
}

#endif

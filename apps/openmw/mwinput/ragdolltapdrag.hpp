#ifndef OPENMW_MWINPUT_RAGDOLLTAPDRAG_H
#define OPENMW_MWINPUT_RAGDOLLTAPDRAG_H

#include <cmath>

namespace MWInput
{
    enum class RagdollTapDragAction
    {
        None,
        QuickAttack,
        BeginDrag,
        EndDrag,
    };

    // Defers the left-button decision just long enough to distinguish a click
    // from deliberate physical control. The threshold reuses the existing
    // 0.16-second physical click extension instead of introducing another
    // independently tuned combat duration.
    class RagdollTapDrag
    {
    public:
        static constexpr float sHoldThreshold = 0.16f;

        bool press()
        {
            if (mPhase != Phase::Idle)
                return false;
            mPhase = Phase::Pending;
            mHeldSeconds = 0.0f;
            return true;
        }

        RagdollTapDragAction advance(float duration, bool held)
        {
            if (mPhase != Phase::Pending || !held || !std::isfinite(duration) || duration <= 0.0f)
                return RagdollTapDragAction::None;
            mHeldSeconds += duration;
            if (mHeldSeconds + 1e-6f < sHoldThreshold)
                return RagdollTapDragAction::None;
            mPhase = Phase::Dragging;
            return RagdollTapDragAction::BeginDrag;
        }

        RagdollTapDragAction release()
        {
            const auto action = mPhase == Phase::Pending ? RagdollTapDragAction::QuickAttack
                : mPhase == Phase::Dragging             ? RagdollTapDragAction::EndDrag
                                                        : RagdollTapDragAction::None;
            reset();
            return action;
        }

        RagdollTapDragAction cancel()
        {
            const auto action
                = mPhase == Phase::Dragging ? RagdollTapDragAction::EndDrag : RagdollTapDragAction::None;
            reset();
            return action;
        }

        bool pending() const { return mPhase == Phase::Pending; }
        bool dragging() const { return mPhase == Phase::Dragging; }
        float heldSeconds() const { return mHeldSeconds; }

    private:
        enum class Phase
        {
            Idle,
            Pending,
            Dragging,
        };

        void reset()
        {
            mPhase = Phase::Idle;
            mHeldSeconds = 0.0f;
        }

        Phase mPhase = Phase::Idle;
        float mHeldSeconds = 0.0f;
    };
}

#endif

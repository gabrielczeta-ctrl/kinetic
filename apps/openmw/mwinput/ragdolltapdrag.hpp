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
        // K2_HOLD_AIM (user 11-05: "holding down click lets the player start the
        // swing from that direction"; "if you miss the click the camera moves a
        // lot"): the undecided press keeps the mouse off the view, and this much
        // travel (raw counts) makes it a drag at once - a click is a press that
        // moved less before it came up.
        static constexpr float sDragCounts = 24.0f;

        // Mouse travel while the press is undecided: a drag once it went far
        // enough (BeginDrag), else None. The travel is kept for the drag's
        // first step (pendingX/Y, in the caller's units).
        RagdollTapDragAction move(float countsX, float countsY, float unitsX, float unitsY)
        {
            if (mPhase != Phase::Pending || !std::isfinite(countsX) || !std::isfinite(countsY))
                return RagdollTapDragAction::None;
            mCountsX += countsX;
            mCountsY += countsY;
            mUnitsX += std::isfinite(unitsX) ? unitsX : 0.0f;
            mUnitsY += std::isfinite(unitsY) ? unitsY : 0.0f;
            if (std::hypot(mCountsX, mCountsY) + 1e-6f < sDragCounts)
                return RagdollTapDragAction::None;
            mPhase = Phase::Dragging;
            return RagdollTapDragAction::BeginDrag;
        }
        float pendingX() const { return mUnitsX; }
        float pendingY() const { return mUnitsY; }

        bool press()
        {
            if (mPhase != Phase::Idle)
                return false;
            mPhase = Phase::Pending;
            mHeldSeconds = 0.0f;
            mCountsX = mCountsY = mUnitsX = mUnitsY = 0.0f;
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
            mCountsX = mCountsY = mUnitsX = mUnitsY = 0.0f;
        }

        Phase mPhase = Phase::Idle;
        float mHeldSeconds = 0.0f;
        float mCountsX = 0.0f, mCountsY = 0.0f, mUnitsX = 0.0f, mUnitsY = 0.0f;
    };
}

#endif

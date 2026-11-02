#include "../../openmw/mwinput/ragdolltapdrag.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace
{
    using MWInput::RagdollTapDrag;
    using MWInput::RagdollTapDragAction;

    TEST(RagdollTapDragTest, releaseBeforeThresholdProducesOneQuickAttack)
    {
        RagdollTapDrag state;
        EXPECT_TRUE(state.press());
        EXPECT_EQ(state.advance(RagdollTapDrag::sHoldThreshold - 0.01f, true), RagdollTapDragAction::None);
        EXPECT_EQ(state.release(), RagdollTapDragAction::QuickAttack);
        EXPECT_EQ(state.release(), RagdollTapDragAction::None);
    }

    TEST(RagdollTapDragTest, crossingThresholdStartsDragAndReleaseOnlyEndsIt)
    {
        RagdollTapDrag state;
        ASSERT_TRUE(state.press());
        EXPECT_EQ(state.advance(RagdollTapDrag::sHoldThreshold * 0.5f, true), RagdollTapDragAction::None);
        EXPECT_EQ(state.advance(RagdollTapDrag::sHoldThreshold * 0.5f, true), RagdollTapDragAction::BeginDrag);
        EXPECT_TRUE(state.dragging());
        EXPECT_EQ(state.advance(1.0f, true), RagdollTapDragAction::None);
        EXPECT_EQ(state.release(), RagdollTapDragAction::EndDrag);
        EXPECT_FALSE(state.pending());
        EXPECT_FALSE(state.dragging());
    }

    TEST(RagdollTapDragTest, cancellationAndInvalidTimeNeverCreateAnAttack)
    {
        for (const float duration : { 0.0f, -1.0f, std::numeric_limits<float>::infinity(),
                 std::numeric_limits<float>::quiet_NaN() })
        {
            RagdollTapDrag state;
            ASSERT_TRUE(state.press());
            EXPECT_EQ(state.advance(duration, true), RagdollTapDragAction::None);
            EXPECT_EQ(state.cancel(), RagdollTapDragAction::None);
            EXPECT_EQ(state.release(), RagdollTapDragAction::None);
        }
        RagdollTapDrag dragging;
        ASSERT_TRUE(dragging.press());
        ASSERT_EQ(dragging.advance(RagdollTapDrag::sHoldThreshold, true), RagdollTapDragAction::BeginDrag);
        EXPECT_EQ(dragging.cancel(), RagdollTapDragAction::EndDrag);
        EXPECT_EQ(dragging.release(), RagdollTapDragAction::None);
    }

    TEST(RagdollTapDragTest, duplicatePressCannotRestartOrShortenOneGesture)
    {
        RagdollTapDrag state;
        ASSERT_TRUE(state.press());
        EXPECT_EQ(state.advance(RagdollTapDrag::sHoldThreshold * 0.75f, true), RagdollTapDragAction::None);
        EXPECT_FALSE(state.press());
        EXPECT_EQ(state.advance(RagdollTapDrag::sHoldThreshold * 0.25f, true), RagdollTapDragAction::BeginDrag);
        EXPECT_FALSE(state.press());
    }
}

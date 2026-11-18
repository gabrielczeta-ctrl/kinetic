#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_FISTSTEER_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_FISTSTEER_HPP

#include <ragdollcore/cutwheel.hpp>
#include <ragdollcore/fistcontrol.hpp>

#include <array>
#include <optional>

namespace RagdollCore
{
    // FIST-06 (K2_FIST_STEER; user 11-12: "when I click and hold with fists
    // nothing happens", then "it must feel like... really good /controllable":
    // FIST-05's flick was a code for a punch, and nothing on the body answered
    // the hand until the button came up). Two hands, two buttons, the mouse is
    // the fist:
    //   a tap throws that hand's straight at the crosshair;
    //   held, the fist follows the mouse - the drag moves it from its guard
    //   out along the wind-up of the fist line that lies that way (to the
    //   hand's own side the hook, down and out the uppercut, up the overhand,
    //   straight back the cross), `mCounts` of travel to the full wind-up;
    //   let go, the fist is thrown from where it is, on the line it was
    //   drawn back on, as hard as it was drawn; under `mStraight` of the full
    //   draw it is the straight.
    // A punch thrown while its hand is busy waits for it, up to 0.6 s.
    //
    // FIST-07 (`mFeet`; user 11-16, on the steered hold: "better than before
    // but still pretty bad", "I kinda want to not use the weapon swinging
    // style"; research/FISTS-02): the mouse is left to the view. A tap throws
    // the punch the movement keys pick - none the straight, a strafe that
    // hand's hook, back the uppercut, forward the long straight - and a held
    // button winds the same punch up (shown: the fist goes out along its
    // wind-up over `mCharge` s) for letting go to throw it harder.
    //
    // Frame: the cut wheel's (the chest centre, x right, y forward, z up).
    // The left hand runs the right's lines mirrored across the body.
    // Pure: no engine types.

    struct FistSteerSettings
    {
        float mCounts = 300.0f; // mouse counts (sensitivity 1) to the full wind-up
        float mTap = 0.2f; // s: a shorter press without travel is a tap
        float mTapCounts = 40.0f; // ... and this much travel makes it a hold
        float mStraight = 0.3f; // the draw under which a held fist throws the straight
        float mRate = 20.0f; // 1/s: how fast the shown fist follows the mouse
        bool mFeet = false; // FIST-07: the movement keys pick the punch, a hold charges it
        // FIST-09: taps and charged holds as mFeet, the punch picked by where
        // the crosshair is on the foe (FistSteerInput::mAimZone).
        bool mAim = false;
        // FIST-12 (user 11-16: "make it so fists can work with clicks but
        // make the charging the same as the other weapons, but each
        // direction is a different hit style"): a click is mAim's tap; a held
        // button winds up as the weapons do - the mouse draws the fist out
        // along the wind-up of the fist line that lies that way, `mCounts` to
        // the full charge, and letting go throws that line as hard as it was
        // drawn. Time alone charges nothing.
        bool mDrag = false;
        // The swing (mDrag; user 11-16: "holding and releasing should also
        // follow the same rule! charge the fist and then swing it hard to the
        // other direction"): a wound fist (the draw at `mStraight` or past
        // it) is thrown with the button still down by a stroke back against
        // its draw - `mSwing` counts of travel against the draw within
        // `mSwingTime`, no further than 50 degrees off it. The punch is the
        // draw's as it was before the stroke, line and power. The hand winds
        // up again once that punch is done; the hold's release throws only
        // what was wound after it. Slower travel back just lessens the draw.
        // The weapons' cut asks 150 counts inward at 2,500 counts/s; a
        // fist's whole draw is 300 and a wound one may be 90. 0 = letting go
        // only.
        float mSwing = 100.0f;
        float mSwingTime = 0.1f;
        // The weapons' rule (user 11-17: "lets make it so fist to fist works
        // like any other weapon ... I need to windup and move the mouse
        // quickly to the other side to deliver the punch"): false = with
        // mSwing on, letting go of a wound fist throws nothing (it goes back
        // to the guard, as a weapon's wind-up does); only the swing and the
        // click punch. A press let go with under mTapCounts of travel inside
        // `mClick` s is still the click.
        bool mRelease = true;
        float mClick = 0.4f;
        // After a swing the hand winds up again only once the mouse has been
        // this still (counts/s) for `mSettleTime` s at some time since it: the stroke's
        // follow-through wound the opposite punch (the user's strokes ran
        // 1,300-2,700 counts, past the wind-up's 800-2,400).
        float mSettle = 400.0f, mSettleTime = 0.08f;
        // K2_FIST_CHARGE (user 11-16: "I wasnt able to charge the attacks in
        // hand to hand"; a held straight sat 5 u from its guard): a charged
        // fist draws this much further back along its wind-up (u at the full
        // charge) and coils the trunk this far (radians, the punching
        // shoulder back). mFeet / mAim only.
        float mChargeDraw = 0.0f, mChargeCoil = 0.0f;
        float mCharge = 0.6f; // s held past the tap to the full wind-up (mFeet)
    };

    struct FistSteerPress
    {
        bool mHeld = false;
        float mDragRight = 0.0f, mDragUp = 0.0f; // mouse counts this step
        // The input layer started an attack of its own for this hand (the
        // game's attack clip): a tap, unless this hand threw within 0.3 s
        // (the same click, seen as a press too).
        bool mTap = false;
        bool mBusy = false; // the hand is in a punch: no hold shown, a punch waits
        bool mCancel = false; // the press ended without a release (a menu, lost focus): nothing is thrown
        // The press was already held this long when it got here (the left
        // button's hold is passed on only once it is a hold, 0.16 s).
        float mHoldLead = 0.0f;
    };

    struct FistSteerInput
    {
        float mDt = 1.0f / 60.0f;
        FistSteerPress mLeft, mRight;
        bool mBlocked = false; // the guard, a shove, a trapped arm: holds drop, nothing is thrown
        bool mCrouched = false; // sneaking: straights go to the body
        // The movement keys (-1..1: to the right, forward), for mFeet.
        float mMoveSide = 0.0f, mMoveForward = 0.0f;
        // Where the crosshair is on the foe, for mAim: 1 its head, 0 its
        // chest, -1 its belly or lower (no foe: the view's pitch).
        int mAimZone = 0;
    };

    // A held fist: where it is, and where on the wheel.
    struct FistSteerHold
    {
        JPH::Vec3 mHand = JPH::Vec3::sZero();
        JPH::Vec3 mPole = -JPH::Vec3::sAxisZ();
        float mTwist = 0.0f; // the chest's turn, radians
        // The wheel's angle of the draw (the right hand's sense: 0 = to the
        // right, pi/2 = up; the left hand's is mirrored already) and how far
        // along the wind-up, 0..1.
        float mAngle = 0.0f, mAmount = 0.0f;
    };

    struct FistSteerPunch
    {
        FistHand mHand = FistHand::Right;
        std::size_t mLine = 0; // in fistLines() (the right hand's; the left mirrors it)
        PunchKind mKind = PunchKind::Straight;
        float mPower = 0.0f; // 0 a quick punch .. 1 the full draw
        bool mWound = false; // thrown from a held fist: its wind-up is done
        JPH::Vec3 mFrom = JPH::Vec3::sZero(); // ... from here
    };

    struct FistSteerOutput
    {
        std::optional<FistSteerHold> mLeft, mRight;
        std::optional<FistSteerPunch> mLeftPunch, mRightPunch;
        // HUD: left, right. -1 means not held; charge is independent of the
        // eased hand pose and uses the same clock as the released punch.
        std::array<float, 2> mCharge{ -1.0f, -1.0f };
        std::array<PunchKind, 2> mKind{ PunchKind::Straight, PunchKind::Straight };
        // mDrag, a held fist: its draw as the mouse went (to the right, up;
        // 1 = the full wind-up), for the weapons' wind-up reticle.
        std::array<float, 2> mDrawRight{ 0.0f, 0.0f }, mDrawUp{ 0.0f, 0.0f };
    };

    // The fist line's index a punch kind is thrown on (fistLines()).
    std::size_t fistLineIndex(PunchKind kind);
    // The punch kind of a fist line (its timing and its name in the log).
    PunchKind fistLineKind(const AttackLine& line);

    // FIST-07: the fist line the movement keys pick (in fistLines()).
    std::size_t fistLineForMovement(float side, float forward, bool crouched);
    // FIST-09: the fist line the crosshair picks. A tap: the straight (the
    // body shot low or sneaking). A charged punch: the hook at the head, the
    // straight at the chest, the uppercut low.
    std::size_t fistLineForAim(int zone, bool charged, bool crouched);

    // A held fist's place: `amount` (0..1) along the wind-up of the wheel at
    // `angle` (the right hand's sense), lifted a little out of the guard at
    // rest so a press shows. `hand` Left: mirrored across the body.
    FistSteerHold fistHoldPlace(FistHand hand, float angle, float amount);

    class FistSteer
    {
    public:
        FistSteerOutput step(const FistSteerInput& input, const FistSteerSettings& settings = {});
        bool holding(FistHand hand) const { return mHands[index(hand)].mHolding; }

    private:
        struct Hand
        {
            bool mWasHeld = false;
            bool mHolding = false;
            float mTime = 0.0f, mTravel = 0.0f;
            float mX = 0.0f, mY = 0.0f; // the draw, counts (the right hand's sense)
            float mShownX = 0.0f, mShownY = 0.0f;
            std::optional<FistSteerPunch> mPending;
            float mPendingAge = 0.0f;
            float mSinceThrown = 1e9f;
            // The swing: the draw at the last steps (the hold's clock, the
            // draw), whether this hold has thrown by a stroke, and that
            // punch's course (1 waiting to start, 2 for its hand to be busy,
            // 3 for it to be done; the draw stays empty meanwhile).
            struct Drawn
            {
                float mAt = 0.0f, mX = 0.0f, mY = 0.0f;
            };
            std::array<Drawn, 48> mDrawn;
            std::size_t mDrawnCount = 0, mDrawnNext = 0;
            bool mStruck = false;
            int mSwung = 0;
            float mSwungAge = 0.0f;
            float mStill = 0.0f; // s the mouse has been still (mSettle)
            bool mSettled = false; // ... for mSettleTime at some time since the swing
        };
        static std::size_t index(FistHand hand) { return hand == FistHand::Left ? 0 : 1; }
        void stepHand(FistHand hand, const FistSteerPress& press, const FistSteerInput& input,
            const FistSteerSettings& settings, std::optional<FistSteerHold>& hold, std::optional<FistSteerPunch>& punch);
        std::array<Hand, 2> mHands;
    };
}

#endif

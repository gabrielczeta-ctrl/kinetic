#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_PALMPARRY_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_PALMPARRY_HPP

#include <ragdollcore/wheelplanner.hpp>

#include <array>
#include <optional>
#include <string_view>

namespace RagdollCore
{
    // FIST-08 (K2_PALM_PARRY; user 11-16: "I'm not sure if the blocking is
    // working and I'm thinking it could be that 'parrying' would be something
    // like throwing a palm in that direction and that causing weapons to be
    // deflected"). The bare-handed guard (both buttons held):
    //   the hands wait open in front of the face, where the mouse's guard
    //   zone puts them (middle, high, low, left, right);
    //   a blow coming in (the foe's stroke, from its wind-up on) is answered
    //   by the hand on its side: after a reaction time the palm goes out to
    //   the foe's weapon, between its hand and its striking part, and follows
    //   it for as long as the stroke runs (`mHold` at the longest);
    //   while the palm is out a blade that meets it, or lands within
    //   `mCover` of it, is deflected: knocked off its line, the cut
    //   ended, the foe open for a moment, a share of the blow still paid by
    //   the arm at low skill;
    //   a blow that lands before the palm is out, after it came back, or
    //   away from it, lands (a late or missed parry takes it on the arm) -
    //   but a blade that comes slowly (under 200 u/s: laid on the forearms,
    //   pushed in from a clinch) onto the raised hands is stopped by them as
    //   the old guard stopped it.
    // One palm a blow. Morrowind's numbers: Hand-to-hand, Agility, Luck and
    // fatigue (`palmParrySkill`, vanilla's block term with Hand-to-hand for
    // Block) set the reaction time, how long the palm stays out, how much of
    // the blow is turned and how long the foe is open. The player's zone is
    // a read of the blow: the zone the blow comes in halves the reaction and
    // adds to the rest ("committed"); another zone slows it; the middle is
    // neither.
    //
    // Frame: the wheel's chest-centre frame (x right, y forward, z up, units).
    struct PalmParrySettings
    {
        bool mOn = false;
        float mReact = 0.22f; // s from seeing the blow to the palm leaving, at no skill (0.35 of it at full)
        float mHold = 1.0f; // s the palm stays out at the longest, at full skill (0.6 of it at none)
        float mCover = 12.0f; // u from the palm's hand within which a blade's contact is deflected
        float mKnock = 30.0f; // u the foe's weapon hand is thrown, at full skill (half at none)
        float mOpen = 0.9f; // s the foe cannot cut again, at full skill (0.4 of it at none)
        // The reaction time's factor with the hands waiting in another zone
        // than the blow's (11-17: a tap's flick picks a zone every time; 13 of
        // the player's 34 palms that day waited the 1.5x). 1 = as the middle.
        float mWrong = 1.5f;
    };

    // "on,react,hold,cover,knock,open,wrong"; null, empty or on <= 0 is off.
    PalmParrySettings parsePalmParrySettings(const char* text);

    // K2_PALM_TAP (registry; user 11-17: "I'm not able to parry", then "Yes,
    // tap parries"): a tap of both buttons with bare fists is a timed palm
    // parry - the guard stays up for `window` s from the press - and the
    // shove is the second press of both inside `shove` s of the last.
    struct PalmTapSettings
    {
        float mWindow = 0.0f; // s from the press the guard stays up after a tap; 0 = a tap of both is the shove
        float mShove = 0.3f; // s: both pressed again this soon after the last press is the shove
        // s past the window a tapped guard stays up while it reads a blow or
        // its palm is out (11-17: the player taps as the attack starts, 0.75-0.9 s
        // before the blade; 8 of 15 parries that failed had the window end
        // 12-100 ms before it). Over 0, a press of both while a blow is read
        // is never the shove either (taps came every 0.1-0.2 s). 0 = neither.
        float mCarry = 0.0f;
    };
    inline constexpr float sPalmTapHeld = 0.2f; // s: both buttons let go sooner than this were a tap

    // "window,shove,carry"; null, empty or window <= 0 is off.
    PalmTapSettings parsePalmTapSettings(const char* text);
    // The guard time left when both buttons, down for `held` s, are let go:
    // the rest of the window after a tap, none after a hold or with it off.
    float palmTapLeft(const PalmTapSettings& settings, float held);
    // Both buttons pressed `sincePress` s after they were last pressed: the shove.
    bool palmTapShoves(const PalmTapSettings& settings, float sincePress);
    // A tapped guard whose window is ending `sincePress` s after its press,
    // with a blow read or the palm out: it stays up.
    bool palmTapCarries(const PalmTapSettings& settings, float sincePress);

    struct PalmParryStats
    {
        float mHandToHand = 30.0f, mAgility = 40.0f, mLuck = 40.0f;
        float mFatigue = 1.0f; // current / maximum
    };

    // 0..1: (Hand-to-hand + Agility / 5 + Luck / 10) x (0.75 + 0.5 x fatigue) / 100.
    float palmParrySkill(const PalmParryStats& stats);

    struct PalmThreat
    {
        JPH::Vec3 mStrike = JPH::Vec3::sZero(); // the weapon's striking part
        JPH::Vec3 mVelocity = JPH::Vec3::sZero(); // its velocity
        JPH::Vec3 mHand = JPH::Vec3::sZero(); // the foe's weapon hand
        // The foe's stroke on the wheel: 0 its guard, 1 the wind-up, 2 the
        // cut, 3 the return; -1 not on the wheel. And a stroke of another
        // kind running (an NPC's AI strike).
        int mStage = -1;
        bool mAttacking = false;
    };

    struct PalmParryInput
    {
        float mDt = 0.0f;
        bool mGuard = false;
        WheelGuardZone mZone = WheelGuardZone::Middle;
        std::optional<PalmThreat> mThreat;
        std::array<JPH::Vec3, 2> mShoulder{ JPH::Vec3(-11.5f, 0.0f, 8.0f), JPH::Vec3(11.5f, 0.0f, 8.0f) }; // left, right
        std::array<float, 2> mArmLength{ 34.0f, 34.0f };
        float mSkill = 0.3f;
    };

    enum class PalmPhase
    {
        Down, // no guard
        Ready, // the hands wait
        Reach, // a palm is out
        Recover, // it comes back
    };

    struct PalmParryOutput
    {
        PalmPhase mPhase = PalmPhase::Down;
        // Where each hand is asked (left, right) and its elbow's side; none with the guard down.
        std::array<std::optional<JPH::Vec3>, 2> mHand;
        std::array<JPH::Vec3, 2> mPole{ -JPH::Vec3::sAxisZ(), -JPH::Vec3::sAxisZ() };
        int mPalm = -1; // the hand that is out or coming back: 0 left, 1 right
        bool mLive = false; // the palm deflects what it meets
        bool mCommitted = false; // the player's zone named this blow's
        bool mLaunched = false; // the palm left this step
        bool mBlow = false; // a blow is coming in
    };

    // The zone a point in front of the chest lies in.
    WheelGuardZone palmZone(const JPH::Vec3& point);

    class PalmParry
    {
    public:
        PalmParryOutput step(const PalmParryInput& input, const PalmParrySettings& settings);
        // The palm met the blade: it stays a moment and comes back.
        void met();
        PalmPhase phase() const { return mPhase; }
        bool live() const { return mLive; }
        bool coming() const { return mComing; } // a blow was read last step
        int palm() const { return mPalm; }
        bool committed() const { return mCommitted; }
        // Seconds since the palm left (Reach and Recover), else -1.
        float sinceLaunch() const { return mPhase == PalmPhase::Reach || mPhase == PalmPhase::Recover ? mSince : -1.0f; }
        // A blow has been seen coming for this long (0 = none).
        float seen() const { return mSeen; }

    private:
        PalmPhase mPhase = PalmPhase::Down;
        float mTime = 0.0f; // in the phase
        float mSince = 0.0f; // since the launch
        float mSeen = 0.0f; // the blow coming in has been seen this long
        float mQuiet = 0.0f; // nothing has come in for this long
        int mStage = -1; // the foe's stroke stage last step
        float mHoldFor = 0.0f; // this palm's time out
        bool mAnswered = false; // this blow has had its palm
        bool mComing = false; // a blow was coming last step
        bool mLive = false;
        bool mCommitted = false;
        int mPalm = -1;
        JPH::Vec3 mTarget = JPH::Vec3::sZero();
        std::array<JPH::Vec3, 2> mShown{ JPH::Vec3::sZero(), JPH::Vec3::sZero() };
    };

    enum class PalmContactOutcome
    {
        None, // no palm parry in it (the guard is down, or the switch is off)
        Deflect, // the palm turned the blow
        Late, // the blow landed before the palm was out, or after it came back
        Miss, // the palm was out and the blow landed away from it
        Covered, // no palm met it, but it came slowly onto the raised hands: the guard's cover holds
    };

    struct PalmContactInput
    {
        PalmPhase mPhase = PalmPhase::Down;
        bool mLive = false;
        bool mCommitted = false;
        bool mOnPalmArm = false; // the blade struck the palm's own arm
        float mDistance = 1e9f; // u from the palm's hand to the contact
        float mSpeed = 1e9f; // u/s the blade came in at
        bool mCovered = false; // the contact is within the guard's cover of a hand
        float mSkill = 0.3f;
        float mWeaponWeight = 0.0f; // Morrowind weight of the foe's weapon
        bool mTwoHanded = false;
    };

    struct PalmContactResult
    {
        PalmContactOutcome mOutcome = PalmContactOutcome::None;
        float mDamageScale = 1.0f; // the share of the blow the arm still pays
        float mKnock = 0.0f; // u the foe's weapon hand is thrown
        float mOpen = 0.0f; // s the foe cannot cut again
        float mImpulseScale = 0.0f; // of the blade's own momentum, turned aside
    };

    PalmContactResult resolvePalmContact(const PalmContactInput& input, const PalmParrySettings& settings);

    std::string_view palmContactOutcomeName(PalmContactOutcome outcome);
    std::string_view palmPhaseName(PalmPhase phase);
}

#endif

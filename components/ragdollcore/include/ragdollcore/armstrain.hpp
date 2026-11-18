#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_ARMSTRAIN_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_ARMSTRAIN_H

namespace RagdollCore
{
    // A guard's arm tires with every clash it takes (user 11-07: "when this
    // happens can we make it so they lose strength in the arm? a couple of
    // clashes should open up their guard"). Each blade-on-blade clash the
    // fighter was the parry in strains its weapon arm by the clash's ratio
    // (the attack's momentum over the guard's firmness): a hard blow strains
    // it more than a tap. The strain holds a moment, then eases off. While
    // strained the arm is weaker - its drive, its guard's firmness in the
    // clash table, how fast it comes back from a knock - and at full strain
    // it is spent: the guard drops and cannot come back up until the arm has
    // eased to half.
    //
    // OPENMW_RAGDOLL_K2_ARM_STRAIN = "per clash, hold s, recover/s, weaken,
    // you" (registry; per clash 0 = off). `weaken`: the arm's strength lost
    // at full strain; `you`: the player's share (0 = NPCs only).
    struct ArmStrainSettings
    {
        float mPerClash = 0.0f;
        float mHold = 1.5f;
        float mRecover = 0.2f;
        float mWeaken = 0.6f;
        float mPlayer = 0.0f;

        bool on() const { return mPerClash > 0.0f; }
    };

    ArmStrainSettings armStrainSettings();

    class ArmStrain
    {
    public:
        // Strain ages: held `mHold` s after the last clash, then eased off.
        void step(float dt, const ArmStrainSettings& settings);

        // A clash this arm parried, at the clash table's ratio; `share`
        // scales it (the player's share). True when it left the arm spent.
        bool takeClash(float ratio, float share, const ArmStrainSettings& settings);

        // The arm's strength share: 1 fresh, 1 - weaken at full strain.
        float strength(const ArmStrainSettings& settings) const;

        bool spent() const { return mSpent; }
        float strain() const { return mStrain; }
        void reset() { *this = ArmStrain{}; }

    private:
        float mStrain = 0.0f; // 0 fresh, 1 spent
        float mSinceClash = 1e9f;
        bool mSpent = false;
    };
}

#endif

#ifndef OPENMW_MECHANICS_COMBAT_H
#define OPENMW_MECHANICS_COMBAT_H

#include <optional>
#include <utility>
#include <ragdollcore/combatstate.hpp>
#include <ragdollcore/dueltactics.hpp>
#include <ragdollcore/physicaldefense.hpp>

namespace osg
{
    class Vec3f;
}

namespace MWWorld
{
    class Ptr;
}

namespace MWMechanics
{
    /// Reach at which a powered ragdoll's physical blow can actually arrive.
    /// The check below compares the gap between the two bodies' bounds, so the
    /// value is what remains of the arm once the target's own half extent is
    /// discounted: large enough to connect at body contact, short enough that
    /// the fist visibly arrives. The authored reach (fCombatDistance times
    /// weapon reach, about two meters) read as contactless hits.
    constexpr float cRagdollMeleePhysicalReach = 40.0f;

    /// Fists are shorter than any weapon. A powered unarmed fighter must close
    /// further before its swing is committed, or the AI attacks from a
    /// distance the arm cannot reach and the duel circles instead of landing.
    constexpr float cRagdollMeleeUnarmedReach = 26.0f;

    struct PhysicalWeaponDamageResult final
    {
        float mImpactFraction = 0.0f;
        float mAuthoredDamage = 0.0f;
        float mFinalDamage = 0.0f;
        // Equivalent reaction impulse for the victim's hit-reaction bands,
        // computed from the pre-blunt damage so a pommel smack still staggers.
        float mReactionImpulse = 0.0f;
        // The same blow before the whole-body push scale: what the struck part
        // feels (the RXN reaction layer's severity).
        float mLocalReactionImpulse = 0.0f;
        bool mBlocked = false;
        RagdollCore::PhysicalDefenseOutcome mDefenseOutcome = RagdollCore::PhysicalDefenseOutcome::None;
    };

    /// Physical strikes feed the reaction bands with damage-scaled impulse.
    /// This is the impulse per point of pre-blunt damage; 0 disables physical
    /// hit reactions. Default 72 (DMG-04): a full claymore blow staggers,
    /// repeated or overwhelming blows put a victim down.
    void setPhysicalReactionPerDamage(float value);
    float getPhysicalReactionPerDamage();

    /// Test-range normalization for bare-handed damage. Hand-to-hand damage
    /// scales directly with the skill value, and a developer character can
    /// carry a maxed skill; 1.0 is the untouched game formula.
    void setPhysicalHandDamageScale(float value);
    float getPhysicalHandDamageScale();

    // No value for empty/non-actor, passive, or not-yet-stepped actors.
    // Read on the game thread; the snapshot owns no physics or actor pointers.
    std::optional<RagdollCore::CombatState> getRagdollCombatState(const MWWorld::Ptr& actor);
    std::optional<RagdollCore::DuelBodyView> getRagdollDuelView(const MWWorld::Ptr& actor);
    // AI-02 item 2: keep a powered NPC's guard up for this long (a backpedal).
    void requestRagdollAiGuard(const MWWorld::Ptr& actor, float seconds);
    // AI-02 item 5: the AI answers its won parry now (the wheel winds up short).
    void requestRagdollAiRiposte(const MWWorld::Ptr& actor);

    bool applyOnStrikeEnchantment(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, const MWWorld::Ptr& object,
        const osg::Vec3f& hitPosition, const bool fromProjectile = false);

    /// @return can we block the attack?
    bool blockMeleeAttack(const MWWorld::Ptr& attacker, const MWWorld::Ptr& blocker, const MWWorld::Ptr& weapon,
        float damage, float attackStrength);

    /// @return does normal weapon resistance and weakness apply to the weapon?
    bool isNormalWeapon(const MWWorld::Ptr& weapon);

    void resistNormalWeapon(
        const MWWorld::Ptr& actor, const MWWorld::Ptr& attacker, const MWWorld::Ptr& weapon, float& damage);

    void applyWerewolfDamageMult(const MWWorld::Ptr& actor, const MWWorld::Ptr& weapon, float& damage);

    /// @note for a thrown weapon, \a weapon == \a projectile, for bows/crossbows, \a projectile is the arrow/bolt
    /// @note \a victim may be empty (e.g. for a hit on terrain), a non-actor (environment objects) or an actor
    void projectileHit(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, MWWorld::Ptr weapon,
        const MWWorld::Ptr& projectile, const osg::Vec3f& hitPosition, float attackStrength, float attackWindUp);

    /// Get the chance (in percent) for \a attacker to successfully hit \a victim with a given weapon skill value
    float getHitChance(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, int skillValue);

    /// Applies damage to attacker based on the victim's elemental shields.
    void applyElementalShields(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim);

    /// @param damage Unmitigated weapon damage of the attack
    /// @param hit Was the attack successful?
    /// @param weapon The weapon used.
    /// @note if the weapon is unequipped as result of condition damage, a new Ptr will be assigned to \a weapon.
    void reduceWeaponCondition(float damage, bool hit, MWWorld::Ptr& weapon, const MWWorld::Ptr& attacker);

    /// Adjust weapon damage based on its condition. A used weapon will be less effective.
    void adjustWeaponDamage(float& damage, const MWWorld::Ptr& weapon, const MWWorld::Ptr& attacker);

    void getHandToHandDamage(
        const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, float& damage, bool& healthdmg, float attackStrength);

    /// Apply the fatigue loss incurred by attacking with the given weapon (weapon may be empty = hand-to-hand)
    void applyFatigueLoss(const MWWorld::Ptr& attacker, const MWWorld::Ptr& weapon, float attackStrength);

    int getFightTerm(const MWWorld::Ptr& actor, const MWWorld::Ptr& target);
    float getFightDispositionBias(float disposition);
    float getFightDistanceBias(const MWWorld::Ptr& actor1, const MWWorld::Ptr& actor2);
    float getAggroDistance(const MWWorld::Ptr& actor, const osg::Vec3f& lhs, const osg::Vec3f& rhs);
    bool isAggressionCapable(const MWWorld::Ptr& actor);
    bool isAggressive(const MWWorld::Ptr& actor, const MWWorld::Ptr& target);

    // Cursed distance calculation used for combat proximity and hit checks in Morrowind
    float getDistanceToBounds(const MWWorld::Ptr& actor, const MWWorld::Ptr& target);

    float getMeleeWeaponReach(const MWWorld::Ptr& actor, const MWWorld::Ptr& weapon);

    bool isInMeleeReach(const MWWorld::Ptr& actor, const MWWorld::Ptr& target, const float reach);

    // Similarly cursed hit target selection
    std::pair<MWWorld::Ptr, osg::Vec3f> getHitContact(const MWWorld::Ptr& actor, float reach);

    // Pushes a ragdolled victim at the melee contact point, along the
    // attacker's aim, scaled by attack strength and by the weapon's mass
    // when one is used, and halved when blocked. Pass an empty weapon for
    // hand-to-hand and natural attacks. Returns false when the victim has no
    // active ragdoll.
    bool applyMeleeRagdollImpulse(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim,
        const MWWorld::Ptr& weapon, const osg::Vec3f& hitPosition, float attackStrength, bool blocked);

    // Skill scaled to a damage multiplier in (0, 1], for the geometric melee
    // connection: when contact is decided by geometry the to-hit roll no
    // longer gates the blow, so the same skill instead scales what the blow
    // does. One means the attacker would always have hit anyway. Never
    // greater than one, so this can only reduce damage.
    float meleeSkillDamageMultiplier(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, int skillValue);

    // Fraction of the momentum this weapon can possibly deliver that a swing
    // at this strength actually delivers, in [0, 1]. Weapon damage scales by
    // it instead of by attack strength directly, so the weapon's maximum
    // damage corresponds to its maximum possible momentum. Uses the same
    // terms as applyMeleeRagdollImpulse, so what a body is pushed by and what
    // it is damaged by cannot drift apart. Blocking is applied separately by
    // the caller and is not included here.
    float meleeImpactMomentumFraction(const MWWorld::Ptr& weapon, float attackStrength);

    // DMG-04: the part of one blow a contact event pays for - the regional
    // split of an impact (1 for a single region), or one installment of an
    // edge draw (a cut).
    struct PhysicalBlowShare final
    {
        float mShare = 1.0f;
        bool mCut = false;
    };

    // Skill as control (DMG-04, user 2026-09-28): the attacker's vanilla
    // attack term (weapon skill, Agility, Luck, fatigue, Fortify Attack,
    // Blind) as how well the weapon is handled, 0.2..1. The range's "always
    // hits" is 1. Drives the weapon controller, never the damage.
    float physicalWeaponControl(const MWWorld::Ptr& attacker, const MWWorld::Ptr& weapon);

    // Alpha 1.0 skill parity: vanilla's hit roll (getHitChance: weapon skill,
    // Agility, Luck, fatigue, Fortify Attack, Blind, the victim's evasion)
    // decides whether a physical blow bites; a failed roll is a glancing blow.
    // Softened by a floor (OPENMW_RAGDOLL_HIT_FLOOR, default 35 percent).
    // The range's "always hits" always bites. weapon empty = hand-to-hand.
    // Returns the chance (0..100) through chance when given.
    bool rollPhysicalHit(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, const MWWorld::Ptr& weapon,
        float* chance = nullptr);

    // Sends a qualified physical-weapon contact through the RPG weapon
    // pipeline (DMG-04). The approach speed at the struck point reads
    // Morrowind's band (chop, slash or thrust, by the physical motion): the
    // weapon class's full-swing speed is a vanilla full-strength attack, a
    // faster swing earns up to 15% more, below the minimum speed nothing.
    // A cut pays the band's maximum per full draw installment. Then vanilla
    // strength, condition, critical strike, resistances and armor. Skill
    // does not scale damage; the victim's concealment effects do.
    ///  regionScale is the hit-region multiplier: head and chest hits hurt
    /// more, limb hits less. It scales both the damage and the reaction.
    ///  nativeAttackStrength is only reported to Lua.
    std::optional<PhysicalWeaponDamageResult> applyPhysicalWeaponStrike(const MWWorld::Ptr& attacker,
        const MWWorld::Ptr& victim, MWWorld::Ptr weapon, const osg::Vec3f& hitPosition, float approachSpeed,
        float damageScale, RagdollCore::PhysicalDefenseOutcome defenseOutcome, float regionScale = 1.0f,
        std::uint64_t physicalHitId = 0, int physicalAttackType = -1, PhysicalBlowShare share = {},
        std::optional<float> nativeAttackStrength = std::nullopt);

    /// The same physical strike pipeline for an unarmed blow: the fist's
    /// approach speed is the hand-to-hand attack strength.
    std::optional<PhysicalWeaponDamageResult> applyPhysicalHandStrike(const MWWorld::Ptr& attacker,
        const MWWorld::Ptr& victim, const osg::Vec3f& hitPosition, float approachSpeed, float damageScale,
        RagdollCore::PhysicalDefenseOutcome defenseOutcome, float regionScale = 1.0f,
        std::uint64_t physicalHitId = 0, PhysicalBlowShare share = {});

    // Pure relationship query used before a physical contact is promoted to
    // combat. Unlike friendlyHit, this does not increment forgiveness state,
    // play dialogue, or mutate AI packages.
    bool isFriendlyPhysicalContact(const MWWorld::Ptr& attacker, const MWWorld::Ptr& target);

    // Testing aid for the manual ragdoll range. Morrowind decides whether a
    // melee attack connects with a dice roll against weapon skill, agility,
    // luck and fatigue, BEFORE any geometry is considered, so a swing that
    // visibly passes through a target can still be a miss and never reaches
    // the physics. That makes calibrating physical impacts by hand nearly
    // impossible. With this set, the PLAYER's melee attacks skip the roll
    // and connect whenever they are in reach and within the vanilla angle
    // tolerances; every other actor is unaffected and damage is unchanged.
    void setPlayerAlwaysHits(bool alwaysHits);
    bool getPlayerAlwaysHits();

    // Multiplies every melee ragdoll impulse. One is the calibrated default;
    // the manual range exposes this so the feel can be tuned live against the
    // measured bands (hit-reactions-2026-09-07.txt) without a rebuild. It
    // scales the push only and never damage.
    void setMeleeRagdollImpulseScale(float scale);
    float getMeleeRagdollImpulseScale();

    bool friendlyHit(const MWWorld::Ptr& attacker, const MWWorld::Ptr& target, bool complain);
}

#endif

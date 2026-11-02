
#include "combat.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <array>

#include <components/misc/rng.hpp>
#include <components/settings/values.hpp>

#include <ragdollcore/impactdamage.hpp>
#include <ragdollcore/tuning.hpp>

#include <components/sceneutil/positionattitudetransform.hpp>

#include <components/esm3/loadench.hpp>
#include <components/esm3/loadmgef.hpp>
#include <components/esm3/loadsoun.hpp>

#include "../mwbase/dialoguemanager.hpp"
#include "../mwbase/environment.hpp"
#include "../mwbase/luamanager.hpp"
#include "../mwbase/mechanicsmanager.hpp"
#include "../mwbase/soundmanager.hpp"
#include "../mwbase/windowmanager.hpp"
#include "../mwbase/world.hpp"

#include "../mwworld/class.hpp"
#include "../mwworld/esmstore.hpp"
#include "../mwworld/globals.hpp"
#include "../mwworld/inventorystore.hpp"
#include "weapontype.hpp"
#include "../mwworld/worldimp.hpp"
#include "../mwphysics/physicssystem.hpp"

#include "actorutil.hpp"
#include "difficultyscaling.hpp"
#include "disease.hpp"
#include "movement.hpp"
#include "npcstats.hpp"
#include "pathfinding.hpp"
#include "spellcasting.hpp"
#include "spellresistance.hpp"

namespace
{

    float signedAngleRadians(const osg::Vec3f& v1, const osg::Vec3f& v2, const osg::Vec3f& normal)
    {
        return std::atan2((normal * (v1 ^ v2)), (v1 * v2));
    }

}

namespace MWMechanics
{
    std::optional<RagdollCore::CombatState> getRagdollCombatState(const MWWorld::Ptr& actor)
    {
        if (actor.isEmpty() || !actor.getClass().isActor())
            return std::nullopt;
        auto* world = dynamic_cast<MWWorld::World*>(static_cast<MWBase::World*>(MWBase::Environment::get().getWorld()));
        if (world == nullptr)
            return std::nullopt;
        return world->getPhysicsSystem().getRagdollCombatState(actor);
    }

    std::optional<RagdollCore::DuelBodyView> getRagdollDuelView(const MWWorld::Ptr& actor)
    {
        if (actor.isEmpty() || !actor.getClass().isActor())
            return std::nullopt;
        auto* world = dynamic_cast<MWWorld::World*>(static_cast<MWBase::World*>(MWBase::Environment::get().getWorld()));
        if (world == nullptr)
            return std::nullopt;
        return world->getPhysicsSystem().getRagdollDuelView(actor);
    }

    void requestRagdollAiRiposte(const MWWorld::Ptr& actor)
    {
        if (actor.isEmpty() || !actor.getClass().isActor())
            return;
        auto* world = dynamic_cast<MWWorld::World*>(static_cast<MWBase::World*>(MWBase::Environment::get().getWorld()));
        if (world != nullptr)
            world->getPhysicsSystem().requestRagdollAiRiposte(actor);
    }

    void requestRagdollAiGuard(const MWWorld::Ptr& actor, float seconds)
    {
        if (actor.isEmpty() || !actor.getClass().isActor())
            return;
        auto* world = dynamic_cast<MWWorld::World*>(static_cast<MWBase::World*>(MWBase::Environment::get().getWorld()));
        if (world != nullptr)
            world->getPhysicsSystem().requestRagdollAiGuard(actor, seconds);
    }

    bool applyOnStrikeEnchantment(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, const MWWorld::Ptr& object,
        const osg::Vec3f& hitPosition, const bool fromProjectile)
    {
        const ESM::RefId enchantmentName = !object.isEmpty() ? object.getClass().getEnchantment(object) : ESM::RefId();
        if (!enchantmentName.empty())
        {
            const ESM::Enchantment* enchantment
                = MWBase::Environment::get().getESMStore()->get<ESM::Enchantment>().find(enchantmentName);
            if (enchantment->mData.mType == ESM::Enchantment::WhenStrikes)
            {
                MWMechanics::CastSpell cast(attacker, victim, fromProjectile);
                cast.mHitPosition = hitPosition;
                cast.cast(object, false);
                // Apply magic effects directly instead of waiting a frame to allow soul trap to work on one-hit kills
                if (!victim.isEmpty() && victim.getClass().isActor())
                    MWBase::Environment::get().getMechanicsManager()->updateMagicEffects(victim);
                return true;
            }
        }
        return false;
    }

    bool blockMeleeAttack(const MWWorld::Ptr& attacker, const MWWorld::Ptr& blocker, const MWWorld::Ptr& weapon,
        float damage, float attackStrength)
    {
        if (!blocker.getClass().hasInventoryStore(blocker))
            return false;

        MWMechanics::CreatureStats& blockerStats = blocker.getClass().getCreatureStats(blocker);

        if (blockerStats.getKnockedDown() // Used for both knockout or knockdown
            || blockerStats.getHitRecovery() || blockerStats.isParalyzed())
            return false;

        if (!MWBase::Environment::get().getMechanicsManager()->isReadyToBlock(blocker))
            return false;

        MWWorld::InventoryStore& inv = blocker.getClass().getInventoryStore(blocker);
        MWWorld::ContainerStoreIterator shield = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
        if (shield == inv.end() || shield->getType() != ESM::Armor::sRecordId)
            return false;

        if (!blocker.getRefData().getBaseNode())
            return false; // shouldn't happen

        float angleDegrees = osg::RadiansToDegrees(signedAngleRadians(
            (attacker.getRefData().getPosition().asVec3() - blocker.getRefData().getPosition().asVec3()),
            blocker.getRefData().getBaseNode()->getAttitude() * osg::Vec3f(0, 1, 0), osg::Vec3f(0, 0, 1)));

        const MWWorld::Store<ESM::GameSetting>& gmst
            = MWBase::Environment::get().getESMStore()->get<ESM::GameSetting>();
        static const float fCombatBlockLeftAngle = gmst.find("fCombatBlockLeftAngle")->mValue.getFloat();
        if (angleDegrees < fCombatBlockLeftAngle)
            return false;
        static const float fCombatBlockRightAngle = gmst.find("fCombatBlockRightAngle")->mValue.getFloat();
        if (angleDegrees > fCombatBlockRightAngle)
            return false;

        MWMechanics::CreatureStats& attackerStats = attacker.getClass().getCreatureStats(attacker);

        float blockTerm = blocker.getClass().getSkill(blocker, ESM::Skill::Block)
            + 0.2f * blockerStats.getAttribute(ESM::Attribute::Agility).getModified()
            + 0.1f * blockerStats.getAttribute(ESM::Attribute::Luck).getModified();
        float enemySwing = attackStrength;
        static const float fSwingBlockMult = gmst.find("fSwingBlockMult")->mValue.getFloat();
        static const float fSwingBlockBase = gmst.find("fSwingBlockBase")->mValue.getFloat();
        float swingTerm = enemySwing * fSwingBlockMult + fSwingBlockBase;

        float blockerTerm = blockTerm * swingTerm;
        if (blocker.getClass().getMovementSettings(blocker).mPosition[1] <= 0)
        {
            static const float fBlockStillBonus = gmst.find("fBlockStillBonus")->mValue.getFloat();
            blockerTerm *= fBlockStillBonus;
        }
        blockerTerm *= blockerStats.getFatigueTerm();

        float attackerSkill = 0;
        if (weapon.isEmpty())
            attackerSkill = attacker.getClass().getSkill(attacker, ESM::Skill::HandToHand);
        else
            attackerSkill = attacker.getClass().getSkill(attacker, weapon.getClass().getEquipmentSkill(weapon));
        float attackerTerm = attackerSkill + 0.2f * attackerStats.getAttribute(ESM::Attribute::Agility).getModified()
            + 0.1f * attackerStats.getAttribute(ESM::Attribute::Luck).getModified();
        attackerTerm *= attackerStats.getFatigueTerm();

        static const int iBlockMaxChance = gmst.find("iBlockMaxChance")->mValue.getInteger();
        static const int iBlockMinChance = gmst.find("iBlockMinChance")->mValue.getInteger();
        int x = std::clamp(static_cast<int>(blockerTerm - attackerTerm), iBlockMinChance, iBlockMaxChance);

        auto& prng = MWBase::Environment::get().getWorld()->getPrng();
        if (Misc::Rng::roll0to99(prng) < x)
        {
            MWBase::SoundManager* sndMgr = MWBase::Environment::get().getSoundManager();
            const ESM::RefId skill = shield->getClass().getEquipmentSkill(*shield);
            if (skill == ESM::Skill::LightArmor)
                sndMgr->playSound3D(blocker, ESM::RefId::stringRefId("Light Armor Hit"), 1.0f, 1.0f);
            else if (skill == ESM::Skill::MediumArmor)
                sndMgr->playSound3D(blocker, ESM::RefId::stringRefId("Medium Armor Hit"), 1.0f, 1.0f);
            else if (skill == ESM::Skill::HeavyArmor)
                sndMgr->playSound3D(blocker, ESM::RefId::stringRefId("Heavy Armor Hit"), 1.0f, 1.0f);

            // Reduce shield durability by incoming damage
            int shieldhealth = shield->getClass().getItemHealth(*shield);

            shieldhealth -= std::min(shieldhealth, int(damage));
            shield->getCellRef().setCharge(shieldhealth);
            if (shieldhealth == 0)
                inv.unequipItem(*shield);
            // Reduce blocker fatigue
            static const float fFatigueBlockBase = gmst.find("fFatigueBlockBase")->mValue.getFloat();
            static const float fFatigueBlockMult = gmst.find("fFatigueBlockMult")->mValue.getFloat();
            static const float fWeaponFatigueBlockMult = gmst.find("fWeaponFatigueBlockMult")->mValue.getFloat();
            MWMechanics::DynamicStat<float> fatigue = blockerStats.getFatigue();
            float normalizedEncumbrance = blocker.getClass().getNormalizedEncumbrance(blocker);
            normalizedEncumbrance = std::min(1.f, normalizedEncumbrance);
            float fatigueLoss = fFatigueBlockBase + normalizedEncumbrance * fFatigueBlockMult;
            if (!weapon.isEmpty())
                fatigueLoss += weapon.getClass().getWeight(weapon) * attackStrength * fWeaponFatigueBlockMult;
            fatigue.setCurrent(fatigue.getCurrent() - fatigueLoss);
            blockerStats.setFatigue(fatigue);

            blockerStats.setBlock(true);

            if (blocker == getPlayer())
                blocker.getClass().skillUsageSucceeded(blocker, ESM::Skill::Block, ESM::Skill::Block_Success);

            return true;
        }
        return false;
    }

    bool isNormalWeapon(const MWWorld::Ptr& weapon)
    {
        if (weapon.isEmpty())
            return false;

        const int flags = weapon.get<ESM::Weapon>()->mBase->mData.mFlags;
        bool isSilver = flags & ESM::Weapon::Silver;
        bool isMagical = flags & ESM::Weapon::Magical;
        bool isEnchanted = !weapon.getClass().getEnchantment(weapon).empty();

        return !isSilver && !isMagical && (!isEnchanted || !Settings::game().mEnchantedWeaponsAreMagical);
    }

    void resistNormalWeapon(
        const MWWorld::Ptr& actor, const MWWorld::Ptr& attacker, const MWWorld::Ptr& weapon, float& damage)
    {
        if (weapon.isEmpty() || !isNormalWeapon(weapon))
            return;

        const MWMechanics::MagicEffects& effects = actor.getClass().getCreatureStats(actor).getMagicEffects();
        const float resistance = effects.getOrDefault(ESM::MagicEffect::ResistNormalWeapons).getMagnitude() / 100.f;
        const float weakness = effects.getOrDefault(ESM::MagicEffect::WeaknessToNormalWeapons).getMagnitude() / 100.f;

        damage *= 1.f - std::min(1.f, resistance - weakness);

        if (resistance - weakness >= 1.f && attacker == getPlayer())
            MWBase::Environment::get().getWindowManager()->messageBox("#{sMagicTargetResistsWeapons}");
    }

    void applyWerewolfDamageMult(const MWWorld::Ptr& actor, const MWWorld::Ptr& weapon, float& damage)
    {
        if (damage == 0 || weapon.isEmpty() || !actor.getClass().isNpc())
            return;

        const int flags = weapon.get<ESM::Weapon>()->mBase->mData.mFlags;
        bool isSilver = flags & ESM::Weapon::Silver;

        if (isSilver && actor.getClass().getNpcStats(actor).isWerewolf())
        {
            const MWWorld::ESMStore& store = *MWBase::Environment::get().getESMStore();
            damage *= store.get<ESM::GameSetting>().find("fWereWolfSilverWeaponDamageMult")->mValue.getFloat();
        }
    }

    void projectileHit(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, MWWorld::Ptr weapon,
        const MWWorld::Ptr& projectile, const osg::Vec3f& hitPosition, float attackStrength, float attackWindUp)
    {
        MWBase::World* world = MWBase::Environment::get().getWorld();
        const MWWorld::Store<ESM::GameSetting>& gmst = world->getStore().get<ESM::GameSetting>();

        bool validVictim = !victim.isEmpty() && victim.getClass().isActor();

        ESM::RefId weaponSkill = ESM::Skill::Marksman;
        if (!weapon.isEmpty())
            weaponSkill = weapon.getClass().getEquipmentSkill(weapon);

        float damage = 0.f;
        if (validVictim)
        {
            if (attacker == getPlayer())
                MWBase::Environment::get().getWindowManager()->setEnemy(victim);

            int skillValue = static_cast<int>(attacker.getClass().getSkill(attacker, weaponSkill));

            if (Misc::Rng::roll0to99(world->getPrng()) >= getHitChance(attacker, victim, skillValue))
            {
                MWBase::Environment::get().getLuaManager()->onHit(attacker, victim, weapon, projectile, 0,
                    attackStrength, attackWindUp, damage, false, hitPosition, false,
                    MWMechanics::DamageSourceType::Ranged);
                MWMechanics::reduceWeaponCondition(damage, false, weapon, attacker);
                return;
            }

            {
                const auto& attack = weapon.get<ESM::Weapon>()->mBase->mData.mChop;
                damage = attack[0] + ((attack[1] - attack[0]) * attackStrength); // Bow/crossbow damage
            }
            {
                // Arrow/bolt damage
                // NB in case of thrown weapons, we are applying the damage twice since projectile == weapon
                const auto& attack = projectile.get<ESM::Weapon>()->mBase->mData.mChop;
                damage += attack[0] + ((attack[1] - attack[0]) * attackStrength);
            }
            adjustWeaponDamage(damage, weapon, attacker);
        }

        reduceWeaponCondition(damage, validVictim, weapon, attacker);

        if (validVictim)
        {
            if (weapon == projectile || Settings::game().mOnlyAppropriateAmmunitionBypassesResistance
                || isNormalWeapon(weapon))
                resistNormalWeapon(victim, attacker, projectile, damage);
            applyWerewolfDamageMult(victim, projectile, damage);

            if (attacker == getPlayer())
                attacker.getClass().skillUsageSucceeded(attacker, weaponSkill, ESM::Skill::Weapon_SuccessfulHit);

            const MWMechanics::AiSequence& sequence = victim.getClass().getCreatureStats(victim).getAiSequence();
            bool unaware = attacker == getPlayer() && !sequence.isInCombat()
                && !MWBase::Environment::get().getMechanicsManager()->awarenessCheck(attacker, victim);
            bool knockedDown = victim.getClass().getCreatureStats(victim).getKnockedDown();
            if (knockedDown || unaware)
            {
                static const float fCombatKODamageMult = gmst.find("fCombatKODamageMult")->mValue.getFloat();
                damage *= fCombatKODamageMult;
                if (!knockedDown)
                    MWBase::Environment::get().getSoundManager()->playSound3D(
                        victim, ESM::RefId::stringRefId("critical damage"), 1.0f, 1.0f);
            }
        }

        // Apply "On hit" effect of the projectile
        bool appliedEnchantment = applyOnStrikeEnchantment(attacker, victim, projectile, hitPosition, true);

        if (validVictim)
        {
            // Non-enchanted arrows shot at enemies have a chance to turn up in their inventory
            if (victim != getPlayer() && !appliedEnchantment)
            {
                static const float fProjectileThrownStoreChance
                    = gmst.find("fProjectileThrownStoreChance")->mValue.getFloat();
                if (Misc::Rng::rollProbability(world->getPrng()) < fProjectileThrownStoreChance / 100.f)
                {
                    // On point-blank hits the projectile is the attacker's live ammo stack -
                    // adding it directly would duplicate its RefNum into the victim's inventory
                    victim.getClass().getContainerStore(victim).add(projectile.getCellRef().getRefId(), 1);
                }
            }

            MWBase::Environment::get().getLuaManager()->onHit(attacker, victim, weapon, projectile, 0, attackStrength,
                attackWindUp, damage, true, hitPosition, true, MWMechanics::DamageSourceType::Ranged);
        }
    }

    float getHitChance(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, int skillValue)
    {
        MWMechanics::CreatureStats& stats = attacker.getClass().getCreatureStats(attacker);
        const MWMechanics::MagicEffects& mageffects = stats.getMagicEffects();

        MWBase::World* world = MWBase::Environment::get().getWorld();
        const MWWorld::Store<ESM::GameSetting>& gmst = world->getStore().get<ESM::GameSetting>();

        float defenseTerm = 0;
        MWMechanics::CreatureStats& victimStats = victim.getClass().getCreatureStats(victim);
        if (victimStats.getFatigue().getCurrent() >= 0)
        {
            // Maybe we should keep an aware state for actors updated every so often instead of testing every time
            bool unaware = (!victimStats.getAiSequence().isInCombat()) && (attacker == getPlayer())
                && (!MWBase::Environment::get().getMechanicsManager()->awarenessCheck(attacker, victim));
            if (!(victimStats.getKnockedDown() || victimStats.isParalyzed() || unaware))
            {
                defenseTerm = victimStats.getEvasion();
            }
            static const float fCombatInvisoMult = gmst.find("fCombatInvisoMult")->mValue.getFloat();
            defenseTerm += std::min(100.f,
                fCombatInvisoMult
                    * victimStats.getMagicEffects().getOrDefault(ESM::MagicEffect::Chameleon).getMagnitude());
            defenseTerm += std::min(100.f,
                fCombatInvisoMult
                    * victimStats.getMagicEffects().getOrDefault(ESM::MagicEffect::Invisibility).getMagnitude());
        }
        float attackTerm = skillValue + (stats.getAttribute(ESM::Attribute::Agility).getModified() / 5.0f)
            + (stats.getAttribute(ESM::Attribute::Luck).getModified() / 10.0f);
        attackTerm *= stats.getFatigueTerm();
        attackTerm += mageffects.getOrDefault(ESM::MagicEffect::FortifyAttack).getMagnitude()
            - mageffects.getOrDefault(ESM::MagicEffect::Blind).getMagnitude();

        return round(attackTerm - defenseTerm);
    }

    void applyElementalShields(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim)
    {
        // Don't let elemental shields harm the player in god mode.
        bool godmode = attacker == getPlayer() && MWBase::Environment::get().getWorld()->getGodModeState();
        if (godmode)
            return;
        auto& prng = MWBase::Environment::get().getWorld()->getPrng();
        static const std::array<ESM::RefId, 3> elementalShieldEffects{ ESM::MagicEffect::FireShield,
            ESM::MagicEffect::LightningShield, ESM::MagicEffect::FrostShield };
        for (const auto elementalShieldEffect : elementalShieldEffects)
        {
            float magnitude = victim.getClass()
                                  .getCreatureStats(victim)
                                  .getMagicEffects()
                                  .getOrDefault(elementalShieldEffect)
                                  .getMagnitude();

            if (!magnitude)
                continue;

            CreatureStats& attackerStats = attacker.getClass().getCreatureStats(attacker);
            float saveTerm = attacker.getClass().getSkill(attacker, ESM::Skill::Destruction)
                + 0.2f * attackerStats.getAttribute(ESM::Attribute::Willpower).getModified()
                + 0.1f * attackerStats.getAttribute(ESM::Attribute::Luck).getModified();

            float fatigueMax = attackerStats.getFatigue().getModified();
            float fatigueCurrent = attackerStats.getFatigue().getCurrent();

            float normalisedFatigue = floor(fatigueMax) == 0 ? 1 : std::max(0.0f, (fatigueCurrent / fatigueMax));

            saveTerm *= 1.25f * normalisedFatigue;

            float x = std::max(0.f, saveTerm - Misc::Rng::roll0to99(prng));

            ESM::RefId element = ESM::MagicEffect::FireDamage;
            if (elementalShieldEffect == ESM::MagicEffect::LightningShield)
                element = ESM::MagicEffect::ShockDamage;
            if (elementalShieldEffect == ESM::MagicEffect::FrostShield)
                element = ESM::MagicEffect::FrostDamage;

            float elementResistance
                = MWMechanics::getEffectResistanceAttribute(element, &attackerStats.getMagicEffects());

            x = std::min(100.f, x + elementResistance);

            static const float fElementalShieldMult = MWBase::Environment::get()
                                                          .getESMStore()
                                                          ->get<ESM::GameSetting>()
                                                          .find("fElementalShieldMult")
                                                          ->mValue.getFloat();
            x = fElementalShieldMult * magnitude * (1.f - 0.01f * x);

            // Note swapped victim and attacker, since the attacker takes the damage here.
            x = scaleDamage(x, victim, attacker);

            MWMechanics::DynamicStat<float> health = attackerStats.getHealth();
            health.setCurrent(health.getCurrent() - x);
            attackerStats.setHealth(health);

            MWBase::Environment::get().getSoundManager()->playSound3D(
                attacker, ESM::RefId::stringRefId("Health Damage"), 1.0f, 1.0f);
        }
    }

    void reduceWeaponCondition(float damage, bool hit, MWWorld::Ptr& weapon, const MWWorld::Ptr& attacker)
    {
        if (weapon.isEmpty())
            return;

        if (!hit)
            damage = 0.f;

        const bool weaphashealth = weapon.getClass().hasItemHealth(weapon);
        if (weaphashealth)
        {
            int weaphealth = weapon.getClass().getItemHealth(weapon);

            bool godmode
                = attacker == MWMechanics::getPlayer() && MWBase::Environment::get().getWorld()->getGodModeState();

            // weapon condition does not degrade when godmode is on
            if (!godmode)
            {
                const float fWeaponDamageMult = MWBase::Environment::get()
                                                    .getESMStore()
                                                    ->get<ESM::GameSetting>()
                                                    .find("fWeaponDamageMult")
                                                    ->mValue.getFloat();
                float x = std::max(1.f, fWeaponDamageMult * damage);

                weaphealth -= std::min(int(x), weaphealth);
                weapon.getCellRef().setCharge(weaphealth);
            }

            // Weapon broken? unequip it
            if (weaphealth == 0)
                weapon = *attacker.getClass().getInventoryStore(attacker).unequipItem(weapon);
        }
    }

    void adjustWeaponDamage(float& damage, const MWWorld::Ptr& weapon, const MWWorld::Ptr& attacker)
    {
        if (weapon.isEmpty())
            return;

        const bool weaphashealth = weapon.getClass().hasItemHealth(weapon);
        if (weaphashealth)
        {
            damage *= weapon.getClass().getItemNormalizedHealth(weapon);
        }

        static const float fDamageStrengthBase = MWBase::Environment::get()
                                                     .getESMStore()
                                                     ->get<ESM::GameSetting>()
                                                     .find("fDamageStrengthBase")
                                                     ->mValue.getFloat();
        static const float fDamageStrengthMult = MWBase::Environment::get()
                                                     .getESMStore()
                                                     ->get<ESM::GameSetting>()
                                                     .find("fDamageStrengthMult")
                                                     ->mValue.getFloat();
        damage *= fDamageStrengthBase
            + (attacker.getClass().getCreatureStats(attacker).getAttribute(ESM::Attribute::Strength).getModified()
                * fDamageStrengthMult * 0.1f);
    }

    void getHandToHandDamage(
        const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, float& damage, bool& healthdmg, float attackStrength)
    {
        const MWWorld::ESMStore& store = *MWBase::Environment::get().getESMStore();
        static const float minstrike = store.get<ESM::GameSetting>().find("fMinHandToHandMult")->mValue.getFloat();
        static const float maxstrike = store.get<ESM::GameSetting>().find("fMaxHandToHandMult")->mValue.getFloat();
        damage = static_cast<float>(attacker.getClass().getSkill(attacker, ESM::Skill::HandToHand));
        damage *= minstrike + ((maxstrike - minstrike) * attackStrength);

        MWMechanics::CreatureStats& otherstats = victim.getClass().getCreatureStats(victim);
        healthdmg = otherstats.isParalyzed() || otherstats.getKnockedDown();
        bool isWerewolf = (attacker.getClass().isNpc() && attacker.getClass().getNpcStats(attacker).isWerewolf());

        // Options in the launcher's combo box: unarmedFactorsStrengthComboBox
        // 0 = Do not factor strength into hand-to-hand combat.
        // 1 = Factor into werewolf hand-to-hand combat.
        // 2 = Ignore werewolves.
        const int factorStrength = Settings::game().mStrengthInfluencesHandToHand;
        if (factorStrength == 1 || (factorStrength == 2 && !isWerewolf))
        {
            damage
                *= attacker.getClass().getCreatureStats(attacker).getAttribute(ESM::Attribute::Strength).getModified()
                / 40.0f;
        }

        if (isWerewolf)
        {
            healthdmg = true;
            // GLOB instead of GMST because it gets updated during a quest
            damage *= MWBase::Environment::get().getWorld()->getGlobalFloat(MWWorld::Globals::sWerewolfClawMult);
        }
        if (healthdmg)
        {
            static const float fHandtoHandHealthPer
                = store.get<ESM::GameSetting>().find("fHandtoHandHealthPer")->mValue.getFloat();
            damage *= fHandtoHandHealthPer;
        }

        MWBase::SoundManager* sndMgr = MWBase::Environment::get().getSoundManager();
        auto& prng = MWBase::Environment::get().getWorld()->getPrng();
        if (isWerewolf)
        {
            const ESM::Sound* sound = store.get<ESM::Sound>().searchRandom("WolfHit", prng);
            if (sound)
                sndMgr->playSound3D(victim, sound->mId, 1.0f, 1.0f);
        }
        else if (!healthdmg)
        {
            static const std::array<ESM::RefId, 2> sounds
                = { ESM::RefId::stringRefId("Hand To Hand Hit"), ESM::RefId::stringRefId("Hand To Hand Hit 2") };
            sndMgr->playSound3D(victim, sounds[Misc::Rng::rollDice(sounds.size(), prng)], 1.0f, 1.0f);
        }
    }

    void applyFatigueLoss(const MWWorld::Ptr& attacker, const MWWorld::Ptr& weapon, float attackStrength)
    {
        // somewhat of a guess, but using the weapon weight makes sense
        const MWWorld::Store<ESM::GameSetting>& store
            = MWBase::Environment::get().getESMStore()->get<ESM::GameSetting>();
        static const float fFatigueAttackBase = store.find("fFatigueAttackBase")->mValue.getFloat();
        static const float fFatigueAttackMult = store.find("fFatigueAttackMult")->mValue.getFloat();
        static const float fWeaponFatigueMult = store.find("fWeaponFatigueMult")->mValue.getFloat();
        CreatureStats& stats = attacker.getClass().getCreatureStats(attacker);
        MWMechanics::DynamicStat<float> fatigue = stats.getFatigue();
        const float normalizedEncumbrance = attacker.getClass().getNormalizedEncumbrance(attacker);

        bool godmode = attacker == MWMechanics::getPlayer() && MWBase::Environment::get().getWorld()->getGodModeState();

        if (!godmode)
        {
            float fatigueLoss = fFatigueAttackBase + normalizedEncumbrance * fFatigueAttackMult;
            if (!weapon.isEmpty())
                fatigueLoss += weapon.getClass().getWeight(weapon) * attackStrength * fWeaponFatigueMult;
            fatigue.setCurrent(fatigue.getCurrent() - fatigueLoss);
            stats.setFatigue(fatigue);
        }
    }

    int getFightTerm(const MWWorld::Ptr& actor, const MWWorld::Ptr& target)
    {
        const auto mechanicsManager = MWBase::Environment::get().getMechanicsManager();
        int disposition = actor.getClass().isNpc() ? mechanicsManager->getDerivedDisposition(actor) : 50;
        int fight = actor.getClass().getCreatureStats(actor).getAiSetting(AiSetting::Fight).getModified()
            + static_cast<int>(
                getFightDistanceBias(actor, target) + getFightDispositionBias(static_cast<float>(disposition)));

        return fight;
    }

    float getFightDispositionBias(float disposition)
    {
        static const float fFightDispMult = MWBase::Environment::get()
                                                .getESMStore()
                                                ->get<ESM::GameSetting>()
                                                .find("fFightDispMult")
                                                ->mValue.getFloat();
        return ((50.f - disposition) * fFightDispMult);
    }

    float getFightDistanceBias(const MWWorld::Ptr& actor1, const MWWorld::Ptr& actor2)
    {
        osg::Vec3f pos1(actor1.getRefData().getPosition().asVec3());
        osg::Vec3f pos2(actor2.getRefData().getPosition().asVec3());

        float d = getAggroDistance(actor1, pos1, pos2);

        static const int iFightDistanceBase = MWBase::Environment::get()
                                                  .getESMStore()
                                                  ->get<ESM::GameSetting>()
                                                  .find("iFightDistanceBase")
                                                  ->mValue.getInteger();
        static const float fFightDistanceMultiplier = MWBase::Environment::get()
                                                          .getESMStore()
                                                          ->get<ESM::GameSetting>()
                                                          .find("fFightDistanceMultiplier")
                                                          ->mValue.getFloat();

        return (iFightDistanceBase - fFightDistanceMultiplier * d);
    }

    float getAggroDistance(const MWWorld::Ptr& actor, const osg::Vec3f& lhs, const osg::Vec3f& rhs)
    {
        if (canActorMoveByZAxis(actor))
            return distanceIgnoreZ(lhs, rhs);
        return distance(lhs, rhs);
    }

    bool isAggressionCapable(const MWWorld::Ptr& actor)
    {
        // Immobile creatures can enter combat, but are never aggressive
        if (!actor.getClass().isMobile(actor))
            return false;

        const MWWorld::Class& actorClass = actor.getClass();
        MagicEffects magEffects = actorClass.getCreatureStats(actor).getMagicEffects();

        // Actor isn't aggressive if a calm effect is active, as it would cause combat to cycle on/off
        // when combat is activated here and then canceled by the calm effect
        if ((actorClass.isNpc() && magEffects.getOrDefault(ESM::MagicEffect::CalmHumanoid).getMagnitude() > 0)
            || (!actorClass.isNpc() && magEffects.getOrDefault(ESM::MagicEffect::CalmCreature).getMagnitude() > 0))
            return false;

        return true;
    }

    bool isAggressive(const MWWorld::Ptr& actor, const MWWorld::Ptr& target)
    {
        // If already in combat with target, consider aggressive
        if (actor.getClass().getCreatureStats(actor).getAiSequence().isInCombat(target))
            return true;

        if (!isAggressionCapable(actor))
            return false;

        // Aggression is easier if the target is a werewolf
        int fight = getFightTerm(actor, target);
        if (actor.getClass().isNpc() && target.getClass().isNpc())
        {
            if (target.getClass().getNpcStats(target).isWerewolf()
                || (target == getPlayer()
                    && MWBase::Environment::get().getWorld()->getGlobalInt(MWWorld::Globals::sPCKnownWerewolf)))
            {
                const ESM::GameSetting* iWerewolfFightMod
                    = MWBase::Environment::get().getESMStore()->get<ESM::GameSetting>().find("iWerewolfFightMod");
                fight += iWerewolfFightMod->mValue.getInteger();
            }
        }

        return (fight >= 100);
    }

    float getDistanceToBounds(const MWWorld::Ptr& actor, const MWWorld::Ptr& target)
    {
        osg::Vec3f actorPos(actor.getRefData().getPosition().asVec3());
        osg::Vec3f targetPos(target.getRefData().getPosition().asVec3());
        MWBase::World* world = MWBase::Environment::get().getWorld();

        float dist = (targetPos - actorPos).length();
        dist -= world->getHalfExtents(actor).y();
        dist -= world->getHalfExtents(target).y();
        return dist;
    }

    float getMeleeWeaponReach(const MWWorld::Ptr& actor, const MWWorld::Ptr& weapon)
    {
        MWBase::World* world = MWBase::Environment::get().getWorld();
        const MWWorld::Store<ESM::GameSetting>& store = world->getStore().get<ESM::GameSetting>();
        const float fCombatDistance = store.find("fCombatDistance")->mValue.getFloat();
        const auto authoredReach = [&]() -> float {
            if (!weapon.isEmpty())
                return fCombatDistance * weapon.get<ESM::Weapon>()->mBase->mData.mReach;
            if (actor.getClass().isNpc())
                return fCombatDistance * store.find("fHandToHandReach")->mValue.getFloat();
            return fCombatDistance;
        };
        const float reach = authoredReach();
        // A powered ragdoll's blow is a physical one: it only counts when the
        // hand or weapon actually arrives, so the authored range is clamped to
        // the body's physical span.
        if (world->isPoweredRagdoll(actor))
            return std::min(reach, cRagdollMeleePhysicalReach);
        return reach;
    }

    bool isInMeleeReach(const MWWorld::Ptr& actor, const MWWorld::Ptr& target, const float reach)
    {
        const float heightDiff = actor.getRefData().getPosition().pos[2] - target.getRefData().getPosition().pos[2];
        return std::abs(heightDiff) < reach && getDistanceToBounds(actor, target) < reach;
    }

    namespace
    {
        float sMeleeRagdollImpulseScale = 1.0f;
        bool sPlayerAlwaysHits = false;
        // 72 since DMG-04 (2026-09-28): blows pay Morrowind's full band, so a
        // full claymore hit (~25 before strength) is ~2200: a stagger, and
        // only repeated or overwhelming hits reach the fall band.
        float sPhysicalReactionPerDamage = 72.0f;
        float sPhysicalHandDamageScale = 1.0f;
    }

    void setPhysicalReactionPerDamage(float value)
    {
        if (std::isfinite(value) && value >= 0.0f)
            sPhysicalReactionPerDamage = value;
    }

    float getPhysicalReactionPerDamage()
    {
        return sPhysicalReactionPerDamage;
    }

    void setPhysicalHandDamageScale(float value)
    {
        if (std::isfinite(value) && value >= 0.0f)
            sPhysicalHandDamageScale = value;
    }

    float getPhysicalHandDamageScale()
    {
        return sPhysicalHandDamageScale;
    }

    namespace
    {
        // Shared with applyMeleeRagdollImpulse so the push and the damage are
        // two readings of one physical quantity.
        constexpr float sBasePunchImpulse = 600.0f;
        constexpr float sStrengthPunchImpulse = 900.0f;
        constexpr float sWeaponReferenceWeight = 20.0f;
        constexpr float sWeaponMassFactorLimit = 1.5f;
        constexpr float sWeaponBaseImpulse = 400.0f;
        constexpr float sWeaponStrengthImpulse = 600.0f;

        float weaponMassFactor(const MWWorld::Ptr& weapon)
        {
            if (weapon.isEmpty())
                return 0.0f;
            const float weight = weapon.getClass().getWeight(weapon);
            if (!std::isfinite(weight) || weight <= 0.0f)
                return 0.0f;
            return std::min(weight / sWeaponReferenceWeight, sWeaponMassFactorLimit);
        }

        float meleeImpulseMagnitude(float massFactor, float strength)
        {
            return sBasePunchImpulse + sWeaponBaseImpulse * massFactor
                + (sStrengthPunchImpulse + sWeaponStrengthImpulse * massFactor) * strength;
        }
    }

    namespace
    {
        // Morrowind's attack term (getHitChance) without the victim's side:
        // how well the attacker handles the weapon (DMG-04).
        float attackTerm(const MWWorld::Ptr& attacker, int skillValue)
        {
            const MWMechanics::CreatureStats& stats = attacker.getClass().getCreatureStats(attacker);
            const MWMechanics::MagicEffects& effects = stats.getMagicEffects();
            float term = skillValue + stats.getAttribute(ESM::Attribute::Agility).getModified() / 5.0f
                + stats.getAttribute(ESM::Attribute::Luck).getModified() / 10.0f;
            term *= stats.getFatigueTerm();
            return term + effects.getOrDefault(ESM::MagicEffect::FortifyAttack).getMagnitude()
                - effects.getOrDefault(ESM::MagicEffect::Blind).getMagnitude();
        }

        // What the victim's Sanctuary, Chameleon and Invisibility would have
        // taken off the vanilla hit chance, as a damage scale: they are about
        // the victim being hard to strike, not about weapon control.
        float concealmentScale(const MWWorld::Ptr& victim, float term)
        {
            const auto& gmst = MWBase::Environment::get().getESMStore()->get<ESM::GameSetting>();
            static const float fCombatInvisoMult = gmst.find("fCombatInvisoMult")->mValue.getFloat();
            const MWMechanics::MagicEffects& effects
                = victim.getClass().getCreatureStats(victim).getMagicEffects();
            const float defense = effects.getOrDefault(ESM::MagicEffect::Sanctuary).getMagnitude()
                + std::min(100.0f,
                    fCombatInvisoMult * effects.getOrDefault(ESM::MagicEffect::Chameleon).getMagnitude())
                + std::min(100.0f,
                    fCombatInvisoMult * effects.getOrDefault(ESM::MagicEffect::Invisibility).getMagnitude());
            if (!(defense > 0.0f) || !std::isfinite(term))
                return 1.0f;
            return std::clamp((term - defense) / std::max(term, 1.0f), 0.0f, 1.0f);
        }

        int weaponSkillValue(const MWWorld::Ptr& attacker, const MWWorld::Ptr& weapon)
        {
            const ESM::RefId skill = weapon.isEmpty() || weapon.getType() != ESM::Weapon::sRecordId
                ? ESM::Skill::HandToHand
                : weapon.getClass().getEquipmentSkill(weapon);
            return static_cast<int>(attacker.getClass().getSkill(attacker, skill));
        }

        // DMG-04: the approach speed of a full swing of a weight-20 weapon (a
        // longsword), world units/s. OPENMW_RAGDOLL_FULL_SWING_SPEED.
        float fullSwingReferenceSpeed()
        {
            static const auto speed_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_FULL_SWING_SPEED");
                const float value = text ? static_cast<float>(std::atof(text)) : 320.0f;
                return std::isfinite(value) && value > 60.0f ? value : 320.0f;
            });
            static const auto& speed = speed_tuned.get();
            return speed;
        }

        // How a slower blow pays (user 10-30 bookmark: "lets make partial/slow
        // hits a bigger % so all hits tend to 100%, some partial hits that
        // connect play the same audio/blood but they do little damage"): the
        // speed's share of a full swing, raised to `exponent` (1 = straight;
        // 0.5: a third of full speed pays 58% of the band, half pays 71%).
        // OPENMW_RAGDOLL_QUALITY_CURVE = exponent.
        float curvedQuality(float quality)
        {
            static const auto exponent_tuned = RagdollCore::tuned<float>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_QUALITY_CURVE");
                const float value = text ? static_cast<float>(std::atof(text)) : 0.5f;
                return std::isfinite(value) && value > 0.05f ? std::min(value, 1.0f) : 0.5f;
            });
            return quality > 0.0f ? std::pow(std::clamp(quality, 0.0f, 1.0f), exponent_tuned.get()) : quality;
        }

        bool damageTrace()
        {
#ifdef OPENMW_RAGDOLL_DEV_API
            static const auto enabled_tuned = RagdollCore::tuned<bool>([] { return RagdollCore::tuningText("OPENMW_RAGDOLL_DAMAGE_TRACE") != nullptr
                || RagdollCore::tuningText("OPENMW_RAGDOLL_PLAYER_DAMAGE_TRACE") != nullptr; });
            static const auto& enabled = enabled_tuned.get();
            return enabled;
#else
            return false;
#endif
        }
    }

    float physicalWeaponControl(const MWWorld::Ptr& attacker, const MWWorld::Ptr& weapon)
    {
        if (attacker.isEmpty() || !attacker.getClass().isActor())
            return 1.0f;
        // The range's "always hits" is full control.
        if (sPlayerAlwaysHits && attacker == getPlayer())
            return 1.0f;
        return RagdollCore::weaponControl(attackTerm(attacker, weaponSkillValue(attacker, weapon)));
    }

    // How much of a blow shoves the whole body (the reaction banding and the
    // root kick), by weapon class (user bookmark 11, 2026-09-30: "pushes by
    // hits should be much more localized ... a sword swing won't send someone
    // flying, keep that for hammers maces and perhaps axes with sufficient
    // strength"). The struck part still takes the weapon's momentum.
    // OPENMW_RAGDOLL_HIT_PUSH = "blade,heavy" shares (blades and spears;
    // blunt and axes, times Strength / 50 within 0.6..1.5).
    // Vanilla's block chance (blockMeleeAttack) without the arc and the
    // shield requirement: the physical contact already put the guard there.
    float physicalBlockChance(const MWWorld::Ptr& blocker, const MWWorld::Ptr& attacker, const MWWorld::Ptr& weapon)
    {
        const MWWorld::Store<ESM::GameSetting>& gmst = MWBase::Environment::get().getESMStore()->get<ESM::GameSetting>();
        const MWMechanics::CreatureStats& blockerStats = blocker.getClass().getCreatureStats(blocker);
        const MWMechanics::CreatureStats& attackerStats = attacker.getClass().getCreatureStats(attacker);
        float blockTerm = blocker.getClass().getSkill(blocker, ESM::Skill::Block)
            + 0.2f * blockerStats.getAttribute(ESM::Attribute::Agility).getModified()
            + 0.1f * blockerStats.getAttribute(ESM::Attribute::Luck).getModified();
        static const float fSwingBlockMult = gmst.find("fSwingBlockMult")->mValue.getFloat();
        static const float fSwingBlockBase = gmst.find("fSwingBlockBase")->mValue.getFloat();
        blockTerm *= fSwingBlockMult + fSwingBlockBase; // a full swing
        blockTerm *= blockerStats.getFatigueTerm();
        float attackerTerm = static_cast<float>(weaponSkillValue(attacker, weapon))
            + 0.2f * attackerStats.getAttribute(ESM::Attribute::Agility).getModified()
            + 0.1f * attackerStats.getAttribute(ESM::Attribute::Luck).getModified();
        attackerTerm *= attackerStats.getFatigueTerm();
        static const int iBlockMaxChance = gmst.find("iBlockMaxChance")->mValue.getInteger();
        static const int iBlockMinChance = gmst.find("iBlockMinChance")->mValue.getInteger();
        return static_cast<float>(
            std::clamp(static_cast<int>(blockTerm - attackerTerm), iBlockMinChance, iBlockMaxChance));
    }

    float wholeBodyPushScale(const MWWorld::Ptr& attacker, const MWWorld::Ptr& weapon)
    {
        static const auto share_tuned = RagdollCore::tuned<std::array<float, 2>>([] {
            std::array<float, 2> value{ 0.35f, 1.0f };
            if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_HIT_PUSH"))
                std::sscanf(text, "%f,%f", &value[0], &value[1]);
            return value;
        });
        static const auto& share = share_tuned.get();
        if (weapon.isEmpty() || weapon.getType() != ESM::Weapon::sRecordId)
            return 1.0f;
        const ESM::RefId skill = weapon.getClass().getEquipmentSkill(weapon);
        if (skill == ESM::Skill::BluntWeapon || skill == ESM::Skill::Axe)
        {
            const float strength = attacker.isEmpty() || !attacker.getClass().isActor()
                ? 50.0f
                : attacker.getClass().getCreatureStats(attacker).getAttribute(ESM::Attribute::Strength).getModified();
            return share[1] * std::clamp(strength / 50.0f, 0.6f, 1.5f);
        }
        return share[0];
    }

    bool rollPhysicalHit(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, const MWWorld::Ptr& weapon,
        float* chance)
    {
        if (chance)
            *chance = 100.0f;
        if (attacker.isEmpty() || victim.isEmpty() || (sPlayerAlwaysHits && attacker == getPlayer()))
            return true;
        // Softened (user 2026-09-30: "softened roll"): a clean blow bites at
        // least OPENMW_RAGDOLL_HIT_FLOOR percent of the time, the rest scaled
        // from vanilla's chance (floor + (100 - floor) * chance / 100). Strict
        // vanilla (floor 0) left duel NPCs at ~6% and DUEL-A02 never ended.
        static const auto floor_tuned = RagdollCore::tuned<float>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_HIT_FLOOR");
            const float value = text ? static_cast<float>(std::atof(text)) : 35.0f;
            return std::isfinite(value) ? std::clamp(value, 0.0f, 100.0f) : 35.0f;
        });
        static const auto& floor = floor_tuned.get();
        const float vanilla = std::clamp(getHitChance(attacker, victim, weaponSkillValue(attacker, weapon)), 0.0f, 100.0f);
        const float hitChance = floor + (100.0f - floor) * vanilla / 100.0f;
        if (chance)
            *chance = hitChance;
        return Misc::Rng::roll0to99(MWBase::Environment::get().getWorld()->getPrng()) < hitChance;
    }

    float meleeSkillDamageMultiplier(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim, int skillValue)
    {
        // The manual range explicitly disables skill-based connection rolls
        // so geometry can be tested. Physical contacts must honor that same
        // override instead of converting the suppressed miss into near-zero
        // damage. Normal gameplay retains its skill multiplier.
        if (sPlayerAlwaysHits && attacker == getPlayer())
            return 1.0f;
        // getHitChance returns the percentage the vanilla roll would have had
        // to beat. Reusing it means the same skill, agility, luck, fatigue and
        // magic effects still decide the outcome; they decide how MUCH rather
        // than WHETHER. A hit chance at or above 100 leaves damage untouched.
        const float chance = getHitChance(attacker, victim, skillValue);
        if (!std::isfinite(chance))
            return 1.0f;
        return std::clamp(chance / 100.0f, 0.0f, 1.0f);
    }

    float meleeImpactMomentumFraction(const MWWorld::Ptr& weapon, float attackStrength)
    {
        const float massFactor = weaponMassFactor(weapon);
        const float strength = std::clamp(attackStrength, 0.0f, 1.0f);
        const float maximum = meleeImpulseMagnitude(massFactor, 1.0f);
        if (!(maximum > 0.0f))
            return strength;
        return std::clamp(meleeImpulseMagnitude(massFactor, strength) / maximum, 0.0f, 1.0f);
    }

    std::optional<PhysicalWeaponDamageResult> applyPhysicalWeaponStrike(const MWWorld::Ptr& attacker,
        const MWWorld::Ptr& victim, MWWorld::Ptr weapon, const osg::Vec3f& hitPosition, float approachSpeed,
        float damageScale, RagdollCore::PhysicalDefenseOutcome defenseOutcome, float regionScale,
        std::uint64_t physicalHitId, int physicalAttackType, PhysicalBlowShare share,
        std::optional<float> nativeAttackStrength)
    {
        if (attacker.isEmpty() || victim.isEmpty() || attacker == victim || weapon.isEmpty()
            || !attacker.getClass().isActor() || !victim.getClass().isActor()
            || weapon.getType() != ESM::Weapon::sRecordId || !hitPosition.valid() || !std::isfinite(approachSpeed)
            || !std::isfinite(damageScale) || damageScale < 0.0f || damageScale > 1.0f
            || !std::isfinite(share.mShare) || share.mShare <= 0.0f
            || defenseOutcome == RagdollCore::PhysicalDefenseOutcome::None)
            return std::nullopt;
        MWMechanics::CreatureStats& victimStats = victim.getClass().getCreatureStats(victim);
        if (victimStats.isDead())
            return std::nullopt;

        const ESM::Weapon::WPDTstruct& data = weapon.get<ESM::Weapon>()->mBase->mData;
        const std::array<const unsigned char*, 3> bands{ data.mChop.data(), data.mSlash.data(), data.mThrust.data() };
        const auto selected = std::max_element(bands.begin(), bands.end(), [](const auto* left, const auto* right) {
            return std::pair(left[1], left[0]) < std::pair(right[1], right[0]);
        });
        if (selected == bands.end())
            return std::nullopt;
        // The attack the body actually made decides which damage band the
        // contact prices: a stab uses thrust, a downward blow chop, a sweep
        // slash. Fall back to the strongest band when nothing committed.
        const unsigned char* band = *selected;
        const std::string_view attackType = attacker.getClass().getCreatureStats(attacker).getAttackType();
        if (attackType == "chop")
            band = data.mChop.data();
        else if (attackType == "slash")
            band = data.mSlash.data();
        else if (attackType == "thrust")
            band = data.mThrust.data();
        if (physicalAttackType == ESM::Weapon::AT_Thrust)
            band = data.mThrust.data();
        else if (physicalAttackType == ESM::Weapon::AT_Slash)
            band = data.mSlash.data();
        else if (physicalAttackType == ESM::Weapon::AT_Chop)
            band = data.mChop.data();

        // DMG-04: the swing's speed at the struck point reads Morrowind's
        // band - a full swing is a vanilla full-strength attack, a faster one
        // earns a small bonus. A cut is paid in installments of the band's
        // maximum per full draw, its speed already in the installment.
        const RagdollCore::SwingQualityProfile swing
            = RagdollCore::swingQualityProfile(data.mWeight, fullSwingReferenceSpeed());
        const float quality = share.mCut ? 1.0f : curvedQuality(RagdollCore::swingQuality(approachSpeed, swing));
        if (!share.mCut && approachSpeed < swing.mMinimumSpeed)
            return std::nullopt;
        const float bonus = share.mCut ? 1.0f : RagdollCore::fullSwingBonus(approachSpeed, swing);
        float damage = RagdollCore::morrowindBandDamage(band[0], band[1], quality) * bonus
            * std::min(share.mShare, 1.0f);
        if (!(damage > 0.0f))
            return std::nullopt;
        // The region multiplier scales the blow before the blunt factor: a
        // blade to the head hurts more, a handle to the arm less, and the
        // reaction reads the same region-scaled quantity.
        damage *= std::max(0.0f, regionScale);
        const float preBluntDamage = damage;
        damage *= damageScale;

        PhysicalWeaponDamageResult result;
        result.mImpactFraction = quality;
        result.mAuthoredDamage = damage;
        result.mLocalReactionImpulse = preBluntDamage * sPhysicalReactionPerDamage;
        result.mReactionImpulse = result.mLocalReactionImpulse * wholeBodyPushScale(attacker, weapon);
        result.mDefenseOutcome = defenseOutcome;
        adjustWeaponDamage(damage, weapon, attacker);
        reduceWeaponCondition(damage, true, weapon, attacker);
        resistNormalWeapon(victim, attacker, weapon, damage);
        applyWerewolfDamageMult(victim, weapon, damage);
        const ESM::RefId skill = weapon.getClass().getEquipmentSkill(weapon);
        // Skill no longer scales a blow that landed: it is weapon control
        // (physicalWeaponControl). Only the victim's concealment keeps its
        // vanilla share.
        if (!(sPlayerAlwaysHits && attacker == getPlayer()))
            damage *= concealmentScale(victim, attackTerm(attacker, weaponSkillValue(attacker, weapon)));

        MWBase::World* world = MWBase::Environment::get().getWorld();
        const MWWorld::Store<ESM::GameSetting>& store = world->getStore().get<ESM::GameSetting>();
        if (attacker == getPlayer())
        {
            MWBase::Environment::get().getWindowManager()->setEnemy(victim);
            attacker.getClass().skillUsageSucceeded(attacker, skill, ESM::Skill::Weapon_SuccessfulHit);
            const MWMechanics::AiSequence& sequence = victimStats.getAiSequence();
            const bool unaware = !sequence.isInCombat()
                && !MWBase::Environment::get().getMechanicsManager()->awarenessCheck(attacker, victim);
            if (unaware)
            {
                damage *= store.find("fCombatCriticalStrikeMult")->mValue.getFloat();
                MWBase::Environment::get().getWindowManager()->messageBox("#{sTargetCriticalStrike}");
            }
        }
        // No knockdown bonus on a physical blow: the reaction bands decide
        // whether a downed victim is finished, not a flat damage multiplier.

        applyOnStrikeEnchantment(attacker, victim, weapon, hitPosition);
        applyElementalShields(attacker, victim);
        result.mBlocked = defenseOutcome == RagdollCore::PhysicalDefenseOutcome::Block
            || defenseOutcome == RagdollCore::PhysicalDefenseOutcome::Parry;
        // Alpha 1.0 skill parity: the guard the blade struck holds by
        // vanilla's block roll (Block skill + Agility/5 + Luck/10, against the
        // attacker's weapon skill, both by fatigue, iBlockMin/MaxChance); a
        // failed roll is a beaten guard - half the blow goes through.
        // OPENMW_RAGDOLL_BLOCK_ROLL=1 (0 = every struck guard holds).
        static const auto blockRoll_tuned = RagdollCore::tuned<bool>([] {
            // On (vanilla parity, Alpha 1.0): duels x1 1 beaten guard of 3
            // struck guards, all duel and GUARD tests pass.
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_BLOCK_ROLL");
            return !(text && std::atoi(text) == 0);
        });
        static const auto& blockRoll = blockRoll_tuned.get();
        float blockedShare = 0.0f;
        if (result.mBlocked && blockRoll && !(sPlayerAlwaysHits && attacker == getPlayer()))
        {
            const float chance = physicalBlockChance(victim, attacker, weapon);
            if (Misc::Rng::roll0to99(world->getPrng()) >= chance)
            {
                blockedShare = 0.5f;
                if (damageTrace())
                    Log(Debug::Info) << "RAGDOLL_GUARD_BEATEN victim=" << victim.getCellRef().getRefId()
                                     << " chance=" << chance;
            }
        }
        // Weight against the guard (C1, build 59 bookmark 10: the staff
        // "blocking most of my attacks, but a chop with a giant claymore
        // should be slowed down by the staff but at least do some damage";
        // CLASH-01: a guarding body's block outcome stopped every blow
        // whatever struck it). The attacking weapon's weight (x1.25
        // two-handed) against what holds the guard - the shield (x2) or the
        // weapon (x1.4 two-handed): at `threshold` times or more, a held guard
        // still lets a quarter of the blow through per unit over it less one
        // (at most three quarters). OPENMW_RAGDOLL_CLASH_WEIGHT = "threshold,
        // push, stagger" (0 = off).
        if (result.mBlocked && blockedShare < 0.75f && !weapon.isEmpty() && weapon.getType() == ESM::Weapon::sRecordId)
        {
            static const auto threshold_tuned = RagdollCore::tuned<float>([] {
                float value = 0.0f;
                if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CLASH_WEIGHT"))
                    std::sscanf(text, "%f", &value);
                return value;
            });
            static const auto& threshold = threshold_tuned.get();
            if (threshold > 0.0f && victim.getClass().hasInventoryStore(victim))
            {
                const auto twoHanded = [](const MWWorld::ConstPtr& item) {
                    return item.getType() == ESM::Weapon::sRecordId
                        && MWMechanics::getWeaponType(item.get<ESM::Weapon>()->mBase->mData.mType)->mFlags
                            & ESM::WeaponType::TwoHanded;
                };
                const float attackWeight
                    = weapon.getClass().getWeight(weapon) * (twoHanded(weapon) ? 1.25f : 1.0f);
                MWWorld::InventoryStore& store = victim.getClass().getInventoryStore(victim);
                float hold = 3.0f; // bare arms
                const auto shield = store.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
                const auto guardWeapon = store.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
                if (shield != store.end() && shield->getType() == ESM::Armor::sRecordId)
                    hold = 2.0f * shield->getClass().getWeight(*shield);
                else if (guardWeapon != store.end())
                    hold = guardWeapon->getClass().getWeight(*guardWeapon) * (twoHanded(*guardWeapon) ? 1.4f : 1.0f);
                const float ratio = attackWeight / std::max(0.5f, hold);
                if (std::isfinite(ratio) && ratio >= threshold)
                {
                    blockedShare = std::max(blockedShare, std::clamp(0.25f * (ratio - threshold + 1.0f), 0.25f, 0.75f));
                    if (damageTrace())
                        Log(Debug::Info) << "RAGDOLL_GUARD_PUSHED victim=" << victim.getCellRef().getRefId()
                                         << " ratio=" << ratio << " share=" << blockedShare;
                }
            }
        }
        // A guard that takes a blow costs the blocker vanilla's block fatigue
        // (fFatigueBlockBase + encumbrance x fFatigueBlockMult + weapon weight
        // x fWeaponFatigueBlockMult). OPENMW_RAGDOLL_BLOCK_FATIGUE=1.
        static const auto blockFatigue_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_BLOCK_FATIGUE");
            return !(text && std::atoi(text) == 0); // on: duels and GUARD pass
        });
        static const auto& blockFatigue = blockFatigue_tuned.get();
        if (result.mBlocked && blockFatigue)
        {
            static const float fFatigueBlockBase = store.find("fFatigueBlockBase")->mValue.getFloat();
            static const float fFatigueBlockMult = store.find("fFatigueBlockMult")->mValue.getFloat();
            static const float fWeaponFatigueBlockMult = store.find("fWeaponFatigueBlockMult")->mValue.getFloat();
            const float encumbrance = std::min(1.0f, victim.getClass().getNormalizedEncumbrance(victim));
            const float loss = fFatigueBlockBase + encumbrance * fFatigueBlockMult
                + weapon.getClass().getWeight(weapon) * fWeaponFatigueBlockMult;
            MWMechanics::DynamicStat<float> fatigue = victimStats.getFatigue();
            fatigue.setCurrent(fatigue.getCurrent() - loss, true);
            victimStats.setFatigue(fatigue);
        }
        if ((result.mBlocked && !(blockedShare > 0.0f)) || (victim == getPlayer() && world->getGodModeState()))
            damage = 0.0f;
        else if (result.mBlocked)
            damage *= blockedShare;
        diseaseContact(victim, attacker);
        result.mFinalDamage = std::max(0.0f, damage);
        if (damageTrace())
            Log(Debug::Info) << "RAGDOLL_DAMAGE attacker=" << attacker.getCellRef().getRefId()
                             << " victim=" << victim.getCellRef().getRefId()
                             << " weapon=" << weapon.getCellRef().getRefId() << " type=" << data.mType
                             << " band=" << int(band[0]) << "-" << int(band[1]) << " speed=" << approachSpeed
                             << " quality=" << quality << " bonus=" << bonus << " share=" << share.mShare
                             << " cut=" << share.mCut << " region=" << regionScale << " scale=" << damageScale
                             << " authored=" << result.mAuthoredDamage << " final=" << result.mFinalDamage
                             << " reaction=" << result.mReactionImpulse
                             << " control=" << physicalWeaponControl(attacker, weapon);
        MWBase::Environment::get().getLuaManager()->onHit(attacker, victim, weapon, MWWorld::Ptr(),
            physicalAttackType >= 0 ? physicalAttackType : ESM::Weapon::AT_Slash, quality,
            nativeAttackStrength ? std::clamp(*nativeAttackStrength, 0.0f, 1.0f) : 0.0f, result.mFinalDamage, true,
            hitPosition, true, MWMechanics::DamageSourceType::Melee, physicalHitId);
        return result;
    }

    std::optional<PhysicalWeaponDamageResult> applyPhysicalHandStrike(const MWWorld::Ptr& attacker,
        const MWWorld::Ptr& victim, const osg::Vec3f& hitPosition, float approachSpeed, float damageScale,
        RagdollCore::PhysicalDefenseOutcome defenseOutcome, float regionScale, std::uint64_t physicalHitId,
        PhysicalBlowShare share)
    {
        if (attacker.isEmpty() || victim.isEmpty() || attacker == victim || !attacker.getClass().isActor()
            || !victim.getClass().isActor() || !hitPosition.valid() || !std::isfinite(approachSpeed)
            || !std::isfinite(damageScale) || damageScale < 0.0f || damageScale > 1.0f
            || !std::isfinite(share.mShare) || share.mShare <= 0.0f
            || defenseOutcome == RagdollCore::PhysicalDefenseOutcome::None)
            return std::nullopt;
        MWMechanics::CreatureStats& victimStats = victim.getClass().getCreatureStats(victim);
        if (victimStats.isDead())
            return std::nullopt;

        // DMG-04: the fist's speed at contact is Morrowind's attack strength.
        const RagdollCore::SwingQualityProfile swing
            = RagdollCore::swingQualityProfile(0.0f, fullSwingReferenceSpeed());
        if (approachSpeed < swing.mMinimumSpeed)
            return std::nullopt;
        const float quality = curvedQuality(RagdollCore::swingQuality(approachSpeed, swing));
        const float bonus = RagdollCore::fullSwingBonus(approachSpeed, swing);
        float damage = 0.0f;
        bool healthdmg = false;
        getHandToHandDamage(attacker, victim, damage, healthdmg, quality);
        damage *= bonus * std::min(share.mShare, 1.0f);
        if (!(damage > 0.0f))
            return std::nullopt;
        // No knockdown/paralysis hand-to-hand health multiplier on a physical
        // blow either: the reaction bands decide how a downed victim is
        // finished, not a flat damage multiplier.
        if (healthdmg)
        {
            const float healthPer = MWBase::Environment::get()
                                        .getESMStore()
                                        ->get<ESM::GameSetting>()
                                        .find("fHandToHandHealthPer")
                                        ->mValue.getFloat();
            if (std::isfinite(healthPer) && healthPer > 0.0f)
                damage /= healthPer;
        }
        damage *= sPhysicalHandDamageScale;
        damage *= std::max(0.0f, regionScale);
        const float preBluntDamage = damage;
        damage *= damageScale;
        if (!(sPlayerAlwaysHits && attacker == getPlayer()))
            damage *= concealmentScale(victim, attackTerm(attacker, weaponSkillValue(attacker, MWWorld::Ptr())));

        MWBase::World* world = MWBase::Environment::get().getWorld();
        const MWWorld::Store<ESM::GameSetting>& store = world->getStore().get<ESM::GameSetting>();
        if (attacker == getPlayer())
        {
            MWBase::Environment::get().getWindowManager()->setEnemy(victim);
            attacker.getClass().skillUsageSucceeded(attacker, ESM::Skill::HandToHand, ESM::Skill::Weapon_SuccessfulHit);
            const MWMechanics::AiSequence& sequence = victimStats.getAiSequence();
            const bool unaware = !sequence.isInCombat()
                && !MWBase::Environment::get().getMechanicsManager()->awarenessCheck(attacker, victim);
            if (unaware)
            {
                damage *= store.find("fCombatCriticalStrikeMult")->mValue.getFloat();
                MWBase::Environment::get().getWindowManager()->messageBox("#{sTargetCriticalStrike}");
            }
        }
        // No knockdown bonus on a physical blow: the reaction bands decide
        // whether a downed victim is finished, not a flat damage multiplier.

        applyElementalShields(attacker, victim);
        PhysicalWeaponDamageResult result;
        result.mImpactFraction = quality;
        result.mAuthoredDamage = damage;
        result.mReactionImpulse = preBluntDamage * sPhysicalReactionPerDamage;
        result.mLocalReactionImpulse = result.mReactionImpulse;
        result.mDefenseOutcome = defenseOutcome;
        result.mBlocked = defenseOutcome == RagdollCore::PhysicalDefenseOutcome::Block
            || defenseOutcome == RagdollCore::PhysicalDefenseOutcome::Parry;
        if (result.mBlocked || (victim == getPlayer() && world->getGodModeState()))
            damage = 0.0f;
        diseaseContact(victim, attacker);
        result.mFinalDamage = std::max(0.0f, damage);
        if (damageTrace())
            Log(Debug::Info) << "RAGDOLL_DAMAGE attacker=" << attacker.getCellRef().getRefId()
                             << " victim=" << victim.getCellRef().getRefId() << " weapon=fists type=-1"
                             << " health=" << healthdmg << " speed=" << approachSpeed << " quality=" << quality
                             << " bonus=" << bonus << " share=" << share.mShare << " region=" << regionScale
                             << " scale=" << damageScale << " authored=" << result.mAuthoredDamage
                             << " final=" << result.mFinalDamage << " reaction=" << result.mReactionImpulse
                             << " control=" << physicalWeaponControl(attacker, MWWorld::Ptr());
        // Morrowind's hand-to-hand smack for a blow on fatigue, as its own
        // attack code plays it (the combat script sounds only health damage,
        // so every physical fist blow on fatigue was silent; user 10-31:
        // "very small hits do not trigger sounds"). The physical hit sounds
        // (OPENMW_RAGDOLL_HIT_SOUNDS) play their own.
        static const auto physicalSounds_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_HIT_SOUNDS");
            return text && std::atoi(text) != 0;
        });
        if (!healthdmg && !result.mBlocked && result.mFinalDamage > 0.0f && !physicalSounds_tuned.get())
        {
            static const std::array<ESM::RefId, 2> sounds
                = { ESM::RefId::stringRefId("Hand To Hand Hit"), ESM::RefId::stringRefId("Hand To Hand Hit 2") };
            MWBase::Environment::get().getSoundManager()->playSound3D(
                victim, sounds[Misc::Rng::rollDice(sounds.size(), world->getPrng())], 1.0f, 1.0f);
        }
        MWBase::Environment::get().getLuaManager()->onHit(attacker, victim, MWWorld::Ptr(), MWWorld::Ptr(),
            ESM::Weapon::AT_Slash, quality, 0.0f, result.mFinalDamage, healthdmg, hitPosition, true,
            MWMechanics::DamageSourceType::Melee, physicalHitId);
        return result;
    }

    bool isFriendlyPhysicalContact(const MWWorld::Ptr& attacker, const MWWorld::Ptr& target)
    {
        if (attacker.isEmpty() || target.isEmpty() || attacker == target)
            return false;
        const auto allies = MWBase::Environment::get().getMechanicsManager()->getActorsSidingWith(attacker);
        return std::find(allies.begin(), allies.end(), target) != allies.end();
    }

    void setPlayerAlwaysHits(bool alwaysHits)
    {
        sPlayerAlwaysHits = alwaysHits;
    }

    bool getPlayerAlwaysHits()
    {
        return sPlayerAlwaysHits;
    }

    void setMeleeRagdollImpulseScale(float scale)
    {
        if (std::isfinite(scale) && scale >= 0.0f)
            sMeleeRagdollImpulseScale = scale;
    }

    float getMeleeRagdollImpulseScale()
    {
        return sMeleeRagdollImpulseScale;
    }

    bool applyMeleeRagdollImpulse(const MWWorld::Ptr& attacker, const MWWorld::Ptr& victim,
        const MWWorld::Ptr& weapon, const osg::Vec3f& hitPosition, float attackStrength, bool blocked)
    {
        auto* world = dynamic_cast<MWWorld::World*>(static_cast<MWBase::World*>(MWBase::Environment::get().getWorld()));
        if (world == nullptr || victim.isEmpty() || attacker.isEmpty())
            return false;
        MWPhysics::PhysicsSystem& physics = world->getPhysicsSystem();
        if (!physics.hasPassiveRagdoll(victim))
            return false;
        // Impulse units are calibrated against the measured reaction bands
        // on a walking target (hit-reactions-2026-09-07.txt): up to about
        // 1250 is absorbed, 1250-1750 staggers, beyond that falls are common.
        // Hand-to-hand spans 600-1500, so a jab is absorbed and a full-
        // strength punch reaches the stagger band.
        // A weapon adds its mass to the blow, so a dagger lands near
        // hand-to-hand and a warhammer reaches the fall band at full
        // strength: light weapon about 700-1750, heavy about 1200-3000.
        // Weapon damage reads the same quantity through
        // meleeImpactMomentumFraction, so the push and the damage cannot
        // drift apart.
        const float strength = std::clamp(attackStrength, 0.0f, 1.0f);
        float magnitude = meleeImpulseMagnitude(weaponMassFactor(weapon), strength);
        if (blocked)
            magnitude *= 0.5f;
        magnitude *= sMeleeRagdollImpulseScale;
        const osg::Vec3f victimPos(victim.getRefData().getPosition().asVec3());
        const osg::Vec3f attackerPos(attacker.getRefData().getPosition().asVec3());
        const ESM::Position& posdata = attacker.getRefData().getPosition();
        // Where the attacker is looking: yaw for everyone, pitch as well for
        // the player, who is the only one that can look up or down.
        osg::Vec3f aim = osg::Quat(posdata.rot[2], osg::Vec3(0, 0, -1))
            * (osg::Quat(posdata.rot[0], osg::Vec3(-1, 0, 0)) * osg::Vec3f(0, 1, 0));
        const bool aimed = aim.valid() && aim.length2() > 1e-6f;
        if (aimed)
            aim.normalize();
        // The vanilla hit point is a random spot on a plane in front of the
        // target, chosen for the blood effect. Physically it means a punch
        // lands somewhere the attacker did not aim at. Prefer the point on
        // the aim ray beside the ragdoll body it passes closest to, so a
        // high punch turns the head and a low one folds the body. The eye
        // level matches the one getHitContact uses for its angle checks.
        std::optional<osg::Vec3f> aimPoint;
        if (aimed)
        {
            const float eyeLevel = world->getHalfExtents(attacker, true).z() * 2.f * 0.85f;
            const osg::Vec3f eye{ attackerPos.x(), attackerPos.y(), attackerPos.z() + eyeLevel };
            aimPoint = physics.pickPassiveRagdollAimPoint(victim, eye, aim, 40.0f);
        }
        osg::Vec3f direction = aimPoint ? aim : victimPos - attackerPos;
        direction.z() = 0.0f;
        if (direction.length2() < 1e-6f)
        {
            direction = osg::Quat(posdata.rot[2], osg::Vec3(0, 0, -1)) * osg::Vec3f(0, 1, 0);
            direction.z() = 0.0f;
        }
        direction.normalize();
        // A little lift reads as a punch rather than a shove.
        direction.z() = 0.2f;
        direction.normalize();
        osg::Vec3f point = aimPoint.value_or(hitPosition);
        if (!point.valid() || point.length2() < 1e-6f)
            point = victimPos + osg::Vec3f(0.0f, 0.0f, world->getHalfExtents(victim, true).z());
        return physics.applyCombatReactionAtPoint(
            victim, physics.nextCombatReactionId(), direction * magnitude, point);
    }

    std::pair<MWWorld::Ptr, osg::Vec3f> getHitContact(const MWWorld::Ptr& actor, float reach)
    {
        // Lasciate ogne speranza, voi ch'entrate
        MWWorld::Ptr result;
        osg::Vec3f hitPos;
        float minDist = std::numeric_limits<float>::max();
        MWBase::World* world = MWBase::Environment::get().getWorld();
        const MWWorld::Store<ESM::GameSetting>& store = world->getStore().get<ESM::GameSetting>();

        // These GMSTs are not in degrees. They're tolerance angle sines multiplied by 90.
        // With the default values of 60, the actual tolerance angles are roughly 41.8 degrees.
        // Don't think too hard about it. In this place, thinking can cause permanent damage to your mental health.
        const float fCombatAngleXY = store.find("fCombatAngleXY")->mValue.getFloat() / 90.f;
        const float fCombatAngleZ = store.find("fCombatAngleZ")->mValue.getFloat() / 90.f;

        const ESM::Position& posdata = actor.getRefData().getPosition();
        const osg::Vec3f actorPos(posdata.asVec3());
        const osg::Vec3f actorDirXY = osg::Quat(posdata.rot[2], osg::Vec3(0, 0, -1)) * osg::Vec3f(0, 1, 0);
        // Only the player can look up, apparently.
        const float actorVerticalAngle = actor == getPlayer() ? -std::sin(posdata.rot[0]) : 0.f;
        const float actorEyeLevel = world->getHalfExtents(actor, true).z() * 2.f * 0.85f;
        const osg::Vec3f actorEyePos{ actorPos.x(), actorPos.y(), actorPos.z() + actorEyeLevel };
        const bool canMoveByZ = canActorMoveByZAxis(actor);

        // The player can target any active actor, non-playable actors only target their targets
        std::vector<MWWorld::Ptr> targets;
        if (actor != getPlayer())
            actor.getClass().getCreatureStats(actor).getAiSequence().getCombatTargets(targets);
        else
            MWBase::Environment::get().getMechanicsManager()->getActorsInRange(
                actorPos, static_cast<float>(Settings::game().mActorsProcessingRange), targets);

        static const auto meleeTrace_tuned = RagdollCore::tuned<bool>([] { return RagdollCore::tuningText("OPENMW_RAGDOLL_MELEE_TRACE") != nullptr; });
        static const auto& meleeTrace = meleeTrace_tuned.get();
        static unsigned meleeTraceRows = 0;
        const bool traceThisAttacker = meleeTrace && actor != getPlayer() && meleeTraceRows < 600;
        for (MWWorld::Ptr& target : targets)
        {
            if (actor == target || target.getClass().getCreatureStats(target).isDead())
                continue;

            const float dist = getDistanceToBounds(actor, target);
            if (traceThisAttacker)
            {
                ++meleeTraceRows;
                Log(Debug::Info) << std::dec << std::setprecision(5) << "RAGDOLL_MELEE schema=1"
                    << " attacker=" << actor.getCellRef().getRefId()
                    << " target=" << target.getCellRef().getRefId() << " dist=" << dist
                    << " reach=" << reach << " inMeleeReach=" << isInMeleeReach(actor, target, reach);
            }
            if (dist >= minDist || !isInMeleeReach(actor, target, reach))
                continue;

            const osg::Vec3f targetPos(target.getRefData().getPosition().asVec3());

            // Horizontal angle checks.
            osg::Vec2f actorToTargetXY{ targetPos.x() - actorPos.x(), targetPos.y() - actorPos.y() };
            actorToTargetXY.normalize();

            // Use dot product to check if the target is behind first...
            if (actorToTargetXY.x() * actorDirXY.x() + actorToTargetXY.y() * actorDirXY.y() <= 0.f)
                continue;

            // And then perp dot product to calculate the hit angle sine.
            // This gives us a horizontal hit range of [-asin(fCombatAngleXY / 90); asin(fCombatAngleXY / 90)]
            if (std::abs(actorToTargetXY.x() * actorDirXY.y() - actorToTargetXY.y() * actorDirXY.x()) > fCombatAngleXY)
                continue;

            // Vertical angle checks. Nice cliff racer hack, Todd.
            if (!canMoveByZ)
            {
                // The idea is that the body should always be possible to hit.
                // fCombatAngleZ is the tolerance for hitting the target's feet or head.
                osg::Vec3f actorToTargetFeet = targetPos - actorEyePos;
                osg::Vec3f actorToTargetHead = actorToTargetFeet;
                actorToTargetFeet.normalize();
                actorToTargetHead.z() += world->getHalfExtents(target, true).z() * 2.f;
                actorToTargetHead.normalize();

                if (actorVerticalAngle - actorToTargetHead.z() > fCombatAngleZ
                    || actorVerticalAngle - actorToTargetFeet.z() < -fCombatAngleZ)
                    continue;
            }

            // Gotta use physics somehow!
            if (!world->getLOS(actor, target))
                continue;

            minDist = dist;
            result = target;
        }
        if (traceThisAttacker)
        {
            Log(Debug::Info) << "RAGDOLL_MELEE_RESULT schema=1"
                << " attacker=" << actor.getCellRef().getRefId()
                << " targets=" << targets.size()
                << " hit=" << (!result.isEmpty() ? result.getCellRef().getRefId().serializeText()
                                                 : std::string("none"));
        }

        // This hit position is currently used for spawning the blood effect.
        // Morrowind does this elsewhere, but roughly at the same time
        // and it would be hard to track the original hit results outside of this function
        // without code duplication
        // The idea is to use a random point on a plane in front of the target
        // that is defined by its width and height
        if (!result.isEmpty())
        {
            osg::Vec3f resultPos(result.getRefData().getPosition().asVec3());
            osg::Vec3f dirToActor = actorPos - resultPos;
            dirToActor.normalize();

            hitPos = resultPos + dirToActor * world->getHalfExtents(result).y();
            // -25% to 25% of width
            float xOffset = Misc::Rng::deviate(0.f, 0.25f, world->getPrng());
            // 20% to 100% of height
            float zOffset = Misc::Rng::deviate(0.6f, 0.4f, world->getPrng());
            hitPos.x() += world->getHalfExtents(result).x() * 2.f * xOffset;
            hitPos.z() += world->getHalfExtents(result).z() * 2.f * zOffset;
        }

        return std::make_pair(result, hitPos);
    }

    bool friendlyHit(const MWWorld::Ptr& attacker, const MWWorld::Ptr& target, bool complain)
    {
        const MWWorld::Ptr& player = getPlayer();
        if (attacker != player)
            return false;

        std::set<MWWorld::Ptr> followersAttacker;
        MWBase::Environment::get().getMechanicsManager()->getActorsSidingWith(attacker, followersAttacker);
        if (followersAttacker.find(target) == followersAttacker.end())
            return false;

        MWMechanics::CreatureStats& statsTarget = target.getClass().getCreatureStats(target);
        if (statsTarget.getAiSequence().isInCombat())
            return true;
        statsTarget.friendlyHit();
        if (statsTarget.getFriendlyHits() >= 4)
            return false;

        if (complain)
            MWBase::Environment::get().getDialogueManager()->say(target, ESM::RefId::stringRefId("hit"));
        return true;
    }

}

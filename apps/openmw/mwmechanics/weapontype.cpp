#include "weapontype.hpp"

#include "creaturestats.hpp"
#include "drawstate.hpp"

#include "../mwbase/environment.hpp"
#include "../mwworld/class.hpp"
#include "../mwworld/esmstore.hpp"
#include "../mwworld/inventorystore.hpp"

#include <components/esm3/loadweap.hpp>

#include <set>

namespace MWMechanics
{
    MWWorld::ContainerStoreIterator getActiveWeapon(const MWWorld::Ptr& actor, ESM::RefId* weaponTypeId)
    {
        MWWorld::InventoryStore& inv = actor.getClass().getInventoryStore(actor);
        CreatureStats& stats = actor.getClass().getCreatureStats(actor);
        if (stats.getDrawState() == MWMechanics::DrawState::Spell)
        {
            *weaponTypeId = ESM::WeaponType::Spell;
            return inv.end();
        }

        if (stats.getDrawState() == MWMechanics::DrawState::Weapon)
        {
            MWWorld::ContainerStoreIterator weapon = inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
            if (weapon == inv.end())
                *weaponTypeId = ESM::WeaponType::HandToHand;
            else
            {
                auto type = weapon->getType();
                if (type == ESM::Weapon::sRecordId)
                {
                    const MWWorld::LiveCellRef<ESM::Weapon>* ref = weapon->get<ESM::Weapon>();
                    *weaponTypeId = ref->mBase->mData.mType;
                }
                else if (type == ESM::Lockpick::sRecordId || type == ESM::Probe::sRecordId)
                    *weaponTypeId = ESM::WeaponType::PickProbe;
            }

            return weapon;
        }

        return inv.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
    }

    const ESM::WeaponType* getWeaponType(ESM::RefId weaponTypeId)
    {
        const auto& weaponTypes = MWBase::Environment::get().getESMStore()->get<ESM::WeaponType>();
        if (const ESM::WeaponType* result = weaponTypes.search(weaponTypeId))
            return result;
        return weaponTypes.find(ESM::WeaponType::ShortBladeOneHand);
    }

    bool isWeaponType(ESM::RefId weaponTypeId)
    {
        return !weaponTypeId.empty() && weaponTypeId != ESM::WeaponType::PickProbe
            && weaponTypeId != ESM::WeaponType::HandToHand && weaponTypeId != ESM::WeaponType::Spell;
    }

    WeaponAttackCandidates weaponAttackCandidates(const ESM::Weapon* weapon)
    {
        if (weapon == nullptr)
            return { true, false, true }; // unarmed: hooks and straights
        const ESM::RefId& type = weapon->mData.mType;
        if (type == ESM::WeaponType::ShortBladeOneHand)
            return { true, false, true };
        if (type == ESM::WeaponType::AxeOneHand || type == ESM::WeaponType::AxeTwoHand)
            return { true, true, false };
        if (type == ESM::WeaponType::BluntOneHand || type == ESM::WeaponType::BluntTwoClose
            || type == ESM::WeaponType::BluntTwoWide)
            return { false, true, true };
        if (type == ESM::WeaponType::SpearTwoWide)
            return { false, false, true };
        // Long blades and anything unrecognised keep the authored three.
        return {};
    }

    bool isWeaponOrToolType(ESM::RefId weaponTypeId)
    {
        return isWeaponType(weaponTypeId) || weaponTypeId == ESM::WeaponType::PickProbe;
    }

    std::vector<std::string_view> getAllWeaponTypeShortGroups()
    {
        // Go via a set to eliminate duplicates.
        std::set<std::string_view> shortGroupSet;
        for (const ESM::WeaponType& type : MWBase::Environment::get().getESMStore()->get<ESM::WeaponType>())
        {
            std::string_view shortGroup = type.mShortGroup;
            if (!shortGroup.empty())
                shortGroupSet.insert(shortGroup);
        }

        return std::vector<std::string_view>(shortGroupSet.begin(), shortGroupSet.end());
    }
}

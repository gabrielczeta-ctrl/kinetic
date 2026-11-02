#ifndef GAME_MWMECHANICS_WEAPONTYPE_H
#define GAME_MWMECHANICS_WEAPONTYPE_H

#include <string_view>
#include <vector>

#include <components/esm/refid.hpp>

namespace ESM
{
    struct Weapon;
    struct WeaponType;
}

namespace MWWorld
{
    class Ptr;

    template <class PtrType>
    class ContainerStoreIteratorBase;

    using ContainerStoreIterator = ContainerStoreIteratorBase<Ptr>;
}

namespace MWMechanics
{
    MWWorld::ContainerStoreIterator getActiveWeapon(const MWWorld::Ptr& actor, ESM::RefId* weaponTypeId);

    const ESM::WeaponType* getWeaponType(ESM::RefId weaponTypeId);

    bool isWeaponType(ESM::RefId weaponTypeId);

    /// Authored attack types a weapon class can actually perform. Daggers and
    /// short blades slash and stab, long blades keep chop, axes chop and
    /// slash, blunt weapons chop or jab and spears thrust. Enforced wherever
    /// an attack type is chosen so a dagger never plays a chop.
    struct WeaponAttackCandidates
    {
        bool mSlash = true;
        bool mChop = true;
        bool mThrust = true;
    };

    WeaponAttackCandidates weaponAttackCandidates(const ESM::Weapon* weapon);

    bool isWeaponOrToolType(ESM::RefId weaponTypeId);

    std::vector<std::string_view> getAllWeaponTypeShortGroups();
}

#endif

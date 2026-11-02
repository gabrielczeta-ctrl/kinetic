#ifndef GAME_RENDER_NPCANIMATION_H
#define GAME_RENDER_NPCANIMATION_H

#include "actoranimation.hpp"
#include "animation.hpp"
#include "weaponanimation.hpp"

#include <components/vfs/pathutil.hpp>

#include "../mwworld/inventorystore.hpp"

#include <osg/MatrixTransform>
#include <osg/observer_ptr>

#include <array>
#include <cstdint>
#include <atomic>
#include <memory>
#include <mutex>

namespace ESM
{
    struct NPC;
    struct BodyPart;
}

namespace MWSound
{
    class Sound;
}

namespace MWRender
{

    class RotateController;
    class HeadAnimationTime;
    class WoundFxSet;

    class NpcAnimation : public ActorAnimation, public WeaponAnimation, public MWWorld::InventoryStoreListener
    {
    public:
        void equipmentChanged() override;

    public:
        typedef std::map<ESM::PartReferenceType, std::string> PartBoneMap;

        enum ViewMode
        {
            VM_Normal,
            VM_FirstPerson,
            VM_HeadOnly
        };

    private:
        static const PartBoneMap sPartList;

        // Bounded Parts
        PartHolderPtr mObjectParts[ESM::PRT_Count];
        std::array<MWSound::Sound*, ESM::PRT_Count> mSounds;

        const ESM::NPC* mNpc;
        VFS::Path::Normalized mHeadModel;
        VFS::Path::Normalized mHairModel;
        ViewMode mViewMode;
        bool mShowWeapons;
        bool mShowCarriedLeft;

        enum NpcType
        {
            Type_Normal,
            Type_Werewolf,
            Type_Vampire
        };
        NpcType mNpcType;

        int mPartslots[ESM::PRT_Count]; // Each part slot is taken by clothing, armor, or is empty
        int mPartPriorities[ESM::PRT_Count];

        // Development aid for localized limb damage: how red each body part
        // is drawn (0..1). Kept per slot so the tint survives the part node
        // being replaced by an equipment change.
        std::array<float, ESM::PRT_Count> mPartDamageTint{};
        void applyPartDamageTint(ESM::PartReferenceType type);
        struct WoundOverlay
        {
            osg::ref_ptr<osg::Group> mParent;
            osg::ref_ptr<osg::Node> mNode;
            ESM::PartReferenceType mPart;
        };
        std::vector<WoundOverlay> mWoundOverlays;
        // Every mark's projection input, in its part node's frame, so a part
        // rebuilt by an equipment or view change gets its marks back (user
        // 2026-10-01: "make sure they don't despawn").
        struct WoundSeed
        {
            ESM::PartReferenceType mPart;
            std::string mRegion;
            int mKind;
            float mSeverity;
            osg::Vec3f mPoint, mNormal, mTangent;
            float mLength = 0, mHalfWidth = 0; // WOUNDFX-01 FX1 test shape, 0 = from severity
            int mSurface = 0; // WOUNDFX-01 FX3: the WoundFxSet::Surface it was placed on
        };
        std::vector<WoundSeed> mWoundSeeds;
        bool mReseedingWounds = false;
        int mReseedSurface = 0; // the surface of the seed being restored
        std::array<int, ESM::PRT_Count> mWoundRestoreFrames{};
        void restoreWoundSeeds(ESM::PartReferenceType type);
        // WOUNDFX-01 FX1/FX2: wounds on bare skin as shader wound volumes
        // (OPENMW_RAGDOLL_WOUND_FX=1); FX3 (=3): stains on clothing, scratches
        // and dents on armour.
        std::unique_ptr<WoundFxSet> mWoundFx;
        int mLastWoundFxKind = 0; // the kind addWoundFx drew (WoundFxSet::Kind), 0 = none
        bool placeWound(std::string_view region, int kind, const osg::Vec3f& point, const osg::Vec3f& normal,
            const osg::Vec3f& tangent, float severity, float length, float halfWidth);
        // With FX on, wounds from a receipt (which may come from the Lua
        // thread, in parallel with the scene's update and cull) wait here and
        // are placed in runAnimation on the main thread: swapping a skinned
        // part's copies under the cull crashed in RigGeometry::updateBounds
        // (10-08 duels, hand cuts).
        struct PendingWound
        {
            std::string mRegion;
            int mKind;
            float mSeverity;
            osg::Vec3f mPoint, mNormal, mTangent;
        };
        std::mutex mPendingWoundsMutex;
        bool mProbeWound = false; // placeWound only finds the surface (no scene change)
        int mProbedSurface = -1; // the surface (WoundFxSet::Surface) the last probe found, -1 none
        std::atomic<int> mLastWoundSurface{ -1 }; // what addLocalizedWound found (getLastWoundSurface)
        std::vector<PendingWound> mPendingWounds;
        // WOUNDFX-01 FX4 (OPENMW_RAGDOLL_WOUND_FX=4): blood on the held
        // weapon. Marks are kept per weapon item (its live reference) and come
        // back on its rebuilt node (sheathed and drawn again, a view change).
        struct WeaponBlood
        {
            const void* mItem; // the weapon's MWWorld::LiveCellRefBase
            float mFrom, mTo, mStrength, mBirth, mSeed;
            int mBloodType;
        };
        std::vector<WeaponBlood> mWeaponBlood;
        // GRIP-05: the transform the weapon part is wrapped in while it is
        // drawn turned in the hand, and the part it wraps.
        osg::ref_ptr<osg::MatrixTransform> mWeaponGripNode;
        osg::observer_ptr<osg::Node> mWeaponGripChild;
        struct PendingWeaponBlood
        {
            float mFrom, mTo, mStrength;
            int mBloodType;
        };
        std::vector<PendingWeaponBlood> mPendingWeaponBlood; // guarded by mPendingWoundsMutex
        osg::observer_ptr<osg::Node> mBloodiedWeaponNode; // the weapon node the marks are on
        std::atomic<std::size_t> mWeaponBloodPlaced{ 0 };
        WoundFxSet& woundFx();
        const void* currentWeaponItem() const;
        void placeWeaponBlood(const PendingWeaponBlood& blood);
        // Draw the current weapon's marks on its node: `only` that one, or
        // all of them (a fresh node).
        void applyWeaponBlood(const WeaponBlood* only);

        osg::Vec3f mFirstPersonOffset;
        // Field of view to use when rendering first person meshes
        float mFirstPersonFieldOfView;

        std::shared_ptr<HeadAnimationTime> mHeadAnimationTime;
        std::shared_ptr<WeaponAnimationTime> mWeaponAnimationTime;

        bool mSoundsDisabled;

        bool mAccurateAiming;
        float mAimingFactor;
        unsigned mPartTraceCounter = 0;

        void updateNpcBase();

        NpcType getNpcType() const;

        PartHolderPtr insertBoundedPart(VFS::Path::NormalizedView model, std::string_view bonename,
            std::string_view bonefilter, bool enchantedGlow, osg::Vec4f* glowColor, bool isLight);

        void removeIndividualPart(ESM::PartReferenceType type);
        void reserveIndividualPart(ESM::PartReferenceType type, int group, int priority);

        bool addOrReplaceIndividualPart(ESM::PartReferenceType type, int group, int priority,
            VFS::Path::NormalizedView mesh, bool enchantedGlow = false, osg::Vec4f* glowColor = nullptr,
            bool isLight = false);
        void removePartGroup(int group);
        void addPartGroup(int group, int priority, const std::vector<ESM::PartReference>& parts,
            bool enchantedGlow = false, osg::Vec4f* glowColor = nullptr);

        void setRenderBin();
        // KINETIC: the own body dissolves near the first-person camera.
        void applyNearDissolve();
        std::uint64_t mDissolveTuning = 0; // tuning generation of the last applyNearDissolve

        osg::ref_ptr<RotateController> mFirstPersonNeckController;

        static bool isFemalePart(const ESM::BodyPart* bodypart);
        static NpcType getNpcType(const MWWorld::Ptr& ptr);

    protected:
        void addControllers() override;
        bool isArrowAttached() const override;
        std::string getSheathedShieldMesh(const MWWorld::ConstPtr& shield) const override;

    public:
        /**
         * @param ptr
         * @param disableListener  Don't listen for equipment changes and magic effects. InventoryStore only supports
         *                         one listener at a time, so you shouldn't do this if creating several NpcAnimations
         *                         for the same Ptr, eg preview dolls for the player.
         *                         Those need to be manually rendered anyway.
         * @param disableSounds    Same as \a disableListener but for playing items sounds
         * @param viewMode
         */
        NpcAnimation(const MWWorld::Ptr& ptr, osg::ref_ptr<osg::Group> parentNode,
            Resource::ResourceSystem* resourceSystem, bool disableSounds = false, ViewMode viewMode = VM_Normal,
            float firstPersonFieldOfView = 55.f);
        virtual ~NpcAnimation();

        void enableHeadAnimation(bool enable) override;

        /// 1: the first person meshes follow the camera's rotation completely
        /// 0: the first person meshes follow the camera with a reduced factor, so you can look down at your own hands
        void setAccurateAiming(bool enabled) override;

        void setWeaponGroup(const std::string& group, bool relativeDuration) override;

        osg::Vec3f runAnimation(float timepassed) override;

        /// A relative factor (0-1) that decides if and how much the skeleton should be pitched
        /// to indicate the facing orientation of the character.
        void setPitchFactor(float factor) override { mPitchFactor = factor; }

        bool getWeaponsShown() const override { return mShowWeapons; }
        void showWeapons(bool showWeapon) override;

        bool updateCarriedLeftVisible(ESM::RefId weaptype) const override;
        bool getCarriedLeftShown() const override { return mShowCarriedLeft; }
        void showCarriedLeft(bool show) override;

        void attachArrow() override;
        void detachArrow() override;
        void releaseArrow(float attackStrength, float attackWindUp) override;

        osg::Group* getArrowBone() override;
        osg::Node* getWeaponNode() override;
        void setWeaponGripTurn(const std::optional<osg::Matrixf>& handFrame) override;
        // The equipped shield's part node (on "Shield Bone"), or null.
        osg::Node* getShieldNode();
        Resource::ResourceSystem* getResourceSystem() override;

        // WeaponAnimation
        void showWeapon(bool show) override { showWeapons(show); }

        void setViewMode(ViewMode viewMode);

        void updateParts();

        /// Rebuilds the NPC, updating their root model, animation sources, and equipment.
        void rebuild();

        /// Get the inventory slot that the given node path leads into, or -1 if not found.
        int getSlot(const osg::NodePath& path) const;

        void setAnatomicalDamageTint(std::string_view region, float intensity) override;
        void clearAnatomicalDamageTints() override;
        bool addLocalizedWound(std::string_view region, int kind, const osg::Vec3f& point,
            const osg::Vec3f& normal, const osg::Vec3f& tangent, float severity) override;
        bool addWoundFx(std::string_view region, const osg::Vec3f& point, const osg::Vec3f& normal,
            const osg::Vec3f& direction, float length, float halfWidth, float severity, int kind) override;
        std::size_t getWoundFxCount() const override;
        int getLastWoundFxKind() const override { return mLastWoundFxKind; }
        int getLastWoundSurface() const override { return mLastWoundSurface.load(); }
        bool addWeaponBlood(float alongFrom, float alongTo, float strength, int bloodType) override;
        std::size_t getWeaponBloodCount() const override { return mWeaponBloodPlaced.load(); }

        void setVampire(bool vampire) override;

        /// Set a translation offset (in object root space) to apply to meshes when in first person mode.
        void setFirstPersonOffset(const osg::Vec3f& offset);

        void updatePtr(const MWWorld::Ptr& updated) override;

        /// Get a list of body parts that may be used by an NPC of given race and gender.
        /// @note This is a fixed size list, one list item for each ESM::PartReferenceType, may contain nullptr body
        /// parts.
        static const std::vector<const ESM::BodyPart*>& getBodyParts(
            const ESM::RefId& raceId, bool female, bool firstperson, bool werewolf);
    };

}

#endif

#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_PIPELINE_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_PIPELINE_HPP

#include <ragdollcore/passiveragdoll.hpp>
#include <ragdollcore/profile.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Math/Vec3.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace RagdollCore
{
    // KINETIC-2's per-step pipeline (PLAN.md "KINETIC-2"): intent ->
    // situation -> channel ownership -> controllers -> composer -> motors ->
    // Jolt -> contact and damage -> observation. This header holds the two
    // stages that make the rest safe to move one area at a time: who owns
    // each channel this step, and the composer that takes a channel's writes
    // from its owner only.
    //
    // An area still on the old integration owns its channels as Legacy: the
    // old code writes them, through the composer, as before. Moving an area
    // flips its channels from Legacy to the area, and any other writer is
    // rejected and counted (the rejections are where two owners fought).
    //
    // Pure: no engine types.

    enum class Channel : std::uint8_t
    {
        Root, // the soft root's horizontal hold on the pelvis
        PelvisHeight, // the vertical support (who holds the body up)
        LeftLeg,
        RightLeg,
        TrunkTilt, // the spine's lean (forward, back, sideways)
        TrunkTurn, // the spine's turn about the vertical
        Head, // neck and head
        LeftArm, // clavicle to hand
        RightArm,
        Count,
    };
    inline constexpr std::size_t sChannelCount = static_cast<std::size_t>(Channel::Count);

    // The areas that can own channels, each one module (and one phase).
    enum class Area : std::uint8_t
    {
        Legacy, // the old integration, until the area moves
        Lifecycle, // building, death, corpses (phase 4)
        Root, // root strength (phase 4)
        Legs, // Stride: stepping, standing, the gait's arms and trunk twist (phase 2)
        Upright, // the upright holder (phase 3)
        Reaction, // blows, staggers, falls, get-up (phase 3)
        Weapon, // the weapon and hands (phase 1)
        Count,
    };
    inline constexpr std::size_t sAreaCount = static_cast<std::size_t>(Area::Count);

    std::string_view channelName(Channel channel);
    std::string_view areaName(Area area);

    // What the body is doing this step (the situation stage's verdict).
    enum class BodyState : std::uint8_t
    {
        Standing, // on its feet: standing, walking, running, fighting
        Airborne, // a jump or a short drop it lands on its feet from
        Staggered, // on its feet after a blow, catching itself
        Falling, // going down
        Downed, // lying
        GettingUp,
        Dead,
    };

    // Which areas have moved to the pipeline.
    struct MovedAreas
    {
        std::array<bool, sAreaCount> mMoved{};
        bool moved(Area area) const { return area != Area::Legacy && mMoved[static_cast<std::size_t>(area)]; }
        MovedAreas& with(Area area)
        {
            mMoved[static_cast<std::size_t>(area)] = true;
            return *this;
        }
    };

    struct OwnershipInput
    {
        BodyState mState = BodyState::Standing;
        bool mWeaponReady = false; // a weapon or fists drawn
        bool mWeaponRight = true; // the weapon hand (a fist fighter's leading hand)
        // A reaction has taken the weapon arm (a flung arm, a stagger); the
        // attack resumes when it gives it back.
        bool mReactionHasWeaponArm = false;
        MovedAreas mMoved;
    };

    using Ownership = std::array<Area, sChannelCount>;

    // The one owner of each channel. The rules (MOVE-01 section 4.8):
    // - the root is root strength's, the pelvis height and the legs Stride's
    //   while on the feet or in the air;
    // - trunk tilt and head the upright holder's, the reaction's while
    //   staggered;
    // - with a weapon ready the weapon owns both arms and the trunk's turn
    //   (proximal-to-distal cuts); without, the gait does (arm swing against
    //   the trunk's twist); a reaction that took the weapon arm has it and
    //   the turn, and a staggered body's free arm flies out with the reaction;
    // - falling, lying and getting up: the reaction owns every channel but
    //   the root;
    // - dead: the lifecycle owns all.
    // A channel whose owner has not moved is Legacy.
    Ownership resolveOwnership(const OwnershipInput& input);

    inline Area owner(const Ownership& ownership, Channel channel)
    {
        return ownership[static_cast<std::size_t>(channel)];
    }

    // Each body's pose channel and, for the spine and what rides on it, its
    // share of the trunk's turn (0 = none, 1 = all of it).
    struct ChannelMap
    {
        std::vector<std::optional<Channel>> mBodyChannel;
        std::vector<float> mTurnShare;
        std::optional<std::size_t> mPelvis; // the turn's pivot
        JPH::Vec3 mUp = JPH::Vec3::sAxisZ(); // model space's vertical
    };

    // From the profile's body names (pelvis -> Root, spine* -> TrunkTilt,
    // neck/head -> Head, legs, arms; others none). The spine's turn shares
    // grow up the spine (spine 0.3, spine1 0.65, spine2 1); neck and head
    // turn with the chest.
    ChannelMap channelMapFromNames(std::span<const std::string_view> bodyNames);

    // The composer: one step's pose from its owners' writes.
    class StepComposer
    {
    public:
        struct Rejections
        {
            // [writer][channel]: writes refused because the writer did not
            // own the channel.
            std::array<std::array<std::uint32_t, sChannelCount>, sAreaCount> mCount{};
            std::uint32_t total() const;
        };

        // The base pose (the animation's), the map and this step's owners.
        void begin(const RagdollPose& base, const ChannelMap& map, const Ownership& ownership);

        // A body's target, in model space. False, and counted, if `writer`
        // does not own the body's channel (or the body has none).
        bool writeBody(Area writer, std::size_t body, const Transform& target);

        // The trunk's turn about the vertical (radians, positive
        // counter-clockwise seen from above). False, and counted, unless
        // `writer` owns TrunkTurn.
        bool writeTrunkTurn(Area writer, float radians);

        // Root and pelvis height targets (their consumers move in phases 2
        // and 4); owner-checked like the rest.
        bool writeRoot(Area writer, const JPH::Vec3& horizontalTarget);
        bool writePelvisHeight(Area writer, float height);

        // The base with every accepted body write, then the trunk's turn
        // applied over the tilt: each spine body, and the neck and head on
        // it, turned by its share about the vertical through the pelvis (the
        // spine stands near that vertical, so this is each segment's turn).
        // Arms are written in model space by their owner and not turned.
        RagdollPose compose() const;

        std::optional<JPH::Vec3> rootTarget() const { return mRoot; }
        std::optional<float> pelvisHeight() const { return mPelvisHeight; }
        float trunkTurn() const { return mTurn; }
        const Rejections& rejections() const { return mRejections; }
        // Rejections are kept across steps (the observation stage reads and
        // clears them).
        void clearRejections() { mRejections = {}; }

    private:
        bool accept(Area writer, Channel channel);

        RagdollPose mPose;
        const ChannelMap* mMap = nullptr;
        Ownership mOwnership{};
        float mTurn = 0.0f;
        std::optional<JPH::Vec3> mRoot;
        std::optional<float> mPelvisHeight;
        Rejections mRejections;
    };
}

#endif

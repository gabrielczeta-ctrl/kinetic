#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_FOOTFALLGAIT_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_FOOTFALLGAIT_HPP

#include <array>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace RagdollCore
{
    // Reference gaits for Footfall, measured from a human motion model
    // (MotionBricks; see files/data/ragdoll/footfall-gaits.cfg and the
    // generator in tools-workspace/motionbricks). Every row is one steady
    // gait of the source leg: lengths in source leg lengths, times in
    // seconds. Rows are matched to another leg by Froude similarity: equal
    // speed^2 / (g * leg), times scaled by sqrt(leg / source leg).
    struct FootfallGaitRow
    {
        float mSpeed = 0.0f;     // source leg lengths per second
        float mDuty = 0.5f;      // share of the stride a foot is on the ground
        float mSwingTime = 0.4f; // seconds, source leg
        float mLead = 0.3f;      // foot ahead of the pelvis at touchdown, leg lengths
        float mCrouch = 0.0f;    // pelvis below standing height, leg lengths
        std::array<float, 21> mLift{};     // swing foot height over the swing, leg lengths
        std::array<float, 21> mProgress{}; // share of the swing distance covered
        std::array<float, 11> mBob{};      // pelvis height over one step about its mean, leg lengths
        // Optional (both or neither): the swing ankle relative to the hip in
        // the travel frame (x along travel, y outward, z up; leg lengths) and
        // the knee flexion over the stance (degrees).
        bool mHasPath = false;
        std::array<std::array<float, 21>, 3> mPath{};
        std::array<float, 21> mStanceKnee{};
    };

    // A gait scaled onto a target leg (lengths in the target's units).
    struct FootfallGaitSample
    {
        float mDuty = 0.5f;
        float mSwingTime = 0.4f;      // seconds
        float mLead = 0.0f;           // target units
        float mCrouch = 0.0f;         // target units
        float mMaximumFroude = 0.0f;  // fastest row's Froude number (beyond it the sample is clamped)
        float mFroude = 0.0f;         // the query's Froude number
        std::array<float, 21> mLift{};     // target units, endpoints removed (0 at lift-off and touchdown)
        std::array<float, 21> mProgress{};
        std::array<float, 11> mBob{};      // target units
        bool mHasPath = false;
        std::array<std::array<float, 21>, 3> mPath{}; // target units, relative to the hip
        std::array<float, 21> mStanceKnee{};          // degrees

        // Curves sampled at a phase in [0, 1].
        float lift(float phase) const;
        float progress(float phase) const;
        float bob(float phase) const;
        float path(std::size_t axis, float phase) const;
        float stanceKnee(float phase) const;

        // a + (b - a) * t for every field (curves too); a path only where both have one.
        static FootfallGaitSample mix(const FootfallGaitSample& a, const FootfallGaitSample& b, float t);
    };

    class FootfallGaitTable
    {
    public:
        // Parses the text format written by to_footfall_cfg.py; empty on failure.
        static std::optional<FootfallGaitTable> load(const std::filesystem::path& path);
        static std::optional<FootfallGaitTable> parse(const std::string& text);

        bool has(const std::string& style) const { return mStyles.count(style) != 0; }
        float sourceLegLength() const { return mSourceLegLength; }

        // travelAngle: degrees, travel relative to facing (0 forward, 90 left,
        // 180 backward, -90 right). speed and legLength in metres (per second).
        std::optional<FootfallGaitSample> sample(
            const std::string& style, float travelAngle, float speed, float legLength, float unitsPerMetre) const;

    private:
        float mSourceLegLength = 0.65f;
        // style -> travel angle (degrees) -> rows sorted by speed
        std::map<std::string, std::map<int, std::vector<FootfallGaitRow>>> mStyles;
    };
}

#endif

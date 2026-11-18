#ifndef OPENMW_RAGDOLLCORE_TESTS_GETUPPOLICY_HPP
#define OPENMW_RAGDOLLCORE_TESTS_GETUPPOLICY_HPP

// The get-up policy of the MOVE-01 training pilot (section 9): the network,
// what it observes, and one episode on StrideSim. Shared by
// ragdollcore-trainer (trainer.cpp) and the evaluation probe
// (teststridesim.cpp). Offline only.

#include <ragdollcore/stridesim.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace GetUpPolicy
{
    using namespace RagdollCore;

    constexpr int sHidden = 64;
    constexpr int sDecisionSteps = 2; // a decision every 2 physics steps (30 Hz)
    constexpr int sSettleSteps = 30;
    constexpr int sEpisodeSteps = 300; // 5 s of policy
    constexpr float sMaxOffset = 1.2f; // radians per joint axis

    struct Net
    {
        int mIn = 0, mOut = 0;
        std::size_t size() const
        {
            return static_cast<std::size_t>(mIn * sHidden + sHidden + sHidden * sHidden + sHidden + sHidden * mOut + mOut);
        }
        void forward(const float* w, const std::vector<float>& x, std::vector<float>& y) const
        {
            float h1[sHidden], h2[sHidden];
            const float* p = w;
            for (int j = 0; j < sHidden; ++j)
            {
                float s = p[mIn * sHidden + j];
                for (int i = 0; i < mIn; ++i)
                    s += x[i] * p[i * sHidden + j];
                h1[j] = std::tanh(s);
            }
            p += mIn * sHidden + sHidden;
            for (int j = 0; j < sHidden; ++j)
            {
                float s = p[sHidden * sHidden + j];
                for (int i = 0; i < sHidden; ++i)
                    s += h1[i] * p[i * sHidden + j];
                h2[j] = std::tanh(s);
            }
            p += sHidden * sHidden + sHidden;
            y.assign(mOut, 0.0f);
            for (int j = 0; j < mOut; ++j)
            {
                float s = p[sHidden * mOut + j];
                for (int i = 0; i < sHidden; ++i)
                    s += h2[i] * p[i * mOut + j];
                y[j] = std::tanh(s);
            }
        }
    };

    // The four starts: on the back, the front, each side.
    inline JPH::Quat lyingRotation(int start)
    {
        const float half = 0.5f * 3.14159265f;
        switch (start % 4)
        {
            case 0:
                return JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -half);
            case 1:
                return JPH::Quat::sRotation(JPH::Vec3::sAxisX(), half);
            case 2:
                return JPH::Quat::sRotation(JPH::Vec3::sAxisY(), half);
            default:
                return JPH::Quat::sRotation(JPH::Vec3::sAxisY(), -half);
        }
    }

    inline void observe(const StrideSim& sim, float progress, std::vector<float>& x)
    {
        x.clear();
        const JPH::RVec3 pelvis = sim.bodyPosition(sim.pelvis());
        for (std::size_t b = 0; b < sim.bodyCount(); ++b)
        {
            const JPH::Vec3 at(sim.bodyPosition(b) - pelvis);
            const JPH::Quat q = sim.bodyRotation(b);
            const JPH::Vec3 up = q * JPH::Vec3::sAxisZ();
            const JPH::Vec3 fwd = q * JPH::Vec3::sAxisY();
            const JPH::Vec3 v = sim.bodyVelocity(b);
            for (float f : { at.GetX() / 100.0f, at.GetY() / 100.0f, at.GetZ() / 100.0f, up.GetX(), up.GetY(),
                     up.GetZ(), fwd.GetX(), fwd.GetY(), fwd.GetZ(), v.GetX() / 300.0f, v.GetY() / 300.0f,
                     v.GetZ() / 300.0f })
                x.push_back(f);
        }
        x.push_back(static_cast<float>(pelvis.GetZ()) / 100.0f);
        x.push_back(progress);
    }

    struct Episode
    {
        float mReward = 0.0f;
        float mBestPelvis = 0.0f; // share of standing height
        float mMeanAssist = 0.0f; // the root's help, 0-1
        bool mStood = false;
    };

    inline std::string gDataDir;

    // assistCap: the most root help this generation allows (the curriculum).
    inline Episode runEpisode(const Net& net, const float* w, int start, float assistCap)
    {
        StrideSim::Settings settings;
        settings.mLyingRotation = lyingRotation(start);
        settings.mLimpAssist = true;
        if (!gDataDir.empty())
            settings.mActiveBodyPath = gDataDir + "/active-body.cfg";
        StrideSim sim(settings);
        Episode episode;
        if (!sim.valid())
        {
            episode.mReward = -10.0f;
            return episode;
        }
        // Lying limp on the base pose's joints, softly, to settle.
        sim.setLimpTarget(sim.basePose(), 0.15f);
        for (int i = 0; i < sSettleSteps; ++i)
            sim.step();
        const float standPelvis = std::max(sim.standingPelvisHeight(), 1.0f);
        const float standHead = standPelvis * 1.75f;
        std::vector<float> x, y;
        RagdollPose target = sim.basePose();
        int stoodSteps = 0;
        float total = 0.0f, assist = 0.0f, assistTotal = 0.0f;
        for (int i = 0; i < sEpisodeSteps; ++i)
        {
            if (i % sDecisionSteps == 0)
            {
                observe(sim, static_cast<float>(i) / sEpisodeSteps, x);
                net.forward(w, x, y);
                target = sim.basePose();
                for (std::size_t b = 1; b < sim.bodyCount(); ++b)
                {
                    const JPH::Vec3 offset(y[(b - 1) * 3], y[(b - 1) * 3 + 1], y[(b - 1) * 3 + 2]);
                    const float angle = offset.Length() * sMaxOffset;
                    if (angle > 1e-5f)
                        target.mBodyTransforms[b].mRotation
                            = (JPH::Quat::sRotation(offset.Normalized(), angle) * target.mBodyTransforms[b].mRotation)
                                  .Normalized();
                }
                sim.setLimpTarget(target, 0.2f + 0.4f * (y[y.size() - 2] + 1.0f));
                // The root's help toward a standing pelvis over where it is.
                assist = assistCap * 0.5f * (y.back() + 1.0f);
                const JPH::RVec3 at = sim.bodyPosition(sim.pelvis());
                sim.setAssist(JPH::RVec3(at.GetX(), at.GetY(), standPelvis), JPH::Quat::sIdentity(), assist);
            }
            assistTotal += assist;
            sim.step();
            const float pelvis = static_cast<float>(sim.bodyPosition(sim.pelvis()).GetZ()) / standPelvis;
            const float head = static_cast<float>(sim.bodyPosition(sim.head()).GetZ()) / standHead;
            const float up = (sim.bodyRotation(sim.pelvis()) * JPH::Vec3::sAxisZ()).GetZ();
            const float tilt = std::max(0.0f, up);
            // Height and uprightness, less the help taken (MOVE-01: the root
            // is a budget, as little as the body needs).
            total += 0.5f * std::min(pelvis, 1.1f) + 0.5f * std::min(head, 1.1f) + 0.3f * tilt - 0.6f * assist;
            episode.mBestPelvis = std::max(episode.mBestPelvis, pelvis);
            stoodSteps = pelvis > 0.85f && head > 0.85f && assist < 0.25f ? stoodSteps + 1 : 0;
            if (stoodSteps >= 30)
                episode.mStood = true;
        }
        episode.mReward = total / sEpisodeSteps + (episode.mStood ? 1.0f : 0.0f);
        episode.mMeanAssist = assistTotal / sEpisodeSteps;
        return episode;
    }

}

#endif

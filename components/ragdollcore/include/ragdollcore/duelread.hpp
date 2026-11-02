#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_DUELREAD_HPP
#define OPENMW_COMPONENTS_RAGDOLLCORE_DUELREAD_HPP

#include <algorithm>
#include <cmath>

namespace RagdollCore
{
    // AI-02 item 6 (docs/ragdoll/designs/AI-02-smart-duel-ai.md): reading an
    // incoming swing for the guard's zone. The attacker's blade in the
    // defender's chest frame (x right, y forward, z up; units, u/s): a slow
    // blade (the wind-up) shows where it will come from - over the head is a
    // chop, beside a shoulder a side cut, below the belt a rising one; a fast
    // blade is carried on to where it will arrive. The read errs by turning
    // that point about the chest axis by up to `mErrorDegrees` (one draw per
    // swing, `mDraw` in -1..1): a clumsy defender guards the wrong side now
    // and then.
    enum class GuardRead
    {
        Middle,
        High,
        Left,
        Right,
        Low,
    };

    struct GuardReadInput
    {
        float mX = 0.0f, mY = 0.0f, mZ = 0.0f; // the blade
        float mVX = 0.0f, mVY = 0.0f, mVZ = 0.0f; // its velocity
        float mErrorDegrees = 0.0f;
        float mDraw = 0.0f; // -1..1
    };

    inline const char* guardReadName(GuardRead zone)
    {
        switch (zone)
        {
            case GuardRead::High:
                return "high";
            case GuardRead::Left:
                return "left";
            case GuardRead::Right:
                return "right";
            case GuardRead::Low:
                return "low";
            default:
                return "middle";
        }
    }

    // `arrival` (optional): for a fast blade, the seconds until it reaches the
    // chest (the carry time below); -1 for a slow one (AI-02 timed parry).
    inline GuardRead readGuardZone(const GuardReadInput& in, float* arrival = nullptr)
    {
        if (arrival)
            *arrival = -1.0f;
        float x = in.mX, y = in.mY, z = in.mZ;
        const float speed = std::sqrt(in.mVX * in.mVX + in.mVY * in.mVY + in.mVZ * in.mVZ);
        const bool fast = speed > 300.0f;
        if (fast)
        {
            // Carried on until it reaches the chest: the time to close the
            // flat distance (less a body's half width), at most 0.4 s.
            const float flat = std::hypot(x, y);
            const float closing = flat > 1e-3f ? -(x * in.mVX + y * in.mVY) / flat : 0.0f;
            const float tau = closing > 1.0f ? std::clamp((flat - 25.0f) / closing, 0.0f, 0.4f) : 0.0f;
            if (arrival && closing > 1.0f)
                *arrival = tau;
            x += in.mVX * tau;
            y += in.mVY * tau;
            z += in.mVZ * tau;
            // A blade driven straight in along the chest axis is a thrust.
            const float along = speed > 1e-3f ? -in.mVY / speed : 0.0f;
            if (along > 0.8f && std::abs(x) < 15.0f && z > -30.0f && z < 25.0f)
                return GuardRead::Middle;
        }
        const float error = std::clamp(in.mDraw, -1.0f, 1.0f) * in.mErrorDegrees * 3.14159265f / 180.0f;
        if (error != 0.0f)
        {
            const float c = std::cos(error), s = std::sin(error);
            const float rx = c * x - s * z, rz = s * x + c * z;
            x = rx;
            z = rz;
        }
        const float high = fast ? 18.0f : 30.0f;
        if (z > high && z >= std::abs(x) * 0.5f)
            return GuardRead::High;
        if (z < -30.0f)
            return GuardRead::Low;
        if (x > 10.0f)
            return GuardRead::Right;
        if (x < -10.0f)
            return GuardRead::Left;
        return GuardRead::Middle;
    }

    // AI-02 item 4: the attack line an opponent leaves open. Candidates are
    // wheel angles (radians; 0 the attacker's right, pi/2 over the top, 3pi/2
    // the thrust). A line arrives at the foe on the side it starts from: 0 at
    // the foe's left, pi at its right, pi/4..3pi/4 high, 5pi/4 and 7pi/4 low,
    // the thrust in the middle. Score: 1 if the foe's guard zone does not
    // cover it (0.5 for a neighbouring zone), + 0.5 x how far the foe's blade
    // is from the arrival (per 40 units, at most 1), - 0.3 if it was one of
    // the last two lines. The best line is taken with chance `mBestChance`
    // (skill), else the one in turn (`mTurnLine`).
    struct LineChoiceInput
    {
        const float* mLines = nullptr;
        int mCount = 0;
        int mGuardZone = -1; // GuardRead as int; -1 no guard up
        bool mBladeKnown = false;
        float mBladeX = 0.0f, mBladeZ = 0.0f; // the foe's blade from its chest, attacker frame (x right, z up)
        float mRecent[2] = { -100.0f, -100.0f };
        float mBestChance = 0.0f;
        float mDraw = 1.0f; // 0..1
        float mTurnLine = 0.0f;
    };

    inline float lineCover(int zone, float angle)
    {
        const int deg = static_cast<int>(std::lround(angle * 180.0f / 3.14159265f)) % 360;
        const int a = deg < 0 ? deg + 360 : deg;
        switch (static_cast<GuardRead>(zone))
        {
            case GuardRead::High:
                return a == 45 || a == 90 || a == 135 ? 1.0f : 0.0f;
            case GuardRead::Low:
                return a == 225 || a == 315 ? 1.0f : 0.0f;
            case GuardRead::Left: // the foe's left = the attacker's right
                return a == 0 ? 1.0f : a == 45 || a == 315 ? 0.5f : 0.0f;
            case GuardRead::Right:
                return a == 180 ? 1.0f : a == 135 || a == 225 ? 0.5f : 0.0f;
            case GuardRead::Middle:
                return a == 270 ? 1.0f : 0.0f;
        }
        return 0.0f;
    }

    inline float lineScore(const LineChoiceInput& in, float angle)
    {
        float score = 1.0f - (in.mGuardZone >= 0 ? lineCover(in.mGuardZone, angle) : 0.0f);
        if (in.mBladeKnown)
        {
            const bool thrust = std::abs(std::sin(angle) + 1.0f) < 1e-3f;
            const float ax = thrust ? 0.0f : 30.0f * std::cos(angle), az = thrust ? 0.0f : 30.0f * std::sin(angle);
            score += 0.5f * std::min(1.0f, std::hypot(in.mBladeX - ax, in.mBladeZ - az) / 40.0f);
        }
        for (const float recent : in.mRecent)
            if (std::abs(recent - angle) < 1e-3f)
                score -= 0.3f;
        return score;
    }

    inline float chooseAttackLine(const LineChoiceInput& in)
    {
        if (!in.mLines || in.mCount <= 0 || in.mDraw >= in.mBestChance)
            return in.mTurnLine;
        float best = in.mTurnLine, bestScore = -1.0e9f;
        for (int i = 0; i < in.mCount; ++i)
        {
            const float score = lineScore(in, in.mLines[i]);
            if (score > bestScore + 1e-4f)
            {
                bestScore = score;
                best = in.mLines[i];
            }
        }
        return best;
    }
}

#endif

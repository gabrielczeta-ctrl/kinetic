#include <ragdollcore/cutwheel.hpp>
#include <ragdollcore/tuning.hpp>

#include <Jolt/Math/Vec4.h>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace RagdollCore
{
    namespace
    {
        constexpr float sPi = std::numbers::pi_v<float>;
        constexpr float sTau = 2.0f * sPi;
        constexpr float sEighth = sPi / 4.0f;

        float wrapAngle(float a)
        {
            a = std::fmod(a, sTau);
            return a < 0.0f ? a + sTau : a;
        }

        ChamberKey key(float s, JPH::Vec3 hand, JPH::Vec3 blade, JPH::Vec3 pole, float twist,
            JPH::Vec3 edge = JPH::Vec3::sZero())
        {
            return ChamberKey{ s, hand, blade, pole, twist, edge };
        }

        WheelLines sorted(WheelLines lines)
        {
            std::sort(lines.begin(), lines.end(), [](const AttackLine& a, const AttackLine& b) { return a.angle() < b.angle(); });
            return lines;
        }

        // The sword lines (11-03 as played; the authoring notes are in git
        // history of passiveragdollentry.inl, attackLines()).
        WheelLines makeWeaponLines()
        {
            const ChamberKey guard
                = key(0.0f, { 8.0f, 24.0f, -8.0f }, { 0.05f, 0.75f, 0.66f }, { 0.3f, 0.1f, -0.95f }, 0.0f, { -0.5f, 0.3f, -0.8f });
            const JPH::Vec3 down(0.0f, 0.0f, -1.0f);
            return sorted(WheelLines{ {
                // Chop from chamber D, straight down (Scheitelhau, fendente).
                { "chop", 0.0f, 1.0f, "chop",
                    { guard, key(0.5f, { 13.0f, 12.0f, 14.0f }, { 0.15f, -0.1f, 0.99f }, { 0.6f, 0.4f, 0.2f }, 22.0f),
                        key(1.0f, { 11.0f, -5.0f, 23.0f }, { -0.1f, -0.45f, -0.89f }, { 0.5f, 0.45f, 0.75f }, 35.0f) },
                    { key(0.0f, { 11.0f, -5.0f, 23.0f }, { -0.1f, -0.45f, -0.89f }, { 0.5f, 0.45f, 0.75f }, 35.0f),
                        key(0.3f, { 9.0f, 8.0f, 28.0f }, { 0.0f, -0.3f, 0.95f }, { 0.4f, 0.6f, 0.6f }, 25.0f),
                        key(0.5f, { 6.0f, 24.0f, 14.0f }, { 0.0f, 0.75f, 0.66f }, { 0.4f, 0.4f, -0.8f }, 10.0f),
                        key(0.7f, { 4.0f, 32.0f, 2.0f }, { 0.0f, 0.95f, -0.3f }, { 0.3f, 0.1f, -0.95f }, 0.0f),
                        key(1.0f, { 3.0f, 22.0f, -14.0f }, { 0.05f, 0.6f, -0.8f }, { 0.25f, -0.1f, -0.96f }, -5.0f) } },
                // Forehand slash, right to left at chest height; the tip swings
                // out round the right side, the finish down and away.
                { "forehand", 1.0f, 0.0f, "slash",
                    { guard, key(0.5f, { 14.0f, 14.0f, 6.0f }, { 0.4f, -0.2f, 0.9f }, { 0.6f, 0.3f, -0.75f }, 20.0f),
                        key(1.0f, { 20.0f, 2.0f, 6.0f }, { 0.3f, -0.95f, 0.05f }, { 0.3f, 0.5f, -0.8f }, 35.0f) },
                    { key(0.0f, { 20.0f, 2.0f, 6.0f }, { 0.3f, -0.95f, 0.05f }, { 0.3f, 0.5f, -0.8f }, 35.0f),
                        key(0.18f, { 19.0f, 12.0f, 5.0f }, { 0.9f, 0.4f, 0.05f }, { 0.35f, 0.4f, -0.85f }, 22.0f),
                        key(0.35f, { 14.0f, 22.0f, 4.0f }, { -0.45f, 0.89f, 0.05f }, { 0.35f, 0.3f, -0.9f }, 10.0f),
                        key(0.65f, { -2.0f, 32.0f, 2.0f }, { -0.8f, 0.6f, 0.0f }, { 0.3f, 0.1f, -0.95f }, -15.0f),
                        key(1.0f, { -14.0f, 20.0f, -8.0f }, { -0.75f, 0.15f, -0.65f }, { 0.1f, -0.1f, -0.99f }, -35.0f) } },
                // Backhand slash, left to right, a straighter arm.
                { "backhand", -1.0f, 0.0f, "slash",
                    { guard, key(0.5f, { -2.0f, 20.0f, 0.0f }, { -0.3f, 0.3f, 0.9f }, { 0.4f, 0.3f, -0.85f }, -20.0f),
                        key(1.0f, { -12.0f, 18.0f, 6.0f }, { -0.55f, -0.75f, 0.35f }, { 0.3f, 0.2f, -0.93f }, -40.0f) },
                    { key(0.0f, { -12.0f, 18.0f, 6.0f }, { -0.55f, -0.75f, 0.35f }, { 0.3f, 0.2f, -0.93f }, -40.0f),
                        key(0.35f, { -4.0f, 26.0f, 4.0f }, { -0.6f, 0.8f, 0.1f }, { 0.35f, 0.2f, -0.9f }, -15.0f),
                        key(0.65f, { 8.0f, 32.0f, 2.0f }, { 0.75f, 0.65f, 0.0f }, { 0.4f, 0.0f, -0.9f }, 5.0f),
                        key(1.0f, { 12.0f, 26.0f, -6.0f }, { 0.75f, 0.15f, -0.65f }, { 0.3f, -0.1f, -0.95f }, 10.0f) } },
                // Thrust: drawn back to the hip, the shoulder turned back, then
                // driven out; the true edge down.
                { "thrust", 0.0f, -1.0f, "thrust",
                    { guard, key(0.5f, { 12.0f, 14.0f, -8.0f }, { 0.0f, 0.95f, 0.3f }, { 0.35f, -0.3f, -0.89f }, 10.0f, down),
                        key(1.0f, { 14.0f, 2.0f, -7.0f }, { 0.0f, 0.97f, 0.25f }, { 0.35f, -0.5f, -0.79f }, 25.0f, down) },
                    { key(0.0f, { 14.0f, 2.0f, -7.0f }, { 0.0f, 0.97f, 0.25f }, { 0.35f, -0.5f, -0.79f }, 25.0f, down),
                        key(0.35f, { 10.0f, 16.0f, -5.0f }, { 0.0f, 0.96f, 0.28f }, { 0.25f, -0.6f, -0.76f }, 10.0f, down),
                        key(1.0f, { 4.0f, 38.0f, 4.0f }, { -0.05f, 0.9f, 0.43f }, { 0.3f, 0.0f, -0.95f }, -30.0f, down) } },
                // Oberhau from the right: vom Tag on the right shoulder, down to
                // low left, ending point forward.
                { "up-right", 0.7071f, 0.7071f, "chop",
                    { guard, key(0.5f, { 14.0f, 12.0f, 10.0f }, { 0.3f, -0.1f, 0.95f }, { 0.6f, 0.4f, -0.6f }, 20.0f),
                        key(1.0f, { 16.0f, 2.0f, 18.0f }, { 0.2f, -0.7f, 0.68f }, { 0.45f, 0.75f, -0.45f }, 30.0f) },
                    { key(0.0f, { 16.0f, 2.0f, 18.0f }, { 0.2f, -0.7f, 0.68f }, { 0.45f, 0.75f, -0.45f }, 30.0f),
                        key(0.35f, { 10.0f, 22.0f, 14.0f }, { -0.4f, 0.7f, 0.6f }, { 0.4f, 0.4f, -0.8f }, 15.0f),
                        key(0.65f, { 0.0f, 32.0f, 0.0f }, { -0.6f, 0.75f, -0.3f }, { 0.3f, 0.1f, -0.95f }, 0.0f),
                        key(1.0f, { -10.0f, 22.0f, -12.0f }, { -0.6f, 0.3f, -0.75f }, { 0.15f, -0.1f, -0.98f }, -25.0f) } },
                // Oberhau from the left, high over the left shoulder to low right.
                { "up-left", -0.7071f, 0.7071f, "chop",
                    { guard, key(0.5f, { 2.0f, 20.0f, 8.0f }, { -0.2f, 0.2f, 0.96f }, { 0.4f, 0.5f, -0.75f }, -15.0f),
                        key(1.0f, { -2.0f, 16.0f, 16.0f }, { -0.3f, -0.6f, 0.74f }, { 0.5f, 0.5f, -0.7f }, -30.0f) },
                    { key(0.0f, { -2.0f, 16.0f, 16.0f }, { -0.3f, -0.6f, 0.74f }, { 0.5f, 0.5f, -0.7f }, -30.0f),
                        key(0.35f, { 0.0f, 26.0f, 12.0f }, { 0.4f, 0.7f, 0.6f }, { 0.4f, 0.4f, -0.8f }, -15.0f),
                        key(0.65f, { 8.0f, 32.0f, 0.0f }, { 0.6f, 0.75f, -0.3f }, { 0.35f, 0.1f, -0.93f }, 5.0f),
                        key(1.0f, { 16.0f, 22.0f, -10.0f }, { 0.6f, 0.3f, -0.75f }, { 0.3f, -0.1f, -0.95f }, 20.0f) } },
                // Unterhau from the right: low by the right hip (Alber), rising to
                // high left, ending Ochs-like with the elbow out.
                { "down-right", 0.7071f, -0.7071f, "slash",
                    { guard, key(0.5f, { 12.0f, 18.0f, -12.0f }, { 0.3f, 0.3f, -0.9f }, { 0.4f, 0.0f, -0.9f }, 15.0f),
                        key(1.0f, { 16.0f, 10.0f, -24.0f }, { 0.2f, -0.8f, -0.55f }, { 0.3f, -0.6f, -0.75f }, 20.0f) },
                    { key(0.0f, { 16.0f, 10.0f, -24.0f }, { 0.2f, -0.8f, -0.55f }, { 0.3f, -0.6f, -0.75f }, 20.0f),
                        key(0.35f, { 10.0f, 22.0f, -8.0f }, { -0.3f, 0.8f, -0.5f }, { 0.35f, 0.1f, -0.93f }, 15.0f),
                        key(0.65f, { 2.0f, 30.0f, 8.0f }, { -0.6f, 0.7f, 0.4f }, { 0.35f, 0.1f, -0.93f }, 0.0f),
                        key(1.0f, { -8.0f, 24.0f, 20.0f }, { -0.5f, 0.3f, 0.8f }, { 0.7f, 0.1f, 0.3f }, -20.0f) } },
                // Unterhau from the left: low left (kept in front), rising to
                // high right; the point rolls over and drops in two keys.
                { "down-left", -0.7071f, -0.7071f, "slash",
                    { guard, key(0.5f, { 3.0f, 22.0f, -8.0f }, { -0.75f, 0.5f, -0.4f }, { 0.5f, 0.2f, -0.85f }, -15.0f),
                        key(1.0f, { 2.0f, 20.0f, -22.0f }, { -0.45f, -0.2f, -0.87f }, { 0.7f, 0.0f, -0.7f }, -15.0f) },
                    { key(0.0f, { 2.0f, 20.0f, -22.0f }, { -0.45f, -0.2f, -0.87f }, { 0.7f, 0.0f, -0.7f }, -15.0f),
                        key(0.35f, { -2.0f, 24.0f, -6.0f }, { 0.3f, 0.8f, -0.5f }, { 0.4f, 0.2f, -0.9f }, -15.0f),
                        key(0.65f, { 6.0f, 30.0f, 8.0f }, { 0.6f, 0.7f, 0.4f }, { 0.35f, 0.1f, -0.93f }, 5.0f),
                        key(1.0f, { 18.0f, 26.0f, 22.0f }, { 0.5f, 0.3f, 0.8f }, { 0.9f, -0.3f, 0.3f }, 20.0f) } },
            } });
        }

        // A long haft's left-to-right sweep: chambered low across the front,
        // the tip out left and never behind the back, rising a little across
        // the target to level on the right.
        WheelLines makeSpearLines()
        {
            WheelLines lines = makeWeaponLines();
            const ChamberKey guard
                = key(0.0f, { 8.0f, 24.0f, -8.0f }, { 0.05f, 0.75f, 0.66f }, { 0.3f, 0.1f, -0.95f }, 0.0f, { -0.5f, 0.3f, -0.8f });
            const JPH::Vec3 chamberHand(-16.0f, 14.0f, -6.0f), chamberBlade(-0.92f, -0.05f, -0.4f), chamberPole(0.3f, 0.2f, -0.93f);
            for (AttackLine& line : lines)
            {
                if (line.mName != "backhand")
                    continue;
                line.mWind = { guard, key(0.5f, { -4.0f, 20.0f, -4.0f }, { -0.6f, 0.55f, 0.55f }, { 0.4f, 0.3f, -0.85f }, -20.0f),
                    key(1.0f, chamberHand, chamberBlade, chamberPole, -40.0f) };
                line.mCut = { key(0.0f, chamberHand, chamberBlade, chamberPole, -40.0f),
                    key(0.35f, { -8.0f, 26.0f, -2.0f }, { -0.75f, 0.65f, -0.1f }, { 0.35f, 0.2f, -0.9f }, -20.0f),
                    key(0.65f, { 8.0f, 32.0f, 2.0f }, { 0.75f, 0.65f, 0.0f }, { 0.4f, 0.0f, -0.9f }, 5.0f),
                    key(1.0f, { 14.0f, 28.0f, 6.0f }, { 0.85f, 0.35f, 0.2f }, { 0.3f, -0.1f, -0.95f }, 10.0f) };
            }
            return lines;
        }

        // K2_HAFT_LINES: a long haft's own swings, authored by both hands. A
        // key gives the chest's turn and, in the turned chest's own frame
        // (the shoulders at +-11, 0, 6), the weapon hand and the support
        // hand; the shaft runs from the support hand through the weapon hand
        // to the point. The swing is the staff's push and pull: the hands
        // stay in front of the trunk, about 21 apart, the shaft turns between
        // them, and the chest's turn carries most of the arc (the sword's
        // lines moved one hand round the body and left the support hand
        // wherever the haft put it: haftfit.hpp). The thrust is the sword's.
        ChamberKey haftKey(float s, float twistDegrees, JPH::Vec3 weaponHand, JPH::Vec3 supportHand,
            JPH::Vec3 elbow = JPH::Vec3(0.4f, 0.1f, -0.9f))
        {
            // Negative turns bring the weapon shoulder forward (counter-
            // clockwise from above).
            const float turn = -twistDegrees * (sTau / 360.0f);
            const float c = std::cos(turn), n = std::sin(turn);
            const auto turned = [&](JPH::Vec3 v) {
                return JPH::Vec3(v.GetX() * c - v.GetY() * n, v.GetX() * n + v.GetY() * c, v.GetZ());
            };
            return ChamberKey{ s, turned(weaponHand), turned((weaponHand - supportHand).NormalizedOr(JPH::Vec3::sAxisY())),
                turned(elbow.Normalized()), twistDegrees, JPH::Vec3::sZero() };
        }

        WheelLines makeHaftLines(bool deepChop, bool ownThrust, bool ownGuard)
        {
            WheelLines lines = makeWeaponLines();
            // K2_HAFT_GUARD: the sword's guard runs a haft's butt back past
            // the weapon hip, through the body; HSC_HAFT_CLEAR then turned the
            // weapon arm's command 30 degrees to push the butt out to the
            // weapon side while the support hand was asked to hold the haft
            // on its own side - the arms fought, the guard sat 8-10 degrees
            // off or fell over to the weapon arm's (user 11-12: "the left
            // hand stays grabbing the spear but it is behind the back").
            // The haft's own: the support hand beside its own hip, the butt
            // running back past it.
            // The weapon shoulder a little forward, as in the point guard.
            const ChamberKey guard = ownGuard ? haftKey(0.0f, -20.0f, { 4.0f, 24.0f, -6.0f }, { -8.0f, 11.0f, -17.0f })
                : key(0.0f, { 8.0f, 24.0f, -8.0f }, { 0.05f, 0.75f, 0.66f }, { 0.3f, 0.1f, -0.95f }, 0.0f, { -0.5f, 0.3f, -0.8f });
            if (ownGuard)
                for (AttackLine& line : lines)
                    if (!line.mWind.empty())
                        line.mWind.front() = guard;
            // The places the hands pass through, in the chest's frame: the
            // point out to the right, half right, ahead, a little left, left.
            struct Hands
            {
                JPH::Vec3 mWeapon, mSupport;
            };
            const Hands right{ { 13.7f, 18.1f, -4.0f }, { -6.0f, 11.0f, -4.0f } };
            const Hands halfRight{ { 12.2f, 23.9f, -4.0f }, { -4.0f, 10.5f, -4.0f } };
            const Hands ahead{ { 3.1f, 29.8f, -3.9f }, { 1.0f, 9.0f, -6.0f } };
            const Hands halfLeft{ { 2.4f, 28.6f, -8.0f }, { 6.0f, 8.0f, -8.0f } };
            const Hands left{ { -0.8f, 24.9f, -10.0f }, { 8.0f, 6.0f, -10.0f } };
            const auto at = [](float s, float twist, const Hands& hands) {
                return haftKey(s, twist, hands.mWeapon, hands.mSupport);
            };
            // A line: the wind-up from the guard through `mid` to the cut's
            // first key, then the cut.
            const auto set = [&](std::string_view name, ChamberKey mid, std::vector<ChamberKey> cut) {
                for (AttackLine& line : lines)
                {
                    if (line.mName != name)
                        continue;
                    ChamberKey chamber = cut.front();
                    chamber.mS = 1.0f;
                    mid.mS = 0.5f;
                    line.mWind = { guard, mid, chamber };
                    line.mCut = std::move(cut);
                }
            };
            // An early wind-up key (the guard's support hand is at its arm's
            // end: a wind-up to the left must bring it in first).
            const auto early = [&](std::string_view name, ChamberKey key) {
                for (AttackLine& line : lines)
                    if (line.mName == name)
                        line.mWind.insert(line.mWind.begin() + 1, key);
            };
            // Level sweeps: right to left, and back.
            set("forehand", haftKey(0.5f, 18.0f, { 12.0f, 22.0f, -6.0f }, { -1.0f, 11.0f, -14.0f }),
                { at(0.0f, 35.0f, right), at(0.3f, 15.0f, halfRight), at(0.55f, -10.0f, ahead),
                    at(0.8f, -35.0f, halfLeft), at(1.0f, -45.0f, left) });
            set("backhand", haftKey(0.5f, -22.0f, { 3.0f, 26.0f, -9.0f }, { 8.0f, 7.0f, -15.0f }),
                { at(0.0f, -45.0f, left), at(0.25f, -30.0f, halfLeft), at(0.5f, 0.0f, ahead),
                    at(0.8f, 20.0f, halfRight), at(1.0f, 30.0f, right) });
            early("backhand", haftKey(0.25f, 0.0f, { 6.0f, 25.0f, -8.5f }, { 6.0f, ownGuard ? 11.0f : 9.0f, -16.0f }));
            // Down from above the head.
            set("chop", haftKey(0.5f, 8.0f, { 6.0f, 18.0f, 10.0f }, { 4.0f, 11.0f, -9.0f }),
                { haftKey(0.0f, 15.0f, { 4.1f, 6.75f, 26.2f }, { 2.0f, 12.0f, 6.0f }),
                    haftKey(0.3f, 5.0f, { 3.0f, 23.5f, 20.0f }, { 2.0f, 13.0f, 2.0f }),
                    haftKey(0.55f, -5.0f, { 1.0f, 29.9f, 3.5f }, { 1.0f, 9.5f, -2.0f }),
                    haftKey(0.8f, -10.0f, { 1.0f, 27.8f, -7.5f }, { 1.0f, 9.5f, 3.0f }),
                    haftKey(1.0f, -10.0f, { 1.0f, 24.7f, -10.9f }, { 1.0f, 10.0f, 4.0f }) });
            // K2_HAFT_THRUST: the dragged thrust is the point guard's, not
            // the sword's (whose chamber draws the weapon hand back to the
            // weapon hip: the support hand, 21 behind it on the haft, is
            // asked 18 behind the chest on the weapon side - haftfit - and
            // went there: user 11-12 "left hand keeps going behind the
            // back"). Side-on, the weapon shoulder forward, the weapon hand
            // left of the middle and the shaft level through it: the support
            // hand stays under its own shoulder beside the hip, and the
            // thrust runs down the shaft as the chest turns on.
            if (ownThrust)
                for (AttackLine& line : lines)
                {
                    if (line.mName != "thrust")
                        continue;
                    const JPH::Vec3 shaft(0.08f, 0.995f, 0.05f), elbow(0.3f, 0.1f, -0.95f), down(0.0f, 0.0f, -1.0f);
                    const ChamberKey drawn = key(1.0f, { -9.0f, 13.0f, -6.0f }, shaft, elbow, -35.0f, down);
                    line.mWind = { guard, key(0.5f, { -6.0f, 28.0f, -7.0f }, { 0.1f, 0.9f, 0.42f }, elbow, -20.0f, down), drawn };
                    line.mCut = { drawn, key(0.5f, { -9.0f, 26.0f, -6.0f }, shaft, elbow, -45.0f, down),
                        key(1.0f, { -9.0f, 33.0f, -6.0f }, shaft, elbow, -52.0f, down) };
                    line.mCut.front().mS = 0.0f;
                }
            // K2_HAFT_CHOP: the chop is wound up further - the weapon hand
            // behind the weapon shoulder at the neck's height, the support
            // hand above the brow, the shaft over the shoulder with its point
            // down behind the back (25 degrees below level). It passes
            // through the chamber above the head both ways.
            if (deepChop)
                for (AttackLine& line : lines)
                {
                    if (line.mName != "chop")
                        continue;
                    const ChamberKey over = line.mCut.front();
                    // The weapon arm's elbow out and up (down, the wrist
                    // stood 12 degrees past its limit and the shaft sagged
                    // 16 -> 54 degrees off within a second: CHOP-02).
                    const ChamberKey behind
                        = haftKey(0.0f, 20.0f, { 10.0f, -4.0f, 18.0f }, { 6.0f, 14.0f, 26.0f }, { 0.75f, 0.3f, 0.6f });
                    line.mWind.back() = over;
                    line.mWind.back().mS = 0.7f;
                    line.mWind[line.mWind.size() - 2].mS = 0.4f;
                    line.mWind.push_back(behind);
                    line.mWind.back().mS = 1.0f;
                    line.mCut.front().mS = 0.15f;
                    line.mCut.insert(line.mCut.begin(), behind);
                    // A chain is in by the chamber above the head (0.7 of
                    // the wind-up; a chain runs its outer half).
                    line.mChainSpan = 0.4f;
                }
            // The diagonals: high right to low left, high left to low right,
            // low right to high left, low left to high right.
            set("up-right", haftKey(0.5f, 15.0f, { 10.0f, 18.0f, 6.0f }, { 2.0f, 11.0f, -10.0f }),
                { haftKey(0.0f, 30.0f, { 10.6f, 9.9f, 18.6f }, { -2.0f, 12.0f, 2.0f }),
                    haftKey(0.35f, 10.0f, { 6.4f, 23.6f, 14.5f }, { -2.0f, 11.0f, 0.0f }),
                    haftKey(0.5f, -3.0f, { 3.0f, 28.0f, 6.0f }, { 0.9f, 10.2f, -4.9f }),
                    haftKey(0.65f, -15.0f, { 0.9f, 28.95f, -6.3f }, { 3.0f, 9.0f, 0.0f }),
                    haftKey(1.0f, -35.0f, { -0.3f, 23.75f, -12.4f }, { 6.0f, 8.0f, 0.0f }) });
            set("up-left", haftKey(0.5f, -18.0f, { 3.0f, 20.0f, 6.0f }, { 7.0f, 9.0f, -11.0f }),
                { haftKey(0.0f, -35.0f, { -1.35f, 12.1f, 17.5f }, { 6.0f, 10.0f, -2.0f }),
                    haftKey(0.35f, -15.0f, { 1.9f, 22.6f, 14.6f }, { 4.0f, 10.0f, -2.0f }),
                    haftKey(0.5f, -3.0f, { 3.0f, 28.0f, 6.0f }, { 2.0f, 10.2f, -4.9f }),
                    haftKey(0.65f, 10.0f, { 4.3f, 28.9f, -6.3f }, { -2.0f, 10.0f, 0.0f }),
                    haftKey(1.0f, 30.0f, { 8.6f, 21.55f, -12.2f }, { -4.0f, 10.0f, 0.0f }) });
            set("down-right", haftKey(0.5f, 15.0f, { 12.0f, 22.0f, -14.0f }, { -2.0f, 11.0f, -8.0f }),
                { haftKey(0.0f, 30.0f, { 10.7f, 15.2f, -16.5f }, { -4.0f, 11.0f, -2.0f }),
                    haftKey(0.35f, 10.0f, { 5.4f, 26.8f, -11.45f }, { -3.0f, 10.0f, -2.0f }),
                    haftKey(0.5f, -3.0f, { 3.0f, 29.5f, -4.0f }, { 2.4f, 9.1f, -4.0f }),
                    haftKey(0.65f, -15.0f, { 0.9f, 27.9f, 2.8f }, { 3.0f, 9.0f, -6.0f }),
                    haftKey(1.0f, -35.0f, { -0.3f, 19.5f, 11.0f }, { 6.0f, 9.0f, -6.0f }) });
            set("down-left", haftKey(0.5f, -18.0f, { 3.0f, 22.0f, -14.0f }, { 9.0f, 8.0f, -6.0f }),
                { haftKey(0.0f, -35.0f, { -1.35f, 16.35f, -18.3f }, { 6.0f, 9.0f, 0.0f }),
                    haftKey(0.35f, -15.0f, { 1.9f, 26.85f, -10.9f }, { 4.0f, 9.0f, 0.0f }),
                    haftKey(0.65f, 10.0f, { 4.3f, 27.85f, 3.0f }, { -2.0f, 10.0f, -6.0f }),
                    haftKey(1.0f, 30.0f, { 7.55f, 19.45f, 8.7f }, { -4.0f, 10.0f, -6.0f }) });
            early("down-left", haftKey(0.25f, 0.0f, { 6.5f, 24.0f, -11.0f }, { 8.0f, ownGuard ? 12.0f : 10.0f, -15.0f }));
            // From the haft's guard the support hand comes round the front
            // of the belly, not through it.
            if (ownGuard)
                early("down-left", haftKey(0.12f, -10.0f, { 5.0f, 26.0f, -8.0f }, { -2.0f, 13.0f, -18.0f }));
            return lines;
        }

        // The fist's lines: punches from a boxer's guard at the chin; "blade"
        // is the knuckle line, the pole the elbow. The straights end past
        // the arm's reach on purpose (a punch is thrown through: FIST-01).
        WheelLines makeFistLines()
        {
            const ChamberKey guard = key(0.0f, { 6.0f, 14.0f, 13.0f }, { 0.1f, 0.55f, 0.83f }, { 0.35f, -0.2f, -0.9f }, 0.0f);
            const auto k = [](float u, JPH::Vec3 hand, JPH::Vec3 knuckles, JPH::Vec3 elbow, float twist) {
                return key(u, hand, knuckles.Normalized(), elbow.Normalized(), twist);
            };
            // K2_FIST_WOUND's third value: the uppercut's wind-up this many
            // units higher (0 = at -4). Read once, when the lines are made.
            std::array<float, 3> wound{ 0.0f, 0.0f, 0.0f };
            tuningList("OPENMW_RAGDOLL_K2_FIST_WOUND", wound);
            const float low = -4.0f + (std::isfinite(wound[2]) ? std::clamp(wound[2], 0.0f, 20.0f) : 0.0f);
            return sorted(WheelLines{ {
                { "cross", 0.0f, -1.0f, "thrust",
                    { guard, k(1.0f, { 9.0f, 6.0f, 12.0f }, { 0.0f, 0.6f, 0.8f }, { 0.5f, -0.4f, -0.75f }, 0.0f) },
                    { k(0.0f, { 9.0f, 6.0f, 12.0f }, { 0.0f, 0.6f, 0.8f }, { 0.5f, -0.4f, -0.75f }, 0.0f),
                        k(0.5f, { 5.0f, 28.0f, 14.0f }, { -0.1f, 0.95f, 0.3f }, { 0.6f, -0.1f, -0.75f }, -40.0f),
                        k(1.0f, { 0.0f, 56.0f, 15.0f }, { -0.05f, 1.0f, 0.05f }, { 0.7f, 0.0f, -0.7f }, -85.0f) } },
                { "overhand", 0.0f, 1.0f, "chop",
                    { guard, k(1.0f, { 16.0f, 2.0f, 24.0f }, { 0.2f, 0.2f, 0.95f }, { 0.8f, -0.3f, 0.5f }, -20.0f) },
                    { k(0.0f, { 16.0f, 2.0f, 24.0f }, { 0.2f, 0.2f, 0.95f }, { 0.8f, -0.3f, 0.5f }, -20.0f),
                        k(0.5f, { 10.0f, 26.0f, 24.0f }, { -0.1f, 0.9f, 0.4f }, { 0.8f, 0.1f, 0.6f }, -60.0f),
                        k(1.0f, { 2.0f, 50.0f, 12.0f }, { -0.1f, 0.9f, -0.4f }, { 0.75f, 0.2f, 0.6f }, -90.0f) } },
                { "hook", 1.0f, 0.0f, "slash",
                    { guard, k(1.0f, { 22.0f, 8.0f, 13.0f }, { 0.4f, 0.7f, 0.6f }, { 0.9f, -0.4f, -0.1f }, -10.0f) },
                    { k(0.0f, { 22.0f, 8.0f, 13.0f }, { 0.4f, 0.7f, 0.6f }, { 0.9f, -0.4f, -0.1f }, -10.0f),
                        k(0.45f, { 18.0f, 30.0f, 14.0f }, { -0.5f, 0.85f, 0.1f }, { 0.95f, 0.0f, 0.3f }, -80.0f),
                        k(1.0f, { -6.0f, 34.0f, 14.0f }, { -0.95f, 0.3f, 0.0f }, { 0.6f, 0.4f, 0.7f }, -90.0f) } },
                { "backfist", -1.0f, 0.0f, "slash",
                    { guard, k(1.0f, { -6.0f, 12.0f, 14.0f }, { -0.6f, 0.2f, 0.75f }, { 0.4f, -0.5f, -0.75f }, 30.0f) },
                    { k(0.0f, { -6.0f, 12.0f, 14.0f }, { -0.6f, 0.2f, 0.75f }, { 0.4f, -0.5f, -0.75f }, 30.0f),
                        k(0.5f, { 4.0f, 32.0f, 15.0f }, { 0.2f, 0.95f, 0.2f }, { 0.3f, -0.2f, -0.93f }, 60.0f),
                        k(1.0f, { 20.0f, 36.0f, 14.0f }, { 0.9f, 0.4f, 0.1f }, { 0.2f, -0.3f, -0.93f }, 80.0f) } },
                { "uppercut", 1.0f, -1.0f, "thrust",
                    { guard, k(1.0f, { 12.0f, 10.0f, low }, { 0.1f, 0.4f, 0.9f }, { 0.4f, -0.4f, -0.8f }, 70.0f) },
                    { k(0.0f, { 12.0f, 10.0f, low },{ 0.1f, 0.4f, 0.9f }, { 0.4f, -0.4f, -0.8f }, 70.0f),
                        k(0.5f, { 7.0f, 24.0f, 8.0f }, { 0.0f, 0.3f, 0.95f }, { 0.3f, -0.1f, -0.95f }, 80.0f),
                        k(1.0f, { 3.0f, 30.0f, 22.0f }, { -0.1f, 0.1f, 1.0f }, { 0.3f, 0.2f, -0.93f }, 85.0f) } },
                { "body", -1.0f, -1.0f, "thrust",
                    { guard, k(1.0f, { 10.0f, 6.0f, 4.0f }, { 0.0f, 0.7f, 0.7f }, { 0.5f, -0.5f, -0.7f }, 0.0f) },
                    { k(0.0f, { 10.0f, 6.0f, 4.0f }, { 0.0f, 0.7f, 0.7f }, { 0.5f, -0.5f, -0.7f }, 0.0f),
                        k(0.5f, { 6.0f, 26.0f, -2.0f }, { -0.1f, 0.95f, 0.1f }, { 0.6f, -0.1f, -0.8f }, -45.0f),
                        k(1.0f, { 2.0f, 52.0f, -8.0f }, { -0.05f, 1.0f, -0.1f }, { 0.7f, 0.0f, -0.7f }, -85.0f) } },
                { "long cross", 1.0f, 1.0f, "thrust",
                    { guard, k(1.0f, { 12.0f, 2.0f, 16.0f }, { 0.2f, 0.4f, 0.9f }, { 0.6f, -0.5f, -0.6f }, 0.0f) },
                    { k(0.0f, { 12.0f, 2.0f, 16.0f }, { 0.2f, 0.4f, 0.9f }, { 0.6f, -0.5f, -0.6f }, 0.0f),
                        k(0.5f, { 6.0f, 28.0f, 17.0f }, { -0.1f, 0.95f, 0.3f }, { 0.7f, -0.1f, -0.7f }, -45.0f),
                        k(1.0f, { 0.0f, 60.0f, 17.0f }, { -0.05f, 1.0f, 0.05f }, { 0.7f, 0.0f, -0.7f }, -85.0f) } },
                { "overhand hook", -1.0f, 1.0f, "chop",
                    { guard, k(1.0f, { 20.0f, 4.0f, 22.0f }, { 0.4f, 0.3f, 0.85f }, { 0.9f, -0.3f, 0.3f }, -20.0f) },
                    { k(0.0f, { 20.0f, 4.0f, 22.0f }, { 0.4f, 0.3f, 0.85f }, { 0.9f, -0.3f, 0.3f }, -20.0f),
                        k(0.5f, { 14.0f, 28.0f, 20.0f }, { -0.4f, 0.85f, 0.2f }, { 0.9f, 0.1f, 0.5f }, -70.0f),
                        k(1.0f, { -4.0f, 36.0f, 12.0f }, { -0.9f, 0.3f, -0.2f }, { 0.6f, 0.4f, 0.7f }, -90.0f) } },
            } });
        }
    }

    float AttackLine::angle() const
    {
        const float t = std::atan2(mDragUp, mDragRight);
        return t < -1e-3f ? t + sTau : t;
    }

    const WheelLines& weaponLines()
    {
        static const WheelLines lines = makeWeaponLines();
        return lines;
    }

    const WheelLines& spearLines()
    {
        static const WheelLines lines = makeSpearLines();
        return lines;
    }

    const WheelLines& haftLines(bool deepChop, bool ownThrust, bool ownGuard)
    {
        static const std::array<WheelLines, 8> lines{ makeHaftLines(false, false, false), makeHaftLines(true, false, false),
            makeHaftLines(false, true, false), makeHaftLines(true, true, false), makeHaftLines(false, false, true),
            makeHaftLines(true, false, true), makeHaftLines(false, true, true), makeHaftLines(true, true, true) };
        return lines[(deepChop ? 1 : 0) + (ownThrust ? 2 : 0) + (ownGuard ? 4 : 0)];
    }

    const WheelLines& fistLines()
    {
        static const WheelLines lines = makeFistLines();
        return lines;
    }

    std::optional<std::vector<JPH::Quat>> routeRotations(const std::vector<ChamberKey>& keys,
        const WeaponStrikeAxes& axes, bool towardEnd, const std::optional<JPH::Quat>& last)
    {
        std::vector<JPH::Quat> rotations(keys.size());
        for (std::size_t i = 0; i < keys.size(); ++i)
        {
            std::optional<JPH::Quat> rotation;
            if (i + 1 == keys.size() && last)
                rotation = last;
            else
            {
                const JPH::Vec3 blade = keys[i].mBlade.Normalized();
                const JPH::Vec3 before = keys[i > 0 ? i - 1 : 0].mBlade.Normalized();
                const JPH::Vec3 after = keys[i + 1 < keys.size() ? i + 1 : i].mBlade.Normalized();
                const JPH::Vec3 moving
                    = keys[i].mEdge.IsNearZero() ? (towardEnd ? after - before : before - after) : keys[i].mEdge;
                rotation = alignedStrikeHandRotation(axes, blade, moving);
            }
            if (!rotation)
                return std::nullopt;
            rotations[i] = rotation->Normalized();
            if (i > 0 && rotations[i].Dot(rotations[i - 1]) < 0.0f)
                rotations[i] = -rotations[i];
        }
        return rotations;
    }

    ChamberPose routePose(const std::vector<ChamberKey>& keys, const std::vector<JPH::Quat>& rotations, float s)
    {
        ChamberPose pose;
        if (keys.size() < 2 || rotations.size() != keys.size())
        {
            if (!keys.empty())
            {
                pose.mHand = keys.front().mHand;
                pose.mPole = keys.front().mPole.NormalizedOr(pose.mPole);
                pose.mTwist = keys.front().mTwistDegrees * (sPi / 180.0f);
                if (!rotations.empty())
                    pose.mRotation = rotations.front();
            }
            return pose;
        }
        s = std::clamp(s, keys.front().mS, keys.back().mS);
        std::size_t i = 0;
        while (i + 2 < keys.size() && s > keys[i + 1].mS)
            ++i;
        const float t = (s - keys[i].mS) / std::max(1e-4f, keys[i + 1].mS - keys[i].mS);
        const auto at = [&](std::ptrdiff_t k) -> std::size_t {
            return static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(k, 0, static_cast<std::ptrdiff_t>(keys.size()) - 1));
        };
        const std::ptrdiff_t index = static_cast<std::ptrdiff_t>(i);
        const std::size_t i0 = at(index - 1), i1 = at(index), i2 = at(index + 1), i3 = at(index + 2);
        const float t2 = t * t, t3 = t2 * t;
        const auto spline = [&](auto p0, auto p1, auto p2, auto p3) {
            return (p1 * 2.0f + (p2 - p0) * t + (p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3) * t2
                       + (p1 * 3.0f - p0 - p2 * 3.0f + p3) * t3)
                * 0.5f;
        };
        pose.mHand = spline(keys[i0].mHand, keys[i1].mHand, keys[i2].mHand, keys[i3].mHand);
        pose.mTwist = spline(keys[i0].mTwistDegrees, keys[i1].mTwistDegrees, keys[i2].mTwistDegrees,
                          keys[i3].mTwistDegrees)
            * (sPi / 180.0f);
        pose.mPole = spline(keys[i0].mPole, keys[i1].mPole, keys[i2].mPole, keys[i3].mPole).NormalizedOr(keys[i1].mPole);
        const JPH::Vec4 r = spline(rotations[i0].GetXYZW(), rotations[i1].GetXYZW(), rotations[i2].GetXYZW(),
            rotations[i3].GetXYZW());
        pose.mRotation = r.Length() > 1e-4f ? JPH::Quat(r).Normalized() : rotations[i1];
        return pose;
    }

    ChamberPose blendPose(const ChamberPose& a, const ChamberPose& b, float w)
    {
        ChamberPose p;
        p.mHand = a.mHand + (b.mHand - a.mHand) * w;
        const JPH::Vec3 poleFrom = a.mPole.NormalizedOr(JPH::Vec3::sAxisZ());
        const JPH::Vec3 poleTo = b.mPole.NormalizedOr(poleFrom);
        const float cosine = std::clamp(poleFrom.Dot(poleTo), -1.0f, 1.0f);
        JPH::Vec3 axis = poleFrom.Cross(poleTo);
        if (axis.LengthSq() < 1e-8f)
        {
            axis = poleFrom.Cross(JPH::Vec3::sAxisY());
            if (axis.LengthSq() < 1e-8f)
                axis = poleFrom.Cross(JPH::Vec3::sAxisX());
        }
        p.mPole = cosine > 0.9999f
            ? (poleFrom * (1.0f - w) + poleTo * w).NormalizedOr(poleFrom)
            : (JPH::Quat::sRotation(axis.Normalized(), std::acos(cosine) * w) * poleFrom).Normalized();
        const JPH::Quat to = a.mRotation.Dot(b.mRotation) < 0.0f ? -b.mRotation : b.mRotation;
        p.mRotation = a.mRotation.SLERP(to, w).Normalized();
        p.mTwist = a.mTwist + (b.mTwist - a.mTwist) * w;
        return p;
    }

    std::optional<ChamberPose> linePose(const AttackLine& line, const WeaponStrikeAxes& axes, float u)
    {
        const auto cutRotations = routeRotations(line.mCut, axes, true, std::nullopt);
        if (!cutRotations || cutRotations->empty())
            return std::nullopt;
        if (u > 1.0f)
            return routePose(line.mCut, *cutRotations, u - 1.0f);
        const auto windRotations = routeRotations(line.mWind, axes, false, cutRotations->front());
        if (!windRotations)
            return std::nullopt;
        return routePose(line.mWind, *windRotations, std::max(0.0f, std::min(u, 1.0f)));
    }

    WheelSlot wheelSlot(float angle)
    {
        const float slot = wrapAngle(angle) / sEighth;
        const float base = std::floor(slot);
        WheelSlot result;
        result.mFirst = static_cast<std::size_t>(base) % 8;
        result.mSecond = (result.mFirst + 1) % 8;
        result.mWeight = std::clamp(slot - base, 0.0f, 1.0f);
        return result;
    }

    std::optional<ChamberPose> wheelPose(const WheelLines& lines, const WeaponStrikeAxes& axes, float angle, float u)
    {
        const WheelSlot slot = wheelSlot(angle);
        const auto a = linePose(lines[slot.mFirst], axes, u);
        const auto b = linePose(lines[slot.mSecond], axes, u);
        if (!a || !b)
            return std::nullopt;
        return blendPose(*a, *b, slot.mWeight);
    }

    std::optional<ChamberPose> guardPose(const WheelLines& lines, const WeaponStrikeAxes& axes)
    {
        const auto rotations = routeRotations(lines.front().mWind, axes, false, std::nullopt);
        if (!rotations)
            return std::nullopt;
        return routePose(lines.front().mWind, *rotations, 0.0f);
    }

    Pace attackPace(const PaceInput& input)
    {
        // HSC_ATTACK_SPEED 1.5 (two-handers 1.3: at 1.5 a claymore lost its
        // support grip, GRIP-01), the player x1.15 / x1.05 (HSC_PLAYER_SPEED).
        const float playerSpeed = input.mPlayer ? (input.mTwoHanded ? 1.05f : 1.15f) : 1.0f;
        Pace pace;
        const float response = std::clamp(input.mResponse, 0.25f, 4.0f);
        pace.mAttackSpeed = (input.mTwoHanded ? 1.3f : 1.5f) * playerSpeed * response;
        pace.mCarried = std::clamp(input.mWeight * 0.08f, 0.4f, 4.0f);
        const float weightPace = std::sqrt(pace.mCarried / 1.6f) / pace.mAttackSpeed;
        pace.mWeightPace = weightPace;
        // HSC_RIPOSTE wind share 0.5, HSC_CHAIN_FLOW wind share 0.4.
        const float flow = std::clamp(input.mChainFlowGain, 0.0f, 1.0f);
        const float combo = (input.mPlayerRiposte ? 0.5f : 1.0f) * (1.0f - 0.4f * flow);
        // NPCs open (NPC_PACE 1.6, 0.15, 0.7, 1.5); a riposte winds up
        // AI_RIPOSTE[3] (0.5) as long and holds no chamber.
        const float npcWind = input.mPlayer ? 1.0f : 1.6f * (input.mNpcRiposte ? 0.5f : 1.0f);
        // The player's fists and daggers (HSC_SHORT_PACE 0.12, 0.15, 0.35, 0.5).
        const bool playerShort = input.mShort && input.mPlayer;
        pace.mWind = combo * (playerShort ? 0.12f / (playerSpeed * response) : 0.35f * weightPace * npcWind);
        pace.mCut = playerShort ? 0.15f / (playerSpeed * response) : 0.25f * weightPace;
        pace.mChamberHold = !input.mPlayer && !input.mNpcRiposte ? 0.15f : 0.0f;
        pace.mReleaseHold
            = std::max(input.mShort ? 0.35f : input.mTwoHanded ? 1.0f : 0.3f, input.mPlayer ? 0.0f : 0.7f);
        // HSC_RETURN 0.4 / 0.6, x1.5 for NPCs, up to x1.2 at no control
        // (HSC_RETURN_SKILL 0.2).
        const float loss = std::clamp((1.0f - input.mControl) / 0.8f, 0.0f, 1.0f);
        pace.mReturn = (input.mShort ? 0.5f : input.mTwoHanded ? 0.6f : 0.4f) * (input.mPlayer ? 1.0f : 1.5f)
            * (1.0f + 0.2f * loss) / response;
        pace.mWindHaste = 1.0f / std::max(0.2f, combo);
        // K2_DAGGER_THRUST (bookmark 11-05: "almost not even cap how fast").
        if (input.mDaggerThrust && input.mPlayer)
        {
            pace.mWind = combo * 0.04f / response;
            pace.mCut = 0.07f / response;
            pace.mReturn = 0.16f / response;
            pace.mReleaseHold = 0.08f;
            // Not the route's haste: at 3x (9x its acceleration) a held wind-up's
            // hand chattered round its target (play 11-06: "the hand starts
            // vibrating"; the commanded tip reversed on 45% of wind-up steps
            // against 29-32%).
        }
        // K2_SPEAR_THRUST_PACE (user 11-16: "almost like dagger (but the speed
        // should be slower than dagger)").
        else if (input.mSpearThrustWind > 0.0f && input.mPlayer && !input.mShort)
        {
            pace.mWind = std::min(pace.mWind, combo * input.mSpearThrustWind / response);
            pace.mCut = std::min(pace.mCut, std::max(0.05f, input.mSpearThrustCut) / response);
            pace.mReturn = std::min(pace.mReturn, std::max(0.05f, input.mSpearThrustReturn) / response);
            pace.mReleaseHold = std::min(pace.mReleaseHold, 0.2f);
        }
        // K2_MOMENTUM_WIND: the body moving along the coming cut.
        const float haste = std::isfinite(input.mMomentumHaste) ? std::clamp(input.mMomentumHaste, 1.0f, 3.0f) : 1.0f;
        pace.mWind /= haste;
        pace.mWindHaste *= haste;
        return pace;
    }

    float pacedWeaponWeight(float weight, PacedWeaponKind kind, float floorShare)
    {
        if (!std::isfinite(weight))
            return weight;
        float iron = 0.0f;
        switch (kind)
        {
            case PacedWeaponKind::ShortBlade:
                iron = 3.0f;
                break;
            case PacedWeaponKind::LongBlade:
                iron = 20.0f;
                break;
            case PacedWeaponKind::LongBladeTwoHand:
                iron = 27.0f;
                break;
            case PacedWeaponKind::BluntOneHand:
                iron = 15.0f;
                break;
            case PacedWeaponKind::BluntTwoClose:
                iron = 32.0f;
                break;
            case PacedWeaponKind::BluntTwoWide:
                iron = 8.0f;
                break;
            case PacedWeaponKind::Spear:
                iron = 14.0f;
                break;
            case PacedWeaponKind::AxeOneHand:
                iron = 24.0f;
                break;
            case PacedWeaponKind::AxeTwoHand:
                iron = 30.0f;
                break;
            case PacedWeaponKind::Other:
                break;
        }
        const float share = std::isfinite(floorShare) ? std::clamp(floorShare, 0.0f, 1.0f) : 0.0f;
        return std::max(weight, iron * share);
    }

    void stepRouteFlow(RouteFlow& flow, float target, float dt, const Pace& pace, float haste)
    {
        if (!(dt > 0.0f))
            return;
        const float carried = std::clamp(pace.mCarried, 0.4f, 4.0f);
        const float top = 5.0f * pace.mAttackSpeed * std::sqrt(1.6f / carried) * haste;
        const float accel = 40.0f * pace.mAttackSpeed * pace.mAttackSpeed * (1.6f / carried) * haste * haste;
        const float remaining = target - flow.mU;
        const float brake = std::sqrt(2.0f * accel * std::abs(remaining));
        const float wanted = std::copysign(std::min(top, brake), remaining);
        const float step = accel * dt;
        float nextV = flow.mV + std::clamp(wanted - flow.mV, -step, step);
        // Land, don't overshoot: a step that would carry past the target ends
        // on it (play 11-06, the dagger's wind-up "vibrating": following a
        // drag, the route overshot and came back on ~30% of steps - a light
        // weapon's acceleration crossing the target in one step; offline
        // 28 reversals in 4 s for a dagger, 77 for a claymore, 1 with this).
        if (std::abs(nextV) * dt > std::abs(remaining) && remaining * nextV > 0.0f)
            nextV = remaining / dt;
        float nextU = flow.mU + 0.5f * (flow.mV + nextV) * dt;
        if (std::abs(remaining) < 1e-3f && std::abs(nextV) < step)
        {
            nextU = target;
            flow.mV = 0.0f;
        }
        else if (std::abs(remaining) < 0.03f && std::abs(flow.mV) < 1.0f)
        {
            const float k = 1.0f - std::exp(-dt * 25.0f);
            nextU = flow.mU + remaining * k;
            flow.mV = remaining * k / dt;
        }
        else
            flow.mV = nextV;
        flow.mU = std::clamp(nextU, 0.0f, 2.0f);
    }

    float windAt(float rn, float stanceRadius)
    {
        if (rn <= stanceRadius)
            return 0.5f * std::max(rn, 0.0f) / stanceRadius;
        const float x = std::min(1.0f, (rn - stanceRadius) / sCommitCounts);
        const float k = std::max(1.0f, sCommitCounts / stanceRadius);
        return 0.5f + 0.5f * (1.0f - std::pow(1.0f - x, k));
    }
}

#ifndef OPENMW_MANUALDIAGNOSTICS_H
#define OPENMW_MANUALDIAGNOSTICS_H

// Read-only manual-play observations. The recorder never owns gameplay state.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include <osg/Vec3f>
#include <osg/Quat>

namespace ManualDiagnostics
{
    inline thread_local bool syntheticInput = false;
    struct SyntheticInputScope
    {
        bool previous = syntheticInput;
        SyntheticInputScope() { syntheticInput = true; }
        ~SyntheticInputScope() { syntheticInput = previous; }
    };
    struct Body
    {
        std::string name;
        int parent = -1;
        osg::Vec3f animation, command, physical;
        osg::Quat animationRotation, commandRotation, physicalRotation;
        float utilization = -1;
        bool animationValid = false, commandValid = false, physicalValid = false;
    };
    struct Frame
    {
        std::uint64_t tick = 0, actor = 0, attack = 0, press = 0;
        double dt = 0;
        int phase = 0;
        bool held = false;
        std::string weapon;
        std::vector<Body> bodies;
        std::array<osg::Vec3f, 2> targets{};
        std::array<float, 2> handError{ -1, -1 }, grip{ -1, -1 };
        osg::Vec3f weaponAxis;
        float weaponLength = 0;
        // PLAY-04 gate inputs: what the player asked (movement keys, eye ray)
        // and where the weapon tip is (physical, commanded), per tick.
        osg::Vec3f move, actorPosition, eyeOrigin, eyeDirection, aimPoint, tip, tipCommand;
        float actorYaw = 0;
        bool firstPerson = false, aimValid = false, tipValid = false, tipCommandValid = false;
        int drawState = 0, tipMode = -1;
        bool driveActive = false;
        std::string support;
        std::array<float, 2> wristExcess{ -1, -1 };
        // Whole-body state (2026-09-29, a stuck lean in play could not be
        // told apart from the arms-only rows): controller mode and time in
        // it, soft root strength/force/position error, pelvis off its
        // anchor, Footfall pinned/swinging per foot.
        std::string body;
        // The nearest powered NPC within 1500 units (2026-09-29: an NPC's
        // fall into the stairs could not be read from the player's rows): id,
        // mode, time in it, pelvis, and how far its deepest body centre lies
        // under the floor straight above it (negative = above).
        std::string npc;
        // Hand control (2026-10-04, the mouse visualizer): the stance point,
        // stage, line, route position, chain and cut state, and the
        // commanded grip, blade and elbow pole; a JSON object, empty = none.
        std::string hsc;
    };
    struct Status
    {
        Frame frame;
        std::string state = "disabled", path;
        std::uint64_t bookmark = 0, bookmarkTick = 0, bytes = 0;
        int mode = 0;
    };
    inline void vector(std::ostream& out, const osg::Vec3f& v)
    {
        out << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
    }
    inline void rotation(std::ostream& out, const osg::Quat& q)
    {
        out << '[' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ']';
    }
    inline void jsonString(std::ostream& out, const std::string& value)
    {
        constexpr char hex[] = "0123456789abcdef";
        out << '"';
        for (unsigned char c : value)
        {
            if (c == '"' || c == '\\') out << '\\' << static_cast<char>(c);
            else if (c < 32) out << "\\u00" << hex[c >> 4] << hex[c & 15];
            else out << static_cast<char>(c);
        }
        out << '"';
    }
    class Recorder
    {
        std::mutex mMutex;
        Status mStatus;
        std::ofstream mFile;
        unsigned mChunk = 0, mButtons = 0;
        double mDx = 0, mDy = 0;
        std::uint64_t mChunkBytes = 0, mSequence = 0;
        const std::chrono::steady_clock::time_point mStart = std::chrono::steady_clock::now();
        void write(const std::string& row, bool flush = false)
        {
            if (mStatus.state != "recording") return;
            if (mStatus.bytes + row.size() + 1 > 1024ull * 1024 * 1024)
            {
                mStatus.state = "STOPPED: 1 GiB session limit";
                mFile.flush();
                return;
            }
            if (!mFile.is_open() || mChunkBytes + row.size() + 1 > 64ull * 1024 * 1024)
            {
                mFile.close();
                const auto path = std::filesystem::u8path(mStatus.path)
                    / ("ticks-" + std::to_string(mChunk++) + ".jsonl");
                // Each launcher provides a fresh directory; refuse stale output.
                std::error_code error;
                if (std::filesystem::exists(path, error) || error)
                {
                    mStatus.state = "ERROR: output already exists";
                    return;
                }
                mFile.open(path, std::ios::binary);
                mChunkBytes = 0;
            }
            mFile << row << '\n';
            if (flush) mFile.flush();
            if (!mFile) { mStatus.state = "ERROR: recording write failed"; return; }
            mStatus.bytes += row.size() + 1;
            mChunkBytes += row.size() + 1;
        }
        std::ostringstream row(const char* kind)
        {
            std::ostringstream out;
            out.imbue(std::locale::classic());
            out << std::setprecision(9) << "{\"kind\":\"" << kind << "\",\"seq\":" << ++mSequence
                << ",\"wall_ms\":" << std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - mStart).count();
            return out;
        }
    public:
        Recorder()
        {
            const char* path = std::getenv("OPENMW_MANUAL_DIAGNOSTICS");
            if (!path || !*path) return;
            mStatus.path = path;
            std::error_code error;
            std::filesystem::create_directories(std::filesystem::u8path(path), error);
            mStatus.state = error ? "ERROR: cannot create recording directory" : "recording";
            write("{\"kind\":\"header\",\"schema\":1,\"task\":\"PLAY-04\",\"coordinates\":\"world\","
                "\"quaternion\":\"xyzw\",\"unavailable_scalar\":-1,\"replay\":false}", true);
        }
        bool enabled() const { return !mStatus.path.empty(); }
        void mouse(double dx, double dy, int button, bool down, bool look, bool gui)
        {
            if (!enabled()) return;
            std::scoped_lock lock(mMutex);
            if (button > 0 && button <= 8)
            {
                const unsigned bit = 1u << (button - 1);
                if (down) mButtons |= bit; else mButtons &= ~bit;
            }
            mDx += dx; mDy += dy;
            auto out = row("input");
            out << ",\"after_tick\":" << mStatus.frame.tick << ",\"dx\":" << dx << ",\"dy\":" << dy
                << ",\"button\":" << button << ",\"down\":" << down << ",\"buttons\":" << mButtons
                << ",\"look\":" << look << ",\"gui\":" << gui
                << ",\"synthetic_handler\":" << syntheticInput << '}';
            write(out.str());
        }
        void contact(std::uint64_t tick, std::uint64_t actor, std::uint64_t target,
            double speed, bool eligible, bool selected, const osg::Vec3f& point)
        {
            if (!enabled()) return;
            std::scoped_lock lock(mMutex);
            auto out = row("contact");
            out << ",\"tick\":" << tick << ",\"actor\":" << actor << ",\"target\":" << target
                << ",\"normal_speed\":" << speed << ",\"eligible\":" << eligible
                << ",\"selected\":" << selected << ",\"point\":";
            vector(out, point); out << '}'; write(out.str());
        }
        void frame(Frame value)
        {
            if (!enabled()) return;
            std::scoped_lock lock(mMutex);
            mStatus.frame = std::move(value);
            const auto& f = mStatus.frame;
            auto out = row("tick");
            out << ",\"tick\":" << f.tick << ",\"actor\":" << f.actor << ",\"dt\":" << f.dt
                << ",\"attack\":" << f.attack << ",\"press\":" << f.press << ",\"phase\":" << f.phase
                << ",\"held\":" << f.held << ",\"weapon\":" << std::quoted(f.weapon)
                << ",\"mouse\":[" << mDx << ',' << mDy << ',' << mButtons << "],\"hand_error\":["
                << f.handError[0] << ',' << f.handError[1] << "],\"grip\":[" << f.grip[0] << ',' << f.grip[1]
                << "],\"targets\":[";
            vector(out, f.targets[0]); out << ','; vector(out, f.targets[1]);
            out << "],\"weapon_axis\":"; vector(out, f.weaponAxis);
            out << ",\"weapon_length\":" << f.weaponLength << ",\"move\":"; vector(out, f.move);
            out << ",\"actor_pos\":"; vector(out, f.actorPosition);
            out << ",\"actor_yaw\":" << f.actorYaw << ",\"first_person\":" << f.firstPerson
                << ",\"draw\":" << f.drawState << ",\"eye\":"; vector(out, f.eyeOrigin);
            out << ",\"eye_dir\":"; vector(out, f.eyeDirection);
            out << ",\"aim_valid\":" << f.aimValid << ",\"aim\":"; vector(out, f.aimPoint);
            out << ",\"tip_valid\":[" << f.tipValid << ',' << f.tipCommandValid << "],\"tip\":"; vector(out, f.tip);
            out << ",\"tip_cmd\":"; vector(out, f.tipCommand);
            out << ",\"drive\":" << f.driveActive << ",\"tip_mode\":" << f.tipMode << ",\"support\":";
            jsonString(out, f.support);
            out << ",\"wrist_excess\":[" << f.wristExcess[0] << ',' << f.wristExcess[1] << "],\"body\":";
            jsonString(out, f.body);
            out << ",\"npc\":";
            jsonString(out, f.npc);
            out << ",\"hsc\":" << (f.hsc.empty() ? std::string("null") : f.hsc);
            out << ",\"bodies\":[";
            bool first = true;
            for (const auto& b : f.bodies)
            {
                if (!first) out << ',';
                first = false;
                out << "{\"name\":" << std::quoted(b.name) << ",\"parent\":" << b.parent
                    << ",\"util\":" << b.utilization << ",\"valid\":[" << b.animationValid << ','
                    << b.commandValid << ',' << b.physicalValid << "],\"animation\":";
                vector(out, b.animation); out << ",\"command\":"; vector(out, b.command);
                out << ",\"physical\":"; vector(out, b.physical);
                out << ",\"animation_q\":"; rotation(out, b.animationRotation);
                out << ",\"command_q\":"; rotation(out, b.commandRotation);
                out << ",\"physical_q\":"; rotation(out, b.physicalRotation); out << '}';
            }
            out << "]}";
            write(out.str(), f.tick % 60 == 0);
            mDx = mDy = 0;
        }
        Status status()
        {
            std::scoped_lock lock(mMutex);
            return mStatus;
        }
        Status control(bool bookmark)
        {
            std::scoped_lock lock(mMutex);
            if (bookmark && mStatus.state == "recording")
            {
                ++mStatus.bookmark;
                mStatus.bookmarkTick = mStatus.frame.tick;
                auto out = row("bookmark");
                out << ",\"id\":" << mStatus.bookmark << ",\"tick\":" << mStatus.bookmarkTick << '}';
                write(out.str(), true);
            }
            else if (!bookmark) mStatus.mode = (mStatus.mode + 1) % 4; // off, vanilla, targets, reference
            return mStatus;
        }
        bool annotate(std::uint64_t id, const std::string& reason)
        {
            std::scoped_lock lock(mMutex);
            // The modal editor owns the latest bookmark; never attach stale text to a new one.
            if (!id || id != mStatus.bookmark || reason.size() > 8192 || mStatus.state != "recording")
                return false;
            auto out = row("bookmark_note");
            out << ",\"id\":" << id << ",\"tick\":" << mStatus.bookmarkTick << ",\"reason\":";
            jsonString(out, reason);
            out << '}';
            write(out.str(), true);
            return mStatus.state == "recording";
        }
    };
    inline Recorder& recorder() { static Recorder value; return value; }
}
#endif

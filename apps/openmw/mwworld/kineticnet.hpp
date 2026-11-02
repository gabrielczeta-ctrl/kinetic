#ifndef OPENMW_MWWORLD_KINETICNET_H
#define OPENMW_MWWORLD_KINETICNET_H

// NET-01 (docs/ragdoll/designs/NET-01-duel.md): the link of a 1v1 duel
// between two games over UDP. Each game simulates its own player; the other
// player is a puppet body that replays the poses sent here. A blow is decided
// by the attacker's game and sent to the victim's, which pays it.
//
// No game types here: the physics and Lua sides convert to and from these
// plain records. Thread-safe (Lua scripts may call from their worker).

#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace MWWorld
{
    struct NetBody
    {
        std::array<float, 3> mPosition{};
        std::array<float, 4> mRotation{ 0, 0, 0, 1 }; // x, y, z, w
    };

    // One physics step of the sender's player body (world space).
    struct NetPose
    {
        std::uint32_t mStep = 0;
        std::array<float, 3> mActor{};
        bool mGuard = false;
        std::array<float, 2> mGuardForward{ 0, 1 };
        std::vector<NetBody> mBodies;
    };

    // The RPG side of a blow the attacker's game priced: the victim's game
    // runs it through its own armour (the Lua combat handler).
    struct NetHit
    {
        float mDamage = 0;
        bool mHealth = true;
        bool mSuccessful = true;
        std::int8_t mType = -1;
        float mStrength = 1;
        std::array<float, 3> mHitPos{};
    };

    // The physical side of the same blow (RagdollCore::Blow without ids).
    struct NetBlow
    {
        std::uint8_t mKind = 0;
        std::uint8_t mRegion = 0;
        std::uint32_t mBodyIndex = 0;
        std::array<float, 3> mPoint{};
        std::array<float, 3> mDirection{};
        std::array<float, 3> mAttackerToVictim{};
        float mApproachSpeed = 0;
        float mCarriedMass = 0;
        float mDamage = 0;
        float mReactionImpulse = 0;
        bool mBlocked = false;
        bool mParried = false;
        bool mGuardBeaten = false;
    };

    class KineticNet
    {
    public:
        enum class State
        {
            Off,
            Waiting, // hosting, no opponent yet
            Connecting, // joining, no answer yet
            Connected,
            Lost, // the opponent went silent; hosts wait again, joiners retry
        };

        static KineticNet& get();
        ~KineticNet();

        // OPENMW_KINETIC_DUEL: "host" or "host:PORT" hosts, "ADDRESS" or
        // "ADDRESS:PORT" joins (port 27015). OPENMW_KINETIC_DUEL_NAME names
        // this side. False when unset or the socket failed (lastError).
        bool startFromEnvironment();
        bool host(std::uint16_t port);
        bool join(const std::string& address, std::uint16_t port);
        void close();
        // Receives, resends, keeps alive. Once a rendered frame.
        void update(float dt);

        State state() const;
        static const char* stateName(State state);
        bool isHost() const;
        bool isActive() const { return state() != State::Off; }
        float pingMs() const;
        std::string lastError() const;
        std::string remoteName() const;
        std::string localName() const;
        std::string endpoint() const;
        std::uint16_t port() const;
        // Counts connections: a new value is a new opponent (or the same one
        // after a restart) and the duel scripts start over.
        std::uint32_t session() const;

        // The pose stream: unreliable, newest wins.
        void sendPose(const NetPose& pose);
        std::vector<NetPose> takePoses();
        // Reliable and in order.
        void sendHit(const NetHit& hit);
        void sendBlow(const NetBlow& blow);
        void sendMessage(const std::string& text);
        std::vector<NetHit> takeHits();
        std::vector<NetBlow> takeBlows();
        std::vector<std::string> takeMessages();

        static constexpr std::uint16_t sDefaultPort = 27015;

    private:
        KineticNet();
        struct Socket;
        struct Outgoing
        {
            std::vector<std::uint8_t> mPacket;
            double mLastSent = -1;
        };

        bool openSocket(std::uint16_t port, bool bindPort);
        void sendRaw(const std::vector<std::uint8_t>& packet);
        void sendReliable(std::uint8_t type, std::vector<std::uint8_t> payload);
        void receive(const std::uint8_t* data, std::size_t size, const void* from, double now);
        void deliver(std::uint8_t type, const std::uint8_t* data, std::size_t size);
        void connected(double now);
        void resetLink();
        static double now();

        mutable std::mutex mMutex;
        std::unique_ptr<Socket> mSocket;
        State mState = State::Off;
        bool mHost = false;
        std::string mError;
        std::string mLocalName = "Player";
        std::string mRemoteName;
        std::string mEndpoint;
        std::uint16_t mPort = sDefaultPort;
        double mLastHeard = 0;
        double mLastHello = -1;
        double mLastPing = -1;
        float mPing = -1;
        std::uint32_t mSession = 0;

        std::uint32_t mNextReliable = 1;
        std::map<std::uint32_t, Outgoing> mUnacked;
        std::uint32_t mNextExpected = 1;
        std::map<std::uint32_t, std::vector<std::uint8_t>> mOutOfOrder;

        std::deque<NetPose> mPoses;
        std::vector<NetHit> mHits;
        std::vector<NetBlow> mBlows;
        std::vector<std::string> mMessages;
    };
}

#endif

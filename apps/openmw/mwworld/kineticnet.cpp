#include "kineticnet.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <utility>

#include <components/debug/debuglog.hpp>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#ifndef SIO_UDP_CONNRESET
#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)
#endif
namespace
{
    using SocketHandle = SOCKET;
    constexpr SocketHandle sInvalidSocket = INVALID_SOCKET;
    void closeHandle(SocketHandle handle) { closesocket(handle); }
    int lastSocketError() { return WSAGetLastError(); }
    bool wouldBlock(int error) { return error == WSAEWOULDBLOCK; }
}
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
namespace
{
    using SocketHandle = int;
    constexpr SocketHandle sInvalidSocket = -1;
    void closeHandle(SocketHandle handle) { ::close(handle); }
    int lastSocketError() { return errno; }
    bool wouldBlock(int error) { return error == EWOULDBLOCK || error == EAGAIN; }
}
#endif

namespace MWWorld
{
    namespace
    {
        constexpr std::uint32_t sMagic = 0x31544e4b; // "KNT1"
        // Bumped whenever a packet's layout changes: both games must match.
        constexpr std::uint16_t sProtocol = 1;
        // Silence before the opponent counts as gone, s.
        constexpr double sTimeout = 6.0;
        constexpr double sHelloInterval = 0.5;
        constexpr double sPingInterval = 1.0;
        constexpr double sResendInterval = 0.15;
        constexpr std::size_t sMaxPoses = 32;
        constexpr std::size_t sMaxBodies = 64;

        enum Kind : std::uint8_t
        {
            Hello = 1,
            Welcome = 2,
            Pose = 3,
            Reliable = 4,
            Ack = 5,
            Ping = 6,
            Pong = 7,
            Bye = 8,
        };

        enum ReliableType : std::uint8_t
        {
            HitMessage = 1,
            BlowMessage = 2,
            TextMessage = 3,
        };

        struct Writer
        {
            std::vector<std::uint8_t> mData;
            template <class T>
            void put(const T& value)
            {
                const auto* bytes = reinterpret_cast<const std::uint8_t*>(&value);
                mData.insert(mData.end(), bytes, bytes + sizeof(T));
            }
            void putString(const std::string& text)
            {
                const std::size_t size = std::min<std::size_t>(text.size(), 255);
                put(static_cast<std::uint8_t>(size));
                mData.insert(mData.end(), text.begin(), text.begin() + size);
            }
            void putLongString(const std::string& text)
            {
                const std::size_t size = std::min<std::size_t>(text.size(), 60000);
                put(static_cast<std::uint16_t>(size));
                mData.insert(mData.end(), text.begin(), text.begin() + size);
            }
            template <class T, std::size_t N>
            void putArray(const std::array<T, N>& values)
            {
                for (const T& value : values)
                    put(value);
            }
        };

        struct Reader
        {
            const std::uint8_t* mData;
            std::size_t mSize;
            std::size_t mOffset = 0;
            bool mOk = true;
            template <class T>
            T get()
            {
                T value{};
                if (mOffset + sizeof(T) > mSize)
                {
                    mOk = false;
                    return value;
                }
                std::memcpy(&value, mData + mOffset, sizeof(T));
                mOffset += sizeof(T);
                return value;
            }
            std::string getString(std::size_t size)
            {
                if (mOffset + size > mSize)
                {
                    mOk = false;
                    return {};
                }
                std::string text(reinterpret_cast<const char*>(mData + mOffset), size);
                mOffset += size;
                return text;
            }
            std::string getString() { return getString(get<std::uint8_t>()); }
            std::string getLongString() { return getString(get<std::uint16_t>()); }
            template <class T, std::size_t N>
            void getArray(std::array<T, N>& values)
            {
                for (T& value : values)
                    value = get<T>();
            }
        };

        Writer header(Kind kind)
        {
            Writer writer;
            writer.put(sMagic);
            writer.put(static_cast<std::uint8_t>(kind));
            return writer;
        }
    }

    struct KineticNet::Socket
    {
        SocketHandle mHandle = sInvalidSocket;
        sockaddr_in mPeer{};
        bool mHasPeer = false;
        ~Socket()
        {
            if (mHandle != sInvalidSocket)
                closeHandle(mHandle);
        }
    };

    KineticNet& KineticNet::get()
    {
        static KineticNet instance;
        return instance;
    }

    KineticNet::KineticNet()
    {
#ifdef _WIN32
        WSADATA data;
        WSAStartup(MAKEWORD(2, 2), &data);
#endif
        if (const char* name = std::getenv("OPENMW_KINETIC_DUEL_NAME"); name && *name)
            mLocalName = name;
        else if (const char* user = std::getenv("USERNAME"); user && *user)
            mLocalName = user;
    }

    KineticNet::~KineticNet()
    {
        close();
#ifdef _WIN32
        WSACleanup();
#endif
    }

    double KineticNet::now()
    {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    bool KineticNet::startFromEnvironment()
    {
        const char* value = std::getenv("OPENMW_KINETIC_DUEL");
        if (value == nullptr || *value == '\0')
            return false;
        std::string text(value);
        std::uint16_t port = sDefaultPort;
        if (const auto colon = text.rfind(':'); colon != std::string::npos)
        {
            const int parsed = std::atoi(text.c_str() + colon + 1);
            if (parsed > 0 && parsed < 65536)
                port = static_cast<std::uint16_t>(parsed);
            text.resize(colon);
        }
        if (text == "host" || text.empty())
            return host(port);
        return join(text, port);
    }

    bool KineticNet::openSocket(std::uint16_t port, bool bindPort)
    {
        auto socket = std::make_unique<Socket>();
        socket->mHandle = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socket->mHandle == sInvalidSocket)
        {
            mError = "could not create a UDP socket (" + std::to_string(lastSocketError()) + ")";
            return false;
        }
#ifdef _WIN32
        u_long nonBlocking = 1;
        ioctlsocket(socket->mHandle, FIONBIO, &nonBlocking);
        // Without this a send to a closed port makes the next receive fail
        // (ICMP port unreachable) - the joiner's hellos before the host opens.
        BOOL reportReset = FALSE;
        DWORD returned = 0;
        WSAIoctl(socket->mHandle, SIO_UDP_CONNRESET, &reportReset, sizeof(reportReset), nullptr, 0, &returned,
            nullptr, nullptr);
#else
        fcntl(socket->mHandle, F_SETFL, fcntl(socket->mHandle, F_GETFL, 0) | O_NONBLOCK);
#endif
        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_addr.s_addr = htonl(INADDR_ANY);
        local.sin_port = htons(bindPort ? port : 0);
        if (::bind(socket->mHandle, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0)
        {
            mError = "could not open UDP port " + std::to_string(port) + " (" + std::to_string(lastSocketError())
                + "): is another game hosting on it?";
            return false;
        }
        mSocket = std::move(socket);
        return true;
    }

    bool KineticNet::host(std::uint16_t port)
    {
        std::scoped_lock lock(mMutex);
        mSocket.reset();
        mState = State::Off;
        if (!openSocket(port, true))
        {
            Log(Debug::Error) << "KINETIC_NET host failed: " << mError;
            return false;
        }
        mHost = true;
        mPort = port;
        mEndpoint = "port " + std::to_string(port);
        mState = State::Waiting;
        resetLink();
        Log(Debug::Info) << "KINETIC_NET hosting on UDP port " << port << " as " << mLocalName;
        return true;
    }

    bool KineticNet::join(const std::string& address, std::uint16_t port)
    {
        std::scoped_lock lock(mMutex);
        mSocket.reset();
        mState = State::Off;
        addrinfo hints{};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        addrinfo* found = nullptr;
        if (getaddrinfo(address.c_str(), nullptr, &hints, &found) != 0 || found == nullptr)
        {
            mError = "could not resolve " + address;
            Log(Debug::Error) << "KINETIC_NET join failed: " << mError;
            return false;
        }
        sockaddr_in peer = *reinterpret_cast<sockaddr_in*>(found->ai_addr);
        freeaddrinfo(found);
        peer.sin_port = htons(port);
        if (!openSocket(0, false))
        {
            Log(Debug::Error) << "KINETIC_NET join failed: " << mError;
            return false;
        }
        mSocket->mPeer = peer;
        mSocket->mHasPeer = true;
        mHost = false;
        mPort = port;
        mEndpoint = address + ":" + std::to_string(port);
        mState = State::Connecting;
        mLastHello = -1;
        resetLink();
        Log(Debug::Info) << "KINETIC_NET joining " << mEndpoint << " as " << mLocalName;
        return true;
    }

    void KineticNet::close()
    {
        std::scoped_lock lock(mMutex);
        if (mSocket && mSocket->mHasPeer && mState == State::Connected)
            sendRaw(header(Bye).mData);
        mSocket.reset();
        mState = State::Off;
    }

    void KineticNet::resetLink()
    {
        mNextReliable = 1;
        mUnacked.clear();
        mNextExpected = 1;
        mOutOfOrder.clear();
        mPoses.clear();
        mHits.clear();
        mBlows.clear();
        mMessages.clear();
        mPing = -1;
        mLastPing = -1;
    }

    void KineticNet::connected(double time)
    {
        resetLink();
        mState = State::Connected;
        mLastHeard = time;
        ++mSession;
        Log(Debug::Info) << "KINETIC_NET connected to " << mRemoteName << " (session " << mSession << ")";
    }

    void KineticNet::sendRaw(const std::vector<std::uint8_t>& packet)
    {
        if (!mSocket || !mSocket->mHasPeer)
            return;
        ::sendto(mSocket->mHandle, reinterpret_cast<const char*>(packet.data()), static_cast<int>(packet.size()), 0,
            reinterpret_cast<const sockaddr*>(&mSocket->mPeer), sizeof(mSocket->mPeer));
    }

    void KineticNet::sendReliable(std::uint8_t type, std::vector<std::uint8_t> payload)
    {
        if (mState != State::Connected)
            return;
        Writer writer = header(Reliable);
        const std::uint32_t id = mNextReliable++;
        writer.put(id);
        writer.put(type);
        writer.mData.insert(writer.mData.end(), payload.begin(), payload.end());
        Outgoing& outgoing = mUnacked[id];
        outgoing.mPacket = std::move(writer.mData);
        outgoing.mLastSent = now();
        sendRaw(outgoing.mPacket);
    }

    void KineticNet::update(float)
    {
        std::scoped_lock lock(mMutex);
        if (!mSocket || mState == State::Off)
            return;
        const double time = now();
        std::array<std::uint8_t, 2048> buffer;
        for (int packets = 0; packets < 512; ++packets)
        {
            sockaddr_in from{};
#ifdef _WIN32
            int fromSize = sizeof(from);
#else
            socklen_t fromSize = sizeof(from);
#endif
            const auto received = ::recvfrom(mSocket->mHandle, reinterpret_cast<char*>(buffer.data()),
                static_cast<int>(buffer.size()), 0, reinterpret_cast<sockaddr*>(&from), &fromSize);
            if (received < 0)
            {
                if (!wouldBlock(lastSocketError()))
                    continue;
                break;
            }
            receive(buffer.data(), static_cast<std::size_t>(received), &from, time);
            if (!mSocket)
                return;
        }

        if (mState == State::Connected && time - mLastHeard > sTimeout)
        {
            mState = State::Lost;
            Log(Debug::Warning) << "KINETIC_NET lost " << mRemoteName << " (silent for " << sTimeout << " s)";
        }
        if (!mHost && (mState == State::Connecting || mState == State::Lost) && time - mLastHello >= sHelloInterval)
        {
            Writer writer = header(Hello);
            writer.put(sProtocol);
            writer.putString(mLocalName);
            sendRaw(writer.mData);
            mLastHello = time;
        }
        if (mState == State::Connected)
        {
            if (mLastPing < 0 || time - mLastPing >= sPingInterval)
            {
                Writer writer = header(Ping);
                writer.put(time);
                sendRaw(writer.mData);
                mLastPing = time;
            }
            for (auto& [id, outgoing] : mUnacked)
                if (time - outgoing.mLastSent >= sResendInterval)
                {
                    sendRaw(outgoing.mPacket);
                    outgoing.mLastSent = time;
                }
        }
    }

    void KineticNet::receive(const std::uint8_t* data, std::size_t size, const void* fromAddress, double time)
    {
        const sockaddr_in& from = *static_cast<const sockaddr_in*>(fromAddress);
        Reader reader{ data, size };
        if (reader.get<std::uint32_t>() != sMagic || !reader.mOk)
            return;
        const auto kind = reader.get<std::uint8_t>();
        const bool fromPeer = mSocket->mHasPeer && from.sin_addr.s_addr == mSocket->mPeer.sin_addr.s_addr
            && from.sin_port == mSocket->mPeer.sin_port;

        if (kind == Hello)
        {
            if (!mHost)
                return;
            const auto protocol = reader.get<std::uint16_t>();
            const std::string name = reader.getString();
            if (!reader.mOk)
                return;
            if (protocol != sProtocol)
            {
                mError = name + " runs another KINETIC version (duel protocol " + std::to_string(protocol)
                    + ", this game " + std::to_string(sProtocol) + ")";
                Log(Debug::Warning) << "KINETIC_NET " << mError;
                return;
            }
            // A new opponent only while none is connected; the same one
            // repeats its hello until the welcome arrives.
            if (mState == State::Connected && !fromPeer)
                return;
            const bool fresh = mState != State::Connected;
            mSocket->mPeer = from;
            mSocket->mHasPeer = true;
            mRemoteName = name;
            Writer writer = header(Welcome);
            writer.put(sProtocol);
            writer.putString(mLocalName);
            sendRaw(writer.mData);
            if (fresh)
                connected(time);
            mLastHeard = time;
            return;
        }
        if (!fromPeer)
            return;
        mLastHeard = time;
        switch (kind)
        {
            case Welcome:
            {
                const auto protocol = reader.get<std::uint16_t>();
                const std::string name = reader.getString();
                if (!reader.mOk || mHost)
                    return;
                if (protocol != sProtocol)
                {
                    mError = name + " runs another KINETIC version (duel protocol " + std::to_string(protocol)
                        + ", this game " + std::to_string(sProtocol) + ")";
                    return;
                }
                mRemoteName = name;
                if (mState != State::Connected)
                    connected(time);
                return;
            }
            case Pose:
            {
                if (mState != State::Connected)
                    return;
                NetPose pose;
                pose.mStep = reader.get<std::uint32_t>();
                reader.getArray(pose.mActor);
                pose.mGuard = reader.get<std::uint8_t>() != 0;
                reader.getArray(pose.mGuardForward);
                const auto count = reader.get<std::uint8_t>();
                if (count > sMaxBodies)
                    return;
                pose.mBodies.resize(count);
                for (NetBody& body : pose.mBodies)
                {
                    reader.getArray(body.mPosition);
                    reader.getArray(body.mRotation);
                }
                if (!reader.mOk)
                    return;
                // Unordered: a late pose is dropped by the consumer by step.
                mPoses.push_back(std::move(pose));
                while (mPoses.size() > sMaxPoses)
                    mPoses.pop_front();
                return;
            }
            case Reliable:
            {
                const auto id = reader.get<std::uint32_t>();
                if (!reader.mOk || mState != State::Connected)
                    return;
                Writer ack = header(Ack);
                ack.put(id);
                sendRaw(ack.mData);
                if (id < mNextExpected || mOutOfOrder.contains(id))
                    return;
                mOutOfOrder.emplace(id, std::vector<std::uint8_t>(data + reader.mOffset, data + size));
                for (auto next = mOutOfOrder.find(mNextExpected); next != mOutOfOrder.end();
                     next = mOutOfOrder.find(mNextExpected))
                {
                    const auto& payload = next->second;
                    if (!payload.empty())
                        deliver(payload[0], payload.data() + 1, payload.size() - 1);
                    mOutOfOrder.erase(next);
                    ++mNextExpected;
                }
                return;
            }
            case Ack:
            {
                const auto id = reader.get<std::uint32_t>();
                if (reader.mOk)
                    mUnacked.erase(id);
                return;
            }
            case Ping:
            {
                Writer pong = header(Pong);
                pong.put(reader.get<double>());
                if (reader.mOk)
                    sendRaw(pong.mData);
                return;
            }
            case Pong:
            {
                const double sent = reader.get<double>();
                if (reader.mOk && sent <= time)
                    mPing = static_cast<float>((time - sent) * 1000.0);
                return;
            }
            case Bye:
                if (mState == State::Connected)
                {
                    mState = State::Lost;
                    Log(Debug::Info) << "KINETIC_NET " << mRemoteName << " left";
                }
                return;
            default:
                return;
        }
    }

    void KineticNet::deliver(std::uint8_t type, const std::uint8_t* data, std::size_t size)
    {
        Reader reader{ data, size };
        switch (type)
        {
            case HitMessage:
            {
                NetHit hit;
                hit.mDamage = reader.get<float>();
                hit.mHealth = reader.get<std::uint8_t>() != 0;
                hit.mSuccessful = reader.get<std::uint8_t>() != 0;
                hit.mType = reader.get<std::int8_t>();
                hit.mStrength = reader.get<float>();
                reader.getArray(hit.mHitPos);
                if (reader.mOk)
                    mHits.push_back(hit);
                return;
            }
            case BlowMessage:
            {
                NetBlow blow;
                blow.mKind = reader.get<std::uint8_t>();
                blow.mRegion = reader.get<std::uint8_t>();
                blow.mBodyIndex = reader.get<std::uint32_t>();
                reader.getArray(blow.mPoint);
                reader.getArray(blow.mDirection);
                reader.getArray(blow.mAttackerToVictim);
                blow.mApproachSpeed = reader.get<float>();
                blow.mCarriedMass = reader.get<float>();
                blow.mDamage = reader.get<float>();
                blow.mReactionImpulse = reader.get<float>();
                const auto flags = reader.get<std::uint8_t>();
                blow.mBlocked = flags & 1;
                blow.mParried = flags & 2;
                blow.mGuardBeaten = flags & 4;
                if (reader.mOk)
                    mBlows.push_back(blow);
                return;
            }
            case TextMessage:
            {
                std::string text = reader.getLongString();
                if (reader.mOk)
                    mMessages.push_back(std::move(text));
                return;
            }
            default:
                return;
        }
    }

    void KineticNet::sendPose(const NetPose& pose)
    {
        std::scoped_lock lock(mMutex);
        if (mState != State::Connected)
            return;
        Writer writer = header(Pose);
        writer.put(pose.mStep);
        writer.putArray(pose.mActor);
        writer.put(static_cast<std::uint8_t>(pose.mGuard ? 1 : 0));
        writer.putArray(pose.mGuardForward);
        const std::size_t count = std::min(pose.mBodies.size(), sMaxBodies);
        writer.put(static_cast<std::uint8_t>(count));
        for (std::size_t i = 0; i < count; ++i)
        {
            writer.putArray(pose.mBodies[i].mPosition);
            writer.putArray(pose.mBodies[i].mRotation);
        }
        sendRaw(writer.mData);
    }

    void KineticNet::sendHit(const NetHit& hit)
    {
        std::scoped_lock lock(mMutex);
        Writer writer;
        writer.put(hit.mDamage);
        writer.put(static_cast<std::uint8_t>(hit.mHealth));
        writer.put(static_cast<std::uint8_t>(hit.mSuccessful));
        writer.put(hit.mType);
        writer.put(hit.mStrength);
        writer.putArray(hit.mHitPos);
        sendReliable(HitMessage, std::move(writer.mData));
    }

    void KineticNet::sendBlow(const NetBlow& blow)
    {
        std::scoped_lock lock(mMutex);
        Writer writer;
        writer.put(blow.mKind);
        writer.put(blow.mRegion);
        writer.put(blow.mBodyIndex);
        writer.putArray(blow.mPoint);
        writer.putArray(blow.mDirection);
        writer.putArray(blow.mAttackerToVictim);
        writer.put(blow.mApproachSpeed);
        writer.put(blow.mCarriedMass);
        writer.put(blow.mDamage);
        writer.put(blow.mReactionImpulse);
        writer.put(static_cast<std::uint8_t>(
            (blow.mBlocked ? 1 : 0) | (blow.mParried ? 2 : 0) | (blow.mGuardBeaten ? 4 : 0)));
        sendReliable(BlowMessage, std::move(writer.mData));
    }

    void KineticNet::sendMessage(const std::string& text)
    {
        std::scoped_lock lock(mMutex);
        Writer writer;
        writer.putLongString(text);
        sendReliable(TextMessage, std::move(writer.mData));
    }

    std::vector<NetPose> KineticNet::takePoses()
    {
        std::scoped_lock lock(mMutex);
        std::vector<NetPose> result(std::make_move_iterator(mPoses.begin()), std::make_move_iterator(mPoses.end()));
        mPoses.clear();
        return result;
    }

    std::vector<NetHit> KineticNet::takeHits()
    {
        std::scoped_lock lock(mMutex);
        return std::exchange(mHits, {});
    }

    std::vector<NetBlow> KineticNet::takeBlows()
    {
        std::scoped_lock lock(mMutex);
        return std::exchange(mBlows, {});
    }

    std::vector<std::string> KineticNet::takeMessages()
    {
        std::scoped_lock lock(mMutex);
        return std::exchange(mMessages, {});
    }

    KineticNet::State KineticNet::state() const
    {
        std::scoped_lock lock(mMutex);
        return mState;
    }

    const char* KineticNet::stateName(State state)
    {
        switch (state)
        {
            case State::Off:
                return "off";
            case State::Waiting:
                return "waiting";
            case State::Connecting:
                return "connecting";
            case State::Connected:
                return "connected";
            case State::Lost:
                return "lost";
        }
        return "off";
    }

    bool KineticNet::isHost() const
    {
        std::scoped_lock lock(mMutex);
        return mHost;
    }

    float KineticNet::pingMs() const
    {
        std::scoped_lock lock(mMutex);
        return mPing;
    }

    std::string KineticNet::lastError() const
    {
        std::scoped_lock lock(mMutex);
        return mError;
    }

    std::string KineticNet::remoteName() const
    {
        std::scoped_lock lock(mMutex);
        return mRemoteName;
    }

    std::string KineticNet::localName() const
    {
        std::scoped_lock lock(mMutex);
        return mLocalName;
    }

    std::string KineticNet::endpoint() const
    {
        std::scoped_lock lock(mMutex);
        return mEndpoint;
    }

    std::uint16_t KineticNet::port() const
    {
        std::scoped_lock lock(mMutex);
        return mPort;
    }

    std::uint32_t KineticNet::session() const
    {
        std::scoped_lock lock(mMutex);
        return mSession;
    }
}

#include <ragdollcore/blow.hpp>

#include <atomic>

namespace RagdollCore
{
    const char* blowKindName(BlowKind kind)
    {
        switch (kind)
        {
            case BlowKind::Edge: return "edge";
            case BlowKind::Blunt: return "blunt";
            case BlowKind::Point: return "point";
            case BlowKind::Fist: return "fist";
            case BlowKind::Scripted: return "scripted";
        }
        return "scripted";
    }

    std::uint64_t nextBlowId()
    {
        static std::atomic<std::uint64_t> next{ 0 };
        return ++next;
    }
}

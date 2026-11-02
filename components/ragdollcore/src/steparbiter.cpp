#include <ragdollcore/steparbiter.hpp>

#include <algorithm>

namespace RagdollCore
{
    const char* stepSourceName(StepSource source)
    {
        switch (source)
        {
            case StepSource::GaveUp:
                return "gaveup";
            case StepSource::Stagger:
                return "stagger";
            case StepSource::Dodge:
                return "dodge";
            case StepSource::Stuck:
                return "stuck";
            case StepSource::Landing:
                return "landing";
            case StepSource::MovingCatch:
                return "moving";
            case StepSource::StandingCatch:
                return "standing";
            case StepSource::StageCatch:
                return "stage";
            case StepSource::Count:
                break;
        }
        return "?";
    }

    StepArbiter::Order StepArbiter::defaultOrder()
    {
        return { StepSource::Stuck, StepSource::Landing, StepSource::MovingCatch, StepSource::StandingCatch,
            StepSource::StageCatch };
    }

    std::optional<StepArbiter::Order> StepArbiter::parseOrder(std::string_view text)
    {
        Order order{};
        std::size_t count = 0;
        while (!text.empty())
        {
            const std::size_t comma = text.find(',');
            std::string_view name = text.substr(0, comma);
            while (!name.empty() && name.front() == ' ')
                name.remove_prefix(1);
            while (!name.empty() && name.back() == ' ')
                name.remove_suffix(1);
            std::optional<StepSource> found;
            for (const StepSource source : defaultOrder())
                if (name == stepSourceName(source))
                    found = source;
            if (!found || count >= sCatches || std::find(order.begin(), order.begin() + count, *found) != order.begin() + count)
                return std::nullopt;
            order[count++] = *found;
            if (comma == std::string_view::npos)
                break;
            text.remove_prefix(comma + 1);
        }
        if (count != sCatches)
            return std::nullopt;
        return order;
    }

    bool StepArbiter::claim(StepSource source, std::string_view reason)
    {
        ++mSteps[static_cast<std::size_t>(source)];
        const bool changed = mOwner != source || mReason != reason;
        mOwner = source;
        mReason = std::string(reason);
        return changed;
    }

    bool StepArbiter::idle()
    {
        const bool changed = mOwner.has_value();
        mOwner.reset();
        mReason.clear();
        return changed;
    }
}

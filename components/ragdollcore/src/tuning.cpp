#include <ragdollcore/tuning.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>

namespace RagdollCore
{
    namespace
    {
        // One immutable snapshot of the applied overrides. Snapshots are kept
        // until exit so the pointers tuningText() returned stay valid.
        struct Snapshot
        {
            std::map<std::string, std::string, std::less<>> mValues;
        };

        struct Registry
        {
            std::mutex mMutex; // pending overrides, the snapshot list, the Tuned list
            std::map<std::string, std::optional<std::string>, std::less<>> mPending;
            std::vector<std::unique_ptr<Snapshot>> mSnapshots;
            std::atomic<const Snapshot*> mCurrent{ nullptr };
            std::vector<const TuningDetail::TunedBase*> mTuned;
            std::atomic<std::uint64_t> mGeneration{ 0 };
        };

        Registry& registry()
        {
            // Never destroyed: Tuned statics unregister during static
            // destruction in an order the registry cannot rely on.
            static Registry* instance = new Registry;
            return *instance;
        }

        thread_local std::vector<std::string>* sRecording = nullptr;

        void record(std::string_view name)
        {
            if (sRecording == nullptr)
                return;
            if (std::find(sRecording->begin(), sRecording->end(), name) == sRecording->end())
                sRecording->emplace_back(name);
        }
    }

    const char* tuningText(const char* name)
    {
        if (name == nullptr)
            return nullptr;
        record(name);
        if (const Snapshot* snapshot = registry().mCurrent.load(std::memory_order_acquire))
        {
            const auto found = snapshot->mValues.find(std::string_view(name));
            if (found != snapshot->mValues.end())
                return found->second.empty() ? nullptr : found->second.c_str();
        }
        return std::getenv(name);
    }

    namespace
    {
        struct Warnings
        {
            std::mutex mMutex;
            std::function<void(const std::string&)> mSink;
            std::vector<std::string> mReported;
        };

        Warnings& warnings()
        {
            static Warnings* instance = new Warnings;
            return *instance;
        }

        void warnOnce(const std::string& name, const std::string& message)
        {
            Warnings& w = warnings();
            std::function<void(const std::string&)> sink;
            {
                std::scoped_lock lock(w.mMutex);
                if (std::find(w.mReported.begin(), w.mReported.end(), name) != w.mReported.end())
                    return;
                w.mReported.push_back(name);
                sink = w.mSink;
            }
            if (sink)
                sink(message);
            else
                std::fprintf(stderr, "%s\n", message.c_str());
        }
    }

    void setTuningWarningSink(std::function<void(const std::string&)> sink)
    {
        Warnings& w = warnings();
        std::scoped_lock lock(w.mMutex);
        w.mSink = std::move(sink);
    }

    std::size_t tuningList(const char* name, float* values, std::size_t count)
    {
        const char* text = tuningText(name);
        if (text == nullptr || *text == '\0')
            return 0;
        std::size_t fields = 0;
        const char* at = text;
        while (true)
        {
            char* end = nullptr;
            const float value = std::strtof(at, &end);
            if (fields < count && end != at && std::isfinite(value))
                values[fields] = value;
            ++fields;
            const char* comma = std::strchr(at, ',');
            if (comma == nullptr)
                break;
            at = comma + 1;
        }
        if (fields != count)
            warnOnce(name,
                std::string("RAGDOLL_TUNING_WARNING ") + name + " has " + std::to_string(fields) + " field"
                    + (fields == 1 ? "" : "s") + ", expected " + std::to_string(count) + " (\"" + text
                    + "\"): " + (fields < count ? "the missing fields keep their defaults" : "the extra fields are ignored"));
        return fields;
    }

    void setTuningOverride(std::string_view name, std::optional<std::string> value)
    {
        Registry& r = registry();
        std::scoped_lock lock(r.mMutex);
        r.mPending[std::string(name)] = std::move(value);
    }

    std::optional<std::string> tuningOverride(std::string_view name)
    {
        if (const Snapshot* snapshot = registry().mCurrent.load(std::memory_order_acquire))
        {
            const auto found = snapshot->mValues.find(name);
            if (found != snapshot->mValues.end())
                return found->second;
        }
        return std::nullopt;
    }

    std::vector<std::string> refreshTuning()
    {
        Registry& r = registry();
        std::vector<std::string> changed;
        std::vector<const TuningDetail::TunedBase*> tuned;
        {
            std::scoped_lock lock(r.mMutex);
            if (r.mPending.empty())
                return changed;
            auto next = std::make_unique<Snapshot>();
            if (const Snapshot* current = r.mCurrent.load(std::memory_order_acquire))
                next->mValues = current->mValues;
            for (auto& [name, value] : r.mPending)
            {
                const auto found = next->mValues.find(name);
                const bool had = found != next->mValues.end();
                if (value)
                {
                    if (had && found->second == *value)
                        continue;
                    next->mValues[name] = *value;
                }
                else
                {
                    if (!had)
                        continue;
                    next->mValues.erase(found);
                }
                changed.push_back(name);
            }
            r.mPending.clear();
            if (changed.empty())
                return changed;
            r.mCurrent.store(next.get(), std::memory_order_release);
            r.mSnapshots.push_back(std::move(next));
            r.mGeneration.fetch_add(1, std::memory_order_relaxed);
            tuned = r.mTuned;
        }
        // Registration order: a Tuned built from another registers after it,
        // so the inner one is fresh when the outer re-reads it.
        for (const TuningDetail::TunedBase* value : tuned)
            if (value->dependsOn(changed))
                value->reevaluate();
        return changed;
    }

    std::uint64_t tuningGeneration()
    {
        return registry().mGeneration.load(std::memory_order_relaxed);
    }

    std::size_t tunedValueCount()
    {
        Registry& r = registry();
        std::scoped_lock lock(r.mMutex);
        return r.mTuned.size();
    }

    std::size_t tunedDependents(std::string_view name)
    {
        Registry& r = registry();
        std::scoped_lock lock(r.mMutex);
        return static_cast<std::size_t>(std::count_if(r.mTuned.begin(), r.mTuned.end(),
            [&](const TuningDetail::TunedBase* value) { return value->dependsOn(name); }));
    }

    namespace TuningDetail
    {
        bool TunedBase::dependsOn(const std::vector<std::string>& names) const
        {
            for (const std::string& name : names)
                if (dependsOn(name))
                    return true;
            return false;
        }

        bool TunedBase::dependsOn(std::string_view name) const
        {
            return std::find(mDependencies.begin(), mDependencies.end(), name) != mDependencies.end();
        }

        void registerTuned(const TunedBase* value)
        {
            Registry& r = registry();
            std::scoped_lock lock(r.mMutex);
            r.mTuned.push_back(value);
        }

        void unregisterTuned(const TunedBase* value)
        {
            Registry& r = registry();
            std::scoped_lock lock(r.mMutex);
            r.mTuned.erase(std::remove(r.mTuned.begin(), r.mTuned.end(), value), r.mTuned.end());
        }

        DependencyScope::DependencyScope(std::vector<std::string>& names)
            : mPrevious(sRecording)
        {
            sRecording = &names;
        }

        DependencyScope::~DependencyScope()
        {
            std::vector<std::string>* inner = sRecording;
            sRecording = mPrevious;
            // The outer evaluation depends on everything the inner one read.
            if (sRecording != nullptr && inner != nullptr)
                for (const std::string& name : *inner)
                    record(name);
        }

        void recordNames(const std::vector<std::string>& names)
        {
            for (const std::string& name : names)
                record(name);
        }

        bool recording()
        {
            return sRecording != nullptr;
        }
    }
}

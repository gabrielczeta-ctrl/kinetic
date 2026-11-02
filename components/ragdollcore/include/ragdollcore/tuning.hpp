#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_TUNING_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_TUNING_H

// The live tuning registry. Every ragdoll switch (OPENMW_RAGDOLL_*,
// OPENMW_KINETIC_*) is read through tuningText(): an in-game override first,
// then the process environment (the launcher's `set` lines and the settings
// files, which are copied into the environment at start).
//
// Values parsed from switches are cached in Tuned<T> objects instead of plain
// `static const` locals. A Tuned records which switch names its evaluation
// read; refreshTuning() re-evaluates exactly the ones whose switches changed,
// so a slider moved in the options page takes effect on the next physics step
// without a restart. Overrides are queued from any thread and applied only by
// refreshTuning(), which the engine calls at a point where no reader runs (the
// start of PhysicsSystem::stepSimulation). Offline tools never call it: there
// every Tuned behaves like the static it replaced.
//
// Pure: no Jolt, game or engine types.

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace RagdollCore
{
    // The value in effect: an override (an empty override reads as unset),
    // else the environment, else null. The pointer stays valid for the whole
    // run (override snapshots are never freed).
    const char* tuningText(const char* name);
    inline const char* tuningText(const std::string& name)
    {
        return tuningText(name.c_str());
    }

    // A list switch ("a,b,c") into `values`: unset or empty leaves them
    // untouched; each field read replaces its default, an unreadable field
    // keeps it. Returns the number of fields the text held. A set switch
    // whose field count is not values.size() is reported once per name to
    // the warning sink: a short value keeps the defaults for the missing
    // fields without a word (11-01: a find-and-replace cut eight 4-field
    // reads to 3 and nothing said so).
    std::size_t tuningList(const char* name, float* values, std::size_t count);
    template <std::size_t N>
    std::size_t tuningList(const char* name, std::array<float, N>& values)
    {
        return tuningList(name, values.data(), N);
    }

    // Where tuning warnings go (the engine logs them; default stderr).
    void setTuningWarningSink(std::function<void(const std::string&)> sink);

    // Queues an override for `name` (nullopt removes it: back to the
    // environment). Any thread; applied by the next refreshTuning().
    void setTuningOverride(std::string_view name, std::optional<std::string> value);

    // The override in effect for `name`, if any (applied ones only).
    std::optional<std::string> tuningOverride(std::string_view name);

    // Applies queued overrides and re-evaluates every Tuned value that read a
    // changed switch, in registration order (a Tuned built from another one
    // registers after it). Returns the changed names. Call only where no
    // Tuned value or tuningText() result is being read on another thread.
    std::vector<std::string> refreshTuning();

    // Counts applied changes (0 until the first override).
    std::uint64_t tuningGeneration();

    // How many Tuned values exist and how many read `name` (diagnostics and
    // the options page: a switch nothing caches is read live anyway).
    std::size_t tunedValueCount();
    std::size_t tunedDependents(std::string_view name);

    namespace TuningDetail
    {
        class TunedBase
        {
        public:
            TunedBase() = default;
            TunedBase(const TunedBase&) = delete;
            TunedBase& operator=(const TunedBase&) = delete;
            virtual ~TunedBase() = default;
            virtual void reevaluate() const = 0;
            bool dependsOn(const std::vector<std::string>& names) const;
            bool dependsOn(std::string_view name) const;

        protected:
            mutable std::vector<std::string> mDependencies;
        };

        void registerTuned(const TunedBase* value);
        void unregisterTuned(const TunedBase* value);

        // Records every switch name read on this thread while alive (nested
        // scopes restore the outer one; an inner Tuned's names are added to
        // the outer when the outer reads it).
        class DependencyScope
        {
        public:
            explicit DependencyScope(std::vector<std::string>& names);
            ~DependencyScope();
            DependencyScope(const DependencyScope&) = delete;
            DependencyScope& operator=(const DependencyScope&) = delete;

        private:
            std::vector<std::string>* mPrevious;
        };

        // Adds names to the scope recording on this thread, if any.
        void recordNames(const std::vector<std::string>& names);
        bool recording();
    }

    template <class T>
    class Tuned final : public TuningDetail::TunedBase
    {
    public:
        template <class F>
        explicit Tuned(F&& evaluate)
            : mEvaluate(std::forward<F>(evaluate))
            , mValue(evaluateNow())
        {
            TuningDetail::registerTuned(this);
        }

        ~Tuned() override { TuningDetail::unregisterTuned(this); }

        const T& get() const
        {
            if (TuningDetail::recording())
                TuningDetail::recordNames(mDependencies);
            return mValue;
        }
        const T& operator*() const { return get(); }
        const T* operator->() const { return &get(); }

        void reevaluate() const override { mValue = evaluateNow(); }

    private:
        T evaluateNow() const
        {
            std::vector<std::string> names;
            T value = [&] {
                TuningDetail::DependencyScope scope(names);
                return static_cast<T>(mEvaluate());
            }();
            mDependencies = std::move(names);
            return value;
        }

        std::function<T()> mEvaluate;
        mutable T mValue;
    };

    // `static const auto x = RagdollCore::tuned([] { ... });` - the type is
    // the lambda's; tuned<T>(...) converts to T.
    template <class F>
    Tuned<std::decay_t<std::invoke_result_t<F>>> tuned(F&& evaluate)
    {
        return Tuned<std::decay_t<std::invoke_result_t<F>>>(std::forward<F>(evaluate));
    }

    template <class T, class F>
    Tuned<T> tuned(F&& evaluate)
    {
        return Tuned<T>(std::forward<F>(evaluate));
    }
}

#endif

#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_BALANCEAUTHORITY_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_BALANCEAUTHORITY_H

#include <ragdollcore/balancemetrics.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace RagdollCore
{
    struct BalanceAuthorityProfile
    {
        float mImpulseEngage = 800.0f;
        float mImpulseFull = 2500.0f;
        float mSupportLossEngage = 0.05f;
        float mSupportLossFull = 0.25f;
        float mStandingAuthority = 0.5f;
        float mWalkingAuthority = 0.25f;
        float mMaximumAnimationDeviationDegrees = 8.0f;
        float mMaximumAnimationDeviationUnits = 3.0f;
        float mAttackRate = 6.0f;
        float mReleaseRate = 1.5f;
    };

    struct BalanceAuthorityGeneration
    {
        std::uint64_t mActorSerial = 0;
        std::uint64_t mLifecycleEpoch = 0;

        bool operator==(const BalanceAuthorityGeneration&) const = default;
    };

    enum class BalanceDisturbancePhase
    {
        BeforeUpdate,
        AfterUpdate,
    };

    struct BalanceDisturbanceReceipt
    {
        BalanceAuthorityGeneration mGeneration;
        std::uint64_t mReceiptSerial = 0;
        std::uint64_t mSourceIdentity = 0;
        std::uint64_t mOccurrencePhysicsStep = 0;
        std::uint64_t mFirstUsablePhysicsStep = 0;
        BalanceDisturbancePhase mPhase = BalanceDisturbancePhase::BeforeUpdate;
        float mMagnitude = 0.0f;
        std::uint32_t mAffectedBodyId = 0;
    };

    enum class BalanceReceiptStatus
    {
        Accepted,
        UnboundGeneration,
        WrongGeneration,
        InvalidReceipt,
        DuplicateOrOlderReceipt,
        ExpiredReceipt,
        QueueFull,
    };

    struct BalanceAuthorityInput
    {
        BalanceAuthorityGeneration mGeneration;
        std::uint64_t mPhysicsStep = 0;
        float mDuration = 0.0f;
        BalanceSupportAvailability mSupport = BalanceSupportAvailability::Unknown;
        // The processed locomotion request. Missing intent never invents a
        // standing baseline; foot count is deliberately not an input.
        std::optional<bool> mWalking;
    };

    enum class BalanceAuthorityConsumeStatus
    {
        Consumed,
        UnboundGeneration,
        WrongGeneration,
        InvalidDuration,
        DuplicatePhysicsStep,
        OlderPhysicsStep,
    };

    struct BalanceAuthorityStep
    {
        BalanceAuthorityConsumeStatus mStatus = BalanceAuthorityConsumeStatus::UnboundGeneration;
        float mAuthority = 0.0f;
        float mDemand = 0.0f;
        float mBaseline = 0.0f;
        float mDisturbance = 0.0f;
        float mSupportLoss = 0.0f;
        float mUnsupportedTime = 0.0f;
        float mConsumedImpulse = 0.0f;
        std::uint32_t mConsumedReceiptCount = 0;
        std::uint64_t mFirstConsumedReceipt = 0;
        std::uint64_t mLastConsumedReceipt = 0;
        std::uint64_t mMissingPhysicsSteps = 0;
        bool mBaselineAvailable = false;
    };

    struct BalanceAuthorityResetResult
    {
        bool mChanged = false;
        std::uint64_t mDiscardedReceipts = 0;
    };

    float balanceAuthorityRamp(float value, float engage, float full);

    // Pure, bounded fixed-step authority/event state. Receipts bind to an
    // explicit actor/lifecycle generation and are consumed once on their
    // declared usable step. Getters do not advance or replay state.
    class BalanceAuthorityController
    {
    public:
        static constexpr std::size_t sMaximumPendingReceipts = 32;

        BalanceAuthorityResetResult reset(BalanceAuthorityGeneration generation);
        // Clears active authority and pending work at an ownership boundary
        // while retaining the bound generation and monotonic receipt floor.
        void suspend();
        BalanceReceiptStatus receive(const BalanceDisturbanceReceipt& receipt);
        BalanceAuthorityStep consume(const BalanceAuthorityInput& input,
            const BalanceAuthorityProfile& profile = {});

        const std::optional<BalanceAuthorityGeneration>& getGeneration() const { return mGeneration; }
        const std::optional<std::uint64_t>& getLastPhysicsStep() const { return mLastPhysicsStep; }
        float getAuthority() const { return mAuthority; }
        float getUnsupportedTime() const { return mUnsupportedTime; }
        std::size_t getPendingReceiptCount() const { return mPendingReceipts.size(); }
        std::uint64_t getLastReceiptSerial() const { return mLastReceiptSerial; }

    private:
        std::optional<BalanceAuthorityGeneration> mGeneration;
        std::optional<std::uint64_t> mLastPhysicsStep;
        std::vector<BalanceDisturbanceReceipt> mPendingReceipts;
        std::uint64_t mLastReceiptSerial = 0;
        float mAuthority = 0.0f;
        float mUnsupportedTime = 0.0f;
    };
}

#endif

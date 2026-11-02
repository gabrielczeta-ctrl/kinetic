#include <ragdollcore/balanceauthority.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace RagdollCore
{
    float balanceAuthorityRamp(float value, float engage, float full)
    {
        if (!std::isfinite(value) || !std::isfinite(engage) || !std::isfinite(full) || value <= engage)
            return 0.0f;
        if (full <= engage || value >= full)
            return 1.0f;
        const float x = (value - engage) / (full - engage);
        return x * x * (3.0f - 2.0f * x);
    }

    BalanceAuthorityResetResult BalanceAuthorityController::reset(BalanceAuthorityGeneration generation)
    {
        if (mGeneration && *mGeneration == generation)
            return {};
        const auto discarded = static_cast<std::uint64_t>(mPendingReceipts.size());
        mGeneration = generation;
        mLastPhysicsStep.reset();
        mPendingReceipts.clear();
        mLastReceiptSerial = 0;
        mAuthority = 0.0f;
        mUnsupportedTime = 0.0f;
        return { true, discarded };
    }

    void BalanceAuthorityController::suspend()
    {
        mLastPhysicsStep.reset();
        mPendingReceipts.clear();
        mAuthority = 0.0f;
        mUnsupportedTime = 0.0f;
    }

    BalanceReceiptStatus BalanceAuthorityController::receive(const BalanceDisturbanceReceipt& receipt)
    {
        if (!mGeneration)
            return BalanceReceiptStatus::UnboundGeneration;
        if (receipt.mGeneration != *mGeneration)
            return BalanceReceiptStatus::WrongGeneration;
        const bool overflow = receipt.mOccurrencePhysicsStep == std::numeric_limits<std::uint64_t>::max();
        const std::uint64_t expectedStep = receipt.mPhase == BalanceDisturbancePhase::BeforeUpdate
            ? receipt.mOccurrencePhysicsStep
            : overflow ? 0 : receipt.mOccurrencePhysicsStep + 1;
        if (receipt.mReceiptSerial == 0 || receipt.mFirstUsablePhysicsStep != expectedStep
            || !std::isfinite(receipt.mMagnitude) || receipt.mMagnitude < 0.0f)
            return BalanceReceiptStatus::InvalidReceipt;
        if (receipt.mReceiptSerial <= mLastReceiptSerial)
            return BalanceReceiptStatus::DuplicateOrOlderReceipt;
        if (mLastPhysicsStep && receipt.mFirstUsablePhysicsStep <= *mLastPhysicsStep)
            return BalanceReceiptStatus::ExpiredReceipt;
        if (mPendingReceipts.size() >= sMaximumPendingReceipts)
            return BalanceReceiptStatus::QueueFull;
        mPendingReceipts.push_back(receipt);
        mLastReceiptSerial = receipt.mReceiptSerial;
        return BalanceReceiptStatus::Accepted;
    }

    BalanceAuthorityStep BalanceAuthorityController::consume(
        const BalanceAuthorityInput& input, const BalanceAuthorityProfile& profile)
    {
        BalanceAuthorityStep result;
        result.mAuthority = mAuthority;
        result.mUnsupportedTime = mUnsupportedTime;
        if (!mGeneration)
            return result;
        if (input.mGeneration != *mGeneration)
        {
            result.mStatus = BalanceAuthorityConsumeStatus::WrongGeneration;
            return result;
        }
        if (!std::isfinite(input.mDuration) || input.mDuration <= 0.0f)
        {
            result.mStatus = BalanceAuthorityConsumeStatus::InvalidDuration;
            return result;
        }
        if (mLastPhysicsStep && input.mPhysicsStep <= *mLastPhysicsStep)
        {
            result.mStatus = input.mPhysicsStep == *mLastPhysicsStep
                ? BalanceAuthorityConsumeStatus::DuplicatePhysicsStep
                : BalanceAuthorityConsumeStatus::OlderPhysicsStep;
            return result;
        }

        if (mLastPhysicsStep && input.mPhysicsStep > *mLastPhysicsStep + 1)
        {
            result.mMissingPhysicsSteps = input.mPhysicsStep - *mLastPhysicsStep - 1;
            mUnsupportedTime = 0.0f;
        }

        for (const auto& receipt : mPendingReceipts)
        {
            if (receipt.mFirstUsablePhysicsStep != input.mPhysicsStep)
                continue;
            result.mConsumedImpulse = std::max(result.mConsumedImpulse, receipt.mMagnitude);
            if (result.mConsumedReceiptCount == 0)
                result.mFirstConsumedReceipt = receipt.mReceiptSerial;
            result.mLastConsumedReceipt = receipt.mReceiptSerial;
            ++result.mConsumedReceiptCount;
        }
        std::erase_if(mPendingReceipts, [&](const auto& receipt) {
            return receipt.mFirstUsablePhysicsStep <= input.mPhysicsStep;
        });

        if (input.mSupport == BalanceSupportAvailability::Supported)
            mUnsupportedTime = 0.0f;
        else if (input.mSupport == BalanceSupportAvailability::Unsupported)
            mUnsupportedTime += input.mDuration;
        else
            mUnsupportedTime = 0.0f;

        if (input.mSupport == BalanceSupportAvailability::Supported && input.mWalking)
        {
            result.mBaselineAvailable = true;
            result.mBaseline = std::clamp(*input.mWalking
                    ? profile.mWalkingAuthority : profile.mStandingAuthority,
                0.0f, 1.0f);
        }
        result.mDisturbance = balanceAuthorityRamp(
            result.mConsumedImpulse, profile.mImpulseEngage, profile.mImpulseFull);
        result.mSupportLoss = input.mSupport == BalanceSupportAvailability::Unsupported
            ? balanceAuthorityRamp(mUnsupportedTime, profile.mSupportLossEngage, profile.mSupportLossFull)
            : 0.0f;
        result.mDemand = std::clamp(
            std::max({ result.mBaseline, result.mDisturbance, result.mSupportLoss }), 0.0f, 1.0f);
        const float rate = result.mDemand > mAuthority ? profile.mAttackRate : profile.mReleaseRate;
        if (std::isfinite(rate) && rate > 0.0f)
        {
            const float change = rate * input.mDuration;
            mAuthority = result.mDemand > mAuthority
                ? std::min(result.mDemand, mAuthority + change)
                : std::max(result.mDemand, mAuthority - change);
        }
        else
            mAuthority = result.mDemand;
        mLastPhysicsStep = input.mPhysicsStep;
        result.mStatus = BalanceAuthorityConsumeStatus::Consumed;
        result.mAuthority = mAuthority;
        result.mUnsupportedTime = mUnsupportedTime;
        return result;
    }
}

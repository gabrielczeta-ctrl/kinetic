#include <ragdollcore/posecomposer.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace RagdollCore
{
    namespace
    {

        Transform blend(const Transform& from, const Transform& to, float weight)
        {
            if (weight >= 0.999f)
                return to;
            Transform result;
            result.mTranslation = from.mTranslation + (to.mTranslation - from.mTranslation) * weight;
            result.mRotation = from.mRotation.SLERP(to.mRotation, weight).Normalized();
            return result;
        }

        float angleBetween(const JPH::Quat& a, const JPH::Quat& b)
        {
            const float dot = std::clamp(std::abs(a.Dot(b)), 0.0f, 1.0f);
            return 2.0f * std::acos(dot);
        }
    }

    bool PoseComposer::changed(const Transform& a, const Transform& b)
    {
        // Any change: a write too small to see still has to be in the layer
        // its owner leaves, or the composition drops it.
        return a.mTranslation != b.mTranslation || a.mRotation != b.mRotation;
    }

    void PoseComposer::begin(const RagdollPose& base, const std::vector<std::optional<BodyChannel>>& channels)
    {
        mActive = true;
        mBase = base;
        mSeen = base;
        mBefore = base;
        mChannels = channels;
        mWrites.clear();
        mSequence = 0;
    }

    void PoseComposer::beforeWrite(const RagdollPose& pose)
    {
        if (!mActive)
            return;
        // Code between writers changed these bodies without a record: that
        // is part of the base the layers compose on.
        if (pose.mBodyTransforms.size() == mSeen.mBodyTransforms.size()
            && pose.mBodyTransforms.size() == mBase.mBodyTransforms.size())
            for (std::size_t body = 0; body < pose.mBodyTransforms.size(); ++body)
                if (changed(mSeen.mBodyTransforms[body], pose.mBodyTransforms[body]))
                    mBase.mBodyTransforms[body] = pose.mBodyTransforms[body];
        mBefore = pose;
    }

    void PoseComposer::record(
        std::size_t body, BodyOwner owner, bool additive, const Transform& before, const Transform& after)
    {
        Write write;
        write.mBody = body;
        write.mOwner = owner;
        write.mAdditive = additive;
        write.mSequence = mSequence++;
        if (additive)
        {
            write.mValue.mTranslation = after.mTranslation - before.mTranslation;
            write.mValue.mRotation = (after.mRotation * before.mRotation.Conjugated()).Normalized();
        }
        else
            write.mValue = after;
        mWrites.push_back(write);
    }

    void PoseComposer::remember()
    {
        for (const Write& write : mWrites)
            if (!write.mAdditive)
                mLast[{ write.mBody, write.mOwner }] = write.mValue;
    }

    void PoseComposer::end()
    {
        if (!mActive)
            return;
        remember();
        mActive = false;
    }

    void PoseComposer::reset()
    {
        mActive = false;
        mWrites.clear();
        mLast.clear();
    }

    PoseComposer::Result PoseComposer::compose(RagdollPose& pose, const BodyLeaseTable& leases)
    {
        Result result;
        if (!mActive)
            return result;
        mActive = false;
        if (pose.mBodyTransforms.size() != mBase.mBodyTransforms.size())
        {
            mWrites.clear();
            return result;
        }
        // Bodies changed after the last writer belong to the base too.
        for (std::size_t body = 0; body < pose.mBodyTransforms.size(); ++body)
            if (changed(mSeen.mBodyTransforms[body], pose.mBodyTransforms[body]))
                mBase.mBodyTransforms[body] = pose.mBodyTransforms[body];

        std::array<std::optional<ChannelResolution>, sBodyChannelCount> resolved;
        const auto resolution = [&](BodyChannel channel) -> const ChannelResolution& {
            auto& slot = resolved[static_cast<std::size_t>(channel)];
            if (!slot)
                slot = leases.resolve(channel);
            return *slot;
        };

        struct Layer
        {
            BodyOwner mOwner;
            int mPriority;
            std::uint32_t mSequence;
            float mWeight;
            bool mAdditive;
            Transform mValue;
        };
        std::vector<Layer> layers;
        for (std::size_t body = 0; body < pose.mBodyTransforms.size() && body < mChannels.size(); ++body)
        {
            if (!mChannels[body])
                continue;
            const ChannelResolution& channel = resolution(*mChannels[body]);
            layers.clear();
            // An offset written before a later absolute write is inside that
            // write (writers build on the pose so far): it is not added again.
            std::optional<std::uint32_t> lastAbsolute;
            for (const Write& write : mWrites)
                if (write.mBody == body && !write.mAdditive)
                    lastAbsolute = write.mSequence;
            // This step's writes: an owner's last absolute value; its
            // additive offsets accumulated in order.
            for (const Write& write : mWrites)
            {
                if (write.mBody != body)
                    continue;
                if (write.mAdditive && lastAbsolute && write.mSequence < *lastAbsolute)
                {
                    ++result.mWrites;
                    continue;
                }
                ++result.mWrites;
                int priority = defaultLeasePriority(write.mOwner);
                float weight = 1.0f;
                for (std::size_t i = 0; i < channel.mPoseLayerCount; ++i)
                {
                    const auto& layer = channel.mPoseLayers[i];
                    if (layer.mOwner == write.mOwner && layer.mAdditive == write.mAdditive)
                    {
                        priority = layer.mPriority;
                        weight = layer.mWeight;
                    }
                }
                auto existing = std::find_if(layers.begin(), layers.end(),
                    [&](const Layer& l) { return l.mOwner == write.mOwner && l.mAdditive == write.mAdditive; });
                if (existing == layers.end())
                    layers.push_back({ write.mOwner, priority, write.mSequence, weight, write.mAdditive, write.mValue });
                else if (write.mAdditive)
                {
                    existing->mValue.mTranslation += write.mValue.mTranslation;
                    existing->mValue.mRotation = (write.mValue.mRotation * existing->mValue.mRotation).Normalized();
                    existing->mSequence = write.mSequence;
                }
                else
                {
                    existing->mValue = write.mValue;
                    existing->mSequence = write.mSequence;
                }
            }
            // Released owners fading out: their last layer, by the fade.
            for (std::size_t i = 0; i < channel.mPoseLayerCount; ++i)
            {
                const auto& layer = channel.mPoseLayers[i];
                if (!layer.mReleasing || layer.mAdditive
                    || std::any_of(layers.begin(), layers.end(),
                        [&](const Layer& l) { return l.mOwner == layer.mOwner && !l.mAdditive; }))
                    continue;
                const auto last = mLast.find({ body, layer.mOwner });
                if (last == mLast.end())
                    continue;
                layers.push_back({ layer.mOwner, layer.mPriority, 0, layer.mWeight, false, last->second });
                ++result.mFading;
            }
            if (layers.empty())
                continue;
            // Lowest priority first; equal priorities in write order (the
            // later wins). Additive layers after every absolute one.
            std::sort(layers.begin(), layers.end(), [](const Layer& a, const Layer& b) {
                if (a.mAdditive != b.mAdditive)
                    return !a.mAdditive;
                if (a.mPriority != b.mPriority)
                    return a.mPriority < b.mPriority;
                return a.mSequence < b.mSequence;
            });
            Transform composed = mBase.mBodyTransforms[body];
            for (const Layer& layer : layers)
            {
                if (!layer.mAdditive)
                    composed = blend(composed, layer.mValue, std::clamp(layer.mWeight, 0.0f, 1.0f));
                else
                {
                    const float w = std::clamp(layer.mWeight, 0.0f, 1.0f);
                    composed.mTranslation += layer.mValue.mTranslation * w;
                    const JPH::Quat offset = w >= 0.999f
                        ? layer.mValue.mRotation
                        : JPH::Quat::sIdentity().SLERP(layer.mValue.mRotation, w).Normalized();
                    composed.mRotation = (offset * composed.mRotation).Normalized();
                }
            }
            const Transform& sequential = pose.mBodyTransforms[body];
            const float move = (composed.mTranslation - sequential.mTranslation).Length();
            const float turn = angleBetween(composed.mRotation, sequential.mRotation);
            if (move > 0.05f || turn > 0.002f)
            {
                ++result.mInversions;
                BodyOwner sequentialOwner = BodyOwner::Animation, composedOwner = BodyOwner::Animation;
                std::uint32_t lastSequence = 0;
                bool any = false;
                for (const Write& write : mWrites)
                    if (write.mBody == body && !write.mAdditive && (!any || write.mSequence >= lastSequence))
                    {
                        sequentialOwner = write.mOwner;
                        lastSequence = write.mSequence;
                        any = true;
                    }
                for (const Layer& layer : layers)
                    if (!layer.mAdditive)
                        composedOwner = layer.mOwner;
                result.mInverted.push_back({ *mChannels[body], sequentialOwner, composedOwner });
            }
            result.mMaxTranslation = std::max(result.mMaxTranslation, move);
            result.mMaxAngle = std::max(result.mMaxAngle, turn);
            pose.mBodyTransforms[body] = composed;
        }
        remember();
        // Drop remembered layers no lease can fade any more.
        for (auto it = mLast.begin(); it != mLast.end();)
        {
            const auto channel = it->first.first < mChannels.size() ? mChannels[it->first.first] : std::nullopt;
            bool held = false;
            if (channel)
            {
                const ChannelResolution& r = resolution(*channel);
                for (std::size_t i = 0; i < r.mPoseLayerCount && !held; ++i)
                    held = r.mPoseLayers[i].mOwner == it->first.second && !r.mPoseLayers[i].mAdditive;
            }
            it = held ? std::next(it) : mLast.erase(it);
        }
        mWrites.clear();
        return result;
    }
}

#include "skeletonadapter.hpp"
#include <ragdollcore/tuning.hpp>

#include <components/misc/strings/lower.hpp>
#include <components/nifosg/matrixtransform.hpp>
#include <components/sceneutil/riggeometry.hpp>
#include <components/sceneutil/skeleton.hpp>
#include <components/sceneutil/keyframe.hpp>

#include <osg/MatrixTransform>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Transform>
#include <osg/observer_ptr>

#include <cmath>
#include <cstddef>
#include <mutex>
#include <string_view>
#include <cstdlib>
#include <unordered_map>
#include <unordered_set>
#include <optional>
#include <utility>

namespace RagdollProfile
{
    namespace
    {
        RagdollCore::Transform toCoreTransform(const osg::Matrixf& matrix);
        bool isFinite(const RagdollCore::Transform& transform);
        osg::ref_ptr<osg::MatrixTransform> copyTransformOnly(const osg::MatrixTransform& node)
        {
            osg::ref_ptr<osg::MatrixTransform> result;
            if (const auto* nif = dynamic_cast<const NifOsg::MatrixTransform*>(&node))
            {
                auto copy = osg::ref_ptr<NifOsg::MatrixTransform>(new NifOsg::MatrixTransform);
                copy->setRotation(nif->mRotationScale);
                copy->setScale(nif->mScale);
                copy->setTranslation(nif->getMatrix().getTrans());
                result = copy;
            }
            else
                result = new osg::MatrixTransform(node.getMatrix());
            result->setReferenceFrame(node.getReferenceFrame());
            result->setName(node.getName());
            return result;
        }
    }

    struct SkeletonPoseBinding::Impl
    {
        struct BoundNode
        {
            osg::observer_ptr<osg::MatrixTransform> mNode;
            std::vector<osg::observer_ptr<osg::Node>> mParentPath;

            bool makeParentPath(osg::NodePath& result) const
            {
                osg::NodePath path;
                path.reserve(mParentPath.size());
                for (const osg::observer_ptr<osg::Node>& parent : mParentPath)
                {
                    if (!parent.valid())
                        return false;
                    path.push_back(parent.get());
                }
                result = std::move(path);
                return true;
            }

            bool makeNodePath(osg::NodePath& result) const
            {
                if (!mNode.valid() || !makeParentPath(result))
                    return false;
                result.push_back(mNode.get());
                return true;
            }
        };

        // Mirror-average a left/right transform pair across the model's x=0
        // plane (x -> -x, quaternion (x,-y,-z,w)) so both sides become the
        // same laterally symmetric pose. Used by OPENMW_RAGDOLL_SYMMETRIC_LEGS
        // for the creation skeleton and for every sampled animation target.
        // Mirror-average the TRANSLATIONS of a left/right pair across the
        // model's x=0 plane. Rotations are left alone: Morrowind's mirrored
        // rig bones do not share axis conventions, so a mirrored quaternion
        // is wrong for the skin (the first build averaged rotations too and
        // the leg meshes floated apart on screen). Positions alone carry the
        // stance geometry the walker's joint zero and base pose need; on the
        // captured game body positions-only passes strafe R/L at 38.5, 45
        // and 60 and the forward walk (context-symP, 2026-09-11).
        static void mirrorAveragePair(RagdollCore::Transform& left, RagdollCore::Transform& right)
        {
            const JPH::Vec3 lt = left.mTranslation;
            const JPH::Vec3 rt = right.mTranslation;
            const JPH::Vec3 average(0.5f * (lt.GetX() - rt.GetX()), 0.5f * (lt.GetY() + rt.GetY()), 0.5f * (lt.GetZ() + rt.GetZ()));
            left.mTranslation = average;
            right.mTranslation = JPH::Vec3(-average.GetX(), average.GetY(), average.GetZ());
        }

        struct State
        {
            // Body index pairs (left, right) whose sampled targets are
            // mirror-averaged (OPENMW_RAGDOLL_SYMMETRIC_LEGS); empty otherwise.
            std::vector<std::pair<std::size_t, std::size_t>> mMirrorPairs;

            bool capturePose(std::vector<RagdollCore::Transform>& pose, bool normalizeTargets = true) const
            {
                std::vector<RagdollCore::Transform> captured;
                captured.reserve(mNodes.size());
                for (const BoundNode& bound : mNodes)
                {
                    osg::NodePath path;
                    if (!bound.makeNodePath(path))
                        return false;
                    captured.push_back(toCoreTransform(osg::computeLocalToWorld(path)));
                    if (!isFinite(captured.back()))
                        return false;
                }
                if (normalizeTargets)
                    for (const auto& [left, right] : mMirrorPairs)
                        if (left < captured.size() && right < captured.size())
                            mirrorAveragePair(captured[left], captured[right]);
                pose = std::move(captured);
                return true;
            }

            bool applyPose(std::span<const RagdollCore::Transform> pose)
            {
                if (pose.size() != mNodes.size())
                    return false;
                std::vector<osg::NodePath> parentPaths(mNodes.size());
                for (std::size_t i = 0; i < mNodes.size(); ++i)
                {
                    if (!mNodes[i].mNode.valid() || !mNodes[i].makeParentPath(parentPaths[i]))
                        return false;
                }
                for (std::size_t i = 0; i < pose.size(); ++i)
                {
                    osg::MatrixTransform* node = mNodes[i].mNode.get();
                    const RagdollCore::Transform& transform = pose[i];
                    osg::Matrixf model;
                    model.makeRotate(osg::Quat(transform.mRotation.GetX(), transform.mRotation.GetY(),
                        transform.mRotation.GetZ(), transform.mRotation.GetW()));
                    model.setTrans(osg::Vec3f(
                        transform.mTranslation.GetX(), transform.mTranslation.GetY(), transform.mTranslation.GetZ()));

                    const osg::Matrixf parentModel = osg::computeLocalToWorld(parentPaths[i]);
                    const osg::Matrixf local = model * osg::Matrixf::inverse(parentModel);
                    if (auto* nifNode = dynamic_cast<NifOsg::MatrixTransform*>(node))
                    {
                        nifNode->setRotation(local.getRotate());
                        nifNode->setTranslation(local.getTrans());
                    }
                    else
                    {
                        const osg::Vec3f scale = node->getMatrix().getScale();
                        osg::Matrixf applied = osg::Matrixf::scale(scale) * osg::Matrixf::rotate(local.getRotate());
                        applied.setTrans(local.getTrans());
                        node->setMatrix(applied);
                    }
                }
                // RigGeometry caches skeleton-space bone matrices by traversal
                // number. The ragdoll override runs after animation traversal,
                // so node matrices alone are not enough: without invalidating
                // this cache, skinned body parts render from the animation pose
                // while rigid attachments follow the physics pose.
                if (mSkeleton.valid())
                    mSkeleton->markDirty();
                return true;
            }

            std::mutex mMutex;
            osg::observer_ptr<SceneUtil::Skeleton> mSkeleton;
            std::vector<BoundNode> mNodes;
            // Initial local transforms supply channels absent from a clip.
            // They must not follow later physics overrides on live nodes.
            std::unordered_map<const osg::Node*, osg::ref_ptr<const osg::MatrixTransform>> mSamplingBase;
            std::vector<RagdollCore::Transform> mPose;
            std::vector<RagdollCore::Transform> mAnimationPose;
        };

        // Skinned meshes take their culling bounds from the bones during the
        // update traversal, which is the animation pose here: the physics
        // pose is written after it. In third person the two poses overlap and
        // nobody notices. In first person they do not: the vanilla one-handed
        // stance parks the left hand below the view, so the hand mesh kept
        // bounds down there and was culled while the physical hand was in
        // plain sight (sleeve and wrist are rigid parts and stayed visible).
        class RefreshRigBounds final : public osg::NodeVisitor
        {
        public:
            explicit RefreshRigBounds(unsigned int traversalNumber)
                : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
            {
                setTraversalNumber(traversalNumber);
            }

            void apply(osg::Drawable& drawable) override
            {
                if (auto* rig = dynamic_cast<SceneUtil::RigGeometry*>(&drawable))
                    rig->refreshBounds(*this);
            }
        };

        class ApplyPoseCallback final : public osg::NodeCallback
        {
        public:
            explicit ApplyPoseCallback(std::weak_ptr<State> state)
                : mState(std::move(state))
            {
            }

            void operator()(osg::Node* node, osg::NodeVisitor* visitor) override
            {
                if (const std::shared_ptr<State> state = mState.lock())
                {
                    std::scoped_lock lock(state->mMutex);
                    if (!state->mAnimationPose.empty())
                        state->applyPose(state->mAnimationPose);
                }
                traverse(node, visitor);
                if (const std::shared_ptr<State> state = mState.lock())
                {
                    std::scoped_lock lock(state->mMutex);
                    std::vector<RagdollCore::Transform> animationPose;
                    // Raw capture: the live animation pose is applied back to
                    // the nodes when the physics override is off and is the
                    // motor target for live-animation routes. Symmetric-leg
                    // normalization would average left/right strides together
                    // (both feet to the same fore-aft position), turning the
                    // walk into a skate. It remains opt-in for sampled targets.
                    if (state->capturePose(animationPose, false))
                        state->mAnimationPose = std::move(animationPose);
                    if (state->applyPose(state->mPose))
                    {
                        RefreshRigBounds refresh(visitor->getTraversalNumber());
                        node->accept(refresh);
                    }
                }
            }

        private:
            std::weak_ptr<State> mState;
        };

        ~Impl()
        {
            if (mState)
            {
                std::scoped_lock lock(mState->mMutex);
                if (!mState->mAnimationPose.empty())
                    mState->applyPose(mState->mAnimationPose);
            }
            if (mSkeleton.valid() && mCallback.valid())
                mSkeleton->removeUpdateCallback(mCallback);
        }

        std::shared_ptr<State> mState = std::make_shared<State>();
        osg::observer_ptr<SceneUtil::Skeleton> mSkeleton;
        osg::ref_ptr<ApplyPoseCallback> mCallback;
    };

    namespace
    {
        struct NodeRecord
        {
            osg::MatrixTransform* mNode = nullptr;
            osg::NodePath mPath;
            osg::Matrixf mModelTransform;
        };

        class CollectNamedTransforms : public osg::NodeVisitor
        {
        public:
            CollectNamedTransforms()
                : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
            {
            }

            void apply(osg::MatrixTransform& node) override
            {
                if (!node.getName().empty())
                {
                    const std::size_t index = mNodes.size();
                    mByName[Misc::StringUtils::lowerCase(node.getName())].push_back(index);
                    mNodes.push_back({ &node, getNodePath(), osg::computeLocalToWorld(getNodePath()) });
                }
                traverse(node);
            }

            std::vector<NodeRecord> mNodes;
            std::unordered_map<std::string, std::vector<std::size_t>> mByName;
        };

        void addIssue(SkeletonSnapshotResult& result, std::string path, std::string message,
            std::optional<RagdollCore::ValidationCode> code = std::nullopt)
        {
            result.mIssues.push_back({ std::move(path), std::move(message), code });
        }

        std::vector<std::string> candidateNames(
            const RagdollCore::RagdollProfile& profile, std::string_view canonical)
        {
            std::vector<std::string> result{ std::string(canonical) };
            const std::string lowerCanonical = Misc::StringUtils::lowerCase(canonical);
            for (const RagdollCore::BoneAliases& aliases : profile.mBoneAliases)
            {
                if (Misc::StringUtils::lowerCase(aliases.mCanonical) != lowerCanonical)
                    continue;
                result.insert(result.end(), aliases.mAliases.begin(), aliases.mAliases.end());
            }
            return result;
        }

        RagdollCore::Transform toCoreTransform(const osg::Matrixf& matrix)
        {
            const osg::Vec3f translation = matrix.getTrans();
            const osg::Quat rotation = matrix.getRotate();
            return { JPH::Vec3(translation.x(), translation.y(), translation.z()),
                JPH::Quat(static_cast<float>(rotation.x()), static_cast<float>(rotation.y()),
                    static_cast<float>(rotation.z()), static_cast<float>(rotation.w())) };
        }

        bool isFinite(const RagdollCore::Transform& transform)
        {
            const JPH::Vec3 translation = transform.mTranslation;
            const JPH::Quat rotation = transform.mRotation;
            return std::isfinite(translation.GetX()) && std::isfinite(translation.GetY())
                && std::isfinite(translation.GetZ()) && std::isfinite(rotation.GetX()) && std::isfinite(rotation.GetY())
                && std::isfinite(rotation.GetZ()) && std::isfinite(rotation.GetW()) && rotation.IsNormalized(1e-3f);
        }
    }

    SkeletonPoseBinding::SkeletonPoseBinding(std::unique_ptr<Impl> impl)
        : mImpl(std::move(impl))
    {
    }

    SkeletonPoseBinding::~SkeletonPoseBinding() = default;
    SkeletonPoseBinding::SkeletonPoseBinding(SkeletonPoseBinding&&) noexcept = default;
    SkeletonPoseBinding& SkeletonPoseBinding::operator=(SkeletonPoseBinding&&) noexcept = default;

    bool SkeletonPoseBinding::applyPose(std::span<const RagdollCore::Transform> pose)
    {
        if (mImpl == nullptr || !mImpl->mSkeleton.valid())
            return false;
        std::scoped_lock lock(mImpl->mState->mMutex);
        if (pose.size() != mImpl->mState->mNodes.size())
            return false;
        for (std::size_t i = 0; i < pose.size(); ++i)
        {
            if (!mImpl->mState->mNodes[i].mNode.valid() || !isFinite(pose[i]))
                return false;
        }
        if (mImpl->mState->mAnimationPose.empty())
            mImpl->mState->capturePose(mImpl->mState->mAnimationPose);
        mImpl->mState->mPose.assign(pose.begin(), pose.end());
        if (!mImpl->mState->applyPose(mImpl->mState->mPose))
            return false;
        if (!mImpl->mCallback.valid())
        {
            mImpl->mCallback = new SkeletonPoseBinding::Impl::ApplyPoseCallback(mImpl->mState);
            mImpl->mSkeleton->addUpdateCallback(mImpl->mCallback);
        }
        return true;
    }

    bool SkeletonPoseBinding::capturePose(std::vector<RagdollCore::Transform>& pose) const
    {
        if (mImpl == nullptr || !mImpl->mSkeleton.valid())
            return false;
        std::scoped_lock lock(mImpl->mState->mMutex);
        return mImpl->mState->capturePose(pose);
    }

    bool SkeletonPoseBinding::captureEvaluatedPose(std::vector<RagdollCore::Transform>& pose) const
    {
        if (mImpl == nullptr || !mImpl->mSkeleton.valid())
            return false;
        std::scoped_lock lock(mImpl->mState->mMutex);
        return mImpl->mState->capturePose(pose, false);
    }

    bool SkeletonPoseBinding::captureAnimationPose(std::vector<RagdollCore::Transform>& pose) const
    {
        if (mImpl == nullptr || !mImpl->mSkeleton.valid())
            return false;
        std::scoped_lock lock(mImpl->mState->mMutex);
        if (mImpl->mState->mAnimationPose.size() != mImpl->mState->mNodes.size())
            return false;
        pose = mImpl->mState->mAnimationPose;
        return true;
    }

    bool SkeletonPoseBinding::sampleAnimationPose(const SceneUtil::KeyframeHolder& source, float time,
        std::vector<RagdollCore::Transform>& pose, std::string_view accumulationBone, JPH::Vec3 accumulationAxes) const
    {
        if (!mImpl || !mImpl->mSkeleton.valid() || !std::isfinite(time)
            || !std::isfinite(accumulationAxes.GetX()) || !std::isfinite(accumulationAxes.GetY())
            || !std::isfinite(accumulationAxes.GetZ()))
            return false;
        const auto accumulationName = Misc::StringUtils::lowerCase(accumulationBone);
        bool accumulationFound = accumulationName.empty();
        struct SampleTime final : SceneUtil::ControllerSource
        {
            float mTime;
            explicit SampleTime(float time) : mTime(time) {}
            float getValue(osg::NodeVisitor*) override { return mTime; }
        };
        const auto clock = std::make_shared<SampleTime>(time);
        std::unordered_map<std::string, const SceneUtil::KeyframeController*> controllers;
        for (const auto& [name, controller] : source.mKeyframeControllers)
            if (!controller || !controllers.emplace(Misc::StringUtils::lowerCase(name), controller.get()).second)
                return false;
        std::scoped_lock lock(mImpl->mState->mMutex);
        std::unordered_map<const osg::Node*, osg::ref_ptr<osg::MatrixTransform>> sampledNodes;
        std::vector<RagdollCore::Transform> sampled;
        for (const auto& bound : mImpl->mState->mNodes)
        {
            osg::NodePath path;
            if (!bound.makeNodePath(path))
                return false;
            for (auto*& node : path)
            {
                const auto base = mImpl->mState->mSamplingBase.find(node);
                if (base == mImpl->mState->mSamplingBase.end())
                {
                    if (node->asTransform())
                        return false;
                    continue;
                }
                auto [it, inserted] = sampledNodes.try_emplace(node);
                if (inserted)
                {
                    // Build a transform with no children, parents or callbacks.
                    // Cloning a Group would temporarily attach shared children
                    // to the clone and thereby mutate the original graph.
                    auto& isolated = it->second;
                    isolated = copyTransformOnly(*base->second);
                    const auto found = controllers.find(Misc::StringUtils::lowerCase(base->second->getName()));
                    if (found != controllers.end())
                    {
                        auto controller = osg::ref_ptr<SceneUtil::KeyframeController>(
                            osg::clone(found->second, osg::CopyOp::SHALLOW_COPY));
                        if (!controller)
                            return false;
                        controller->setSource(clock);
                        const auto transform = controller->getCurrentTransformation(nullptr);
                        if (auto* nif = dynamic_cast<NifOsg::MatrixTransform*>(isolated.get()))
                        {
                            if (transform.mRotation) nif->setRotation(*transform.mRotation);
                            if (transform.mTranslation) nif->setTranslation(*transform.mTranslation);
                            if (transform.mScale) nif->setScale(*transform.mScale);
                        }
                        else
                        {
                            const auto& baseMatrix = base->second->getMatrix();
                            const auto scale = transform.mScale ? osg::Vec3f(*transform.mScale, *transform.mScale,
                                *transform.mScale) : osg::Vec3f(baseMatrix.getScale());
                            osg::Matrixf local = osg::Matrixf::scale(scale)
                                * osg::Matrixf::rotate(transform.mRotation.value_or(baseMatrix.getRotate()));
                            local.setTrans(transform.mTranslation.value_or(baseMatrix.getTrans()));
                            isolated->setMatrix(local);
                        }
                    }
                    if (!accumulationName.empty()
                        && Misc::StringUtils::lowerCase(base->second->getName()) == accumulationName)
                    {
                        // Match Animation's ResetAccumRootCallback: remove
                        // accumulated local translation before parent transforms.
                        accumulationFound = true;
                        auto matrix = isolated->getMatrix();
                        auto translation = matrix.getTrans();
                        if (accumulationAxes.GetX() != 0) translation.x() = 0;
                        if (accumulationAxes.GetY() != 0) translation.y() = 0;
                        if (accumulationAxes.GetZ() != 0) translation.z() = 0;
                        if (auto* nif = dynamic_cast<NifOsg::MatrixTransform*>(isolated.get()))
                            nif->setTranslation(translation);
                        else
                        {
                            matrix.setTrans(translation);
                            isolated->setMatrix(matrix);
                        }
                    }
                }
                node = it->second.get();
            }
            sampled.push_back(toCoreTransform(osg::computeLocalToWorld(path)));
            if (!isFinite(sampled.back()))
                return false;
        }
        if (!accumulationFound)
            return false;
        pose = std::move(sampled);
        return true;
    }

    AnimationCycleSample SkeletonPoseBinding::sampleAnimationCycle(const SceneUtil::KeyframeHolder& source,
        std::string_view group, std::string_view accumulationBone, JPH::Vec3 accumulationAxes,
        std::size_t sampleCount) const
    {
        AnimationCycleSample result;
        if (sampleCount < 2 || sampleCount > 4096 || group.empty() || accumulationBone.empty())
        {
            result.mError = "Invalid cycle sample count, group or accumulation bone";
            return result;
        }
        const std::string prefix = std::string(group) + ": ";
        std::optional<float> start, stop, loopStart, loopStop;
        for (const auto& [time, key] : source.mTextKeys)
        {
            if (key == prefix + "start") start = time;
            else if (key == prefix + "stop") stop = time;
            else if (key == prefix + "loop start") loopStart = time;
            else if (key == prefix + "loop stop") loopStop = time;
        }
        start = loopStart ? loopStart : start;
        stop = loopStop ? loopStop : stop;
        if (!start || !stop || !std::isfinite(*start) || !std::isfinite(*stop) || *stop <= *start)
        {
            result.mError = "Animation source has no finite complete cycle interval";
            return result;
        }
        result.mStartTime = *start;
        result.mEndTime = *stop;
        for (std::size_t i = 0; i <= sampleCount; ++i)
        {
            const float time = i == sampleCount ? *stop
                : *start + (*stop - *start) * (static_cast<float>(i) / static_cast<float>(sampleCount));
            std::vector<RagdollCore::Transform> pose;
            if (!sampleAnimationPose(source, time, pose, accumulationBone, accumulationAxes))
            {
                result.mError = "Animation source returned invalid transforms";
                result.mFrames.clear();
                return result;
            }
            if (i != sampleCount)
                result.mFrames.push_back(std::move(pose));
            else
                for (std::size_t body = 0; body < pose.size(); ++body)
                {
                    const auto& first = result.mFrames.front()[body];
                    result.mMaximumSeamTranslation = std::max(result.mMaximumSeamTranslation,
                        (pose[body].mTranslation - first.mTranslation).Length());
                    result.mMinimumSeamRotationDot = std::min(result.mMinimumSeamRotationDot,
                        std::abs(pose[body].mRotation.Dot(first.mRotation)));
                }
        }
        if (result.mMaximumSeamTranslation >= 0.05f || result.mMinimumSeamRotationDot <= 0.99999f)
        {
            result.mError = "Animation cycle seam exceeds the existing translation/rotation bounds";
            result.mFrames.clear();
        }
        return result;
    }

    SkeletonSnapshotResult snapshotSkeleton(
        const RagdollCore::RagdollProfile& profile, SceneUtil::Skeleton& skeleton)
    {
        SkeletonSnapshotResult result;
        CollectNamedTransforms collector;
        skeleton.accept(collector);

        std::vector<const NodeRecord*> resolved(profile.mBodies.size(), nullptr);
        std::unordered_map<const osg::Node*, std::size_t> nodeToBody;
        for (std::size_t bodyIndex = 0; bodyIndex < profile.mBodies.size(); ++bodyIndex)
        {
            const RagdollCore::BodyProfile& body = profile.mBodies[bodyIndex];
            const std::string path = "profile.bodies[" + std::to_string(bodyIndex) + "].bone";
            std::unordered_set<std::size_t> matches;
            for (const std::string& candidate : candidateNames(profile, body.mBone))
            {
                const auto found = collector.mByName.find(Misc::StringUtils::lowerCase(candidate));
                if (found != collector.mByName.end())
                    matches.insert(found->second.begin(), found->second.end());
            }

            if (matches.empty())
            {
                addIssue(result, path, "No skeleton node matches bone '" + body.mBone + "' or its aliases.");
                continue;
            }
            std::size_t resolvedIndex = *matches.begin();
            if (matches.size() != 1)
            {
                // Assembled actors can contain a skin/attachment transform named
                // "head" or "neck" below the corresponding Bip01 rig bone.
                // Prefer the unique shallowest match, which is the transform the
                // skeleton itself uses; equally shallow duplicates remain unsafe.
                std::size_t shallowestDepth = collector.mNodes[resolvedIndex].mPath.size();
                bool shallowestIsAmbiguous = false;
                for (const std::size_t match : matches)
                {
                    const std::size_t depth = collector.mNodes[match].mPath.size();
                    if (depth < shallowestDepth)
                    {
                        resolvedIndex = match;
                        shallowestDepth = depth;
                        shallowestIsAmbiguous = false;
                    }
                    else if (match != resolvedIndex && depth == shallowestDepth)
                        shallowestIsAmbiguous = true;
                }
                if (shallowestIsAmbiguous)
                {
                    addIssue(result, path,
                        "Bone '" + body.mBone + "' resolves to multiple skeleton nodes; the binding is ambiguous.");
                    continue;
                }
            }

            const NodeRecord& record = collector.mNodes[resolvedIndex];
            const auto [existing, inserted] = nodeToBody.emplace(record.mNode, bodyIndex);
            if (!inserted)
            {
                addIssue(result, path, "The resolved skeleton node is already bound to body index "
                        + std::to_string(existing->second) + ".");
                continue;
            }
            resolved[bodyIndex] = &record;
        }

        if (!result.mIssues.empty())
            return result;

        RagdollCore::SkeletonDescriptor snapshot;
        snapshot.mId = skeleton.getName().empty() ? profile.mId + ":skeleton" : skeleton.getName();
        snapshot.mBones.reserve(resolved.size());
        for (std::size_t bodyIndex = 0; bodyIndex < resolved.size(); ++bodyIndex)
        {
            const NodeRecord& record = *resolved[bodyIndex];
            int parent = -1;
            for (auto it = record.mPath.rbegin() + 1; it != record.mPath.rend(); ++it)
            {
                const auto found = nodeToBody.find(*it);
                if (found != nodeToBody.end())
                {
                    parent = static_cast<int>(found->second);
                    break;
                }
            }
            snapshot.mBones.push_back(
                { profile.mBodies[bodyIndex].mBone, parent, toCoreTransform(record.mModelTransform) });
        }

        // ENV-GATED (default off, 2026-09-11, strafe-native-input): the
        // snapshot is the pose the actor happens to be animated in when the
        // ragdoll is created. For a standing NPC that is the idle clip,
        // whose stance is staggered (right foot 4.8 units farther out and
        // 4.5 ahead in the captured strafe fall). The builder derives the
        // hip/knee/ankle constraint frames from these transforms, so the
        // walker's joint zero is that staggered stance; on the captured
        // body the authored right strafe falls (step 176-228) and the left
        // passes, exactly the native pattern, and mirror-averaging the two
        // legs passes both. OPENMW_RAGDOLL_SYMMETRIC_LEGS=1 replaces each
        // left_/right_ leg bone pair by the mirror average of the two
        // (x -> -x, quaternion (x,-y,-z,w)) so the creation skeleton is
        // laterally symmetric whatever clip is playing.
        // Default ON since 2026-09-11 (stage 1 accepted by the user with it); OPENMW_RAGDOLL_SYMMETRIC_LEGS=0 restores the raw snapshot.
        //
        // Not on the kinematic-root route (2026-09-18). The average moves
        // each leg bone origin toward its mirror twin while the rotations stay
        // raw, so the rigid thigh and calf meshes no longer end where the next
        // bone is drawn. On the idle clip that is a couple of units; on a body
        // created in a weapon stance or mid-stride the feet are 20 and more
        // units apart fore-aft and the knees and ankles visibly come apart,
        // for the life of the ragdoll, because the joint anchors are built
        // from the same averaged positions. The dynamic walker needed the
        // symmetric joint zero to balance; a held root follows the live
        // animation and does not, and a death ragdoll never did.
        // OPENMW_RAGDOLL_SYMMETRIC_LEGS=1 still forces it for A/B runs.
        static const auto symmetricLegsOverride_tuned = RagdollCore::tuned<int>([] {
            const char* t = RagdollCore::tuningText("OPENMW_RAGDOLL_SYMMETRIC_LEGS");
            return t ? (std::atoi(t) == 0 ? 0 : 1) : -1;
        });
        static const auto& symmetricLegsOverride = symmetricLegsOverride_tuned.get();
        const bool symmetricLegs = symmetricLegsOverride >= 0
            ? symmetricLegsOverride == 1
            : profile.mActiveController.mLocomotionRoute != RagdollCore::LocomotionRoute::KinematicRoot;
        if (symmetricLegs)
        {
            const auto find = [&](const std::string& name) -> RagdollCore::SkeletonBone* {
                for (RagdollCore::SkeletonBone& bone : snapshot.mBones)
                    if (Misc::StringUtils::lowerCase(bone.mName) == name)
                        return &bone;
                return nullptr;
            };
            for (const char* part : { "thigh", "calf", "foot" })
            {
                RagdollCore::SkeletonBone* left = find(std::string("left_") + part);
                RagdollCore::SkeletonBone* right = find(std::string("right_") + part);
                if (!left || !right)
                    continue;
                SkeletonPoseBinding::Impl::mirrorAveragePair(left->mModelTransform, right->mModelTransform);
            }
        }
        result.mSkeleton = std::move(snapshot);
        const RagdollCore::ValidationResult validation = RagdollCore::validateProfile(profile, *result.mSkeleton);
        for (const RagdollCore::ValidationIssue& issue : validation.mIssues)
            addIssue(result, issue.mPath, issue.mMessage, issue.mCode);
        if (!result.mIssues.empty())
            return result;

        auto binding = std::make_unique<SkeletonPoseBinding::Impl>();
        binding->mState->mSkeleton = &skeleton;
        binding->mState->mNodes.reserve(resolved.size());
        for (const NodeRecord* record : resolved)
        {
            for (const auto* node : record->mPath)
                if (const auto* matrix = dynamic_cast<const osg::MatrixTransform*>(node))
                    if (!binding->mState->mSamplingBase.contains(node))
                        binding->mState->mSamplingBase.emplace(node, copyTransformOnly(*matrix));
            osg::NodePath parentPath = record->mPath;
            parentPath.pop_back();
            SkeletonPoseBinding::Impl::BoundNode bound;
            bound.mNode = record->mNode;
            bound.mParentPath.reserve(parentPath.size());
            for (osg::Node* parent : parentPath)
                bound.mParentPath.emplace_back(parent);
            binding->mState->mNodes.push_back(std::move(bound));
        }
        if (symmetricLegs)
        {
            // The creation legs are symmetric now; the targets sampled from
            // the animated skeleton must be too, or the walker's base pose
            // (taken from the targets) disagrees with its joint zero. The
            // captured strafe fall with symmetric creation legs alone ran
            // away sideways (replay probe OPENMW_RAGDOLL_REPLAY_SYMMETRIC_
            // TARGETS: no fall once the targets were symmetrized as well).
            for (const char* part : { "thigh", "calf", "foot" })
            {
                std::optional<std::size_t> left, right;
                for (std::size_t bodyIndex = 0; bodyIndex < profile.mBodies.size(); ++bodyIndex)
                {
                    const std::string bone = Misc::StringUtils::lowerCase(profile.mBodies[bodyIndex].mBone);
                    if (bone == std::string("left_") + part)
                        left = bodyIndex;
                    if (bone == std::string("right_") + part)
                        right = bodyIndex;
                }
                if (left && right)
                    binding->mState->mMirrorPairs.emplace_back(*left, *right);
            }
        }
        binding->mSkeleton = &skeleton;
        result.mPoseBinding = std::unique_ptr<SkeletonPoseBinding>(new SkeletonPoseBinding(std::move(binding)));
        return result;
    }
}

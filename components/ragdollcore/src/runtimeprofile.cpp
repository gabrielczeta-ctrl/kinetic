#include <ragdollcore/runtimeprofile.hpp>

#include <ragdollcore/anatomicalhinge.hpp>

#include <array>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <vector>

namespace RagdollCore
{
    std::optional<std::string_view> applyAnatomicalFrames(
        RagdollProfile& runtime, const SkeletonDescriptor& skeleton, bool knees)
    {
        const auto findBody = [&](const std::string& name) -> BodyProfile* {
            for (auto& body : runtime.mBodies)
                if (body.mName == name)
                    return &body;
            return nullptr;
        };
        const auto findBone = [&](const std::string& name) -> const Transform* {
            for (const auto& bone : skeleton.mBones)
                if (bone.mName == name)
                    return &bone.mModelTransform;
            return nullptr;
        };
        const auto& landmarks = runtime.mLandmarks;
        for (const auto& handName : { landmarks.mLeftHand, landmarks.mRightHand })
        {
            auto* hand = handName ? findBody(*handName) : nullptr;
            auto* lower = hand && hand->mParentBody ? findBody(*hand->mParentBody) : nullptr;
            auto* upper = lower && lower->mParentBody ? findBody(*lower->mParentBody) : nullptr;
            if (!upper || !lower->mConstraint || !lower->mConstraint->mDeriveFrameFromSkeleton
                || lower->mConstraint->mType != ConstraintType::Hinge)
                continue;
            const auto* a = findBone(upper->mBone);
            const auto* b = findBone(lower->mBone);
            const auto* c = findBone(hand->mBone);
            if (!a || !b || !c
                || !setAnatomicalHingeFrame(
                    *lower->mConstraint, *a, *b, *c, upper->mCenterOfMass, lower->mCenterOfMass))
                return "elbow";
        }
        if (!knees)
            return std::nullopt;
        std::array<std::array<BodyProfile*, 3>, 2> legs{};
        std::size_t side = 0;
        for (const auto& footName : { landmarks.mLeftFoot, landmarks.mRightFoot })
        {
            auto* foot = footName ? findBody(*footName) : nullptr;
            auto* lower = foot && foot->mParentBody ? findBody(*foot->mParentBody) : nullptr;
            auto* upper = lower && lower->mParentBody ? findBody(*lower->mParentBody) : nullptr;
            if (!upper || !lower->mConstraint)
                return "knee";
            legs[side++] = { upper, lower, foot };
        }
        const auto* leftHip = findBone(legs[0][0]->mBone);
        const auto* rightHip = findBone(legs[1][0]->mBone);
        if (!leftHip || !rightHip)
            return "knee";
        const JPH::Vec3 lateral = rightHip->mTranslation - leftHip->mTranslation;
        for (const auto& leg : legs)
        {
            // Authored frames retain their owner. Correct only the generic
            // frame that otherwise calls this bent pose zero.
            if (!leg[1]->mConstraint->mDeriveFrameFromSkeleton)
                continue;
            const auto* a = findBone(leg[0]->mBone);
            const auto* b = findBone(leg[1]->mBone);
            const auto* c = findBone(leg[2]->mBone);
            if (!a || !b || !c
                || !setAnatomicalHingeFrame(*leg[1]->mConstraint, *a, *b, *c, leg[0]->mCenterOfMass,
                    leg[1]->mCenterOfMass, -1.0f, lateral))
                return "knee";
        }
        return std::nullopt;
    }

    namespace
    {
        // name|parent|tx ty tz|qx qy qz qw (names hold spaces).
        void writeBones(std::ostream& out, const char* tag, const SkeletonDescriptor& skeleton)
        {
            out << tag << "_id|" << skeleton.mId << "\n";
            for (const auto& bone : skeleton.mBones)
            {
                const auto& t = bone.mModelTransform.mTranslation;
                const auto& q = bone.mModelTransform.mRotation;
                char numbers[192];
                std::snprintf(numbers, sizeof(numbers), "%.6f %.6f %.6f|%.8f %.8f %.8f %.8f", t.GetX(), t.GetY(),
                    t.GetZ(), q.GetX(), q.GetY(), q.GetZ(), q.GetW());
                out << tag << "|" << bone.mName << "|" << bone.mParent << "|" << numbers << "\n";
            }
        }
    }

    bool writeSkeletonDump(const std::string& path, const SkeletonDump& dump)
    {
        std::ofstream out(path);
        if (!out)
            return false;
        out << "scale|" << dump.mUniformScale << "\nmass|" << dump.mTotalMass << "\n";
        writeBones(out, "bone", dump.mSkeleton);
        if (dump.mFrames)
            writeBones(out, "frame", *dump.mFrames);
        return static_cast<bool>(out);
    }

    std::optional<SkeletonDump> readSkeletonDump(const std::string& path)
    {
        std::ifstream in(path);
        if (!in)
            return std::nullopt;
        SkeletonDump dump;
        std::string line;
        while (std::getline(in, line))
        {
            std::vector<std::string> field;
            std::stringstream split(line);
            for (std::string part; std::getline(split, part, '|');)
                field.push_back(part);
            if (field.size() == 2 && field[0] == "scale")
                dump.mUniformScale = std::stof(field[1]);
            else if (field.size() == 2 && field[0] == "mass")
                dump.mTotalMass = std::stof(field[1]);
            else if (field.size() == 2 && field[0] == "bone_id")
                dump.mSkeleton.mId = field[1];
            else if (field.size() == 2 && field[0] == "frame_id")
            {
                dump.mFrames.emplace();
                dump.mFrames->mId = field[1];
            }
            else if (field.size() == 5 && (field[0] == "bone" || field[0] == "frame"))
            {
                SkeletonBone bone;
                bone.mName = field[1];
                bone.mParent = std::stoi(field[2]);
                float tx, ty, tz, qx, qy, qz, qw;
                if (std::sscanf(field[3].c_str(), "%f %f %f", &tx, &ty, &tz) != 3
                    || std::sscanf(field[4].c_str(), "%f %f %f %f", &qx, &qy, &qz, &qw) != 4)
                    return std::nullopt;
                bone.mModelTransform.mTranslation = JPH::Vec3(tx, ty, tz);
                bone.mModelTransform.mRotation = JPH::Quat(qx, qy, qz, qw).Normalized();
                if (field[0] == "bone")
                    dump.mSkeleton.mBones.push_back(std::move(bone));
                else
                {
                    if (!dump.mFrames)
                        dump.mFrames.emplace();
                    dump.mFrames->mBones.push_back(std::move(bone));
                }
            }
        }
        if (dump.mSkeleton.mBones.empty())
            return std::nullopt;
        return dump;
    }

    std::string clipTrackLine(float requestSpeed, const std::vector<Transform>& bodies)
    {
        std::string line;
        char number[160];
        std::snprintf(number, sizeof(number), "%.2f", requestSpeed);
        line += number;
        for (const Transform& t : bodies)
        {
            std::snprintf(number, sizeof(number), " %.4f %.4f %.4f %.6f %.6f %.6f %.6f", t.mTranslation.GetX(),
                t.mTranslation.GetY(), t.mTranslation.GetZ(), t.mRotation.GetX(), t.mRotation.GetY(),
                t.mRotation.GetZ(), t.mRotation.GetW());
            line += number;
        }
        return line;
    }

    std::optional<ClipTrack> readClipTrack(const std::string& path)
    {
        std::ifstream in(path);
        if (!in)
            return std::nullopt;
        ClipTrack track;
        std::string line;
        while (std::getline(in, line))
        {
            std::istringstream values(line);
            float speed = 0.0f;
            if (!(values >> speed))
                continue;
            std::vector<Transform> bodies;
            float v[7];
            while (values >> v[0] >> v[1] >> v[2] >> v[3] >> v[4] >> v[5] >> v[6])
            {
                Transform t;
                t.mTranslation = JPH::Vec3(v[0], v[1], v[2]);
                t.mRotation = JPH::Quat(v[3], v[4], v[5], v[6]).Normalized();
                bodies.push_back(t);
            }
            if (!track.mPoses.empty() && bodies.size() != track.mPoses.front().size())
                return std::nullopt;
            track.mRequestSpeed.push_back(speed);
            track.mPoses.push_back(std::move(bodies));
        }
        if (track.mPoses.empty())
            return std::nullopt;
        return track;
    }
}

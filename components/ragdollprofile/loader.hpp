#ifndef OPENMW_COMPONENTS_RAGDOLLPROFILE_LOADER_H
#define OPENMW_COMPONENTS_RAGDOLLPROFILE_LOADER_H

#include <components/vfs/pathutil.hpp>

#include <ragdollcore/profile.hpp>

#include <istream>
#include <optional>
#include <string>
#include <vector>

namespace VFS
{
    class Manager;
}

namespace RagdollProfile
{
    struct LoadIssue
    {
        std::string mPath;
        std::string mMessage;
    };

    struct LoadResult
    {
        std::optional<RagdollCore::RagdollProfile> mProfile;
        std::vector<LoadIssue> mIssues;

        bool isValid() const { return mProfile.has_value() && mIssues.empty(); }
    };

    LoadResult load(std::istream& stream);
    LoadResult load(const VFS::Manager& vfs, VFS::Path::NormalizedView path);
}

#endif

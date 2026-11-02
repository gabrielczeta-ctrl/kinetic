#include <ragdollcore/footfallgait.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

namespace RagdollCore
{
    static FootfallGaitRow blend(const FootfallGaitRow& a, const FootfallGaitRow& b, float t)
    {
        const auto mix = [t](float x, float y) { return x + (y - x) * t; };
        FootfallGaitRow r;
        r.mSpeed = mix(a.mSpeed, b.mSpeed);
        r.mDuty = mix(a.mDuty, b.mDuty);
        r.mSwingTime = mix(a.mSwingTime, b.mSwingTime);
        r.mLead = mix(a.mLead, b.mLead);
        r.mCrouch = mix(a.mCrouch, b.mCrouch);
        for (std::size_t i = 0; i < r.mLift.size(); ++i)
        {
            r.mLift[i] = mix(a.mLift[i], b.mLift[i]);
            r.mProgress[i] = mix(a.mProgress[i], b.mProgress[i]);
        }
        for (std::size_t i = 0; i < r.mBob.size(); ++i)
            r.mBob[i] = mix(a.mBob[i], b.mBob[i]);
        r.mHasPath = a.mHasPath && b.mHasPath;
        if (r.mHasPath)
            for (std::size_t i = 0; i < r.mStanceKnee.size(); ++i)
            {
                for (std::size_t axis = 0; axis < 3; ++axis)
                    r.mPath[axis][i] = mix(a.mPath[axis][i], b.mPath[axis][i]);
                r.mStanceKnee[i] = mix(a.mStanceKnee[i], b.mStanceKnee[i]);
            }
        return r;
    }

    namespace
    {
        constexpr float sGravity = 9.81f;

        template <std::size_t N>
        float sampleCurve(const std::array<float, N>& curve, float phase)
        {
            const float x = std::clamp(phase, 0.0f, 1.0f) * static_cast<float>(N - 1);
            const std::size_t i = std::min(static_cast<std::size_t>(x), N - 2);
            const float f = x - static_cast<float>(i);
            return curve[i] + (curve[i + 1] - curve[i]) * f;
        }

        template <std::size_t N>
        bool readCurve(std::istringstream& line, std::array<float, N>& curve)
        {
            for (float& value : curve)
                if (!(line >> value) || !std::isfinite(value))
                    return false;
            return true;
        }

        // A row interpolated at `speed` (source leg lengths per second), clamped to the ends.
        FootfallGaitRow interpolate(const std::vector<FootfallGaitRow>& rows, float speed)
        {
            if (speed <= rows.front().mSpeed)
                return rows.front();
            if (speed >= rows.back().mSpeed)
                return rows.back();
            const auto upper = std::find_if(rows.begin(), rows.end(), [&](const auto& r) { return r.mSpeed >= speed; });
            const auto& b = *upper;
            const auto& a = *(upper - 1);
            const float t = (speed - a.mSpeed) / std::max(1e-6f, b.mSpeed - a.mSpeed);
            return blend(a, b, t);
        }
    }

    float FootfallGaitSample::lift(float phase) const
    {
        return sampleCurve(mLift, phase);
    }

    float FootfallGaitSample::progress(float phase) const
    {
        return sampleCurve(mProgress, phase);
    }

    float FootfallGaitSample::bob(float phase) const
    {
        return sampleCurve(mBob, phase);
    }

    float FootfallGaitSample::path(std::size_t axis, float phase) const
    {
        return sampleCurve(mPath[std::min<std::size_t>(axis, 2)], phase);
    }

    float FootfallGaitSample::stanceKnee(float phase) const
    {
        return sampleCurve(mStanceKnee, phase);
    }

    FootfallGaitSample FootfallGaitSample::mix(const FootfallGaitSample& a, const FootfallGaitSample& b, float t)
    {
        const auto lerp = [t](float x, float y) { return x + (y - x) * t; };
        FootfallGaitSample r = a;
        r.mDuty = lerp(a.mDuty, b.mDuty);
        r.mSwingTime = lerp(a.mSwingTime, b.mSwingTime);
        r.mLead = lerp(a.mLead, b.mLead);
        r.mCrouch = lerp(a.mCrouch, b.mCrouch);
        r.mMaximumFroude = lerp(a.mMaximumFroude, b.mMaximumFroude);
        r.mFroude = lerp(a.mFroude, b.mFroude);
        for (std::size_t i = 0; i < r.mLift.size(); ++i)
        {
            r.mLift[i] = lerp(a.mLift[i], b.mLift[i]);
            r.mProgress[i] = lerp(a.mProgress[i], b.mProgress[i]);
        }
        for (std::size_t i = 0; i < r.mBob.size(); ++i)
            r.mBob[i] = lerp(a.mBob[i], b.mBob[i]);
        r.mHasPath = a.mHasPath && b.mHasPath;
        if (r.mHasPath)
            for (std::size_t i = 0; i < r.mStanceKnee.size(); ++i)
            {
                for (std::size_t axis = 0; axis < 3; ++axis)
                    r.mPath[axis][i] = lerp(a.mPath[axis][i], b.mPath[axis][i]);
                r.mStanceKnee[i] = lerp(a.mStanceKnee[i], b.mStanceKnee[i]);
            }
        return r;
    }

    std::optional<FootfallGaitTable> FootfallGaitTable::load(const std::filesystem::path& path)
    {
        std::ifstream file(path);
        if (!file)
            return std::nullopt;
        std::stringstream text;
        text << file.rdbuf();
        return parse(text.str());
    }

    std::optional<FootfallGaitTable> FootfallGaitTable::parse(const std::string& text)
    {
        FootfallGaitTable table;
        std::istringstream lines(text);
        std::string raw;
        FootfallGaitRow* current = nullptr;
        while (std::getline(lines, raw))
        {
            std::istringstream line(raw);
            std::string key;
            if (!(line >> key) || key[0] == '#')
                continue;
            if (key == "leg_length_m")
            {
                if (!(line >> table.mSourceLegLength) || !(table.mSourceLegLength > 0.0f))
                    return std::nullopt;
            }
            else if (key == "gait")
            {
                std::string style;
                int angle = 0;
                FootfallGaitRow row;
                if (!(line >> style >> angle >> row.mSpeed >> row.mDuty >> row.mSwingTime >> row.mLead >> row.mCrouch))
                    return std::nullopt;
                auto& rows = table.mStyles[style][angle];
                rows.push_back(row);
                current = &rows.back();
            }
            else if (current && key == "lift")
            {
                if (!readCurve(line, current->mLift))
                    return std::nullopt;
            }
            else if (current && key == "progress")
            {
                if (!readCurve(line, current->mProgress))
                    return std::nullopt;
            }
            else if (current && key == "bob")
            {
                if (!readCurve(line, current->mBob))
                    return std::nullopt;
            }
            else if (current && (key == "path_x" || key == "path_y" || key == "path_z"))
            {
                if (!readCurve(line, current->mPath[static_cast<std::size_t>(key[5] - 'x')]))
                    return std::nullopt;
            }
            else if (current && key == "stance_knee")
            {
                if (!readCurve(line, current->mStanceKnee))
                    return std::nullopt;
                current->mHasPath = true;
            }
        }
        for (auto& [style, angles] : table.mStyles)
            for (auto& [angle, rows] : angles)
                std::sort(rows.begin(), rows.end(), [](const auto& a, const auto& b) { return a.mSpeed < b.mSpeed; });
        if (table.mStyles.empty())
            return std::nullopt;
        return table;
    }

    std::optional<FootfallGaitSample> FootfallGaitTable::sample(
        const std::string& style, float travelAngle, float speed, float legLength, float unitsPerMetre) const
    {
        const auto found = mStyles.find(style);
        if (found == mStyles.end() || !(legLength > 0.0f) || !std::isfinite(speed) || !std::isfinite(travelAngle))
            return std::nullopt;
        // Directions on the circle; a missing side mirrors the other one (a
        // gait to the front-left is the gait to the front-right mirrored).
        std::map<float, const std::vector<FootfallGaitRow>*> directions;
        for (const auto& [angle, rows] : found->second)
            if (!rows.empty())
                directions[static_cast<float>(angle)] = &rows;
        for (const auto& [angle, rows] : found->second)
            if (!rows.empty() && angle != 0 && angle != 180 && !directions.count(static_cast<float>(-angle)))
                directions[static_cast<float>(-angle)] = &rows;
        if (directions.empty())
            return std::nullopt;
        if (directions.count(180.0f))
            directions[-180.0f] = directions[180.0f];

        float angle = std::fmod(travelAngle + 540.0f, 360.0f) - 180.0f;
        auto upper = directions.lower_bound(angle);
        const std::vector<FootfallGaitRow>* rowsA = nullptr;
        const std::vector<FootfallGaitRow>* rowsB = nullptr;
        float weight = 0.0f;
        if (upper == directions.end())
            rowsA = rowsB = std::prev(upper)->second;
        else if (upper == directions.begin() || upper->first == angle)
            rowsA = rowsB = upper->second;
        else
        {
            const auto lower = std::prev(upper);
            rowsA = lower->second;
            rowsB = upper->second;
            weight = (angle - lower->first) / std::max(1e-3f, upper->first - lower->first);
        }

        // Froude similarity: the source row with the same speed^2 / (g * leg).
        const float froude = speed / std::sqrt(sGravity * legLength);
        const float sourceSpeed = froude * std::sqrt(sGravity * mSourceLegLength) / mSourceLegLength;
        const FootfallGaitRow row = blend(interpolate(*rowsA, sourceSpeed), interpolate(*rowsB, sourceSpeed), weight);
        const auto maximumFroude = [&](const std::vector<FootfallGaitRow>& rows) {
            return rows.back().mSpeed * mSourceLegLength / std::sqrt(sGravity * mSourceLegLength);
        };

        FootfallGaitSample sample;
        const float lengthScale = legLength * unitsPerMetre;
        const float timeScale = std::sqrt(legLength / mSourceLegLength);
        sample.mDuty = std::clamp(row.mDuty, 0.2f, 0.8f);
        sample.mSwingTime = row.mSwingTime * timeScale;
        sample.mLead = row.mLead * lengthScale;
        sample.mCrouch = row.mCrouch * lengthScale;
        sample.mFroude = froude;
        sample.mMaximumFroude = std::min(maximumFroude(*rowsA), maximumFroude(*rowsB));
        const float first = row.mLift.front();
        const float last = row.mLift.back();
        const float progressEnd = std::max(1e-3f, row.mProgress.back());
        for (std::size_t i = 0; i < row.mLift.size(); ++i)
        {
            const float t = static_cast<float>(i) / static_cast<float>(row.mLift.size() - 1);
            sample.mLift[i] = std::max(0.0f, row.mLift[i] - (first + (last - first) * t)) * lengthScale;
            sample.mProgress[i] = std::clamp(row.mProgress[i] / progressEnd, 0.0f, 1.0f);
        }
        for (std::size_t i = 1; i < sample.mProgress.size(); ++i)
            sample.mProgress[i] = std::max(sample.mProgress[i], sample.mProgress[i - 1]);
        for (std::size_t i = 0; i < row.mBob.size(); ++i)
            sample.mBob[i] = row.mBob[i] * lengthScale;
        sample.mHasPath = row.mHasPath;
        if (row.mHasPath)
            for (std::size_t i = 0; i < row.mStanceKnee.size(); ++i)
            {
                for (std::size_t axis = 0; axis < 3; ++axis)
                    sample.mPath[axis][i] = row.mPath[axis][i] * lengthScale;
                sample.mStanceKnee[i] = row.mStanceKnee[i];
            }
        return sample;
    }
}

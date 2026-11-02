#include "woundfx.hpp"
#include <ragdollcore/tuning.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include <osg/BufferObject>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Program>
#include <osg/StateSet>
#include <osg/TriangleIndexFunctor>
#include <osg/Uniform>

#include <components/debug/debuglog.hpp>
#include <components/sceneutil/morphgeometry.hpp>
#include <components/sceneutil/riggeometry.hpp>
#include <components/shader/shadermanager.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/world.hpp"
#include "../mwworld/datetimemanager.hpp"

namespace
{
    constexpr const char* sBindArrayName = "woundBindPosition";

    // Closest point of triangle abc to p as barycentric weights (Ericson,
    // Real-Time Collision Detection 5.1.5).
    osg::Vec3f closestBarycentric(const osg::Vec3f& p, const osg::Vec3f& a, const osg::Vec3f& b, const osg::Vec3f& c)
    {
        const osg::Vec3f ab = b - a, ac = c - a, ap = p - a;
        const float d1 = ab * ap, d2 = ac * ap;
        if (d1 <= 0 && d2 <= 0)
            return { 1, 0, 0 };
        const osg::Vec3f bp = p - b;
        const float d3 = ab * bp, d4 = ac * bp;
        if (d3 >= 0 && d4 <= d3)
            return { 0, 1, 0 };
        const float vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0)
        {
            const float v = d1 / (d1 - d3);
            return { 1 - v, v, 0 };
        }
        const osg::Vec3f cp = p - c;
        const float d5 = ab * cp, d6 = ac * cp;
        if (d6 >= 0 && d5 <= d6)
            return { 0, 0, 1 };
        const float vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0)
        {
            const float w = d2 / (d2 - d6);
            return { 1 - w, 0, w };
        }
        const float va = d3 * d6 - d5 * d4;
        if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0)
        {
            const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return { 0, 1 - w, w };
        }
        const float denominator = va + vb + vc;
        if (!(std::abs(denominator) > 0))
            return { 1, 0, 0 };
        const float v = vb / denominator, w = vc / denominator;
        return { 1 - v - w, v, w };
    }

    // The posed triangle nearest the struck point; its indices address the
    // bind (source) vertices too.
    struct NearestTriangle
    {
        const osg::Vec3Array* mVertices = nullptr;
        osg::Vec3f mPoint;
        float mDistance = std::numeric_limits<float>::max();
        unsigned int mIndex[3] = { 0, 0, 0 };
        osg::Vec3f mWeights;

        void operator()(unsigned int i1, unsigned int i2, unsigned int i3)
        {
            const std::size_t size = mVertices->size();
            if (i1 >= size || i2 >= size || i3 >= size)
                return;
            const osg::Vec3f& a = (*mVertices)[i1];
            const osg::Vec3f& b = (*mVertices)[i2];
            const osg::Vec3f& c = (*mVertices)[i3];
            if (((b - a) ^ (c - a)).length2() < 1e-12f)
                return;
            const osg::Vec3f weights = closestBarycentric(mPoint, a, b, c);
            const float distance = (a * weights.x() + b * weights.y() + c * weights.z() - mPoint).length2();
            if (!(distance < mDistance))
                return;
            mDistance = distance;
            mIndex[0] = i1;
            mIndex[1] = i2;
            mIndex[2] = i3;
            mWeights = weights;
        }
    };

    // Skinning near one triangle as a linear map from the posed frame to the
    // bind pose: the local inverse of the bone transforms (WOUNDFX-01 FX1).
    struct LocalUnskin
    {
        osg::Vec3f mDual[3];
        osg::Vec3f mBind[3];

        bool init(const osg::Vec3f& p0, const osg::Vec3f& p1, const osg::Vec3f& p2, const osg::Vec3f& b0,
            const osg::Vec3f& b1, const osg::Vec3f& b2)
        {
            const osg::Vec3f e1 = p1 - p0, e2 = p2 - p0;
            const osg::Vec3f f1 = b1 - b0, f2 = b2 - b0;
            osg::Vec3f n = e1 ^ e2;
            osg::Vec3f m = f1 ^ f2;
            const float area = n.length();
            const float bindArea = m.length();
            if (!(area > 1e-8f) || !(bindArea > 1e-8f))
                return false;
            n /= area;
            m /= bindArea;
            // Dual basis of (e1, e2, n); e1 . (e2 ^ n) is the area.
            mDual[0] = (e2 ^ n) / area;
            mDual[1] = (n ^ e1) / area;
            mDual[2] = n;
            mBind[0] = f1;
            mBind[1] = f2;
            mBind[2] = m * std::sqrt(bindArea / area);
            return true;
        }

        osg::Vec3f operator()(const osg::Vec3f& v) const
        {
            return mBind[0] * (v * mDual[0]) + mBind[1] * (v * mDual[1]) + mBind[2] * (v * mDual[2]);
        }
    };

    // The geometry holding a wounded drawable's bind pose: a rig's unskinned
    // source, a morphed head's unmorphed source, or a rigid geometry itself
    // (its frame already follows the bone).
    const osg::Geometry* bindGeometry(const osg::Drawable& drawable)
    {
        if (const auto* rig = dynamic_cast<const SceneUtil::RigGeometry*>(&drawable))
            return rig->getSourceGeometry().get();
        if (const auto* morph = dynamic_cast<const SceneUtil::MorphGeometry*>(&drawable))
            return morph->getSourceGeometry().get();
        return drawable.asGeometry();
    }

    const osg::Vec3Array* bindPositions(const osg::Drawable& drawable)
    {
        const osg::Geometry* geometry = bindGeometry(drawable);
        return geometry ? dynamic_cast<const osg::Vec3Array*>(geometry->getVertexArray()) : nullptr;
    }

    // The bind normals that go with bindPositions, when the mesh has them.
    const osg::Vec3Array* bindNormals(const osg::Drawable& drawable)
    {
        const osg::Geometry* geometry = bindGeometry(drawable);
        return geometry ? dynamic_cast<const osg::Vec3Array*>(geometry->getNormalArray()) : nullptr;
    }

    float renderingSimulationTime()
    {
        MWBase::World* world = MWBase::Environment::get().getWorld();
        if (!world || !world->getTimeManager())
            return 0.0f;
        return static_cast<float>(world->getTimeManager()->getRenderingSimulationTime());
    }
}

namespace MWRender
{
    int WoundFxSet::level()
    {
        static const auto value_tuned = RagdollCore::tuned<int>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_WOUND_FX");
            const int number = text ? std::atoi(text) : 0;
            return number >= 1 && number <= 4 ? number : 0;
        });
        static const auto& value = value_tuned.get();
        return value;
    }

    float WoundFxSet::now()
    {
        return renderingSimulationTime();
    }

    void WoundFxSet::setBloodType(int type)
    {
        mBloodType = type == Blood_Skeleton || type == Blood_Dwemer ? type : Blood_Red;
    }

    float WoundFxSet::newSeed()
    {
        return std::fmod(0.6180339f * static_cast<float>(++mSeedCounter), 1.0f);
    }

    int WoundFxSet::drawnKind(int kind, float severity)
    {
        switch (kind)
        {
            case Kind_Bruise:
                // Bruises as volumes only at level 2 (owner 2026-10-08).
                return level() >= 2 ? kind : 0;
            case Kind_Stab:
            case Kind_Chop:
                return kind;
            case Kind_Slash:
                return severity >= sChopSeverity ? Kind_Chop : Kind_Slash;
            default:
                return 0;
        }
    }

    namespace
    {
        const std::array<float, 2>& armourMarkSettings()
        {
            static const auto value_tuned = RagdollCore::tuned<std::array<float, 2>>([] {
                std::array<float, 2> value{ 1.7f, 0.55f };
                if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_ARMOUR_MARKS"))
                    std::sscanf(text, "%f,%f", &value[0], &value[1]);
                value[0] = std::isfinite(value[0]) ? std::clamp(value[0], 0.5f, 3.0f) : 1.0f;
                value[1] = std::isfinite(value[1]) && value[1] > 0.0f ? value[1] : 2.0f;
                return value;
            });
            return value_tuned.get();
        }
    }

    float WoundFxSet::armourMarkScale()
    {
        return armourMarkSettings()[0];
    }

    float WoundFxSet::armourBleedSeverity()
    {
        return armourMarkSettings()[1];
    }

    int WoundFxSet::surfaceKind(int stroke, int surface)
    {
        if (stroke < Kind_Bruise || stroke > Kind_Chop)
            return 0;
        switch (surface)
        {
            case Surface_Skin:
                return stroke;
            case Surface_Cloth:
                // A blow through cloth leaves no blood on it (WOUNDFX-01
                // section 4 item 4).
                return level() >= 3 && stroke != Kind_Bruise ? Kind_Stain : 0;
            case Surface_Armour:
                return level() >= 3 ? Kind_Scratch : 0;
            default:
                return 0;
        }
    }

    float WoundFxSet::stainRadius(int stroke, float halfWidth, float severity)
    {
        // A cut's blood soaks a few gapes out from the line, a stab's more
        // (it bleeds from a deep, narrow hole); never past a hand's width.
        const float strength = std::clamp(severity, 0.0f, 1.0f);
        return std::min(((stroke == Kind_Stab ? 2.6f : 2.0f) + 2.0f * strength) * halfWidth + 0.6f, 4.5f);
    }

    bool WoundFxSet::addWound(osg::Drawable& drawable, osg::Group* parent, const osg::Geometry& posed,
        const osg::Matrix& posedToWorld, const osg::Vec3f& hitLocal, const osg::Vec3f& normal,
        const osg::Vec3f& direction, int kind, float length, float halfWidth, float severity, float seed,
        Shader::ShaderManager& shaderManager, int stroke, bool softArmour, osg::Node* runFrame)
    {
        if (kind < Kind_Bruise || kind > Kind_Scratch)
            return false;
        if ((kind == Kind_Stain || kind == Kind_Scratch) && (stroke < Kind_Bruise || stroke > Kind_Chop))
            return false;
        if (!enabled() || !(length > 0) || !(halfWidth > 0) || !std::isfinite(length) || !std::isfinite(halfWidth)
            || !std::isfinite(severity) || !hitLocal.valid() || !normal.valid() || !direction.valid()
            || normal.length2() < 1e-6f)
            return false;
        // For a rigid part posed is the geometry itself, the bind positions
        // are its own vertices, and the local unskin below is the identity.
        const osg::Vec3Array* bindVertices = bindPositions(drawable);
        const auto* posedVertices = dynamic_cast<const osg::Vec3Array*>(posed.getVertexArray());
        if (!bindVertices || !posedVertices || bindVertices->size() != posedVertices->size())
            return false;

        // The struck point in the bind pose: the same barycentric weights on
        // the same triangle of the source mesh.
        osg::TriangleIndexFunctor<NearestTriangle> nearest;
        nearest.mVertices = posedVertices;
        nearest.mPoint = hitLocal;
        posed.accept(nearest);
        if (nearest.mDistance == std::numeric_limits<float>::max())
            return false;
        const unsigned int i0 = nearest.mIndex[0], i1 = nearest.mIndex[1], i2 = nearest.mIndex[2];
        const osg::Vec3f& w = nearest.mWeights;
        LocalUnskin unskin;
        if (!unskin.init((*posedVertices)[i0], (*posedVertices)[i1], (*posedVertices)[i2], (*bindVertices)[i0],
                (*bindVertices)[i1], (*bindVertices)[i2]))
            return false;
        // The nearest surface point, then the rest of the way through the
        // local unskin: nothing for a wound on this skin, the offset past the
        // edge for one spilling over from a neighbouring part.
        const osg::Vec3f posedNearest
            = (*posedVertices)[i0] * w.x() + (*posedVertices)[i1] * w.y() + (*posedVertices)[i2] * w.z();
        const osg::Vec3f bindCenter
            = (*bindVertices)[i0] * w.x() + (*bindVertices)[i1] * w.y() + (*bindVertices)[i2] * w.z()
            + unskin(hitLocal - posedNearest);

        // World vectors into the posed frame, then through the local unskin.
        const osg::Matrix toLocal = osg::Matrix::inverse(posedToWorld);
        osg::Vec3f n = normal;
        n.normalize();
        osg::Vec3f u = direction - n * (direction * n);
        if (u.length2() < 1e-6f)
            u = n ^ osg::Vec3f(1, 0, 0);
        if (u.length2() < 1e-6f)
            u = n ^ osg::Vec3f(0, 0, 1);
        u.normalize();
        const osg::Vec3f v = n ^ u;

        // The skin's own normal there (bind normals are the source normals).
        osg::Vec3f bindNormal = unskin(osg::Matrix::transform3x3(n, toLocal));
        if (const osg::Vec3Array* normals = bindNormals(drawable);
            normals && normals->size() == bindVertices->size())
        {
            const osg::Vec3f smooth = (*normals)[i0] * w.x() + (*normals)[i1] * w.y() + (*normals)[i2] * w.z();
            if (smooth.length2() > 1e-8f)
                bindNormal = smooth;
        }
        if (!(bindNormal.normalize() > 0))
            return false;
        osg::Vec3f along = unskin(osg::Matrix::transform3x3(u, toLocal));
        const float lengthScale = along.length();
        along -= bindNormal * (along * bindNormal);
        if (!(lengthScale > 0) || !(along.normalize() > 1e-6f))
            return false;
        float widthScale = unskin(osg::Matrix::transform3x3(v, toLocal)).length();
        if (!(widthScale > 0))
            widthScale = lengthScale;
        const float bindLength = length * lengthScale;
        const float scale = 0.5f * (lengthScale + widthScale);

        Record record;
        // A cut runs along the travel: deep where it starts, a tail where it
        // leaves (the shader's spindle). A stab's slit lies along the blade's
        // edge axis; a bruise is a short smear round the struck point.
        record.mStart = bindCenter - along * (0.5f * bindLength);
        record.mEnd = bindCenter + along * (0.5f * bindLength);
        record.mNormal = bindNormal;
        record.mHalfWidth = halfWidth * widthScale;
        record.mSeverity = std::clamp(severity, 0.0f, 1.0f);
        // Depth the slab reaches into the limb: enough for the skin to curve
        // away under a long cut, short of the far side of a forearm. (A
        // bruise is measured in 3D instead, see objects.frag.)
        record.mReach = std::clamp(0.3f * bindLength, 1.2f * scale, 2.6f * scale);
        // A stain is measured in 3D like a bruise (no slab): woundAxis.w
        // carries its full radius instead (WOUNDFX-01 FX3).
        if (kind == Kind_Stain)
            record.mReach = stainRadius(stroke, halfWidth, severity) * scale;
        record.mBirth = renderingSimulationTime();
        record.mSeed = std::isfinite(seed) ? seed : 0.0f;
        record.mKind = kind;
        record.mStroke = kind == Kind_Stain || kind == Kind_Scratch ? stroke : 0;
        record.mSoftArmour = kind == Kind_Scratch && softArmour;
        // FX4: an open wound's blood (and a stain) runs downhill. The run
        // frame's world matrix now, then the posed frame and the local unskin
        // as a matrix: a direction in the run frame into bind space. Later
        // frames give world down in bind space through it (update).
        if (bloodEnabled() && runFrame && kind >= Kind_Slash && kind <= Kind_Stain)
        {
            const osg::MatrixList frames = runFrame->getWorldMatrices();
            if (!frames.empty())
            {
                osg::Matrix unskinMatrix; // identity; v * unskinMatrix = unskin(v)
                for (int row = 0; row < 3; ++row)
                    for (int column = 0; column < 3; ++column)
                        unskinMatrix(row, column) = unskin.mDual[0][row] * unskin.mBind[0][column]
                            + unskin.mDual[1][row] * unskin.mBind[1][column]
                            + unskin.mDual[2][row] * unskin.mBind[2][column];
                record.mRunFromFrame = frames.front() * toLocal * unskinMatrix;
                if (runDirection(record, frames.front(), record.mRunDirection, record.mRunSlope))
                    record.mRunFrame = runFrame;
                else
                {
                    record.mRunDirection = osg::Vec3f();
                    record.mRunSlope = 0;
                }
            }
        }

        Entry* entry = prepare(drawable, parent, shaderManager);
        if (!entry)
            return false;
        // Up to 12 per drawable; the oldest gives way (WOUNDFX-01 section 4).
        while (entry->mRecords.size() >= sMaxWounds)
            entry->mRecords.erase(entry->mRecords.begin());
        entry->mRecords.push_back(record);
        apply(*entry);
        return true;
    }

    bool WoundFxSet::addBladeBlood(osg::Drawable& drawable, osg::Group* parent, const osg::Vec3f& grip,
        const osg::Vec3f& tip, float from, float to, float strength, float birth, float seed, int bloodType,
        Shader::ShaderManager& shaderManager)
    {
        // A weapon is rigid: only a plain geometry (its own vertex space is
        // the blade's frame). Dwemer machines leave no blood.
        if (!bloodEnabled() || !drawable.asGeometry() || bloodType == Blood_Dwemer || !grip.valid() || !tip.valid()
            || (tip - grip).length2() < 1e-4f || !std::isfinite(from) || !std::isfinite(to)
            || !std::isfinite(strength) || !std::isfinite(birth))
            return false;
        from = std::clamp(from, -0.2f, 1.2f);
        to = std::clamp(to, -0.2f, 1.2f);
        if (to < from)
            std::swap(from, to);
        if (to - from < 0.02f)
        {
            from -= 0.01f;
            to += 0.01f;
        }
        strength = std::clamp(strength, 0.0f, 1.0f);
        if (!(strength > 0))
            return false;
        Record record;
        record.mStart = grip;
        record.mEnd = tip;
        record.mNormal = osg::Vec3f(); // (unused for a blade mark)
        record.mHalfWidth = from;
        record.mSeverity = strength;
        record.mReach = to;
        record.mBirth = birth;
        record.mSeed = std::isfinite(seed) ? seed : 0.0f;
        record.mKind = Kind_BladeBlood;
        record.mStroke = bloodType == Blood_Skeleton ? Blood_Skeleton : Blood_Red;
        Entry* entry = prepare(drawable, parent, shaderManager);
        if (!entry)
            return false;
        // Up to sMaxBladeMarks on the blade: the oldest mark gives way.
        const auto isBlade = [](const Record& existing) { return existing.mKind == Kind_BladeBlood; };
        while (static_cast<unsigned int>(std::count_if(entry->mRecords.begin(), entry->mRecords.end(), isBlade))
            >= sMaxBladeMarks)
            entry->mRecords.erase(std::find_if(entry->mRecords.begin(), entry->mRecords.end(), isBlade));
        while (entry->mRecords.size() >= sMaxWounds)
            entry->mRecords.erase(entry->mRecords.begin());
        entry->mRecords.push_back(record);
        apply(*entry);
        return true;
    }

    bool WoundFxSet::runDirection(
        const Record& record, const osg::Matrix& frameToWorld, osg::Vec3f& direction, float& slope)
    {
        osg::Matrix worldToFrame;
        if (!worldToFrame.invert(frameToWorld))
            return false;
        osg::Vec3f down = osg::Matrix::transform3x3(osg::Vec3f(0, 0, -1), worldToFrame);
        down = osg::Matrix::transform3x3(down, record.mRunFromFrame);
        if (!down.valid() || !(down.normalize() > 0))
            return false;
        // Downhill along the skin; how steep the skin is there (0 facing
        // straight up or down: the blood pools or drips off, 1 vertical).
        const osg::Vec3f along = down - record.mNormal * (down * record.mNormal);
        const float length = along.length();
        slope = std::clamp(length, 0.0f, 1.0f);
        if (length > 1e-3f)
            direction = along / length;
        else if (direction.length2() < 0.25f)
        {
            direction = record.mNormal ^ osg::Vec3f(1, 0, 0);
            if (direction.length2() < 1e-4f)
                direction = record.mNormal ^ osg::Vec3f(0, 0, 1);
            if (!(direction.normalize() > 0))
                return false;
        }
        return true;
    }

    void WoundFxSet::update()
    {
        if (!bloodEnabled() || mEntries.empty())
            return;
        const float time = now();
        const auto running = [time](const Record& record) {
            return record.mRunFrame.valid() && time >= record.mBirth && time - record.mBirth < sRunSeconds;
        };
        for (Entry& entry : mEntries)
        {
            if (std::none_of(entry.mRecords.begin(), entry.mRecords.end(), running))
                continue;
            // About ten times a second: each change is a new state set.
            if (entry.mLastRunUpdate >= 0 && time >= entry.mLastRunUpdate && time - entry.mLastRunUpdate < 0.1f)
                continue;
            osg::ref_ptr<osg::Drawable> drawable;
            if (!entry.mDrawable.lock(drawable) || drawable->getStateSet() != entry.mApplied.get())
                continue;
            const float step = entry.mLastRunUpdate >= 0 && time > entry.mLastRunUpdate
                ? std::min(time - entry.mLastRunUpdate, 1.0f)
                : 0.1f;
            entry.mLastRunUpdate = time;
            // The streak follows gravity smoothly: a limb waved about does
            // not swing it round at once.
            const float follow = 1.0f - std::exp(-step / 0.4f);
            bool changed = false;
            for (Record& record : entry.mRecords)
            {
                if (!running(record))
                    continue;
                osg::ref_ptr<osg::Node> frame;
                if (!record.mRunFrame.lock(frame))
                    continue;
                const osg::MatrixList frames = frame->getWorldMatrices();
                if (frames.empty())
                    continue;
                osg::Vec3f direction = record.mRunDirection;
                float slope = record.mRunSlope;
                if (!runDirection(record, frames.front(), direction, slope))
                    continue;
                osg::Vec3f blended = record.mRunDirection * (1.0f - follow) + direction * follow;
                record.mRunDirection = blended.normalize() > 1e-3f ? blended : direction;
                record.mRunSlope += (slope - record.mRunSlope) * follow;
                changed = true;
            }
            if (changed)
                apply(entry);
        }
    }

    WoundFxSet::Entry* WoundFxSet::prepare(
        osg::Drawable& drawable, osg::Group* parent, Shader::ShaderManager& shaderManager)
    {
        mEntries.erase(std::remove_if(mEntries.begin(), mEntries.end(),
                           [](const Entry& entry) { return !entry.mDrawable.valid(); }),
            mEntries.end());
        auto found = std::find_if(mEntries.begin(), mEntries.end(),
            [&](const Entry& entry) { return entry.mDrawable.get() == &drawable; });
        Entry* entry = nullptr;
        if (found != mEntries.end())
        {
            entry = &*found;
            // Someone else replaced the drawable's state: build on theirs.
            if (drawable.getStateSet() != entry->mApplied.get())
            {
                entry->mBase = drawable.getStateSet();
                entry->mProgram = nullptr;
            }
        }
        else if (auto* rig = dynamic_cast<SceneUtil::RigGeometry*>(&drawable))
        {
            const osg::ref_ptr<osg::Geometry> source = rig->getSourceGeometry();
            const auto* vertices = source ? dynamic_cast<const osg::Vec3Array*>(source->getVertexArray()) : nullptr;
            if (!vertices)
                return nullptr;
            const osg::Array* existing = source->getTexCoordArray(sBindPositionUnit);
            if (existing && existing->getName() != sBindArrayName)
            {
                static bool warned = false;
                if (!warned)
                    Log(Debug::Warning) << "WOUNDFX01 texture unit " << sBindPositionUnit << " is used by "
                                        << rig->getName() << "; no wound volumes on it";
                warned = true;
                return nullptr;
            }
            if (!existing)
            {
                // The bind position: a copy of the source positions with its
                // own static buffer, shared by both skinned copies.
                osg::ref_ptr<osg::Vec3Array> bind = new osg::Vec3Array(osg::Array::BIND_PER_VERTEX);
                bind->reserve(vertices->size());
                bind->insert(bind->end(), vertices->begin(), vertices->end());
                bind->setName(sBindArrayName);
                bind->setVertexBufferObject(new osg::VertexBufferObject);
                rig->setSharedTexCoordArray(sBindPositionUnit, bind);
            }
            mEntries.emplace_back();
            entry = &mEntries.back();
            entry->mDrawable = rig;
            entry->mBase = rig->getStateSet();
        }
        else if (auto* morph = dynamic_cast<SceneUtil::MorphGeometry*>(&drawable))
        {
            // A morphed head (WOUNDFX-01 FX2): the bind pose is the unmorphed
            // source; the expressions move the face too little to matter.
            // Morphed drawables are cloned per actor instance (SceneUtil
            // CopyOp), so this one is ours: its source becomes a shallow copy
            // carrying the source positions again on unit 6 (the same static
            // array, no copy), and setSourceGeometry builds new morphed copies
            // sharing it. The draw thread keeps last frame's copy as is.
            const osg::ref_ptr<osg::Geometry> source = morph->getSourceGeometry();
            osg::Array* vertices = source ? source->getVertexArray() : nullptr;
            if (!dynamic_cast<osg::Vec3Array*>(vertices))
                return nullptr;
            const osg::Array* existing = source->getTexCoordArray(sBindPositionUnit);
            if (existing && existing != vertices)
            {
                static bool warned = false;
                if (!warned)
                    Log(Debug::Warning) << "WOUNDFX01 texture unit " << sBindPositionUnit << " is used by "
                                        << morph->getName() << "; no wound volumes on it";
                warned = true;
                return nullptr;
            }
            if (!existing)
            {
                osg::ref_ptr<osg::Geometry> carrier = new osg::Geometry(*source, osg::CopyOp::SHALLOW_COPY);
                carrier->setTexCoordArray(sBindPositionUnit, vertices, osg::Array::BIND_PER_VERTEX);
                morph->setSourceGeometry(carrier);
                // The new copies hold the unmorphed shape until the next morph.
                morph->dirty();
            }
            mEntries.emplace_back();
            entry = &mEntries.back();
            entry->mDrawable = morph;
            entry->mBase = morph->getStateSet();
        }
        else if (osg::Geometry* geometry = drawable.asGeometry())
        {
            // A rigid part (WOUNDFX-01 FX1): its vertex space is the bind
            // space, already carried by the bone. The geometry can be a
            // template shared by every actor wearing the part (scene cache),
            // so this actor gets a shallow copy carrying the vertex array
            // again on unit 6 (the same array: no copy, no per-frame cost),
            // swapped in under this actor's own parent.
            osg::Array* vertices = geometry->getVertexArray();
            if (!dynamic_cast<osg::Vec3Array*>(vertices))
                return nullptr;
            const osg::Array* existing = geometry->getTexCoordArray(sBindPositionUnit);
            if (existing && existing != vertices)
            {
                static bool warned = false;
                if (!warned)
                    Log(Debug::Warning) << "WOUNDFX01 texture unit " << sBindPositionUnit << " is used by "
                                        << geometry->getName() << "; no wound volumes on it";
                warned = true;
                return nullptr;
            }
            osg::ref_ptr<osg::Geometry> target = geometry;
            if (!existing)
            {
                if (!parent || !parent->containsNode(geometry))
                    return nullptr;
                target = new osg::Geometry(*geometry, osg::CopyOp::SHALLOW_COPY);
                target->setTexCoordArray(sBindPositionUnit, vertices, osg::Array::BIND_PER_VERTEX);
                if (!parent->replaceChild(geometry, target))
                    return nullptr;
            }
            mEntries.emplace_back();
            entry = &mEntries.back();
            entry->mDrawable = target.get();
            entry->mBase = target->getStateSet();
        }
        else
            return nullptr;

        if (!entry->mProgram)
        {
            // The program this drawable is drawn with: its own state first
            // (as it was before FX1), then up the path.
            const osg::Program* program = nullptr;
            if (entry->mBase)
                program = dynamic_cast<const osg::Program*>(entry->mBase->getAttribute(osg::StateAttribute::PROGRAM));
            osg::ref_ptr<osg::Drawable> target;
            if (!entry->mDrawable.lock(target))
                return nullptr;
            const osg::NodePathList paths = target->getParentalNodePaths();
            if (!program && !paths.empty())
            {
                const osg::NodePath& path = paths.front();
                for (auto it = path.rbegin(); it != path.rend() && !program; ++it)
                {
                    if (*it == target.get() || !(*it)->getStateSet())
                        continue;
                    program = dynamic_cast<const osg::Program*>(
                        (*it)->getStateSet()->getAttribute(osg::StateAttribute::PROGRAM));
                }
            }
            // FX4 (level 4): the variant with the blood code (woundVolumes=2);
            // levels 1-3 keep theirs exactly.
            if (program)
                entry->mProgram
                    = shaderManager.getProgramVariant(program,
                        { { "woundVolumes", bloodEnabled() ? "2" : "1" },
                            // ARMOUR_MARKS: darker dents and scored lines
                            { "woundArmourBold", armourMarkScale() > 1.01f ? "1" : "0" } });
            if (!entry->mProgram)
            {
                static bool warned = false;
                if (!warned)
                    Log(Debug::Warning) << "WOUNDFX01 no shader recipe for " << target->getName()
                                        << "; no wound volumes on it";
                warned = true;
                return nullptr;
            }
        }
        return entry;
    }

    void WoundFxSet::apply(Entry& entry)
    {
        osg::ref_ptr<osg::Drawable> drawable;
        if (!entry.mDrawable.lock(drawable))
            return;
        // Always a fresh state set: the one in use may still be drawn.
        osg::ref_ptr<osg::StateSet> state = entry.mBase
            ? new osg::StateSet(*entry.mBase, osg::CopyOp::SHALLOW_COPY)
            : new osg::StateSet;
        state->setAttributeAndModes(entry.mProgram, osg::StateAttribute::ON);
        const int count = static_cast<int>(std::min<std::size_t>(entry.mRecords.size(), sMaxWounds));
        osg::ref_ptr<osg::Uniform> start = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "woundStart", sMaxWounds);
        osg::ref_ptr<osg::Uniform> end = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "woundEnd", sMaxWounds);
        osg::ref_ptr<osg::Uniform> axis = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "woundAxis", sMaxWounds);
        osg::ref_ptr<osg::Uniform> info = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "woundInfo", sMaxWounds);
        for (unsigned int i = 0; i < sMaxWounds; ++i)
        {
            Record record;
            if (i < static_cast<unsigned int>(count))
                record = entry.mRecords[i];
            start->setElement(i, osg::Vec4f(record.mStart, record.mHalfWidth));
            end->setElement(i, osg::Vec4f(record.mEnd, record.mSeverity));
            axis->setElement(i, osg::Vec4f(record.mNormal, record.mReach));
            info->setElement(i,
                osg::Vec4f(record.mBirth, record.mSeed, static_cast<float>(record.mKind),
                    static_cast<float>(record.mStroke + (record.mSoftArmour ? sSoftArmourFlag : 0))));
        }
        state->addUniform(new osg::Uniform("woundCount", count));
        state->addUniform(start);
        state->addUniform(end);
        state->addUniform(axis);
        state->addUniform(info);
        if (bloodEnabled())
        {
            // FX4: each wound's downhill run (zero: none) and the actor's
            // blood type.
            osg::ref_ptr<osg::Uniform> run = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "woundRun", sMaxWounds);
            for (unsigned int i = 0; i < sMaxWounds; ++i)
            {
                osg::Vec4f value;
                if (i < static_cast<unsigned int>(count) && entry.mRecords[i].mRunDirection.length2() > 0.25f)
                    value = osg::Vec4f(entry.mRecords[i].mRunDirection, entry.mRecords[i].mRunSlope);
                run->setElement(i, value);
            }
            state->addUniform(run);
            state->addUniform(new osg::Uniform("woundBlood", mBloodType));
        }
        drawable->setStateSet(state);
        entry.mApplied = state;
    }

    void WoundFxSet::clear()
    {
        for (Entry& entry : mEntries)
        {
            osg::ref_ptr<osg::Drawable> drawable;
            if (entry.mDrawable.lock(drawable) && drawable->getStateSet() == entry.mApplied.get())
                drawable->setStateSet(entry.mBase.get());
        }
        mEntries.clear();
    }

    std::size_t WoundFxSet::getCount() const
    {
        std::size_t count = 0;
        for (const Entry& entry : mEntries)
            if (entry.mDrawable.valid())
                count += entry.mRecords.size();
        return count;
    }
}

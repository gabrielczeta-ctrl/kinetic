#ifndef OPENMW_MWRENDER_WOUNDFX_H
#define OPENMW_MWRENDER_WOUNDFX_H

#include <cstddef>
#include <vector>

#include <osg/Drawable>
#include <osg/Matrix>
#include <osg/Program>
#include <osg/StateSet>
#include <osg/Vec3f>
#include <osg/observer_ptr>
#include <osg/ref_ptr>

namespace osg
{
    class Drawable;
    class Geometry;
    class Group;
}

namespace SceneUtil
{
    class RigGeometry;
}

namespace Shader
{
    class ShaderManager;
}

namespace MWRender
{
    /// WOUNDFX-01 FX1/FX2: wound volumes in the skin shader. Each wounded
    /// drawable gets its bind-pose positions as texture array 6, the
    /// woundVolumes program variant and up to 12 wound records as uniforms.
    /// FX3 (level 3): the same records on clothing (a blood stain) and armour
    /// (a scratch or a dent).
    /// Skinned parts (SceneUtil::RigGeometry) use their source positions;
    /// morphed heads (SceneUtil::MorphGeometry) their unmorphed source; rigid
    /// parts (a plain osg::Geometry on the bone, like the vanilla forearms)
    /// their own vertex space, which already follows the bone. Unwounded
    /// drawables are untouched. Main thread only.
    /// FX4 (level 4, the woundVolumes=2 program variant): blood runs downhill
    /// from fresh open wounds on skin and stains sag a little (see update), the
    /// actor's blood type (red, skeleton dust, Dwemer metal), and blood marks
    /// along a held weapon's blade (addBladeBlood).
    class WoundFxSet
    {
    public:
        static constexpr unsigned int sBindPositionUnit = 6;
        static constexpr unsigned int sMaxWounds = 12;
        /// FX4: blood marks kept on one weapon drawable (the oldest gives way).
        static constexpr unsigned int sMaxBladeMarks = 3;
        /// FX4: how long (simulation seconds) a fresh wound's blood still runs
        /// and follows gravity; afterwards its streak stays where it dried.
        static constexpr float sRunSeconds = 10.0f;

        /// The shader's wound kinds (woundInfo.z). 1-3 are the receipt kinds
        /// (RagdollCore::WoundKind); a chop is a heavy slash. 5 and 6 are the
        /// FX3 surface kinds; their woundInfo.w is the stroke (1-4) that made
        /// them.
        enum Kind : int
        {
            Kind_Bruise = 1,
            Kind_Slash = 2,
            Kind_Stab = 3,
            Kind_Chop = 4,
            Kind_Stain = 5, // clothing: blood soaking out from the cut, a slit for an edge
            Kind_Scratch = 6, // armour: a scored line and dent for an edge, a dent for a point or a blunt blow
            Kind_BladeBlood = 7, // FX4: blood along a weapon's blade (woundInfo.w = the blood type)
        };
        /// Morrowind's blood types (ESM::NPC/Creature::mBloodType, the
        /// Blood_Texture_N settings spawnRagdollBlood uses).
        enum BloodType : int
        {
            Blood_Red = 0,
            Blood_Skeleton = 1, // white dust: no blood, a cut is a dry crack in bone
            Blood_Dwemer = 2, // metal sparks: no blood, a cut is a scratch in metal
        };
        /// What the struck part slot holds (FX3).
        enum Surface : int
        {
            Surface_Skin = 0,
            Surface_Cloth = 1,
            Surface_Armour = 2,
        };
        /// Added to a scratch's woundInfo.w (its stroke 1-4) when the armour
        /// is leather-like (FX3): a matte scuffed cut with a frayed edge
        /// instead of bright scored metal (objects.frag).
        static constexpr int sSoftArmourFlag = 8;
        /// A slash this severe is drawn as a chop (WOUNDFX-01 FX2).
        static constexpr float sChopSeverity = 0.8f;

        /// The kind drawn for a wound kind (1 bruise, 2 slash, 3 stab, 4 chop)
        /// and its severity; 0 when the kind has no volume (FX off, or a
        /// bruise below level 2).
        static int drawnKind(int kind, float severity);
        /// The kind drawn on a surface for a skin kind from drawnKind (the
        /// stroke): the skin kind itself on skin; at level 3 a stain on
        /// clothing (none for a bruise: no blood) and a scratch on armour; 0
        /// otherwise.
        static int surfaceKind(int stroke, int surface);
        /// A stain's full radius round the cut line (world units) for a
        /// stroke of that half gape and severity; it grows to it over the
        /// first seconds (objects.frag).
        static float stainRadius(int stroke, float halfWidth, float severity);

        /// OPENMW_RAGDOLL_WOUND_FX, read once: 0 off, 1 slashes, stabs and
        /// chops as volumes (bruises keep the mesh-copy marks), 2 bruises as
        /// volumes too, 3 clothing stains and armour scratches too, 4 blood
        /// (runs, blood types, blood on weapons) too.
        static int level();
        static bool enabled() { return level() >= 1; }
        static bool bloodEnabled() { return level() >= 4; }
        /// Armour marks (user 10-31: "make armour hits clearly visible ...
        /// blood seeping out at the armour's edges on hard hits"):
        /// OPENMW_RAGDOLL_ARMOUR_MARKS = "size, bleed" - a scratch or dent on
        /// armour drawn `size` times as long and wide, and a blow of severity
        /// `bleed` or more (red blood, FX4) also oozes blood from the mark
        /// down the plate. "1,0" = before (small faint marks, no blood).
        static float armourMarkScale();
        static float armourBleedSeverity(); // >1 = never

        /// FX4: this actor's blood type (BloodType), for the wounds applied
        /// from now on. Red when unset or unknown.
        void setBloodType(int type);
        int getBloodType() const { return mBloodType; }

        /// A noise seed for one wound; the parts one wound spills over share
        /// it.
        float newSeed();

        /// Add one wound of a drawn kind (see Kind) to `drawable` (a
        /// RigGeometry, a MorphGeometry or a plain Geometry) under `parent`
        /// (this actor's node holding it). `posed` is the drawn shape in the
        /// vertex order of the source (the deformed copy, or the geometry
        /// itself), `posedToWorld` its local-to-world matrix and `hitLocal`
        /// the wound's middle in that frame: on the struck skin, or past this
        /// drawable's edge when the wound spills over from a neighbouring
        /// part. `normal` (out of the skin) and `direction` (the stroke; a
        /// stab's edge axis) are world vectors; `length` (the segment) and
        /// `halfWidth` (the half gape; a bruise's radius) are world units;
        /// severity is 0..1. `stroke` is the skin kind that made a stain or a
        /// scratch (1 bruise, 2 slash, 3 stab, 4 chop); ignored for 1-4.
        /// `softArmour`: a scratch on leather-like armour (see
        /// sSoftArmourFlag); ignored for the other kinds.
        /// `runFrame` (FX4): a node the skin round the wound moves with (the
        /// region's bone for a skinned part, the parent of a rigid one); a
        /// fresh cut's blood and a stain run downhill in it (see update).
        /// Null or below level 4: no run.
        bool addWound(osg::Drawable& drawable, osg::Group* parent, const osg::Geometry& posed,
            const osg::Matrix& posedToWorld, const osg::Vec3f& hitLocal, const osg::Vec3f& normal,
            const osg::Vec3f& direction, int kind, float length, float halfWidth, float severity, float seed,
            Shader::ShaderManager& shaderManager, int stroke = 0, bool softArmour = false,
            osg::Node* runFrame = nullptr);

        /// FX4: a blood mark along a rigid weapon drawable (a plain
        /// osg::Geometry under `parent`). `grip` and `tip` are the blade's ends
        /// in the drawable's own vertex space; `from` and `to` the bloodied
        /// range as fractions of grip-to-tip (0 the grip, 1 the tip); strength
        /// 0..1; birth: when it was made (simulation seconds: a mark restored
        /// on a rebuilt weapon keeps drying); bloodType: the victim's
        /// (BloodType; Dwemer leaves none). Up to sMaxBladeMarks per drawable.
        bool addBladeBlood(osg::Drawable& drawable, osg::Group* parent, const osg::Vec3f& grip,
            const osg::Vec3f& tip, float from, float to, float strength, float birth, float seed, int bloodType,
            Shader::ShaderManager& shaderManager);

        /// FX4, once a frame on the main thread: the fresh wounds' blood
        /// keeps running downhill (world down in each wound's bind frame, from
        /// its run frame's current world matrix), for sRunSeconds after each
        /// hit; only drawables with such wounds are touched, about ten times a
        /// second. No-op below level 4.
        void update();

        /// Forget every record and give the drawables their own state back
        /// (the hub reset). The shared bind array stays; it is inert.
        void clear();

        std::size_t getCount() const;

        /// The simulation time the shader ages wounds by.
        static float now();

    private:
        struct Record
        {
            osg::Vec3f mStart, mEnd, mNormal;
            float mHalfWidth = 0, mSeverity = 0, mReach = 0, mBirth = 0, mSeed = 0;
            int mKind = 2;
            int mStroke = 0; // a stain's or a scratch's stroke (woundInfo.w); a blade mark's blood type
            bool mSoftArmour = false; // a scratch on leather-like armour (woundInfo.w + sSoftArmourFlag)
            // FX4: the downhill run (woundRun): the node the skin moves with,
            // a direction in its space into bind space (as at placement), and
            // the run's current bind-space direction along the skin and slope.
            osg::observer_ptr<osg::Node> mRunFrame;
            osg::Matrix mRunFromFrame;
            osg::Vec3f mRunDirection;
            float mRunSlope = 0;
        };
        struct Entry
        {
            osg::observer_ptr<osg::Drawable> mDrawable; // the rig, or this actor's copy of a rigid geometry
            osg::ref_ptr<osg::StateSet> mBase; // the drawable's own state before FX1 (may be null)
            osg::ref_ptr<osg::StateSet> mApplied; // ours, while the drawable still has it
            osg::ref_ptr<osg::Program> mProgram; // the woundVolumes variant
            std::vector<Record> mRecords;
            float mLastRunUpdate = -1; // FX4: simulation time the runs were last followed
        };
        std::vector<Entry> mEntries;
        unsigned int mSeedCounter = 0;
        int mBloodType = Blood_Red;

        /// FX4: world down in a record's bind frame along its skin, with the
        /// run frame at `frameToWorld`; false when it cannot be told.
        static bool runDirection(const Record& record, const osg::Matrix& frameToWorld, osg::Vec3f& direction,
            float& slope);

        Entry* prepare(osg::Drawable& drawable, osg::Group* parent, Shader::ShaderManager& shaderManager);
        void apply(Entry& entry);
    };
}

#endif

#version 120

#pragma import_defines(FORCE_OPAQUE, DISTORTION)

#if @useGPUShader4
    #extension GL_EXT_gpu_shader4: require
#endif

#if @diffuseMap
uniform sampler2D diffuseMap;
varying vec2 diffuseMapUV;
#endif

#if @darkMap
uniform sampler2D darkMap;
varying vec2 darkMapUV;
#endif

#if @detailMap
uniform sampler2D detailMap;
varying vec2 detailMapUV;
#endif

#if @decalMap
uniform sampler2D decalMap;
varying vec2 decalMapUV;
#endif

#if @emissiveMap
uniform sampler2D emissiveMap;
varying vec2 emissiveMapUV;
#endif

#if @normalMap
uniform sampler2D normalMap;
varying vec2 normalMapUV;
#endif

#if @envMap
uniform sampler2D envMap;
varying vec2 envMapUV;
uniform vec4 envMapColor;
#endif

#if @specularMap
uniform sampler2D specularMap;
varying vec2 specularMapUV;
#endif

#if @bumpMap
uniform sampler2D bumpMap;
varying vec2 bumpMapUV;
uniform vec2 envMapLumaBias;
uniform mat2 bumpMapMatrix;
#endif

#if @glossMap
uniform sampler2D glossMap;
varying vec2 glossMapUV;
#endif

uniform vec2 screenRes;
uniform float near;
uniform float far;
uniform float alphaRef;
// Development injury visualization, neutral for every unaffected surface.
uniform float anatomicalDamageTint;
uniform float distortionStrength;
uniform float alpha;
uniform float actorFade;
// KINETIC (10-09, user: "make the character model not block the player
// vision"): the player's own body and weapon dissolve near the first-person
// camera. x = view distance fully gone, y = distance fully solid at the edge
// of the screen, z = extra solid distance toward the centre (share of y).
// Zero (the default everywhere else) = off.
uniform vec3 nearDissolve;

// WOUNDFX-01 FX1: wounded drawables are always lit per pixel.
#define PER_PIXEL_LIGHTING (@normalMap || @specularMap || @forcePPL || @woundVolumes)

centroid varying vec4 passColor;

#if !PER_PIXEL_LIGHTING
centroid varying vec3 shadedLighting;
centroid varying vec3 shadedSpecular;
centroid varying vec3 passLighting;
centroid varying vec3 passSpecular;
#else
#include "lib/light/clamp.glsl"
#endif
varying vec3 passViewPos;
varying vec3 passNormal;
#if @normalMap || @diffuseParallax
varying vec4 passTangent;
#endif

#if @additiveBlending
#define ADDITIVE_BLENDING
#endif

#include "lib/core/fragment.h.glsl"
#include "lib/material/parallax.glsl"
#include "lib/material/alpha.glsl"
#include "lib/util/distortion.glsl"

#include "fog.glsl"
#include "lib/material/vertexcolors.glsl"
#include "shadows_fragment.glsl"
#include "compatibility/normals.glsl"

#if @softParticles
#include "lib/particle/soft.glsl"

uniform float particleSize;
uniform bool particleFade;
uniform float softFalloffDepth;
#endif

#if @particleOcclusion
#include "lib/particle/occlusion.glsl"
uniform sampler2D orthoDepthMap;
varying vec3 orthoDepthMapCoord;
#endif

#if @woundVolumes
// WOUNDFX-01 FX1/FX2: wounds as volumes in the part's bind pose (Left 4 Dead
// 2's idea), drawn per pixel, so they stay on the skin through every bend.
// Kinds (woundInfo.z): 1 bruise, 2 slash, 3 stab, 4 chop; FX3: 5 stain (on
// clothing), 6 scratch or dent (on armour), whose woundInfo.w is the stroke
// that made them (1 blunt, 2 slash, 3 stab, 4 chop), plus 8 on leather-like
// armour (WoundFxSet::sSoftArmourFlag: a matte scuffed cut, no metal).
#define WOUND_MAX 12
varying vec3 woundBindPos;
uniform int woundCount;
uniform vec4 woundStart[WOUND_MAX]; // bind-space start of the stroke, half gape (a bruise's radius)
uniform vec4 woundEnd[WOUND_MAX];   // bind-space end of the stroke, severity 0..1
uniform vec4 woundAxis[WOUND_MAX];  // bind-space skin normal, depth reach (a stain: its full radius)
uniform vec4 woundInfo[WOUND_MAX];  // birth (simulation seconds), seed, kind, stroke (kinds 5 and 6)
// ARMOUR_MARKS (user 10-31: armour hits "clearly visible"): 1 = darker dents
// and scored lines, a dark scuff round a line and a wider channel.
const float woundBold = float(@woundArmourBold);
uniform float osg_SimulationTime;
// One pixel's extent in bind units, set in main before the wounds are drawn:
// the scored lines' fine relief fades where a pixel covers it (no sparkle).
float woundFootprint = 0.0;
#if @woundVolumes >= 2
// WOUNDFX-01 FX4 (OPENMW_RAGDOLL_WOUND_FX=4, the woundVolumes=2 variant):
// blood. Kind 7 is a blood mark on a weapon's blade: woundStart.xyz the grip
// and woundEnd.xyz the tip (the drawable's own space), woundStart.w and
// woundAxis.w the bloodied range (0 grip .. 1 tip), woundEnd.w its strength,
// woundInfo.w the victim's blood type.
uniform vec4 woundRun[WOUND_MAX]; // bind-space downhill direction along the skin, w: the slope (0 level .. 1 vertical)
uniform int woundBlood;           // the actor's blood: 0 red, 1 skeleton (dust), 2 Dwemer (no blood)
// A stain's distance from its cut with its downhill sag (woundHeight sets it
// for woundSurfaceHeight); -1: none.
float woundStainThrough = -1.0;
// How much of the pixel is covered by blood: it dulls a weapon's shine.
float woundCover = 0.0;
#endif

// Hash without sine (Dave Hoskins) and value noise on it: no texture, no seams.
float woundHash(vec3 p)
{
    p = fract(p * 0.1031);
    p += dot(p, p.zyx + 31.32);
    return fract((p.x + p.y) * p.z);
}

float woundNoise(vec3 p)
{
    vec3 i = floor(p);
    vec3 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float a = mix(woundHash(i), woundHash(i + vec3(1.0, 0.0, 0.0)), f.x);
    float b = mix(woundHash(i + vec3(0.0, 1.0, 0.0)), woundHash(i + vec3(1.0, 1.0, 0.0)), f.x);
    float c = mix(woundHash(i + vec3(0.0, 0.0, 1.0)), woundHash(i + vec3(1.0, 0.0, 1.0)), f.x);
    float d = mix(woundHash(i + vec3(0.0, 1.0, 1.0)), woundHash(i + vec3(1.0, 1.0, 1.0)), f.x);
    return mix(mix(a, b, f.y), mix(c, d, f.y), f.z);
}

int woundKind(int i)
{
    return int(woundInfo[i].z + 0.5);
}

int woundStroke(int i)
{
    return int(mod(floor(woundInfo[i].w + 0.5), 8.0) + 0.5);
}

// A scratch or dent on leather-like armour (FX3).
bool woundSoft(int i)
{
    return woundInfo[i].w > 7.5;
}

// How much of a pixel a thin line covers (FX3): line.x is the signed distance
// from its edge (bind units, negative inside), line.y its half width. A line
// thinner than a pixel fades instead of shimmering.
float woundLine(vec2 line, float footprint)
{
    float pixel = max(footprint, 0.00001);
    return clamp(0.5 - line.x / pixel, 0.0, 1.0) * min(2.0 * line.y / pixel, 1.0);
}

// WOUNDFX-01 FX3, the surface kinds, with woundHeight's outputs (below).
// 5 stain: blood soaking out from the cut line through the cloth, an
// irregular edge, growing a little over the first seconds; a frayed slit where
// an edge cut the cloth. across: 0 on the line, 1 at the stain's edge; core:
// how dense (thickest along the line); halo: the stain. 6 scratch: an edge
// scores a bright channel (tapering where the stroke leaves) with the metal
// dented beside it; a point dents a small dish with a bright pit (core); a
// blunt blow presses in a dish about its radius. On leather the same strokes
// cut a frayed slit, punch a hole or press a shallow scuff. For an edge,
// across is the signed distance from the line in line half widths (positive
// on the dented side) and core the stroke's profile; for a dent, across is
// the radius fraction. halo: the dent's shade (its occlusion). line: the
// slit or the scored line (see woundLine; y = 0 for none).
float woundSurfaceHeight(int i, int kind, float along, float len, float x, float depth, float beyond,
    float halfGape, float severity, out float across, out float reach, out float core, out float halo,
    out vec2 line)
{
    // Out parameters start undefined: every return path needs them set.
    across = 100.0;
    reach = 0.0;
    core = 0.0;
    halo = 0.0;
    line = vec2(0.0);
    int stroke = woundStroke(i);
    float through = length(vec3(x, beyond, depth));
    float c = clamp(along, 0.0, 1.0);
    vec3 q = vec3(0.8 * along * len + 0.6 * x, 0.8 * x - 0.6 * along * len, depth) * 1.7
        + woundInfo[i].y * vec3(61.0, 83.0, 47.0);
    if (kind == 5)
    {
#if @woundVolumes >= 2
        // FX4: the stain sags downhill (woundHeight).
        if (woundStainThrough >= 0.0)
            through = woundStainThrough;
#endif
        // Measured in 3D like a bruise (no slab): a sleeve's far side is
        // farther than the radius.
        float age = max(osg_SimulationTime - woundInfo[i].x, 0.0);
        float radius = max(woundAxis[i].w, 0.05) * (0.55 + 0.45 * (1.0 - exp(-age / 4.0)));
        // (Past 1.65 radii no lobe reaches: no clipped edge.)
        if (through > 1.65 * radius)
            return 0.0;
        reach = 1.0;
        // Broad lobes and smaller wicking, and fibrous streaks along two
        // thread directions where the blood runs through the weave.
        float lobes = (woundNoise(q * 0.35) - 0.5) * 0.6 + (woundNoise(q * 1.1) - 0.5) * 0.3;
        float weave = woundNoise(vec3(q.x * 5.0, q.y * 0.8, q.z))
            + woundNoise(vec3(q.x * 0.8, q.y * 5.0, q.z + 13.0)) - 1.0;
        across = through / radius + lobes + 0.16 * weave;
        halo = 1.0 - smoothstep(0.88, 1.0, across);
        core = 1.0 - smoothstep(0.0, 0.85, through / radius);
        if (stroke == 1 || abs(beyond) > 0.02 * len)
            return 0.0;
        // The slit: the cloth parted along the edge (a stab's short one is
        // pointed at both ends), its threads frayed out of the edge.
        float shape = stroke == 3 ? 1.0 - (2.0 * c - 1.0) * (2.0 * c - 1.0)
                                  : max(sin(3.14159265 * pow(c, 0.7)), 0.0);
        float slitHalf = (stroke == 4 ? 0.3 : 0.16) * halfGape * sqrt(shape);
        float fray = (woundNoise(q * 7.0) - 0.5) * 0.6 + (woundNoise(q * 15.0) - 0.5) * 0.3;
        float xs = abs(x - (woundNoise(q * 0.45) - 0.5) * 0.3 * halfGape);
        line = vec2(xs - slitHalf * (1.0 + fray), slitHalf);
        // The parted cloth sinks a little; the stain is flat.
        return -(0.03 + 0.04 * severity) * shape * (1.0 - smoothstep(0.0, 2.0 * slitHalf + 0.001, xs));
    }
    bool soft = woundSoft(i);
    if (stroke == 1 && !soft)
    {
        // A blunt dent in metal: a dish about the blow's radius, a flattish
        // bottom and steep walls whose tilt turns the light (a shaded side
        // and a bright crescent on the lip facing the light), a slight raised
        // lip, and a few short radial creases where the plate folded round
        // its edge. across: the radius fraction (a slightly uneven outline);
        // halo: the bottom's occlusion; core: the lip; line: the nearest
        // crease (y = 0 for none).
        float radius = max(1.15 * halfGape, 0.3);
        float r = through / radius;
        if (r > 1.4)
            return 0.0;
        reach = 1.0;
        float rr = r + (woundNoise(q * 0.6) - 0.5) * 0.1;
        across = rr;
        float bowl = 1.0 - smoothstep(0.2, 1.0, rr);
        halo = 1.0 - smoothstep(0.0, 0.7, rr);
        core = smoothstep(0.88, 1.03, rr) * (1.0 - smoothstep(1.03, 1.32, rr));
        float sink = (0.3 + 0.22 * severity) * radius;
        float fold = 0.0;
        if (r > 0.6)
        {
            // Seven slots round the dent's middle (in the wound's frame, so
            // seams match); a fold in most, each its own length, bending a
            // little with the noise.
            float cx = x;
            float cy = (along - 0.5) * len;
            float turns = atan(cy, cx) * (7.0 / 6.28318531)
                + 0.3 * (woundNoise(vec3(cx, cy, depth) * (1.5 / radius) + woundInfo[i].y * 40.0) - 0.5);
            float slot = floor(turns);
            float pick = woundHash(vec3(slot + 17.0, woundInfo[i].y * 97.0, 5.0));
            float inner = 0.7 + 0.15 * pick;
            float outer = 1.1 + 0.2 * fract(pick * 7.13);
            float creaseOn = step(0.3, pick) * smoothstep(inner, inner + 0.1, r)
                * (1.0 - smoothstep(outer - 0.12, outer, r));
            float creaseHalf = (0.05 + 0.04 * severity) * radius * sqrt(creaseOn);
            if (creaseHalf > 0.0)
            {
                float arc = abs(fract(turns) - 0.5) * (6.28318531 / 7.0) * length(vec2(cx, cy));
                line = vec2(arc - creaseHalf, creaseHalf);
                // The fold's relief fades where a pixel covers it.
                float fine = clamp(creaseHalf / max(1.5 * woundFootprint, 0.00001), 0.0, 1.0);
                fold = (0.05 + 0.04 * severity) * radius * fine * creaseOn
                    * (1.0 - smoothstep(0.0, 1.8 * creaseHalf, arc));
            }
        }
        return -sink * bowl + 0.1 * sink * core - fold;
    }
    if (stroke == 1 || stroke == 3)
    {
        // A dent: a dish pressed in round the struck point, a slight rim
        // round it; a point's is smaller and deeper, with a bright pit.
        // Leather gives less (a shallow press, no rim); a point punches a
        // hole with a frayed edge.
        float radius = stroke == 3 ? max(2.2 * halfGape, 0.45) : max(1.15 * halfGape, 0.3);
        float r = through / radius;
        if (r > 1.4)
            return 0.0;
        reach = 1.0;
        across = r;
        float dish = max(1.0 - r * r, 0.0);
        // Occlusion-like: darkest in the bottom, nothing past the edge.
        halo = dish * dish * (3.0 - 2.0 * dish);
        float ragged = soft ? (woundNoise(q * 9.0) - 0.5) * 0.3 + (woundNoise(q * 21.0) - 0.5) * 0.15
                            : (woundNoise(q * 4.0) - 0.5) * 0.15;
        core = stroke == 3 ? 1.0 - smoothstep(0.1, 0.28, r + ragged) : 0.0;
        float rim = smoothstep(0.85, 1.0, r) * (1.0 - smoothstep(1.0, 1.35, r));
        float sink = soft ? (stroke == 3 ? 0.2 : 0.05 + 0.05 * severity) * radius
                          : (stroke == 3 ? 0.28 : 0.14 + 0.16 * severity) * radius;
        return -sink * dish * dish + (soft ? 0.0 : 0.12 * sink * rim) - (soft ? 0.25 * radius * core : 0.0);
    }
    // An edge's scratch: a slab like a cut's, so it follows a curved plate.
    float dentHalf = (stroke == 4 ? 2.4 : 1.5) * halfGape;
    if (abs(x) > 2.8 * dentHalf + 0.3 || abs(beyond) > 0.3 * halfGape)
        return 0.0;
    reach = 1.0 - smoothstep(0.7 * woundAxis[i].w, woundAxis[i].w, abs(depth));
    if (reach <= 0.0)
        return 0.0;
    // Biting in where the stroke starts, thinning out where it leaves; a
    // chop gouges wider. The channel is a few millimetres wide (it reads at
    // a few metres). The edge chatters: an uneven line; leather frays.
    float shape = max(sin(3.14159265 * pow(c, 0.6)), 0.0);
    float lineHalf = max(((stroke == 4 ? 0.36 : 0.26) * halfGape + 0.035 + 0.03 * woundBold) * sqrt(shape), 0.004);
    float xs = x - (woundNoise(q * 0.5) - 0.5) * 0.25 * halfGape;
    float chatter = soft ? (woundNoise(q * 14.0) - 0.5) * 0.9 + (woundNoise(q * 33.0) - 0.5) * 0.45
                         : (woundNoise(q * 9.0) - 0.5) * 0.5;
    line = vec2(abs(xs) - lineHalf * (1.0 + chatter), lineHalf);
    // The metal pushed in beside the line, on one side (by the seed).
    float side = woundInfo[i].y < 0.5 ? 1.0 : -1.0;
    float dx = (xs - side * 0.45 * dentHalf) / dentHalf;
    float dent = exp(-2.5 * dx * dx) * shape;
    across = side * xs / lineHalf;
    core = shape;
    halo = dent * reach * (soft ? 0.35 : 1.0);
    float ax = abs(xs);
    // The channel's fine relief (groove and burr) fades where a pixel covers
    // it; the dent beside it is broad and stays.
    float fine = clamp(lineHalf / max(1.5 * woundFootprint, 0.00001), 0.0, 1.0);
    float groove = (soft ? 0.06 + 0.1 * severity : 0.04 + 0.06 * severity) * shape
        * (1.0 - smoothstep(0.0, 1.6 * lineHalf, ax));
    float burr = (soft ? 0.006 + 0.01 * severity : 0.015 + 0.025 * severity) * shape
        * smoothstep(lineHalf, 1.8 * lineHalf, ax) * (1.0 - smoothstep(1.8 * lineHalf, 3.0 * lineHalf, ax));
    float press = (stroke == 4 ? 0.2 + 0.25 * severity : 0.06 + 0.1 * severity) * halfGape * dent
        * (soft ? 0.3 : 1.0);
    return (fine * (burr - groove) - press) * reach;
}

// The size a wound's relief is drawn at (its fade with distance, see main):
// the gape, or for armour the dent's.
float woundReliefScale(int i, int kind, float halfGape)
{
    if (kind != 6)
        return halfGape;
    int stroke = woundStroke(i);
    if (stroke == 1)
        return max(1.15 * halfGape, 0.3);
    if (stroke == 3)
        return max(2.2 * halfGape, 0.45);
    return (stroke == 4 ? 2.4 : 1.5) * halfGape;
}

// Skin height round wound i at bind point p (bind units: the lips raised, the
// gape sunken, a bruise swollen). across is 0 on the cut line and 1 at the
// lips (a bruise: 0 at its middle, about 1 at its rim); reach fades the slab
// out with depth, so the far side of the limb stays clean; core is the
// profile along the stroke (1 where it is deepest); halo is how much the
// skin is bruised (a bruise's blotch, the margins of a chop). The noise runs
// in the wound's own frame, so a wound spilling over two parts' meshes (each
// with its own bind space) matches at the seam.
float woundHeight(int i, vec3 p, out float across, out float reach, out float core, out float halo, out vec2 line)
{
    across = 100.0;
    reach = 0.0;
    core = 0.0;
    halo = 0.0;
    line = vec2(0.0);
    int kind = woundKind(i);
    vec3 a = woundStart[i].xyz;
    vec3 ab = woundEnd[i].xyz - a;
    float len = max(length(ab), 0.001);
    vec3 t = ab / len;
    vec3 n = woundAxis[i].xyz;
    vec3 s = normalize(cross(n, t));
    vec3 d = p - a;
    float along = dot(d, t) / len;
    float x = dot(d, s);
    float halfGape = woundStart[i].w;
    float severity = woundEnd[i].w;
    float depth = dot(d, n);
    // How far past the segment's ends, along it.
    float beyond = (along - clamp(along, 0.0, 1.0)) * len;
#if @woundVolumes >= 2
    woundStainThrough = -1.0;
    if (kind == 5 && woundBlood == 0 && dot(woundRun[i].xyz, woundRun[i].xyz) > 0.25)
    {
        // FX4: blood soaking a sleeve or a shirt creeps downhill a little over
        // its first seconds, more where the cloth hangs steeply: the point is
        // measured as if pulled up toward the cut by up to that sag.
        float sagAge = max(osg_SimulationTime - woundInfo[i].x, 0.0);
        float sag = 0.6 * woundAxis[i].w * woundRun[i].w * (1.0 - exp(-sagAge / 6.0));
        vec3 sagged = d - woundRun[i].xyz * clamp(dot(d - 0.5 * ab, woundRun[i].xyz), 0.0, sag);
        float sagAlong = dot(sagged, t) / len;
        float sagBeyond = (sagAlong - clamp(sagAlong, 0.0, 1.0)) * len;
        woundStainThrough = length(vec3(dot(sagged, s), sagBeyond, dot(sagged, n)));
    }
#endif
    if (kind >= 5)
        return woundSurfaceHeight(
            i, kind, along, len, x, depth, beyond, halfGape, severity, across, reach, core, halo, line);
    // A bruise and the bruised margin of a chop reach round the line.
    float outer = kind == 1 ? 1.5 * halfGape : (kind == 4 ? 3.6 : 2.5) * halfGape + 0.3;
    if (abs(x) > outer || abs(beyond) > (kind == 2 || kind == 3 ? 0.02 * len : outer))
        return 0.0;
    vec3 woundLocal = vec3(along * len, x, depth);
    vec3 q = vec3(0.8 * woundLocal.x + 0.6 * woundLocal.y, 0.8 * woundLocal.y - 0.6 * woundLocal.x, woundLocal.z) * 1.7
        + woundInfo[i].y * vec3(61.0, 83.0, 47.0);
    // Distance through the limb from the stroke: a blotch measured this way
    // stays about its radius round a curved limb (a flat distance across the
    // skin would wrap a thin forearm's whole width).
    float through = length(vec3(x, beyond, depth));
    if (kind == 1)
    {
        // Bruise: blood under unbroken skin, a soft mottled blotch round the
        // struck point fading to nothing, slightly swollen (WOUNDFX-01
        // section 1). No depth slab: the far side is farther than the radius.
        float r = through / max(halfGape, 0.05);
        if (r > 1.5)
            return 0.0;
        reach = 1.0;
        float mottle = (woundNoise(q * 0.8) - 0.5) * 0.45 + (woundNoise(q * 2.3) - 0.5) * 0.2;
        across = r + 0.5 * mottle;
        core = 1.0 - smoothstep(0.0, 0.7, r);
        halo = (1.0 - smoothstep(0.3, 1.0, across)) * (0.7 + 0.3 * woundNoise(q * 1.4 + 7.0));
        float swell = max(1.0 - r * r, 0.0);
        return (0.03 + 0.06 * severity) * halfGape * swell * swell;
    }
    reach = 1.0 - smoothstep(0.7 * woundAxis[i].w, woundAxis[i].w, abs(depth));
    if (reach <= 0.0)
        return 0.0;
    float shape, gape, meander, wobble, sink, lip, lipFall;
    if (kind == 3)
    {
        // Stab: a slit about the blade's width along its edge axis, pointed
        // at both ends, gaping open, deeper than long.
        float c = 2.0 * clamp(along, 0.0, 1.0) - 1.0;
        shape = 1.0 - c * c;
        gape = max(halfGape * shape, 0.015);
        meander = (woundNoise(q * 0.45) - 0.5) * 0.25 * halfGape;
        wobble = (woundNoise(q * 3.1) - 0.5) * 0.12;
        sink = (0.3 + 0.5 * severity) * sqrt(shape);
        lip = (0.025 + 0.03 * severity) * sqrt(shape);
        lipFall = 1.6;
    }
    else if (kind == 4)
    {
        // Chop: a heavy edge driven in - deep along most of its length, a
        // wide gape, torn and ragged lips.
        shape = sqrt(max(sin(3.14159265 * pow(clamp(along, 0.0, 1.0), 0.85)), 0.0));
        gape = max(halfGape * pow(shape, 0.4), 0.02);
        meander = (woundNoise(q * 0.45) - 0.5) * 0.8 * halfGape;
        wobble = (woundNoise(q * 1.9) - 0.5) * 0.42 + (woundNoise(q * 5.3) - 0.5) * 0.24
            + (woundNoise(q * 12.7) - 0.5) * 0.12;
        sink = (0.3 + 0.55 * severity) * shape;
        lip = (0.06 + 0.06 * severity) * sqrt(shape);
        lipFall = 2.6;
        // Bruised, scraped margins all round it.
        float ring = through / (3.2 * halfGape);
        halo = (1.0 - smoothstep(0.3, 1.0, ring + (woundNoise(q * 0.9) - 0.5) * 0.3)) * reach;
    }
    else
    {
        // Slash: spindle gape, widest a third of the way in: deep where the
        // stroke starts, a long shallow tail where it leaves (WOUNDFX-01
        // section 1). A clean incised edge, slightly uneven: a gentle meander
        // of the line and a fine wobble of each lip.
        shape = max(sin(3.14159265 * pow(clamp(along, 0.0, 1.0), 0.7)), 0.0);
        gape = max(halfGape * pow(shape, 0.6), 0.015);
        meander = (woundNoise(q * 0.45) - 0.5) * 0.6 * halfGape;
        wobble = (woundNoise(q * 2.6) - 0.5) * 0.2 + (woundNoise(q * 7.3) - 0.5) * 0.09;
        sink = (0.12 + 0.45 * severity) * shape;
        lip = (0.035 + 0.045 * severity) * sqrt(shape);
        lipFall = 1.8;
    }
    across = abs(x - meander) / gape + wobble;
    core = shape;
    float height = across < 1.0
        ? lip * smoothstep(0.55, 1.0, across) - sink * (1.0 - smoothstep(0.25, 1.0, across))
        : lip * (1.0 - smoothstep(1.0, lipFall, across));
    return height * reach;
}

#if @woundVolumes >= 2
// FX4: a bind point in wound i's own frame (along the stroke, across it, out
// of the skin), scaled for the noise: the same on both sides of a seam.
vec3 woundFramePos(int i, vec3 p)
{
    vec3 a = woundStart[i].xyz;
    vec3 ab = woundEnd[i].xyz - a;
    vec3 t = ab / max(length(ab), 0.001);
    vec3 n = woundAxis[i].xyz;
    vec3 s = normalize(cross(n, t));
    vec3 d = p - a;
    return vec3(dot(d, t), dot(d, s), dot(d, n)) * 1.7 + woundInfo[i].y * vec3(61.0, 83.0, 47.0);
}

// FX4: blood running from fresh open wound i (2 slash, 3 stab, 4 chop) at bind
// point p: up to three thin rivulets spill over the lower lip and run downhill
// along the skin (woundRun), lengthening over the first seconds after the hit
// and then stopping; a bigger, deeper wound bleeds longer runs, a steeper slope
// carries them further. Each is a wandering thread with a noisy edge (noise in
// the wound's frame) and a drop gathered at its end. Returns the blood's
// cover (0..1); thick: 1 in a rivulet's middle and in its drop.
float woundRunCover(int i, int kind, vec3 p, float footprint, float age, out float thick)
{
    thick = 0.0;
    vec3 r = woundRun[i].xyz;
    if (dot(r, r) < 0.25)
        return 0.0;
    float slope = clamp(woundRun[i].w, 0.0, 1.0);
    vec3 a = woundStart[i].xyz;
    vec3 ab = woundEnd[i].xyz - a;
    float len = max(length(ab), 0.001);
    vec3 t = ab / len;
    vec3 n = woundAxis[i].xyz;
    vec3 side = cross(n, r);
    if (dot(side, side) < 1e-6)
        return 0.0;
    side = normalize(side);
    float halfGape = woundStart[i].w;
    float severity = woundEnd[i].w;
    float seed = woundInfo[i].y;
    float size = clamp(halfGape / 0.5, 0.6, 1.8) * (kind == 4 ? 1.35 : (kind == 3 ? 1.15 : 1.0));
    float reach = (2.5 + 7.5 * severity) * size * (0.2 + 0.8 * slope);
    float grow = 1.0 - exp(-age / 2.2);
    float longest = reach * grow;
    if (longest < 0.05)
        return 0.0;
    // Quick bound round all the runs: the cut's span plus the longest run.
    vec3 dm = p - (a + 0.5 * ab);
    float spanDown = 0.5 * len * abs(dot(t, r)) + 2.0 * halfGape + 0.5;
    float spanSide = 0.5 * len * abs(dot(t, side)) + 2.0 * halfGape + 1.0 + 0.2 * longest;
    float down = dot(dm, r);
    if (down < -spanDown || down > longest + spanDown || abs(dot(dm, side)) > spanSide)
        return 0.0;
    float cover = 0.0;
    float pixel = max(footprint, 0.0001);
    for (int k = 0; k < 3; ++k)
    {
        float fk = float(k);
        float h1 = woundHash(vec3(seed * 113.0, fk * 7.0 + 1.0, 3.0));
        float h2 = woundHash(vec3(seed * 71.0 + 5.0, fk * 11.0, 9.0));
        float h3 = woundHash(vec3(fk * 5.0 + 2.0, seed * 37.0, 17.0));
        // A stab bleeds from its middle, a cut from anywhere along it; the
        // blood leaves over the lip on the downhill side.
        float at = kind == 3 ? 0.5 + (h1 - 0.5) * 0.4 : mix(0.12, 0.8, h1);
        vec3 source = a + ab * at + r * (0.6 * halfGape);
        float runLength = k == 0 ? longest : longest * mix(0.35, 0.9, h2);
        vec3 d = p - source;
        float u = dot(d, r);
        float depth = dot(d, n);
        if (u < -2.0 * halfGape || u > runLength + 1.0)
            continue;
        // The skin curves away round a limb: the run follows it down the
        // side, never through to the far side.
        float down0 = max(u, 0.0);
        if (-depth > 1.1 * down0 + 0.6 || depth > 0.6 + 0.5 * down0)
            continue;
        float v = dot(d, side);
        vec3 q = vec3(u, v, depth) * 1.7 + seed * vec3(61.0, 83.0, 47.0) + fk * vec3(13.0, 29.0, 7.0);
        // It wanders a little sideways, more the further it has run.
        float meander = (woundNoise(vec3(u * 0.45, fk * 3.7, seed * 41.0)) - 0.5) * (0.4 + 0.12 * down0);
        float uc = clamp(u, 0.0, runLength);
        float width = mix(0.09, 0.2, h3) * (0.7 + 0.6 * severity) * (1.0 - 0.4 * uc / max(runLength, 0.01));
        // Wide where it leaves the wound, a drop gathered at its end.
        width += 0.5 * halfGape * (1.0 - smoothstep(0.0, 0.8 + halfGape, u));
        float dropAt = (uc - runLength + 1.2 * width) / (1.5 * width);
        float drop = runLength > 0.3 ? exp(-dropAt * dropAt) : 0.0;
        width *= (1.0 + 0.7 * drop) * (1.0 + (woundNoise(q * 3.0) - 0.5) * 0.5 + (woundNoise(q * 8.0) - 0.5) * 0.25);
        // A capsule round its line: rounded where it starts and ends.
        float dist = length(vec2(u - uc, v - meander)) - width;
        // (Thinner than a pixel far off: it fades instead of shimmering.)
        float c = clamp(0.5 - dist / pixel, 0.0, 1.0) * min(2.0 * width / pixel, 1.0);
        if (c > cover)
        {
            cover = c;
            thick = max(drop, 1.0 - clamp(abs(v - meander) / max(width, 0.0001), 0.0, 1.0));
        }
    }
    return cover;
}
#endif
#endif

void main()
{
#if @particleOcclusion
    applyOcclusionDiscard(orthoDepthMapCoord, texture2D(orthoDepthMap, orthoDepthMapCoord.xy * 0.5 + 0.5).r);
#endif

    Material material = getMaterial();

    // only offset diffuse and normal maps for now, other textures are more likely to be using a completely different UV set
    vec2 offset = vec2(0.0);

#if @parallax || @diffuseParallax
#if @parallax
    float height = texture2D(normalMap, normalMapUV).a;
#else
    float height = texture2D(diffuseMap, diffuseMapUV).a;
#endif
    offset = getParallaxOffset(transpose(normalToViewMatrix) * normalize(-passViewPos), height);
#endif

vec2 screenCoords = gl_FragCoord.xy / screenRes;

    if (nearDissolve.y > 0.0)
    {
        float centre = 1.0 - clamp(length(screenCoords - 0.5) * 2.0, 0.0, 1.0);
        float solid = nearDissolve.y * (1.0 + nearDissolve.z * centre);
        float keep = smoothstep(nearDissolve.x, solid, length(passViewPos));
        // 4x4 ordered dither: a screen-door fade with no sorting.
        vec2 cell = floor(gl_FragCoord.xy);
        vec2 half_ = floor(cell * 0.5);
        float b2 = fract(dot(cell, vec2(0.5, cell.y * 0.75)));
        float b4 = fract(dot(half_, vec2(0.5, half_.y * 0.75))) * 0.25 + b2;
        if (b4 + 0.5 / 16.0 > keep)
            discard;
    }

#if @diffuseMap
    gl_FragData[0] = texture2D(diffuseMap, diffuseMapUV + offset);

#if defined(DISTORTION) && DISTORTION
    gl_FragData[0].a *= getDiffuseColor(material, passColor).a;
    gl_FragData[0] = applyDistortion(gl_FragData[0], distortionStrength, gl_FragCoord.z, sampleOpaqueDepthTex(screenCoords / @distorionRTRatio).x);
    return;
#endif

#if @diffuseParallax
    gl_FragData[0].a = 1.0;
#else
    gl_FragData[0].a *= coveragePreservingAlphaScale(diffuseMap, diffuseMapUV + offset);
#endif
#else
    gl_FragData[0] = vec4(1.0);
#endif

    vec4 diffuseColor = getDiffuseColor(material, passColor);
    gl_FragData[0].a *= diffuseColor.a * alpha * actorFade;

#if @darkMap
    gl_FragData[0] *= texture2D(darkMap, darkMapUV);
    gl_FragData[0].a *= coveragePreservingAlphaScale(darkMap, darkMapUV);
#endif

    gl_FragData[0].a = alphaTest(gl_FragData[0].a, alphaRef);

#if @normalMap
    vec4 normalTex = texture2D(normalMap, normalMapUV + offset);
    vec3 normal = normalTex.xyz * 2.0 - 1.0;
#if @reconstructNormalZ
    normal.z = sqrt(1.0 - dot(normal.xy, normal.xy));
#endif
    vec3 viewNormal = normalToView(normal);
#else
    vec3 viewNormal = normalize(gl_NormalMatrix * passNormal);
#endif

    vec3 viewVec = normalize(passViewPos);

#if @detailMap
    gl_FragData[0].xyz *= texture2D(detailMap, detailMapUV).xyz * 2.0;
#endif

#if @decalMap
    vec4 decalTex = texture2D(decalMap, decalMapUV);
    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, decalTex.xyz, decalTex.a * diffuseColor.a);
#endif

#if @woundVolumes
    // WOUNDFX-01 FX1/FX2: colour bands inward over the skin colour, the
    // normal tilted by the relief, wet fresh blood; bruises tint the skin.
    // Derivatives first, outside any branch.
    float woundWet = 0.0;
    float woundShine = 0.0; // FX3: bare metal in a scratch
    float woundMatte = 0.0; // FX3: a cut or scuff in leather: no highlight
    {
        vec3 bindDx = dFdx(woundBindPos);
        vec3 bindDy = dFdy(woundBindPos);
        vec3 viewDx = dFdx(passViewPos);
        vec3 viewDy = dFdy(passViewPos);
        float footprint = max(length(bindDx), length(bindDy));
        woundFootprint = footprint;
        vec2 slope = vec2(0.0);
        vec3 albedo = gl_FragData[0].xyz;
        for (int i = 0; i < WOUND_MAX; ++i)
        {
            if (i >= woundCount)
                break;
            int kind = woundKind(i);
#if @woundVolumes >= 2
            if (kind == 7)
            {
                // FX4: blood on a blade where it went in - wiped along it in
                // streaks parallel to the blade, the ends of the range ragged
                // and uneven across its width; wet red drying dark over
                // minutes (a skeleton's: pale bone dust).
                vec3 bladeStart = woundStart[i].xyz;
                vec3 bladeVector = woundEnd[i].xyz - bladeStart;
                float bladeLength = max(length(bladeVector), 0.001);
                vec3 bladeAxis = bladeVector / bladeLength;
                vec3 fromGrip = woundBindPos - bladeStart;
                float onBlade = dot(fromGrip, bladeAxis);
                float bladeAlong = onBlade / bladeLength;
                vec3 acrossBlade = fromGrip - bladeAxis * onBlade;
                float from = woundStart[i].w;
                float to = woundAxis[i].w;
                float range = to - from + 0.1;
                vec3 qs = acrossBlade * 2.2 + woundInfo[i].y * vec3(61.0, 83.0, 47.0);
                float streak = woundNoise(qs) * 0.65 + woundNoise(qs * 2.7 + 5.0) * 0.35;
                float lo = from - 0.12 * range * woundNoise(qs * 1.3 + 11.0);
                float hi = to + 0.15 * range * (woundNoise(qs * 1.1 + 23.0) - 0.3);
                float edge = max(0.01, footprint / bladeLength);
                float body = smoothstep(lo - edge, lo + edge, bladeAlong) * (1.0 - smoothstep(hi - edge, hi + edge, bladeAlong));
                float bladeCover = clamp(body * woundEnd[i].w * (0.45 + 0.75 * streak) * 1.25, 0.0, 1.0);
                if (bladeCover <= 0.0)
                    continue;
                float bladeAge = max(osg_SimulationTime - woundInfo[i].x, 0.0);
                if (woundInfo[i].w > 0.5)
                {
                    albedo = mix(albedo, vec3(0.74, 0.71, 0.64), 0.6 * bladeCover);
                    woundMatte = max(woundMatte, 0.5 * bladeCover);
                    woundCover = max(woundCover, 0.5 * bladeCover);
                    continue;
                }
                float bladeDry = smoothstep(40.0, 400.0, bladeAge);
                albedo = mix(albedo, mix(vec3(0.34, 0.02, 0.025), vec3(0.15, 0.06, 0.045), bladeDry), bladeCover);
                woundWet = max(woundWet, bladeCover * (1.0 - bladeDry) * 0.85);
                woundMatte = max(woundMatte, bladeCover * bladeDry * 0.5);
                woundCover = max(woundCover, bladeCover);
                continue;
            }
            if (kind >= 2 && kind <= 4 && woundBlood == 0)
            {
                // FX4: the fresh cut's blood running downhill, under the cut
                // itself (drawn below over it); wet and glossy, drying to dark
                // brown like a stain.
                float runAge = max(osg_SimulationTime - woundInfo[i].x, 0.0);
                float runThick;
                float run = woundRunCover(i, kind, woundBindPos, footprint, runAge, runThick);
                if (run > 0.0)
                {
                    float runDry = smoothstep(25.0, 240.0, runAge);
                    vec3 blood = mix(vec3(0.40, 0.025, 0.03), vec3(0.24, 0.012, 0.016), 0.6 * runThick);
                    blood = mix(blood, vec3(0.17, 0.075, 0.05), runDry);
                    albedo = mix(albedo, blood, run * (0.82 + 0.15 * runThick));
                    woundWet = max(woundWet, run * (1.0 - runDry) * (0.5 + 0.5 * runThick));
                    woundCover = max(woundCover, run);
                }
            }
#endif
            float across, reach, core, halo;
            float acrossX, reachX, coreX, haloX, acrossY, reachY, coreY, haloY;
            vec2 line, lineX, lineY;
            float height = woundHeight(i, woundBindPos, across, reach, core, halo, line);
            if (reach <= 0.0 || (kind != 6 && across > (kind == 4 ? 2.8 : 2.0) && halo <= 0.0))
                continue;
            float halfGape = woundStart[i].w;
            // Relief fades where one pixel covers the gape (an armour dent:
            // the dent): no shimmer far off.
            float relief = clamp(woundReliefScale(i, kind, halfGape) / max(3.0 * footprint, 0.00001), 0.0, 1.0);
            slope += relief * vec2(woundHeight(i, woundBindPos + bindDx, acrossX, reachX, coreX, haloX, lineX) - height,
                woundHeight(i, woundBindPos + bindDy, acrossY, reachY, coreY, haloY, lineY) - height);
            float severity = woundEnd[i].w;
            float age = max(osg_SimulationTime - woundInfo[i].x, 0.0);
            if (kind == 5)
            {
#if @woundVolumes >= 2
                if (woundBlood != 0)
                {
                    // FX4: no blood under the cloth (a skeleton, a Dwemer
                    // machine): only the slit, round it a skeleton's dust.
                    if (woundBlood == 1)
                        albedo = mix(albedo, vec3(0.80, 0.77, 0.69), 0.2 * halo * core);
                    if (line.y > 0.0)
                        albedo = mix(albedo, vec3(0.06, 0.05, 0.045), 0.9 * woundLine(line, footprint));
                    continue;
                }
#endif
                // Stain (FX3): a multiply over the cloth colour, dark wet red
                // and glossy when fresh, drying darker brown and matte over
                // minutes with a darker tide line at its edge; thickest along
                // the cut. The slit shows dark through the parted cloth.
                float dry = smoothstep(20.0, 240.0, age);
                vec3 tint = mix(vec3(0.42, 0.05, 0.05), vec3(0.20, 0.09, 0.06), dry);
                float tide = smoothstep(0.62, 0.9, across) * (1.0 - smoothstep(0.9, 1.0, across)) * dry;
                float density = min(halo * (0.6 + 0.3 * core + 0.1 * severity + 0.2 * tide), 1.0);
                albedo *= mix(vec3(1.0), tint, density);
                if (line.y > 0.0)
                    albedo = mix(albedo, mix(vec3(0.06, 0.012, 0.012), vec3(0.04, 0.02, 0.015), dry),
                        0.9 * woundLine(line, footprint));
                woundWet = max(woundWet, halo * (1.0 - dry) * (0.35 + 0.35 * core));
                continue;
            }
            if (kind == 6)
            {
                // Scratch or dent (FX3), no blood.
                int stroke = woundStroke(i);
                float pixel = max(footprint, 0.00001);
                float lum = dot(albedo, vec3(0.3, 0.59, 0.11));
                if (woundSoft(i))
                {
                    // Leather: the grain scuffed paler and duller round the
                    // mark, a pressed patch darker; a cut or a punched hole
                    // dark with a frayed, slightly fuzzy edge. Matte: the
                    // highlight goes, nothing shines.
                    vec3 scuffed = mix(albedo, vec3(lum), 0.45) * 1.12 + vec3(0.015);
                    if (stroke == 1 || stroke == 3)
                    {
                        albedo *= 1.0 - (0.22 + 0.18 * severity) * halo;
                        float ring = (1.0 - smoothstep(0.55, 1.05, across)) * (1.0 - halo);
                        albedo = mix(albedo, scuffed, 0.35 * ring);
                        float fibre = smoothstep(0.2, 0.4, core) * (1.0 - smoothstep(0.75, 1.0, core));
                        albedo = mix(albedo, scuffed, 0.3 * fibre);
                        albedo = mix(albedo, albedo * 0.18, 0.9 * smoothstep(0.75, 1.0, core));
                        woundMatte = max(woundMatte, max(halo, core));
                        continue;
                    }
                    float band = (1.0 - smoothstep(1.2, 4.0, abs(across))) * core * reach;
                    albedo = mix(albedo, scuffed, 0.4 * band);
                    albedo *= 1.0 - (0.2 + 0.15 * severity) * halo;
                    float fray = 0.0;
                    float slit = 0.0;
                    if (line.y > 0.0)
                    {
                        fray = max(clamp(0.5 - (line.x - 0.7 * line.y) / pixel, 0.0, 1.0)
                                - clamp(0.5 - line.x / pixel, 0.0, 1.0),
                            0.0);
                        slit = woundLine(line, footprint);
                    }
                    albedo = mix(albedo, scuffed * 1.08, 0.3 * fray);
                    albedo = mix(albedo, albedo * 0.2 + vec3(0.012, 0.008, 0.006), 0.9 * slit);
                    woundMatte = max(woundMatte, max(band, slit));
                    continue;
                }
                // Metal.
                vec3 metal = min(mix(albedo * 1.8 + vec3(0.12), vec3(0.86, 0.86, 0.83), 0.65), vec3(1.0));
                if (stroke == 1)
                {
                    // A blunt dent: its steep walls turn the light (the
                    // tilted normal: a shaded side, a bright crescent toward
                    // the light, tightened by the shine); the dish a little
                    // darker, deep shade in its bottom, a slightly bright
                    // lip, dark creases round its edge.
                    float bowl = 1.0 - smoothstep(0.75, 1.02, across);
                    albedo *= 1.0 - (0.15 + 0.12 * woundBold) * bowl;
                    albedo *= 1.0 - (0.45 + 0.18 * woundBold + (0.25 + 0.05 * woundBold) * severity) * halo;
                    albedo = mix(albedo, metal, 0.3 * core);
                    if (line.y > 0.0)
                        albedo *= 1.0 - 0.6 * woundLine(line, footprint);
                    woundShine = max(woundShine, max(0.55 * bowl * (1.0 - halo), 0.4 * core));
                    continue;
                }
                if (stroke == 1 || stroke == 3)
                {
                    // A dent: the dish shades itself (darkest in the bottom)
                    // and its tilted walls bend the highlight across it; a
                    // point's pit is scraped bright.
                    albedo *= 1.0 - (0.38 + 0.2 * woundBold + (0.25 + 0.05 * woundBold) * severity) * halo;
                    float pit = stroke == 3 ? core : 0.0;
                    albedo = mix(albedo, metal, pit);
                    float dish = 1.0 - smoothstep(0.6, 1.15, across);
                    woundShine = max(woundShine, max(pit, 0.3 * dish));
                    continue;
                }
                // A scored line: a burnished band, a dark groove along the
                // channel's edges (deeper on the dented side), the bright
                // bare channel; the dent beside it shaded. Fine striations
                // along the stroke where they are bigger than a pixel.
                vec3 dir = woundEnd[i].xyz - woundStart[i].xyz;
                float along = dot(woundBindPos - woundStart[i].xyz, dir / max(length(dir), 0.001));
                float stria = (woundNoise(vec3(along * 0.6, across * line.y * 22.0, woundInfo[i].y * 50.0)) - 0.5)
                    * (1.0 - smoothstep(0.012, 0.04, footprint));
                // (Bright steel shows its scratch more by the groove and the
                // highlight than by a paler band: less brightening on it.)
                albedo *= 1.0 - (0.3 + 0.22 * woundBold + (0.2 + 0.1 * woundBold) * severity) * halo;
                float band = (1.0 - smoothstep(1.0, 4.0, abs(across))) * core * reach;
                // (Bold: a dark scuff round the bright line, so it reads from
                // a few paces against any plate.)
                float scuff = smoothstep(1.2, 2.2, abs(across)) * (1.0 - smoothstep(2.6, 5.0, abs(across))) * core * reach;
                albedo *= 1.0 - 0.4 * woundBold * scuff;
                albedo = mix(albedo, min(albedo * 1.45 + vec3(0.05), vec3(1.0)),
                    clamp(0.55 * band * (1.0 + stria) * (1.2 - lum), 0.0, 1.0));
                float bare = 0.0;
                if (line.y > 0.0)
                {
                    float gw = 0.9 * line.y;
                    float groove = max(clamp(0.5 - (line.x - gw) / pixel, 0.0, 1.0)
                            - clamp(0.5 - line.x / pixel, 0.0, 1.0),
                        0.0);
                    albedo *= 1.0 - (0.6 + 0.3 * woundBold) * groove * (across > 0.0 ? 1.0 : 0.5);
                    bare = woundLine(line, footprint);
                }
                albedo = mix(albedo, metal, bare);
                woundShine = max(woundShine, max(bare, 0.45 * band));
                continue;
            }
#if @woundVolumes >= 2
            if (woundBlood == 1)
            {
                // FX4: a skeleton has no blood (white dust). A blow chips and
                // powders the bone: a pale dusty scuff. A cut, stab or chop is
                // a dry crack: a dark shadowed gap with broken, powdery
                // bone-white edges and hairline cracks running out from it;
                // nothing wet.
                vec3 dust = vec3(0.80, 0.77, 0.69);
                if (kind == 1)
                {
                    albedo = mix(albedo, dust, 0.35 * halo * (0.5 + 0.5 * severity));
                    woundMatte = max(woundMatte, 0.5 * halo);
                    continue;
                }
                float crackSoft = clamp(footprint / max(halfGape, 0.015), 0.06, 0.5);
                float crackInside = (1.0 - smoothstep(1.0 - crackSoft, 1.0 + crackSoft, across)) * reach;
                float crackRim = smoothstep(0.55, 1.0, across) * (1.0 - smoothstep(1.0, 1.8, across)) * reach;
                vec3 gap = mix(vec3(0.10, 0.09, 0.08), vec3(0.30, 0.27, 0.22), smoothstep(0.3, 0.9, across));
                albedo = mix(albedo, dust, 0.55 * crackRim);
                if (kind == 4)
                    albedo = mix(albedo, dust, 0.25 * halo);
                albedo = mix(albedo, gap, 0.9 * crackInside);
                vec3 qc = woundFramePos(i, woundBindPos) * 0.9;
                float hairline = 1.0 - smoothstep(0.0, 0.04 + 0.4 * footprint, abs(woundNoise(qc) - 0.5));
                hairline *= step(1.0, across) * (1.0 - smoothstep(1.0, 2.0, across)) * reach * (0.4 + 0.6 * severity);
                albedo *= 1.0 - 0.55 * hairline;
                woundMatte = max(woundMatte, 0.6 * max(crackInside, crackRim));
                continue;
            }
#endif
            if (kind == 1)
            {
                // Bruise colour by age (WOUNDFX-01 section 1): red when fresh,
                // purple within minutes (brown, green and yellow over days are
                // FX5). A multiply over the skin colour, so dark skin, scales
                // and fur show less of it; the middle darkest.
                float purple = smoothstep(15.0, 180.0, age);
                vec3 tint = mix(vec3(0.90, 0.52, 0.50), vec3(0.60, 0.42, 0.64), purple);
                tint *= 1.0 - 0.18 * core * severity;
                albedo *= mix(vec3(1.0), tint, halo * (0.4 + 0.45 * severity));
                continue;
            }
            float soft = clamp(footprint / max(halfGape, 0.015), 0.06, 0.5);
            float inside = (1.0 - smoothstep(1.0 - soft, 1.0 + soft, across)) * reach;
            float fresh = exp(-age / 150.0);
            // Inward: a pale dermis rim, dark red flesh, a darker deep centre.
            vec3 flesh = vec3(0.50, 0.06, 0.05);
            vec3 dermis = mix(vec3(0.86, 0.60, 0.55), albedo, 0.3);
            vec3 cut;
            float bone = 0.0;
            if (kind == 3)
            {
                // A stab is mostly a dark hole: a thin band of flesh and rim.
                vec3 deep = mix(vec3(0.09, 0.008, 0.012), vec3(0.05, 0.004, 0.008), severity);
                cut = mix(deep, flesh, smoothstep(0.45, 0.85, across));
                cut = mix(cut, dermis, smoothstep(0.8, 0.97, across));
            }
            else
            {
                vec3 deep = mix(vec3(0.20, 0.016, 0.02), vec3(0.10, 0.008, 0.012), severity);
                cut = mix(deep, flesh, smoothstep(0.12, 0.62, across));
                if (kind == 4)
                {
                    // A chop shows muted fat only in its deepest middle (not
                    // speckled along the rim), and the most severe a pale
                    // hint of bone there (WOUNDFX-01 section 1). Its torn rim
                    // is bloody, not pale: the ragged edge's islands read as
                    // torn skin, not white flecks.
                    float fat = smoothstep(0.62, 0.95, core) * smoothstep(0.4, 0.9, severity)
                        * smoothstep(0.1, 0.22, across) * (1.0 - smoothstep(0.32, 0.48, across));
                    cut = mix(cut, vec3(0.60, 0.46, 0.26), 0.35 * fat);
                    bone = smoothstep(0.88, 1.0, severity) * smoothstep(0.55, 0.9, core)
                        * (1.0 - smoothstep(0.08, 0.26, across));
                    dermis = mix(vec3(0.66, 0.36, 0.32), albedo, 0.35);
                }
                cut = mix(cut, dermis, smoothstep(0.7, 0.96, across));
            }
            // Fresh blood is bright; it darkens to red-brown as it dries.
            cut = mix(cut * vec3(0.62, 0.48, 0.45), cut, fresh);
            cut = mix(cut, vec3(0.86, 0.80, 0.68), 0.8 * bone);
            // The skin round a fresh cut reddens a little; round a chop it
            // is bruised.
            float margin = smoothstep(0.85, 1.0, across) * (1.0 - smoothstep(1.0, 1.9, across)) * reach;
            albedo *= mix(vec3(1.0), vec3(1.0, 0.78, 0.76), margin * (0.3 + 0.4 * fresh));
            albedo *= mix(vec3(1.0), vec3(0.64, 0.44, 0.52), halo * (0.3 + 0.3 * severity));
            albedo = mix(albedo, cut, 0.94 * inside);
            // (A chop's ragged rim stays out of the wet highlight: no glints
            // on its torn islands.)
            float wetEdge = kind == 4 ? 1.0 - smoothstep(0.45, 0.8, across) : 1.0 - smoothstep(0.6, 0.95, across);
            woundWet = max(woundWet, inside * wetEdge * (0.3 + 0.7 * fresh) * (1.0 - 0.6 * bone));
        }
        gl_FragData[0].xyz = albedo;
        // Bump from the relief by the surface gradient (Mikkelsen 2010): no
        // tangents or UVs needed.
        vec3 r1 = cross(viewDy, viewNormal);
        vec3 r2 = cross(viewNormal, viewDx);
        float det = dot(viewDx, r1);
        if (abs(det) > 1e-12)
            viewNormal = normalize(abs(det) * viewNormal - sign(det) * (slope.x * r1 + slope.y * r2));
    }
#endif

#if @envMap

    vec2 envTexCoordGen = envMapUV;
    float envLuma = 1.0;

#if @normalMap
    // if using normal map + env map, take advantage of per-pixel normals for envTexCoordGen
    vec3 r = reflect( viewVec, viewNormal );
    float m = 2.0 * sqrt( r.x*r.x + r.y*r.y + (r.z+1.0)*(r.z+1.0) );
    envTexCoordGen = vec2(r.x/m + 0.5, r.y/m + 0.5);
#endif

#if @bumpMap
    vec4 bumpTex = texture2D(bumpMap, bumpMapUV);
    envTexCoordGen += bumpTex.rg * bumpMapMatrix;
    envLuma = clamp(bumpTex.b * envMapLumaBias.x + envMapLumaBias.y, 0.0, 1.0);
#endif

    vec3 envEffect = texture2D(envMap, envTexCoordGen).xyz * envMapColor.xyz * envLuma;

#if @glossMap
    envEffect *= texture2D(glossMap, glossMapUV).xyz;
#endif
#if @woundVolumes >= 2
    // FX4: blood on a blade dulls its reflection.
    envEffect *= 1.0 - 0.85 * woundCover;
#endif

#if @preLightEnv
    gl_FragData[0].xyz += envEffect;
#endif

#endif

    float shadowing = unshadowedLightRatio(-passViewPos.z);
    vec3 lighting, specular;
#if !PER_PIXEL_LIGHTING
    lighting = mix(shadedLighting, passLighting, shadowing);
    specular = mix(shadedSpecular, passSpecular, shadowing);
#else
#if @specularMap
    vec4 specTex = texture2D(specularMap, specularMapUV);
    float shininess = specTex.a * 255.0;
    vec3 specularColor = specTex.xyz;
#else
    float shininess = material.shininess;
    vec3 specularColor = getSpecularColor(material, passColor).xyz;
#endif
#if @woundVolumes
    shininess = mix(shininess, 48.0, woundWet); // wet blood: a tight highlight
    shininess = mix(shininess, 96.0, woundShine); // bare metal: tighter
    shininess = mix(shininess, 4.0, woundMatte); // scuffed leather: broad and dull
#endif
    vec3 diffuseLight, ambientLight, specularLight;

    doLighting(gl_FragCoord.xy, passViewPos, viewNormal, shininess, shadowing, diffuseLight, ambientLight, specularLight);

    lighting = diffuseColor.xyz * diffuseLight + getAmbientColor(material, passColor).xyz * ambientLight + getEmissionColor(material, passColor).xyz * material.emissiveMult;
    specular = specularColor * specularLight * material.specStrength;
#if @woundVolumes
    specular = mix(specular, vec3(0.55, 0.42, 0.40) * specularLight, woundWet);
    specular = mix(specular, vec3(0.85, 0.85, 0.82) * specularLight, woundShine);
    specular *= 1.0 - 0.85 * woundMatte;
#endif
    clampLighting(lighting);
#endif

    gl_FragData[0].xyz = gl_FragData[0].xyz * lighting + specular;

#if @envMap && !@preLightEnv
    gl_FragData[0].xyz += envEffect;
#endif

#if @emissiveMap
    gl_FragData[0].xyz += texture2D(emissiveMap, emissiveMapUV).xyz;
#endif

    // Keep the underlying skin/clothing visible even at maximum regional
    // damage. A muted stain reads as injury instead of solid red paint.
    float injuryBlend = 0.35 * clamp(anatomicalDamageTint, 0.0, 1.0);
    gl_FragData[0].rgb = mix(gl_FragData[0].rgb, vec3(0.65, 0.08, 0.06), injuryBlend);
    gl_FragData[0] = applyFogAtPos(gl_FragData[0], passViewPos, near, far);

#if !defined(FORCE_OPAQUE) && @softParticles
    gl_FragData[0].a *= calcSoftParticleFade(
        viewVec,
        passViewPos,
        viewNormal,
        near,
        far,
        sampleOpaqueDepthTex(screenCoords).x,
        particleSize,
        particleFade,
        softFalloffDepth,
        waterEnabled,
        isReflection,
        waterHeight,
        osg_ViewMatrixInverse
    );
#endif

#if defined(FORCE_OPAQUE) && FORCE_OPAQUE
    // having testing & blending isn't enough - we need to write an opaque pixel to be opaque
    gl_FragData[0].a = 1.0;
#endif

#if !defined(FORCE_OPAQUE) && !@disableNormals
    gl_FragData[1].xyz = viewNormal * 0.5 + 0.5;
#endif

    applyShadowDebugOverlay();
}

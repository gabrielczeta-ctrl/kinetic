#ifndef OPENMW_COMPONENTS_RAGDOLLCORE_REFERENCEGAIT_H
#define OPENMW_COMPONENTS_RAGDOLLCORE_REFERENCEGAIT_H

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/Math/Quat.h>
#include <Jolt/Math/Vec3.h>
// clang-format on

#include <ragdollcore/controller.hpp>
#include <ragdollcore/gaitcontroller.hpp>
#include <ragdollcore/passiveragdoll.hpp>
#include <ragdollcore/swingtrajectory.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace RagdollCore
{
    // Reference locomotion controller (plan step 2). It reproduces the
    // structure of the audited SIMBICON controller with the existing native
    // joint motors instead of virtual pelvis/foot force pairs:
    //
    //  - every joint tracks a target pose with its native spring, plus an
    //    inverse-static load torque planned on the designated support foot;
    //  - the swing thigh tracks a world-frame orientation from the swing-foot
    //    trajectory, so pelvis motion cannot misdirect the swing leg;
    //  - the pelvis is held at its heading-upright world orientation by the
    //    loaded hip springs themselves: the stance thigh target is its
    //    measured world orientation and the pelvis target is the desired one,
    //    so the native implicit spring pulls the pelvis, not the anchored leg.
    //    Measured dynamic reactions of every other root-attached joint
    //    (spine, swing hip) are cancelled on the loaded hips as feed-forward;
    //  - the swing foot is placed from the linear-inverted-pendulum capture
    //    point with an outward lateral offset and a speed-dependent forward
    //    fraction; stance transitions require confirmed contact and report
    //    missing support or late touchdown without inventing a step.
    //
    // All actuation stays inside the ragdoll: native motor impulses act
    // equally and oppositely on the two connected bodies and are clamped once
    // by the authored per-axis capacities. No external force is applied.

    // Defaults: combat search winner 2026-09-12 (history/weekend-batches-2026-09-12.txt): game body, hold-out 0.7% falls,
    // double-shove box 0.0% (was 5.0%), single-shove box 0.0%, tiled 0.3%.
    struct ReferenceGaitProfile
    {
        // Confirm initial stance load and motion toward that stance before
        // ordinary lift-off. Historical fixtures can pin command-only startup.
        bool mRequireInitialSupportTransfer = true;
        // Opt-in experiment: coordinate planted legs for initial COM transfer.
        bool mCoordinateInitialTransfer = false;
        // Diagnostic extension of planted-foot coordination to later transfers.
        bool mCoordinateWeightTransfers = false;
        // Timing in seconds. Both feet stay planted for the double-support
        // interval while the plan already loads the new stance foot only.
        float mDoubleSupportDuration = 0.020226f;
        float mSwingDuration = 0.25011f;
        // Swing-foot contact counts as a step only after this much swing time.
        float mMinimumSwingDuration = 0.15f;
        float mContactConfirmationDuration = 0.03f;
        // Load-aware transitions. The stance transfers on confirmed swing
        // foot contact carrying at least the transfer fraction of body
        // weight (zero: contact presence, the retained default, because the
        // single-support plan keeps the load on the stance leg until the
        // transfer). The trailing foot lifts when the new stance foot bears
        // the swing-start fraction. After a confirmed touchdown, failure to
        // take load by the double-support timeout permits swapping back to
        // the supported trailing foot. Initial transfer keeps its destination;
        // swing requires stance support, with the deadline reporting a late
        // phase instead of forcing an unsupported swing.
        float mTransferLoadFraction = 0.0f;
        float mSwingStartLoadFraction = 0.0f;
        float mMaximumDoubleSupportDuration = 0.3f;
        // Missing support in transfer, or missing touchdown in swing, reports
        // a timeout after this duration in that stage.
        float mMaximumStepDuration = 0.75f;
        // Swing trajectory shape, world units.
        float mSwingFootLift = 9.3305f;
        // Opt-in continuous horizontal command; vertical lift/press unchanged.
        bool mContinuousSwingTrajectory = false;
        // Vertical target descent rate applied when the swing exceeds its
        // nominal duration without contact, and its floor below the stance
        // foot level.
        float mTouchdownPressSpeed = 24.9714f;
        float mMaximumTouchdownPressDepth = 6.0f;
        // Placement. Lateral: capture point plus an outward offset, never
        // narrower than the minimum foot-center separation. Forward: the
        // steady-state fraction of the capture distance is derived from the
        // stance period; velocity error above the request steps further.
        float mLateralCaptureOffset = 4.3841f;
        // Forward placement gain on the speed error above the request. One
        // reproduces the capture point; larger values brake harder while the
        // linear inverted-pendulum return map stays stable below about 1.7.
        // Matched trials at 1.5 and 1.7 did not walk further than 1.0.
        float mForwardVelocityErrorGain = 0.36989f;
        float mMinimumStepWidth = 6.9374f;
        float mMaximumStepWidth = 40.052f;
        float mMinimumForwardPlacement = 0.0f;
        float mMaximumStepReach = 55.0f;
        // Requested forward COM acceleration = gain * (requested - measured
        // forward speed), realized by the inverse-static plan through the
        // planned center of pressure within the support hull. Placement owns
        // most of the speed regulation; the ankle contribution stays small so
        // the stance ankle does not saturate at the heel/toe edge.
        float mVelocityGain = 0.2474f;
        // Double-support lateral COM spring toward the new stance foot: the
        // plan spans both feet, so this loads the trailing foot (push-off).
        // Zero leaves the lateral motion to placement alone, as in the
        // reference controller; the single-support diagnostic needs it to
        // bring the mass over one foot before holding. In single support the
        // plan keeps the lateral center of pressure at the stance foot center
        // so the inverted pendulum reverses naturally for the next step.
        float mWeightTransferFrequency = 0.0f;
        float mWeightTransferDamping = 1.0f;
        // Double-support lateral velocity regulation. The linear inverted
        // pendulum return map of the capture placement has a stable fixed
        // point whose landing velocity toward the new stance foot is
        // u* = (b / tau) sinh(T / tau) / (1 + exp(-T / tau)), with b the
        // lateral capture offset, T the stance period and tau the pendulum
        // time constant. Landing errors and the pendulum's lower physical
        // efficiency move the state off that cycle; double support is the
        // only phase with lateral center-of-pressure authority across both
        // feet. The historical rule requests gain * (u* - u); the
        // experimental braking rule halves u* after initial transfer.
        // Zero disables the regulation.
        float mTransferVelocityGain = 7.5343f;
        // Experimental reduction of the double-support lateral cycle target.
        // Disabled: ID-variation screening worsened survival (2026-09-09).
        // Coordinated pose profiles retain their authored transfer request.
        // False preserves the historical pendulum-cycle target.
        bool mBrakeLateralTransfer = false;
        float mMaximumRequestedAcceleration = 163.12f;
        // Limit the planning speed's lead over measured speed to one nominal
        // step of bounded acceleration. Disable only for historical replays.
        bool mLimitVelocityLead = true;
        // Preserve the qualified low-speed gait, including its startup and
        // transferred-stance handover. Higher demands get a moving ceiling
        // based on measured speed and one step of bounded acceleration.
        float mUnrestrictedRequestSpeed = 60.0f;
        // Fraction of the total acceleration budget available to forward
        // speed prediction after lateral stabilization. This changes the
        // high-demand planning lead, not force authority or eventual speed.
        // One reproduces the historical optimistic prediction.
        float mVelocityLeadAccelerationFraction = 0.25f;
        // Swing-leg spring during the swing stage (frequency in Hz, damping
        // ratio). The authored 4 Hz / 1.0 leg motors settle in 0.55-0.62 s in
        // the recorded response measurement, longer than one swing; this is
        // a bandwidth choice, not a capacity change: torque limits stay fixed.
        // Zero keeps the authored spring.
        float mSwingLegFrequency = 13.637f;
        float mSwingLegDamping = 2.0f;
        // Loaded-leg spring override (both legs in double support, the stance
        // leg in single support). Zero keeps the authored spring.
        float mStanceLegFrequency = 0.0f;
        float mStanceLegDamping = 1.0f;
        // Loaded-knee target flexion in radians below the base pose, so the
        // stance leg vaults less and the swing leg keeps reach for landing.
        // Raised from 0.3109 to 0.4 on 2026-09-07: the swing foot was reaching
        // at nearly full extension (leg 59.5 against a hip height of 66),
        // which is why large shoves during a swing failed geometrically.
        // Crouching deeper buys reach. Measured on the 60-variant set:
        // 0.3109 -> 50/60, 0.35 -> 52, 0.38 -> 54 but with a fall, 0.40 ->
        // 53 with none, 0.42 and 0.45 -> 50, 0.50 -> 20, 0.60 -> 0. The
        // optimum is narrow and 0.40 is the best point with zero falls
        // (basin-widening-2026-09-07.txt).
        float mStanceKneeFlexion = 0.40414f;
        // Knee target continuity at takeoff (seconds). A leg that has just
        // become the stance leg eases its knee target from the MEASURED knee
        // flexion to mStanceKneeFlexion over this window instead of snapping,
        // which jumped the command when the landing press handed the leg to
        // the stance flexion target at takeoff (0.487 rad in the E10 trace).
        // The measured start is recomputed every step from the current pose,
        // so no command history is stored and the chase stays smooth as the
        // motor tracks. The window is timed from the stage clock, which
        // restarts at the takeover: during the landing press the press owns
        // the knee, so the ease effectively covers the first frames of single
        // support. Zero restores the immediate snap.
        float mStanceKneeEaseDuration = 0.03f;
        // Fraction of the swing duration after which the placement stops
        // being replanned, so the foot decelerates before touchdown. One
        // replans until contact.
        float mPlacementLatchFraction = 1.0f;
        // Trunk (root-attached spine chain) spring override while walking.
        // The reference treats the torso as one rigid body; a stiffer trunk
        // lets the torso yaw inertia resist the swing leg yaw reaction on
        // the light pelvis. Zero keeps the authored spine springs.
        float mTrunkFrequency = 0.0f;
        float mTrunkDamping = 1.0f;
        // Loaded ankle policy. The plan places the center of pressure at the
        // stance foot center, which needs (near) zero stance ankle torque
        // beyond the planned load: the loaded ankle holds its measured joint
        // orientation with this soft spring instead of a world-level target,
        // so it does not tip the foot onto an edge nor pogo the body. Zero
        // keeps the world-level target with the authored spring.
        float mLoadedAnkleFrequency = 4.6694f;
        float mLoadedAnkleDamping = 1.0f;
        // Active ankle push-off (radians of plantar-flexion target offset).
        // The loaded ankle is a soft angle-holder with no push-off, which
        // saturates speed well below the request (E7 showed stiffening alone
        // cannot fix that -- an ACTIVE plantar-flexion term is needed). In
        // the last fraction of single support before lift, the stance ankle
        // target is rotated toward plantar flexion about the leg's lateral
        // hinge axis, ramping in with a smoothstep from
        // mStanceAnklePushOffStartFraction of the swing duration to full at
        // the nominal touchdown. The offset shifts the soft spring's
        // equilibrium; motor authority and joint limits are unchanged, and
        // the final-command feasibility gate still bounds the result.
        // Zero disables the offset.
        // DEFAULT OFF, measured 2026-09-08 on the 7 s / 60 u/s contract over
        // layer 1+2: no configuration improved speed -- 0.08/0.15/0.25 rad
        // at start 0.8, 0.15 at 0.6, 0.20 at 0.7, trailing-only and both
        // scopes all landed at mean 27.9-29.2 (layer 2 alone: 29.08) with
        // final speed 32.6-34.0 (34.23) and higher heading RMS and slide.
        // The 1.74 Hz soft holder converts the offset into a torque too
        // small to propel; what reaches the foot is an orientation
        // disturbance. A real push-off torque must go through the
        // inverse-statics plan instead -- adding it to mLoadTorques directly
        // would violate the zero-net-actuation contract.
        float mStanceAnklePushOff = 0.0f;
        float mStanceAnklePushOffStartFraction = 0.8f;
        // Also push with the trailing (about to lift) ankle through double
        // support -- the terminal-stance push of human gait.
        bool mStanceAnklePushOffTrailing = true;
        // Terminal-stance forward push through the inverse-statics request
        // (layer 4). Trace diagnosis on the layer 1+2 stack (2026-09-08):
        // during single support the requested acceleration vector sits at
        // mMaximumRequestedAcceleration in 100% of frames while the lateral
        // capture term consumes ~74.6 of the 75.56 budget, leaving ~11.7 for
        // forward; the friction clamp never binds and the hull projection
        // removes under 2% of the request, so forward authority is lost to
        // budget allocation inside the magnitude clamp, not to a hard limit.
        // This field adds a forward acceleration bias (u/s^2) late in single
        // support -- the terminal stance of the stance leg -- ramping in with
        // a smoothstep from mTerminalStancePushStartFraction of the swing
        // duration. The bias is added BEFORE the magnitude clamp, so the
        // total request never exceeds mMaximumRequestedAcceleration: it
        // reallocates budget from lateral to forward late in stance instead
        // of raising any limit. Motor authority, joint limits and the
        // feasibility gate are unchanged. Zero disables the bias.
        float mTerminalStancePush = 15.0f;
        float mTerminalStancePushStartFraction = 0.30f;
        // Lateral speed (u/s) at which the terminal-stance push starts fading
        // out, reaching zero at twice this value, so a lateral shove keeps the
        // full request budget for the capture response. Cruise lateral speed
        // peaks near 16 u/s, so 25 leaves normal walking unaffected. The push
        // is also skipped whenever braking is requested (forward shove
        // recovery). Non-positive disables the lateral fade.
        float mTerminalStancePushLateralLimit = 25.0f;
        // Cap on the lateral component of the requested acceleration, as a
        // fraction of mMaximumRequestedAcceleration (layer 5). Trace diagnosis
        // on the layer 1-4 stack: during single support the raw lateral term
        // is the full inverted-pendulum acceleration about the stance foot
        // center, ~|lateral offset| * g / h ~= 11.8 * 15.1 ~= 178 u/s^2 with
        // the measured ~25-unit landing width, so the magnitude clamp scales
        // the whole request by ~0.42 and crushes the forward component to a
        // third of what was asked. Capping the lateral component before the
        // clamp reserves forward budget; it also turns the lateral request
        // from "track the passive pendulum about a wide foot" into a mild
        // sway brake. The capture response to a lateral shove works through
        // placement, not CoP, so a capped request still steps wide. One
        // disables the cap (legacy behavior).
        // DEFAULT OFF, measured 2026-09-09 on the fixed 8-scenario screen over
        // layer 4: 0.9 with terminal_push 12 reached mean 32.95 / final 38.31,
        // fixed push120-lat1000 (slide 1.49) and held 6/8, but fails
        // coordinatedTransfersSatisfyGaitInvariants at any push but 15;
        // 0.9 with push 15 passes the tests and fixes speed75 (final 43.66)
        // but loses push200-lat1500 (falls), worsens push120-lat1000 to a
        // fall (slide 4.26) and adds +0.97 heading RMS. No configuration was
        // a clean win for the lateral goal, so the safest default is off;
        // the machinery stays available via lateral_request_fraction.
        float mMaximumLateralRequestFraction = 0.84004f;
        // The swing foot target is kept within this fraction of the straight
        // leg length from the measured hip, so the foot can reach the floor
        // at the current hip height instead of hovering above a far target
        // until the body has moved under it (a late landing stretches the
        // stance and breaks the placement timing). Zero disables the limit;
        // the matched trials did not show a benefit yet, so it stays off.
        float mSwingReachFraction = 0.0f;
        // Landing and double-support handling.
        // Contract each foot's horizontal contact hull toward its COM after
        // projecting that center onto the measured patch. This reduces planned
        // pressure travel without inventing support outside a toe/heel/edge
        // patch; it does not itself prove ankle torque feasibility. One keeps
        // the full measured hull.
        float mSupportHullScale = 0.7018f;
        // A landed foot that loses contact after its touchdown transfer while
        // the trailing foot still carries load swaps the stance back after
        // this delay in double support instead of the maximum duration.
        float mLostLandingSwapDelay = 0.05f;
        // Through the double support entered by a touchdown, the landed leg
        // keeps a swing-style leg target at its latched landing point pressed
        // this far below the landing height, so the leg loads the foot
        // instead of holding whatever pose it arrived in. Zero restores the
        // measured hold.
        float mLandingPressDepth = 0.8533f;
        // Swing trajectory shape as fractions of the swing duration: the foot
        // rises to its clearance by the rise end, completes its horizontal
        // travel and descent by the travel end, and presses toward the floor
        // from the press start.
        float mSwingRiseEndFraction = 0.4607f;
        float mSwingTravelEndFraction = 0.8252f;
        // The foot descends from its clearance between the rise end and the
        // descent end, after the horizontal travel has finished, so it
        // touches down without horizontal velocity relative to the floor.
        float mSwingDescentEndFraction = 0.9864f;
        float mTouchdownPressStartFraction = 0.9912f;
        // Place the swing foot from the linear-inverted-pendulum prediction of
        // the center of mass at the nominal touchdown, integrating the plan's
        // own single-support center-of-pressure policy over the remaining
        // swing time, instead of from the current state.
        bool mPredictPlacement = true;
        // Feed the swing hip and knee motors the angular velocity of their
        // trajectory targets so the implicit spring damping does not brake
        // the tracked motion.
        bool mSwingVelocityFeedForward = true;
        // Loaded-hip spring override (the stance hip, and both hips in a
        // double support without a landing press), separate from the knee
        // and ankle. Zero keeps the authored spring. Native springs derive
        // their stiffness from the two connected bodies alone, so the
        // pelvis hold needs far more than the authored 4 Hz to matter.
        float mStanceHipFrequency = 9.5133f;
        float mStanceHipDamping = 2.226f;
        // Rate-consistent hold: extrapolate each held thigh target by its
        // measured world rate over one step and feed that rate to the motor
        // damping, so the hold damps the pelvis without braking the stance
        // leg's rotation over the planted foot.
        bool mStanceHipRateHold = true;
        // Fraction of the swing duration before the horizontal travel starts,
        // so a lifting foot is not dragged while it still touches the floor.
        float mSwingTravelStartFraction = 0.033f;
        // Computed-torque feed-forward for the swing leg: the angular
        // acceleration of the trajectory targets times the leg's point-mass
        // inertia about the hip (and the calf and foot about the knee) is
        // added to the load torques, with its reaction routed through the
        // stance hip, so the springs only correct the residual. Native
        // springs alone are far too soft for the whole leg. Zero disables.
        float mSwingInertiaFeedForward = 0.703f;
        // Bound on the trajectory angular acceleration used by that
        // feed-forward (radians per second squared). Finite differences of
        // the leg IK blow up near full extension; the bound keeps a kink
        // from becoming a torque spike. Zero disables the bound.
        float mSwingFeedForwardAccelerationLimit = 56.6687f;
        // Reactive step. In double support, when the linear-inverted-pendulum
        // capture point (center of mass plus velocity times the pendulum time
        // constant) lies farther than this margin outside the segment between
        // the two foot centers, the loaded foot nearer the capture point
        // becomes the stance foot and the other swings at once instead of
        // waiting for the double-support timer. World units; zero disables.
        // Measured 2026-09-06 (push-recovery results): 8 units raises the
        // 60-variant push validation from 48 to 50 with no falls and leaves
        // the nominal walk untouched. Two other candidates for shoves during
        // the swing (a braking placement gain above the request, and an
        // emergency crouch for reach) were measured and rejected there.
        float mEmergencyCaptureMargin = 19.887f;
        // Emergency swings bypass the ordinary transfer duration. A measured
        // contact must bear this fraction of body weight before that shortcut
        // may unload the other foot. Missing impulse data retains contact gating.
        float mEmergencyStanceLoadFraction = 0.5f;
        // Steering filter (ReferenceGaitSteering) between the gameplay
        // request and the plan. The plan aims the pelvis and each landing
        // foot at the requested heading, so a request that jumps 90 degrees
        // would twist the loaded hip in one step; the filter turns the
        // commanded heading toward the request at most this fast (rad/s),
        // changes the commanded speed at most this fast (u/s^2) while
        // walking, and on a zero request decelerates at the stop rate until
        // the command falls below the stop speed and snaps to zero, which
        // hands the body to the standing controller. Zero disables a limit
        // (the command snaps; a zero stop deceleration is the pre-steering
        // immediate stop, which the lifecycle tests pin).
        // Measured 2026-09-06 (steering results): 1.0-1.5 rad/s complete
        // 45-180 degree turns started at any gait phase without a fall (the
        // stance foot pivots 1.6-2.1 units during a turn); 1.0 has the
        // smallest worst case, 2.5 and above fall on 90-180 degree turns.
        // Deceleration 80 u/s^2 brings 60 u/s to the creep speed in 0.7 s;
        // the hand-over to standing then waits for double support with the
        // center of mass below twice the stop speed. A snapped stop hands a
        // moving body to the standing controller and falls.
        // 2026-09-11: 1.0 -> 0.5. Kimi i46 measured the faced 90-degree turn
        // passing at <= 0.5 rad/s (slide 1.53 -> 1.20, heading error halves) and
        // the Opus CMA-ES search independently drove this field to 0.55.
        float mSteeringTurnRate = 0.5f; // combat winner 0.787 tilted the pelvis past the scripted-turn gate (0.83 > 0.85), 0.305 under-turned it (45.7 < 55 deg); 0.5 sits between (2026-09-12)
        // 2026-09-11 strafe search (strafe-search-2026-09-11.txt): the pelvis
        // heading target turns from the requested facing toward the travel
        // direction by at most this many degrees (i55). 0 disables.
        float mSteeringPelvisTwist = 53.434f;
        // A sideways travel starts with the foot on the travel side swinging
        // first (i57), so a strafe does not begin with a crossover.
        bool mInitialStanceByTravel = true;
        float mSteeringSpeedChangeRate = 0.0f;
        float mSteeringStopDeceleration = 98.51f;
        float mSteeringStopSpeed = 3.0f;
        // Fraction of the commanded speed given up while the command still
        // has to turn 90 degrees or more toward the request, recovering
        // linearly as the remaining angle closes (walkers slow down to turn
        // around). Zero keeps full speed through turns.
        float mSteeringTurnSlowdown = 0.0f;
        // While stopping, the forward placement uses this velocity-error gain
        // instead of mForwardVelocityErrorGain, so each step lands beyond the
        // capture point and takes speed out. Measured live: at the creep
        // command the gait otherwise settles at about 9 u/s and never gets
        // below the hand-over speed. Zero keeps the walking gain.
        float mStoppingVelocityErrorGain = 1.2f;
        // A stop that has crept for longer than this (seconds) hands over at
        // the next double support whatever the speed, so a stop always ends.
        float mSteeringStopTimeout = 3.0f;
        // Hand-over to standing (ReferenceGaitHandover): once the stop hands
        // the body over, the standing controller reaches its motor pose,
        // pelvis-height target and horizontal support target by a timed
        // blend from the measured walking end state over this many seconds.
        // Without it the authored springs snap the flexed walking knees to
        // the straight standing pose, and the standing balance requests up
        // to 70 u/s^2 through the ankles because the walking end pose leaves
        // the center of mass far from the mid-foot support center; that
        // ankle torque lifts a heel, raises the body and drops the contacts
        // from 8 to 1, after which the request chases the shrinking support.
        // Measured flat from 0.3 to 1.0 s on the stop multi-push set and
        // three stop times (stop-handover-results-2026-09-07.txt). Zero
        // keeps the snap.
        float mStandingHandoverDuration = 0.5f;
        // Hit stun (ReferenceGaitHitStun). An impulse at or above the
        // minimum suppresses the victim's locomotion request for a moment,
        // so a hit visibly interrupts what they were doing instead of being
        // walked off. The controller keeps full balance authority: the stun
        // changes the REQUEST, never the mode, and the physics still decides
        // whether the body absorbs, staggers or falls. Measured bands on a
        // walking target (hit-reactions-2026-09-07.txt): up to about 1250
        // units is absorbed, 1250-1750 staggers, above that falls become
        // common and are direction-dependent. Zero disables the stun.
        float mHitStunMinimumImpulse = 400.0f;
        // Stun length at mHitStunReferenceImpulse, scaled linearly with the
        // impulse and capped at twice the reference.
        float mHitStunDuration = 0.35f;
        float mHitStunReferenceImpulse = 1500.0f;
        // Impulse bands measured on a walking target: below the stagger
        // threshold the body absorbs the hit on its feet; at or above it the
        // root is released so the body physically staggers; at or above the
        // fall threshold a knock-down is expected. Repeated smaller hits
        // escalate: their decaying sum is compared against the fall
        // threshold, so a flurry can put the victim down.
        float mHitStunStaggerImpulse = 1250.0f;
        float mHitStunFallImpulse = 1750.0f;
        // Final-command feasibility gate (radians). When positive, each
        // driven hip/ankle target is evaluated against its authored
        // SwingTwist cone shrunken by the role's margin (cone half-angles
        // and twist range) at the END of the plan, after the rate-consistent
        // thigh hold, the landing press and the coordinated pose, and is
        // projected onto that shrunken boundary when it lies outside, so the
        // motor is never commanded closer than the margin to the anatomical
        // limit. The projection is Jolt's ClampSwingTwist: the swing part
        // moves to the boundary, twist is preserved where possible. The
        // stance (loaded) joints take the strict margin -- the trace showed
        // every hip violation on the stance hip under full load -- while the
        // airborne swing leg would take a lenient one. In double support
        // both legs are loaded and take the stance margin. Zero disables
        // that role's gate and leaves its command bit-identical.
        // Swing-side default is OFF, measured 2026-09-08 on the 7 s / 60 u/s
        // contract (layer 1 screen): clamping the swing leg's final command
        // fights the swing trajectory's velocity feed-forward, whose rate
        // targets are computed from the UNCLAMPED plan, so the motor is
        // pulled toward the clamped pose while its damping is fed the
        // unclamped rate. Every tried margin degraded the walk: 0.05 -> mean
        // 28.35 (baseline 29.85), 0.15 -> landing width 27.69 (24.41
        // stance-only, 25.06 baseline), 0.30 -> fell. The trace put all hip
        // violations on the stance hip, so the swing role has no measured
        // problem to solve. The field stays for future consistent
        // (position+rate) swing clamping.
        float mFinalCommandFeasibilityStanceMargin = 0.05f;
        float mFinalCommandFeasibilitySwingMargin = 0.0f;
    };

    // Canonical identity of controller settings, separate from rig/anatomy hash.
    std::uint64_t hashReferenceGaitProfile(const ReferenceGaitProfile& profile);

    // Rate-limits the locomotion request for the reference route; both
    // adapters run one instance per powered body and feed its command to
    // the plan and the state machine instead of the raw request.
    struct ReferenceGaitSteering
    {
        JPH::Vec3 mCommand = JPH::Vec3::sZero();
        // True while the request is zero but the command still walks the
        // body down to the hand-over; the plan brakes harder then.
        bool mStopping = false;
        float mStoppingTime = 0.0f;
        // Unit heading of the last non-zero command. A restart from rest
        // starts along it and turns toward the request at the turn rate,
        // like a moving walker; a walker with no heading yet takes the
        // request at once. A restart snapped 90 degrees from the staggered
        // standing stance fell in 2-4 of 8 stop/restart cases
        // (stop-handover-results-2026-09-07.txt).
        JPH::Vec3 mLastHeading = JPH::Vec3::sZero();
        // Rate-limited facing, unit length, zero until a facing is requested.
        // An AI walker never sets one and faces its travel direction, exactly
        // as before. A player sets it from the camera every step: their aim
        // must never be rate-limited, only their feet, so the travel command
        // follows the request directly (directTravel) while the body's
        // facing turns at mSteeringTurnRate behind it (plan item 2.3b).
        JPH::Vec3 mFacing = JPH::Vec3::sZero();
        bool stopping() const { return mStopping; }
        JPH::Vec3 facing() const { return mFacing; }
        // Turns the body's facing toward the request at the profile turn
        // rate. A zero or non-finite request, or a zero turn rate, snaps to
        // the request. Independent of step(): travel and facing are separate
        // degrees of freedom.
        void stepFacing(JPH::Vec3Arg requestedFacing, float duration, const ReferenceGaitProfile& profile);
        // Returns the filtered horizontal command for this step. A non-finite
        // request or duration leaves the command unchanged. standingReady
        // says both feet carry the body (double support or no active gait):
        // a stop only hands the body to the standing controller then, and
        // otherwise keeps creeping at the stop speed until it is, because
        // the standing controller cannot take over with a foot in the air.
        // standingReadyIgnoringSpeed says both feet are planted regardless of
        // speed; after the stop timeout the hand-over accepts that instead.
        // directTravel leaves the travel heading unfiltered, so a player's
        // input takes effect at once and the body accelerates rather than
        // sweeping its direction round at the turn rate. AI walkers leave it
        // false and keep the filtered heading their gates were measured on.
        // When available, measured velocity caps the initial stopping speed.
        // Unattained command speed must not prolong acceleration after release.
        // Omission preserves command-only replay/fixture semantics.
        JPH::Vec3 step(JPH::Vec3Arg request, float duration, const ReferenceGaitProfile& profile,
            bool standingReady = true, bool standingReadyIgnoringSpeed = true, bool directTravel = false,
            std::optional<JPH::Vec3> measuredVelocity = std::nullopt);
        void reset()
        {
            mCommand = JPH::Vec3::sZero();
            mStopping = false;
            mStoppingTime = 0.0f;
            mLastHeading = JPH::Vec3::sZero();
            mFacing = JPH::Vec3::sZero();
        }
    };

    // Suppresses the locomotion request for a moment after a hit, so a blow
    // interrupts the victim instead of being walked off. One instance per
    // powered body in the game adapter; the offline scenarios have no melee.
    struct ReferenceGaitHitStun
    {
        float mRemaining = 0.0f;
        bool active() const { return mRemaining > 0.0f; }
        // Starts (or extends) the stun for an impulse of this magnitude. A
        // magnitude below the profile minimum, or a non-finite one, does
        // nothing. A second hit never shortens an existing stun.
        void begin(float impulseMagnitude, const ReferenceGaitProfile& profile);
        void step(float duration);
        void reset() { mRemaining = 0.0f; }
    };

    // Blends the standing controller's motor pose and pelvis-height target
    // from the measured pose at the step it takes over from the reference
    // gait (ReferenceGaitProfile::mStandingHandoverDuration). Both adapters
    // run one instance per powered body: begin() at the hand-over, step()
    // once per fixed step while standing drives, reset() when the gait,
    // a release or a recovery takes over.
    struct ReferenceGaitHandover
    {
        std::optional<RagdollPose> mSourcePose;
        float mSourcePelvisHeight = 0.0f;
        JPH::RVec3 mSourceCenterOfMass = JPH::RVec3::sZero();
        float mTime = 0.0f;
        bool active() const { return mSourcePose.has_value(); }
        // Starts the blend from the measured model pose, pelvis height and
        // center of mass. A zero duration, a non-finite input or an empty
        // pose leaves it off.
        void begin(const RagdollPose& measuredPose, float pelvisHeight, JPH::RVec3Arg centerOfMass,
            const ReferenceGaitProfile& profile);
        // Advances one fixed step; the blend ends by itself at the duration.
        void step(float duration, const ReferenceGaitProfile& profile);
        // Smoothstep weight of the standing target, 0 at begin() and 1 at the end.
        float factor(const ReferenceGaitProfile& profile) const;
        // Motor target for this step: the source pose slerped toward the
        // standing pose. False when inactive or the poses do not match.
        bool blendPose(const RagdollPose& standing, const ReferenceGaitProfile& profile, RagdollPose& out) const;
        // Pelvis-height target for this step; the standing target when inactive.
        float blendPelvisHeight(float standingTarget, const ReferenceGaitProfile& profile) const;
        // Support reference for the standing joint support while the blend
        // runs: its position moves from the center of mass at the hand-over
        // to the support center and its velocity from the measured
        // center-of-mass velocity to rest, so the standing balance request
        // (stiffness and damping) starts at zero and ramps in. Unset when
        // inactive.
        std::optional<SupportMotionReference> supportReference(JPH::RVec3Arg supportCenter,
            JPH::Vec3Arg centerOfMassVelocity, const ReferenceGaitProfile& profile) const;
        void reset()
        {
            mSourcePose.reset();
            mSourcePelvisHeight = 0.0f;
            mTime = 0.0f;
        }
    };

    enum class ReferenceGaitStage : std::uint8_t
    {
        Idle,
        DoubleSupport,
        Swing,
    };

    const char* referenceGaitStageName(ReferenceGaitStage stage);

    // Forward-reserve governor (experimental). Guarantees the forward
    // component of the combined request up to forwardReserveFraction of the
    // magnitude budget whenever forward demand exists, and caps the lateral
    // component to the remaining budget sqrt(max^2 - reserve^2). A step with
    // no forward demand keeps the full lateral budget, so pure lateral crises
    // (shove capture) are unaffected. Braking demand (forward < 0) reserves
    // the same way. reserveFraction <= 0 leaves the request unchanged.
    // Motivation, measured on the a63 immediate-turn 120 FPS replay: the
    // lateral pendulum term pins the request at the 75.6 u/s^2 magnitude
    // clamp while forward demand of 44-78 is served 2-18; the run falls.
    JPH::Vec3 governForwardReserve(JPH::Vec3Arg acceleration, JPH::Vec3Arg forward, JPH::Vec3Arg right,
        float maximumAcceleration, float forwardReserveFraction);

    struct ReferenceGaitStateInput
    {
        bool mEnabled = false;
        // Stationary recovery may change support only for a confirmed capture
        // emergency. Disable ordinary cadence without bypassing load gates.
        bool mEmergencyOnly = false;
        // World-space foot centers used to establish the balance goal at
        // entry to double support; they do not replace measured contacts.
        JPH::RVec3 mLeftFootCenter = JPH::RVec3::sZero();
        JPH::RVec3 mRightFootCenter = JPH::RVec3::sZero();
        bool mLeftFootSupported = false;
        bool mRightFootSupported = false;
        // Solved normal impulse under each foot over the step as a fraction
        // of body weight times the step; negative when unknown.
        float mLeftFootLoadFraction = -1.0f;
        float mRightFootLoadFraction = -1.0f;
        float mLeftFootHeight = 0.0f;
        float mRightFootHeight = 0.0f;
        float mLeftFootVerticalVelocity = 0.0f;
        float mRightFootVerticalVelocity = 0.0f;
        // Center-of-mass state for the reactive step rule
        // (ReferenceGaitProfile::mEmergencyCaptureMargin). Unset, a
        // non-positive height or gravity leaves the rule inactive.
        std::optional<JPH::RVec3> mCenterOfMass;
        JPH::Vec3 mCenterOfMassVelocity = JPH::Vec3::sZero();
        float mCenterOfMassHeight = 0.0f;
        float mGravity = 0.0f;
        // Requested horizontal travel direction (any length; zero falls back
        // to the direction perpendicular to the foot span). It defines the
        // lateral axis that decides which foot may step without crossing.
        JPH::Vec3 mHeading = JPH::Vec3::sZero();
        // i39 (delivery-comp-2026-09-11.txt): feedback of the plan's
        // predicted touchdown COM velocity from the previous swing, used
        // only by the env-gated delivery-error observer. Absent leaves the
        // observer without a measurement (bias stays 0).
        std::optional<JPH::Vec3> mPlacementPredictedVelocity;
    };

    struct ReferenceGaitState
    {
        GaitPhase mPhase = GaitPhase::Idle;
        std::optional<JPH::RVec3> mTransferCenter;
        bool mEnteredFromTouchdown = false;
        bool mInitialTransfer = true;
        // The current swing was started by the reactive step rule; cleared at
        // the next stage change. The count accumulates over the episode.
        bool mEmergencyStep = false;
        std::uint32_t mEmergencyStepCount = 0;
        // Disturbance observer (i12, placement-lead-observer-2026-09-11.txt):
        // remaining seconds of the latched push window. Published for the
        // placement lead; nothing in the state machine reads it back.
        float mDisturbanceLatchRemaining = 0.0f;
        // Delivery-error observer (i39, delivery-comp-2026-09-11.txt):
        // low-passed signed difference between measured and planned
        // touchdown velocity along the travel axis (u/s). Updated at
        // touchdown transitions when the caller feeds
        // ReferenceGaitStateInput::mPlacementPredictedVelocity and the
        // OPENMW_RAGDOLL_DELIVERY_COMP env gate sets alpha; otherwise
        // stays exactly 0 and the placement path is byte-identical.
        float mDeliveryVelocityBias = 0.0f;
        ReferenceGaitStage mStage = ReferenceGaitStage::Idle;
        float mTimeInPhase = 0.0f;
        float mTimeInStage = 0.0f;
        float mSwingContactTime = 0.0f;
        float mSwingTimeSinceLift = 0.0f;
        float mSwingStartRelativeHeight = 0.0f;
        bool mSwingFootLifted = false;
        bool mTransitioned = false;
        bool mTimedOut = false;

        bool hasStance() const { return mPhase != GaitPhase::Idle; }
        bool leftFootIsStance() const { return mPhase == GaitPhase::LeftStance; }
    };

    // Contact-driven stance machine. A stance begins on measured foot support;
    // swing requires the double-support interval and actual stance support.
    // Transfer requires confirmed swing-foot contact after the minimum swing
    // time. Exceeding a deadline is reported, never fabricated into a transfer.
    class ReferenceGaitStateMachine
    {
    public:
        bool step(const ReferenceGaitStateInput& input, float duration, const ReferenceGaitProfile& profile,
            ReferenceGaitState& output);
        void reset();
        const ReferenceGaitState& state() const { return mState; }

    private:
        ReferenceGaitState mState;
        float mLeftEmergencyLoadTime = 0.0f;
        float mRightEmergencyLoadTime = 0.0f;
        float mInitialStanceLoadTime = 0.0f;
        // Disturbance observer state (i12): previous planar COM velocity for
        // the per-step jerk test, and the remaining latch time.
        std::optional<JPH::Vec3> mPreviousComVelocity;
        float mDisturbanceLatch = 0.0f;
    };

    // Maps the reference state onto the shared gait output consumed by stage
    // classification, telemetry and the existing acceptance summaries.
    GaitControllerOutput toGaitControllerOutput(const ReferenceGaitState& state, const ReferenceGaitProfile& profile);

    struct ReferenceGaitPlacementInput
    {
        bool mStationaryCorrectiveStep = false;
        GaitPhase mPhase = GaitPhase::Idle;
        // Retained facing input. Placement currently keeps its lateral side
        // in the travel frame; the facing-based sign flip was reverted after
        // native forward-walking regressions. Do not infer that this field
        // changes placement side (see calculateReferenceGaitPlacement).
        JPH::Vec3 mFacing = JPH::Vec3::sZero();
        // Fable i50: MEASURED body heading (pelvis yaw), used only by the
        // env-gated body-frame side rule. mFacing is the requested facing,
        // which the harness defaults to the travel direction, so it cannot
        // tell a backpedal or a strafe from a forward walk.
        std::optional<JPH::Vec3> mBodyForward;
        JPH::RVec3 mCenterOfMass = JPH::RVec3::sZero();
        JPH::Vec3 mCenterOfMassVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mRequestedVelocity = JPH::Vec3::sZero();
        // Stance foot bone origin; its height is the landing height.
        JPH::RVec3 mStanceFootPosition = JPH::RVec3::sZero();
        // Swing foot position, in world units: lets the guarded
        // measured-side override in calculateReferenceGaitPlacement keep
        // the swing foot on its measured side when travel opposes facing
        // (backpedal), where the anatomical side rule inverts. Supplied by
        // the production planner since i37
        // (backpedal-side-guard-2026-09-11.txt). The unguarded 1e-4
        // version was tried natively on 2026-09-08 and rejected after
        // post-recovery stance reversals; the current guard requires
        // separation beyond half the lateral capture offset.
        std::optional<JPH::RVec3> mSwingStartPosition;
        // i39 (delivery-comp-2026-09-11.txt): signed travel-frame velocity
        // bias (u/s) from the delivery-error observer; added to the
        // predicted touchdown speed in the forward-offset error term so
        // placements anticipate systematic over/under-delivery. Absent = 0
        // = byte-identical legacy behavior.
        std::optional<float> mDeliveryVelocityBias;
        float mCenterOfMassHeight = 0.0f;
        float mGravity = 0.0f;
        // Double support plus swing: the period of one stance.
        float mStancePeriod = 0.0f;
        // Prediction: remaining swing time and the single-support
        // center-of-pressure policy (forward request gain and the effective
        // forward half-length of the stance foot hull). Zero half-length
        // keeps the center of pressure at the foot center.
        bool mPredict = false;
        float mRemainingSwingTime = 0.0f;
        float mVelocityGain = 0.0f;
        float mForwardPressureHalfLength = 0.0f;
        JPH::RVec3 mStanceFootCenter = JPH::RVec3::sZero();
        // Replaces the profile's forward velocity-error gain when set (the
        // plan passes the stopping gain while the steering filter stops).
        std::optional<float> mForwardVelocityErrorGain;
        // Opt-in lateral capture lead in seconds (i12). The plan passes the
        // env knob value only while the disturbance latch is active; zero
        // keeps the nominal capture tracking exactly.
        float mCaptureLead = 0.0f;
        // Opt-in signed travel-frame yaw rate in rad/s (i33,
        // rotation-predict-2026-09-11.txt). When set (env-gated) and nonzero,
        // the placement offset axes are rotated forward by
        // mTravelYawRate * mRemainingSwingTime so the touchdown target is
        // expressed in the frame as it will exist at touchdown; zero keeps
        // the current-frame placement exactly.
        float mTravelYawRate = 0.0f;
    };

    struct ReferenceGaitPlacement
    {
        JPH::RVec3 mTarget = JPH::RVec3::sZero();
        JPH::Vec3 mForward = JPH::Vec3::sAxisY();
        JPH::Vec3 mRight = JPH::Vec3::sAxisX();
        float mCaptureTimeConstant = 0.0f;
        float mSteadyForwardFraction = 0.0f;
        float mForwardOffset = 0.0f;
        float mLateralOffset = 0.0f;
        bool mWidthClamped = false;
        bool mReachClamped = false;
        // Center of mass state the placement was computed from (predicted
        // when prediction is enabled, otherwise the input).
        JPH::RVec3 mPlacementCenterOfMass = JPH::RVec3::sZero();
        JPH::Vec3 mPlacementVelocity = JPH::Vec3::sZero();
    };

    std::optional<ReferenceGaitPlacement> calculateReferenceGaitPlacement(
        const ReferenceGaitPlacementInput& input, const ReferenceGaitProfile& profile);

    struct ReferenceSwingTrajectoryInput
    {
        JPH::Vec3 mLiftOffPosition = JPH::Vec3::sZero();
        JPH::Vec3 mPlacementPosition = JPH::Vec3::sZero();
        // Seconds since the swing stage began.
        float mSwingTime = 0.0f;
        // Diagnostic: rise to clearance above lift-off and hold there.
        bool mHoldLifted = false;
        // Opt-in world velocity of the placement target (i45,
        // swing-intercept-2026-09-12.txt), expressed in the model frame.
        // Supplied only under the intercept env gate while the travel frame
        // rotates; zero keeps the legacy trajectory exactly.
        JPH::Vec3 mPlacementVelocity = JPH::Vec3::sZero();
    };

    std::optional<JPH::Vec3> calculateReferenceSwingTrajectory(
        const ReferenceSwingTrajectoryInput& input, const ReferenceGaitProfile& profile);

    // Use authored gains to express a world-space PD request as a velocity
    // target: K*error - D*omega = D*(targetRate - omega). This is a preference;
    // the prepared physical solve still supplies all actual actuation.
    std::optional<JPH::Vec3> referenceGaitRootRecoveryRate(
        JPH::Vec3Arg worldOrientationError, float stiffness, float damping);

    struct ReferenceRootRoutingInput
    {
        // World torque the pelvis should receive from its joints in addition
        // to the inverse-static plan and the hip springs' own response.
        JPH::Vec3 mPelvisTorqueRequest = JPH::Vec3::sZero();
        // Estimated dynamic (non-static) child torque of every root-attached
        // joint that is not a routing hip, e.g. the spine and the swing hip.
        std::vector<JPH::Vec3> mBranchDynamicTorques;
        float mLeftHipLoadFraction = 0.0f;
        float mRightHipLoadFraction = 0.0f;
    };

    struct ReferenceRootRouting
    {
        // Child-positive torque additions for the left and right hips.
        JPH::Vec3 mLeftHipTorque = JPH::Vec3::sZero();
        JPH::Vec3 mRightHipTorque = JPH::Vec3::sZero();
        JPH::Vec3 mBranchReaction = JPH::Vec3::sZero();
    };

    // The pelvis receives minus the sum of its children's torques. To give it
    // the requested torque despite the other branches, the loaded hips take
    // -(request + sum of branch dynamic torques), split by load fraction.
    std::optional<ReferenceRootRouting> routeReferenceRootTorque(const ReferenceRootRoutingInput& input);

    struct ReferenceGaitLeg
    {
        std::size_t mHip = 0;
        std::size_t mKnee = 0;
        std::size_t mFoot = 0;
        std::optional<HingeLegGeometry> mHingeGeometry;
        std::optional<SwingTwistJointGeometry> mHipGeometry;
        std::optional<SwingTwistJointGeometry> mAnkleGeometry;
    };

    struct ReferenceStancePosture
    {
        RagdollPose mPose;
        float mMinimumPelvisHeight = 0.0f;
        float mMaximumPelvisHeight = 0.0f;
    };

    // Project leg joint rotations into native limits and rebuild knee/foot
    // origins from native hinge geometry. Preserves pelvis, hip origins and
    // non-leg bodies. Feet are free: this does not establish contact, balance
    // or a safe path between poses. Returns a reference, never a body teleport.
    std::optional<RagdollPose> projectReferenceLegPosture(std::span<const Transform> pose,
        std::size_t pelvis, const ReferenceGaitLeg& left, const ReferenceGaitLeg& right);

    // Choose the nearest height to desiredPelvis at which both leg chains can
    // reach their foot targets, with hips above feet. All quantities are model
    // space, including height along +Z. Requires native hinge geometry; accounts
    // for axial offsets and knee limits. Solves both legs in one pelvis frame.
    // This is a kinematic prerequisite, not proof of hip/ankle limits, collision
    // clearance, balance or dynamic tracking. The pose is a motor reference;
    // callers must never teleport the physical pelvis to its translation.
    std::optional<ReferenceStancePosture> planReferenceStancePosture(std::span<const Transform> basePose,
        std::size_t pelvis, const ReferenceGaitLeg& left, const ReferenceGaitLeg& right,
        const Transform& desiredPelvis, JPH::Vec3Arg leftFoot, JPH::Vec3Arg rightFoot,
        JPH::Vec3Arg preferredBendDirection);

    // Preserves both requested foot positions and every specified orientation,
    // and accepts only poses inside the supplied native hip/ankle limits.
    // An unspecified airborne-foot orientation may tilt within the ankle
    // limits toward the base foot orientation; its endpoint is never moved.
    // Requires all six joint geometries. Searches bounded pelvis heights and
    // knee-plane turns toward each specified foot orientation, then a bounded
    // full-swivel fallback for rejected legs, refining feasible brackets.
    // Successful earlier solutions are retained. Failure is conservative,
    // not proof of impossibility.
    // Does not change the earlier radial-only planner or actuate a body.
    std::optional<ReferenceStancePosture> planJointLimitedReferenceStancePosture(std::span<const Transform> basePose,
        std::size_t pelvis, const ReferenceGaitLeg& left, const ReferenceGaitLeg& right,
        const Transform& desiredPelvis, JPH::Vec3Arg leftFoot, JPH::Vec3Arg rightFoot,
        const std::optional<JPH::Quat>& leftFootRotation, const std::optional<JPH::Quat>& rightFootRotation,
        JPH::Vec3Arg preferredBendDirection);

    struct ReferencePostureMass
    {
        float mMass = 0;
        // Body-local offset expressed in the same model units as the pose.
        JPH::Vec3 mLocalCenterOfMass = JPH::Vec3::sZero();
    };

    // Keeps pelvis height/orientation and both foot transforms fixed while
    // finding pelvis XY for the requested horizontal full-body COM. Requires
    // native leg geometry and each body's real mass/COM offset. The caller
    // chooses any allowed torso lean; this query never silently changes it.
    // Bounded iteration may conservatively reject a feasible configuration.
    // A valid motor pose does not establish collision or dynamic feasibility.
    // allowHeightLowering permits geometry-required lowering during the
    // bounded solve; the default preserves the fixed-height query contract.
    std::optional<ReferenceStancePosture> planBalancedReferenceStancePosture(
        std::span<const Transform> basePose, std::span<const ReferencePostureMass> masses,
        std::size_t pelvis, const ReferenceGaitLeg& left, const ReferenceGaitLeg& right,
        const Transform& desiredPelvis, const Transform& leftFoot, const Transform& rightFoot,
        JPH::Vec3Arg desiredCenterOfMass, JPH::Vec3Arg preferredBendDirection,
        bool allowHeightLowering = false);

    // Coordinated supported swing: preserve both foot endpoints and the
    // stance foot orientation, and solve pelvis XY for horizontal full-body
    // COM within 0.001 model units. Height is clamped to the two legs' reach;
    // the resulting pose must satisfy native limits. The caller supplies the
    // intended height/orientation trajectory. The
    // swing foot may tilt from its base orientation inside the ankle cone;
    // its supplied rotation is ignored. swingFoot must identify exactly one
    // of the two leg feet.
    // This plans a motor reference; it does not establish physical tracking.
    std::optional<ReferenceStancePosture> planBalancedReferenceSwingPosture(
        std::span<const Transform> basePose, std::span<const ReferencePostureMass> masses,
        std::size_t pelvis, const ReferenceGaitLeg& left, const ReferenceGaitLeg& right,
        const Transform& desiredPelvis, const Transform& leftFoot, const Transform& rightFoot,
        std::size_t swingFoot, JPH::Vec3Arg desiredCenterOfMass, JPH::Vec3Arg preferredBendDirection);

    struct ReferenceGaitPlanInput
    {
        const RagdollTelemetry* mTelemetry = nullptr;
        std::span<const Transform> mBasePose;
        std::span<const Transform> mMeasuredPose;
        std::span<const GroundContact> mContacts;
        SupportEstimate mSupport;
        std::size_t mPelvis = 0;
        // Root-attached spine body and its chain toward the head.
        std::vector<std::size_t> mTrunk;
        ReferenceGaitLeg mLeftLeg;
        ReferenceGaitLeg mRightLeg;
        // Where the body POINTS, which need not be where it travels. Zero
        // means face the direction of travel, which is what every AI walker
        // does and what the gait did exclusively before the split: the plan
        // built both its placement frame and its pelvis aim from the
        // requested velocity, so a player could never strafe or back up
        // (plan item 2.3a). Setting it aims the pelvis independently while
        // the feet keep stepping along the travel direction.
        JPH::Vec3 mFacing = JPH::Vec3::sZero();
        // The steering filter is walking the body down to a stop: placement
        // brakes with ReferenceGaitProfile::mStoppingVelocityErrorGain.
        bool mStopping = false;
        bool mStationaryCorrectiveStep = false;
        JPH::Vec3 mCenterOfMassVelocity = JPH::Vec3::sZero();
        JPH::Vec3 mRequestedHorizontalVelocity = JPH::Vec3::sZero();
        float mGravity = 0.0f;
        float mFixedTimeStep = 0.0f;
        JPH::RVec3 mRootPosition = JPH::RVec3::sZero();
        JPH::Quat mRootRotation = JPH::Quat::sIdentity();
        float mUniformScale = 1.0f;
        ReferenceGaitState mState;
        // Model-space swing foot origin latched when the swing stage began.
        std::optional<JPH::Vec3> mSwingStartModel;
        // Previous command history in this same model frame; adapters must rebase it.
        std::optional<SwingTrajectoryHistory> mSwingTrajectoryHistory;
        // Model-space landed foot origin latched when a touchdown transfer
        // entered double support; the landed leg is pressed toward it.
        std::optional<JPH::Vec3> mLandingFootModel;
        // Previous step: commanded load torque and measured motor torque per
        // body (world, child-positive). Their difference estimates each
        // joint's dynamic spring torque for root reaction accounting.
        std::span<const JPH::Vec3> mPreviousLoadTorques;
        std::span<const JPH::Vec3> mPreviousMotorTorques;
        // Placement chosen on the previous step of this swing, kept once the
        // latch fraction of the swing has elapsed.
        std::optional<JPH::RVec3> mPreviousPlacement;
        bool mHoldSwingFoot = false;
        // Caller-observed horizontal landing plane plus foot geometry at its
        // intended orientation, expressed as world BODY-origin height.
        // Unset retains the legacy stance-origin height.
        std::optional<double> mLandingFootOriginHeight;
        bool mCoordinateLegPose = false;
        // Preferred pelvis COM height above the observed ground, world units.
        // Geometry may lower/raise the body-origin target to a feasible pose.
        std::optional<double> mPreferredPelvisCenterHeight;
        // Signed travel-frame yaw rate in rad/s, measured by the caller from
        // successive steering commands (i33). Consumed only under the
        // rotation-predict env gate; zero keeps placement exactly current.
        float mTravelYawRate = 0.0f;
    };

    struct ReferenceGaitPlan
    {
        RagdollPose mTargetPose;
        // World, child-positive; index zero (root) is always zero.
        std::vector<JPH::Vec3> mLoadTorques;
        std::optional<std::size_t> mSwingHip;
        std::optional<JPH::Quat> mSwingHipWorldTarget;
        // Hip, knee and foot of the swinging leg while its spring override applies.
        std::vector<std::size_t> mSwingLegSpringBodies;
        float mSwingLegFrequency = 0.0f;
        float mSwingLegDamping = 0.0f;
        std::vector<std::size_t> mStanceLegSpringBodies;
        float mStanceLegFrequency = 0.0f;
        float mStanceLegDamping = 0.0f;
        std::vector<std::size_t> mStanceHipSpringBodies;
        float mStanceHipFrequency = 0.0f;
        float mStanceHipDamping = 0.0f;
        std::vector<std::size_t> mTrunkSpringBodies;
        float mTrunkFrequency = 0.0f;
        float mTrunkDamping = 0.0f;
        std::vector<std::size_t> mLoadedAnkleBodies;
        float mLoadedAnkleFrequency = 0.0f;
        float mLoadedAnkleDamping = 0.0f;
        bool mPlacementLatched = false;
        std::size_t mStanceHip = 0;
        std::size_t mStanceAnkle = 0;
        std::vector<std::pair<std::size_t, JPH::Quat>> mWorldFootTargets;
        JointSupportOutput mSupport;
        std::optional<JPH::RVec3> mPlacement;
        std::optional<JPH::Vec3> mSwingFootTarget;
        // Empty outside the continuous swing path, including hold/coordinated modes.
        std::optional<SwingTrajectoryHistory> mSwingTrajectoryHistory;
        bool mSwingPoseApplied = false;
        bool mCoordinatedPoseAttempted = false;
        bool mCoordinatedPoseApplied = false;
        // Either the explicit placement bound or native leg IK limited reach.
        bool mSwingReachLimited = false;
        JPH::Vec3 mRequestedAcceleration = JPH::Vec3::sZero();
        JPH::Vec3 mTrackingVelocity = JPH::Vec3::sZero();
        // Axis-angle error from the measured pelvis to its heading-upright target.
        JPH::Vec3 mPelvisOrientationError = JPH::Vec3::sZero();
        JPH::Vec3 mBranchReaction = JPH::Vec3::sZero();
        JPH::Vec3 mStanceHipTorque = JPH::Vec3::sZero();
        float mLeftLoadFraction = 0.0f;
        float mRightLoadFraction = 0.0f;
        float mPlannedStanceFootLoad = 0.0f;
        // Lateral COM velocity toward the stance foot and its cycle reference.
        float mTransferVelocity = 0.0f;
        float mTransferReferenceVelocity = 0.0f;
        // World angular velocity targets for trajectory-driven joints: the
        // child's absolute rate for a joint with a world target, otherwise
        // the child-minus-parent relative rate.
        std::vector<std::pair<std::size_t, JPH::Vec3>> mJointAngularVelocityTargets;
        bool mLandingPressApplied = false;
        // Diagnostics: computed-torque feed-forward on the swing hip and knee
        // (world, child-positive), the swing hip's target and measured world
        // rates, and the center-of-mass state the placement was computed from.
        JPH::Vec3 mSwingHipFeedForwardTorque = JPH::Vec3::sZero();
        JPH::Vec3 mSwingKneeFeedForwardTorque = JPH::Vec3::sZero();
        JPH::Vec3 mSwingHipRateTarget = JPH::Vec3::sZero();
        JPH::Vec3 mSwingHipMeasuredRate = JPH::Vec3::sZero();
        std::optional<JPH::RVec3> mPlacementCenterOfMass;
        JPH::Vec3 mPlacementVelocity = JPH::Vec3::sZero();
        // Final-command feasibility gate engagement: hip/ankle motor targets
        // this step projected onto the margin-shrunk SwingTwist cone (zero
        // when the gate is disabled or no target lay outside it).
        std::uint32_t mFeasibilityClampedCount = 0;
    };

    // A rejected observation reports its reason when a string is supplied.
    // Freeze world targets at the measured parent orientation, preserving all
    // other planned relative rotations. Suitable for native relative motors.
    std::optional<RagdollPose> referenceGaitRelativeMotorPose(const ReferenceGaitPlan& plan,
        std::span<const Transform> measured, std::span<const int> parents, JPH::QuatArg rootRotation);

    std::optional<ReferenceGaitPlan> planReferenceGaitActuation(const ReferenceGaitPlanInput& input,
        const ReferenceGaitProfile& profile, std::string* rejection = nullptr);
}

#endif

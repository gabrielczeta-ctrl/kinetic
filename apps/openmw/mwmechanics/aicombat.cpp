#include "aicombat.hpp"
#include <ragdollcore/tuning.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <vector>

#include <components/debug/debuglog.hpp>
#include <components/detournavigator/navigatorutils.hpp>
#include <components/esm3/aisequence.hpp>
#include <components/esm3/loadarmo.hpp>
#include <components/misc/coordinateconverter.hpp>
#include <components/misc/mathutil.hpp>
#include <components/misc/pathgridutils.hpp>
#include <components/misc/rng.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>

#include "../mwphysics/raycasting.hpp"

#include "../mwworld/class.hpp"
#include "../mwworld/inventorystore.hpp"
#include "../mwworld/esmstore.hpp"

#include "../mwbase/dialoguemanager.hpp"
#include "../mwbase/environment.hpp"
#include "../mwbase/mechanicsmanager.hpp"
#include "../mwbase/world.hpp"

#include "actorutil.hpp"
#include "aicombataction.hpp"
#include "character.hpp"
#include "combat.hpp"
#include "creaturestats.hpp"
#include "movement.hpp"
#include "pathgrid.hpp"
#include "steering.hpp"
#include "weapontype.hpp"

namespace
{
    // CROWD (user 10-09: "better AI when fighting multiple enemies, they start
    // overlapping each other because they grab the same path to approach the
    // player"; chosen: all attack, spaced). Every melee attacker writes where
    // it stands about its target each frame; each one then keeps an angle
    // from the others round that target (all of them still attack, from
    // their own sides), heads for its own point on the way in instead of the
    // target's position, and holds its cut while an ally stands in the line.
    // OPENMW_RAGDOLL_CROWD=1 (0 = each fights alone, as before).
    bool crowdEnabled()
    {
        static const auto enabled_tuned = RagdollCore::tuned<bool>([] {
            const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_CROWD");
            return text && std::atoi(text) != 0;
        });
        static const auto& enabled = enabled_tuned.get();
        return enabled;
    }

    struct CrowdMember
    {
        const void* mActor = nullptr;
        osg::Vec3f mPosition;
        std::chrono::steady_clock::time_point mSeen;
    };

    struct CrowdSteer
    {
        float mStrafe = 0.0f; // toward more room round the target (actor frame, facing the target)
        float mBearing = 0.0f; // the angle round the target to head for (radians, world)
        bool mHasOthers = false;
        bool mAllyInLine = false;
    };

    std::map<const void*, std::vector<CrowdMember>>& crowdRings()
    {
        static std::map<const void*, std::vector<CrowdMember>> rings;
        return rings;
    }

    CrowdSteer stepCrowd(const MWWorld::Ptr& actor, const MWWorld::Ptr& target)
    {
        CrowdSteer result;
        const auto now = std::chrono::steady_clock::now();
        const osg::Vec3f self = actor.getRefData().getPosition().asVec3();
        const osg::Vec3f centre = target.getRefData().getPosition().asVec3();
        auto& ring = crowdRings()[target.mRef];
        ring.erase(std::remove_if(ring.begin(), ring.end(),
                       [&](const CrowdMember& m) {
                           return m.mActor == actor.mRef || now - m.mSeen > std::chrono::milliseconds(500);
                       }),
            ring.end());
        osg::Vec2f radial(self.x() - centre.x(), self.y() - centre.y());
        const float distance = radial.length();
        const float bearing = std::atan2(radial.y(), radial.x());
        result.mBearing = bearing;
        // Room enough for each: a share of the circle, 50-110 degrees.
        int others = 0;
        for (const auto& m : ring)
            if ((m.mPosition - centre).length() < 700.0f)
                ++others;
        result.mHasOthers = others > 0;
        float push = 0.0f;
        if (others > 0 && distance > 1.0f)
        {
            const float gap = std::clamp(2.0f * osg::PIf / static_cast<float>(others + 1),
                osg::DegreesToRadians(50.0f), osg::DegreesToRadians(110.0f));
            const osg::Vec2f toTarget = -radial / distance;
            for (const auto& m : ring)
            {
                const osg::Vec2f r(m.mPosition.x() - centre.x(), m.mPosition.y() - centre.y());
                if (r.length() > 700.0f || r.length() < 1.0f)
                    continue;
                float d = bearing - std::atan2(r.y(), r.x());
                while (d > osg::PIf)
                    d -= 2.0f * osg::PIf;
                while (d < -osg::PIf)
                    d += 2.0f * osg::PIf;
                if (std::abs(d) < gap)
                {
                    // Even with both on one line, they part: the order of
                    // their records decides which way each goes.
                    const float side = std::abs(d) > 1e-3f ? (d > 0.0f ? 1.0f : -1.0f)
                                                           : (actor.mRef < m.mActor ? 1.0f : -1.0f);
                    push += side * (1.0f - std::abs(d) / gap);
                }
                // An ally between it and the target, near and in the line.
                const osg::Vec2f toAlly(m.mPosition.x() - self.x(), m.mPosition.y() - self.y());
                const float allyDistance = toAlly.length();
                if (allyDistance > 1.0f && allyDistance < 120.0f && allyDistance < distance
                    && (toAlly / allyDistance) * toTarget > std::cos(osg::DegreesToRadians(35.0f)))
                    result.mAllyInLine = true;
            }
            result.mBearing = bearing + std::clamp(push, -1.0f, 1.0f) * gap;
        }
        // Facing the target, moving right turns the bearing up (x east, y north).
        result.mStrafe = std::clamp(1.5f * push, -1.0f, 1.0f);
        ring.push_back(CrowdMember{ actor.mRef, self, now });
        return result;
    }

    // chooses an attack depending on probability to avoid uniformity
    std::string_view chooseBestAttack(const ESM::Weapon* weapon);

    bool hitAttemptMatchesTarget(const MWWorld::Ptr& actor, const MWWorld::Ptr& target)
    {
        ESM::RefNum hitNum = actor.getClass().getCreatureStats(actor).getHitAttemptActor();
        return hitNum.isSet() && target.getCellRef().getRefNum() == hitNum;
    }
}

namespace MWMechanics
{
    AiCombat::AiCombat(const MWWorld::Ptr& actor)
    {
        mTargetActor = actor.getCellRef().getRefNum();
    }

    AiCombat::AiCombat(const ESM::AiSequence::AiCombat* combat)
    {
        mTargetActor = combat->mTargetActor;
    }

    void AiCombat::init() {}

    /*
     * Current AiCombat movement states (as of 0.29.0), ignoring the details of the
     * attack states such as CombatMove, Strike and ReadyToAttack:
     *
     *    +----(within strike range)----->attack--(beyond strike range)-->follow
     *    |                                 | ^                            | |
     *    |                                 | |                            | |
     *  pursue<---(beyond follow range)-----+ +----(within strike range)---+ |
     *    ^                                                                  |
     *    |                                                                  |
     *    +-------------------------(beyond follow range)--------------------+
     *
     *
     * Below diagram is high level only, the code detail is a little different
     * (but including those detail will just complicate the diagram w/o adding much)
     *
     *    +----------(same)-------------->attack---------(same)---------->follow
     *    |                                 |^^                            |||
     *    |                                 |||                            |||
     *    |       +--(same)-----------------+|+----------(same)------------+||
     *    |       |                          |                              ||
     *    |       |                          | (in range)                   ||
     *    |   <---+         (too far)        |                              ||
     *  pursue<-------------------------[door open]<-----+                  ||
     *    ^^^                                            |                  ||
     *    |||                                            |                  ||
     *    ||+----------evade-----+                       |                  ||
     *    ||                     |    [closed door]      |                  ||
     *    |+----> maybe stuck, check --------------> back up, check door    ||
     *    |         ^   |   ^                          |   ^                ||
     *    |         |   |   |                          |   |                ||
     *    |         |   +---+                          +---+                ||
     *    |         +-------------------------------------------------------+|
     *    |                                                                  |
     *    +---------------------------(same)---------------------------------+
     *
     * FIXME:
     *
     * The new scheme is way too complicated, should really be implemented as a
     * proper state machine.
     *
     * TODO:
     *
     * Use the observer pattern to coordinate attacks, provide intelligence on
     * whether the target was hit, etc.
     */

    bool AiCombat::execute(
        const MWWorld::Ptr& actor, CharacterController& characterController, AiState& state, float duration)
    {
        // Get or create temporary storage
        AiCombatStorage& storage = state.get<AiCombatStorage>();

        const MWWorld::Class& actorClass = actor.getClass();
        if (!actorClass.isActor())
            return true;

        MWMechanics::CreatureStats& actorStats = actorClass.getCreatureStats(actor);

        // No combat for dead creatures
        if (actorStats.isDead())
            return true;

        const MWWorld::Ptr target = getTarget(); // The target to follow

        // Stop if the target doesn't exist
        if (target.isEmpty() || !target.getCellRef().getCount() || !target.getRefData().isEnabled()
            || target.getClass().getCreatureStats(target).isDead() || !target.getRefData().getBaseNode())
            return true;

        if (actor == target) // This should never happen.
            return true;

        // No actions for totally static creatures
        if (!actorClass.isMobile(actor))
        {
            storage.mFleeState = AiCombatStorage::FleeState_Idle;
            return false;
        }

        if (actorStats.isParalyzed() || actorStats.getKnockedDown())
            return false;

        if (!storage.isFleeing())
        {
            const ESM::Weapon* weapon = nullptr;
            bool isRangedCombat = false;

            if (storage.mCurrentAction.get()) // need to wait to init action with its attack range
            {
                // Update every frame. UpdateLOS uses a timer, so the LOS check does not happen every frame.
                updateLOS(actor, target, duration, storage);
                const float targetReachedTolerance
                    = storage.mLOS && !storage.mUseCustomDestination ? storage.mAttackRange : 0.0f;
                osg::Vec3f destination = storage.mUseCustomDestination
                    ? storage.mCustomDestination
                    : target.getRefData().getPosition().asVec3();
                // CROWD: on the way in, each heads for its own side of the
                // target (half the way, at most 160 units out from it).
                if (crowdEnabled() && !storage.mUseCustomDestination)
                {
                    const auto crowd = stepCrowd(actor, target);
                    const float away = (actor.getRefData().getPosition().asVec3() - destination).length();
                    if (crowd.mHasOthers && away > 180.0f)
                    {
                        const float out = std::min(0.5f * away, 160.0f);
                        destination += osg::Vec3f(std::cos(crowd.mBearing), std::sin(crowd.mBearing), 0.0f) * out;
                    }
                }
                const bool isTargetReached = pathTo(actor, destination, duration,
                    characterController.getSupportedMovementDirections(), targetReachedTolerance);
                if (isTargetReached)
                    storage.mReadyToAttack = true;

                weapon = storage.mCurrentAction->getWeapon();
                storage.mCurrentAction->getCombatRange(isRangedCombat);

                if (!isRangedCombat && storage.mShouldApproach && storage.mReadyToAttack)
                {
                    if (getDistanceToBounds(actor, target) > 64.f)
                    {
                        storage.stopCombatMove();
                        storage.mMovement.mPosition[1] = 1.f;
                    }
                    else
                    {
                        storage.mShouldApproach = false;
                        storage.mMovement.mPosition[1] = 0.f;
                    }
                }
            }

            // DUEL-AI-01 stage 1 (user 2026-09-29: "the duels slower and more
            // thought out, instead of running and spamming at the player, they
            // wait for the correct moment"): a powered melee fighter near its
            // target holds at the edge of reach and steps in to strike on an
            // opening (RagdollCore::stepDuelTactics), where vanilla ran in and
            // attacked whenever the clip allowed, with random sidesteps.
            // OPENMW_RAGDOLL_DUEL_TACTICS=0 restores vanilla.
            static const auto duelTactics_tuned = RagdollCore::tuned<bool>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DUEL_TACTICS");
                return !(text && std::atoi(text) == 0);
            });
            static const auto& duelTactics = duelTactics_tuned.get();
            const bool wasDuelling = storage.mDuelActive;
            storage.mDuelActive = false;
            storage.mDuelGuardWanted = false;
            if (duelTactics && !isRangedCombat && storage.mCurrentAction && storage.mLOS
                && MWBase::Environment::get().getWorld()->isPoweredRagdoll(actor))
            {
                const float distance = getDistanceToBounds(actor, target);
                const auto selfBody = getRagdollDuelView(actor);
                if (selfBody && distance < 400.0f)
                {
                    storage.mDuelActive = true;
                    if (!wasDuelling)
                        storage.mDuel = RagdollCore::DuelTacticsState{};
                    if (storage.mDuel.mSeed == 1)
                        storage.mDuel.mSeed = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(actor.mRef)) | 1u;
                    const auto reachOf = [](bool armed) {
                        return armed ? cRagdollMeleePhysicalReach : cRagdollMeleeUnarmedReach;
                    };
                    // The reach by the weapon's length (user 10-05, build 57: enemies
                    // should connect more and approach more thoughtfully): an armed
                    // body's 40 units are a longsword's (tip 66 from the hand);
                    // others hold `share` of the difference nearer or farther
                    // (-25..+40). Off by default: at 0.6 the duels landed fewer blows
                    // (DUEL-A02/L01/W01 + ARMS-01, one run: impacts 45 vs 66), the
                    // long weapons holding too far. OPENMW_RAGDOLL_DUEL_WEAPON_REACH
                    // = share (0 = 40 for every weapon).
                    static const auto weaponReachShare_tuned = RagdollCore::tuned<float>([] {
                        const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DUEL_WEAPON_REACH");
                        return text ? static_cast<float>(std::atof(text)) : 0.0f;
                    });
                    static const auto& weaponReachShare = weaponReachShare_tuned.get();
                    const auto bodyReach = [&](const RagdollCore::DuelBodyView& body) {
                        if (!body.mArmed || body.mWeaponReach <= 0.0f || weaponReachShare <= 0.0f)
                            return reachOf(body.mArmed);
                        return cRagdollMeleePhysicalReach
                            + std::clamp((body.mWeaponReach - 66.0f) * weaponReachShare, -25.0f, 40.0f);
                    };
                    RagdollCore::DuelSelfView self;
                    self.mReach = bodyReach(*selfBody);
                    const auto& fatigue = actorStats.getFatigue();
                    self.mFatigue = fatigue.getModified() > 0.0f
                        ? std::clamp(fatigue.getCurrent() / fatigue.getModified(), 0.0f, 1.0f)
                        : 1.0f;
                    self.mAttacking = storage.mAttack || !characterController.readyToStartAttack();
                    self.mAggression
                        = std::clamp(actorStats.getAiSetting(AiSetting::Fight).getModified() / 100.0f, 0.0f, 1.0f);
                    const ESM::RefId skill
                        = weapon ? getWeaponType(weapon->mData.mType)->mSkill : ESM::Skill::HandToHand;
                    self.mSkill = std::clamp(actorClass.getSkill(actor, skill) / 100.0f, 0.0f, 1.0f);
                    // Dodging learns like the guard (user 10-05, build 57: after
                    // repeated hits "guard that side, block or dodge"): each blow
                    // taken lately adds this much to the dodge chance (up to 0.9).
                    // OPENMW_RAGDOLL_DUEL_DODGE_LEARN (0 = off).
                    static const auto dodgeLearn_tuned = RagdollCore::tuned<float>([] {
                        const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DUEL_DODGE_LEARN");
                        return text ? static_cast<float>(std::atof(text)) : 0.08f;
                    });
                    static const auto& dodgeLearn = dodgeLearn_tuned.get();
                    self.mDodgeBonus = dodgeLearn * selfBody->mHitsTaken;
                    self.mGuardBroken = selfBody->mGuardBroken;
                    // AI-02 fighter terms from the stats (read by the AI_* items).
                    {
                        RagdollCore::DuelStatsInput in;
                        in.mWeaponSkill = actorClass.getSkill(actor, skill);
                        in.mBlock = actorClass.getSkill(actor, ESM::Skill::Block);
                        if (actorClass.hasInventoryStore(actor))
                        {
                            const MWWorld::InventoryStore& inventory = actorClass.getInventoryStore(actor);
                            const auto left = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedLeft);
                            in.mShield = left != inventory.end() && left->getType() == ESM::Armor::sRecordId;
                        }
                        in.mAgility = actorStats.getAttribute(ESM::Attribute::Agility).getModified();
                        in.mLuck = actorStats.getAttribute(ESM::Attribute::Luck).getModified();
                        in.mSpeed = actorStats.getAttribute(ESM::Attribute::Speed).getModified();
                        in.mWillpower = actorStats.getAttribute(ESM::Attribute::Willpower).getModified();
                        in.mFight = actorStats.getAiSetting(AiSetting::Fight).getModified();
                        in.mFlee = actorStats.getAiSetting(AiSetting::Flee).getModified();
                        if (target.getClass().isActor())
                            in.mLevelDifference = static_cast<float>(
                                actorStats.getLevel() - target.getClass().getCreatureStats(target).getLevel());
                        in.mFatigueTerm = actorStats.getFatigueTerm();
                        const RagdollCore::DuelStats terms = RagdollCore::duelStats(in);
                        self.mAttackSkill = terms.mAttack;
                        self.mGuardSkill = terms.mGuard;
                        self.mNerve = terms.mNerve;
                        self.mFootwork = terms.mFootwork;
                        const auto& health = actorStats.getHealth();
                        self.mHealth = health.getModified() > 0.0f
                            ? std::clamp(health.getCurrent() / health.getModified(), 0.0f, 1.0f)
                            : 1.0f;
                        // AI-02 item 3: when it tires and rests.
                        self.mWillpower = std::clamp(in.mWillpower / 100.0f, 0.0f, 1.0f);
                        self.mEndurance = std::clamp(
                            actorStats.getAttribute(ESM::Attribute::Endurance).getModified() / 100.0f, 0.0f, 1.0f);
                    }
                    // AI-02 item 5: the parry it just won (filled by the physics).
                    self.mSinceParryWon = selfBody->mSinceParryWon;
                    // AI-02 item 1a: seconds in the wheel's recover stage.
                    storage.mDuelWheelRecover
                        = selfBody->mWheelStage == 3 ? storage.mDuelWheelRecover + duration : 0.0f;
                    RagdollCore::DuelOpponentView opponent;
                    opponent.mDistance = distance;
                    if (const auto targetBody = getRagdollDuelView(target))
                    {
                        opponent.mReach = bodyReach(*targetBody);
                        opponent.mSwinging = targetBody->mTipSpeed > 300.0f;
                        opponent.mSinceSwing = targetBody->mSinceFast;
                        opponent.mSinceBounce = targetBody->mSinceBounce;
                        opponent.mOffBalance = targetBody->mOffBalance;
                        opponent.mDown = targetBody->mDown;
                        opponent.mWindingUp = targetBody->mWindingUp;
                        // Facing each other, its right is this fighter's left.
                        opponent.mBladeSide = -targetBody->mTipSide;
                        opponent.mCannotBlock = targetBody->mGuardBroken || targetBody->mStaggered;
                    }
                    else
                    {
                        const auto& targetStats = target.getClass().getCreatureStats(target);
                        opponent.mSwinging = targetStats.getAttackingOrSpell();
                        opponent.mDown = targetStats.getKnockedDown();
                    }
                    // AI-02 item 3: the target's fatigue share, as its own.
                    {
                        const auto& targetFatigue = target.getClass().getCreatureStats(target).getFatigue();
                        opponent.mFatigue = targetFatigue.getModified() > 0.0f
                            ? std::clamp(targetFatigue.getCurrent() / targetFatigue.getModified(), 0.0f, 1.0f)
                            : 1.0f;
                    }
                    // AI-02 item 2: the target's own speed toward this fighter
                    // (u/s, smoothed over ~0.1 s), its position differenced per frame.
                    {
                        const osg::Vec3f selfNow(actor.getRefData().getPosition().asVec3());
                        const osg::Vec3f targetNow(target.getRefData().getPosition().asVec3());
                        osg::Vec3f toSelf = selfNow - targetNow;
                        toSelf.z() = 0.0f;
                        const float gap = toSelf.normalize();
                        // Another target, or a jump (a teleport, a restage): no
                        // speed from it (review 10-08: a false charge read).
                        const bool sameTarget = storage.mDuelTargetRef == static_cast<const void*>(target.mRef);
                        const bool jumped = (targetNow - storage.mDuelTargetLast).length() > 50.0f;
                        if (wasDuelling && sameTarget && storage.mDuelTargetLastValid && !jumped && duration > 0.0f
                            && gap > 0.0f)
                        {
                            const float closing = std::clamp(
                                ((targetNow - storage.mDuelTargetLast) / duration) * toSelf, -600.0f, 600.0f);
                            storage.mDuelClosing += (closing - storage.mDuelClosing) * std::min(1.0f, duration / 0.1f);
                        }
                        else
                            storage.mDuelClosing = 0.0f;
                        storage.mDuelTargetRef = target.mRef;
                        storage.mDuelTargetLast = targetNow;
                        storage.mDuelTargetLastValid = true;
                        opponent.mClosing = storage.mDuelClosing;
                    }
                    const RagdollCore::DuelPhase before = storage.mDuel.mPhase;
                    // OPENMW_RAGDOLL_DUEL_CHASE = retreat speed (units/s) that starts
                    // a chase (0 = no chase, the stage-1 tactics; see
                    // DuelTacticsSettings::mRetreatSpeed for the default).
                    static const auto tactics_tuned = RagdollCore::tuned<RagdollCore::DuelTacticsSettings>([] {
                        RagdollCore::DuelTacticsSettings value;
                        if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DUEL_CHASE"))
                            value.mRetreatSpeed = static_cast<float>(std::atof(text));
                        // OPENMW_RAGDOLL_DUEL_CIRCLE = sideways share while
                        // measuring (0 = no footwork).
                        // OPENMW_RAGDOLL_DUEL_CIRCLE="share,lock gap".
                        if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DUEL_CIRCLE"))
                            std::sscanf(text, "%f,%f", &value.mCircle, &value.mLockGap);
                        // OPENMW_RAGDOLL_DUEL_DODGE_SIDE = sideways share of a
                        // dodge (0 = straight back, the old give-ground).
                        if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DUEL_DODGE_SIDE"))
                            value.mDodgeSide = static_cast<float>(std::atof(text));
                        // OPENMW_RAGDOLL_DUEL_DODGE_READ = units past reach within
                        // which a read wind-up is dodged with a committed step
                        // (DuelTacticsSettings::mDodgeRead; 0 = off, the old read
                        // of the cut). Needs OPENMW_RAGDOLL_DODGE for the step.
                        if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DUEL_DODGE_READ"))
                            value.mDodgeRead = std::max(0.0f, static_cast<float>(std::atof(text)));
                        // OPENMW_RAGDOLL_DUEL_COMPOSED = "patience scale, back-off
                        // chance at skill 0, at skill 1" (build 59 bookmarks 4-5;
                        // off: 1,0,0).
                        if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_DUEL_COMPOSED"))
                            std::sscanf(text, "%f,%f,%f", &value.mPatienceScale, &value.mBackOffUnskilled,
                                &value.mBackOffSkilled);
                        // AI-02 (docs/ragdoll/designs/AI-02-smart-duel-ai.md), each item
                        // off when unset (or "0").
                        const auto aiKnob = [](const char* name) -> const char* {
                            const char* text = RagdollCore::tuningText(name);
                            return text && std::string_view(text) != "0" ? text : nullptr;
                        };
                        // OPENMW_RAGDOLL_AI_AIM = "lead s, hold share, depth, grace s" (item 1a,
                        // start 0.45,1,30,0.3).
                        if (const char* text = aiKnob("OPENMW_RAGDOLL_AI_AIM"))
                            value.mAim = std::sscanf(text, "%f,%f,%f,%f", &value.mAimLead, &value.mAimHoldShare,
                                             &value.mAimDepth, &value.mAimGrace)
                                >= 1;
                        // OPENMW_RAGDOLL_AI_THREAT = "1" or "charge speed u/s, threat min"
                        // (item 2; 1 = 180, 0.35).
                        if (const char* text = aiKnob("OPENMW_RAGDOLL_AI_THREAT"))
                        {
                            value.mThreat = true;
                            float speed = 0.0f, least = value.mThreatMin;
                            if (std::sscanf(text, "%f,%f", &speed, &least) >= 1 && speed > 1.0f)
                                value.mChargeSpeed = speed;
                            value.mThreatMin = least;
                        }
                        // OPENMW_RAGDOLL_AI_STAMINA = "pace, exploit below" (item 3, start 1.5,0.3).
                        if (const char* text = aiKnob("OPENMW_RAGDOLL_AI_STAMINA"))
                            value.mStamina
                                = std::sscanf(text, "%f,%f", &value.mStaminaPace, &value.mStaminaExploit) >= 1;
                        // OPENMW_RAGDOLL_AI_RIPOSTE = "window s, chance A0, A100, wind share"
                        // (item 5, start 0.6,0.3,0.9,0.5; window 0 = off).
                        if (const char* text = aiKnob("OPENMW_RAGDOLL_AI_RIPOSTE"))
                            std::sscanf(text, "%f,%f,%f,%f", &value.mRiposteWindow, &value.mRiposteChance0,
                                &value.mRiposteChance100, &value.mRiposteWindShare);
                        return value;
                    });
                    static const auto& tactics = tactics_tuned.get();
                    // AI-02 item 1a (AI_AIM): its own attack lasts as long as the
                    // wheel's wind-up and cut (and the recover's first 0.2 s), not
                    // only the animation's attack (NPCHIT-01 10-07: that ended first,
                    // so it stepped out before the cut started).
                    if (tactics.mAim)
                    {
                        const int stage = selfBody->mWheelStage;
                        storage.mDuelWheelTime = stage == 1 || stage == 2 ? storage.mDuelWheelTime + duration : 0.0f;
                        // At most 1.5 s: a stalled wind-up must not hold the
                        // step-in for ever (review 10-08).
                        self.mAttacking = self.mAttacking || ((stage == 1 || stage == 2) && storage.mDuelWheelTime < 1.5f)
                            || (stage == 3 && storage.mDuelWheelRecover < 0.2f);
                        self.mStrikeLead = tactics.mAimLead;
                    }
                    storage.mDuelSinceDown = selfBody->mDown ? 0.0f : storage.mDuelSinceDown + duration;
                    self.mSinceDown = storage.mDuelSinceDown;
                    const auto out = RagdollCore::stepDuelTactics(storage.mDuel, self, opponent, duration, tactics);
                    storage.stopCombatMove();
                    storage.mShouldApproach = false;
                    storage.mMovement.mPosition[0] = out.mStrafe;
                    bool crowdHold = false;
                    if (crowdEnabled())
                    {
                        // CROWD: room round the target; no cut through an ally.
                        const auto crowd = stepCrowd(actor, target);
                        if (crowd.mHasOthers)
                        {
                            storage.mMovement.mPosition[0]
                                = std::clamp(out.mStrafe + crowd.mStrafe, -1.0f, 1.0f);
                            crowdHold = crowd.mAllyInLine;
                            if (crowdHold && std::abs(crowd.mStrafe) < 0.3f)
                                storage.mMovement.mPosition[0] = actor.mRef < target.mRef ? 1.0f : -1.0f;
                        }
                    }
                    storage.mMovement.mPosition[1] = out.mForward;
                    // A read dodge's committed step (AI dodges, DODGE_READ).
                    if (out.mDodgeBurst)
                    {
                        const osg::Vec3f toTarget = target.getRefData().getPosition().asVec3()
                            - actor.getRefData().getPosition().asVec3();
                        osg::Vec3f away(-toTarget.x(), -toTarget.y(), 0.0f);
                        away.normalize();
                        const osg::Vec3f right(-away.y(), away.x(), 0.0f); // the fighter's right, facing the target
                        osg::Vec3f direction = right * out.mDodgeSideways + away * out.mDodgeBack;
                        direction.normalize();
                        MWBase::Environment::get().getWorld()->requestActorRagdollDodge(actor, direction);
                    }
                    storage.mReadyToAttack = out.mAttack && !crowdHold;
                    storage.mDuelRun = out.mRun;
                    // AI-02 item 2: a backpedal from a charge raises the guard.
                    storage.mDuelGuardWanted = out.mGuard;
                    if (out.mGuard)
                        requestRagdollAiGuard(actor, 0.3f);
                    // AI-02 item 5: the vanilla 0.1-1.0 s attack cooldown
                    // (updateAttack) would eat the riposte window.
                    if (out.mReason == "riposte")
                        storage.mAttackCooldown = 0.0f;
                    // ... and only a decided riposte winds up short (review 10-08:
                    // every attack in the window did, also after a "no").
                    if (out.mRiposte)
                        requestRagdollAiRiposte(actor);
                    actorStats.setMovementFlag(CreatureStats::Flag_Run, out.mRun);
                    const osg::Vec3f actorPos(actor.getRefData().getPosition().asVec3());
                    const osg::Vec3f targetPos(target.getRefData().getPosition().asVec3());
                    storage.mMovement.mRotation[2] = getZAngleToDir(targetPos - actorPos);
                    updateActorsMovement(actor, duration, storage);
                    static const auto trace_tuned = RagdollCore::tuned<bool>([] { return RagdollCore::tuningText("OPENMW_RAGDOLL_DAMAGE_TRACE") != nullptr; });
                    static const auto& trace = trace_tuned.get();
                    // Dodges are logged too (they keep the phase).
                    const bool dodging = out.mReason == "side-step" || out.mReason == "give ground";
                    if (trace && (out.mPhase != before || !wasDuelling || (dodging && !storage.mDuelDodgeLogged)))
                        Log(Debug::Info) << "RAGDOLL_DUEL actor=" << actor.getCellRef().getRefId()
                                         << " phase=" << RagdollCore::duelPhaseName(out.mPhase)
                                         << " reason=" << out.mReason << " distance=" << static_cast<int>(distance)
                                         << " fatigue=" << self.mFatigue << " closing=" << opponent.mClosing;
                    storage.mDuelDodgeLogged = dodging;
                    // AI-02 trace: one line per decision (OPENMW_RAGDOLL_AI_TRACE=1).
                    static const auto ai2Trace_tuned = RagdollCore::tuned<bool>([] {
                        const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_AI_TRACE");
                        return text && std::atoi(text) != 0;
                    });
                    static const auto& ai2Trace = ai2Trace_tuned.get();
                    if (ai2Trace)
                    {
                        if (!out.mThreatRead.empty())
                            Log(Debug::Info) << "RAGDOLL_AI2 actor=" << actor.getCellRef().getRefId()
                                             << " what=threat T=" << out.mThreat << " N=" << self.mNerve
                                             << " response=" << out.mThreatRead
                                             << " closing=" << static_cast<int>(opponent.mClosing)
                                             << " distance=" << static_cast<int>(distance);
                        if (out.mPaceRead)
                            Log(Debug::Info) << "RAGDOLL_AI2 actor=" << actor.getCellRef().getRefId()
                                             << " what=pace fatigue=" << self.mFatigue
                                             << " patience=" << storage.mDuel.mPatience
                                             << " foe=" << opponent.mFatigue;
                        if (!out.mRiposteRead.empty())
                            Log(Debug::Info) << "RAGDOLL_AI2 actor=" << actor.getCellRef().getRefId()
                                             << " what=riposte chosen=" << out.mRiposteRead
                                             << " chance=" << out.mRiposteChance << " since=" << self.mSinceParryWon
                                             << " distance=" << static_cast<int>(distance);
                    }
                }
            }

            storage.updateCombatMove(duration);
            storage.mRotateMove = false;
            if (storage.mReadyToAttack)
            {
                MWBase::World* world = MWBase::Environment::get().getWorld();
                const osg::Vec3f actorPos(actor.getRefData().getPosition().asVec3());
                const osg::Vec3f targetPos(target.getRefData().getPosition().asVec3());
                const osg::Vec3f targetRelativePos = world->aimToTarget(actor, target, isRangedCombat);
                storage.mMovement.mRotation[0] = getXAngleToDir(targetRelativePos);
                // using targetRelativePos results in spastic movements since the head is animated
                storage.mMovement.mRotation[2] = getZAngleToDir(targetPos - actorPos);
                updateActorsMovement(actor, duration, storage);
                if (storage.mRotateMove)
                    return false;
            }

            storage.updateAttack(actor, characterController, weapon, isRangedCombat, duration);
        }
        else
        {
            updateFleeing(actor, target, duration, characterController.getSupportedMovementDirections(), storage);
        }
        storage.mActionCooldown -= duration;

        if (storage.mReaction.update(duration) == Misc::TimerStatus::Waiting)
            return false;

        return attack(actor, target, storage, characterController);
    }

    bool AiCombat::attack(const MWWorld::Ptr& actor, const MWWorld::Ptr& target, AiCombatStorage& storage,
        CharacterController& characterController)
    {
        const MWWorld::CellStore*& currentCell = storage.mCell;
        bool cellChange = currentCell && (actor.getCell() != currentCell);
        if (!currentCell || cellChange)
        {
            currentCell = actor.getCell();
        }

        const MWWorld::Class& actorClass = actor.getClass();
        MWMechanics::CreatureStats& stats = actorClass.getCreatureStats(actor);

        bool forceFlee = false;
        if (!canFight(actor, target))
        {
            storage.stopAttack();
            stats.setAttackingOrSpell(false);
            storage.mActionCooldown = 0.f;
            // Continue combat if target is player or player follower/escorter and an attack has been attempted
            const auto& playerFollowersAndEscorters
                = MWBase::Environment::get().getMechanicsManager()->getActorsSidingWith(MWMechanics::getPlayer());
            bool targetSidesWithPlayer
                = (std::find(playerFollowersAndEscorters.begin(), playerFollowersAndEscorters.end(), target)
                    != playerFollowersAndEscorters.end());
            if ((target == MWMechanics::getPlayer() || targetSidesWithPlayer)
                && (hitAttemptMatchesTarget(actor, target) || hitAttemptMatchesTarget(target, actor)))
                forceFlee = true;
            else // Otherwise end combat
                return true;
        }

        // Duel tactics walk at measure and run only to close a long gap.
        stats.setMovementFlag(CreatureStats::Flag_Run, !storage.mDuelActive || storage.mDuelRun);
        float& actionCooldown = storage.mActionCooldown;
        std::unique_ptr<Action>& currentAction = storage.mCurrentAction;

        if (!forceFlee)
        {
            if (actionCooldown > 0)
                return false;

            if (characterController.readyToPrepareAttack())
            {
                currentAction = prepareNextAction(actor, target);
                actionCooldown = currentAction->getActionCooldown();
            }
        }
        else
        {
            currentAction = std::make_unique<ActionFlee>();
            actionCooldown = currentAction->getActionCooldown();
        }

        if (!currentAction)
            return false;

        if (storage.isFleeing() != currentAction->isFleeing())
        {
            if (currentAction->isFleeing())
            {
                storage.startFleeing();
                MWBase::Environment::get().getDialogueManager()->say(actor, ESM::RefId::stringRefId("flee"));
                return false;
            }
            else
                storage.stopFleeing();
        }

        bool isRangedCombat = false;
        float& rangeAttack = storage.mAttackRange;
        rangeAttack = currentAction->getCombatRange(isRangedCombat);
        // A powered ragdoll fights with physical blows: it must close to the
        // body's reach, otherwise the animation hit path lands contactless
        // blows from the authored combat distance. Fists are shorter still.
        if (!isRangedCombat && MWBase::Environment::get().getWorld()->isPoweredRagdoll(actor))
            rangeAttack = std::min(rangeAttack,
                currentAction->getWeapon() == nullptr ? cRagdollMeleeUnarmedReach : cRagdollMeleePhysicalReach);

        ESM::Position pos = actor.getRefData().getPosition();
        const osg::Vec3f vActorPos(pos.asVec3());
        const osg::Vec3f vTargetPos(target.getRefData().getPosition().asVec3());

        float distToTarget = getDistanceToBounds(actor, target);

        storage.mReadyToAttack = (currentAction->isAttackingOrSpell() && distToTarget <= rangeAttack && storage.mLOS);

        if (storage.mReadyToAttack)
        {
            storage.startCombatMove(isRangedCombat, distToTarget, rangeAttack, actor, target);
        }
        else
        {
            storage.mShouldApproach = true;
        }

        // If actor uses custom destination it has to try to rebuild path because environment can change
        // (door is opened between actor and target) or target position has changed and current custom destination
        // is not good enough to attack target.
        if (storage.mCurrentAction->isAttackingOrSpell()
            && ((!storage.mReadyToAttack && !mPathFinder.isPathConstructed())
                || (storage.mUseCustomDestination && (storage.mCustomDestination - vTargetPos).length() > rangeAttack)))
        {
            const MWBase::World* world = MWBase::Environment::get().getWorld();
            // Try to build path to the target.
            const auto agentBounds = world->getPathfindingAgentBounds(actor);
            const DetourNavigator::Flags navigatorFlags = getNavigatorFlags(actor);
            const DetourNavigator::AreaCosts areaCosts = getAreaCosts(actor, navigatorFlags);
            const ESM::Pathgrid* pathgrid = world->getStore().get<ESM::Pathgrid>().search(*actor.getCell()->getCell());
            const auto& pathGridGraph = getPathGridGraph(pathgrid);
            mPathFinder.buildPath(actor, vActorPos, vTargetPos, pathGridGraph, agentBounds, navigatorFlags, areaCosts,
                storage.mAttackRange, PathType::Full);

            if (!mPathFinder.isPathConstructed())
            {
                // If there is no path, try to find a point on a line from the actor position to target projected
                // on navmesh to attack the target from there.
                const auto navigator = world->getNavigator();
                const auto hit
                    = DetourNavigator::raycast(*navigator, agentBounds, vActorPos, vTargetPos, navigatorFlags);

                if (hit.has_value() && (*hit - vTargetPos).length() <= rangeAttack)
                {
                    // If the point is close enough, try to find a path to that point.
                    mPathFinder.buildPath(actor, vActorPos, *hit, pathGridGraph, agentBounds, navigatorFlags, areaCosts,
                        storage.mAttackRange, PathType::Full);
                    if (mPathFinder.isPathConstructed())
                    {
                        // If path to that point is found use it as custom destination.
                        storage.mCustomDestination = *hit;
                        storage.mUseCustomDestination = true;
                    }
                }

                if (!mPathFinder.isPathConstructed())
                {
                    storage.mUseCustomDestination = false;
                    storage.stopAttack();
                    stats.setAttackingOrSpell(false);
                    currentAction = std::make_unique<ActionFlee>();
                    actionCooldown = currentAction->getActionCooldown();
                    storage.startFleeing();
                    MWBase::Environment::get().getDialogueManager()->say(actor, ESM::RefId::stringRefId("flee"));
                }
            }
            else
            {
                storage.mUseCustomDestination = false;
            }
        }

        return false;
    }

    void MWMechanics::AiCombat::updateLOS(
        const MWWorld::Ptr& actor, const MWWorld::Ptr& target, float duration, MWMechanics::AiCombatStorage& storage)
    {
        const float losUpdateDuration = 0.5f;
        if (storage.mUpdateLOSTimer <= 0.f)
        {
            storage.mLOS = MWBase::Environment::get().getWorld()->getLOS(actor, target);
            storage.mUpdateLOSTimer = losUpdateDuration;
        }
        else
            storage.mUpdateLOSTimer -= duration;
    }

    void MWMechanics::AiCombat::updateFleeing(const MWWorld::Ptr& actor, const MWWorld::Ptr& target, float duration,
        MWWorld::MovementDirectionFlags supportedMovementDirections, AiCombatStorage& storage)
    {
        const float blindRunDuration = 1.0f;

        updateLOS(actor, target, duration, storage);

        AiCombatStorage::FleeState& state = storage.mFleeState;
        switch (state)
        {
            case AiCombatStorage::FleeState_None:
                return;

            case AiCombatStorage::FleeState_Idle:
            {
                float triggerDist = getMaxAttackDistance(target);
                const MWWorld::Cell* cellVariant = storage.mCell->getCell();
                if (storage.mLOS && (triggerDist >= 1000 || getDistanceMinusHalfExtents(actor, target) <= triggerDist))
                {
                    const ESM::Pathgrid* pathgrid
                        = MWBase::Environment::get().getESMStore()->get<ESM::Pathgrid>().search(*cellVariant);

                    bool runFallback = true;

                    if (pathgrid != nullptr && !pathgrid->mPoints.empty()
                        && !actor.getClass().isPureWaterCreature(actor))
                    {
                        ESM::Pathgrid::PointList points;
                        const Misc::CoordinateConverter coords
                            = Misc::makeCoordinateConverter(*storage.mCell->getCell());

                        osg::Vec3f localPos = actor.getRefData().getPosition().asVec3();
                        coords.toLocal(localPos);

                        const std::size_t closestPointIndex = Misc::getClosestPoint(*pathgrid, localPos);
                        for (std::size_t i = 0; i < pathgrid->mPoints.size(); i++)
                        {
                            if (i != closestPointIndex
                                && getPathGridGraph(pathgrid).isPointConnected(closestPointIndex, i))
                            {
                                points.push_back(pathgrid->mPoints[i]);
                            }
                        }

                        if (!points.empty())
                        {
                            auto& prng = MWBase::Environment::get().getWorld()->getPrng();
                            ESM::Pathgrid::Point dest = points[Misc::Rng::rollDice(points.size(), prng)];
                            coords.toWorld(dest);

                            state = AiCombatStorage::FleeState_RunToDestination;
                            storage.mFleeDest = ESM::Pathgrid::Point(dest.mX, dest.mY, dest.mZ);

                            runFallback = false;
                        }
                    }

                    if (runFallback)
                    {
                        state = AiCombatStorage::FleeState_RunBlindly;
                        storage.mFleeBlindRunTimer = 0.0f;
                    }
                }
            }
            break;

            case AiCombatStorage::FleeState_RunBlindly:
            {
                // timer to prevent twitchy movement that can be observed in vanilla MW
                if (storage.mFleeBlindRunTimer < blindRunDuration)
                {
                    storage.mFleeBlindRunTimer += duration;

                    storage.mMovement.mRotation[0] = -actor.getRefData().getPosition().rot[0];
                    storage.mMovement.mRotation[2] = osg::PIf
                        + getZAngleToDir(
                            target.getRefData().getPosition().asVec3() - actor.getRefData().getPosition().asVec3());
                    storage.mMovement.mPosition[1] = 1;
                    updateActorsMovement(actor, duration, storage);
                }
                else
                    state = AiCombatStorage::FleeState_Idle;
            }
            break;

            case AiCombatStorage::FleeState_RunToDestination:
            {
                static const float fFleeDistance = MWBase::Environment::get()
                                                       .getESMStore()
                                                       ->get<ESM::GameSetting>()
                                                       .find("fFleeDistance")
                                                       ->mValue.getFloat();

                float dist
                    = (actor.getRefData().getPosition().asVec3() - target.getRefData().getPosition().asVec3()).length();
                if ((dist > fFleeDistance && !storage.mLOS)
                    || pathTo(
                        actor, Misc::Convert::makeOsgVec3f(storage.mFleeDest), duration, supportedMovementDirections))
                {
                    state = AiCombatStorage::FleeState_Idle;
                }
            }
            break;
        };
    }

    void AiCombat::updateActorsMovement(const MWWorld::Ptr& actor, float duration, AiCombatStorage& storage)
    {
        // apply combat movement
        float deltaAngle = storage.mMovement.mRotation[2] - actor.getRefData().getPosition().rot[2];
        osg::Vec2f movement = Misc::rotateVec2f(
            osg::Vec2f(storage.mMovement.mPosition[0], storage.mMovement.mPosition[1]), -deltaAngle);

        MWMechanics::Movement& actorMovementSettings = actor.getClass().getMovementSettings(actor);
        actorMovementSettings.mPosition[0] = movement.x();
        actorMovementSettings.mPosition[1] = movement.y();
        actorMovementSettings.mPosition[2] = storage.mMovement.mPosition[2];

        rotateActorOnAxis(actor, 2, actorMovementSettings, storage);
        rotateActorOnAxis(actor, 0, actorMovementSettings, storage);
    }

    void AiCombat::rotateActorOnAxis(
        const MWWorld::Ptr& actor, int axis, MWMechanics::Movement& actorMovementSettings, AiCombatStorage& storage)
    {
        actorMovementSettings.mRotation[axis] = 0;
        bool isRangedCombat = false;
        storage.mCurrentAction->getCombatRange(isRangedCombat);
        float eps = isRangedCombat ? osg::DegreesToRadians(0.5f) : osg::DegreesToRadians(3.f);
        float targetAngleRadians = storage.mMovement.mRotation[axis];
        storage.mRotateMove = !smoothTurn(actor, targetAngleRadians, axis, eps);
    }

    void AiCombat::writeState(ESM::AiSequence::AiSequence& sequence) const
    {
        auto combat = std::make_unique<ESM::AiSequence::AiCombat>();
        combat->mTargetActor = mTargetActor;

        ESM::AiSequence::AiPackageContainer package;
        package.mType = ESM::AiSequence::Ai_Combat;
        package.mPackage = std::move(combat);
        sequence.mPackages.push_back(std::move(package));
    }

    AiCombatStorage::AiCombatStorage()
        : mAttackCooldown(0.0f)
        , mReaction(MWBase::Environment::get().getWorld()->getPrng())
        , mTimerCombatMove(0.0f)
        , mReadyToAttack(false)
        , mAttack(false)
        , mAttackRange(0.0f)
        , mCombatMove(false)
        , mRotateMove(false)
        , mCell(nullptr)
        , mCurrentAction()
        , mActionCooldown(0.0f)
        , mStrength()
        , mForceNoShortcut(false)
        , mShortcutFailPos()
        , mMovement()
        , mFleeState(FleeState_None)
        , mLOS(false)
        , mUpdateLOSTimer(0.0f)
        , mFleeBlindRunTimer(0.0f)
        , mUseCustomDestination(false)
        , mCustomDestination()
    {
    }

    void AiCombatStorage::startCombatMove(bool isDistantCombat, float distToTarget, float rangeAttack,
        const MWWorld::Ptr& actor, const MWWorld::Ptr& target)
    {
        auto& prng = MWBase::Environment::get().getWorld()->getPrng();

        // get the range of the target's weapon
        MWWorld::Ptr targetWeapon = MWWorld::Ptr();
        const MWWorld::Class& targetClass = target.getClass();

        if (targetClass.hasInventoryStore(target))
        {
            ESM::RefId weaponTypeId;
            MWWorld::ContainerStoreIterator weaponSlot = MWMechanics::getActiveWeapon(target, &weaponTypeId);
            if (isWeaponType(weaponTypeId))
                targetWeapon = *weaponSlot;
        }

        bool targetUsesRanged = false;
        float rangeAttackOfTarget = ActionWeapon(targetWeapon).getCombatRange(targetUsesRanged);

        if (mMovement.mPosition[0] != 0.0f)
        {
            mTimerCombatMove = 0.1f + 0.1f * Misc::Rng::rollClosedProbability(prng);
            mCombatMove = true;
        }
        // dodge movements (for NPCs and bipedal creatures)
        // Note: do not use for ranged combat yet since in couple with back up behaviour can move actor out of cliff
        else if (actor.getClass().isBipedal(actor) && !isDistantCombat)
        {
            float moveDuration = 0;
            double angleToTarget
                = Misc::normalizeAngle(mMovement.mRotation[2] - actor.getRefData().getPosition().rot[2]);
            // Apply a big side step if enemy tries to get around and come from behind.
            // Otherwise apply a random side step (kind of dodging) with some probability
            // if actor is within range of target's weapon.
            if (std::abs(angleToTarget) > osg::PI / 4)
                moveDuration = 0.2f;
            else if (distToTarget <= rangeAttackOfTarget && Misc::Rng::rollClosedProbability(prng) < 0.25)
                moveDuration = 0.1f + 0.1f * Misc::Rng::rollClosedProbability(prng);
            if (moveDuration > 0)
            {
                mMovement.mPosition[0] = Misc::Rng::rollProbability(prng) < 0.5 ? 1.0f : -1.0f; // to the left/right
                mTimerCombatMove = moveDuration;
                mCombatMove = true;
            }
        }

        mMovement.mPosition[1] = 0;
        if (isDistantCombat)
        {
            // Backing up behaviour
            // Actor backs up slightly further away than opponent's weapon range
            // (in vanilla - only as far as opponent's weapon range),
            // or not at all if opponent is using a ranged weapon

            if (targetUsesRanged
                || distToTarget > rangeAttackOfTarget * 1.5) // Don't back up if the target is wielding ranged weapon
                return;

            // actor should not back up into water
            if (MWBase::Environment::get().getWorld()->isUnderwater(MWWorld::ConstPtr(actor), 0.5f))
                return;

            int mask = MWPhysics::Layers::WORLD | MWPhysics::Layers::HEIGHTMAP | MWPhysics::Layers::DOOR;

            // Actor can not back up if there is no free space behind
            // Currently we take the 35% of actor's height from the ground as vector height.
            // This approach allows us to detect small obstacles (e.g. crates) and curved walls.
            osg::Vec3f halfExtents = MWBase::Environment::get().getWorld()->getHalfExtents(actor);
            osg::Vec3f pos = actor.getRefData().getPosition().asVec3();
            osg::Vec3f source = pos + osg::Vec3f(0, 0, 0.75f * halfExtents.z());
            osg::Vec3f fallbackDirection = actor.getRefData().getBaseNode()->getAttitude() * osg::Vec3f(0, -1, 0);
            osg::Vec3f destination = source + fallbackDirection * (halfExtents.y() + 16);

            const auto* rayCasting = MWBase::Environment::get().getWorld()->getRayCasting();
            bool isObstacleDetected = rayCasting->castRay(source, destination, mask).mHit;
            if (isObstacleDetected)
                return;

            // Check if there is nothing behind - probably actor is near cliff.
            // A current approach: cast ray 1.5-yard ray down in 1.5 yard behind actor from 35% of actor's height.
            // If we did not hit anything, there is a cliff behind actor.
            source = pos + osg::Vec3f(0, 0, 0.75f * halfExtents.z()) + fallbackDirection * (halfExtents.y() + 96);
            destination = source - osg::Vec3f(0, 0, 0.75f * halfExtents.z() + 96);
            bool isCliffDetected = !rayCasting->castRay(source, destination, mask).mHit;
            if (isCliffDetected)
                return;

            mMovement.mPosition[1] = -1;
        }
    }

    void AiCombatStorage::updateCombatMove(float duration)
    {
        if (mCombatMove)
        {
            mTimerCombatMove -= duration;
            if (mTimerCombatMove <= 0)
            {
                stopCombatMove();
            }
        }
    }

    void AiCombatStorage::stopCombatMove()
    {
        mTimerCombatMove = 0;
        mMovement.mPosition[0] = 0;
        mCombatMove = false;
    }

    void AiCombatStorage::updateAttack(const MWWorld::Ptr& actor, CharacterController& characterController,
        const ESM::Weapon* weapon, bool distantCombat, float duration)
    {
        const MWWorld::Class& actorClass = actor.getClass();
        MWMechanics::CreatureStats& actorStats = actorClass.getCreatureStats(actor);

        if (mAttack)
        {
            float attackStrength = characterController.calculateWindUp();
            if (characterController.readyToPrepareAttack() || attackStrength >= mStrength || attackStrength == -1.f)
                mAttack = false;
        }
        else if (mReadyToAttack && characterController.readyToStartAttack())
        {
            if (mAttackCooldown <= 0)
            {
                mAttack = true; // attack starts just now

                if (!distantCombat)
                    characterController.setAIAttackType(chooseBestAttack(weapon));

                auto& prng = MWBase::Environment::get().getWorld()->getPrng();
                mStrength = Misc::Rng::rollClosedProbability(prng);

                const MWWorld::ESMStore& store = *MWBase::Environment::get().getESMStore();

                bool canShout = true;
                ESM::RefId spellId = mCurrentAction->getSpell();
                if (!spellId.empty())
                {
                    const ESM::Spell* spell = store.get<ESM::Spell>().find(spellId);
                    if (spell->mEffects.mList.empty() || spell->mEffects.mList[0].mData.mRange != ESM::RT_Target)
                        canShout = false;
                }

                if (canShout)
                {
                    // Say a provoking combat phrase
                    const int iVoiceAttackOdds
                        = store.get<ESM::GameSetting>().find("iVoiceAttackOdds")->mValue.getInteger();
                    if (Misc::Rng::roll0to99(prng) < iVoiceAttackOdds)
                    {
                        MWBase::Environment::get().getDialogueManager()->say(actor, ESM::RefId::stringRefId("attack"));
                    }
                }
                if (mCurrentAction->getActionCooldown() > 0.f)
                {
                    mAttackCooldown = mCurrentAction->getActionCooldown();
                }
                else
                {
                    std::string_view delayGmst = actor.getClass().isNpc() ? "fCombatDelayNPC" : "fCombatDelayCreature";
                    const float baseDelay = store.get<ESM::GameSetting>().find(delayGmst)->mValue.getFloat();
                    mAttackCooldown = std::min(baseDelay + 0.01f * Misc::Rng::roll0to99(prng), baseDelay + 0.9f);
                }
            }
            else
                mAttackCooldown -= duration;
        }

        actorStats.setAttackingOrSpell(mAttack);
    }

    void AiCombatStorage::stopAttack()
    {
        mMovement.mPosition[0] = 0;
        mMovement.mPosition[1] = 0;
        mMovement.mPosition[2] = 0;
        mReadyToAttack = false;
        mAttack = false;
    }

    void AiCombatStorage::startFleeing()
    {
        stopFleeing();
        mFleeState = FleeState_Idle;
    }

    void AiCombatStorage::stopFleeing()
    {
        mMovement.mPosition[0] = 0;
        mMovement.mPosition[1] = 0;
        mMovement.mPosition[2] = 0;
        mFleeState = FleeState_None;
        mFleeDest = ESM::Pathgrid::Point(0, 0, 0);
    }

    bool AiCombatStorage::isFleeing() const
    {
        return mFleeState != FleeState_None;
    }
}

namespace
{

    std::string_view chooseBestAttack(const ESM::Weapon* weapon)
    {
        if (weapon != nullptr)
        {
            // Only the attack types this weapon class can actually perform,
            // weighted by the damage each deals. A dagger slashes and stabs;
            // it never plays a chop.
            const MWMechanics::WeaponAttackCandidates candidates
                = MWMechanics::weaponAttackCandidates(weapon);
            int slash = candidates.mSlash ? (weapon->mData.mSlash[0] + weapon->mData.mSlash[1]) / 2 : 0;
            int chop = candidates.mChop ? (weapon->mData.mChop[0] + weapon->mData.mChop[1]) / 2 : 0;
            int thrust = candidates.mThrust ? (weapon->mData.mThrust[0] + weapon->mData.mThrust[1]) / 2 : 0;
            const int total = slash + chop + thrust;
            if (total <= 0)
                return "thrust";

            auto& prng = MWBase::Environment::get().getWorld()->getPrng();
            const float roll = Misc::Rng::rollClosedProbability(prng) * static_cast<float>(total);
            if (slash > 0 && roll <= static_cast<float>(slash))
                return "slash";
            if (thrust > 0 && roll <= static_cast<float>(slash + thrust))
                return "thrust";
            if (chop > 0)
                return "chop";
            return thrust > 0 ? "thrust" : "slash";
        }
        // Unarmed: hooks and straights only, never a chop.
        return MWMechanics::CharacterController::getRandomHandToHandAttackType();
    }

}

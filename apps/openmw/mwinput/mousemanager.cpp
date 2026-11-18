#include "mousemanager.hpp"
#include <ragdollcore/tuning.hpp>
#include <ragdollcore/palmparry.hpp>
#include "../manualdiagnostics.hpp"

#include "actions.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <components/debug/debuglog.hpp>

#include <MyGUI_Button.h>
#include <MyGUI_InputManager.h>
#include <MyGUI_RenderManager.h>

#include <components/sdlutil/sdlinputwrapper.hpp>
#include <components/sdlutil/sdlmappings.hpp>
#include <components/settings/values.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/inputmanager.hpp"
#include "../mwbase/luamanager.hpp"
#include "../mwbase/windowmanager.hpp"
#include "../mwbase/world.hpp"

#include "../mwgui/settingswindow.hpp"

#include "../mwworld/player.hpp"
#include "../mwworld/class.hpp"
#include "../mwworld/inventorystore.hpp"
#include "../mwmechanics/creaturestats.hpp"
#include "../mwmechanics/drawstate.hpp"

#include "actions.hpp"
#include "bindingsmanager.hpp"

namespace MWInput
{
    namespace
    {
        // KICK-01: the middle button kicks while the player's body is
        // powered (it was Physical thrust's default; user 11-03: "kick
        // replaces it"). OPENMW_RAGDOLL_KICK's first value (0 = off).
        bool kickEnabled()
        {
            static const auto on_tuned = RagdollCore::tuned<bool>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_KICK");
                return text && std::atof(text) > 0.0;
            });
            static const auto& on = on_tuned.get();
            return on;
        }

        // K2_HOLD_AIM (registry): an undecided attack press keeps the mouse
        // off the view, and RagdollTapDrag::sDragCounts of travel make it a
        // drag at once, its wind-up from the way the mouse went (0 = the view
        // turns for the press's first 0.16 s, as before).
        bool holdAimEnabled()
        {
            static const auto on_tuned = RagdollCore::tuned<bool>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_K2_HOLD_AIM");
                return text && std::atof(text) > 0.0;
            });
            return on_tuned.get();
        }

        // K2_MIDDLE_PARRY (registry): what the middle button does. 0 = nothing
        // here (it is kept for the left-hand spell; the kick is its own
        // action, A_PhysicalKick); 1 = it guards and parries as the right one
        // does, without the shove; 2 = it kicks.
        int middleButtonMode()
        {
            static const auto mode_tuned = RagdollCore::tuned<int>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_K2_MIDDLE_PARRY");
                return text ? std::clamp(std::atoi(text), 0, 2) : 0;
            });
            return mode_tuned.get();
        }
        bool middleParryEnabled()
        {
            return middleButtonMode() == 1;
        }

        // K2_TAP_PARRY_AIM (registry): a tapped parry's zone takes the mouse
        // until its window ends, not only while the button is down (a tap is
        // under 0.2 s: the flick that aims it mostly comes after the release).
        // Its second value, flick counts: for a press's first 0.35 s (the
        // tap's window) this much mouse travel picks a zone, where the guard
        // asks 150 (PassiveRagdollEntry::sGuardZoneDeadCounts) - of the
        // user's 33 single parry taps on 11-11/12, 21 stayed under 150 in
        // the window and 12 under 60. 0 or 150 = the guard's 150.
        const std::array<float, 2>& tapParryAimSettings()
        {
            static const auto value_tuned = RagdollCore::tuned<std::array<float, 2>>([] {
                std::array<float, 2> value{ 0.0f, 0.0f };
                if (const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_K2_TAP_PARRY_AIM"))
                    std::sscanf(text, "%f,%f", &value[0], &value[1]);
                value[0] = std::isfinite(value[0]) && value[0] != 0.0f ? 1.0f : 0.0f;
                value[1] = std::isfinite(value[1]) ? std::clamp(value[1], 0.0f, 150.0f) : 0.0f;
                return value;
            });
            return value_tuned.get();
        }
        bool tapParryAimEnabled()
        {
            return tapParryAimSettings()[0] > 0.0f;
        }
        // The mouse's gain on the guard's zone `sincePress` s into a press.
        float guardZoneGain(float sincePress)
        {
            const float flick = tapParryAimSettings()[1];
            return tapParryAimEnabled() && flick >= 10.0f && sincePress < 0.35f ? 150.0f / flick : 1.0f;
        }

        // K2_FIST_STEER (registry): FIST-06's steered fists - with bare fists
        // the right button is the left fist, both buttons the guard.
        // Its mode: 1 = the steered fists (a held button gives the mouse to
        // the fist), 2 = FIST-07 (the movement keys pick the punch; the mouse
        // stays the view's).
        int fistSteerMode()
        {
            static const auto mode_tuned = RagdollCore::tuned<int>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_K2_FIST_STEER");
                const double value = text ? std::atof(text) : 0.0;
                // 4 (FIST-12): a held button gives the mouse to the fist, as 1.
                return value >= 4.0 ? 1 : value >= 2.0 ? 2 : value > 0.0 ? 1 : 0;
            });
            return mode_tuned.get();
        }
        bool fistSteerEnabled()
        {
            return fistSteerMode() > 0;
        }

        // K2_PALM_TAP (registry): with bare fists a tap of both buttons is a
        // timed palm parry (the guard stays up for its window, counted from
        // the press) and the shove is the second press of both. Window 0 = a
        // tap of both shoves, as FIST-06 had it.
        const RagdollCore::PalmTapSettings& palmTapSettings()
        {
            static const auto value_tuned = RagdollCore::tuned<RagdollCore::PalmTapSettings>([] {
                return RagdollCore::parsePalmTapSettings(RagdollCore::tuningText("OPENMW_RAGDOLL_K2_PALM_TAP"));
            });
            return value_tuned.get();
        }

        bool rmbGuardEnabled()
        {
            static const auto on_tuned = RagdollCore::tuned<bool>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_RMB_GUARD");
                return text && std::atoi(text) != 0;
            });
            static const auto& on = on_tuned.get();
            return on;
        }

        // GUARD-03 (user 10-31: "when guard button is clicked and then the
        // attack button is clicked at the same time the guard stops and
        // immediately goes into attack/drag mode"): an attack press while the
        // guard is up (held, or a tap's parry window) lowers the guard and the
        // press goes to the attack - a click swings, a hold drags on the cut
        // wheel. The guard stays down until its button is let go. Before, the
        // guard stayed up and the attack path ignored the click under it (the
        // block owns the arm; a click attack starts only with no guard).
        // OPENMW_RAGDOLL_GUARD_TO_ATTACK=0 = the guard wins as before.
        bool guardToAttackEnabled()
        {
            static const auto on_tuned = RagdollCore::tuned<bool>([] {
                const char* text = RagdollCore::tuningText("OPENMW_RAGDOLL_GUARD_TO_ATTACK");
                return !(text && std::atoi(text) == 0);
            });
            static const auto& on = on_tuned.get();
            return on;
        }
    }

    namespace
    {
        MWWorld::Ptr playerWeapon(MWBase::World& world)
        {
            const auto player = world.getPlayerPtr();
            auto& inventory = player.getClass().getInventoryStore(player);
            const auto equipped = inventory.getSlot(MWWorld::InventoryStore::Slot_CarriedRight);
            return equipped == inventory.end() ? MWWorld::Ptr{} : *equipped;
        }

        // FIST-06: the player's powered body has its bare fists up.
        bool steeredFists(MWBase::World& world)
        {
            if (!fistSteerEnabled())
                return false;
            const MWWorld::Ptr player = world.getPlayerPtr();
            return !player.isEmpty() && world.isActorPoweredRagdoll(player)
                && player.getClass().getCreatureStats(player).getDrawState() == MWMechanics::DrawState::Weapon
                && playerWeapon(world).isEmpty();
        }
    }

    MouseManager::MouseManager(
        BindingsManager* bindingsManager, SDLUtil::InputWrapper* inputWrapper, SDL_Window* window)
        : mBindingsManager(bindingsManager)
        , mInputWrapper(inputWrapper)
        , mWindow(window)
        , mWindowHadFocus((SDL_GetWindowFlags(window) & SDL_WINDOW_INPUT_FOCUS) != 0)
        , mGuiCursorX(0)
        , mGuiCursorY(0)
        , mMouseWheel(0)
        , mMouseLookEnabled(false)
        , mGuiCursorEnabled(true)
        , mLastWarpX(-1)
        , mLastWarpY(-1)
        , mMouseMoveX(0)
        , mMouseMoveY(0)
    {
        int w, h;
        SDL_GetWindowSize(window, &w, &h);

        float uiScale = MWBase::Environment::get().getWindowManager()->getScalingFactor();
        mGuiCursorX = w / (2.f * uiScale);
        mGuiCursorY = h / (2.f * uiScale);
    }

    void MouseManager::mouseMoved(const SDLUtil::MouseMotionEvent& arg)
    {
        ManualDiagnostics::recorder().mouse(arg.xrel, arg.yrel, 0, false, mMouseLookEnabled, mGuiCursorEnabled);
        cancelInvalidRagdollTapDrag();
        static unsigned traceCount = 0;
        if (std::getenv("OPENMW_MOUSE_INPUT_TRACE") && traceCount++ < 32)
            Log(Debug::Info) << "MOUSE_INPUT gameplay dx=" << arg.xrel << " dy=" << arg.yrel
                << " look=" << mMouseLookEnabled << " use=" << mBindingsManager->actionIsActive(A_Use)
                << " pending=" << (mPendingRagdollMouseUse ? (*mPendingRagdollMouseUse ? 1 : 0) : -1);
        mBindingsManager->mouseMoved(arg);

        MWBase::InputManager* input = MWBase::Environment::get().getInputManager();
        input->setJoystickLastUsed(false);
        input->resetIdleTime();

        if (mGuiCursorEnabled)
        {
            input->setGamepadGuiCursorEnabled(true);

            // We keep track of our own mouse position, so that moving the mouse while in
            // game mode does not move the position of the GUI cursor
            MWBase::WindowManager* winMgr = MWBase::Environment::get().getWindowManager();
            float uiScale = winMgr->getScalingFactor();
            mGuiCursorX = static_cast<float>(arg.x) / uiScale;
            mGuiCursorY = static_cast<float>(arg.y) / uiScale;

            if (arg.zrel != 0)
                mMouseWheel = static_cast<int>(arg.z);

            MyGUI::InputManager::getInstance().injectMouseMove(
                static_cast<int>(mGuiCursorX), static_cast<int>(mGuiCursorY), mMouseWheel);

            winMgr->setCursorActive(true);

            // Check if this movement is from our recent mouse warp
            bool isFromWarp = (mLastWarpX >= 0 && mLastWarpY >= 0 && std::abs(mGuiCursorX - mLastWarpX) < 0.5f
                && std::abs(mGuiCursorY - mLastWarpY) < 0.5f);

            if (Settings::gui().mControllerMenus && !winMgr->getCursorVisible()
                && (std::abs(arg.xrel) > 1 || std::abs(arg.yrel) > 1) && !isFromWarp)
            {
                // Unhide the cursor if it was hidden to show a controller tooltip.
                winMgr->setControllerTooltipVisible(false);
                winMgr->setCursorVisible(true);
            }

            // Clear warp tracking after processing
            mLastWarpX = -1;
            mLastWarpY = -1;
        }

        if (mMouseLookEnabled && !input->controlsDisabled())
        {
            MWBase::World* world = MWBase::Environment::get().getWorld();

            const float cameraSensitivity = Settings::input().mCameraSensitivity;
            float x = arg.xrel * cameraSensitivity * (Settings::input().mInvertXAxis ? -1 : 1) / 256.f;
            float y = arg.yrel * cameraSensitivity * (Settings::input().mInvertYAxis ? -1 : 1)
                * Settings::input().mCameraYMultiplier / 256.f;

            // The right-click guard's zone takes the mouse while it is held.
            if ((mRmbGuardHeld || (mRmbGuardTapLeft > 0.0f && tapParryAimEnabled())) && !mGuardYielded)
            {
                const float gain
                    = guardZoneGain(mRmbGuardHeld ? mRmbGuardHeldTime : mGuardTapWindow - mRmbGuardTapLeft);
                if (world->driveActorRagdollGuardZone(x * gain, -y * gain, false))
                    return;
            }

            // FIST-07: with the fists the mouse is the view's, whatever is held.
            const bool viewFists = fistSteerMode() == 2 && !mRagdollThrustClaimed && steeredFists(*world);

            // FIST-06: the right button holds the left fist - the mouse moves it.
            if (mFistLeftHeld && !viewFists && world->driveActorRagdollLeftFist(x, -y, true, false))
                return;

            // K2_HOLD_AIM: the undecided press - no view turn; far enough, a drag.
            if (holdAimEnabled() && mRagdollTapDrag.pending() && !viewFists)
            {
                if (mRagdollTapDrag.move(static_cast<float>(arg.xrel), static_cast<float>(arg.yrel), x, y)
                    == RagdollTapDragAction::BeginDrag)
                {
                    world->cancelPlayerRagdollAttackInput();
                    if (world->driveActorRagdollSwing(0.0f, 0.0f, true))
                    {
                        mRagdollUseClaimed = true;
                        mRagdollSwingHeld = true;
                        world->driveActorRagdollSwing(mRagdollTapDrag.pendingX(), -mRagdollTapDrag.pendingY(), true);
                    }
                    else
                        mRagdollTapDrag.cancel();
                }
                return;
            }

            // While the attack is held the mouse belongs to the weapon, not
            // the view. This consumes the motion only when the player's body
            // is a powered ragdoll; otherwise nothing changes and the camera
            // turns exactly as it always did.
            if (!viewFists && (!mRagdollAnimationUseClaimed || mRagdollThrustClaimed)
                && (mRagdollThrustClaimed || mBindingsManager->actionIsActive(A_PhysicalThrust) || mRagdollUseClaimed
                    || mPendingRagdollMouseUse.value_or(mBindingsManager->actionIsActive(A_Use)))
                && world->driveActorRagdollSwing(x, -y, true,
                    mRagdollThrustClaimed || mBindingsManager->actionIsActive(A_PhysicalThrust)))
            {
                mRagdollSwingHeld = true;
                return;
            }

            float rot[3];
            rot[0] = -y;
            rot[1] = 0.0f;
            rot[2] = -x;

            // Only actually turn player when we're not in vanity mode
            if (!world->vanityRotateCamera(rot) && input->getControlSwitch("playerlooking"))
            {
                MWWorld::Player& player = world->getPlayer();
                player.yaw(x);
                player.pitch(y);
            }
            else if (!input->getControlSwitch("playerlooking"))
                MWBase::Environment::get().getWorld()->disableDeferredPreviewRotation();
        }
    }

    void MouseManager::mouseReleased(const SDL_MouseButtonEvent& arg, Uint8 id)
    {
        ManualDiagnostics::recorder().mouse(0, 0, id, false, mMouseLookEnabled, mGuiCursorEnabled);
        // FIST-06: the guard of both buttons ends with either. A tap of both
        // is a timed palm parry (K2_PALM_TAP: the guard stays up for the rest
        // of its window, as a weapon's tapped guard does; 11-17, the player's
        // presses of both were down a median 0.127 s and the guard was down
        // for 22 of the 24 blows that landed), or with the window 0 the shove
        // as before. The other button, still down, does nothing more.
        if (mFistGuard && (id == SDL_BUTTON_LEFT || id == SDL_BUTTON_RIGHT))
        {
            mFistGuard = false;
            // (pressed again while the other is still down, it is both again)
            mFistGuardLinger = id == SDL_BUTTON_LEFT ? SDL_BUTTON_RIGHT : SDL_BUTTON_LEFT;
            if (mRmbGuardHeld)
            {
                mRmbGuardHeld = false;
                if (mRmbGuardHeldTime < RagdollCore::sPalmTapHeld && !mGuardYielded)
                {
                    if (palmTapSettings().mWindow > 0.0f)
                    {
                        mGuardTapWindow = palmTapSettings().mWindow;
                        mRmbGuardTapLeft = RagdollCore::palmTapLeft(palmTapSettings(), mRmbGuardHeldTime);
                        mFistTapParry = true;
                    }
                    else
                        MWBase::Environment::get().getWorld()->shoveActorRagdoll();
                }
            }
            MWBase::Environment::get().getLuaManager()->inputEvent(
                { MWBase::LuaManager::InputEvent::MouseButtonReleased, arg.button });
            return;
        }
        if (id == mFistGuardLinger)
            mFistGuardLinger = 0;
        if (mFistLeftHeld && id == SDL_BUTTON_RIGHT)
        {
            mFistLeftHeld = false;
            MWBase::Environment::get().getWorld()->driveActorRagdollLeftFist(0.0f, 0.0f, false, false);
            MWBase::Environment::get().getLuaManager()->inputEvent(
                { MWBase::LuaManager::InputEvent::MouseButtonReleased, arg.button });
            return;
        }
        if (mRmbGuardHeld && id == mGuardButton)
        {
            mRmbGuardHeld = false;
            // A tap holds the guard up long enough to meet the blow: the parry
            // window counts from the raise (not after an attack took over).
            if (mRmbGuardHeldTime < 0.2f && !mGuardYielded)
            {
                mGuardTapWindow = 0.35f;
                mRmbGuardTapLeft = 0.35f - mRmbGuardHeldTime;
            }
            return;
        }
        cancelInvalidRagdollTapDrag();
        MWBase::Environment::get().getInputManager()->setJoystickLastUsed(false);
        if (mRagdollThrustClaimed && mBindingsManager->isMouseButtonBound(A_PhysicalThrust, id))
        {
            mRagdollThrustClaimed = false;
            MWBase::Environment::get().getWorld()->driveActorRagdollSwing(0, 0, mRagdollUseClaimed, false);
            mRagdollSwingHeld = mRagdollUseClaimed;
            return;
        }
        const bool releasesAnimationUse
            = mRagdollAnimationUseClaimed && mBindingsManager->isMouseButtonBound(A_Use, id);
        const bool releasesTapDrag = (mRagdollTapDrag.pending() || mRagdollTapDrag.dragging())
            && mBindingsManager->isMouseButtonBound(A_Use, id);
        const bool releasesClaimedRagdollUse
            = (mRagdollUseClaimed || releasesAnimationUse || releasesTapDrag)
            && mBindingsManager->isMouseButtonBound(A_Use, id);

        if (mBindingsManager->isDetectingBindingState())
        {
            if (!releasesClaimedRagdollUse)
                mBindingsManager->mouseReleased(arg, id);
        }
        else
        {
            bool guiMode = MWBase::Environment::get().getWindowManager()->isGuiMode();
            guiMode = MyGUI::InputManager::getInstance().injectMouseRelease(static_cast<int>(mGuiCursorX),
                          static_cast<int>(mGuiCursorY), SDLUtil::sdlMouseButtonToMyGui(id))
                && guiMode;

            if (mBindingsManager->isDetectingBindingState())
                return; // don't allow same mouseup to bind as initiated bind

            mBindingsManager->setPlayerControlsEnabled(!guiMode);
            if (!releasesClaimedRagdollUse)
                mBindingsManager->mouseReleased(arg, id);
        }

        if (mBindingsManager->isMouseButtonBound(A_Use, id))
        {
            mPendingRagdollMouseUse = false;
            const auto tapDrag = releasesTapDrag ? mRagdollTapDrag.release() : RagdollTapDragAction::None;
            mRagdollTapDragWeapon.reset();
            if (tapDrag == RagdollTapDragAction::QuickAttack)
            {
                // A quick click enters and releases the native animation in
                // one preserved input pair. No physical gesture was started.
                auto world = MWBase::Environment::get().getWorld();
                if (world->queuePlayerRagdollAttackInput(true))
                    world->queuePlayerRagdollAttackInput(false);
                else
                    world->cancelPlayerRagdollAttackInput();
            }
            else if (tapDrag == RagdollTapDragAction::EndDrag)
            {
                mRagdollUseClaimed = false;
                MWBase::Environment::get().getWorld()->driveActorRagdollSwing(
                    0.0f, 0.0f, mRagdollThrustClaimed, mRagdollThrustClaimed);
                mRagdollSwingHeld = mRagdollThrustClaimed;
            }
            else if (releasesAnimationUse)
            {
                mRagdollAnimationUseClaimed = false;
                MWBase::Environment::get().getWorld()->queuePlayerRagdollAttackInput(false);
            }
            else if (releasesClaimedRagdollUse)
            {
                mRagdollUseClaimed = false;
                MWBase::Environment::get().getWorld()->driveActorRagdollSwing(0.0f, 0.0f,
                    mRagdollThrustClaimed, mRagdollThrustClaimed);
                mRagdollSwingHeld = mRagdollThrustClaimed;
            }
        }

        MWBase::Environment::get().getLuaManager()->inputEvent(
            { MWBase::LuaManager::InputEvent::MouseButtonReleased, arg.button });
    }

    void MouseManager::mouseWheelMoved(const SDL_MouseWheelEvent& arg)
    {
        MWBase::InputManager* input = MWBase::Environment::get().getInputManager();
        if (mBindingsManager->isDetectingBindingState() || !input->controlsDisabled())
        {
            mBindingsManager->mouseWheelMoved(arg);
        }

        input->setJoystickLastUsed(false);
        MWBase::Environment::get().getLuaManager()->inputEvent({ MWBase::LuaManager::InputEvent::MouseWheel,
            MWBase::LuaManager::InputEvent::WheelChange{ arg.x, arg.y } });
    }

    void MouseManager::mousePressed(const SDL_MouseButtonEvent& arg, Uint8 id)
    {
        ManualDiagnostics::recorder().mouse(0, 0, id, true, mMouseLookEnabled, mGuiCursorEnabled);
        MWBase::InputManager* input = MWBase::Environment::get().getInputManager();
        input->setJoystickLastUsed(false);
        bool guiMode = MWBase::Environment::get().getWindowManager()->isGuiMode();

        if (id == SDL_BUTTON_LEFT || id == SDL_BUTTON_RIGHT) // MyGUI only uses these mouse events
        {
            guiMode = MWBase::Environment::get().getWindowManager()->isGuiMode();
            guiMode = MyGUI::InputManager::getInstance().injectMousePress(static_cast<int>(mGuiCursorX),
                          static_cast<int>(mGuiCursorY), SDLUtil::sdlMouseButtonToMyGui(id))
                && guiMode;
            if (MyGUI::InputManager::getInstance().getMouseFocusWidget() != nullptr)
            {
                MyGUI::Button* b
                    = MyGUI::InputManager::getInstance().getMouseFocusWidget()->castType<MyGUI::Button>(false);
                if (b && b->getEnabled() && id == SDL_BUTTON_LEFT)
                {
                    MWBase::Environment::get().getWindowManager()->playSound(ESM::RefId::stringRefId("Menu Click"));
                }
            }
            MWBase::Environment::get().getWindowManager()->setCursorActive(true);
        }

        mBindingsManager->setPlayerControlsEnabled(!guiMode);

        // FIST-06 (K2_FIST_STEER): with bare fists the right button is the
        // left fist (a tap its straight; held, the mouse moves it; let go, it
        // is thrown), and both buttons together are the guard.
        if ((id == SDL_BUTTON_LEFT || id == SDL_BUTTON_RIGHT) && !guiMode && !mBindingsManager->isDetectingBindingState()
            && !MWBase::Environment::get().getWindowManager()->isSettingsWindowVisible() && !input->controlsDisabled()
            && mMouseLookEnabled && !mFistGuard && steeredFists(*MWBase::Environment::get().getWorld()))
        {
            MWBase::World* world = MWBase::Environment::get().getWorld();
            const bool rightFistDown = mRagdollUseClaimed || mRagdollTapDrag.pending() || mRagdollTapDrag.dragging();
            // K2_PALM_TAP (11-17, the first session with the tap: of the
            // guard's two buttons one came up and went down again with the
            // other still held, 0.1-0.2 s apart; that press was the left
            // fist's, ended the tapped guard and threw a cross): the button
            // of an ended guard pressed again while the other is still down
            // is both.
            const bool again = palmTapSettings().mWindow > 0.0f && mFistGuardLinger != 0 && id != mFistGuardLinger;
            const bool both = again
                || (id == SDL_BUTTON_RIGHT ? rightFistDown
                        : mFistLeftHeld && mBindingsManager->isMouseButtonBound(A_Use, id));
            if (both)
            {
                // The guard goes up first, then the holds end: a hold let go
                // under the guard throws nothing (ended first, a physics step
                // in between threw the right fist: FIST-06 11-16).
                mRagdollGuardHeld = world->setActorRagdollGuard(true);
                cancelRagdollUse();
                world->driveActorRagdollLeftFist(0.0f, 0.0f, false, true);
                mFistLeftHeld = false;
                mFistGuard = true;
                mGuardButton = id;
                // K2_PALM_TAP: both pressed again this soon after the last
                // press is the shove (the weapons' double click), thrown from
                // the raised guard as the tap's shove was; the guard is not
                // held under it. A shove that does not start (its cooldown)
                // leaves the press the guard.
                // With a carry set, never while a blow is read: the player's
                // parries came every 0.1-0.2 s (11-17: 5 blows landed on a
                // shove the second tap threw).
                const bool shoved = RagdollCore::palmTapShoves(palmTapSettings(), mFistGuardSincePress)
                    && !(palmTapSettings().mCarry > 0.0f && world->isPlayerRagdollPalmBusy())
                    && world->shoveActorRagdoll();
                mFistGuardLinger = 0;
                mFistTapParry = false;
                mFistGuardSincePress = shoved ? 10.0f : 0.0f;
                mRmbGuardHeld = !shoved;
                mGuardYielded = false;
                mRmbGuardHeldTime = 0.0f;
                mRmbGuardTapLeft = 0.0f;
                world->driveActorRagdollGuardZone(0.0f, 0.0f, true);
                MWBase::Environment::get().getLuaManager()->inputEvent(
                    { MWBase::LuaManager::InputEvent::MouseButtonPressed, arg.button });
                return;
            }
            if (id == SDL_BUTTON_RIGHT && world->driveActorRagdollLeftFist(0.0f, 0.0f, true, false))
            {
                mFistLeftHeld = true;
                // (a fist's press in a tapped guard's window ends it, as the
                // right fist's does through GUARD-03)
                if (palmTapSettings().mWindow > 0.0f)
                    mRmbGuardTapLeft = 0.0f;
                MWBase::Environment::get().getLuaManager()->inputEvent(
                    { MWBase::LuaManager::InputEvent::MouseButtonPressed, arg.button });
                return;
            }
        }

        // Right-click guard (user 10-07): with a weapon drawn in the game the
        // right button is the guard - held, the mouse picks its zone; a tap is
        // a timed parry. Sheathed, it stays the inventory (also on I).
        // OPENMW_RAGDOLL_RMB_GUARD=1.
        const bool rightGuard = id == SDL_BUTTON_RIGHT && rmbGuardEnabled();
        const bool middleGuard = id == SDL_BUTTON_MIDDLE && middleParryEnabled();
        if ((rightGuard || middleGuard) && !guiMode && !mBindingsManager->isDetectingBindingState()
            && !MWBase::Environment::get().getWindowManager()->isSettingsWindowVisible() && !input->controlsDisabled())
        {
            MWBase::World* world = MWBase::Environment::get().getWorld();
            const MWWorld::Ptr player = world->getPlayerPtr();
            if (!player.isEmpty() && world->isActorPoweredRagdoll(player)
                && player.getClass().getCreatureStats(player).getDrawState() == MWMechanics::DrawState::Weapon)
            {
                // A double click shoves (SHOVE-01, user 10-29); the first
                // click was a tap of the guard. Clicking on rapidly adds power.
                if (rightGuard && (mRmbShoveBurst > 0.0f || mRmbSincePress < 0.3f))
                {
                    mRmbSincePress = 10.0f;
                    if (world->shoveActorRagdoll())
                    {
                        mRmbGuardHeld = false;
                        mRmbGuardTapLeft = 0.0f;
                        mRmbShoveBurst = 0.3f;
                        return;
                    }
                }
                // The other guard button while one holds the guard: nothing new.
                if (mRmbGuardHeld && id != mGuardButton)
                    return;
                if (rightGuard)
                    mRmbSincePress = 0.0f;
                mGuardButton = id;
                mRmbGuardHeld = true;
                mGuardYielded = false;
                mRmbGuardHeldTime = 0.0f;
                world->driveActorRagdollGuardZone(0.0f, 0.0f, true);
                return;
            }
        }

        if (id == SDL_BUTTON_MIDDLE && kickEnabled() && middleButtonMode() == 2 && !guiMode
            && !mBindingsManager->isDetectingBindingState()
            && !MWBase::Environment::get().getWindowManager()->isSettingsWindowVisible() && !input->controlsDisabled())
        {
            MWBase::World* world = MWBase::Environment::get().getWorld();
            if (world->isActorPoweredRagdoll(world->getPlayerPtr()))
            {
                world->kickActorRagdoll();
                return;
            }
        }

        // Don't trigger any mouse bindings while in settings menu, otherwise rebinding controls becomes impossible
        // Also do not trigger bindings when input controls are disabled, e.g. during save loading
        if (!MWBase::Environment::get().getWindowManager()->isSettingsWindowVisible() && !input->controlsDisabled())
        {
            MWBase::World* world = MWBase::Environment::get().getWorld();
            // The physical body owns attack while it is powered: with a weapon
            // the swing drives the blade, bare-handed the same hold drives the
            // fist (a strike while the fists are drawn, a free hand otherwise).
            const bool playerPhysical = world->isPlayerRagdollWeaponAttached()
                || world->isActorPoweredRagdoll(world->getPlayerPtr());
            // GUARD-03: the attack press lowers the guard first, so the press
            // reaches a body that is no longer blocking.
            if (guardToAttackEnabled() && !guiMode && !mBindingsManager->isDetectingBindingState() && playerPhysical
                && (mBindingsManager->isMouseButtonBound(A_Use, id)
                    || mBindingsManager->isMouseButtonBound(A_PhysicalThrust, id))
                && (mRagdollGuardHeld || mRmbGuardHeld || mRmbGuardTapLeft > 0.0f
                    || mBindingsManager->actionIsActive(A_PhysicalGuard)))
            {
                mGuardYielded = true;
                mRmbGuardTapLeft = 0.0f;
                if (mRagdollGuardHeld)
                    mRagdollGuardHeld = !world->setActorRagdollGuard(false);
            }
            if (!guiMode && !mBindingsManager->isDetectingBindingState()
                && mBindingsManager->isMouseButtonBound(A_PhysicalThrust, id) && playerPhysical)
            {
                cancelRagdollUse();
                if (world->driveActorRagdollSwing(0, 0, true, true))
                {
                    mRagdollThrustClaimed = true;
                    mRagdollSwingHeld = true;
                    return;
                }
            }
            const bool normalUse = !guiMode && !mBindingsManager->isDetectingBindingState()
                && mBindingsManager->isMouseButtonBound(A_Use, id);
            const bool tapDragUse = normalUse && !mRagdollThrustClaimed
                && world->usesPlayerRagdollAttackAnimation()
                && (mRagdollTapDrag.pending() || mRagdollTapDrag.dragging() || mRagdollTapDrag.press());
            const bool ragdollUse = normalUse && !tapDragUse && playerPhysical
                && world->driveActorRagdollSwing(0.0f, 0.0f, true);
            if (tapDragUse)
            {
                if (!mRagdollTapDragWeapon)
                    mRagdollTapDragWeapon = playerWeapon(*world);
                // Keep the undecided press out of the polled Use channel.
                // Before the threshold mouse motion remains ordinary aim.
                mPendingRagdollMouseUse = false;
            }
            else if (ragdollUse)
            {
                // The physical weapon owns this complete press/release pair.
                // Do not arm the ordinary animation hit path as well.
                mRagdollUseClaimed = true;
                mRagdollSwingHeld = true;
                mPendingRagdollMouseUse = true;
            }
            else
            {
                mBindingsManager->mousePressed(arg, id);
                if (!guiMode && !mBindingsManager->isDetectingBindingState()
                    && mBindingsManager->isMouseButtonBound(A_Use, id))
                    mPendingRagdollMouseUse = true;
            }
        }
        MWBase::Environment::get().getLuaManager()->inputEvent(
            { MWBase::LuaManager::InputEvent::MouseButtonPressed, arg.button });
    }

    void MouseManager::updateCursorMode()
    {
        bool grab = !MWBase::Environment::get().getWindowManager()->containsMode(MWGui::GM_MainMenu)
            && !MWBase::Environment::get().getWindowManager()->isConsoleMode();

        bool wasRelative = mInputWrapper->getMouseRelative();
        bool isRelative = !MWBase::Environment::get().getWindowManager()->isGuiMode();

        // don't keep the pointer away from the window edge in gui mode
        // stop using raw mouse motions and switch to system cursor movements
        mInputWrapper->setMouseRelative(isRelative);

        // we let the mouse escape in the main menu
        mInputWrapper->setGrabPointer(grab && (Settings::input().mGrabCursor || isRelative));

        // we switched to non-relative mode, move our cursor to where the in-game
        // cursor is
        if (!isRelative && wasRelative != isRelative)
        {
            warpMouse();
        }
    }

    void MouseManager::setMouseLookEnabled(bool enabled)
    {
        if (mMouseLookEnabled && !enabled)
            cancelRagdollUse();
        mMouseLookEnabled = enabled;
    }

    void MouseManager::cancelRagdollUse()
    {
        MWBase::Environment::get().getWorld()->cancelPlayerRagdollAttackInput();
        mRagdollAnimationUseClaimed = false;
        const auto tapDrag = mRagdollTapDrag.cancel();
        mRagdollTapDragWeapon.reset();
        if (tapDrag == RagdollTapDragAction::EndDrag
            || mRagdollUseClaimed || mRagdollThrustClaimed || mRagdollSwingHeld)
            MWBase::Environment::get().getWorld()->driveActorRagdollSwing(0.0f, 0.0f, false);
        mRagdollUseClaimed = false;
        mRagdollThrustClaimed = false;
        mRagdollSwingHeld = false;
        mPendingRagdollMouseUse = false;
    }

    void MouseManager::cancelInvalidRagdollTapDrag()
    {
        if (!mRagdollTapDrag.pending() && !mRagdollTapDrag.dragging())
            return;
        auto& world = *MWBase::Environment::get().getWorld();
        if (!world.usesPlayerRagdollAttackAnimation() || !mRagdollTapDragWeapon
            || *mRagdollTapDragWeapon != playerWeapon(world))
            cancelRagdollUse();
    }

    void MouseManager::update(float dt)
    {
        // The press belongs to the equipment and powered body that received
        // it. A replacement cannot inherit an undecided click or held drag.
        cancelInvalidRagdollTapDrag();
        // BindingsManager has now advanced the action channels. During event
        // capture, button edges must govern motion in that same event batch.
        mPendingRagdollMouseUse.reset();
        SDL_GetRelativeMouseState(&mMouseMoveX, &mMouseMoveY);

        MWBase::InputManager* input = MWBase::Environment::get().getInputManager();
        const bool windowHasFocus = (SDL_GetWindowFlags(mWindow) & SDL_WINDOW_INPUT_FOCUS) != 0;
        if (mWindowHadFocus && !windowHasFocus)
        {
            cancelRagdollUse();
            if (mFistLeftHeld)
            {
                mFistLeftHeld = false;
                MWBase::Environment::get().getWorld()->driveActorRagdollLeftFist(0.0f, 0.0f, false, true);
            }
        }
        mWindowHadFocus = windowHasFocus;
        if (!mMouseLookEnabled || input->controlsDisabled()
            || MWBase::Environment::get().getWindowManager()->isGuiMode())
            cancelRagdollUse();
        if (mRagdollTapDrag.pending())
        {
            // The pending press is intentionally withheld from BindingsManager,
            // so its inactive Use channel cannot be treated as a release. The
            // button-up edge and the explicit focus/control/GUI gates above own
            // cancellation.
            if (mRagdollTapDrag.advance(dt, true) == RagdollTapDragAction::BeginDrag)
            {
                auto world = MWBase::Environment::get().getWorld();
                world->cancelPlayerRagdollAttackInput();
                if (world->driveActorRagdollSwing(0.0f, 0.0f, true))
                {
                    mRagdollUseClaimed = true;
                    mRagdollSwingHeld = true;
                }
                else
                    mRagdollTapDrag.cancel();
            }
        }
        const bool held = (!mRagdollAnimationUseClaimed || mRagdollThrustClaimed)
            && mMouseLookEnabled && !input->controlsDisabled()
            && (mRagdollUseClaimed || mRagdollThrustClaimed
                || mBindingsManager->actionIsActive(A_PhysicalThrust));
        // An idle input device must not cancel a swing submitted by another
        // input source. Still release a native hold on button-up, disabled
        // controls or leaving mouse-look, including a press/move/release that
        // was consumed entirely between update calls.
        // Hold the guard key: the body's physical block/parry stance.
        // Raised once per press, so holding it turns a parry into a block.
        if (mRmbGuardHeld)
            mRmbGuardHeldTime += dt;
        // K2_PALM_TAP's carry: a tapped fist guard whose window is ending
        // stays up while it reads a blow or its palm is out (a window an
        // attack's press or a new press ended is at 0 and stays there).
        if (mFistTapParry)
        {
            if (mRmbGuardTapLeft <= 0.0f)
                mFistTapParry = false;
            else if (mRmbGuardTapLeft <= dt && RagdollCore::palmTapCarries(palmTapSettings(), mFistGuardSincePress)
                && MWBase::Environment::get().getWorld()->isPlayerRagdollPalmBusy())
                mRmbGuardTapLeft = dt + 0.001f;
        }
        mRmbGuardTapLeft = std::max(0.0f, mRmbGuardTapLeft - dt);
        mRmbSincePress += dt;
        mFistGuardSincePress += dt;
        mRmbShoveBurst = std::max(0.0f, mRmbShoveBurst - dt);
        if (!mMouseLookEnabled || input->controlsDisabled()
            || MWBase::Environment::get().getWindowManager()->isGuiMode())
        {
            mRmbGuardHeld = false;
            mFistGuard = false;
            mFistGuardLinger = 0;
        }
        // FIST-06: the left fist's hold ends unthrown with the game's input
        // (a menu; lost focus, above) or the fists themselves (a weapon drawn).
        if (mFistLeftHeld
            && (!mMouseLookEnabled || input->controlsDisabled()
                || MWBase::Environment::get().getWindowManager()->isGuiMode()
                || !steeredFists(*MWBase::Environment::get().getWorld())))
        {
            mFistLeftHeld = false;
            MWBase::Environment::get().getWorld()->driveActorRagdollLeftFist(0.0f, 0.0f, false, true);
        }
        else if (mFistLeftHeld)
            MWBase::Environment::get().getWorld()->driveActorRagdollLeftFist(0.0f, 0.0f, true, false);
        const bool guardInput
            = mBindingsManager->actionIsActive(A_PhysicalGuard) || mRmbGuardHeld || mRmbGuardTapLeft > 0.0f;
        // GUARD-03: a guard an attack lowered comes back with a new press.
        if (!guardInput)
            mGuardYielded = false;
        const bool guard = guardInput && !mGuardYielded && !input->controlsDisabled();
        if (guard != mRagdollGuardHeld)
            mRagdollGuardHeld = MWBase::Environment::get().getWorld()->setActorRagdollGuard(guard) && guard;
        if (held || mRagdollSwingHeld)
        {
            const bool accepted = MWBase::Environment::get().getWorld()->driveActorRagdollSwing(0.0f, 0.0f, held,
                mRagdollThrustClaimed || mBindingsManager->actionIsActive(A_PhysicalThrust));
            mRagdollSwingHeld = held && accepted;
        }

        if (!mMouseLookEnabled)
            return;

        float xAxis = mBindingsManager->getActionValue(A_LookLeftRight) * 2.0f - 1.0f;
        float yAxis = mBindingsManager->getActionValue(A_LookUpDown) * 2.0f - 1.0f;
        if (xAxis == 0 && yAxis == 0)
            return;

        const float cameraSensitivity = Settings::input().mCameraSensitivity;
        const float rot[3] = {
            -yAxis * dt * 1000.0f * cameraSensitivity * (Settings::input().mInvertYAxis ? -1 : 1)
                * Settings::input().mCameraYMultiplier / 256.f,
            0.0f,
            -xAxis * dt * 1000.0f * cameraSensitivity * (Settings::input().mInvertXAxis ? -1 : 1) / 256.f,
        };

        // Only actually turn player when we're not in vanity mode
        bool playerLooking = MWBase::Environment::get().getInputManager()->getControlSwitch("playerlooking");
        if (!MWBase::Environment::get().getWorld()->vanityRotateCamera(rot) && playerLooking)
        {
            MWWorld::Player& player = MWBase::Environment::get().getWorld()->getPlayer();
            player.yaw(-rot[2]);
            player.pitch(-rot[0]);
        }
        else if (!playerLooking)
            MWBase::Environment::get().getWorld()->disableDeferredPreviewRotation();

        MWBase::Environment::get().getInputManager()->resetIdleTime();
    }

    bool MouseManager::injectMouseButtonPress(Uint8 button)
    {
        return MyGUI::InputManager::getInstance().injectMousePress(
            static_cast<int>(mGuiCursorX), static_cast<int>(mGuiCursorY), SDLUtil::sdlMouseButtonToMyGui(button));
    }

    bool MouseManager::injectMouseButtonRelease(Uint8 button)
    {
        return MyGUI::InputManager::getInstance().injectMouseRelease(
            static_cast<int>(mGuiCursorX), static_cast<int>(mGuiCursorY), SDLUtil::sdlMouseButtonToMyGui(button));
    }

    void MouseManager::injectMouseMove(float xMove, float yMove, float mouseWheelMove)
    {
        mGuiCursorX += xMove;
        mGuiCursorY += yMove;
        mMouseWheel += static_cast<int>(mouseWheelMove);

        const MyGUI::IntSize& viewSize = MyGUI::RenderManager::getInstance().getViewSize();
        mGuiCursorX = std::clamp<float>(mGuiCursorX, 0.f, viewSize.width - 1.f);
        mGuiCursorY = std::clamp<float>(mGuiCursorY, 0.f, viewSize.height - 1.f);

        MyGUI::InputManager::getInstance().injectMouseMove(
            static_cast<int>(mGuiCursorX), static_cast<int>(mGuiCursorY), mMouseWheel);
    }

    void MouseManager::warpMouse()
    {
        float guiUiScale = Settings::gui().mScalingFactor;
        mInputWrapper->warpMouse(
            static_cast<int>(mGuiCursorX * guiUiScale), static_cast<int>(mGuiCursorY * guiUiScale));
    }

    void MouseManager::warpMouseToWidget(MyGUI::Widget* widget)
    {
        float widgetX = widget->getAbsoluteCoord().left + widget->getWidth() / 2.f;
        float widgetY = widget->getAbsoluteCoord().top + widget->getHeight() / 4.f;
        if (std::abs(mGuiCursorX - widgetX) > 1 || std::abs(mGuiCursorY - widgetY) > 1)
        {
            mGuiCursorX = widgetX;
            mGuiCursorY = widgetY;
            // Remember where we warped to so we can ignore movement from this warp
            mLastWarpX = widgetX;
            mLastWarpY = widgetY;
            warpMouse();
        }
    }

}

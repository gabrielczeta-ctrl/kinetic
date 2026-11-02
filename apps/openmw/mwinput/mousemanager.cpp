#include "mousemanager.hpp"
#include <ragdollcore/tuning.hpp>
#include "../manualdiagnostics.hpp"

#include "actions.hpp"
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
            if (mRmbGuardHeld && !mGuardYielded && world->driveActorRagdollGuardZone(x, -y, false))
                return;

            // While the attack is held the mouse belongs to the weapon, not
            // the view. This consumes the motion only when the player's body
            // is a powered ragdoll; otherwise nothing changes and the camera
            // turns exactly as it always did.
            if ((!mRagdollAnimationUseClaimed || mRagdollThrustClaimed)
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
        if (id == SDL_BUTTON_RIGHT && mRmbGuardHeld)
        {
            mRmbGuardHeld = false;
            // A tap holds the guard up long enough to meet the blow: the parry
            // window counts from the raise (not after an attack took over).
            if (mRmbGuardHeldTime < 0.2f && !mGuardYielded)
                mRmbGuardTapLeft = 0.35f - mRmbGuardHeldTime;
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

        // Right-click guard (user 10-07): with a weapon drawn in the game the
        // right button is the guard - held, the mouse picks its zone; a tap is
        // a timed parry. Sheathed, it stays the inventory (also on I).
        // OPENMW_RAGDOLL_RMB_GUARD=1.
        if (id == SDL_BUTTON_RIGHT && rmbGuardEnabled() && !guiMode && !mBindingsManager->isDetectingBindingState()
            && !MWBase::Environment::get().getWindowManager()->isSettingsWindowVisible() && !input->controlsDisabled())
        {
            MWBase::World* world = MWBase::Environment::get().getWorld();
            const MWWorld::Ptr player = world->getPlayerPtr();
            if (!player.isEmpty() && world->isActorPoweredRagdoll(player)
                && player.getClass().getCreatureStats(player).getDrawState() == MWMechanics::DrawState::Weapon)
            {
                // A double click shoves (SHOVE-01, user 10-29); the first
                // click was a tap of the guard. Clicking on rapidly adds power.
                if (mRmbShoveBurst > 0.0f || mRmbSincePress < 0.3f)
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
                mRmbSincePress = 0.0f;
                mRmbGuardHeld = true;
                mGuardYielded = false;
                mRmbGuardHeldTime = 0.0f;
                world->driveActorRagdollGuardZone(0.0f, 0.0f, true);
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
            cancelRagdollUse();
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
        mRmbGuardTapLeft = std::max(0.0f, mRmbGuardTapLeft - dt);
        mRmbSincePress += dt;
        mRmbShoveBurst = std::max(0.0f, mRmbShoveBurst - dt);
        if (!mMouseLookEnabled || input->controlsDisabled()
            || MWBase::Environment::get().getWindowManager()->isGuiMode())
            mRmbGuardHeld = false;
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

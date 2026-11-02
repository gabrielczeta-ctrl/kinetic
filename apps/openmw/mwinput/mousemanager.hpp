#ifndef MWINPUT_MWMOUSEMANAGER_H
#define MWINPUT_MWMOUSEMANAGER_H

#include <components/sdlutil/events.hpp>
#include <components/settings/settings.hpp>
#include <optional>

#include "ragdolltapdrag.hpp"
#include "../mwworld/ptr.hpp"

namespace SDLUtil
{
    class InputWrapper;
}

namespace MWInput
{
    class BindingsManager;

    class MouseManager : public SDLUtil::MouseListener
    {
    public:
        MouseManager(BindingsManager* bindingsManager, SDLUtil::InputWrapper* inputWrapper, SDL_Window* window);

        virtual ~MouseManager() = default;

        void updateCursorMode();
        void update(float dt);

        void mouseMoved(const SDLUtil::MouseMotionEvent& arg) override;
        void mousePressed(const SDL_MouseButtonEvent& arg, Uint8 id) override;
        void mouseReleased(const SDL_MouseButtonEvent& arg, Uint8 id) override;
        void mouseWheelMoved(const SDL_MouseWheelEvent& arg) override;

        bool injectMouseButtonPress(Uint8 button);
        bool injectMouseButtonRelease(Uint8 button);
        void injectMouseMove(float xMove, float yMove, float mouseWheelMove);
        void warpMouse();
        void warpMouseToWidget(MyGUI::Widget* widget);

        void setMouseLookEnabled(bool enabled);
        void cancelRagdollUse();
        void setGuiCursorEnabled(bool enabled) { mGuiCursorEnabled = enabled; }

        int getMouseMoveX() const { return mMouseMoveX; }
        int getMouseMoveY() const { return mMouseMoveY; }

    private:
        void cancelInvalidRagdollTapDrag();
        BindingsManager* mBindingsManager;
        SDLUtil::InputWrapper* mInputWrapper;
        SDL_Window* mWindow;
        bool mWindowHadFocus = false;

        float mGuiCursorX;
        float mGuiCursorY;
        int mMouseWheel;
        bool mMouseLookEnabled;
        bool mGuiCursorEnabled;
        bool mRagdollUseClaimed = false;
        bool mRagdollAnimationUseClaimed = false;
        bool mRagdollSwingHeld = false;
        bool mRagdollGuardHeld = false;
        // Right-click guard (OPENMW_RAGDOLL_RMB_GUARD=1): held, and how long; a
        // short tap keeps the guard up a moment (a timed parry).
        bool mRmbGuardHeld = false;
        float mRmbGuardHeldTime = 0.0f;
        float mRmbGuardTapLeft = 0.0f;
        // SHOVE-01: a second right press this soon after the last is a shove.
        float mRmbSincePress = 10.0f;
        // Right clicks this soon after the last one, once a shove started,
        // add power to it instead of raising the guard.
        float mRmbShoveBurst = 0.0f;
        // GUARD-03: an attack press while the guard is up lowers it until the
        // guard input is let go (OPENMW_RAGDOLL_GUARD_TO_ATTACK).
        bool mGuardYielded = false;
        bool mRagdollThrustClaimed = false;
        RagdollTapDrag mRagdollTapDrag;
        std::optional<MWWorld::Ptr> mRagdollTapDragWeapon;
        std::optional<bool> mPendingRagdollMouseUse;
        float mLastWarpX;
        float mLastWarpY;

        int mMouseMoveX;
        int mMouseMoveY;
    };
}
#endif

#pragma once

#include "Constants.h"

#include <frc/Joystick.h>
#include <map>

#define MAP_BB_XBOX(bb, xbox) { bb, XboxButtonTrans { xbox } }

namespace controllers {
    using constants::XboxButtons;
    using constants::XboxAxis;
    using constants::ButtonBoardButtons;

    struct XboxButtonTrans {
        XboxButtonTrans(XboxButtons button) {
            isAxis = false;
            xboxButton = button;
        }

        XboxButtonTrans(XboxAxis axis) {
            isAxis = true;
            xboxAxis = axis;
        }

        XboxButtonTrans(int pov) {
            this->pov = pov;
            isAxis = false;
        }

        int pov = -1;
        XboxButtons xboxButton;
        XboxAxis xboxAxis;
        bool isAxis;
    };

    namespace Constants {
        constexpr int kXbox = 1;
        constexpr int kButtonBoard = 2;

        // TODO: actually map all the buttons
        const std::map<ButtonBoardButtons, XboxButtonTrans> xboxTranslationMap = {
            MAP_BB_XBOX(ButtonBoardButtons::eSwitch, XboxButtons::eBack),
            MAP_BB_XBOX(ButtonBoardButtons::eShelf, XboxButtons::eButtonY),
            MAP_BB_XBOX(ButtonBoardButtons::eSingle, XboxButtons::eButtonX),
            MAP_BB_XBOX(ButtonBoardButtons::eGround, XboxAxis::eLeftTrigger),
            MAP_BB_XBOX(ButtonBoardButtons::eGroundTipped, XboxAxis::eRightTrigger),
            MAP_BB_XBOX(ButtonBoardButtons::eExtend, XboxButtons::eButtonA),
            MAP_BB_XBOX(ButtonBoardButtons::eRetract, XboxButtons::eButtonB),
            MAP_BB_XBOX(ButtonBoardButtons::eCube, XboxButtons::eButtonRB),
            MAP_BB_XBOX(ButtonBoardButtons::eShoot, XboxButtons::eButtonLB),
            MAP_BB_XBOX(ButtonBoardButtons::eHigh, 0),
            MAP_BB_XBOX(ButtonBoardButtons::eMid, 90),
            MAP_BB_XBOX(ButtonBoardButtons::eLow, 180),
            MAP_BB_XBOX(ButtonBoardButtons::ePanic, 270)
        };
    };

    class AuxController {
        public:
            /// @brief Returns from [-1,1] of the shoulder joystick.
            /// @return [-1,1] value of adjustment.
            double GetShoulderAdjust();

            /// @brief Returns from [-1,1] of the shoulder joystick.
            /// @return [-1,1] value of adjustment.
            double GetWristAdjust();

            /// @brief Returns whether a specific button is pressed.
            /// @param button The specific button to check (CAN BE XBOX OR BUTTON BOARD).
            /// @return True if the button is pressed, false otherwise.
            bool Get(constants::ButtonBoardButtons button);

            /// @brief Updates the object such that we know if we need to switch controllers.
            void Tick();
        private:
            frc::Joystick xbox { Constants::kXbox };
            frc::Joystick buttonboard { Constants::kButtonBoard };
            bool isXbox = false;
    };
}
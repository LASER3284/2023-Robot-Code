#include "AuxController.h"

using namespace controllers;

double AuxController::GetShoulderAdjust() {
    return isXbox ? xbox.GetRawAxis(constants::XboxAxis::eLeftAxisY) : 0.0;
}

double AuxController::GetWristAdjust() {
    return isXbox ? xbox.GetRawAxis(constants::XboxAxis::eRightAxisY) : 0.0;
}

void AuxController::Tick() {
    if (xbox.GetRawButtonPressed(constants::XboxButtons::eStart) || buttonboard.GetRawButtonPressed(constants::ButtonBoardButtons::eSwitch))
        isXbox = !isXbox;
}

bool AuxController::Get(constants::ButtonBoardButtons button) {
    if (isXbox) {
        // Get the translation of the BB to Xbox
        XboxButtonTrans trans = Constants::xboxTranslationMap.at(button);
        // If the translation is an axis, return true if the axis is pressed more than halfway
        // Else if the POV is valid, return true if the POV matches the intended POV
        // Else return true if the button indicated by the translation is pressed
        return trans.isAxis ? xbox.GetRawAxis(trans.xboxAxis) > 0.50 : (trans.pov != -1 ? (xbox.GetPOV() == trans.pov) : xbox.GetRawButtonPressed(trans.xboxButton));
    } else {
        return buttonboard.GetRawButtonPressed(button);
    }
}
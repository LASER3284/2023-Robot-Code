#pragma once

#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

namespace constants {
    constexpr static double Pi = 3.14159265358979323846;

    // Xbox Controller Button Assignments.
    enum XboxButtons 		{ eButtonA = 1, eButtonB, eButtonX, eButtonY, eButtonLB, eButtonRB, eBack, eStart, eButtonLS, eButtonRS};

    // Xbox Controller Axis Assignments.
    enum XboxAxis			{ eLeftAxisX = 0, eLeftAxisY, eLeftTrigger, eRightTrigger, eRightAxisX, eRightAxisY};

    // Logitech Flight Stick Button Assignments.
    enum LogButtons	 		{ eButtonTrigger = 1, eButton2, eButton3, eButton4, eButton5, eButton6, eButton7, eButton8, eButton9, eButton10, eButton11, eButton12};

    enum ButtonBoardButtons {
        eUp = 1,
        eOut = 2,
        eRight = 3, 
        eMid = 4, 
        eLeft = 5, 
        eDown = 6, 

        eConeMode = 8,
        eCubeMode = 9,

        eShelf = 10,
        eGround = 7, 
        eForkDown = 11,
        eForkUp = 12,
    };

    enum ButtonBoardAxis			{ eJoystickAxisX = 0, eJoyStickAxisY };

    static double mapScalarToRange(double inValue, double out_min, double out_max, double in_min = -1, double in_max = 1) {
        double x = (inValue - in_min) / (in_max - in_min);
        return out_min + (out_max - out_min) * x;
    }

    static units::revolutions_per_minute_t falconToRPM(double velocity, double gearRatio) {
        double motorRPM = velocity * (600.0 / 2048.0);
        double mechRPM = motorRPM / gearRatio;
        return units::revolutions_per_minute_t(mechRPM);
    }

    static units::meters_per_second_t falconToMPS(double velocity, units::meter_t circumference, double gearRatio) {
        units::revolutions_per_minute_t wheelRPM = falconToRPM(velocity, gearRatio);
        units::meters_per_second_t wheelMPS = units::meters_per_second_t( (wheelRPM.value() * circumference.value()) / 60);
        return wheelMPS;
    }

    static units::meter_t falconToMeters(double position, units::meter_t circumference, double gearRatio) {
        double m = position * (circumference.value() / (gearRatio * 2048));
        return units::meter_t(m);
    }

    static units::degree_t falconToDeg(double position, double gearRatio) {
        double m = position * (gearRatio * 2048);
        return units::degree_t(m);
    }
}
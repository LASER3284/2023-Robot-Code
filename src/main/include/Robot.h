// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <string>
#include <tuple>
#include <frc/Errors.h>
#include <rev/CANSparkMaxLowLevel.h>

#include "FieldConstants.h"
#include "SwerveModule.h"
#include "Drive.h"
#include "Lights.h"
#include "Shoulder.h"
#include "Intake.h"
#include "Arm.h"
#include "Wrist.h"
#include "Kinematics.h"
#include <deque>

class Robot : public frc::TimedRobot {
    public:
        Robot();
        ~Robot();
        void RobotInit() override;
        void RobotPeriodic() override;
        void AutonomousInit() override;
        void AutonomousPeriodic() override;
        void TeleopInit() override;
        void TeleopPeriodic() override;
        void DisabledInit() override;
        void DisabledPeriodic() override;
        void TestInit() override;
        void TestPeriodic() override;
        void SimulationInit() override;
        void SimulationPeriodic() override;

    private:
        frc::Joystick driveController { 0 };
        frc::GenericHID auxController { 2 };

        /// @brief A sendable chooser / dropdown for selecting the current autonomous trajectory
        frc::SendableChooser<std::string> m_chooser;

        /// @brief A string representing the currently selected ""human"" trajectory name
        std::string currentAutonomousState;

        /// @brief The subsystem for handling all of the swerve drive train tasks
        drive::Drive drivetrain { &driveController };

        /// @brief A basic subsystem for handling the WS128b LED strips
        lights::LightHandler lighthandler;

        /// @brief The subsystem for handling/controlling shoulder movements
        shoulder::Shoulder shoulder;

        /// @brief The subsystem for handling/controlling arm extensions
        arm::Arm arm;

        /// @brief The system that handles the intake and whether it is cone or cube.
        intake::Intake intake;

        /// @brief The subsystem that handles wrist motion/controls
        wrist::Wrist wrist;

        /// @brief A map filled with the "human friendly" path name and the actual pathplanner file name saved in the deploy folder
        const std::map<std::string, std::string> mTrajectoryMap {
            { "Test Path", "Test Path" },

            { "Mobile Cone", "MobileCone"},

            { "Cone Cube - Balance", "ConeCubeBalance" },
            { "Cone Cube", "ConeCubeNoBalance" },

            { "Far Cone Cube - Balance", "FarConeCubeBalance"},
            { "Far Cone Cube", "FarConeCubeNoBalance"},

            { "Far Triple Score", "FarTripleScore"}
        };

        /// @brief An object that represents the locations of all the field elements
        constants::FieldConstants fieldConstants;

        /// @brief A boolean to track whether or not the button timer has been started
        bool buttonTimerStarted = false;

        /// @brief A timer object used to track delays between button board presses
        frc::Timer buttonTimer;

        /// @brief A boolean tracking if the robot is currently trying to align to the current grid position
        bool aligningToGrid = false;

        /// @brief The currently targeted scoring location to rotate the arm to
        /// This only applies when aligningToGrid is currently true
        frc::Translation3d targetedScoringLocation;

        /// @brief What button was previously pressed
        std::deque<constants::ButtonBoardButtons> previousButtons;

        /// @brief A timer object used for timing the rumble of the main driving controller
        frc::Timer rumbleTimer;
};

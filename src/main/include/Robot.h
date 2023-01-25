// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include "SwerveModule.h"
#include "Drive.h"
#include "Lights.h"

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

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
        frc::Joystick drivecontroller { 0 };

        /// @brief A sendable chooser / dropdown for selecting the current autonomous trajectory
        frc::SendableChooser<std::string> m_chooser;

        /// @brief A string representing the currently selected ""human"" trajectory name
        std::string currentAutonomousState;

        /// @brief The subsystem for handling all of the swerve drive train tasks
        drive::Drive drivetrain { &drivecontroller };

        /// @brief A basic subsystem for handling the WS128b LED strips
        lights::LightHandler lighthandler;

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
};

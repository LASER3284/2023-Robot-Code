// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include "SwerveModule.h"
#include "Drive.h"

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
        frc::SendableChooser<std::string> m_chooser;
        std::string currentAutonomousState;
        frc::Joystick drivecontroller { 0 };
        drive::Drive drivetrain { &drivecontroller };

        const std::map<std::string, std::vector<std::string>> mTrajectoryMap {
            { "Test Path", { "Test Path" }}
        };
};

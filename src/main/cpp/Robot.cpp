// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

Robot::Robot() : frc::TimedRobot() {
}

Robot::~Robot() {
}

void Robot::RobotInit() {
    m_chooser.SetDefaultOption("Test Path", "Test Path");

    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    // Add in a boolean for field relative/oriented drive.
    frc::SmartDashboard::PutBoolean("bFieldOriented", true);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
    drivetrain.LogEncoders();
}

void Robot::AutonomousInit() {
    currentAutonomousState = m_chooser.GetSelected();
    fmt::print("Currently selected auto path: {}\n", currentAutonomousState);
    drivetrain.SetTrajectory(currentAutonomousState);
}

void Robot::AutonomousPeriodic() {
    bool trajectoryCompleted = drivetrain.FollowTrajectory();
    if(trajectoryCompleted) {
        if(currentAutonomousState == "Test Path") {
            // TODO: Figure something to do here
        }
    }
}

void Robot::TeleopInit() {
    drivetrain.SetJoystick(true);
}

void Robot::TeleopPeriodic() {
    drivetrain.Tick();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
    drivetrain.SetJoystick(false);
}

void Robot::TestPeriodic() {
    drivetrain.Tick();
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif

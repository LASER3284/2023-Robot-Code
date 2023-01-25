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
    for (auto const& [human_name, path_file] : mTrajectoryMap)
    {
        m_chooser.AddOption(human_name, path_file);
    }
    m_chooser.SetDefaultOption("Test Path", "Test Path");

    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
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
    drivetrain.SetTrajectory(currentAutonomousState, true);
}

void Robot::AutonomousPeriodic() {    
    // All of these autos start with a preloaded cone and immediately score it
    if(currentAutonomousState == "Mobile Cone" || currentAutonomousState == "Cone Cube - Balance" || 
        currentAutonomousState == "Cone Cube"  || currentAutonomousState == "Far Cone Cube - Balance" ||
        currentAutonomousState == "Far Triple Score") {
        // TODO: Score the preloaded cone and return if we're still holding a cone
    }

    const drive::AutonomousState trajectoryCompleted = drivetrain.FollowTrajectory();
    SmartDashboard::PutBoolean("bAtStopPoint", trajectoryCompleted.bAtStopPoint);
    SmartDashboard::PutBoolean("bCompletedPath", trajectoryCompleted.bCompletedPath);

    // By default, if the trajectory is completed, we might as well stop the drive train.
    bool bForceStop = trajectoryCompleted.bCompletedPath;
    if(trajectoryCompleted.bAtStopPoint) {
        // All of these autos end up picking up the preloaded cube
        if((currentAutonomousState == "Cone Cube" || currentAutonomousState == "Cone Cube - Balance") || 
            (currentAutonomousState == "Far Cone Cube" || currentAutonomousState == "Far Cone Cube - Balance") ||
            (currentAutonomousState == "Far Triple Score")) {
            if(trajectoryCompleted.currentStopIndex == 1) {
                // TODO: Pick up the staged cube
            }

            else if(trajectoryCompleted.currentStopIndex == 2) {
                // TODO: Score the loaded cube
            }
        }

        if(currentAutonomousState == "Far Triple Score") {
            if(trajectoryCompleted.currentStopIndex == 3) {
                // TODO: Pickup the staged cone
            }
            else if(trajectoryCompleted.currentStopIndex == 4) {
                // TODO: Place the cone
            }
        }
    }

    // If we're fully finished with the path, force stop the drive train in order to stop moving
    if(bForceStop) {
        drivetrain.ForceStop();
    }
}

void Robot::TeleopInit() {
    lighthandler.SetColor(frc::Color::kPurple);
    drivetrain.SetJoystick(true);
}

void Robot::TeleopPeriodic() {
    drivetrain.Tick();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
    lighthandler.Rainbow();
}

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

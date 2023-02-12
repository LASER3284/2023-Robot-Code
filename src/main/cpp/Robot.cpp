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
    drivetrain.SetJoystick(false);

    shoulder.ToggleControl(false);
    arm.ToggleControl(false);
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
    drivetrain.Tick();
    drivetrain.LogEncoders();

    shoulder.Tick();
    arm.Tick();
}

void Robot::AutonomousInit() {
    currentAutonomousState = m_chooser.GetSelected();
    fmt::print("Currently selected auto path: {}\n", currentAutonomousState);
    drivetrain.SetTrajectory(currentAutonomousState, true);

    shoulder.ToggleControl(true);
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
        else if(currentAutonomousState == "Far Triple Score") {
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
    lighthandler.SetColor(frc::Color::kRed);
    drivetrain.SetJoystick(true);

    shoulder.ToggleControl(true);
    arm.ToggleControl(true);
}

void Robot::TeleopPeriodic() {
    const auto currentRobotPose = drivetrain.GetPose();

    // Iterate over all of the grid locations in order to determine what grid that the robot is currently in
    int currentGrid = 0;
    for(const auto &gridLocations : fieldConstants.gridLocations) {
        const auto bottomLeft = std::get<0>(gridLocations);
        const auto topRight = std::get<1>(gridLocations);

        if((currentRobotPose.X() >= bottomLeft.X() && currentRobotPose.X() <= topRight.X()) && (currentRobotPose.Y() >= bottomLeft.Y() && currentRobotPose.Y() <= topRight.Y())) {
            break;
        }
        
        // Move onto the next grid if we aren't in the right one
        currentGrid++;
    }

    SmartDashboard::PutNumber("currentGrid", currentGrid);

    // TODO: Implement automatic scoring
    if(auxController.GetRawButton(constants::ButtonBoardButtons::eConeMode)) {
        lighthandler.SetColor(frc::Color::kOrange);
        intake.ConeMode();
    }
    else if (auxController.GetRawButton(constants::ButtonBoardButtons::eCubeMode))
    {
        lighthandler.SetColor(frc::Color::kPurple);
        intake.CubeMode();
    }
    else if(auxController.GetRawButton(constants::ButtonBoardButtons::eOut)) {
        fmt::print("Spitting out element...");
        intake.Spit();
    }
    else if(intake.HasElement()) {
        lighthandler.SetColor(frc::Color::kGreen);
        intake.Stop();
        driveController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
    }
    else {
        intake.Stop();
        driveController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
    }

    // We're using a deque in order to store the most recent (0.25sec) buttons pressed in order to determine the scoring location

    if(auxController.GetRawButtonPressed(constants::ButtonBoardButtons::eUp)) 
        previousButtons.push_front(constants::ButtonBoardButtons::eUp);
    if(auxController.GetRawButtonPressed(constants::ButtonBoardButtons::eDown))
        previousButtons.push_front(constants::ButtonBoardButtons::eDown);
    if(auxController.GetRawButtonPressed(constants::ButtonBoardButtons::eMid))
        previousButtons.push_front(constants::ButtonBoardButtons::eMid);
    if(auxController.GetRawButtonPressed(constants::ButtonBoardButtons::eRight))
        previousButtons.push_front(constants::ButtonBoardButtons::eRight);
    if(auxController.GetRawButtonPressed(constants::ButtonBoardButtons::eLeft))
        previousButtons.push_front(constants::ButtonBoardButtons::eLeft);

    if(previousButtons.size() > 1) {
        buttonTimer.Restart();
    }
    
    if(buttonTimer.AdvanceIfElapsed(0.25_s)) {
        // If the user still hasn't pressed a second button within 0.25 seconds, reset the deque
        if(previousButtons.size() <= 1) {
            previousButtons.clear();
            buttonTimer.Stop();
            buttonTimer.Reset();
        }
        // If the user has pressed atleast a second button within the 0.25 seconds, start auto placing / moving to the grid
        else {
            fmt::print("Current grid: %d :: Selecting scoring location", currentGrid);
        }
    }

    // Use the joysticks to manually control the arm
    double manualOverride = frc::ApplyDeadband(auxController.GetRawAxis(constants::ButtonBoardAxis::eJoystickAxisX), 0.10);
    if(manualOverride != 0) {
        double factor = 0.25;
        if(manualOverride > 0) {
            factor = 0.15;
        }
        
        shoulder.ManualControl(wpi::sgn(manualOverride) * factor);
        shoulder.AdjustFeedforward(-0.3_V);
    }
    else {
        shoulder.ManualControl(0.0);
        shoulder.AdjustFeedforward(-0.3_V);
    }

    double wristOverride = frc::ApplyDeadband(auxController.GetRawAxis(constants::ButtonBoardAxis::eJoyStickAxisY), 0.10);
    if(wristOverride != 0) {
        double factor = 0.25;
        if(manualOverride > 0) {
            factor = 0.15;
        }

        wrist.ManualControl(wpi::sgn(wristOverride) * factor);
    }
    else {
        wrist.ManualControl(0.0);
    }

    // Use the ""fork"" up/down buttons in order to control the arm manually up/down
    if(auxController.GetRawButton(constants::ButtonBoardButtons::eForkUp)) {
        arm.ManualControl(0.5);
    }
    else if(auxController.GetRawButton(constants::ButtonBoardButtons::eForkDown)) {
        arm.ManualControl(-0.5);
    }
    else {
        arm.ManualControl(0.0);
    }
    
    arm.Tick();

    //shoulder.AdjustFeedforward(kinematics::Kinematics::CalculateShoulderkG(arm.GetPosition(), shoulder.GetRotation()));
    shoulder.Tick();
    wrist.Tick();

    frc::SmartDashboard::PutNumber("intakeCurrent_a", intake.GetFilteredCurrent().value());
}

void Robot::DisabledInit() {
    // For easier testing, when the robot first gets disabled, reinitialize the field constants (since it can change due to alliance color)
    fieldConstants.Initialize();
}

void Robot::DisabledPeriodic() {
    lighthandler.Rainbow();
}

void Robot::TestInit() {
    drivetrain.SetJoystick(false);
    arm.ToggleControl(false);
    shoulder.ToggleControl(false);
    rev::CANSparkMaxLowLevel::EnableExternalUSBControl(true);
}

void Robot::TestPeriodic() {
    arm.Tick();
    shoulder.Tick();
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif

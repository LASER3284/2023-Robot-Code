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
    m_chooser.SetDefaultOption("Autonomous Idle", "Autonomous Idle");

    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    drivetrain.SetJoystick(false);

    shoulder.ToggleControl(false);
    arm.ToggleControl(false);

    frc::SmartDashboard::PutBoolean("extensionLimits", true);
    shoulder.SetRotationGoal(shoulder.GetRotation());
    arm.SetPositionGoal(arm.GetPosition());
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

    shoulder.Tick(arm.GetPosition());
    arm.Tick();
    wrist.Tick();
}

void Robot::AutonomousInit() {
    currentAutonomousState = m_chooser.GetSelected();
    fmt::print("Currently selected auto path: {}\n", currentAutonomousState);
    if(currentAutonomousState == "Autonomous Idle") { 
        drivetrain.ForceVisionPose(); 
    }
    else { 
        drivetrain.SetTrajectory(currentAutonomousState, true); 
    }

    shoulder.ToggleControl(true);
}

void Robot::AutonomousPeriodic() {
    units::degree_t shoulderGoal = shoulder.GetRotation();
    units::meter_t armGoal = arm.GetPosition();

    if(currentAutonomousState == "Autonomous Idle") {
        drivetrain.ForceStop();
    }
    
    // All of these autos start with a preloaded cone and immediately score it
    if(currentAutonomousState == "MobileCone" || currentAutonomousState == "ConeCubeBalance" || 
        currentAutonomousState == "ConeCube"  || currentAutonomousState == "FarConeCubeBalance" ||
        currentAutonomousState == "FarTripleScore") {
        // TODO: Score the preloaded cone and return if we're still holding a cone
    }

    const drive::AutonomousState trajectoryCompleted = drivetrain.FollowTrajectory();
    SmartDashboard::PutBoolean("bAtStopPoint", trajectoryCompleted.bAtStopPoint);
    SmartDashboard::PutBoolean("bCompletedPath", trajectoryCompleted.bCompletedPath);

    // By default, if the trajectory is completed, we might as well stop the drive train.
    bool bForceStop = trajectoryCompleted.bCompletedPath;
    if(trajectoryCompleted.bAtStopPoint) {
        // All of these autos end up picking up the preloaded cube
        if((currentAutonomousState == "ConeCube" || currentAutonomousState == "ConeCubeBalance") || 
            (currentAutonomousState == "FarConeCube" || currentAutonomousState == "FarConeCubeBalance") ||
            (currentAutonomousState == "FarTripleScore")) {
            if(trajectoryCompleted.currentStopIndex == 1) {
                // TODO: Pick up the staged cube
            }

            else if(trajectoryCompleted.currentStopIndex == 2) {
                // TODO: Score the loaded cube
            }
        }
        else if(currentAutonomousState == "FarTripleScore") {
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

    arm.SetPositionGoal(armGoal);
    shoulder.SetRotationGoal(shoulderGoal);
}

void Robot::TeleopInit() {
    lighthandler.SetColor(frc::Color::kRed);
    drivetrain.SetJoystick(true);

    shoulder.ToggleControl(true);
    arm.ToggleControl(true);

    // On teleop init, set the wrist to go to the current wrist angle so that way its not trying to move to a previous setpoint
    wrist.SetRotationGoal(wrist.GetRotation());
    shoulder.SetRotationGoal(shoulder.GetRotation());
    arm.SetPositionGoal(arm.GetPosition());
}

void Robot::TeleopPeriodic() {
    units::degree_t shoulderGoal = shoulder.GetRotation();
    units::meter_t armGoal = arm.GetPosition();

    const auto currentRobotPose = drivetrain.GetPose();

    // Iterate over all of the grid locations in order to determine what grid that the robot is currently in
    int currentGrid = 0;
    bool bInsideCommunity = false;
    for(const auto &gridLocations : fieldConstants.gridLocations) {
        const auto bottomLeft = std::get<0>(gridLocations);
        const auto topRight = std::get<1>(gridLocations);

        // Check whether or not the robot is currently inside the square for the given grid
        if(
            (currentRobotPose.X() >= bottomLeft.X() && currentRobotPose.Y() >= bottomLeft.Y()) && 
            (currentRobotPose.X() <= topRight.X() && currentRobotPose.Y() <= topRight.Y())
        ) {
            bInsideCommunity = true;
            break;
        }

        currentGrid++;
    }

    if(!bInsideCommunity) currentGrid = -1;

    SmartDashboard::PutNumber("currentGrid", currentGrid);
    SmartDashboard::PutBoolean("bInsideCommunity", bInsideCommunity);

    // TODO: Implement automatic scoring and positioning
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
        lighthandler.SetColor(frc::Color::kRed);
        intake.Spit();
    }
    else if(intake.HasElement()) {
        // Set the lights to green whenever we've picked something up
        lighthandler.SetColor(frc::Color::kGreen);

        // Start spinning the intake *slightly* in order to hold objects
        intake.Hold();

        // Start a small timer (if it's not been started) in order to time the duration of controller rumbling
        rumbleTimer.Start();

        // After 2 seconds, we want to stop rumbling the controller in order to avoid annoying the driver.
        driveController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, rumbleTimer.HasElapsed(2_s) ? 0.0 : 1.0);
    }
    else {
        rumbleTimer.Reset();
        rumbleTimer.Stop();
        intake.Stop();
        // Make sure to stop rumbling the controller
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

    if(previousButtons.size() > 1 && !buttonTimerStarted) {
        fmt::print("Restarting button timer\n");

        buttonTimer.Restart();
        buttonTimerStarted = true;
        aligningToGrid = false;
        locationSelected = false;
    }
    
    frc::SmartDashboard::PutBoolean("locationSelected", locationSelected);
    frc::SmartDashboard::PutBoolean("aligningToGrid", aligningToGrid);

    if(locationSelected) {
        if(bInsideCommunity) {
            if(!aligningToGrid && driveController.GetRawButtonPressed(constants::XboxButtons::eButtonLB)) {
                // goalPose is the translation that the robot needs to drive to in order to be in the proper position
                frc::Translation2d goalPose = frc::Translation2d();

                // Set the goal pose based on the buttons pressed
                int offset = 0;
                if(pressedButtons[constants::ButtonBoardButtons::eLeft]) {
                    goalPose = fieldConstants.lowLocations[0 + (currentGrid * 3)].location.ToTranslation2d();
                }
                else if(pressedButtons[constants::ButtonBoardButtons::eMid]) {
                    offset = 1;
                    goalPose = fieldConstants.lowLocations[1 + (currentGrid * 3)].location.ToTranslation2d();
                }
                else if(pressedButtons[constants::ButtonBoardButtons::eRight]) {
                    offset = 2;
                    goalPose = fieldConstants.lowLocations[2 + (currentGrid * 3)].location.ToTranslation2d();
                }

                if(pressedButtons[constants::ButtonBoardButtons::eUp]) {
                    targetedScoringLocation = fieldConstants.highLocations[offset + (currentGrid * 3)].location;
                }
                else if(pressedButtons[constants::ButtonBoardButtons::eMid]) {
                    targetedScoringLocation = fieldConstants.midLocations[offset + (currentGrid * 3)].location;
                }
                else if(pressedButtons[constants::ButtonBoardButtons::eDown]) {
                    targetedScoringLocation = fieldConstants.lowLocations[offset + (currentGrid * 3)].location;
                }

                const units::inch_t alignmentOffset = 14.5_in;

                // Add/subtract an extra half robot width in order to account for robot size.
                // If we're on the red alliance, we actually want to subtract this difference due to the lack of field symmetry.
                goalPose = goalPose + frc::Translation2d(
                    alignmentOffset * (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ? 1 : -1), 
                    0_in
                );
                
                // Set the drive train to actually start moving to the goal pose
                // TODO: Potentially automatically rotate the robot to face properly
                // It might be best to keep the robot facing the same way it currently is to avoid hitting unknown objects (or the field)
                drivetrain.SetTrajectory(frc::Pose2d(goalPose, drivetrain.GetPose().Rotation()));
                
                aligningToGrid = true;
            }
            else if(aligningToGrid) {
                const drive::AutonomousState state = drivetrain.FollowTrajectory();
                if(!drivetrain.GetTrajectoryActive()) {
                    fmt::print("Trajectory cancelled... Skipping colors\n");
                    // If the trajectory got cancelled, flip the colors to red
                    lighthandler.SetColor(frc::Color::kRed);

                    buttonTimerStarted = false;
                    aligningToGrid = false;
                    locationSelected = false;
                    previousButtons.clear();
                }
                else if(state.bCompletedPath) {
                    // Flash the lights white whenever we've auto lined up to the scoring position.
                    lighthandler.SetColor(frc::Color::kWhite);
                    
                    buttonTimerStarted = false;
                    aligningToGrid = false;
                    locationSelected = false;
                    previousButtons.clear();

                    drivetrain.ForceStop();
                    const frc::Translation2d drivePose = drivetrain.GetPose().Translation();
                    const auto state = kinematics::Kinematics::GetKinematicState(
                        !intake.IsCubeMode(), 
                        targetedScoringLocation,
                        drivePose
                    );

                    //shoulder.SetRotationGoal(state.shoulderAngle);
                }
            }
        }
    }
    else if(buttonTimer.AdvanceIfElapsed(0.1_s)) {        
        // Force the deque to only have the first 2 elements (buttons) in it
        previousButtons.resize(2);

        // Use an array that stores the values of all of the pressed buttons to avoid triple checking the same "if button = left" multiple times and making the code worse.
        for(int i = 0; i < 12; i++) {
            pressedButtons[i] = std::find(previousButtons.begin(), previousButtons.end(), (constants::ButtonBoardButtons)i) != previousButtons.end();
        }

        buttonTimer.Stop();
        aligningToGrid = false;
        locationSelected = true;
    }

    // Use the joysticks to manually control the arm
    double shoulderOverride = frc::ApplyDeadband(auxController.GetRawAxis(constants::ButtonBoardAxis::eJoystickAxisX), 0.10);
    if(shoulderOverride != 0) {
        double factor = 0.25;
        if(shoulderOverride > 0) {
            factor = 0.2;
        }
        
        shoulder.ManualControl(wpi::sgn(shoulderOverride) * factor);
        shoulder.AdjustFeedforward(0_V);
    }
    else {
        shoulder.ManualControl(0.0);
    }

    double wristOverride = frc::ApplyDeadband(auxController.GetRawAxis(constants::ButtonBoardAxis::eJoyStickAxisY), 0.10);
    if(wristOverride != 0) {
        double factor = 0.3;
        wrist.ManualControl(wpi::sgn(wristOverride) * factor);
    }
    else {
        wrist.ManualControl(0.0);
    }

    // Use the ""fork"" up/down buttons in order to control the arm manually up/down
    if(auxController.GetRawButton(constants::ButtonBoardButtons::eForkUp)) arm.ManualControl(0.5);
    else if(auxController.GetRawButton(constants::ButtonBoardButtons::eForkDown)) arm.ManualControl(-0.5);
    else {
        arm.ManualControl(0.0);
    }
    
    // TODO: Implement extension limits
    frc::SmartDashboard::PutBoolean("cubeMode", intake.IsCubeMode());
    const kinematics::KinematicState kinematicState = { arm.GetPosition(), shoulder.GetRotation(), wrist.GetRotation() };
    if(frc::SmartDashboard::GetBoolean("extensionLimits", true)) {
        // Check whether or not the current kinematic state is illegal
        if(!kinematics::Kinematics::IsValid(!intake.IsCubeMode(), kinematicState)) {
            FRC_ReportError(warn::Warning, "[Robot] Running into arm extension limits... Retracting arm in");
            arm.ManualControl(0.0);
            armGoal = arm.GetPosition() - 1_in;
        }
    }

    frc::SmartDashboard::PutNumber("shoulderOverride", shoulderOverride);
    // B is the panic button, if we've hit the panic button, move the arm up into the air
    if(driveController.GetRawButton(constants::XboxButtons::eButtonB)) {
        shoulderGoal = 90_deg;
        armGoal = 0_m;
    }

    if(auxController.GetRawButton(constants::ButtonBoardButtons::eGround)) {
        armGoal = 0_m;        
        shoulderGoal = -10_deg;
    }

    shoulder.SetRotationGoal(shoulderGoal);    
    arm.SetPositionGoal(armGoal);

    if(wristOverride != 0) {
        wrist.SetRotationGoal(wrist.GetRotation());
    }

    frc::SmartDashboard::PutNumber("intakeCurrent_a", intake.GetFilteredCurrent().value());
}

void Robot::DisabledInit() {
    // For easier testing, when the robot first gets disabled, reinitialize the field constants (since it can change due to alliance color)
    fieldConstants.Initialize();

    drivetrain.SetJoystick(false);
    arm.ToggleControl(false);
    shoulder.ToggleControl(false);
}

void Robot::DisabledPeriodic() {
    lighthandler.Rainbow();
}

void Robot::TestInit() {
    drivetrain.SetJoystick(false);
    arm.ToggleControl(false);
    shoulder.ToggleControl(false);
    
    rev::CANSparkMaxLowLevel::EnableExternalUSBControl(true);

    // In test mode, force the drive train pose to the current estimated robot pose
    drivetrain.ForceVisionPose();
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif

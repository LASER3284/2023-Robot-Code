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
    for (auto const traj : mTrajectoryMap)
    {
        m_chooser.AddOption(traj.humanName, traj);
    }
    m_chooser.SetDefaultOption("Autonomous Idle", { "Autonomous Idle", "Autonomous Idle" });
    
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

    const auto currentRobotPose = drivetrain.GetPose();

    // Iterate over all of the grid locations in order to determine what grid that the robot is currently in
    currentGrid = 0;
    bInsideCommunity = false;
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

    shoulder.Tick(arm.GetPosition());
    arm.Tick();
    wrist.Tick(shoulder.GetRotation());

    frc::SmartDashboard::PutNumber("autoTimer_s", autoTimer.Get().value());
    frc::SmartDashboard::PutNumber("intakeCurrent_a", intake.GetFilteredCurrent().value());
}

void Robot::AutonomousInit() {
    // When connected to the FMS, the FMS may not have sent the correct driver station alliance to the robot
    // so we should reinitialize the field constants just to be safe.
    fieldConstants.Initialize();

    currentAutonomousState = m_chooser.GetSelected().pathName;
    fmt::print("Currently selected auto path: {}\n", currentAutonomousState);
    if(currentAutonomousState == "Autonomous Idle") { 
        drivetrain.ForceVisionPose(); 
    }
    else { 
        drivetrain.SetTrajectory(m_chooser.GetSelected(), true);
    }
    
    // Force the wheels to be facing forward in order to have more accurate auto driving and pathplanning
    drivetrain.ForceForward();

    shoulder.ToggleControl(true);
    arm.ToggleControl(true);

    bDriveTrainStopped = false;
    bHasProcessedStartingAction = false;
    bHasAutoBalanced = false;
    bHasStartedBalancing = false;

    wrist.RefreshController();
    shoulder.RefreshController();
    arm.RefreshController();
    
    autoTimer.Stop();
    autoTimer.Reset();
}

void Robot::AutonomousPeriodic() {
    frc::SmartDashboard::PutString("currentAutonomousState", currentAutonomousState);
    frc::SmartDashboard::PutBoolean("bHasStartedBalancing", bHasStartedBalancing);
    frc::SmartDashboard::PutBoolean("bHasAutoBalanced", bHasAutoBalanced);

    units::degree_t shoulderGoal = shoulder.GetRotation();
    units::meter_t armGoal = arm.GetPosition();
    units::degree_t wristGoal = wrist.GetLastSetpoint();

    bool bForceStop = false;
    if(currentAutonomousState == "Autonomous Idle") {
        drivetrain.ForceStop();
        // In idle mode, update the expected odometry pose to be based on the estimated apriltag position.
        // This isn't as accurate as starting via using the known robot pose but we don't know the exact pose of the robot when in idle mode.
        drivetrain.ForceVisionPose();
    }
    
    // All of these autos start with a preloaded cone and immediately score it
    if(!bHasProcessedStartingAction) {
        // Force the wheels to be facing forward in order to have more accurate auto driving and pathplanning
        drivetrain.ForceForward();

        // Autos that do nothing at all at the start
        if(currentAutonomousState == "MidBalance" || currentAutonomousState == "FarMobile" || currentAutonomousState == "Mobile" || currentAutonomousState == "TestPath") { 
            bHasProcessedStartingAction = true; 
        }
        else if(currentAutonomousState == "MobileCone" || currentAutonomousState == "FarMobileCone" || currentAutonomousState == "FarMidConeBalance" || currentAutonomousState == "FarConeCone"  || currentAutonomousState == "FarConeCube" || currentAutonomousState == "FarMobileConeBalance" || currentAutonomousState == "FarMobileConeCubeBalance") {
            shoulderGoal = 160_deg;
            armGoal = 0.287_m;
            wristGoal = 10_deg;

            if(units::math::abs(shoulder.GetRotation() - shoulderGoal) <= 7.25_deg && units::math::abs(arm.GetPosition() - armGoal) <= 2.125_in) {
                autoTimer.Start();
                autoTimerRunning = true;
            }

            if(autoTimer.HasElapsed(0.5_s)) {
                bHasProcessedStartingAction = true;
                autoTimer.Reset();
                autoTimer.Stop();
            }
            else if(autoTimer.HasElapsed(0.25_s)) {
                intake.Stop();
                shoulderGoal = 0_deg;
                armGoal = 0_m;
                wristGoal = 0_deg;
            }
            else if(autoTimer.HasElapsed(0.125_s)) {
                intake.CubeMode();
            }
        }
    }
    else if(!bDriveTrainStopped) {
        const drive::AutonomousState trajectoryCompleted = drivetrain.FollowTrajectory();
        SmartDashboard::PutBoolean("bAtStopPoint", trajectoryCompleted.bAtStopPoint);
        SmartDashboard::PutBoolean("bCompletedPath", trajectoryCompleted.bCompletedPath);

        // By default, if the trajectory is completed, we might as well stop the drive train.
        bool bForceStop = trajectoryCompleted.bCompletedPath;
        if(!trajectoryCompleted.bCompletedPath && !trajectoryCompleted.bAtStopPoint) {
            // All of these autos grab the preloaded cone so we want to be moving the arm pre-emptively.
            if(currentAutonomousState == "FarConeCone") {
                if(trajectoryCompleted.currentStopIndex == 0) {
                    armGoal = 0_m;
                    shoulderGoal = -31_deg;
                    wristGoal = 24_deg;
                    intake.ConeMode();
                }
                else if(trajectoryCompleted.currentStopIndex == 1) {                    
                    // Avoid extending the arm out at first in order to avoid crashing into things with the arm accidentally.
                    shoulderGoal = 90_deg;
                    wristGoal = 10_deg;
                    intake.Hold();
                }
            }
            // Grab the prestaged cube instead of the cone
            else if(currentAutonomousState == "FarConeCube") {
                if(trajectoryCompleted.currentStopIndex == 0) {
                    armGoal = 0_m;
                    shoulderGoal = -31_deg;
                    wristGoal = 21_deg;
                    intake.CubeMode();
                }
                // Rotate / spin the arm for scoring the cube
                else if(trajectoryCompleted.currentStopIndex == 1) {                    
                    shoulderGoal = 135_deg;
                    armGoal = 0.0_m;
                    wristGoal = 18_deg;
                }
            }
            else if(currentAutonomousState == "FarMobileConeCubeBalance") {
                if(trajectoryCompleted.currentStopIndex == 0) {
                    armGoal = 0_m;
                    shoulderGoal = -31_deg;
                    wristGoal = 21_deg;
                    intake.CubeMode();
                }
                else if(trajectoryCompleted.currentStopIndex == 1) {
                    shoulderGoal = 45_deg;
                    armGoal = 0_m;
                    wristGoal = 0_deg; 
                }
            }
            // Spin the arm around in order to shift the center of mass 
            else if(currentAutonomousState == "FarMobileConeBalance") {
                shoulderGoal = 45_deg;
                armGoal = 0_m;
                wristGoal = 0_deg;
            }
        }
        else if(trajectoryCompleted.bAtStopPoint) {
            autoTimer.Start();
            
            // Score the prestaged cube
            if(currentAutonomousState == "FarConeCube") {
                if(trajectoryCompleted.currentStopIndex == 0) {
                    shoulderGoal = 0_deg;
                    if(autoTimer.HasElapsed(0.5_s)) {
                        drivetrain.StartNextTrajectory();
                    }
                    else {
                        drivetrain.ForceStop();
                    }
                }
                else if(trajectoryCompleted.currentStopIndex == 1) {
                    shoulderGoal = 160_deg;
                    armGoal = 0.0_m;
                    wristGoal = 18_deg;

                    bDriveTrainStopped = true;
                    intake.Shoot();
                }
            }
            // These autos pick up / score the preloaded cone
            else if(currentAutonomousState == "FarConeCone") {
                if(trajectoryCompleted.currentStopIndex == 0) {
                    shoulderGoal = 0_deg;
                    if(autoTimer.HasElapsed(0.5_s) || intake.HasElement()) {
                        drivetrain.StartNextTrajectory();
                    }
                    else {
                        drivetrain.ForceStop();
                    }
                }
                else if(trajectoryCompleted.currentStopIndex == 1) {
                    shoulderGoal = 160_deg;
                    armGoal = 0.285_m;
                    wristGoal = 10_deg;

                    if(units::math::abs(shoulder.GetRotation() - shoulderGoal) <= 4.25_deg && units::math::abs(arm.GetPosition() - armGoal) <= 1.5_in) {
                        autoTimer.Start();
                        autoTimerRunning = true;
                    }

                    if(autoTimer.HasElapsed(2.5_s)) {
                        bDriveTrainStopped = true;
                    }
                    else if(autoTimer.HasElapsed(0.5_s)) {
                        intake.Stop();
                        shoulderGoal = 0_deg;
                        armGoal = 0_m;
                        wristGoal = 0_deg;
                    }
                    else if(autoTimer.HasElapsed(0.25_s)) {
                        intake.CubeMode();
                    }
                }
            }
            else if(currentAutonomousState == "FarMobileConeCubeBalance") {
                if(trajectoryCompleted.currentStopIndex == 0) {
                    drivetrain.StartNextTrajectory();
                }
                else if(trajectoryCompleted.currentStopIndex == 1) {
                    bDriveTrainStopped = true;
                }
            }
            // Autos that do nothing at all and just force stop
            else if(currentAutonomousState == "MidBalance" || currentAutonomousState == "TestPath") {
                // For the balancing autos, we want to ""force stop"" but just to avoid logic issues with force stopping the drive train before we're ready.
                bDriveTrainStopped = true;
            }
            else if(bForceStop) {
                bDriveTrainStopped = true;
                drivetrain.ForceStop();
            }
        }

    }
    else if(currentAutonomousState == "MidBalance" || currentAutonomousState == "FarMobileConeBalance"  || currentAutonomousState == "FarMidConeBalance"  || currentAutonomousState == "TestPath" || currentAutonomousState == "FarMobileConeCubeBalance") {
        // Move the arm down and start driving forward in order to shift the CoG towards the charge station
        shoulderGoal = 25_deg;
        armGoal = 0_m;
        wristGoal = 0_deg;
        bool bFlipPowers = false;
        // If we're climbing from the opposite side of the charge station, we want to flip the arm around the other way
        if(currentAutonomousState == "TestPath" || currentAutonomousState == "FarMobileConeBalance" || currentAutonomousState == "FarMobileConeCubeBalance") {
            bFlipPowers = true;
        }
        
        bForceStop = bHasAutoBalanced;
        if(!bHasStartedBalancing || !bHasAutoBalanced) {
            const auto pitch = drivetrain.GetPitch();

            // Start driving until we hit ~5 deg of pitch in order to become engaged
            if(!bHasStartedBalancing) {
                const auto power = 0.75 * (bFlipPowers ? -1 : 1);
                drivetrain.DriveRelative(power);
            }
            
            // If we've hit -5deg of pitch, we're on the ramp, start driving forward and start the balancing process and get ready to stop driving
            if(units::math::abs(drivetrain.GetPitch()) > 5_deg && !bHasStartedBalancing) {
                bHasStartedBalancing = true;
            }

            if(bHasStartedBalancing) {
                // If we're within ~2.5 degrees in general, stop driving the wheels
                if(units::math::abs(pitch) < 2_deg) {
                    bHasAutoBalanced = true;
                    bForceStop = true;
                    drivetrain.XPattern();
                    drivetrain.ForceStop();
                }
                else {
                    const double powerSign = pitch > 0_deg ? -1 : 1;
                    const units::degree_t maxPitch = 30_deg;
                    
                    // Calculate a value between -1 and 1 based on the value of the min/max pitch
                    // This does the inverse of a lerp and then copies the inverse of the sign of the pitch
                    double percentage = std::clamp((units::math::abs(pitch) / maxPitch).value(), 0.0, 0.5);

                    drivetrain.DriveRelative(percentage * powerSign);
                }
            }
        }
        else {
            drivetrain.XPattern();
            drivetrain.ForceStop();
        }
    }
    // If we're fully finished with the path, force stop the drive train in order to stop moving
    else if(bDriveTrainStopped || bForceStop) {
        drivetrain.ForceStop();
        bDriveTrainStopped = true;
    }

    const kinematics::KinematicState kinematicState = { arm.GetPosition(), shoulder.GetRotation(), wrist.GetRotation() };
    if(frc::SmartDashboard::GetBoolean("extensionLimits", true)) {
        // Check whether or not the current kinematic state is illegal
        if(!kinematics::Kinematics::IsValid(!intake.IsCubeMode(), kinematicState)) {
            FRC_ReportError(warn::Warning, "[Auto] Running into arm extension limits... Retracting arm in");
            armGoal = arm.GetPosition() - 3.5_in;
        }
    }

    arm.SetPositionGoal(armGoal);
    shoulder.SetRotationGoal(shoulderGoal);
    wrist.SetRotationGoal(wristGoal);
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
    
    if(shoulder.GetRotation() > 120_deg) {
        rotationalFlip = true;
    }
    rotationalTimer.Restart();
}

void Robot::TeleopPeriodic() {
    units::degree_t shoulderGoal = shoulder.GetRotation();
    units::meter_t armGoal = arm.GetPosition();
    units::degree_t wristGoal = wrist.GetLastSetpoint();

    if(auxController.GetLeftTriggerAxis() >= 0.5) {
        lighthandler.SetColor(frc::Color::kOrange);
        intake.ConeMode();
    }
    else if (auxController.GetRightTriggerAxis() >= 0.5)
    {
        lighthandler.SetColor(frc::Color::kPurple);
        intake.CubeMode();
    }
    else if(auxController.GetRightBumper()) {
        lighthandler.SetColor(frc::Color::kRed);
        intake.Spit();
    }
    else if(auxController.GetLeftBumper()) {
        lighthandler.SetColor(frc::Color::kRed);
        intake.Shoot();
    }
    else if(intake.HasElement()) {
        // Set the lights to green whenever we've picked something up
        lighthandler.SetColor(frc::Color::kGreen);

        // Start spinning the intake *slightly* in order to hold objects
        intake.Hold();

        fmt::print("Holding element...\n");

        // Start a small timer (if it's not been started) in order to time the duration of controller rumbling
        rumbleTimer.Start();

        // After 2 seconds, we want to stop rumbling the controller in order to avoid annoying the driver.
        driveController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, rumbleTimer.HasElapsed(5_s) ? 0.0 : 1.0);
    }
    else {
        rumbleTimer.Reset();
        rumbleTimer.Stop();
        intake.Stop();
        // Make sure to stop rumbling the controller
        driveController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
    }
    
    const bool bAutoAlign = false;
    if(!bAutoAlign) {
        const double dpadValue = auxController.GetPOV();
        // If the aux driver is hitting DPAD Up, drive the arm to the high position
        if(dpadValue == 0) {
            shoulderGoal = 170_deg;
            wristGoal = 0_deg;
            armGoal = 0.74_m;
        }
        else if(dpadValue == 180) {
            shoulderGoal = 24_deg;
            armGoal = 0.254_m;
            wristGoal = 24_deg;
        }
        else if(dpadValue == 270) {
            shoulderGoal = 90_deg;
            wristGoal = 0_deg;
            armGoal = 0_m;
        }
    }
    else {
        const double dpadValue = auxController.GetPOV();
        if(dpadValue != -1) {
            POVDebouncer.Start();
            // Use a timer in order to debounce the dpad inputs
            // This way, when you are unpressing the dpad values, it doesn't override as you unpress one button before another
            if(POVDebouncer.HasElapsed(0.25_s)) {
                storedPOVValue = dpadValue;
                POVDebouncer.Reset();
                POVDebouncer.Stop();
            }
        }

        frc::SmartDashboard::PutNumber("storedPOVValue", storedPOVValue);
        frc::SmartDashboard::PutBoolean("aligningToGrid", aligningToGrid);
        frc::SmartDashboard::PutBoolean("bAligningToScoringLocation", bAligningToScoringLocation);
        frc::SmartDashboard::PutNumber("alignmentTimer_s", alignmentTimer.Get().value());

        if(bAligningToScoringLocation && alignmentTimer.HasElapsed(3_s)) {
            bAligningToScoringLocation = false;
            aligningToGrid = false;
            alignmentTimer.Reset();
            alignmentTimer.Stop();
        }

        if(bInsideCommunity) {
            if(bAligningToScoringLocation && driveController.GetRawButton(constants::XboxButtons::eButtonLB)) {
                lighthandler.SetColor(frc::Color::kPeachPuff);

                const frc::Translation2d drivePose = drivetrain.GetPose().Translation();
                const auto state = kinematics::Kinematics::GetKinematicState(
                    !intake.IsCubeMode(), 
                    targetedScoringLocation,
                    drivePose
                );

                fmt::print("Rotation State: ({}_deg,{}_m,{}_deg)\n", state.shoulderAngle.value(), state.armExtension.value(), state.wristAngle.value());
                shoulderGoal = state.shoulderAngle;
                armGoal = state.armExtension;
                wristGoal = state.wristAngle;

                // If we're within ~2 deg of the desired rotation, call it good for rotation
                if(units::math::abs(state.shoulderAngle - shoulder.GetRotation()) >= 2_deg) {
                    alignmentTimer.Reset();
                    alignmentTimer.Stop();
                    bAligningToScoringLocation = false;
                    aligningToGrid = false;
                }
            }
            else if((!aligningToGrid && !bAligningToScoringLocation) && driveController.GetRawButtonPressed(constants::XboxButtons::eButtonLB)) {
                // goalPose is the translation that the robot needs to drive to in order to be in the proper position
                frc::Translation2d goalPose = frc::Translation2d();

                // Set the goal pose based on the buttons pressed
                int offset = 0;
                if(storedPOVValue == 225 || storedPOVValue == 270 || storedPOVValue == 315) {
                    goalPose = fieldConstants.lowLocations[0 + (currentGrid * 3)].location.ToTranslation2d();
                }
                else if(storedPOVValue == -1 || (storedPOVValue == 0 || storedPOVValue == 180)) {
                    offset = 1;
                    goalPose = fieldConstants.lowLocations[1 + (currentGrid * 3)].location.ToTranslation2d();
                }
                else if(storedPOVValue == 45 || storedPOVValue == 135 || storedPOVValue == 90) {
                    offset = 2;
                    goalPose = fieldConstants.lowLocations[2 + (currentGrid * 3)].location.ToTranslation2d();
                }

                if(storedPOVValue == 0 || storedPOVValue == 45 || storedPOVValue == 315) {
                    targetedScoringLocation = fieldConstants.highLocations[offset + (currentGrid * 3)].location;
                }
                else if(storedPOVValue == -1 || storedPOVValue == 270 || storedPOVValue == 90) {
                    targetedScoringLocation = fieldConstants.midLocations[offset + (currentGrid * 3)].location;
                }
                else if(storedPOVValue == 180 || storedPOVValue == 135 || storedPOVValue == 225) {
                    targetedScoringLocation = fieldConstants.lowLocations[offset + (currentGrid * 3)].location;
                }

                const units::inch_t alignmentOffset = 16.5_in;

                // Add/subtract an extra half robot width in order to account for robot size.
                // If we're on the red alliance, we actually want to subtract this difference due to the lack of field symmetry.
                goalPose = goalPose + frc::Translation2d(
                    alignmentOffset * (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ? 1 : -1), 
                    7.25_in
                );
                
                // Set the drive train to actually start moving to the goal pose
                // TODO: Potentially automatically rotate the robot to face properly
                // It might be best to keep the robot facing the same way it currently is to avoid hitting unknown objects (or the field)
                drivetrain.SetTrajectory(frc::Pose2d(goalPose, drivetrain.GetPose().Rotation()));
                
                aligningToGrid = true;
            }
            else if(aligningToGrid) {
                const drive::AutonomousState state = drivetrain.FollowTrajectory();
                if(state.bCompletedPath) {
                    // Flash the lights white whenever we've auto lined up to the scoring position.
                    lighthandler.SetColor(frc::Color::kWhite);
                    
                    drivetrain.ForceStop();

                    bAligningToScoringLocation = true;
                    alignmentTimer.Restart();
                }
                else if(!drivetrain.GetTrajectoryActive()) {
                    fmt::print("Trajectory cancelled... Flipping colors\n");
                    // If the trajectory got cancelled, flip the colors to red
                    lighthandler.SetColor(frc::Color::kRed);
                    
                    aligningToGrid = false;
                    bAligningToScoringLocation = false;
                    alignmentTimer.Reset();
                    alignmentTimer.Stop();
                }
            }
            else {
                alignmentTimer.Restart();
            }
        }
    }
    frc::SmartDashboard::PutNumber("rotationalTimer_s", rotationalTimer.Get().value());
    frc::SmartDashboard::PutBoolean("rotationalFlip", rotationalFlip);

    // If the aux driver hasn't touched the controls for 0.5s, flip the controls based on the arm position
    // This way if the arm is 180deg around, the up joystick will still drive the arm/wrist "up".
    const units::second_t rotationalDelay = 0.125_s;
    double rotationalOverride = rotationalFlip ? 1 : -1;
    if(shoulder.GetRotation() > 120_deg) {
        if(rotationalTimer.HasElapsed(rotationalDelay)) {
            rotationalOverride = 1;
            rotationalFlip = true;
        }
    }
    else if(shoulder.GetRotation() < 120_deg) {
        if(rotationalTimer.HasElapsed(rotationalDelay)) {
            rotationalOverride = -1;
            rotationalFlip = false;
        }
    }

    // Use the joysticks to manually control the arm
    double shoulderOverride = frc::ApplyDeadband(auxController.GetLeftY(), 0.10);
    if(shoulderOverride != 0) {
        shoulder.ManualControl(shoulderOverride * (0.50 * rotationalOverride));
        shoulder.AdjustFeedforward(0_V);
        rotationalTimer.Restart();
    }
    else {
        shoulder.ManualControl(0.0);
    }

    double wristOverride = frc::ApplyDeadband(auxController.GetRightY(), 0.10);

    if(wrist.GetRotation() >= 85_deg) {
        if(!rotationalFlip && wristOverride < 0) {
            wristOverride = 0;
        }
        else if(rotationalFlip && wristOverride > 0) {
            wristOverride = 0;
        }
        
    }

    if(wristOverride != 0) {
        wrist.ManualControl(wristOverride * (0.35 * rotationalOverride));
    }
    else {
        wrist.ManualControl(0.0);
    }

    // Use the ""fork"" up/down buttons in order to control the arm manually up/down
    if(auxController.GetAButton()) arm.ManualControl(0.5);
    else if(auxController.GetBButton()) arm.ManualControl(-0.5);
    else {
        arm.ManualControl(0.0);
    }
    frc::SmartDashboard::PutNumber("wristOverride", wristOverride);
    frc::SmartDashboard::PutNumber("shoulderOverride", shoulderOverride);
    // TODO: Implement extension limits
    frc::SmartDashboard::PutBoolean("cubeMode", intake.IsCubeMode());
    const kinematics::KinematicState kinematicState = { arm.GetPosition(), shoulder.GetRotation(), wrist.GetRotation() };
    if(frc::SmartDashboard::GetBoolean("extensionLimits", true)) {
        // Check whether or not the current kinematic state is illegal
        if(!kinematics::Kinematics::IsValid(!intake.IsCubeMode(), kinematicState)) {
            FRC_ReportError(warn::Warning, "[Robot] Running into arm extension limits... Retracting arm in");
            armGoal = arm.GetPosition() - 3.5_in;
            auxController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
        }
        else {
            auxController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
        }
    }

    frc::SmartDashboard::PutNumber("shoulderOverride", shoulderOverride);
    // B is the panic button, if we've hit the panic button, move the arm up into the air
    if(driveController.GetRawButton(constants::XboxButtons::eButtonB)) {
        shoulderGoal = 90_deg;
        armGoal = 0_m;
    }

    if(auxController.GetXButton()) {
        armGoal = 0_m;
        shoulderGoal = -33_deg;
        wristGoal = 24_deg;
    }
    if(auxController.GetYButton()) {
        armGoal = 0_m;
        shoulderGoal = 30_deg;
        wristGoal = -90_deg;
    }

    shoulder.SetRotationGoal(shoulderGoal);    
    arm.SetPositionGoal(armGoal);
    wrist.SetRotationGoal(wristGoal);

    bWasInCommunity = bInsideCommunity;
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
    if(m_chooser.GetSelected().pathName != currentAutonomousState) {
        currentAutonomousState = m_chooser.GetSelected().pathName;
        drivetrain.UpdateFieldTrajectory(currentAutonomousState);
    }
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

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

    frc::SmartDashboard::PutBoolean("intakeFlipping", true);

    cubert.Init();
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

    intake.Tick();

    shoulder.Tick(arm.GetPosition());
    arm.Tick(shoulder.GetRotation());
    wrist.Tick(shoulder.GetRotation());

    frc::SmartDashboard::PutNumber("autoTimer_s", autoTimer.Get().value());
    frc::SmartDashboard::PutBoolean("autoTimerRunning", autoTimerRunning);

    bWasInCommunity = bInsideCommunity;

    bool intakeFlipping = frc::SmartDashboard::GetBoolean("intakeFlipping", true);
    if(intakeFlipping) {
        if(shoulder.GetRotation() >= 135_deg) {
            if(wrist.GetRotation() > 100_deg) {
                intake.FlipDirection(true);
            }
            else {
                intake.FlipDirection(false);
            }
        }
        else {
            if(wrist.GetRotation() < -100_deg) {
                intake.FlipDirection(false);
            }
            else {
                intake.FlipDirection(true);
            }
        }
    }

    frc::SmartDashboard::PutBoolean("cubertHasElement", cubert.HasElement());

    cubert.Tick();
}

void Robot::AutonomousInit() {
    // When connected to the FMS, the FMS may not have sent the correct driver station alliance to the robot
    // so we should reinitialize the field constants just to be safe.
    fieldConstants.Initialize();

    currentAutonomousState = m_chooser.GetSelected().humanName;
    currentAutonomousPath = m_chooser.GetSelected();
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

    totalAutoTimer.Reset();
    totalAutoTimer.Restart();
}

void Robot::AutonomousPeriodic() {
    if(intake.HasElement()) {
        lighthandler.SetColor(frc::Color::kGreen);
    }
    else {
        lighthandler.Rainbow();
    }
    
    frc::SmartDashboard::PutString("currentAutonomousState", currentAutonomousState);
    frc::SmartDashboard::PutBoolean("bHasStartedBalancing", bHasStartedBalancing);
    frc::SmartDashboard::PutBoolean("bHasAutoBalanced", bHasAutoBalanced);

    units::degree_t shoulderGoal = shoulder.GetRotationalGoal();
    units::meter_t armGoal = arm.GetPositionalGoal();
    units::degree_t wristGoal = wrist.GetRotation();

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
        if(currentAutonomousPath.startingAction == drive::AutonomousPath::eNone) { 
            bHasProcessedStartingAction = true; 
        }
        else if(currentAutonomousPath.startingAction == drive::AutonomousPath::eMidCone) {
            shoulderGoal = 150_deg;
            armGoal = 0.254_m;
            wristGoal = 10_deg;

            if(units::math::abs(shoulder.GetRotation() - shoulderGoal) <= 5_deg && units::math::abs(arm.GetPosition() - armGoal) <= 2.5_in) {
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
                shoulderGoal = 90_deg;
                armGoal = 0_m;
                wristGoal = 0_deg;
            }
            else if(autoTimer.HasElapsed(0.125_s)) {
                intake.Shoot();
            }
            else {
                intake.SetFlangeLocation(intake::FlangeLocation::eBackwards);
                intake.Hold();
            }
        }
        else if(currentAutonomousPath.startingAction == drive::AutonomousPath::eHighCone) {
            shoulderGoal = 148_deg;
            wristGoal = 4_deg;
            
            if(units::math::abs(shoulderGoal - shoulder.GetRotation()) <= 45_deg) {
                armGoal = 0.78_m;
            }

            if(units::math::abs(shoulder.GetRotation() - shoulderGoal) <= 2.5_deg && units::math::abs(arm.GetPosition() - armGoal) <= 4_in) {
                autoTimer.Start();
                autoTimerRunning = true;
            }

            if(autoTimer.HasElapsed(0.5_s)) {
                bHasProcessedStartingAction = true;
                autoTimerRunning = false;
                autoTimer.Reset();
                autoTimer.Stop();
            }
            else if(autoTimer.HasElapsed(0.25_s)) {
                intake.Stop();
                shoulderGoal = 90_deg;
                armGoal = 0_m;
                wristGoal = 0_deg;
            }
            else if(autoTimer.HasElapsed(0.125_s)) {
                intake.Shoot();
            }
            else {
                intake.SetFlangeLocation(intake::FlangeLocation::eBackwards);
                intake.Hold();
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
            if(currentAutonomousState == "Far Mid Cone Cone" || currentAutonomousState == "Mid Cone Cone" || currentAutonomousState == "High Cone Cone") {
                if(trajectoryCompleted.currentStopIndex == 0) {
                    armGoal = 0_m;
                    shoulderGoal = -21_deg;
                    wristGoal = -72_deg;
                    intake.ConeMode();
                }
                else if(trajectoryCompleted.currentStopIndex == 1 || trajectoryCompleted.currentStopIndex == 2) {
                    // Avoid extending the arm out at first in order to avoid crashing into things with the arm accidentally.
                    shoulderGoal = 140_deg;
                    if(currentAutonomousState == "High Cone Cone") {
                        wristGoal = -18_deg;
                    }
                    else {
                        wristGoal = 12_deg;
                    }
                    
                    autoTimerRunning = false;
                    intake.Hold();
                }
            }
            else if(currentAutonomousState == "High Cone Cube Run" || currentAutonomousState == "Mid Cone Cube" || currentAutonomousState == "High Cone Cube Balance") {
                if(trajectoryCompleted.currentStopIndex == 0) {
                    armGoal = 0.035_in;
                    shoulderGoal = -21_deg;
                    wristGoal = 23_deg;
                    intake.ConeMode();
                }
                else if(trajectoryCompleted.currentStopIndex == 1) {
                    shoulderGoal = 140_deg;
                    wristGoal = 65_deg;
                    if(currentAutonomousState == "High Cone Cube Run") {
                        armGoal = 0.473_m;
                    }
                    
                    autoTimerRunning = false;
                    intake.Hold();
                }
                else if(trajectoryCompleted.currentStopIndex == 2) {
                    shoulderGoal = 90_deg;
                    armGoal = 0_m;
                }
            }
            else if(currentAutonomousState == "Mid Cone Balance" || currentAutonomousState == "High Cone Balance") {
                shoulderGoal = 90_deg;
                armGoal = 0_m;
                wristGoal = 0_deg;
            }
        }
        else if(trajectoryCompleted.bAtStopPoint) {
            // These autos pick up / score the preloaded cone mid
            if(currentAutonomousState == "Far Mid Cone Cone" || currentAutonomousState == "Mid Cone Cone") {
                if(trajectoryCompleted.currentStopIndex == 0) {
                    shoulderGoal = 0_deg;
                    drivetrain.StartNextTrajectory();
                }
                else if(trajectoryCompleted.currentStopIndex == 1) {
                    shoulderGoal = 150_deg;
                    armGoal = 0.254_m;
                    wristGoal = 10_deg;

                    if(units::math::abs(arm.GetPosition() - armGoal) <= 2.5_in) {
                        intake.Shoot();
                        if(!autoTimerRunning) {
                            autoTimer.Reset();
                            autoTimer.Restart();
                        }
                        autoTimerRunning = true;
                    }

                    if(autoTimer.HasElapsed(0.5_s)) {
                        intake.Stop();

                        shoulderGoal = 90_deg;
                        if(units::math::abs(shoulder.GetRotation() - shoulderGoal) <= 10_deg) {
                            armGoal = 0_m;
                            wristGoal = 0_deg;
                            drivetrain.StartNextTrajectory();
                        }
                    }

                    drivetrain.ForceStop();
                }
                else if(trajectoryCompleted.currentStopIndex == 2) {
                    drivetrain.ForceStop();
                }
            }
            else if(currentAutonomousState == "High Cone Cone") {
                if(trajectoryCompleted.currentStopIndex == 0) {
                    shoulderGoal = 0_deg;
                    drivetrain.StartNextTrajectory();
                }
                else if(trajectoryCompleted.currentStopIndex == 1) {
                    shoulderGoal = 148_deg;
                    wristGoal = 2_deg;
            
                    if(units::math::abs(shoulderGoal - shoulder.GetRotation()) <= 45_deg) {
                        armGoal = 0.78_m;
                    }

                    if(units::math::abs(arm.GetPosition() - armGoal) <= 2.5_in) {
                        intake.Shoot();
                        if(!autoTimerRunning) {
                            autoTimer.Reset();
                            autoTimer.Restart();
                        }
                        autoTimerRunning = true;
                    }

                    if(autoTimer.HasElapsed(0.5_s)) {
                        intake.Stop();

                        shoulderGoal = 90_deg;
                        armGoal = 0_m;
                        wristGoal = 0_deg;

                        drivetrain.StartNextTrajectory();
                    }

                    drivetrain.ForceStop();
                }
                else if(trajectoryCompleted.currentStopIndex == 2) {
                    drivetrain.ForceStop();
                }
            }
            // These autos score the pre-staged cube
            else if(currentAutonomousState == "Mid Cone Cube Run" || currentAutonomousState == "High Cone Cube Run" || currentAutonomousState == "High Cone Cube Balance") {
                if(trajectoryCompleted.currentStopIndex == 0) {
                    shoulderGoal = 0_deg;
                    drivetrain.StartNextTrajectory();
                }
                else if(trajectoryCompleted.currentStopIndex == 1) {
                    shoulderGoal = 148_deg;
                    wristGoal = 65_deg;
                    armGoal = 0.473_m;

                    drivetrain.ForceStop();
                    if(units::math::abs(arm.GetPosition() - armGoal) <= 8_in) {
                        intake.Shoot();
                        if(!autoTimerRunning) {
                            autoTimer.Reset();
                            autoTimer.Restart();
                        }
                        autoTimerRunning = true;
                    }

                    if(autoTimer.HasElapsed(0.125_s)) {
                        intake.Stop();

                        shoulderGoal = 90_deg;
                        armGoal = 0_m;
                        wristGoal = 0_deg;

                        drivetrain.StartNextTrajectory();
                    }
                }
                else if(trajectoryCompleted.currentStopIndex == 2 && currentAutonomousState == "High Cone Cube Run") {
                    intake.Stop();
                    shoulderGoal = 90_deg;
                    armGoal = 0_m;
                    wristGoal = 0_deg;
                    drivetrain.ForceStop();
                }
                else if(trajectoryCompleted.currentStopIndex == 2 && currentAutonomousState == "High Cone Cube Balance") {
                    drivetrain.ForceStop();
                    bDriveTrainStopped = true;
                }
            }
            else if(currentAutonomousState == "Mid Cone Balance" || currentAutonomousState == "High Cone Balance" || currentAutonomousState == "Balance") {
                bDriveTrainStopped = true;
                drivetrain.ForceStop();
                autoTimerRunning = false;
            }
            else if(bForceStop) {
                bDriveTrainStopped = true;
                drivetrain.ForceStop();
            }
        }

    }
    else if(currentAutonomousState == "Balance" || currentAutonomousState == "Mid Cone Balance" || currentAutonomousState == "High Cone Balance" || currentAutonomousState == "High Cone Cube Balance") {
        // Move the arm down and start driving forward in order to shift the CoG towards the charge station
        shoulderGoal = 75_deg;
        armGoal = 0_m;
        wristGoal = 0_deg;
        bool bFlipPowers = false;
        // If we're climbing from the opposite side of the charge station, we want to flip the arm around the other way
        if(currentAutonomousState == "FarMobileConeBalance" || currentAutonomousState == "FarMobileConeCubeBalance" || currentAutonomousState == "ConeCubeBalance") {
            bFlipPowers = true;
        }
        
        if(bFlipPowers) {
            shoulderGoal = 145_deg;
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
            if(units::math::abs(drivetrain.GetPitch()) > 8_deg && !bHasStartedBalancing) {
                bHasStartedBalancing = true;
            }

            if(bHasStartedBalancing) {
                if(lastPitch - drivetrain.GetPitch() > (lastPitch * (bFlipPowers ? 1 : -1) + 0.02_deg)) {
                    if(!autoTimerRunning) {
                        autoTimer.Reset();
                        autoTimer.Restart();
                        autoTimerRunning = true;
                    }
                    
                    if(autoTimerRunning && !autoTimer.HasElapsed(1.4_s)) {
                        drivetrain.DriveRelative(-0.2);
                    }
                    else {
                        bHasAutoBalanced = true;
                        bForceStop = true;
                        drivetrain.XPattern();
                    }
                }
                else {
                    autoTimerRunning = false;
                    drivetrain.DriveRelative(0.25);
                }
            }

            lastPitch = drivetrain.GetPitch();
        }
        else {
            drivetrain.XPattern();
        }
    }
    // If we're fully finished with the path, force stop the drive train in order to stop moving
    else if(bDriveTrainStopped || bForceStop) {
        drivetrain.XPattern();
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
    shoulder.RefreshController();
    
    wrist.SetRotationGoal(wrist.GetRotation());
    shoulder.SetRotationGoal(shoulder.GetRotation());
    arm.SetPositionGoal(arm.GetPosition());
    
    if(shoulder.GetRotation() > 120_deg) {
        rotationalFlip = true;
    }
    rotationalTimer.Restart();
}

void Robot::TeleopPeriodic() {
    units::degree_t shoulderGoal = shoulder.GetRotationalGoal();
    units::meter_t armGoal = arm.GetPositionalGoal();
    units::degree_t wristGoal = wrist.GetRotation();

    if(auxController.GetLeftTriggerAxis() >= 0.5) {
        lighthandler.SetColor(frc::Color::kOrange);
        intake.ConeMode();
        intakedCube = false;
    }
    else if(auxController.GetRightBumper()) {
        lighthandler.SetColor(frc::Color::kRed);
        intake.Spit();
        intakedCube = false;
    }
    else if(intake.HasElement() || cubert.HasElement()) {
        // Set the lights to green whenever we've picked something up
        lighthandler.SetColor(frc::Color::kGreen);

        // Start spinning the intake *slightly* in order to hold objects
        intake.Hold();

        // Start a small timer (if it's not been started) in order to time the duration of controller rumbling
        rumbleTimer.Start();

        // After 2 seconds, we want to stop rumbling the controller in order to avoid annoying the driver.
        driveController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, rumbleTimer.HasElapsed(1_s) ? 0.0 : 0.5);

        intakedCube = !intake.HasElement();
    }
    else {
        rumbleTimer.Reset();
        rumbleTimer.Stop();
        intake.ConeMode();
        // Make sure to stop rumbling the controller
        driveController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
    }
    
    const bool bAutoAlign = false;
    auto cubeGoal = -1;

    if(!bAutoAlign) {
        const double dpadValue = auxController.GetPOV();
        // If the aux driver is hitting DPAD Up, drive the arm to the high position
        // If the driver is hitting it and we don't have an element in the cone intake, shoot out a cube
        if(dpadValue == 0) {
            if(intakedCube) {
                cubeGoal = constants::FieldConstants::GridHeights::eUp;
            }
            else {
                shoulderGoal = 27_deg; // Formerly: 40_deg
                wristGoal = 21_deg;
                if(units::math::abs(shoulderGoal - shoulder.GetRotation()) <= 5_deg) {
                    armGoal = 0.78_m;
                }
            }
        }
        else if(dpadValue == 90) {
            shoulderGoal = 158_deg;
            wristGoal = 61.5_deg;
            armGoal = 0_m;
        }
        // Drive the arm to the mid position when hitting down
        else if(dpadValue == 180) {
            if(intakedCube) {
                cubeGoal = constants::FieldConstants::GridHeights::eMid;
            }
            else {
                shoulderGoal = 25.3_deg; // Formerly: 38.5_deg
                wristGoal = 9_deg; // Formerly: 0_deg
            
                if(units::math::abs(shoulderGoal - shoulder.GetRotation()) <= 25_deg) {
                    armGoal = 0.2896_m; // Formerly: 0.254_m - 3.5_in
                }
            }
        }
        else if(dpadValue == 270) {
            shoulderGoal = 90_deg;
            armGoal = 0_m;

            // Change the wrist angle based on the flange location so that way we can help passively hold the cone better
            switch (intake.GetFlangeLocation())
            {
                // NOT NEEDED B/C WE USE ONLY ONE SIDE FOR INTAKE
                //case intake::FlangeLocation::eBackwards:
                //    wristGoal = -67_deg;
                //    break;
                default:
                    wristGoal = 90_deg;
                    break;
            }
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
    if(false) {
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
    }
    double rotationalOverride = -1;
    rotationalFlip = false;

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
    if(auxController.GetStartButtonReleased()) {
        wrist.ResetRotation();
    }

    if(auxController.GetBackButtonReleased()) {
        shoulder.ResetRotation();
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

    frc::SmartDashboard::PutBoolean("cubeMode", intake.IsCubeMode());
    const kinematics::KinematicState kinematicState = { arm.GetPosition(), shoulder.GetRotation(), wrist.GetRotation() };
    if(frc::SmartDashboard::GetBoolean("extensionLimits", true)) {
        // Check whether or not the current kinematic state is illegal
        if(!kinematics::Kinematics::IsValid(!intake.IsCubeMode(), kinematicState)) {
            FRC_ReportError(warn::Warning, "[Robot] Running into arm extension limits... Retracting arm in");
            armGoal = arm.GetPosition() - 1.5_in;
            auxController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 1.0);
        }
        else {
            auxController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
        }
    }

    frc::SmartDashboard::PutNumber("shoulderOverride", shoulderOverride);

    // Ground Button
    if(auxController.GetXButton()) {
        armGoal = 0_m;
        shoulderGoal = -15_deg - 6.2_deg; // Previously: -15_deg
        wristGoal = 60_deg;
        intakedCube = false;
    }
    
    if(driveController.GetRawButton(constants::XboxButtons::eButtonLB)) {
        cubert.Shoot(constants::FieldConstants::GridHeights::eIntake);
        intakedCube = true;
    }
    else if(cubeGoal != -1 || auxController.GetRightTriggerAxis() >= 0.25 || auxController.GetLeftBumper()) {
        if(auxController.GetRightTriggerAxis() >= 0.25) {
            cubeGoal = constants::FieldConstants::GridHeights::eGround;
        }
        else if(auxController.GetLeftBumper()) {
            cubeGoal = constants::FieldConstants::GridHeights::eGroundSpit;
        }
        // Set the cubert to start shooting at the specified height
        cubert.Shoot((constants::FieldConstants::GridHeights)cubeGoal);

        lighthandler.SetColor(frc::Color::kRed);
    }
    else {
        cubert.Shoot(constants::FieldConstants::GridHeights::eStopped);
    }

    // B is the panic button, if we've hit the panic button, move the arm up into the air
    // If the intake is deployed, we also want to pull the arm in for panic mode.
    if(driveController.GetRawButton(constants::XboxButtons::eButtonB) || cubert.IsDeployed()) {
        shoulderGoal = 90_deg;
        armGoal = 0_m;
    }

    shoulder.SetRotationGoal(shoulderGoal);    
    arm.SetPositionGoal(armGoal);
    wrist.SetRotationGoal(wristGoal);
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

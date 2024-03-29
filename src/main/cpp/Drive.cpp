#include "Drive.h"
#include <frc/DriverStation.h>

using namespace pathplanner;

drive::Drive::Drive(frc::Joystick* controller_in) {
    controller = controller_in;
    gyro.Reset();
    
    headingPIDController.EnableContinuousInput(-constants::Pi, constants::Pi);

    headingSnapPIDController.EnableContinuousInput(-constants::Pi, constants::Pi);
    headingSnapPIDController.SetTolerance(units::radian_t(0.1_deg).value());

    frc::SmartDashboard::PutData("headingPIDController", &headingPIDController);
    frc::SmartDashboard::PutData("headingSnapPIDController", &headingSnapPIDController);

    frc::SmartDashboard::PutBoolean("bFieldOriented", true);
    frc::SmartDashboard::GetBoolean("bProtectHeading", true);

    frc::SmartDashboard::PutData("field", &field);

    photonPoseEstimator.SetMultiTagFallbackStrategy(photonlib::PoseStrategy::CLOSEST_TO_REFERENCE_POSE);
    poseEstimator.SetVisionMeasurementStdDevs({28.8, 26.2, constants::Pi * 2});
}

void drive::Drive::SetJoystick(bool state) {
    headingProtectionTimer.Reset();
    headingProtectionTimer.Start();
    is_joystickControl = state;
}

void drive::Drive::Tick() {
    UpdateOdometry();

    if(!is_joystickControl) return;

    bool fieldRelative = frc::SmartDashboard::GetBoolean("bFieldOriented", true);
    bProtectHeading = frc::SmartDashboard::GetBoolean("bProtectHeading", true);

    // Reset the gyro if the driver presses start.
    // This'll help avoid drift in field-oriented mode.
    if(controller->GetRawButtonReleased(constants::XboxButtons::eStart)) {
        gyro.Reset();

        desiredHeading = gyro.GetRotation2d();
        frc::SmartDashboard::PutNumber("desired_heading_rad", desiredHeading.Radians().value());
        headingPIDController.Reset();
    }

    double xAxis = frc::ApplyDeadband(-controller->GetRawAxis(constants::XboxAxis::eLeftAxisX), 0.10);
    double yAxis = frc::ApplyDeadband(-controller->GetRawAxis(constants::XboxAxis::eLeftAxisY), 0.10);
    double rAxis = frc::ApplyDeadband(-controller->GetRawAxis(constants::XboxAxis::eRightAxisX), 0.10);

    if(pathCommand != nullptr && pathCommand->IsFinished()) {
        bFollowTrajectory = false;
    }

    if((xAxis != 0 || yAxis != 0 || rAxis != 0 || !controller->GetRawButton(constants::XboxButtons::eButtonLB)) && pathCommand != nullptr) {
        bFollowTrajectory = false;
    }

    // If we're currently following a path, don't both to do anything else with the drive train
    if(pathCommand != nullptr && bFollowTrajectory) {
        return;
    }

    auto maxTranslationalVelocity = drive::Constants::maxTranslationalVelocity / 2;
    auto maxRotationalVelocity = drive::Constants::maxRotationalVelocity / 2;

    if(controller->GetRawAxis(constants::XboxAxis::eRightTrigger) >= 0.10) {
        double divisorAmount = wpi::Lerp<double>(1.0, 0.250, controller->GetRawAxis(constants::XboxAxis::eRightTrigger));
        maxTranslationalVelocity *= divisorAmount;
        maxRotationalVelocity *= divisorAmount;
    }
    else if(controller->GetRawAxis(constants::XboxAxis::eLeftTrigger) >= 0.10) {
        double additionalAmount = wpi::Lerp<double>(1.0, 2.0, controller->GetRawAxis(constants::XboxAxis::eLeftTrigger));
        maxTranslationalVelocity *= additionalAmount;
        maxRotationalVelocity *= additionalAmount;
    }

    // These initial casts to meter_t are pretty much just to make the compiler happy. 
    // Once we multiply by the max speed is when we get the actual speed
    frc::Translation2d translation = frc::Translation2d(units::meter_t(yAxis), units::meter_t(xAxis)) * maxTranslationalVelocity.value();
    auto rotation = rAxis * maxRotationalVelocity;

    bool bForceAngle = false;

    // If you are field relative and hold RB, snap the robot itself to the nearest 45deg angle based on the right joystick
    if(fieldRelative && controller->GetRawButton(constants::XboxButtons::eButtonRB)) {
        units::degree_t snap = 0_deg;

        // Scoring 
        if (controller->GetRawButton(constants::XboxButtons::eButtonA)) {
            snap = 180_deg;
        } else if (controller->GetRawButton(constants::XboxButtons::eButtonX)) {
            if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
                snap = -90_deg;
            } else {
                snap = 90_deg;
            }
        } else if (controller->GetRawButton(constants::XboxButtons::eButtonY)) {
            snap = 0_deg;
        }

        frc::SmartDashboard::PutNumber("snap_angle_deg", units::degree_t(snap).value());
        frc::SmartDashboard::PutNumber("snap_angle_rad", snap.value());
            
        // Use the PID controller to calculate our angular velocity from the given angle (measurement) & snap point (setpoint)
        const units::radians_per_second_t new_rotation =  units::radians_per_second_t(
            headingSnapPIDController.Calculate(
                // Wraps the angle from -pi to +pi radians
                frc::AngleModulus(gyro.GetRotation2d().Radians()).value(),
                frc::AngleModulus(snap).value()
            )
        );

        if(headingSnapPIDController.AtSetpoint()) {
            // If we're at the setpoint, don't even bother rotating with the joystick anymore
            rotation = 0_rad_per_s;
        }
        else {
            rotation = new_rotation;
            bForceAngle = true;
        }
    } else {
        headingSnapPIDController.Reset();
    }

    if(bProtectHeading) {
        frc::SmartDashboard::PutNumber("headingProtectionTimer_s", headingProtectionTimer.Get().value());

        // If we're not touching the joystick, try and keep with the current heading unless we were massively offset
        if(rotation == 0_rad_per_s) {
            if(!headingProtectionTimer.HasElapsed(0.85_s)) {
                desiredHeading = gyro.GetRotation2d();
                frc::SmartDashboard::PutNumber("desired_heading_rad", desiredHeading.Radians().value());
                headingPIDController.Reset();
            }

            // If we're >5 deg offset from the initial heading, don't even bother trying to maintain the current heading
            if(headingProtectionTimer.HasElapsed(0.85_s) && units::math::abs((gyro.GetRotation2d() - desiredHeading).Radians()) <= 5_deg) {
                units::radians_per_second_t angleAdjustment = units::radians_per_second_t(headingPIDController.Calculate(
                    frc::AngleModulus(gyro.GetRotation2d().Radians()).value(),
                    frc::AngleModulus(desiredHeading.Radians()).value()
                ));

                rotation = angleAdjustment;
            }
        }
        // If we are touching the joystick, set the desired heading to the current heading
        else {
            headingProtectionTimer.Reset();
            headingProtectionTimer.Start();
        }
    }

    auto xVelocity = xSpeedLimiter.Calculate(translation.X() / 1_s);
    auto yVelocity = ySpeedLimiter.Calculate(translation.Y() / 1_s);

    frc::ChassisSpeeds nonrelspeeds = frc::ChassisSpeeds();
    nonrelspeeds.omega = rotation;
    nonrelspeeds.vx = xVelocity;
    nonrelspeeds.vy = yVelocity;

    auto relspeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        xVelocity,
        yVelocity,
        rotation,
        gyro.GetRotation2d()
    );

    wpi::array<frc::SwerveModuleState, 4> states = kinematics.ToSwerveModuleStates(fieldRelative ? relspeeds : nonrelspeeds);

    bool bXPattern = controller->GetRawButton(constants::XboxButtons::eButtonX);
    frc::SmartDashboard::PutBoolean("bXPattern", bXPattern);
    if(bXPattern) {
        bForceAngle = true;

        states = wpi::array<frc::SwerveModuleState, 4> {
            frc::SwerveModuleState { 0_mps, frc::Rotation2d(45_deg) },
            frc::SwerveModuleState { 0_mps, frc::Rotation2d(315_deg) },
            frc::SwerveModuleState { 0_mps, frc::Rotation2d(135_deg) },
            frc::SwerveModuleState { 0_mps, frc::Rotation2d(225_deg) },
        };
    }

    kinematics.DesaturateWheelSpeeds(
        &states, 
        fieldRelative ? relspeeds : nonrelspeeds, 
        SwerveModule::kMaxSpeed, 
        maxTranslationalVelocity, 
        maxRotationalVelocity
    );

    // If we're driving at a slow speed (but commanded by a ""large"" joystick value, force the angle)
    if(xAxis <= 0.20 || yAxis <= 0.20) {
        bForceAngle = true;
    }

    auto [fl, fr, bl, br] = states;

    frontleft.SetDesiredState(fl, bForceAngle);
    frontright.SetDesiredState(fr, bForceAngle);
    backleft.SetDesiredState(bl, bForceAngle);
    backright.SetDesiredState(br, bForceAngle);
}

void drive::Drive::LogEncoders() {
    frc::SmartDashboard::PutNumber("fl_turn", frontleft.getTurnEncPos());
    frc::SmartDashboard::PutNumber("fl_drive", frontleft.GetPosition().distance.value());
    frc::SmartDashboard::PutNumber("fr_turn", frontright.getTurnEncPos());
    frc::SmartDashboard::PutNumber("fr_drive", frontright.GetPosition().distance.value());
    frc::SmartDashboard::PutNumber("bl_turn", backleft.getTurnEncPos());
    frc::SmartDashboard::PutNumber("bl_drive", backleft.GetPosition().distance.value());
    frc::SmartDashboard::PutNumber("br_turn", backright.getTurnEncPos());
    frc::SmartDashboard::PutNumber("br_drive", backright.GetPosition().distance.value());

    frc::SmartDashboard::PutNumber("fl_vel", frontleft.GetState().speed.value());
    frc::SmartDashboard::PutNumber("br_vel", backright.GetState().speed.value());

    frc::SmartDashboard::PutNumber("gyro_pitch", (double)gyro.GetPitch());
    frc::SmartDashboard::PutNumber("curr_angle_deg", units::degree_t{frc::AngleModulus(gyro.GetRotation2d().Radians())}.value());

}

void drive::Drive::ResetOdometry() {
    frontleft.ResetEncoders();
    frontright.ResetEncoders();
    backleft.ResetEncoders();
    backright.ResetEncoders();

    gyro.ZeroYaw();
}

void drive::Drive::DriveRelative(double power, units::meters_per_second_t maxVelocity) {
    frc::ChassisSpeeds nonrelspeeds = frc::ChassisSpeeds();
    nonrelspeeds.omega = 0_deg_per_s;
    nonrelspeeds.vx = maxVelocity * power;
    nonrelspeeds.vy = 0_mps;

    wpi::array<frc::SwerveModuleState, 4> states = kinematics.ToSwerveModuleStates(nonrelspeeds);

    kinematics.DesaturateWheelSpeeds(
        &states, 
        nonrelspeeds, 
        SwerveModule::kMaxSpeed, 
        maxVelocity, 
        0_deg_per_s
    );

    auto [fl, fr, bl, br] = states;

    this->frontleft.SetDesiredState(fl, true);
    this->frontright.SetDesiredState(fr, true);
    this->backleft.SetDesiredState(bl, true);
    this->backright.SetDesiredState(br, true);
}

const drive::AutonomousState drive::Drive::FollowTrajectory() {
    if(pathCommand == nullptr) {
        FRC_ReportError(frc::err::Error, "[Robot] Unable to follow trajectory; No valid path command generated");
        return { false, 9999, true };
    }

    if(bFollowTrajectory && !pathCommand->IsFinished()) {
        pathCommand->Execute();
    }
    
    return { 
        pathCommand->IsFinished(),
        currentStopPoint,
        (pathCommand->IsFinished() && currentStopPoint + 1 >= subpaths.size())
    };
}

void drive::Drive::StartNextTrajectory() {
    if(currentStopPoint + 1 < subpaths.size()) {
        currentStopPoint += 1;
        frc::SmartDashboard::PutNumber("currentStopPoint", currentStopPoint);

        pathCommand = std::make_unique<PPSwerveControllerCommand>(
            subpaths[currentStopPoint],
            [this]() { return this->GetPose(); },
            kinematics,
            frc2::PIDController(drive::Constants::kTrajectoryX_P, drive::Constants::kTrajectoryX_I, drive::Constants::kTrajectoryX_D),
            frc2::PIDController(drive::Constants::kTrajectoryY_P, drive::Constants::kTrajectoryY_I, drive::Constants::kTrajectoryY_D),
            frc2::PIDController(drive::Constants::kTrajectoryTheta_P, drive::Constants::kTrajectoryTheta_I, drive::Constants::kTrajectoryTheta_D),
            [this](auto states) {
                auto [fl, fr, bl, br] = states;

                this->frontleft.SetDesiredState(fl, true);
                this->frontright.SetDesiredState(fr, true);
                this->backleft.SetDesiredState(bl, true);
                this->backright.SetDesiredState(br, true);
            }
        );

        field.GetObject("trajectory")->SetTrajectory(subpaths[currentStopPoint].asWPILibTrajectory());

        // Start and schedule the path following command
        pathCommand->Schedule();
    }
}

void drive::Drive::SetTrajectory(frc::Pose2d pose) {
    bFollowTrajectory = true;
    
    frc::Pose2d robotPose = GetPose();
    std::vector<PathPoint> points = { 
        { robotPose.Translation(), robotPose.Rotation(), robotPose.Rotation() }, 
        { pose.Translation(), pose.Rotation(), robotPose.Rotation() }
    };
    subpaths = { PathPlanner::generatePath( { 1.0_mps, 1.0_mps_sq }, points) };
    currentStopPoint = -1;

    field.GetObject("end_point")->SetPose(pose);

    StartNextTrajectory();
}

void drive::Drive::SetTrajectory(const AutonomousPath path, bool resetPose) {
    bFollowTrajectory = true;
    
    // Construct a list of dynamic path constraints based on the autonomous path objects
    std::vector<pathplanner::PathConstraints> constraints = {};
    for(int i = 0; i < path.velocityOverrides.size(); i++) {
        constraints.push_back(pathplanner::PathConstraints(path.velocityOverrides[i], path.accelerationOverrides[i]));
    }

    subpaths = PathPlanner::loadPathGroup(path.pathName, constraints);
    currentStopPoint = -1;
    
    // In order to properly initialize the paths, you need to flip the trajectory based on the alliance color (red only).
    const frc::DriverStation::Alliance alliance = frc::DriverStation::GetAlliance();
    if(alliance == frc::DriverStation::Alliance::kRed) {
        for(int i = 0; i < subpaths.size(); i++) {
            subpaths[i] = pathplanner::PathPlannerTrajectory::transformTrajectoryForAlliance(subpaths[i], alliance);
        }
    }

    StartNextTrajectory();

    if(resetPose) {
        poseEstimator.ResetPosition(
            gyro.GetRotation2d(),
            { frontleft.GetPosition(), frontright.GetPosition(), backleft.GetPosition(), backright.GetPosition() },
            subpaths[currentStopPoint].getInitialHolonomicPose()
        );
    }
}

void drive::Drive::UpdateFieldTrajectory(const std::string& pathName) {
    const std::string file_path = (frc::filesystem::GetDeployDirectory() + "/pathplanner/" + pathName + ".path");
    if(std::filesystem::exists(file_path)) {
        const auto path = pathplanner::PathPlannerTrajectory::transformTrajectoryForAlliance(
            PathPlanner::loadPath(pathName, { 1.23444_mps, 0.25_mps_sq } ),
            frc::DriverStation::GetAlliance()
        );
        
        field.GetObject("trajectory")->SetTrajectory(path.asWPILibTrajectory());
    }
}

frc::Pose2d drive::Drive::GetPose() {
    return poseEstimator.GetEstimatedPosition();
} 

void drive::Drive::UpdateOdometry() {
    poseEstimator.Update(
        gyro.GetRotation2d(), 
        { frontleft.GetPosition(), frontright.GetPosition(), backleft.GetPosition(), backright.GetPosition() }
    );

    photonPoseEstimator.SetReferencePose(frc::Pose3d(poseEstimator.GetEstimatedPosition()));
    auto result = photonPoseEstimator.Update();

    if(result) {
        const auto estimate = result.value();
        
        double averageAmbiguity = 0;
        auto averageCameraToTarget = frc::Transform3d();
        int count = 0;
        for(const auto &target : estimate.targetsUsed) {
            averageAmbiguity += target.GetPoseAmbiguity();
            averageCameraToTarget = averageCameraToTarget + target.GetBestCameraToTarget();
            count += 1;
        }
        averageAmbiguity = averageAmbiguity / count;
        averageCameraToTarget = averageCameraToTarget / count;

        frc::SmartDashboard::PutNumber("targetCount", count);
        frc::SmartDashboard::PutNumber("averageAmbiguity", averageAmbiguity);
        frc::SmartDashboard::PutNumber("averageCameraToTarget_x", averageCameraToTarget.X().value());
        frc::SmartDashboard::PutNumber("averageCameraToTarget_y", averageCameraToTarget.Y().value());

        // Ignore very ambiguous pose estimations in order to avoid jitter at long distances.
        if(averageAmbiguity <= 0.05 && (averageCameraToTarget.X() < 1.25_m && averageCameraToTarget.Y() < 1.25_m)) {
            const auto pose = result.value().estimatedPose.ToPose2d();
            if(units::math::abs(pose.Rotation().Degrees() - gyro.GetRotation2d().Degrees()) <= 5_deg) {
                poseEstimator.AddVisionMeasurement(pose, result.value().timestamp);
            }
        }

        pastRobotPose = result.value().estimatedPose;
    }

    field.SetRobotPose(poseEstimator.GetEstimatedPosition());
}

void drive::Drive::ForceForward() {
    wpi::array<frc::SwerveModuleState, 4> states = wpi::array<frc::SwerveModuleState, 4> {
        frc::SwerveModuleState { 0_mps, 0_deg },
        frc::SwerveModuleState { 0_mps, 0_deg },
        frc::SwerveModuleState { 0_mps, 0_deg },
        frc::SwerveModuleState { 0_mps, 0_deg },
    };

    auto [fl, fr, bl, br] = states;

    frontleft.SetDesiredState(fl, true);
    frontright.SetDesiredState(fr, true);
    backleft.SetDesiredState(bl, true);
    backright.SetDesiredState(br, true);
}

void drive::Drive::ForceStop() {
    wpi::array<frc::SwerveModuleState, 4> states = wpi::array<frc::SwerveModuleState, 4> {
        frc::SwerveModuleState { 0_mps, frontleft.GetPosition().angle },
        frc::SwerveModuleState { 0_mps, frontright.GetPosition().angle },
        frc::SwerveModuleState { 0_mps, backleft.GetPosition().angle },
        frc::SwerveModuleState { 0_mps, backright.GetPosition().angle },
    };

    auto [fl, fr, bl, br] = states;

    frontleft.SetDesiredState(fl, true);
    frontright.SetDesiredState(fr, true);
    backleft.SetDesiredState(bl, true);
    backright.SetDesiredState(br, true);
}

void drive::Drive::XPattern() {
    wpi::array<frc::SwerveModuleState, 4> states = wpi::array<frc::SwerveModuleState, 4> {
        frc::SwerveModuleState { 0_mps, frc::Rotation2d(45_deg) },
        frc::SwerveModuleState { 0_mps, frc::Rotation2d(315_deg) },
        frc::SwerveModuleState { 0_mps, frc::Rotation2d(135_deg) },
        frc::SwerveModuleState { 0_mps, frc::Rotation2d(225_deg) },
    };

    auto [fl, fr, bl, br] = states;

    frontleft.SetDesiredState(fl, true);
    frontright.SetDesiredState(fr, true);
    backleft.SetDesiredState(bl, true);
    backright.SetDesiredState(br, true);
}
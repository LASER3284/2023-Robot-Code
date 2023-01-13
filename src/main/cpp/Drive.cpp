#include "Drive.h"

using namespace pathplanner;

drive::Drive::Drive(frc::Joystick* controller_in) {
    controller = controller_in;
    gyro.Reset();
    
    headingPIDController.EnableContinuousInput(-constants::Pi, constants::Pi);
    headingPIDController.SetTolerance(units::radian_t(2_deg).value());
    
    headingSnapPIDController.EnableContinuousInput(-constants::Pi, constants::Pi);
    headingSnapPIDController.SetTolerance(units::radian_t(2_deg).value());

    frc::SmartDashboard::PutData("headingPIDController", &headingPIDController);
    frc::SmartDashboard::PutData("headingSnapPIDController", &headingSnapPIDController);
}

void drive::Drive::SetJoystick(bool state) {
    is_joystickControl = state;
}

void drive::Drive::Tick() {
    UpdateOdometry();

    if(!is_joystickControl) return;

    bool fieldRelative = frc::SmartDashboard::GetBoolean("bFieldOriented", true);
    if(controller->GetRawButtonReleased(constants::XboxButtons::eBack)) {
        bProtectHeading = !bProtectHeading;
        desiredHeading = poseEstimator.GetEstimatedPosition().Rotation();
        frc::SmartDashboard::PutBoolean("bProtectHeading", bProtectHeading);
    }

    // Reset the gyro if the driver presses start.
    // This'll help avoid drift in field-oriented mode.
    if(controller->GetRawButtonReleased(constants::XboxButtons::eStart)) {
        gyro.Reset();
    }

    double yAxis = frc::ApplyDeadband(-controller->GetRawAxis(constants::XboxAxis::eLeftAxisY), 0.15);
    double xAxis = frc::ApplyDeadband(-controller->GetRawAxis(constants::XboxAxis::eLeftAxisX), 0.15);
    double rAxis = frc::ApplyDeadband(-controller->GetRawAxis(constants::XboxAxis::eRightAxisX), 0.15);

    if(controller->GetRawAxis(constants::XboxAxis::eRightTrigger) >= 0.25) {
        double divisorAmount = (controller->GetRawAxis(constants::XboxAxis::eRightTrigger) - 0.25) * 4;
        yAxis /= divisorAmount;
        xAxis /= divisorAmount;
        rAxis /= divisorAmount;
    }

    // These initial casts to meter_t are pretty much just to make the compiler happy. 
    // Once we multiply by the max speed is when we get the actual speed
    frc::Translation2d translation = frc::Translation2d(units::meter_t(yAxis), units::meter_t(xAxis)) * SwerveModule::kMaxSpeed.value();

    auto rotation = (rAxis * SwerveModule::kMaxAngularSpeed);

    if(bProtectHeading) {
        double angleAdjustment = headingPIDController.Calculate(
            gyro.GetRotation2d().Radians().value(),
            desiredHeading.Radians().value()
        );

        rotation *= angleAdjustment;
    }


    bool bForceAngle = false;

    // If you are field relative and hold RB, snap the robot itself to the nearest 45deg angle based on the right joystick
    if(fieldRelative && controller->GetRawButton(constants::XboxButtons::eButtonRB)) {
        // Apply a small deadband in order to avoid snapping to 90deg at rest
        if(frc::ApplyDeadband(controller->GetRawAxis(constants::XboxAxis::eRightAxisX), 0.10) != 0 || frc::ApplyDeadband(controller->GetRawAxis(constants::XboxAxis::eRightAxisY), 0.10) != 0) {
            // Convert the controller axes to radians
            const double controllerX = constants::mapScalarToRange(
                controller->GetRawAxis(constants::XboxAxis::eRightAxisX),
                -(constants::Pi / 2), (constants::Pi / 2)
            );
            const double controllerY = constants::mapScalarToRange(
                controller->GetRawAxis(constants::XboxAxis::eRightAxisY),
                -(constants::Pi / 2), (constants::Pi / 2)
            );
            
            // Calculate the angle of the controller
            const units::radian_t p = units::radian_t(atan2(controllerX, controllerY)) + 180_deg;

            // This will snap the angle from the controller to the nearest 45deg angle
            const units::degree_t snap_angle = 45_deg;
            const units::radian_t snap = frc::AngleModulus(units::degree_t(round((units::degree_t(p) / snap_angle).value()) * snap_angle.value()));

            frc::SmartDashboard::PutNumber("controllerX", controllerX);
            frc::SmartDashboard::PutNumber("controllerY", controllerY);

            frc::SmartDashboard::PutNumber("snap_angle_deg", units::degree_t(snap).value());
            frc::SmartDashboard::PutNumber("snap_angle_rad", snap.value());
            frc::SmartDashboard::PutNumber("curr_angle_rad", frc::AngleModulus(gyro.GetRotation2d().Radians()).value());
            
            // Use the PID controller to calculate our angular velocity from the given angle (measurement) & snap point (setpoint)
            const units::radians_per_second_t new_rotation =  units::radians_per_second_t(
                headingSnapPIDController.Calculate(
                    // Wraps the angle from -pi to +pi radians
                    frc::AngleModulus(gyro.GetRotation2d().Radians()).value(),
                    snap.value()
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
            headingPIDController.Reset();
        }
    } else if (fieldRelative) {
        headingPIDController.Reset();
    }

    frc::ChassisSpeeds nonrelspeeds = frc::ChassisSpeeds();
    nonrelspeeds.omega = rotation;
    nonrelspeeds.vx = translation.X() / 1_s;
    nonrelspeeds.vy = translation.Y() / 1_s;

    const auto relspeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        translation.X() / 1_s,
        translation.Y() / 1_s,
        rotation, 
        gyro.GetRotation2d()
    );

    wpi::array<frc::SwerveModuleState, 4> states = kinematics.ToSwerveModuleStates(fieldRelative ? relspeeds : nonrelspeeds);

    bool bXPattern = (controller->GetRawAxis(constants::XboxAxis::eLeftTrigger) >= 0.75);
    if(bXPattern) {
        bForceAngle = true;

        states = wpi::array<frc::SwerveModuleState, 4> {
            frc::SwerveModuleState { 0_mps, frc::Rotation2d(45_deg) },
            frc::SwerveModuleState { 0_mps, frc::Rotation2d(315_deg) },
            frc::SwerveModuleState { 0_mps, frc::Rotation2d(135_deg) },
            frc::SwerveModuleState { 0_mps, frc::Rotation2d(225_deg) },
        };
    }
    else {
        kinematics.DesaturateWheelSpeeds(
            &states, 
            fieldRelative ? relspeeds : nonrelspeeds, 
            SwerveModule::kMaxSpeed, 
            drive::Constants::maxTranslationalVelocity, 
            drive::Constants::maxRotationVelocity
        );
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
}

void drive::Drive::ResetOdometry() {
    frontleft.ResetEncoders();
    frontright.ResetEncoders();
    backleft.ResetEncoders();
    backright.ResetEncoders();

    gyro.ZeroYaw();
}

const drive::AutonomousState drive::Drive::FollowTrajectory() {
    if(pathCommand == nullptr) {
        FRC_ReportError(1, "Unable to follow trajectory; No valid path command generated");
        return { false, 9999, true };
    }

    if(!pathCommand->IsFinished()) {
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

                this->frontleft.SetDesiredState(fl);
                this->frontright.SetDesiredState(fr);
                this->backleft.SetDesiredState(bl);
                this->backright.SetDesiredState(br);
            }
        );

        // Start and schedule the path following command
        pathCommand->Schedule();
    }
}

void drive::Drive::SetTrajectory(const std::string& pathName, bool resetPose) {
    subpaths = PathPlanner::loadPathGroup(pathName, { { 4_mps, 3_mps_sq } });
    currentStopPoint = -1;
    
    StartNextTrajectory();

    if(resetPose) {
        ResetOdometry();
        poseEstimator.ResetPosition(
            gyro.GetRotation2d(),
            { frontleft.GetPosition(), frontright.GetPosition(), backleft.GetPosition(), backright.GetPosition() },
            subpaths[currentStopPoint].getInitialHolonomicPose()
        );
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
    units::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
    auto result = photonPoseEstimator.Update();

    // If we have a latency delay of 0ms, then we definitely didn't actually get some sort of result for AprilTag parsing.
    if(result.second > 0_ms) {
        units::millisecond_t timestamp = (currentTime - result.second);
        poseEstimator.AddVisionMeasurement(result.first.ToPose2d(), timestamp);
    }
}

void drive::Drive::ForceStop() {
    wpi::array<frc::SwerveModuleState, 4> states = wpi::array<frc::SwerveModuleState, 4> {
        frc::SwerveModuleState { 0_mps, frontleft.GetPosition().angle },
        frc::SwerveModuleState { 0_mps, frontright.GetPosition().angle },
        frc::SwerveModuleState { 0_mps, backleft.GetPosition().angle },
        frc::SwerveModuleState { 0_mps, backright.GetPosition().angle },
    };

    auto [fl, fr, bl, br] = states;

    frontleft.SetDesiredState(fl, false);
    frontright.SetDesiredState(fr, false);
    backleft.SetDesiredState(bl, false);
    backright.SetDesiredState(br, false);
}
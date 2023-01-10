#include "Drive.h"

using namespace pathplanner;

drive::Drive::Drive(frc::Joystick* controller_in) {
    controller = controller_in;
    gyro.Reset();
    
    headingPIDController.EnableContinuousInput(-180, 180);
    headingPIDController.SetTolerance(4.5);
    
    headingSnapPIDController.EnableContinuousInput(-constants::Pi, constants::Pi);
    headingSnapPIDController.SetTolerance(5);

    frc::SmartDashboard::PutData("headingPIDController", &headingPIDController);
    frc::SmartDashboard::PutData("headingSnapPIDController", &headingSnapPIDController);
}

void drive::Drive::SetJoystick(bool state) {
    is_joystickControl = state;
}

void drive::Drive::Tick() {
    UpdateOdometry();

    if(!is_joystickControl) return;

    bool fieldRelative = frc::SmartDashboard::GetBoolean("bFieldOriented", false);

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
    const frc::Translation2d translation = frc::Translation2d(units::meter_t(yAxis), units::meter_t(xAxis)) * SwerveModule::kMaxSpeed.value();

    if(bProtectHeading) {
        double angleAdjustment = headingPIDController.Calculate(
            poseEstimator.GetEstimatedPosition().Rotation().Degrees().value(),
            desiredHeading.Degrees().value()
        );

        rAxis = std::clamp(angleAdjustment, -0.5, 0.5);
    }

    auto rotation = (rAxis * SwerveModule::kMaxAngularSpeed);

    if(fieldRelative && controller->GetRawButtonPressed(constants::XboxButtons::eButtonRB)) {
        // Convert the controller axes to radians
        const units::radian_t controllerX = constants::mapScalarToRange<units::radian_t>(
            frc::ApplyDeadband(controller->GetRawAxis(constants::XboxAxis::eRightAxisX), 0.10),
            -(3.14159 / 2), (3.14159 / 2)
        );
        const units::radian_t controllerY = constants::mapScalarToRange<units::radian_t>(
            frc::ApplyDeadband(controller->GetRawAxis(constants::XboxAxis::eRightAxisY), 0.10),
            -(3.14159 / 2), (3.14159 / 2)
        );
        
        // Calculate the angle of the controller
        const units::radian_t p = units::math::atan2(controllerY, controllerX);

        // This will snap the angle from the controller to the nearest 45deg angle
        const units::radian_t snap_angle = 45_deg;
        const units::radian_t snap = units::radian_t(round((p / snap_angle).value() * snap_angle.value()));

        frc::SmartDashboard::PutNumber("snap_angle", units::degree_t(snap).value());

        if(!headingSnapPIDController.AtSetpoint()) {
            // Use the PID controller to calculate our angular velocity from the given angle (measurement) & snap point (setpoint)
            rotation = units::radians_per_second_t(
                headingSnapPIDController.Calculate(
                    // Wraps the angle from -pi to +pi radians
                    frc::AngleModulus(poseEstimator.GetEstimatedPosition().Rotation().Radians()).value(),
                    snap.value()
                )
            );
        }
        else {
            // If we're at the setpoint, don't even bother rotating with the joystick anymore
            rotation = 0_rad_per_s;
        }
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

    frontleft.SetDesiredState(fl, bXPattern);
    frontright.SetDesiredState(fr, bXPattern);
    backleft.SetDesiredState(bl, bXPattern);
    backright.SetDesiredState(br, bXPattern);
}

void drive::Drive::LogEncoders() {
    frc::SmartDashboard::PutNumber("fl_turn", frontleft.getTurnEncPos());
    frc::SmartDashboard::PutNumber("fl_drive", frontleft.getDriveEncPos());
    frc::SmartDashboard::PutNumber("fr_turn", frontright.getTurnEncPos());
    frc::SmartDashboard::PutNumber("fr_drive", frontright.getDriveEncPos());
    frc::SmartDashboard::PutNumber("bl_turn", backleft.getTurnEncPos());
    frc::SmartDashboard::PutNumber("bl_drive", backleft.getDriveEncPos());
    frc::SmartDashboard::PutNumber("br_turn", backright.getTurnEncPos());
    frc::SmartDashboard::PutNumber("br_drive", backright.getDriveEncPos());
}

void drive::Drive::ResetOdometry() {
    frontleft.ResetEncoders();
    frontright.ResetEncoders();
    backleft.ResetEncoders();
    backright.ResetEncoders();

    gyro.ZeroYaw();
}

bool drive::Drive::FollowTrajectory() {
    if(pathCommand == nullptr) {
        FRC_ReportError(1, "Unable to follow trajectory; No valid path command generated");
        return true;
    }
    
    if(!pathCommand->IsFinished()) {
        pathCommand->Execute();
    }

    return pathCommand->IsFinished();
}

void drive::Drive::SetTrajectory(std::string pathName, bool resetPose) {
    PathPlannerTrajectory path = PathPlanner::loadPath(pathName, { 4_mps, 3_mps_sq });
    pathCommand = std::make_unique<PPSwerveControllerCommand>(
        path, 
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

    if(resetPose) {
        ResetOdometry();
        poseEstimator.ResetPosition(
            gyro.GetRotation2d(),
            { frontleft.GetPosition(), frontright.GetPosition(), backleft.GetPosition(), backright.GetPosition() },
            path.getInitialHolonomicPose()
        );
    }

    // Start and schedule the path following command
    pathCommand->Schedule();
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
#include "Drive.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

using namespace pathplanner;
using namespace units::literals;

drive::Drive::Drive(frc::Joystick* controller_in) {
    controller = controller_in;
    gyro.Reset();
    
    headingPIDController.EnableContinuousInput(-180, 180);
    headingPIDController.SetTolerance(4.5);
    frc::SmartDashboard::PutData("headingPIDController", &headingPIDController);
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
        desiredHeading = gyro.GetRotation2d();
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
    const frc::Translation2d translation = frc::Translation2d(units::length::meter_t(yAxis), units::length::meter_t(xAxis)) * SwerveModule::kMaxSpeed.value();

    if(bProtectHeading) {
        double angleAdjustment = headingPIDController.Calculate(
            gyro.GetRotation2d().Degrees().value(), 
            desiredHeading.Degrees().value()
        );

        rAxis = std::clamp(angleAdjustment, -0.5, 0.5);
    }

    const auto rotation = (rAxis * SwerveModule::kMaxAngularSpeed);

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
        kinematics.DesaturateWheelSpeeds(&states, SwerveModule::kMaxSpeed);
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
    frc::SmartDashboard::PutNumber("currentHeading", gyro.GetRotation2d().Degrees().value());

    // TODO: Implement additional pose estimation via PhotonLib / AprilTags
    poseEstimator.Update(
        gyro.GetRotation2d(), 
        { frontleft.GetPosition(), frontright.GetPosition(), backleft.GetPosition(), backright.GetPosition() }
    );
}
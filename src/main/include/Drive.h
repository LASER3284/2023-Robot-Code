#pragma once

#include "SwerveModule.h"
#include "Constants.h"

#include <frc/Joystick.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/Errors.h>
#include <filesystem>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <frc/smartdashboard/Field2d.h>
#include <AHRS.h>

#include <memory>
#include <string>
#include <units/voltage.h>
#include <units/length.h>
#include <frc/Filesystem.h>
#include <algorithm>
#include <vector>
#include <map>
#include <vector>
#include <tuple>

namespace drive {
    class Constants {
        public:
            static constexpr units::meters_per_second_t  maxTranslationalVelocity = 4.93776_mps;
            static constexpr units::radians_per_second_t maxRotationalVelocity = 700_deg_per_s;

            static constexpr double kTrajectoryX_P = 2.5;
            static constexpr double kTrajectoryX_I = 0.0;
            static constexpr double kTrajectoryX_D = 0.0;

            static constexpr double kTrajectoryY_P = 2.5;
            static constexpr double kTrajectoryY_I = 0.0;
            static constexpr double kTrajectoryY_D = 0.0;

            static constexpr double kTrajectoryTheta_P = 2.5;
            static constexpr double kTrajectoryTheta_I = 0.0;
            static constexpr double kTrajectoryTheta_D = 0.0;
    };

    struct AutonomousState {
        bool bAtStopPoint = false;
        unsigned int currentStopIndex = -1;

        bool bCompletedPath = false;
    };

    struct AutonomousPath {
        /// @brief The "human-friendly" name of the path, used for displaying in the chooser
        std::string humanName;

        std::string pathName;

        std::vector<units::meters_per_second_t> velocityOverrides = {
            2.46888_mps
        };

        std::vector<units::meters_per_second_squared_t> accelerationOverrides = {
            0.35_mps_sq
        };
    };

    class Drive {
        public:
            Drive(frc::Joystick* controller_in);
            void SetJoystick(bool control);
            void Tick();
            void LogEncoders();

            void ResetOdometry();
            void UpdateOdometry();
            frc::Pose2d GetPose();
            units::degree_t GetPitch() { return units::degree_t(gyro.GetPitch()); }

            void DriveRelative(double power, units::meters_per_second_t maxVelocity = Constants::maxTranslationalVelocity / 2);
            void SetTrajectory(const frc::Pose2d pose);
            void SetTrajectory(const drive::AutonomousPath path, bool resetPose = false);
            void UpdateFieldTrajectory(const std::string& pathName);
            void StartNextTrajectory();
            const AutonomousState FollowTrajectory();

            void ForceForward();
            void ForceStop();
            void XPattern();

            void ForceVisionPose() {
                poseEstimator.ResetPosition(
                    gyro.GetRotation2d(), 
                    { frontleft.GetPosition(), frontright.GetPosition(), backleft.GetPosition(), backright.GetPosition() },
                    pastRobotPose.ToPose2d()
                );

                field.SetRobotPose(pastRobotPose.ToPose2d());
            }

            /// @brief Returns whether or not the current trajectory is active and not cancelled
            /// @return True if there is a valid trajectory and if the trajectory is not cancelled.
            bool GetTrajectoryActive() { return bFollowTrajectory; }
        private:
            frc::SlewRateLimiter<units::velocity::meters_per_second> xSpeedLimiter { drive::Constants::maxTranslationalVelocity / 0.0625_s };
            frc::SlewRateLimiter<units::velocity::meters_per_second> ySpeedLimiter { drive::Constants::maxTranslationalVelocity / 0.0625_s };

            frc::Translation2d frontLeftLocation { +0.277812_m, +0.277812_m };
            frc::Translation2d frontRightLocation { +0.277812_m, -0.277812_m };
            frc::Translation2d backLeftLocation { -0.277812_m, +0.277812_m };
            frc::Translation2d backRightLocation { -0.277812_m, -0.277812_m };
        
            frc::Joystick* controller;
            bool is_joystickControl;

            SwerveModule frontleft  = SwerveModule(14, 61, 3);
            SwerveModule frontright = SwerveModule(10, 33, 16);
            SwerveModule backleft   = SwerveModule(7,  62, 15);
            SwerveModule backright  = SwerveModule(11, 13, 2);

            /// @brief Current heading for which to retain for holding straight lines / staying consistent
            frc::Rotation2d desiredHeading { 0_deg };
            
            /// @brief Whether or not to keep the robot rotation heading controlled
            bool bProtectHeading = false;

            /// @brief A timer used to control the heading protection
            frc::Timer headingProtectionTimer;

            /// @brief A PID controller used to keep the robot held to a specific heading
            frc2::PIDController headingPIDController { 4.25, 0.0, 0.0 };
            
            /// @brief Used to snap the robot to specific rotation angles based on the pose angle
            frc2::PIDController headingSnapPIDController { 4.250000, 0.0, 0.1250 };

            AHRS gyro { frc::SerialPort::Port::kMXP };

            frc::SwerveDriveKinematics<4> kinematics {
                frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
            };
            
            /// @brief A PhotonPoseEstimator grabs the ""best"" pose to be used for the given AprilTags in view
            photonlib::PhotonPoseEstimator photonPoseEstimator {
                frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp),
                photonlib::MULTI_TAG_PNP,
                std::move(photonlib::PhotonCamera("mainCam")),
                frc::Transform3d(
                    frc::Translation3d(4.5_in, -2_in, 22.625_in),
                    frc::Rotation3d(0_deg, 5_deg, 180_deg)
                )
            };
            frc::Pose3d pastRobotPose = frc::Pose3d();

            frc::SwerveDrivePoseEstimator<4> poseEstimator = frc::SwerveDrivePoseEstimator(
                kinematics,
                frc::Rotation2d {},
                { frontleft.GetPosition(), frontright.GetPosition(), backleft.GetPosition(), backright.GetPosition() },
                frc::Pose2d {}
            );

            /// @brief An object representing the field for displaying the robot pose from the poseEstimator
            frc::Field2d field;

            /// @brief Whether or not the user has cancelled the current trajectory (in teleop)
            bool bFollowTrajectory = true;

            /// @brief A list/vector holding all of the paths to follow, each subpath is separated by the given stop point
            std::vector<pathplanner::PathPlannerTrajectory> subpaths;
            unsigned int currentStopPoint = -1;

            /// @brief The PPSwerveControllerCommand for the current loaded subpath / stop point
            std::unique_ptr<pathplanner::PPSwerveControllerCommand> pathCommand;
    };
}
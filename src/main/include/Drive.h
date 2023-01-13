#pragma once

#include "SwerveModule.h"
#include "Constants.h"

#include <frc/Joystick.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/StateSpaceUtil.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/controller/ProfiledPIDController.h>

#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/RobotPoseEstimator.h>

#include <AHRS.h>

#include <memory>
#include <string>
#include <units/voltage.h>
#include <units/length.h>
#include <frc/Filesystem.h>
#include <algorithm>
#include <vector>
#include <map>

namespace drive {
    class Constants {
        public:
            static constexpr units::meters_per_second_t maxTranslationalVelocity = 3.5_mps;
            static constexpr units::radians_per_second_t maxRotationVelocity = 3.75_rad_per_s;

            static constexpr double kTrajectoryX_P = 1.0;
            static constexpr double kTrajectoryX_I = 0.0;
            static constexpr double kTrajectoryX_D = 0.0;

            static constexpr double kTrajectoryY_P = 1.0;
            static constexpr double kTrajectoryY_I = 0.0;
            static constexpr double kTrajectoryY_D = 0.0;

            static constexpr double kTrajectoryTheta_P = 1.0;
            static constexpr double kTrajectoryTheta_I = 0.0;
            static constexpr double kTrajectoryTheta_D = 0.0;
    };

    struct AutonomousState {
        bool bAtStopPoint = false;
        unsigned int currentStopIndex = -1;

        bool bCompletedPath = false;
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

            void SetTrajectory(const std::string& pathName, bool resetPose = false);
            void StartNextTrajectory();
            const AutonomousState FollowTrajectory();

            void ForceStop();
        private:

            // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
            frc::SlewRateLimiter<units::scalar> xspeedLimiter { 3 / 1_s };
            frc::SlewRateLimiter<units::scalar> yspeedLimiter { 3 / 1_s };
            frc::SlewRateLimiter<units::scalar> rotLimiter { 2 / 1_s };

            frc::Translation2d frontLeftLocation { +0.37465_m, +0.37465_m };
            frc::Translation2d frontRightLocation { +0.37465_m, -0.37465_m };
            frc::Translation2d backLeftLocation { -0.37465_m, +0.37465_m };
            frc::Translation2d backRightLocation { -0.37465_m, -0.37465_m };
        
            frc::Joystick* controller;
            bool is_joystickControl;

            SwerveModule frontleft = SwerveModule(1, 2, 9, true);
            SwerveModule frontright = SwerveModule(3, 4, 10, true);
            SwerveModule backleft = SwerveModule(5, 6, 11, true);
            SwerveModule backright = SwerveModule(7, 8, 12, true);

            /// @brief Current heading for which to retain for holding straight lines / staying consistent
            frc::Rotation2d desiredHeading { 0_deg };
            
            /// @brief Whether or not to keep the robot rotation heading controlled
            bool bProtectHeading = false;

            /// @brief A PID controller used to keep the robot held to a specific heading
            frc2::PIDController headingPIDController { 0.0000001, 0, 0.0 };
            
            /// @brief Used to snap the robot to specific rotation angles based on the pose angle
            frc2::PIDController headingSnapPIDController { 4.250000, 0.0, 0.1250 };

            AHRS gyro { frc::SerialPort::Port::kUSB };

            frc::SwerveDriveKinematics<4> kinematics {
                frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
            };

            /// @brief A shared ptr managing the AprilTag field layout
            std::shared_ptr<frc::AprilTagFieldLayout> tagLayout = std::make_shared<frc::AprilTagFieldLayout>(
                (frc::filesystem::GetDeployDirectory() + "/TagLayout.json")
            );

            /// @brief A list of mappings between photonlib::PhotonCamera and Transform3d
            std::vector<std::pair<std::shared_ptr<photonlib::PhotonCamera>, frc::Transform3d>> cameras {
                { std::make_shared<photonlib::PhotonCamera>("mainCam"), frc::Transform3d() }
            };
            
            /// @brief A RobotPoseEstimator grabs the ""best"" pose to be used for the given AprilTags in view
            photonlib::RobotPoseEstimator photonPoseEstimator { tagLayout, photonlib::CLOSEST_TO_REFERENCE_POSE, cameras };
            frc::Pose3d pastRobotPose = frc::Pose3d();

            frc::SwerveDrivePoseEstimator<4> poseEstimator = frc::SwerveDrivePoseEstimator(
                kinematics,
                frc::Rotation2d {},
                { frontleft.GetPosition(), frontright.GetPosition(), backleft.GetPosition(), backright.GetPosition() },
                frc::Pose2d {}
            );

            /// @brief A list/vector holding all of the paths to follow, each subpath is separated by the given stop point
            std::vector<pathplanner::PathPlannerTrajectory> subpaths;
            unsigned int currentStopPoint = -1;

            /// @brief The PPSwerveControllerCommand for the current loaded subpath / stop point
            std::unique_ptr<pathplanner::PPSwerveControllerCommand> pathCommand;
    };
}
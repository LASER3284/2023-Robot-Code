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
#include <frc/controller/ProfiledPIDController.h>
#include <frc/Errors.h>

#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/RobotPoseEstimator.h>
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

namespace drive {
    class Constants {
        public:
            static constexpr units::meters_per_second_t  maxTranslationalVelocity = 4.93776_mps;
            static constexpr units::radians_per_second_t maxRotationalVelocity = 700_deg_per_s;

            static constexpr double kTrajectoryX_P = 7.25;
            static constexpr double kTrajectoryX_I = 0.0;
            static constexpr double kTrajectoryX_D = 0.0;

            static constexpr double kTrajectoryY_P = 7.25;
            static constexpr double kTrajectoryY_I = 0.0;
            static constexpr double kTrajectoryY_D = 0.0;

            static constexpr double kTrajectoryTheta_P = 7.25;
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

            void SetTrajectory(const frc::Pose2d pose);
            void SetTrajectory(const std::string& pathName, bool resetPose = false);
            void StartNextTrajectory();
            const AutonomousState FollowTrajectory();

            void ForceStop();

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

            SwerveModule frontleft  = SwerveModule(3, 6, 10);
            SwerveModule frontright = SwerveModule(1, 2, 9);
            SwerveModule backleft   = SwerveModule(5, 8, 11);
            SwerveModule backright  = SwerveModule(7, 4, 12);

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

            /// @brief A shared ptr managing the AprilTag field layout
            std::shared_ptr<frc::AprilTagFieldLayout> tagLayout = std::make_shared<frc::AprilTagFieldLayout>(
                (frc::filesystem::GetDeployDirectory() + "/TagLayout.json")
            );

            /// @brief A list of mappings between photonlib::PhotonCamera and Transform3d
            std::vector<std::pair<std::shared_ptr<photonlib::PhotonCamera>, frc::Transform3d>> cameras {
                { 
                    std::make_shared<photonlib::PhotonCamera>("mainCam"), 
                    frc::Transform3d(
                        frc::Translation3d(-4.5_in, 2_in, 22.625_in),
                        frc::Rotation3d(10_deg, 0_deg, 0_deg)
                    ) 
                }
            };
            
            /// @brief A RobotPoseEstimator grabs the ""best"" pose to be used for the given AprilTags in view
            photonlib::RobotPoseEstimator photonPoseEstimator { tagLayout, photonlib::PoseStrategy::LOWEST_AMBIGUITY, cameras };
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
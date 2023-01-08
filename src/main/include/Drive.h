/*
 * File created by Charlotte Patton
 */

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
#include <AHRS.h>
#include <memory>
#include <string>
#include <units/voltage.h>
#include <units/length.h>
#include <frc/Filesystem.h>
#include <algorithm>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <vector>
#include <map>
namespace drive {
    class Constants {
        public:
            //static constexpr double kDefaultS = 0.0_V;
            //static constexpr double kDefaultV = 0.0_V * 1_s / 1_m;
            //static constexpr double kDefaultA = 0.0_V * 1_s / 1_m;

            static constexpr double kTrajectoryX_P = 0.0;
            static constexpr double kTrajectoryX_I = 0.0;
            static constexpr double kTrajectoryX_D = 0.0;

            static constexpr double kTrajectoryY_P = 0.0;
            static constexpr double kTrajectoryY_I = 0.0;
            static constexpr double kTrajectoryY_D = 0.0;

            static constexpr double kTrajectoryTheta_P = 0.0;
            static constexpr double kTrajectoryTheta_I = 0.0;
            static constexpr double kTrajectoryTheta_D = 0.0;
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

            void SetTrajectory(std::string pathName, bool resetPose = false);
            bool FollowTrajectory();
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

            SwerveModule frontleft = SwerveModule(1, 2, 9);
            SwerveModule frontright = SwerveModule(3, 4, 10, true);
            SwerveModule backleft = SwerveModule(5, 6, 11);
            SwerveModule backright = SwerveModule(7, 8, 12);

            frc::Rotation2d desiredHeading { 0_deg };
            bool bProtectHeading = false;
            frc2::PIDController headingPIDController { 0.0000001, 0, 0.0 };

            AHRS gyro { frc::SerialPort::Port::kUSB };

            frc::SwerveDriveKinematics<4> kinematics {
                frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
            };

            // TODO: Implement AprilTagFieldLayout
            // frc::AprilTagFieldLayout layout { frc::filesystem::GetDeployDirectory() + "/TagLayout.json" };

            // TODO: Use PhotonLib once it gets built for 2023
            
            frc::SwerveDrivePoseEstimator<4> poseEstimator = frc::SwerveDrivePoseEstimator(
                kinematics,
                frc::Rotation2d {},
                { frontleft.GetPosition(), frontright.GetPosition(), backleft.GetPosition(), backright.GetPosition() },
                frc::Pose2d {},
                
                // Standard deviations of model states. 
                // Increase these numbers to trust your model's state estimates less. 
                // This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
                {0.1, 0.1, 0.1},

                // Standard deviations of the vision measurements. 
                // Increase these numbers to trust global measurements from vision less. 
                // This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
                { 0.1, 0.1, 0.1 }
            );
            
            std::unique_ptr<pathplanner::PPSwerveControllerCommand> pathCommand;
    };
}
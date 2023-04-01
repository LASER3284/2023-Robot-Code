// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <string>
#include <tuple>
#include <frc/Errors.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/XboxController.h>

#include "FieldConstants.h"
#include "SwerveModule.h"
#include "Drive.h"
#include "Lights.h"
#include "Shoulder.h"
#include "Intake.h"
#include "Arm.h"
#include "Wrist.h"
#include "Kinematics.h"
#include <deque>
#include <vector>

class Robot : public frc::TimedRobot {
    public:
        Robot();
        ~Robot();
        void RobotInit() override;
        void RobotPeriodic() override;
        void AutonomousInit() override;
        void AutonomousPeriodic() override;
        void TeleopInit() override;
        void TeleopPeriodic() override;
        void DisabledInit() override;
        void DisabledPeriodic() override;
        void TestInit() override;
        void TestPeriodic() override;
        void SimulationInit() override;
        void SimulationPeriodic() override;

    private:
        frc::Joystick driveController { 0 };
        frc::XboxController auxController { 1 };

        /// @brief A sendable chooser / dropdown for selecting the current autonomous trajectory
        frc::SendableChooser<drive::AutonomousPath> m_chooser;

        /// @brief A string representing the currently selected ""human"" trajectory name
        std::string currentAutonomousState;

        /// @brief The subsystem for handling all of the swerve drive train tasks
        drive::Drive drivetrain { &driveController };

        /// @brief A basic subsystem for handling the WS128b LED strips
        lights::LightHandler lighthandler;

        /// @brief The subsystem for handling/controlling shoulder movements
        shoulder::Shoulder shoulder;

        /// @brief The subsystem for handling/controlling arm extensions
        arm::Arm arm;

        /// @brief The system that handles the intake and whether it is cone or cube.
        intake::Intake intake;

        /// @brief The subsystem that handles wrist motion/controls
        wrist::Wrist wrist;

        /// @brief A map filled with the "human friendly" path name and the actual pathplanner file name saved in the deploy folder
        const std::vector<drive::AutonomousPath> mTrajectoryMap {
            { "Mid Balance", "MidBalance"},
            { "Mobile", "Mobile"},
            { "Mobile Mid-Cone", "MobileCone"},
            { "Mid Cone Balance", "MidBalance"},
            /*
            { 
                "Mid Cone Cube", "ConeCube",
                { 2.46888_mps, 2.46888_mps },
                { 0.4_mps_sq, 1_mps_sq }
            },
            { 
                "Mid-Cone Cube Balance", "ConeCubeBalance",
                { 2.46888_mps, 2.46888_mps },
                { 0.4_mps_sq, 2.46888_mps_sq }
            },
            */ 
            { "Far Mobile", "FarMobile"},
            { "Far Mobile Mid-Cone", "FarMobileCone"},
            { "Far Mobile Mid-Cone Balance", "FarMobileConeBalance"},
            { 
                "Far Mid Cone Cone", "FarConeCone",
                { 2.46888_mps, 2.46888_mps },
                { 0.35_mps_sq, 1.25_mps_sq }
            },
            { 
                "Mid Cone Cone", "MidConeCone"
            }
            /*
            { 
                "Far Mid Cone Cube", "FarConeCube",
                { 2.46888_mps, 2.46888_mps },
                { 0.35_mps_sq, 1.25_mps_sq }
            },
            { 
                "Far Mid-Cone Cube Balance", "FarMobileConeCubeBalance",
                { 2.46888_mps, 2.46888_mps },
                { 0.35_mps_sq, 2_mps_sq }
            },
            */
        };

        /// @brief Whether or not the robot has processed the starting action within auto
        bool bHasProcessedStartingAction = false;

        /// @brief Whether or not the drive train has been force stopped for auto
        bool bDriveTrainStopped = false;

        /// @brief An object that represents the locations of all the field elements
        constants::FieldConstants fieldConstants;
        
        /// @brief A boolean tracking if the robot is currently trying to align to the current grid position
        bool aligningToGrid = false;

        /// @brief Whether or not the robot is currently trying to rotate the arm to the current scoring position
        bool bAligningToScoringLocation = false;

        /// @brief The currently targeted scoring location to rotate the arm to
        /// This only applies when aligningToGrid is currently true
        frc::Translation3d targetedScoringLocation;

        /// @brief Stores the most recent POV value given from the aux controller
        double storedPOVValue = -1;

        frc::Timer POVDebouncer;

        /// @brief Whether or not the robot was inside as of the last tick
        bool bWasInCommunity = false;

        /// @brief Whether or not the robot is currently inside the community as of right now
        bool bInsideCommunity = false;

        int currentGrid = -1;

        /// @brief A timer object used for timing the rumble of the main driving controller
        frc::Timer rumbleTimer;

        /// @brief A timer object used to track the initial timing of the alignment process
        frc::Timer alignmentTimer;

        /// @brief A timer object used to track the delay for flipping rotational controls.
        /// If the controller has not been touched for x amount of seconds, flip the voltage signs.
        frc::Timer rotationalTimer;

        bool rotationalFlip = false;

        /// @brief A timer used for tracking time durations during auto in order to avoid ""race conditions"" in robot actions
        frc::Timer autoTimer;

        bool autoTimerRunning = false;

        bool bHasAutoBalanced = false;
        bool bHasStartedBalancing = false;
        units::degree_t lastPitch = 0_deg;
};

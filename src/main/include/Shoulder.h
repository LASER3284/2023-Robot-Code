#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/time.h>
#include <units/voltage.h>
#include <frc/controller/PIDController.h>
#include <frc/filter/MedianFilter.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/AnalogEncoder.h>
#include <frc/AnalogInput.h>
#include <frc/RobotController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include "Kinematics.h"
#include "Constants.h"


namespace shoulder {

    class Constants {
        public:
            /// @brief CAN ID for the TalonFX to be used for the NEO on the shoulder.
            static constexpr int kShoulderMotorID = 18;

            /// @brief The CAN id for the follower TalonFX motor
            static constexpr int kShoulderFollowerID = 35;

            /// @brief PWM Slot ID for the encoder to be used to measure the angle of the arm on the shoulder.
            static constexpr int kShoulderPortID = 0;

            /// @brief The angle offset of the arm where 0_deg is facing forward, horizontally
            static constexpr units::degree_t kAngleOffset = 185.0_deg;

            /// @brief The gear ratio from the main encoder to the final sprocket/arm shaft
            static constexpr double gearRatio = 1 / 261.8182;

            // Additionally: 837.8182

            /// @brief The maximum achievable rotation of the arm
            static constexpr units::degree_t maxRotation = 272.1_deg;

            static constexpr units::volt_t kS = 0.10116_V;
            static constexpr units::volt_t kG = 0.038945_V;
            static constexpr auto kV = 4.39_V / 1_rad_per_s;
            static constexpr auto kA = 0.1843_V / 1_rad_per_s_sq;
    };

    class Shoulder {
        public:            
            /// @brief Initializes SparkMAX and such
            Shoulder();
            
            /// @brief Periodically updates the shoulder PID controller & navigates to the goal
            void Tick(units::meter_t armExtension);

            /// @brief Gets the current rotation of the shoulder
            ///
            /// The rotation is 0 degrees parallel to the bottom of the frame.
            /// @return The current rotation of the shoulder in units::degree_t.
            units::degree_t GetRotation();

            units::degree_t GetRotationInverse();

            /// @brief Gets the current angular velocity of the shoulder
            /// @return Returns the current angular velocity of the shoulder
            units::degrees_per_second_t GetVelocity();

            /// @brief Sets the current goal for the shoulder angle
            ///
            /// The rotation is 0 degrees parallel to the bottom of the frame.
            /// @param rot The rotation goal to use.
            void SetRotationGoal(units::degree_t rot);

            units::degree_t GetRotationalGoal() {
                return shoulderGoal.position;
            }

            /// @brief Manually controls the arm with a set percentage
            /// @param percentage The percent output to move the motors with
            void ManualControl(double percentage) { 
                manualPercentage = percentage; 
            }

            /// @brief Edit and adjust the base voltage to keep the arm in place in the air for manual control
            /// @param ff The voltage to hold the arm at
            void AdjustFeedforward(units::volt_t ff) {
                feedforward = ff;
            }

            /// @brief Toggles control of the arm 
            /// @param enable Whether or not to enable control of the arm
            void ToggleControl(bool enable) {
                bEnabled = enable;
            }

            void RefreshController() { 
                shoulderSetpoint = { GetRotation(), 0_rad_per_s };
                shoulderGoal = { GetRotation(), 0_rad_per_s };
                shoulderTimer.Restart();
            }

            void ResetRotation() {
                manualOffset = 0_deg;
                manualOffset = (90_deg - GetRotation());
                RefreshController();
                shoulderTimer.Restart();
            }
        private:
            void _set_motor_voltage(units::volt_t volts) {
                motor.SetVoltage(volts);
                followerMotor.SetVoltage(volts);
            }

            /// @brief The main motor for driving the rotation of the shoulder
            ctre::phoenix::motorcontrol::can::WPI_TalonFX motor { Constants::kShoulderMotorID };

            /// @brief The secondary follower motor for rotating the shoulder
            ctre::phoenix::motorcontrol::can::WPI_TalonFX followerMotor { Constants::kShoulderFollowerID };

            /// @brief The absolute encoder (thrifty encoder) used for locating the shoulder
            frc::AnalogEncoder encoder { Constants::kShoulderPortID };

            frc::ArmFeedforward angleFeedForward {Constants::kS, Constants::kG, Constants::kV, Constants::kA};

            bool bEnabled = false;
            double manualPercentage = 0.0;
            units::volt_t feedforward = 0_V;

            /// @brief The trapezoidal profile constraints for the shoulder rotation
            /// This specifies the max rotational velocity *and* the max rotational acceleration
            /// Ideally this would be in the constants but it would not let me do that.
            frc::TrapezoidProfile<units::degrees>::Constraints defaultRotationalConstraints { 90_deg_per_s, 125_deg_per_s_sq };

            /// @brief The constraints for a profile thats going towards the bumper (ground)
            frc::TrapezoidProfile<units::radians>::Constraints slowRotationalConstraints { 35_deg_per_s, 30_deg_per_s_sq };

            frc::PIDController angleController {
                0.78706, // kP
                0.0, // kI
                0.0 // kD
            };

            /// @brief The current goal to rotate the shoulder to
            frc::TrapezoidProfile<units::degrees>::State shoulderGoal;

            /// @brief The current setpoint for the shoulder rotation
            frc::TrapezoidProfile<units::degrees>::State shoulderSetpoint;

            /// @brief A timer used for overriding the manual percentage vs the feedforward calculations
            frc::Timer shoulderTimer;

            units::degree_t manualOffset = 0_deg;
            units::degree_t encoderSeed = 0_deg;

            frc::MedianFilter<units::degree_t> angle_filter {7};
            frc::MedianFilter<units::volt_t> output_filter {5};
    };
}

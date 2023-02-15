#pragma once

#include <rev/CANSparkMax.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/time.h>
#include <units/voltage.h>
#include <frc/controller/PIDController.h>
#include <frc/AnalogEncoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

namespace shoulder {

    class Constants {
        public:
            /// @brief CAN ID for the SparkMAX to be used for the NEO on the shoulder.
            static constexpr int kShoulderMotorID = 18;

            /// @brief The CAN id for the follower spark max motor
            static constexpr int kShoulderFollowerID = 35;

            /// @brief PWM Slot ID for the encoder to be used to measure the angle of the arm on the shoulder.
            static constexpr int kShoulderPortID = 1;

            /// @brief The angle offset of the arm where 0_deg is facing forward, horizontally
            static constexpr units::degree_t kAngleOffset = 74.2_deg;

            /// @brief The gear ratio from the main encoder to the final sprocket/arm shaft
            static constexpr double gearRatio = 1 / 261.8182;

            /// @brief The maximum rotational velocity of the arm, used for feedforward calculations
            static constexpr units::radians_per_second_t maxRotationalVelocity = 1_deg_per_s;

            /// @brief The maximum achievable rotation of the arm
            static constexpr units::degree_t maxRotation = 272.1_deg;
    };

    class Shoulder {
        public:            
            /// @brief Initializes SparkMAX and such
            Shoulder();
            
            /// @brief Periodically updates the shoulder PID controller & navigates to the goal
            void Tick();

            /// @brief Gets the current rotation of the shoulder
            ///
            /// The rotation is 0 degrees parallel to the bottom of the frame.
            /// @return The current rotation of the shoulder in units::degree_t.
            units::degree_t GetRotation();

            /// @brief Gets the current angular velocity of the shoulder
            /// @return Returns the current angular velocity of the shoulder
            units::degrees_per_second_t GetVelocity();

            /// @brief Sets the current goal for the shoulder angle
            ///
            /// The rotation is 0 degrees parallel to the bottom of the frame.
            /// @param rot The rotation goal to use.
            void SetRotationGoal(units::degree_t rot);

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

        private:
            /// @brief The main motor for driving the rotation of the shoulder
            rev::CANSparkMax motor { Constants::kShoulderMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
            rev::SparkMaxRelativeEncoder mainEncoder = motor.GetEncoder();

            /// @brief The secondary follower motor for rotating the shoulder
            rev::CANSparkMax followerMotor { Constants::kShoulderFollowerID, rev::CANSparkMaxLowLevel::MotorType::kBrushless };

            /// @brief The absolute encoder (thrifty encoder) used for locating the shoulder
            frc::AnalogEncoder encoder { Constants::kShoulderPortID };

            frc2::PIDController angleController { 
                0.0, // kP
                0.0, // kI
                0.0, // kD
            };

            bool bEnabled = false;
            double manualPercentage = 0.0;
            units::volt_t feedforward = 0_V;
    };
}

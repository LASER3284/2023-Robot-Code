#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/ProfiledPIDController.h>

namespace wrist {

    class Constants {
        public:
            /// @brief The CAN ID of the SparkMAX to use for the NEO 550 on the wrist.
            /// @warning Value set to 999 because real value is unknown.
            static constexpr int kWristMotorID = 999;

            /// @brief The CAN ID of the CANCoder to use for rotation of the wrist.
            /// @warning Value set to 998 because real value is unknown.
            static constexpr int kWristEncoderID = 998;

            /// @brief The gear ratio from the NEO 550 to the output of the wrist.
            /// @warning This value DOES NOT include the pulley ratio.
            /// @todo Find pulley ratio and include it.
            static constexpr double kWristGearRatio = 90 / 1;
    };

    class Wrist {
        public:
            /// @brief Constructor for the Wrist class, which also acts to initialize the class.
            Wrist();
            
            /// @brief Calculates the percent output to set the motor to based on the angle of the wrist.
            void Tick();
            
            /// @brief Returns the rotation of the output shaft.
            /// @return Rotation of output shaft, 0 degrees is parallel to the bottom of the frame.
            units::degree_t GetRotation();
            
            /// @brief Sets the rotation goal for the profiled PID controller.
            ///
            /// Rotation should be 0 degrees parallel to the bottom of the frame in CCW+ orientation.
            /// @param rot The rotation goal to set the profiled PID controller to calculate against.
            void SetRotationGoal(units::degree_t rot);
        private:
            rev::CANSparkMax motor { Constants::kWristMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless };

            ctre::phoenix::sensors::CANCoder encoder { Constants::kWristEncoderID };
            
            frc::ProfiledPIDController<units::degree> controller {
                0.0,    // kP
                0.0,    // kI
                0.0,    // kD
                // Trapezoidal profile for the constraints that we're looking for
                frc::TrapezoidProfile<units::degree>::Constraints { 1_deg_per_s, 1_deg_per_s / 1_s }
            };
    };

}

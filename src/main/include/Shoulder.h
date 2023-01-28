#pragma once

#include <rev/CANSparkMax.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <frc/controller/ProfiledPIDController.h>
#include <ctre/phoenix/sensors/CANCoder.h>

namespace shoulder {

    class Constants {
        public:
            /// @brief CAN ID for the SparkMAX to be used for the NEO on the shoulder.
            /// @warning This value is currently 999 because the real value is not yet known.
            static constexpr int kShoulderMotorID = 999;

            /// @brief CAN ID for the CANCoder to be used to measure the angle of the arm on the shoulder.
            /// @warning This value is currently 998 because the real value is not yet known.
            static constexpr int kShoulderCancoderID = 998;
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

            /// @brief Sets the current goal for the shoulder angle
            ///
            /// The rotation is 0 degrees parallel to the bottom of the frame.
            /// @param rot The rotation goal to use for the profiled PID controller.
            void SetRotationGoal(units::degree_t rot);

        private:
            rev::CANSparkMax motor { Constants::kShoulderMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless };

            ctre::phoenix::sensors::CANCoder encoder { Constants::kShoulderCancoderID };

            frc::ProfiledPIDController<units::degree> angleController { 
                0.0, // kP
                0.0, // kI
                0.0, // kD
                // The trapezoid profile provides a set of constraints for the PID controller to obey.
                frc::TrapezoidProfile<units::degree>::Constraints { 1_deg_per_s, 1_deg_per_s / 1_s }
            };
    };

}

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace wrist {

    class Constants {
        public:
            /// @brief The CAN ID of the SparkMAX to use for the NEO 550 on the wrist.
            static constexpr int kWristMotorID = 56;

            /// @brief The gear ratio from the NEO 550 to the output of the wrist.
            static constexpr double kWristGearRatio = (90 / 1) * ( (24 / 36) / 1);

            /// @brief The starting angle for the wrist
            static constexpr units::degree_t kStartingAngle = 0_deg;
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

            /// @brief Manually control the rotation of the wrist
            void ManualControl(double percentage) { manualPercentage = percentage; }
        private:
            rev::CANSparkMax wristMotor { Constants::kWristMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
            rev::SparkMaxRelativeEncoder wristEncoder = wristMotor.GetEncoder();

            frc::ProfiledPIDController<units::degree> controller {
                0.0,    // kP
                0.0,    // kI
                0.0,    // kD
                // Trapezoidal profile for the constraints that we're looking for
                frc::TrapezoidProfile<units::degree>::Constraints { 1_deg_per_s, 1_deg_per_s / 1_s }
            };

            double manualPercentage = 0.0;
    };

}

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/time.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace wrist {

    class Constants {
        public:
            /// @brief The CAN ID of the SparkMAX to use for the NEO 550 on the wrist.
            static constexpr int kWristMotorID = 56;

            /// @brief The gear ratio from the NEO 550 to the output of the wrist.
            static constexpr double kWristGearRatio = 90.0 * (24.0 / 36.0);

            /// @brief The starting angle for the wrist
            static constexpr units::degree_t kStartingAngle = 90_deg;

            static constexpr units::volt_t kG = 0.35_V;
            static constexpr auto kV = 0.59_V / 1_rad_per_s;
            static constexpr units::volt_t kS = 0.0_V;
            static constexpr auto kA = 0.0_V / 1_rad_per_s_sq;

            static constexpr units::degrees_per_second_t kMaxRotationalVelocity = 10_deg_per_s;
            static constexpr units::degree_t kMaxAngle = 185_deg;
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
            
            /// @brief Sets the rotation goal for the PID controller and feed forward.
            ///
            /// Rotation should be 0 degrees parallel to the bottom of the frame in CCW+ orientation.
            /// @param rot The rotation goal to rotate the wrist to.
            void SetRotationGoal(units::degree_t rot);

            /// @brief Manually control the rotation of the wrist
            void ManualControl(double percentage) { manualPercentage = percentage; }
        private:
            rev::CANSparkMax wristMotor { Constants::kWristMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
            rev::SparkMaxRelativeEncoder wristEncoder = wristMotor.GetEncoder();

            frc2::PIDController controller {
                1.2374,    // kP
                0.0,    // kI
                0.26825,    // kD
            };

            frc::ArmFeedforward feedforward { Constants::kS, Constants::kG, Constants::kV, Constants::kA };

            double manualPercentage = 0.0;
            units::radian_t goal = 0_deg;
    };

}

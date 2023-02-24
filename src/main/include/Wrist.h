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
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/Timer.h>

namespace wrist {

    class Constants {
        public:
            /// @brief The CAN ID of the SparkMAX to use for the NEO 550 on the wrist.
            static constexpr int kWristMotorID = 60;

            /// @brief The gear ratio from the NEO 550 to the output of the wrist.
            static constexpr double kWristGearRatio = 90.0 * (24.0 / 36.0);

            /// @brief The starting angle for the wrist
            static constexpr units::degree_t kStartingAngle = -145_deg;

            static constexpr units::volt_t kG = 0.70606_V;
            static constexpr auto kV = 0.69_V / 1_rad_per_s;

            static constexpr units::degree_t kMaxAngle = 185_deg;
    };

    class Wrist {
        public:
            /// @brief Constructor for the Wrist class, which also acts to initialize the class.
            Wrist();
            
            /// @brief Calculates the percent output to set the motor to based on the angle of the wrist.
            void Tick(units::degree_t shoulderRotation);
            
            /// @brief Returns the current rotation of the output shaft.
            /// @return Rotation of output shaft, 0 degrees is parallel to the bottom of the frame.
            units::degree_t GetRotation();
            
            /// @brief Returns the most recent commanded setpoint of the output shaft
            /// @return The most recently commanded angle of the wrist
            units::degree_t GetLastSetpoint() { return lastSetpoint; }

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
                0.0,    // kP
                0.0,    // kI
                0.0,    // kD
            };

            double manualPercentage = 0.0;

            /// @brief The trapezoidal profile constraints for the shoulder rotation
            /// This specifies the max rotational velocity *and* the max rotational acceleration
            /// Ideally this would be in the constants but it would not let me do that.
            frc::TrapezoidProfile<units::radians>::Constraints rotationalConstraints { 120_deg_per_s, 360_deg_per_s_sq };

            /// @brief The current goal to rotate the shoulder to
            frc::TrapezoidProfile<units::radians>::State wristGoal;

            /// @brief The current setpoint for the shoulder rotation
            frc::TrapezoidProfile<units::radians>::State wristSetpoint;

            /// @brief A timer used for overriding the manual percentage vs the feedforward calculations
            frc::Timer wristTimer;
            
            /// @brief The last commanded setpoint for the wrist
            units::degree_t lastSetpoint;
    };

}

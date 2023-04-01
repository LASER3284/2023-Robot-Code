#pragma once

#include "FieldConstants.h"

#include <frc/controller/PIDController.h>
#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <units/voltage.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angle.h>

namespace shooter {
    class Constants {
        public:
            static constexpr int kFlywheelMotorId = 9999;

            static constexpr double kFlywheelRatio = 1.4;

            static constexpr int kIntakeMotorId = 9998;

            static constexpr int kSolenoidChannel = 9997;

            static constexpr double kP = 0.0;
            static constexpr double kI = 0.0;
            static constexpr double kD = 0.0;

            static constexpr auto kV = 0.0_V * 1.0_rad_per_s;
            static constexpr auto kA = 0.0_V * 1.0_rad_per_s_sq;
            static constexpr auto kS = 0.0_V;

            static constexpr auto kGroundSetpoint = 0.0_rad_per_s;
            static constexpr auto kMidSetpoint = 0.0_rad_per_s;
            static constexpr auto kHighSetpoint = 0.0_rad_per_s;
    };

    class CubeShooter {
        public:
            CubeShooter();

            /// @brief Deploy the intake motors such that we intake a cube.
            /// @param intake A boolean value saying whether to spin the intake motor.
            void Deploy(bool intake);

            /// @brief Retract the intake and stop the motors unless we're shooting.
            void Retract();

            /// @brief Shoot the cube for a specific height.
            /// @param height The height to set the shooter to aim for.
            void Shoot(constants::FieldConstants::GridHeights height);

        private:
            rev::CANSparkMax flywheelMotor { Constants::kFlywheelMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless };

            rev::SparkMaxRelativeEncoder flywheelEnc = flywheelMotor.GetEncoder();

            rev::CANSparkMax intakeMotor { Constants::kIntakeMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless };

            frc::Solenoid deploySolenoid { frc::PneumaticsModuleType::REVPH, Constants::kSolenoidChannel };

            frc2::PIDController flywheelController { 
                Constants::kP,
                Constants::kI,
                Constants::kD
            };

            frc::SimpleMotorFeedforward<units::radians> flywheelFF {
                Constants::kS,
                Constants::kV,
                Constants::kA
            };
    };
};
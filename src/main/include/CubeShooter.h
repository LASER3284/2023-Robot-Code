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
#include <frc/Timer.h>

namespace shooter {
    class Constants {
        public:
            static constexpr int kFlywheelMotorId = 39;

            static constexpr double kFlywheelRatio = 1.4;
            static constexpr double kIntakeRatio = 1.56;

            static constexpr int kIntakeMotorId = 59;

            static constexpr int kSolenoidChannel = 8;

            static constexpr double kIntakeP = 1.2551E-07;
            static constexpr double kIntakeI = 0.0;
            static constexpr double kIntakeD = 0.0;

            static constexpr auto kIntakekV = 0.031185_V / 1.0_rad_per_s;
            static constexpr auto kIntakekA = 0.0011115_V / 1.0_rad_per_s_sq;
            static constexpr auto kIntakekS = 0.36342_V;

            static constexpr double kFlywheelP = 8.0966E-44;
            static constexpr double kFlywheelI = 0.0;
            static constexpr double kFlywheelD = 0.0;

            static constexpr auto kFlywheelkV = 0.028584_V / 1.0_rad_per_s;
            static constexpr auto kFlywheelkA = 0.0007214_V / 1.0_rad_per_s_sq;
            static constexpr auto kFlywheelkS = 0.16201_V;

            static constexpr auto kGroundSetpoint = -210_rad_per_s;
            static constexpr auto kMidSetpoint = -130_rad_per_s;
            static constexpr auto kHighSetpoint = -360_rad_per_s;
            static constexpr auto kSpitSetpoint = -420_rad_per_s;
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

            units::radians_per_second_t GetAngularFlywheelVelocity() {
                return units::radians_per_second_t(flywheelEnc.GetVelocity());
            }

            units::radians_per_second_t GetAngularIntakeVelocity() {
                return units::radians_per_second_t(intakeEnc.GetVelocity());
            }

            double GetFlywheelOutput() {
                return flywheelMotor.GetAppliedOutput();
            }

            double GetIntakeOutput() {
                return intakeMotor.GetAppliedOutput();
            }

            units::ampere_t GetFlywheelCurrent() {
                return units::ampere_t { flywheelMotor.GetOutputCurrent() };
            }

            bool IsDeployed() {
                return isDeployed;
            }

            bool HasElement() {
                return (GetFlywheelOutput() > 0.0) && (GetAngularFlywheelVelocity() <= 50000_rad_per_s && GetAngularFlywheelVelocity() > -10000_rad_per_s);
            }
        private:
            rev::CANSparkMax flywheelMotor { Constants::kFlywheelMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless };

            rev::SparkMaxRelativeEncoder flywheelEnc = flywheelMotor.GetEncoder();

            rev::CANSparkMax intakeMotor { Constants::kIntakeMotorId, rev::CANSparkMaxLowLevel::MotorType::kBrushless };

            rev::SparkMaxRelativeEncoder intakeEnc = intakeMotor.GetEncoder();

            frc::Solenoid deploySolenoid { frc::PneumaticsModuleType::REVPH, Constants::kSolenoidChannel };

            frc2::PIDController flywheelController { 
                Constants::kFlywheelP,
                Constants::kFlywheelI,
                Constants::kFlywheelD
            };
            frc::SimpleMotorFeedforward<units::radians> flywheelFF {
                Constants::kFlywheelkS,
                Constants::kFlywheelkV,
                Constants::kFlywheelkA
            };

            frc2::PIDController intakeController { 
                Constants::kIntakeP,
                Constants::kIntakeI,
                Constants::kIntakeD
            };
            frc::SimpleMotorFeedforward<units::radians> intakeFF {
                Constants::kIntakekS,
                Constants::kIntakekV,
                Constants::kIntakekA
            };

            frc::Timer deployTimer;
            bool isDeployed = false;
    };
};
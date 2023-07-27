#pragma once

#include "FieldConstants.h"
#include <memory>
#include <units/angular_velocity.h>
#include <map>
#include <units/angle.h>
#include <units/voltage.h>
#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>

namespace shooter {
    namespace constants {
        /// @brief Top Cubert Falcon
        constexpr int kUppie = 44;
        /// @brief Bottom Cubert Falcon
        constexpr int kDownie = 45;
        /// @brief Deploying NEO
        constexpr int kDeployNeo = 46;

        /// @brief Angle offset for thru-bore (this is subtracted from the
        /// reported angle).
        constexpr units::degree_t kThruBoreAngleOffset = 0.0_deg;

        /// @brief Proportional Gain for the deploy controller
        constexpr double kDeployP = 0.099122;
        /// @brief Integral Gain for the deploy controller
        constexpr double kDeployI = 0.0;
        /// @brief Derivative Gain for the deploy controller
        constexpr double kDeployD = 0.0084273;
        
        /// @brief Proportional Gain for the roller controller
        constexpr double kRollerP = 0.0;
        /// @brief Integral Gain for the roller controller
        constexpr double kRollerI = 0.0;
        /// @brief Derivative Gain for the roller controller
        constexpr double kRollerD = 0.0;

        /// @brief Roller speed in RPM
        constexpr units::revolutions_per_minute_t kRollerSetpoint = 9000.0_rpm;
        /// @brief Roller intake speed in RPM
        constexpr units::revolutions_per_minute_t kRollerIntakeSetpoint = -kRollerSetpoint / 2;

        constexpr units::degree_t kUpperLimit = 103.6_deg;
        constexpr units::degree_t kLowerLimit = 222.3_deg;

        /// @brief This is a map of GridHeights to degree values for where the
        /// Cubert should position itself.
        const std::map<
            ::constants::FieldConstants::GridHeights,
            units::degree_t
        > kAngleGridMap = {
            { ::constants::FieldConstants::GridHeights::eUp, kUpperLimit },
            { ::constants::FieldConstants::GridHeights::eMid, kUpperLimit },
            { ::constants::FieldConstants::GridHeights::eGround, kLowerLimit },
            { ::constants::FieldConstants::GridHeights::eIntake, kLowerLimit },
            { ::constants::FieldConstants::GridHeights::eGroundSpit, kLowerLimit },
            { ::constants::FieldConstants::GridHeights::eStopped, kUpperLimit },
        };

        constexpr double kRollerRatio = 2.0 / 3.0;
    }

    class Cubert {
        public:
            /// @brief Initializes the motors and ensures break mode on the NEO.
            void Init();

            /// @brief Updates SmartDashboard values; should be called in
            /// RobotPeriodic().
            void Tick();

            /// @brief Returns whether the cubert was last deployed or retracted.
            /// @return True if deployed, false otherwise.
            bool IsDeployed() {
                return isDeployed;
            }

            /// @todo Actually implement
            /// @brief Returns whether the shooter has obtained a game element.
            /// @return false for now.
            bool HasElement() {
                return false;
            }

            /// @brief Shoot the cube for a specific height.
            /// @param height The height to set the shooter to aim for.
            void Shoot(::constants::FieldConstants::GridHeights height);

            units::degree_t GetAngle() {
                return units::degree_t{thruboreEnc.Get()}
                    - constants::kThruBoreAngleOffset;
            }

        private:
            /// @brief Stops the roller motors completely.
            void _stop_rollers();

            /// @brief Sets the voltage of the roller motors.
            /// @param volts The voltage to set the motors at.
            void _set_rollers(units::volt_t volts);

            /// @brief Sets the voltage of the deploy motor.
            /// @param volts The voltage to apply.
            void _set_deploy(units::volt_t volts);

            /// @brief The motor that deploys the Cubert
            rev::CANSparkMax deployMotor {
                constants::kDeployNeo,
                rev::CANSparkMaxLowLevel::MotorType::kBrushless
            };

            /// @brief The encoder to measure the absolute angle of the Cubert
            frc::DutyCycleEncoder thruboreEnc { 1 };

            /// @brief The PID controller for the position of the Cubert
            frc2::PIDController deployController { 
                constants::kDeployP,
                constants::kDeployI,
                constants::kDeployD
            };

            /// @brief The downie Falcon 500 as std::unique_ptr
            std::unique_ptr<
                ctre::phoenix::motorcontrol::can::WPI_TalonFX
            > downieMotor = std::unique_ptr<
                ctre::phoenix::motorcontrol::can::WPI_TalonFX
            >(new ctre::phoenix::motorcontrol::can::WPI_TalonFX(constants::kDownie));

            /// @brief The uppie Falcon 500 as std::unique_ptr
            std::unique_ptr<
                ctre::phoenix::motorcontrol::can::WPI_TalonFX
            > uppieMotor = std::unique_ptr<
                ctre::phoenix::motorcontrol::can::WPI_TalonFX
            >(new ctre::phoenix::motorcontrol::can::WPI_TalonFX(constants::kUppie));

            frc2::PIDController rollerController {
                constants::kRollerP,
                constants::kRollerI,
                constants::kRollerD
            };

            /// @brief Is the intake deployed?
            bool isDeployed = false;
    };
}

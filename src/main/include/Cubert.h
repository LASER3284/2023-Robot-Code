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
        constexpr int kUppie = 999;
        /// @brief Bottom Cubert Falcon
        constexpr int kDownie = 998;
        /// @brief Deploying NEO
        constexpr int kDeployNeo = 997;

        /// @brief Angle offset for thru-bore
        constexpr units::degree_t kThruBoreAngleOffset = 0.0_deg;

        /// @brief Proportional Gain for the deploy controller
        constexpr double kDeployP = 0.0;
        /// @brief Integral Gain for the deploy controller
        constexpr double kDeployI = 0.0;
        /// @brief Derivative Gain for the deploy controller
        constexpr double kDeployD = 0.0;

        /// @brief This is a map of GridHeights to degree values for where the
        /// Cubert should position itself.
        const std::map<
            ::constants::FieldConstants::GridHeights,
            units::degree_t
        > kAngleGridMap = {
            { ::constants::FieldConstants::GridHeights::eUp, 90.0_deg },
            { ::constants::FieldConstants::GridHeights::eMid, 45.0_deg },
            { ::constants::FieldConstants::GridHeights::eStopped, 100_deg },
        };
    }

    class Cubert {
        public:
            /// @brief Updates SmartDashboard values; should be called in
            /// RobotPeriodic().
            void Tick();

            /// @brief Deploy the intake motors such that we intake a cube.
            /// @param intake A boolean value saying whether to spin the intake
            /// motor.
            void Deploy(bool intake);

            /// @brief Returns whether the cubert was last deployed or retracted.
            /// @return True if deployed, false otherwise.
            bool IsDeployed() {
                return isDeployed;
            }

            /// @brief Retract the intake and stop the motors unless we're
            /// shooting.
            void Retract();

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

            /// @brief The downie Falcon 500
            std::unique_ptr<
                ctre::phoenix::motorcontrol::can::WPI_TalonFX
            > downieMotor = std::unique_ptr<
                ctre::phoenix::motorcontrol::can::WPI_TalonFX
            >(new ctre::phoenix::motorcontrol::can::WPI_TalonFX(constants::kDownie));

            /// @brief The uppie Falcon 500
            std::unique_ptr<
                ctre::phoenix::motorcontrol::can::WPI_TalonFX
            > uppieMotor = std::unique_ptr<
                ctre::phoenix::motorcontrol::can::WPI_TalonFX
            >(new ctre::phoenix::motorcontrol::can::WPI_TalonFX(constants::kUppie));

            /// @brief Is the intake deployed?
            bool isDeployed = false;
    };
}

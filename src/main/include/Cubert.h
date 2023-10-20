#pragma once

#include "FieldConstants.h"
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/velocity.h>
#include <memory>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <map>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>
#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/Timer.h>

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
        constexpr units::degree_t kThruBoreAngleOffset = -15.0_deg;

        // NOTE: deploy PID values are with max control effort from SysID set to
        // 0.25V

        /// @brief Proportional Gain for the deploy controller
        constexpr double kDeployP = 0.063508;
        /// @brief Integral Gain for the deploy controller
        constexpr double kDeployI = 0.0;
        /// @brief Derivative Gain for the deploy controller
        constexpr double kDeployD = 0.0023459;
        
        /// @brief The statically applied voltage.
        constexpr auto kDeployKs = 0.21995_V;
        /// @brief The voltage applied per velocity unit.
        constexpr auto kDeployKv = 0.009124_V / 1_deg_per_s;
        /// @brief The voltage applied per acceleration unit.
        constexpr auto kDeployKa = 0.00033642_V / 1_deg_per_s_sq;

        /// @brief The gravitationally applied voltage.
        constexpr auto kDeployKg = 0.14336_V;
        /// @brief The point at which the cubert is horizontal.
        constexpr units::degree_t kDeployKgMaxAngle = -56.311_deg;

        /// @brief The statically applied voltage.
        constexpr auto kRollerKs = 0.24554_V;
        /// @brief The voltage applied per velocity unit.
        constexpr auto kRollerKv = 6.0142E-05_V / 1_deg_per_s;
        /// @brief The voltage applied per acceleration unit.
        constexpr auto kRollerKa = 1.3203E-06_V / 1_deg_per_s_sq;

        /// @brief Proportional Gain for the roller controller
        constexpr double kRollerP = 0.00175;
        /// @brief Integral Gain for the roller controller
        constexpr double kRollerI = 0.0012;
        /// @brief Derivative Gain for the roller controller
        constexpr double kRollerD = 0.0;

        /// @brief Roller speed in RPM
        constexpr units::revolutions_per_minute_t kRollerSetpoint = -9000_rpm;
        /// @brief Roller intake speed in RPM
        /// @todo Verify speeds are valid.
        constexpr units::revolutions_per_minute_t kRollerIntakeSetpoint = 2000_rpm;

        constexpr units::degree_t kUpperLimit = 120.0_deg;
        constexpr units::degree_t kShootingPos = 102.0_deg;
        constexpr units::degree_t kLowerLimit = 5.0_deg;

        /// @brief This is a map of GridHeights to degree values for where the
        /// Cubert should position itself.
        const std::map<
            ::constants::FieldConstants::GridHeights,
            units::degree_t
        > kAngleGridMap = {
            { ::constants::FieldConstants::GridHeights::eUp, kUpperLimit },
            { ::constants::FieldConstants::GridHeights::eMid, kShootingPos },
            { ::constants::FieldConstants::GridHeights::eGround, kLowerLimit },
            { ::constants::FieldConstants::GridHeights::eIntake, kLowerLimit },
            { ::constants::FieldConstants::GridHeights::eGroundSpit, kLowerLimit },
            { ::constants::FieldConstants::GridHeights::eStopped, kUpperLimit },
        };

        constexpr double kRollerRatio = 2.0 / 3.0;

        constexpr units::second_t kRollerDelay = 100_ms;
        constexpr units::revolutions_per_minute_t kRollerDeadband = 3_rpm;
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

            /// @todo Test implement.
            /// @brief Returns whether the shooter has obtained a game element.
            /// @return false for now.
            bool HasElement() {
                return abs(_get_roller_avel().value()) < constants::kRollerDeadband.value();
            }

            /// @brief Shoot the cube for a specific height.
            /// @param height The height to set the shooter to aim for.
            void Shoot(::constants::FieldConstants::GridHeights height);

            units::degree_t GetAngle() {
                units::degree_t cw_plus = units::degree_t {thruboreEnc.Get()};

                units::degree_t ccw_plus = 180_deg - cw_plus;

                return ccw_plus + constants::kThruBoreAngleOffset;
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

            /// @brief Has the side effect of setting the deploySetpoint and
            /// deployGoal to the specified position.
            /// @param angle The desired position.
            /// @warning Bounds checking is not done for this function! Be
            /// cautious of its side effects.
            void _set_deploy_goal(units::degree_t angle);

            /// @brief Gets the rollers' angular velocity.
            /// @return Roller angular velocity in RPM.
            units::revolutions_per_minute_t _get_roller_avel();

            /// @brief Gets the rollers' surface velocity.
            /// @return Roller surface velocity in ft/s.
            units::feet_per_second_t _get_roller_lvel();

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

            frc::ArmFeedforward deployFF {
                constants::kDeployKs,
                constants::kDeployKg,
                constants::kDeployKv,
                constants::kDeployKa
            };

            frc::TrapezoidProfile<units::degrees>::Constraints deployConstraints {
                240_deg_per_s,
                120_deg_per_s_sq
            };

            frc::TrapezoidProfile<units::degrees>::State deployGoal;

            frc::TrapezoidProfile<units::degrees>::State deploySetpoint;

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

            frc::Timer timekeeper {};
            units::second_t start_shoot = 0_s;
    };
}

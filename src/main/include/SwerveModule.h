#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Rotation2d.h>
#include <rev/CANSparkMax.h>

namespace drive {
    class SwerveModule {
        public:
            SwerveModule(int drivemotor_in, int turnmotor_in, int encoder_in, bool drive_inverted = false);
            frc::SwerveModuleState GetState() const;
            frc::SwerveModulePosition GetPosition() const;
            void SetDesiredState(const frc::SwerveModuleState& state, bool force_angle = false);
            double getTurnEncPos();
            double getDriveEncPos();
            void ResetEncoders();

            static constexpr units::meters_per_second_t kMaxSpeed = 4.0_mps;
            static constexpr units::radians_per_second_t kMaxAngularSpeed {
                3.14159 * 2
            };  // 1 rotation per second
            
        private:
            static constexpr double kWheelRadius = 0.0508;
            static constexpr int kEncoderResolution = 2048;

            static constexpr auto kModuleMaxAngularVelocity =
                3.14159 * 1_rad_per_s;  // pi radians per second
            static constexpr auto kModuleMaxAngularAcceleration =
                3.14159 * 2_rad_per_s / 1_s;  // 2pi radians per second^2

            ctre::phoenix::motorcontrol::can::WPI_TalonFX*  drivemotor;
            rev::CANSparkMax*  turnmotor;
            ctre::phoenix::sensors::CANCoder*               encoder;

            frc2::PIDController drivePIDController { 0.0001, 0.0, 0.0};
            frc2::PIDController turnPIDController {
                0.009, // P: 0.011
                0.000, // I: 0.00
                0.0000000025, // D: 0.0000000025
            };

            frc::SimpleMotorFeedforward<units::meters> driveFeedforward { 1_V, 2.5_V / 1_mps };

            units::degree_t lastAngle = 0_deg;
    };
}
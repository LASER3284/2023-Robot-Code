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
#include <units/angle.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Rotation2d.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"

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

            static constexpr units::meters_per_second_t kMaxSpeed = 4.93776_mps;
        private:
            static constexpr double kGearRatio = (6.86 / 1.0);
            static constexpr units::meter_t kWheelDiameter = 0.0952_m;
            static constexpr units::meter_t kWheelCircumference = (kWheelDiameter * constants::Pi);
            static constexpr int kEncoderResolution = 2048;

            static constexpr auto kModuleMaxAngularVelocity =
                3.14159 * 1_rad_per_s;  // pi radians per second
            static constexpr auto kModuleMaxAngularAcceleration =
                3.14159 * 2_rad_per_s / 1_s;  // 2pi radians per second^2

            ctre::phoenix::motorcontrol::can::WPI_TalonFX*  drivemotor;
            rev::CANSparkMax*  turnmotor;
            ctre::phoenix::sensors::CANCoder*               encoder;

            // TODO: Run drive-train characterization
            frc2::PIDController drivePIDController { 0.00038489, 0.0, 0.0};
            frc2::PIDController turnPIDController {
                0.009, // P: 0.011
                0.000, // I: 0.000
                0.000, // D: 0.000
            };

            frc::SimpleMotorFeedforward<units::meters> driveFeedforward { 
                0.17827_V,                      // kS
                2.166 * 1_V * 1_s / 1_m,      // kV
                0.18284 * 1_V * 1_s * 1_s / 1_m // kA
            };

            units::degree_t lastAngle = 0_deg;
    };
}

#include "Shoulder.h"

shoulder::Shoulder::Shoulder() {
    motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    followerMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    const ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration currentLimit = { 
        true, // Enable
        40, // Continuous Current Limit
        60, // Peak Current Limit
        0.5 // Peak Current Duration
    };
    motor.ConfigSupplyCurrentLimit(currentLimit);
    followerMotor.ConfigSupplyCurrentLimit(currentLimit);
    
    motor.SetInverted(false);
    followerMotor.Follow(motor);

    // Use the main encoder to ""zero"" out the NEO encoder
    // Zero out the falcon encoder (use Constants::falconToDeg)
    motor.ConfigIntegratedSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero);
    motor.ConfigIntegratedSensorAbsoluteRange(ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360);
    motor.ConfigIntegratedSensorOffset(0);
    encoderSeed = GetRotation();

    frc::SmartDashboard::PutData("shoulder_encoder", &encoder);

    shoulderGoal = { GetRotation(), 0_deg_per_s };
    shoulderSetpoint = { GetRotation(), 0_deg_per_s };
}

void shoulder::Shoulder::Tick(units::meter_t armExtension) {
    frc::SmartDashboard::PutNumber("shoulder_angle_abs", encoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("shoulder_angle_deg", GetRotation().value());
    frc::SmartDashboard::PutNumber("shoulder_velocity", GetVelocity().value());

    if(bEnabled) {
        if(manualPercentage != 0.0) {
            motor.SetVoltage(manualPercentage * 12_V);
            shoulderGoal = { GetRotation(), 0_deg_per_s };
            shoulderSetpoint = { GetRotation(), 0_deg_per_s };
            shoulderTimer.Reset();
            shoulderTimer.Stop();
        }
        else {
            if(units::math::abs(shoulderGoal.position - GetRotation()) > 1.5_deg) {
                // Create a motion profile with the given maximum velocity and maximum 
                // acceleration constraints for the next setpoint, the desired goal, and the
                // current setpoint.
                frc::TrapezoidProfile<units::radians> rotationalProfile { 
                    rotationalConstraints, 
                    shoulderGoal,
                    shoulderSetpoint,
                };

                shoulderSetpoint = rotationalProfile.Calculate(20_ms);
            }
            else {
                motor.SetVoltage(0_V);
                shoulderTimer.Start();
                if(shoulderTimer.HasElapsed(0.125_s)) {
                    shoulderSetpoint = { GetRotation(), 0_deg_per_s };
                    shoulderGoal = { GetRotation(), 0_deg_per_s };   
                }
            }

            frc::SmartDashboard::PutNumber("shoulderSetpoint_velocity", units::degrees_per_second_t(shoulderSetpoint.velocity).value());
            frc::SmartDashboard::PutNumber("shoulderSetpoint_position", units::degree_t(shoulderSetpoint.position).value());
            frc::SmartDashboard::PutNumber("shoulderGoal_position", units::degree_t(shoulderGoal.position).value());

            AdjustFeedforward(
                kinematics::Kinematics::CalculateShoulderFeedforward(armExtension, GetRotation(), shoulderSetpoint.velocity)
            );

            const auto control_effort_v = angleController.Calculate(
                units::radian_t(GetRotation()).value(), 
                shoulderGoal.position.value()
            );

            frc::SmartDashboard::PutNumber("shoulder_effort_v", control_effort_v);
            frc::SmartDashboard::PutNumber("shoulder_measurement", units::radian_t(GetRotation()).value());
            frc::SmartDashboard::PutNumber("shoulder_setpoint", shoulderSetpoint.position.value());

            motor.SetVoltage(feedforward);
        }
    }
    else {
        shoulderTimer.Restart();
        motor.SetVoltage(0_V);
    }
}

void shoulder::Shoulder::SetRotationGoal(units::degree_t rot) {
    angleController.SetSetpoint(rot.value());
    shoulderGoal = { rot, 0_deg_per_s };
}

units::degree_t shoulder::Shoulder::GetRotation() {
    auto rollover_deg = units::degree_t((360 * (encoder.GetAbsolutePosition() - encoder.GetPositionOffset())));
    rollover_deg += 360_deg;
    if(rollover_deg > 180_deg) {
        rollover_deg -= 360_deg;
    }
    rollover_deg -= 180_deg;
    rollover_deg -= Constants::kAngleOffset;
    rollover_deg -= manualOffset;
    rollover_deg *= -1;
    return rollover_deg;
}

units::degrees_per_second_t shoulder::Shoulder::GetVelocity() {
    return units::degrees_per_second_t(constants::falconToRPM(motor.GetSelectedSensorVelocity(), Constants::gearRatio) * 360 * 60);
}
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
    motor.ConfigIntegratedSensorOffset(GetRotation().value());

    frc::SmartDashboard::PutData("shoulder_encoder", &encoder);
}

void shoulder::Shoulder::Tick(units::meter_t armExtension) {
    frc::SmartDashboard::PutNumber("shoulder_angle_abs", encoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("shoulder_angle_deg", GetRotation().value());
    frc::SmartDashboard::PutNumber("shoulder_velocity", GetVelocity().value());

    if(bEnabled) {
        if(manualPercentage != 0.0) {
            motor.SetVoltage(manualPercentage * 12_V);
            shoulderTimer.Restart();
        }
        else {
            if(units::math::abs(shoulderGoal.position - shoulderSetpoint.position) > 2.5_deg && shoulderTimer.HasElapsed(0.15_s)) {
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
                shoulderSetpoint = { GetRotation(), 0_deg_per_s };
            }

            frc::SmartDashboard::PutNumber("shoulderSetpoint_velocity", units::degrees_per_second_t(shoulderSetpoint.velocity).value());
            frc::SmartDashboard::PutNumber("shoulderSetpoint_position", units::degree_t(shoulderSetpoint.position).value());
            frc::SmartDashboard::PutNumber("shoulderGoal_position", units::degree_t(shoulderGoal.position).value());

            AdjustFeedforward(
                kinematics::Kinematics::CalculateShoulderFeedforward(armExtension, GetRotation(), shoulderSetpoint.velocity)
            );

            motor.SetVoltage(feedforward + units::volt_t(angleController.Calculate(GetRotation().value())));
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
    // TODO:: Make this actually good and not awful
    return ((units::degree_t((-1289.09493 * encoder.GetAbsolutePosition())) + 260.30902_deg));
}

units::degrees_per_second_t shoulder::Shoulder::GetVelocity() {
    return units::degrees_per_second_t(constants::falconToRPM(motor.GetSelectedSensorVelocity(), Constants::gearRatio) * 360 * 60);
}
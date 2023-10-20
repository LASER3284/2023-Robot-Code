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
    followerMotor.SetInverted(false);

    // Use the main encoder to ""zero"" out the NEO encoder
    // Zero out the falcon encoder (use Constants::falconToDeg)
    motor.ConfigIntegratedSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy::BootToZero);
    motor.ConfigIntegratedSensorAbsoluteRange(ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360);
    motor.ConfigIntegratedSensorOffset(0);
    encoderSeed = GetRotation();

    frc::SmartDashboard::PutData("shoulder_encoder", &encoder);

    shoulderGoal = { GetRotation(), 0_deg_per_s };
    shoulderSetpoint = { GetRotation(), 0_deg_per_s };

    shoulderTimer.Restart();

    angleController.SetTolerance(2.5);
    angleController.EnableContinuousInput(-180.0, 180.0);
}

void shoulder::Shoulder::Tick(units::meter_t armExtension) {
    frc::SmartDashboard::PutNumber("shoulder_angle_abs", encoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("shoulder_angle_deg", GetRotation().value());
    frc::SmartDashboard::PutNumber("shoulder_angle_inverse_deg", GetRotationInverse().value());
    frc::SmartDashboard::PutNumber("shoulder_velocity", GetVelocity().value());
    frc::SmartDashboard::PutNumber("shoulder_uppie_amps", motor.GetOutputCurrent());

    units::volt_t output_voltage = 0_V;

    if (bEnabled) {
        if(manualPercentage != 0.0) {
            output_voltage = manualPercentage * 12_V;
            shoulderGoal = { GetRotation(), 0_deg_per_s };
            shoulderSetpoint = { GetRotation(), 0_deg_per_s };
            shoulderTimer.Restart();
        }
        else {
            // Create the motion profile with the given maximum velocity and maximum acceleration for the given setpoint
            frc::TrapezoidProfile<units::degrees> rotationalProfile{
                defaultRotationalConstraints,
                shoulderGoal,
                shoulderSetpoint,
            };

            shoulderSetpoint = rotationalProfile.Calculate(20_ms);

            frc::SmartDashboard::PutNumber("shoulderSetpoint_velocity", units::degrees_per_second_t(shoulderSetpoint.velocity).value());
            frc::SmartDashboard::PutNumber("shoulderSetpoint_position", units::degree_t(shoulderSetpoint.position).value());
            frc::SmartDashboard::PutNumber("shoulderGoal_position", units::degree_t(shoulderGoal.position).value());

            AdjustFeedforward(
                angleFeedForward.Calculate(shoulderSetpoint.position, shoulderSetpoint.velocity)
            );

            const auto control_effort_v = angleController.Calculate(
                GetRotation().value(),
                shoulderSetpoint.position.value()
            );

            frc::SmartDashboard::PutNumber("shoulder_effort_v", control_effort_v);
            frc::SmartDashboard::PutNumber("shoulder_measurement", units::radian_t(GetRotation()).value());
            frc::SmartDashboard::PutNumber("shoulder_setpoint", shoulderSetpoint.position.value());
            frc::SmartDashboard::PutNumber("shoulder_ff", feedforward.value());

            output_voltage = feedforward + units::volt_t{frc::ApplyDeadband(control_effort_v, 0.1, HUGE_VAL)};

            frc::SmartDashboard::PutNumber("shoulder_output_v", output_voltage.value());

        }

        // soft stops, should stop at horizontal
        //if((GetRotation() <= 0_deg && GetRotation() >= -90_deg && output_voltage < 0_V) || (GetRotation() <= 0_deg && GetRotation() <= -90_deg && output_voltage > 0_V)) {
        //    output_voltage = 0_V;
        //}
        
        _set_motor_voltage(output_voltage);
    }
    else {
        shoulderTimer.Restart();
        _set_motor_voltage(0_V);
    }
}

void shoulder::Shoulder::SetRotationGoal(units::degree_t rot) {
    shoulderGoal = { rot, 0_deg_per_s };
}

units::degree_t shoulder::Shoulder::GetRotation() {
    auto rollover_deg = units::degree_t((360 * (encoder.GetAbsolutePosition() - encoder.GetPositionOffset())));

    rollover_deg = 180_deg - rollover_deg;
    rollover_deg += Constants::kAngleOffset;

    if (rollover_deg >= 360_deg || rollover_deg > 180_deg) {
        rollover_deg -= 360_deg;
    }

    //rollover_deg = angle_filter.Calculate(rollover_deg);

    return rollover_deg;
}

units::degree_t shoulder::Shoulder::GetRotationInverse() {
    units::degree_t raw = units::degree_t(360 * (encoder.GetAbsolutePosition() - encoder.GetPositionOffset()));

    raw -= Constants::kAngleOffset;

    if (raw >= 360_deg || raw > 180_deg) {
        raw -= 360_deg;
    }

    return raw;
}

units::degrees_per_second_t shoulder::Shoulder::GetVelocity() {
    return units::degrees_per_second_t(constants::falconToRPM(motor.GetSelectedSensorVelocity(), Constants::gearRatio) * 360 * 60);
}
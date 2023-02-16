#include "Shoulder.h"

shoulder::Shoulder::Shoulder() {
    motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    motor.SetSmartCurrentLimit(60);
    motor.SetInverted(false);
    followerMotor.Follow(motor);

    // Use the main encoder to ""zero"" out the NEO encoder
    mainEncoder.SetPosition(
        (encoder.GetPositionOffset() * 360) / Constants::gearRatio
    );

    frc::SmartDashboard::PutData("shoulder_encoder", &encoder);
}

void shoulder::Shoulder::Tick() {
    frc::SmartDashboard::PutNumber("shoulder_angle_abs", encoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("shoulder_angle_deg", GetRotation().value());
    frc::SmartDashboard::PutNumber("shoulder_angle_motor", mainEncoder.GetPosition() * Constants::gearRatio * 360);
    frc::SmartDashboard::PutNumber("shoulder_velocity", GetVelocity().value());

    if(bEnabled) {
        if(manualPercentage != 0.0) {
            motor.SetVoltage(
                (manualPercentage * 12_V) + feedforward
            );
        }
        else {
            motor.SetVoltage(0_V);
            //motor.SetVoltage(feedforward + units::volt_t(angleController.Calculate(GetRotation().value())));
        }
    }
    else {
        motor.SetVoltage(0_V);
    }
}

void shoulder::Shoulder::SetRotationGoal(units::degree_t rot) {
    angleController.SetSetpoint(rot.value());
}

units::degree_t shoulder::Shoulder::GetRotation() {
    // TODO:: Make this actually good and not awful
    return ((units::degree_t((-1289.09493 * encoder.GetAbsolutePosition())) + 262.30902_deg));
    //return ((units::degree_t((encoder.GetAbsolutePosition()) * 360_deg)) - 25_deg) + 180_deg;
}

units::degrees_per_second_t shoulder::Shoulder::GetVelocity() {
    return units::degrees_per_second_t(mainEncoder.GetVelocity() * Constants::gearRatio * 360);
}
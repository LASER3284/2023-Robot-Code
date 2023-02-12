#include "Shoulder.h"

shoulder::Shoulder::Shoulder() {
    motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    motor.SetSmartCurrentLimit(60);
    motor.SetInverted(true);
    followerMotor.Follow(motor);
}

void shoulder::Shoulder::Tick() {
    frc::SmartDashboard::PutNumber("shoulder_angle_deg", GetRotation().value());
    frc::SmartDashboard::PutNumber("shoulder_angle_abs", encoder.GetAbsolutePosition());

    if(bEnabled) {
        if(manualPercentage != 0.0) {
            motor.SetVoltage(
                (manualPercentage * 12_V) + feedforward
            );
        }
        else {
            motor.SetVoltage(feedforward);
            // motor.SetVoltage(
            //     units::volt_t(angleController.Calculate(GetRotation())) + feedforward
            // );
        }
    }
    else {
        motor.SetVoltage(0_V);
    }
}

void shoulder::Shoulder::SetRotationGoal(units::degree_t rot) {
    angleController.SetGoal(rot);
}

units::degree_t shoulder::Shoulder::GetRotation() {
    return (units::degree_t(encoder.GetAbsolutePosition() * 360) - Constants::kAngleOffset) * -1;
}

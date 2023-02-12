#include "Wrist.h"

wrist::Wrist::Wrist() {
    wristMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    wristMotor.SetSmartCurrentLimit(25);
}

void wrist::Wrist::Tick() {
    frc::SmartDashboard::PutNumber("wrist_angle", GetRotation().value());
    if(manualPercentage != 0) {
        wristMotor.SetVoltage(
            (manualPercentage * 12_V) + 0.2_V
        );
    }
    else {
        // TODO: Implement basic feedforward calculations and add PID controls
        wristMotor.SetVoltage(0.2_V);
    }
}

void wrist::Wrist::SetRotationGoal(units::degree_t rot) {
    controller.SetGoal(rot);
}

units::degree_t wrist::Wrist::GetRotation() {
    return units::degree_t { (wristEncoder.GetPosition() / Constants::kWristGearRatio) };
}

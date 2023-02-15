#include "Wrist.h"

wrist::Wrist::Wrist() {
    wristMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    wristMotor.SetSmartCurrentLimit(25);
    wristEncoder.SetPosition(0);
    SetRotationGoal(GetRotation());
}

void wrist::Wrist::Tick() {
    frc::SmartDashboard::PutNumber("wrist_angle", GetRotation().value());
    frc::SmartDashboard::PutNumber("wrist_pos", wristEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("wrist_goal", units::degree_t(goal).value());

    if(manualPercentage != 0) {
        wristMotor.SetVoltage((manualPercentage * 12_V));
    }
    else {
        units::radians_per_second_t moveVelocity = wpi::Lerp<units::radians_per_second_t>(
            0_rad_per_s,
            Constants::kMaxRotationalVelocity,
            units::math::abs(((goal - GetRotation()) / Constants::kMaxAngle)).value()
        );
        const units::volt_t controlEffort = units::volt_t(
            controller.Calculate(goal.value())) + 
            feedforward.Calculate(goal, moveVelocity);
        wristMotor.SetVoltage(controlEffort);
    }
}

void wrist::Wrist::SetRotationGoal(units::degree_t rot) {
    goal = units::radian_t(rot);
    controller.SetSetpoint(goal.value());
}

units::degree_t wrist::Wrist::GetRotation() {
    units::degree_t angle = units::degree_t { (wristEncoder.GetPosition() / Constants::kWristGearRatio) * 360 } - Constants::kStartingAngle;
    return angle;
}

#include "Wrist.h"

wrist::Wrist::Wrist() {
    wristMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    wristMotor.SetSmartCurrentLimit(25);
    wristMotor.SetInverted(true);
    wristEncoder.SetPosition(0);
    SetRotationGoal(GetRotation());
    wristTimer.Restart();
}

void wrist::Wrist::Tick(units::degree_t shoulderRotation) {
    frc::SmartDashboard::PutNumber("wrist_angle", GetRotation().value());
    frc::SmartDashboard::PutNumber("wrist_pos", wristEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("wrist_goal", units::degree_t(wristGoal.position).value());
    frc::SmartDashboard::PutNumber("wristTimer_s", wristTimer.Get().value());

    if(manualPercentage != 0) {
        wristMotor.SetVoltage((manualPercentage * 12_V));
        wristTimer.Restart();
        wristGoal = { GetRotation(), 0_deg_per_s };
        wristSetpoint = { GetRotation(), 0_deg_per_s };
    }
    else {
        // Since the movements on the end of the wrist are amplified by the shoulder, we really don't care too much about
        // extreme precision on the wrist most of the time
        if(units::math::abs(wristGoal.position - GetRotation()) > 0.5_deg) {
            // Create a motion profile with the given maximum velocity and maximum 
            // acceleration constraints for the next setpoint, the desired goal, and the
            // current setpoint.
            frc::TrapezoidProfile<units::radians> rotationalProfile { 
                rotationalConstraints, 
                wristGoal,
                wristSetpoint,
            };

            wristSetpoint = rotationalProfile.Calculate(20_ms);
        }

        frc::SmartDashboard::PutNumber("wristSetpoint_velocity", units::degrees_per_second_t(wristSetpoint.velocity).value());
        frc::SmartDashboard::PutNumber("wristSetpoint_position", units::degree_t(wristSetpoint.position).value());
        frc::SmartDashboard::PutNumber("wristGoal_position", units::degree_t(wristGoal.position).value());

        units::volt_t ff = (Constants::kS * wpi::sgn(wristSetpoint.velocity)) + (Constants::kG * units::math::cos(shoulderRotation)) + ((Constants::kV * wristSetpoint.velocity));
        frc::SmartDashboard::PutNumber("wrist_ff_v", ff.value());
        wristMotor.SetVoltage(ff + units::volt_t(controller.Calculate(units::radian_t(GetRotation()).value(), wristGoal.position.value())));
    }
}

void wrist::Wrist::SetRotationGoal(units::degree_t rot) {
    wristGoal = { rot, 0_deg_per_s };
    controller.SetSetpoint(wristGoal.position.value());
}

units::degree_t wrist::Wrist::GetRotation() {
    units::degree_t angle = units::degree_t { (wristEncoder.GetPosition() / Constants::kWristGearRatio) * 360 } + Constants::kStartingAngle;
    return angle;
}

#include "Wrist.h"

wrist::Wrist::Wrist() {
    wristMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    wristMotor.SetSmartCurrentLimit(25);
    wristMotor.SetInverted(true);
    SetRotationGoal(GetRotation());
    wristTimer.Restart();
}

void wrist::Wrist::Tick(units::degree_t shoulderRotation) {
    frc::SmartDashboard::PutNumber("wrist_angle", GetRotation().value());
    frc::SmartDashboard::PutNumber("wrist_pos", thruboreEnc.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("wrist_goal", units::degree_t(wristGoal.position).value());
    frc::SmartDashboard::PutNumber("wristTimer_s", wristTimer.Get().value());

    if(manualPercentage != 0) {
        // If the rotation is within the allowed range OR the intake is trying
        // to get back within the range, allow voltage, else don't
        if ((GetRotation().value() >= -90 && GetRotation().value() <= 90) || manualPercentage * (GetRotation().value() >= 0 ? 1 : -1) < 0) {
            wristMotor.SetVoltage((manualPercentage * 12_V));
            wristTimer.Restart();
            wristGoal = { GetRotation(), 0_deg_per_s };
            wristSetpoint = { GetRotation(), 0_deg_per_s };
        } else {
            wristMotor.SetVoltage(0_V);
        }
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

        units::volt_t ff = (Constants::kS * wpi::sgn(wristSetpoint.velocity));
        frc::SmartDashboard::PutNumber("wrist_ff_v", ff.value());
        frc::SmartDashboard::PutNumber("wrist_pid_v", controller.Calculate(units::radian_t(GetRotation()).value(), wristGoal.position.value()));
        const auto voltage = ff + units::volt_t(controller.Calculate(units::radian_t(GetRotation()).value(), wristGoal.position.value()));
        //if ((GetRotation().value() >= -90 && GetRotation().value() <= 90) || voltage.value() * (GetRotation().value() >= 0 ? -1 : 1) < 0 )
        wristMotor.SetVoltage(voltage);
        //else
        //    wristMotor.SetVoltage(0_V);
    }
}

void wrist::Wrist::SetRotationGoal(units::degree_t rot) {
    wristGoal = { rot, 0_deg_per_s };
    //controller.SetSetpoint(wristGoal.position.value());
}

units::degree_t wrist::Wrist::GetRotation() {
    units::degree_t raw = thruboreEnc.Get();

    raw = (180_deg - raw) + Constants::kAngleOffset;

    while (units::math::abs(raw) > 360_deg) {
        raw += raw < 0_deg ? 360_deg : -360_deg;
    }

    raw = -raw;

    raw = raw > 180_deg ? raw - 360_deg : raw;

    return raw;
}

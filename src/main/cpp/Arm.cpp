#include "Arm.h"

arm::Arm::Arm() {
    extensionMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    extensionMotor.SetSmartCurrentLimit(60);
    extensionMotor.SetInverted(true);
    extensionEncoder.SetPosition(0);
    controlTimer.Restart();
    positionController.Reset();
    SetPositionGoal(GetPosition());
}

units::meter_t arm::Arm::GetPosition() {
    auto value = (extensionEncoder.GetPosition() / Constants::kArmRatio) * constants::Pi * Constants::kSprocketDiameter;
    return value;
}

units::meters_per_second_t arm::Arm::GetVelocity() {
    const units::meters_per_second_t velocity =
        (((extensionEncoder.GetVelocity() / Constants::kArmRatio) * constants::Pi * Constants::kSprocketDiameter) / 60_s);
    return velocity;
}

void arm::Arm::SetPositionGoal(units::meter_t distance) {
    if (distance > 0.62_m) { distance = 0.61_m; }

    positionController.SetSetpoint(distance.value());
    extensionGoal = { distance, 0_mps };
}

void arm::Arm::Tick(units::degree_t shoulderRotation) {
    // Reset the arm encoder if it's somehow gone negative from it's zero point
    
    
    frc::SmartDashboard::PutNumber("arm_extension_m", GetPosition().value());
    frc::SmartDashboard::PutNumber("arm_extension_mps", GetVelocity().value());

    frc::SmartDashboard::PutNumber("arm_controlTimer_s", controlTimer.Get().value());
    frc::SmartDashboard::PutNumber("arm_extension_goal_m", extensionGoal.position.value());


    if(bEnabled) {
        if(manualPercentage != 0.0) {
            if (GetPosition() < 1.5_in && manualPercentage < 0) {
                extensionMotor.Set(0.0);
            } else {
                frc::SmartDashboard::PutNumber("smax_extend_volts", extensionMotor.GetAppliedOutput() * 12);
                extensionMotor.SetVoltage(units::volt_t { 12 * manualPercentage });
            }
            controlTimer.Restart();
            extensionGoal = { GetPosition(), 0_mps };
            extensionSetpoint = { GetPosition(), 0_mps };
        }
        else {
            if(controlTimer.HasElapsed(167.6_ms)) {
                // Create a motion profile with the given maximum velocity and maximum 
                // acceleration constraints for the next setpoint, the desired goal, and the
                // current setpoint.
                frc::TrapezoidProfile<units::meters> extensionProfile { 
                    constraints, 
                    extensionGoal,
                    extensionSetpoint,
                };

                extensionSetpoint = extensionProfile.Calculate(20_ms);
                frc::SmartDashboard::PutNumber("extensionSetpoint_vel", extensionSetpoint.velocity.value());
                frc::SmartDashboard::PutNumber("extensionSetpoint_pos", extensionSetpoint.position.value());
                const auto output_voltage = feedforward.Calculate(extensionSetpoint.velocity);
                frc::SmartDashboard::PutNumber("extensionFF_v", output_voltage.value());
                extensionMotor.SetVoltage(
                    units::volt_t(positionController.Calculate(GetPosition().value(), extensionSetpoint.position.value()))
                    + output_voltage
                );
            }
            
            else {
                extensionMotor.SetVoltage(0_V);
                extensionGoal = { GetPosition(), 0_mps };
                extensionSetpoint = { GetPosition(), 0_mps };
            }
        }
    }
    else {
        extensionMotor.SetVoltage(0_V);
        controlTimer.Restart();
    }
}

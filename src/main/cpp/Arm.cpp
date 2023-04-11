#include "Arm.h"

arm::Arm::Arm() {
    extensionMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    extensionMotor.SetSmartCurrentLimit(20);
    extensionMotor.SetInverted(true);
    extensionEncoder.SetPosition(6.3);
    controlTimer.Restart();
    positionController.Reset();
    SetPositionGoal(GetPosition());
}

units::meter_t arm::Arm::GetPosition() {
    auto value = (extensionEncoder.GetPosition() / Constants::kArmRatio) * Constants::kSprocketDiameter;
    return value;
}

units::meters_per_second_t arm::Arm::GetVelocity() {
    const units::meters_per_second_t velocity = units::meters_per_second_t(
        (((extensionEncoder.GetVelocity() / Constants::kArmRatio) * Constants::kSprocketDiameter) / 60).value()
    );
    return velocity;
}

void arm::Arm::SetPositionGoal(units::meter_t distance) {
    if(distance <= 0.03_m) { distance = 0.026_m; }

    positionController.SetSetpoint(distance.value());
    extensionGoal = { distance, 0_mps };
}

void arm::Arm::Tick(units::degree_t shoulderRotation) {
    // Reset the arm encoder if it's somehow gone negative from it's zero point
    if(GetPosition() < 0_m) {
        extensionEncoder.SetPosition(0);
        RefreshController();
    }
    
    frc::SmartDashboard::PutNumber("arm_extension_m", GetPosition().value());
    frc::SmartDashboard::PutNumber("arm_extension_mps", GetVelocity().value());

    frc::SmartDashboard::PutNumber("arm_controlTimer_s", controlTimer.Get().value());
    frc::SmartDashboard::PutNumber("arm_extension_goal_m", extensionGoal.position.value());


    if(bEnabled) {
        if(manualPercentage != 0.0) {
            extensionMotor.Set(manualPercentage);
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
                const auto output_voltage = feedforward.Calculate(extensionSetpoint.velocity) + (units::math::cos(90_deg - shoulderRotation) * Constants::kG);
                frc::SmartDashboard::PutNumber("extensionFF_v", output_voltage.value());
                extensionMotor.SetVoltage(
                    units::volt_t(positionController.Calculate(GetPosition().value(), extensionSetpoint.position.value())) +
                    output_voltage
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

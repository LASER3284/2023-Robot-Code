#include "Arm.h"

arm::Arm::Arm() {
    extensionMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    extensionMotor.SetSmartCurrentLimit(25);
    extensionMotor.SetInverted(true);
    extensionEncoder.SetPosition(0);
}

units::meter_t arm::Arm::GetPosition() {
    return (extensionEncoder.GetPosition() / Constants::kArmRatio) * Constants::kSprocketDiameter;
}

void arm::Arm::SetPositionGoal(units::meter_t distance) {
    // positionController.SetGoal(distance);
    extensionGoal = { distance, 0_mps };
}

void arm::Arm::Tick() {
    // Reset the arm encoder if it's somehow gone negative from it's zero point
    if(GetPosition() < 0_m) extensionEncoder.SetPosition(0);
    
    frc::SmartDashboard::PutNumber("arm_extension_m", GetPosition().value());
    frc::SmartDashboard::PutNumber("arm_controlTimer_s", controlTimer.Get().value());
    frc::SmartDashboard::PutNumber("arm_extension_goal_m", extensionGoal.position.value());


    if(bEnabled) {
        if(manualPercentage != 0.0) {
            extensionMotor.Set(manualPercentage);
            controlTimer.Restart();
        }
        else {
            if(extensionGoal.position != extensionSetpoint.position && controlTimer.HasElapsed(0.125_s)) {
                // Create a motion profile with the given maximum velocity and maximum 
                // acceleration constraints for the next setpoint, the desired goal, and the
                // current setpoint.
                frc::TrapezoidProfile<units::meters> extensionProfile { 
                    constraints, 
                    extensionGoal,
                    extensionSetpoint,
                };

                extensionSetpoint = extensionProfile.Calculate(20_ms);
            }
            else {
                extensionSetpoint = { GetPosition(), 0_mps };
            }

            extensionMotor.SetVoltage(
                //units::volt_t(positionController.Calculate(GetPosition()))
                feedforward.Calculate(extensionSetpoint.velocity)
            );
        }
    }
    else {
        extensionMotor.SetVoltage(0_V);
        controlTimer.Restart();
    }
}

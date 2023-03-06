#include "Arm.h"

arm::Arm::Arm() {
    extensionMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    extensionMotor.SetSmartCurrentLimit(25);
    extensionMotor.SetInverted(true);
    extensionEncoder.SetPosition(0);
    controlTimer.Restart();
    positionController.Reset();
}

units::meter_t arm::Arm::GetPosition() {
    return (extensionEncoder.GetPosition() / Constants::kArmRatio) * Constants::kSprocketDiameter;
}

void arm::Arm::SetPositionGoal(units::meter_t distance) {
    positionController.SetSetpoint(distance.value());
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
            lastGoal = GetPosition();
        }
        else {
            if(extensionGoal.position != GetPosition() && controlTimer.HasElapsed(0.250_s)) {
                // Create a motion profile with the given maximum velocity and maximum 
                // acceleration constraints for the next setpoint, the desired goal, and the
                // current setpoint.
                frc::TrapezoidProfile<units::meters> extensionProfile { 
                    constraints, 
                    extensionGoal,
                    extensionSetpoint,
                };

                extensionSetpoint = extensionProfile.Calculate(20_ms);
                lastGoal = extensionSetpoint.position;
            }
            else {
                positionController.Reset();
                extensionSetpoint = { lastGoal, 0_mps };
            }

            extensionMotor.SetVoltage(
                units::volt_t(positionController.Calculate(GetPosition().value(), extensionGoal.position.value())) +
                feedforward.Calculate(extensionSetpoint.velocity)
            );
        }
    }
    else {
        positionController.Reset();
        extensionMotor.SetVoltage(0_V);
        controlTimer.Restart();
    }
}

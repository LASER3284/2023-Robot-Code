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
    positionController.SetGoal(distance);
}

void arm::Arm::Tick() {
    frc::SmartDashboard::PutNumber("arm_extension_m", GetPosition().value());
    
    if(bEnabled) {
        if(manualPercentage != 0.0) {
            extensionMotor.Set(manualPercentage);
        }
        else {
            extensionMotor.Set(
                positionController.Calculate(GetPosition())
            );
        }
    } 
}

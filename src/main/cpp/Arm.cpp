#include "Arm.h"

arm::Arm::Arm(){
    extentionMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    extentionMotor.SetSmartCurrentLimit(25);

}

units::meter_t arm::Arm::GetPosition() {
    return units::meter_t{extentionMotor.GetEncoder().GetPosition() / arm::Constants::kArmRatio};

}
void arm::Arm::SetPositionGoal(units::meter_t distance) {
    positionController.SetGoal(distance);

}

void arm::Arm::Tick() {
    extentionMotor.Set(
        positionController.Calculate(
            units::meter_t{extentionMotor.GetEncoder().GetPosition()}
        )
    );      
}

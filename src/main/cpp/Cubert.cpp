#include "Cubert.h"
#include "FieldConstants.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ::constants;

void shooter::Cubert::Retract() {

    isDeployed = false;
}

void shooter::Cubert::Deploy(bool intake) {
    
    isDeployed = true;
}

void shooter::Cubert::Tick() {
    frc::SmartDashboard::PutNumber("cubert_enc_deg", GetAngle().value());
    frc::SmartDashboard::PutNumber("cubert_enc_real", thruboreEnc.GetAbsolutePosition() * 360);
}

void shooter::Cubert::Shoot(FieldConstants::GridHeights height) {
    // Have a default setpoint of -1000
    units::degree_t deploySetpoint = -1000_deg;
    deploySetpoint = constants::kAngleGridMap.at(height);
    
    if (deploySetpoint != -1000_deg)
        _set_deploy(units::volt_t { deployController.Calculate(GetAngle().value(), deploySetpoint.value()) });
}
void shooter::Cubert::_stop_rollers() {
    uppieMotor->SetVoltage(0_V);
    downieMotor->SetVoltage(0_V);
}

void shooter::Cubert::_set_rollers(units::volt_t volts) {
    uppieMotor->SetVoltage(volts);
    downieMotor->SetVoltage(volts);
}

void shooter::Cubert::_set_deploy(units::volt_t volts) {
    deployMotor.SetVoltage(volts);
}

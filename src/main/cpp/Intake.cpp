#include "Intake.h"

intake::Intake::Intake() {
    intakeMotor.SetSmartCurrentLimit(25);
}

void intake::Intake::ConeMode(){
    intakeSolenoid1.Set(true);
    intakeSolenoid2.Set(true);
}

void intake::Intake::CubeMode() { 
    intakeSolenoid1.Set(false);
    intakeSolenoid2.Set(false);
}

void intake::Intake::Inhale() {
    intakeMotor.Set(0.25);
}

void intake::Intake::Spit() {
    intakeMotor.Set(-0.25);
}

bool intake::Intake::HasElement() {
    // TODO: Implement current spike detection (?)
    return GetFilteredCurrent() >= 10_A;
}


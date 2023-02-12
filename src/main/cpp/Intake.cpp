#include "Intake.h"

intake::Intake::Intake() {
    intakeMotor.SetSmartCurrentLimit(25);
}

void intake::Intake::ConeMode(){
    intakeSolenoid1.Set(false);
    intakeSolenoid2.Set(true);
    Inhale();
}

void intake::Intake::CubeMode() { 
    intakeSolenoid1.Set(true);
    intakeSolenoid2.Set(false);
    Inhale();
}

void intake::Intake::Inhale() {
    intakeMotor.Set(0.90);
}

void intake::Intake::Spit() {
    intakeMotor.Set(-0.5);
}

void intake::Intake::Stop() {
    intakeMotor.Set(0.0);
}

bool intake::Intake::HasElement() {
    return GetFilteredCurrent() >= 20_A;
}


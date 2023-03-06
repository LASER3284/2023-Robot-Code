#include "Intake.h"

intake::Intake::Intake() {
    intakeMotor.SetSmartCurrentLimit(25);
    intakeMotor.SetInverted(false);
    intakeSolenoid1.Set(false);
    intakeSolenoid2.Set(false);
    cubeMode = false;
}

void intake::Intake::ConeMode(){
    intakeSolenoid1.Set(false);
    intakeSolenoid2.Set(false);
    Inhale();

    cubeMode = false;
}

void intake::Intake::CubeMode() { 
    intakeSolenoid1.Set(true);
    intakeSolenoid2.Set(true);
    Inhale();

    cubeMode = true;
}

void intake::Intake::Inhale() {
    // Run the main intake motor at 90% in order to start intaking things
    intakeMotor.Set(-1.00);
}

void intake::Intake::Hold() {
    // Apply a small amount of voltage in order to keep the item in the claw
    intakeMotor.Set(0.0);
}

void intake::Intake::Spit() {
    intakeMotor.Set(0.5);
}

void intake::Intake::SmallSpit() {
    intakeMotor.Set(0.15);
}

void intake::Intake::Shoot() {
    intakeMotor.Set(1.0);
}

void intake::Intake::Stop() {
    intakeMotor.Set(0.0);
}

bool intake::Intake::HasElement() {
    return GetFilteredCurrent() >= 20.5_A;
}


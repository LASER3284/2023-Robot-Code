#include "Intake.h"

intake::Intake::Intake() {
    intakeMotor.SetSmartCurrentLimit(20, 25, 500);
    intakeMotor.SetInverted(false);
    cubeMode = false;
}

void intake::Intake::Tick() {
    intakeCurrent = medianFilter.Calculate(units::ampere_t { intakeMotor.GetOutputCurrent() });
    bool bHasNewElement = fallingDebouncer.Calculate(risingDebouncer.Calculate(intakeCurrent >= 19.5_A) && wpi::sgn(intakeMotor.GetAppliedOutput()) != -holdPower);

    frc::SmartDashboard::PutNumber("intakeCurrent_a", intakeMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("filteredIntakeCurrent_a", intakeCurrent.value());
    frc::SmartDashboard::PutBoolean("hasElement", bHasNewElement);
    frc::SmartDashboard::PutNumber("intakeSpeed_rpm", intakeEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("intakePosition", intakeEncoder.GetPosition());

    if(!hasElement && bHasNewElement) {
        holdPower = wpi::sgn(intakeMotor.GetAppliedOutput());
    }
    else if(!hasElement && !bHasNewElement) {
        holdPower = 0;
    }

    hasElement = bHasNewElement;
}

void intake::Intake::ConeMode(){
    Inhale();
}

void intake::Intake::CubeMode() { 
    Inhale();
}

void intake::Intake::Inhale() {
    // Run the main intake motor at 100% in order to start intaking things
    intakeMotor.Set(-1.00 * intakePower);
}

void intake::Intake::Hold() {
    // Apply a small amount of voltage in order to keep the item in the claw
    intakeMotor.Set(1.00 * holdPower);
}

void intake::Intake::Spit() {
    intakeMotor.Set(0.5 * intakePower);
}

void intake::Intake::SmallSpit() {
    intakeMotor.Set(0.15 * intakePower);
}

void intake::Intake::Shoot() {
    intakeMotor.Set(1.0 * intakePower);
}

void intake::Intake::Stop() {
    intakeMotor.Set(0.0);
}
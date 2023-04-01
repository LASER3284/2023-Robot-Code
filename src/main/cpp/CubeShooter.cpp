#include "CubeShooter.h"
#include "Constants.h"

using namespace shooter;

CubeShooter::CubeShooter() {
    // Setup PID Controllers
    Retract();

    // Setup Encoder conversion factor
    flywheelEnc.SetVelocityConversionFactor(60 * 2 * constants::Pi * Constants::kFlywheelRatio);
}

void CubeShooter::Deploy(bool intake) {
    // Deploy the intake with the solenoids
    deploySolenoid.Set(true);

    // Spin the motor!!!!!!!!!!!! pls and ty
    if (intake)
        intakeMotor.Set(0.5);
    else
        intakeMotor.Set(0.0);
}

void CubeShooter::Retract(){
    // Retract the intake with the solenoids
    deploySolenoid.Set(false);

    //Stop spinning the motor!!!!!!! no thx
    intakeMotor.Set(0.0);
    flywheelMotor.Set(0.0);
}

void CubeShooter::Shoot(constants::FieldConstants::GridHeights height) {
    // Switch-case based on the height we're aiming for.
    double setpoint = 0.0;

    switch (height) {
        case constants::FieldConstants::GridHeights::eUp:
            setpoint = Constants::kHighSetpoint.value();
            break;
        case constants::FieldConstants::GridHeights::eMid:
            setpoint = Constants::kMidSetpoint.value();
            break;
        case constants::FieldConstants::GridHeights::eGround:
            setpoint = Constants::kGroundSetpoint.value();
            break;
        default:
            break;
    }

    // If the setpoint has a real value, calculate the voltage using PID and FF
    // Else, switch-case to determine whether to intake or stop
    if (setpoint != 0.0) {
        Deploy(false);
        flywheelMotor.SetVoltage(
            units::volt_t { flywheelController.Calculate(flywheelEnc.GetVelocity(), setpoint) }
            + flywheelFF.Calculate( units::radians_per_second_t { setpoint } )
        );
    } else {
        switch (height) {
            case constants::FieldConstants::GridHeights::eIntake:
                Deploy(true);
                break;
            case constants::FieldConstants::GridHeights::eStopped:
                flywheelMotor.SetVoltage(0.0_V);
                break;
            default:
                flywheelMotor.SetVoltage(0.0_V);
                break;
        }
    }
}
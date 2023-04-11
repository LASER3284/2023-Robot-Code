#include "CubeShooter.h"
#include "Constants.h"

using namespace shooter;

CubeShooter::CubeShooter() {
    flywheelMotor.SetInverted(true);
    intakeMotor.SetInverted(false);

    flywheelMotor.SetSmartCurrentLimit(40);
    intakeMotor.SetSmartCurrentLimit(40);

    // Setup PID Controllers
    Retract();

    // Setup Encoder conversion factor
    flywheelEnc.SetVelocityConversionFactor(60 * 2 * constants::Pi * Constants::kFlywheelRatio);
    intakeEnc.SetVelocityConversionFactor(60 * 2 * constants::Pi * Constants::kIntakeRatio);
}

void CubeShooter::Deploy(bool intake) {
    // Deploy the intake with the solenoids
    deploySolenoid.Set(true);

    // Spin the motor!!!!!!!!!!!! pls and ty
    if (intake) {
        intakeMotor.Set(1.0);
        flywheelMotor.Set(0.95);
    }
    else {
        intakeMotor.Set(0.0);
        flywheelMotor.Set(0.0);
    }

    if(!isDeployed) {
        isDeployed = true;
        deployTimer.Restart();
    }
}

void CubeShooter::Retract(){
    // Retract the intake with the solenoids
    deploySolenoid.Set(false);

    //Stop spinning the motor!!!!!!! no thx
    intakeMotor.Set(0.0);
    flywheelMotor.Set(0.5);

    deployTimer.Reset();
    deployTimer.Stop();
    isDeployed = false;
}

void CubeShooter::Shoot(constants::FieldConstants::GridHeights height) {
    // Switch-case based on the height we're aiming for.
    units::radians_per_second_t setpoint = 0_rad_per_s;

    switch (height) {
        case constants::FieldConstants::GridHeights::eUp:
            setpoint = Constants::kHighSetpoint;
            break;
        case constants::FieldConstants::GridHeights::eMid:
            setpoint = Constants::kMidSetpoint;
            break;
        case constants::FieldConstants::GridHeights::eGround:
            setpoint = Constants::kGroundSetpoint;
            break;
        case constants::FieldConstants::GridHeights::eGroundSpit:
            setpoint = Constants::kSpitSetpoint;
        default:
            break;
    }

    // If the setpoint has a real value, calculate the voltage using PID and FF
    // Else, switch-case to determine whether to intake or stop
    if (setpoint != 0_rad_per_s) {
        units::radians_per_second_t intakeSetpoint = setpoint * 0.5;

        if(height == constants::FieldConstants::GridHeights::eGroundSpit) {
            intakeSetpoint += (intakeSetpoint * 0.5);
        }

        // In ground mode, we spin the intake wheels the same way as the flywheel
        // Otherwise, flip the intake setpoint  
        if(height != constants::FieldConstants::GridHeights::eGround && height != constants::FieldConstants::GridHeights::eGroundSpit) {
            intakeSetpoint *= -1;
        }

        if(height == constants::FieldConstants::GridHeights::eMid) {
            setpoint += (setpoint * 0.25);
        }

        // In high mode, we want to spin the intake wheels a little faster than 50% of the flywheel speed.
        if(height == constants::FieldConstants::GridHeights::eUp) {
            intakeSetpoint += (intakeSetpoint * 0.25);
        }

        Deploy(false);
        if(deployTimer.HasElapsed(0.125_s)) {
            intakeMotor.SetVoltage(
                units::volt_t { intakeController.Calculate(intakeEnc.GetVelocity(), intakeSetpoint.value()) }
                + intakeFF.Calculate(intakeSetpoint)
            );

            if(units::math::abs(GetAngularIntakeVelocity()) >= (intakeSetpoint * 0.5)) {
                flywheelMotor.SetVoltage(
                    units::volt_t { flywheelController.Calculate(flywheelEnc.GetVelocity(), setpoint.value()) }
                    + flywheelFF.Calculate(setpoint)
                );
            }
        }
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
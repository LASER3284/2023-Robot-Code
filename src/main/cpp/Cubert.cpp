#include "Cubert.h"
#include "FieldConstants.h"
#include "Constants.h"
#include <ctime>
#include <units/length.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ::constants;

void shooter::Cubert::Init() {
    deployMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    deployMotor.SetInverted(true);
    timekeeper.Start();
}

void shooter::Cubert::Tick() {
    frc::SmartDashboard::PutNumber("cubert_enc_deg", GetAngle().value());
    frc::SmartDashboard::PutNumber("cubert_enc_real", thruboreEnc.GetAbsolutePosition() * 360);
    frc::SmartDashboard::PutNumber("cubert_roller_rpm", _get_roller_avel().value());
    frc::SmartDashboard::PutNumber("cubert_roller_fps", _get_roller_lvel().value());
}

void shooter::Cubert::Shoot(FieldConstants::GridHeights height) {
    if (start_shoot == 0_s)
        start_shoot = timekeeper.Get();

    _set_deploy_goal(constants::kAngleGridMap.at(height));

    frc::TrapezoidProfile<units::degrees> deploy_profile {
        deployConstraints,
        deployGoal,
        deploySetpoint
    };

    deploySetpoint = deploy_profile.Calculate(20_ms);
    _set_deploy(
        units::volt_t { deployController.Calculate(GetAngle().value(), deploySetpoint.position.value()) }
        + deployFF.Calculate(deploySetpoint.velocity)
    );

    if (height == FieldConstants::GridHeights::eStopped) {
        start_shoot = 0_s;
        _set_rollers(-0.75_V);
    } else {
        if (height != FieldConstants::GridHeights::eIntake) {
            if (start_shoot - timekeeper.Get() >= constants::kRollerDelay) {
                _set_rollers(
                    units::volt_t {
                        rollerController.Calculate(
                            _get_roller_avel().value(),
                            constants::kRollerSetpoint.value()
                        )
                    } + constants::kRollerKs
                );
            }
        } else {
            _set_rollers(
                units::volt_t {
                    rollerController.Calculate(
                        falconToRPM(
                            uppieMotor->GetSelectedSensorVelocity(),
                            constants::kRollerRatio
                        ).value(), constants::kRollerIntakeSetpoint.value()
                    )
                }
            );
        }
    }
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

void shooter::Cubert::_set_deploy_goal(units::degree_t angle) {
    deploySetpoint = {angle};
    deployGoal = {angle};
}

units::revolutions_per_minute_t shooter::Cubert::_get_roller_avel() {
    return falconToRPM(
        uppieMotor->GetSelectedSensorVelocity(),
        constants::kRollerRatio
    );
}

units::feet_per_second_t shooter::Cubert::_get_roller_lvel() {
    return falconToMPS(
        uppieMotor->GetSelectedSensorVelocity(),
        units::inch_t{1.125} * 2 * Pi,
        constants::kRollerRatio
    );
}

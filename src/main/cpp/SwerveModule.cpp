#include "SwerveModule.h"
using namespace ctre::phoenix;

drive::SwerveModule::SwerveModule(int drivemotor_in, int turnmotor_in, int encoder_in, bool drive_inverted) {
    // Instantiate each pointer object for WPI_TalonFX and CANCoder
    drivemotor = new motorcontrol::can::WPI_TalonFX(drivemotor_in);
    turnmotor = new rev::CANSparkMax(turnmotor_in, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
    encoder = new sensors::CANCoder(encoder_in);

    drivemotor->SetNeutralMode(motorcontrol::NeutralMode::Brake);
    turnmotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    drivemotor->SetInverted(drive_inverted);

    drivemotor->ConfigSupplyCurrentLimit({ 
        true, // Enable
        35, // Continuous Current Limit
        60, // Peak Current Limit
        0.1 // Peak Current Duration
    });

    turnmotor->SetSmartCurrentLimit(60);
    
    encoder->ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180);
    encoder->ConfigSensorInitializationStrategy(ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition);

    // Limit the PID Controller's input range between -180 and 180 to decrease the maximum travel distance for PID.
    turnPIDController.EnableContinuousInput(
        -180,
        180
    );
    turnPIDController.SetTolerance(2.5);    // For later :)
}

frc::SwerveModuleState drive::SwerveModule::GetState() const {
    return {
        constants::falconToMPS(drivemotor->GetSelectedSensorVelocity(), kWheelCircumference, kGearRatio),
        units::degree_t { encoder->GetAbsolutePosition() }
    };
}

frc::SwerveModulePosition drive::SwerveModule::GetPosition() const {
    return {
        constants::falconToMeters(drivemotor->GetSelectedSensorPosition(), kWheelCircumference, kGearRatio),
        units::degree_t { encoder->GetAbsolutePosition() }
    };
}


void drive::SwerveModule::ResetEncoders() {
    drivemotor->SetSelectedSensorPosition(0);
}

void drive::SwerveModule::SetDesiredState(const frc::SwerveModuleState& refstate, bool force_angle) {    
    // Optimize the reference state to avoid spinning further than 90 degrees
    const auto state = frc::SwerveModuleState::Optimize(
        refstate, frc::Rotation2d((units::degree_t)encoder->GetAbsolutePosition())
    );

    // Calculate the drive output from the drive PID controller.
    const auto driveOutput = drivePIDController.Calculate(
        drivemotor->GetSelectedSensorVelocity(),
        state.speed.value()
    );

    const auto driveff = driveFeedforward.Calculate(state.speed);    
    
    // If we're not moving at <0.1x speed, then we can just give up on rotating
    // This will help avoid jitter on the turn motors.
    double setpoint = state.angle.Degrees().value();
    if(!force_angle && abs(state.speed.value()) < (SwerveModule::kMaxSpeed * 0.1).value()) {
        setpoint = lastAngle.value();
    }

    turnPIDController.SetSetpoint(setpoint);
    const auto turnOutput = turnPIDController.Calculate(encoder->GetAbsolutePosition());

    if(!turnPIDController.AtSetpoint()) {
        // If we're not at the setpoint, move the turn motor.
        turnmotor->Set(turnOutput);
    }
    else {
        // If we're at the setpoint, stop the turn motor
        turnmotor->Set(0.0);
    }

    // If we aren't commanding the wheels to move at all, don't apply the feed forward or anything
    // Set the motor outputs.
    drivemotor->SetVoltage(units::volt_t{driveOutput} + driveff);
    
    lastAngle = state.angle.Degrees();
}

double drive::SwerveModule::getTurnEncPos() {
    return encoder->GetAbsolutePosition();
}

double drive::SwerveModule::getDriveEncPos() {
    return drivemotor->GetSelectedSensorPosition();
}
#include "Shoulder.h"

using namespace ctre::phoenix;

shoulder::Shoulder::Shoulder() {
    motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    motor.SetSmartCurrentLimit(60);

    encoder.ConfigAbsoluteSensorRange(sensors::AbsoluteSensorRange::Signed_PlusMinus180);
    encoder.ConfigSensorInitializationStrategy(sensors::SensorInitializationStrategy::BootToAbsolutePosition);
}

void shoulder::Shoulder::Tick() {
    motor.Set(
        angleController.Calculate(
            units::degree_t{encoder.GetAbsolutePosition()}
        )
    );
}

void shoulder::Shoulder::SetRotationGoal(units::degree_t rot) {
    angleController.SetGoal(rot);
}

units::degree_t shoulder::Shoulder::GetRotation() {
    return units::degree_t{encoder.GetAbsolutePosition()};
}

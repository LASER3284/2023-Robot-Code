#include "Wrist.h"

using namespace ctre::phoenix;

wrist::Wrist::Wrist() {
    motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    motor.SetSmartCurrentLimit(25);

    encoder.ConfigAbsoluteSensorRange(sensors::AbsoluteSensorRange::Signed_PlusMinus180);
    encoder.ConfigSensorInitializationStrategy(sensors::SensorInitializationStrategy::BootToAbsolutePosition);
}

void wrist::Wrist::Tick() {
    motor.Set(
        controller.Calculate(
            units::degree_t{encoder.GetAbsolutePosition()}
        )
    );
}

void wrist::Wrist::SetRotationGoal(units::degree_t rot) {
    controller.SetGoal(rot);
}

units::degree_t wrist::Wrist::GetRotation() {
    return units::degree_t{encoder.GetAbsolutePosition()};
}

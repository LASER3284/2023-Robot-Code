#include "Constants.h"
#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <frc/controller/ProfiledPIDController.h>
#include <units/velocity.h>

namespace arm {
    class Constants{
        public: 
            static constexpr int kArmCANID = 9999;

            ///@brief the arm gear ratio
            static constexpr double kArmRatio = 7000/1;
    };

    class Arm {
        public:
            Arm();

            units::meter_t GetPosition();

            void SetPositionGoal(units::meter_t distance);

            void Tick();

        private:
            rev::CANSparkMax extentionMotor { Constants::kArmCANID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

        frc::ProfiledPIDController<units::meter> positionController { 
                0.0, // kP
                0.0, // kI
                0.0, // kD
                // The trapezoid profile provides a set of constraints for the PID controller to obey.
                frc::TrapezoidProfile<units::meter>::Constraints { 1_mps, 1_mps / 1_s }
            };

    };




}
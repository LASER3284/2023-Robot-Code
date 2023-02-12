#include "Constants.h"
#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <frc/controller/ProfiledPIDController.h>
#include <units/velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace arm {
    class Constants{
        public:
            /// @brief The CAN ID for the Spark Max for extending the arm
            static constexpr int kArmCANID = 19;

            /// @brief The gear ratio for the arm (sprockets are 1 to 1)
            static constexpr double kArmRatio = 27.78/1;

            /// @brief The outer diameter of the sprocket * pi in order to convert to linear units
            static constexpr units::meter_t kSprocketDiameter = (0.0381762_m * constants::Pi);
    };

    class Arm {
        public:
            Arm();

            units::meter_t GetPosition();

            void SetPositionGoal(units::meter_t distance);

            void Tick();

            void ToggleControl(bool enable) { bEnabled = enable; }

            void ManualControl(double percentage) { manualPercentage = percentage; }
        private:
            bool bEnabled = false;
            double manualPercentage = 0.0;

            rev::CANSparkMax extensionMotor { Constants::kArmCANID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
            rev::SparkMaxRelativeEncoder extensionEncoder = extensionMotor.GetEncoder();

            frc::ProfiledPIDController<units::meter> positionController { 
                0.0, // kP
                0.0, // kI
                0.0, // kD
                // The trapezoid profile provides a set of constraints for the PID controller to obey.
                frc::TrapezoidProfile<units::meter>::Constraints { 1_mps, 1_mps / 1_s }
            };

    };
}
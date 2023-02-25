#include "Constants.h"
#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/acceleration.h>
#include <units/time.h>
#include <frc/Timer.h>
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

            /// @brief The feedforward object for the extension of the arm (it acts an ""elevator"")
            frc::ElevatorFeedforward feedforward { 0.49542_V, 0_V, 14.539_V / 1_mps, 0.80597_V / 1_mps_sq };

            /// @brief The trapezoidal profile constraints for the shoulder rotation
            /// This specifies the max rotational velocity *and* the max rotational acceleration
            /// Ideally this would be in the constants but it would not let me do that.
            frc::TrapezoidProfile<units::meters>::Constraints constraints { 0.1_mps, 0.1_mps_sq };

            /// @brief The current goal to rotate the shoulder to
            frc::TrapezoidProfile<units::meters>::State extensionGoal;

            /// @brief The current setpoint for the shoulder rotation
            frc::TrapezoidProfile<units::meters>::State extensionSetpoint;

            /// @brief A timer used for overriding the manual percentage vs the feedforward calculations
            frc::Timer controlTimer;
    };
}
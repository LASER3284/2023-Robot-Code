#include "Constants.h"

#include <rev/CANSparkMax.h>
#include <frc/Solenoid.h>
#include <frc/filter/LinearFilter.h>

namespace intake {
    class Constants {
        public:
            /// @brief The CAN ID for the intake motor
            static constexpr int kIntakeID = 60;

            /// @brief The solenoid port/ID for the intake
            static constexpr int kIntakeSolenoidID1 = 6;

            /// @brief The solenoid port/ID for the intake mode
            static constexpr int kIntakeSolenoidID2 = 7;
    };

    class Intake {
        public:
            Intake();

            void ConeMode();
            void CubeMode();

            void Inhale();
            void Spit();

            void Stop();

            units::ampere_t GetFilteredCurrent() {
                return movingAverage.Calculate( units::ampere_t{ intakeMotor.GetOutputCurrent() });
            }

            bool HasElement();
        private:
            rev::CANSparkMax intakeMotor { Constants::kIntakeID, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
            frc::Solenoid intakeSolenoid1 { frc::PneumaticsModuleType::REVPH, Constants::kIntakeSolenoidID1 };
            frc::Solenoid intakeSolenoid2 { frc::PneumaticsModuleType::REVPH, Constants::kIntakeSolenoidID2 };

            frc::LinearFilter<units::ampere_t> movingAverage = frc::LinearFilter<units::ampere_t>::MovingAverage(4);
    };
}
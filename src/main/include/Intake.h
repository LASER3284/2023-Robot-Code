#include "Constants.h"

#include <rev/CANSparkMax.h>
#include <frc/Solenoid.h>
#include <frc/filter/MedianFilter.h>

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

            /// @brief Flips the intake solenoids into cone mode as well as starts the intake spin motor
            void ConeMode();

            /// @brief Flips the intake solenoids into cubemode as well as changing the intake spin motor
            void CubeMode();

            /// @brief Slowly spins the intake in order to keep the motor stalling and hold an element
            void Hold();

            /// @brief Start the intake motor in order to start intaking things at 90% speed
            void Inhale();

            /// @brief Spin the intake motor in reverse in order to spit out an element
            void Spit();

            /// @brief Stop spinning the intake motor
            void Stop();

            /// @brief Returns the current filtered intake motor current via a 4-sample sized median filter
            /// @return The filtered intake motor current
            units::ampere_t GetFilteredCurrent() {
                return medianFilter.Calculate( units::ampere_t{ intakeMotor.GetOutputCurrent() });
            }

            /// @brief Returns true based on the median current of the intake motor
            /// @return Whether or not the intake current has an element in it
            bool HasElement();

            /// @brief Returns whether or not the intake is currently in cube mode
            /// @return Whether or not the intake is cube mode
            bool IsCubeMode() { return cubeMode; }
        private:

            /// @brief Whether or not the intake is currently in cube mode
            bool cubeMode = false;
            
            /// @brief A reference for the intake NEO 550 object
            rev::CANSparkMax intakeMotor { Constants::kIntakeID, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
            frc::Solenoid intakeSolenoid1 { frc::PneumaticsModuleType::REVPH, Constants::kIntakeSolenoidID1 };
            frc::Solenoid intakeSolenoid2 { frc::PneumaticsModuleType::REVPH, Constants::kIntakeSolenoidID2 };

            /// @brief A median filter used to measure the current of the NEO 550 to detect current spikes for object detection
            frc::MedianFilter<units::ampere_t> medianFilter = frc::MedianFilter<units::ampere_t>(4);
    };
}
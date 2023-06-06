#include "Constants.h"

#include <rev/CANSparkMax.h>
#include <frc/Solenoid.h>
#include <frc/filter/MedianFilter.h>
#include <frc/filter/Debouncer.h>
#include <frc/filter/LinearFilter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <wpi/MathExtras.h>

namespace intake {
    class Constants {
        public:
            /// @brief The CAN ID for the intake motor
            static constexpr int kIntakeID = 56;
    };

    enum FlangeLocation {
        eForwards = 0,
        eBackwards = 1,
        eNone = -1
    };

    class Intake {
        public:
            Intake();

            void Tick();

            /// @brief Flips the intake solenoids into cone mode as well as starts the intake spin motor
            void ConeMode();

            /// @brief Flips the intake solenoids into cubemode as well as changing the intake spin motor
            void CubeMode();

            /// @brief Slowly spins the intake in order to keep the motor stalling and hold an element
            void Hold();

            /// @brief Start the intake motor in order to start intaking things at 90% speed
            void Inhale();

            void SmallSpit();

            /// @brief Change the direction of the intake
            void FlipDirection(bool bInverted) {
                intakePower = bInverted ? 1 : -1;
            }

            void SetHoldPower(bool bInverted) {
                holdPower = bInverted ? 1 : -1;
            }

            /// @brief Spin the intake motor in reverse in order to spit out an element
            void Spit();

            /// @brief Spin the motor in reverse to shoot out cones/cubes
            void Shoot();

            /// @brief Stop spinning the intake motor
            void Stop();

            /// @brief Returns the current filtered intake motor current via a 4-sample sized median filter
            /// @return The filtered intake motor current
            units::ampere_t GetFilteredCurrent() { return intakeCurrent; }

            /// @brief Returns true based on the median current of the intake motor
            /// @return Whether or not the intake current has an element in it
            bool HasElement() { 
                return false;
                //return hasElement; 
            }

            /// @brief Returns whether or not the intake is currently in cube mode
            /// @return Whether or not the intake is cube mode
            bool IsCubeMode() { return cubeMode; }

            double GetOutput() { return intakeMotor.GetAppliedOutput(); }

            FlangeLocation GetFlangeLocation() { 
                switch (holdPower)
                {
                    case -1:
                        return FlangeLocation::eForwards;
                    case 1:
                        return FlangeLocation::eBackwards;
                    default:
                        return FlangeLocation::eNone;
                }
            }

            void SetFlangeLocation(FlangeLocation location) {
                switch(location) {
                    case FlangeLocation::eForwards:
                        holdPower = -1;
                        break;
                    case FlangeLocation::eBackwards:
                        holdPower = 1;
                        break;
                    default:
                        holdPower = 0;
                        break;
                }
            }
        private:
            /// @brief A boolean whether or not the intake has an element
            bool hasElement = false;

            /// @brief Whether or not the intake is currently in cube mode
            bool cubeMode = false;
            
            /// @brief A reference for the intake NEO 550 object
            rev::CANSparkMax intakeMotor { Constants::kIntakeID, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
            rev::SparkMaxRelativeEncoder intakeEncoder = intakeMotor.GetEncoder();

            /// @brief A median filter used to measure the current of the NEO 550 to detect current spikes for object detection
            frc::MedianFilter<units::ampere_t> medianFilter = frc::MedianFilter<units::ampere_t>(18);

            frc::Debouncer fallingDebouncer { 0.35_s, frc::Debouncer::DebounceType::kFalling };
            frc::Debouncer risingDebouncer  { 0.15_s, frc::Debouncer::DebounceType::kRising };

            units::ampere_t intakeCurrent = 0_A;

            int intakePower = 1;
            int holdPower = 1;
    };
}
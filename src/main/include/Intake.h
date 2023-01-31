// this code is to create the classes for the intake and crate the numatc controls of the intake 
#include "Robot.h"
#include "Constants.h"

#include <rev/CANSparkMax.h>
#include <frc/Solenoid.h>
#include <frc/filter/LinearFilter.h>

namespace intake {
    class Constants {
        public:
            static constexpr int kIntakeID = 10;
        
    };

    class Intake {
        public:
            Intake();

            void ConeMode();
            void CubeMode();

            void Inhale();
            void Spit();

            units::ampere_t GetFilteredCurrent() {
                return highPass.Calculate(movingAverage.Calculate(units::ampere_t{intakeMotor.GetOutputCurrent()}));
            }


            // TODO: Implement
            bool HasElement();
        private:
            rev::CANSparkMax intakeMotor { Constants::kIntakeID, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
            frc::Solenoid intakeSolenoid1 { frc::PneumaticsModuleType::REVPH, 1 };
            frc::Solenoid intakeSolenoid2 { frc::PneumaticsModuleType::REVPH, 2 };

            frc::LinearFilter<units::ampere_t> movingAverage = frc::LinearFilter<units::ampere_t>::MovingAverage(8);
            frc::LinearFilter<units::ampere_t> highPass = frc::LinearFilter<units::ampere_t>::HighPass(0.5, 0.02_s);

    };
}
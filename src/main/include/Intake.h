// this code is to create the classes for the intake and crate the numatc controls of the intake 
#include "Robot.h"
#include "Constants.h"

#include <rev/CANSparkMax.h>

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

            void Spin();
            void Spit();

            // TODO: Implement
            bool HasElement();
        private:
            rev::CANSparkMax intakeMotor { Constants::kIntakeID, rev::CANSparkMaxLowLevel::MotorType::kBrushless };  
    };
    
}

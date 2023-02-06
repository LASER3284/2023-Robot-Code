#pragma once

#include "Constants.h"
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <frc/geometry/Pose3d.h>

namespace kinematics {
    class Constants {
        public:
            static constexpr units::meter_t coneModeLength = 12_in;
            static constexpr units::meter_t cubeModeLength = 16.5_in;

            static constexpr units::meter_t staticArmLength = 14_in;

            static constexpr units::meter_t pivotPointHorizontalOffset = 14_in;

            static constexpr units::meter_t wheelToBellyPan = 4_in;
            static constexpr units::meter_t bellyPanToPivotPoint = 18_in;

            static constexpr frc::Pose3d    pivotPoint { 0_m, 0_m, 0_m };
    };

    struct KinematicState {
        const units::meter_t armExtension;
        const units::degree_t shoulderAngle;
        const units::degree_t wristAngle;
    };

    class Kinematics {

        /// @brief Returns whether or not a kinematic state / intake mode is valid
        /// @param coneMode Whether or not the intake is in cone mode
        /// @param state The current kinematic state of the arm
        /// @return Whether or not the kinematic state is within the legal extension limits
        static bool IsValid(bool coneMode, KinematicState state) {
            const units::meter_t intakeLength = (coneMode ? Constants::coneModeLength : Constants::cubeModeLength);

            // di is the horizontal extension of the wrist
            units::meter_t di = 0_m;
            if(state.wristAngle != 90_deg) {
                units::radian_t C = units::radian_t( (180_deg - abs(state.wristAngle.value()) - 90_deg) );
                di = (
                    intakeLength * sin(C.value()) / 
                    sin(units::radian_t(90_deg).value())
                );
            }
            else if(state.wristAngle == 90_deg) {
                di = intakeLength;
            }

            // m is the total horizontal extension of the arm up to the wrist
            units::radian_t mC = units::radian_t(180_deg - state.shoulderAngle - 90_deg);
            units::meter_t m = (Constants::staticArmLength + state.armExtension) * sin(mC.value());

            units::meter_t horizontalExtension = (m - Constants::pivotPointHorizontalOffset) + di;
            if(horizontalExtension >= 48_in) {
                return false;
            }

            // he is the vertical extension of the arm including static points
            units::meter_t he = 0_m;
            if(state.shoulderAngle != 180_deg && state.shoulderAngle != 0_deg) {
                // see: pythagorean theorem
                he = units::meter_t(
                    sqrt(
                        pow(((Constants::staticArmLength + state.armExtension)).value(), 2) - pow(m.value(), 2)
                    )
                );
            }
            else if(state.shoulderAngle == 180_deg) {
                he = m;
            }

            // hi is the vertical extension of the intake
            units::meter_t hi = 0_m;
            if(state.wristAngle != 180_deg && state.wristAngle != 0_deg) {
                hi = units::meter_t(
                    sqrt(
                        pow(intakeLength.value(), 2) - pow(di.value(), 2)
                    )
                );
            }
            else if(state.wristAngle == 180_deg) {
                hi = di;
            }

            // Check the vertical extension and return true if we're less than 72in
            return Constants::wheelToBellyPan + Constants::bellyPanToPivotPoint + he + hi < 72_in;
        }

        const KinematicState GetKinematicState(bool coneMode, frc::Pose3d intakePoint) {
            const units::meter_t intakeLength = (coneMode ? Constants::coneModeLength : Constants::cubeModeLength);

            const units::meter_t xO = intakePoint - Constants::pivotPoint.X();
            const units::meter_t zO = intakePoint - Constants::pivotPoint.Z();

            const units::meter_t L = units::meter_t( 
                sqrt(pow(xO.value(), 2) + pow(zO.value(), 2)) - 
                Constants::staticArmLength + intakeLength
            );

            const units::radian_t shoulderTheta = units::radian_t(
                asin((zO / (Constants::staticArmLength + L + intakeLength)).value())
            );

            const units::meter_t wristHeight = (Constants::staticArmLength + L) * sin(shoulderTheta.value());

            const units::meter_t intakeHeight = zO - wristHeight;

            const units::radian_t intakeTheta = units::radian_t(
                asin((intakeHeight / intakeLength).value())
            );

            return KinematicState {  L, shoulderTheta, intakeTheta };
        }
    
        const units::volt_t CalculateShoulderkG(const units::degree_t x) {
            return units::volt_t(x.value() * 0);
        }

        const units::volt_t CalculateWristkG(const units::degree_t x) {
            return units::volt_t(x.value() * 0);
        }
    };
}
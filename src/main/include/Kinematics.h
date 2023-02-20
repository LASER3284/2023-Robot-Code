#pragma once

#include "Constants.h"
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <limits>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Pose3d.h>

namespace kinematics {    
    class Constants {
        public:
            /// @brief The length of the intake in cone mode
            static constexpr units::meter_t coneModeLength = 12_in;

            /// @brief The length of the intake in cube mode
            static constexpr units::meter_t cubeModeLength = 16.5_in;

            /// @brief The horizontal offset from the edge of the robot of the center of the pivot point
            static constexpr units::meter_t pivotPointHorizontalOffset = 14_in;

            /// @brief The length of the static arm (minimum extension) relative to the center of shoulder rotation
            static constexpr units::meter_t staticArmLength = 40_in - pivotPointHorizontalOffset;

            /// @brief The distance from the bottom of the wheel to the edge of the bellypan
            static constexpr units::meter_t wheelToBellyPan = 4_in;
            
            /// @brief The distance from the bellypan to the pivot point
            static constexpr units::meter_t bellyPanToPivotPoint = 18_in;

            /// @brief The height and location of the pivot point of the shoulder relative to the center of rotation of the robot
            static constexpr frc::Translation3d pivotPoint = { 0_in, 0_m, bellyPanToPivotPoint + wheelToBellyPan };

            /// @brief The maximum extension length for the arm
            static constexpr units::meter_t maxArmExtension = 0.77_m;
    };

    struct KinematicState {
        const units::meter_t armExtension;
        const units::degree_t shoulderAngle;
        const units::degree_t wristAngle;
    };

    class Kinematics {
        public:
            
            /// @brief Returns whether or not a kinematic state / intake mode is valid
            /// @param coneMode Whether or not the intake is in cone mode
            /// @param state The current kinematic state of the arm
            /// @return Whether or not the kinematic state is within the legal extension limits
            static bool IsValid(bool coneMode, KinematicState state) {
                const units::meter_t intakeLength = (coneMode ? Constants::coneModeLength : Constants::cubeModeLength);

                // di is the horizontal extension of the wrist
                units::meter_t di = 0_m;
                if(state.wristAngle != 90_deg) {
                    units::radian_t C = units::radian_t( (180_deg - units::math::abs(state.wristAngle) - 90_deg) );
                    di = (
                        intakeLength * units::math::sin(C) / 
                        units::math::sin(90_deg)
                    );
                }
                else if(state.wristAngle == 90_deg) {
                    di = intakeLength;
                }

                // m is the total horizontal extension of the arm up to the wrist
                units::radian_t mC = units::radian_t(180_deg - state.shoulderAngle - 90_deg);
                units::meter_t m = (Constants::staticArmLength + state.armExtension) * units::math::sin(mC);

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

            static const KinematicState GetKinematicState(bool coneMode, frc::Translation3d intakePoint, frc::Translation2d robotPose) {
                return GetKinematicState(coneMode, intakePoint, frc::Translation3d(robotPose.X(), robotPose.Y(), 0_m));
            }

            static const KinematicState GetKinematicState(bool coneMode, frc::Translation3d intakePoint, frc::Translation3d robotPose) {
                const units::meter_t intakeLength = (coneMode ? Constants::coneModeLength : Constants::cubeModeLength);

                // Intake location with respect to the robot pose
                const frc::Translation3d intakeR = (intakePoint - robotPose);

                const units::meter_t xO = intakeR.X() - Constants::pivotPoint.X();
                const units::meter_t zO = intakeR.Z() - Constants::pivotPoint.Z();

                units::meter_t L = units::math::sqrt(units::math::pow<2>(xO) + units::math::pow<2>(zO)) - Constants::staticArmLength + intakeLength;

                // If the arm is trying to extend further than the arm physically can, limit its extension
                if(L > Constants::maxArmExtension) {
                    L = Constants::maxArmExtension;
                }

                const units::radian_t shoulderTheta = units::math::asin((zO / (Constants::staticArmLength + L + intakeLength)));

                const units::meter_t wristHeight = (Constants::staticArmLength + L) * units::math::sin(shoulderTheta);

                const units::meter_t intakeHeight = zO - wristHeight;

                // Get the angle of the wrist relative to the shoulder angle
                units::radian_t intakeTheta = units::math::asin(intakeHeight / intakeLength) - shoulderTheta;
                // Clamp the intake angle to be 0 if the angle is less than 0.5 degrees
                intakeTheta = units::math::abs(intakeTheta) < 0.5_deg ? 0_rad : intakeTheta;

                return KinematicState {  L, shoulderTheta, intakeTheta };
            }


            /// @brief Calculates the feed forward voltage based on the angle of the arm and the extension length
            /// @param extension The extension of the arm
            /// @param angle The angle setpoint in radians (This angle should have 0 as parallel to the floor)
            /// @param velocity The velocity setpoint in radians per second
            /// @param accel The acceleration setpoint in radians per second²
            /// @return The calculated feedforward in volts
            static const units::volt_t CalculateShoulderFeedforward(const units::meter_t extension, const units::radian_t angle, const units::radians_per_second_t velocity, const units::radians_per_second_squared_t accel = 0_rad_per_s_sq) {
                //const units::volt_t kS = ((-0.0611_V *  extension.value()) + 0.57817_V);
                const units::volt_t kG = ((0.10971_V * extension.value()) + 0.28335_V);
                const auto kV =          ((0.17423_V * extension.value()) + 3.90020_V) / 1_rad_per_s;
                const auto kA =          ((-0.83796_V * extension.value()) + 1.44075_V) / 1_rad_per_s_sq;
                
                return (kG * units::math::cos(angle)) + (kV * velocity) + (kA * accel);
            }
    };
}
#pragma once

#include <units/length.h>
#include <array>
#include <tuple>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/DriverStation.h>
#include "pathplanner/lib/PathPlannerTrajectory.h"

namespace constants {
    class FieldConstants {
        public:
            enum GridHeights {
                eUp = 0,
                eMid,
                eGround,
                eGroundSpit,
                eIntake,
                eStopped,
            };

            static constexpr units::inch_t fieldLength = 651.25_in;
            static constexpr units::inch_t fieldWidth = 315.5_in;

            struct ScoringLocation {
                frc::Translation3d location;
                bool isCube;
            };

            static constexpr int nodeColumnCount = 3;

            static constexpr units::inch_t communityMidX = 132.375_in; // The X location of the tape line to the left of the charge station (when viewed from the blue alliance driver station)
            static constexpr units::inch_t communityOuterX = 192.25_in; // X location of the tape line to the right of the charge station (when viewed from the blue alliance driver station)

            static constexpr units::inch_t outerX = 54.25_in;
            static constexpr units::inch_t lowX = outerX - 7.125_in; // Centered when under cube nodes
            static constexpr units::inch_t midX = outerX - 22.75_in;
            static constexpr units::inch_t highX = outerX - 39.75_in;

            static constexpr int nodeRowCount = 9;

            static constexpr units::inch_t nodeFirstY = 20.19_in;
            static constexpr units::inch_t nodeSeparationY = 22.0_in;

            static constexpr units::inch_t additionalOffset = 15.5_in;

            static constexpr units::inch_t cubeEdgeHigh = 3.0_in;
            static constexpr units::inch_t highCubeZ = 35.5_in - cubeEdgeHigh + additionalOffset;
            static constexpr units::inch_t midCubeZ =  23.5_in - cubeEdgeHigh + additionalOffset;
            static constexpr units::inch_t highConeZ = 46.0_in + additionalOffset;
            static constexpr units::inch_t midConeZ =  34.0_in + additionalOffset;

            static constexpr units::inch_t complexLowXCones = outerX - units::inch_t(16 / 2);
            static constexpr units::inch_t complexLowXCubes = lowX; // Centered X under cube nodes
            static constexpr units::inch_t complexLowOuterYOffset = nodeFirstY - 3.0_in - units::inch_t(25.75 / 2.0);

            std::array<ScoringLocation, nodeRowCount> lowLocations;
            std::array<ScoringLocation, nodeRowCount> midLocations;
            std::array<ScoringLocation, nodeRowCount> highLocations;
            std::array<ScoringLocation, nodeRowCount> lowComplexLocations;

            /// @brief An array of tuples which defines the bottom left hand corner of the grid and the top right corner of the grid
            std::array<std::tuple<frc::Translation2d, frc::Translation2d>, 3> gridLocations;

            FieldConstants() { Initialize(); }

            void Initialize() {
                for(int row = 0; row < nodeRowCount; row++) {
                    bool isCube = (row == 1 || row == 4 || row == 7);
                    
                    lowLocations[row] = { 
                        frc::Translation3d(lowX, units::inch_t(nodeFirstY.value() + (nodeSeparationY.value() * row)), 7.5_in), 
                        true
                    };
                    midLocations[row] = {
                        frc::Translation3d(midX, units::inch_t(nodeFirstY.value() + (nodeSeparationY.value() * row)), isCube ? midCubeZ : midConeZ), 
                        isCube
                    };
                    highLocations[row] = {
                        frc::Translation3d(highX, units::inch_t(nodeFirstY.value() + (nodeSeparationY.value() * row)), isCube ? highCubeZ : highConeZ), 
                        isCube
                    };
                }

                lowComplexLocations[0] = { frc::Translation3d(complexLowXCones, nodeFirstY - complexLowOuterYOffset, 0_in), false };
                lowComplexLocations[1] = { frc::Translation3d(complexLowXCones, nodeFirstY - complexLowOuterYOffset, 0_in), false };
                lowComplexLocations[2] = { frc::Translation3d(complexLowXCubes, nodeFirstY + nodeSeparationY * 1, 0_in), true };
                lowComplexLocations[3] = { frc::Translation3d(complexLowXCones, nodeFirstY + nodeSeparationY * 2, 0_in), false };
                lowComplexLocations[4] = { frc::Translation3d(complexLowXCones, nodeFirstY + nodeSeparationY * 3, 0_in), false };
                lowComplexLocations[5] = { frc::Translation3d(complexLowXCubes, nodeFirstY + nodeSeparationY * 4, 0_in), true };
                lowComplexLocations[6] = { frc::Translation3d(complexLowXCones, nodeFirstY + nodeSeparationY * 5, 0_in), false };
                lowComplexLocations[7] = { frc::Translation3d(complexLowXCones, nodeFirstY + nodeSeparationY * 6, 0_in), false };
                lowComplexLocations[8] = { frc::Translation3d(complexLowXCubes, nodeFirstY + nodeSeparationY * 7, 0_in), true };
                lowComplexLocations[9] = { frc::Translation3d(complexLowXCones, nodeFirstY + nodeSeparationY * 8 + complexLowOuterYOffset, 0_in), false };

                gridLocations[0] = { 
                    frc::Translation2d(0_m, 0_m), 
                    frc::Translation2d(communityOuterX, lowComplexLocations[0].location.Y() + (nodeSeparationY * 3)) 
                };
                gridLocations[1] = {
                    frc::Translation2d(0_m, lowComplexLocations[0].location.Y() + (nodeSeparationY * 3)),
                    frc::Translation2d(communityOuterX, lowComplexLocations[0].location.Y() + (nodeSeparationY * 6))
                };
                gridLocations[2] = {
                    frc::Translation2d(0_m, lowComplexLocations[0].location.Y() + (nodeSeparationY * 6)),
                    frc::Translation2d(communityMidX, lowComplexLocations[0].location.Y() + (nodeSeparationY * 9))
                };

                // Flip based on alliance color
                if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
                    for(int r = 0; r < nodeRowCount; r++) {
                        // In order to flip the location, just change the X location based on the field length when you're on the red alliance
                        lowLocations[r].location        = frc::Translation3d(fieldLength - lowLocations[r].location.X(), lowLocations[r].location.Y(), lowLocations[r].location.Z());
                        midLocations[r].location        = frc::Translation3d(fieldLength - midLocations[r].location.X(), midLocations[r].location.Y(), midLocations[r].location.Z());
                        highLocations[r].location       = frc::Translation3d(fieldLength - highLocations[r].location.X(), highLocations[r].location.Y(), highLocations[r].location.Z());
                        lowComplexLocations[r].location = frc::Translation3d(fieldLength - lowComplexLocations[r].location.X(), lowComplexLocations[r].location.Y(), lowComplexLocations[r].location.Z());
                    }

                    for(int i = 0; i < 3; i++) {
                        const frc::Translation2d bl = std::get<0>(gridLocations[i]);
                        const frc::Translation2d tr = std::get<1>(gridLocations[i]);
                        gridLocations[i] = { 
                            frc::Translation2d(fieldLength - bl.X(), bl.Y()), 
                            frc::Translation2d(fieldLength - tr.X(), tr.Y()) 
                        };
                    }
                }
            }
    };
}

# 2023 Charged Up FRC Robot Code - 3284 Camdenton LASER

## Robot base
* Swerve drive train
    * NEO turning and Falcon 500 drive
    * SDS Mk. III L1 modules
* Cubert does cubes on low and mid positions on grid
* Arm does cones on mid and high, as well as high cubes

## Code structure
* Namespaces are used for different parts of the robot
    * NOTE: There is an incosistency with `constants` namespace vs `Constants`
    class with static members for mechanism-specific constants.
    * `arm` is for extension, `shoulder` is for pivot, `wrist` is for intake
    pivot, `intake` is for arm intake, `cubert` wraps both the cube intake and
    pivot.
* `frc::TimedRobot` base is used for main control process
* Path planning through PathPlanner (tweaked to flip alliance stations)
* Pose estimation via AprilTags and PhotonVision / photonlib

## Contributors
* Thomas Iliff
* Charlotte Patton
* Nate Franklin
* Jordan Branson
* Emma Weiss

## TODO
* Do more auto testing
    * 2.5pc
    * Balance on actual charge station

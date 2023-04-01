# 2023 Charged Up FRC Robot Code - 3284 Camdenton LASER

## Robot base
* Swerve drive train
    * NEO turning and Falcon 500 drive
    * SDS Mk. III L2 modules
* Cube Shooter acts for cube intake and shooter for all three positions on the grid
* Cone Arm acts for cone intake and placement for all three positions on the grid

## Code structure
* Namespaces are used for different parts of the robot
* `frc::TimedRobot` base is used for main control process
* Path planning through PathPlanner (tweaked to flip alliance stations)
* Pose estimation via AprilTags and PhotonVision / photonlib
* Aux controllers are both placed into a single wrapper class, since we have two for aux driver.

## Contributors
* Thomas Iliff
* Charlotte Patton
* Nate Franklin
* Jordan Branson
* Emma Weiss
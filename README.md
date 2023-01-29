# RoveJoint
Embedded software libraries for control of robotic joints. A [RoveJoint](RoveJoint.h) requires a motor, with support for an encoder, soft/hard limits, and closed loop control. A [RoveDifferentialJoint](RoveDifferentialJoint.h) contains the same information, but for two joints connected in a differential pairing (typically used for the Arm's wrist).

## Dependencies
 - [RoveMotor](https://github.com/MissouriMRDT/RoveMotor)
 - [RoveEncoder](https://github.com/MissouriMRDT/RoveEncoder)
 - [RoveSwitch](https://github.com/MissouriMRDT/RoveSwitch)
 - [RoveControl](https://github.com/MissouriMRDT/RoveControl)
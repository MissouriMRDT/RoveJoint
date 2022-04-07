#include "RoveJointBetaTest.h"

void setup()
{
    Serial.begin(115200);
    ShoulderTilt.motor.attach(MotorINA_1,MotorINB_1,MotorPWM_1);
    ShoulderTwist.motor.attach(MotorINA_2,MotorINB_2,MotorPWM_2);
    ElbowTilt.motor.attach(MotorINA_3,MotorINB_3,MotorPWM_3);
    ElbowTilt.motor.attach(MotorINA_4,MotorINB_4,MotorPWM_4);
    Wrist.rightmotor.attach(MotorINA_5,MotorINB_5,MotorPWM_5);
    Wrist.leftmotor.attach(MotorINA_6,MotorINB_6,MotorPWM_6);
    Gripper.motor.attach(MotorINA_7,MotorINB_7,MotorPWM_7);

    ShoulderTilt.encoder.attach(Encoder_ShoulderTilt);
    ShoulderTwist.encoder.attach(Encoder_ShoulderTwist);
    ElbowTilt.encoder.attach(Encoder_ElbowTilt);
    ElbowTilt.encoder.attach(Encoder_ElbowTwist);
    Wrist.tiltEncoder.attach(Encoder_WristTilt);
    Wrist.twistEncoder.attach(Encoder_WristTwist);

    ShoulderTilt.attachLimitSwitches(LimitSwitchLower_J1, LimitSwitchUpper_J1);
    ShoulderTwist.attachLimitSwitches(LimitSwitchLower_J2, LimitSwitchUpper_J2);
    ElbowTilt.attachLimitSwitches(LimitSwitchLower_J3, LimitSwitchUpper_J3);

    ShoulderTilt.encoder.start();
    ShoulderTwist.encoder.start();
    ElbowTilt.encoder.start();
    ElbowTilt.encoder.start();
    Wrist.tiltEncoder.start();
    Wrist.twistEncoder.start();


}

void loop()
{

}

void parsePackets()
{

}

void updatePosition()
{

}

void closedLoop()
{

}
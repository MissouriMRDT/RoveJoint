#include "RoveJointTest2.h"

void setup()
{
    Serial.begin(115200);
    Serial.println("Begun");
    RoveComm.begin(RC_MICROPIBOARD_FOURTHOCTET, &TCPServer);
    delay(100);
    Wrist.rightMotor.attach(MotorINA_1, MotorINB_1, MotorPWM_1);
    Wrist.LeftMotor.attach(MotorINA_2, MotorINB_2, MotorPWM_2);
    Watchdog.begin(estop);
    watchdog.start(2000);
}

void loop()
{
    packet = RoveComm.read();

    switch ( packet.data_id )
    {
    case RC_ARMBOARD_ARMVELOCITYCONTROL_DATA_ID:
        int16_t* motorSpeeds = (int16_t*)packet.data;
        Wrist.tiltTwistDrive(motorSpeeds[4], motorSpeeds[5]);
        break;
    default:
        break;
    }
}
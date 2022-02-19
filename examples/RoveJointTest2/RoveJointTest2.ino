#include "RoveJointTest2.h"

void setup()
{
    Serial.begin(115200);
    Serial.println("Begun");
    RoveComm.begin(RC_ARMBOARD_FOURTHOCTET, &TCPServer);
    delay(100);

    J1.motor_1.attach(MotorINA_1, MotorINB_1, MotorPWM_1);
    /*Shoulder.rightMotor.attach(SHOULDER_RIGHT_INA, SHOULDER_RIGHT_INB, SHOULDER_RIGHT_PWM);
    Elbow.LeftMotor.attach(ELBOW_LEFT_INA, ELBOW_LEFT_INB, ELBOW_LEFT_PWM);
    Elbow.rightMotor.attach(ELBOW_RIGHT_INA, ELBOW_RIGHT_INB, ELBOW_RIGHT_PWM);*/
}

void loop()
{
    Serial.println("Driving Motor 1");
    delay(1000);
    J1.DriveMotor(1000);
}
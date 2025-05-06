#include "RoveJointTest2.h"

void setup()
{
    Serial.begin(115200);
    Serial.println("Begun");
    RoveComm.begin(RC_ARMBOARD_FOURTHOCTET, &TCPServer);
    delay(100);
    J1.motor_1.attach(MotorINA_1, MotorINB_1, MotorPWM_1);
    J2.motor_1.attach(MotorINA_2, MotorINB_2, MotorPWM_2);
    J3.motor_1.attach(MotorINA_3, MotorINB_3, MotorPWM_3);
    J4.motor_1.attach(MotorINA_4, MotorINB_4, MotorPWM_4);
    Gripper.motor_1.attach(MotorINA_5, MotorINB_5, MotorPWM_5);
    //J1.encoder_1.attach(Encoder_J3,false,7,250200);
    //J1.encoder_1.start();
    Watchdog.attach(estop);
    Watchdog.start(watchdogTimeout);
}

void loop()
{
    packet = RoveComm.read();

    switch ( packet.data_id )
    {
        case RC_ARMBOARD_ARMVELOCITYCONTROL_DATA_ID:
        {
            int16_t* motorSpeeds = (int16_t*)packet.data;
            J1.DriveMotor(motorSpeeds[0]);
            J2.DriveMotor(motorSpeeds[1]);
            J3.DriveMotor(motorSpeeds[2]);
            J4.DriveMotor(motorSpeeds[3]);
            RoveComm.writeTo(RC_ARMBOARD_ARMVELOCITYCONTROL_DATA_ID, RC_ARMBOARD_ARMVELOCITYCONTROL_DATA_COUNT,
                            motorSpeeds, RC_ROVECOMM_SUBNET_IP_FIRST_OCTET, RC_ROVECOMM_SUBNET_IP_SECOND_OCTET,
                            RC_ROVECOMM_SUBNET_IP_THIRD_OCTET, RC_MICROPIBOARD_FOURTHOCTET, RC_ROVECOMM_ETHERNET_UDP_PORT);
            Watchdog.clear();
            break;
        }
        case RC_ARMBOARD_GRIPPERMOVE_DATA_ID:
        {
            int16_t* gripperSpeed = (int16_t*)packet.data;
            Gripper.DriveMotor(gripperSpeed[0]);
            Watchdog.clear();
            break;
        }
        case RC_ARMBOARD_LASERS_DATA_ID:
        {
            uint8_t* laser = (uint8_t*)packet.data;
            RoveComm.writeTo(RC_ARMBOARD_LASERS_DATA_ID, RC_ARMBOARD_LASERS_DATA_COUNT,
                            laser, RC_ROVECOMM_SUBNET_IP_FIRST_OCTET, RC_ROVECOMM_SUBNET_IP_SECOND_OCTET,
                            RC_ROVECOMM_SUBNET_IP_THIRD_OCTET, RC_HEATERBOARD_FOURTHOCTET, RC_ROVECOMM_ETHERNET_UDP_PORT);
            
            Watchdog.clear();
            break;
        }
        case RC_ARMBOARD_SOLENOID_DATA_ID:
        {
            uint8_t* solenoid = (uint8_t*)packet.data;
            RoveComm.writeTo(RC_ARMBOARD_SOLENOID_DATA_ID, RC_ARMBOARD_SOLENOID_DATA_COUNT,
                            solenoid, RC_ROVECOMM_SUBNET_IP_FIRST_OCTET, RC_ROVECOMM_SUBNET_IP_SECOND_OCTET,
                            RC_ROVECOMM_SUBNET_IP_THIRD_OCTET, RC_HEATERBOARD_FOURTHOCTET, RC_ROVECOMM_ETHERNET_UDP_PORT);
            Watchdog.clear();
            break;
        }
        default:
            break;
    } 
}

void estop()
{
    J1.DriveMotor(0);
    J2.DriveMotor(0);
    J3.DriveMotor(0);
    J4.DriveMotor(0);
    Watchdog.clear();
}
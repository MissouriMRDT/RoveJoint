#include "RoveJointBetaTest.h"

void setup()
{
    Serial.begin(115200);
    ShoulderTilt.motor.attach(MotorINA_1,MotorINB_1,MotorPWM_1);
    ShoulderTwist.motor.attach(MotorINA_2,MotorINB_2,MotorPWM_2);
    ElbowTilt.motor.attach(MotorINA_3,MotorINB_3,MotorPWM_3);
    ElbowTilt.motor.attach(MotorINA_4,MotorINB_4,MotorPWM_4);
    Wrist.rightMotor.attach(MotorINA_5,MotorINB_5,MotorPWM_5);
    Wrist.leftMotor.attach(MotorINA_6,MotorINB_6,MotorPWM_6);
    Gripper.motor.attach(MotorINA_7,MotorINB_7,MotorPWM_7);

    ShoulderTilt.encoder.attach(Encoder_ShoulderTilt);
    ShoulderTwist.encoder.attach(Encoder_ShoulderTwist);
    ElbowTilt.encoder.attach(Encoder_ElbowTilt);
    ElbowTwist.encoder.attach(Encoder_ElbowTwist);
    Wrist.tiltEncoder.attach(Encoder_WristTilt);
    Wrist.twistEncoder.attach(Encoder_WristTwist);

    ShoulderTilt.attachLimitSwitches(LimitSwitchLower_J1, LimitSwitchUpper_J1);
    ShoulderTwist.attachLimitSwitches(LimitSwitchLower_J2, LimitSwitchUpper_J2);
    ElbowTilt.attachLimitSwitches(LimitSwitchLower_J3, LimitSwitchUpper_J3);

    ShoulderTilt.encoder.start();
    ShoulderTwist.encoder.start();
    ElbowTilt.encoder.start();
    ElbowTwist.encoder.start();
    Wrist.tiltEncoder.start();
    Wrist.twistEncoder.start();

    RoveComm.begin(RC_ARMBOARD_FOURTHOCTET, &TCPServer);

    Watchdog.attach(estop);
    WatchdogTelemetry.attach(telemetry);
    Watchdog.start(watchdogTimeout);
    WatchdogTelemetry.start(ROVECOMM_UPDATE_RATE);
}

void loop()
{
    packet = RoveComm.read();

    switch ( packet.data_id )
    {
        case RC_ARMBOARD_ARMVELOCITYCONTROL_DATA_ID:
            int16_t* motorSpeeds; 
            motorSpeeds = (int16_t*)packet.data;
            ShoulderTilt.DriveMotor(motorSpeeds[0]);
            ShoulderTwist.DriveMotor(motorSpeeds[1]);
            ElbowTilt.DriveMotor(motorSpeeds[2]);
            ElbowTwist.DriveMotor(motorSpeeds[3]);
            Wrist.tiltTwistDrive(motorSpeeds[4], motorSpeeds[5]);
            Watchdog.clear();
            break;
        case RC_ARMBOARD_LASERS_DATA_ID:
            if (packet.data[0])
            {
                digitalWrite(LaserToggle, HIGH);
            }
            else
            {
                digitalWrite(LaserToggle, LOW);
            }
            Watchdog.clear();
            break;
        case RC_ARMBOARD_SOLENOID_DATA_ID:
            if (packet.data[0])
            {
                digitalWrite(SolenoidToggle, HIGH);
            }
            else
            {
                digitalWrite(SolenoidToggle, LOW);
            }
            Watchdog.clear();
            break;
        case RC_ARMBOARD_GRIPPERMOVE_DATA_ID:
            int16_t* gripperSpeed;
            gripperSpeed = (int16_t*)packet.data;
            Gripper.DriveMotor(gripperSpeed[0]);
            Watchdog.clear();
            break;
        default:
            break;
    }   
}

/*void parsePackets()
{

}


void updatePosition()
{

}

void closedLoop()
{

}
*/

void telemetry()
{
    return;
}


void estop()
{
    ShoulderTilt.DriveMotor(0);
    ShoulderTwist.DriveMotor(0);
    ElbowTilt.DriveMotor(0);
    ElbowTwist.DriveMotor(0);
    Wrist.tiltTwistDrive(0, 0);
    digitalWrite(LaserToggle, LOW);
    digitalWrite(SolenoidToggle, LOW);
    Gripper.DriveMotor(0);
}
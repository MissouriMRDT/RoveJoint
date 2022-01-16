#include "RoveJointBrushedDC.h"
#include "RoveComm.h"

RoveJointBrushed joint;

const uint8_t JOINT_RIGHT_INA      = PL_0;
const uint8_t JOINT_RIGHT_INB      = PL_1;
const uint8_t JOINT_RIGHT_PWM      = PF_1;
const uint8_t JOINT_ENCODER        = PM_1;

const uint8_t LS_1 = PM_5;
const uint8_t LS_2 = PM_4;

RoveCommEthernet RoveComm;
rovecomm_packet packet;
EthernetServer TCPServer(RC_ROVECOMM_DRIVEBOARD_PORT);

void setup()
{
    Serial.begin(9600);
    RoveComm.begin(RC_ARMBOARD_FOURTHOCTET, &TCPServer);
    Serial.println("Initialised");
    delay(100);
    joint.attachJoint(JOINT_ENCODER);
    joint.Motor.attach(JOINT_RIGHT_INA, JOINT_RIGHT_INB, JOINT_RIGHT_PWM);
}

void loop()
{
    packet = RoveComm.read();

    Serial.println("Data id: ");
    Serial.println(packet.data_id);

    switch(packet.data_id)
    {
        case RC_ARMBOARD_ARMVELOCITYCONTROL_DATA_ID:
        Serial.println("Open Loop");
        Serial.println(packet.data_id);
        break;
        case RC_ARMBOARD_REQUESTJOINTPOSITIONS_DATA_ID:
        Serial.println("Angle");
        Serial.println(joint.getJointAngle());
        break;
        default:
        break;
    }
}
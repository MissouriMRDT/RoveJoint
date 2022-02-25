#pragma once

#include "RoveComm.h"
#include "RoveJoint.h"

#define MotorPWM_1  PF_1
#define MotorPWM_2  PF_2

#define MotorINA_1  PC_6
#define MotorINA_2  PD_3

#define MotorINB_1  PE_5
#define MotorINB_2  PC_7

RoveJointDifferential Wrist;

RoveCommEthernet RoveComm;
rovecomm_packet packet;
RoveWatchdog Watchdog;
EthernetServer TCPServer(RC_ROVECOMM_MICROPIBOARD_PORT);
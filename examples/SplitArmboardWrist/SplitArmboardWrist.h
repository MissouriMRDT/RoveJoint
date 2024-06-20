#pragma once

#include "RoveComm.h"
#include "RoveJoint.h"

#define MotorPWM_1  PF_1
#define MotorPWM_2  PF_2

#define MotorINA_1  PL_0
#define MotorINA_2  PL_2

#define MotorINB_1  PL_1
#define MotorINB_2  PL_3

RoveJointDifferential Wrist;

RoveCommEthernet RoveComm;
rovecomm_packet packet;
RoveWatchdog Watchdog;
EthernetServer TCPServer(RC_ROVECOMM_MICROPIBOARD_PORT);
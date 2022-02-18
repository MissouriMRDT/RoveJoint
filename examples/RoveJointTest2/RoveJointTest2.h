#pragma once

#include "RoveComm.h"
#include "RoveJoint.h"
#include "RoveWatchdog"

//Motor PWM Pins
#define MotorPWM_1  PF_1
#define MotorPWM_2  PF_2
#define MotorPWM_3  PF_3
#define MotorPWM_4  PG_1
#define MotorPWM_5  PK_4
#define MotorPWM_6  PK_5

//Motor INA Pins
#define MotorINA_1  PC_6
#define MotorINA_2  PD_3
#define MotorINA_3  PB_2
#define MotorINA_4  PD_4
#define MotorINA_5  PQ_0
#define MotorINA_6  PN_5

//Motor INB Pins
#define MotorINB_1  PE_5
#define MotorINB_2  PC_7
#define MotorINB_3  PB_3
#define MotorINB_4  PG_1
#define MotorINB_5  PP_4
#define MotorINB_6  PN_4

//Joint Encoder Pins
#define Encoder_J1  PM_1
#define Encoder_J2  PD_2
#define Encoder_J3  PD_0
#define Encoder_J4  PD_1
#define Encoder_J5  PD_7
#define Encoder_J6  PM_4

//Limit Switch Pins
#define LimitSwitchLower_J1  PP_5
#define LimitSwitchUpper_J1  PA_7
#define LimitSwitchLower_J2  PQ_2
#define LimitSwitchUpper_J2  PQ_3
#define LimitSwitchLower_J3  PL_1
#define LimitSwitchUpper_J3  PL_3

RoveJoint J1, J2, J3, J4;
RoveJointDifferential Wrist;

RoveCommEthernet RoveComm;
RoveWatchdog Watchdog;
EthernetServer TCPServer(RC_ROVECOMM_ARMBOARD_PORT);

const uint16_t watchdogTimeout = 1500;

uint16_t jointAngles[6];

float jointTargets[6];

void parsePackets();
void updatePosition();
void closedLoop();
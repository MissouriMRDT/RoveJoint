#ifndef FOREARM_H
#define FOREARM_H

#include "RoveDifferentialJoint.h"
#include "RoveComm.h"

RoveDifferentialJoint Base;

RoveStmVnhPwm Gripper;

RoveCommEthernetUdp RoveComm;

struct rovecomm_packet rovecomm_packet;

const uint8_t BASE_LEFT_INA      = PL_0;
const uint8_t BASE_LEFT_INB      = PH_2;
const uint8_t BASE_LEFT_PWM      = PF_1;

const uint8_t BASE_RIGHT_INA     = PL_1;
const uint8_t BASE_RIGHT_INB     = PH_3;
const uint8_t BASE_RIGHT_PWM     = PF_2;

const uint8_t GRIPPER_INA        = PH_0;
const uint8_t GRIPPER_INB        = PP_5;
const uint8_t GRIPPER_PWM        = PG_1;

uint16_t disconnectCount = 0;


#endif

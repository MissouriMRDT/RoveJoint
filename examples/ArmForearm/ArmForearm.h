#ifndef FOREARM_H
#define FOREARM_H

#include "RoveDifferentialJoint.h"
#include "RoveComm.h"

RoveDifferentialJoint Base;

RoveStmVnhPwm Gripper;

RoveCommEthernetUdp RoveComm;

struct rovecomm_packet rovecomm_packet;

const uint8_t BASE_LEFT_INA      = PL_0;
<<<<<<< HEAD
<<<<<<< HEAD
const uint8_t BASE_LEFT_INB      = PL_1;
const uint8_t BASE_LEFT_PWM      = PF_1;

const uint8_t BASE_RIGHT_INA     = PL_2;
const uint8_t BASE_RIGHT_INB     = PL_3;
const uint8_t BASE_RIGHT_PWM     = PF_2;

const uint8_t GRIPPER_INA        = PQ_2;
const uint8_t GRIPPER_INB        = PQ_3;
const uint8_t GRIPPER_PWM        = PK_4;
=======
=======
>>>>>>> face3f141de0360c5160b71f2ba15fcf0535faf0
const uint8_t BASE_LEFT_INB      = PH_2;
const uint8_t BASE_LEFT_PWM      = PF_1;

const uint8_t BASE_RIGHT_INA     = PL_1;
const uint8_t BASE_RIGHT_INB     = PH_3;
const uint8_t BASE_RIGHT_PWM     = PF_2;

const uint8_t GRIPPER_INA        = PH_0;
const uint8_t GRIPPER_INB        = PP_5;
const uint8_t GRIPPER_PWM        = PG_1;
<<<<<<< HEAD
>>>>>>> face3f1... Initial code dump, needs restructuring
=======
>>>>>>> face3f141de0360c5160b71f2ba15fcf0535faf0

uint16_t disconnectCount = 0;


#endif

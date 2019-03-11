#ifndef BICEP_H
#define BICEP_H

#include "RoveDifferentialJoint.h"
#include "RoveComm.h"

RoveDifferentialJoint Shoulder;
RoveDifferentialJoint Elbow;

RoveCommEthernetUdp RoveComm;

RoveTimerInterrupt TelemetryTimer;

struct rovecomm_packet rovecomm_packet;

const uint8_t SHOULDER_RIGHT_ADC_PIN = INVALID;
const uint8_t SHOULDER_LEFT_ADC_PIN  = INVALID;

const uint8_t SHOULDER_LEFT_INA      = PL_0;
const uint8_t SHOULDER_LEFT_INB      = PH_2;
const uint8_t SHOULDER_LEFT_PWM      = PF_1;

const uint8_t SHOULDER_RIGHT_INA     = PL_1;
const uint8_t SHOULDER_RIGHT_INB     = PH_3;
const uint8_t SHOULDER_RIGHT_PWM     = PF_2;

const uint8_t ELBOW_RIGHT_ADC_PIN    = INVALID;
const uint8_t ELBOW_LEFT_ADC_PIN     = INVALID;

const uint8_t ELBOW_RIGHT_INA        = PH_0;
const uint8_t ELBOW_RIGHT_INB        = PP_5;
const uint8_t ELBOW_RIGHT_PWM        = PG_1;

const uint8_t ELBOW_LEFT_INA         = PH_1;
const uint8_t ELBOW_LEFT_INB         = PA_7;
const uint8_t ELBOW_LEFT_PWM         = PK_4;

uint16_t disconnectCount = 0;


#endif

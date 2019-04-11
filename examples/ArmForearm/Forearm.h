#ifndef FOREARM_H
#define FOREARM_H

#include "RoveDiffernetial.h"
#include "RoveComm.h"

RoveDifferential Wrist;

RoveStmVnhPwm Gripper;
RoveStmVnhPwm Poker;

RoveCommEthernetUdp RoveComm;

RoveTimerInterrupt TelemetryTimer;

struct rovecomm_packet rovecomm_packet;

void telemetryUpdate();

const uint8_t TELEMETRY_PIN = INVALID;

const uint8_t RIGHT_ADC_PIN = INVALID;
const uint8_t LEFT_ADC_PIN = INVALID;

const uint8_t WRIST_RIGHT_INA = INVALID;
const uint8_t WRIST_RIGHT_INB = INVALID;
const uint8_t WRIST_RIGHT_PWM = INVALID;

const uint8_t WRIST_LEFT_INA = INVALID;
const uint8_t WRIST_LEFT_INB = INVALID;
const uint8_t WRIST_LEFT_PWM = INVALID;

const uint8_t GRIPPER_INA = INVALID;
const uint8_t GRIPPER_INB = INVALID;
const uint8_t GRIPPER_PWM = INVALID;

const uint8_t POKER_INA = INVALID;
const uint8_t POKER_INB = INVALID;
const uint8_t POKER_PWM = INVALID;

#endif

#ifndef BICEP_H
#define BICEP_H

#include "RoveDifferentialJoint.h"
#include "RoveComm.h"


RoveCommEthernetUdp RoveComm;
struct rovecomm_packet rovecomm_packet;

RoveTimerInterrupt TelemetryTimer;



#endif

#ifndef BICEP_H
#define BICEP_H

#include "RoveDifferentialJoint.h"
#include "RoveComm.h"
#include "LocalManifest.h"

RoveCommEthernetUdp RoveComm;

RoveTimerInterrupt TelemetryTimer;

struct rovecomm_packet rovecomm_packet;

#endif

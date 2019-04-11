#ifndef BICEP_H
#define BICEP_H

#include "RoveDifferentialJoint.h"
#include "RoveComm.h"
<<<<<<< HEAD

RoveCommEthernetUdp RoveComm;
struct rovecomm_packet rovecomm_packet;

RoveTimerInterrupt TelemetryTimer;

=======
#include "LocalManifest.h"

RoveCommEthernetUdp RoveComm;

RoveTimerInterrupt TelemetryTimer;

struct rovecomm_packet rovecomm_packet;

>>>>>>> face3f1... Initial code dump, needs restructuring
#endif

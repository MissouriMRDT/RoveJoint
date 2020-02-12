///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT Differential Joint 2020
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ROVE_DIFF_BRUSHLESS
#define ROVE_DIFF_BRUSHLESS

#include "RovesODrive.h"
#include "RoveUsDigiMa3Pwm.h"
#include "RoveBoardMap.h"
#include "RoveWatchdog.h"
#include "RovePid.h"

#include <stdint.h>

class RoveDifferentialJoint
{
  public:

    RoveUsDigiMa3Pwm TiltEncoder;
    RoveUsDigiMa3Pwm TwistEncoder;

    RovePidFloats TiltPid;
    RovePidFloats TwistPid;

    uint8_t LS_UPPER = INVALID;
    uint8_t LS_LOWER = INVALID;

    int left_limit = 0;
    int right_limit = 0;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //Limit Switch Handling
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    void attachLimitSwitches(uint8_t upperPin, uint8_t lowerPin);
    //returns whether or not the Limit switch is pressed (if we are moving past that given limit)
    bool isLowerLSPressed();
    bool isUpperLSPressed();
    //sets angle limits to use as hardstops for movement on an axis without limit switches
    void setTwistLimits(int left_lim, int right_lim);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //Calculations
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    void tiltTwistDecipercent( int tilt_decipercent, int twist_decipercent, comp_side compensation = None, float comp_factor=1.0);
    bool atTiltLimit(int drive_speed);
    bool atTwistLimit(int drive_speed, uint32_t current_angle);
    //TODO: move_to_position wrapper based off of commanded positions and absolute/incremental encoder output

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //Encoder Handling
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    bool TwistEncoderDisconnect();
    bool TiltEncoderDisconnect();

};

#endif // ROVE_DIFF

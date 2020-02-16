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
#include "Energia.h"
#include "RoveComm.h"

#include <stdint.h>

//struct to return the appropriate errors/error types for Odrives
//check the mappings in RovesODrive.h
struct JointError
{
  Error_Type ErrorType;
  uint8_t    Error;

  //constructor
  JointError(Error_Type type, uint8_t odrive_error)
  {
    ErrorType = type;
    Error = odrive_error;
  }
};

class RoveDifferentialJointBrushless
{
  public:

    const int MAX_SPEED_REVERSE;
    const int MAX_SPEED_FORWARD;

    const int ENC_CPR = 8192; //Encoder counts per motor, should only change if we change encoder type
    const int GEAR_RATIO;

    RoveUsDigiMa3Pwm TiltEncoder;
    RoveUsDigiMa3Pwm TwistEncoder;

    RovePidFloats TiltPid;
    RovePidFloats TwistPid;

    RovesODrive Joint;

    uint8_t LS_UPPER = INVALID;
    uint8_t LS_LOWER = INVALID;

    int left_limit = 0;
    int right_limit = 0;

    
    //Constructor
    RoveDifferentialJointBrushless(HardwareSerial* odrive_serial, uint8_t tilt_encoder_pin, uint8_t twist_encoder_pin);

    //Limit Switch Handling
    void attachLimitSwitches(uint8_t upper_pin, uint8_t lower_pin);
    //returns whether or not the Limit switch is pressed (if we are moving past that given limit)
    bool isLowerLSPressed();
    bool isUpperLSPressed();
    //sets angle limits to use as hardstops for movement on an axis without limit switches
    void setTwistLimits(int left_lim, int right_lim);

    //Handle ODrive errors 
    JointError handleError();

    //Get absolute angles 
    float getTiltAngle();
    float getTwistAngle();

    //Calculations
    void tiltTwistDecipercent( int tilt_decipercent, int twist_decipercent);
    bool atTiltLimit(int drive_speed);
    bool atTwistLimit(int drive_speed, uint32_t current_angle);
    int getPositionCount(float angle);
    //TODO: move_to_position wrapper based off of commanded positions and absolute/incremental encoder output
    
    //Move the arm 
    void posMoveTilt(float tilt_angle, float tilt_velocity);
    void posMoveTwist(float twist_angle float twist_velocity);

    //Encoder Handling
    bool TwistEncoderDisconnect();
    bool TiltEncoderDisconnect();

};

#endif // ROVE_DIFF

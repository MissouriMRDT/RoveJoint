///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT Rove Joint 2022
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ROVE_JOINT
#define ROVE_JOINT

#include "RoveStmVnhPwm.h"
#include "RoveUsDigiMa3Pwm.h"
#include "RoveBoardMap.h"
#include "RoveWatchdog.h"
#include "RovePid.h"
#include "RovesODrive.h"

#include <stdint.h>

class RoveJoint
{
    public:

        RoveStmVnhPwm Motor_1;

        RoveUsDigiMa3Pwm Encoder_1;

        RovePidFloats Pid_1;

        uint8_t LS_1 = INVALID;
        uint8_t LS_2 = INVALID;

        uint16_t limit_1 = 0;
        uint16_t limit_2 = 0;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        //Limit Switch Handling
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        void attachLimitSwitches(uint8_t Pin1, uint8_t Pin2);
        //returns whether or not the Limit switch is pressed (if we are moving past that given limit)
        bool isLS1Pressed();
        bool isLS2Pressed();
        void setLimits(uint16_t lim1, uint16_t lim2);

};

class RoveJointBrushedDC : public RoveJoint
{
    public:

        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        //Calculations
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        void motorDecipercent( int16_t decipercent );
        bool atLimit(int16_t drive_speed);
        
};

class RoveJointDifferentialBrushedDC : public RoveJoint
{
    public:

        #define     RightMotor      Motor_1
        #define     TiltEncoder     Encoder_1
        #define     TiltPid         Pid_1
        #define     LS_UPPER        LS_1
        #define     LS_LOWER        LS_2
        #define     Left_Limit      limit_1
        #define     Right_Limit     limit_2

        RoveStmVnhPwm LeftMotor;
        RoveUsDigiMa3Pwm TwistEncoder;
        RovePidFloats TwistPid;


        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        //Limit Switch Handling
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        void attachLimitSwitches(uint8_t upperPin, uint8_t lowerPin);
        //returns whether or not the Limit switch is pressed (if we are moving past that given limit)
        bool isLowerLSPressed();
        bool isUpperLSPressed();
        void setTwistLimits(int16_t left_lim, int16_t right_lim);

        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        //Calculations
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        void tiltTwistDecipercent( int16_t tilt_decipercent, int16_t twist_decipercent);
        bool atTiltLimit(int16_t drive_speed);
        bool atTwistLimit(int16_t drive_speed, uint32_t current_angle);
        
};

//struct to return the appropriate errors/error types for Odrives
//check the mappings in RovesODrive.h
struct JointError
{
  Error_Types ErrorType;
  uint8_t    Error;

  //constructor
  JointError(Error_Types type, uint8_t odrive_error)
  {
    ErrorType = type;
    Error = odrive_error;
  }
};

class RoveJointDifferentialBrushless : public RoveJoint
{
  public:

    #define     RightMotor      Motor_1
    #define     TiltEncoder     Encoder_1
    #define     TiltPid         Pid_1
    #define     LS_UPPER        LS_1
    #define     LS_LOWER        LS_2
    #define     Left_Limit      limit_1
    #define     Right_Limit     limit_2

    const int MAX_SPEED_REVERSE;
    const int MAX_SPEED_FORWARD;

    const int ENC_CPR = 8192; //Encoder counts per motor, should only change if we change encoder type
    const float MAX_ENCODER_ANGLE = 360; //Change if we use radians or smth
    const int GEAR_RATIO;

    const int ANGLE_TO_ENC_COUNTS = ( (ENC_CPR * GEAR_RATIO) / (MAX_ENCODER_ANGLE));

    const float PID_TOLERANCE;

    RoveUsDigiMa3Pwm TwistEncoder;

    RovePidFloats TwistPid;

    RovesODrive Joint;

    float twistAngle = 0;
    float tiltAngle = 0;

    float leftLastKnownEncCount = 0;
    float rightLastKnownEncCount = 0;

    RoveJointDifferentialBrushless(int gear_ratio, int max_forward, int max_reverse, float PID_tolerance);

    //Attach Joint
    void attachJoint(HardwareSerial* odrive_serial, uint8_t tilt_encoder_pin, uint8_t twist_encoder_pin, 
                     float min_output_tilt, float max_output_tilt, float kp_tilt, float ki_tilt, float kd_tilt,
                     float min_output_twist, float max_output_twist, float kp_twist, float ki_twist, float kd_twist
                    );

    //Limit Switch Handling
    void attachLimitSwitches(uint8_t upper_pin, uint8_t lower_pin);
    //returns whether or not the Limit switch is pressed (if we are moving past that given limit)
    bool isLowerLSPressed();
    bool isUpperLSPressed();
    //sets angle limits to use as hardstops for movement on an axis without limit switches
    void setTwistLimits(int left_lim, int right_lim);

    //Set both motors into closed loop
    void setClosedLoop();

    //Use incremental encoders to get angle 
    void getIncrementedAngles(float incrementedAngles[2]);

    //Handle ODrive errors 
    JointError handleError();

    //Get angles from absolute encoders
    float getTiltAngleAbsolute();
    float getTwistAngleAbsolute();

    //Get the twist and tilt angles 
    float getTiltAngle();
    float getTwistAngle();

    void rehomePosition();

    //Calculations
    void tiltTwistDecipercent(int tilt_decipercent, int twist_decipercent);
    void moveToPos(float goalTilt, float goalTwist, float outputCounts[2]);

    //Encoder Handling
    bool TwistEncoderDisconnect();
    bool TiltEncoderDisconnect();

};

#endif // ROVE_JOINT

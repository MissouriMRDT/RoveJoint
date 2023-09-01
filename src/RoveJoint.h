///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT Rove Joint 2022
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ROVE_JOINT
#define ROVE_JOINT

#include "Energia.h"
#include <stdint.h>

#include "RoveStmVnhPwm.h"
#include "RoveUsDigiMa3Pwm.h"
#include "RoveBoardMap.h"
#include "RoveWatchdog.h"
#include "RovePid.h"

class RoveJoint
{
    public:

        RoveStmVnhPwm motor;

        RoveUsDigiMa3Pwm encoder;

        RovePidFloats pid;

        uint8_t limitSwitch_1 = INVALID;
        uint8_t limitSwitch_2 = INVALID;

        uint16_t angleLimit_1 = 0;
        uint16_t angleLimit_2 = 0;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        // Limit Switch Handling
        ///////////////////////////////////////////////////////////////////////////////////////////////////////

        void attachLimitSwitches(uint8_t pin1, uint8_t pin2);
        bool isLS1Pressed();
        bool isLS2Pressed();
        void setAngleLimits(float angle_1, float angle_2);
        bool atSoftLimit(float currentAngle, float goalAngle);
        bool atHardLimit(int16_t driveSpeed);

        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        // Motor Movement
        ///////////////////////////////////////////////////////////////////////////////////////////////////////

        void moveJoint( int16_t driveSpeed );
        void stopJoint( int16_t brakeSpeed );
};

class RoveJointDifferential : public RoveJoint
{
    public:

        #define     rightMotor              motor
        #define     tiltEncoder             encoder
        #define     tiltPid                 pid
        #define     limitSwitchLower        limitSwitch_1
        #define     limitSwitchUpper        limitSwitch_2
        #define     forwardTiltAngleLimit   angleLimit_1
        #define     backwardTiltAngleLimit  angleLimit_2

        RoveStmVnhPwm leftMotor;

        RoveUsDigiMa3Pwm twistEncoder;

        RovePidFloats twistPid;

        float leftTwistAngleLimit = 0;
        float rightTwistAngleLimit = 0;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        //Limit Switch Handling
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
      
        //returns whether or not the Limit switch is pressed (if we are moving past that given limit)
        bool isLowerLSPressed();
        bool isUpperLSPressed();
        void setTiltLimits(float lowerLimit, float upperLimit);
        void setTwistLimits(float leftLimit, float rightLimit);

        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        //Calculations
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        bool atTiltSoftLimit(float currentAngle, float goalAngle);
        bool atTiltHardLimit(int16_t driveSpeed);
        bool atTwistLimit(float currentAngle, float goalAngle);
        void moveDiffJoint( int16_t tiltSpeed, int16_t twistSpeed);
        void stopDiffJoint( int16_t tiltSpeed, int16_t twistSpeed);
        
};

#endif // ROVE_JOINT

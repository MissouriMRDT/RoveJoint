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

        uint16_t currentAngle = 0;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        // Limit Switch Handling
        ///////////////////////////////////////////////////////////////////////////////////////////////////////

        void attachLimitSwitches(uint8_t pin1, uint8_t pin2);
        bool isLS1Pressed();
        bool isLS2Pressed();
        void setAngleLimits(uint16_t angle_1, uint16_t angle_2);
        bool atSoftLimit(int16_t driveSpeed);
        bool atHardLimit(int16_t driveSpeed);

        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        // Motor Movement
        ///////////////////////////////////////////////////////////////////////////////////////////////////////

        void DriveMotor( int16_t driveSpeed );
};

class RoveJointDifferential : public RoveJoint
{
    public:

        #define     rightMotor              motor
        #define     tiltEncoder             encoder
        #define     currentTiltAngle        currentAngle
        #define     tiltPid                 pid
        #define     limitSwitchLower        limitSwitch_1
        #define     limitSwitchUpper        limitSwitch_2
        #define     forwardTiltAngleLimit   angleLimit_1
        #define     backwardTiltAngleLimit  angleLimit_2

        RoveStmVnhPwm leftMotor;

        RoveUsDigiMa3Pwm twistEncoder;

        RovePidFloats twistPid;

        uint16_t leftTwistAngleLimit = 0;
        uint16_t rightTwistAngleLimit = 0;

        uint16_t currentTwistAngle = 0;

        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        //Limit Switch Handling
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
      
        //returns whether or not the Limit switch is pressed (if we are moving past that given limit)
        bool isLowerLSPressed();
        bool isUpperLSPressed();
        void setTiltLimits(uint16_t lowerLimit, uint16_t upperLimit);
        void setTwistLimits(uint16_t leftLimit, uint16_t rightLimit);

        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        //Calculations
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        bool atTiltSoftLimit(int16_t driveSpeed);
        bool atTiltHardLimit(int16_t driveSpeed);
        bool atTwistLimit(int16_t driveSpeed);
        void tiltTwistDrive( int16_t tiltSpeed, int16_t twistSpeed);
        
};

#endif // ROVE_JOINT

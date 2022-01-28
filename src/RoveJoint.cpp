#include "RoveJoint.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////
// RoveJoint 2022
///////////////////////////////////////////////////////////////////////////////////////////////////////

//Intializes the limit switches.
void RoveJoint::attachLimitSwitches( uint8_t pin1, uint8_t pin2 )
{
    limitSwitch_1 = pin1;
    limitSwitch_2 = pin2;
}

//Returns whether or not the Limit switch is pressed (if we are moving past the given angle limit)
bool RoveJoint::isLS1Pressed()
{
    return (digitalRead(limitSwitch_1));
}

//Returns whether or not the Limit switch is pressed (if we are moving past the given angle limit)
bool RoveJoint::isLS2Pressed()
{
    return (digitalRead(limitSwitch_2));
}

//Sets the Soft Angle Limits to prevent the joint from hitting the limit switch.
void RoveJoint::setAngleLimits( uint16_t angle_1, uint16_t angle_2 )
{
    angleLimit_1 = angle_1;
    angleLimit_2 = angle_2;
}

//Returns whether or not the angle limits were reached
bool RoveJoint::atSoftLimit( int16_t driveSpeed )
{
    //if we are trying to move foward, and we are hitting the forward limit switch stop
    //the limit is hit if the switch is no longer being pressed
    if( driveSpeed > 0 && ( currentAngle <= angleLimit_1 && currentAngle > 180) )
    {
        return true;
    }
    //if we are trying to move backward, and we are hitting the backward limit switch stop
    //the limit is hit if the switch is no longer being pressed
    else if( driveSpeed < 0 && ( currentAngle >= angleLimit_2 && currentAngle < 180) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

//Returns whether or not we are trying to move the joint past the hard limits.
bool RoveJoint::atHardLimit( int16_t driveSpeed )
{
    //if we are trying to move foward, and we are hitting the forward limit switch stop
    //the limit is hit if the switch is no longer being pressed
    if( driveSpeed > 0 && !isLS2Pressed() )
    {
        return true;
    }
    //if we are trying to move backward, and we are hitting the backward limit switch stop
    //the limit is hit if the switch is no longer being pressed
    else if( driveSpeed < 0 && !isLS1Pressed() )
    {
        return true;
    }
    else
    {
        return false;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Drives Motor CW/CCW
///////////////////////////////////////////////////////////////////////////////////////////////////////
void RoveJoint::DriveMotor( int16_t driveSpeed )
{
    if ( atSoftLimit( driveSpeed ) )
    {
        driveSpeed = 0;
    }
    else if ( atHardLimit( driveSpeed ) )
    {
        driveSpeed = 0;
    }
    else if ( driveSpeed > 1000 )
    {
        driveSpeed = 1000;
    }
    else if ( driveSpeed < -1000 )
    {
        driveSpeed = -1000;
    }
    motor_1.drive( driveSpeed );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT RoveJointDifferential 2022
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/////Limit Switch initalization and accessor functions
//////////////////////////////////////////////////////////////////////////////

bool RoveJointDifferential::isLowerLSPressed()
{
    //HIGH or LOW, but we can just map to a boolean
    return isLS1Pressed();
}

bool RoveJointDifferential::isUpperLSPressed()
{
    //HIGH or LOW, but we can just map to a boolean
    return isLS2Pressed();
}

//////////////////////////////////////////////////////////////////////////////
//Sets Tilt soft angle limits.
/////////////////////////////////////////////////////////////////////////////
void RoveJointDifferential::setTiltLimits(uint16_t lowerLimit, uint16_t upperLimit)
{
    setAngleLimits(lowerLimit, upperLimit);
}

//////////////////////////////////////////////////////////////////////////////
//Sets Twist soft angle limits.
/////////////////////////////////////////////////////////////////////////////
void RoveJointDifferential::setTwistLimits(uint16_t leftLimit, uint16_t rightLimit)
{
    leftTwistAngleLimit = leftLimit;
    rightTwistAngleLimit = rightLimit;
}

//////////////////////////////////////////////////////////////////////////////
//Returns whether we are moving past our limit switches
//////////////////////////////////////////////////////////////////////////////
bool RoveJointDifferential::atTiltHardLimit(int16_t driveSpeed)
{
    atHardLimit(driveSpeed);
}

//////////////////////////////////////////////////////////////////////////////
//Returns whether we are moving past our angle limits
//////////////////////////////////////////////////////////////////////////////
bool RoveJointDifferential::atTiltSoftLimit(int16_t driveSpeed)
{
    atSoftLimit(driveSpeed);
}

//////////////////////////////////////////////////////////////////////////////
//Returns whether we are moving past our angle limits
//////////////////////////////////////////////////////////////////////////////
bool RoveJointDifferential::atTwistLimit(int16_t driveSpeed)
{

  //if we are driving to the left, and we are past the limits
  if(driveSpeed < 0 && (currentTwistAngle <= leftTwistAngleLimit && currentTwistAngle > 180000))
  {
    return true;
  }
  //if we are driving to the right, and we are past the limits
  else if(driveSpeed > 0 && (currentTwistAngle >= rightTwistAngleLimit && currentTwistAngle < 180000))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void RoveJointDifferential::tiltTwistDrive( int16_t tiltSpeed, int16_t twistSpeed)
{
    int16_t left_speed  = tiltSpeed - twistSpeed;
    int16_t right_speed = tiltSpeed + twistSpeed;

    if ( atTiltSoftLimit(tiltSpeed) || atTwistLimit(tiltSpeed) || atTiltHardLimit(twistSpeed) )
    {
        left_speed = 0;
        right_speed = 0;
    }
    else if(left_speed > 1000)
    {
        right_speed = right_speed-(left_speed-1000);
        left_speed = 1000;
    }
    else if(left_speed < - 1000)
    {
        right_speed = right_speed+(abs(left_speed)-1000);
        left_speed = -1000;
    }
    else if(right_speed > 1000)
    {
        left_speed = left_speed-(right_speed-1000);
        right_speed = 1000;
    }
    else if(right_speed < - 1000)
    {
        left_speed = left_speed+(abs(right_speed)-1000);
        right_speed = -1000;
    }

    rightMotor.drive(right_speed);
    LeftMotor.drive(left_speed);
}
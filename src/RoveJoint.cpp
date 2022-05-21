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
void RoveJoint::setAngleLimits( float angle_1, float angle_2 )
{
    angleLimit_1 = angle_1;
    angleLimit_2 = angle_2;
}

//Returns whether or not the angle limits were reached
bool RoveJoint::atSoftLimit( float currentAngle, float goalAngle)
{
    //if we are trying to move foward, and we are hitting the forward limit switch stop
    //the limit is hit if the switch is no longer being pressed
    if( ( currentAngle >= angleLimit_2 && goalAngle > angleLimit_2 ) )
    {
        return true;
    }
    //if we are trying to move backward, and we are hitting the backward limit switch stop
    //the limit is hit if the switch is no longer being pressed
    else if( ( currentAngle <= angleLimit_1 && goalAngle < angleLimit_1 ) )
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
    if( driveSpeed > 0 && isLS2Pressed() )
    {
        return true;
    }
    //if we are trying to move backward, and we are hitting the backward limit switch stop
    //the limit is hit if the switch is no longer being pressed
    else if( driveSpeed < 0 && isLS1Pressed() )
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
void RoveJoint::moveJoint( int16_t driveSpeed )
{
    if ( driveSpeed > 1000 )
    {
        driveSpeed = 1000;
    }
    else if ( driveSpeed < -1000 )
    {
        driveSpeed = -1000;
    }
    motor.drive( driveSpeed );
}

void RoveJoint::stopJoint( int16_t brakeSpeed )
{
    if ( brakeSpeed > 1000 )
    {
        brakeSpeed = 1000;
    }
    else if ( brakeSpeed < -1000 )
    {
        brakeSpeed = -1000;
    }
    motor.hardBrake( brakeSpeed );
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
    return (digitalRead(limitSwitchUpper));
}

//////////////////////////////////////////////////////////////////////////////
//Sets Tilt soft angle limits.
/////////////////////////////////////////////////////////////////////////////
void RoveJointDifferential::setTiltLimits(float lowerLimit, float upperLimit)
{
    setAngleLimits(lowerLimit, upperLimit);
}

//////////////////////////////////////////////////////////////////////////////
//Sets Twist soft angle limits.
/////////////////////////////////////////////////////////////////////////////
void RoveJointDifferential::setTwistLimits(float leftLimit, float rightLimit)
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
bool RoveJointDifferential::atTiltSoftLimit(float currentAngle, float goalAngle)
{
    atSoftLimit( currentAngle,  goalAngle);
}

//////////////////////////////////////////////////////////////////////////////
//Returns whether we are moving past our angle limits
//////////////////////////////////////////////////////////////////////////////
bool RoveJointDifferential::atTwistLimit(float currentAngle, float goalAngle)
{

  //if we are driving to the left, and we are past the limits
  if( ( currentAngle <= leftTwistAngleLimit && goalAngle > leftTwistAngleLimit ))
  {
    return true;
  }
  //if we are driving to the right, and we are past the limits
  else if( ( currentAngle >= rightTwistAngleLimit && goalAngle < rightTwistAngleLimit ))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void RoveJointDifferential::moveDiffJoint( int16_t tiltSpeed, int16_t twistSpeed)
{
    int16_t left_speed  = tiltSpeed - twistSpeed;
    int16_t right_speed = tiltSpeed + twistSpeed;

    if(left_speed > 1000)
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
    leftMotor.drive(left_speed);
}

void RoveJointDifferential::stopDiffJoint( int16_t tiltSpeed, int16_t twistSpeed)
{
    int16_t left_speed  = tiltSpeed - twistSpeed;
    int16_t right_speed = tiltSpeed + twistSpeed;

    if(left_speed > 1000)
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

    rightMotor.hardBrake(right_speed);
    leftMotor.hardBrake(left_speed);
}

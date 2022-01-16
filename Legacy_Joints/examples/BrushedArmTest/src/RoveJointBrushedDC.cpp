///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT Differential Joint 2022
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "RoveStmVnhPwm.h"
#include "RoveUsDigiMa3Pwm.h"
#include "RoveBoardMap.h"
#include "Energia.h"
#include <stdint.h>
#include "RoveJointBrushedDC.h"

void RoveJointBrushed::attachJoint(uint8_t encoder_pin)
{
    Encoder.attach(encoder_pin);
}
//////////////////////////////////////////////////////////////////////////////
/////Limit Switch initalization and accessor functions
//////////////////////////////////////////////////////////////////////////////
void RoveJointBrushed::attachLimitSwitches(uint8_t upperPin, uint8_t lowerPin)
{
    LS_UPPER = upperPin;
    LS_LOWER = lowerPin;
}

bool RoveJointBrushed::isLowerLSPressed()
{
  //HIGH or LOW, but we can just map to a boolean
  return (digitalRead(LS_LOWER));
}

bool RoveJointBrushed::isUpperLSPressed()
{
  //HIGH or LOW, but we can just map to a boolean
  return (digitalRead(LS_UPPER));
}

//////////////////////////////////////////////////////////////////////////////
//Since we do not have limit switches for twist on the 2019 Valkyrie arm
//we will instead set angle limits.
/////////////////////////////////////////////////////////////////////////////
void RoveJointBrushed::setJointLimits(int up_lim, int low_lim)
{
  up_limit = up_lim;
  low_limit = low_lim;
}

//////////////////////////////////////////////////////////////////////////////
//Scale our motor speeds so we can do a simultaneous twist and tilt
//////////////////////////////////////////////////////////////////////////////
void RoveJointBrushed::jointDecipercent( int joint_decipercent)
{
  int drive_speed  = joint_decipercent;

	if(drive_speed > 1000)
	{
	  drive_speed = 1000;
	}
	else if(drive_speed < - 1000)
	{
	  drive_speed = -1000;
	}

  Motor.drive(drive_speed);
}

//////////////////////////////////////////////////////////////////////////////
//Returns whether we are moving past our limit switches
//////////////////////////////////////////////////////////////////////////////
bool RoveJointBrushed::atJointLimit(int drive_speed)
{
  //if we are trying to move downwards, and we are hitting the lower limit switch stop
  //the limit is hit if the switch is no longer being pressed
  if(drive_speed > 0 && !isLowerLSPressed())
  {
    return true;
  }
  //if we are trying to move upwards, and we are hitting the upper limit switch stop
  //the limit is hit if the switch is no longer being pressed
  else if(drive_speed < 0 && !isUpperLSPressed())
  {
    return true;
  }
  else
  {
    return false;
  }
}


float RoveJointBrushed::getJointAngle()
{
    return Encoder.readDegrees();
}

void RoveJointBrushed::moveToPos(float goal)
{
    float curr = getJointAngle();
    while(abs(goal - curr) > MAX_POS_ERROR)
    {
      if(goal > curr)
      {
        if(abs(goal - curr) < 180) //Clockwise is the shorter path
        {
          Motor.drive(100);
        }
        else //Counterclockwise is the shorter path
        {
          Motor.drive(-100);
        }
      }
      else //The direction is reversed if the goal is on the other side of the current pos.
      {
        if(abs(goal - curr) < 180) //Counterclockwise is the shorter path
        {
          Motor.drive(-100);
        }
        else //Clockwise is the shorter path
        {
          Motor.drive(100);
        }
      }
      curr = getJointAngle(); //Update the current position
    }
    Motor.drive(0); //Finished moving.
    return;
}

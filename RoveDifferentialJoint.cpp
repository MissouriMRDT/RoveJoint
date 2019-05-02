///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT Differential Joint 2019
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "RoveStmVnhPwm.h"
#include "RoveUsDigiMa3Pwm.h"
#include "RoveBoardMap.h"
#include "Energia.h"
#include <stdint.h>
#include "RoveDifferentialJoint.h"


//////////////////////////////////////////////////////////////////////////////
/////Scale our motor speeds so we can do a simultaneous twist and tilt////////
//////////////////////////////////////////////////////////////////////////////
void RoveDifferentialJoint::tiltTwistDecipercent( int tilt_decipercent, int twist_decipercent, comp_side compensation, float comp_factor)
{
  int left_speed  = tilt_decipercent - twist_decipercent;
	int right_speed = tilt_decipercent + twist_decipercent;

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
  
  //compensation of motors, added because belt tensioning was unequal
  if(compensation == Right)
  {
    right_speed = (right_speed*comp_factor);
  }
  if(compensation == Left)
  {
    left_speed = (left_speed*comp_factor);
  }
  RightMotor.drive(right_speed);
  LeftMotor.drive(left_speed);
}

void RoveDifferentialJoint::LowerLSPressed()
{
  //HIGH or LOW, but we can just map to a boolean
  return digitalRead(LS_LOWER);
}

void RoveDifferentialJoint::UpperLSPressed()
{
  //HIGH or LOW, but we can just map to a boolean
  return digitalRead(LS_UPPER);
}

bool RoveDifferentialJoint::atTiltLimit(int drive_speed)
{
  //if we are trying to move downwards, and we are hitting the lower limit switch stop
  if(drive_speed < 0 && LowerLSPressed())
  {
    return true;
  }
  //if we are trying to move upwards, and we are hitting the upper limit switch stop
  else if(drive_speed > 0 && UpperLSPressed())
  {
    return true;
  }
  else
  {
    return false;
  }
}

void RoveDifferentialJoint::attachLimitSwitches(uint8_t upperPin, uint8_t lowerPin)
{
    uint8_t LS_UPPER = upperPin;
    uint8_t LS_LOWER = lowerPin;
}
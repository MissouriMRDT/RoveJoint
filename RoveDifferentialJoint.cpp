///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => Energia Texas Instruments Tiva C, Todo
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
void RoveDifferentialJoint::tiltTwistDecipercent( int tilt_decipercent, int twist_decipercent )
{
  int left_speed  = tilt_decipercent - twist_decipercent/2;
	int right_speed = tilt_decipercent + twist_decipercent/2;

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


  Serial.print("Left speed:");
	Serial.println(left_speed);
  Serial.print("Right speed:");
	Serial.println(right_speed);

  RightMotor.drive(right_speed);
  LeftMotor.drive(left_speed);
}

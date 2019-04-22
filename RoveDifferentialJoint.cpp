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
void RoveDifferentialJoint::tiltTwistDecipercent( int tilt_decipercent, int twist_decipercent, bool right_compensation, bool left_compensation)
{
  int left_tilt  = tilt_decipercent; // motors move in same direction to tilt on the pitch axis
  int right_tilt = tilt_decipercent;

  int left_twist   = -twist_decipercent; // motors move in opposite direction to twist on the roll axis
  int right_twist  =  twist_decipercent;

  int left_speed  =( ( left_tilt + left_twist ) / 2 ) ;// combined differenterial
  int right_speed = ( ( right_tilt + right_twist ) / 2 );
  
  if(right_compensation == true)
  {
    right_speed = (right_speed*190)/100;
  }
  if(left_compensation == true)
  {
    left_speed = (left_speed*190)/100;
  }
  RightMotor.drive(right_speed);
  LeftMotor.drive(left_speed);
}
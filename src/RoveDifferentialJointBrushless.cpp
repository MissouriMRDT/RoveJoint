///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT Differential Joint 2020
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "RoveDifferentialJointBrushless.h"

//////////////////////////////////////////////////////////////////////////////
/////Setups the Joint with necessary pins and constants
//////////////////////////////////////////////////////////////////////////////
RoveDifferentialJointBrushless::RoveDifferentialJointBrushless(HardwareSerial* odrive_serial, uint8_t tilt_encoder_pin, uint8_t twist_encoder_pin, int max_forward, int max_reverse) :
                                                               MAX_SPEED_FORWARD(max_forward), MAX_SPEED_REVERSE(max_reverse)
{
  //attach the encoder pins
  TiltEncoder.attach(tilt_encoder_pin);
  TwistEncoder.attach(twist_encoder_pin);

  //start the odrive serial communication
  Joint.begin(odrive_serial);
  delay(100);

  //set the odrive state to closed loop control
  Joint.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);
  delay(100);  
}

//////////////////////////////////////////////////////////////////////////////
/////Handle various ODrive errors, and report them
//////////////////////////////////////////////////////////////////////////////
RoveDifferentialJointBrushless::handleError()
{
  int motorError = Joint.checkMotorErrors();
  if(motorError)
  {
    Serial.println("Motor Error:");
    Serial.println(motorError);
  }
  
  int encoderError = Joint.checkEncoderErrors();
  if(encoderError)
  {
    Serial.println("Encoder Error:");
    Serial.println(encoderError);
  }

  int axisError = Joint.checkAxisErrors();
  if(axisError))
  {
    Serial.println("Axis Error:");
    Serial.println(axisError);
  }

  int controllerError = Joint.checkControllerErrors();
  if(controllerError)
  {
    Serial.println("Controller Error:");
    Serial.println(controllerError);
  }
}

//////////////////////////////////////////////////////////////////////////////
/////Limit Switch initalization and accessor functions
//////////////////////////////////////////////////////////////////////////////
void RoveDifferentialJointBrushless::attachLimitSwitches(uint8_t upperPin, uint8_t lowerPin)
{
    LS_UPPER = upperPin;
    LS_LOWER = lowerPin;
}

bool RoveDifferentialJointBrushless::isLowerLSPressed()
{
  //HIGH or LOW, but we can just map to a boolean
  return (digitalRead(LS_LOWER));
}

bool RoveDifferentialJointBrushless::isUpperLSPressed()
{
  //HIGH or LOW, but we can just map to a boolean
  return (digitalRead(LS_UPPER));
}

//////////////////////////////////////////////////////////////////////////////
//Since we do not have limit switches for twist on the 2020 Icarus arm
//we will instead set angle limits.
/////////////////////////////////////////////////////////////////////////////
void RoveDifferentialJointBrushless::setTwistLimits(int left_lim, int right_lim)
{
  left_limit = left_lim;
  right_limit = right_lim;
}

//////////////////////////////////////////////////////////////////////////////
//Scale our motor speeds so we can do a simultaneous twist and tilt
//////////////////////////////////////////////////////////////////////////////
void RoveDifferentialJointBrushless::tiltTwistDecipercent( int tilt_decipercent, int twist_decipercent, comp_side compensation, float comp_factor)
{
  int left_speed = tilt_decipercent + twist_decipercent;
  int right_speed = tilt_decipercent - twist_decipercent;

  //adjust the speeds to scale between -1000 and 1000
  if(left_speed > 1000)
  {
      left_speed /= 2;
      if(right_speed!=0)
      {
          right_speed/=2;
      }
  }
  else if(left_speed < -1000)
  {
      left_speed/=2;
      if(right_speed!=0)
      {
          right_speed/=2;
      }
  }
  else if(right_speed > 1000)
  {
      right_speed/=2;
      if(left_speed!=0)
      {
          left_speed/=2;
      }
  }
  else if(right_speed < -1000)
  {
      right_speed/=2;
      if(left_speed!=0)
      {
          left_speed/=2;
      }
  }
  
  //map the speed to the encoder counts/s the ODrives expect
  right_speed = map(right_speed, -1000, 1000, MAX_SPEED_REVERSE, MAX_SPEED_FORWARD);
  right_speed = map(right_speed, -1000, 1000, MAX_SPEED_REVERSE, MAX_SPEED_FORWARD);

  //write the speed set point to the motors
  Joint.left.writeVelocitySetpoint(right_speed, 0);
  Joint.right.writeVelocitySetpoint(left_speed, 0);
}

//////////////////////////////////////////////////////////////////////////////
//Returns whether we are moving past our limit switches
//////////////////////////////////////////////////////////////////////////////
bool RoveDifferentialJointBrushless::atTiltLimit(int drive_speed)
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

//////////////////////////////////////////////////////////////////////////////
//Returns whether we are moving past our angle limits
//////////////////////////////////////////////////////////////////////////////
bool RoveDifferentialJointBrushless::atTwistLimit(int drive_speed, uint32_t current_angle)
{

  //if we are driving to the left, and we are past the limits
  if(drive_speed < 0 && (current_angle <= left_limit && current_angle > 180000))
  {
    return true;
  }
  //if we are driving to the right, and we are past the limits
  else if(drive_speed > 0 && (current_angle >= right_limit && current_angle < 180000))
  {
    return true;
  }
  else
  {
    return false;
  }
}


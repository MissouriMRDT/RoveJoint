//MRDT Differential Joint 2020
#include "RoveDifferentialJointBrushless.h"

//Setups the Joint with necessary pins and constants
RoveDifferentialJointBrushless::RoveDifferentialJointBrushless(HardwareSerial* odrive_serial, uint8_t tilt_encoder_pin, uint8_t twist_encoder_pin, int gear_ratio, int max_forward, int max_reverse) :
                                                               GEAR_RATIO(gear_ratio), MAX_SPEED_FORWARD(max_forward), MAX_SPEED_REVERSE(max_reverse)
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

//Handle various ODrive errors, and report them
JointError RoveDifferentialJointBrushless::handleError()
{
  uint8_t motorError = Joint.checkMotorErrors();
  if(motorError)
  {
    Serial.println("Motor Error:");
    Serial.println(motorError);
    return JointError(ERROR_MOTOR, motorError);
  }
  
  uint8_t encoderError = Joint.checkEncoderErrors();
  if(encoderError)
  {
    Serial.println("Encoder Error:");
    Serial.println(encoderError);
    return JointError(ERROR_ENCODER, encoderError);
  }

  uint8_t axisError = Joint.checkAxisErrors();
  if(axisError))
  {
    Serial.println("Axis Error:");
    Serial.println(axisError);
    return JointError(ERROR_AXIS, axisError);
  }

  uint8_t controllerError = Joint.checkControllerErrors();
  if(controllerError)
  {
    Serial.println("Controller Error:");
    Serial.println(controllerError);
    return JointError(ERROR_CONTROLLER, controllerError);
  }
}

//Get absolute angles 
float getTiltAngle()
{
  return TiltEncoder.readDegrees();
}

float getTwistAngle()
{
  return TwistEncoder.readDegrees();
}

//Limit Switch initalization and accessor functions
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

//Since we do not have limit switches for twist on the 2020 Icarus arm
//we will instead set angle limits.
void RoveDifferentialJointBrushless::setTwistLimits(int left_lim, int right_lim)
{
  left_limit = left_lim;
  right_limit = right_lim;
}

//Scale our motor speeds so we can do a simultaneous twist and tilt
JointError RoveDifferentialJointBrushless::tiltTwistDecipercent( int tilt_decipercent, int twist_decipercent, comp_side compensation, float comp_factor)
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
  return handleError();
}

//Returns whether we are moving past our limit switches
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

//Returns whether we are moving past our angle limits
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

//Calculate position value from given angle
int getPositionCount(float angle) 
{
  return(abs( (angle*ENC_CPR) / (gear_ratio*2*pi()) ));
}



//Move the arm based on position control from odrives
void posMoveTilt(float tilt_angle_relative, float tilt_velocity)
{
  int positionCount = getPositionCount(tilt_angle);
  //Tilt, so both motors should go same direction
  if(tilt_angle < 0) 
  {
    tilt_velocity *= -1;
  }
  joint.left.writePosSetPoint(positionCount, tilt_velocity, 0);
  joint.right.writePosSetPoint(positionCount, tilt_velocity, 0);
}

void posMoveTwist(float twist_angle_relative, float twist_velocity)
{
  oppositeVelocity = twist_velocity * -1;
  int positionCount = getPositionCount(twist_angle);
  if(twist_angle < 0) 
  {
    joint.left.writePosSetPoint(positionCount, oppositeVelocity, 0);
    joint.right.writePosSetPoint(positionCount, twist_velocity, 0);
  }
  else {
    joint.left.writePosSetPoint(positionCount, twist_velocity, 0);
    joint.right.writePosSetPoint(positionCount, oppositeVelocity, 0);
  }
}

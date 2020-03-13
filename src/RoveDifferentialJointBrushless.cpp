//MRDT Differential Joint 2020
#include "RoveDifferentialJointBrushless.h"

RoveDifferentialJointBrushless::RoveDifferentialJointBrushless(int gear_ratio, int max_forward, int max_reverse, float PID_tolerance):
  GEAR_RATIO(gear_ratio), MAX_SPEED_FORWARD(max_forward), MAX_SPEED_REVERSE(max_reverse), PID_TOLERANCE(PID_tolerance) {}

//Setups the Joint with necessary pins and constants
void RoveDifferentialJointBrushless::attachJoint(HardwareSerial* odrive_serial, uint8_t tilt_encoder_pin, uint8_t twist_encoder_pin, 
                 float min_output_tilt, float max_output_tilt, float kp_tilt, float ki_tilt, float kd_tilt,
                 float min_output_twist, float max_output_twist, float kp_twist, float ki_twist, float kd_twist
                )
{
  
  //attach the encoder pins
  TiltEncoder.attach(tilt_encoder_pin);
  TwistEncoder.attach(twist_encoder_pin);

  //start the odrive serial communication
  Joint.begin(odrive_serial);

  TiltPid.attach(min_output_tilt, max_output_tilt, kp_tilt, ki_tilt, kd_tilt);
  TwistPid.attach(min_output_twist, max_output_twist, kp_twist, ki_twist, kd_twist);

  tiltAngle = getTiltAngleAbsolute();
  twistAngle = getTwistAngleAbsolute();

}

//Handle various ODrive errors, and report them
JointError RoveDifferentialJointBrushless::handleError()
{
  uint8_t motorErrorR = Joint.right.checkMotorErrors();
  if(motorErrorR)
  {
    Serial.println("Right Motor Error:");
    Serial.println(motorErrorR);
    return JointError(ERROR_MOTOR, motorErrorR);
  }

  uint8_t motorErrorL = Joint.left.checkMotorErrors();
  if(motorErrorL)
  {
    Serial.println("Left Motor Error:");
    Serial.println(motorErrorL);
    return JointError(ERROR_MOTOR, motorErrorL);
  }
  
  uint8_t encoderErrorR = Joint.right.checkEncoderErrors();
  if(encoderErrorR)
  {
    Serial.println("Right Encoder Error:");
    Serial.println(encoderErrorR);
    return JointError(ERROR_ENCODER, encoderErrorR);
  }

  uint8_t encoderErrorL = Joint.left.checkEncoderErrors();
  if(encoderErrorL)
  {
    Serial.println("Left Encoder Error:");
    Serial.println(encoderErrorL);
    return JointError(ERROR_ENCODER, encoderErrorL);
  }

  uint8_t axisErrorR = Joint.right.checkAxisErrors();
  if(axisErrorR)
  {
    Serial.println("Right Axis Error:");
    Serial.println(axisErrorR);
    return JointError(ERROR_AXIS, axisErrorR);
  }

  uint8_t axisErrorL = Joint.left.checkAxisErrors();
  if(axisErrorL)
  {
    Serial.println("Left Axis Error:");
    Serial.println(axisErrorL);
    return JointError(ERROR_AXIS, axisErrorL);
  }

  uint8_t controllerErrorR = Joint.right.checkControllerErrors();
  if(controllerErrorR)
  {
    Serial.println("Right Controller Error:");
    Serial.println(controllerErrorR);
    return JointError(ERROR_CONTROLLER, controllerErrorR);
  }

  uint8_t controllerErrorL = Joint.left.checkControllerErrors();
  if(controllerErrorL)
  {
    Serial.println("Left Controller Error:");
    Serial.println(controllerErrorL);
    return JointError(ERROR_CONTROLLER, controllerErrorL);
  }
}

//Get absolute angles 
float RoveDifferentialJointBrushless::getTiltAngleAbsolute()
{
  return TiltEncoder.readDegrees();
}

float RoveDifferentialJointBrushless::getTwistAngleAbsolute()
{
  return TwistEncoder.readDegrees();
}

float RoveDifferentialJointBrushless::getTiltAngle() 
{
  return tiltAngle;
}

float RoveDifferentialJointBrushless::getTwistAngle()
{
  return twistAngle;
}

void RoveDifferentialJointBrushless::getIncrementedAngles(float incrementedAngles[2])
{
  //0 is tilt, 1 is twist
  float leftEncCountsDelta, rightEncCountsDelta;
  float leftEncCounts = Joint.left.readPosEstimate();
  float rightEncCounts = Joint.right.readPosEstimate();

  //To get the relative angle moved we first find the change in the encoder counts  
  leftEncCountsDelta = abs(leftEncCounts - leftEncCountsSetpoint);
  rightEncCountsDelta = abs(rightEncCounts - rightEncCountsSetpoint);

  //Next we apply a ratio to convert from left and right counts to tilt and twist angles
  incrementedAngles[0] = ( ( (leftEncCountsDelta + rightEncCountsDelta) / 2) / ANGLE_TO_ENC_COUNTS);
  incrementedAngles[1] = ( ( (leftEncCountsDelta - rightEncCountsDelta) / 2) / ANGLE_TO_ENC_COUNTS);

  //Now we add the relative angles to the previous angles to get our new value
  incrementedAngles[0] += tiltAngle;
  incrementedAngles[1] += twistAngle;
 
  //Last the new encoder positions are moved to be the current setpoints
  leftEncCountsSetpoint = leftEncCounts;
  rightEncCountsSetpoint = rightEncCounts;
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
void RoveDifferentialJointBrushless::tiltTwistDecipercent(int tilt_decipercent, int twist_decipercent)
{
  int left_speed = (tilt_decipercent + twist_decipercent);
  int right_speed = (tilt_decipercent - twist_decipercent);

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
  left_speed = map(left_speed, -1000, 1000, MAX_SPEED_REVERSE, MAX_SPEED_FORWARD);
  
  Serial.println(right_speed);
  Serial.println(left_speed);
  //write the speed set point to the motors
  Joint.left.writeVelocitySetpoint(right_speed, 0);
  Joint.right.writeVelocitySetpoint(left_speed, 0);
  //return handleError();
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

void RoveDifferentialJointBrushless::moveToPos(float goalTiltAngle, float goalTwistAngle, float outputAngles[2])
{
  float goalAngles[2] = {goalTiltAngle, goalTwistAngle};
  float currentAngles[2] = {};
  float smallerAngle, largerAngle;
  float clockWiseAngle, counterClockWiseAngle;

  getIncrementedAngles(currentAngles);

  //The goal of this program is to find the fastest path to our goal. Due
  //to stupidity, the ik software gets weird around the 0 angle. So we check 
  //if going clockwise is quicker than counterclockwise or vise versa. 

  //Do tilt first, then calculate twist
  for(int i = 0; i <= 1; i++) 
  {
    smallerAngle = min(currentAngles[i], goalAngles[i]);
    largerAngle = max(currentAngles[i], goalAngles[i]);

    clockWiseAngle = (largerAngle - smallerAngle);
    counterClockWiseAngle = (smallerAngle - largerAngle);

    if(clockWiseAngle > abs(counterClockWiseAngle))
    {
      outputAngles[i] = counterClockWiseAngle;
    }
    else 
    {
      outputAngles[i] = clockWiseAngle;
    }
      outputAngles[i] *= ANGLE_TO_ENC_COUNTS;
  }
}

//Calculate position value from given angle
int RoveDifferentialJointBrushless::getPositionCount(float angle) 
{
  return(abs( (angle*ENC_CPR) / (GEAR_RATIO*2*M_PI )) );
}
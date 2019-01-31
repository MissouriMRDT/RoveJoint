///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => Energia Texas Instruments Tiva C, Todo
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ROVE_DIFF_STM_VNH_US_DIGI_MA3_H
#define ROVE_DIFF_STM_VNH_US_DIGI_MA3_H

#include "RoveStmVnhPwm.h"
#include "RoveUsDigiMa3Pwm.h"
#include "RoveBoardMap.h"

#include <stdint.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Given a RoveJoint instance
// RoveStmVnhUsDigiMa3    RoveJoint;

// RoveJoint.attachStarboardMotor( PIN_x1, PIN_x2, PIN_x3, PIN_x4, bool invert_motor, int drive_decipercent );
// RoveJoint.attachPortMotor(      PIN_x1, PIN_x2, PIN_x3, PIN_x4, bool invert_motor, int drive_decipercent );

// pay attention to wiring polarity and choose port/starboard invert motors and for differential gearing
// scale the drive command pwm output for 0%*DcBus ~ 100.0%*DcBus in decipercent int 0~1000 => 0.0~100.0

// Call attachStarboardMotorStmVnh and attachPortMotorStmVnh and invert one of the two for differenterial mode

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class RoveDiffStmVnhUsDigiMa3
{
public:

  RoveStmVnhPwm      StarboardMotor; // Todo? maybe these guys are the abstract/inherit based on peripheral type (PWM vs I2C vs ANALOG?)
  RoveStmVnhPwm      PortMotor;

  RoveUsDigiMa3Pwm   PitchEncoder;
  RoveUsDigiMa3Pwm   RollEncoder;

//RovePid            PitchPid;
//RovePid            RollPid;

  int  pitch_ccw_digital_limit_pin  = INVALID; // optional pin isDrivingPastLimits 
  int  pitch_cw_digital_limit_pin   = INVALID; // optional pin isDrivingPastLimits
  int  roll_ccw_digital_limit_pin   = INVALID; // optional pin isDrivingPastLimits 
  int  roll_cw_digital_limit_pin    = INVALID; // optional pin isDrivingPastLimits
  bool pitch_digital_limits_nc      = true;
  bool roll_digital_limits_nc       = true;

  int  pitch_ccw_limit_millidegrees = 360000;  // one full revolution in millidegrees 
  int  pitch_cw_limit_millidegrees  = 0;       // 0~360.00 degrees => 0~360000 millidegrees
  int  roll_ccw_limit_millidegrees  = 360000;  // one full revolution in millidegrees 
  int  roll_cw_limit_millidegrees   = 0;       // 0~360.00 degrees => 0~360000 millidegrees

  int  current_limits_milliamps     = 20000;   // 20amps in milliamps => 20000 milliamps
//int  pid_setpoint                 = 0;
//int  cmd                          = MOTION::NO_CMD;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void  attachStarboardMotor(     uint8_t ina_pin,
                                  uint8_t inb_pin,
                                  uint8_t pwm_pin,
                                  bool    invert_motor         = false,
                                  int     bus_millivolts       = 12000,   // 12V bus
                                  int     scale_pwm_millivolts = 12000,   // scale_pwm_millivolts / bus_millivolts => 100.0% scale
                                  uint8_t adc_pin              = INVALID,
                                  int     scale_adc_milliamps  = 20000 ); // 20A max

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void  attachPortMotor(          uint8_t ina_pin,
                                  uint8_t inb_pin,
                                  uint8_t pwm_pin,
                                  bool    invert_motor         = false,
                                  int     bus_millivolts       = 12000,   // 12V bus
                                  int     scale_pwm_millivolts = 12000,   // scale_pwm_millivolts / bus_millivolts => 100.0% scale
                                  uint8_t adc_pin              = INVALID,
                                  int     scale_adc_milliamps  = 20000 ); // 20A max

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void  attachPitchEncoder(       uint8_t pin                                                  );
  void  attachPitchEncoderLimits( int     ccw_limit_millidegrees, int    cw_limit_millidegrees );
  void  attachPitchDigitalLimits( uint8_t ccw_digital_limit_pin,  uint8_t cw_digital_limit_pin, bool normally_closed=true );
  void  attachRollEncoder(        uint8_t pin                                                  );
  void  attachRollEncoderLimits(  int     ccw_limit_millidegrees, int    cw_limit_millidegrees );
  void  attachRollDigitalLimits(  uint8_t ccw_digital_limit_pin,  uint8_t cw_digital_limit_pin, bool normally_closed=true );
  void  attachMotorCurrentLimits( int     current_limits_milliamps );

  ////////////////////////////////
  bool isPitchEncoderWireBroken();
  bool isRollEncoderWireBroken();
  bool isEncoderWireBroken();

  ////////////////////////////////////
  int  readPitchEncoderMillidegrees();
  int  readRollEncoderMillidegrees();
  int  readPitchEncoderRadians();
  int  readRollEncoderRadians();

  ///////////////////////////////////
  bool isWithinPitchEncoderLimitCcw();
  bool isWithinPitchEncoderLimitCw();
  bool isWithinRollEncoderLimitCcw();
  bool isWithinRollEncoderLimitCw();
  bool isWithinPitchEncoderLimits();
  bool isWithinRollEncoderLimits();
  bool isWithinEncoderLimits();

  ////////////////////////////////////
  bool isWithinPitchDigitalLimitCcw();
  bool isWithinPitchDigitalLimitCw();
  bool isWithinRollDigitalLimitCcw();
  bool isWithinRollDigitalLimitCw();
  bool isWithinPitchDigitalLimits();
  bool isWithinRollDigitalLimits();
  bool isWithinDigitalLimits();

  //////////////////////
  bool isWireBreaks();
  bool isOverTravel();
  bool isOverCurrent();

  ///////////////////////////////////////////////////////////
  bool isDrivingDecipercentCcw(      int drive_decipercent );
  bool isDrivingDecipercentCw(       int drive_decipercent );
  bool isPitchMovingMillidegreesCcw( int move_millidegrees );
  bool isPitchMovingMillidegreesCw(  int move_millidegrees );
  bool isRollMovingMillidegreesCcw(  int move_millidegrees );
  bool isRollMovingMillidegreesCw(   int move_millidegrees );

  /////////////////////////////////////////////////////////////////////////////////////
  bool isDrivingDecipercentPastLimits( int pitch_decipercent,  int roll_decipercent  );
  bool isMovingMillidegreesPastLimits( int pitch_millidegrees, int roll_millidegrees );

  //////////////////////////////////////////
  bool isStarboardMotorWithinCurrentLimit();
  bool isPortMotorWithinCurrentLimit();
  bool isWithinCurrentLimits();

  /////////////
  void estop();

  //////////////////////////////////////////////////////////////////////////////
  void  pitchRollDecipercent(  int pitch_decipercent,  int roll_decipercent   );
  void  pitchRollMillidegrees( int pitch_millidegrees, int  roll_millidegrees );

  void motionScan(); // Todo => pitchRollMillidegrees() => CMD::PITCH_ROLL_MILLIDEGREES
};

#endif // ROVE_DIFF_STM_VNH_US_DIGI_MA3_H






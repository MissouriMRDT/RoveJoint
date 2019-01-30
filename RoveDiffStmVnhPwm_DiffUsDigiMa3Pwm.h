///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => Energia Texas Instruments Tiva C, Todo
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DIFF_STM_VNH_PWM_DIFF_US_DIGI_MA3_PWM_H
#define DIFF_STM_VNH_PWM_DIFF_US_DIGI_MA3_PWM_H

#include "DiffStmVnhPwm_DiffUsDigiMa3Pwm.h"
#include "RoveMotor.h"
#include "RoveEncoder.h"

/*
#include "StmVnhPwm.h"
#include "UsDigiMa3Pwm.h"
#include "RoveBoardMap.h"
*/

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Given a RoveJoint instance
// StmVnh_UsDigi12bPwmMa3    RoveJoint;

// RoveJoint.attachStarboardMotor( PIN_x1, PIN_x2, PIN_x3, PIN_x4, bool invert_motor, int drive_decipercent );
// RoveJoint.attachPortMotor(      PIN_x1, PIN_x2, PIN_x3, PIN_x4, bool invert_motor, int drive_decipercent );

// pay attention to wiring polarity and choose port/starboard invert motors and for differential gearing
// scale the drive command pwm output for 0%*DcBus ~ 100.0%*DcBus in decipercent int 0~1000 => 0.0~100.0

// Call attachStarboardMotorStmVnh or attachPortMotorStmVnh
// attach both starboard and port motors and invert one of the two for differenterial mode

/////////////////////////////////////////////////////////////////////////////////////////////////
class DiffStmVnhPwm_DiffUsDigiMa3Pwm
{
public:
  StmVnhPwm          StarBoardMotor;
  StmVnhPwm          PortMotor;

  UsDigiMa3Pwm       PitchEncoder;
  UsDigiMa3Pwm       RollEncoder;

//RovePid            PitchPid;
//RovePid            RollPid;

/*  enum class CMD: uint32_t {   NO_CMD, 
                               ESTOP, 
                               DRIVE_DECIPERCENT,
                               MOVE_MILLDEGREES }

  enum class ERR: uint32_t {   NO_ERR, 
                               UNKNOWN }*/

// uint32_t   cmd                     = CMD::NO_CMD;
  int        cmd_setpoint            = 0;       // scaling/units based on present CMD

//uint32_t   err                     = ERR::NO_ERR;

  const int  ccw_digital_limit_pin   = INVALID; // optional pin isDrivingPastLimits 
  const int  cw_digital_limit_pin    = INVALID; // optional pin isDrivingPastLimits

  const int  ccw_limit_millidegrees  = 360000;  // one full revolution in millidegrees 
  const int  cw_limit_millidegrees   = 0;       // 0~360.00 degrees => 0~360000 millidegrees

  const int  current_limit_milliamps = 20000;   // 20amps in milliamps => 20000 milliamps

  void  attachStarboardMotor(     const uint8_t ina_pin,
                                  const uint8_t inb_pin,
                                  const uint8_t pwm_pin,
                                  const uint8_t adc_pin              = INVALID,
                                  const bool    invert_motor         = false,
                                  const int     bus_millivolts       = 12000, // 12V bus
                                  const int     scale_pwm_millivolts = 12000, // 12V max = 100.0% scale
                                  const int     scale_adc_milliamps  = 20000 );

  void  attachPortMotor(          const uint8_t ina_pin,
                                  const uint8_t inb_pin,
                                  const uint8_t pwm_pin,
                                  const uint8_t adc_pin              = INVALID,
                                  const bool    invert_motor         = false,
                                  const int     bus_millivolts       = 12000, // 12V bus
                                  const int     scale_pwm_millivolts = 12000, // 12V max = 100.0% scale
                                  const int     scale_adc_milliamps  = 20000 );
  void  attachMotorCurrentLimits( const int     current_limits_milliamps );

  void  attachPitchEncoder(       const uint8_t pin );

  void  attachPitchEncoderLimits( const int      cw_milli_degree_limit, 
                                  const int     ccw_milli_degree_limit );

  void  attachPitchDigitalLimits( const uint8_t cw_digital_limit_pin,
                                  const uint8_t ccw_digital_limit_pin );

  void  attachRollEncoder(       const uint8_t pin );

  void  attachRollEncoderLimits( const int      cw_milli_degree_limit, 
                                 const int     ccw_milli_degree_limit );

  void  attachRollDigitalLimits( const uint8_t cw_digital_limit_pin,
                                 const uint8_t ccw_digital_limit_pin );

  void  pitchRollDecipercent(    const int     pitch_decipercent,
                                 const int     roll_decipercent );

//void  pitchRollMillidegrees(     const int     pitch_millidegrees,
//                                 const int     roll_millidegrees );

  int   readGearMillidegrees();
  int   readStarboardMilliamps();
  int   readPortMilliamps();

  bool  isGearEncoderWireBroken();

  bool  isDrivingGearCw();
  bool  isDrivingGearCcw();
  bool  isDrivingGearPastLimits();

  bool  isWithinGearDigitalLimitCw();
  bool  isWithinGearDigitalLimitCcw();
  bool  isWithinGearEncoderLimitCw();
  bool  isWithinGearEncoderLimitCcw();

  bool  isWithinCurrentLimits();
  bool  isWithinStarboardCurrentLimit();
  bool  isWithinPortCurrentLimit();

  void  estop();
};

#endif // DIFF_STM_VNH_PWM_DIFF_US_DIGI_MA3_PWM_H




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => Energia Texas Instruments Tiva C, Todo
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RoveDiffStmVnhUsDigiMa3.h"
#include "RoveStmVnhPwm.h"
#include "RoveUsDigiMa3Pwm.h"
#include "RoveBoardMap.h"

#include "Energia.h"

#include <stdint.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////
void RoveDiffStmVnhUsDigiMa3::attachStarboardMotor( uint8_t ina_pin,        uint8_t inb_pin, 
                                                    uint8_t pwm_pin,        bool    invert_motor, 
                                                    int     bus_millivolts, int     scale_to_millivolts,
                                                    uint8_t adc_pin,        int     scale_to_milliamps )
{ this->StarboardMotor.attach(                              ina_pin,                inb_pin, 
                                                            pwm_pin,                invert_motor, 
                                                            bus_millivolts,         scale_to_millivolts,
                                                            adc_pin,                scale_to_milliamps ); 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void RoveDiffStmVnhUsDigiMa3::attachPortMotor( uint8_t ina_pin,        uint8_t inb_pin, 
                                               uint8_t pwm_pin,        bool    invert_motor, 
                                               int     bus_millivolts, int     scale_to_millivolts,
                                               uint8_t adc_pin,        int     scale_to_milliamps ) 
{ this->PortMotor.attach(                              ina_pin,                inb_pin, 
                                                       pwm_pin,                invert_motor, 
                                                       bus_millivolts,         scale_to_millivolts,
                                                       adc_pin,                scale_to_milliamps ); 
}

//////////////////////////////////////////////////////////////////
void  RoveDiffStmVnhUsDigiMa3::attachPitchEncoder( uint8_t pin )
{                              this->PitchEncoder.attach(  pin); }

////////////////////////////////////////////////////////////////////////////////////
void RoveDiffStmVnhUsDigiMa3::attachPitchEncoderLimits( int ccw_limit_millidegrees, 
                                                        int cw_limit_millidegrees )
{                     this->pitch_ccw_limit_millidegrees  = ccw_limit_millidegrees; 
                      this->pitch_cw_limit_millidegrees   = cw_limit_millidegrees; 
}

///////////////////////////////////////////////////////////////////////////////////////
void RoveDiffStmVnhUsDigiMa3::attachPitchDigitalLimits( uint8_t ccw_digital_limit_pin, 
                                                        uint8_t cw_digital_limit_pin,
                                                        bool    normally_closed )
{                          this->pitch_ccw_digital_limit_pin  = ccw_digital_limit_pin; 
                           this->pitch_cw_digital_limit_pin   = cw_digital_limit_pin;
                           this->roll_digital_limits_nc       = normally_closed; 
}

/////////////////////////////////////////////////////////////////
void  RoveDiffStmVnhUsDigiMa3::attachRollEncoder( uint8_t pin )
{                              this->RollEncoder.attach(  pin); }

//////////////////////////////////////////////////////////////////////////////////
void RoveDiffStmVnhUsDigiMa3::attachRollEncoderLimits( int ccw_limit_millidegrees, 
                                                       int cw_limit_millidegrees )
{                      this->roll_ccw_limit_millidegrees = ccw_limit_millidegrees; 
                       this->roll_cw_limit_millidegrees  = cw_limit_millidegrees; 
}

/////////////////////////////////////////////////////////////////////////////////////
void RoveDiffStmVnhUsDigiMa3::attachRollDigitalLimits( uint8_t ccw_digital_limit_pin, 
                                                       uint8_t cw_digital_limit_pin,
                                                       bool    normally_closed  )
{                          this->roll_ccw_digital_limit_pin  = ccw_digital_limit_pin; 
                           this->roll_cw_digital_limit_pin   = cw_digital_limit_pin; 
                           this->roll_digital_limits_nc      = normally_closed;
}

///////////////////////////////////////////////////////////////////////////////////////
void RoveDiffStmVnhUsDigiMa3::attachMotorCurrentLimits( int current_limits_milliamps )
{                          this->current_limits_milliamps = current_limits_milliamps; }

/////////////////////////////////////////////////////////////////////////////////////////////////////
bool RoveDiffStmVnhUsDigiMa3::isPitchEncoderWireBroken(){ return this->PitchEncoder.isWireBroken(); }
bool RoveDiffStmVnhUsDigiMa3::isRollEncoderWireBroken() { return this->RollEncoder.isWireBroken();  }
bool RoveDiffStmVnhUsDigiMa3::isEncoderWireBroken()     { return this->isPitchEncoderWireBroken() 
                                                              && this->isRollEncoderWireBroken();   }

///////////////////////////////////////////////////////////////////////////////////////////////////////
int  RoveDiffStmVnhUsDigiMa3::readPitchEncoderMillidegrees() { this->PitchEncoder.readMillidegrees(); }
int  RoveDiffStmVnhUsDigiMa3::readPitchEncoderRadians()      { this->PitchEncoder.readRadians();      }
int  RoveDiffStmVnhUsDigiMa3::readRollEncoderMillidegrees()  { this->RollEncoder.readMillidegrees();  }
int  RoveDiffStmVnhUsDigiMa3::readRollEncoderRadians()       { this->RollEncoder.readRadians();       }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool RoveDiffStmVnhUsDigiMa3::isWithinPitchEncoderLimitCcw() { return ( this->PitchEncoder.readMillidegrees() < this->pitch_ccw_limit_millidegrees ); }
bool RoveDiffStmVnhUsDigiMa3::isWithinPitchEncoderLimitCw()  { return ( this->PitchEncoder.readMillidegrees() > this->pitch_cw_limit_millidegrees  ); }
bool RoveDiffStmVnhUsDigiMa3::isWithinRollEncoderLimitCcw()  { return ( this->RollEncoder.readMillidegrees()  < this->roll_ccw_limit_millidegrees  ); }
bool RoveDiffStmVnhUsDigiMa3::isWithinRollEncoderLimitCw()   { return ( this->RollEncoder.readMillidegrees()  > this->roll_cw_limit_millidegrees   ); }

bool RoveDiffStmVnhUsDigiMa3::isWithinPitchEncoderLimits()   { return this->isWithinPitchEncoderLimitCcw() && this->isWithinPitchEncoderLimitCw(); }
bool RoveDiffStmVnhUsDigiMa3::isWithinRollEncoderLimits()    { return this->isWithinRollEncoderLimitCcw()  && this->isWithinRollEncoderLimitCw();  }
bool RoveDiffStmVnhUsDigiMa3::isWithinEncoderLimits()        { return this->isWithinPitchEncoderLimits()   && this->isWithinRollEncoderLimits();   }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool RoveDiffStmVnhUsDigiMa3::isWithinPitchDigitalLimitCcw() { return digitalRead( this->pitch_ccw_digital_limit_pin ) == this->roll_digital_limits_nc; }
bool RoveDiffStmVnhUsDigiMa3::isWithinPitchDigitalLimitCw()  { return digitalRead( this->pitch_cw_digital_limit_pin  ) == this->roll_digital_limits_nc; }
bool RoveDiffStmVnhUsDigiMa3::isWithinRollDigitalLimitCcw()  { return digitalRead( this->roll_ccw_digital_limit_pin  ) == this->roll_digital_limits_nc; }
bool RoveDiffStmVnhUsDigiMa3::isWithinRollDigitalLimitCw()   { return digitalRead( this->roll_cw_digital_limit_pin   ) == this->roll_digital_limits_nc; }

bool RoveDiffStmVnhUsDigiMa3::isWithinPitchDigitalLimits()   { return this->isWithinPitchDigitalLimitCcw() && this->isWithinPitchDigitalLimitCw(); }
bool RoveDiffStmVnhUsDigiMa3::isWithinRollDigitalLimits()    { return this->isWithinRollDigitalLimitCcw()  && this->isWithinRollDigitalLimitCw();  }
bool RoveDiffStmVnhUsDigiMa3::isWithinDigitalLimits()        { return this->isWithinPitchDigitalLimits()   && this->isWithinRollDigitalLimits();  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool RoveDiffStmVnhUsDigiMa3::isStarboardMotorWithinCurrentLimit() { return this->StarboardMotor.readMilliamps() > this->current_limits_milliamps; }
bool RoveDiffStmVnhUsDigiMa3::isPortMotorWithinCurrentLimit()      { return this->PortMotor.readMilliamps()      > this->current_limits_milliamps; }

bool RoveDiffStmVnhUsDigiMa3::isWithinCurrentLimits()              { return this->isStarboardMotorWithinCurrentLimit() 
                                                                         && this->isPortMotorWithinCurrentLimit(); }

////////////////////////////////////////////////////////////////////////////////////////////////////
bool RoveDiffStmVnhUsDigiMa3::isWireBreaks()  { return   this->isEncoderWireBroken();              }
bool RoveDiffStmVnhUsDigiMa3::isOverTravel()  { return ( this->isWithinEncoderLimits() == false ) 
                                                    || ( this->isWithinDigitalLimits() == false ); }
bool RoveDiffStmVnhUsDigiMa3::isOverCurrent() { return ( this->isWithinCurrentLimits() == false ); }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool RoveDiffStmVnhUsDigiMa3::isDrivingDecipercentCcw(      int drive_decipercent  ) { return drive_decipercent  > 0; }
bool RoveDiffStmVnhUsDigiMa3::isDrivingDecipercentCw(       int drive_decipercent  ) { return drive_decipercent  < 0; }
bool RoveDiffStmVnhUsDigiMa3::isPitchMovingMillidegreesCcw( int move_millidegrees )  { return move_millidegrees > this->readPitchEncoderMillidegrees(); }
bool RoveDiffStmVnhUsDigiMa3::isPitchMovingMillidegreesCw(  int move_millidegrees )  { return move_millidegrees < this->readPitchEncoderMillidegrees(); }
bool RoveDiffStmVnhUsDigiMa3::isRollMovingMillidegreesCcw(  int move_millidegrees )  { return move_millidegrees > this->readRollEncoderMillidegrees();  }
bool RoveDiffStmVnhUsDigiMa3::isRollMovingMillidegreesCw(   int move_millidegrees )  { return move_millidegrees < this->readRollEncoderMillidegrees();  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool RoveDiffStmVnhUsDigiMa3::isDrivingDecipercentPastLimits( int pitch_decipercent, int roll_decipercent  )
{
  if(     ( this->isStarboardMotorWithinCurrentLimit() == false )
  ||      ( this->isPortMotorWithinCurrentLimit()      == false ) )
  { return true; }

  ////////////////////////////////////////////////////////////////
  if(  (    this->isDrivingDecipercentCcw( pitch_decipercent ) )
  && ( (    this->isWithinPitchDigitalLimitCcw() == false )
       || ( this->isWithinPitchEncoderLimitCcw() == false ) ) )
  { return true; }

  else if( ( this->isDrivingDecipercentCw( pitch_decipercent ) )
  && (    (  this->isWithinPitchDigitalLimitCw() == false )
       || (  this->isWithinPitchEncoderLimitCw() == false ) ) )
  { return true; }

  ////////////////////////////////////////////////////////////////
  if(  (    this->isDrivingDecipercentCcw( roll_decipercent ) )
  && ( (    this->isWithinRollDigitalLimitCcw() == false )
       || ( this->isWithinRollEncoderLimitCcw() == false ) ) )
  { return true; }

  else if( ( this->isDrivingDecipercentCw( roll_decipercent ) )
  && (    (  this->isWithinRollDigitalLimitCw() == false )
       || (  this->isWithinRollEncoderLimitCw() == false ) ) )
  { return true; }
  
  /////////////
  return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool RoveDiffStmVnhUsDigiMa3::isMovingMillidegreesPastLimits( int pitch_millidegrees, int roll_millidegrees )
{
  if(      this->isWithinCurrentLimits() == false )
  { return true; }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  if(  (    this->isPitchMovingMillidegreesCcw( pitch_millidegrees ) )
  && ( (    this->isWithinPitchDigitalLimitCcw() == false )
       || ( this->isWithinPitchEncoderLimitCcw() == false ) ) )
  { return true; }

  else if( ( this->isPitchMovingMillidegreesCw( pitch_millidegrees ) )
  && (    (  this->isWithinPitchDigitalLimitCw() == false )
       || (  this->isWithinPitchEncoderLimitCw() == false ) ) )
  { return true; }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  if(  (    this->isRollMovingMillidegreesCcw( roll_millidegrees ) )
  && ( (    this->isWithinRollDigitalLimitCcw() == false )
       || ( this->isWithinRollEncoderLimitCcw() == false ) ) )
  { return true; }

  else if( ( this->isRollMovingMillidegreesCw( roll_millidegrees ) )
  && (    (  this->isWithinRollDigitalLimitCw() == false )
       || (  this->isWithinRollEncoderLimitCw() == false ) ) )
  { return true; }

  /////////////
  return false;
}

///////////////////////////////////////////////////////////////////////////////////
void RoveDiffStmVnhUsDigiMa3::estop()
{
  if( this->StarboardMotor.pwm_mode != INVALID ) { this->StarboardMotor.coast(); }
  if( this->PortMotor.pwm_mode      != INVALID ) { this->PortMotor.coast();      }
}

//////////////////////////////////////////////////////////////////////////////////////////////////
void RoveDiffStmVnhUsDigiMa3::pitchRollDecipercent( int pitch_decipercent, int roll_decipercent )
{
  if( this->isDrivingDecipercentPastLimits( pitch_decipercent, roll_decipercent ) )
  {   this->estop(); }

  else if( ( this->StarboardMotor.pwm_mode != INVALID )
       &&  ( this->PortMotor.pwm_mode      != INVALID ) ) 
  { // Todo =>

    int starboard_pitch_decipercent  = pitch_decipercent; // motors move in same direction to tilt on the pitch axis
    int port_pitch_decipercent       = pitch_decipercent; 

    int starboard_roll_decipercent   = -roll_decipercent; // motors move in opposite direction to twist on the roll axis
    int port_roll_decipercent        =  roll_decipercent; 

    this->StarboardMotor.drive( ( starboard_pitch_decipercent + starboard_roll_decipercent ) / 2 ); // combined differenterial
    this->PortMotor.drive(      ( port_pitch_decipercent      + port_roll_decipercent      ) / 2 ); } 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void RoveDiffStmVnhUsDigiMa3::pitchRollMillidegrees( int pitch_millidegrees, int roll_millidegrees )
{
  if( this->isMovingMillidegreesPastLimits( pitch_millidegrees, roll_millidegrees ) )
  {   this->estop(); }

  else if( ( this->StarboardMotor.pwm_mode != INVALID )
       &&  ( this->PortMotor.pwm_mode      != INVALID ) ) 
  {  /* Todo => setup pid_setpoint and CMD flags for motionScan */ }
}



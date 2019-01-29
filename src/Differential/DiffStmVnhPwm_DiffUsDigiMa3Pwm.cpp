// Todo

#include "DiffStmVnhPwm_DiffUsDigiMa3Pwm.h"
#include "RoveMotor.h"
#include "RoveEncoder.h"

#include <stdint.h>

/*
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DiffStmVnhPwm_DiffUsDigiMa3Pwm::attachStarboardMotor( 
  const uint8_t      ina_pin, const uint8_t    inb_pin, const uint8_t pwm_pin, const uint8_t adc_pin,
  const bool    invert_motor, const int bus_millivolts, const int     scale_to_millivolts )

{ this->StarboardMotor.attach( ina_pin, inb_pin, pwm_pin, adc_pin, invert_motor, bus_millivolts, scale_to_millivolts ); }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DiffStmVnhPwm_DiffUsDigiMa3Pwm::attachPortMotor(
  const uint8_t      ina_pin, const uint8_t    inb_pin, const uint8_t pwm_pin, const uint8_t adc_pin,
  const bool    invert_motor, const int bus_millivolts, const int     scale_to_millivolts )

{ this->PortMotor.attach( ina_pin, inb_pin, pwm_pin, adc_pin,invert_motor, bus_millivolts, scale_to_millivolts ); }

/////////////////////////////////////////////////////////////////////////////////////////////////////
void DiffStmVnhPwm_DiffUsDigiMa3Pwm::attachStarboardCurrentLimit(  const int  current_limit_milliamps )
{                                          this->current_limit_milliamps = current_limit_milliamps; }

////////////////////////////////////////////////////////////////////////////////////////////////
void DiffStmVnhPwm_DiffUsDigiMa3Pwm::attachPortCurrentLimit(  const int  current_limit_milliamps )
{                                     this->current_limit_milliamps = current_limit_milliamps; }

//////////////////////////////////////////////////////////////////////////////////////////////////
void DiffStmVnhPwm_DiffUsDigiMa3Pwm::attachGearDigitalLimitCw(  const uint8_t cw_digital_limit_pin )
{                                             this->cw_digital_limit_pin = cw_digital_limit_pin; }

///////////////////////////////////////////////////////////////////////////////////////////////////
void DiffStmVnhPwm_DiffUsDigiMa3Pwm::attachGearDigitalLimitCcw( const uint8_t ccw_digital_limit_pin )
{                                            this->ccw_digital_limit_pin = ccw_digital_limit_pin; }

////////////////////////////////////////////////////////////////////////////////////////////////
void DiffStmVnhPwm_DiffUsDigiMa3Pwm::attachGearEncoderLimitCw( const int  cw_milli_degree_limit )
{                                        this->cw_limit_millidegrees = cw_milli_degree_limit; }

/////////////////////////////////////////////////////////////////////////////////////////////////
void DiffStmVnhPwm_DiffUsDigiMa3Pwm::attachEncoderLimitGearCcw( const int  ccw_limit_millidegrees )
{                                        this->ccw_limit_millidegrees = ccw_limit_millidegrees; }

///////////////////////////////////////////////////
void DiffStmVnhPwm_DiffUsDigiMa3Pwm::estop()
{
  this->cmd                          = ESTOP;
  this->cmd_setpoint                 = 0;

  if( this->StarboardMotor.pwm_mode != INVALID ) {
      this->StarboardMotor.coast();              }
      
  if( this->PortMotor.pwm_mode      != INVALID ) {
      this->PortMotor.coast();                   }
} }

//////////////////////////////////////////////////////////////////////
void DiffStmVnhPwm_DiffUsDigiMa3Pwm::driveDecipercent( int drive_decipercent )
{
  if( this->isDrivingPastLimits() )
  {   this->estop(); }
  else
  {
    this->cmd                           = this::CMD::DRIVE_DECIPERCENT;
    this->cmd_setpoint                  = drive_decipercent;

    if( this->StarboardMotor.pwm_mode != INVALID )             {
        this->StarboardMotor.drive(      this->cmd_setpoint ); }

    if( this->PortMotor.pwm_mode      != INVALID )             {
        this->PortMotor.drive(           this->cmd_setpoint ); }
} }

//////////////////////////////////////////////////////////
int  DiffStmVnhPwm_DiffUsDigiMa3Pwm::readGearMillidegrees()
{         this->GearEncoder.readMillidegrees(); }

/////////////////////////////////////////////////////////
bool StmVnhPwm_UsDigiMa3Pwm::isGearEncoderWireBroken()
{              return this->GearEncoder.isWireBroken(); }

////////////////////////////////////////////////////////////
bool StmVnhPwm_UsDigiMa3Pwm::isDrivingGearCcw()
{
  if (        this->cmd == this::CMD::DRIVE_DECIPERCENT )
  { if(       this->cmd_setpoint >  0 ) 
    {  return true;  }

    else if(  this->cmd_setpoint <= 0 ) 
    {  return false; } }

  else if(    this->cmd == this::CMD::MOVE_MILLIDEGREES )
  {   int millidegrees   = this->readGearMillidegrees();
    if(       this->cmd_setpoint >  millidegrees ) 
    {  return true; }

    else if(  this->cmd_setpoint <= millidegrees )
    { return  false; } }
}

/////////////////////////////////////////////////////////
bool StmVnhPwm_UsDigiMa3Pwm::isDrivingGearPastLimits()
{
  if(      this->isWithinCurrentLimits()   == false )
  { return true; }

  else if( this->isDrivingGearCcw()                     ) 
  {    if( this->isWithinGearDigitalLimitCcw() == false )
       ||( this->isWithinGearEncoderLimitCcw() == false ))
       { return true; } }

  else // isDrivingCw
  {    if( this->isWithinGearDigitalLimitCw() == false )
       ||( this->isWithinGearEncoderLimitCw() == false ))
       { return true; } }

  else 
  { return  false; }
}

//////////////////////////////////////////////////////
bool StmVnhPwm_UsDigiMa3Pwm::isWithinCurrentLimits()
{ return this->isStarboardWithinCurrentLimit()
           && this->isPortWithinCurrentLimit(); }

//////////////////////////////////////////////////////////////////////////////////
bool StmVnhPwm_UsDigiMa3Pwm::isWithinStarboardCurrentLimit()
{ 
  if( this->StarboardMotor.adc_pin == INVALID ) 
  { return true; }
  else 
  { return this->current_limit_milliamps <= this->Motor_Starboard.readMillamps() }
}

//////////////////////////////////////////////////////////////////////////////
bool StmVnhPwm_UsDigiMa3Pwm::isPortWithinCurrentLimit()
{ 
  if( this->PortMotor.adc_pin == INVALID ) 
  { return true; }
  else 
  { return this->current_limit_milliamps <= this->Motor_Port.readMillamps() }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
bool StmVnhPwm_UsDigiMa3Pwm::isWithinGearDigitalLimitCw()
{ if(  this->cw_digital_limit_pin == INVALID ) { return true;                                      }
  else                                         { return digitalRead( this->cw_digital_limit_pin ); }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
bool StmVnhPwm_UsDigiMa3Pwm::isWithinGearDigitalLimitCcw()
{ if(  this->ccw_digital_limit_pin == INVALID ) { return true;                                       }
  else                                          { return digitalRead( this->ccw_digital_limit_pin ); }
}

////////////////////////////////////////////////////////////////////////////////
bool StmVnhPwm_UsDigiMa3Pwm::isWithinGearEncoderLimitCw()
{ return this->GearEncoder.readMillidegrees() >= this->cw_milli_degree_limit; }

/////////////////////////////////////////////////////////////////////////////////
bool StmVnhPwm_UsDigiMa3Pwm::isWithinGearEncoderLimitCcw()
{ return this->GearEncoder.readMillidegrees() <= this->ccw_milli_degree_limit; }

*/
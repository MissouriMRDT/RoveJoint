///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019 => Energia Texas Instruments Tiva C, Todo
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Todo => Ask Drue / Novatny for xyz orientation of BaseRotateOffsetAngle? WristTiltHardStopUp? BaseRotateDeadband? //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//#include "RoveCommEthernetAtlas.h"
#include "RoveDiffStmVnhUsDigiMa3.h"
//#include "RoveUsDigiMa3.h"
#include "RoveTimerInterrupt.h"
//#include "RoveWatchdog.h"

///////////////////////////////////////////////////////////////////////////////

#define  CMD_TEST_ESTOP                     0 // Todo reuse RoveCommAtlas ArmId
#define  CMD_TEST_DRIVE_DECIPERCENT         0

#define  TELEM_TESTBOARD_DID_ESTOP          0
#define  TELEM_TESTBOARD_MOTORS_MILLIAMPS   0
#define  TELEM_TESTBOARD_GEARS_MILLIDEGREES 0

#define  TELEM_DATA_TEST_BOARD_OVERCURRENT  0
#define  TELEM_DATA_TEST_BOARD_WIREBREAK    0
#define  TELEM_DATA_TEST_BOARD_OVERTRAVEL   0 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 4 Motors on 2 Dual Motor Differenterial Joints control 4 axes of => RoveJoint DualDiffLink
// Two push pull DC motors operate a four gear link at EACH of two RoveJoints, each with with PIDs on Pitch, Roll axes
// Two joints => Four axis, Eight Gears, Four motors, Four encoders => BaseBiaxialJoint.pitchRoll( ... )... ExtendBiaxialJoint.pitchRoll( ... )...

////////////////////////////////////////////////////////////////////////////
// Move the motors the same direction for joint millidegrees about the Pitch z axis
// Move the Motors opposite direction for about the Roll  axis 

/////////////////////////////////////////////////////////////////////////////
// Using RoveJoint/Diff_StmVnhPwm_UsDigiMa3Pwm
// =>  DriveDecipercent voltage control or MoveMillidegrees position control

////////////////////////////////////////////////////////////////
// => BaseBiaxialJoint.pitchRollDecipercent()
//    ExtendBiaxialJoint.pitchRollDecipercent() for open loop voltage control

// or

//////////////////////////////////////////////////////////////////////////
// Todo => TimerInterrupt.attach( motionScan() ...
//      => BaseBiaxialJoint.pitchRollMillidegrees()
//         ExtendBiaxialJoint.pitchRollMillidegrees()  for closed loop position control

// ArmCurrentPosition = 0x318

//////////////////////////////////////////////////////////

class DualDiffLinksBoard
{
  public:

  //RoveCommEthernetAtlas      RoveComm;

    RoveDiffStmVnhUsDigiMa3    BaseDiffJoint;
    RoveDiffStmVnhUsDigiMa3    ExtendDiffJoint;

  //UsDigiMa3PwmWireBreaks     WireBreaksScan;

    RoveTimerInterrupt         TelemetryScan;
    RoveTimerInterrupt         MotionScan;

  //RoveWatchdog               Watchdog;
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
DualDiffLinksBoard DualDiffLinksBoard;                                      // -1000~1000 decipercent  (    100.0 % )
                                                                            //  0~360000  millidegrees (   36.000 o )
                                                                            //  0~36000   millivolts   (   36.000 V )
                                                                            //  0~20000   milliamps    (   20.000 A )
void telemetryScan();
void motionScan();
void estop();

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  pinMode( PN_1, OUTPUT ); // 1294 Launchpad Led D1
//pinMode( PN_0, OUTPUT ); // 1294 Launchpad Led D2
//pinMode( PF_4, OUTPUT ); // 1294 Launchpad Led D3
//pinMode( PF_0, OUTPUT ); // 1294 Launchpad Led D4
  Serial.begin( 9600 );

  DualDiffLinksBoard.BaseDiffJoint.attachStarboardMotor(   PN_0, PF_4, PF_0 ); // ina_pin, inb_pin, pwm_pin, adc_pin, invert_motor
//DualDiffLinksBoard.BaseDiffJoint.attachPortMotor(        PX_0, PX_0, PX_0, PX_0,  true ); // ina_pin, inb_pin, pwm_pin, adc_pin, invert_motor
//DualDiffLinksBoard.BaseDiffJoint.attachPitchEncoder(     PX_0                          ); // gear_encoder_pin
//DualDiffLinksBoard.BaseDiffJoint.attachRollEncoder(      PX_0                          ); // gear_encoder_pin

//DualDiffLinksBoard.BaseDiffJoint.attachPitchEncoderLimits(                   0, 360000 ); // cw_milli_degree_limit, ccw_milli_degree_limit
//DualDiffLinksBoard.BaseDiffJoint.attachPitchDigitalLimits(                 PX_0,  PX_0 ); // cw_digital_limit_pin, ccw_digital_limit_pin
//DualDiffLinksBoard.BaseDiffJoint.attachPitchGearPid(    1000, 1000, 1000, -1000,  1000 ); // Kp, Ki, Kd, min_output, max_output
//DualDiffLinksBoard.BaseDiffJoint.attachRollEncoderLimits(                    0, 360000 ); // cw_milli_degree_limit, ccw_milli_degree_limit
//DualDiffLinksBoard.BaseDiffJoint.attachRollDigitalLimits(                 PX_0,   PX_0 ); // cw_digital_limit_pin, ccw_digital_limit_pin
//DualDiffLinksBoard.BaseDiffJoint.attachRollGearPid(    1000, 1000, 1000, -1000,   1000 ); // Kp, Ki, Kd, min_output, max_output
//DualDiffLinksBoard.BaseDiffJoint.attachMotorCurrentLimits(                       20000 ); // current_limit_milliamps

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DualDiffLinksBoard.ExtendDiffJoint.attachStarboardMotor( PX_0, PX_0, PX_0, PX_0, false ); // ina_pin, inb_pin, pwm_pin, adc_pin, invert_motor
//DualDiffLinksBoard.ExtendDiffJoint.attachPortMotor(      PX_0, PX_0, PX_0, PX_0,  true ); // ina_pin, inb_pin, pwm_pin, adc_pin, invert_motor
//DualDiffLinksBoard.ExtendDiffJoint.attachPitchEncoder(   PX_0                          ); // gear_encoder_pin
//DualDiffLinksBoard.ExtendDiffJoint.attachRollEncoder(    PX_0                          ); // gear_encoder_pin

//DualDiffLinksBoard.ExtendDiffJoint.attachPitchEncoderLimits(                 0, 360000 ); // cw_milli_degree_limit, ccw_milli_degree_limit
//DualDiffLinksBoard.ExtendDiffJoint.attachPitchDigitalLimits(               PX_0,  PX_0 ); // cw_digital_limit_pin, ccw_digital_limit_pin
//DualDiffLinksBoard.ExtendDiffJoint.attachPitchGearPid(  1000, 1000, 1000, -1000,  1000 ); // Kp, Ki, Kd, min_output, max_output
//DualDiffLinksBoard.ExtendDiffJoint.attachRollEncoderLimits(                  0, 360000 ); // cw_milli_degree_limit, ccw_milli_degree_limit
//DualDiffLinksBoard.ExtendDiffJoint.attachRollDigitalLimits(               PX_0,   PX_0 ); // cw_digital_limit_pin, ccw_digital_limit_pin
//DualDiffLinksBoard.ExtendDiffJoint.attachRollGearPid(    1000, 1000, 1000, -1000,   1000 ); // Kp, Ki, Kd, min_output, max_output
//DualDiffLinksBoard.ExtendDiffJoint.attachMotorCurrentLimits(                     20000 ); // current_limit_milliamps

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  DualDiffLinksBoard.MotionScan.attachMillis(     motionScan,    T6_A,  100 ); //      10 scans per sec
  DualDiffLinksBoard.TelemetryScan.attachMillis(  telemetryScan, T7_A, 1000 ); //       1 scan  per sec

//DualDiffLinksBoard.WireBreaksScan.attachMicros(                T7_B,  100 ); //  10,000 scans per sec

//DualDiffLinksBoard.Watchdog.attachMillis(       estop,               1000 ); //       1 scan  per sec

  //////////////////////////////////////////////////
 //DualDiffLinksBoard.RoveComm.begin( IP::TESTBOARD );

  /////////////////////////////////////////
//DualDiffLinksBoard.WireBreaksScan.start();
  DualDiffLinksBoard.TelemetryScan.start();
  DualDiffLinksBoard.MotionScan.start();

//DualDiffLinksBoard.Watchdog.start();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  ///////////////////////////////////////////
  // first debug => StarboardMotor.drive(...
  // 30 second accel forward to max speed
  // 30 sec decel
  //  3 sec pause
  // 30 second accel reverse to max speed
  // 30 sec decel
  //  3 sec pause
  Serial.println("BaseBiaxialJoint.StarboardMotor sweep motor");

  for(int i=0; i <= 1000; i++ )
  {
    DualDiffLinksBoard.BaseDiffJoint.StarboardMotor.drive( i );
  //BrushedDcMotor2.drive( i ...
    delay(30);
  }

  for(int i=1000; i > 0; i-- )
  {
    DualDiffLinksBoard.BaseDiffJoint.StarboardMotor.drive( i );
  //BrushedDcMotor2.drive( i ...
    delay(30);
  }

  delay(3000);

  for(int i=0; i > -1000; i-- )
  {
    DualDiffLinksBoard.BaseDiffJoint.StarboardMotor.drive( i );
  //BrushedDcMotor2.drive( i ...
    delay(30);
  }

  for(int i=-1000; i < 0; i++ )
  {
    DualDiffLinksBoard.BaseDiffJoint.StarboardMotor.drive( i );
  //BrushedDcMotor2.drive( i ...
    delay(30);
  }

  delay(3000);

  /*
  struct    RoveCommPacket RoveCommPacket = DualDiffLinksBoard.RoveComm.read();
  if(       RoveCommPacket.data_id       == ID::CMD_TEST_ESTOP:
  {         estop(); }
  else if ( RoveCommPacket.data_id       == ID::CMD_TEST_DRIVE_DECIPERCENT )
  {         DualDiffLinksBoard.BaseDiffJoint.pitchRollDecipercent(   RoveCommPacket.data[ 1 ],   // annoying data order last year imho
                                                                     RoveCommPacket.data[ 0 ] ); // shoulder was roll followed by pitch
            DualDiffLinksBoard.ExtendDiffJoint.pitchRollDecipercent( RoveCommPacket.data[ 2 ],   // elbow was pitch followed by roll
                                                                     RoveCommPacket.data[ 3 ] ); // me ocd no like dat
            DualDiffLinksBoard.Watchdog.clear(); }*/
}

///////////////////////////////////////////////////////////////////////////////////
void motionScan()
{
/*
  // static bool toggle = false; digitalWrite( PN_1, toggle = !toggle ); // debug
  if( ( DualDiffLinksBoard.BaseDiffJoint.isWireBreaks()   )
  ||  ( DualDiffLinksBoard.ExtendDiffJoint.isWireBreaks() ) )
  {     estop(); 
      //digitalWrite( PN_1, HIGH ); // debug
        DualDiffLinksBoard.RoveComm.write( IP::RED_BASE_STATION, 
                                           ID::TELEM_TESTBOARD_DID_ESTOP, 
                                          CNT::TELEM_TESTBOARD_DID_ESTOP, 
                                         DATA::TELEM_DATA_TEST_BOARD_WIREBREAK,  );
        Serial.print("TESTBOARD WIREBREAK"); }

  if( ( DualDiffLinksBoard.BaseDiffJoint.isOverTravel()   )
  ||  ( DualDiffLinksBoard.ExtendDiffJoint.isOverTravel() ) )
  {     estop(); 
      //digitalWrite( PN_1, HIGH ); // debug
        DualDiffLinksBoard.RoveComm.write( IP::RED_BASE_STATION, 
                                           ID::TELEM_TESTBOARD_DID_ESTOP, 
                                          CNT::TELEM_TESTBOARD_DID_ESTOP, 
                                         DATA::TELEM_DATA_TEST_BOARD_OVERTRAVEL );

        Serial.print("TESTBOARD OVERTRAVEL"); }

  if( ( DualDiffLinksBoard.BaseDiffJoint.isOverCurrent()   )
  ||  ( DualDiffLinksBoard.ExtendDiffJoint.isOverCurrent() ) )
  {     estop(); 
      //digitalWrite( PN_1, HIGH ); // debug
        DualDiffLinksBoard.RoveComm.write( IP::RED_BASE_STATION, 
                                           ID::TELEM_TESTBOARD_DID_ESTOP, 
                                          CNT::TELEM_TESTBOARD_DID_ESTOP, 
                                         DATA::TELEM_DATA_TEST_BOARD_OVERCURRENT );
        Serial.print("TESTBOARD OVERCURRENT"); } */
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void telemetryScan()
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  int gears_angles_millidegrees[4] = { DualDiffLinksBoard.BaseDiffJoint.readPitchEncoderMillidegrees(),
                                      DualDiffLinksBoard.BaseDiffJoint.readRollEncoderMillidegrees(),
                                      DualDiffLinksBoard.ExtendDiffJoint.readPitchEncoderMillidegrees(),
                                      DualDiffLinksBoard.ExtendDiffJoint.readRollEncoderMillidegrees() };
/*
  DualDiffLinksBoard.RoveComm.write(  IP::RED_BASE_STATION
                                      ID::TELEM_TESTBOARD_GEARS_MILLIDEGREES,
                                     CNT::TELEM_TESTBOARD_GEARS_MILLIDEGREES,
                                      gears_angles_millidegrees );
*/
  //////////////////////////////////////////////////////////////////////////////////////////////////////
  int motor_currents_milliamps[4] = {  DualDiffLinksBoard.BaseDiffJoint.StarboardMotor.readMilliamps(),
                                      DualDiffLinksBoard.BaseDiffJoint.PortMotor.readMilliamps(),
                                      DualDiffLinksBoard.ExtendDiffJoint.StarboardMotor.readMilliamps(),
                                      DualDiffLinksBoard.ExtendDiffJoint.PortMotor.readMilliamps() };
/*
  DualDiffLinksBoard.RoveComm.write(  IP::RED_BASE_STATION
                                      ID::TELEM_TESTBOARD_MOTORS_MILLIAMPS,
                                     CNT::TELEM_TESTBOARD_MOTORS_MILLIAMPS,
                                      motor_currents_milliamps );
*/
}

/////////////////////////////////////////////////////////////////////////////////////////////
void estop()                                    // Coast all motors and alert Base station //
{
  DualDiffLinksBoard.BaseDiffJoint.estop();
  DualDiffLinksBoard.ExtendDiffJoint.estop();
//DualDiffLinksBoard.RoveComm.write( IP::RED_BASE_STATION, 
//                                   ID::TELEM_TESTBOARD_DID_ESTOP ); }
} // todo overload with zero data option and write_repeat_count option
#include "RoveJointTest.h"

void setup()
{
  Serial.begin(115200);
  Serial.println("Begun");
  RoveComm.begin(RC_ARMBOARD_FOURTHOCTET, &TCPServer);
  delay(100);

  Shoulder.LeftMotor.attach(SHOULDER_LEFT_INA, SHOULDER_LEFT_INB, SHOULDER_LEFT_PWM);
  Shoulder.RightMotor.attach(SHOULDER_RIGHT_INA, SHOULDER_RIGHT_INB, SHOULDER_RIGHT_PWM);
  Elbow.LeftMotor.attach(ELBOW_LEFT_INA, ELBOW_LEFT_INB, ELBOW_LEFT_PWM);
  Elbow.RightMotor.attach(ELBOW_RIGHT_INA, ELBOW_RIGHT_INB, ELBOW_RIGHT_PWM);

  Shoulder.TiltEncoder.attach(SHOULDER_TILT_ENCODER,false,7,250200); //offsets to have arm pointing straight up
  Shoulder.TwistEncoder.attach(SHOULDER_TWIST_ENCODER,false,7,247680);
  Shoulder.attachLimitSwitches(LS_1, LS_2);
  Shoulder.setTwistLimits(295000,57000);

  Elbow.TiltEncoder.attach(ELBOW_TILT_ENCODER,false,7,82800);
  Elbow.TwistEncoder.attach(ELBOW_TWIST_ENCODER,false,7,244800);
  Elbow.attachLimitSwitches(LS_4, LS_7);

  pinMode(LS_1, INPUT);
  pinMode(LS_2, INPUT);
  pinMode(LS_7, INPUT); //limit switch 7 on the arm moco, 2 does not work
  pinMode(LS_4, INPUT);

  pinMode(SW_IND_1, OUTPUT);

  Shoulder.LeftMotor.drive(0);
  Shoulder.RightMotor.drive(0);
  Shoulder.TiltEncoder.start();
  Shoulder.TwistEncoder.start();

  Elbow.LeftMotor.drive(0);
  Elbow.RightMotor.drive(0);
  Elbow.TiltEncoder.start();
  Elbow.TwistEncoder.start();

  Elbow.TiltPid.attach( -300.0, 300.0, 27, 0, 0);
  Elbow.TwistPid.attach( -300.0, 300.0, 15.0, 0, 0 );

  Shoulder.TiltPid.attach( -700.0, 400.0, 20.0, 0, 0 );
  Shoulder.TwistPid.attach( -600.0, 600.0, 20.0, 0, 0 );

  Watchdog.attach(stop);
  Watchdog.start(1000);

}

uint32_t last_update_time = millis();

void loop()
{
  ////////////////////////////////////////////////////////////////////////////////////////////////
  //We collect commands every loop, as well as send back our current position.
  //Closed Loop runs on every iteration but will only perform an action if we were told a specific
  //angle and we still have someways to go to ge there
  ////////////////////////////////////////////////////////////////////////////////////////////////
  parsePackets();
  updatePosition();
  openLoop();  

}

void stop()
{
  Elbow.LeftMotor.drive(0);
  Elbow.RightMotor.drive(0);
  Shoulder.LeftMotor.drive(0);
  Shoulder.RightMotor.drive(0);
  Wrist.LeftMotor.drive(0);
  Wrist.RightMotor.drive(0);
  Watchdog.clear();
}


void updatePosition()
{
   jointAngles[0] = Shoulder.TwistEncoder.readMillidegrees();
   jointAngles[1] = Shoulder.TiltEncoder.readMillidegrees();
   jointAngles[2] = Elbow.TiltEncoder.readMillidegrees();
   jointAngles[3] = Elbow.TwistEncoder.readMillidegrees();
   jointAngles[4] = Elbow.TiltEncoder.readMillidegrees();
   jointAngles[5] = Elbow.TwistEncoder.readMillidegrees();
   if(millis()-last_update_time >= ROVECOMM_UPDATE_RATE)
  {
      RoveComm.write(RC_ARMBOARD_JOINTANGLES_DATA_ID, RC_ARMBOARD_JOINTANGLES_DATA_COUNT, jointAngles);
      last_update_time = millis();
  }
}

void parsePackets()
{
    packet = RoveComm.read();

    Serial.println("Data id: ");
    Serial.println(packet.data_id);

    switch(packet.data_id)
    {
    case RC_ARMBOARD_ARMVELOCITYCONTROL_DATA_ID:
        Serial.println("Open Loop");
        Serial.println(packet.data_id);
        openLoop();
        break;
    case RC_ARMBOARD_LIMITSWITCHOVERRIDE_DATA_ID:
        Serial.println("DoLS");
        Serial.println(packet.data[0]);
        do_ls = packet.data[0];
        digitalWrite(SW_IND_1, do_ls);
        break;
    default:
        break;
    }
    digitalWrite(SW_IND_1, do_ls);
}

void openLoop()
{
  Serial.print("1:");Serial.println(packet.data[0]);
  Serial.print("2:");Serial.println(packet.data[1]);
  Serial.print("3:");Serial.println(packet.data[2]);
  Serial.print("4:");Serial.println(packet.data[3]);
      
  if(Shoulder.atTiltLimit(packet.data[1]) && do_ls)
  {
    Serial.println("Bicep Limit");
    packet.data[1] = 0;
  }

  if(Elbow.atTiltLimit(packet.data[2]) && do_ls)
  {
    Serial.println("Elbow Limit");
    packet.data[2] = 0;
  }
  
  if(packet.data[1] >= 0)
  {
    Shoulder.tiltTwistDecipercent((packet.data[1]), (packet.data[0]));
  }
  else if(packet.data[1] <= 0)
  {
    Shoulder.tiltTwistDecipercent((packet.data[1])*2/3, (packet.data[0])*2/3);
  }

  if(packet.data[0] >= 0)
  {
    Shoulder.tiltTwistDecipercent((packet.data[1]), (packet.data[0]));
  }
  else if(packet.data[0] <= 0)
  {
    Shoulder.tiltTwistDecipercent((packet.data[1]), (packet.data[0]));
  }

  ///////////////////////////////////////////////////////////////////////////////////
  //Same process for elbow as for shoulder
  ///////////////////////////////////////////////////////////////////////////////////
  if(packet.data[2] >= 0)
  {
      Elbow.tiltTwistDecipercent((packet.data[2])*2/3, (packet.data[3])*2/3);
  }
  else if(packet.data[2] <= 0)
  {
      Elbow.tiltTwistDecipercent((packet.data[2])*1/3, (packet.data[3])*2/3);
  }
  if(packet.data[3] >= 0)
  {
      Elbow.tiltTwistDecipercent((packet.data[2])*2/3, (packet.data[3])*2/3);
  }
  else if(packet.data[3] <= 0)
  {
      Elbow.tiltTwistDecipercent((packet.data[2])*2/3, (packet.data[3])*2/3);
  }
  
  Watchdog.clear();
}

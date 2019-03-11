#include "ArmBicep.h"

void setup()
{
  Serial.begin(9600);
  RoveComm.begin(RC_BICEP_FOURTHOCTET);
  Shoulder.LeftMotor.attach(SHOULDER_LEFT_INA, SHOULDER_LEFT_INB, SHOULDER_LEFT_PWM);
  Shoulder.RightMotor.attach(SHOULDER_RIGHT_INA, SHOULDER_RIGHT_INB, SHOULDER_RIGHT_PWM);
  Shoulder.LeftMotor.drive(0);
  Shoulder.RightMotor.drive(0);
  Elbow.LeftMotor.attach(ELBOW_LEFT_INA, ELBOW_LEFT_INB, ELBOW_LEFT_PWM);
  Elbow.RightMotor.attach(ELBOW_RIGHT_INA, ELBOW_RIGHT_INB, ELBOW_RIGHT_PWM);
  Elbow.LeftMotor.drive(0);
  Elbow.RightMotor.drive(0);
}

void loop()
{
  rovecomm_packet = RoveComm.read();
  if(rovecomm_packet.data_id == RC_ARMBOARD_BICEP_DATAID)
  {
    Serial.println(rovecomm_packet.data_id);
    disconnectCount = 0;
    Serial.println(rovecomm_packet.data[0]);
    Serial.println(rovecomm_packet.data[1]);
    Serial.println(rovecomm_packet.data[2]);
    Serial.println(rovecomm_packet.data[3]);
    if(rovecomm_packet.data[0] == 0 && rovecomm_packet.data[1] == 0 && rovecomm_packet.data[3] == 0&& rovecomm_packet.data[4] == 0)
    {
      estop();
    }
    Shoulder.tiltTwistDecipercent((rovecomm_packet.data[1]), (rovecomm_packet.data[0]));
    Elbow.tiltTwistDecipercent((rovecomm_packet.data[2]), (rovecomm_packet.data[3]));
  }
  else if(rovecomm_packet.data_id != RC_ARMBOARD_BICEP_DATAID)
  {
    disconnectCount++;
    if(disconnectCount >= 1500)
    {
      disconnectCount = 0;
      estop();
    }
  }
}

void estop()
{
  Serial.println("WE ESTOPPED");
  Elbow.LeftMotor.drive(0);
  Elbow.RightMotor.drive(0);
  Shoulder.LeftMotor.drive(0);
  Shoulder.RightMotor.drive(0);
}

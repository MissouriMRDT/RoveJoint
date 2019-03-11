#include "ArmForearm.h"

void setup()
{
  Serial.begin(9600);
  RoveComm.begin(RC_FOREARM_FOURTHOCTET);
  Base.LeftMotor.attach(BASE_LEFT_INA, BASE_LEFT_INB, BASE_LEFT_PWM);
  Base.RightMotor.attach(BASE_RIGHT_INA, BASE_RIGHT_INB, BASE_RIGHT_PWM);
  Base.LeftMotor.drive(0);
  Base.RightMotor.drive(0);
  Gripper.attach(GRIPPER_INA, GRIPPER_INB, GRIPPER_PWM);
  Gripper.drive(0);
}

void loop()
{
  rovecomm_packet = RoveComm.read();
  if(rovecomm_packet.data_id == RC_ARMBOARD_FOREARM_DATAID)
  {
    disconnectCount = 0;
    if(rovecomm_packet.data[0] == 0 && rovecomm_packet.data[1] == 0 && rovecomm_packet.data[2] == 0)
    {
      estop();
    }
    Base.tiltTwistDecipercent((rovecomm_packet.data[1]), (rovecomm_packet.data[0]));
    Gripper.drive(rovecomm_packet.data[2]);
    Serial.print("Gripper");
    Serial.println(rovecomm_packet.data[2]);
  }
  else if(rovecomm_packet.data_id != RC_ARMBOARD_FOREARM_DATAID)
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
  Base.LeftMotor.drive(0);
  Base.RightMotor.drive(0);
  Gripper.drive(0);
}

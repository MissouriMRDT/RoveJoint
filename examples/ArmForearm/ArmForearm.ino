#include "ArmForearm.h"

void setup()
{
  //start our communications
  Serial.begin(9600);
  RoveComm.begin(RC_FOREARM_FOURTHOCTET);

  //initialize motors
  Wrist.LeftMotor.attach(BASE_LEFT_INA, BASE_LEFT_INB, BASE_LEFT_PWM);
  Wrist.RightMotor.attach(BASE_RIGHT_INA, BASE_RIGHT_INB, BASE_RIGHT_PWM);
  Gripper.attach(GRIPPER_INA, GRIPPER_INB, GRIPPER_PWM);
  //set motor speeds to 0, to be safe
  Wrist.LeftMotor.drive(0);
  Wrist.RightMotor.drive(0);
}

void loop()
{
  rovecomm_packet = RoveComm.read();
  Serial.println(rovecomm_packet.data_id);
  if(rovecomm_packet.data_id == RC_ARMBOARD_FOREARM_DATAID)
  {
    disconnectCount = 0;
    if(rovecomm_packet.data[0] == 0 && rovecomm_packet.data[1] == 0 && rovecomm_packet.data[2] == 0 && rovecomm_packet.data[3] == 0)
    {
      estop();
    }
    //print telemetry
    Serial.print("Wrist tilt: ");
    Serial.println(rovecomm_packet.data[0]);
    Serial.print("Wrist twist: ");
    Serial.println(rovecomm_packet.data[1]);
    Wrist.tiltTwistDecipercent((rovecomm_packet.data[0]), (rovecomm_packet.data[1]));

    Serial.print("Gripper value: ");
    Serial.println(rovecomm_packet.data[2]);
    Gripper.drive(rovecomm_packet.data[2]);
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

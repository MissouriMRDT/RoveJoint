#include "ArmForearm.h"

void setup()
{
<<<<<<< HEAD
  //start our communications
  Serial.begin(9600);
  RoveComm.begin(RC_FOREARM_FOURTHOCTET);

  //initialize motors
  Base.LeftMotor.attach(BASE_LEFT_INA, BASE_LEFT_INB, BASE_LEFT_PWM);
  Base.RightMotor.attach(BASE_RIGHT_INA, BASE_RIGHT_INB, BASE_RIGHT_PWM);
  Gripper.attach(GRIPPER_INA, GRIPPER_INB, GRIPPER_PWM);
  //set motor speeds to 0, to be safe
  Base.LeftMotor.drive(0);
  Base.RightMotor.drive(0);
=======
  Serial.begin(9600);
  RoveComm.begin(RC_FOREARM_FOURTHOCTET);
  Base.LeftMotor.attach(BASE_LEFT_INA, BASE_LEFT_INB, BASE_LEFT_PWM);
  Base.RightMotor.attach(BASE_RIGHT_INA, BASE_RIGHT_INB, BASE_RIGHT_PWM);
  Base.LeftMotor.drive(0);
  Base.RightMotor.drive(0);
  Gripper.attach(GRIPPER_INA, GRIPPER_INB, GRIPPER_PWM);
>>>>>>> face3f1... Initial code dump, needs restructuring
  Gripper.drive(0);
}

void loop()
{
  rovecomm_packet = RoveComm.read();
<<<<<<< HEAD
  Serial.println(rovecomm_packet.data_id);
  if(rovecomm_packet.data_id == RC_ARMBOARD_FOREARM_DATAID)
  {
    disconnectCount = 0;
    if(rovecomm_packet.data[0] == 0 && rovecomm_packet.data[1] == 0 && rovecomm_packet.data[2] == 0 && rovecomm_packet.data[3] == 0)
    {
      estop();
    }
    //print telemetry
    Serial.print("Base tilt: ");
    Serial.println(rovecomm_packet.data[0]);
    Serial.print("Base twist: ");
    Serial.println(rovecomm_packet.data[1]);
    Base.tiltTwistDecipercent((rovecomm_packet.data[0]), (rovecomm_packet.data[1]));

    Serial.print("Gripper value: ");
    Serial.println(rovecomm_packet.data[2]);
    Gripper.drive(rovecomm_packet.data[2]);
  }
  else if(rovecomm_packet.data_id == 0)
  {
    Serial.println("Zero packet");
    disconnectCount++;
    if(disconnectCount >= 5000)
    {
      Serial.println("ESTOP DUE TO TIMEOUT");
=======
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
>>>>>>> face3f1... Initial code dump, needs restructuring
      disconnectCount = 0;
      estop();
    }
  }
<<<<<<< HEAD
  else
  {
    Serial.println("Data id: ");
    Serial.print(rovecomm_packet.data_id);
  }
=======
>>>>>>> face3f1... Initial code dump, needs restructuring
}

void estop()
{
  Serial.println("WE ESTOPPED");
  Base.LeftMotor.drive(0);
  Base.RightMotor.drive(0);
  Gripper.drive(0);
}

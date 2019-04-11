#include "ArmBicep.h"

void setup()
{
<<<<<<< HEAD
<<<<<<< HEAD
  //start our communications
=======
>>>>>>> face3f1... Initial code dump, needs restructuring
=======
>>>>>>> face3f141de0360c5160b71f2ba15fcf0535faf0
  Serial.begin(9600);
  RoveComm.begin(RC_BICEP_FOURTHOCTET);
  Shoulder.LeftMotor.attach(SHOULDER_LEFT_INA, SHOULDER_LEFT_INB, SHOULDER_LEFT_PWM);
  Shoulder.RightMotor.attach(SHOULDER_RIGHT_INA, SHOULDER_RIGHT_INB, SHOULDER_RIGHT_PWM);
<<<<<<< HEAD
<<<<<<< HEAD
  Elbow.LeftMotor.attach(ELBOW_LEFT_INA, ELBOW_LEFT_INB, ELBOW_LEFT_PWM);
  Elbow.RightMotor.attach(ELBOW_RIGHT_INA, ELBOW_RIGHT_INB, ELBOW_RIGHT_PWM);
  Shoulder.LeftMotor.drive(0);
  Shoulder.RightMotor.drive(0);
=======
=======
>>>>>>> face3f141de0360c5160b71f2ba15fcf0535faf0
  Shoulder.LeftMotor.drive(0);
  Shoulder.RightMotor.drive(0);
  Elbow.LeftMotor.attach(ELBOW_LEFT_INA, ELBOW_LEFT_INB, ELBOW_LEFT_PWM);
  Elbow.RightMotor.attach(ELBOW_RIGHT_INA, ELBOW_RIGHT_INB, ELBOW_RIGHT_PWM);
<<<<<<< HEAD
>>>>>>> face3f1... Initial code dump, needs restructuring
=======
>>>>>>> face3f141de0360c5160b71f2ba15fcf0535faf0
  Elbow.LeftMotor.drive(0);
  Elbow.RightMotor.drive(0);
}

void loop()
{
  rovecomm_packet = RoveComm.read();
  if(rovecomm_packet.data_id == RC_ARMBOARD_BICEP_DATAID)
  {
<<<<<<< HEAD
<<<<<<< HEAD
=======
    Serial.println(rovecomm_packet.data_id);
>>>>>>> face3f1... Initial code dump, needs restructuring
=======
    Serial.println(rovecomm_packet.data_id);
>>>>>>> face3f141de0360c5160b71f2ba15fcf0535faf0
    disconnectCount = 0;
    Serial.println(rovecomm_packet.data[0]);
    Serial.println(rovecomm_packet.data[1]);
    Serial.println(rovecomm_packet.data[2]);
    Serial.println(rovecomm_packet.data[3]);
<<<<<<< HEAD
<<<<<<< HEAD

    if(abs(rovecomm_packet.data[0]) < 50 && abs(rovecomm_packet.data[1]) < 50 && abs(rovecomm_packet.data[3]) < 50 && abs(rovecomm_packet.data[4]) < 50)
    {
        estop();
    }
    if(abs(rovecomm_packet.data[0]) >= 50 || abs(rovecomm_packet.data[1]) >= 50)
        Shoulder.tiltTwistDecipercent((rovecomm_packet.data[0]), (rovecomm_packet.data[1]));
    if(abs(rovecomm_packet.data[2]) >= 50 || abs(rovecomm_packet.data[3]) >= 50)
        Elbow.tiltTwistDecipercent((rovecomm_packet.data[2]), (rovecomm_packet.data[3]));
  }
  else if(rovecomm_packet.data_id == 0)
  {
    //Serial.println("Zero packet");
    disconnectCount++;
    //Serial.println(disconnectCount);
    if(disconnectCount >= 15000)
    {
      //Serial.println("Greater than 5000");
=======
=======
>>>>>>> face3f141de0360c5160b71f2ba15fcf0535faf0
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
<<<<<<< HEAD
>>>>>>> face3f1... Initial code dump, needs restructuring
=======
>>>>>>> face3f141de0360c5160b71f2ba15fcf0535faf0
      disconnectCount = 0;
      estop();
    }
  }
<<<<<<< HEAD
<<<<<<< HEAD
  else
  {
    Serial.println("Data id: ");
    Serial.println(rovecomm_packet.data_id);
  }
=======
>>>>>>> face3f1... Initial code dump, needs restructuring
=======
>>>>>>> face3f141de0360c5160b71f2ba15fcf0535faf0
}

void estop()
{
  Serial.println("WE ESTOPPED");
  Elbow.LeftMotor.drive(0);
  Elbow.RightMotor.drive(0);
  Shoulder.LeftMotor.drive(0);
  Shoulder.RightMotor.drive(0);
}

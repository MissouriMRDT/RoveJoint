#include "ArmBoard.h"

void setup()
{
  Serial.begin(9600);
  RoveComm.begin(RC_ARMBOARD_FOURTHOCTET);
}

void loop()
{
 rovecomm_packet = RoveComm.read();
 Serial.println("Current id: ");
 Serial.println(rovecomm_packet.data_id);
 if(rovecomm_packet.data_id == RC_ARMBOARD_MOVEOPENLOOP_DATAID)
 {
   int16_t bicepVals[4];
   int16_t forearmVals[3];
   Serial.println("We have a packet");
   Serial.println("Entry 1");
   Serial.println(rovecomm_packet.data[0]);
   Serial.println("Entry 2");
   Serial.println(rovecomm_packet.data[1]);
   Serial.println("Entry 3");
   Serial.println(rovecomm_packet.data[2]);
   Serial.println("Entry 4");
   Serial.println(rovecomm_packet.data[3]);
   Serial.println("Entry 5");
   Serial.println(rovecomm_packet.data[4]);
   Serial.println("Entry 6");
   Serial.println(rovecomm_packet.data[5]);
   //Because of the way that the joints are mapped (see #insert documentation)
   //we have to reorder our inputs slightly
   if(rovecomm_packet.data[0] >= 200)
   {
      bicepVals[0] = 400;
   }
   else if(rovecomm_packet.data[0] <= -200)
   {
      bicepVals[0] = -400;
   }
   if(rovecomm_packet.data[1] >= 200)
   {
      bicepVals[1] = 400;
   }
   else if(rovecomm_packet.data[1] <= -200)
   {
      bicepVals[1] = -400;
   }
   if(rovecomm_packet.data[3] >= 200)
   {
      bicepVals[2] = 400;
   }
   else if(rovecomm_packet.data[3] <= -200)
   {
      bicepVals[2] = -400;
   }
   if(rovecomm_packet.data[2] >= 200)
   {
      bicepVals[3] = 400;
   }
   else if(rovecomm_packet.data[2] <= -200)
   {
      bicepVals[3] = -400;
   }

   if(rovecomm_packet.data[5] >= 200)
   {
      forearmVals[0] = 700;
   }
   else if(rovecomm_packet.data[5] <= -200)
   {
      forearmVals[0] = -700;
   }

   if(rovecomm_packet.data[4] >= 200)
   {
      forearmVals[1] = 700;
   }
   else if(rovecomm_packet.data[4] <= -200)
   {
      forearmVals[1] = -700;
   }

   if(rovecomm_packet.data[6] >= 200)
   {
      forearmVals[2] = 300;
   }
   else if(rovecomm_packet.data[6] <= -200)
   {
      forearmVals[2] = -300;
   }

   //bicepVals[0] = rovecomm_packet.data[0]; //J1
   //bicepVals[1] = rovecomm_packet.data[1]; //J2
   //bicepVals[2] = rovecomm_packet.data[3]; //J3
   //bicepVals[3] = rovecomm_packet.data[2]; //J4
   //forearmVals[0] = rovecomm_packet.data[5]; //J5
   //forearmVals[1] = rovecomm_packet.data[4]; //J6
   //forearmVals[2] = rovecomm_packet.data[6]; //J6

   RoveComm.writeTo(RC_ARMBOARD_BICEP_DATAID, 4, bicepVals, 192, 168, 1, RC_BICEP_FOURTHOCTET, 11000);
   RoveComm.writeTo(RC_ARMBOARD_FOREARM_DATAID, 3, forearmVals, 192, 168, 1, RC_FOREARM_FOURTHOCTET, 11000);

 }
}

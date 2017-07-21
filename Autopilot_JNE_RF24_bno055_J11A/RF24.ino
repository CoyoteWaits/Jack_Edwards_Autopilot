#if RF24_Attached == 1
 /*  RADIO RF24 */
 // based on formatted data sketch
 /* 
Sending formatted data packet with nRF24L01. 
Maximum size of data struct is 32 bytes.
 contributted by iforce2d

The wire numbers listed are using a ribbon wire and a two row ribbon wire connector
1 - GND, wire 2
2 - VCC 3.3V !!! NOT 5V, wire 1
3 - CE to Arduino pin 9, wire 4
4 - CSN to Arduino pin 10, wire 3 
5 - SCK to Arduino pin 13 for Uno, 52 on Mega, wire 6
6 - MOSI to Arduino pin 11 for Uno, 51  on Mega, wire 5
7 - MISO to Arduino pin 12 for Uno,  50 on Mega, wire 8
8 - UNUSED, wire 7
*/

void sendData1() 
{
 // test data to be commented out
 //Waypoint_next = "test point";
 //course = 456;
 // end test data
 
 RFdata.RFD_set = 1;
 Waypoint_next.toCharArray(RFdata.RFD_text,8);
 RFdata.RFD_float1 = heading;
 RFdata.RFD_float2 = heading_to_steer;
 RFdata.RFD_float3 = course;
 RFdata.RFD_float4 = course_to_steer;
 RFdata.RFD_float5 = Steering_Mode;
 
 radio.stopListening(); // stop listening so we can send data 
 radio.write(&RFdata, sizeof(RF_DATA));  
 radio.startListening(); // resume listening

 // test data
 //XTE = 15.6;
// Bearing_origin_to_destination = 168;
// Bearing_to_destination = 179;
// MSG=1;
 
 RFdata.RFD_set = 2;
 Mode.toCharArray(RFdata.RFD_text,8);
// RFdata.RFD_float1 = XTE;
 RFdata.RFD_float1 = bearingrate;  // temporary displays bearing rate on RF remote where XTE is programed
 RFdata.RFD_float2 = Bearing_origin_to_destination;
 RFdata.RFD_float3 = Bearing_to_destination;
 RFdata.RFD_float4 = MSG;
 #if Compass == 1
 RFdata.RFD_float5 = float(bnoCAL_status);
 #endif
 radio.stopListening(); // stop listening so we can send data 
 radio.write(&RFdata, sizeof(RF_DATA));  
 radio.startListening(); // resume listening
}
/********************************/
/*
void sendData2() 
{
// test data comment out
SOG = 6.2;
XTE = 123;
Bearing_origin_to_destination = 234;
Bearing_to_destination = 245;
Range_Destination = 2.3;
course_to_steer = 241;
// end test data
 RFdata2.RFdata_set = 2;
 RFdata2.SOG = SOG;
 RFdata2.XTE = XTE;
 RFdata2.Bearing_origin_to_destination = Bearing_origin_to_destination;
 RFdata2.Bearing_to_destination = Bearing_to_destination;
 RFdata2.Range_Destination = Range_Destination;
 RFdata2.course_to_steer = course_to_steer;


 radio.stopListening(); // stop listening so we can send data 
 radio.write(&RFdata2, sizeof(RF_DATA2));
  
 radio.startListening(); // resume listening
}
*/
/*********************************************/

void Recv_Data()
{
       if ( radio.available()) 
  {
   // Serial.println ("radio Available");
    radio.read( &KeyIn2, sizeof(KeyIn2) );
   // Serial.print("KeyIn2 = ");  Serial.println(KeyIn2);
    KeyPressed(KeyIn2);
    
  } 
}
/*******************/

#endif




/*****************************************************************************
TERMS AND CONDITIONS

THE USER OF THIS SOFTWARE, WHICH IS MADE AVAILABLE FREE, ACCEPTS ALL RESPOSIBILTY ASSOCIATED WITH ITS USE.
SOFTWRE ERRORS OR HARWARE PROBLEMS CAN CAUSE THE BOAT TO MAKE SUDDEN TURNS OR FAIL TO MAKE EXPECTED TURNS.
THE BOAT OPERATOR MUST HAVE A POSITIVE MEANS TO REGAIN MANUAL CONTROL IF A PROBLEM OCCURS.   UNDER CONDITIONS 
WHERE A SUDDEN TURN OR LACK THERE COULD CAUSE A HAZARD  THE SOFTWARE SHOULD NOT BE 
LEFT TO CONTROL THE BOAT WITHOUT THE OPERATOR BEING PRESENT. 

This software wa developed as an experiment by the author to provide an auto pilot for his boat.  It is made available
to others who want to develop a Do It Yourself autopilot.  The USER acepts all responsibilty for the safe operation of
the boat recognizing that hardware and software erros can occur.  The User also acknowledges that it is their responsibily to
safely wire and install the autopilot components in accordance wire appropriate codes and standards.

THIS AUTOPILOT SOFTWARE AND ASSOCIATED HARDWARE DESCRIPTIONS IS MADE AVAILABE UNDER CREATIVE
COMMONS LICENSING

*****************************************************************************************************************************/

// V29 started at 25 to experiment with ways to get stable operation.  This version seems to be working,  It has the print routines
// commented out, not sure why that works,  must be a timig issue.
// V29B added compass to display on LCD
// V29C, try to handle GPS not available or route go to waypoint not available. 
//Auto pilot 4_0 is working model.
//Autopilot 4_1 adds keypad and event handler
// Autopilot 4_2 adds rudder signals from PID to operate rudder relays for n seconds less than +/- 5 from center
// A/P 4_3 will addXTE for integral error and compass rate for differential error
// A/P 4_6 Starting at 4_3, 4_6 will change  control from rudder motion to rudder position See Void PID 
// A/P 4_7 based on 4_6 update to Arduino 1.0.  elimnate type byte replace with int or char
//A/P 4_9 based on 4_7 to integrate the MinIMU9 gyro compass, also moved subroutines to TABS.
// A/P 4_10 based on 4_9 willput GPS input to separate Arduino because gps was slowing down MinIMU9.
//  testing shows separate processing will transfer needed data in 1 milli()
// A/P 10 to 13 incorporated bearing rate from gyro vertical component and reworked PID
// A/P 14 add Simulation
// AP 15 Change rudder control to eliminate use of delays. instead sets rudder command turns rudder on then updates position 
      //AP 15 (cont'd)   each cycle until rudder position  = rudder command.
// AP 16 Revise the GPS Steering mode and Simulator to test GPS steering
// AP 17 screwed up
// AP 18 -  AP 16 is working on Compass and GPS simulation but LCD display has conflicts AP is to fix that
// AP 19 working version after AP 16 
// AP 20 update keypad to switch modes 4/3/2012
// AP 21 pull out the verbose keypad with Heirarchial screens in v20 and use AutopilotKeypad v1 with each key having a specific function
// AP 22  21 was working 22 is trying to rudder control working with dodge keys
// AP 23 add Tack as Mode 3 change HTS by tack angle.  Add Magnetic variation so HDGs are True
// AP 24 makes simulation mode available from keypad (HOLD key 5)
// AP 25  add limits to rudder during TACK maneuver;
// AP 26 added UTC seconds print to LCD comes from GPS should count to 60 shows if GPS data is updating. moved get_GPS to fast loop.
// AP 26 con't also move the Easy Transfer "get_GPS() from slow loop to fast loop to read data more frequently
// AP 4_27 commented out waypoint next in Data Structure and Data recieve to stopp chrashes while looking for alternative data transfer
// AP 5_0 fixed data structure crashes and 5_0 is the go to the boat Test Model.
// AP 5_1 added heading_error_max to limit rudder for large course changes. added sw2 function as helm steering permissive switch.
// AP 5_2 Last version tested on the boat in August 2012, don't know what changes from 5_1.
// AP 5_3 Editorial Changes back home 11/2012.
// AP 5_3_1 Editorial changes
// Ap 5_3_2 Switched over to rudder position indicator for feed back instead of tracking rudder on times
// AP 5_3_3 modified rudder command structure
// AP 5_3_4 further mods to rudder command coding
// AP 5_3_5 Added data structure to receive new data for GPRMB
// AP 5_4_2 Working version with GPRMB,and good data ET Transfer, Added screen 0/1 on key 5 
// AP 5_4_4 Added UTC Timer, void Date_Time To track on LCD running time and max Dt between ETdata updates
// AP 5_4_5 added XTE control to Steer GPS and new ETdata XTE_differential _error. Compute DXTE/DT in GPS_JNE
// AP 5_4_7 Started revising simulation in 5_4_6 and 5_4_6_1.  this version is revising simulation anew from 5_4_5 
// AP 5_4_7_1 modified KeyPad 5/19/13  ** This is starting point on boat June 2013 **
// AP 5_4_7_2 installed rudder indicator on boat aand rewrote Rudder_Position() calibration
// AP 5_4_7_3  put tracking erro in separate subroutine and update it continuously.
// AP 5_4_7_4 rearranged A_P_Loop, added logic to turn rudder off after n ms to slow down response, testedin compass mode worked great
// AP 5_5 New baseline, same as 5_4_7_4 tested working version, compiled in the Knob gain for this version to lock in this working version
// AP 5_5_1 play with rudder deadband and timing
// AP 6_0 same as 5_5_1 but includes new Pololu MinIMU9 v1.2.1 software to support MinIMU9 V2 chip
// AP 6_1 same as 6_0 but change commands from Wagerner motor controller to Pololu controller i.e. digital.write(12, low) becomes Serial2.write(0xc2) etc
// AP 6_2 6_1 had a constant motor speed and rudder off after duration. 6_2 has variible rudder speed and no off time rudder modes added
// AP 6_2_1 integrating rudder control fro PWM_3 into AP and revising to accommodate input from Garmin GPSmap 198C (has no GPAPB sentence 10/6/13
// AP 6_2_2 10/10/13 restructuring GPS steering for course to steer and heading to steer
// AP 6_2_3 10/24/13 contiuning with 6..2.2 changes just want to sae 622 as a go back point.  Added smoothed bearing rate
// Note SW2 is not being used I am tking out the actuating code but there maybe some comment references I didn't get removed
// AP 6_3 add knob steering
// AP 6_4 minor tweaks
// AP-6_5 add second startup execution to reduce IMU9 gyro biasing when powered up
// AP 6_6_5_1 Modified Print NAV DATA for GPS Steering analysis, added maximum rudder angle and speed for tacking, user input, keypad4/6
// AP 6_6_5_2 changed rudder indication positive direction
// AP 6_7 renamed 6_6_5_2.  Did sea trials with reinstalled equipment and all modes working including GPS with cross track error correction. 11/22/13
// AP 6_7_1 same as above but add code to limit cross track error correction for a minute when steering to new waypoint to eliminate oversteer with big course change See GPS_Steer()
  // this version sea tested Nov 2013.All modes working.  GPS is rough in the turns goes through the waypoint then has to double back
// AP 6_7_3 same as 671 added limit on newwaypoint rudder max, modified Tack rudder max implementation.
// AP 6_7_4  changed the rudder controls so user can select between Pololu Trex Controller or Pololu Qik controller
// AP 6_7_5 added line 145 PID to set rudder position to zero in RUDDER_MODE 1
// AP 6_7_6 Added Terms and conditions shown above and added start up screen requiring acceptance
// AP 6_7_6_1  Added Serial Keypad remote
// AP 6_7_6_2 Added Easy Transfer send data to the remote,  Can now send an receive data between remote and Main AP Mega.
// AP 6_7_6_3 corrrected errors in Dodge Mode
// AP 6_7_6_4 deleted
// AP 6_7_6_5 Added the key released function to the Remote Serial KeyPad 1/7/2014
// AP 6_7_6_6  Added Print Motor Commands to print motor controller command open/ close solenoid/ L/R rudder and Rudder stop,ported Lat and Lon over 
             //  Must be used with GPS_JNE_5_5_7_2 or later
// AP B1_0  Same as 6_7_6_6 plus a switch to turn off serial remote since it screws up AP if not plugged in.  
// the B series in both AB and GPS are adopted because they must go together for easy transfer to work after I added
// transfer of Lat and Lon
// AP B1_2 started from B1_0 update for Pololu latest libraries and IMU code 2/2/14 for library nov 2013. 
    // Changed LCD print screen 3 from MAGVAR to Magnetic_Variation. (uses default or GPS input if GPRMC Fix is valid) 
// AP B1_2b added Key "0" function to include reset screen to 0  Added Next turn, deleted LAT/LON (too much data)
// AP B1_2d  add data fo easy transfer to serial remote
// AP C1 Use with C series of GPS and Serial Remote got the serial remote TFT working.  Changed how I get Easy Transfer data
      // use if(ET.receiveData()) before trying to read data.
// AP C3 Took out the stop rudder every loop to cut down inductive kickback load on motor controller
// AP C4 In GPS Steer make default on waypoint arrival to continue on course.  Auto turning to new course  is an option.
// AP D extension of C4. Added in MSG to eztransfer to send a message ID compactly to display messages on TFT
// AP D4, D5 updated MSG handling for GPS available or lost. 
// AP E same as D5 numbering to keep uniform with Serial Remote
// AP F Jack's experimental with hooks for wind steering
// Fa tested well for GPS steering. turn anticipation turned off in GPS F3
// Used with GPS F3 which uses Delta lat, Delta lon to compute XTE more accurately which cut out the lumpyness caused by XTE changing by 15 ft
// Fa2 Added Kxte integral .0005. Fa tested well with GPS G0 comming down from swinomish, now try XTE integral correction. Serial remote baud to 19200
// Fa3 same as Fa2 but testing variousKxte coefficients
// Fa4 change XTE to be based on BOD instead of BRG
// Fa5 set Xte_correction and XTE_integral_error to zero when key 0 pressed
// AP_JNE_G  Same as Fa5
// G1 constrain integral error +/- 15, varying Kxte, Kxte differentila, sign XTE differential = sign Dxte/Dt, change the Key 0 reset to Key 2 reset of XTE integral
// G1 final version summer 2014 tested in heavy weather in GPS mode crossing wind and current
// IMUv3 G1 updated code so it could use IMU V3 lbrary.  Use the New I2C tab, other tabs are the same.
// Note  This version needs to use the  L3G and LSM303 libraries from IMU_LIBRARY_2015 
// IMUv3 G2 added Bearingrate_correction as user input
// RF version attempt to transmitt the eazy transfer data via wifi
// Version IMUv3_RF24_G2_3 has the RF remote working to send data and receive keypad input
// IMUv3_RF24_G3 Saved previous working version of RF24 and added code to smooth GPS course and use in GPS steering
// G4 saved working version making changes to improve GPS, changed MSG structure to alwaways print see print1 screen 0, adding second RFDATA2
// G4c saved working RF version
// G4d deleted the serial remote call and reference to see if it was causing instability
// G4e brought in course to steer from APB and called it GPS_course_to_steer and will evaluate the effect of using it instead of course to steer based
//   my lengthy derivation of course to steer based on bearing and cross track error
// this worked fabuously with the Garmin 740s whichgenerates a cross track error corrected course to steer 9/21/15
// H1 saved the previous G4e as default version 
// H3 got Anticipated turning working
// H4 same as H3 now trying to add #define code turn turn various features on at the compile stage
// H4B  added #define IMU to set which dcalibration is used
// H4C add code for Pololu Simple Controller modify rudder control to be more general in term max min rudder speed Added parameter Clutch_Solenoid to turn off trying to 
//    write to the solenoid if it was not attached because it could have unforseen issues to motor controller if not specifically attached
// H4D  took out waypoint timer, dead reckon stuff which was replaced with anticipated turns, deleted a lot of old commented out stuff
// H4E going to add steering mode GPS2 which will be like compass steer but will capture current COG as CTS and maintain it use key hold to activate
//   In GPS2 mode the CTS is set = to the current COG it can then be incremented up and down with keys 4, 6, 7 and 9 like compass course
// J versions put #define COMPASS in to select between Pololu IMU9 and Bosch BNO055.  So far Test J2 has BNO055 heading working but does not yet have bearing rate.
// J3 added BNO055 calibration restore from EEPROM
// J6 changed RF inherited printf statements to regular print statements. Took out test designator and posted to DropBox a current version. 
// J7  House cleaning
// J8 J9 rearranged some of the user input to get important things up front, Add PWM LCD contrast control to eliminate 10K pot for contrast,  added user option to not use SW2
   //  Added #define GPS_Used 0/1 0 leaves out the easy transfer library and GPS functions though GPS variables are still defined
// J9a 3.20.16 moved BNO campass cal status to Screen 3(fourth screen) had conflict on screen 0 with rudder position
//   put in a user note to set motorspeedMIN
//  J9a saved as working model tested 3.25.16 with GPS H6.  tested GPS and compass using Pololu IMU9v3 and using CTS from Garmin GPS 740 saved as working version.
// J10 starting with J9a to work on BNO055 compass.
//  Note BNO055 compass not working it did not update rapidly and bearing rate juped around between 0 and twice the rate from Pololu IMU9v3
// J10 was tested in anticipated turn mode and worked well.
// J11 same as J10.  starting J11 so J10 is archived as the working tested version. Revised Minimum Rudder Speed. 
// J11 Increase frequency of reading the compass (line 57 A_P_Loop) if counter > 8 changed to if Counter > 1 (reads every other fast loop) 
// J11 RF remote Update frequency is slow need to check. J11 Notes revised 5/14/16 prior to posting.
/*J11A Added note to USER INPUT for user to verify Keypad Definitions. Keypad wiring changes required to support RF and is different than EventKeypad
 * /
 */
 
 #include <Keypad.h>
 #include <LiquidCrystal.h>
 #include <Wire.h>

/******       USER INPUT       ********/

#define Compass 1  //  0 for Pololu, 1 for BNO055
#define GPS_Used 0 // 1 to include GPS code, 0 to exclude it
#define Motor_Controller 2  // 1 for Pololu Qik dual controller, 2 for Pololu Trex dual controller, 3 for Pololu Simple Controller
#define Clutch_Solenoid 1 // 1 a clutch solenoid is used, 0 is not used, currently clutch solenoid does not work with single Simple Pololou Controller
#define RUDDER_MODE 1 // 0 uses rudder position, 1 does not
#define RF24_Attached 0 // 0 if RF 24 radio modules are not attached, 1 if they are used
#define IMU 3 // IMU version 2 or 3  determines which set of calibration data is used

 // float PID_Ks[4] = {.75,.4,.01,0}; // use with rudder_command = rudder_command  + PID_output;
 float PID_Ks[4] = {2, .4, 1, .000};  // [ K(overall), K(heading error), K(differential error), K(integral error)] // use with rudder_command = PID_output
 // when tested summer 2013 used 2, .4, 4, 0.  Used {2, .4, 1, .000} in 2015. Used {1, .4, 2, .000} operationg without Rudder indicator
 int PID_MODE = 0; // See description PID tab.

 boolean Accept_Terms = 1; //  1 user must accept terms to run.  0 user accepts terms by setting this value to 0
 boolean just_started = 1; // to do a second setup so get a better gyro startup
 //float Kxte[3] = {0, 0, 0}; // used for XTE PID, use this to zero out XTE tracking
 float Kxte[3] = {.36, 0, 0}; // {.2, 4, .0004} baseline; {.05, .5, .0005}last used;  0 is proportional, 1 is differential, 2 is integral error, see GPS_Steer() in PID
                        // .36 will give 45 deg correction at 120 ft XTE to emulate my Garmin GPSMAP 740s see PID tab, voidActual_Gps_Steering()
 float K_course_avg = .999; //used to smooth gps course in PID/ void Actual_GPS_Steering().999 will smooth over 1000 points
 int Maximum_Rudder = 42; // rudder in degrees
 // User set motorspeedMIN around lines 359, 360, 371 for your controller type and rudder steering motor Use crtl-F to find motorspeedMIN 
 float Tack_Angle = 100;  // angle throug which boat will tack when tack L or R pressed (keys 4 and 6 in TACK mode(3)
 int Tack_rudder_MAX = 32;// limits rudder so it doesn't slow boat too much,  need to tune
 float Tack_rudder_speed = .5; // rudder speed during tack , value * full speed, will use min of tack speed and regular speed, user adjust
 float MagVar_default =18.4;// 18.4 Seattle   User should keep up to date for loaction.  Pgm will use GPS value if available + east, - west
 boolean Serial_Remote_Is_Connected = 0  ; // 1 remote connected, 0 remote not connected Pgm hangs up if remote data is sent and remote is not connected
 boolean GPS_Auto_Steer = 1; // 0 will cause GPS steering to go to compass mode and maintain heading if waypoint reached. 
                             //  1 will cause GPS to automatically steer to next waypoint.  NOTE THIS CAN BE DANGEROUS
 boolean Use_CTS = 0 ;// 0 no, 1 yes;  if the GPS generates a GPS course to steer and you want to use it instead of calculating course to steer from BOD and XTE                            
 boolean wind_instrument_available = 0; // 0 not available, 1 is available    
 float bearingrate_correction = 0.0; //use to get avg stationary bearing rate to read 0 on screen 2

 #define LCD_Contrast 100  // 0 to 255 over 128 not recommended This replaces the 10k pot for V0 contrast control input
 #define SW2_Used 0 // 0 if SW not used. 1 if SW2 used, must be 1 to use SW2 on wired remote
 
 int print_level_max = 3;  //  0 to 4, used to set how much serial monitor detail to print
 // 0 = none, 1=PID Output, 2 Adds parsed GPS data, 3 adds raw GPS input, 4 adds checksum results
 int print_time =5;  // print interval in sec
 boolean Print_ETdata = 0; //prints GPS incoming data turn this off to see actual loop time
 boolean Print_ETtime = 0;  // prints Easy Transfer loop time 1 on 0 off
 boolean Print_heading  = 0 ; // diagnostic serial print heading, interval set in A_P_loop
 boolean Print_LCD_IMU9 = 0;  //prints Head, Pitch, Roll, Yaw on LCD conflicts with other LCD Prints
 boolean Print_LCD_AP = 1; // prints main A/P LCD output data and Menus, only do one of these at a time
 boolean Print_Gyro = 0; //  Prints LCD and Serial scaled X(roll), Y(pitch), Z(Yaw) deg/sec
 boolean Print_PID = 0;
 boolean Print_UTC = 0;
 boolean print_Nav_Data = 0; // Print_1 Tab
 boolean Print_Motor_Commands = 0;  // prints rudder commands in PID tab
 boolean Print_Anticpate_Turn = 0;  // prints data from void Actual_GPS_Steering to evaluate Anticipate turn function
 int print_level=print_level_max;
//  print modes for MinIMU9
/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1
#define PRINT_DATA 0 // 1 to print serial data or run python display
#define PRINT_EULER 0  //Will print the Euler angles Roll, Pitch and Yaw, needed for python display
//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data

 //********************************

/******  COMPASS  ***************/
#if Compass == 0
 #include <L3G.h>  // get library from Pololu http://www.pololu.com/product/1268
 #include <LSM303.h>  // get library from Pololu, use version that corresponds to your version of the IMU
#endif

#if Compass == 1
 #include <Adafruit_Sensor.h>
 #include <Adafruit_BNO055.h>
 #include <utility/imumaths.h>
 #include <EEPROM.h>
  Adafruit_BNO055 bno = Adafruit_BNO055(55);
 #define BNO055_SAMPLERATE_DELAY_MS (100)
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;
  bool DataStored = false;
  int bnoCAL_status;  //used to send 4 digit cal status to RF remotes
#endif

#if GPS_Used == 1
  #include <EasyTransfer.h> // get library from Bill Porter http://www.billporter.info/2011/05/30/easytransfer-arduino-library/
#endif
/** RADIO RF24 DECLARATIONS ***/

#if RF24_Attached == 1
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
//  radio CE,CS pins
RF24 radio(9,10);
//  RF DATA STRUCTURE 
// The sizeof this struct should not exceed 32 bytes
// Change this when uploading to each Arduino

boolean sender = 1;// 1 is trnsmitter and 0 is receiver
int RFdata_set;
// Print some info to serial after this number of packets
unsigned long printRate = 100;

char KeyIn2; // to test key data from other radio
 // RF24 packet size limit 32 bytes
 
   struct RF_DATA
     {
      // total data per packet is 32 byte. int = 2, float = 4, + char text
      int RFD_set;  // read this value to determine data set
      char RFD_text[8];
      float RFD_float1;
      float RFD_float2;
      float RFD_float3;
      float RFD_float4;
      float RFD_float5;
      
      //float heading;
      //float heading_to_steer;;
      //char Mode[5];
      //int MSG;  
     }; // end RF_DATA1 structure    
    RF_DATA RFdata; //NAME THE DATA STRUCTURE

// End Radio Declarations  
#endif

// LCD library code:
// initialize the LCD with the numbers of the interface pins FOR LCD pins (RS,E,D4,D5,D6,D7)
LiquidCrystal lcd(41,43,45,47,49,39);
#define LCD_Contrast_Pin 7 // this pin can be used to control to LCD V0 for for contrast control

//  KEYPAD SETUP
const byte ROWS = 4; //four rows
const byte COLS = 3; //four columns
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
//byte rowPins[ROWS] = {23,25,27,29,}; //connect to the row pinouts of the keypad
//byte colPins[COLS] = {31,33,35}; //connect to the column pinouts of the keypad
byte rowPins[ROWS] = {34,44,42,38}; //connect to the row pinouts of the keypad  JACK'S KEYPAD
byte colPins[COLS] = {36,32,40}; //connect to the column pinouts of the keypad  JACK'S KEYPAD
//byte rowPins[ROWS] = {25,35,33,29}; //Use these with Fritz Diagram 6C, note 9 less than jack's
//byte colPins[COLS] = {27,23,31}; //Use these with Fritz Diagram 6C


char key = 0;
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
String string1 = "";
//char input[15];
//String string2 = "";
//String string3 = "";
//String Kname;
//int iKname;
//float Kvalue;
//const int motorspeed = 32; // 0 t0 127  // use with pololu motor controller

 //Servo myservo;  // used to operate a servo to simulate the rudder
 float Magnetic_Variation; 
 float heading_error =0;
 float differential_error = 0;
 float integral_error = 0; 
 float deadband;  // steering deadband
 float rudder_position = 0;
 float rudder_command = 0;
 float rudder_error;
 int motorspeed;  // sets rudder motor speed in Pololu controller can be constant or variable
 int rudder_MAX; //allows rudder to be changed for various maneuvers like TACK
 boolean TACK_ON = false;  // goes on for a tack and off when within 10 deg final course
// float rudder_total_time;
 float bearingrate=0;
 float bearingrate_smoothed;
 float bearingrate2;
 unsigned long bearingrate2_time;
 float heading_old;
 float delta_heading;
 long delta_compass_time; // computed in compass tab, used in PID for integral error
 float PID_output = 0; 
// float GPS_PID = 0;
 boolean GPS_Steering = false;
 int Steering_Mode = 0;
 String Mode = "OFF";
 String Previous_Mode;
 boolean Steering = false;
 boolean sw1_turned_on = false;
 boolean sw1 = false;
 boolean sw2 = false;
 boolean sw1Status = 1;
 boolean DODGE_MODE = false;
 int Screen=0;
 boolean rudder_on;
 boolean rudder_was_off;
 unsigned long rudder_time_old;
  
  #if Motor_Controller == 1  //  Motor controll parameters, 1 for Pololu Qik Dual controller
   int Motor_0_fwd = 137;
   int Motor_0_rev = 139;
   int Motor_1_fwd = 141; //change these to reverse left and right rudder
   int Motor_1_rev = 143;
   int motorspeedMIN = 30; // this value is the minimum speed sent to controller if left or right rudder is commanded
                            //  rudder stop will still send 0. Use to overcome starting torque. Set so if rudder error greater than the deadband the rudder
                            //  moves at a noticable but slow speed.  Higher values will be more responsive.  
     int  motorspeedMAX = 127;
 #endif
 
 #if Motor_Controller == 2  //  Motor controll parameters, 2 for Pololu Trex Dual controller
   int Motor_0_fwd = 202;
   int Motor_0_rev = 201; 
   int Motor_1_fwd = 194; //change these to reverse left and right rudder
   int Motor_1_rev = 193;
   int motorspeedMIN = 30; // this value is the minimum speed sent to controller if left or right rudder is commanded
                            //  rudder stop will still send 0. Use to overcome starting torque. Set so if rudder error greater than the deadband the rudder
                            //  moves at a noticable but slow speed.  Higher values will be more responsive. 
   int motorspeedMAX = 127;
 #endif
 
  #if Motor_Controller == 3 // Pololu Simple controller
  // int Motor_0_fwd = 202; // motor 0 is the clutch solenoid
  // int Motor_0_rev = 201; 
   int Motor_1_fwd = 133; // change these to reverse left and right rudder
   int Motor_1_rev = 134;
   int motorspeedMIN = 555; // this value is the minimum speed sent to controller if left or right rudder is commanded
                           //  rudder stop will still send 0. Use to overcome starting torque. Set so if rudder error greater than the deadband the rudder
                           //  moves at a noticable but slow speed.  Higher values will be more responsive. 
   int motorspeedMAX = 3200;
 #endif
 
float heading;
int PT_old = 0; // for pritnt timer
boolean GPRMC_fix = false;
boolean GPAPB_fix = false;
String GPS_status= "NO GPS";
float course;
float Avg_course;
float course_to_steer; 
float CTS_GPS2;
float GPS_course_to_steer;
String CTS_MorT = "";
float SOG; 
float course_error;
float Lat_current;
float Lon_current;
//float tracking_error;
float AVG_tracking_error;
float MagVar; // Magnetic Variation East is plus and West is Minus
float heading_to_steer=0; //  see PID
float XTE;
float XTE_differential_error;
float XTE_integral_error;
float XTE_course_correction;
String XTE_LR="";
String XTE_unit="";
String Waypoint_next="";
String Origin_Waypoint;
float Bearing_origin_to_destination=0;
String BOD_MorT = "";
float Bearing_to_destination;
float Range_Destination;
float Next_Turn;
String UTC_string;
long UTC;
//long UTC_start;
unsigned long UTC_timer;
unsigned long UTC_timer_old;
//long Date;
float float1;
long long1;
 char *brkb, *pEnd;
 float Time_decimal;
 float Time_decimal_old;
 float Time_decimal_delta;

 boolean GPS_Available = 0;
 boolean GPS_Was_Available = 0;
 int MSG = 0; // message to send number for message display on serial remote

 
 
 boolean toggle = false;
/*************************************************/
//float system_cal; float gyro_cal;  float accel_cal; float mag_cal;
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
unsigned int counter=0;
unsigned int counter2=0;
unsigned long counter3=0; // used for 10 minute print interval in A_P_Loop
float roll;
float pitch;
float yaw;

#if Compass == 0
// setup data for MinIMU9 from Tab minIMU9AHRS of Pololu software
int SENSOR_SIGN[9] = {1,-1,-1, -1,1,1, 1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168
// LSM303 accelerometer: 8 g sensitivity
// 3.8 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 
// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board


// this is data for Jacks IMU9V2,
#if IMU == 2 // Jack's version 2 IMU calibration 
  #define M_X_MIN -663   
  #define M_Y_MIN -683
  #define M_Z_MIN -611   
  #define M_X_MAX 453
  #define M_Y_MAX 427
  #define M_Z_MAX 460 
#endif


#if IMU == 3 // This is the data for Jacks IMU9V3  
  #define M_X_MIN -3350   
  #define M_Y_MIN -3276
  #define M_Z_MIN -3284   
  #define M_X_MAX 3500
  #define M_Y_MAX 3150
  #define M_Z_MAX 2670 
#endif

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors
int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z; 
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};
// end IMU-9 data
#endif // end if compass == 0
/************************************/
//this block is JNE stuff 
long lcdtimer=0;   //LCD Print timer
float MAG_Heading_Degrees; //LCD_compass tab
unsigned int index; //LCD_compass tab used to index the compass correction interpolation

/************************************************/
#if GPS_Used == 1
// Set up for Easy Transfer to receive GPS data 
 //create object
  EasyTransfer ET, ET2; 
 
  struct RECEIVE_DATA_STRUCTURE{
    //put your variable definitions here for the data you want to receive
    //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  float SD_course;
  float SD_course_to_steer;
 // char SD_CTS_MorT[2];
  float SD_SOG;
  float SD_MagVar;
  float SD_XTE;
  float SD_XTE_differential_error;
//  char SD_XTE_LR[2];
 // char SD_XTE_unit[2];
  char SD_Waypoint_next[11]; 
//  boolean SD_NEMA_sentence;
  boolean SD_GPRMC_fix;
  boolean SD_GPAPB_fix;
  float SD_Bearing_origin_to_destination; // Same as course to steer
  char SD_BOD_MorT[2];
 // char SD_Origin_Waypoint[11];
  float SD_Bearing_to_destination;
 // char SD_BTD_MorT[2];
  float SD_Range_Destination;
 // float SD_Velocity_towards_destination; 
  long SD_UTC;
  //long SD_Date;
 // float SD_Lat_current;
//  float SD_Lon_current;
  float SD_NEXT_TURN;
 // float SD_Time_decimal;
  };
  
  //give a name to the group of data
  RECEIVE_DATA_STRUCTURE ETdata;
  #endif //
/*********************************************************************************/  
  // Second ETData structure to send data to to remote controller
/*    
  struct SEND_DATA_STRUCTURE{  //Arduino to Arduino communication Eazy Transfer by Bill Porter
    //put your variable definitions here for the data you want to send
    //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  float course;
  float SOG;
  float MagVar;
  float XTE;
  char Waypoint_next[11]; 
  float Bearing_origin_to_destination;
  float Bearing_to_destination;
  float Range_Destination;
  //long UTC;
  float Next_Turn;
  float heading;
  float heading_to_steer;
  float course_to_steer;
  float bearingrate;
  float rudder_position;
  char Mode[5];
  int MSG;

  
  }; // End Data Sttructure  
  
  SEND_DATA_STRUCTURE ET2data;
*/  
 /******    SETUP   ************/ 

 void setup() {

   pinMode(48, INPUT); //SW1
   pinMode(46, INPUT); //SW2

   Serial.begin(57600); // Serial conection to Serial Monitor
  // Serial1.begin(19200); //Communication to Serial Remote
   Serial2.begin(19200); //Serial output for the Pololu Motor Controller
   Serial3.begin(57600); // input data from second Arduino MEGA with GPS data using Easy Transfer
    //start the library, pass in the data details and the name of the serial port.  
   #if GPS_Used == 1
    ET.begin(details(ETdata), &Serial3);
   #endif
 //   ET2.begin(details(ET2data), &Serial1);

 pinMode (LCD_Contrast_Pin, OUTPUT); //LCD Contrast Pin
 analogWrite(LCD_Contrast_Pin, LCD_Contrast); // pwm control of LCD contrast  Replaces 10K pot for contrast control 
 lcd.begin(20,4);

#if RF24_Attached == 1
// Radio Setup  
  radio.begin();   
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
  int dataSize = sizeof(RF_DATA);
  Serial.print("Size of RF_DATA1: "); Serial.println(dataSize);
  if ( dataSize > 32 )
    Serial.println("***  RF_DATA1 struct is too large ***");
  radio.startListening();
//  End Radio Setup  
#endif 
 
   keypad.addEventListener(keypadEvent); //add an event listener for this keypad 
   
   lcd.begin(20,4);  // initialize LCD for 4 row 20 characters
   lcd.setCursor(0,0);

 #if Compass == 0
 //SETUP FOR MinIMU9 
 lcd.print("Starting IMU");
     I2C_Init();
   //Serial.println("Pololu MinIMU-9 + Arduino AHRS");
   // digitalWrite(STATUS_LED,LOW);
  delay(1500);
  Accel_Init();
  Compass_Init();
  Gyro_Init();  
  delay(20);  
  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }    
  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;   
    AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];  
  //Serial.println("Offset:");
  for(int y=0; y<6; y++)
    Serial.println(AN_OFFSET[y]);
 
  delay(2000);
 // digitalWrite(STATUS_LED,HIGH);
    
  timer=millis();
  delay(20);
  counter=0;
  // End setup data for MinIMU9
 #endif
 
  if(Motor_Controller == 1) Serial2.write(170); // sends packet that is detected for auto baud rate detection and starts normal operation not need for Trex
  if(Motor_Controller == 3)  // ditto for Pololu Simple controller
  {
  Serial2.write(170);
  Serial2.write(131);
  }
 #if Compass == 1
 if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    lcd.setCursor(0,0);
    lcd.print(" No BNO055");
    while(1);
  }
 //  Get and restore BNO Calibration offsets   
 BNO_RestoreCal();
#endif  // Compass == 1
 }  // end setup
  
/*********************************************/

void loop()
{
 #if Compass == 0
  if(just_started)
  {  setup();  //this runs setup a second time because I (JE) find gyros zero state does not initialize when powered up, runs once
     just_started = 0;
  }
 #endif  
   sw1 = digitalRead(48); // V3 
   sw2 = digitalRead(46); // V3 
  if(SW2_Used == 0) sw2 = true;  // if SW2_Used is 0 then sw2 set true so only sw1 is controling
   if (!sw1 || !sw2)  // if sw1 or sw2 is off both have to be on to engage steering either one will turn steering off
   {
     Rudder_Stop(); 
    #if Clutch_Solenoid == 1 // if a clutch solenoid is used
     Open_Solenoid();   // open solenoid to enable manual steering      
    #endif
      Steering = false;
     // rudder_position = 0; // delete when rudder position is measured
      //rudder_command = 0; // probably delete, let rudder comand run, just don't steer
      Steering_Mode = 0;

      Mode = "OFF";
     // heading_to_steer = 0;
   }
   KEYPAD();
   if (Accept_Terms) Terms_and_Conditions();
      //  Remote_Keypad();
      
  #if RF24_Attached == 1
    Recv_Data();
  #endif
    
   A_P_Loop(); // Autopilot Loop
 }    

/***********************************************/
   void Terms_and_Conditions()
   { lcd.setCursor(0,0);
     lcd.print("I Accept the Terms  ");
     lcd.setCursor(0,1);
     lcd.print("and Conditions      ");
     lcd.setCursor(0,2);
     lcd.print("Press 0             ");
     lcd.setCursor(0,4);
     lcd.print("to Accept           ");
     delay (100);
   }  // end accept terms and conditions

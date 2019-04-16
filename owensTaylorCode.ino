#include <LIDARLite.h>
#include <EEPROM.h>
#include <XBee.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <SPI.h>                // SPI library included for SparkFunLSM9DS1
#include <Wire.h>               // I2C library included for SparkFunLSM9DS1
#include <SparkFunLSM9DS1.h>    // SparkFun LSM9DS1 library


//Define feedback pins
#define feedbackLeftFront  A0  // Define left front feedback pin
#define feedbackRightFront A1  // Define right front feedback pin
#define feedbackLeftRear   A2  // Define left rear feedback pin
#define feedbackRightRear  A3  // Define right rear feedback pin

const int attempts = 20;       // Number of attemps both lsm and xbee try to make a connection

// Define Lidar
LIDARLite myLidarLite;
int lidarDistance = 0 ;
int pos = 90;

// Define servos
Servo servoLeftDrive;         // Define left drive servos
Servo servoRightDrive;        // Define right drive servos
Servo servoLeftSteer;         // Define left steering servos
Servo servoRightSteer;        // Define right steering servos
Servo servoPan;               // Define pan of pan/tilt servos
Servo servoTilt;              // Define tilt of pan/tilt servos

enum DIR {
  STOP, FWD, LEFT, RIGHT, REV, CHECK
};

//////////////////////////////////
///////LSM Code
LSM9DS1 imu;
//imu.settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C
//imu.settings.device.mAddress = LSM9DS1_M; // Set mag address to 0x1E
//imu.settings.device.agAddress = LSM9DS1_AG; // Set ag address to 0x6B

void setupLSM() {
  int c = 0;
  if (!imu.begin())
  {
      Serial.println("Failed to communicate with LSM9DS1.");
      delay(1000);
      while (1)
        if (++c > attempts)
          break;
  }
}
///////LSM End Code
//////////////////////////////////

//////////////////////////////////
// xBee Code
SoftwareSerial outputSerial(10, 9);       // SET RX, TX to pins connected
XBee xbee;
Rx16Response rx16 = Rx16Response();
int resetRSSI = -1000;                    //The value that RSSI is reset to after each pass through filter
#define samples 110
int temp, smoothData, rawData;
int timeToScan = 2000;
short currentHeading;

//Variable for i2c comms
uint8_t currHeadingI2c[2];

//Structure to contain the readings from the beacon
struct{
  float heading;
  int signalStrength;
} readings[samples];

//Union for converting between byte[4] and float
union{
  float f;
  uint8_t b[4];
} heading_converter;

void Retrieve(int i){
  xbee.readPacket(10);    //Wait 50 to receive packet
  if (xbee.getResponse().isAvailable())     //Execute only if packet found
  {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE)
    {
      xbee.getResponse().getRx16Response(rx16);
      //Store the transmitted data and RSSI
      for(int i = 0; i<4; i++)
        heading_converter.b[i] = rx16.getData(i);
      int currentRSSI = -rx16.getRssi();

      //Write to array
      readings[i].heading = heading_converter.f;
      readings[i].signalStrength = currentRSSI;
    }
  }else{
    readings[i].heading = 0;
    readings[i].signalStrength = resetRSSI;
  }
}

int ProcessData(){
  int maxRSSI;
  unsigned long maxIndex = 0;  // THIS LINE DOES NOTHING
  maxRSSI = readings[0].signalStrength;

  //Find max RSSI value
  for (int i=1; i < samples; i++) {
    if (maxRSSI < readings[i].signalStrength) {
      maxRSSI = readings[i].signalStrength;
      maxIndex = i;
    }
  }
  //If there is no valid data
  if(maxRSSI == resetRSSI){
    return -1;
  }

  float headingx = 0;
  float headingy = 0;
  for(int i = 0; i < samples; i++)
  {
    if (readings[i].signalStrength == -1000 && readings[i].heading == 0)
    {
       Serial.println("TOM THIS IS NOT THE FUCKING HEADING BRO!!! ");
    }
    else
    {
      Serial.print("THE CURRENT HEADING OF YOUR XBEE: ");
      Serial.println(readings[i].heading);
      Serial.print("\t");
      Serial.print("THE STRENGTH OF THE FORCE OF: ");
      Serial.println(readings[i].signalStrength);
      // Set magnitude of vector by signal strength
      headingx += readings[i].signalStrength * cos(readings[i].heading * PI / 180);
      headingy += readings[i].signalStrength * sin(readings[i].heading * PI / 180);
    }
  }

  float heading = atan2(headingy, headingx);
  if (heading < 0) heading += 2 * PI;
  heading = heading * 180 / PI;

  return (int) heading;
}

void getSamples()
{
  for(int i = 0;i<samples;i++){
    Retrieve(i);
    float propComplete = ((float)i)/(float)samples; // this line does NOTHING
    delay(timeToScan/samples);
  }
}

void setupXBEE()
{
  int c = 0;
  while (!outputSerial) {
    outputSerial.begin(57600);
    xbee.setSerial(outputSerial);
    if (millis() % 1000 == 0) {
      Serial.print("Failed to connect to xBee!, Attempts: ");
      Serial.println(++c);
      if (c > attempts)
        break;
    }
  }
}
// END of xBee CODE
//////////////////////////////////

// Movement cases
void drive(DIR dir){
  switch(dir){
    
    case FWD:
      servoLeftSteer.write(90);
      servoRightSteer.write(90);
      servoLeftDrive.write(120);
      servoRightDrive.write(70);
      servoTilt.write(120);
      for (pos = 68; pos <= 112; pos += 22) {  // goes from 68 to 112 degrees in 3 steps
        servoPan.write(pos);                   // tell servo to go to position in variable 'pos'
        lidarDistance = myLidarLite.distance();// take lidar reading
        delay(200);                            // waits 15ms for the servo to reach the position
  }
      for (pos = 112; pos >= 68; pos -= 10) {  // goes from 112 to 68 degrees in 3 steps
        servoPan.write(pos);                   // tell servo to go to position in variable 'pos'
        lidarDistance = myLidarLite.distance();// take lidar reading
        delay(200);                            // waits 15ms for the servo to reach the position
  }
      Serial.println("Forward");
      break;
      
    case REV:
      servoLeftSteer.write(90);
      servoRightSteer.write(90);
      servoLeftDrive.write(60);
      servoRightDrive.write(120);
      Serial.println("Reverse");
      delay(1000);
      break;
    
    
    case LEFT:
      servoLeftSteer.write(0);
      servoRightSteer.write(180);
      servoLeftDrive.write(70);
      servoRightDrive.write(110);
      Serial.println("Left turn");
      break;
    
    case RIGHT:
      servoLeftSteer.write(180);
      servoRightSteer.write(0);
      servoLeftDrive.write(110);
      servoRightDrive.write(70);
      Serial.println("Right turn");
      break;


      case CHECK:
        servoLeftDrive.write(90);
        servoRightDrive.write(90);
        servoTilt.write(130);
        Serial.println("Stop");
       for (pos = 45; pos <= 135; pos += 15) {  // goes from 45 to 135 degrees in steps of 10
        servoPan.write(pos);                    // tell servo to go to position in variable 'pos'
        lidarDistance = myLidarLite.distance(); // take lidar reading
       if (lidarDistance < 40 && pos < 90){    // ASSUMING 0-90 is left side CHECK IT!!
          drive(RIGHT);                             // Move right
      } 
       else{
       if (lidarDistance < 40 && pos > 90){    // ASSUMING 0-90 is left side CHECK IT!!
          drive(LEFT); 
      }
       else{
        delay(15);                             // waits 15ms for the servo to reach the position
       }}}  
        break;
}}


void setup()
{

  servoLeftDrive.attach(11);  // Set left drive servo to digital pin 11
  servoRightDrive.attach(10); // Set right drive servo to pin 12
  servoLeftSteer.attach(4);   // Set left steer servo to digital pin 9
  servoRightSteer.attach(5);  // Set right steer servo to pin 5
  servoPan.attach(12);        // Set pan servo to pin 12
  servoTilt.attach(13);       // Set tilt servo to pin 13
  
  Serial.begin(115200); // Initialize serial connection to display distance readings

  myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz

  /*
    configure(int configuration, char lidarliteAddress)

    Selects one of several preset configurations.

    Parameters
    ----------------------------------------------------------------------------
    configuration:  Default 0.
      0: Default mode, balanced performance.
      1: Short range, high speed. Uses 0x1d maximum acquisition count.
      2: Default range, higher speed short range. Turns on quick termination
          detection for faster measurements at short range (with decreased
          accuracy)
      3: Maximum range. Uses 0xff maximum acquisition count.
      4: High sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for high sensitivity and noise.
      5: Low sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for low sensitivity and noise.
    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
  */
  myLidarLite.configure(0); // Change this number to try out alternate configurations

 // setupXBEE();
 // setupLSM();
}

void loop()
{
  //double heading = imu.readMag();     // Gets heading from imu compass
                         // Sets heading in global variable -> Beacon
  /*  To get beacon heading
  float beaconHeading = heading_converter.f;

  float goTo = 360.0 - beaconHeading;
  Serial.print("Angle to Beacon: ");
  Serial.println(heading - goTo);       // Prints angle to beacon
   */
  
  // Take a measurement with receiver bias correction and print to serial terminal
  lidarDistance = myLidarLite.distance();
  Serial.println(myLidarLite.distance());
  if (lidarDistance < 50){ 
    Serial.println("Stop / Check");
    drive(LEFT);             
    delay(200);
  } 
  else
  {
    Serial.println("Forward");
    drive(FWD);            
    delay(200);
  }
 }

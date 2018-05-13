/*
 rcProto.
 2.1 15 Jan 2015: -Added GPS, renaming the beaglebone serial line as serial1
               and removing any write calls to that port that have existed.
 2.2 17 Jan 2015: Fixed the timer serial read
 2.3 18 jan 2015: Output NAV_HEADING based on 9dof IMU
 3.0 25 Nov 2015: MEGA
                  Reconfigured hardware: GPS is now on Arduino. BBB shoreside comms via wifi.
                  GPS on serial1. BBB on serial0.
 3.1 25 Nov 2015: Kalman filter. Pushed some stuff into function calls.
 3.2 25 Nov 2015: Modified Kalman filter to suit my preferred syntax/naming conventions.
                  Removed code for handling pitch/roll jumps (i.e. overturning vehicle).
 3.4 30 Nov 2015: Should've versioned before this... UTM conversions. Calibrated accelerometer.
                  Ultrasonic sensor braking. Now: rear-axle speed sensor...
 3.5 10 Dec 2015: Again: lots of changes that warranted versioning...
                  Speed sensor inputs okay. Three pots on top of cab can be used to
                  adjust settings in real-time. Added an XBee back into the mix here.
                  Now we can control remotely via PS2 controller for drive command debugging.
                  Cleaned up code.

 3.6 27 Dec 2015: Rebuilt motor shield and added proto shield to accomodate
                  bluetooth serial device; changed some pin numbers.
                  Changed mode booleans into a union.
 3.62             Still dicking around trying to improve the ARHS solution. 3.61 was further
                  monkeying with Kalman filter. Here we're going to try a quarternion solution
                  from Madgwick.
 3.63             Corrected improper indexing in the 9dof arrays.
                  Fixed error in dt calculation. Got Madgwick mostly dialed in.
 3.7 24 Feb 2018: added motor.setSpeed(160) and .run commands to main loop outside of XBee if loop for testing
                  Motor still isn't working

 4.0 11 Mar 2018: Took out XBee related things because we switched it out to Radio module.
                  Switched to treating thrust motor as servo because motorshield library was not working
                  Thrust motor works now.
                  Commented out a lot of main loop for testing purposes in the process.
		  Commented out headlights/taillights as they are unused.
TODO
  * Periodically check that all sensors are online and reading as expected
*/

//PIN numbers: kA = 3, kB  = 4
//kA and A4(PIN 10, servo) are from the Arduino and kB and B4(Aileron) are from the receiver
//A3(Pin 9), B3 from the receiver
//AUX1 goes to switch F
//GEAR goes to switch A


// ----------------------- Includes -----------------------
#include <Arduino.h>
#include <Wire.h>                            // I2C comms
#include <Adafruit_MotorShield.h>            // DC motors
#include "Adafruit_PWMServoDriver.h" //Steering servo //had to delete utility
#include <Adafruit_Sensor.h>   // accelerometer
#include <Adafruit_LSM303_U.h> // accelerometer
#include <Adafruit_L3GD20_U.h> // Gyro
#include <Servo.h>  //Steering servo
#include <SoftwareSerial.h> // Adafruit GPS need this
#include <Adafruit_GPS.h>   // GPS
#include <string.h>
#include <Kalman.h>
#include <NewPing.h>
#include "LatLong-UTMconversion.h"
#include "serialParser.h"


// ----------------------- Debugging Flags -----------------------
#define DBG_COMMS true
#define GPSECHO  false
#define BBBECHO  false
//#define XBECHO  false
#define WAITFORGPS false


// ----------------------- Pins -----------------------
// --- Digital
const int pinSteer = 10; // Set up steering servo on pin 10
const int pinThrottle = 9; //Setup throttle motor as servo, writeMicroseconds to change speed
//const int pinLamps = 6; // Headlights
//const int pinTaillights = 5; // brakelights
//const int pinPing = 4;  // 40kHz acoustic ranger
// --- Analog
//const int pinSpeedo = A0; // dc motor on rear axle for speedometer


// ----------------------- Comm Settings -----------------------
serialIO BBB(&Serial3); // Beablebone Black
//serialIO XBee(&Serial2); // XBee radio
Adafruit_GPS GPS(&Serial1);  // Adafruit GPS breakout
boolean usingInterrupt = true;
void useInterrupt(boolean);


// ----------------------- Sensors -----------------------
// --- 9DOF breakout
Adafruit_LSM303_Accel_Unified  accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified    mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified        gyro  = Adafruit_L3GD20_Unified(20);
// --- DC motor Speedometer
double rpm_now = 0;
double rpm_last = 0;
#define VOLTS_TO_RPM 16*60;
double wheelD = 8.25; // in cm


// ----------------------- Motors -----------------------
// --- Throttle
//Adafruit_MotorShield AFMS = Adafruit_MotorShield();
//Adafruit_DCMotor *throttle = AFMS.getMotor(4); // Port M1 thru M4

int desired_thrust;

// --- Steering
Servo steer; // servo object for front steering
Servo throttle; //servo object for back thrust
// Calibration points
// 90 is center
int steerC = 90;
int steerR = 180;
int steerL = 0;


// ----------------------- Ranger -----------------------
// collision avoidance
//NewPing ranger(pinPing, pinPing, 400);  // sig, rx, maximum distance in cm
//unsigned int objRange;                  // distance to obstacle (in cm)


// ----------------------- Timers -----------------------
uint32_t rangerTimer = millis();
uint32_t rangerWait = 500;  // How often (in ms) to check for obstacles
uint32_t manualTimer = millis();
uint32_t manualTimeout = 500; // How long (in ms) to allow manual without a command
uint32_t commTimer = millis();
uint32_t commWait = 100;  // How often (in ms) to output serial messages
uint32_t t_nm1; // For dt calculation
uint32_t t_n; // For dt calculation
uint32_t dt; // For dt calculation


// ----------------------- State variables -----------------------
Kalman kalVelE, kalVelN;
Kalman kalE, kalN;

double velE = 0;
double velN = 0;


// --- Location information
char UTMZone[4];
int RefEllipsoid = 23; //WGS-84
// 15 hatsawap
//double latOrigin = 38.583025;
//double lonOrigin = -76.108210;
// Lab
// double latOrigin = 38.585740;
// double lonOrigin = -76.131784;
double latOrigin = 34.414888;
double lonOrigin = -119.843005;
double magVar = -11.29; //minus is West

double gps_e, gps_n;                // location fix in UTM
double east0, north0;               // Origin in UTM
double nav_e, nav_n;                // State estimate of northing/easting, relative to origin

// Buffer for sampling 9DOF
const int win = 15;
int bufInd = 0;
double A[3][win];
double M[3][win];
double G[3][win];

double Abar[3];   // Buffered 9DOF readings (x,y,z)
double Mbar[3];
double Gbar[3];

double yaw, pitch, roll, heading;        // Best state estimate
double fastyaw, fastpitch, fastroll;        // Initial AHRS results

// Quarternion state
double deltat;    // 1/sample frequency in 1/Hz
double beta = 0.1;        // algorithm gain
double q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
//double phi, theta, psi;

// ----------------------- Tuning -----------------------
// Accelerometer calibration
const int AXM = -1057;
const int AXP = 936;
const int AYM = -983;
const int AYP =  963;
const int AZM = -996;
const int AZP =  987;

float MXB = 2.64; // Bias in uT;
float MYB = -2.26;
float MZB = 10.25;
float MXS = 53.25; // Scale in uT;
float MYS = 51.57;
float MZS = 54.62;
float Mscale;


double velQa = 0.0043;
double velQb = 0.0250;
double velRm = 0.0001;

double navQa = 0.01;
double navQb = 0.01;
double navRm = 0.00001;

// Tuning pots
int pot1, pot2, pot3;
double c1 = 10;
double c2 = 10;
double c3 = 10;

// Here's how a union/struct combo works here:
// The union between "unit8_t MODE" and "struct flags" will
// use up the same memory space (provided that flags is declared with
// the same number of bits as MODE). The trick here is that
// we initialize and update the individual flag fields;  when we call
// for the value of MODE, we just end up reading all of the individual flag bits.
// Tricky...
union {
  uint8_t MODE;
  struct {
    // Don't move. Period.
    uint8_t  allstop : 1; // bit position 0
    // Move via remote override.
    uint8_t  manual : 1; // bit position 1
    // Don't move forward.
    uint8_t  fwdobj : 1; // bit position 2

    uint8_t  spare3 : 1; // bit position 3
    uint8_t  spare4 : 1; // bit position 4
    uint8_t  spare5 : 1; // bit position 5
    uint8_t  spare6 : 1; // bit position 6
    uint8_t  leader : 1; // bit position 7
    // total # of bits  needs to add up to the uint8_t size for MODE
  } flags;
} mode;


// =======================================================
void setup() {

  // Initial mode setting
  mode.flags.allstop = true;
  mode.flags.manual = false;
  mode.flags.fwdobj = false;
  mode.flags.spare3 = 0;
  mode.flags.spare4 = 0;
  mode.flags.spare5 = 0;
  mode.flags.spare6 = 0;
  mode.flags.leader = 1; // We set this to 1 so hex value is two spaces wide


  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < win; j++) {
      A[i][j] = 0;
      M[i][j] = 0;
      G[i][j] = 0;
    }
  }

  // --- Comms ---
  Serial.begin(115200);
  delay(500);

  //XBee.begin(115200);
  BBB.begin(57600);  // Can't get 115200 to work
  GPS.begin(9600);

  // Indicator lights
  //pinMode(pinTaillights, OUTPUT);
  //pinMode(pinLamps, OUTPUT);

  // Establish comms polling via TIMER0
  useInterrupt(usingInterrupt);

  // --- Peripherals ---
  motor_setup();
  gpsStart();
  imuStart();

  //  // Establish STATE
  //  setKalmanInitialState();

  // Wait for GPS position
  int n = 0;
  while (GPS.latitude == 0 && WAITFORGPS) {
    if (n == 0)
      Serial.write("Waiting for GPS fix.");
    digitalWrite(pinTaillights, HIGH);

    delay(100);
    digitalWrite(pinTaillights, LOW);
    delay(900);
    gpsParse();
    Serial.print(".");
    n++;
    if (n > 20) {
      Serial.println();
      n = 0;
    }
  }
  // convert reference point to UTM easting/northing
  LLtoUTM(RefEllipsoid, latOrigin, lonOrigin,
          (double&)north0, (double&)east0, UTMZone);

  // convert current location to UTM
  LLtoUTM(RefEllipsoid, GPS.latitudeDegrees, GPS.longitudeDegrees,
          (double&)nav_n, (double&)nav_e, UTMZone);

  // reference location to origin
  nav_e = nav_e - east0;
  nav_n = nav_n - north0;

  kalE.setAngle(nav_e);   // initial x location
  kalN.setAngle(nav_n);   // initial y location


  // initialize time step counters
  rangerTimer = millis();
  manualTimer = millis();
  commTimer = millis();
  t_n = millis();
  t_nm1 = millis();

  pitch = roll = yaw = heading = 0.0f;


  // Ready to go.
  mode.flags.allstop = false;
  // Turn on headlights
  //digitalWrite(pinLamps, HIGH);
  Serial.println("Finished setup.");

}


// =======================================================
void loop() {

  // Use potentiometers to adjust whatnot
  pot1 = (analogRead(A1) + 1); ///1023.*3. - 1.46)*VOLTS_TO_RPM;
  pot2 = (analogRead(A2) + 1); ///1023.*3. - 1.46)*VOLTS_TO_RPM;
  pot3 = (analogRead(A3) + 1); ///1023.*3. - 1.46)*VOLTS_TO_RPM;

  // Update all sensor information
  // --- acc*, mag*, gyro*
  imuUpdate();

  // Call quaternion solution
  //deltat = 0.93f * deltat + 0.07f * (double)(micros() - last_timestep) / 1000000.0f; // Calculate delta time
  t_nm1 = t_n;
  t_n = millis();
  dt = (t_n - t_nm1);
  deltat = ((float)dt) * 0.001f; // Calculate delta time
  //  Serial.print("Mbar in loop: ");
  //  Serial.print(Mbar[0]); Serial.print(", ");
  //  Serial.print(Mbar[1]); Serial.print(", ");
  //  Serial.println(Mbar[2]);
  beta = (float)map(pot2, 1,  1024, 30, 40) / 1000.0f;

  MadgwickAHRSupdate(Gbar[0], Gbar[1], Gbar[2], -Abar[0], -Abar[1], -Abar[2], -Mbar[0], -Mbar[1], -Mbar[2]);

  // Extract pitch/roll/yaw angles
  fastyaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  yaw = fastyaw;
  fastpitch =  -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  pitch = fastpitch;
  fastroll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  roll = 0.99f * roll + 0.01f * fastroll;

  // --- Velocity
  rpm_last = rpm_now;
  rpm_now = ((double)analogRead(pinSpeedo) - 613.0f) / 1023.0f * 0.2f * 60.0f;
  //if (abs(rpm) < 3)
  //rpm = 0;

  // --GPS position
  gpsParse();

  // --Kalman filter to obtain best state estimate
  //kalmanUpdate();


  // Check for obstacles
  //  if ((millis() - rangerTimer) > rangerWait) {
  //    objRange = ranger.convert_cm(ranger.ping());
  //    // Range in cm.
  //    if (objRange > 0 && objRange < 150 * sqrt(pow(velE, 2) + pow(velN, 2))) {
  //      mode.flags.fwdobj = true;
  //      digitalWrite(pinTaillights, HIGH); // Stopping...
  //    }
  //    else {
  //      mode.flags.fwdobj = false;
  //      digitalWrite(pinTaillights, LOW);
  //    }
  //  }


   if (BBB.newMSGreceived()) {
     if (DBG_COMMS)
       Serial.println(BBB.lastMSG());
     if (!BBB.parse(BBB.lastMSG()))
       return;
     setThrottle(BBB.desired_thrust);
     setSteering(BBB.desired_rudder);
   }
/*
  if (XBee.newMSGreceived()) {
    if (DBG_COMMS)
      Serial.println(XBee.lastMSG());
    if (!XBee.parse(XBee.lastMSG()))
      return;

    // Set manual flag
    mode.flags.manual = true;
    mode.flags.allstop = false;
    manualTimer = millis();

    // Issue driving commands
    setThrottle(XBee.desired_thrust * 20);
    setSteering(XBee.desired_rudder);
  }
  else */
  // if (t_n * 1000 - manualTimer > manualTimeout) {
  //   // Remote commands are stale.
  //   mode.flags.manual = false;
  //
  //   // If MOOS isn't engaged, post allstop
  //   // For now: we know MOOS isn't engaged...
  //   if (true)
  //   {
  //     mode.flags.allstop = true;
  //     digitalWrite(pinTaillights, HIGH);
  //     //setThrottle(160);
  //   }
  // }
  //
  // // Maintain timers for wraparound
  // if (manualTimer > t_n)  manualTimer = millis();
  // if (t_nm1 > t_n)  t_nm1 = millis();
  // if (commTimer > t_n)  commTimer = millis();
  //
  // if ((t_n - commTimer) > commWait) {
  //   serialPrint('S');
  //   commTimer = millis();
  // }

serialPrint('N');
}

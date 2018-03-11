
#include "serialParser.h"

// how long are max msg lines to parse?
#define BUFLEN 120

// we double buffer: read one line in and leave one for the main program
volatile char lineA[BUFLEN];
volatile char lineB[BUFLEN];
// our index into filling the current line
volatile uint8_t idx = 0;
// pointers to the double buffers
volatile char *currentMsg;
volatile char *lastMsg;
volatile boolean newMail;

// -------------------------------------------- 
boolean serialIO::parse(char *msg) {
  // do checksum check

  // first look if we even have one
  if (msg[strlen(msg) - 4] == '*') {
    uint16_t sum = parseHex(msg[strlen(msg) - 3]) * 16;
    sum += parseHex(msg[strlen(msg) - 2]);

    // check checksum
    for (uint8_t i = 1; i < (strlen(msg) - 4); i++) {
      sum ^= msg[i];
    }
    if (sum != 0) {
      // bad checksum :(
      Serial.println("Bad checksum");
      return false;
    }
  }

  if (strstr(msg, "#C,Rudder")) {
    // found desired thrust command
    char *p = msg;
    p = strchr(p, ',') + 1; // skips "#C,"
    p = strchr(p, ',') + 1; // skips "Rudder,"
    desired_rudder = atoi(p);  // takes value before ",*chksum\n"
    //Serial.print("Thrust: "); Serial.println(desired_thrust);
    p = strchr(p, ',') + 1; // skips thrustvalue,
    p = strchr(p, ',') + 1; // skips "Thrust,"
    desired_thrust = atoi(p);

    return true;
  }
//  if (strstr(msg, "#C,Rudder")) {
//    // found desired rudder command
//    char *p = msg;
//    p = strchr(p, ',') + 1; // skips #C,
//    p = strchr(p, ',') + 1; // skips Rudder,
//    desired_rudder = atoi(p);
//    //Serial.print("Rudder: "); Serial.println(desired_rudder);
//    return true;
//  }
  return false;
}

// -------------------------------------------- 
char serialIO::read(void) {
  char c = 0;

  if (paused) return c;

  if (!HwSerial->available()) return c;
  c = HwSerial->read();

  //Serial.print(c);

  //  if (c == '$') {         //please don't eat the dollar sign - rdl 9/15/14
  //    currentMsg[idx] = 0;
  //    idx = 0;
  //  }
  if (c == '\n') {
    currentMsg[idx] = 0;

    if (currentMsg == lineA) {
      currentMsg = lineB;
      lastMsg = lineA;
    } else {
      currentMsg = lineA;
      lastMsg = lineB;
    }

    //Serial.println("----");
    //Serial.println((char *)lastMsgline);
    //Serial.println("----");
    idx = 0;
    newMail = true;
  }

  currentMsg[idx++] = c;
  if (idx >= BUFLEN)
    idx = BUFLEN - 1;

  return c;
}



// -------------------------------------------- 
// Constructor when using HardwareSerial
serialIO::serialIO(HardwareSerial *ser) {
  common_init();  // Set everything to common state, then...
  HwSerial = ser; // ...override HwSerial with value passed.
}

// -------------------------------------------- 
// Initialization code used by all constructor types
void serialIO::common_init(void) {
  //#ifdef __AVR__
  //  SwSerial = NULL; // Set both to NULL, then override correct
  //#endif
  HwSerial = NULL; // port pointer in corresponding constructor
  newMail   = false;
  paused      = false;
  idx     = 0;
  currentMsg = lineA;
  lastMsg    = lineB;

  desired_rudder = 0;
  desired_thrust = 0;
}


// -------------------------------------------- 
void serialIO::begin(uint32_t baud)
{
  Serial.print("Starting serialIO at baud: "); Serial.println(baud);
  delay(100);

  HwSerial->begin(baud);

  delay(100);
}


// -------------------------------------------- 
uint8_t serialIO::parseHex(char c) {
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 0;
  if (c <= 'F')
    return (c - 'A') + 10;
  // if (c > 'F')
  return 0;
}


// -------------------------------------------- 
boolean serialIO::newMSGreceived(void) {
  return newMail;
}


// -------------------------------------------- 
void serialIO::pause(boolean p) {
  paused = p;
}


// -------------------------------------------- 
char *serialIO::lastMSG(void) {
  newMail = false;
  return (char *)lastMsg;
}


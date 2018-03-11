
#ifndef _SERIALPARSER_H
#define _SERIALPARSER_H

#include "Arduino.h"

class serialIO {
 public:
  void begin(uint32_t baud); 

  serialIO(HardwareSerial *ser); // Constructor when using HardwareSerial

  char *lastMSG(void);

  boolean newMSGreceived();
  void common_init(void);
  
  void pause(boolean b);
  
  boolean parseMSG(char *response);
  uint8_t parseHex(char c);

  char read(void);
  boolean parse(char *);
  
  int desired_rudder;
  int desired_thrust;
  
 private:
  boolean paused;
  
  uint8_t parseResponse(char *response);

  HardwareSerial *HwSerial;
};

#endif


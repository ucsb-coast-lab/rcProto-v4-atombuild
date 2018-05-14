#include <Arduino.h>

void serialPrint(char msgType) {
  // Output options.
  switch (msgType) {
    case '0':
      return;
    case 'S': { // State message
        String msg = "#S,T" + String(millis()) + \
                     ",LAT" + String(GPS.latitudeDegrees, 5) + \
                     ",LON" + String(GPS.longitudeDegrees, 5) + \
                     ",D" + "0" + \
                     ",DG" + "0" + \
                     ",A" + "0" + \
                     ",P" + String(pitch * 180.0 / PI, 1) + \
                     ",R" + String(roll * 180.0 / PI, 1) + \
                     ",TR" + String(0) + \
                     ",TRG" + String(desired_thrust) + \
                     ",V" + String(sqrt(pow(velE, 2) + pow(velN, 2))) + \
                     ",H" + String(heading, 1) + \
                     ",HR" + "0" + \
                     ",HG" + String(0) + \
                     ",M" + String(mode.MODE, BIN);
        msg = appendChecksum(msg);
        Serial.print(msg);
        return;
      }
    case 'N': { // Nav position
        String msg = "#N,T" + String(millis()) + \
                     ",LAT" + String(GPS.latitudeDegrees, 5) + \
                     ",LON" + String(GPS.longitudeDegrees, 5) + \
                     ",NAE" + String(nav_e, 1) + \
                     ",NAN" + String(nav_n, 1) + \
                     ",VEL" + String(sqrt(pow(velE, 2) + pow(velN, 2))) + \
                     ",HDG" + String(heading, 1);
        msg = appendChecksum(msg);
        Serial.print(msg);
        BBB.print(msg);
        return;
      }
    case 'W': { // Wheel debugging
        String msg = "#W,T" + String(millis()) + \
                     ",V" + String(sqrt(pow(velE, 2) + pow(velN, 2))) + \
                     ",RPM" + String(rpm_now, 2) + \
                     ",A0" + String("0");//analogRead(pinSpeedo));
        msg = appendChecksum(msg);
        Serial.print(msg);
        return;
      }
    case 'I': { // Raw IMU readings
        String msg = "#I,T" + String(millis()) + \
                     ",AX" + String(Abar[0], 2) + \
                     ",AY" + String(Abar[1], 2) + \
                     ",AZ" + String(Abar[2], 2) + \
                     ",MX" + String(Mbar[0], 2) + \
                     ",MY" + String(Mbar[1], 2) + \
                     ",MZ" + String(Mbar[2], 2) + \
                     ",GX" + String(Gbar[0], 2) + \
                     ",GY" + String(Gbar[1], 2) + \
                     ",GZ" + String(Gbar[2], 2);
        msg = appendChecksum(msg);
        Serial.print(msg);
        return;
      }
    case 'P': { // Potentiometer readings
        String msg = "#P,T" + String(millis()) + \
                     ",P1" + String(pot1) + \
                     ",P2" + String(pot2) + \
                     ",P3" + String(pot3);
        msg = appendChecksum(msg);
        Serial.print(msg);
        return;
      }
    default:
      return;
  }
}



String appendChecksum(String msg)
{
  uint16_t sum = 0;

  for (uint8_t i = 1; i < (msg.length()); i++) {
    sum ^= msg[i];
  }
  msg = msg + "*" + String(sum, HEX) + "\n";

  return msg;
}


// --------------------------------------------
SIGNAL(TIMER0_COMPA_vect) {
  // Interrupt is called once a millisecond,
  //   looks for new serial data, and stores it
  char c = BBB.read();
  char g = GPS.read();
  //char x = XBee.read();

#ifdef UDR0
  if (BBBECHO)
    if (c) UDR0 = c;

  if (GPSECHO)
    if (g) UDR0 = g;

//  if (XBECHO)
//    if (x) UDR0 = x;
#endif
}


// --------------------------------------------
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

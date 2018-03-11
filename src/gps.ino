#include <Arduino.h>

// --------------------------------------------
void gpsStart() {
  // Start-up settings for GPS

#define PMTK_SET_NAVSPEED_THRESHOLD_20CMS "$PMTK386,0.2*3F"
#define PMTK_SET_NAVSPEED_THRESHOLD_40CMS "$PMTK386,0.4*39"
#define PMTK_SET_NAV_ACK "$PMTK001,386,3*3D"
#define PMTK_Q_NAV_THRESHOLD "$PMTK447*35"

  Serial.println("Starting GPS.");

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // messages
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);     // echo rate (100_ or 200_MILLIHERTZ or 1HZ or 5HZ or 10HZ)
  Serial.println("GPS: Set echo rate.");
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);     // fix update rate (100_ or 200_MILLIHERTZ or 1HZ or 5HZ)
  Serial.println("GPS: Set update rate");

  GPS.sendCommand(PMTK_SET_NAVSPEED_THRESHOLD_40CMS);
  if (GPS.waitForSentence(PMTK_SET_NAV_ACK))
    Serial.println("GPS: NAVSPEED set.");
  else
    Serial.println("GPS: NAVSPEED set FAILED.");

  GPS.sendCommand(PMTK_Q_NAV_THRESHOLD);
  if (GPS.waitForSentence("$PMTK527"))
    Serial.println(GPS.lastNMEA());

  GPS.sendCommand(PGCMD_NOANTENNA); // OFF
  //GPS.sendCommand(PGCMD_ANTENNA); // ON

  delay(1000);
  Serial.println("GPS: Started.");

}


// --------------------------------------------
void gpsParse() {
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
}

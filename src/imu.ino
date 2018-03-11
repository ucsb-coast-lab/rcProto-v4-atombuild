#include <Arduino.h>

// --------------------------------------------
void imuStart() {
  // Check for comms with all sensors
  // Then poll sensors for initial state estimate

  Serial.println("IMU: Initializing imu sensor.");

  if (!accel.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("IMU: No accelerometer. Faulted."));
    while (1);
  }

  if (!mag.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("IMU: Warning. Magnetomete may not be present.");
    //while (1);
  }
  //mag.setMagGain(LSM303_MAGGAIN_8_1);
  mag.setMagRate(LSM303_MAGRATE_15);


  //GYRO_RANGE_2000DPS
  if (!gyro.begin()) {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("IMU: No L3GD20 detected. Faulted.");
    while (1);
  }

  // Calculate scale for magnetometer and then rescale the scales
  Mscale = (MXS + MYS + MZS) / 3.0f;
  MXS = Mscale / MXS;
  MYS = Mscale / MYS;
  MZS = Mscale / MZS;

  Serial.println("IMU: Ready.");

}

// --------------------------------------------
void imuUpdate() {
  // Should be doing some checking here to verify that sensors are still online
  // and giving real data

  // Get reading from 9DOF sensors.
  // Place into circular buffers.
  // Re-map acceleration using calibration.
  // Mag reports in uT from Adafruit library.
  //    1 Tesla = 10^4 gauss. 1E-6 T = 1uT = 1E-2 gauss
  //    So: 1 uT * 1E2 = 1 gauss

  sensors_event_t imu;
  accel.getEvent(&imu);
  A[0][bufInd] = (double)map(imu.acceleration.x * 100, AXM, AXP, -981, 981) / 100;
  A[1][bufInd] = (double)map(imu.acceleration.y * 100, AYM, AYP, -981, 981) / 100;
  A[2][bufInd] = (double)map(imu.acceleration.z * 100, AZM, AZP, -981, 981) / 100;

  mag.getEvent(&imu);
  // Handle bad mag readings
  if (imu.magnetic.x == 0) {
    int lastGood = bufInd - 1;
    M[0][bufInd] = M[0][lastGood];
    M[1][bufInd] = M[1][lastGood];
    M[2][bufInd] = M[2][lastGood];
  } else {
    M[0][bufInd] = MXS * (imu.magnetic.x - MXB);
    M[1][bufInd] = MYS * (imu.magnetic.y - MYB);
    M[2][bufInd] = MZS * (imu.magnetic.z - MZB);
  }

  gyro.getEvent(&imu);
  G[0][bufInd] = imu.gyro.x;
  G[1][bufInd] = imu.gyro.y;
  G[2][bufInd] = imu.gyro.z;

  bufInd++;
  if (bufInd == win)
    bufInd = 0;

  // Zero sum counter
  for (int i = 0; i < 3; i++) {
    Abar[i] = 0;
    Mbar[i] = 0;
    Gbar[i] = 0;
  }

  // Add fractional readings to form an average
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < win; j++) {
      Abar[i] += A[i][j] / win;
      Mbar[i] += M[i][j] / win;
      Gbar[i] += G[i][j] / win;
    }
  }

  //serialPrint('I');
}


// --------------------------------------------
double wrapHeading(double hdg) {
  // Correct for wrap-around across 360--0

  if (hdg < 0)
    hdg += 360;
  else if (hdg > 360)
    hdg -= 360;

  return hdg;

}

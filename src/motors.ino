#include <Arduino.h>

// --------------------------------------------
void motorsStart() {
  // Initialize motor controls

  // Create moter shield instanace
  AFMS.begin();
  steer.attach(pinServo); // Pin where servo is connected to
  steer.write(steerC);    // Set steering to center point

  throttle->setSpeed(0);
  throttle->run(RELEASE);

}


// --------------------------------------------
void setThrottle(int desired_thrust) {
  // set throttle to specified level
  if (DBG_COMMS) {
    Serial.print("-->Setting throttle: ");
    Serial.println(desired_thrust);
  }

  if (mode.flags.allstop) {
    throttle->setSpeed(0);
    throttle->run(RELEASE);
  }
  else if (desired_thrust > 0 && !mode.flags.fwdobj) {
    throttle->setSpeed(abs(desired_thrust));
    throttle->run(FORWARD);
    if (DBG_COMMS) Serial.println("Run forward");
  }
  else if (desired_thrust < 0) {
    throttle->setSpeed(abs(desired_thrust));
    throttle->run(BACKWARD);
    if (DBG_COMMS) Serial.println("Run backward");
  }
  else
    throttle->setSpeed(0);

}

// --------------------------------------------
void setSteering(int desired_rudder) {
  // set rudder to specified angle

  int mapped_rudder = map(desired_rudder, -10, 10, steerL, steerR);
  if (DBG_COMMS) {
    Serial.print("-->Setting steering: "); Serial.print(desired_rudder);
    Serial.print(" mapped to: ");
    Serial.println(mapped_rudder);
  }

  steer.write(mapped_rudder);

}

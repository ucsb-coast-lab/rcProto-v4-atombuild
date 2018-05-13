#include <Arduino.h>

void motor_setup() {
  // initialize motor controls to pins
  steer.attach(pinSteer);
  steer.write(steerC);

  throttle.attach(pinThrottle); //attach throttle motor
  throttle.writeMicroseconds(1500); //force throttle to begin at dead stop

}

/*
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
*/

// --------------------------------------------

void setThrottle(int desired_thrust) {

  int mapped_thrust = map(desired_thrust, -255, 255, 1200, 1800);
  if (DBG_COMMS) {
    Serial.print("-->Setting thrust: "); Serial.print(desired_thrust);
    Serial.print(" mapped to: ");
    Serial.println(mapped_thrust);
  }

  throttle.writeMicroseconds(mapped_thrust);


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
//*/

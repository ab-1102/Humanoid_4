#ifndef JOINT_H
#define JOINT_H

#include <Arduino.h>
#include <Servo.h>


class joint {
private:  
  Servo servo;
  int pin; // Servo pin
  float normalServoAngle; // Angle of servo when joint is at 0
  float targetAngle; // Servo angle

  float minAngle; // Joint limits
  float maxAngle; // Joint limits
  float currentJointAngle; // latest Joint angle
  bool flip = false; // true if +ve joint angle is less that 90 for servo angle.

  // Microsecond limits 
  const int MIN_PULSE = 544;
  const int MAX_PULSE = 2400;

public:

  joint(int p, int o, int min, int max, bool f) {
    pin = p;
    normalServoAngle = o;
    minAngle = min;
    maxAngle = max;
    flip = f;
  }  

  void begin(int angle = 0) {
    servo.attach(pin);
    servo.write(normalServoAngle + angle);
  }

  float jointToServo(float jointAngle) {

    if (jointAngle > maxAngle) {
      jointAngle = maxAngle;
    }
    else if (jointAngle < minAngle) {
      jointAngle = minAngle;
      }
    if (!flip) {
      return normalServoAngle + jointAngle;
    }
    else {
      return normalServoAngle - jointAngle;
    }
  }

  int degreeToMicro(float angle) {
    angle = constrain(angle, 0.0f, 180.0f);
    float pulse = MIN_PULSE + (angle / 180.0f) * (MAX_PULSE - MIN_PULSE);
    return (int)(pulse + 0.5f);
  }

  bool setJointAngle(float targetJointAngle) {
    targetAngle = targetJointAngle;
    currentJointAngle = jointToServo(targetAngle);
    servo.write(degreeToMicro(currentJointAngle));
    return true;
  }

  float getJointAngle() {
    return targetAngle;
  }

  float getServoAngle() {
    return servo.read();
  }

  int getPin() {
    return pin;
  }
};

#endif
#ifndef JOINT_H
#define JOINT_H

#include <Arduino.h>
#include <Servo.h>


class joint {
private:  
  Servo servo;
  int pin; // Servo pin
  float normalServoAngle; // Angle of servo when joint is at 0
  String name; // Joint name
  float minAngle; // Joint limits
  float maxAngle; // Joint limits
  float currentServoAngle; // latest Servo angle
  float currentJointAngle; // latest Joint angle
  bool flip = false; // servo flipping

  // Microsecond limits 
  const int MIN_PULSE = 544;
  const int MAX_PULSE = 2400;

  int degreeToMicro(float angle) {
    angle = constrain(angle, 0.0f, 180.0f);
    float pulse = MIN_PULSE + (angle / 180.0f) * (MAX_PULSE - MIN_PULSE);
    return (int)(pulse + 0.5f);
  }

public:

  joint(int p, int o, int min, int max, bool f, String n = "") {
    pin = p;
    normalServoAngle = o;
    minAngle = min;
    maxAngle = max;
    flip = f;
    name = n;
  }  

  void begin(int angle = 0) {
    servo.attach(pin);
    servo.write(normalServoAngle + angle);
  }

  bool setJointAngle(float targetJointAngle) {
    if (targetJointAngle > maxAngle) {
      currentJointAngle = maxAngle;
      Serial.print(name);
      Serial.print(" Limited to = ");
      Serial.println(currentJointAngle);
    }
    else if (targetJointAngle < minAngle) {
      currentJointAngle = minAngle;
      Serial.print(name);
      Serial.print(" Limited to = ");
      Serial.println(currentJointAngle);
    }
    else {
      currentJointAngle = targetJointAngle;
      Serial.print(name);
      Serial.print(" Set to = ");
      Serial.println(currentJointAngle);
    }
    
    if (!flip) {
      currentServoAngle =  normalServoAngle + currentJointAngle;
    }
    else {
      currentServoAngle =  normalServoAngle - currentJointAngle;
    }
    servo.write(degreeToMicro(currentServoAngle));
    return true;
    

  }

  float getJointAngle() {
    return currentJointAngle;
  }

  float getServoAngle() {
    return servo.read();
  }

  int getPin() {
    return pin;
  }

  bool outLimit(float angle) {
    if (angle < minAngle) {
      String n = name + " Joint";
      Serial.print(n);
      Serial.println(" MinAngle HIT");
      return false; // temp
    }
    if (angle > maxAngle) {
      String n = name + " Joint";
      Serial.print(n);
      Serial.println(" MaxAngle HIT");
      return false; // temp
    }
    return false;
  }
};

#endif
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include <Joint.h>

struct Point {
  float x;
  float y;
  float z;
};

class body {
private:
  
  const float L = 50.0f; // Link Length
  const float K = 20.2f; // Knee Length

  const int rightKneePin = 9, rightKneeNormal = 92, rightKneeMin = -30, rightKneeMax = 30;
  const int leftKneePin = 8, leftKneeNormal = 83, leftKneeMin = -30, leftKneeMax = 30;
  const int rightFootPin = 7, rightFootNormal = 94, rightFootMin = -30, rightFootMax = 30;
  const int leftFootPin = 6, leftFootNormal = 100, leftFootMin = -30, leftFootMax = 30;

  const int rightHipPin = 11, rightHipNormal = 90, rightHipMin = -10, rightHipMax = 20;
  const int leftHipPin = 10, leftHipNormal = 70, leftHipMin = -10, leftHipMax = 20;
  const int HipPin = 12, HipNormal = 90, HipMin = -20, HipMax = 20;
  
  const int leftShoulderAPin = 14, leftShoulderANormal = 89, leftShoulderAMin = -90, leftShoulderAMax = 90;
  const int rightShoulderAPin = 15, rightShoulderANormal = 90, rightShoulderAMin = -90, rightShoulderAMax = 90;

  const int rightShoulderBPin = 16, rightShoulderBNormal = 90, rightShoulderBMin = -90, rightShoulderBMax = 90;
  const int leftShoulderBPin = 17, leftShoulderBNormal = 91, leftShoulderBMin = -90, leftShoulderBMax = 90;

  const int rightElbowPin = 18, rightElbowNormal = 90, rightElbowMin = -90, rightElbowMax = 90;
  const int leftElbowPin = 19, leftElbowNormal = 90, leftElbowMin = -90, leftElbowMax = 90;

  const int rightAnklePin = 4, rightAnkleNormal = 92, rightAnkleMin = -10, rightAnkleMax = 20;
  const int leftAnklePin = 5, leftAnkleNormal = 82, leftAnkleMin = -10, leftAnkleMax = 20;

  joint right_knee; 
  joint left_knee; 

  joint right_foot; 
  joint left_foot; 

  joint right_hip;
  joint left_hip;

  joint hip;

  joint left_shoulderA;
  joint right_shoulderA;

  joint left_shoulderB;
  joint right_shoulderB;

  joint left_elbow;
  joint right_elbow;

  joint left_ankle;
  joint right_ankle;

  bool rightLegMovementDone;
  bool leftLegMovementDone;
 

  bool IK_CalAngle(Point p, float &theta, float &alpha, float &gamma) {
    float X = p.x;
    float Y = p.y;
    float Z = p.z;

    float ZL = sqrt(( Z * Z ) + ( Y * Y ));

    float Leg = 50.0f; // Length of the leg segment
    float Knee = 20.2f; // Length of the knee segment
    
    float a = acos(( 0.5 * sqrt(X * X + ( ZL - Knee ) * ( ZL - Knee )) ) / Leg);
    float b = atan(X / ( ZL - Knee ));

    if (!isfinite(a) || !isfinite(b)) {
      Serial.println("IK_CalAngle: Invalid angles calculated. Check the input point.");
      return false;
    }

    theta = degrees(a + b);
    alpha = degrees(a - b);
    gamma = degrees(atan(Y / Z));

    if (!isfinite(theta) || !isfinite(alpha) || !isfinite(gamma)) {
      Serial.println("IK_CalAngle: Invalid joint angles calculated.");
      return false;
    }
    return true;
  }
  
  bool setRightFoot(Point P) {
    float kneeAngle;
    float footAngle;
    float hipAngle;
    if (!IK_CalAngle(P, kneeAngle, footAngle, hipAngle)){
      return false;
    }
    float ankleAngle = hipAngle;
    if (!right_knee.setJointAngle(kneeAngle) || !right_foot.setJointAngle(footAngle) || !right_hip.setJointAngle(hipAngle) || !right_ankle.setJointAngle(ankleAngle)){
      return false;
    }
    return true;
  }

  bool setLeftFoot(Point P) {
    float kneeAngle;
    float footAngle;
    float hipAngle;
    if (!IK_CalAngle(P, kneeAngle, footAngle, hipAngle)){
      return false;
    }
    float ankleAngle = hipAngle;
    if (!left_knee.setJointAngle(kneeAngle) || !left_foot.setJointAngle(footAngle) || !left_hip.setJointAngle(hipAngle) || !left_ankle.setJointAngle(ankleAngle)){
      return false;
    }
    return true;
  }

  bool getDeltaPoint(unsigned long startTime, unsigned long totalTime, unsigned long deltaTime, Point startPoint, Point endPoint, Point &deltaPoint) {
    if (deltaTime >= totalTime) {
      // Serial.println("getDeltaPoint : deltaTime > totalTime");
      deltaPoint = endPoint;    
      return true;
    }
    else {
      float Vx = (endPoint.x - startPoint.x) / totalTime;
      float Vy = (endPoint.y - startPoint.y) / totalTime;
      float Vz = (endPoint.z - startPoint.z) / totalTime;
      deltaPoint.x = (deltaTime * Vx) + startPoint.x;
      deltaPoint.y = (deltaTime * Vy) + startPoint.y;
      deltaPoint.z = (deltaTime * Vz) + startPoint.z;
      return false;
    }
  }

public:

  body() :
    right_knee(rightKneePin,rightKneeNormal,rightKneeMin,rightKneeMax,true),
    left_knee(leftKneePin,leftKneeNormal,leftKneeMin,leftKneeMax,false),
    right_foot(rightFootPin,rightFootNormal,rightFootMin,rightFootMax,false),
    left_foot(leftFootPin,leftFootNormal,leftFootMin,leftFootMax,true),
    right_hip(rightHipPin,rightHipNormal,rightHipMin,rightHipMax,false),
    left_hip(leftHipPin,leftHipNormal,leftHipMin,leftHipMax,true),
    hip(HipPin,HipNormal,HipMin,HipMax,false),

    left_shoulderA(leftShoulderAPin,leftShoulderANormal,leftShoulderAMin,leftShoulderAMax,true),
    right_shoulderA(rightShoulderAPin,rightShoulderANormal,rightShoulderAMin,rightShoulderAMax,false),
    
    left_shoulderB(leftShoulderBPin,leftShoulderBNormal,leftShoulderBMin,leftShoulderBMax,false),
    right_shoulderB(rightShoulderBPin,rightShoulderBNormal,rightShoulderBMin,rightShoulderBMax,true),
    
    left_elbow(leftElbowPin,leftElbowNormal,leftElbowMin,leftElbowMax,false),
    right_elbow(rightElbowPin,rightElbowNormal,rightElbowMin,rightElbowMax,true),
    left_ankle(leftAnklePin,leftAnkleNormal,leftAnkleMin,leftAnkleMax,true),
    right_ankle(rightAnklePin,rightAnkleNormal,rightAnkleMin,rightAnkleMax,false)
  {}

  bool setup() {
    right_knee.begin();
    left_knee.begin();
    right_foot.begin();
    left_foot.begin();
    right_hip.begin();
    left_hip.begin();
    hip.begin();
    left_shoulderA.begin();
    right_shoulderA.begin();
    left_shoulderB.begin(90);
    right_shoulderB.begin(-90);
    left_elbow.begin();
    right_elbow.begin();
    left_ankle.begin();
    right_ankle.begin();
    return true;
  }

  bool walk(char motion, int cycles, unsigned long totalTime, int contactTime = 500) {
    // A -t1> B -t2> C -t3> A

    left_shoulderB.setJointAngle(90);
    right_shoulderB.setJointAngle(90);

    Point A;
    Point B;
    Point C;

    if (motion == 'F') {
      A = {0, 0, 85};
      B = {5, 25, 110};
      C = {-15, -25, 110};
    }
    else if (motion == 'B') {
      A = {0, 0, 110};
      B = {0, 25, 115};
      C = {20, 25, 115};
    }
    else {
      return false;
    }

    int cycle = 0;

    unsigned long t2 = contactTime;
    unsigned long t1 = (totalTime - t2)/2;
    unsigned long t3 = t1;
    

    setRightFoot(B);
    setLeftFoot(C);

    for (cycle = 1; cycle <= cycles; cycle++) {
      Serial.print("---------------");
      Serial.print(cycle);
      Serial.println("---------------");
      char rightLegStage = 'B';
      setRightFoot(B);
      Point rightLegPoint = B;
      rightLegMovementDone = true;
      char leftLegStage = 'C'; 
      setLeftFoot(C);
      Point leftLegPoint = C;
      leftLegMovementDone = true;
      bool cycleRunning = true;
      unsigned long startTime = millis();
      unsigned long AtB_StartingTimeR;
      unsigned long BtC_StartingTimeR;
      unsigned long CtA_StartingTimeR;
      unsigned long AtB_StartingTimeL;
      unsigned long BtC_StartingTimeL;
      unsigned long CtA_StartingTimeL;

      unsigned long AtB_DetlaTimeR;
      unsigned long BtC_DetlaTimeR;
      unsigned long CtA_DetlaTimeR;
      unsigned long AtB_DetlaTimeL;
      unsigned long BtC_DetlaTimeL;
      unsigned long CtA_DetlaTimeL;

      while (cycleRunning) {
        unsigned long currentTime = millis();     

        if (currentTime - startTime >= totalTime) {
          cycleRunning = false;
          break;
        }
        
        // Serial.print(" RFS:" + String(rightLegStage) + " LFS:" + String(leftLegStage));

        if (rightLegStage == 'A') {
          if (rightLegMovementDone) {
            AtB_StartingTimeR = currentTime;
            rightLegMovementDone = false;
          }
          AtB_DetlaTimeR = currentTime - AtB_StartingTimeR;
          if (!getDeltaPoint(AtB_StartingTimeR, t1, AtB_DetlaTimeR, A, B, rightLegPoint)) {
            setRightFoot(rightLegPoint);
          }
          else {
            rightLegStage = 'B';
            rightLegMovementDone = true;
          }          
        }
        else if (rightLegStage == 'B') {
          if (rightLegMovementDone) {
            BtC_StartingTimeR = currentTime;
            rightLegMovementDone = false;
          }
          BtC_DetlaTimeR = currentTime - BtC_StartingTimeR;
          if (!getDeltaPoint(BtC_StartingTimeR, t2, BtC_DetlaTimeR, B, C, rightLegPoint)) {
            setRightFoot(rightLegPoint);
          }
          else {
            rightLegStage = 'C';
            rightLegMovementDone = true;
          }
        }
        else if (rightLegStage == 'C') {
          if (rightLegMovementDone) {
            CtA_StartingTimeR = currentTime;
            rightLegMovementDone = false;
          }
          CtA_DetlaTimeR = currentTime - CtA_StartingTimeR;
          if (!getDeltaPoint(CtA_StartingTimeR, t3, CtA_DetlaTimeR, C, A, rightLegPoint)) {
            setRightFoot(rightLegPoint);
          }
          else {
            rightLegStage = 'A';
            rightLegMovementDone = true;
          }
        }
        
        // Serial.print(" RFS:" + String(rightLegStage) + " LFS:" + String(leftLegStage));

        if (leftLegStage == 'A') {
          if (leftLegMovementDone) {
            AtB_StartingTimeL = currentTime;
            leftLegMovementDone = false;
          }
          AtB_DetlaTimeL = currentTime - AtB_StartingTimeL;
          if (!getDeltaPoint(AtB_StartingTimeL, t1, AtB_DetlaTimeL, A, B, leftLegPoint)) {
            setLeftFoot(leftLegPoint);
          }
          else {
            leftLegStage = 'B';
            leftLegMovementDone = true;
          }
        }
        else if (leftLegStage == 'B') {
          if (leftLegMovementDone) {
            BtC_StartingTimeL = currentTime;
            leftLegMovementDone = false;
          }
          BtC_DetlaTimeL = currentTime - BtC_StartingTimeL;
          if (!getDeltaPoint(BtC_StartingTimeL, t2, BtC_DetlaTimeL, B, C, leftLegPoint)) {
            setLeftFoot(leftLegPoint);
          }
          else {
            leftLegStage = 'C';
            leftLegMovementDone = true;
          }
        }
        else if (leftLegStage == 'C') {
          if (leftLegMovementDone) {
            CtA_StartingTimeL = currentTime;
            leftLegMovementDone = false;
          }
          CtA_DetlaTimeL = currentTime - CtA_StartingTimeL;
          if (!getDeltaPoint(CtA_StartingTimeL, t3, CtA_DetlaTimeL, C, A, leftLegPoint)) {
            setLeftFoot(leftLegPoint);
          }
          else {
            leftLegStage = 'A';
            leftLegMovementDone = true;
          }
        }
      }
    } 
    return true;  
  }

  bool statue() {
    Point R = {5, 0, 110};
    Point L = {5, 0, 90};
    float theta, alpha, gamma;
    if (!IK_CalAngle(R, theta, alpha, gamma) || !IK_CalAngle(L, theta, alpha, gamma)) {
      Serial.println("statu: IK_CalAngle failed.");
      return false;
    }
    Serial.println(String("Angles for position " + String(R.x) + ", " + String(R.y) + ", " + String(R.z) + " are:" + String(theta) + ", " + String(alpha) + ", " + String(gamma)));
    if (!setRightFoot(R) || !setLeftFoot(L) || !hip.setJointAngle(0) || !left_shoulderA.setJointAngle(0) || !right_shoulderA.setJointAngle(0) || !left_shoulderB.setJointAngle(90) || !right_shoulderB.setJointAngle(90) || !left_elbow.setJointAngle(0) || !right_elbow.setJointAngle(0)) {
      return false;
    }
    return true;
  }

  bool setJoints(Point rightFoot, Point leftFoot, float hipAngle, float leftShoulderA, float rightShoulderA, float leftShoulderB, float rightShoulderB, float leftElbow, float rightElbow) {
    if (!setRightFoot(rightFoot) || !setLeftFoot(leftFoot) || !hip.setJointAngle(hipAngle) || !left_shoulderA.setJointAngle(leftShoulderA) || !right_shoulderA.setJointAngle(rightShoulderA) || !left_shoulderB.setJointAngle(leftShoulderB) || !right_shoulderB.setJointAngle(rightShoulderB) || !left_elbow.setJointAngle(leftElbow) || !right_elbow.setJointAngle(rightElbow)) {
      return false;
    }
    return true;
  }
};

#endif
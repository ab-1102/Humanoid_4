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

  const int rightKneePin = 9, rightKneeNormal = 92, rightKneeMin = -35, rightKneeMax = 35;
  const int leftKneePin = 8, leftKneeNormal = 83, leftKneeMin = -35, leftKneeMax = 35;
  const int rightFootPin = 7, rightFootNormal = 94, rightFootMin = -35, rightFootMax = 35;
  const int leftFootPin = 6, leftFootNormal = 100, leftFootMin = -35, leftFootMax = 35;
  
  const int rightHipPin = 11, rightHipNormal = 90, rightHipMin = -25, rightHipMax = 15;
  const int leftHipPin = 10, leftHipNormal = 70, leftHipMin = -25, leftHipMax = 15;
  const int HipPin = 12, HipNormal = 90, HipMin = -20, HipMax = 20;
  
  const int leftShoulderAPin = 14, leftShoulderANormal = 89, leftShoulderAMin = -90, leftShoulderAMax = 90;
  const int rightShoulderAPin = 15, rightShoulderANormal = 90, rightShoulderAMin = -90, rightShoulderAMax = 90;
  
  const int rightShoulderBPin = 16, rightShoulderBNormal = 90, rightShoulderBMin = -90, rightShoulderBMax = 90;
  const int leftShoulderBPin = 17, leftShoulderBNormal = 91, leftShoulderBMin = -90, leftShoulderBMax = 90;
  
  const int rightElbowPin = 18, rightElbowNormal = 90, rightElbowMin = -90, rightElbowMax = 90;
  const int leftElbowPin = 19, leftElbowNormal = 90, leftElbowMin = -90, leftElbowMax = 90;
  
  const int rightAnklePin = 4, rightAnkleNormal = 92, rightAnkleMin = -15, rightAnkleMax = 30;
  const int leftAnklePin = 5, leftAnkleNormal = 82, leftAnkleMin = -15, leftAnkleMax = 30;
  
  String rightKneeName = "Right Knee",
    leftKneeName = "Left Knee",
    rightFootName = "Right Foot",
    leftFootName = "Left Foot",
    rightHipName = "Right Hip",
    leftHipName = "Left Hip",
    hipName = "Hip",
    leftShoulderAName = "Left Shoulder A",
    rightShoulderAName = "Right Shoulder A",
    leftShoulderBName = "Left Shoulder B",
    rightShoulderBName = "Right Shoulder B",
    leftElbowName = "Left Elbow",
    rightElbowName = "Right Elbow",
    leftAnkleName = "Left Ankle",
    rightAnkleName = "Right Ankle";

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

    float Zl = sqrt(( Z * Z ) + ( Y * Y ));

    float Leg = L; // Length of the leg segment
    float Knee = K; // Length of the knee segment
    
    float a = acos(( 0.5 * sqrt(X * X + ( Zl - Knee ) * ( Zl - Knee )) ) / Leg);
    float b = atan(X / ( Zl - Knee ));

    if (!isfinite(a) || !isfinite(b)) {
      return false;
    }

    theta = degrees(a + b);
    alpha = degrees(a - b);
    gamma = degrees(atan(Y / Z));

    if (!isfinite(theta) || !isfinite(alpha) || !isfinite(gamma)) {
      return false;
    }
    return true;
  }
  
  Point FK_CalPoint(float theta, float alpha, float gamma) {
    Point P;
    float thetaRad = radians(theta);
    float alphaRad = radians(alpha);
    float gammaRad = radians(gamma);
    float Zl = (L * (cos(thetaRad) + cos(alphaRad))) + K;
    P.x = L * (sin(thetaRad) - sin(alphaRad));
    P.y = Zl * sin(gammaRad);
    P.z = Zl * cos(gammaRad);  
    return P;
  }

  bool outLimitsR(float kneeAngle, float footAngle, float hipAngle, float ankleAngle) {
    if (right_hip.outLimit(hipAngle) || right_knee.outLimit(kneeAngle) || right_foot.outLimit(footAngle) || right_ankle.outLimit(ankleAngle)) {
      return true;
    }
    return false;
  }
  bool outLimitsL(float kneeAngle, float footAngle, float hipAngle, float ankleAngle) {
    if (left_hip.outLimit(hipAngle) || left_knee.outLimit(kneeAngle) || left_foot.outLimit(footAngle) || left_ankle.outLimit(ankleAngle)) {
      return true;
    }
    return false;
  }

  bool setRightFoot(Point P) {
    float kneeAngle;
    float footAngle;
    float hipAngle;
    if (!IK_CalAngle(P, kneeAngle, footAngle, hipAngle)){
      Serial.println("sRF-IK failed");
      return false;
    } 
    float ankleAngle = hipAngle;
    if (outLimitsR(kneeAngle, footAngle, hipAngle, ankleAngle)) {
      return false;
    }
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
      Serial.println("sLF-IK failed");
      return false;
    }
    float ankleAngle = hipAngle;
    if (outLimitsL(kneeAngle, footAngle, hipAngle, ankleAngle)) {
      return false;
    }
    if (!left_knee.setJointAngle(kneeAngle) || !left_foot.setJointAngle(footAngle) || !left_hip.setJointAngle(hipAngle) || !left_ankle.setJointAngle(ankleAngle)){
      return false;
    }
    return true;
  }

  bool getDeltaPoint(unsigned long startTime, unsigned long totalTime, unsigned long deltaTime, Point startPoint, Point endPoint, Point &deltaPoint) {
    if (deltaTime >= totalTime) {
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

  float pointDistance(Point p1, Point p2) {
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float dz = p2.z - p1.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
  }
public:

  body() :
    right_knee(rightKneePin,rightKneeNormal,rightKneeMin,rightKneeMax,true, rightKneeName),
    left_knee(leftKneePin,leftKneeNormal,leftKneeMin,leftKneeMax,false, leftKneeName),
    right_foot(rightFootPin,rightFootNormal,rightFootMin,rightFootMax,false, rightFootName),
    left_foot(leftFootPin,leftFootNormal,leftFootMin,leftFootMax,true, leftFootName),
    right_hip(rightHipPin,rightHipNormal,rightHipMin,rightHipMax,false, rightHipName),
    left_hip(leftHipPin,leftHipNormal,leftHipMin,leftHipMax,true, leftHipName),
    hip(HipPin,HipNormal,HipMin,HipMax,false, hipName),

    left_shoulderA(leftShoulderAPin,leftShoulderANormal,leftShoulderAMin,leftShoulderAMax,true, leftShoulderAName),
    right_shoulderA(rightShoulderAPin,rightShoulderANormal,rightShoulderAMin,rightShoulderAMax,false, rightShoulderAName),

    left_shoulderB(leftShoulderBPin,leftShoulderBNormal,leftShoulderBMin,leftShoulderBMax,false, leftShoulderBName),
    right_shoulderB(rightShoulderBPin,rightShoulderBNormal,rightShoulderBMin,rightShoulderBMax,true, rightShoulderBName),

    left_elbow(leftElbowPin,leftElbowNormal,leftElbowMin,leftElbowMax,false, leftElbowName),
    right_elbow(rightElbowPin,rightElbowNormal,rightElbowMin,rightElbowMax,true, rightElbowName),
    left_ankle(leftAnklePin,leftAnkleNormal,leftAnkleMin,leftAnkleMax,true, leftAnkleName),
    right_ankle(rightAnklePin,rightAnkleNormal,rightAnkleMin,rightAnkleMax,false, rightAnkleName)
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
    
    statue(B, C, 2000);

    for (cycle = 1; cycle <= cycles; cycle++) {
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

  bool statue(Point RightFoot = {5, -10, 110}, Point LeftFoot = {5, -10, 110}, float time = 2000) {
    Point Rt = RightFoot;
    Point Lt = LeftFoot;
    Point Rs = getRightFoot();
    Point Ls = getLeftFoot();
    Point Rd;
    Point Ld;
    float rTheta, lTheta, rAlpha, lAlpha, rGamma, lGamma;

    unsigned long startTime = millis();
    while (true) {
      unsigned long currentTime = millis();
      unsigned long deltaTime = currentTime - startTime;
      if (currentTime - startTime >= time) {
        break;
      }
      getDeltaPoint(startTime, time, deltaTime, Rs, Rt, Rd);
      getDeltaPoint(startTime, time, deltaTime, Ls, Lt, Ld);
      if (!IK_CalAngle(Rd, rTheta, rAlpha, rGamma) || !IK_CalAngle(Ld, lTheta, lAlpha, lGamma)) {
        return false;
      }
      if (!setRightFoot(Rd) || !setLeftFoot(Ld) || !hip.setJointAngle(0) || !left_shoulderA.setJointAngle(0) || !right_shoulderA.setJointAngle(0) || !left_shoulderB.setJointAngle(90) || !right_shoulderB.setJointAngle(90) || !left_elbow.setJointAngle(0) || !right_elbow.setJointAngle(0)) {
        return false;
      }
    }
    return true;
  }

  bool setLegs(float values[22]) {
    Point rightFoot = {values[0], values[1], values[2]};
    Point leftFoot = {values[3], values[4], values[5]};
    float hipAngle = values[6];
    float leftShoulderA = values[7];
    float rightShoulderA = values[8];
    float leftShoulderB = values[9];
    float rightShoulderB = values[10];
    float leftElbow = values[11];
    float rightElbow = values[12];
    
    if (!setRightFoot(rightFoot) || !setLeftFoot(leftFoot) || !hip.setJointAngle(hipAngle) || !left_shoulderA.setJointAngle(leftShoulderA) || !right_shoulderA.setJointAngle(rightShoulderA) || !left_shoulderB.setJointAngle(leftShoulderB) || !right_shoulderB.setJointAngle(rightShoulderB) || !left_elbow.setJointAngle(leftElbow) || !right_elbow.setJointAngle(rightElbow)) {
      return false;
    }
    return true;
  }

  bool setJoints(float values[9]) {//15
    float hipAngle = values[0];
    float rightHip = values[1];
    float leftHip = values[2];
    float rightKnee = values[3];
    float leftKnee = values[4];
    float rightFoot = values[5];
    float leftFoot = values[6];
    float rightAnkle = values[7];
    float leftAnkle = values[8];

    if (!right_knee.setJointAngle(rightKnee) || !left_knee.setJointAngle(leftKnee) || !right_foot.setJointAngle(rightFoot) || !left_foot.setJointAngle(leftFoot) || !right_hip.setJointAngle(rightHip) || !left_hip.setJointAngle(leftHip) || !hip.setJointAngle(hipAngle) || !left_ankle.setJointAngle(leftAnkle) || !right_ankle.setJointAngle(rightAnkle)) {
      return false;
    }
    return true;
  }
  
  bool setFoots(Point R, Point L) {
    if (setRightFoot(R)) {
      if (setLeftFoot(L)) {
        return true;
      }
      Serial.println("setLeftFoot failed");
      return false;
    }
    Serial.println("setRightFoot failed");
    return false;
  }

  Point getRightFoot() {
    Point P = FK_CalPoint(right_knee.getJointAngle(), right_foot.getJointAngle(), right_hip.getJointAngle());
    return P;
  }

  Point getLeftFoot() {
    Point P = FK_CalPoint(left_knee.getJointAngle(), left_foot.getJointAngle(), left_hip.getJointAngle());
    return P;
  }
};
#endif
#include <Arduino.h>
#include <Kinematics.h>

body humanoid;

// -------------------------
// Configuration
// -------------------------

constexpr uint8_t NUM_JOINTS = 9;

const char* const JOINTS[NUM_JOINTS] = {
  "HA", "RH", "LH", "RK", "LK", "RF", "LF", "RA", "LA"
};

float angles[NUM_JOINTS] = {0};

// -------------------------
// Serial command buffer
// -------------------------

constexpr uint8_t CMD_BUFFER_SIZE = 32;

char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIndex = 0;

// -------------------------
// Setup
// -------------------------

void setup() {

  Serial.begin(115200);

  humanoid.setup();

  Serial.println("READY");
}

// -------------------------
// Find joint index
// -------------------------

int8_t findJoint(const char* target) {

  for (uint8_t i = 0; i < NUM_JOINTS; i++) {

    if (JOINTS[i][0] == target[0] &&
        JOINTS[i][1] == target[1]) {

      return i;
    }
  }

  return -1;
}

Point rf(0,0,0);
Point lf(0,0,0);

void executeCommand(const char* action,
                    const char* target,
                    float angle) {

  // SA = Set Angle
  if (action[0] == 'S' && action[1] == 'A') {

    int8_t joint = findJoint(target);

    if (joint < 0) {

      Serial.print("ERROR: Unknown joint ");
      Serial.println(target);

      return;
    }

    angles[joint] = angle;

    Serial.print("OK: ");
    Serial.print(target);
    Serial.print(" = ");
    Serial.println(angle);

    return;
  }

  // EX = Execute / send all angles
  if (action[0] == 'E' && action[1] == 'X') {

    // Serial.print("ANGLES:");

    // for (uint8_t i = 0; i < NUM_JOINTS; i++) {

    //   Serial.print(' ');
    //   Serial.print(JOINTS[i]);
    //   Serial.print('=');
    //   Serial.print(angles[i]);
    // }

    Serial.println();

    humanoid.setJoints(angles);

    rf = humanoid.getRightFoot();
    lf = humanoid.getLeftFoot();

    Serial.print("RF:");
    Serial.print(rf);
    Serial.print(" LF:");
    Serial.println(lf);


    rf = humanoid.FK_CalPoint(angles[3],angles[5],angles[1]);
    lf = humanoid.FK_CalPoint(angles[4],angles[6],angles[2]);

    Serial.print("RFT:");
    Serial.print(rf);
    Serial.print(" LFT:");
    Serial.println(lf);


    Serial.println("END");
    return;
  }

  if (action[0] == 'S' && action[1] == 'F') {
    if (target[0] == 'R' && target[1] == 'X') {
      rf.x = angle;
    }
    else if (target[0] == 'R' && target[1] == 'Y') {
      rf.y = angle;
    }
    else if (target[0] == 'R' && target[1] == 'Z') {
      rf.z = angle;
    }
    else if (target[0] == 'L' && target[1] == 'X') {
      lf.x = angle;
    }
    else if (target[0] == 'L' && target[1] == 'Y') {
      lf.y = angle;
    }
    else if (target[0] == 'L' && target[1] == 'Z') {
      lf.z = angle;
    }
    else {
      Serial.print("NO target foot  ");
      Serial.println(target);
    }
    return;
  }
  
  if (action[0] == 'X' && action[1] == 'F') {
    if (humanoid.setFoots(rf,lf)) {
      Serial.println("SetFoot Success");
    }
    else{
      Serial.println("Failed to set foots");
    }

    Serial.print("Target RF:");
    Serial.print(rf);
    Serial.print(" LF:");
    Serial.println(lf);

    rf = humanoid.getRightFoot();
    lf = humanoid.getLeftFoot();

    Serial.print("Current RF:");
    Serial.print(rf);
    Serial.print(" LF:");
    Serial.println(lf);
    Serial.println("END");
    return;

  }

  
  Serial.print("ERROR: Unknown action ");
  Serial.println(action);
  Serial.println("END");
  return;
}

// -------------------------
// Process complete command
// -------------------------

void processCommand(char* command) {

  // Minimum valid command:
  // SA RK 10
  // => SARK10
  if (strlen(command) < 5) {
    Serial.println("ERROR: Invalid command");
    return;
  }

  char action[3];
  char target[3];

  action[0] = command[0];
  action[1] = command[1];
  action[2] = '\0';

  target[0] = command[2];
  target[1] = command[3];
  target[2] = '\0';

  float angle = atof(command + 4);

  executeCommand(action, target, angle);
}

// -------------------------
// Read serial input
// -------------------------

void readSerial() {

  while (Serial.available()) {

    char c = Serial.read();

    // End of command
    if (c == '\n' || c == '\r') {

      if (cmdIndex > 0) {

        cmdBuffer[cmdIndex] = '\0';

        processCommand(cmdBuffer);

        cmdIndex = 0;
      }

      continue;
    }

    // Add character to buffer
    if (cmdIndex < CMD_BUFFER_SIZE - 1) {

      cmdBuffer[cmdIndex++] = c;
    }
    else {

      // Buffer overflow
      cmdIndex = 0;

      Serial.println("ERROR: Command too long");
    }
  }
}

// -------------------------
// Main loop
// -------------------------

void loop() {

  readSerial();
}
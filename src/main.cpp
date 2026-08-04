#include <Arduino.h>
#include <Kinematics.h>

body humanoid;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  humanoid.setup();
  humanoid.statue();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("MAIN LOOP");
  Serial.println("Walking in....");
  for (int i = 5; i > 0; i--) {
    Serial.println(i);
    delay(1000);
  }
  humanoid.walk('F', 10, 2500, 1200);
}

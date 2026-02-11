/*
  CQRobot Photoelectric Liquid Level Sensor Test
  D13 Pin, 5V for the DIP Switch 
*/

const int levelPin = 13;

void setup() {
  Serial.begin(115200);

  // Use pulldown so pin is stable when sensor disconnected
  pinMode(levelPin, INPUT_PULLDOWN);

  Serial.println("Liquid Level Sensor Digital Test");
}

void loop() {
  bool wet = digitalRead(levelPin);

  if (wet) {
    Serial.println("Water detected");
  } else {
    Serial.println("Dry");
  }

  delay(500);
}
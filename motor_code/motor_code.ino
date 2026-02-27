// Single relay / pump test
// Arduino D9 -> Relay IN1

const int RELAY_PIN = 9;

// Most 5V relay boards are "active LOW":
// LOW  = relay ON
// HIGH = relay OFF
const int RELAY_ON  = LOW;
const int RELAY_OFF = HIGH;

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);  // start OFF
}

void loop() {
  digitalWrite(RELAY_PIN, RELAY_ON);   // pump ON
  delay(200);

  digitalWrite(RELAY_PIN, RELAY_OFF);  // pump OFF
  delay(200);
}
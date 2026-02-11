// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(9600);
//   pinMode(A3, INPUT);


// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   int ph_sensor_value = analogRead(A3);
//   float ph_voltage = ph_sensor_value * (5.0 / 1023.0);
//   float ph_value = (ph_voltage * 3.5) + 0.5;
//   Serial.println(ph_value);
//   delay(500);
// }

#include <LiquidCrystal_I2C.h>
#include "DFRobot_PH.h"
#include <EEPROM.h>

#define PH_PIN A3
//LCD initialization
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,16,2); 

//ph initialization
float voltage, phValue, temperature = 25;
DFRobot_PH ph;

void setup() {
  Serial.begin(115200);
  ph.begin();

    // Initiate the LCD and turn on the backlight
  lcd.init();          // Initiate the LCD module
  lcd.backlight();    // Turn on the backlight
}

void loop() {
//set cursor for LCD to 0,0
lcd.setCursor(0, 0);

  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) {
    timepoint = millis();

    int raw = analogRead(PH_PIN);
    voltage = raw / 1024.0 * 5000;         // mV
    phValue = ph.readPH(voltage, temperature);

    lcd.print(phValue);
    Serial.print("raw:");
    Serial.print(raw);
    Serial.print("  mV:");
    Serial.print(voltage, 0);
    Serial.print("  pH:");
    Serial.println(phValue, 2);
  }

  
  ph.calibration(voltage, temperature);
}


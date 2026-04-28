
#include <LiquidCrystal_I2C.h>
#include "DFRobot_PH.h"
#include <EEPROM.h>


#define PH_PIN A3
//LCD initialization
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,16,2); 

//ph initialization
float voltage, phValue, temperature = 25;
DFRobot_PH ph;



//TDS initialization
#define TdsSensorPin A2
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature_tds = 21;
static bool tdsReady = false;
static int samplesCollected = 0;

//Pump initialization
const unsigned long PUMP_INTERVAL = 15UL * 60UL * 1000UL;  // 15 minutes
const unsigned long PUMP_ON_TIME  = 3000UL;                // 3 seconds on

unsigned long lastPhDoseTime = 0;
unsigned long lastTdsDoseTime = 0;

const int PH_DOWN_RELAY_PIN = 9;
const int PH_UP_RELAY_PIN = 10;
const int TDS_DOWN_RELAY_PIN = 5;
const int TDS_UP_RELAY_PIN = 6;
const int RESEVOIR_RELAY_PIN = 3;
//5V relay boards are "active LOW":
// LOW  = relay ON
// HIGH = relay OFF
const int RELAY_ON  = LOW;
const int RELAY_OFF = HIGH;

void setup() {
  Serial.begin(115200);
  ph.begin();

    // Initiate the LCD and turn on the backlight
  lcd.init();          // Initiate the LCD module
  lcd.backlight();    // Turn on the backlight

//pump
pinMode(PH_DOWN_RELAY_PIN, OUTPUT);
digitalWrite(PH_DOWN_RELAY_PIN, RELAY_OFF);  // start OFF
pinMode(PH_UP_RELAY_PIN, OUTPUT);
digitalWrite(PH_UP_RELAY_PIN, RELAY_OFF);  // start OFF
pinMode(TDS_DOWN_RELAY_PIN, OUTPUT);
digitalWrite(TDS_DOWN_RELAY_PIN, RELAY_OFF);  // start OFF
pinMode(TDS_UP_RELAY_PIN, OUTPUT);
digitalWrite(TDS_UP_RELAY_PIN, RELAY_OFF);  // start OFF
pinMode(RESEVOIR_RELAY_PIN, OUTPUT);
digitalWrite(RESEVOIR_RELAY_PIN, RELAY_OFF);
//


  analogReadResolution(10);
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

    lcd.print("phValue: ");
    lcd.print(phValue);
    Serial.print("raw:");
    Serial.print(raw);
    Serial.print("  mV:");
    Serial.print(voltage, 0);
    Serial.print("  pH:");
    Serial.println(phValue, 2);
  }

  
  ph.calibration(voltage, temperature);





  //beginning of TDS value calculations
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)  
    analogBufferIndex = 0;


    
    // if (samplesCollected < SCOUNT) samplesCollected++;
    // if (samplesCollected == SCOUNT) tdsReady = true;
  }
  static unsigned long printTimepoint = millis();
  // if (tdsReady && (millis() - printTimepoint > 800U)){
    if ((millis() - printTimepoint > 800U)){
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1023.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature_tds - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.25; //convert voltage value to tds value
    Serial.print("TDS----Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");

    lcd.setCursor(0, 1);
    lcd.print("TDS Value: ");
    lcd.print(tdsValue);
  }

/////////////////////////////////////////////////////////////////////pump
unsigned long now = millis();
// default: all pumps OFF
digitalWrite(PH_DOWN_RELAY_PIN, RELAY_OFF);
digitalWrite(PH_UP_RELAY_PIN, RELAY_OFF);
digitalWrite(TDS_DOWN_RELAY_PIN, RELAY_OFF);
digitalWrite(TDS_UP_RELAY_PIN, RELAY_OFF);

digitalWrite(RESEVOIR_RELAY_PIN, RELAY_ON);
// pH pump runs only once every 15 minutes if needed
if(now - lastPhDoseTime >= PUMP_INTERVAL){
  if(phValue > 6){
      digitalWrite(PH_DOWN_RELAY_PIN, RELAY_ON);   // pump ON 
      delay(PUMP_ON_TIME);
      digitalWrite(PH_DOWN_RELAY_PIN, RELAY_OFF);   // pump ON
      lastPhDoseTime = now;

  }
  else if(phValue < 6){
    digitalWrite(PH_UP_RELAY_PIN, RELAY_ON);   // pump On
    delay(PUMP_ON_TIME);
    digitalWrite(PH_UP_RELAY_PIN, RELAY_OFF); //pump off
    lastPhDoseTime = now;
  }
}

if (now - lastTdsDoseTime >= PUMP_INTERVAL){
  if(tdsValue < 800){
    digitalWrite(TDS_UP_RELAY_PIN, RELAY_ON);   // pump On
    delay(PUMP_ON_TIME);
    digitalWrite(TDS_UP_RELAY_PIN, RELAY_OFF); //pump off
    lastTdsDoseTime = now;
  }
  else if(tdsValue > 800){
    digitalWrite(TDS_DOWN_RELAY_PIN, RELAY_ON);   // pump On
    delay(PUMP_ON_TIME);
    digitalWrite(TDS_DOWN_RELAY_PIN, RELAY_OFF); //pump off
    lastTdsDoseTime = now;
  }
}
/////////////////pump

}


//median filtering method for TDS reading
int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

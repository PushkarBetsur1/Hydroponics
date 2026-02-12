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
    // Serial.print("raw:");
    // Serial.print(raw);
    // Serial.print("  mV:");
    // Serial.print(voltage, 0);
    // Serial.print("  pH:");
    // Serial.println(phValue, 2);
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

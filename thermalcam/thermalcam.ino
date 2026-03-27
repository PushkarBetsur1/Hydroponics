#include <Adafruit_MLX90640.h>

Adafruit_MLX90640 mlx;
float frame[24 * 32]; // 768 pixel temperature buffer

// --- Configuration ---
const float EMISSIVITY = 0.95;     // Plant leaf emissivity
const int   CENTER_ZONE = 3;       // Averaging zone radius (pixels) around center
const float AMBIENT_OFFSET = 8.0;  // Library default Ta offset (OPENAIR_TA_SHIFT)

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("MLX90640 Plant Leaf Thermometer");

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("ERROR: MLX90640 not found. Check wiring!");
    while (1) delay(100);
  }

  // Chess mode is more accurate than interleaved
  mlx.setMode(MLX90640_CHESS);

  // 18-bit ADC resolution — good balance of speed and accuracy
  mlx.setResolution(MLX90640_ADC_18BIT);

  // 2 Hz refresh rate (plenty for plant monitoring)
  mlx.setRefreshRate(MLX90640_2_HZ);

  Serial.print("Serial #: ");
  Serial.print(mlx.serialNumber[0], HEX);
  Serial.print(mlx.serialNumber[1], HEX);
  Serial.println(mlx.serialNumber[2], HEX);

  delay(500);
}

void loop() {
  // Get ambient (air) temperature from the sensor
  float ambientTemp = mlx.getTa(false) - AMBIENT_OFFSET;

  // Read a full thermal frame
  if (mlx.getFrame(frame) != 0) {
    Serial.println("Frame read failed, retrying...");
    delay(500);
    return;
  }

  // --- Analyze the frame ---
  float minTemp =  999.0;
  float maxTemp = -999.0;
  float sumCenter = 0.0;
  int   centerCount = 0;

  // The sensor is 32 wide x 24 tall
  int centerCol = 16; // horizontal center
  int centerRow = 12; // vertical center

  for (int row = 0; row < 24; row++) {
    for (int col = 0; col < 32; col++) {
      float t = frame[row * 32 + col];

      if (t < minTemp) minTemp = t;
      if (t > maxTemp) maxTemp = t;

      // Average pixels in a CENTER_ZONE box around the center
      if (abs(row - centerRow) <= CENTER_ZONE && abs(col - centerCol) <= CENTER_ZONE) {
        sumCenter += t;
        centerCount++;
      }
    }
  }

  float centerAvg = sumCenter / centerCount;

  // --- Print results ---
  Serial.println("=============================");
  Serial.print("Ambient Air Temp : "); Serial.print(ambientTemp, 1); Serial.println(" °C");
  Serial.print("Leaf Center Avg  : "); Serial.print(centerAvg,   1); Serial.println(" °C");
  Serial.print("Frame Min Temp   : "); Serial.print(minTemp,      1); Serial.println(" °C");
  Serial.print("Frame Max Temp   : "); Serial.print(maxTemp,      1); Serial.println(" °C");
  Serial.print("Leaf vs Air Delta: ");
  Serial.print(centerAvg - ambientTemp, 2);
  Serial.println(" °C");

  // print full thermal grid as CSV (useful for logging)
  // printThermalGrid();

  delay(1000); // Wait 1 second between readings
}

//  dump full 24x32 grid to Serial (for heatmap visualization)
void printThermalGrid() {
  Serial.println("--- Thermal Grid (CSV) ---");
  for (int row = 0; row < 24; row++) {
    for (int col = 0; col < 32; col++) {
      Serial.print(frame[row * 32 + col], 1);
      if (col < 31) Serial.print(",");
    }
    Serial.println();
  }
}
```

---

### Serial Output Example
```
=============================
Ambient Air Temp : 22.4 °C
Leaf Center Avg  : 21.1 °C
Frame Min Temp   : 20.8 °C
Frame Max Temp   : 24.3 °C
Leaf vs Air Delta: -1.30 °C
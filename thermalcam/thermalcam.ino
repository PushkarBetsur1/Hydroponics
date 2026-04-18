#include <Wire.h>
#include <Adafruit_MLX90640.h>

Adafruit_MLX90640 mlx;
float frame[24 * 32]; // 768 values

void setup() {
  Serial.begin(115200);
  delay(2000);  

  Serial.println("MLX90640 Thermal Stream Starting...");

  Wire.begin();
  Wire.setClock(400000);  

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("ERROR: MLX90640 not found!");
    while (1);
  }

  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_2_HZ);
}

void loop() {
  // Read full thermal frame
  if (mlx.getFrame(frame) != 0) {
    Serial.println("ERR");
    return;
  }

  //  CSV (for Python visualization)
  for (int i = 0; i < 768; i++) {
    Serial.print(frame[i]);
    if (i < 767) Serial.print(",");
  }
  Serial.println();

  delay(100); // ~10 FPS
}
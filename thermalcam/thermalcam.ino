#include <Adafruit_MLX90640.h>

Adafruit_MLX90640 mlx;
float frame[24 * 32];

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("MLX90640 Test");

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("ERROR: MLX90640 not found!");
    while (1);
  }

  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX90640_2_HZ);
}

void loop() {
  if (mlx.getFrame(frame) != 0) {
    Serial.println("Frame read error");
    return;
  }

  // Print a few sample pixels + center pixel

  Serial.print("Center Temp: ");
  Serial.print(center);
  Serial.println(" °C");

  delay(500);
}
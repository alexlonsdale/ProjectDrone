#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED display specifications
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // Common I2C address for 128x64 OLEDs (try 0x3D if it doesn't load)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int MPU_ADDRESS = 0x68;

// Calibration offsets
int32_t gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
int32_t accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;

// Moving Average Filter Settings for Gyro
const int WINDOW_SIZE = 10;
int16_t gx_history[WINDOW_SIZE] = {0};
int16_t gy_history[WINDOW_SIZE] = {0};
int16_t gz_history[WINDOW_SIZE] = {0};
int sample_index = 0;

void calibrateMPU() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Calibrating...\nKeep sensor still!");
  display.display();

  int num_samples = 300;
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;

  for (int i = 0; i < num_samples; i++) {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 14, true);

    if (Wire.available() == 14) {
      int16_t ax = (Wire.read() << 8) | Wire.read();
      int16_t ay = (Wire.read() << 8) | Wire.read();
      int16_t az = (Wire.read() << 8) | Wire.read();
      Wire.read(); Wire.read(); // Skip temp
      int16_t gx = (Wire.read() << 8) | Wire.read();
      int16_t gy = (Wire.read() << 8) | Wire.read();
      int16_t gz = (Wire.read() << 8) | Wire.read();

      sum_ax += ax; sum_ay += ay; sum_az += az;
      sum_gx += gx; sum_gy += gy; sum_gz += gz;
    }
    delay(3);
  }

  gyro_x_offset = sum_gx / num_samples;
  gyro_y_offset = sum_gy / num_samples;
  gyro_z_offset = sum_gz / num_samples;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000); // 400kHz fast I2C for smooth screen updates

  // Initialize SSD1306 OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed. Check wiring/address!"));
    while(1);
  }

  // Wake up MPU6500
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  calibrateMPU();
}

void loop() {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x43); // Start reading from Gyro registers (GYRO_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true);

  if (Wire.available() == 6) {
    int16_t raw_gx = ((Wire.read() << 8) | Wire.read()) - gyro_x_offset;
    int16_t raw_gy = ((Wire.read() << 8) | Wire.read()) - gyro_y_offset;
    int16_t raw_gz = ((Wire.read() << 8) | Wire.read()) - gyro_z_offset;

    // Apply Moving Average Filter
    gx_history[sample_index] = raw_gx;
    gy_history[sample_index] = raw_gy;
    gz_history[sample_index] = raw_gz;
    sample_index = (sample_index + 1) % WINDOW_SIZE;

    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
      sum_gx += gx_history[i];
      sum_gy += gy_history[i];
      sum_gz += gz_history[i];
    }

    int16_t filtered_gx = sum_gx / WINDOW_SIZE;
    int16_t filtered_gy = sum_gy / WINDOW_SIZE;
    int16_t filtered_gz = sum_gz / WINDOW_SIZE;

    // Update OLED Display
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    display.setCursor(0, 0);
    display.println(F("MPU6500 Gyro Data"));
    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

    display.setCursor(0, 18);
    display.print("GX: "); display.println(filtered_gx);

    display.setCursor(0, 32);
    display.print("GY: "); display.println(filtered_gy);

    display.setCursor(0, 46);
    display.print("GZ: "); display.println(filtered_gz);

    display.display();
  }

  delay(50); // Refresh rate limit for readable text
}

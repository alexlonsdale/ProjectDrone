#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED display specifications
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C 

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int MPU_ADDRESS = 0x68;
const int BUTTON_PIN = 0; // Built-in BOOT button on GPIO 0

// --- USER-ADJUSTABLE VARIABLES ---
const float GYRO_SCALE_FACTOR  = 131.0;   // Conversion factor for ±250 °/s range
const float ACCEL_SCALE_FACTOR = 16384.0; // Conversion factor for ±2g range
const float GRAVITY_MS2        = 9.80665; // Conversion factor from g to m/s^2
const int   DEAD_ZONE          = 20;      // Raw threshold below which gyro values clamp to 0
const int   WINDOW_SIZE        = 10;      // Moving average smoothing window size
// ---------------------------------

int32_t gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
int32_t accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;

int16_t gx_history[WINDOW_SIZE] = {0};
int16_t gy_history[WINDOW_SIZE] = {0};
int16_t gz_history[WINDOW_SIZE] = {0};
int sample_index = 0;

void calibrateMPU() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);



    display.setCursor(0, 0);
  display.println(F("Calibration Routine"));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  display.setCursor(0, 18);
  display.println(F("Keep still on flat surface."));
  display.display();
  
  // 1-second delay to ensure device is still before sampling
  delay(1000); 

  int num_samples = 300;
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;

  for (int i = 0; i < num_samples; i++) {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3B); // Start at ACCEL_XOUT_H to read accel + temp + gyro in one burst
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 14, true);

    if (Wire.available() == 14) {
      int16_t ax = (Wire.read() << 8) | Wire.read();
      int16_t ay = (Wire.read() << 8) | Wire.read();
      int16_t az = (Wire.read() << 8) | Wire.read();
      Wire.read(); Wire.read(); // Read and discard temperature
      int16_t gx = (Wire.read() << 8) | Wire.read();
      int16_t gy = (Wire.read() << 8) | Wire.read();
      int16_t gz = (Wire.read() << 8) | Wire.read();

      sum_ax += ax; sum_ay += ay; sum_az += az;
      sum_gx += gx; sum_gy += gy; sum_gz += gz;
    }
    
    delay(3);
  }

  // Gyro offsets (zero velocity baseline)
  gyro_x_offset = sum_gx / num_samples;
  gyro_y_offset = sum_gy / num_samples;
  gyro_z_offset = sum_gz / num_samples;

  // Accel offsets (X & Y expected 0g, Z expected +1g / ACCEL_SCALE_FACTOR)
  accel_x_offset = sum_ax / num_samples;
  accel_y_offset = sum_ay / num_samples;
  accel_z_offset = (sum_az / num_samples) - ACCEL_SCALE_FACTOR;
  
  // Clear history buffers on re-calibration to avoid old spikes
  for(int i = 0; i < WINDOW_SIZE; i++) {
    gx_history[i] = 0;
    gy_history[i] = 0;
    gz_history[i] = 0;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000); // 400kHz I2C

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed."));
    while(1);
  }

  // Brief pause to let charge pump power up cleanly
  delay(100);

  // Wake up MPU6500
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  // Splash Screen on Boot
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println(F("A. Lonsdale"));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  display.setCursor(0, 18);
  display.println(F("Guidance Control Program v1.2"));
  display.display();
  delay(1500); // Show splash screen for 1.5 seconds

  calibrateMPU(); // Initial calibration routine
}

void loop() {
  // Check if the BOOT button is pressed (active low) to trigger re-calibration
  if (digitalRead(BUTTON_PIN) == LOW) {
    calibrateMPU();
    delay(300); // Debounce delay
  }

  // Read Accel (0x3B) through Gyro in one single 14-byte transaction
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 14, true);

  if (Wire.available() == 14) {
    // Calibrated Accelerometer raw values
    int16_t raw_ax = ((Wire.read() << 8) | Wire.read()) - accel_x_offset;
    int16_t raw_ay = ((Wire.read() << 8) | Wire.read()) - accel_y_offset;
    int16_t raw_az = ((Wire.read() << 8) | Wire.read()) - accel_z_offset;
    Wire.read(); Wire.read(); // Discard temp

    // Calibrated Gyroscope raw values
    int16_t raw_gx = ((Wire.read() << 8) | Wire.read()) - gyro_x_offset;
    int16_t raw_gy = ((Wire.read() << 8) | Wire.read()) - gyro_y_offset;
    int16_t raw_gz = ((Wire.read() << 8) | Wire.read()) - gyro_z_offset;

    // Store gyro in history arrays for moving average smoothing
    gx_history[sample_index] = raw_gx;
    gy_history[sample_index] = raw_gy;
    gz_history[sample_index] = raw_gz;
    sample_index = (sample_index + 1) % WINDOW_SIZE;

    // Calculate moving average for gyro
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
      sum_gx += gx_history[i];
      sum_gy += gy_history[i];
      sum_gz += gz_history[i];
    }

    int16_t filtered_gx = sum_gx / WINDOW_SIZE;
    int16_t filtered_gy = sum_gy / WINDOW_SIZE;
    int16_t filtered_gz = sum_gz / WINDOW_SIZE;

    // Apply Dead Zone Threshold on gyro raw values before conversion
    if (abs(filtered_gx) < DEAD_ZONE) filtered_gx = 0;
    if (abs(filtered_gy) < DEAD_ZONE) filtered_gy = 0;
    if (abs(filtered_gz) < DEAD_ZONE) filtered_gz = 0;

    // Convert filtered gyro to degrees per second
    float deg_gx = (float)filtered_gx / GYRO_SCALE_FACTOR;
    float deg_gy = (float)filtered_gy / GYRO_SCALE_FACTOR;
    float deg_gz = (float)filtered_gz / GYRO_SCALE_FACTOR;

    // Corrected Yaw/Roll Assignments
    float deg_pitch = deg_gx;
    float deg_yaw   = deg_gz;
    float deg_roll  = deg_gy;

    // Convert accelerometer values to m/s^2
    float accel_x = ((float)raw_ax / ACCEL_SCALE_FACTOR) * GRAVITY_MS2;
    float accel_y = ((float)raw_ay / ACCEL_SCALE_FACTOR) * GRAVITY_MS2;
    float accel_z = ((float)raw_az / ACCEL_SCALE_FACTOR) * GRAVITY_MS2;

    // --- STREAM TO MATLAB (Pitch, Yaw, Roll, Ax, Ay, Az) ---
    Serial.print(deg_pitch); Serial.print(",");
    Serial.print(deg_yaw);   Serial.print(",");
    Serial.print(deg_roll);  Serial.print(",");
    Serial.print(accel_x);   Serial.print(",");
    Serial.print(accel_y);   Serial.print(",");
    Serial.println(accel_z);

    // Update OLED Display (2-column layout: Gyro left, Accel right)
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    // Header
    display.setCursor(0, 0);
    display.print(F("G(deg/s)   A(m/s2)"));
    display.drawLine(0, 9, 128, 9, SSD1306_WHITE);

    // Row 1: Pitch & Accel X
    display.setCursor(0, 16);
    display.print("P:"); display.print(deg_pitch, 1);
    display.setCursor(68, 16);
    display.print("X:"); display.print(accel_x, 1);

    // Row 2: Yaw & Accel Y
    display.setCursor(0, 30);
    display.print("Y:"); display.print(deg_yaw, 1);
    display.setCursor(68, 30);
    display.print("Y:"); display.print(accel_y, 1);

    // Row 3: Roll & Accel Z
    display.setCursor(0, 44);
    display.print("R:"); display.print(deg_roll, 1);
    display.setCursor(68, 44);
    display.print("Z:"); display.print(accel_z, 1);

    display.display();
  }

  delay(20); // 50Hz update loop rate
}

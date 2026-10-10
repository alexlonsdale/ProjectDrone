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
const float GYRO_SCALE_FACTOR = 131.0; 
const int   DEAD_ZONE         = 20;    
const int   WINDOW_SIZE       = 10;    
// ---------------------------------

int32_t gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;

int16_t gx_history[WINDOW_SIZE] = {0};
int16_t gy_history[WINDOW_SIZE] = {0};
int16_t gz_history[WINDOW_SIZE] = {0};
int sample_index = 0;

void calibrateMPU() {
  // Ensure text size is set properly for the calibration screen
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Phase 1: 3-second grace period with animated loading bar (0% to 30%)
  for (int i = 0; i <= 30; i++) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(F("Calibrating:\nLeave still on\na flat surface"));
    
    // Draw loading bar frame and filled progress
    display.drawRect(14, 48, 100, 10, SSD1306_WHITE);
    int fill_width = map(i, 0, 100, 0, 96);
    display.fillRect(16, 50, fill_width, 6, SSD1306_WHITE);
    display.display();
    
    delay(100); // 30 steps * 100ms = 3 seconds grace period
  }

  // Phase 2: Sample gathering with continuation of loading bar (30% to 100%)
  int num_samples = 300;
  int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

  for (int i = 0; i < num_samples; i++) {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 6, true);

    if (Wire.available() == 6) {
      int16_t gx = (Wire.read() << 8) | Wire.read();
      int16_t gy = (Wire.read() << 8) | Wire.read();
      int16_t gz = (Wire.read() << 8) | Wire.read();
      sum_gx += gx; sum_gy += gy; sum_gz += gz;
    }

    // Update the progress bar smoothly during sampling
    if (i % 5 == 0) {
      int progress = map(i, 0, num_samples, 30, 100);
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print(F("Calibrating:\nLeave still on\na flat surface"));
      display.drawRect(14, 48, 100, 10, SSD1306_WHITE);
      int fill_width = map(progress, 0, 100, 0, 96);
      display.fillRect(16, 50, fill_width, 6, SSD1306_WHITE);
      display.display();
    }
    
    delay(3);
  }

  gyro_x_offset = sum_gx / num_samples;
  gyro_y_offset = sum_gy / num_samples;
  gyro_z_offset = sum_gz / num_samples;
  
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
  Wire.setClock(400000); 

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed."));
    while(1);
  }

  // Brief pause to let the OLED charge pump power up cleanly on boot
  delay(100);

  // Wake up MPU6500
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  calibrateMPU(); // Initial calibration on boot with loading bar
}

void loop() {
  // Check if the BOOT button is pressed (active low)
  if (digitalRead(BUTTON_PIN) == LOW) {
    calibrateMPU();
    delay(300); // Debounce delay
  }

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true);

  if (Wire.available() == 6) {
    // Fixed missing bit-shift operators here:
    int16_t raw_gx = ((Wire.read() << 8) | Wire.read()) - gyro_x_offset;
    int16_t raw_gy = ((Wire.read() << 8) | Wire.read()) - gyro_y_offset;
    int16_t raw_gz = ((Wire.read() << 8) | Wire.read()) - gyro_z_offset;

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

    if (abs(filtered_gx) < DEAD_ZONE) filtered_gx = 0;
    if (abs(filtered_gy) < DEAD_ZONE) filtered_gy = 0;
    if (abs(filtered_gz) < DEAD_ZONE) filtered_gz = 0;

    float deg_gx = (float)filtered_gx / GYRO_SCALE_FACTOR;
    float deg_gy = (float)filtered_gy / GYRO_SCALE_FACTOR;
    float deg_gz = (float)filtered_gz / GYRO_SCALE_FACTOR;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    display.setCursor(0, 0);
    display.println(F("Gyro Speed (deg/s)"));
    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

    display.setCursor(0, 18);
    display.print("Pitch: "); display.print(deg_gx, 1); display.println(" deg/s");

    display.setCursor(0, 32);
    display.print("Yaw: "); display.print(deg_gy, 1); display.println(" deg/s");

    display.setCursor(0, 46);
    display.print("Roll: "); display.print(deg_gz, 1); display.println(" deg/s");

    display.display();
  }

  delay(50);
}

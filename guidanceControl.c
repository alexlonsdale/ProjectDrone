#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// OLED display specifications
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int MPU_ADDRESS = 0x68;
const int BUTTON_PIN = 0;

// --- SENSOR VARIABLES ---
const float GYRO_SCALE_FACTOR  = 131.0;
const float ACCEL_SCALE_FACTOR = 16384.0;
const float GRAVITY_MS2        = 9.80665;
const float FILTER_ALPHA       = 0.98;

int32_t gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
int32_t accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;

float pitch_angle = 0.0;
float roll_angle = 0.0;
unsigned long last_loop_time = 0;

// --- PID GAINS (Tune these later) ---
float Kp_pitch = 1.3, Ki_pitch = 0.04, Kd_pitch = 18.0;
float Kp_roll  = 1.3, Ki_roll  = 0.04, Kd_roll  = 18.0;
float Kp_yaw   = 4.0, Ki_yaw   = 0.02, Kd_yaw   = 0.0;

// --- PID STATE VARIABLES ---
float pitch_error, pitch_prev_error, pitch_integral;
float roll_error, roll_prev_error, roll_integral;
float yaw_error, yaw_prev_error, yaw_integral;

// --- CONTROL TARGETS ---
float setpoint_pitch = 0.0; // Desired angle (0 = flat)
float setpoint_roll  = 0.0;
float setpoint_yaw   = 0.0; // Desired rotational rate (0 = stop spinning)
float base_throttle  = 1000.0; // ESC idle (1000us = off, 2000us = max)

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
    delay(1000);

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
            Wire.read(); Wire.read();
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
    accel_x_offset = sum_ax / num_samples;
    accel_y_offset = sum_ay / num_samples;
    accel_z_offset = (sum_az / num_samples) - ACCEL_SCALE_FACTOR;

    pitch_angle = 0.0;
    roll_angle = 0.0;

    // Reset PID states on calibration
    pitch_integral = 0; roll_integral = 0; yaw_integral = 0;
    pitch_prev_error = 0; roll_prev_error = 0; yaw_prev_error = 0;

    last_loop_time = micros();
}

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setClock(400000);

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        while(1);
    }

    delay(100);
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(100);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("A. Lonsdale"));
    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
    display.setCursor(0, 18);
    display.println(F("Guidance Control v1.4"));
    display.display();
    delay(1500);

    calibrateMPU();
}

void loop() {
    // 1. Calculate true dt
    unsigned long current_time = micros();
    float dt = (current_time - last_loop_time) / 1000000.0;
    last_loop_time = current_time;

    if (digitalRead(BUTTON_PIN) == LOW) {
        calibrateMPU();
        delay(300);
    }

    // 2. Read Sensors
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 14, true);

    if (Wire.available() == 14) {
        int16_t raw_ax = ((Wire.read() << 8) | Wire.read()) - accel_x_offset;
        int16_t raw_ay = ((Wire.read() << 8) | Wire.read()) - accel_y_offset;
        int16_t raw_az = ((Wire.read() << 8) | Wire.read()) - accel_z_offset;
        Wire.read(); Wire.read();
        int16_t raw_gx = ((Wire.read() << 8) | Wire.read()) - gyro_x_offset;
        int16_t raw_gy = ((Wire.read() << 8) | Wire.read()) - gyro_y_offset;
        int16_t raw_gz = ((Wire.read() << 8) | Wire.read()) - gyro_z_offset;

        float deg_pitch_rate = (float)raw_gx / GYRO_SCALE_FACTOR;
        float deg_roll_rate  = (float)raw_gy / GYRO_SCALE_FACTOR;
        float deg_yaw_rate   = (float)raw_gz / GYRO_SCALE_FACTOR;

        float accel_x = ((float)raw_ax / ACCEL_SCALE_FACTOR) * GRAVITY_MS2;
        float accel_y = ((float)raw_ay / ACCEL_SCALE_FACTOR) * GRAVITY_MS2;
        float accel_z = ((float)raw_az / ACCEL_SCALE_FACTOR) * GRAVITY_MS2;

        float accel_pitch = atan2(accel_y, accel_z) * RAD_TO_DEG;
        float accel_roll  = atan2(-accel_x, accel_z) * RAD_TO_DEG;

        // 3. Filtered State Estimation
        pitch_angle = FILTER_ALPHA * (pitch_angle + (deg_pitch_rate * dt)) + (1.0 - FILTER_ALPHA) * accel_pitch;
        roll_angle  = FILTER_ALPHA * (roll_angle  + (deg_roll_rate * dt))  + (1.0 - FILTER_ALPHA) * accel_roll;

        // --- 4. PID CONTROLLER ---

        // Calculate Errors (Setpoint - Actual)
        pitch_error = setpoint_pitch - pitch_angle;
        roll_error  = setpoint_roll  - roll_angle;
        yaw_error   = setpoint_yaw   - deg_yaw_rate; // Yaw controls rate, not angle

        // Pitch PID
        float P_pitch = Kp_pitch * pitch_error;
        pitch_integral += pitch_error * dt;
        pitch_integral = constrain(pitch_integral, -400.0, 400.0); // Anti-windup
        float I_pitch = Ki_pitch * pitch_integral;
        float D_pitch = Kd_pitch * ((pitch_error - pitch_prev_error) / dt);
        float PID_pitch = P_pitch + I_pitch + D_pitch;
        pitch_prev_error = pitch_error;

        // Roll PID
        float P_roll = Kp_roll * roll_error;
        roll_integral += roll_error * dt;
        roll_integral = constrain(roll_integral, -400.0, 400.0);
        float I_roll = Ki_roll * roll_integral;
        float D_roll = Kd_roll * ((roll_error - roll_prev_error) / dt);
        float PID_roll = P_roll + I_roll + D_roll;
        roll_prev_error = roll_error;

        // Yaw PID
        float P_yaw = Kp_yaw * yaw_error;
        yaw_integral += yaw_error * dt;
        yaw_integral = constrain(yaw_integral, -400.0, 400.0);
        float I_yaw = Ki_yaw * yaw_integral;
        float D_yaw = Kd_yaw * ((yaw_error - yaw_prev_error) / dt);
        float PID_yaw = P_yaw + I_yaw + D_yaw;
        yaw_prev_error = yaw_error;

        // --- 5. MOTOR MIXER (Quad-X Layout) ---
        // CW motors: Front-Left (FL), Rear-Right (RR)
        // CCW motors: Front-Right (FR), Rear-Left (RL)

        float motor_FL = base_throttle + PID_pitch + PID_roll - PID_yaw;
        float motor_FR = base_throttle + PID_pitch - PID_roll + PID_yaw;
        float motor_RL = base_throttle - PID_pitch + PID_roll + PID_yaw;
        float motor_RR = base_throttle - PID_pitch - PID_roll - PID_yaw;

        // Constrain PWM outputs to standard ESC range
        motor_FL = constrain(motor_FL, 1000.0, 2000.0);
        motor_FR = constrain(motor_FR, 1000.0, 2000.0);
        motor_RL = constrain(motor_RL, 1000.0, 2000.0);
        motor_RR = constrain(motor_RR, 1000.0, 2000.0);

        // Update OLED Display
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.print(F("FL:")); display.print((int)motor_FL);
        display.setCursor(64, 0);
        display.print(F("FR:")); display.print((int)motor_FR);
        display.setCursor(0, 16);
        display.print(F("RL:")); display.print((int)motor_RL);
        display.setCursor(64, 16);
        display.print(F("RR:")); display.print((int)motor_RR);
        display.display();
    }
}

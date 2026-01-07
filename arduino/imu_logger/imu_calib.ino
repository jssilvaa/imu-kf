/*
Calibrate the MPU6050

Setup:
- MPU6050 flat on the table
- Gravity aligned with +Z direction (+1 g)

1) Gyroscope calibration:
   Mean of raw gyroscope measurements over 10s is used as constant bias.

2) Accelerometer calibration:
   Bias-only calibration using a single static pose.
   Nominal sensitivity assumed: ±2 g range, 16384 LSB/g.
   No scale or cross-axis calibration is performed here.

NOTE:
- Accelerometer offsets are applied in software ( not written to MPU registers with set*AccelOffset(...) ).
- This script is intended for bias sanity checking.
- Fixed-rate logging is used in a separate data acquisition script for KF tuning.
*/

#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int buffersize = 1000;

// IMPORTANT: initialize all accumulators
long gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;
long acc_x_cal  = 0, acc_y_cal  = 0, acc_z_cal  = 0;

void calibrateGyro() {
  Serial.println("Calibrating gyroscope...");
  for (int i = 0; i < buffersize; i++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gyro_x_cal += gx;
    gyro_y_cal += gy;
    gyro_z_cal += gz;
    delay(3);
  }
  gyro_x_cal /= buffersize;
  gyro_y_cal /= buffersize;
  gyro_z_cal /= buffersize;
}

void calibrateAccel() {
  Serial.println("Calibrating accelerometer...");
  for (int i = 0; i < buffersize; i++) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    acc_x_cal += ax;
    acc_y_cal += ay;
    acc_z_cal += az;
    delay(3);
  }

  acc_x_cal /= buffersize;
  acc_y_cal /= buffersize;
  acc_z_cal /= buffersize;

  // Subtract gravity contribution (+1 g on Z)
  acc_z_cal -= 16384;  // nominal s_i = 16384 LSB/g
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  accelgyro.initialize();

  calibrateGyro();
  calibrateAccel();
}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Bias-corrected accelerometer
  ax -= acc_x_cal;
  ay -= acc_y_cal;
  az -= acc_z_cal;

  // Bias-corrected gyroscope
  gx -= gyro_x_cal;
  gy -= gyro_y_cal;
  gz -= gyro_z_cal;

  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.println(gz);

  delay(100);  // not for KF timing; sanity-check only
}

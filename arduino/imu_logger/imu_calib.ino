#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int buffersize = 1000;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;

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
  long acc_x_cal = 0, acc_y_cal = 0, acc_z_cal = 0;

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

  accelgyro.setXAccelOffset(-acc_x_cal);
  accelgyro.setYAccelOffset(-acc_y_cal);
  accelgyro.setZAccelOffset(16384 - acc_z_cal);
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

  gx -= gyro_x_cal;
  gy -= gyro_y_cal;
  gz -= gyro_z_cal;

  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.print(gz); Serial.println();

  delay(100);
}
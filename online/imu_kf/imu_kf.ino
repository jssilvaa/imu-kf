#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Config 
static const uint32_t FS_HZ  = 100;
static const uint32_t TS_US  = 1000000UL / FS_HZ;

static const uint32_t CALIB_MS = 10000;   // 10 s gyro bias calibration

// Full-scale settings (datasheet nominal sensitivities)
static const float ACC_LSB_PER_G    = 16384.0f;  // ±2g
static const float GYRO_LSB_PER_DPS =   131.0f;  // ±250 dps

// paste results here if 6 face (from calib.json).
// for now use nominal values from datasheet 
static const bool  USE_6FACE_ACCEL_CALIB = false;
static const float ACC_BX = 0.0f, ACC_BY = 0.0f, ACC_BZ = 0.0f;
static const float ACC_SX = ACC_LSB_PER_G, ACC_SY = ACC_LSB_PER_G, ACC_SZ = ACC_LSB_PER_G;

// Gating accel measurement (static when ||a|| ~ 1g)
static const float EPS_STATIC_G = 0.10f; // | ||a|| - 1 | < eps
                                         // i.e. in other words, measurement 
                                         // is not accepted if 10% greater or less than expected 

// KF parameters (tune offline; results should go here ) 
// Units:
// - theta in rad
// - bias in rad/s
// Q entries are process variances, we assume them constant here as a simplification and tune it i guess
static const float Q_THETA = 1e-5f;
static const float Q_BIAS  = 1e-7f;
// R is measurement variance in rad^2 (accel tilt noise); set from offline var, or start with (2deg)^2.
static const float R_MEAS  = (0.034906585f * 0.034906585f); // (2 deg)^2

// KF state (roll and pitch independent) 
struct KF2 {
  float theta;   // rad
  float bias;    // rad/s
  float P00, P01, P10, P11;
};

KF2 kf_roll {0,0, 1,0,0,1};
KF2 kf_pitch{0,0, 1,0,0,1};

// Gyro raw bias (LSB)
float gyro_bias_raw[3] = {0,0,0};

// Run mode
volatile bool calibrated = false;
volatile bool streaming  = false;

// Math utils 
static inline float sqr(float x){ return x*x; }

static inline void accel_to_g(int16_t ax, int16_t ay, int16_t az, float &ax_g, float &ay_g, float &az_g) {
  if (USE_6FACE_ACCEL_CALIB) {
    ax_g = ( (float)ax - ACC_BX ) / ACC_SX;
    ay_g = ( (float)ay - ACC_BY ) / ACC_SY;
    az_g = ( (float)az - ACC_BZ ) / ACC_SZ;
  } else {
    // bias/scale assumed nominal
    ax_g = (float)ax / ACC_LSB_PER_G;
    ay_g = (float)ay / ACC_LSB_PER_G;
    az_g = (float)az / ACC_LSB_PER_G;
  }
}

static inline void accel_tilt(float ax_g, float ay_g, float az_g, float &roll, float &pitch) {
  roll  = atan2f(ay_g, az_g);
  pitch = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g));
}

static inline void kf_step(KF2 &kf, float omega_rad_s, float z_rad, float dt, bool use_meas) {
  // Model:
  // theta_k = theta_{k-1} + dt*(omega - bias) + w_theta
  // bias_k  = bias_{k-1} + w_bias
  //
  // A = [[1, -dt],[0,1]], H = [1,0]

  // Predict state
  kf.theta = kf.theta + dt*(omega_rad_s - kf.bias);
  // kf.bias unchanged in prediction (random walk mean)

  // Predict covariance: P = A P A' + Q
  // with P = [[P00, P01]; 
  //           [P10, P11]]
  const float P00 = kf.P00, P01 = kf.P01, P10 = kf.P10, P11 = kf.P11;

  kf.P00 = P00 - dt*(P01 + P10) + (dt*dt)*P11 + Q_THETA;
  kf.P01 = P01 - dt*P11;
  kf.P10 = P10 - dt*P11;
  kf.P11 = P11 + Q_BIAS;

  // Update if accel not gated, i.e. when twist measurement is reliable
  if (use_meas) {
    const float y = z_rad - kf.theta;           // innovation
    const float S = kf.P00 + R_MEAS;            // scalar
    const float K0 = kf.P00 / S;                // P H' / S
    const float K1 = kf.P10 / S;

    // State update
    kf.theta += K0 * y;
    kf.bias  += K1 * y;

    // Cov update: P = (I - K H) P, H=[1,0]
    // P_new = [[(1-K0)P00, (1-K0)P01],
    //          [P10-K1 P00, P11 - K1 P01]]
    const float P00u = kf.P00, P01u = kf.P01, P10u = kf.P10, P11u = kf.P11;
    kf.P00 = (1.0f - K0) * P00u;
    kf.P01 = (1.0f - K0) * P01u;
    kf.P10 = P10u - K1 * P00u;
    kf.P11 = P11u - K1 * P01u;

    // symmetry enforcement (numerical hygiene)
    const float P01sym = 0.5f*(kf.P01 + kf.P10);
    kf.P01 = P01sym;
    kf.P10 = P01sym;
  }
}

// Calibration 
static void calibrate_gyro_bias_10s() {
  const uint32_t t_start = millis();
  uint32_t n = 0;
  double sum_gx=0, sum_gy=0, sum_gz=0;

  while ((millis() - t_start) < CALIB_MS) {
    int16_t ax,ay,az,gx,gy,gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    sum_gx += gx; sum_gy += gy; sum_gz += gz;
    n++;
    delay(3); // ok during calibration
  }

  gyro_bias_raw[0] = (float)(sum_gx / (double)n);
  gyro_bias_raw[1] = (float)(sum_gy / (double)n);
  gyro_bias_raw[2] = (float)(sum_gz / (double)n);

  // Initialize angle states from accel (one sample after calibration)
  int16_t ax,ay,az,gx,gy,gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  float ax_g, ay_g, az_g;
  accel_to_g(ax,ay,az, ax_g,ay_g,az_g);
  float roll0, pitch0;
  accel_tilt(ax_g,ay_g,az_g, roll0,pitch0); // assumes measurement is valid here, so watch out for that 
  kf_roll.theta  = roll0;
  kf_pitch.theta = pitch0;
  kf_roll.bias = 0; kf_pitch.bias = 0;
  kf_roll.P00 = 1; kf_roll.P01 = 0; kf_roll.P10 = 0; kf_roll.P11 = 1;
  kf_pitch.P00 = 1; kf_pitch.P01 = 0; kf_pitch.P10 = 0; kf_pitch.P11 = 1;

  calibrated = true;
}

// Arduino hooks 
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  if (!mpu.testConnection()) {
    // Fail hard: no sensor detected
    while (1) { delay(100); } // so if it just idles, it means the sensor isn't being detected
  }

  // Configure ranges
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // ASEL = 2
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // FSEL = 0 

  // 100 Hz: base 1 kHz with DLPF on; setRate(9) => 100 Hz
  mpu.setRate(9);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);

  // Wait for host to request calibration
  calibrated = false;
  streaming  = false;
}

void loop() {
  // Host commands:
  // 'a' => calibrate gyro bias (10s), then reply "1"
  // 's' => stop streaming

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'a') {
      calibrate_gyro_bias_10s();
      streaming = true;
      Serial.println("1"); // handshake token expected by python after calib
                           // from here on, it streams data which is parsed and used in the application 
    } else if (c == 's') {
      streaming = false;
    }
  }
  if (!streaming || !calibrated) return;

  static uint32_t t_last_us = micros();
  const uint32_t now_us = micros();
  if ((uint32_t)(now_us - t_last_us) < TS_US) return;
  const float dt = (now_us - t_last_us) * 1e-6f;
  t_last_us += TS_US;

  // Read IMU
  int16_t ax,ay,az,gx,gy,gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  // Accel in g
  float ax_g, ay_g, az_g;
  accel_to_g(ax,ay,az, ax_g,ay_g,az_g);

  // Accel tilt measurement
  float roll_z, pitch_z;
  accel_tilt(ax_g,ay_g,az_g, roll_z, pitch_z);

  // Gating based on accel norm
  const float a_norm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  const bool static_ok = fabsf(a_norm - 1.0f) < EPS_STATIC_G;

  // Gyro in rad/s (subtract raw bias estimated at calibration)
  const float gx_dps = ((float)gx - gyro_bias_raw[0]) / GYRO_LSB_PER_DPS;
  const float gy_dps = ((float)gy - gyro_bias_raw[1]) / GYRO_LSB_PER_DPS;

  const float gx_rad = gx_dps * 0.01745329252f; // times pi / 180
  const float gy_rad = gy_dps * 0.01745329252f; // idem 

  // KF updates: roll uses gx, pitch uses gy
  kf_step(kf_roll,  gx_rad, roll_z,  dt, static_ok);
  kf_step(kf_pitch, gy_rad, pitch_z, dt, static_ok);

  // Output for Python: roll_deg,pitch_deg
  const float roll_deg  = kf_roll.theta  * 57.2957795f; // times 180 / pi
  const float pitch_deg = kf_pitch.theta * 57.2957795f; // idem 

  Serial.print(roll_deg, 3);
  Serial.print(',');
  Serial.println(pitch_deg, 3);
}

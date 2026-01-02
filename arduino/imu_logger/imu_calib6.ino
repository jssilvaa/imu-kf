#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// ---------------- Config ----------------
static const uint32_t FS_HZ  = 100;
static const uint32_t TS_US  = 1000000UL / FS_HZ;

static const uint32_t CALIB_MS = 10000;   // 10s flat on table 
static const uint32_t WAIT_MS  = 2000;    // protocol wait segments

// Nominal sensitivities for ±2g, ±250 dps
static const float ACC_LSB_PER_G    = 16384.0f;
static const float GYRO_LSB_PER_DPS =   131.0f;

// ---------------- State ----------------
enum Mode : uint8_t { MODE_CALIB=0, MODE_IDLE=1, MODE_PROTOCOL=2, MODE_6FACE=3 };
Mode mode = MODE_CALIB;

int phase = -1;           // protocol phase (0,1,2,3...)
char pose[3] = "ZP";      // 6-face pose label: XP XN YP YN ZP ZN (2 chars + '\0')

bool logging = true;      // during CALIB we log; later controlled by commands

uint32_t t0_ms = 0;
uint32_t t_protocol_ms = 0;
uint32_t t_last_us = 0;

// Biases in raw LSB (estimated from CALIB)
float gyro_bias[3] = {0,0,0}; // gx,gy,gz
float acc_bias[3]  = {0,0,0}; // ax,ay,az (z bias adjusted to make +1g at rest)

// CALIB accumulators
uint32_t calib_count = 0;
double sum_ax=0, sum_ay=0, sum_az=0;
double sum_gx=0, sum_gy=0, sum_gz=0;

// ---------------- Utilities ----------------
static void print_header() {
  Serial.println(F("t_us,mode,phase,pose,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw"));
}

static void print_comment_biases() {
  Serial.print(F("# gyro_bias_raw="));
  Serial.print(gyro_bias[0], 3); Serial.print(',');
  Serial.print(gyro_bias[1], 3); Serial.print(',');
  Serial.println(gyro_bias[2], 3);

  Serial.print(F("# acc_bias_raw="));
  Serial.print(acc_bias[0], 3); Serial.print(',');
  Serial.print(acc_bias[1], 3); Serial.print(',');
  Serial.println(acc_bias[2], 3);
}

static void set_pose(const char a, const char b) {
  pose[0] = a;
  pose[1] = b;
  pose[2] = '\0';
}

static void start_protocol() {
  mode = MODE_PROTOCOL;
  t_protocol_ms = millis();
  phase = 0;
  set_pose('P','R'); // "PR" = protocol run
  logging = true;
  Serial.println(F("# protocol start"));
}

static void start_6face() {
  mode = MODE_6FACE;
  phase = -1;
  set_pose('Z','P'); // default
  logging = true;
  Serial.println(F("# 6face start (set pose with XP/XN/YP/YN/ZP/ZN keys)"));
}

static void stop_logging() {
  logging = false;
  mode = MODE_IDLE;
  phase = -1;
  set_pose('I','D'); // "ID"
  Serial.println(F("# logging stop"));
}

static void handle_serial_commands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    // Skip newline
    if (c == '\n' || c == '\r') continue;

    // Start/stop commands
    if (c == 'p') { start_protocol(); continue; }
    if (c == '6') { start_6face();   continue; }
    if (c == 's') { stop_logging();  continue; }

    // 6-face pose commands (only meaningful in MODE_6FACE)
    // Use uppercase for +, lowercase for -
    // X: 'X' -> XP, 'x' -> XN, etc.
    if (mode == MODE_6FACE) {
      if (c == 'X') { set_pose('X','P'); Serial.println(F("# pose=XP")); continue; }
      if (c == 'x') { set_pose('X','N'); Serial.println(F("# pose=XN")); continue; }
      if (c == 'Y') { set_pose('Y','P'); Serial.println(F("# pose=YP")); continue; }
      if (c == 'y') { set_pose('Y','N'); Serial.println(F("# pose=YN")); continue; }
      if (c == 'Z') { set_pose('Z','P'); Serial.println(F("# pose=ZP")); continue; }
      if (c == 'z') { set_pose('Z','N'); Serial.println(F("# pose=ZN")); continue; }
    }
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial) {}

  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("# ERROR: MPU6050 not connected"));
    while(1) {}
  }

  // iff enum differs, adjust here.
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  // With DLPF enabled, base sample is 1kHz; SMPLRT_DIV=9 => 100Hz
  mpu.setRate(9);

  // if library supports DLPF config, ENABLE it (much better results).
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);

  t0_ms = millis();
  t_last_us = micros();

  print_header();
  Serial.println(F("# Commands: p=start protocol, 6=start 6-face, s=stop; in 6-face: X/x Y/y Z/z set pose"));
}

void loop() {
  handle_serial_commands();

  uint32_t now_us = micros();
  if ((uint32_t)(now_us - t_last_us) < TS_US) return;
  t_last_us += TS_US;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  uint32_t now_ms = millis();

  // ---------- CALIB block ----------
  if (mode == MODE_CALIB) {
    // Accumulate for CALIB_MS
    sum_ax += ax; sum_ay += ay; sum_az += az;
    sum_gx += gx; sum_gy += gy; sum_gz += gz;
    calib_count++;

    if ((now_ms - t0_ms) >= CALIB_MS) {
      gyro_bias[0] = (float)(sum_gx / calib_count);
      gyro_bias[1] = (float)(sum_gy / calib_count);
      gyro_bias[2] = (float)(sum_gz / calib_count);

      // Flat-on-table assumption: (ax,ay,az) ≈ (0,0,+1g)
      acc_bias[0] = (float)(sum_ax / calib_count);               // target 0
      acc_bias[1] = (float)(sum_ay / calib_count);               // target 0
      acc_bias[2] = (float)(sum_az / calib_count) - ACC_LSB_PER_G; // target +1g

      print_comment_biases();

      mode = MODE_IDLE;
      logging = false;
      set_pose('I','D');
      Serial.println(F("# CALIB done. Send 'p' for protocol, '6' for 6-face."));
    }
  }

  // ---------- protocol phase update ----------
  if (mode == MODE_PROTOCOL) {
    uint32_t tseg = now_ms - t_protocol_ms;
    if (tseg < WAIT_MS) phase = 0;
    else if (tseg < 2*WAIT_MS) phase = 1;
    else if (tseg < 3*WAIT_MS) phase = 2;
    else phase = 3; // after protocol window (keep logging if you want)
  }

  // ---------- CSV output ----------
  if (!logging && mode != MODE_CALIB) return;

  Serial.print(now_us);
  Serial.print(',');
  Serial.print((int)mode);
  Serial.print(',');
  Serial.print(phase);
  Serial.print(',');
  Serial.print(pose);
  Serial.print(',');

  Serial.print(ax); Serial.print(',');
  Serial.print(ay); Serial.print(',');
  Serial.print(az); Serial.print(',');
  Serial.print(gx); Serial.print(',');
  Serial.print(gy); Serial.print(',');
  Serial.println(gz);
}

// also may need to change here the libs if they start spitting shit
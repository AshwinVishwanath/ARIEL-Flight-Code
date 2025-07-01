#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <ArduinoEigen.h>
#include <math.h>
#include <algorithm>

// =================== BNO055 Calibration Offsets ===================
const adafruit_bno055_offsets_t BNO_CALIBRATION_OFFSETS = {
  7,    // accel_offset_x
  -50,  // accel_offset_y
  -42,  // accel_offset_z
  83,   // mag_offset_x
  115,  // mag_offset_y
  -420, // mag_offset_z
  -1,   // gyro_offset_x
  -1,   // gyro_offset_y
  0,    // gyro_offset_z
  1000, // accel_radius
  769   // mag_radius45
};

// =================== User-Configurable Parameters ===================
const int LED_PIN = 13;

// Roll-control PID parameters
const double PID_KP       = 10.0;
const double PID_KI       = 5.0;
const double PID_KD       = 3.0;
const int    operating_feq = 200;            // loop frequency (Hz)
const double PID_DT       = 1.0 / operating_feq;
const double PID_PWM_FREQ = 20.0;           // PWM frequency (Hz)

// Actuator pins
const int ACTUATOR_LEFT_PIN  = 0;
const int ACTUATOR_RIGHT_PIN = 2;

// Timing & Thresholds
const unsigned long INIT_DURATION               = 2000;  // ms
const unsigned long CALIBRATION_DURATION        = 100;   // ms
const int           ASCENT_BUFFER_SIZE          = 5000;  // samples at 1 kHz
const float         ASCENT_ACCEL_THRESHOLD      = 6.0f;  // m/s²
const unsigned long ASCENT_ACCEL_TIME_THRESHOLD = 600;   // ms

const float  AP_VY_ASCENT_THRESHOLD   = 20.0f;  // m/s
const unsigned long AP_MIN_ASCENT_TIME  = 1000;  // ms
const float  AP_VY_FALL_THRESHOLD      = -5.0f; // m/s
const float  AP_MIN_ALTITUDE           = 10.0f; // m

const unsigned long POST_APOGEE_LOG_DURATION = 5000; // ms
const unsigned long LOW_RATE_LOG_INTERVAL    = 100;  // ms
const float         DESCENT_VY_FAST_THRESHOLD = 20.0f; // m/s

const float  LANDING_ALT_CHANGE_THRESHOLD = 1.0f;   // m
const unsigned long LANDING_TIME_THRESHOLD = 5000;  // ms

// =================== Roll-Control Utilities ===================
template<typename T>
inline T clampVal(T v, T lo, T hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

inline double computeAngleError(double desired, double actual) {
  return desired - actual;
}

inline double computeVelocityError(double angularVelocity) {
  return -angularVelocity;
}

class PIDController {
public:
  PIDController(double Kp, double Ki, double Kd, double dt)
    : Kp(Kp), Ki(Ki), Kd(Kd), dt(dt), integral(0.0) {}

  double update(double angleError, double velError) {
    integral += angleError * dt;
    return Kp * angleError + Ki * integral + Kd * velError;
  }

  void reset() {
    integral = 0.0;
  }

private:
  double Kp, Ki, Kd, dt;
  double integral;
};

class PWMGenerator {
public:
  PWMGenerator(double freq, double dt)
    : freq(freq), dt(dt), timeInCycle(0.0) {}

  int update(double controlSignal) {
    double period = 1.0 / freq;
    timeInCycle += dt;
    if (timeInCycle >= period) timeInCycle -= period;

    double phase = timeInCycle / period;
    double duty  = clampVal(controlSignal, -1.0, 1.0);

    if      (duty >  0.0) return (phase < duty)  ? +1 : 0;
    else if (duty <  0.0) return (phase < -duty) ? -1 : 0;
    else                  return 0;
  }

private:
  double freq, dt, timeInCycle;
};

PIDController rollPid(PID_KP, PID_KI, PID_KD, PID_DT);
PWMGenerator  rollPwm(PID_PWM_FREQ, PID_DT);

// =================== Flight States ===================
enum SystemState {
  ASCENT,
  DESCENT,
  LANDED
};

// =================== Global File Objects ===================
File rawDataFile;
File filteredDataFile;
File systemLogFile;

// =================== State & Buffer Variables ===================
SystemState systemState;

unsigned long stateStartTime        = 0;
unsigned long launchDetectTimestamp = 0;
unsigned long landingDetectTimestamp= 0;

float ascentVyMax    = -1e6f;
float ascentAyMax    = -1e6f;
float maxAltitudeSum = -1e6f;

unsigned long prelaunchIndex = 0;

// Rolling buffer entry
struct LogEntry {
  String rawLine;
  String filteredLine;
};

LogEntry ascentBuffer[ASCENT_BUFFER_SIZE];
int     bufferIndex  = 0;
bool    bufferFull   = false;
bool    bufferDumped = false;

// Acceleration & apogee detection
bool    accelActive        = false;
unsigned long accelStartMicros = 0;

bool    ascentDetected     = false;
unsigned long ascentStartMicros = 0;

bool    apogeeDetected     = false;
unsigned long apogeeDetectMicros = 0;

float   maxAltitude        = 0.0f;

// Landing detection
float   prevAltitude       = 0.0f;
unsigned long landingTimerStart = 0;

// =================== Sensor & EKF Globals ===================
float baselineAlt = 0.0f;
float altitudeBias= 0.0f;

Adafruit_BNO055 bno(55);
Adafruit_BMP3XX  bmp;

static const int n_x = 6;
static const int n_z = 1;

static Eigen::VectorXf x_mean;
static Eigen::MatrixXf P, Q, R;
Eigen::MatrixXf lastKalmanGain = Eigen::MatrixXf::Zero(n_x, n_z);

static bool ekfInitialized = false;

// =================== Function Prototypes ===================
void setupSensors();
float manualCalibrateBMP388();
float getRelativeAltitude();

void setupFiles();
void logSensorData();
void logRawData();
void logFilteredData();
void Summarylog(const String &msg);
void flushAllBuffers();

void ekfInit(float x, float y, float z, float vx, float vy, float vz);
void ekfPredict(float ax, float ay, float az, float dt);
void ekfUpdateBaro(float alt_meas);
void ekfGetState(float &x, float &y, float &z,
                 float &vx, float &vy, float &vz);

void updateIntegratedAngles(float gx, float gy, float gz, float dt);
void getIntegratedAngles(float &roll, float &pitch, float &yaw);

void getSensorData(float &ax, float &ay, float &az,
                   float &gx, float &gy, float &gz,
                   float &mx, float &my, float &mz);

// =================== Setup Function ===================
void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(ACTUATOR_LEFT_PIN,  OUTPUT);
  pinMode(ACTUATOR_RIGHT_PIN, OUTPUT);

  // ---- INIT Blink (1 Hz) ----
  unsigned long t0 = millis();
  unsigned long lastToggle = t0;
  bool ledState = false;

  while (millis() - t0 < INIT_DURATION) {
    if (millis() - lastToggle >= 500) {
      lastToggle = millis();
      ledState = !ledState;
      analogWrite(LED_PIN, ledState ? 255 : 0);
    }
  }
  analogWrite(LED_PIN, 0);
  Summarylog("INIT complete; starting CALIBRATION...");

  // ---- CALIBRATION Blink (4 Hz) ----
  t0 = millis();
  lastToggle = t0;
  ledState = false;

  while (millis() - t0 < CALIBRATION_DURATION) {
    if (millis() - lastToggle >= 125) {
      lastToggle = millis();
      ledState = !ledState;
      analogWrite(LED_PIN, ledState ? 255 : 0);
    }
  }
  analogWrite(LED_PIN, 0);
  Summarylog("Calibration blink complete.");

  // ---- Sensor Init & EKF Setup ----
  setupSensors();
  float alt0 = getRelativeAltitude();

  ekfInit(0.0f, 0.0f, alt0,
          0.0f, 0.0f, 0.0f);
  Summarylog("Calibration complete; baseline altitude = " + String(alt0));

  // ---- File Setup ----
  setupFiles();
  Summarylog("Files opened; transitioning to ASCENT.");

  // ---- Initialize Ascent Variables ----
  bufferIndex         = 0;
  bufferFull          = false;
  bufferDumped        = false;
  accelActive         = false;
  ascentStartMicros   = micros();
  maxAltitude         = alt0;
  ascentDetected      = false;
  apogeeDetected      = false;

  systemState = ASCENT;
  stateStartTime = millis();
}

// =================== Loop Function ===================
void loop() {
  // ---- Always: Read & Fuse Sensors ----
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  getSensorData(ax, ay, az,
                gx, gy, gz,
                mx, my, mz);

  float relAlt = getRelativeAltitude();

  ekfPredict(ax, ay, az, PID_DT);
  ekfUpdateBaro(relAlt);

  float x, y, z, vx, vy, vz;
  ekfGetState(x, y, z,
              vx, vy, vz);

  updateIntegratedAngles(gx, gy, gz, PID_DT);

  float Roll, Pitch, Yaw;
  getIntegratedAngles(Roll, Pitch, Yaw);

  // ---- Always: Roll Control ----
  double angleErr = computeAngleError(0.0,
                                      Roll * (M_PI / 180.0));
  double velErr   = computeVelocityError(gy);
  double ctrl     = rollPid.update(angleErr, velErr);
  int    pwmOut   = rollPwm.update(ctrl);

  Serial.print("Roll: ");  Serial.print(Roll, 3);
  Serial.print(" | Gy: ");  Serial.print(gy,   3);
  Serial.print(" | PWM: "); Serial.println(pwmOut);

  if      (pwmOut >  0) {
    digitalWrite(ACTUATOR_RIGHT_PIN, HIGH);
    digitalWrite(ACTUATOR_LEFT_PIN,  LOW);
  }
  else if (pwmOut <  0) {
    digitalWrite(ACTUATOR_RIGHT_PIN, LOW);
    digitalWrite(ACTUATOR_LEFT_PIN,  HIGH);
  }
  else {
    digitalWrite(ACTUATOR_RIGHT_PIN, LOW);
    digitalWrite(ACTUATOR_LEFT_PIN,  LOW);
  }

  // ---- State-Specific Behavior ----
  switch (systemState) {
    case ASCENT: {
      // LED blinking at 2 Hz (toggle every 250 ms)
      const unsigned long ledInterval = 250;
      static unsigned long lastLed = 0;
      static bool ledBlink = false;
      if (millis() - lastLed >= ledInterval) {
        lastLed = millis();
        ledBlink = !ledBlink;
        analogWrite(LED_PIN, ledBlink ? 255 : 0);
      }

      // sensor loop at operating_feq
      static unsigned long loopLast = micros();
      const unsigned long loopInt = 1000000UL / operating_feq;
      unsigned long now = micros();
      if (now - loopLast >= loopInt) {
        loopLast = now;
        float dt = PID_DT;

        // (Use the same ax,ay,az,gx,gy,gz,mx,my,mz,relAlt,x,y,z,vx,vy,vz,Roll,Pitch,Yaw)

        // Update maxAltitude
        if (y > maxAltitude) maxAltitude = y;

        // Detect ascent start
        if (!ascentDetected && (vy > AP_VY_ASCENT_THRESHOLD)) {
          ascentDetected = true;
          ascentStartMicros = micros();
        }
        launchDetectTimestamp = micros();

        // Rolling buffer index
        int idxOffset = 2 * operating_feq;
        int twoSecIdx = bufferIndex - idxOffset;
        if (twoSecIdx < 0) twoSecIdx += ASCENT_BUFFER_SIZE;
        prelaunchIndex = twoSecIdx;

        // Buffering
        if (!bufferDumped) {
          unsigned long tsUs = micros();
          unsigned long t_s  = tsUs / 1000000;
          String rawLine = String(t_s) + "," +
            String(ax,3) + "," + String(ay,3) + "," + String(az,3) + "," +
            String(gx,3) + "," + String(gy,3) + "," + String(gz,3) + "," +
            String(mx,3) + "," + String(my,3) + "," + String(mz,3) + "," +
            String(relAlt,3);
          String filtLine = String(t_s) + "," +
            String(x,3) + "," + String(y,3) + "," + String(z,3) + "," +
            String(vx,3) + "," + String(vy,3) + "," + String(vz,3) + "," +
            String(Roll,3) + "," + String(Pitch,3) + "," + String(Yaw,3);

          ascentBuffer[bufferIndex].rawLine      = rawLine;
          ascentBuffer[bufferIndex].filteredLine = filtLine;
          bufferIndex = (bufferIndex + 1) % ASCENT_BUFFER_SIZE;
          if (bufferIndex == 0) bufferFull = true;

          if (ay > ASCENT_ACCEL_THRESHOLD) {
            if (!accelActive) {
              accelActive = true;
              accelStartMicros = millis();
            } else if (millis() - accelStartMicros >= ASCENT_ACCEL_TIME_THRESHOLD) {
              Summarylog("LAUNCH DETECTED; dumping buffer to SD.");
              int count = bufferFull ? ASCENT_BUFFER_SIZE : bufferIndex;
              for (int i = 0; i < count; i++) {
                rawDataFile.println(ascentBuffer[i].rawLine);
                filteredDataFile.println(ascentBuffer[i].filteredLine);
              }
              rawDataFile.flush();
              filteredDataFile.flush();
              bufferDumped = true;
              bufferIndex = 0;
              bufferFull  = false;
              accelActive = false;
            }
          } else {
            accelActive = false;
          }
        } else {
          logSensorData();
        }

        if (vy > ascentVyMax)    ascentVyMax = vy;
        if (ay > ascentAyMax)    ascentAyMax = ay;
        if (y  > maxAltitudeSum) maxAltitudeSum = y;

        // Apogee detection
        if (!apogeeDetected && ascentDetected) {
          if ((millis() - ascentStartMicros >= AP_MIN_ASCENT_TIME) &&
              (vy < AP_VY_FALL_THRESHOLD) &&
              (maxAltitude > AP_MIN_ALTITUDE) &&
              (y < maxAltitude)) {
            apogeeDetected     = true;
            apogeeDetectMicros = millis();
            Summarylog("APOGEE DETECTED.");
            systemState        = DESCENT;
            stateStartTime     = millis();
          }
        }
      }
      break;
    }

    case DESCENT: {
      // LED blinking at 2 Hz
      const unsigned long ledInterval = 250;
      static unsigned long lastLed = 0;
      static bool ledBlink = false;
      if (millis() - lastLed >= ledInterval) {
        lastLed   = millis();
        ledBlink  = !ledBlink;
        analogWrite(LED_PIN, ledBlink ? 255 : 0);
      }

      // sensor loop
      static unsigned long loopLast = micros();
      const unsigned long loopInt = 1000000UL / operating_feq;
      unsigned long now = micros();
      if (now - loopLast >= loopInt) {
        loopLast = now;
        float dt = PID_DT;

        // (Use ax,ay,az,gx,gy,gz,mx,my,mz,relAlt,x,y,z,vx,vy,vz,Roll,Pitch,Yaw)

        // Logging rate adaptation
        static unsigned long lastLog = 0;
        if ((millis() - apogeeDetectMicros < POST_APOGEE_LOG_DURATION) &&
            (fabs(vy) <= DESCENT_VY_FAST_THRESHOLD)) {
          if (millis() - lastLog >= LOW_RATE_LOG_INTERVAL) {
            logSensorData();
            lastLog = millis();
          }
        } else {
          logSensorData();
        }

        // Landing detection
        if (landingTimerStart == 0) {
          landingTimerStart = millis();
          prevAltitude      = y;
        }
        if ((fabs(y - prevAltitude) < LANDING_ALT_CHANGE_THRESHOLD) &&
            (fabs(relAlt - prevAltitude) < LANDING_ALT_CHANGE_THRESHOLD)) {
          if (millis() - landingTimerStart >= LANDING_TIME_THRESHOLD) {
            Summarylog("Landing detected; transitioning to LANDED.");
            systemState = LANDED;
            stateStartTime = millis();
            landingDetectTimestamp = millis();
          }
        } else {
          landingTimerStart = millis();
          prevAltitude      = y;
        }

        // Serial telemetry
        Serial.print(">");
        Serial.print("altitude:"); Serial.print(y,3);
        Serial.print(", vy:");       Serial.print(vy,3);
        Serial.print(", roll:");     Serial.print(Roll,3);
        Serial.print(", pitch:");    Serial.print(Pitch,3);
        Serial.print(", yaw:");      Serial.print(Yaw,3);
        Serial.print(", ay:");       Serial.print(ay,3);
        Serial.println();
      }
      break;
    }

    case LANDED: {
      // LED blinking at 1 Hz
      const unsigned long ledInterval = 500;
      static unsigned long lastLed = 0;
      static bool ledBlink = false;
      if (millis() - lastLed >= ledInterval) {
        lastLed   = millis();
        ledBlink  = !ledBlink;
        analogWrite(LED_PIN, ledBlink ? 255 : 0);
      }

      // Final logging & summary
      logFilteredData();
      flushAllBuffers();

      Summarylog("Flight ended. Logging stopped.");
      Summarylog("===== FLIGHT SUMMARY =====");
      Summarylog("Launch at: " + String(launchDetectTimestamp) + " µs");
      Summarylog("Prelaunch entry: " +
                 ascentBuffer[prelaunchIndex].filteredLine);
      float apogeeTimeSec = (apogeeDetectMicros - launchDetectTimestamp) / 1000.0f;
      Summarylog("Apogee at: " + String(apogeeTimeSec,2) + " s after launch");
      unsigned long landingTimeCorrected = landingDetectTimestamp - LANDING_TIME_THRESHOLD;
      Summarylog("Landing at: " + String(landingTimeCorrected) + " ms");
      Summarylog("Max vy: " + String(ascentVyMax,3) + " m/s");
      Summarylog("Max ay: " + String(ascentAyMax,3) + " m/s²");
      Summarylog("Max altitude: " + String(maxAltitudeSum,3) + " m");
      Summarylog("===========================");
      
      while (1) {
        // Idle forever
      }
      break;
    }
  }
}

// =================== Function Implementations ===================

void setupSensors() {
  // BNO055 init
  if (!bno.begin()) {
    Summarylog("ERROR: BNO055 init failed!"); while (1);
  }
  bno.setExtCrystalUse(true);
  bno.setSensorOffsets(BNO_CALIBRATION_OFFSETS);
  Summarylog("BNO055 initialized.");

  // BMP388 init
  if (!bmp.begin_I2C(0x76, &Wire1)) {
    Summarylog("ERROR: BMP388 init failed!"); while (1);
  }
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);
  Summarylog("BMP388 initialized.");

  altitudeBias = manualCalibrateBMP388();
  if (bmp.performReading()) {
    baselineAlt = bmp.readAltitude(1013.25) - altitudeBias;
  }
}

float manualCalibrateBMP388() {
  Summarylog("BMP388 calibration start...");
  const int N = 1000;
  float sum = 0.0f;
  for (int i = 0; i < N; i++) {
    if (bmp.performReading()) sum += bmp.readAltitude(1013.25);
    if (i % 100 == 0) {
      char buf[50];
      sprintf(buf, "Calib progress: %.1f%%", (i/(float)N)*100);
      Summarylog(buf);
    }
  }
  float bias = sum / N;
  Summarylog("BMP388 bias = " + String(bias));
  return bias;
}

float getRelativeAltitude() {
  if (!bmp.performReading()) {
    Summarylog("ERROR: baro read failed");
    return 0.0f;
  }
  float a = bmp.readAltitude(1013.25) - altitudeBias;
  return a - baselineAlt;
}

void getSensorData(float &ax, float &ay, float &az,
                   float &gx, float &gy, float &gz,
                   float &mx, float &my, float &mz) {
  sensors_event_t ae, ge, me;
  bno.getEvent(&ae, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&ge, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&me, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  ax = ae.acceleration.x; ay = ae.acceleration.y; az = ae.acceleration.z;
  gx = ge.gyro.x;         gy = ge.gyro.y;         gz = ge.gyro.z;
  mx = me.magnetic.x;     my = me.magnetic.y;     mz = me.magnetic.z;
}

static float iRoll=0, iPitch=0, iYaw=0;
void updateIntegratedAngles(float gx, float gy, float gz, float dt) {
  const float alpha = 0.7f;
  static float fGx=0, fGy=0, fGz=0;
  fGx = alpha*gx + (1-alpha)*fGx;
  fGy = alpha*gy + (1-alpha)*fGy;
  fGz = alpha*gz + (1-alpha)*fGz;
  iRoll  += fGx  * (180.0/M_PI) * dt;
  iPitch += fGy  * (180.0/M_PI) * dt;
  iYaw   += fGz  * (180.0/M_PI) * dt;
  if (iRoll  < 0) iRoll  += 360; else if (iRoll  >=360) iRoll  -=360;
  if (iPitch < 0) iPitch += 360; else if (iPitch >=360) iPitch -=360;
  if (iYaw   < 0) iYaw   += 360; else if (iYaw   >=360) iYaw   -=360;
}

void getIntegratedAngles(float &roll, float &pitch, float &yaw) {
  roll = iRoll; pitch = iPitch; yaw = iYaw;
}

void ekfInit(float x, float y, float z, float vx, float vy, float vz) {
  x_mean = Eigen::VectorXf(n_x);
  P = Eigen::MatrixXf::Identity(n_x,n_x)*0.1f;
  Q = Eigen::MatrixXf::Zero(n_x,n_x);
  R = Eigen::MatrixXf::Zero(n_z,n_z);
  x_mean << x,y,z,vx,vy,vz;
  Q(0,0)=0.02f; Q(1,1)=0.04f; Q(2,2)=0.03f;
  Q(3,3)=0.4f;  Q(4,4)=0.36f; Q(5,5)=0.32f;
  R(0,0)=0.3f;
  ekfInitialized = true;
}

void ekfPredict(float ax, float ay, float az, float dt) {
  if (!ekfInitialized) return;
  float x=x_mean(0), y=x_mean(1), z=x_mean(2),
        vx=x_mean(3), vy=x_mean(4), vz=x_mean(5);
  auto f = [&](float vx_,float vy_,float vz_,float ax_,float ay_,float az_){
    Eigen::VectorXf d(n_x);
    d << vx_,vy_,vz_,ax_,ay_,az_;
    return d;
  };
  Eigen::VectorXf k1=f(vx,vy,vz,ax,ay,az),
                   k2=f(vx+0.5f*k1(3)*dt, vy+0.5f*k1(4)*dt, vz+0.5f*k1(5)*dt, ax,ay,az),
                   k3=f(vx+0.5f*k2(3)*dt, vy+0.5f*k2(4)*dt, vz+0.5f*k2(5)*dt, ax,ay,az),
                   k4=f(vx+k3(3)*dt,     vy+k3(4)*dt,     vz+k3(5)*dt,     ax,ay,az);
  Eigen::VectorXf dx = (dt/6.0f)*(k1 + 2*k2 + 2*k3 + k4);
  x  += dx(0); y  += dx(1); z  += dx(2);
  vx += dx(3); vy += dx(4); vz += dx(5);
  Eigen::VectorXf x_pred(n_x); x_pred << x,y,z,vx,vy,vz;
  Eigen::MatrixXf f_mat = Eigen::MatrixXf::Identity(n_x,n_x);
  f_mat(0,3)=dt; f_mat(1,4)=dt; f_mat(2,5)=dt;
  P = f_mat * P * f_mat.transpose() + Q;
  x_mean = x_pred;
}

void ekfUpdateBaro(float alt_meas) {
  if (!ekfInitialized) return;
  Eigen::VectorXf z(1), h(1);
  z(0)=alt_meas; h(0)=x_mean(1);
  Eigen::VectorXf y_err = z - h;
  Eigen::MatrixXf H = Eigen::MatrixXf::Zero(1,n_x);
  H(0,1)=1.0f;
  Eigen::MatrixXf S = H*P*H.transpose() + R;
  Eigen::MatrixXf K = P*H.transpose()*S.inverse();
  x_mean += K * y_err;
  P = (Eigen::MatrixXf::Identity(n_x,n_x) - K*H) * P;
  lastKalmanGain = K;
}

void ekfGetState(float &x, float &y, float &z,
                 float &vx, float &vy, float &vz) {
  x = x_mean(0);
  y = x_mean(1);
  z = x_mean(2);
  vx = x_mean(3);
  vy = x_mean(4);
  vz = x_mean(5);
}

// ------------------- Logging & File I/O -------------------

String rawDataBuffer       = "";
String filteredDataBuffer  = "";
String systemLogBuffer     = "";
int    rawDataCount        = 0;
int    filteredDataCount   = 0;
int    systemLogCount      = 0;
const int RAW_THRESH  = 100;
const int FILT_THRESH = 100;
const int SYS_THRESH  = 1;

void flushBuffer(File &f, String &b, int &c) {
  if (f) { f.print(b); f.flush(); }
  b = ""; c = 0;
}

void flushSystemLog() {
  flushBuffer(systemLogFile, systemLogBuffer, systemLogCount);
}

void Summarylog(const String &msg) {
  unsigned long t = micros();
  systemLogBuffer += String(t) + ": " + msg + "\n";
  systemLogCount++;
  if (systemLogCount >= SYS_THRESH) {
    flushSystemLog();
  }
}

void logData(File &f, String &b, int &c, const String &d, int thresh) {
  b += d + "\n";
  c++;
  if (c >= thresh) {
    flushBuffer(f, b, c);
  }
}

void setupFiles() {
  // Initialize SD card
  if (!SD.begin(BUILTIN_SDCARD)) {
    Summarylog("ERROR: SD init failed!");
  } else {
    Summarylog("SD initialized.");
  }

  // Open files
  rawDataFile      = SD.open("Raw_Data.csv",      FILE_WRITE);
  filteredDataFile = SD.open("Filtered_Data.csv", FILE_WRITE);
  systemLogFile    = SD.open("summarylog.txt",    FILE_WRITE);

  if (!rawDataFile || !filteredDataFile || !systemLogFile) {
    Summarylog("ERROR: File open failed!");
  } else {
    Summarylog("Opened Raw_Data.csv, Filtered_Data.csv, summarylog.txt");
  }

  // Write headers
  if (rawDataFile) {
    rawDataFile.println("time(s),ax,ay,az,gx,gy,gz,mx,my,mz,baroAlt");
    rawDataFile.flush();
  }
  if (filteredDataFile) {
    filteredDataFile.println("time(s),x,y,z,vx,vy,vz,roll,pitch,yaw");
    filteredDataFile.flush();
  }
}

void logRawData() {
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  getSensorData(ax, ay, az, gx, gy, gz, mx, my, mz);
  bmp.performReading();
  float baro = getRelativeAltitude();

  unsigned long t_s = micros() / 1000000;
  String line = String(t_s) + "," +
    String(ax,3) + "," + String(ay,3) + "," + String(az,3) + "," +
    String(gx,3) + "," + String(gy,3) + "," + String(gz,3) + "," +
    String(mx,3) + "," + String(my,3) + "," + String(mz,3) + "," +
    String(baro,3);

  logData(rawDataFile, rawDataBuffer, rawDataCount, line, RAW_THRESH);
}

void logFilteredData() {
  float x,y,z,vx,vy,vz;
  ekfGetState(x,y,z,vx,vy,vz);
  float r,p,yw;
  getIntegratedAngles(r,p,yw);

  unsigned long t_s = micros() / 1000000;
  String line = String(t_s) + "," +
    String(x,3) + "," + String(y,3) + "," + String(z,3) + "," +
    String(vx,3) + "," + String(vy,3) + "," + String(vz,3) + "," +
    String(r,3) + "," + String(p,3) + "," + String(yw,3);

  logData(filteredDataFile, filteredDataBuffer, filteredDataCount, line, FILT_THRESH);
}

void logSensorData() {
  logRawData();
  logFilteredData();
}

void flushAllBuffers() {
  flushBuffer(rawDataFile,      rawDataBuffer,      rawDataCount);
  flushBuffer(filteredDataFile, filteredDataBuffer, filteredDataCount);
  flushSystemLog();
}

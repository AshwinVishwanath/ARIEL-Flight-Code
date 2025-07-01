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

#define RAD_TO_DEG 57.295779513082320876f

// =================== Project Declarations ===================

// ---- BNO_LUT.h content ----
const adafruit_bno055_offsets_t BNO_CALIBRATION_OFFSETS = {
  7,      // accel_offset_x
  -50,    // accel_offset_y
  -42,    // accel_offset_z
  83,     // mag_offset_x
  115,    // mag_offset_y
  -420,   // mag_offset_z
  -1,     // gyro_offset_x
  -1,     // gyro_offset_y
  0,      // gyro_offset_z
  1000,   // accel_radius
  769     // mag_radius45
};

// ==== User-configurable parameters ====
const int LED_PIN = 13;

// PID roll control
const double PID_KP = 10.0;
const double PID_KI = 5.0;
const double PID_KD = 3.0;
const int operating_feq = 200;          // main loop frequency (Hz)
const double PID_DT = 1.0 / operating_feq; // controller timestep (s)
const double PID_PWM_FREQ = 20.0;       // PWM carrier frequency (Hz)
const int ACTUATOR_LEFT_PIN  = 0;       // anticlockwise rotation, servo 1
const int ACTUATOR_RIGHT_PIN = 2;       // clockwise rotation , servo 3

// Timing and threshold constants
const unsigned long INIT_DURATION              = 2000;  // ms in INIT state
const unsigned long CALIBRATION_DURATION       = 100;   // ms after sensor setup
const int  ASCENT_BUFFER_SIZE                  = 5000;  // samples at 1 kHz
const float ASCENT_ACCEL_THRESHOLD             = 6.0f;  // m/s² acceleration
const unsigned long ASCENT_ACCEL_TIME_THRESHOLD = 600;  // ms high accel

// Apogee detection
const float AP_VY_ASCENT_THRESHOLD  = 20.0f;  // m/s minimal ascent velocity
const unsigned long AP_MIN_ASCENT_TIME = 1000; // ms of sustained ascent
const float AP_VY_FALL_THRESHOLD   = -5.0f;  // m/s falling velocity
const float AP_MIN_ALTITUDE        = 10.0f;  // m minimum altitude before apogee

// Post-apogee logging
const unsigned long POST_APOGEE_LOG_DURATION = 5000;  // ms
const unsigned long LOW_RATE_LOG_INTERVAL    = 100;   // ms interval for 10Hz
const float DESCENT_VY_FAST_THRESHOLD       = 20.0f;  // m/s fast descent

// Landing detection
const float LANDING_ALT_CHANGE_THRESHOLD    = 1.0f;   // m of altitude change
const unsigned long LANDING_TIME_THRESHOLD  = 5000;   // ms of minimal change

// === Roll control PID components ===
template<typename T>
inline T clampVal(T v, T lo, T hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

inline double computeAngleError(double desiredAngle, double actualAngle) {
  return desiredAngle - actualAngle;
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
    double P = Kp * angleError;
    double I = Ki * integral;
    double D = Kd * velError;
    return P + I + D;
  }

  void reset() { integral = 0.0; }

private:
  double Kp, Ki, Kd, dt;
  double integral;
};

class PWMGenerator {
public:
  PWMGenerator(double pwmFrequency, double dt)
    : freq(pwmFrequency), dt(dt), timeInCycle(0.0) {}

  int update(double controlSignal) {
    double period = 1.0 / freq;
    timeInCycle += dt;
    if (timeInCycle >= period) timeInCycle -= period;
    double phase = timeInCycle / period;
    double duty = clampVal(controlSignal, -1.0, 1.0);
    if (duty > 0) return (phase < duty) ? +1 : 0;
    else if (duty < 0) return (phase < -duty) ? -1 : 0;
    else return 0;
  }

private:
  double freq, dt, timeInCycle;
};

PIDController rollPid(PID_KP, PID_KI, PID_KD, PID_DT);
PWMGenerator rollPwm(PID_PWM_FREQ, PID_DT);

// Flight state definitions:
enum SystemState {
  ASCENT,        // on Launch Detect, Runs this section
  DESCENT,       // After Apogee Detection, runs this
  LANDED         // On Landing, logging summary then idle
};

// ----------------------------------------------------------
// Global File objects (three files: raw data, filtered data, system log)
// ----------------------------------------------------------
File rawDataFile;       // Will store raw IMU + barometer data
File filteredDataFile;  // Will store EKF state + orientation data
File systemLogFile;     // Will store system log messages

// State & timing globals
SystemState systemState;
unsigned long stateStartTime = 0;
unsigned long launchDetectTimestamp = 0;
unsigned long landingDetectTimestamp = 0;
float ascentBufferVyMax = -9999.0f;
float ascentBufferAyMax = -9999.0f;
float maxAltitudeSummary = -9999.0f;
unsigned long ascentEntryTwoSecondsBeforeLaunchIndex = 0;

// =================== Global Variables for Ascent Rolling Buffer ===================
struct LogEntry {
  String rawLine;      // Raw data CSV line
  String filteredLine; // Filtered data CSV line
};

LogEntry ascentRollingBuffer[ASCENT_BUFFER_SIZE];
int ascentBufferIndex = 0;
bool ascentBufferFull = false;
bool ascentBufferDumped = false;

// For detecting continuous high acceleration during ascent:
bool ascentAccelActive = false;
unsigned long ascentAccelStartTime = 0;

// For apogee detection during ascent:
bool ascentDetected = false;
unsigned long ascentStartTime = 0;
float maxAltitude = 0.0;

// Apogee detection flags:
bool apogeeDetected = false;
unsigned long apogeeTimestamp = 0;

// Landing detection variables:
float prevAltitude = 0.0;
unsigned long landingTimerStart = 0;

// =================== Sensor & EKF globals ===================
float baselineAltitude = 0.0f; // Baseline for relative altitude
float altitudeBias = 0.0f;     // Bias used to correct altitude readings

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BMP3XX bmp;

// EKF internal
static const int n_x = 6;
static const int n_z = 1;
static Eigen::VectorXf x_mean;
static Eigen::MatrixXf P, Q, R;
Eigen::MatrixXf lastKalmanGain = Eigen::MatrixXf::Zero(n_x, n_z);
static bool ekfInitialized = false;

// ---- Function Prototypes ----
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
void ekfGetState(float &x, float &y, float &z, float &vx, float &vy, float &vz);

void updateIntegratedAngles(float gx, float gy, float gz, float dt);
void getIntegratedAngles(float &roll, float &pitch, float &yaw);

void getSensorData(float &ax, float &ay, float &az, 
                   float &gx, float &gy, float &gz, 
                   float &mx, float &my, float &mz);

// =================== Setup Function ===================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ACTUATOR_LEFT_PIN, OUTPUT);
  pinMode(ACTUATOR_RIGHT_PIN, OUTPUT);
  stateStartTime = millis();

  // INIT (in setup only): blink LED at 1 Hz for INIT_DURATION
  unsigned long initStart = millis();
  unsigned long lastToggle = initStart;
  bool ledState = false;
  while (millis() - initStart < INIT_DURATION) {
    if (millis() - lastToggle >= 500) {
      lastToggle = millis();
      ledState = !ledState;
      analogWrite(LED_PIN, ledState ? 255 : 0);
    }
  }
  analogWrite(LED_PIN, 0);
  Summarylog("INIT complete, starting sensor setup...");

  // CALIBRATION (in setup only): blink LED at 4 Hz for CALIBRATION_DURATION
  unsigned long calibStart = millis();
  lastToggle = calibStart;
  ledState = false;
  while (millis() - calibStart < CALIBRATION_DURATION) {
    if (millis() - lastToggle >= 125) {
      lastToggle = millis();
      ledState = !ledState;
      analogWrite(LED_PIN, ledState ? 255 : 0);
    }
  }
  analogWrite(LED_PIN, 0);
  Summarylog("Calibration blink complete...");

  // Sensor init & EKF setup
  setupSensors();
  float alt0 = getRelativeAltitude();
  ekfInit(0.0f, 0.0f, alt0, 0.0f, 0.0f, 0.0f);
  Summarylog(String("Calibration complete, Baseline Altitude: ") + alt0);

  // File setup
  setupFiles();
  Summarylog("Calibration done. Transitioning to ASCENT state.");

  // Initialize ascent variables
  ascentBufferIndex = 0;
  ascentBufferFull = false;
  ascentBufferDumped = false;
  ascentAccelActive = false;
  ascentStartTime = micros();
  maxAltitude = alt0;
  ascentDetected = false;
  apogeeDetected = false;

  // Start in ASCENT
  systemState = ASCENT;
  stateStartTime = millis();
}

// =================== Loop Function ===================
void loop() {
  switch (systemState) {

    case ASCENT: {
      // LED blinking at 2 Hz (toggle every 250 ms)
      const unsigned long ledInterval = 250;
      static unsigned long lastLedToggle = 0;
      static bool ledBlinkState = false;
      if (millis() - lastLedToggle >= ledInterval) {
        lastLedToggle = millis();
        ledBlinkState = !ledBlinkState;
        analogWrite(LED_PIN, ledBlinkState ? 255 : 0);
      }

      // sensor processing loop at operating_feq
      static unsigned long loopStartTime = micros();
      const unsigned long loopInterval = 1000000UL / operating_feq;
      unsigned long currentTime = micros();

      if (currentTime - loopStartTime >= loopInterval) {
        loopStartTime = currentTime;
        float dt = PID_DT;

        // Read sensors and update EKF/orientation
        float ax, ay, az, gx, gy, gz, mx, my, mz;
        getSensorData(ax, ay, az, gx, gy, gz, mx, my, mz);
        float relAlt = getRelativeAltitude();
        ekfPredict(ax, ay, az, dt);
        ekfUpdateBaro(relAlt);
        float x, y, z, vx, vy, vz;
        ekfGetState(x, y, z, vx, vy, vz);
        updateIntegratedAngles(gx, gy, gz, dt);
        float Roll, Pitch, Yaw;
        getIntegratedAngles(Roll, Pitch, Yaw);

        // Always run roll control (no lockouts)
        double angleErr = computeAngleError(0.0, Roll / RAD_TO_DEG);
        double velErr   = computeVelocityError(gy);
        double control  = rollPid.update(angleErr, velErr);
        int pwmOut      = rollPwm.update(control);
        if (pwmOut > 0) {
          digitalWrite(ACTUATOR_RIGHT_PIN, HIGH);
          digitalWrite(ACTUATOR_LEFT_PIN, LOW);
        } else if (pwmOut < 0) {
          digitalWrite(ACTUATOR_RIGHT_PIN, LOW);
          digitalWrite(ACTUATOR_LEFT_PIN, HIGH);
        } else {
          digitalWrite(ACTUATOR_RIGHT_PIN, LOW);
          digitalWrite(ACTUATOR_LEFT_PIN, LOW);
        }

        // Update maximum altitude for apogee detection.
        if (y > maxAltitude) {
          maxAltitude = y;
        }

        // Mark ascent detection when vertical velocity exceeds threshold.
        if (!ascentDetected && (vy > AP_VY_ASCENT_THRESHOLD)) {
          ascentDetected = true;
          ascentStartTime = micros();
        }
        launchDetectTimestamp = micros();

        // Index for 2 seconds before launch, assuming operating_feq
        int indexOffset = 2 * operating_feq;
        int twoSecIndex = ascentBufferIndex - indexOffset;
        if (twoSecIndex < 0) twoSecIndex += ASCENT_BUFFER_SIZE;
        ascentEntryTwoSecondsBeforeLaunchIndex = twoSecIndex;

        // Rolling buffer handling
        if (!ascentBufferDumped) {
          unsigned long timestamp = micros();
          unsigned long Time_S = timestamp / 1000000;
          String rawLine = String(Time_S) + "," +
                           String(ax, 3) + "," + String(ay, 3) + "," + String(az, 3) + "," +
                           String(gx, 3) + "," + String(gy, 3) + "," + String(gz, 3) + "," +
                           String(mx, 3) + "," + String(my, 3) + "," + String(mz, 3) + "," +
                           String(relAlt, 3);
          String filteredLine = String(Time_S) + "," +
                                String(x, 3) + "," + String(y, 3) + "," + String(z, 3) + "," +
                                String(vx, 3) + "," + String(vy, 3) + "," + String(vz, 3) + "," +
                                String(Roll, 3) + "," + String(Pitch, 3) + "," + String(Yaw, 3);

          ascentRollingBuffer[ascentBufferIndex].rawLine      = rawLine;
          ascentRollingBuffer[ascentBufferIndex].filteredLine = filteredLine;
          ascentBufferIndex = (ascentBufferIndex + 1) % ASCENT_BUFFER_SIZE;
          if (ascentBufferIndex == 0) ascentBufferFull = true;

          if (ay > ASCENT_ACCEL_THRESHOLD) {
            if (!ascentAccelActive) {
              ascentAccelActive = true;
              ascentAccelStartTime = millis();
            } else if (millis() - ascentAccelStartTime >= ASCENT_ACCEL_TIME_THRESHOLD) {
              Summarylog("LAUNCH DETECTED; dumping rolling buffer to SD.");
              int count = (ascentBufferFull ? ASCENT_BUFFER_SIZE : ascentBufferIndex);
              for (int i = 0; i < count; i++) {
                rawDataFile.println(ascentRollingBuffer[i].rawLine);
                filteredDataFile.println(ascentRollingBuffer[i].filteredLine);
              }
              rawDataFile.flush();
              filteredDataFile.flush();
              ascentBufferDumped = true;
              ascentBufferIndex = 0;
              ascentBufferFull = false;
              ascentAccelActive = false;
            }
          } else {
            ascentAccelActive = false;
          }
        } else {
          logSensorData();
        }

        if (vy > ascentBufferVyMax) ascentBufferVyMax = vy;
        if (ay > ascentBufferAyMax) ascentBufferAyMax = ay;
        if (y > maxAltitudeSummary) maxAltitudeSummary = y;

        // Apogee detection
        if (!apogeeDetected && ascentDetected) {
          if ((millis() - ascentStartTime >= AP_MIN_ASCENT_TIME) &&
              (vy < AP_VY_FALL_THRESHOLD) &&
              (maxAltitude > AP_MIN_ALTITUDE) &&
              (y < maxAltitude)) {
            apogeeDetected = true;
            apogeeTimestamp = millis();
            Summarylog("APOGEE DETECTED.");
            systemState = DESCENT;
            stateStartTime = millis();
          }
        }
      }

      break;
    }

    case DESCENT: {
      // LED blinking at 2 Hz (toggle every 250 ms)
      const unsigned long ledInterval = 250;
      static unsigned long lastLedToggleD = 0;
      static bool ledBlinkStateD = false;
      if (millis() - lastLedToggleD >= ledInterval) {
        lastLedToggleD = millis();
        ledBlinkStateD = !ledBlinkStateD;
        analogWrite(LED_PIN, ledBlinkStateD ? 255 : 0);
      }

      // sensor processing loop in descent at operating_feq
      static unsigned long loopStartTimeD = micros();
      const unsigned long loopIntervalD = 1000000UL / operating_feq;
      unsigned long currentTimeD = micros();
      if (currentTimeD - loopStartTimeD >= loopIntervalD) {
        loopStartTimeD = currentTimeD;
        float dt = PID_DT;
        
        // Get sensor data and update EKF as before
        float ax, ay, az, gx, gy, gz, mx, my, mz;
        getSensorData(ax, ay, az, gx, gy, gz, mx, my, mz);
        float relAlt = getRelativeAltitude();
        ekfPredict(ax, ay, az, dt);
        ekfUpdateBaro(relAlt);
        float x, y, z, vx, vy, vz;
        ekfGetState(x, y, z, vx, vy, vz);
        updateIntegratedAngles(gx, gy, gz, dt);
        float Roll, Pitch, Yaw;
        getIntegratedAngles(Roll, Pitch, Yaw);

        // Always run roll control (no lockouts)
        double angleErr = computeAngleError(0.0, Roll / RAD_TO_DEG);
        double velErr   = computeVelocityError(gy);
        double control  = rollPid.update(angleErr, velErr);
        int pwmOut      = rollPwm.update(control);
        if (pwmOut > 0) {
          digitalWrite(ACTUATOR_RIGHT_PIN, HIGH);
          digitalWrite(ACTUATOR_LEFT_PIN, LOW);
        } else if (pwmOut < 0) {
          digitalWrite(ACTUATOR_RIGHT_PIN, LOW);
          digitalWrite(ACTUATOR_LEFT_PIN, HIGH);
        } else {
          digitalWrite(ACTUATOR_RIGHT_PIN, LOW);
          digitalWrite(ACTUATOR_LEFT_PIN, LOW);
        }

        // Decide on logging rate
        static unsigned long lastLogTime = 0;
        if ((millis() - apogeeTimestamp < POST_APOGEE_LOG_DURATION) &&
            (fabs(vy) <= DESCENT_VY_FAST_THRESHOLD)) {
          if (millis() - lastLogTime >= LOW_RATE_LOG_INTERVAL) {
            logSensorData();
            lastLogTime = millis();
          }
        } else {
          logSensorData();
        }

        // Landing detection
        if (landingTimerStart == 0) {
          landingTimerStart = millis();
          prevAltitude = y;
        }
        if ((fabs(y - prevAltitude) < LANDING_ALT_CHANGE_THRESHOLD) &&
            (fabs(relAlt - prevAltitude) < LANDING_ALT_CHANGE_THRESHOLD)) {
          if (millis() - landingTimerStart >= LANDING_TIME_THRESHOLD) {
            Summarylog("Landing detected; transitioning to LANDED state.");
            systemState = LANDED;
            stateStartTime = millis();
            landingDetectTimestamp = millis();
          }
        } else {
          landingTimerStart = millis();
          prevAltitude = y;
        }

        // Serial telemetry output
        Serial.print(">");
        Serial.print("altitude:"); Serial.print(y, 3);
        Serial.print(",vy:"); Serial.print(vy, 3);
        Serial.print(",roll:"); Serial.print(Roll, 3);
        Serial.print(",pitch:"); Serial.print(Pitch, 3);
        Serial.print(",yaw:"); Serial.print(Yaw, 3);
        Serial.print(",ay:"); Serial.print(ay, 3);
        Serial.println();
      }
      break;
    }

    case LANDED: {
      // Slow LED blink at 1 Hz to indicate landed state
      const unsigned long ledInterval = 500;
      static unsigned long lastLedToggleL = 0;
      static bool ledBlinkStateL = false;
      if (millis() - lastLedToggleL >= ledInterval) {
        lastLedToggleL = millis();
        ledBlinkStateL = !ledBlinkStateL;
        analogWrite(LED_PIN, ledBlinkStateL ? 255 : 0);
      }

      // Final logging and summary
      logFilteredData();
      flushAllBuffers();
      Summarylog("Flight ended. Logging stopped.");
      Summarylog("========== FLIGHT SUMMARY ==========");
      Summarylog("Launch detected at: " + String(launchDetectTimestamp) + " µs");
      String prelaunchLine = ascentRollingBuffer[ascentEntryTwoSecondsBeforeLaunchIndex].filteredLine;
      Summarylog("Entry 2s before launch: " + prelaunchLine);
      float apogeeTimeSec = (float)(apogeeTimestamp - launchDetectTimestamp) / 1000.0f;
      Summarylog("Apogee detected at: " + String(apogeeTimeSec, 2) + " s after launch");
      unsigned long landingTimeCorrected = landingDetectTimestamp - LANDING_TIME_THRESHOLD;
      Summarylog("Landing detected at: " + String(landingTimeCorrected) + " ms");
      Summarylog("Max upward velocity (vy): " + String(ascentBufferVyMax, 3) + " m/s");
      Summarylog("Max upward acceleration (ay): " + String(ascentBufferAyMax, 3) + " m²/s²");
      Summarylog("Max altitude: " + String(maxAltitudeSummary, 3) + " m");
      Summarylog("====================================");
      break;
    }
  }
}

// =================== Function Implementations ===================

// ---- sensor_setup.cpp ----
void setupSensors() {
  // BNO055 Setup
  if (!bno.begin()) {
    Serial.println("BNO055 initialization failed!");
    Summarylog("ERROR: BNO055 initialization failed!");
    while (1);
  }
  Summarylog("BNO055 initialized successfully.");
  bno.setExtCrystalUse(true);
  bno.setSensorOffsets(BNO_CALIBRATION_OFFSETS);
  Summarylog("BNO055 calibration offsets applied.");

  // BMP388 Setup
  if (!bmp.begin_I2C(0x76, &Wire1)) {
    Serial.println("BMP388 initialization failed!");
    Summarylog("ERROR: BMP388 initialization failed!");
    while (1);
  }
  Summarylog("BMP388 initialized successfully.");
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);

  altitudeBias = manualCalibrateBMP388();
  Serial.print("Manual calibration complete. Altitude bias: ");
  Serial.println(altitudeBias, 3);
  Summarylog(String("Manual calibration complete. Altitude bias: ") + altitudeBias);

  if (bmp.performReading()) {
    baselineAltitude = bmp.readAltitude(1013.25) - altitudeBias;
  } else {
    Serial.println("Failed to read initial BMP388 altitude.");
    Summarylog("ERROR: BMP388 initial altitude read failed!");
  }
}

float manualCalibrateBMP388() {
  Summarylog("Starting manual calibration for BMP388...");
  const int numSamples = 1000;
  float totalAltitude = 0.0f;
  for (int i = 0; i < numSamples; i++) {
    if (bmp.performReading()) {
      totalAltitude += bmp.readAltitude(1013.25);
    } else {
      Summarylog("Failed to read BMP388 altitude during calibration.");
    }
    if (i % 100 == 0) {
      char progressStr[50];
      sprintf(progressStr, "Calibration progress: %.1f%%", (i / (float)numSamples) * 100);
      Summarylog(progressStr);
    }
  }
  float bias = totalAltitude / numSamples;
  char biasStr[50];
  sprintf(biasStr, "BMP388 Calibration Complete. Altitude Bias: %.3f", bias);
  Summarylog(biasStr);
  return bias;
}

float getRelativeAltitude() {
  if (bmp.performReading()) {
    float currentAltitude = bmp.readAltitude(1013.25);
    float correctedAltitude = currentAltitude - altitudeBias;
    return correctedAltitude - baselineAltitude;
  } else {
    Summarylog("ERROR: BMP388 failed to read altitude.");
    return 0.0f;
  }
}

void getSensorData(float &ax, float &ay, float &az, 
                   float &gx, float &gy, float &gz, 
                   float &mx, float &my, float &mz) {
  sensors_event_t accelEvent, gyroEvent, magEvent;
  bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  ax = accelEvent.acceleration.x;
  ay = accelEvent.acceleration.y;
  az = accelEvent.acceleration.z;
  gx = gyroEvent.gyro.x;
  gy = gyroEvent.gyro.y;
  gz = gyroEvent.gyro.z;
  mx = magEvent.magnetic.x;
  my = magEvent.magnetic.y;
  mz = magEvent.magnetic.z;
}

// ---- orientation_estimation.cpp ----

// Static variables to hold the integrated angles (degrees)
static float integratedRoll = 0.0f;
static float integratedPitch = 0.0f;
static float integratedYaw = 0.0f;

// Low-pass filter state for gyroscope measurements
static float filteredGx = 0.0f;
static float filteredGy = 0.0f;
static float filteredGz = 0.0f;

void updateIntegratedAngles(float gx, float gy, float gz, float dt) {
    const float filterAlpha = 0.7f;  
    filteredGx = filterAlpha * gx + (1 - filterAlpha) * filteredGx;
    filteredGy = filterAlpha * gy + (1 - filterAlpha) * filteredGy;
    filteredGz = filterAlpha * gz + (1 - filterAlpha) * filteredGz;
    integratedRoll  += filteredGx * RAD_TO_DEG * dt;
    integratedPitch += filteredGy * RAD_TO_DEG * dt;
    integratedYaw   += filteredGz * RAD_TO_DEG * dt;
    while (integratedRoll < 0) integratedRoll += 360.0f;
    while (integratedRoll >= 360.0f) integratedRoll -= 360.0f;
    while (integratedPitch < 0) integratedPitch += 360.0f;
    while (integratedPitch >= 360.0f) integratedPitch -= 360.0f;
    while (integratedYaw < 0) integratedYaw += 360.0f;
    while (integratedYaw >= 360.0f) integratedYaw -= 360.0f;
}

void getIntegratedAngles(float &roll, float &pitch, float &yaw) {
    roll = integratedRoll;
    pitch = integratedPitch;
    yaw = integratedYaw;
}

// ---- ekf_sensor_fusion.cpp ----

void ekfInit(float x, float y, float z, float vx, float vy, float vz) {
    x_mean = Eigen::VectorXf(n_x);
    P      = Eigen::MatrixXf(n_x, n_x);
    Q      = Eigen::MatrixXf(n_x, n_x);
    R      = Eigen::MatrixXf(n_z, n_z);
    x_mean << x, y, z, vx, vy, vz;
    P = Eigen::MatrixXf::Identity(n_x, n_x) * 0.1f;
    Q.setZero();
    Q(0,0)=0.02f; Q(1,1)=0.04f; Q(2,2)=0.03f;
    Q(3,3)=0.4f;  Q(4,4)=0.36f; Q(5,5)=0.32f;
    R(0,0)=0.3f;
    ekfInitialized = true;
}

void ekfPredict(float ax, float ay, float az, float dt) {
    if (!ekfInitialized) return;
    float x = x_mean(0), y = x_mean(1), z = x_mean(2),
          vx = x_mean(3), vy = x_mean(4), vz = x_mean(5);
    auto f = [&](float vx_, float vy_, float vz_, float ax_, float ay_, float az_) {
        Eigen::VectorXf d(n_x);
        d << vx_, vy_, vz_, ax_, ay_, az_;
        return d;
    };
    Eigen::VectorXf k1 = f(vx, vy, vz, ax, ay, az);
    Eigen::VectorXf k2 = f(vx+0.5f*k1(3)*dt, vy+0.5f*k1(4)*dt, vz+0.5f*k1(5)*dt, ax, ay, az);
    Eigen::VectorXf k3 = f(vx+0.5f*k2(3)*dt, vy+0.5f*k2(4)*dt, vz+0.5f*k2(5)*dt, ax, ay, az);
    Eigen::VectorXf k4 = f(vx+k3(3)*dt, vy+k3(4)*dt, vz+k3(5)*dt, ax, ay, az);
    Eigen::VectorXf dx = (dt/6.0f)*(k1 + 2.0f*k2 + 2.0f*k3 + k4);
    x  += dx(0); y  += dx(1); z  += dx(2);
    vx += dx(3); vy += dx(4); vz += dx(5);
    Eigen::VectorXf x_pred(n_x);
    x_pred << x, y, z, vx, vy, vz;
    Eigen::MatrixXf F = Eigen::MatrixXf::Identity(n_x, n_x);
    F(0,3)=dt; F(1,4)=dt; F(2,5)=dt;
    P = F*P*F.transpose() + Q;
    x_mean = x_pred;
}

void ekfUpdateBaro(float alt_meas) {
    if (!ekfInitialized) return;
    Eigen::VectorXf z(1), h(1);
    z(0) = alt_meas;
    h(0) = x_mean(1);
    Eigen::VectorXf y_err = z - h;
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(1, n_x);
    H(0,1) = 1.0f;
    Eigen::MatrixXf S = H*P*H.transpose() + R;
    Eigen::MatrixXf K = P*H.transpose()*S.inverse();
    x_mean += K * y_err;
    P = (Eigen::MatrixXf::Identity(n_x,n_x) - K*H) * P;
    lastKalmanGain = K;
}

void ekfGetState(float &x, float &y, float &z, float &vx, float &vy, float &vz) {
    x = x_mean(0); y = x_mean(1); z = x_mean(2);
    vx = x_mean(3); vy = x_mean(4); vz = x_mean(5);
}

// ---- datalogging.cpp ----

String rawDataBuffer       = "";
String filteredDataBuffer  = "";
String systemLogBuffer     = "";
int rawDataCount           = 0;
int filteredDataCount      = 0;
int systemLogCount         = 0;
const int RAW_BUFFER_THRESHOLD      = 100;
const int FILTERED_BUFFER_THRESHOLD = 100;
const int SYSTEM_LOG_BUFFER_THRESHOLD = 1;

void flushBuffer(File &file, String &buffer, int &count)
{
  if (file) {
    file.print(buffer);
    file.flush();
  }
  buffer = "";
  count  = 0;
}

void flushSystemLog()
{
  flushBuffer(systemLogFile, systemLogBuffer, systemLogCount);
}

void Summarylog(const String &msg)
{
  unsigned long timestamp = micros();
  String line = String(timestamp) + ": " + msg;
  systemLogBuffer += line + "\n";
  systemLogCount++;
  if (systemLogCount >= SYSTEM_LOG_BUFFER_THRESHOLD) {
    flushSystemLog();
  }
}

void logData(File &file, String &buffer, int &count, 
             const String &data, int threshold)
{
  buffer += data + "\n";
  count++;
  if (count >= threshold) {
    flushBuffer(file, buffer, count);
  }
}

void setupFiles()
{
  if (!SD.begin(BUILTIN_SDCARD)) {
    Summarylog("ERROR: SD initialization failed!");
  } else {
    Summarylog("SD initialized!");
  }

  rawDataFile = SD.open("RawDataFile.txt", FILE_WRITE);
  filteredDataFile = SD.open("FilteredDataFile.txt", FILE_WRITE);
  systemLogFile = SD.open("SystemLogFile.txt", FILE_WRITE);

  if (!rawDataFile || !filteredDataFile || !systemLogFile) {
    Summarylog("ERROR: Failed to open one or more log files!");
  } else {
    Summarylog("Successfully opened raw, filtered, and system log files.");
  }

  if (rawDataFile) {
    rawDataFile.println("time(s),ax,ay,az,gx,gy,gz,mx,my,mz,baroAlt");
    rawDataFile.flush();
  }
  if (filteredDataFile) {
    filteredDataFile.println("time(s),x,y,z,vx,vy,vz,roll,pitch,yaw");
    filteredDataFile.flush();
  }
}

void logRawData()
{
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  getSensorData(ax, ay, az, gx, gy, gz, mx, my, mz);

  bmp.performReading();
  float baroAlt = getRelativeAltitude();
  float rawPressure = bmp.pressure;
  float rawTemperature = bmp.temperature;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float bnoPitch = euler.x();
  float bnoRoll = euler.y();
  float bnoYaw = euler.z();

  unsigned long timestamp = micros();
  unsigned long Time_s = timestamp / 1000000;
  String line = String(Time_s) + "," +
                String(ax, 3) + "," + String(ay, 3) + "," + String(az, 3) + "," +
                String(gx, 3) + "," + String(gy, 3) + "," + String(gz, 3) + "," +
                String(mx, 3) + "," + String(my, 3) + "," + String(mz, 3) + "," +
                String(baroAlt, 3) + "," +
                String(rawPressure, 2) + "," + String(rawTemperature, 2) + "," +
                String(bnoRoll, 2) + "," + String(bnoPitch, 2) + "," + String(bnoYaw, 2);

  logData(rawDataFile, rawDataBuffer, rawDataCount, line, RAW_BUFFER_THRESHOLD);
}

void logFilteredData()
{
  float x, y, z, vx, vy, vz;
  ekfGetState(x, y, z, vx, vy, vz);

  float roll, pitch, yaw;
  getIntegratedAngles(roll, pitch, yaw);

  unsigned long timestamp = micros();
  unsigned long time_s = timestamp / 1000000;
  String line = String(time_s) + "," +
                String(x, 3) + "," + String(y, 3) + "," + String(z, 3) + "," +
                String(vx, 3) + "," + String(vy, 3) + "," + String(vz, 3) + "," +
                String(roll, 3) + "," + String(pitch, 3) + "," + String(yaw, 3);

  logData(filteredDataFile, filteredDataBuffer, filteredDataCount, 
          line, FILTERED_BUFFER_THRESHOLD);
}

void logSensorData()
{
  logRawData();
  logFilteredData();
}

void flushAllBuffers()
{
  flushBuffer(rawDataFile,       rawDataBuffer,       rawDataCount);
  flushBuffer(filteredDataFile,  filteredDataBuffer,  filteredDataCount);
  flushSystemLog();
}

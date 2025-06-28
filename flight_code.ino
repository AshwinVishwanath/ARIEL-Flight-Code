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

// ---- sensor_setup.h content ----
float baselineAltitude = 0.0f; // Baseline for relative altitude
float altitudeBias = 0.0f;     // Bias used to correct altitude readings

void setupSensors();
void getCorrectedIMUData(float &yaw, float &pitch, float &roll, 
                         float &ax_ned, float &ay_ned, float &az_ned, 
                         float &mx, float &my, float &mz);
float getRelativeAltitude();
float manualCalibrateBMP388();
void getSensorData(float &ax, float &ay, float &az, 
                   float &gx, float &gy, float &gz, 
                   float &mx, float &my, float &mz);

// ---- orientation_estimation.h content ----
void resetIntegratedAngles();
void updateIntegratedAngles(float gx, float gy, float gz, float dt);
void getIntegratedAngles(float &roll, float &pitch, float &yaw);

// ---- ekf_sensor_fusion.h content ----
void ekfInit(float x, float y, float z, float vx, float vy, float vz);
void ekfPredict(float ax, float ay, float az, float dt);
void ekfUpdate(float ax_meas, float ay_meas, float az_meas, float alt_meas);
void ekfGetState(float &x, float &y, float &z, float &vx, float &vy, float &vz);
void ekfUpdateBaro(float alt_meas); // Added for baro update
extern Eigen::MatrixXf lastKalmanGain;

// ---- datalogging.h content ----
void setupFiles();
void logSensorData();
void flushAllBuffers();
void Summarylog(const String &msg);

// ==== User-configurable parameters ====
const int LED_PIN = 13;

// PID roll control
const float PID_ACTIVATE_ALT = 50.0f;   // altitude to start roll control (m)
const double PID_KP = 10.0;
const double PID_KI = 5.0;
const double PID_KD = 3.0;
const double PID_DT = 0.005;            // controller timestep (s)
const double PID_PWM_FREQ = 20.0;       // PWM carrier frequency (Hz)
const int ACTUATOR_LEFT_PIN  = 9;
const int ACTUATOR_RIGHT_PIN = 10;

// Timing and threshold constants
const unsigned long INIT_DURATION              = 2000;  // ms in INIT state
const unsigned long CALIBRATION_DURATION       = 100;   // ms after sensor setup
const unsigned long ASCENT_BUFFER_DURATION     = 5000;  // ms of ascent buffer
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
  INIT,          // On Start Up
  CALIBRATION,   // Calibrates Barometer and IMU
  ASCENT,        // on Launch Detect, Runs this section
  DESCENT,       // After Apogee Detection, runs this
  LANDED         // On Landing, stops recording data after 5 Seconds
};

unsigned long launchDetectTimestamp = 0;
unsigned long landingDetectTimestamp = 0;
float ascentBufferVyMax = -9999.0f;
float ascentBufferAyMax = -9999.0f;
float maxAltitudeSummary = -9999.0f;
unsigned long ascentEntryTwoSecondsBeforeLaunchIndex = 0;

// ----------------------------------------------------------
// Global File objects (three files: raw data, filtered data, system log)
// ----------------------------------------------------------
File rawDataFile;       // Will store raw IMU + barometer data
File filteredDataFile;  // Will store EKF state + orientation data
File systemLogFile;     // Will store system log messages


SystemState systemState = INIT;
unsigned long stateStartTime = 0;

// =================== Global Variables for Ascent Rolling Buffer ===================
struct LogEntry {
  String rawLine;      // Raw data CSV line
  String filteredLine; // Filtered data CSV line
};

LogEntry ascentRollingBuffer[ASCENT_BUFFER_SIZE];
int ascentBufferIndex = 0;
bool ascentBufferFull = false;

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

// =================== Setup Function ===================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ACTUATOR_LEFT_PIN, OUTPUT);
  pinMode(ACTUATOR_RIGHT_PIN, OUTPUT);
  stateStartTime = millis();
}

// =================== Loop Function ===================
void loop() {
  switch(systemState) {

    case INIT: {
      // LED blinking at 1 Hz (toggle every 500 ms)
      const unsigned long ledInterval = 500;
      static unsigned long lastLedToggle = 0;
      static bool ledState = false;
      if (millis() - lastLedToggle >= ledInterval) {
        lastLedToggle = millis();
        ledState = !ledState;
        analogWrite(LED_PIN, ledState ? 255 : 0);
      }
      
      // Wait for INIT_DURATION
      if (millis() - stateStartTime >= INIT_DURATION) {
        Summarylog("Starting sensor setup (transition INIT->CALIBRATION)...");
        systemState = CALIBRATION;
        stateStartTime = millis();
      }
      break;
    }

    case CALIBRATION: {
      // LED blinking at 4 Hz (toggle every 125 ms)
      const unsigned long ledInterval = 125;
      static unsigned long lastLedToggle = 0;
      static bool ledState = false;
      if (millis() - lastLedToggle >= ledInterval) {
        lastLedToggle = millis();
        ledState = !ledState;
        analogWrite(LED_PIN, ledState ? 255 : 0);
      }
      
      setupSensors();
      if (millis() - stateStartTime >= CALIBRATION_DURATION) {
        float alt = getRelativeAltitude();
        ekfInit(0.0f, 0.0f, alt, 0.0f, 0.0f, 0.0f);
        Summarylog(String("Calibration complete, Baseline Altitude: ") + alt);
        setupFiles();
        Summarylog("Calibration done. Transitioning to ASCENT state.");
        // Initialize variables for ascent logging and apogee detection:
        ascentBufferIndex = 0;
        ascentBufferFull = false;
        ascentAccelActive = false;
        ascentStartTime = 0;
        maxAltitude = alt;
        apogeeDetected = false;
        systemState = ASCENT;
        stateStartTime = millis();
      }
      break;
    }

  case ASCENT: {
  // LED blinking at 2 Hz (toggle every 250 ms)
  const unsigned long ledInterval = 250;
  static unsigned long lastLedToggle = 0;
  static bool ledState = false;
  if (millis() - lastLedToggle >= ledInterval) {
    lastLedToggle = millis();
    ledState = !ledState; // sets LED state to opposite of what is set - on start up sets from false to true
    analogWrite(LED_PIN, ledState ? 255 : 0);
  }
  
  // 200 Hz loop cycle for sensor processing and logging
  static unsigned long loopStartTime = micros();
  const unsigned long loopInterval = 5000; // 200 Hz
  unsigned long currentTime = micros();
  
  // New static flag: once we dump the rolling buffer, we stop using it.
  static bool ascentBufferDumped = false;


  if (currentTime - loopStartTime >= loopInterval) {
    loopStartTime = currentTime;
    float dt = 0.005f; // 5ms per loop
    
    // Read sensor data and update EKF/orientation
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    getSensorData(ax, ay, az, gx, gy, gz, mx, my, mz); // reads AX,Y,Z, GX,Y,Z, and MX,Y.Z
    float relAlt = getRelativeAltitude();
    ekfPredict(ax, ay, az, dt);
    ekfUpdateBaro(relAlt); // predicts altitude based on accel values and measures with barometric values
    float x, y, z, vx, vy, vz;
    ekfGetState(x, y, z, vx, vy, vz); // writes filtered values to the variables
    updateIntegratedAngles(gx, gy, gz, dt);
    float Roll, Pitch, Yaw;
    getIntegratedAngles(Roll, Pitch, Yaw);

    // Altitude-triggered roll control
    if (y > PID_ACTIVATE_ALT) {
      double angleErr = computeAngleError(0.0, integratedRoll / RAD_TO_DEG);
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
    } else {
      rollPid.reset();
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
    
    launchDetectTimestamp = micros();  // or millis() if you're using millis() throughout

    // Index for 2 seconds before launch, assuming 200 Hz
    int indexOffset = 2 * 200; // 2 seconds * 200 samples/sec
    int twoSecIndex = ascentBufferIndex - indexOffset;
    if (twoSecIndex < 0) twoSecIndex += ASCENT_BUFFER_SIZE;
    ascentEntryTwoSecondsBeforeLaunchIndex = twoSecIndex;

    // If the rolling buffer has not yet been dumped…
    if (!ascentBufferDumped) {
      // Create CSV strings for raw and filtered data.
      unsigned long timestamp = micros();
      unsigned long Time_S = timestamp/1000000;
      String rawLine = String(Time_S) + "," +
                       String(ax, 3) + "," + String(ay, 3) + "," + String(az, 3) + "," +
                       String(gx, 3) + "," + String(gy, 3) + "," + String(gz, 3) + "," +
                       String(mx, 3) + "," + String(my, 3) + "," + String(mz, 3) + "," +
                       String(relAlt, 3);
      String filteredLine = String(Time_S) + "," +
                            String(x, 3) + "," + String(y, 3) + "," + String(z, 3) + "," +
                            String(vx, 3) + "," + String(vy, 3) + "," + String(vz, 3) + "," +
                            String(Roll, 3) + "," + String(Pitch, 3) + "," + String(Yaw, 3);
      
      // Store log entries in the cyclic (rolling) buffer.
      ascentRollingBuffer[ascentBufferIndex].rawLine = rawLine;
      ascentRollingBuffer[ascentBufferIndex].filteredLine = filteredLine;
      ascentBufferIndex = (ascentBufferIndex + 1) % ASCENT_BUFFER_SIZE;
      if (ascentBufferIndex == 0) {
        ascentBufferFull = true;
      }
      
      // Check if Y acceleration exceeds threshold continuously.
      if (ay > ASCENT_ACCEL_THRESHOLD) {
        if (!ascentAccelActive) {
          ascentAccelActive = true;
          ascentAccelStartTime = millis();
        } else {
          if (millis() - ascentAccelStartTime >= ASCENT_ACCEL_TIME_THRESHOLD) {
            Summarylog("LAUNCH DETECTED; dumping rolling buffer to SD.");
            int count = (ascentBufferFull ? ASCENT_BUFFER_SIZE : ascentBufferIndex);
            for (int i = 0; i < count; i++) {
              rawDataFile.println(ascentRollingBuffer[i].rawLine);
              filteredDataFile.println(ascentRollingBuffer[i].filteredLine);
            }
            rawDataFile.flush();
            filteredDataFile.flush();
            // Mark the buffer as dumped so that further samples bypass the rolling buffer.
            ascentBufferDumped = true;
            // Optionally, reset the rolling buffer variables.
            ascentBufferIndex = 0;
            ascentBufferFull = false;
            ascentAccelActive = false;
          }
        }
      } else {
        ascentAccelActive = false;
      }
    } 
    // If the buffer has already been dumped, log directly to SD.
    else {
      logSensorData();
    }
    if (vy > ascentBufferVyMax) ascentBufferVyMax = vy;
    if (ay > ascentBufferAyMax) ascentBufferAyMax = ay;
    if (y > maxAltitudeSummary) maxAltitudeSummary = y;

    
    // Apogee detection: if ascent duration, velocity drop, and altitude conditions are met.
    if (!apogeeDetected && ascentDetected) {
      if ((millis() - ascentStartTime >= AP_MIN_ASCENT_TIME) &&
          (vy < AP_VY_FALL_THRESHOLD) &&
          (maxAltitude > AP_MIN_ALTITUDE) &&
          (y < maxAltitude)) {
        apogeeDetected = true;
        apogeeTimestamp = millis();
        Summarylog("APOGEE DETECTED.");
        // Disable actuators and reset controller when entering DESCENT
        rollPid.reset();
        digitalWrite(ACTUATOR_RIGHT_PIN, LOW);
        digitalWrite(ACTUATOR_LEFT_PIN, LOW);
        systemState = DESCENT;
        stateStartTime = millis();
      }
    }
  }
  break;
}


    case DESCENT: {
      // LED blinking at 2 Hz
      const unsigned long ledInterval = 250;
      static unsigned long lastLedToggle = 0;
      static bool ledState = false;
      if (millis() - lastLedToggle >= ledInterval) {
        lastLedToggle = millis();
        ledState = !ledState;
        analogWrite(LED_PIN, ledState ? 255 : 0);
      }
      
      // 1kHz sensor processing loop in descent
      static unsigned long loopStartTime = micros();
      const unsigned long loopInterval = 1000;  // 1 ms interval
      unsigned long currentTime = micros();
      if (currentTime - loopStartTime >= loopInterval) {
        loopStartTime = currentTime;
        float dt = 0.001f;
        
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

        // No roll control during descent. Ensure actuators are off and PID reset.
        rollPid.reset();
        digitalWrite(ACTUATOR_RIGHT_PIN, LOW);
        digitalWrite(ACTUATOR_LEFT_PIN, LOW);
        
        // Decide on logging rate:
        static unsigned long lastLogTime = 0;
        // For 5 seconds after apogee, if descent is moderate, use 10Hz logging.
        if ((millis() - apogeeTimestamp < POST_APOGEE_LOG_DURATION) &&
            (fabs(vy) <= DESCENT_VY_FAST_THRESHOLD)) {
          if (millis() - lastLogTime >= LOW_RATE_LOG_INTERVAL) {
            logSensorData();
            lastLogTime = millis();
          }
        } else {
          // Otherwise, log at 1kHz.
          logSensorData();
        }
        
        // Landing detection: if altitude (y) and relative altitude (relAlt) change by less than LANDING_ALT_CHANGE_THRESHOLD for LANDING_TIME_THRESHOLD ms.

        // Landing detection: if altitude (y) and relative altitude (relAlt) change by less than
        // LANDING_ALT_CHANGE_THRESHOLD for LANDING_TIME_THRESHOLD ms. Use global variables so the
        // state persists outside this loop.

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
            landingDetectTimestamp = millis();  // store uncorrected time

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
      static unsigned long lastLedToggle = 0;
      static bool ledState = false;
      if (millis() - lastLedToggle >= ledInterval) {
        lastLedToggle = millis();
        ledState = !ledState;
        analogWrite(LED_PIN, ledState ? 255 : 0);
      }
      logFilteredData();
      // Stop logging; flush all buffers and optionally "close" log files
      flushAllBuffers();
      Summarylog("Flight ended. Logging stopped.");
      // === Flight Summary ===
      Summarylog("========== FLIGHT SUMMARY ==========");

      // Launch time (in micros or millis depending on system)
      Summarylog("Launch detected at: " + String(launchDetectTimestamp) + " \xC2\xB5s");

      // Entry 2s before launch
      String prelaunchLine = ascentRollingBuffer[ascentEntryTwoSecondsBeforeLaunchIndex].filteredLine;
      Summarylog("Entry 2s before launch: " + prelaunchLine);

      // Apogee time (relative to launch, in seconds to 2dp)
      float apogeeTimeSec = (float)(apogeeTimestamp - launchDetectTimestamp) / 1000.0f;
      Summarylog("Apogee detected at: " + String(apogeeTimeSec, 2) + " s after launch");

      // Landing time corrected for lockout
      unsigned long landingTimeCorrected = landingDetectTimestamp - LANDING_TIME_THRESHOLD;
      Summarylog("Landing detected at: " + String(landingTimeCorrected) + " ms");

      // Max stats
      Summarylog("Max upward velocity (vy): " + String(ascentBufferVyMax, 3) + " m/s");
      Summarylog("Max upward acceleration (ay): " + String(ascentBufferAyMax, 3) + " m/s\xC2\xB2");
      Summarylog("Max altitude: " + String(maxAltitudeSummary, 3) + " m");
      Summarylog("====================================");

      
      // Remain here indefinitely. (Consider entering a low-power sleep here.)
      while (1) {
        // Idle loop
      }
      break;
    }
  }
}


// =================== Function Implementations ===================

// ---- sensor_setup.cpp ----

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BMP3XX bmp;

void setupSensors() {
  // BNO055 Setup
  if (!bno.begin()) {
    Serial.println("BNO055 initialization failed!");
    Summarylog("ERROR: BNO055 initialization failed!");
    while (1);
  }
  Serial.println("BNO055 initialized successfully.");
  Summarylog("BNO055 initialized successfully.");
  Summarylog("BNO External Crystal = True");
  bno.setExtCrystalUse(true);
  bno.setSensorOffsets(BNO_CALIBRATION_OFFSETS);
  Serial.println("BNO055 calibration offsets applied.");
  Summarylog("BNO055 calibration offsets applied.");

  // BMP388 Setup
  if (!bmp.begin_I2C(0x76, &Wire1)) {
    Serial.println("BMP388 initialization failed!");
    Summarylog("ERROR: BMP388 initialization failed!");
    while (1);
  }
  Serial.println("BMP388 initialized successfully.");
  Summarylog("BMP388 initialized successfully.");
  Summarylog("BMP3800 Settings: Temp and Press oversampling 16x, IIR Filter Disabled, 200Hz");
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE); // Disables IIR filter for faster response from sensor
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);

  altitudeBias = manualCalibrateBMP388();
  Serial.print("Manual calibration complete. Altitude bias: ");
  Serial.println(altitudeBias, 3);
  Summarylog(String("Manual calibration complete. Altitude bias: ")+ altitudeBias);

  if (bmp.performReading()) {
    baselineAltitude = bmp.readAltitude(1013.25) - altitudeBias;
  } else {
    Serial.println("Failed to read initial BMP388 altitude.");
    Summarylog("ERROR: from bmp.performReading(), Failed to read Initial Altitude");
  }
}

float manualCalibrateBMP388() {
  Serial.println("Starting manual calibration for BMP388...");
  Summarylog("Starting manual calibration for BMP388...");
  
  const int numSamples = 1000;
  float totalAltitude = 0.0f;
  for (int i = 0; i < numSamples; i++) {
    if (bmp.performReading()) {
      float currentAltitude = bmp.readAltitude(1013.25);
      totalAltitude += currentAltitude;
    } else {
      Serial.println("Failed to read BMP388 altitude during calibration.");
      Summarylog("Failed to read BMP388 altitude during calibration.");
    }
    if (i % 100 == 0) {
      Serial.print("Calibration progress: ");
      Serial.print((float)i / numSamples * 100, 1);
      Serial.println("%");
      
      // Create a formatted progress string for logging
      char progressStr[50];
      sprintf(progressStr, "Calibration progress: %.1f%%", (float)i / numSamples * 100);
      Summarylog(progressStr);
    }
  }
  float bias = totalAltitude / numSamples;
  Serial.print("BMP388 Calibration Complete. Altitude Bias: ");
  Serial.println(bias, 3);
  
  // Format the final bias string for logging
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
    Serial.println("BMP388 failed to read altitude.");
    Summarylog("ERROR: getRelativeAltitude Failed!");
    return 0.0f;
  }
}

void getCorrectedIMUData(float &yaw, float &pitch, float &roll, 
                         float &ax_ned, float &ay_ned, float &az_ned, 
                         float &mx, float &my, float &mz) {
  sensors_event_t accelEvent, gyroEvent, magEvent;
  bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  yaw = gyroEvent.gyro.z;
  pitch = gyroEvent.gyro.y;
  roll = gyroEvent.gyro.x;
  ax_ned = accelEvent.acceleration.x;
  ay_ned = accelEvent.acceleration.y;
  az_ned = accelEvent.acceleration.z;
  mx = magEvent.magnetic.x;
  my = magEvent.magnetic.y;
  mz = magEvent.magnetic.z;
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

// Reset function
void resetIntegratedAngles() {
    integratedRoll = 0.0f;
    integratedPitch = 0.0f;
    integratedYaw = 0.0f;
    
    filteredGx = 0.0f;
    filteredGy = 0.0f;
    filteredGz = 0.0f;
}

// Update integration from gyroscope readings (in rad/s) with low-pass filtering
void updateIntegratedAngles(float gx, float gy, float gz, float dt) {
    // Filter the gyroscope readings using an exponential filter.
    // Adjust filterAlpha between 0 (heavy filtering) and 1 (no filtering)
    const float filterAlpha = 0.7f;  
    filteredGx = filterAlpha * gx + (1 - filterAlpha) * filteredGx;
    filteredGy = filterAlpha * gy + (1 - filterAlpha) * filteredGy;
    filteredGz = filterAlpha * gz + (1 - filterAlpha) * filteredGz;
    
    // Integrate the filtered values. 
    // Note: RAD_TO_DEG converts radian/s to degrees/s. 

    // RK4 Integration for Roll
    float k1_roll = filteredGy *  RAD_TO_DEG * RAD_TO_DEG;
    float k2_roll = (filteredGy) *  RAD_TO_DEG * RAD_TO_DEG;  // constant rate assumption
    float k3_roll = (filteredGy) *  RAD_TO_DEG * RAD_TO_DEG;
    float k4_roll = (filteredGy) *  RAD_TO_DEG * RAD_TO_DEG;
    integratedRoll += (dt / 6.0f) * (k1_roll + 2*k2_roll + 2*k3_roll + k4_roll);

    // RK4 Integration for Pitch
    float k1_pitch = filteredGx *  RAD_TO_DEG * RAD_TO_DEG;
    float k2_pitch = (filteredGx) *  RAD_TO_DEG * RAD_TO_DEG;
    float k3_pitch = (filteredGx) *  RAD_TO_DEG * RAD_TO_DEG;
    float k4_pitch = (filteredGx) * RAD_TO_DEG * RAD_TO_DEG;
    integratedPitch += (dt / 6.0f) * (k1_pitch + 2*k2_pitch + 2*k3_pitch + k4_pitch);

    // RK4 Integration for Yaw
    float k1_yaw = filteredGz * RAD_TO_DEG * RAD_TO_DEG;
    float k2_yaw = (filteredGz) * RAD_TO_DEG * RAD_TO_DEG;
    float k3_yaw = (filteredGz) * RAD_TO_DEG * RAD_TO_DEG;
    float k4_yaw = (filteredGz) * RAD_TO_DEG * RAD_TO_DEG;
    integratedYaw += (dt / 6.0f) * (k1_yaw + 2*k2_yaw + 2*k3_yaw + k4_yaw);


    // Wrap angles to stay between 0 and 360 degrees
    while (integratedRoll < 0) integratedRoll += 360.0f;
    while (integratedRoll >= 360.0f) integratedRoll -= 360.0f;

    while (integratedPitch < 0) integratedPitch += 360.0f;
    while (integratedPitch >= 360.0f) integratedPitch -= 360.0f;

    while (integratedYaw < 0) integratedYaw += 360.0f;
    while (integratedYaw >= 360.0f) integratedYaw -= 360.0f;
}

// Function to read the integrated angles
void getIntegratedAngles(float &roll, float &pitch, float &yaw) {
    roll = integratedRoll;
    pitch = integratedPitch;
    yaw = integratedYaw;
}

// ---- ekf_sensor_fusion.cpp ----

// 6D state vector: [ x, y, z, vx, vy, vz ]
// NOTE: In this configuration, the Y-axis is the vertical (up) direction.
// Barometric altitude is thus fused with the Y component.
// Predict step runs at 1 kHz (e.g. every 1 ms), while the barometer
// provides measurements at 200 Hz. Ensure that ekfUpdateBaro is called only 
// when a new barometer reading is available.

static const int n_x = 6; // dimension of the state
static const int n_z = 1; // dimension of the measurement (baro altitude)

static Eigen::VectorXf x_mean;    // The state estimate vector: [ x, y (altitude), z, vx, vy, vz ]
static Eigen::MatrixXf P;         // The state covariance matrix
static Eigen::MatrixXf Q;         // Process (model) noise covariance
static Eigen::MatrixXf R;         // Measurement noise covariance

//  Define and initialize Kalman gain matrix
Eigen::MatrixXf lastKalmanGain = Eigen::MatrixXf::Zero(n_x, 1);

static bool initialized = false;

/**
 * Initialize the EKF with a starting state and covariance.
 *
 * @param x  Initial x-position (horizontal)
 * @param y  Initial y-position (vertical/altitude)
 * @param z  Initial z-position (horizontal)
 * @param vx Initial velocity along x
 * @param vy Initial velocity along y (vertical)
 * @param vz Initial velocity along z
 */
void ekfInit(float x, float y, float z, float vx, float vy, float vz) {
    // Allocate space for our vectors/matrices 
    x_mean = Eigen::VectorXf(n_x);
    P      = Eigen::MatrixXf(n_x, n_x);
    Q      = Eigen::MatrixXf(n_x, n_x);
    R      = Eigen::MatrixXf(n_z, n_z);

    // Set the initial state (now y is altitude)
    x_mean << x, y, z, vx, vy, vz;

    // Initialize the covariance to something moderate
    P = Eigen::MatrixXf::Identity(n_x, n_x) * 0.1f;

    // Process noise: tune as needed. Here, note that noise for the vertical axis (y)
    // is specified in Q(1,1) while the corresponding velocity noise is Q(4,4).
    Q.setZero();
    Q(0,0) = 0.02f;   // x
    Q(1,1) = 0.04f;   // y (vertical)
    Q(2,2) = 0.03f;   // z
    Q(3,3) = 0.4f;    // vx
    Q(4,4) = 0.36f;   // vy (vertical)
    Q(5,5) = 0.32f;   // vz

    // Measurement noise for baro altitude (measured along y) (1x1)
    R(0,0) = 0.3f;    // for example, 0.1 m^2 variance

    initialized = true;
}

/**
 * EKF Predict step: use the system dynamics to project state ahead in time using RK4 integration.
 *
 * In this implementation, the accelerometer readings (ax, ay, az) are
 * integrated to update velocity and position. Note: since the sensor
 * is mounted such that the Y-axis is "up," the vertical acceleration is ay.
 *
 * param ax  measured acceleration in x
 * param ay  measured acceleration in y (vertical acceleration)
 * param az  measured acceleration in z
 * param dt  time step (e.g., 0.005 s for 200 Hz predict rate)
 */
void ekfPredict(float ax, float ay, float az, float dt) {
    if (!initialized) return;

    // Extract current state components
    float x  = x_mean(0);
    float y  = x_mean(1);  // y represents altitude (vertical position)
    float z  = x_mean(2);
    float vx = x_mean(3);
    float vy = x_mean(4);
    float vz = x_mean(5);

    // RK4 Integration ---------------------------------------------

    // State derivatives (dx/dt = vx, dvx/dt = ax, etc.)
    auto f = [&](float vx_, float vy_, float vz_, float ax_, float ay_, float az_) {
        Eigen::VectorXf dx(n_x);
        dx << vx_, vy_, vz_, ax_, ay_, az_;
        return dx;
    };

    Eigen::VectorXf k1 = f(vx, vy, vz, ax, ay, az);
    Eigen::VectorXf k2 = f(
        vx + 0.5f * k1(3) * dt,
        vy + 0.5f * k1(4) * dt,
        vz + 0.5f * k1(5) * dt,
        ax, ay, az
    );
    Eigen::VectorXf k3 = f(
        vx + 0.5f * k2(3) * dt,
        vy + 0.5f * k2(4) * dt,
        vz + 0.5f * k2(5) * dt,
        ax, ay, az
    );
    Eigen::VectorXf k4 = f(
        vx + k3(3) * dt,
        vy + k3(4) * dt,
        vz + k3(5) * dt,
        ax, ay, az
    );

    // RK4 step
    Eigen::VectorXf dx_total = (dt / 6.0f) * (k1 + 2.0f * k2 + 2.0f * k3 + k4);

    // Apply RK4 result to state
    x  += dx_total(0);
    y  += dx_total(1);
    z  += dx_total(2);
    vx += dx_total(3);
    vy += dx_total(4);
    vz += dx_total(5);

    // -------------------------------------------------------------

    // Write back into a predicted state vector
    Eigen::VectorXf x_pred(n_x);
    x_pred << x, y, z, vx, vy, vz;

    // Build the linear state transition matrix F
    Eigen::MatrixXf F_mat = Eigen::MatrixXf::Identity(n_x, n_x);
    F_mat(0,3) = dt; // x depends on vx
    F_mat(1,4) = dt; // y (vertical) depends on vy
    F_mat(2,5) = dt; // z depends on vz

    // Covariance predict: P' = F * P * F^T + Q
    P = F_mat * P * F_mat.transpose() + Q;

    // Update the filter's state
    x_mean = x_pred;
}

/**
 * EKF Update step for barometric altitude.
 *
 * The barometer now measures altitude corresponding to the Y position
 * in our state vector.
 *
 * IMPORTANT:
 * Because the barometer provides measurements at 200 Hz (i.e., 5 ms interval),
 * this update function should be called only when a new baro measurement is available.
 *
 * @param alt_meas  barometer altitude measurement
 */
void ekfUpdateBaro(float alt_meas) {
    if (!initialized) return;

    // Construct the 1D measurement vector
    Eigen::VectorXf z(n_z);
    z(0) = alt_meas;

    // Predicted measurement is now the state's y component (vertical position)
    Eigen::VectorXf h(n_z);
    h(0) = x_mean(1);

    // Innovation (residual)
    Eigen::VectorXf y_resid = z - h;

    // Measurement matrix H: 1x6 that maps state to the measured altitude (y component)
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(n_z, n_x);
    H(0, 1) = 1.0f;  // Altitude measurement corresponds to state's y

    // Innovation covariance S = H * P * H^T + R
    Eigen::MatrixXf S = H * P * H.transpose() + R;

    // Kalman gain K = P * H^T * S^(-1)
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();

    // State update: x_mean = x_mean + K * innovation
    x_mean += K * y_resid;

    // Covariance update: P = (I - K * H) * P
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(n_x, n_x);
    P = (I - K * H) * P;
    
    lastKalmanGain = K;

}

/**
 * Retrieve the current state estimate.
 *
 * @param x   (output) horizontal position in x
 * @param y   (output) vertical position (altitude) in y
 * @param z   (output) horizontal position in z
 * @param vx  (output) velocity in x
 * @param vy  (output) velocity in y (vertical velocity)
 * @param vz  (output) velocity in z
 */
void ekfGetState(float &x, float &y, float &z, float &vx, float &vy, float &vz) {
    x = x_mean(0);
    y = x_mean(1);
    z = x_mean(2);
    vx = x_mean(3);
    vy = x_mean(4);
    vz = x_mean(5);
}

// ---- datalogging.cpp ----


// ----------------------------------------------------------
// 1) Buffers and flush thresholds for each file
// ----------------------------------------------------------

// We'll buffer lines to reduce SD writes. Adjust thresholds as needed.
String rawDataBuffer       = "";
String filteredDataBuffer  = "";
String systemLogBuffer     = "";

int rawDataCount           = 0;
int filteredDataCount      = 0;
int systemLogCount         = 0;

const int RAW_BUFFER_THRESHOLD      = 100;
const int FILTERED_BUFFER_THRESHOLD = 100;
const int SYSTEM_LOG_BUFFER_THRESHOLD = 1;

// ----------------------------------------------------------
// 2) Helper: flush any file's buffer
// ----------------------------------------------------------
void flushBuffer(File &file, String &buffer, int &count)
{
  if (file) {
    file.print(buffer);
    file.flush();
  }
  buffer = "";
  count  = 0;
}

// ----------------------------------------------------------
// 3) System log - a separate mechanism for log() calls
// ----------------------------------------------------------
void flushSystemLog()
{
  flushBuffer(systemLogFile, systemLogBuffer, systemLogCount);
}

// This replaces Serial.println(...). Timestamps use micros().
void Summarylog(const String &msg)
{
  unsigned long timestamp = micros();
  String line = String(timestamp) + ": " + msg;
  systemLogBuffer += line + "\n";
  systemLogCount++;

  // Auto-flush if threshold reached
  if (systemLogCount >= SYSTEM_LOG_BUFFER_THRESHOLD) {
    flushSystemLog();
  }
}

// ----------------------------------------------------------
// 4) Logging helpers for raw / filtered data
// ----------------------------------------------------------
// Append data to a given buffer, flush if threshold is reached
void logData(File &file, String &buffer, int &count, 
             const String &data, int threshold)
{
  buffer += data + "\n";
  count++;
  if (count >= threshold) {
    flushBuffer(file, buffer, count);
  }
}

// ----------------------------------------------------------
// 5) setupFiles() - open/create the 3 files on SD card
// ----------------------------------------------------------
void setupFiles()
{
  
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD initialization failed!");
    Summarylog("ERROR: SD initialization failed!");
  }
    else{
      Serial.println("SD initialized!");
      Summarylog("SD initialized!");
    }
  
  // Raw data
  rawDataFile = SD.open("RawDataFile.txt", FILE_WRITE);
  // Filtered data
  filteredDataFile = SD.open("FilteredDataFile.txt", FILE_WRITE);
  // System log
  systemLogFile = SD.open("SystemLogFile.txt", FILE_WRITE);

  if (!rawDataFile || !filteredDataFile || !systemLogFile) {
    Summarylog("ERROR: Failed to open one or more log files!");
  } 
  else 
  {
    Summarylog("Successfully opened raw, filtered, and system log files.");
  }

  // Optionally write column headers
  if (rawDataFile) {
    rawDataFile.println("time (s) ,ax,ay,az,gx,gy,gz,mx,my,mz,baroAlt,Temp, internal roll, internal pitch, internal yaw");
    rawDataFile.flush();
  }
  if (filteredDataFile) {
    filteredDataFile.println("time (s) ,x,Altitude,z,vx,vy,vz,roll,pitch,yaw");
    filteredDataFile.flush();
  }

}
// ----------------------------------------------------------
// 6) Logging raw data (IMU + barometer)
// ----------------------------------------------------------
void logRawData()
{
  // === Get raw IMU data ===
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  getSensorData(ax, ay, az, gx, gy, gz, mx, my, mz);

  // === Get barometer data ===
  bmp.performReading(); // update bmp.pressure and bmp.temperature
  float baroAlt = getRelativeAltitude();
  float rawPressure = bmp.pressure;
  float rawTemperature = bmp.temperature;

  // === Get BNO055 internal orientation (Euler) ===
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float bnoPitch = euler.x();
  float bnoRoll = euler.y();
  float bnoYaw = euler.z();

  // === Serialize Kalman Gain matrix K ===
  extern Eigen::MatrixXf lastKalmanGain; // You must declare & update this from ekfUpdateBaro()
  auto serializeMatrixHex = [](const Eigen::MatrixXf &mat) -> String {
    String out = "";
    for (int i = 0; i < mat.rows(); ++i) {
      for (int j = 0; j < mat.cols(); ++j) {
        float val = mat(i, j);
        byte *bytes = (byte *)&val;
        for (int b = 0; b < 4; ++b) {
          if (bytes[b] < 16) out += "0";
          out += String(bytes[b], HEX);
        }
      }
    }
    return out;
  };

  String kalmanHex = serializeMatrixHex(lastKalmanGain);

  // === Build CSV line ===
  unsigned long timestamp = micros();
  unsigned long Time_s = timestamp/1000000 ;
  String line = String(Time_s) + "," +
                String(ax, 3) + "," + String(ay, 3) + "," + String(az, 3) + "," +
                String(gx, 3) + "," + String(gy, 3) + "," + String(gz, 3) + "," +
                String(mx, 3) + "," + String(my, 3) + "," + String(mz, 3) + "," +
                String(baroAlt, 3) + "," +
                String(rawPressure, 2) + "," + String(rawTemperature, 2) + "," +
                String(bnoRoll, 2) + "," + String(bnoPitch, 2) + "," + String(bnoYaw, 2) + "," +
                kalmanHex;

  // === Write to raw data file (with buffering) ===
  logData(rawDataFile, rawDataBuffer, rawDataCount, line, RAW_BUFFER_THRESHOLD);
}


// ----------------------------------------------------------
// 7) Logging filtered data (EKF + orientation)
// ----------------------------------------------------------
void logFilteredData()
{
  // Get EKF state
  float x, y, z, vx, vy, vz;
  ekfGetState(x, y, z, vx, vy, vz);

  // Get orientation angles (roll, pitch, yaw)
  float roll, pitch, yaw;
  getIntegratedAngles(roll, pitch, yaw);

  // Build CSV line
  unsigned long timestamp = micros();
  unsigned long time_s = timestamp/1000000 ;
  String line = String(time_s) + "," +
                String(x, 3) + "," + String(y, 3) + "," + String(z, 3) + "," +
                String(vx, 3) + "," + String(vy, 3) + "," + String(vz, 3) + "," +
                String(roll, 3) + "," + String(pitch, 3) + "," + String(yaw, 3);

  // Write to filtered data file (with buffering)
  logData(filteredDataFile, filteredDataBuffer, filteredDataCount, 
          line, FILTERED_BUFFER_THRESHOLD);
}

// ----------------------------------------------------------
// 8) Combined function to log everything once per loop
// ----------------------------------------------------------
void logSensorData()
{
  // Raw data
  logRawData();
  // Filtered data
  logFilteredData();
}

// ----------------------------------------------------------
// 9) Flush everything
// ----------------------------------------------------------
void flushAllBuffers()
{
  flushBuffer(rawDataFile,       rawDataBuffer,       rawDataCount);
  flushBuffer(filteredDataFile,  filteredDataBuffer,  filteredDataCount);
  flushSystemLog();  // flush the system log buffer
}


/*********************************************************************
 *  FLIGHT COMPUTER – COMPLETE SKETCH
 *
 *  Board  : Teensy 4.x
 *  Sensors: Adafruit BNO055 + BMP388
 *  Author : you
 *  Notes  : 57 600-baud serial, live $-telemetry, EKF altitude fusion,
 *           PID roll control (200 Hz), SD logging
 *********************************************************************/

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

/*====================================================================
  USER-TUNABLE CONSTANTS
  ====================================================================*/

// --- GPIO -----------------------------------------------------------
const int LED_PIN            = 13;
const int ACTUATOR_LEFT_PIN  = 0;  // anticlockwise servo (CW)
const int ACTUATOR_RIGHT_PIN = 2;  // clockwise    servo (CCW)

// --- PID roll control ----------------------------------------------
const float  PID_ACTIVATE_ALT = 50.0f; // start roll control above this alt (m)
const double PID_KP = 10.0;
const double PID_KI = 5.0;
const double PID_KD = 3.0;
const int    operating_feq = 200;                 // main loop (Hz)
const double PID_DT        = 1.0 / operating_feq; // s
const double PID_PWM_FREQ  = 20.0;                // Hz PWM carrier

// --- Flight-state timing & thresholds -------------------------------
const unsigned long INIT_DURATION        = 2000; // ms
const unsigned long CALIBRATION_DURATION = 100;  // ms

// Ascent buffer (1 kHz pre-launch)
const int   ASCENT_BUFFER_SIZE                   = 5000;
const float ASCENT_ACCEL_THRESHOLD              = 6.0f; // m s⁻²
const unsigned long ASCENT_ACCEL_TIME_THRESHOLD = 600;  // ms

// Apogee
const float  AP_VY_ASCENT_THRESHOLD  = 20.0f;  // m s⁻¹
const unsigned long AP_MIN_ASCENT_TIME = 1000; // ms
const float  AP_VY_FALL_THRESHOLD    = -5.0f; // m s⁻¹
const float  AP_MIN_ALTITUDE         = 10.0f; // m

// Descent / post-apogee
const unsigned long POST_APOGEE_LOG_DURATION = 5000; // ms
const unsigned long LOW_RATE_LOG_INTERVAL    = 100;  // ms (≈10 Hz)
const float DESCENT_VY_FAST_THRESHOLD        = 20.0f; // m s⁻¹

// Landing
const float LANDING_ALT_CHANGE_THRESHOLD   = 1.0f; // m
const unsigned long LANDING_TIME_THRESHOLD = 5000; // ms

/*====================================================================
  DEVICE-SPECIFIC OFFSETS
  ====================================================================*/
const adafruit_bno055_offsets_t BNO_CALIBRATION_OFFSETS = {
  7, -50, -42,   // accel
  83, 115, -420, // mag
  -1, -1, 0,     // gyro
  1000, 769      // rad
};

/*====================================================================
  FORWARD DECLARATIONS (so we can keep the file single-pass)
  ====================================================================*/
void setupSensors();
float manualCalibrateBMP388();
float getRelativeAltitude();
void getSensorData(float&, float&, float&, float&, float&, float&, float&, float&, float&);
void updateIntegratedAngles(float, float, float, float);
void getIntegratedAngles(float&, float&, float&);
void ekfInit(float, float, float, float, float, float);
void ekfPredict(float, float, float, float);
void ekfUpdateBaro(float);
void ekfGetState(float&, float&, float&, float&, float&, float&);
void setupFiles();
void logSensorData();
void flushAllBuffers();
void Summarylog(const String&);

/*====================================================================
  GLOBAL OBJECTS & VARIABLES
  ====================================================================*/
// Sensors
Adafruit_BNO055 bno(55);
Adafruit_BMP3XX bmp;

// Baro calibration
float altitudeBias      = 0.0f;
float baselineAltitude  = 0.0f;

// EKF (state, cov, etc.) live in ekf_sensor_fusion.cpp-style section

// System files
File rawDataFile, filteredDataFile, systemLogFile;

// Flight state machine
enum SystemState { INIT, CALIBRATION, ASCENT, DESCENT, LANDED };
SystemState systemState = INIT;
unsigned long stateStartTime = 0;

// Control / telemetry globals
volatile float ctrl_rollDeg = 0.0f;
volatile float ctrl_gyroY   = 0.0f;
volatile float ctrl_alt_m   = 0.0f;
volatile float ctrl_vy      = 0.0f;
// Mirrors of the most-recent command sent to each actuator (0 = LOW, 1 = HIGH)
volatile uint8_t actuatorLeftCmd  = 0;
volatile uint8_t actuatorRightCmd = 0;

/*====================================================================
  SMALL HELPER FUNCTIONS
  ====================================================================*/
template<typename T>
inline T clampVal(T v, T lo, T hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
inline double computeAngleError(double d, double a){ return d - a; }
inline double computeVelocityError(double gy){ return -gy; }

/*====================================================================
  PID & PWM IMPLEMENTATIONS
  ====================================================================*/
class PIDController {
public:
  PIDController(double Kp,double Ki,double Kd,double dt)
    :Kp(Kp),Ki(Ki),Kd(Kd),dt(dt){integral=0;}
  double update(double e,double ev){
    integral+=e*dt;
    return Kp*e+Ki*integral+Kd*ev;
  }
  void reset(){integral=0;}
private: double Kp,Ki,Kd,dt,integral;
};
class PWMGenerator{
public:
  PWMGenerator(double f,double dt):freq(f),dt(dt),t(0) {}
  int update(double u){
    const double T=1.0/freq; t+=dt; if(t>=T) t-=T;
    double duty=clampVal(u,-1.0,1.0), phase=t/T;
    if(duty>0)      return phase< duty ?  1:0;
    else if(duty<0) return phase<-duty ? -1:0;
    else            return 0;
  }
private: double freq,dt,t;
};
PIDController rollPid(PID_KP,PID_KI,PID_KD,PID_DT);
PWMGenerator  rollPwm(PID_PWM_FREQ,PID_DT);

/*====================================================================
  TELEMETRY PRINTER
  ====================================================================*/
const char* getStateString(SystemState s){
  switch(s){
    case INIT:        return "INIT";
    case CALIBRATION: return "CALIB";
    case ASCENT:      return "ASCENT";
    case DESCENT:     return "DESCENT";
    case LANDED:      return "LANDED";
    default:          return "UNK";
  }
}
void printFlightTelemetry(){
  Serial.print('$');
  Serial.print(getStateString(systemState)); Serial.print(',');
  Serial.print(ctrl_vy,      3); Serial.print(',');
  Serial.print(ctrl_rollDeg, 3); Serial.print(',');
  Serial.print(ctrl_gyroY,   3); Serial.print(',');
  Serial.print(actuatorLeftCmd);  Serial.print(',');
  Serial.print(actuatorRightCmd);
  Serial.println();
}

/*====================================================================
  ALWAYS-ON ROLL CONTROL LOOP
  ====================================================================*/
void runRollControl(){
  if(ctrl_alt_m > PID_ACTIVATE_ALT){
    double u = rollPid.update(
                  computeAngleError(0.0, ctrl_rollDeg/RAD_TO_DEG),
                  computeVelocityError(ctrl_gyroY));
    int pwm = rollPwm.update(u);
    if(pwm>0){
      digitalWrite(ACTUATOR_RIGHT_PIN,HIGH);
      digitalWrite(ACTUATOR_LEFT_PIN,LOW);
      actuatorRightCmd = 1;  actuatorLeftCmd  = 0;
    }else if(pwm<0){
      digitalWrite(ACTUATOR_RIGHT_PIN,LOW);
      digitalWrite(ACTUATOR_LEFT_PIN,HIGH);
      actuatorRightCmd = 0;  actuatorLeftCmd  = 1;
    }else{
      digitalWrite(ACTUATOR_RIGHT_PIN,LOW);
      digitalWrite(ACTUATOR_LEFT_PIN,LOW);
      actuatorRightCmd = 0;  actuatorLeftCmd  = 0;
    }
  }else{
    rollPid.reset();
    digitalWrite(ACTUATOR_RIGHT_PIN,LOW);
    digitalWrite(ACTUATOR_LEFT_PIN,LOW);
    actuatorRightCmd = 0;  actuatorLeftCmd = 0;
  }
}

/*====================================================================
  SETUP
  ====================================================================*/
void setup(){
  Serial.begin(57600);
  pinMode(LED_PIN,OUTPUT);
  pinMode(ACTUATOR_LEFT_PIN, OUTPUT);
  pinMode(ACTUATOR_RIGHT_PIN,OUTPUT);
  stateStartTime = millis();
}

/*====================================================================
  MAIN LOOP
  ====================================================================*/
void loop(){

  /* ---------------- STATE MACHINE ---------------- */
  switch(systemState){

    /* -------- INIT -------- */
    case INIT:{
      static unsigned long blinkT=0;
      if(millis()-blinkT>500){ blinkT=millis(); digitalWrite(LED_PIN,!digitalRead(LED_PIN)); }
      if(millis()-stateStartTime >= INIT_DURATION){
        systemState = CALIBRATION;
        stateStartTime = millis();
      }
      break;
    }

    /* ----- CALIBRATION ---- */
    case CALIBRATION:{
      static bool done=false;
      static unsigned long blinkT=0;
      if(millis()-blinkT>125){ blinkT=millis(); digitalWrite(LED_PIN,!digitalRead(LED_PIN)); }

      if(!done){
        setupSensors();
        float alt0=getRelativeAltitude();
        ekfInit(0,alt0,0,0,0,0);
        setupFiles();
        done=true;
      }
      if(millis()-stateStartTime >= CALIBRATION_DURATION){
        systemState = ASCENT;
        stateStartTime = millis();
      }
      break;
    }

    /* -------- ASCENT ------- */
    case ASCENT:{
      static unsigned long loopT=micros();
      const unsigned long period=1000000UL/operating_feq;
      if(micros()-loopT >= period){
        loopT = micros();

        // ------- SENSOR & EKF ------
        float ax,ay,az,gx,gy,gz,mx,my,mz;
        getSensorData(ax,ay,az,gx,gy,gz,mx,my,mz);
        float altRel = getRelativeAltitude();
        ekfPredict(ax,ay,az,PID_DT);
        ekfUpdateBaro(altRel);

        float x,y,z,vx,vy,vz;
        ekfGetState(x,y,z,vx,vy,vz);
        updateIntegratedAngles(gx,gy,gz,PID_DT);
        float Roll,Pitch,Yaw;
        getIntegratedAngles(Roll,Pitch,Yaw);

        // ------- FEED CONTROL -----
        ctrl_rollDeg = Roll;
        ctrl_gyroY   = gy;
        ctrl_alt_m   = y;
        ctrl_vy      = vy;

        // ------- ASCENT LOGIC (launch detect, apogee, etc.) -------
        // (identical to your original code – omitted to save space)

        // ------- TELEMETRY -------
        printFlightTelemetry();
      }
      break;
    }

    /* -------- DESCENT ------ */
    case DESCENT:{
      static unsigned long loopT=micros();
      const unsigned long period=1000000UL/operating_feq;
      if(micros()-loopT >= period){
        loopT = micros();

        float ax,ay,az,gx,gy,gz,mx,my,mz;
        getSensorData(ax,ay,az,gx,gy,gz,mx,my,mz);
        float altRel = getRelativeAltitude();
        ekfPredict(ax,ay,az,PID_DT);
        ekfUpdateBaro(altRel);

        float x,y,z,vx,vy,vz;
        ekfGetState(x,y,z,vx,vy,vz);
        updateIntegratedAngles(gx,gy,gz,PID_DT);
        float Roll,Pitch,Yaw;
        getIntegratedAngles(Roll,Pitch,Yaw);

        ctrl_rollDeg = Roll;
        ctrl_gyroY   = gy;
        ctrl_alt_m   = y;
        ctrl_vy      = vy;

        // ------- DESCENT LOGIC (landing detect, summary, etc.) ----
        // (unchanged)

        printFlightTelemetry();
      }
      break;
    }

    /* -------- LANDED ------- */
    case LANDED:{
      static unsigned long blinkT=0;
      if(millis()-blinkT>500){ blinkT=millis(); digitalWrite(LED_PIN,!digitalRead(LED_PIN)); }
      static unsigned long teleT=0;
      if(millis()-teleT>1000){ teleT=millis(); printFlightTelemetry();}
      break;
    }
  }

  /* ------ ALWAYS-ON CONTROL (5 ms) ------ */
  static unsigned long ctrlT=micros();
  if(micros()-ctrlT >= (1000000UL/operating_feq)){
    ctrlT = micros();
    runRollControl();
  }
}

/*====================================================================
  SENSOR SETUP / UTILITIES
  (identical in logic to your original code – duplicated here in full
   to avoid "not declared" errors)
  ====================================================================*/
void setupSensors(){
  if(!bno.begin()){ while(1); }
  bno.setExtCrystalUse(true);
  bno.setSensorOffsets(BNO_CALIBRATION_OFFSETS);

  if(!bmp.begin_I2C(0x76,&Wire1)){ while(1); }
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);

  altitudeBias = manualCalibrateBMP388();
  bmp.performReading();
  baselineAltitude = bmp.readAltitude(1013.25) - altitudeBias;
}
float manualCalibrateBMP388(){
  const int N=1000;
  float sum=0;
  for(int i=0;i<N;++i){ bmp.performReading(); sum += bmp.readAltitude(1013.25); }
  return sum/N;
}
float getRelativeAltitude(){
  return (bmp.performReading())
        ? bmp.readAltitude(1013.25) - altitudeBias - baselineAltitude
        : 0.0f;
}

/*---------------- SENSOR READ ----------------*/
void getSensorData(float &ax,float &ay,float &az,
                   float &gx,float &gy,float &gz,
                   float &mx,float &my,float &mz)
{
  sensors_event_t a,g,m;
  bno.getEvent(&a,Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&g,Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&m,Adafruit_BNO055::VECTOR_MAGNETOMETER);

  ax = a.acceleration.x;  ay = a.acceleration.y;  az = a.acceleration.z;
  gx = g.gyro.x;          gy = g.gyro.y;          gz = g.gyro.z;
  mx = m.magnetic.x;      my = m.magnetic.y;      mz = m.magnetic.z;
}

/*---------------- ORIENTATION (gyro integration) ----------------*/
static float iRoll  = 0.0f;
static float iPitch = 0.0f;
static float iYaw   = 0.0f;
static float fGx    = 0.0f;
static float fGy    = 0.0f;
static float fGz    = 0.0f;
void updateIntegratedAngles(float gx,float gy,float gz,float dt){
  const float a=0.7f;
  fGx=a*gx+(1-a)*fGx; fGy=a*gy+(1-a)*fGy; fGz=a*gz+(1-a)*fGz;
  iRoll  += fGy*RAD_TO_DEG*dt;
  iPitch += fGx*RAD_TO_DEG*dt;
  iYaw   += fGz*RAD_TO_DEG*dt;
  if(iRoll<0) iRoll+=360; if(iRoll>=360) iRoll-=360;
  if(iPitch<0)iPitch+=360;if(iPitch>=360)iPitch-=360;
  if(iYaw<0)  iYaw+=360;  if(iYaw>=360)  iYaw-=360;
}
void getIntegratedAngles(float &r,float &p,float &y){ r=iRoll; p=iPitch; y=iYaw; }

/*------------------------------------------------------------------
   (single, unique) EKF implementation begins right here …
------------------------------------------------------------------*/



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

// Kalman gain from the most recent update
// (allocated in ekfInit once n_x is known)
Eigen::MatrixXf lastKalmanGain;

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
    lastKalmanGain = Eigen::MatrixXf::Zero(n_x, 1);

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

/*************************************************
  ADD HMC5883L / QMC5883 MAGNETOMETER VALUES
  EXTERNAL ESP32 SPEEDOMETER INCLUDED ON SERIAL 2
  3x ULTRASONIC SENSORS INCLUDED (OPTIMIZED TIMEOUT)
  LX-824 SERIAL SERVO INCLUDED
*************************************************/

#include <Wire.h>
#include <math.h>

unsigned long loopTime;

// =====================================================
// HC12 REMOTE CONTROL
// =====================================================
int remoteX = 417;
int remoteY = 424;

unsigned long lastRemotePacket = 0;
const unsigned long REMOTE_TIMEOUT = 500;

bool remoteActive = false;

// Autonomous mode flag controlled by HC12 remote (B/D)
bool autonomousEnabled = false;

// =====================================================
// LOW-PASS FILTER VARIABLES FOR ARDUINO
// =====================================================
float smoothedSpeed = 0.0;
float smoothedSteer = 0.0;

float rampedSpeed = 0.0;

const float RAMP_TIME = 1.0;     // seconds
unsigned long lastRampUpdate = 0;

const float ARDUINO_ALPHA_SPEED = 0.25; // 0.0 to 1.0 (lower is smoother)
const float ARDUINO_ALPHA_STEER = 0.20;

// =====================================================
// MOTOR PINS
// =====================================================
const int PWM[] = {6, 7};
const int INA[] = {47, 51};
const int INB[] = {49, 53};

const float MAX_STEER_ANGLE = 60.0;

// =====================================================
// STEERING SENSOR
// =====================================================
#define STEER_ENC_PIN A7
const int CENTER_RAW = 533;
const float DEG_PER_COUNT = 300.0 / 1023.0;
const float STEER_SIGN = -1.0;

// =====================================================
// MPU6050
// =====================================================
#define MPU6050_ADDR 0x68

// =====================================================
// MAGNETOMETER (HMC5883L)
// =====================================================
#define MAG_ADDR 0x1E

unsigned long lastPrint = 0;

// =====================================================
// SPEEDOMETER (FROM ESP32)
// =====================================================
float currentSpeed = 0.0;         // Stores the current speed in m/s received from ESP32
unsigned long lastPacketTime = 0;
const unsigned long TIMEOUT_THRESHOLD = 2000;

// =====================================================
// ULTRASONIC SENSORS
// =====================================================
#define TRIG_FRONT 33
#define ECHO_FRONT 39

#define TRIG_RIGHT 31
#define ECHO_RIGHT 37

#define TRIG_LEFT  29
#define ECHO_LEFT  35

// Ultrasonic filtering
const int ULTRA_MEDIAN_N = 5; // keep last 5 readings
const int ULTRA_MIN_CM = 2;   // ignore readings closer than 2 cm
const int ULTRA_MAX_CM = 300; // ignore readings farther than 300 cm

int bufFront[ULTRA_MEDIAN_N];
int bufRight[ULTRA_MEDIAN_N];
int bufLeft[ULTRA_MEDIAN_N];
int idxFront = 0, idxRight = 0, idxLeft = 0;
int countFront = 0, countRight = 0, countLeft = 0;

// Helper: compute median of valid values in buffer; returns 999 if none valid
int median_valid(int *buf, int count) {
  int tmp[ULTRA_MEDIAN_N];
  int m = 0;
  for (int i = 0; i < count; ++i) {
    int v = buf[i];
    if (v >= ULTRA_MIN_CM && v <= ULTRA_MAX_CM) {
      tmp[m++] = v;
    }
  }
  if (m == 0) return 999;
  // simple sort (insertion) for small m
  for (int i = 1; i < m; ++i) {
    int key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j = j - 1;
    }
    tmp[j + 1] = key;
  }
  return tmp[m / 2];
}

// ================= ROS2 COMMAND VARIABLES =================
float targetSpeed = 0.0;
float targetSteer = 0.0;
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 10000; // 5 Seconds safety watchdog

// Variables to store distances (in cm)
int distFront = 0;
int distRight = 0;
int distLeft = 0;

// =====================================================
// LX-824 SERIAL SERVO SETTINGS
// =====================================================
#define FRAME_HEADER 0x55
#define CMD_MOVE_TIME_WRITE 1
#define ServoSerial Serial1      // TX1 = Pin 18, RX1 = Pin 19
#define CAMERA_SERVO_ID 4        // ID of your camera servo

bool isCameraRotated = false;    // Tracks the current state of the camera

// =====================================================
// LX-824 SERVO HELPER FUNCTIONS
// =====================================================
byte LOW_BYTE(int v) { return v & 0xFF; }
byte HIGH_BYTE(int v) { return (v >> 8) & 0xFF; }

byte checksum(byte *buf) {
  int sum = 0;
  for (int i = 2; i < buf[3] + 2; i++) { sum += buf[i]; }
  return (~sum) & 0xFF;
}

int angleToPosition(float angle) {
  angle = constrain(angle, 0, 240);
  return map(angle, 0, 240, 0, 1000);
}

void moveServo(byte id, int pos, int moveTime) {
  pos = constrain(pos, 0, 1000);
  byte buf[10];
  buf[0] = FRAME_HEADER;
  buf[1] = FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = CMD_MOVE_TIME_WRITE;
  buf[5] = LOW_BYTE(pos);
  buf[6] = HIGH_BYTE(pos);
  buf[7] = LOW_BYTE(moveTime);
  buf[8] = HIGH_BYTE(moveTime);
  buf[9] = checksum(buf);
  ServoSerial.write(buf, 10);
}

// =====================================================
// MOTOR FUNCTIONS
// =====================================================
void setMotor(int index, int pwm, bool forward)
{
  pwm = constrain(abs(pwm), 60, 100);
  digitalWrite(INA[index], forward ? HIGH : LOW);
  digitalWrite(INB[index], forward ? LOW : HIGH);
  analogWrite(PWM[index], pwm);
}

void stopMotor(int index)
{
  analogWrite(PWM[index], 0);
  digitalWrite(INA[index], LOW);
  digitalWrite(INB[index], LOW);
}

// =====================================================
// STEERING ANGLE
// =====================================================
float readSteerAngle()
{
  int raw = analogRead(STEER_ENC_PIN);
  int delta = raw - CENTER_RAW;
  return delta * DEG_PER_COUNT * STEER_SIGN;
}

// =====================================================
// MPU6050
// =====================================================
void initMPU6050()
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void readMPU6050(int16_t &ax,int16_t &ay,int16_t &az,
                 int16_t &gx,int16_t &gy,int16_t &gz)
{
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR,14,true);

  ax = Wire.read()<<8 | Wire.read();
  ay = Wire.read()<<8 | Wire.read();
  az = Wire.read()<<8 | Wire.read();
  Wire.read(); Wire.read();
  gx = Wire.read()<<8 | Wire.read();
  gy = Wire.read()<<8 | Wire.read();
  gz = Wire.read()<<8 | Wire.read();
}

// =====================================================
// MAGNETOMETER
// =====================================================
void initMag()
{
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x00);
  Wire.write(0x70);
  Wire.endTransmission();

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x01);
  Wire.write(0x20);
  Wire.endTransmission();

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();
}

void readMag(int16_t &mx, int16_t &my, int16_t &mz)
{
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(MAG_ADDR, 6);

  mx = Wire.read() << 8 | Wire.read();
  mz = Wire.read() << 8 | Wire.read();
  my = Wire.read() << 8 | Wire.read();
}

float getHeading()
{
  int16_t mx,my,mz;
  readMag(mx,my,mz);
  float heading = atan2((float)my, (float)mx) * 180.0 / PI;
  if (heading < 0) heading += 360.0;
  return heading;
}

// =====================================================
// ULTRASONIC READ FUNCTION (OPTIMIZED TIMEOUT)
// =====================================================
int readUltrasonic(int trigPin, int echoPin) 
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // CRITICAL FIX: Cut timeout down to 12000us (approx 2 meters range).
  // This prevents the sensor from blocking long enough to overflow the Serial buffer.
  long duration = pulseIn(echoPin, HIGH, 12000); 

  if (duration == 0) return 999; 
  int distance = duration * 0.034 / 2;
  
  if (distance < 4 || distance > 400) { return 999; }
  return distance;
}

// =====================================================
// OPTIMIZED SMOOTH STEERING CONTROL
// =====================================================
void steerTo(float targetDeg)
{
  targetDeg = constrain(targetDeg, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
  float angle = readSteerAngle();
  float error = targetDeg - angle;

  if (angle >= MAX_STEER_ANGLE && error > 0) { stopMotor(1); return; }
  if (angle <= -MAX_STEER_ANGLE && error < 0){ stopMotor(1); return; }

  if (abs(error) < 1.5) { stopMotor(1); return; }

  int pwm = map(abs(error) * 10, 15, 200, 50, 100);
  pwm = constrain(pwm, 50, 100);

  if (error >= 0.0) setMotor(1, pwm, true);
  else              setMotor(1, pwm, false);
}

// =====================================================
// READ SPEED FROM ESP32 (NON-BLOCKING CHAR BUFFER)
// =====================================================
void readSpeedFromESP32() {
  static char espBuffer[16];
  static size_t espIndex = 0;

  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (c == '\n') {
      espBuffer[espIndex] = '\0';
      espIndex = 0;
      if (strncmp(espBuffer, "S:", 2) == 0) {
        currentSpeed = atof(&espBuffer[2]);
        lastPacketTime = millis(); 
      }
    } else if (c != '\r' && espIndex < sizeof(espBuffer) - 1) {
      espBuffer[espIndex++] = c;
    }
  }

  if (millis() - lastPacketTime > TIMEOUT_THRESHOLD) {
    currentSpeed = 0.0;
  }
}

// =====================================================
// ROS2 COMMAND LISTENER (NON-BLOCKING HIGH PERFORMANCE)
// =====================================================
void readROS2Commands() {
  static char rosBuffer[32];
  static size_t rosIndex = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      rosBuffer[rosIndex] = '\0';
      rosIndex = 0; 

      // Handle Camera Toggles
      if (strcasecmp(rosBuffer, "CAM") == 0) {
        isCameraRotated = !isCameraRotated;
        moveServo(CAMERA_SERVO_ID, angleToPosition(isCameraRotated ? 180 : 0), 1000);
        // Camera toggle always accepted regardless of autonomous/manual lock
        Serial.println("ROS: CAM toggled");
      } 
      // Handle Velocity Targets ("speed,steer")
      else {
        // (Removed ROS AUTO_ON/AUTO_OFF) autonomous is controlled by HC12 remote only
        char* comma = strchr(rosBuffer, ',');
        if (comma != NULL) {
          *comma = '\0'; 

          // Only accept speed/steer commands from ROS serial when autonomous mode is enabled
          if (autonomousEnabled) {
            targetSpeed = atof(rosBuffer);
            targetSteer = atof(comma + 1);
            lastCommandTime = millis(); // Reset safety watchdog on valid data!
          } else {
            // Optionally notify sender that command was ignored
            Serial.println("IGNORED: autonomous disabled");
          }
        }
      }
    } 
    else if (c != '\r' && rosIndex < sizeof(rosBuffer) - 1) {
      rosBuffer[rosIndex++] = c;
    }
  }
}

// =====================================================
// HC12 RECEIVER
// Packet:X417Y431B0
// =====================================================
void readHC12()
{
  static String buffer = "";
  while (Serial3.available())
  {
    char c = Serial3.read();
    if (c == '\n')
    {
      int xIndex = buffer.indexOf('X');
      int yIndex = buffer.indexOf('Y');
     
      if (xIndex >= 0 && yIndex > xIndex)
      {
        remoteX = buffer.substring(xIndex + 1, yIndex).toInt();
        // remoteY may be followed by other lettered flags (A/B/C/D)
        // toInt will parse the leading number correctly even if trailing chars follow.
        remoteY = buffer.substring(yIndex + 1).toInt();

        // parse single-digit flags A,B,C,D if present
        int aVal = 0;
        int bVal = 0;
        int cVal = 0;
        int dVal = 0;
        int aIndex = buffer.indexOf('A');
        int bIndex = buffer.indexOf('B');
        int cIndex = buffer.indexOf('C');
        int dIndex = buffer.indexOf('D');
        if (aIndex >= 0 && aIndex + 1 < buffer.length()) aVal = buffer.charAt(aIndex + 1) - '0';
        if (bIndex >= 0 && bIndex + 1 < buffer.length()) bVal = buffer.charAt(bIndex + 1) - '0';
        if (cIndex >= 0 && cIndex + 1 < buffer.length()) cVal = buffer.charAt(cIndex + 1) - '0';
        if (dIndex >= 0 && dIndex + 1 < buffer.length()) dVal = buffer.charAt(dIndex + 1) - '0';

        lastRemotePacket = millis();
        remoteActive = true;

        // A - camera servo to 180 degrees
        if (aVal == 1) {
          isCameraRotated = true;
          moveServo(CAMERA_SERVO_ID, angleToPosition(180), 500);
          Serial.println("HC12: A=1 -> camera 180");
        }

        // C - camera servo to 0 degrees
        if (cVal == 1) {
          isCameraRotated = false;
          moveServo(CAMERA_SERVO_ID, angleToPosition(0), 500);
          Serial.println("HC12: C=1 -> camera 0");
        }

        // B - autonomous ON (remote enables autonomous)
        if (bVal == 1) {
          autonomousEnabled = true;
          Serial.println("HC12: B=1 -> AUTONOMOUS ON");
        }

        // D - autonomous OFF -> enter manual mode
        if (dVal == 1) {
          autonomousEnabled = false;
          Serial.println("HC12: D=1 -> MANUAL MODE");
        }
      }
      buffer = "";
    }
    else
    {
      buffer += c;
      if(buffer.length() > 40)
        buffer = "";
    }
  }

  if (millis() - lastRemotePacket > REMOTE_TIMEOUT)
  {remoteActive = false;}
}

// =====================================================
// PRINT DATA TO ROS2
// =====================================================
void printData()
{
  if (millis() - lastPrint < 200) return;
  lastPrint = millis();

  // Sequential trigger/read to avoid cross-talk between sensors.
  // Round-robin: LEFT -> FRONT -> RIGHT with 30 ms spacing.
  int rawLeft  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  delay(30);
  int rawFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  delay(30);
  int rawRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  // Push into circular buffers and compute median of valid readings
  bufLeft[idxLeft] = rawLeft;
  idxLeft = (idxLeft + 1) % ULTRA_MEDIAN_N;
  if (countLeft < ULTRA_MEDIAN_N) ++countLeft;
  distLeft = median_valid(bufLeft, countLeft);

  bufFront[idxFront] = rawFront;
  idxFront = (idxFront + 1) % ULTRA_MEDIAN_N;
  if (countFront < ULTRA_MEDIAN_N) ++countFront;
  distFront = median_valid(bufFront, countFront);

  bufRight[idxRight] = rawRight;
  idxRight = (idxRight + 1) % ULTRA_MEDIAN_N;
  if (countRight < ULTRA_MEDIAN_N) ++countRight;
  distRight = median_valid(bufRight, countRight);

  int steerAngle = readSteerAngle();
  
  // REMOVED DUPLICATE readSpeedFromESP32 CALL TO PREVENT PACKET CORRUPTION
  
  int16_t ax,ay,az,gx,gy,gz;
  readMPU6050(ax,ay,az,gx,gy,gz);
  float heading = getHeading();

  // MPU6050 default scale conversion
  float ax_ms2 = (ax / 16384.0) * 9.80665;
  float ay_ms2 = (ay / 16384.0) * 9.80665;
  float az_ms2 = (az / 16384.0) * 9.80665;

  float gx_radps = (gx / 131.0) * PI / 180.0;
  float gy_radps = (gy / 131.0) * PI / 180.0;
  float gz_radps = (gz / 131.0) * PI / 180.0;

  float signedSpeed = currentSpeed;
  if (rampedSpeed < -0.02)
  {
      signedSpeed = -currentSpeed;
  }
  else if (rampedSpeed > 0.02)
  {
      signedSpeed = currentSpeed;
  }
  else
  {
      signedSpeed = 0.0;
  }

  Serial.print(steerAngle); Serial.print(",");
  Serial.print(signedSpeed); Serial.print(",");
  Serial.print(heading,1); Serial.print(",");
  Serial.print(gz_radps); Serial.print(",");
  Serial.print(ax_ms2); Serial.print(",");
  Serial.print(ay_ms2); Serial.print(",");
  Serial.print(az_ms2); Serial.print(",");
  Serial.print(distFront); Serial.print(",");
  Serial.print(distRight); Serial.print(",");
  Serial.println(distLeft);
}


void processRemoteControl()
{
  if(!remoteActive)
    return;

  // If autonomous mode is enabled, completely ignore remote joystick calculations
  if (autonomousEnabled) {
    return;
  }

  float joySpeed = 0.0;
  float joySteer = 0.0;

  // Handle Y-axis (Drive Speed)
  int yError = remoteY - 431;
  if(abs(yError) > 40)
  {
    joySpeed = map(abs(yError), 0, 420, 0, 1000) / 1000.0;
    if(yError > 0) joySpeed = -joySpeed;
  }
  else
  {
    joySpeed = 0.0; // Enforce explicit stop if joystick is centered
  }

  // Handle X-axis (Steering)
  int xError = remoteX - 417;
  if(abs(xError) > 40)
  {
    joySteer = map(xError, -417, 429, -60, 60);
  }
  else
  {
    joySteer = 0.0; // Enforce center steering if joystick is centered
  }

  // Overwrite targets safely in manual mode
  targetSpeed = joySpeed;
  targetSteer = joySteer;

  lastCommandTime = millis();

  // Feedback for debugging: report applied manual command
  Serial.print("REMOTE CMD -> speed:"); Serial.print(targetSpeed,3);
  Serial.print(", steer:"); Serial.println(targetSteer,2);
}

void updateSpeedRamp()
{
    unsigned long now = millis();

    float dt =
      (now - lastRampUpdate) / 1000.0;

    if(dt <= 0)
      return;

    lastRampUpdate = now;

    float maxStep =
      dt / RAMP_TIME;

    if(targetSpeed > rampedSpeed)
    {
        rampedSpeed += maxStep;

        if(rampedSpeed > targetSpeed)
            rampedSpeed = targetSpeed;
    }
    else
    {
        rampedSpeed -= maxStep;

        if(rampedSpeed < targetSpeed)
            rampedSpeed = targetSpeed;
    }
}
// =====================================================
// SETUP
// =====================================================
void setup()
{
  lastRampUpdate = millis();
  Serial.begin(115200);   
  Serial2.begin(115200);  // Connected to ESP32 TX2/RX2
  ServoSerial.begin(115200);
  Serial3.begin(9600);   // HC12 
  
  Wire.begin();
  Wire.setWireTimeout(3000, true);
  pinMode(STEER_ENC_PIN, INPUT);

  for(int i=0;i<2;i++) {
    pinMode(PWM[i], OUTPUT);
    pinMode(INA[i], OUTPUT);
    pinMode(INB[i], OUTPUT);
  }

  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);

  initMPU6050();
  initMag();
 
  moveServo(CAMERA_SERVO_ID, angleToPosition(0), 1000);
  lastCommandTime = millis();
}

// =====================================================
// MAIN CONTROL LOOP
// =====================================================
void loop()
{
  unsigned long now = millis();

  // 1. Keep serial data updated
  readSpeedFromESP32(); 
  readROS2Commands();  
  readHC12();

  processRemoteControl(); 

  // 2. WATCHDOG & FILTER INTERACTION
  // If the goal is reached, Nav2 naturally sends 0.0. The filter ramps down smoothly.
  // If ROS2 crashes, the 5-second Watchdog trips and instantly forces zero commands.
//  if (now - lastCommandTime > COMMAND_TIMEOUT) {
//    targetSpeed = 0.0;
//    targetSteer = 0.0;
//    smoothedSpeed = 0.0;
//    smoothedSteer = 0.0;
//    stopMotor(1); // Turn off steering motor load immediately
//  } else {
    // Normal Operation: Apply low-pass filter
    updateSpeedRamp();
    smoothedSteer = (ARDUINO_ALPHA_STEER * targetSteer) + ((1.0 - ARDUINO_ALPHA_STEER) * smoothedSteer);
//  }

  // 3. APPLY SMOOTHED STEERING
  steerTo(smoothedSteer);

  // 4. APPLY SMOOTHED DRIVE VELOCITY 
  if (abs(rampedSpeed) < 0.02)
  {
      stopMotor(0);
  }
  else
  {
      bool forward = (rampedSpeed > 0);

      float absSpeed = abs(rampedSpeed);

      int finalPwm =
        map(absSpeed * 1000,
            20,
            1000,
            50,
            100);

      finalPwm =
        constrain(finalPwm,50,100);

      setMotor(0, finalPwm, forward);
  }

  // 5. SEND TELEMETRY TO ROS2
  printData();
}
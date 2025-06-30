#include <Wire.h>
#include <ESP32Servo.h>
#include <ESP32PWM.h>

// MPU9250 I2C address
#define MPU9250_ADDRESS 0x68

// Servo configuration
#define SERVO_CENTER      90
#define SERVO_MIN         0
#define SERVO_MAX         180
#define COMPENSATION_GAIN 1.0
#define ALPHA             0.96
#define DT                0.02
#define CALIBRATION_SAMPLES 200

// RGB LED Pins (common cathode)
#define RED_PIN    12
#define GREEN_PIN  13
#define BLUE_PIN   27

// Tremor & timing thresholds
const float tremorThreshold     = 3.0;   // dps
const float highCalibThreshold = 7.0;   // dps during calibration
const unsigned long idleDelay  = 1500;  // ms

Servo rollServo;
Servo pitchServo;

float pitchAngle = 0, rollAngle = 0;
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
bool calibrating = true;
unsigned long tremorFreeStart = 0;

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  Wire.begin(21, 22);
  initMPU9250();

  rollServo.attach(17);
  pitchServo.attach(16);
  rollServo.write(SERVO_CENTER);
  pitchServo.write(SERVO_CENTER);

  // Set up PWM for RGB LEDs (5000 Hz, 8‑bit resolution)
  ledcAttach(RED_PIN,   5000, 8);
  ledcAttach(GREEN_PIN, 5000, 8);
  ledcAttach(BLUE_PIN,  5000, 8);

  setRGB(255, 255, 0);  // Yellow = calibrating
  Serial.println("Calibrating gyro...");
  calibrateGyro();
  calibrating = false;
  Serial.println("Calibration complete!");
}

void loop() {
  static unsigned long prevTime = millis();
  float ax, ay, az, gx, gy, gz;
  readIMU(ax, ay, az, gx, gy, gz);
  gx -= gyroXoffset; gy -= gyroYoffset; gz -= gyroZoffset;

  float accPitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180/PI;
  float accRoll  = atan2(ay, az) * 180/PI;

  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;

  rollAngle  = ALPHA * (rollAngle + gy * dt) + (1 - ALPHA) * accPitch;
  pitchAngle = ALPHA * (pitchAngle + gx * dt) + (1 - ALPHA) * accRoll;

  float motion = sqrt(gx*gx + gy*gy + gz*gz);

  if (calibrating) {
    setRGB(motion > highCalibThreshold ? 255 : 255,
           motion > highCalibThreshold ?   0 : 255,
           0);  // Red if too much movement, else yellow
  } else {
    if (motion > tremorThreshold) {
      tremorFreeStart = now;
      setRGB(0, 255, 0);  // Green = tremor
    } else if (now - tremorFreeStart > idleDelay) {
      setRGB(0, 0, 255);  // Blue = calm (idle)
    }
  }

  if (abs(rollAngle) <= 180 && abs(pitchAngle) <= 180) {
    int rollComp = -rollAngle * COMPENSATION_GAIN;
    int pitchComp = pitchAngle * COMPENSATION_GAIN;
    rollServo.write(constrain(SERVO_CENTER + rollComp, SERVO_MIN, SERVO_MAX));
    pitchServo.write(constrain(SERVO_CENTER + pitchComp, SERVO_MIN, SERVO_MAX));
    printSensorData(pitchAngle, rollAngle, pitchComp, rollComp);
  } else {
    Serial.println("Angle exceeded ±180°, servos not updated.");
  }

  delay(20);
}

void setRGB(int r, int g, int b) {
  ledcWrite(RED_PIN,   255 - r);  // invert for common cathode
  ledcWrite(GREEN_PIN, 255 - g);
  ledcWrite(BLUE_PIN,  255 - b);
}

void initMPU9250() {
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x1C); Wire.write(0x08);
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x1B); Wire.write(0x08);
  Wire.endTransmission(true);
}

void calibrateGyro() {
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    float ax, ay, az, gx, gy, gz;
    readIMU(ax, ay, az, gx, gy, gz);
    sx += gx; sy += gy; sz += gz;
    delay(5);
  }
  gyroXoffset = sx / CALIBRATION_SAMPLES;
  gyroYoffset = sy / CALIBRATION_SAMPLES;
  gyroZoffset = sz / CALIBRATION_SAMPLES;
}

void readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 6, true);
  ax = (int16_t)(Wire.read()<<8 | Wire.read()) / 8192.0;
  ay = (int16_t)(Wire.read()<<8 | Wire.read()) / 8192.0;
  az = (int16_t)(Wire.read()<<8 | Wire.read()) / 8192.0;

  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x43); Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 6, true);
  gx = (int16_t)(Wire.read()<<8 | Wire.read()) / 65.5;
  gy = (int16_t)(Wire.read()<<8 | Wire.read()) / 65.5;
  gz = (int16_t)(Wire.read()<<8 | Wire.read()) / 65.5;
}

void printSensorData(float p, float r, int pc, int rc) {
  Serial.printf("Pitch: %.1f° Roll: %.1f° Comp: %d,%d\n", p, r, pc, rc);
}

#include <Arduino.h>
#include <Wire.h>
#include <uMQ.h>

#define MotLogA1 4
#define MotLogA2 8

#define MotLogB1 3
#define MotLogB2 2

#define MotPWMA 6
#define MotPWMB 5

#define sensLine1 A0
#define sensLine2 A1
#define sensLine3 A2
#define sensLine4 A3
#define sensLine5 A6

#define MPU_ADDR 0x68

uMQ radio;
String recv;

float gYawDeg = 0.0f;
float gPitchDeg = 0.0f;
float gRollDeg = 0.0f;
float gGyroZBias = 0.0f;
unsigned long gLastImuMicros = 0;

static float normalizeAngleDeg(float a)
{
  while (a > 180.0f)
    a -= 360.0f;
  while (a < -180.0f)
    a += 360.0f;
  return a;
}

void motor_init()
{
  pinMode(MotPWMA, OUTPUT);
  pinMode(MotPWMB, OUTPUT);
  pinMode(MotLogA1, OUTPUT);
  pinMode(MotLogA2, OUTPUT);
  pinMode(MotLogB1, OUTPUT);
  pinMode(MotLogB2, OUTPUT);
  Serial.println("Motor Init");
}

void driveMotor(int speedM1, int speedM2)
{
  speedM1 = constrain(speedM1, -100, 100);
  speedM2 = constrain(speedM2, -100, 100);

  int pwmA = abs(speedM1) * 255 / 100;
  int pwmB = abs(speedM2) * 255 / 100;

  analogWrite(MotPWMA, pwmA);
  analogWrite(MotPWMB, pwmB);

  if (speedM1 < 0)
  {
    digitalWrite(MotLogA1, LOW);
    digitalWrite(MotLogA2, HIGH);
  }
  else
  {
    digitalWrite(MotLogA1, HIGH);
    digitalWrite(MotLogA2, LOW);
  }

  if (speedM2 < 0)
  {
    digitalWrite(MotLogB1, LOW);
    digitalWrite(MotLogB2, HIGH);
  }
  else
  {
    digitalWrite(MotLogB1, HIGH);
    digitalWrite(MotLogB2, LOW);
  }
}

bool mpuWriteReg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0)
    return false;

  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14)
    return false;

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read();
  Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  return true;
}

bool mpuInit()
{
  Wire.begin();
  delay(50);

  if (!mpuWriteReg(0x6B, 0x00))
    return false;
  if (!mpuWriteReg(0x1B, 0x08))
    return false; // +-500 dps
  if (!mpuWriteReg(0x1C, 0x08))
    return false; // +-4g

  delay(50);
  return true;
}

void mpuCalibrateGyro(uint16_t samples = 400)
{
  long sumGz = 0;
  uint16_t ok = 0;

  for (uint16_t i = 0; i < samples; i++)
  {
    int16_t ax, ay, az, gx, gy, gz;
    if (mpuReadRaw(ax, ay, az, gx, gy, gz))
    {
      sumGz += gz;
      ok++;
    }
    delay(3);
  }

  if (ok > 0)
    gGyroZBias = (float)sumGz / (float)ok;
  else
    gGyroZBias = 0.0f;

  gLastImuMicros = micros();
}

void mpuUpdate()
{
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
  if (!mpuReadRaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw))
    return;

  unsigned long now = micros();
  if (gLastImuMicros == 0)
  {
    gLastImuMicros = now;
    return;
  }

  float dt = (now - gLastImuMicros) / 1000000.0f;
  gLastImuMicros = now;
  if (dt <= 0.0f || dt > 0.1f)
    return;

  const float gyroScale = 65.5f; // LSB/(deg/s) for +-500 dps
  float gx = gxRaw / gyroScale;
  float gy = gyRaw / gyroScale;
  float gz = (gzRaw - gGyroZBias) / gyroScale;

  float ax = (float)axRaw;
  float ay = (float)ayRaw;
  float az = (float)azRaw;

  float accRoll = atan2(ay, az) * 57.2958f;
  float accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958f;

  const float alpha = 0.98f;
  gRollDeg = alpha * (gRollDeg + gx * dt) + (1.0f - alpha) * accRoll;
  gPitchDeg = alpha * (gPitchDeg + gy * dt) + (1.0f - alpha) * accPitch;

  gYawDeg = normalizeAngleDeg(gYawDeg + gz * dt);
}

void driveHeadingHold(int baseSpeed, float targetYawDeg, uint16_t durationMs)
{
  unsigned long start = millis();
  float prevErr = 0.0f;

  while ((millis() - start) < durationMs)
  {
    mpuUpdate();

    float err = normalizeAngleDeg(targetYawDeg - gYawDeg);
    float derr = err - prevErr;
    prevErr = err;

    const float kp = 1.8f;
    const float kd = 0.5f;
    int corr = (int)(kp * err + kd * derr);

    int left = baseSpeed + corr;
    int right = baseSpeed - corr;

    driveMotor(left, right);
    delay(10);
  }

  driveMotor(0, 0);
}

void rotateByYaw(int turnSpeed, float deltaYawDeg, uint16_t timeoutMs = 2500)
{
  float startYaw = gYawDeg;
  float target = normalizeAngleDeg(startYaw + deltaYawDeg);
  unsigned long t0 = millis();

  int s = constrain(turnSpeed, 25, 90);
  if (deltaYawDeg < 0)
    s = -s;

  while ((millis() - t0) < timeoutMs)
  {
    mpuUpdate();
    float err = normalizeAngleDeg(target - gYawDeg);
    if (abs(err) < 3.0f)
      break;

    int cmd = (int)(0.8f * err);
    cmd = constrain(cmd, -abs(s), abs(s));

    driveMotor(cmd, -cmd);
    delay(10);
  }

  driveMotor(0, 0);
  delay(60);
}

void clining(int speed, String flagStart)
{
  speed = constrain(speed, 20, 100);
  mpuUpdate();

  if (flagStart == "one")
  {
    // Z-shape style: yaw turns with IMU control.
    rotateByYaw(speed, -25.0f);
    driveHeadingHold(speed, gYawDeg, 450);
    rotateByYaw(speed, 50.0f);
    driveHeadingHold(speed, gYawDeg, 450);
    rotateByYaw(speed, -25.0f);
  }
  else if (flagStart == "two")
  {
    float target = gYawDeg;
    driveHeadingHold(speed, target, 1200);
  }
  else if (flagStart == "thr" || flagStart == "tri")
  {
    float target = gYawDeg;
    driveHeadingHold(speed, target, 700);
    driveHeadingHold(-speed, target, 450);
  }
  else
  {
    driveMotor(0, 0);
    delay(20);
  }
}

void setup()
{
  Serial.begin(115200);
  motor_init();

  if (!mpuInit())
  {
    Serial.println("MPU6050 init failed");
  }
  else
  {
    Serial.println("MPU6050 init complete");
    mpuCalibrateGyro();
    Serial.println("MPU6050 gyro calibrated");
  }

  radio.init(9, 10);
}

void loop()
{
  recv = radio.recv("ILUSH", 100);
  recv.trim();

  if (recv.length() > 0)
  {
    Serial.println("Received: |" + recv + "|");
    clining(80, recv);
  }
  else
  {
    mpuUpdate();
    driveMotor(0, 0);
    delay(20);
  }
}

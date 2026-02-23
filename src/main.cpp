#include <Arduino.h>
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

#define Ts_us 1000

uMQ radio;
String recv;

void setup()
{
  Serial.begin(115200);
  radio.init(9, 10);
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

  float speedM1p = (speedM1 / 100.0) * 255.0;
  float speedM2p = (speedM2 / 100.0) * 255.0;

  analogWrite(MotPWMA, speedM1p);
  analogWrite(MotPWMB, speedM2p);

  if (speedM1p < 0)
  {
    digitalWrite(MotLogA1, LOW);
    digitalWrite(MotLogA2, HIGH);
  }
  else
  {
    digitalWrite(MotLogA1, HIGH);
    digitalWrite(MotLogA2, LOW);
  }

  if (speedM2p < 0)
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

void clining(int speed, String FlagStart)
{
  if (FlagStart == "one")
  {
    int procent = 2;
    for (int i = 0; i < procent; i++)
    {
      driveMotor(0, speed);
      delay(100);
      driveMotor(speed, 0);
      delay(100);
    }
  }
  else if (FlagStart == "two")
  {
    int procent = 2;
    for (int i = 0; i < procent; i++)
    {
      driveMotor(80, 80);
      delay(500);
    }
  }
  else if (FlagStart == "thr")
  {
    int procent = 2;
    for (int i = 0; i < procent; i++)
    {
      driveMotor(speed, speed);
      delay(600);

      driveMotor(-speed, -speed);
      delay(300);
    }
    driveMotor(0, 0);
    delay(100);
  }
  else
  {
    driveMotor(0, 0);
    delay(100);
  }
}

void loop()
{
  recv = radio.recv("ILUSH", 100);
  Serial.println("Received: |" + recv + "|");
  recv.trim();
  clining(80, recv);
  // clining(80, "one");
}
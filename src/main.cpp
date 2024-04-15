#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <HCSR04.h>

#include "MotorControl.h"
#include "MPU6050MadgwickAHRS.h"

//change it first
#define MOT1_ENC1 21
#define MOT1_DIRECT 20
#define MOT2_ENC1 19
#define MOT2_DIRECT 18

#define triggerPin 13
#define echoPin 12

UltraSonicDistanceSensor distanceSensor(triggerPin, echoPin);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);

volatile int MOT1_ENC1_TICKS = 0;
volatile int MOT2_ENC1_TICKS = 0;


void ISR_A() {
  if (digitalRead(MOT1_ENC1) == digitalRead(MOT1_DIRECT)) {
    MOT1_ENC1_TICKS++;
  } else {
    MOT1_ENC1_TICKS--;
  }
}

void ISR_B() {
  if (digitalRead(MOT2_ENC1) == digitalRead(MOT2_DIRECT)) {
    MOT2_ENC1_TICKS++;
  } else {
    MOT2_ENC1_TICKS--;
  }
}

float getDistance() {
  return distanceSensor.measureDistanceCm();
}

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(MOT1_ENC1), ISR_A, RISING);
  attachInterrupt(digitalPinToInterrupt(MOT2_ENC1), ISR_B, RISING);
  pwm1.begin();
  pwm1.setPWMFreq(60);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  delay(500);
}

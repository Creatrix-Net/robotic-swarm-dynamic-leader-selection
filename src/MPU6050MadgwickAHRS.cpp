#include "MPU6050MadgwickAHRS.h"
#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <MadgwickAHRS.h>
Madgwick filter;
float accelScale, gyroScale;

Adafruit_MPU6050 mpu;

MPU6050MadgwickAHRS::MPU6050MadgwickAHRS() {
    Serial.begin(115200);
  if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
		  delay(10);
		}
	}
	Serial.println("MPU6050 Found!");
	// set accelerometer range to +-8G
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
	// set gyro range to +- 500 deg/s
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    // set filter bandwidth to 21 Hz
	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
	delay(100);
}

float MPU6050MadgwickAHRS::convertRawAcceleration(int aRaw) {
  // since we are using 2 g range
  // -2 g maps to a raw value of -32768
  // +2 g maps to a raw value of 32767
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float MPU6050MadgwickAHRS::convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

void MPU6050MadgwickAHRS::update() {
    sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);
    gx = convertRawGyro(g.gyro.x);
    gy = convertRawGyro(g.gyro.y);
    gz = convertRawGyro(g.gyro.z);
    ax = convertRawAcceleration(a.acceleration.x);
    ay = convertRawAcceleration(a.acceleration.y);
    az = convertRawAcceleration(a.acceleration.z);
    filter.updateIMU(gx, gy, gz, ax, ay, az);
}

float MPU6050MadgwickAHRS::getRollU() {
    return filter.getRoll();
}

float MPU6050MadgwickAHRS::getPitchU() {
    return filter.getPitch();
}

float MPU6050MadgwickAHRS::getYawU() {
    return filter.getYaw();
}

float MPU6050MadgwickAHRS::getRollRadiansU() {
    return filter.getRollRadians();
}

float MPU6050MadgwickAHRS::getPitchRadiansU() {
    return filter.getPitchRadians();
}

float MPU6050MadgwickAHRS::getYawRadiansU() {
    return filter.getYawRadians();
}

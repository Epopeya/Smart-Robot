#include <Arduino.h>
#include <MPU9250.h>
#include <debug.h>

#include "httpServer.h"

#define MPU_ADDRESS 0x68
#define MPU_CALIBRATION_ITERATIONS 1000

#define DIR_KP 1.2
#define DIR_KI 0
#define DIR_KD 1.2

long gyro_dt;

// PID variables
float error, last_error, cum_error, rate_error;

MPU9250 mpu;
HardwareSerial hs(1);

float current_direction = 0.0f;
unsigned long gyro_last_time = 0;
float gyro_offset;
float target_angle = 0.0f;

int total_encoders = 0;

void motorSpeed(int speed) {
  hs.print("M");
  hs.println(speed);
}

void servoAngle(int angle) {
  hs.print("S");
  hs.println(angle);
}

void calibrateImu() {
  float totalAngle = 0;
  for(int i = 0; i < MPU_CALIBRATION_ITERATIONS; i++) {
    while(!mpu.update()) {}
    totalAngle += mpu.getGyroZ();
  }
  gyro_offset = totalAngle / MPU_CALIBRATION_ITERATIONS;
}

bool updateGyro() {
  if(mpu.update()) {
    long current_millis = millis();
    gyro_dt = current_millis - gyro_last_time;
    gyro_last_time = current_millis;
    float gyro_value = mpu.getGyroZ() - gyro_offset;
    current_direction += gyro_value * gyro_dt / 1000;
    return true;
  }
  return false;
}

float computeServoSpeed() {
  error = target_angle - current_direction;
  cum_error += error * gyro_dt;
  rate_error = (error - last_error) / gyro_dt;

  float output = DIR_KP*error + DIR_KI*cum_error + DIR_KD*rate_error;

  last_error = error;

  return output;
}

void setup() {
  // Motor
  hs.begin(1000000, SERIAL_8N1, 4, 2);

  Wire.begin();
  mpu.setup(MPU_ADDRESS);
  calibrateImu();

  motorSpeed(0);
  servoAngle(90);
  debug_init();

  //Debugging
  Serial.begin(9600);
  gyro_last_time = millis();

  setupServer();
}

void loop() {
  if(hs.available() > 0) {
    total_encoders += hs.readStringUntil('\n').toInt();
    if(total_encoders > 200) {
      target_angle += 90;
      total_encoders = 0;
    }
  }
  if(updateGyro()) {
    servoAngle(computeServoSpeed() + 90);
  }
  loopServer();
}
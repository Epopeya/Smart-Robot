#include <Arduino.h>
#include <MPU9250.h>
#include <debug.h>

#include "httpServer.h"
#include <RPLidar.h>

#define MPU_ADDRESS 0x68
#define MPU_CALIBRATION_ITERATIONS 1000
#define RPLIDAR_MOTOR 5

#define DIR_KP 1.2
#define DIR_KI 0
#define DIR_KD 1.2

// You need to create an driver instance 
HardwareSerial hs(1);
HardwareSerial lidar_serial(2);
RPLidar lidar;
MPU9250 mpu;
TaskHandle_t lidar_task;

// PID variables
float error, last_error, cum_error, rate_error;

// navigation vars
float current_direction = 0.0f;
float target_angle = 0.0f;
long gyro_dt;
unsigned long gyro_last_time = 0;
float gyro_offset;

// lidar vars
float left_distance = 0.0f;
float right_distance = 0.0f;
float front_distace = 0.0f;

// slave stats
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

// use absloute angle for this
void liderTask(void * pvParameters) {
  for (;;) {
    vTaskDelay(1);
    

    if (IS_OK(lidar.waitPoint())) {
      float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
      float angle    = lidar.getCurrentPoint().angle; //anglue value in degree

      if (distance < 10.0 || distance > 3000.0) {
        continue;
      }

      float r_angle = angle + (target_angle - current_direction);
      // Serial.println(r_angle);

      // front
      if (r_angle < 15 || r_angle > 345) {
        front_distace = distance;
      }

      // right
      else if (r_angle < 105 || r_angle > 75) {
        right_distance = distance;
        Serial.println(r_angle);
      }

      // left
      else if (r_angle < 285 || r_angle > 255) {
        left_distance = distance;
      }
    }
  }
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

  // begin the lidar
  lidar.begin(lidar_serial);
  rplidar_response_device_info_t info;
  while (!IS_OK(lidar.getDeviceInfo(info, 100))) delay(500);
  rplidar_response_device_health_t health;
  lidar.getHealth(health);
  Serial.println("info: " + String(health.status) +", " + String(health.error_code));
  // detected...
  lidar.startScan();

  xTaskCreatePinnedToCore(
  liderTask,
  "liderTask",
  100000,
  NULL,
  10,
  &lidar_task,
  0);

  analogWrite(RPLIDAR_MOTOR, 255);
}

void loop() {
  updateGyro();
  return;
  Serial.print("left: ");
  Serial.print(left_distance);
  Serial.print(" right: ");
  Serial.print(right_distance);
  Serial.print(" front: ");
  Serial.println(front_distace);


  
  return;
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
}
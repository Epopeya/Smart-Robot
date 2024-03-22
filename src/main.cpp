#include <Arduino.h>
#include <MPU9250.h>
#include "debug.h"

#include <RPLidar.h>

#define MPU_ADDRESS 0x68
#define MPU_CALIBRATION_ITERATIONS 1000
#define RPLIDAR_MOTOR 5
#define MILIMETERS_PER_ENCODER 6
#define SERVO_MIDPOINT 93

#define DIR_KP 0.1
#define DIR_KI 0
#define DIR_KD 0.02

#define LIDAR_SMOOTHING 0.4f
#define LIDAR_INV_SMOOTHING (1 - LIDAR_SMOOTHING)

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
float last_turn_millis = 0.0;
long gyro_dt;
unsigned long gyro_last_time = 0;
float gyro_offset;
float posX = 0;
float posY = 0; 

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
    float gyro_value = (mpu.getGyroZ() - gyro_offset) *2*PI/360;
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
        front_distace =LIDAR_SMOOTHING * front_distace + LIDAR_INV_SMOOTHING * distance;
      }

      // right
      else if (r_angle < 105 && r_angle > 75) {
        right_distance = LIDAR_SMOOTHING * right_distance + LIDAR_INV_SMOOTHING * distance;
      }

      // left
      else if (r_angle < 285 && r_angle > 255) {
        left_distance = LIDAR_SMOOTHING * left_distance + LIDAR_INV_SMOOTHING * distance;
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

  motorSpeed(10);
  servoAngle(SERVO_MIDPOINT);

  //Debugging
  //Serial.begin(9600);
  debug_init();
  gyro_last_time = millis();

  // begin the lidar
  lidar.begin(lidar_serial);
  rplidar_response_device_info_t info;
  //while (!IS_OK(lidar.getDeviceInfo(info, 100))) delay(500);
  rplidar_response_device_health_t health;
  lidar.getHealth(health);
  //Serial.println("info: " + String(health.status) +", " + String(health.error_code));
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

int waypoints[8] = {2000, 2000, 
                    -2000, 2000, 
                    -2000, -2000, 
                    2000, -2000};
int waypoint_index = 0;

void loop() {
  if(hs.available() > 0) {
    int encoders = hs.readStringUntil('\n').toInt() * MILIMETERS_PER_ENCODER;

    // current direction vector
    float dirX = cos(current_direction);
    float dirY = sin(current_direction);

    posX += encoders * dirX;
    posY += encoders * dirY;

    // vector pointing to target
    float targetX = waypoints[waypoint_index] - posX;
    float targetY = waypoints[waypoint_index + 1] - posY;

    // shortest angle to target
    float angle_diff = atan2(targetY, targetX) - atan2(dirY, dirX);
    if (angle_diff > PI) {
      angle_diff = angle_diff - 2*PI;
    } else if (angle_diff < -PI) {
      angle_diff = angle_diff + 2*PI;
    } 

    target_angle = angle_diff + current_direction;

    if((targetX*targetX + targetY*targetY) < 300*300) {
      waypoint_index = (waypoint_index + 2) % 8;
    }    
  }  
  // Serial.print("posX: ");
  // Serial.print(posX);
  // Serial.print(" posY: ");
  // Serial.print(posY);
  // Serial.print(" current dir: ");
  // Serial.print(current_direction);
  // Serial.print(" target angle: ");
  // Serial.println(target_angle);

  if(updateGyro()) {
    servoAngle(computeServoSpeed() * 360.0f/2*PI + SERVO_MIDPOINT); // convert to degrees now
  }
}

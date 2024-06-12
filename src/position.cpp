#include "position.h"

MPU9250 mpu;
float rotation = 0;
float target_rotation = 0;
float absolute_target_rot = 0; // rotation from lidar
float relative_target_rot = 0; // rotation from lidar

unsigned long gyro_last_time = 0;
float gyro_dt = 0;
float gyro_offset = 0;
vector2_t orientation = { .x = 1.0, .y = 0 };

vector2_t position = { .x = 0, .y = -1000 };


void positionCalibrate() {
  float totalAngle = 0;
  for (int i = 0; i < MPU_CALIBRATION_ITERATIONS; i++) {
    while (!mpu.update()) {}
    totalAngle += mpu.getGyroZ();
  }
  gyro_offset = totalAngle / MPU_CALIBRATION_ITERATIONS;
}

void positionSetup() {
  mpu.setup(MPU_ADDRESS);
}

bool updateGyro() {
  if (mpu.update()) {
    long current_millis = micros();
    gyro_dt = (current_millis - gyro_last_time) / 1000000.0f;
    gyro_last_time = current_millis;
    float gyro_value = (mpu.getGyroZ() - gyro_offset) * 2 * PI / 360;
    rotation += gyro_value * gyro_dt;
    return true;
  }
  return false;
}

float error, last_error, cum_error, rate_error;
float servoPid(float error) {
  cum_error += error * gyro_dt;
  rate_error = (error - last_error) / gyro_dt;

  float XP = DIR_KP * error;
  float XD = DIR_KD * rate_error;
  float XI = DIR_KI * cum_error;

  float output = XP + XD + XI;

  debug_msg("df: %f", gyro_dt);

  last_error = error;

  return output;
}

void positionUpdate(float error) {
  if (updateGyro()) {
    orientation.x = cos(rotation);
    orientation.y = sin(rotation);

    servoAngle(servoPid(error) * (360.0 / (2.0 * PI)) + SERVO_MIDPOINT);
  }
}

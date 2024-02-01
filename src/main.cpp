#include <Arduino.h>
#include <MPU9250.h>

#define MPU_ADDRESS 0x68
#define MPU_CALIBRATION_ITERATIONS 1000

MPU9250 mpu;
HardwareSerial hs(1);

float current_direction = 0.0f;
unsigned long gyro_last_time = 0;
float gyro_offset;

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

void updateGyro() {
  if(mpu.update()) {
    long current_millis = millis();
    long gyro_dt = current_millis - gyro_last_time;
    gyro_last_time = current_millis;
    float gyro_value = mpu.getGyroZ() - gyro_offset;
    current_direction += gyro_value * gyro_dt / 1000;
    Serial.println(current_direction);
  }
}

void setup() {
  // Motor
  hs.begin(112500, SERIAL_8N1, 4, 2);
  motorSpeed(90);
  servoAngle(90);

  Wire.begin();
  mpu.setup(MPU_ADDRESS);
  calibrateImu();

  //Debugging
  Serial.begin(9600);
}

void loop() {
  updateGyro();
}
#include "slave.h"

HardwareSerial hs(1);

void slaveSetup() {
  hs.begin(1000000, SERIAL_8N1, 4, 2);
}

enum SerialCommands {
  SerialMotor,
  SerialServo,
  SerialEncoder,
  SerialBattery
};

void motorSpeed(int speed) {
  hs.write(SerialMotor);
  hs.write((uint8_t *)&speed, sizeof(int));
}

void servoAngle(int angle) {
  hs.write(SerialServo);
  hs.write((uint8_t *)&angle, sizeof(int));
}

void receiveFromSlave() {
  if (hs.available() > 0) {
    int header = hs.read();
    switch (header) {
      // Update position
      case SerialEncoder:
        {
          uint8_t encoders = 0;
          hs.readBytes(&encoders, 1);
          encoders *= MILIMETERS_PER_ENCODER;
          position.x += encoders * orientation.x;
          position.y += encoders * orientation.y;
          break;
        }
      case SerialBattery:
        {
          float voltage = 0;
          hs.readBytes((uint8_t *)&voltage, sizeof(float));
          debug_battery(voltage);
        }
    }
  }
}
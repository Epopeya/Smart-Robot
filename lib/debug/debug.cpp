#include <Arduino.h>
#include <stdint.h>
#include <string.h>

typedef enum { Message, TargetDirection, CurrentDirection, Servo } DebugHeader;

HardwareSerial hs_debug(0);

void debug_init() { hs_debug.begin(112500, SERIAL_8N1, 23, 19); }

void debug_msg(const char *msg) {
  uint8_t msg_len = (uint8_t)strlen(msg);
  hs_debug.write(Message);
  hs_debug.write(msg_len);
  hs_debug.write(msg, msg_len);
}

void debug_target_direction(float angle) {
  hs_debug.write(TargetDirection);
  hs_debug.write((uint8_t *)&angle, sizeof(float));
}

void debug_current_direction(float angle) {
  hs_debug.write(CurrentDirection);
  hs_debug.write((uint8_t *)&angle, sizeof(float));
}

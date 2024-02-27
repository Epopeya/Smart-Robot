#include <Arduino.h>
#include <string.h>
#include <stdint.h>

typedef enum {
	Message,
	TargetDirection,
	CurrentDirection,
	Servo
} DebugHeader;

HardwareSerial hs_debug(2);

void debug_init() {
	hs_debug.begin(112500, SERIAL_8N1, 4, 2);
}

void debug_msg(char *msg) {
	uint8_t msg_len = (uint8_t)strlen(msg);

	hs_debug.write(Message);
	hs_debug.write(msg_len);
	hs_debug.write(msg, msg_len);
}

void debug_target_direction(float angle) {
	hs_debug.write(TargetDirection);
	uint8_t float_array[4];
	memcpy(float_array, &angle, 4);
	hs_debug.write(float_array, 4);
}

void debug_current_direction(float angle) {
	hs_debug.write(TargetDirection);
	uint8_t float_array[4];
	memcpy(float_array, &angle, 4);
	hs_debug.write(float_array, 4);
}


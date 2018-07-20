#include "color_sensor.h"

uint8_t color_sensor_data[5][13];

/*
WHITE: 0
RED: 1
BLUE: 2
GREEN: 3
BLACK: 4
*/

color_code color_from_hex(uint8_t hex) {
	switch (hex) {
		case 0x80:
			return COLOR_CYAN;
		case 0x40:
			return COLOR_BLUE;
		case 0x20:
			return COLOR_GREEN;
		case 0x10:
			return COLOR_BLACK;
		case 0x08:
			return COLOR_WHITE;
		case 0x04:
			return COLOR_PINK;
		case 0x02:
			return COLOR_YELLOW;
		case 0x01:
			return COLOR_RED;
	}
	return COLOR_UNKNOWN;
}

void color_sensor_handler(void) {
}

void color_sensor_init(uint8_t uart_id){
	uart_init(uart_id, 9600, WORD_LEN_8B, STOP_BITS_1, PARITY_NONE);
  uart_recv_callback_register(uart_id, color_sensor_handler);
  uart_receive_start(uart_id, color_sensor_data[uart_id-1], 13);
}


void color_sensor_brightness(uint8_t uart_id, uint8_t brightness) {
	if (brightness > 10) brightness = 10;
	brightness = 10 - brightness;
	uint8_t data[3];
	data[0] = 0xA5;
	data[1] = 0x60 | brightness;
	data[2] = data[0] + data[1];
	write_uart(uart_id, data, 3);
}

void color_sensor_calibrate(uint8_t uart_id) {
	uint8_t data[3] = {0xA5, 0xBB, 0x60};
	write_uart(uart_id, data, 3);
}

void color_sensor_config(uint8_t uart_id, uint8_t save, uint8_t mode) {
	uint8_t data[3];
	data[0] = 0xA5;
	data[1] = (save ? 0x80 : 0x00) | mode;
	data[2] = data[0] + data[1];
	write_uart(uart_id, data, 3);
}

color_data color_sensor_get(uint8_t uart_id) {
	color_data color_detected;
	uint8_t *data = color_sensor_data[uart_id-1];
	if (data[0] != 0x5A || data[1]  != 0x5A) return color_detected; // Byte0 and Byte1 is always 0x45
	uint8_t length = data[3] + 4; // Byte 3 is the length of the following data
	uint32_t sum = 0;
	for (uint8_t i = 0; i < length; i++)
		sum += data[i]; // Sum up all the bytes except the last one
	if ((uint8_t)sum != data[length]) return color_detected; // Compare the sum to the last byte, abandon if don't match
	
	switch (data[2]) {
		case 0x45:
			{
				color_detected.rgb.r = data[4];
				color_detected.rgb.g = data[5];
				color_detected.rgb.b = data[6];
			}break;
		case 0x25:
			{
				color_detected.lcc.lux = (data[4] << 8) | data[5];
				color_detected.lcc.ct = (data[6] << 8) | data[7];
				// Byte 8 is always 0x00
				color_detected.lcc.color = color_from_hex(data[9]);
			}break;
		case 0x15:
			{
				color_detected.raw.r = (data[4] << 8) | data[5];
				color_detected.raw.g = (data[6] << 8) | data[7];
				color_detected.raw.b = (data[8] << 8) | data[9];
				color_detected.raw.c = (data[10] << 8) | data[11];
			}break;
	}
	return color_detected;
}

#include "ultrasonic.h"


uint8_t data_recv[2];

static void uart_callback(void) {
}

void ultrasonic_init(uint8_t uart_id) {
	uart_init(uart_id, 9600, WORD_LEN_8B, STOP_BITS_1, PARITY_NONE);
	uart_recv_callback_register(uart_id, uart_callback);
	uart_receive_start(uart_id, data_recv, 2);
}

void ultrasonic_config_uart(uint8_t uart_id) {
	uint8_t data[1] = {0x01};
	write_uart(uart_id, data, 1);
}

uint16_t ultrasonic_update_distance(uint8_t uart_id) {
	uint8_t data[1] = {0xC1};
	write_uart(uart_id, data, 1);
	return (((uint16_t)data_recv[0]) << 8 ) | data_recv[1];;
}

void ultrasonic_toggle_power(uint8_t uart_id, uint8_t power) {
	uint8_t data[1] = {power ? 0xC4 : 0xC3};
	write_uart(uart_id, data, 1);
}

#ifndef __ULTRASONIC_C__
#define __ULTRASONIC_C__

#include "sys.h"
#include "ultrasonic.h"


void ultrasonic_init(uint8_t uart_id);

void ultrasonic_config_uart(uint8_t uart_id);

uint16_t ultrasonic_update_distance(uint8_t uart_id);
void ultrasonic_toggle_power(uint8_t uart_id, uint8_t power);

#endif

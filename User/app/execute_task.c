#include "execute_task.h"
#include "can_device.h"
#include "uart_device.h"
#include "cmsis_os.h"
#include "calibrate.h"
#include "pid.h"
#include "sys.h"
#include "ultrasonic.h"

void uart_callback(void) {
	write_led_io(LED_IO1, LED_ON);
}


uint16_t distance;
void execute_task(const void* argu)
{
	osDelay(500);
	ultrasonic_init(USER_UART4);
	ultrasonic_config_uart(USER_UART4);
	
  while(1) {
		distance = ultrasonic_update_distance(USER_UART4);
    osDelay(50);
  }
}

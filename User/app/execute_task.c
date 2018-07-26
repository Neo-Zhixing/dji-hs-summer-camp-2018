#include "execute_task.h"
#include "can_device.h"
#include "uart_device.h"
#include "cmsis_os.h"
#include "calibrate.h"
#include "pid.h"
#include "sys.h"

#include "math.h"

#include "color_sensor.h"

color_data my_color_data;

void execute_task(const void* argu)
{
	color_sensor_init(USER_UART3);
	color_sensor_brightness(USER_UART3, 10);
	color_sensor_config(USER_UART3, 0, COLOR_SENSOR_RGB);
  while(1) {
		my_color_data = color_sensor_get(USER_UART3);
		color_rgb color = my_color_data.rgb;
		//If Red is the largest
		uint8_t color_flag = 0;
		// If red is the largest AND the difference between B and G is relatively small
		if(color.r > color.b && color.r > color.g && (fabs((float)color.g - (float)color.b) / (float)color.r) < 0.08) {
			color_flag = 1;//RED
		}
		else if (color.r > color.b && color.r > color.g) {
			color_flag = 4;
		}
		// If green is the largest
		else if (color.g > color.r && color.g > color.b) {
			color_flag = 2;
		}
		// If Blue is the largest
		else if (color.b > color.r && color.b > color.g){
			color_flag = 3;
		}
		for (uint8_t i=1; i<=4; i++) {
			write_led_io(i, i == color_flag ? LED_ON : LED_OFF);
		}
		osDelay(5);
  }
}

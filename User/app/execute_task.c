#include "execute_task.h"
#include "can_device.h"
#include "uart_device.h"
#include "cmsis_os.h"
#include "calibrate.h"
#include "pid.h"
#include "sys.h"

#include "color_sensor.h"



void execute_task(const void* argu)
{
	set_pwm_group_param(PWM_GROUP2, 0xFF);
	
	start_pwm_output(PWM_IO5);
	start_pwm_output(PWM_IO6);
	start_pwm_output(PWM_IO7);

	color_sensor_init(USER_UART3);
	color_sensor_config(USER_UART3, 0, COLOR_SENSOR_RGB);
	color_sensor_brightness(USER_UART3, 2);

	digital_tube_init();
  while(1) {
		uint8_t theKey;
		read_key_io(KEY_IO1, &theKey);
		if (!theKey) {
			switch_display_num(1, 0x60);
			color_sensor_calibrate(USER_UART3);
			osDelay(1000);
			refresh_digital_tube();
		}
		
		color_data color = color_sensor_get(USER_UART3);
		color_rgb rgb;
		rgb.r = 0;
		rgb.b = 0;
		rgb.g = 0;
		if (color.rgb.r + color.rgb.b + color.rgb.g > 220) {
			rgb.r = 50;
			rgb.b = 50;
			rgb.g = 50;
		}
		if (color.rgb.r > color.rgb.b && color.rgb.r > color.rgb.g) { // RED
			rgb.r = 50;
		}
		else if (color.rgb.g > color.rgb.r && color.rgb.g > color.rgb.b) { // GREEN
			rgb.g = 50;
		}
		else if (color.rgb.b > color.rgb.r && color.rgb.b > color.rgb.g) { // BLUE
			rgb.b = 50;
		}
		set_pwm_param(PWM_IO5, 0xFF - rgb.b);
		set_pwm_param(PWM_IO6, 0xFF - rgb.r);
		set_pwm_param(PWM_IO7, 0xFF - rgb.g);
    osDelay(5);
  }
}

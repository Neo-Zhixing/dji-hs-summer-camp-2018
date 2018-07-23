#include "chassis_autonomous.h"
#include "chassis_task.h"
#include "sys.h"
#include "pid.h"
#include "can_device.h"
#include "uart_device.h"
#include "math.h"
#include "keyboard.h"


struct {
	double x, y, w;
	pid_t xPid, yPid, wPid;
} chassis_autonomous_target;

struct {
	double x, y, w;
} chassis_current_position;


imu_t imudata;

void chassis_autonomous_init(void) {
	pid_init(&(chassis_autonomous_target.xPid), 3000, 1000, 0.08f, 0, 0);
	pid_init(&(chassis_autonomous_target.yPid), 3000, 1000, 0.08f, 0, 0);
	pid_init(&(chassis_autonomous_target.wPid), 3000, 500, 5.0f, 0.02, 0.5);
	chassis_autonomous_target.x = 0;
	chassis_autonomous_target.y = 0;
	chassis_autonomous_target.w = 0;
}

void chassis_autonomous_reset(void) {
	// TODO: Figure out a way to reset the motor encoders
}

void update_chassis_current_position() {
	get_imu_data(&imudata);
	chassis_current_position.x = -motor_chassis[0].total_angle - motor_chassis[1].total_angle + motor_chassis[2].total_angle + motor_chassis[3].total_angle;
	chassis_current_position.y = -motor_chassis[0].total_angle +	motor_chassis[1].total_angle + motor_chassis[2].total_angle - motor_chassis[3].total_angle;
	
	chassis_current_position.w = -imudata.angle_z;
}
void limited_accelerate (float *originalValue, float value, float limit) {
	if (value - *originalValue > limit)
		value = *originalValue + limit;
	else if (value - *originalValue < -limit)
		value = *originalValue - limit;
	*originalValue = value;
}
void chassis_autonomous_information_get(void) {
	update_chassis_current_position();
	if (rc.ch3 < 20 && rc.ch3 > -20) {
		chassis_autonomous_target.w = round(chassis_current_position.w / 90) * 90;
		chassis_autonomous_target.wPid.iout *= 0.99f;
		float speed = pid_calc(&(chassis_autonomous_target.wPid), chassis_current_position.w, chassis_autonomous_target.w);
		limited_accelerate(&(chassis.vw), speed, 15);
	}
	else
	  chassis.vw = chassis.vw = rc.ch3 * CHASSIS_RC_MOVE_RATIO_R / RC_MAX_VALUE * MAX_CHASSIS_VR_SPEED + rc.mouse.x * CHASSIS_PC_MOVE_RATIO_R;
	if (rc.ch1 < 20 && rc.ch1 > -20) {
		chassis_autonomous_target.x = round(chassis_current_position.x / 50000) * 50000;
		float speed = pid_calc(&(chassis_autonomous_target.xPid), chassis_current_position.x, chassis_autonomous_target.x);
		limited_accelerate(&(chassis.vx), speed, 80);
	}
	else
		chassis.vx = rc.ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE * MAX_CHASSIS_VX_SPEED + km.vx * CHASSIS_PC_MOVE_RATIO_X;
	if (rc.ch2 < 20 && rc.ch2 > -20) {
		chassis_autonomous_target.y = round(chassis_current_position.y / 50000) * 50000;
		float speed = pid_calc(&(chassis_autonomous_target.yPid), chassis_current_position.y, chassis_autonomous_target.y);
		limited_accelerate(&(chassis.vy), speed, 80);
	}
	else
		chassis.vy = rc.ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE * MAX_CHASSIS_VY_SPEED + km.vy * CHASSIS_PC_MOVE_RATIO_Y;
}

void move(double yDistance, double xDistance) {
	chassis_autonomous_target.y += yDistance;
	chassis_autonomous_target.x += xDistance;
}

void forward(double distance) {
	move(distance, 0);
}

void turn(double angle) {
	chassis_autonomous_target.w += angle;
}

#include "chassis_autonomous.h"
#include "sys.h"
#include "pid.h"
#include "can_device.h"

struct {
	double x, y, w;
	pid_t xPid, yPid, wPid;
} chassis_autonomous_target;

struct {
	double x, y, w;
} chassis_current_position;

void chassis_autonomous_init(void) {
	pid_init(&(chassis_autonomous_target.xPid), 3000, 1000, 0.08f, 0, 0);
	pid_init(&(chassis_autonomous_target.yPid), 3000, 1000, 0.08f, 0, 0);
	pid_init(&(chassis_autonomous_target.wPid), 3000, 1000, 0.08f, 0, 0);
	chassis_autonomous_target.x = 0;
	chassis_autonomous_target.y = 0;
	chassis_autonomous_target.w = 0;
}


void update_chassis_current_position() {
	chassis_current_position.y = -moto_chassis[0].total_angle +	moto_chassis[1].total_angle + moto_chassis[2].total_angle - moto_chassis[3].total_angle;
	chassis_current_position.w = moto_chassis[0].total_angle + moto_chassis[1].total_angle + moto_chassis[2].total_angle + moto_chassis[3].total_angle;
	chassis_current_position.x = -moto_chassis[0].total_angle - moto_chassis[1].total_angle + moto_chassis[2].total_angle + moto_chassis[3].total_angle;
}
void chassis_autonomous_information_get(void) {
	update_chassis_current_position();
	chassis.vy = pid_calc(&(chassis_autonomous_target.yPid), chassis_current_position.y, chassis_autonomous_target.y);
	chassis.vx = pid_calc(&(chassis_autonomous_target.xPid), chassis_current_position.x, chassis_autonomous_target.x);
	chassis.vw = pid_calc(&(chassis_autonomous_target.wPid), chassis_current_position.w, chassis_autonomous_target.w);
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

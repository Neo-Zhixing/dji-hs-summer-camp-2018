#include "elevator_storage.h"


typedef enum {
	BLOCK_EMPTY,
	BLOCK_RED,
	BLOCK_GREEN,
	BLOCK_YELLOW,
	BLOCK_BLUE,
} block_t;


const uint8_t block_storage_x_slot_number = 2;
const uint8_t block_storage_y_slot_number = 3;
block_t block_storage[block_storage_x_slot_number][block_storage_y_slot_number];

typedef struct {
	uint8_t x, y;
} coordinate_t;

/*
	-------------
2   |     |     |
	|     |     |
	-------------
1   |     |     |
	|     |     |
	-------------
0   |     |     |
	|     |     |
y   -------------
  x    0     1
*/

block_t block_storage_get(coordinate_t coordinate) {
	return block_storage[coordinate.x][coordinate.y];
}

void block_storage_set(coordinate_t coordinate, block_t block) {
	block_storage[coordinate.x][coordinate.y] = block;
}

static coordinate_t block_storage_cooordinates;
coordinate_t * block_storage_coordinate_for_block(block_t block) {
	for (uint8_t x=0; x < block_storage_x_slot_number; x++)
		for (uint8_t y=0; y < block_storage_y_slot_number; y++) {
		if (block_storage[x][y] == block) {
			block_storage_cooordinates.x = x;
			block_storage_cooordinates.y = y;
			return &block_storage_cooordinates;
		}
	}
	return NULL;
}


typedef enum {
	STORAGE_BLOCKER_SWING_LEFT = 400,
	STORAGE_BLOCKER_SWING_RIGHT = 800,
	STORAGE_BLOCKER_SWING_MID = 600,
	STORAGE_BLOCKER_SWING_OPEN_LEFT = 1500,
	STORAGE_BLOCKER_SWING_OPEN_RIGHT = 0,
} storage_blocker_swing_state;


void storage_blocker_init() {
	set_pwm_group_param(PWM_GROUP1, 2000);
	start_pwm_output(PWM_IO1);
	set_pwm_param(PWM_IO1, STORAGE_BLOCKER_SWING_MID);
}

void storage_blocker_swing(storage_blocker_swing_state state) {
	set_pwm_param(PWM_IO1, state);
}


#include "pid.h"
#include "can_device.h"
struct {
	float x, y;
} elevator_target_coordinates;

static struct {
	int16_t elevator_left, elevator_right, claw_move;
} initial_coordinates;
static struct {
	pid_t left_balance, right_balance;
	pid_t elevator_pos, claw_pos;
	pid_t left_speed, right_speed, claw_speed;
} elevator_pids;

#define ELEVATOR_LIMIT_SWITCH_LEFT     5
#define ELEVATOR_LIMIT_SWITCH_RIGHT    6
#define CLAW_MOVE_LIMIT_SWITCH         7
void elevator_init() {
	set_digital_io_dir(ELEVATOR_LIMIT_SWITCH_LEFT, IO_INPUT);
	set_digital_io_dir(ELEVATOR_LIMIT_SWITCH_RIGHT, IO_INPUT);
	set_digital_io_dir(CLAW_MOVE_LIMIT_SWITCH, IO_INPUT);
	pid_init(&elevator_pids.left_balance, 7000, 1000, 1, 0, 0);
	pid_init(&elevator_pids.right_balance, 7000, 1000, 1, 0, 0);
	pid_init(&elevator_pids.elevator_pos, 7000, 1000, 1, 0, 0);
	pid_init(&elevator_pids.claw_pos, 7000, 1000, 1, 0, 0);
	pid_init(&elevator_pids.left_speed, 7000, 1000, 1, 0, 0);
	pid_init(&elevator_pids.right_speed, 7000, 1000, 1, 0, 0);
	pid_init(&elevator_pids.claw_speed, 7000, 1000, 1, 0, 0);
	
	
	uint8_t elevator_switch_left = 0;
	uint8_t elevator_switch_right = 0;
	uint8_t claw_switch = 0;
	while(!elevator_switch_left && !elevator_switch_right && !claw_switch) {
		read_digital_io(ELEVATOR_LIMIT_SWITCH_LEFT, &elevator_switch_left);
		read_digital_io(ELEVATOR_LIMIT_SWITCH_RIGHT, &elevator_switch_right);
		read_digital_io(CLAW_MOVE_LIMIT_SWITCH, &claw_switch);
		send_elevator_motor_current(
			elevator_switch_left * GM3510_MAX_CURRENT,
			elevator_switch_left * GM3510_MAX_CURRENT,
			claw_switch * GM3510_MAX_CURRENT
		);
	}
	initial_coordinates.elevator_left = motor_elevator_left.total_angle;
	initial_coordinates.elevator_right = motor_elevator_right.total_angle;
	initial_coordinates.claw_move = motor_claw_move.total_angle;
}

void elevator_update() {
	int32_t left_pos = motor_elevator_left.total_angle - initial_coordinates.elevator_left;
	int32_t right_pos = motor_elevator_right.total_angle - initial_coordinates.elevator_right;
	float current_elevator_pos = (float)(left_pos + right_pos) / 2.0f;
	
	float left_balance_speed = pid_calc(&(elevator_pids.left_balance), left_pos, current_elevator_pos);
	float right_balance_speed = pid_calc(&(elevator_pids.right_balance), right_pos, current_elevator_pos);
	
	float elevation_speed = pid_calc(&(elevator_pids.elevator_pos), current_elevator_pos, elevator_target_coordinates.y);
	
	float left_power = pid_calc(&(elevator_pids.left_speed), motor_elevator_left.speed_rpm, left_balance_speed + elevation_speed);
	float right_power = pid_calc(&(elevator_pids.right_speed), motor_elevator_right.speed_rpm, right_balance_speed + elevation_speed);
	
	int32_t claw_pos = motor_claw_move.total_angle - initial_coordinates.claw_move;
	float claw_move_speed = pid_calc(&(elevator_pids.claw_pos), claw_pos, elevator_target_coordinates.x);
	float claw_move_power = pid_calc(&(elevator_pids.claw_speed), motor_claw_move.speed_rpm, claw_move_speed);
	
	send_elevator_motor_current((int16_t)left_power, (int16_t)right_power, (int16_t) claw_move_power);
}



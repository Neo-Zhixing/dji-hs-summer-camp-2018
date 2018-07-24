#ifndef __ELEVATOR_STORAGE_H__
#define __ELEVATOR_STORAGE_H__

#include "sys.h"
typedef struct {
	float x, y;
} elevator_target_coordinates_t;
extern elevator_target_coordinates_t elevator_target_coordinates;
void elevator_init(void);

void elevator_update(void);


typedef enum {
	STORAGE_BLOCKER_SWING_LEFT = 1300,
	STORAGE_BLOCKER_SWING_RIGHT = 1600,
	STORAGE_BLOCKER_SWING_MID = 1450,
	STORAGE_BLOCKER_SWING_OPEN_LEFT = 950,
	STORAGE_BLOCKER_SWING_OPEN_RIGHT = 1950,
} storage_blocker_swing_state;
void storage_blocker_swing(storage_blocker_swing_state state);

void claw_horizontal_set(uint8_t closed);
void claw_vertical_set(uint8_t closed);

#endif

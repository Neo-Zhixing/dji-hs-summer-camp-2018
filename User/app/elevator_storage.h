#ifndef __ELEVATOR_STORAGE_H__
#define __ELEVATOR_STORAGE_H__

#include "sys.h"
typedef struct {
	float x, y;
} elevator_target_coordinates_t;
extern elevator_target_coordinates_t elevator_target_coordinates;
void elevator_init(void);

void elevator_update(void);

#endif

#include "elevator_task.h"
#include "cmsis_os.h"

#include "elevator_storage.h"

void elevator_task(const void* argu)
{
	elevator_init();
  while(1) {
		elevator_update();
		osDelay(5);
  }
}

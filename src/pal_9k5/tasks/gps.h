#ifndef GPS_H
#define GPS_H

#include "status.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#define GPS_POLL_PERIOD_MS (50)

Status gps_init();

void task_gps(TaskHandle_t* handle_ptr);

#endif  // GPS_H

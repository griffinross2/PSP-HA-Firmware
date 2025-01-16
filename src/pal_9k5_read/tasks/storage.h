#ifndef STORAGE_H
#define STORAGE_H

#include "max_m10s.h"
#include "status.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#define GPS_QUEUE_LENGTH (40UL)
#define GPS_QUEUE_ITEM_SIZE (sizeof(GPS_Fix_TypeDef))

#define STORAGE_INTERVAL_MS (1000)

Status storage_init();

Status storage_queue_gps(const GPS_Fix_TypeDef* gps_frame);

void task_storage(TaskHandle_t* handle_ptr);

void storage_dump();

Status storage_write_log(const char* log, size_t size);

#endif  // STORAGE_H

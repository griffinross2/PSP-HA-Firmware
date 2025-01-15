#include <stdio.h>

#include "board.h"
#include "gpio/gpio.h"
#include "i2c/i2c.h"
#include "max_m10s.h"
#include "storage.h"
#include "timer.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"

#ifdef HWIL_TEST
#include "hwil/hwil.h"
#endif

/*********************/
/* PERIPHERAL CONFIG */
/*********************/

// GPS
static I2cDevice s_gps_conf = {
    .address = 0x42,
    .clk = I2C_SPEED_STANDARD,
    .periph = P_I2C2,
    .scl = PIN_PB10,
    .sda = PIN_PB11,
};

/********************/
/* STATIC VARIABLES */
/********************/
static TaskHandle_t* s_handle_ptr = NULL;

/*****************/
/* API FUNCTIONS */
/*****************/
Status gps_init() {
    ASSERT_OK(max_m10s_init(&s_gps_conf), "failed to init GPS\n");

    return STATUS_OK;
}

void task_gps(TaskHandle_t* handle_ptr) {
    s_handle_ptr = handle_ptr;
    TickType_t last_iteration_start_tick = xTaskGetTickCount();

    while (1) {
        GPS_Fix_TypeDef fix;
        if (max_m10s_poll_fix(&s_gps_conf, &fix) == STATUS_OK) {
#ifdef HWIL_TEST
            // If we're doing a HWIL test, overwrite the actual GPS fix
            // with one from the test data based on the current timestamp
            GPS_Fix_TypeDef hwil_fix;
            if (get_hwil_gps_fix(&hwil_fix) == STATUS_OK) {
                fix = hwil_fix;
            }
#endif  // HWIL_TEST

            // Set LED to indicate GPS fix
            gpio_write(PIN_BLUE, (fix.fix_valid && !fix.invalid_llh)
                                     ? GPIO_HIGH
                                     : GPIO_LOW);

            // Storage
            storage_queue_gps(&fix);
        } else {
            // Set LED low
            gpio_write(PIN_BLUE, GPIO_LOW);
        }

        // Delay until next time
        vTaskDelayUntil(&last_iteration_start_tick,
                        pdMS_TO_TICKS(GPS_POLL_PERIOD_MS));
    }
}

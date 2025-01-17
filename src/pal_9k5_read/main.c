#include "main.h"

#include "board.h"
#include "clocks.h"
#include "data.h"
#include "gpio/gpio.h"
#include "malloc.h"
#include "pyros.h"
#include "rtc/rtc.h"
#include "status.h"
#include "stdio.h"
#include "stm32h7xx.h"
#include "tasks/gps.h"
#include "tasks/storage.h"
#include "timer.h"
#include "usb.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

/*****************/
/* HELPER MACROS */
/*****************/

#define PANIC(msg, ...)                                       \
    do {                                                      \
        gpio_write(PIN_RED, GPIO_HIGH);                       \
        DELAY(1000);                                          \
        gpio_write(PIN_RED, GPIO_LOW);                        \
        DELAY(1000);                                          \
        /* Print might itself cause a fault, so do it last */ \
        PAL_LOGE("PANIC @ %s:%d: ", __FILE__, __LINE__);      \
        PAL_LOGE(msg, ##__VA_ARGS__);                         \
    } while (1)

#define TASK_CREATE(func, pri, ss)                                    \
    do {                                                              \
        static TaskHandle_t s_##func##_handle;                        \
        if (xTaskCreate((void *)func,             /* Task function */ \
                        #func,                    /* Task name */     \
                        ss,                       /* Stack size */    \
                        &s_##func##_handle,       /* Parameters */    \
                        tskIDLE_PRIORITY + (pri), /* Priority */      \
                        &s_##func##_handle        /* Task handle */   \
                        ) != pdPASS) {                                \
            PANIC("failed to launch task %s\n", #func);               \
        }                                                             \
    } while (0)

/******************/
/* MAIN FUNCTIONS */
/******************/

/**
 * Entry point function.
 */
int main(void) {
    // Perform critical bare-metal initialization
    HAL_Init();
    SystemClock_Config();
    init_timers();
    DELAY(2);

    // Light all LEDs to indicate initialization
    gpio_write(PIN_RED, GPIO_HIGH);
    gpio_write(PIN_YELLOW, GPIO_HIGH);
    gpio_write(PIN_GREEN, GPIO_HIGH);
    gpio_write(PIN_BLUE, GPIO_HIGH);

    // Pullup gps reset
    gpio_write(PIN_GPS_RST, GPIO_HIGH);

    // Launch FreeRTOS kernel and init
    uint32_t init_error = 0;  // Set if error occurs during initialization

    init_error |= (EXPECT_OK(storage_init(), "init storage") != STATUS_OK) << 0;
    init_error |= (EXPECT_OK(usb_init(), "init usb") != STATUS_OK) << 1;

    TASK_CREATE(task_usb, +2, 8192);
    TASK_CREATE(storage_dump, +1, 32768);

    PAL_LOGI("Starting scheduler\n");

    vTaskStartScheduler();

    while (1) {
    }
}

/**********************/
/* EXCEPTION HANDLERS */
/**********************/

void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   signed char *pcTaskName) {
    PANIC("stack overflow in task '%s'", pcTaskName);
}

extern void xPortSysTickHandler(void);

void SysTick_Handler(void) {
    /* Clear overflow flag */
    SysTick->CTRL;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        /* Call tick handler */
        xPortSysTickHandler();
    }
}

void Error_Handler(void) { PANIC("unexpected exception\n"); }

void NMI_Handler(void) { PANIC("unexpected exception\n"); }

void HardFault_Handler(void) { PANIC("unexpected exception\n"); }

void MemManage_Handler(void) { PANIC("unexpected exception\n"); }

void BusFault_Handler(void) { PANIC("unexpected exception\n"); }

void UsageFault_Handler(void) { PANIC("unexpected exception\n"); }

void DebugMon_Handler(void) { PANIC("unexpected exception\n"); }

#include "storage.h"

#include <sys/types.h>

#include "coding/convolutional.h"
#include "fifos.h"
#include "gpio/gpio.h"
#include "main.h"
#include "mt29f4g.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "timer.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"

static uint32_t s_gps_overflows;
static QueueHandle_t s_gps_queue;

static int nand_pos_block = 0;
static int nand_pos_page = 0;

Status find_log_end(int* block, int* page);

Status storage_init() {
    // Create queue
    s_gps_queue = xQueueCreate(GPS_QUEUE_LENGTH, GPS_QUEUE_ITEM_SIZE);
    configASSERT(s_gps_queue);

    // Init NAND
    if (mt29f4g_init() != STATUS_OK) {
        PAL_LOGE("Failed to init NAND\n");
        return STATUS_ERROR;
    }

    // Find the end of the log
    if (find_log_end(&nand_pos_block, &nand_pos_page) != STATUS_OK) {
        PAL_LOGE("Failed to find log end\n");
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

Status storage_queue_gps(const GPS_Fix_TypeDef* gps_frame) {
    if (xQueueSend(s_gps_queue, gps_frame, 0) != pdPASS) {
        s_gps_overflows += 1;
        return STATUS_BUSY;
    }

    return STATUS_OK;
}

void task_storage(TaskHandle_t* handle_ptr) {
    // Initialize LEDs
    gpio_write(PIN_YELLOW, GPIO_LOW);
    gpio_write(PIN_GREEN, GPIO_LOW);

    TickType_t last_iteration_start_tick = xTaskGetTickCount();

    while (1) {
        // Check if storage is full
        if (nand_pos_block == MT29F4G_BLOCK_COUNT - 1 &&
            nand_pos_page == MT29F4G_PAGE_PER_BLOCK - 1) {
            PAL_LOGE("NAND storage full\n");
            gpio_write(PIN_RED, GPIO_HIGH);
            goto store_task_end;
        }

        // Empty bursts of 16 frames from the queue
        while (uxQueueMessagesWaiting(s_gps_queue) >= 16) {
            // Set disk activity warning LED
            gpio_write(PIN_YELLOW, GPIO_HIGH);

            Status flush_status = STATUS_OK;
            uint8_t whole_page[4096];

            for (int i = 0; i < 16; i++) {
                // Receive from the queue and store it
                GPS_Fix_TypeDef gps_frame;
                xQueueReceive(s_gps_queue, &gps_frame, 0);

                // Encode and store frame
                GPSConvolutionalFrame conv_frame;
                gps_frame_to_convolutional_frame(&gps_frame, &conv_frame);
                GPSStorageFrame storage_frame;
                gps_convolutional_frame_to_storage_frame(&conv_frame,
                                                         storage_frame);

                // Write to NAND buffer
                memcpy(whole_page + i * 256, storage_frame,
                       sizeof(GPSStorageFrame));
            }

            // If this is a new block, first erase it
            if (nand_pos_page == 0) {
                if (mt29f4g_erase_blocks(nand_pos_block, nand_pos_block) !=
                    STATUS_OK) {
                    PAL_LOGE("Failed to erase block %d\n", nand_pos_block);
                    flush_status = STATUS_ERROR;
                    goto store_end;
                }
            }

            // Write the whole page
            if (mt29f4g_write_pages(
                    whole_page,
                    nand_pos_block * MT29F4G_PAGE_PER_BLOCK + nand_pos_page,
                    1) != STATUS_OK) {
                PAL_LOGE(
                    "Failed to write page %d\n",
                    nand_pos_block * MT29F4G_PAGE_PER_BLOCK + nand_pos_page);
                flush_status = STATUS_ERROR;
            }

        store_end:
            // Increment block position
            nand_pos_block = (nand_pos_block + 1) % MT29F4G_PAGE_PER_BLOCK;

            // Once we come back to the first block, start writing that next
            // pages
            if (nand_pos_block == 0) {
                nand_pos_page = (nand_pos_page + 1) % MT29F4G_PAGE_PER_BLOCK;
            }

            gpio_write(PIN_GREEN, flush_status == STATUS_OK);
            gpio_write(PIN_YELLOW, GPIO_LOW);
        }

        if (s_gps_overflows) {
            PAL_LOGW("%lu overflows in gps queue\n", s_gps_overflows);
            s_gps_overflows = 0;
        }

    store_task_end:
        // Delay until next time
        vTaskDelayUntil(&last_iteration_start_tick, STORAGE_INTERVAL_MS);
    }
}

Status find_log_end(int* block, int* page) {
    // Nested binary search to the end of the log
    int block_start = 0;
    int page_start = 0;
    int block_end = MT29F4G_BLOCK_COUNT - 1;
    int page_end = MT29F4G_PAGE_PER_BLOCK - 1;
    *block = block_end / 2;
    *page = page_end / 2;

    while (true) {
        // Read the sync words
        int sync_pos = 0;
        uint8_t sync[4];
    read_sync:
        if (mt29f4g_read_within_page(
                sync, block_start * MT29F4G_PAGE_PER_BLOCK + page_start,
                sync_pos, 4) != STATUS_OK) {
            PAL_LOGE("Failed to read page %d\n",
                     block_start * MT29F4G_PAGE_PER_BLOCK + page_start);
            return STATUS_ERROR;
        } else {
            // Check sync word
            if (memcmp(sync, GPS_STORAGE_FRAME_SYNC, 4) == 0) {
                // Checks out, continue the search
                // Since we found a log, we know the start is here or higher
                page_start = *page;

                // If we narrowed down the page, work on the blocks now
                if (page_start == page_end) {
                    block_start = *block;
                }

                // If we have narrowed down both, we are done
                if (block_start == block_end && page_start == page_end) {
                    return STATUS_OK;
                }

                // Else, set the new position
                *block = (block_start + block_end) / 2;
                *page = (page_start + page_end) / 2;

                // Start over
                continue;
            } else {
                sync_pos += 256;
                if (sync_pos < 4096) {
                    // Going back gives more chances in case of bit errors in
                    // the sync word
                    goto read_sync;
                }
            }
        }
    }

    // We didn't find any log, so we need to move the end down
    page_end = *page;

    // If we narrowed down the page, work on the blocks now
    if (page_start == page_end) {
        block_start = *block;
    }

    // If we have narrowed down both, we are done
    if (block_start == block_end && page_start == page_end) {
        return STATUS_OK;
    }

    // Else, set the new position and start over
    *block = (block_start + block_end) / 2;
    *page = (page_start + page_end) / 2;
}
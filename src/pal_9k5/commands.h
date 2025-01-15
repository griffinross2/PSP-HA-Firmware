#ifndef COMMANDS_H
#define COMMANDS_H

#include "backup/backup.h"
#include "board_config.h"
#include "mt29f4g.h"
#include "pspcom.h"
#include "regex.h"
#include "rtc/rtc.h"
#include "status.h"
#include "timer.h"

// Help command
char regex_help[] = "^help[\n]*$";
// clang-format off
void cmd_help(char *str) {
    printf(
        "Commands:\n"
        "  help                                  this command\n"
        "  erase_flash_chip                      full block-level flash erase\n"
        "  get_firmware_spec                     prints the firmware spec\n");
}
// clang-format on

// Config erase chip
char regex_erase_flash_chip[] = "^erase_flash_chip[\n]*$";
void cmd_erase_flash_chip(char *str) {
    if (mt29f4g_erase_chip() == STATUS_OK) {
        PAL_LOGI("Successfully erased flash chip\n");
    } else {
        PAL_LOGE("Failed to erase flash chip\n");
    }
}

// Print firmware spec command
char regex_get_firmware_spec[] = "^get_firmware_spec[\n]*$";
void cmd_get_firmware_spec(char *str) {
    printf("Firmware spec: %s\n", FIRMWARE_SPECIFIER);
}

#endif  // COMMANDS_H
#include "math.h"
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

static const uint8_t GPS_STORAGE_FRAME_SYNC[] = {0xAC, 0x20, 0x5C, 0xED};

#define nstates (64)
#define ninput (200 * 8)
#define rate (2)

static uint16_t path_metric[nstates][ninput / rate + 1];

typedef struct {
    // UTC Time
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    int32_t nano;

    // Validity flags
    uint8_t date_valid;
    uint8_t time_valid;
    uint8_t time_resolved;

    /*
    0 = no fix, 1 = dead-reckoning only, 2 = 2D fix,
    3 = 3D fix, 4 = GNSS + dead-reckoning, 5 = time only
    */
    uint8_t fix_type;

    // Fix validity flags
    uint8_t fix_valid;
    uint8_t diff_used;
    /*
    0 = PSM not active, 1 = enabled, 2 = acquisition,
    3 = tracking, 4 = power optimized tracking, 5 = inactive
    */
    uint8_t psm_state;
    uint8_t hdg_veh_valid;
    /*
    0 = no carrier phase range soln, 1 = floating ambiguities,
    2 = fixed ambiguities
    */
    uint8_t carrier_phase;

    // Navigation info
    uint8_t num_sats;
    float lon;                // deg
    float lat;                // deg
    float height;             // m
    float height_msl;         // m
    float accuracy_horiz;     // m
    float accuracy_vertical;  // m
    float vel_north;          // m/s
    float vel_east;           // m/s
    float vel_down;           // m/s
    float ground_speed;       // m/s
    float hdg;                // deg
    float accuracy_speed;     // m/s
    float accuracy_hdg;       // deg

    // Additional flags
    uint8_t invalid_llh;  // Invalid lat, lon, height

} GPS_Fix_TypeDef;

int gps_storage_frame_to_fix(const uint8_t* storage_frame,
                             GPS_Fix_TypeDef* gps_frame);
uint32_t hamming_dist2(uint32_t val1, uint32_t val2);
void viterbi_decode(uint8_t* data, uint8_t* result);
uint8_t get_conv_out(uint8_t state, uint8_t input);

uint32_t hamming_dist2(uint32_t val1, uint32_t val2) {
    uint32_t dist = 0;
    for (int i = 0; i < 2; i++) {
        dist += (val1 & 0x1) ^ (val2 & 0x1);
        val1 >>= 1;
        val2 >>= 1;
    }
    return dist;
}

int gps_storage_frame_to_fix(const uint8_t* storage_frame,
                             GPS_Fix_TypeDef* gps_frame) {
    // Deinterleave (2B x 100B)
    uint8_t deinterleaved_data[200];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 100; j++) {
            deinterleaved_data[i + j * 2] = storage_frame[i * 100 + j];
        }
    }

    // Expand the data out bit by bit
    uint8_t encoded_data[200 * 8];
    for (int i = 0; i < 200; i++) {
        for (int j = 0; j < 8; j++) {
            encoded_data[i * 8 + j] = (deinterleaved_data[i] >> j) & 0x1;
        }
    }

    // Decode the data
    uint8_t decoded_data[100 * 8];
    viterbi_decode(encoded_data, decoded_data);

    // Pack the bits back into bytes
    uint8_t result[100];
    for (int i = 0; i < 100; i++) {
        result[i] = 0;
        for (int j = 0; j < 8; j++) {
            result[i] |= decoded_data[i * 8 + j] << j;
        }
    }

    // Check the tail bytes
    for (int i = 0; i < 100 - sizeof(GPS_Fix_TypeDef); i++) {
        if (result[sizeof(GPS_Fix_TypeDef) + i] != 0) {
            return -1;
        }
    }

    // Copy the data into the GPS frame
    memcpy(gps_frame, result, sizeof(GPS_Fix_TypeDef));

    return 0;
}

// Just copying the galileo coding algorithm lol
uint8_t get_conv_out(uint8_t state, uint8_t input) {
    static uint8_t G1 = 0b1111001;
    static uint8_t G2 = 0b1011011;

    uint8_t conv_in = state | (input << 6);

    // res[1:0] = output bits
    uint8_t result = 0;
    result |= (__builtin_parity(conv_in & G1) << 1);
    result |= (__builtin_parity(conv_in & G2) == 0);

    return result;
}

// Just copying the galileo coding algorithm lol
uint8_t get_conv_out_2(uint8_t* state, uint8_t input) {
    static uint8_t G1 = 0b1111001;
    static uint8_t G2 = 0b1011011;

    uint8_t conv_in = *state | (input << 6);

    // res[1:0] = output bits
    uint8_t result = 0;
    result |= (__builtin_parity(conv_in & G1) << 1);
    result |= (__builtin_parity(conv_in & G2) == 0);

    *state = conv_in >> 1;

    return result;
}

void viterbi_decode(uint8_t* data, uint8_t* result) {
    // Setup initial state
    memset(path_metric, 0xFF, sizeof(path_metric));
    path_metric[0][0] = 0;

    // Create trellis
    for (int j = 0; j < ninput / rate; j++) {
        for (int i = 0; i < nstates; i++) {
            // skip unaccessible states
            if (path_metric[i][j] == 0xFFFF) continue;

            // input bit 0
            uint8_t next_state = (i >> 1);
            uint32_t conv_out = get_conv_out(i, 0);
            uint8_t rcvd = (data[j * rate] << 1) | data[j * rate + 1];
            uint32_t dist = hamming_dist2(rcvd, conv_out);
            if (path_metric[next_state][j + 1] > path_metric[i][j] + dist) {
                path_metric[next_state][j + 1] = path_metric[i][j] + dist;
            }
            // input bit 1
            next_state |= 0b100000;
            conv_out = get_conv_out(i, 1);
            dist = hamming_dist2(rcvd, conv_out);
            if (path_metric[next_state][j + 1] > path_metric[i][j] + dist) {
                path_metric[next_state][j + 1] = path_metric[i][j] + dist;
            }
        }
    }

    // Find best ending
    int best_state = 0;
    uint32_t best_metric = 0xFFFF;
    for (int i = 0; i < nstates; i++) {
        if (path_metric[i][ninput / rate] < best_metric) {
            best_metric = path_metric[i][ninput / rate];
            best_state = i;
        }
    }

    // Traceback through best path
    for (int i = ninput / rate - 1; i >= 0; i--) {
        result[i] = (best_state >> 5) & 1;

        // Zero path
        uint8_t previous_state = (best_state << 1) & 0b111111;
        best_metric = path_metric[previous_state][i];
        best_state = previous_state;

        // One path
        previous_state = (best_state | 1);
        if (path_metric[previous_state][i] < best_metric) {
            best_metric = path_metric[previous_state][i];
            best_state = previous_state;
        }
    }
}

void print_frame(GPS_Fix_TypeDef *fix);

int main() {
    FILE* fp = fopen("D:/github_repos/PSP-HA-Firmware-Griffin/putty.log", "rb");
    if (fp == NULL) {
        printf("Failed to open file\n");
        return 1;
    }

    printf(
        "year, month, day, hour, min, sec, nano, date_valid, time_valid, "
        "time_resolved, fix_type, fix_valid, diff_used, psm_state, "
        "hdg_veh_valid, carrier_phase, num_sats, lon, lat, height, "
        "height_msl, accuracy_horiz, accuracy_vertical, vel_north, "
        "vel_east, vel_down, ground_speed, hdg, accuracy_speed, "
        "accuracy_hdg, invalid_llh\n");

    while (1) {
        uint8_t header_hist[4] = {0, 0, 0, 0};
        while (memcmp(header_hist, GPS_STORAGE_FRAME_SYNC, 4) != 0) {
            memmove(header_hist, header_hist + 1, 3);
            if (fread(header_hist + 3, 1, 1, fp) != 1) {
                // EOF
                return 0;
            }
        }

        GPS_Fix_TypeDef fix;
        uint8_t data[200];
        if (fread(data, 1, 200, fp) < 200) {
            // EOF
            return 0;
        }

        if (gps_storage_frame_to_fix(data, &fix) == 0) {
            printf(
                "%04d,%02d,%02d,%02d,%02d,%02d,%d,"  // Time
                "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"     // Flags
                "%.10f,%.10f,%.4f,%.4f,"             // Position
                "%.4f,%.4f,"                         // Accuracy
                "%.4f,%.4f,%.4f,%.4f,"               // Speeds
                "%f,%f,%f,%d\n",                     // Misc
                fix.year, fix.month, fix.day, fix.hour, fix.min, fix.sec,
                fix.nano, fix.date_valid, fix.time_valid, fix.time_resolved,
                fix.fix_type, fix.fix_valid, fix.diff_used, fix.psm_state,
                fix.hdg_veh_valid, fix.carrier_phase, fix.num_sats, fix.lon,
                fix.lat, fix.height, fix.height_msl, fix.accuracy_horiz,
                fix.accuracy_vertical, fix.vel_north, fix.vel_east,
                fix.vel_down, fix.ground_speed, fix.hdg, fix.accuracy_speed,
                fix.accuracy_hdg, fix.invalid_llh);
        }
    }
}
#include "coding/convolutional.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "time.h"

void print_frame(GPS_Fix_TypeDef* fix);
void encode_frame(GPS_Fix_TypeDef* fix, GPSStorageFrame storage_frame);
int check_decode_frame(GPS_Fix_TypeDef* orig_frame,
                       GPSStorageFrame storage_frame);
int generate_test_random();
int generate_test_random_noise(double BER);

int main() {
    srand(time(NULL));

    int pass_count = 0;
    int no_data_count = 0;
    int bad_data_count = 0;
    const int num_tests = 5 * 60 * 60 * 24;  // 24 hours of 5Hz data
    const double BER = 0.001;                // Very bad BER

    for (int i = 0; i < num_tests; i++) {
        int res = generate_test_random_noise(BER);
        switch (res) {
            case 0:
                pass_count += 1;
                break;
            case -1:
                no_data_count += 1;
                break;
            case -2:
                bad_data_count += 1;
                break;
        }
    }

    printf("\nTest with Bit Error Rate: %0.6f%%\n", BER * 100.0);

    printf("\n%d/%d passed\n", pass_count, num_tests);
    printf("Pass Rate: %0.06f\n", (double)pass_count / num_tests);

    printf("\n%d/%d were not decoded\n", no_data_count, num_tests);
    printf("Loss Rate: %0.06f\n", (double)no_data_count / num_tests);

    printf("\n%d/%d returned bad data\n", bad_data_count, num_tests);
    printf("False Data Rate: %0.06f\n", (double)bad_data_count / num_tests);

    return 0;
}

void print_frame(GPS_Fix_TypeDef* fix) {
    printf("GPS FIX\n");
    printf("Date, Time: %02d/%02d/%02d %02d:%02d:%02d.%09d\n", fix->day,
           fix->month, fix->year, fix->hour, fix->min, fix->sec, fix->nano);
    printf("Date Valid, Time Valid, Time Resolved: %c, %c, %c\n",
           fix->date_valid ? 'Y' : 'N', fix->time_valid ? 'Y' : 'N',
           fix->time_resolved ? 'Y' : 'N');
    printf("Fix Type: %d\n", fix->fix_type);
    printf("Fix Valid, Diff Used: %c, %c\n", fix->fix_valid ? 'Y' : 'N',
           fix->diff_used ? 'Y' : 'N');
    printf("PSM State: %d\n", fix->psm_state);
    printf("HDG Veh Valid: %c\n", fix->hdg_veh_valid ? 'Y' : 'N');
    printf("Carrier Phase: %d\n", fix->carrier_phase);
    printf("Num Sats: %d\n", fix->num_sats);
    printf("Latitude: %f\n", fix->lat);
    printf("Longitude: %f\n", fix->lon);
    printf("Altitude: %f\n", fix->height);
    printf("Altitude MSL: %f\n", fix->height_msl);
    printf("Horizontal Accuracy: %f\n", fix->accuracy_horiz);
    printf("Vertical Accuracy: %f\n", fix->accuracy_vertical);
    printf("Vel N, E, D: %f, %f, %f\n", fix->vel_north, fix->vel_east,
           fix->vel_down);
    printf("Ground Speed: %f\n", fix->ground_speed);
    printf("HDG: %f\n", fix->hdg);
    printf("Speed Accuracy: %f\n", fix->accuracy_speed);
    printf("HDG Accuracy: %f\n", fix->accuracy_hdg);
    printf("Invalid LLH: %c\n", fix->invalid_llh ? 'Y' : 'N');
}

void encode_frame(GPS_Fix_TypeDef* fix, GPSStorageFrame storage_frame) {
    GPSConvolutionalFrame conv_frame;
    gps_frame_to_convolutional_frame(fix, &conv_frame);
    gps_convolutional_frame_to_storage_frame(&conv_frame, storage_frame);
}

int check_decode_frame(GPS_Fix_TypeDef* orig_frame,
                       GPSStorageFrame storage_frame) {
    GPS_Fix_TypeDef gps_frame_out;
    if (gps_storage_frame_to_fix(storage_frame, &gps_frame_out)) {
        // printf("Failed to decode GPS frame\n");
        return -1;
    }

    if (memcmp(&gps_frame_out, orig_frame, sizeof(GPS_Fix_TypeDef)) != 0) {
        // printf("Decoded frame does not match original\n");
        return -2;
    }

    // printf("Decoded frame matches original\n");
    return 0;
}

int generate_test_random() {
    GPS_Fix_TypeDef gps;
    GPSStorageFrame storage;

    // Generate random gps fix
    for (int i = 0; i < sizeof(GPS_Fix_TypeDef); i++) {
        ((uint8_t*)&gps)[i] = rand() % 0xFF;
    }

    // Encode, decode, check
    encode_frame(&gps, storage);
    return check_decode_frame(&gps, storage);
}

int generate_test_random_noise(double BER) {
    GPS_Fix_TypeDef gps;
    GPSStorageFrame storage;

    // Generate random gps fix
    for (int i = 0; i < sizeof(GPS_Fix_TypeDef); i++) {
        ((uint8_t*)&gps)[i] = rand() % 0xFF;
    }

    // Encode
    encode_frame(&gps, storage);

    // Add noise
    for (int i = 0; i < sizeof(GPSStorageFrame); i++) {
        for (int j = 0; j < 8; j++) {
            double random_chance = (double)rand() / RAND_MAX;

            // Evaluate random draw
            if (random_chance < BER) {
                // Flip bit
                storage[i] ^= (0x1 << j);
            }
        }
    }

    // Decode, check
    return check_decode_frame(&gps, storage);
}
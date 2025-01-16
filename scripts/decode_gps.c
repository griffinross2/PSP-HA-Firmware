#include "math.h"
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

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

void print_frame(GPS_Fix_TypeDef *fix);

int main() {
    GPS_Fix_TypeDef fix;
    uint8_t data[80];
    fread(data, 1, 80, stdin);
    memcpy(&fix, data, sizeof(GPS_Fix_TypeDef));
    print_frame(&fix);
}

void print_frame(GPS_Fix_TypeDef *fix) {
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
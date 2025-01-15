#ifndef CONVOLUTIONAL_H
#define CONVOLUTIONAL_H

#include <stdint.h>

#include "max_m10s.h"

// Random bytes to identify the start of a GPS storage frame with decent enough
// confidence
static const uint8_t GPS_STORAGE_FRAME_SYNC[] = {0xAC, 0x20, 0x5C, 0xED};

typedef struct {
    GPS_Fix_TypeDef gps_fix;
    uint8_t
        tail[100 - sizeof(GPS_Fix_TypeDef)];  // To get a nice round 100 bytes
} GPSConvolutionalFrame;

_Static_assert(sizeof(GPSConvolutionalFrame) == 100,
               "GPSConvolutionalFrame is not 100 bytes");

typedef uint8_t GPSStorageFrame[204];

void gps_frame_to_convolutional_frame(const GPS_Fix_TypeDef* gps_frame,
                                      GPSConvolutionalFrame* conv_frame);

void gps_convolutional_frame_to_storage_frame(
    const GPSConvolutionalFrame* conv_frame, GPSStorageFrame storage_frame);

int gps_storage_frame_to_fix(const GPSStorageFrame storage_frame,
                             GPS_Fix_TypeDef* gps_frame);

#endif  // CONVOLUTIONAL_H
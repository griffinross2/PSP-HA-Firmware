#include "convolutional.h"

#include "string.h"

uint8_t get_conv_out(uint8_t state, uint8_t input);

void gps_frame_to_convolutional_frame(const GPS_Fix_TypeDef* gps_frame,
                                      GPSConvolutionalFrame* conv_frame) {
    conv_frame->gps_fix = *gps_frame;
    memset(conv_frame->tail, 0, sizeof(conv_frame->tail));
}

void gps_convolutional_frame_to_storage_frame(
    const GPSConvolutionalFrame* conv_frame, GPSStorageFrame* storage_frame) {
    // First copy the sync word in
    memcpy(storage_frame, GPS_STORAGE_FRAME_SYNC,
           sizeof(GPS_STORAGE_FRAME_SYNC));

    // Copy the GPS frame to a buffer
    uint8_t data[sizeof(GPSConvolutionalFrame)];
    memcpy(data, conv_frame, sizeof(GPSConvolutionalFrame));

    // Now convolutionally encode the GPS frame
    uint8_t encoded_data[sizeof(GPSConvolutionalFrame) * 2];
    memset(encoded_data, 0, sizeof(encoded_data));
    uint8_t state = 0;
    for (int i = 0; i < sizeof(data); i++) {
        for (int j = 0; j < 8; j++) {
            // Enter the next bit into the convolutional encoder
            uint8_t conv_out = get_conv_out(state, (data[i] >> j) & 0x1);

            // Get G1 and move into the next bit of the output
            encoded_data[(i * 2) + ((2 * j) / 8)] |= ((conv_out >> 1) & 0x1)
                                                     << ((2 * j) % 8);

            // Get G2 and move into the next bit of the output
            encoded_data[(i * 2) + ((2 * j) / 8)] |= (conv_out & 0x1)
                                                     << (((2 * j) % 8) + 1);
        }
    }

    // Then interleave (2B x 50B)
    uint8_t interleaved_data[sizeof(encoded_data)];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 50; j++) {
            interleaved_data[i * 50 + j] = data[i + j * 2];
        }
    }

    // Copy the final data to the output
    memcpy(storage_frame + sizeof(GPS_STORAGE_FRAME_SYNC), interleaved_data,
           sizeof(interleaved_data));
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
#include "convolutional.h"

#include "string.h"

uint8_t get_conv_out(uint8_t state, uint8_t input);
uint8_t get_conv_out_2(uint8_t* state, uint8_t input);
void viterbi_decode(uint8_t* data, uint8_t* result);
uint32_t hamming_dist2(uint32_t val1, uint32_t val2);

void gps_frame_to_convolutional_frame(const GPS_Fix_TypeDef* gps_frame,
                                      GPSConvolutionalFrame* conv_frame) {
    conv_frame->gps_fix = *gps_frame;
    memset(conv_frame->tail, 0, sizeof(conv_frame->tail));
}

void gps_convolutional_frame_to_storage_frame(
    const GPSConvolutionalFrame* conv_frame, GPSStorageFrame storage_frame) {
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
            uint8_t conv_out = get_conv_out_2(&state, (data[i] >> j) & 0x1);

            // Get G1 and move into the next bit of the output
            encoded_data[(i * 2) + ((2 * j) / 8)] |= ((conv_out >> 1) & 0x1)
                                                     << ((2 * j) % 8);

            // Get G2 and move into the next bit of the output
            encoded_data[(i * 2) + ((2 * j) / 8)] |= (conv_out & 0x1)
                                                     << (((2 * j) % 8) + 1);
        }
    }

    // Then interleave (2B x 100B)
    uint8_t interleaved_data[sizeof(encoded_data)];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 100; j++) {
            interleaved_data[i * 100 + j] = encoded_data[i + j * 2];
        }
    }

    // Copy the final data to the output
    memcpy(storage_frame + sizeof(GPS_STORAGE_FRAME_SYNC), interleaved_data,
           sizeof(interleaved_data));
}

int gps_storage_frame_to_fix(const GPSStorageFrame storage_frame,
                             GPS_Fix_TypeDef* gps_frame) {
    // First check the sync word
    if (memcmp(storage_frame, GPS_STORAGE_FRAME_SYNC,
               sizeof(GPS_STORAGE_FRAME_SYNC)) != 0) {
        return -1;
    }

    // Deinterleave (2B x 100B)
    uint8_t deinterleaved_data[200];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 100; j++) {
            deinterleaved_data[i + j * 2] = storage_frame[i * 100 + j + 4];
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
    static const int nstates = 64;
    static const int ninput = 200 * 8;
    static const int rate = 2;

    uint32_t path_metric[nstates][ninput / rate + 1];

    // Setup initial state
    memset(path_metric, 0xFF, sizeof(path_metric));
    path_metric[0][0] = 0;

    // Create trellis
    for (int j = 0; j < ninput / rate; j++) {
        for (int i = 0; i < nstates; i++) {
            // skip unaccessible states
            if (path_metric[i][j] == 0xFFFFFFFF) continue;

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
    uint32_t best_metric = 0xFFFFFFFF;
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

uint32_t hamming_dist2(uint32_t val1, uint32_t val2) {
    uint32_t dist = 0;
    for (int i = 0; i < 2; i++) {
        dist += (val1 & 0x1) ^ (val2 & 0x1);
        val1 >>= 1;
        val2 >>= 1;
    }
    return dist;
}
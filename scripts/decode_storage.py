GPS_STORAGE_FRAME_SYNC = [0xAC, 0x20, 0x5C, 0xED]

import numpy as np
import os
import subprocess

def viterbi_decode(data):
    nstates = 64
    ninput = 200 * 8
    rate = 2
    result = np.zeros(int(ninput / rate), dtype=np.uint8)

    path_metric = np.zeros((nstates, int(ninput / rate) + 1), dtype=np.uint32)

    # Setup initial state
    path_metric[:, :] = 0xFFFFFFFF
    path_metric[0][0] = 0

    # Create trellis
    for j in range(int(ninput / rate)):
        for i in range(nstates):
            # skip unaccessible states
            if path_metric[i][j] == 0xFFFFFFFF:
                continue

            # input bit 0
            next_state = (i >> 1)
            conv_out = get_conv_out(i, 0)
            rcvd = (data[j * rate] << 1) | data[j * rate + 1]
            dist = hamming_dist2(rcvd, conv_out)
            if path_metric[next_state][j + 1] > path_metric[i][j] + dist:
                path_metric[next_state][j + 1] = path_metric[i][j] + dist

            # input bit 1
            next_state |= 0b100000
            conv_out = get_conv_out(i, 1)
            dist = hamming_dist2(rcvd, conv_out)
            if path_metric[next_state][j + 1] > path_metric[i][j] + dist:
                path_metric[next_state][j + 1] = path_metric[i][j] + dist

    # Find best ending
    best_state = 0
    best_metric = 0xFFFFFFFF
    for i in range(nstates):
        if path_metric[i][int(ninput / rate)] < best_metric:
            best_metric = path_metric[i][int(ninput / rate)]
            best_state = i
            
    # Traceback through best path
    for i in range(int(ninput / rate) - 1, -1, -1):
        result[i] = (best_state >> 5) & 1

        # Zero path
        previous_state = (best_state << 1) & 0b111111
        best_metric = path_metric[previous_state][i]
        best_state = previous_state

        # One path
        previous_state = (best_state | 1)
        if path_metric[previous_state][i] < best_metric:
            best_metric = path_metric[previous_state][i]
            best_state = previous_state

    return result

def hamming_dist2(val1, val2):
    dist = 0
    for i in range(2):
        dist += (val1 & 0x1) ^ (val2 & 0x1)
        val1 >>= 1
        val2 >>= 1
    return dist

def get_byte_parity(val):
    parity = 0
    while val:
        parity ^= val & 1
        val >>= 1
    return parity

def get_conv_out(state, input):
    G1 = 0b1111001
    G2 = 0b1011011

    conv_in = state | (input << 6)

    # res[1:0] = output bits
    result = 0
    result |= (get_byte_parity(conv_in & G1) << 1)
    result |= (get_byte_parity(conv_in & G2) == 0)

    return result

def gps_storage_frame_to_fix(storage_frame):
    # First check the sync word
    if list(storage_frame[0:4]) != GPS_STORAGE_FRAME_SYNC:
        return -1

    # Deinterleave (2B x 100B)
    deinterleaved_data = [0]*200
    for i in range(2):
        for j in range(100):
            deinterleaved_data[i + j * 2] = storage_frame[i * 100 + j + 4]

    # Expand the data out bit by bit
    encoded_data = (200 * 8) * [0]
    for i in range(200):
        for j in range(8):
            encoded_data[i * 8 + j] = (deinterleaved_data[i] >> j) & 0x1

    # Decode the data
    decoded_data = viterbi_decode(encoded_data)

    # Pack the bits back into bytes
    result = 100 * [0]
    for i in range(100):
        result[i] = 0
        for j in range(8):
            result[i] |= decoded_data[i * 8 + j] << j

    # Check the tail bytes
    for i in range(20):
        if (result[80 + i] != 0):
            return -1

    return result[0:80]

with open("C:/temp/capture.txt", "rb") as f:
    f.seek(36)
    while True:
        data = f.read(256)
        if not data or len(data) < 256:
            break
        decoded = gps_storage_frame_to_fix(data)
        result = subprocess.run(["D:/github_repos/PSP-HA-Firmware-Griffin/scripts/decode_gps.exe"], input=bytes(decoded), stdout=subprocess.PIPE)
        print(result.stdout.decode("utf-8"))
    f.close()
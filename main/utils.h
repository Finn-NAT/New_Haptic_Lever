#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

// Chuyển 1 float thành 4 bytes (little-endian)
void float_to_bytes(float value, uint8_t* bytes) {
    union {
        float f;
        uint8_t b[4];
    } data;
    data.f = value;
    memcpy(bytes, data.b, 4);
}

// Chuyển 4 bytes thành 1 float (little-endian)
float bytes_to_float(const uint8_t* bytes) {
    union {
        float f;
        uint8_t b[4];
    } data;
    memcpy(data.b, bytes, 4);
    return data.f;
}

#endif // UTILS_H
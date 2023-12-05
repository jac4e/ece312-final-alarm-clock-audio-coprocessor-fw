#ifndef TRACKS_H
#define TRACKS_H

#include <stdint.h>

typedef const struct {
    const uint8_t *data;
    const uint32_t length;
    const uint32_t rate;
    const uint32_t channels;
} track_t;

extern track_t track1;
extern track_t track2;

#endif // TRACKS_H
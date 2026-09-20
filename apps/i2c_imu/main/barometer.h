#ifndef QAV250_BAROMETER_H
#define QAV250_BAROMETER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "bmp3.h"

/* Portable BMP388/BMP390 acquisition; all values remain scalars, outside AHRS. */
#define BAROMETER_MAX_AGE_US INT64_C(500000)
struct barometer {
    struct bmp3_dev dev;
    struct bmp3_data data;
    bool initialized;
    bool valid;
    uint8_t address;
    int64_t sample_time_us;
};

int8_t barometer_init(struct barometer *sensor);
/* Poll once, never wait for a conversion. 1 = new sample, 0 = not ready,
 * negative = error. now_us is the monotonic time at the start of the read. */
int barometer_poll(struct barometer *sensor, int64_t now_us);
bool barometer_fresh(const struct barometer *sensor, int64_t now_us);
/* JSON object members including leading comma, shared by raw and MEKF output. */
int barometer_json(const struct barometer *sensor, int64_t now_us, char *out, size_t size);

#endif

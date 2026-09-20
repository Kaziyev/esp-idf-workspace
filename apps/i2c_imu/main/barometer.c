#include "barometer.h"
#include <math.h>
#include <stdio.h>

#ifndef BMP3_FLOAT_COMPENSATION
#error "This adapter requires the Bosch floating-point compensation (Pa and degrees C)."
#endif

int8_t barometer_init(struct barometer *sensor)
{
    sensor->initialized = false;
    sensor->valid = false;
    sensor->sample_time_us = -1;
    int8_t result = bmp3_init(&sensor->dev);
    if (result != BMP3_OK) return result;

    struct bmp3_settings settings = {0};
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.odr = BMP3_ODR_25_HZ;
    settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
    /* One register per write avoids the upstream API's interleaved burst
     * buffer padding. This adds transactions only during initialization. */
    const uint32_t groups[] = {BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN,
        BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS, BMP3_SEL_ODR, BMP3_SEL_IIR_FILTER};
    for (size_t i = 0; i < sizeof(groups) / sizeof(groups[0]); i++) {
        result = bmp3_set_sensor_settings(groups[i], &settings, &sensor->dev);
        if (result != BMP3_OK) return result;
    }
    settings.op_mode = BMP3_MODE_NORMAL;
    result = bmp3_set_op_mode(&settings, &sensor->dev);
    sensor->initialized = result == BMP3_OK;
    return result;
}

int barometer_poll(struct barometer *sensor, int64_t now_us)
{
    if (!sensor->initialized) return BMP3_E_DEV_NOT_FOUND;
    /* ERR (0x02) and STATUS (0x03) are adjacent. Reading STATUS does not
     * clear DRDY; the subsequent six-byte pressure+temperature read does. */
    uint8_t status[2] = {0};
    int8_t result = bmp3_get_regs(BMP3_REG_ERR, status, sizeof(status), &sensor->dev);
    if (result == BMP3_OK && (status[0] & (BMP3_ERR_FATAL | BMP3_ERR_CMD | BMP3_ERR_CONF)))
        result = BMP3_E_CONFIGURATION_ERR;
    if (result != BMP3_OK) {
        sensor->valid = false;
        return result;
    }
    const uint8_t ready = BMP3_DRDY_PRESS | BMP3_DRDY_TEMP;
    if ((status[1] & ready) != ready) return 0;

    struct bmp3_data sample = {0};
    result = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &sample, &sensor->dev);
    /* Reject Bosch range warnings as well as bus errors; do not graph clipped
     * compensation results as if they were measurements. */
    if (result != BMP3_OK || !isfinite(sample.pressure) || !isfinite(sample.temperature) ||
        sample.pressure < 30000 || sample.pressure > 125000 ||
        sample.temperature < -40 || sample.temperature > 85) {
        sensor->valid = false;
        return result < 0 ? result : BMP3_E_CONFIGURATION_ERR;
    }
    sensor->data = sample;
    sensor->sample_time_us = now_us;
    sensor->valid = true;
    return 1;
}

bool barometer_fresh(const struct barometer *sensor, int64_t now_us)
{
    return sensor->initialized && sensor->valid && sensor->sample_time_us >= 0 &&
        now_us >= sensor->sample_time_us && now_us - sensor->sample_time_us <= BAROMETER_MAX_AGE_US;
}

int barometer_json(const struct barometer *sensor, int64_t now_us, char *out, size_t size)
{
    const char *model = !sensor->initialized ? "null" :
        sensor->dev.chip_id == BMP390_CHIP_ID ? "\"BMP390\"" : "\"BMP388\"";
    char age[32] = "null", address[8] = "null";
    if (sensor->sample_time_us >= 0 && now_us >= sensor->sample_time_us)
        snprintf(age, sizeof(age), "%.3f", (double)(now_us - sensor->sample_time_us) / 1000.0);
    if (sensor->initialized) snprintf(address, sizeof(address), "%u", (unsigned)sensor->address);
    if (barometer_fresh(sensor, now_us)) {
        return snprintf(out, size,
            ",\"pressure_pa\":%.3f,\"temperature_c\":%.3f,\"baro_valid\":true"
            ",\"baro_age_ms\":%s,\"baro_model\":%s,\"baro_address\":%s",
            sensor->data.pressure, sensor->data.temperature, age, model, address);
    }
    return snprintf(out, size,
        ",\"pressure_pa\":null,\"temperature_c\":null,\"baro_valid\":false"
        ",\"baro_age_ms\":%s,\"baro_model\":%s,\"baro_address\":%s", age, model, address);
}

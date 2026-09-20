/* Host tests exercise the real Bosch API, calibration decode and adapter.
 * Register values are synthetic fixtures, not measurements of the user's board. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../apps/i2c_imu/main/barometer.h"

struct fake_bus {
    uint8_t regs[256];
    int fail_read_reg;
    bool fail_write;
    unsigned data_reads;
    unsigned writes;
    unsigned delays;
};

static BMP3_INTF_RET_TYPE read_reg(uint8_t reg, uint8_t *data, uint32_t len, void *ptr)
{
    struct fake_bus *bus = ptr;
    if (reg == bus->fail_read_reg) return -1;
    assert((unsigned)reg + len <= sizeof(bus->regs));
    memcpy(data, bus->regs + reg, len);
    if (reg == BMP3_REG_DATA) {
        assert(len == 6); /* Atomic compensated pressure + temperature pair. */
        bus->data_reads++;
        bus->regs[BMP3_REG_SENS_STATUS] &= ~(BMP3_DRDY_PRESS | BMP3_DRDY_TEMP);
    }
    return 0;
}

static BMP3_INTF_RET_TYPE write_reg(uint8_t reg, const uint8_t *data, uint32_t len, void *ptr)
{
    struct fake_bus *bus = ptr;
    if (bus->fail_write) return -1;
    assert(len == 1); /* Adapter intentionally configures one register per call. */
    bus->writes++;
    bus->regs[reg] = data[0];
    return 0;
}

static void delay_us(uint32_t period, void *ptr)
{
    (void)period;
    ((struct fake_bus *)ptr)->delays++;
}

static void le16(uint8_t *out, uint16_t v) {out[0] = v & 255; out[1] = v >> 8;}
static void le24(uint8_t *out, uint32_t v) {out[0] = v & 255; out[1] = (v >> 8) & 255; out[2] = v >> 16;}

static void ready(struct fake_bus *bus)
{
    bus->regs[BMP3_REG_SENS_STATUS] = BMP3_CMD_RDY | BMP3_DRDY_PRESS | BMP3_DRDY_TEMP;
}

static struct barometer fixture(struct fake_bus *bus, uint8_t id)
{
    memset(bus, 0, sizeof(*bus));
    bus->fail_read_reg = -1;
    bus->regs[BMP3_REG_CHIP_ID] = id;
    ready(bus);
    uint8_t *cal = bus->regs + BMP3_REG_CALIB_DATA;
    le16(cal + 0, 16384); /* T1 = 4194304 */
    le16(cal + 2, 16384); /* T2 = 1/65536 */
    le16(cal + 5, 20480); /* P1 = 1/256 */
    le16(cal + 7, 16384); /* P2 = 0 */
    le16(cal + 11, 12000); /* P5 = 96000 */
    le16(cal + 13, 64); /* P6 = 1 */
    le24(bus->regs + BMP3_REG_DATA, 1024000);
    le24(bus->regs + BMP3_REG_DATA + 3, 5832704);
    return (struct barometer){.sample_time_us = -1, .address = BMP3_ADDR_I2C_PRIM,
        .dev = {.intf = BMP3_I2C_INTF, .intf_ptr = bus,
            .read = read_reg, .write = write_reg, .delay_us = delay_us}};
}

int main(void)
{
    struct fake_bus bus;
    char json[240];
    for (unsigned model = 0; model < 2; model++) {
        struct barometer sensor = fixture(&bus, model ? BMP390_CHIP_ID : BMP3_CHIP_ID);
        sensor.address = model ? 0x77 : 0x76;
        assert(barometer_init(&sensor) == BMP3_OK);
        assert(sensor.initialized && !sensor.valid);
        assert(bus.regs[BMP3_REG_PWR_CTRL] == 0x33); /* pressure/temp + normal */
        assert(bus.regs[BMP3_REG_OSR] == 0x0B); /* 8x pressure, 2x temperature */
        assert(bus.regs[BMP3_REG_ODR] == BMP3_ODR_25_HZ);
        assert(bus.regs[BMP3_REG_CONFIG] == 0x04); /* IIR coefficient 3 */
        unsigned delays = bus.delays;
        assert(barometer_poll(&sensor, 1000000) == 1);
        assert(bus.delays == delays && bus.data_reads == 1); /* no conversion wait */
        assert(fabs(sensor.data.temperature - 25) < 1e-8);
        assert(fabs(sensor.data.pressure - 100025) < 1e-8);
        assert(barometer_fresh(&sensor, 1500000));
        assert(!barometer_fresh(&sensor, 1500001));
        assert(!barometer_fresh(&sensor, 999999));
        int len = barometer_json(&sensor, 1001000, json, sizeof(json));
        assert(len > 0 && (size_t)len < sizeof(json));
        assert(strstr(json, "\"pressure_pa\":100025.000"));
        assert(strstr(json, "\"temperature_c\":25.000"));
        assert(strstr(json, "\"baro_age_ms\":1.000"));
        assert(strstr(json, model ? "BMP390" : "BMP388"));
        assert(strstr(json, model ? "\"baro_address\":119" : "\"baro_address\":118"));
        printf("{%s}\n", json + 1); /* Feed this actual C output to dashboard test. */

        assert(barometer_poll(&sensor, 1100000) == 0); /* DRDY cleared by read */
        assert(sensor.sample_time_us == 1000000 && bus.data_reads == 1);
        bus.regs[BMP3_REG_SENS_STATUS] = BMP3_DRDY_PRESS;
        assert(barometer_poll(&sensor, 1200000) == 0); /* require both channels */
        barometer_json(&sensor, 1600000, json, sizeof(json));
        assert(strstr(json, "\"baro_valid\":false") && strstr(json, "\"pressure_pa\":null"));

        ready(&bus); bus.fail_read_reg = BMP3_REG_DATA;
        assert(barometer_poll(&sensor, 1700000) < 0 && !sensor.valid);
        bus.fail_read_reg = -1;
        assert(barometer_poll(&sensor, 1800000) == 1 && sensor.valid);
        bus.fail_read_reg = BMP3_REG_ERR;
        assert(barometer_poll(&sensor, 1900000) < 0 && !sensor.valid);
        bus.fail_read_reg = -1; bus.regs[BMP3_REG_ERR] = BMP3_ERR_CONF;
        assert(barometer_poll(&sensor, 2000000) < 0 && !sensor.valid);
        bus.regs[BMP3_REG_ERR] = 0; ready(&bus);
        assert(barometer_poll(&sensor, 2100000) == 1);

        le24(bus.regs + BMP3_REG_DATA + 3, 0xFFFFFF); ready(&bus);
        assert(barometer_poll(&sensor, 2200000) < 0); /* range warning not a sample */
        assert(!barometer_fresh(&sensor, 2200000));
        barometer_json(&sensor, 2200000, json, sizeof(json));
        assert(strstr(json, "\"temperature_c\":null"));
        le24(bus.regs + BMP3_REG_DATA + 3, 5832704);
        le24(bus.regs + BMP3_REG_DATA, 0xFFFFFF); ready(&bus);
        assert(barometer_poll(&sensor, 2300000) < 0); /* pressure overflow */
    }

    struct barometer bad = fixture(&bus, 0x58); /* BMP280 must not masquerade as BMP3xx. */
    assert(barometer_init(&bad) == BMP3_E_DEV_NOT_FOUND && !bad.initialized);
    assert(bus.writes == 0);
    barometer_json(&bad, 3000000, json, sizeof(json));
    assert(strstr(json, "\"baro_model\":null") && strstr(json, "\"baro_age_ms\":null"));
    assert(strstr(json, "\"baro_valid\":false"));
    printf("{%s}\n", json + 1);
    bad = fixture(&bus, BMP3_CHIP_ID); bus.fail_read_reg = BMP3_REG_CALIB_DATA;
    assert(barometer_init(&bad) < 0 && !bad.initialized);
    bad = fixture(&bus, BMP3_CHIP_ID); bus.fail_write = true;
    assert(barometer_init(&bad) < 0 && !bad.initialized);
    assert(barometer_poll(&bad, 0) < 0);
    puts("PASS: BMP388/BMP390 IDs, settings, compensated Pa/C, DRDY, timestamps, stale/error/null output.");
    return 0;
}

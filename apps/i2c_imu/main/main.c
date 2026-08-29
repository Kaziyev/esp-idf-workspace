#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "driver/i2c_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_rom_sys.h"

#include "bmi270.h"
#include "bmi2.h"


/* ============================================================
   ESP32-S3 I2C CONFIG
   ============================================================ */

#define I2C_PORT_NUM           0
#define I2C_SDA_GPIO           40
#define I2C_SCL_GPIO           39
#define I2C_SPEED_HZ           100000

#define BMI270_ADDR_1          0x68
#define BMI270_ADDR_2          0x69

#define BMI270_EXPECTED_ID     0x24


static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t bmi_dev = NULL;


/* ============================================================
   BOSCH I2C READ CALLBACK
   ============================================================ */

static BMI2_INTF_RETURN_TYPE bmi_i2c_read(
    uint8_t reg_addr,
    uint8_t *reg_data,
    uint32_t len,
    void *intf_ptr)
{
    i2c_master_dev_handle_t dev =
        (i2c_master_dev_handle_t)intf_ptr;

    esp_err_t ret = i2c_master_transmit_receive(
        dev,
        &reg_addr,
        1,
        reg_data,
        len,
        1000
    );

    if (ret == ESP_OK)
    {
        return BMI2_INTF_RET_SUCCESS;
    }

    printf(
        "Bosch READ error: reg=0x%02X, %s\n",
        reg_addr,
        esp_err_to_name(ret)
    );

    return -1;
}


/* ============================================================
   BOSCH I2C WRITE CALLBACK
   ============================================================ */

static BMI2_INTF_RETURN_TYPE bmi_i2c_write(
    uint8_t reg_addr,
    const uint8_t *reg_data,
    uint32_t len,
    void *intf_ptr)
{
    i2c_master_dev_handle_t dev =
        (i2c_master_dev_handle_t)intf_ptr;

    /*
     * Bosch example uses read_write_len = 46,
     * so 64 bytes is enough.
     */
    uint8_t buffer[64];

    if ((len + 1) > sizeof(buffer))
    {
        printf("Bosch WRITE buffer too small: %lu\n",
               (unsigned long)len);

        return -1;
    }

    buffer[0] = reg_addr;

    memcpy(
        &buffer[1],
        reg_data,
        len
    );

    esp_err_t ret = i2c_master_transmit(
        dev,
        buffer,
        len + 1,
        1000
    );

    if (ret == ESP_OK)
    {
        return BMI2_INTF_RET_SUCCESS;
    }

    printf(
        "Bosch WRITE error: reg=0x%02X, %s\n",
        reg_addr,
        esp_err_to_name(ret)
    );

    return -1;
}


/* ============================================================
   BOSCH DELAY CALLBACK
   ============================================================ */

static void bmi_delay_us(
    uint32_t period,
    void *intf_ptr)
{
    (void)intf_ptr;

    esp_rom_delay_us(period);
}


/* ============================================================
   FIND BMI270 I2C ADDRESS
   ============================================================ */

static uint8_t find_bmi270_address(void)
{
    printf("\nScanning BMI270 addresses...\n");

    if (i2c_master_probe(
            i2c_bus,
            BMI270_ADDR_1,
            100) == ESP_OK)
    {
        printf("FOUND device at 0x68\n");

        return BMI270_ADDR_1;
    }


    if (i2c_master_probe(
            i2c_bus,
            BMI270_ADDR_2,
            100) == ESP_OK)
    {
        printf("FOUND device at 0x69\n");

        return BMI270_ADDR_2;
    }


    printf("BMI270 not found!\n");

    return 0;
}


/* ============================================================
   DIRECT REGISTER READ
   ============================================================ */

static esp_err_t direct_read_register(
    i2c_master_dev_handle_t dev,
    uint8_t reg,
    uint8_t *value)
{
    return i2c_master_transmit_receive(
        dev,
        &reg,
        1,
        value,
        1,
        500
    );
}


/* ============================================================
   APP MAIN
   ============================================================ */

void app_main(void)
{
    printf("\n");
    printf("=============================\n");
    printf(" ESP32-S3 + BMI270 START\n");
    printf("=============================\n\n");


    /* ========================================================
       1. CREATE I2C BUS
       ======================================================== */

    i2c_master_bus_config_t bus_config = {

        .clk_source =
            I2C_CLK_SRC_DEFAULT,

        .i2c_port =
            I2C_PORT_NUM,

        .sda_io_num =
            I2C_SDA_GPIO,

        .scl_io_num =
            I2C_SCL_GPIO,

        .glitch_ignore_cnt =
            7,

        .flags.enable_internal_pullup =
            true,
    };


    esp_err_t ret = i2c_new_master_bus(
        &bus_config,
        &i2c_bus
    );


    if (ret != ESP_OK)
    {
        printf(
            "ERROR creating I2C bus: %s\n",
            esp_err_to_name(ret)
        );

        return;
    }


    printf("I2C bus created\n");

    printf(
        "SDA GPIO = %d\n",
        I2C_SDA_GPIO
    );

    printf(
        "SCL GPIO = %d\n",
        I2C_SCL_GPIO
    );


    /*
     * Give sensor some time after startup
     */
    vTaskDelay(
        pdMS_TO_TICKS(200)
    );


    /* ========================================================
       2. FIND BMI270 ADDRESS
       ======================================================== */

    uint8_t bmi_address =
        find_bmi270_address();


    if (bmi_address == 0)
    {
        printf(
            "ERROR: Check SDA, SCL, GND and power.\n"
        );

        return;
    }


    /* ========================================================
       3. ADD BMI270 DEVICE TO I2C BUS
       ======================================================== */

    i2c_device_config_t dev_config = {

        .dev_addr_length =
            I2C_ADDR_BIT_LEN_7,

        .device_address =
            bmi_address,

        .scl_speed_hz =
            I2C_SPEED_HZ,
    };


    ret = i2c_master_bus_add_device(
        i2c_bus,
        &dev_config,
        &bmi_dev
    );


    if (ret != ESP_OK)
    {
        printf(
            "ERROR adding BMI270: %s\n",
            esp_err_to_name(ret)
        );

        return;
    }


    printf(
        "BMI270 device created at 0x%02X\n",
        bmi_address
    );


    /* ========================================================
       4. DIRECT CHIP-ID TEST
       ======================================================== */

    uint8_t chip_id = 0;

    ret = direct_read_register(
        bmi_dev,
        0x00,
        &chip_id
    );


    if (ret != ESP_OK)
    {
        printf("\n");
        printf(
            "ERROR: Cannot read CHIP_ID.\n"
        );

        printf(
            "I2C address ACK works, but register read fails.\n"
        );

        printf(
            "Check that BMI270 CSB pin is HIGH.\n"
        );

        return;
    }


    printf(
        "DIRECT CHIP_ID = 0x%02X\n",
        chip_id
    );


    if (chip_id != BMI270_EXPECTED_ID)
    {
        printf(
            "ERROR: Expected BMI270 CHIP_ID = 0x24\n"
        );

        printf(
            "Actual value = 0x%02X\n",
            chip_id
        );

        return;
    }


    printf(
        "BMI270 CHIP_ID correct!\n"
    );


    /* ========================================================
       5. BOSCH BMI270 DRIVER CONFIG
       ======================================================== */

    struct bmi2_dev bmi;

    memset(
        &bmi,
        0,
        sizeof(bmi)
    );


    bmi.intf =
        BMI2_I2C_INTF;

    /*
     * Pass ESP-IDF device handle directly
     */
    bmi.intf_ptr =
        (void *)bmi_dev;

    bmi.read =
        bmi_i2c_read;

    bmi.write =
        bmi_i2c_write;

    bmi.delay_us =
        bmi_delay_us;


    /*
     * Bosch official example uses 46 bytes
     */
    bmi.read_write_len =
        46;


    /*
     * NULL:
     * bmi270_init() uses the configuration
     * included in bmi270.c
     */
    bmi.config_file_ptr =
        NULL;


    /* ========================================================
       6. INITIALIZE BMI270
       ======================================================== */

    printf("\nInitializing BMI270...\n");


    int8_t rslt =
        bmi270_init(&bmi);


    if (rslt != BMI2_OK)
    {
        printf(
            "ERROR: bmi270_init failed: %d\n",
            rslt
        );

        return;
    }


    printf(
        "BMI270 initialized successfully!\n"
    );


    printf(
        "Bosch CHIP ID = 0x%02X\n",
        bmi.chip_id
    );


    /* ========================================================
       7. ACCELEROMETER CONFIGURATION
       ======================================================== */

    struct bmi2_sens_config config;

    memset(
        &config,
        0,
        sizeof(config)
    );


    config.type =
        BMI2_ACCEL;


    /*
     * First get existing/default configuration
     */
    rslt = bmi2_get_sensor_config(
        &config,
        1,
        &bmi
    );


    if (rslt != BMI2_OK)
    {
        printf(
            "ERROR getting accel config: %d\n",
            rslt
        );

        return;
    }


    /*
     * Accelerometer configuration:
     *
     * 100 Hz
     * +/-2g
     * normal averaging
     * high-performance filter
     */

    config.cfg.acc.odr =
        BMI2_ACC_ODR_100HZ;

    config.cfg.acc.range =
        BMI2_ACC_RANGE_2G;

    config.cfg.acc.bwp =
        BMI2_ACC_NORMAL_AVG4;

    config.cfg.acc.filter_perf =
        BMI2_PERF_OPT_MODE;


    rslt = bmi2_set_sensor_config(
        &config,
        1,
        &bmi
    );


    if (rslt != BMI2_OK)
    {
        printf(
            "ERROR setting accel config: %d\n",
            rslt
        );

        return;
    }


    printf(
        "Accelerometer configured\n"
    );


    /* ========================================================
       8. ENABLE ACCELEROMETER
       ======================================================== */

    uint8_t sensor_list[1] = {
        BMI2_ACCEL
    };


    rslt = bmi2_sensor_enable(
        sensor_list,
        1,
        &bmi
    );


    if (rslt != BMI2_OK)
    {
        printf(
            "ERROR enabling accelerometer: %d\n",
            rslt
        );

        return;
    }


    printf(
        "Accelerometer enabled!\n"
    );


    vTaskDelay(
        pdMS_TO_TICKS(100)
    );


    /* ========================================================
       9. READ ACCELEROMETER
       ======================================================== */

    struct bmi2_sens_data sensor_data;

    memset(
        &sensor_data,
        0,
        sizeof(sensor_data)
    );


    printf("\n");
    printf(
        "Reading accelerometer...\n\n"
    );


    while (1)
    {
        rslt = bmi2_get_sensor_data(
            &sensor_data,
            &bmi
        );


        if (rslt != BMI2_OK)
        {
            printf(
                "ERROR reading sensor: %d\n",
                rslt
            );

            vTaskDelay(
                pdMS_TO_TICKS(500)
            );

            continue;
        }


        /*
         * Check whether new accel data is ready
         */
        if (sensor_data.status &
            BMI2_DRDY_ACC)
        {
            int16_t raw_x =
                sensor_data.acc.x;

            int16_t raw_y =
                sensor_data.acc.y;

            int16_t raw_z =
                sensor_data.acc.z;


            /*
             * For +/-2g:
             * approximately 16384 LSB = 1 g
             */
            float x_g =
                raw_x / 16384.0f;

            float y_g =
                raw_y / 16384.0f;

            float z_g =
                raw_z / 16384.0f;


            printf(
                "RAW X=%6d Y=%6d Z=%6d | "
                "g X=% .3f Y=% .3f Z=% .3f\n",
                raw_x,
                raw_y,
                raw_z,
                x_g,
                y_g,
                z_g
            );
        }


        vTaskDelay(
            pdMS_TO_TICKS(200)
        );
    }
}
/* QAV250 telemetry kit: ESP32-S3 + BMI270 + BMM150 + optional BMP388/BMP390.
 * Derived from the user's supplied MEKF implementation.
 * Default: raw ax..mz JSON with read timestamps, for browser AHRS.
 * Optional: firmware MEKF quaternion JSON (idf.py menuconfig > QAV250).
 * Filter/alignment/calibration limitations: docs/QAV250_EKF_review.md.
 */
#include "sdkconfig.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "driver/i2c_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include <inttypes.h>

#include "bmi270.h"
#include "bmi2.h"
#include "bmm150.h"
#include "sensor_axes.h"
#include "barometer.h"

/* ============================================================
   ESP32-S3 + BMI270 + BMM150
   Multiplicative / error-state EKF (MEKF)

   BMI270:
     accelerometer 100 Hz, +/-2 g
     gyroscope     100 Hz, +/-2000 dps

   BMM150:
     magnetometer, Bosch compensated output

   I2C:
     SDA GPIO40
     SCL GPIO39

   IMPORTANT:
   - BMM150 is a MAGNETOMETER, not a barometer.
   - Yaw is referenced to the magnetic direction measured at startup.
   - For accurate yaw, calibrate BMM150 hard-iron/soft-iron offsets
     and make sure BMM150 axes are aligned with BMI270 axes.
   ============================================================ */


/* ============================================================
   USER CONFIGURATION
   ============================================================ */

#define I2C_PORT_NUM           0
#define I2C_SDA_GPIO           CONFIG_QAV250_I2C_SDA_GPIO
#define I2C_SCL_GPIO           CONFIG_QAV250_I2C_SCL_GPIO
#define I2C_SPEED_HZ           100000

#define BMI270_ADDR_1          0x68
#define BMI270_ADDR_2          0x69
#define BMI270_EXPECTED_ID     0x24

#define BMM150_ADDR_1          0x10
#define BMM150_ADDR_2          0x11
#define BMM150_ADDR_3          0x12
#define BMM150_ADDR_4          0x13

#define ACC_LSB_PER_G          16384.0f
#define GYRO_DPS_PER_LSB       (2000.0f / 32768.0f)

#define DEG_TO_RAD             0.01745329251994329577f
#define RAD_TO_DEG             57.295779513082320876f

#define IMU_PERIOD_MS          10      /* 100 Hz */
#define MAG_DIVIDER            10      /* 100 Hz / 10 = 10 Hz */
#define PRINT_DIVIDER          10      /* 10 Hz serial output */


/* ============================================================
   BMM150 USER CALIBRATION

   These values are placeholders. They let the code run, but for
   accurate yaw you must later perform hard-iron/soft-iron calibration.

   calibrated_x = (raw_x - OFFSET_X) * SCALE_X
   ============================================================ */

#define MAG_OFFSET_X           0.0f
#define MAG_OFFSET_Y           0.0f
#define MAG_OFFSET_Z           0.0f

#define MAG_SCALE_X            1.0f
#define MAG_SCALE_Y            1.0f
#define MAG_SCALE_Z            1.0f


/* ============================================================
   GLOBAL I2C HANDLES
   ============================================================ */

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t bmi_dev = NULL;
static i2c_master_dev_handle_t bmm_dev = NULL;
static i2c_master_dev_handle_t bmp_dev = NULL;

/* BMP3 transactions have a short timeout so a missing/disconnected barometer
 * cannot stall IMU acquisition for the default one-second I2C timeout. */
static BMP3_INTF_RET_TYPE bmp_i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *ptr)
{
    return i2c_master_transmit_receive((i2c_master_dev_handle_t)ptr,
        &reg, 1, data, len, 10) == ESP_OK ? 0 : -1;
}

static BMP3_INTF_RET_TYPE bmp_i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *ptr)
{
    uint8_t buffer[64];
    if (len > sizeof(buffer) - 1) return -1;
    buffer[0] = reg;
    memcpy(buffer + 1, data, len);
    return i2c_master_transmit((i2c_master_dev_handle_t)ptr, buffer, len + 1, 10) == ESP_OK ? 0 : -1;
}

static void bmp_delay_us(uint32_t period, void *ptr)
{
    (void)ptr;
    esp_rom_delay_us(period);
}

static void start_barometer(struct barometer *sensor)
{
    /* Verify chip ID before resetting/configuring anything: another I2C
     * peripheral may acknowledge these addresses. Try both if the first fails. */
    const uint8_t addresses[] = {BMP3_ADDR_I2C_PRIM, BMP3_ADDR_I2C_SEC};
    for (size_t i = 0; i < sizeof(addresses); i++) {
        if (i2c_master_probe(i2c_bus, addresses[i], 10) != ESP_OK) continue;
        const i2c_device_config_t config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addresses[i],
            .scl_speed_hz = I2C_SPEED_HZ,
        };
        if (i2c_master_bus_add_device(i2c_bus, &config, &bmp_dev) != ESP_OK) continue;
        uint8_t chip = 0;
        if (bmp_i2c_read(BMP3_REG_CHIP_ID, &chip, 1, bmp_dev) == 0 &&
            (chip == BMP3_CHIP_ID || chip == BMP390_CHIP_ID)) {
            *sensor = (struct barometer){.address = addresses[i], .sample_time_us = -1,
                .dev = {.intf = BMP3_I2C_INTF, .intf_ptr = bmp_dev,
                    .read = bmp_i2c_read, .write = bmp_i2c_write, .delay_us = bmp_delay_us}};
            const int8_t result = barometer_init(sensor);
            if (result == BMP3_OK) {
                printf("BMP%u initialized at 0x%02X: 25 Hz, pressure 8x, temperature 2x, IIR 3\n",
                    chip == BMP390_CHIP_ID ? 390U : 388U, addresses[i]);
                return;
            }
            printf("WARNING: BMP3xx init at 0x%02X failed (%d)\n", addresses[i], result);
        }
        i2c_master_bus_rm_device(bmp_dev);
        bmp_dev = NULL;
    }
    *sensor = (struct barometer){.sample_time_us = -1};
    printf("WARNING: BMP388/BMP390 unavailable at 0x76/0x77; IMU continues without barometer\n");
}


/* ============================================================
   COMMON ESP-IDF I2C READ
   ============================================================ */

static esp_err_t i2c_read_reg(
    i2c_master_dev_handle_t dev,
    uint8_t reg,
    uint8_t *data,
    uint32_t len)
{
    return i2c_master_transmit_receive(
        dev,
        &reg,
        1,
        data,
        len,
        1000
    );
}


/* ============================================================
   COMMON ESP-IDF I2C WRITE
   ============================================================ */

static esp_err_t i2c_write_reg(
    i2c_master_dev_handle_t dev,
    uint8_t reg,
    const uint8_t *data,
    uint32_t len)
{
    uint8_t buffer[64];

    if ((len + 1U) > sizeof(buffer))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    buffer[0] = reg;

    if (len > 0)
    {
        memcpy(&buffer[1], data, len);
    }

    return i2c_master_transmit(
        dev,
        buffer,
        len + 1U,
        1000
    );
}


/* ============================================================
   BMI270 BOSCH CALLBACKS
   ============================================================ */

static BMI2_INTF_RETURN_TYPE bmi_i2c_read(
    uint8_t reg_addr,
    uint8_t *reg_data,
    uint32_t len,
    void *intf_ptr)
{
    i2c_master_dev_handle_t dev =
        (i2c_master_dev_handle_t)intf_ptr;

    return (i2c_read_reg(dev, reg_addr, reg_data, len) == ESP_OK)
        ? 0
        : -1;
}


static BMI2_INTF_RETURN_TYPE bmi_i2c_write(
    uint8_t reg_addr,
    const uint8_t *reg_data,
    uint32_t len,
    void *intf_ptr)
{
    i2c_master_dev_handle_t dev =
        (i2c_master_dev_handle_t)intf_ptr;

    return (i2c_write_reg(dev, reg_addr, reg_data, len) == ESP_OK)
        ? 0
        : -1;
}


static void bmi_delay_us(
    uint32_t period,
    void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period);
}


/* ============================================================
   BMM150 BOSCH CALLBACKS
   ============================================================ */

static BMM150_INTF_RET_TYPE bmm_i2c_read(
    uint8_t reg_addr,
    uint8_t *reg_data,
    uint32_t len,
    void *intf_ptr)
{
    i2c_master_dev_handle_t dev =
        (i2c_master_dev_handle_t)intf_ptr;

    return (i2c_read_reg(dev, reg_addr, reg_data, len) == ESP_OK)
        ? 0
        : -1;
}


static BMM150_INTF_RET_TYPE bmm_i2c_write(
    uint8_t reg_addr,
    const uint8_t *reg_data,
    uint32_t len,
    void *intf_ptr)
{
    i2c_master_dev_handle_t dev =
        (i2c_master_dev_handle_t)intf_ptr;

    return (i2c_write_reg(dev, reg_addr, reg_data, len) == ESP_OK)
        ? 0
        : -1;
}


static void bmm_delay_us(
    uint32_t period,
    void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period);
}


/* ============================================================
   I2C ADDRESS SEARCH
   ============================================================ */

static uint8_t find_bmi270_address(void)
{
    const uint8_t addresses[] =
    {
        BMI270_ADDR_1,
        BMI270_ADDR_2
    };

    for (size_t i = 0; i < sizeof(addresses); i++)
    {
        if (i2c_master_probe(i2c_bus, addresses[i], 100) == ESP_OK)
        {
            return addresses[i];
        }
    }

    return 0;
}


static uint8_t find_bmm150_address(void)
{
    const uint8_t addresses[] =
    {
        BMM150_ADDR_1,
        BMM150_ADDR_2,
        BMM150_ADDR_3,
        BMM150_ADDR_4
    };

    for (size_t i = 0; i < sizeof(addresses); i++)
    {
        if (i2c_master_probe(i2c_bus, addresses[i], 100) == ESP_OK)
        {
            return addresses[i];
        }
    }

    return 0;
}


/* ============================================================
   SMALL VECTOR HELPERS
   ============================================================ */

static float norm3(float x, float y, float z)
{
    return sqrtf(x*x + y*y + z*z);
}


static int normalize3(float v[3])
{
    float n = norm3(v[0], v[1], v[2]);

    if (n < 1e-9f)
    {
        return 0;
    }

    v[0] /= n;
    v[1] /= n;
    v[2] /= n;

    return 1;
}


/* ============================================================
   QUATERNION HELPERS
   Convention:
     q = [w, x, y, z]
     q represents body -> world rotation
   ============================================================ */

static void quat_normalize(float q[4])
{
    float n = sqrtf(
        q[0]*q[0] +
        q[1]*q[1] +
        q[2]*q[2] +
        q[3]*q[3]
    );

    if (n < 1e-9f)
    {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;
}


static void quat_multiply(
    const float a[4],
    const float b[4],
    float out[4])
{
    out[0] =
        a[0]*b[0] -
        a[1]*b[1] -
        a[2]*b[2] -
        a[3]*b[3];

    out[1] =
        a[0]*b[1] +
        a[1]*b[0] +
        a[2]*b[3] -
        a[3]*b[2];

    out[2] =
        a[0]*b[2] -
        a[1]*b[3] +
        a[2]*b[0] +
        a[3]*b[1];

    out[3] =
        a[0]*b[3] +
        a[1]*b[2] -
        a[2]*b[1] +
        a[3]*b[0];
}


static void quat_from_euler(
    float roll,
    float pitch,
    float yaw,
    float q[4])
{
    float cr = cosf(0.5f * roll);
    float sr = sinf(0.5f * roll);

    float cp = cosf(0.5f * pitch);
    float sp = sinf(0.5f * pitch);

    float cy = cosf(0.5f * yaw);
    float sy = sinf(0.5f * yaw);

    q[0] = cr*cp*cy + sr*sp*sy;
    q[1] = sr*cp*cy - cr*sp*sy;
    q[2] = cr*sp*cy + sr*cp*sy;
    q[3] = cr*cp*sy - sr*sp*cy;

    quat_normalize(q);
}


static void quat_to_euler(
    const float q[4],
    float *roll,
    float *pitch,
    float *yaw)
{
    float qw = q[0];
    float qx = q[1];
    float qy = q[2];
    float qz = q[3];

    *roll = atan2f(
        2.0f * (qw*qx + qy*qz),
        1.0f - 2.0f * (qx*qx + qy*qy)
    );

    float s = 2.0f * (qw*qy - qz*qx);

    if (s > 1.0f)  s = 1.0f;
    if (s < -1.0f) s = -1.0f;

    *pitch = asinf(s);

    *yaw = atan2f(
        2.0f * (qw*qz + qx*qy),
        1.0f - 2.0f * (qy*qy + qz*qz)
    );
}


/* ============================================================
   ROTATE VECTOR BODY -> WORLD
   ============================================================ */

static void body_to_world(
    const float q[4],
    const float b[3],
    float w[3])
{
    float qw = q[0];
    float qx = q[1];
    float qy = q[2];
    float qz = q[3];

    float r00 = 1.0f - 2.0f*(qy*qy + qz*qz);
    float r01 = 2.0f*(qx*qy - qz*qw);
    float r02 = 2.0f*(qx*qz + qy*qw);

    float r10 = 2.0f*(qx*qy + qz*qw);
    float r11 = 1.0f - 2.0f*(qx*qx + qz*qz);
    float r12 = 2.0f*(qy*qz - qx*qw);

    float r20 = 2.0f*(qx*qz - qy*qw);
    float r21 = 2.0f*(qy*qz + qx*qw);
    float r22 = 1.0f - 2.0f*(qx*qx + qy*qy);

    w[0] = r00*b[0] + r01*b[1] + r02*b[2];
    w[1] = r10*b[0] + r11*b[1] + r12*b[2];
    w[2] = r20*b[0] + r21*b[1] + r22*b[2];
}


/* ============================================================
   ROTATE VECTOR WORLD -> BODY
   ============================================================ */

static void world_to_body(
    const float q[4],
    const float w[3],
    float b[3])
{
    float qw = q[0];
    float qx = q[1];
    float qy = q[2];
    float qz = q[3];

    float r00 = 1.0f - 2.0f*(qy*qy + qz*qz);
    float r01 = 2.0f*(qx*qy - qz*qw);
    float r02 = 2.0f*(qx*qz + qy*qw);

    float r10 = 2.0f*(qx*qy + qz*qw);
    float r11 = 1.0f - 2.0f*(qx*qx + qz*qz);
    float r12 = 2.0f*(qy*qz - qx*qw);

    float r20 = 2.0f*(qx*qz - qy*qw);
    float r21 = 2.0f*(qy*qz + qx*qw);
    float r22 = 1.0f - 2.0f*(qx*qx + qy*qy);

    /* R^T * w */
    b[0] = r00*w[0] + r10*w[1] + r20*w[2];
    b[1] = r01*w[0] + r11*w[1] + r21*w[2];
    b[2] = r02*w[0] + r12*w[1] + r22*w[2];
}


/* ============================================================
   3x3 MATRIX INVERSE
   ============================================================ */

static int inverse3x3(
    const float A[3][3],
    float inv[3][3])
{
    float det =
        A[0][0] * (A[1][1]*A[2][2] - A[1][2]*A[2][1])
      - A[0][1] * (A[1][0]*A[2][2] - A[1][2]*A[2][0])
      + A[0][2] * (A[1][0]*A[2][1] - A[1][1]*A[2][0]);

    if (fabsf(det) < 1e-10f)
    {
        return 0;
    }

    float id = 1.0f / det;

    inv[0][0] =  (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * id;
    inv[0][1] =  (A[0][2]*A[2][1] - A[0][1]*A[2][2]) * id;
    inv[0][2] =  (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * id;

    inv[1][0] =  (A[1][2]*A[2][0] - A[1][0]*A[2][2]) * id;
    inv[1][1] =  (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * id;
    inv[1][2] =  (A[0][2]*A[1][0] - A[0][0]*A[1][2]) * id;

    inv[2][0] =  (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * id;
    inv[2][1] =  (A[0][1]*A[2][0] - A[0][0]*A[2][1]) * id;
    inv[2][2] =  (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * id;

    return 1;
}


/* ============================================================
   MEKF STATE

   Nominal state:
     q[4]   = attitude quaternion
     bias[3]= gyro bias

   Error state covariance P[6][6]:
     [ dtheta_x dtheta_y dtheta_z dbias_x dbias_y dbias_z ]
   ============================================================ */

typedef struct
{
    float q[4];
    float bias[3];

    float P[6][6];

    float mag_ref_world[3];
    float mag_strength_ref;

} mekf_t;


/* ============================================================
   MEKF INITIALIZATION

   Roll/Pitch are initialized from gravity.
   Yaw = 0 at startup.
   The first magnetic vector becomes the world magnetic reference,
   therefore yaw is RELATIVE to startup heading.
   ============================================================ */

static void mekf_init(
    mekf_t *f,
    float ax,
    float ay,
    float az,
    float gyro_bias_x,
    float gyro_bias_y,
    float gyro_bias_z,
    float mx,
    float my,
    float mz)
{
    memset(f, 0, sizeof(*f));

    float roll = atan2f(ay, az);

    float pitch = atan2f(
        -ax,
        sqrtf(ay*ay + az*az)
    );

    quat_from_euler(
        roll,
        pitch,
        0.0f,
        f->q
    );

    f->bias[0] = gyro_bias_x;
    f->bias[1] = gyro_bias_y;
    f->bias[2] = gyro_bias_z;

    for (int i = 0; i < 6; i++)
    {
        f->P[i][i] = (i < 3)
            ? 0.02f
            : 0.002f;
    }

    f->mag_strength_ref = norm3(mx, my, mz);

    float m_body[3] = { mx, my, mz };

    if (!normalize3(m_body))
    {
        m_body[0] = 1.0f;
        m_body[1] = 0.0f;
        m_body[2] = 0.0f;
    }

    body_to_world(
        f->q,
        m_body,
        f->mag_ref_world
    );

    normalize3(f->mag_ref_world);
}


/* ============================================================
   MEKF PREDICTION FROM GYROSCOPE
   ============================================================ */

static void mekf_predict(
    mekf_t *f,
    float gx,
    float gy,
    float gz,
    float dt)
{
    float wx = gx - f->bias[0];
    float wy = gy - f->bias[1];
    float wz = gz - f->bias[2];

    float omega = norm3(wx, wy, wz);

    float dq[4];

    if (omega < 1e-8f)
    {
        dq[0] = 1.0f;
        dq[1] = 0.5f * wx * dt;
        dq[2] = 0.5f * wy * dt;
        dq[3] = 0.5f * wz * dt;
    }
    else
    {
        float half_angle = 0.5f * omega * dt;
        float s = sinf(half_angle) / omega;

        dq[0] = cosf(half_angle);
        dq[1] = wx * s;
        dq[2] = wy * s;
        dq[3] = wz * s;
    }

    quat_normalize(dq);

    float q_new[4];

    quat_multiply(
        f->q,
        dq,
        q_new
    );

    memcpy(f->q, q_new, sizeof(q_new));
    quat_normalize(f->q);


    /* Error-state transition: dtheta_dot = -skew(w)dtheta - dbias */

    float F[6][6] = {0};

    for (int i = 0; i < 6; i++)
    {
        F[i][i] = 1.0f;
    }

    /* I - skew(w)*dt */
    F[0][1] =  wz * dt;
    F[0][2] = -wy * dt;

    F[1][0] = -wz * dt;
    F[1][2] =  wx * dt;

    F[2][0] =  wy * dt;
    F[2][1] = -wx * dt;

    /* gyro bias -> attitude error */
    F[0][3] = -dt;
    F[1][4] = -dt;
    F[2][5] = -dt;


    /* temp = F * P */

    float temp[6][6] = {0};

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            for (int k = 0; k < 6; k++)
            {
                temp[i][j] += F[i][k] * f->P[k][j];
            }
        }
    }


    /* P_new = F * P * F^T */

    float P_new[6][6] = {0};

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            for (int k = 0; k < 6; k++)
            {
                P_new[i][j] += temp[i][k] * F[j][k];
            }
        }
    }


    /* Process noise.
       These are practical initial tuning values, not universal constants. */

    const float gyro_process_noise = 3.0e-4f;
    const float bias_process_noise = 1.0e-6f;

    for (int i = 0; i < 3; i++)
    {
        P_new[i][i] += gyro_process_noise * dt;
        P_new[i+3][i+3] += bias_process_noise * dt;
    }

    memcpy(f->P, P_new, sizeof(P_new));
}


/* ============================================================
   GENERIC VECTOR MEASUREMENT UPDATE

   Measurement:
      normalized vector measured in BODY frame

   Reference:
      normalized known vector in WORLD frame

   Used for:
      accelerometer -> gravity
      magnetometer  -> Earth's magnetic field
   ============================================================ */

static int mekf_vector_update(
    mekf_t *f,
    float vx,
    float vy,
    float vz,
    const float ref_world[3],
    float measurement_noise)
{
    float z[3] = { vx, vy, vz };

    if (!normalize3(z))
    {
        return 0;
    }


    /* Predicted direction in body frame */

    float h[3];

    world_to_body(
        f->q,
        ref_world,
        h
    );

    if (!normalize3(h))
    {
        return 0;
    }


    /* innovation = measured - predicted */

    float innovation[3] =
    {
        z[0] - h[0],
        z[1] - h[1],
        z[2] - h[2]
    };


    /* H = [ skew(h)  0 ] */

    float H[3][6] =
    {
        {  0.0f, -h[2],  h[1], 0.0f, 0.0f, 0.0f },
        {  h[2],  0.0f, -h[0], 0.0f, 0.0f, 0.0f },
        { -h[1],  h[0],  0.0f, 0.0f, 0.0f, 0.0f }
    };


    /* HP = H * P */

    float HP[3][6] = {0};

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            for (int k = 0; k < 6; k++)
            {
                HP[i][j] += H[i][k] * f->P[k][j];
            }
        }
    }


    /* S = HPH^T + R */

    float S[3][3] = {0};

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < 6; k++)
            {
                S[i][j] += HP[i][k] * H[j][k];
            }
        }
    }

    S[0][0] += measurement_noise;
    S[1][1] += measurement_noise;
    S[2][2] += measurement_noise;


    float S_inv[3][3];

    if (!inverse3x3(S, S_inv))
    {
        return 0;
    }


    /* PHt = P * H^T */

    float PHt[6][3] = {0};

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < 6; k++)
            {
                PHt[i][j] += f->P[i][k] * H[j][k];
            }
        }
    }


    /* K = PHt * S^-1 */

    float K[6][3] = {0};

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                K[i][j] += PHt[i][k] * S_inv[k][j];
            }
        }
    }


    /* dx = K * innovation */

    float dx[6] = {0};

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            dx[i] += K[i][j] * innovation[j];
        }
    }


    /* Correct quaternion with small-angle error */

    float dq[4] =
    {
        1.0f,
        0.5f * dx[0],
        0.5f * dx[1],
        0.5f * dx[2]
    };

    quat_normalize(dq);

    float q_new[4];

    quat_multiply(
        f->q,
        dq,
        q_new
    );

    memcpy(f->q, q_new, sizeof(q_new));
    quat_normalize(f->q);


    /* Correct gyro bias */

    f->bias[0] += dx[3];
    f->bias[1] += dx[4];
    f->bias[2] += dx[5];


    /* Joseph covariance update:
       P = (I-KH)P(I-KH)^T + K R K^T
       R = measurement_noise * I
    */

    float A[6][6] = {0};

    for (int i = 0; i < 6; i++)
    {
        A[i][i] = 1.0f;
    }

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                A[i][j] -= K[i][k] * H[k][j];
            }
        }
    }


    /* AP = A * P */

    float AP[6][6] = {0};

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            for (int k = 0; k < 6; k++)
            {
                AP[i][j] += A[i][k] * f->P[k][j];
            }
        }
    }


    /* P_new = AP * A^T */

    float P_new[6][6] = {0};

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            for (int k = 0; k < 6; k++)
            {
                P_new[i][j] += AP[i][k] * A[j][k];
            }
        }
    }


    /* + K R K^T, where R = noise * I */

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                P_new[i][j] +=
                    measurement_noise *
                    K[i][k] *
                    K[j][k];
            }
        }
    }


    /* Force covariance symmetry to limit numerical drift */

    for (int i = 0; i < 6; i++)
    {
        for (int j = i + 1; j < 6; j++)
        {
            float avg = 0.5f * (P_new[i][j] + P_new[j][i]);
            P_new[i][j] = avg;
            P_new[j][i] = avg;
        }
    }

    memcpy(f->P, P_new, sizeof(P_new));

    return 1;
}


/* ============================================================
   ACCELEROMETER UPDATE

   We only trust accelerometer as gravity when magnitude is
   reasonably near 1 g.
   ============================================================ */

static int mekf_update_accel(
    mekf_t *f,
    float ax,
    float ay,
    float az)
{
    float a = norm3(ax, ay, az);

    if (a < 0.80f || a > 1.20f)
    {
        return 0;
    }

    static const float gravity_world[3] =
    {
        0.0f,
        0.0f,
        1.0f
    };

    return mekf_vector_update(
        f,
        ax,
        ay,
        az,
        gravity_world,
        0.02f
    );
}


/* ============================================================
   MAGNETOMETER UPDATE

   Reject very large changes in magnetic field strength because
   motors, ESCs, wires and metal can disturb BMM150.
   ============================================================ */

static int mekf_update_mag(
    mekf_t *f,
    float mx,
    float my,
    float mz)
{
    float m = norm3(mx, my, mz);

    if (m < 1e-6f || f->mag_strength_ref < 1e-6f)
    {
        return 0;
    }

    float ratio = m / f->mag_strength_ref;

    if (ratio < 0.60f || ratio > 1.40f)
    {
        return 0;
    }

    return mekf_vector_update(
        f,
        mx,
        my,
        mz,
        f->mag_ref_world,
        0.05f
    );
}


/* ============================================================
   BMM150 HARD/SOFT IRON CALIBRATION PLACEHOLDER
   ============================================================ */

static void calibrate_mag(
    float *mx,
    float *my,
    float *mz)
{
    *mx = (*mx - MAG_OFFSET_X) * MAG_SCALE_X;
    *my = (*my - MAG_OFFSET_Y) * MAG_SCALE_Y;
    *mz = (*mz - MAG_OFFSET_Z) * MAG_SCALE_Z;
}


/* ============================================================
   BMM150 -> BMI270 AXIS ALIGNMENT

   Default: user's breadboard photo, BMM -> BMI = (-X, +Y, -Z).
   This runs before MEKF initialization, updates, and both JSON outputs.
   Choose identity in menuconfig only when the sensor axes already agree.
   ACC/GYRO stay in native BMI270 axes; body mounting is handled by the UI.
   ============================================================ */

static void remap_mag_axes(
    float *mx,
    float *my,
    float *mz)
{
#ifdef CONFIG_QAV250_MAG_AXES_PHOTO
    qav250_mag_photo_to_imu(mx, my, mz);
#else
    (void)mx;
    (void)my;
    (void)mz;
#endif
}


/* ============================================================
   READ ONE BMM150 SAMPLE
   ============================================================ */

static int read_bmm150(
    struct bmm150_dev *bmm,
    float *mx,
    float *my,
    float *mz)
{
    struct bmm150_mag_data mag = {0};

    int8_t rslt = bmm150_read_mag_data(
        &mag,
        bmm
    );

    if (rslt != BMM150_OK)
    {
        return 0;
    }

    *mx = (float)mag.x;
    *my = (float)mag.y;
    *mz = (float)mag.z;

    calibrate_mag(mx, my, mz);
    remap_mag_axes(mx, my, mz);

    return 1;
}


/* ============================================================
   APP MAIN
   ============================================================ */

void app_main(void)
{
    setvbuf(stdout, NULL, _IOLBF, 0);
#ifdef CONFIG_QAV250_MAG_AXES_PHOTO
    printf("MAG axes: photo mounting (-X,+Y,-Z) -> BMI270\n");
#else
    printf("MAG axes: identity (physically aligned with BMI270)\n");
#endif
#ifdef CONFIG_QAV250_OUTPUT_EKF_JSON
    printf("Telemetry: firmware MEKF JSON (q + sensors)\n");
#else
    printf("Telemetry: raw JSON (ax..mz + timestamps; browser AHRS)\n");
#endif
    printf("\n");
    printf("====================================================\n");
    printf(" ESP32-S3 + BMI270 + BMM150 + 9-DoF MEKF\n");
    printf("====================================================\n\n");


    /* ========================================================
       1. CREATE I2C BUS
       ======================================================== */

    i2c_master_bus_config_t bus_config =
    {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_NUM,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
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

    printf(
        "I2C OK: SDA=%d SCL=%d speed=%d Hz\n",
        I2C_SDA_GPIO,
        I2C_SCL_GPIO,
        I2C_SPEED_HZ
    );

    vTaskDelay(pdMS_TO_TICKS(200));


    /* ========================================================
       2. FIND + ADD BMI270
       ======================================================== */

    uint8_t bmi_address = find_bmi270_address();

    if (bmi_address == 0)
    {
        printf("ERROR: BMI270 not found at 0x68/0x69\n");
        return;
    }

    printf(
        "BMI270 ACK at 0x%02X\n",
        bmi_address
    );


    i2c_device_config_t bmi_dev_config =
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = bmi_address,
        .scl_speed_hz = I2C_SPEED_HZ,
    };

    ret = i2c_master_bus_add_device(
        i2c_bus,
        &bmi_dev_config,
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


    /* ========================================================
       3. DIRECT BMI270 CHIP-ID TEST
       ======================================================== */

    uint8_t bmi_chip_id = 0;

    ret = i2c_read_reg(
        bmi_dev,
        0x00,
        &bmi_chip_id,
        1
    );

    if (ret != ESP_OK)
    {
        printf(
            "ERROR reading BMI270 CHIP_ID: %s\n",
            esp_err_to_name(ret)
        );
        return;
    }

    printf(
        "BMI270 CHIP_ID = 0x%02X\n",
        bmi_chip_id
    );

    if (bmi_chip_id != BMI270_EXPECTED_ID)
    {
        printf(
            "ERROR: expected BMI270 CHIP_ID 0x24\n"
        );
        return;
    }


    /* ========================================================
       4. BMI270 BOSCH DRIVER INIT
       ======================================================== */

    struct bmi2_dev bmi;
    memset(&bmi, 0, sizeof(bmi));

    bmi.intf = BMI2_I2C_INTF;
    bmi.intf_ptr = (void *)bmi_dev;
    bmi.read = bmi_i2c_read;
    bmi.write = bmi_i2c_write;
    bmi.delay_us = bmi_delay_us;
    bmi.read_write_len = 46;
    bmi.config_file_ptr = NULL;

    int8_t bmi_rslt = bmi270_init(&bmi);

    if (bmi_rslt != BMI2_OK)
    {
        printf(
            "ERROR: bmi270_init() = %d\n",
            bmi_rslt
        );
        return;
    }

    printf("BMI270 initialized\n");


    /* ========================================================
       5. BMI270 ACCEL + GYRO CONFIG
       ======================================================== */

    struct bmi2_sens_config bmi_config[2];
    memset(bmi_config, 0, sizeof(bmi_config));

    bmi_config[0].type = BMI2_ACCEL;
    bmi_config[1].type = BMI2_GYRO;

    bmi_rslt = bmi2_get_sensor_config(
        bmi_config,
        2,
        &bmi
    );

    if (bmi_rslt != BMI2_OK)
    {
        printf(
            "ERROR getting BMI270 config: %d\n",
            bmi_rslt
        );
        return;
    }


    /* Accelerometer: 100 Hz, +/-2g */

    bmi_config[0].cfg.acc.odr =
        BMI2_ACC_ODR_100HZ;

    bmi_config[0].cfg.acc.range =
        BMI2_ACC_RANGE_2G;

    bmi_config[0].cfg.acc.bwp =
        BMI2_ACC_NORMAL_AVG4;

    bmi_config[0].cfg.acc.filter_perf =
        BMI2_PERF_OPT_MODE;


    /* Gyroscope: 100 Hz, +/-2000 dps */

    bmi_config[1].cfg.gyr.odr =
        BMI2_GYR_ODR_100HZ;

    bmi_config[1].cfg.gyr.range =
        BMI2_GYR_RANGE_2000;

    bmi_config[1].cfg.gyr.bwp =
        BMI2_GYR_NORMAL_MODE;

    bmi_config[1].cfg.gyr.noise_perf =
        BMI2_POWER_OPT_MODE;

    bmi_config[1].cfg.gyr.filter_perf =
        BMI2_PERF_OPT_MODE;


    bmi_rslt = bmi2_set_sensor_config(
        bmi_config,
        2,
        &bmi
    );

    if (bmi_rslt != BMI2_OK)
    {
        printf(
            "ERROR setting BMI270 config: %d\n",
            bmi_rslt
        );
        return;
    }


    uint8_t sensor_list[2] =
    {
        BMI2_ACCEL,
        BMI2_GYRO
    };

    bmi_rslt = bmi2_sensor_enable(
        sensor_list,
        2,
        &bmi
    );

    if (bmi_rslt != BMI2_OK)
    {
        printf(
            "ERROR enabling BMI270 sensors: %d\n",
            bmi_rslt
        );
        return;
    }

    printf("BMI270 ACC + GYRO enabled\n");

    vTaskDelay(pdMS_TO_TICKS(100));


    /* ========================================================
       6. FIND + ADD BMM150
       ======================================================== */

    uint8_t bmm_address = find_bmm150_address();

    if (bmm_address == 0)
    {
        printf(
            "ERROR: BMM150 not found at 0x10/0x11/0x12/0x13\n"
        );
        return;
    }

    printf(
        "BMM150 ACK at 0x%02X\n",
        bmm_address
    );


    i2c_device_config_t bmm_dev_config =
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = bmm_address,
        .scl_speed_hz = I2C_SPEED_HZ,
    };

    ret = i2c_master_bus_add_device(
        i2c_bus,
        &bmm_dev_config,
        &bmm_dev
    );

    if (ret != ESP_OK)
    {
        printf(
            "ERROR adding BMM150: %s\n",
            esp_err_to_name(ret)
        );
        return;
    }


    /* ========================================================
       7. BMM150 BOSCH DRIVER INIT
       ======================================================== */

    struct bmm150_dev bmm;
    memset(&bmm, 0, sizeof(bmm));

    bmm.intf = BMM150_I2C_INTF;
    bmm.intf_ptr = (void *)bmm_dev;
    bmm.read = bmm_i2c_read;
    bmm.write = bmm_i2c_write;
    bmm.delay_us = bmm_delay_us;

    int8_t bmm_rslt = bmm150_init(&bmm);

    if (bmm_rslt != BMM150_OK)
    {
        printf(
            "ERROR: bmm150_init() = %d\n",
            bmm_rslt
        );
        return;
    }

    printf(
        "BMM150 CHIP_ID = 0x%02X\n",
        bmm.chip_id
    );


    struct bmm150_settings bmm_settings;
    memset(
        &bmm_settings,
        0,
        sizeof(bmm_settings)
    );


    /* Normal measurement mode */

    bmm_settings.pwr_mode =
        BMM150_POWERMODE_NORMAL;

    bmm_rslt = bmm150_set_op_mode(
        &bmm_settings,
        &bmm
    );

    if (bmm_rslt != BMM150_OK)
    {
        printf(
            "ERROR setting BMM150 power mode: %d\n",
            bmm_rslt
        );
        return;
    }


    /* High-accuracy repetition preset */

    bmm_settings.preset_mode =
        BMM150_PRESETMODE_HIGHACCURACY;

    bmm_rslt = bmm150_set_presetmode(
        &bmm_settings,
        &bmm
    );

    if (bmm_rslt != BMM150_OK)
    {
        printf(
            "ERROR setting BMM150 preset: %d\n",
            bmm_rslt
        );
        return;
    }

    printf("BMM150 initialized\n");

    struct barometer baro = {.sample_time_us = -1};
    start_barometer(&baro);

    vTaskDelay(pdMS_TO_TICKS(200));


    /* ========================================================
       8. INITIAL STATIONARY CALIBRATION

       Do NOT move the board during this stage.
       We estimate initial gyro bias and gravity direction.
       ======================================================== */

    printf("\n");
    printf("KEEP THE BOARD STILL FOR ~2 SECONDS...\n");

    struct bmi2_sens_data imu_data;
    memset(&imu_data, 0, sizeof(imu_data));

    const int calibration_samples = 200;

    float acc_sum[3] = {0};
    float gyro_sum[3] = {0};

    int collected = 0;

    while (collected < calibration_samples)
    {
        bmi_rslt = bmi2_get_sensor_data(
            &imu_data,
            &bmi
        );

        if (
            bmi_rslt == BMI2_OK &&
            (imu_data.status & BMI2_DRDY_ACC) &&
            (imu_data.status & BMI2_DRDY_GYR)
        )
        {
            float ax =
                (float)imu_data.acc.x /
                ACC_LSB_PER_G;

            float ay =
                (float)imu_data.acc.y /
                ACC_LSB_PER_G;

            float az =
                (float)imu_data.acc.z /
                ACC_LSB_PER_G;


            float gx =
                (float)imu_data.gyr.x *
                GYRO_DPS_PER_LSB *
                DEG_TO_RAD;

            float gy =
                (float)imu_data.gyr.y *
                GYRO_DPS_PER_LSB *
                DEG_TO_RAD;

            float gz =
                (float)imu_data.gyr.z *
                GYRO_DPS_PER_LSB *
                DEG_TO_RAD;


            acc_sum[0] += ax;
            acc_sum[1] += ay;
            acc_sum[2] += az;

            gyro_sum[0] += gx;
            gyro_sum[1] += gy;
            gyro_sum[2] += gz;

            collected++;
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_PERIOD_MS));
    }


    float acc_init[3] =
    {
        acc_sum[0] / (float)calibration_samples,
        acc_sum[1] / (float)calibration_samples,
        acc_sum[2] / (float)calibration_samples
    };


    float gyro_bias_init[3] =
    {
        gyro_sum[0] / (float)calibration_samples,
        gyro_sum[1] / (float)calibration_samples,
        gyro_sum[2] / (float)calibration_samples
    };


    printf(
        "Initial gyro bias: X=% .3f Y=% .3f Z=% .3f deg/s\n",
        gyro_bias_init[0] * RAD_TO_DEG,
        gyro_bias_init[1] * RAD_TO_DEG,
        gyro_bias_init[2] * RAD_TO_DEG
    );


    /* ========================================================
       9. AVERAGE INITIAL BMM150 FIELD

       Keep the board still and away from motors/large metal.
       ======================================================== */

    float mag_sum[3] = {0};
    int mag_samples = 0;
    int64_t last_mag_read_us = 0;

    while (mag_samples < 10)
    {
        float mx, my, mz;

        if (read_bmm150(
                &bmm,
                &mx,
                &my,
                &mz))
        {
            last_mag_read_us = esp_timer_get_time();
            mag_sum[0] += mx;
            mag_sum[1] += my;
            mag_sum[2] += mz;

            mag_samples++;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }


    float mag_init[3] =
    {
        mag_sum[0] / (float)mag_samples,
        mag_sum[1] / (float)mag_samples,
        mag_sum[2] / (float)mag_samples
    };


    printf(
        "Initial MAG: X=% .2f Y=% .2f Z=% .2f\n",
        mag_init[0],
        mag_init[1],
        mag_init[2]
    );


    /* ========================================================
       10. INITIALIZE MEKF
       ======================================================== */

    mekf_t ekf;

    mekf_init(
        &ekf,

        acc_init[0],
        acc_init[1],
        acc_init[2],

        gyro_bias_init[0],
        gyro_bias_init[1],
        gyro_bias_init[2],

        mag_init[0],
        mag_init[1],
        mag_init[2]
    );

    printf("MEKF initialized\n\n");


    /* ========================================================
       11. MAIN 100 Hz LOOP
       ======================================================== */

    TickType_t last_wake =
        xTaskGetTickCount();

    TickType_t previous_tick =
        last_wake;

    uint32_t loop_counter = 0;
    uint32_t telemetry_seq = 0;

    float last_mx = mag_init[0];
    float last_my = mag_init[1];
    float last_mz = mag_init[2];

    int last_acc_used = 0;
    int last_mag_used = 0;


    while (1)
    {
        /*
         * Makes the loop approximately periodic at 100 Hz.
         * Filter dt below still uses ticks; telemetry timestamps use esp_timer.
         */
        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(IMU_PERIOD_MS)
        );


        bmi_rslt = bmi2_get_sensor_data(
            &imu_data,
            &bmi
        );


        if (bmi_rslt != BMI2_OK)
        {
            printf(
                "BMI270 read error: %d\n",
                bmi_rslt
            );
            continue;
        }


        if (
            !(imu_data.status & BMI2_DRDY_ACC) ||
            !(imu_data.status & BMI2_DRDY_GYR)
        )
        {
            continue;
        }


        /* ====================================================
           dt from FreeRTOS ticks
           ==================================================== */

        /* Timestamp of host read completion, NOT BMI270 hardware sample time. */
        const int64_t sample_time_us = esp_timer_get_time();

        TickType_t now_tick =
            xTaskGetTickCount();


        float dt =
            (float)(now_tick - previous_tick) *
            ((float)portTICK_PERIOD_MS / 1000.0f);


        previous_tick =
            now_tick;


        if (dt < 0.001f || dt > 0.05f)
        {
            dt = 0.010f;
        }


        /* ====================================================
           BMI270 ACCEL -> g
           ==================================================== */

        float ax =
            (float)imu_data.acc.x /
            ACC_LSB_PER_G;

        float ay =
            (float)imu_data.acc.y /
            ACC_LSB_PER_G;

        float az =
            (float)imu_data.acc.z /
            ACC_LSB_PER_G;


        /* ====================================================
           BMI270 GYRO -> rad/s
           ==================================================== */

        float gx =
            (float)imu_data.gyr.x *
            GYRO_DPS_PER_LSB *
            DEG_TO_RAD;

        float gy =
            (float)imu_data.gyr.y *
            GYRO_DPS_PER_LSB *
            DEG_TO_RAD;

        float gz =
            (float)imu_data.gyr.z *
            GYRO_DPS_PER_LSB *
            DEG_TO_RAD;


        /* ====================================================
           EKF PREDICTION FROM GYRO
           ==================================================== */

        mekf_predict(
            &ekf,
            gx,
            gy,
            gz,
            dt
        );


        /* ====================================================
           EKF CORRECTION FROM ACCEL
           ==================================================== */

        last_acc_used =
            mekf_update_accel(
                &ekf,
                ax,
                ay,
                az
            );


        /* ====================================================
           BMM150 + EKF MAG CORRECTION AT ~10 Hz
           ==================================================== */

        if ((loop_counter % MAG_DIVIDER) == 0U)
        {
            float mx, my, mz;

            if (read_bmm150(
                    &bmm,
                    &mx,
                    &my,
                    &mz))
            {
                last_mag_read_us = esp_timer_get_time();
                last_mx = mx;
                last_my = my;
                last_mz = mz;

                last_mag_used =
                    mekf_update_mag(
                        &ekf,
                        mx,
                        my,
                        mz
                    );
            }
            else
            {
                last_mag_used = 0;
            }
        }


        /* ====================================================
           PRINT AT ~10 Hz
           ==================================================== */

        if ((loop_counter % PRINT_DIVIDER) == 0U)
        {
            /* Sensor converts in normal mode at 25 Hz; read latest ready pair
             * at telemetry rate (~10 Hz). No busy-wait and no AHRS coupling. */
            if (baro.initialized) barometer_poll(&baro, esp_timer_get_time());
            /* Exact nominal quaternion; no reconstruction from rounded Euler angles.
             * mag_age_ms is age since the last successful API read, not hardware DRDY.
             * This telemetry change does not fix timing or sample-freshness handling.
             */
            const int64_t emit_time_us = esp_timer_get_time();
            char baro_json[240];
            barometer_json(&baro, emit_time_us, baro_json, sizeof(baro_json));
            const float mag_read_age_ms =
                (float)(emit_time_us - last_mag_read_us) / 1000.0f;
            const float values[] = {
                ax, ay, az, gx, gy, gz, last_mx, last_my, last_mz,
                dt, mag_read_age_ms,
#ifdef CONFIG_QAV250_OUTPUT_EKF_JSON
                ekf.q[0], ekf.q[1], ekf.q[2], ekf.q[3],
                ekf.bias[0], ekf.bias[1], ekf.bias[2],
#endif
            };
            int finite_packet = 1;
            for (size_t i = 0; i < sizeof(values)/sizeof(values[0]); i++)
                if (!isfinite(values[i])) finite_packet = 0;
            if (finite_packet) {
#ifdef CONFIG_QAV250_OUTPUT_EKF_JSON
                printf(
                    "{\"v\":1,\"t_us\":%" PRId64 ",\"seq\":%" PRIu32
                    ",\"q\":[%.7f,%.7f,%.7f,%.7f]"
                    ",\"mag_frame\":\"bmi270\""
                    ",\"a_g\":[%.5f,%.5f,%.5f]"
                    ",\"g_dps\":[%.4f,%.4f,%.4f]"
                    ",\"m_uT\":[%.4f,%.4f,%.4f]"
                    ",\"bias_dps\":[%.5f,%.5f,%.5f]"
                    ",\"dt_s\":%.6f,\"acc_used\":%s,\"mag_used\":%s"
                    ",\"mag_age_ms\":%.3f%s}\n",
                    sample_time_us, telemetry_seq++,
                    ekf.q[0], ekf.q[1], ekf.q[2], ekf.q[3],
                    ax, ay, az,
                    gx * RAD_TO_DEG, gy * RAD_TO_DEG, gz * RAD_TO_DEG,
                    last_mx, last_my, last_mz,
                    ekf.bias[0] * RAD_TO_DEG, ekf.bias[1] * RAD_TO_DEG,
                    ekf.bias[2] * RAD_TO_DEG,
                    dt, last_acc_used ? "true" : "false",
                    last_mag_used ? "true" : "false", mag_read_age_ms, baro_json
                );
#else
                /* Same flat sensor keys as the user's working firmware.
                 * t_us is read-completion time, not the sensor's internal clock.
                 * The browser derives orientation; no firmware q is implied.
                 */
                printf(
                    "{\"v\":1,\"t_us\":%" PRId64 ",\"seq\":%" PRIu32
                    ",\"ax\":%.5f,\"ay\":%.5f,\"az\":%.5f"
                    ",\"mag_frame\":\"bmi270\""
                    ",\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f"
                    ",\"mx\":%.4f,\"my\":%.4f,\"mz\":%.4f"
                    ",\"mag_age_ms\":%.3f,\"acc_used\":%s,\"mag_used\":%s%s}\n",
                    sample_time_us, telemetry_seq++,
                    ax, ay, az,
                    gx * RAD_TO_DEG, gy * RAD_TO_DEG, gz * RAD_TO_DEG,
                    last_mx, last_my, last_mz, mag_read_age_ms,
                    last_acc_used ? "true" : "false",
                    last_mag_used ? "true" : "false", baro_json
                );
#endif
            } else {
                printf("ERROR: non-finite telemetry sample\n");
            }
        }


        loop_counter++;
    }
}

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../apps/i2c_imu/main/sensor_axes.h"

int main(void)
{
    float basis[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const float expected[3][3] = {{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}};
    for (int i = 0; i < 3; ++i) {
        qav250_mag_photo_to_imu(&basis[i][0], &basis[i][1], &basis[i][2]);
        for (int j = 0; j < 3; ++j) assert(basis[i][j] == expected[i][j]);
    }
    assert(basis[0][0] * basis[1][1] * basis[2][2] == 1); /* no reflection */
    float x = 13.5f, y = -27.25f, z = 42.0f;
    const float norm2 = x*x + y*y + z*z;
    qav250_mag_photo_to_imu(&x, &y, &z);
    assert(x == -13.5f && y == -27.25f && z == -42.0f);
    assert(fabsf(x*x + y*y + z*z - norm2) < 0.001f);
    qav250_mag_photo_to_imu(&x, &y, &z);
    assert(x == 13.5f && y == -27.25f && z == 42.0f);
    puts("PASS: firmware BMM150 -> BMI270 photo axes, handedness, norm, round trip.");
    return 0;
}

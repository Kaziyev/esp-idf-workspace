# Included third-party sensor drivers

## BMI270 / BMI2

The existing repository's `bmi2.c`, `bmi2.h`, `bmi2_defs.h`, `bmi270.c`, and `bmi270.h` are retained. They identify Bosch Sensor API version **v2.86.1**, dated **2023-05-03**, and include Bosch's BSD-3-Clause license notices. Their exact upstream commit was not recorded in the original repository.

Upstream: [Bosch BMI270 Sensor API](https://github.com/boschsensortec/BMI270_SensorAPI).

## BMM150

`apps/i2c_imu/main/bmm150.c`, `bmm150.h`, and `bmm150_defs.h` are copied unmodified from the official Bosch repository at commit:

`0dce0617873cda1f6d51f6b7b961fdc2641e0c7c`

Upstream: [Bosch BMM150 Sensor API](https://github.com/boschsensortec/BMM150_SensorAPI/tree/0dce0617873cda1f6d51f6b7b961fdc2641e0c7c).

License: [BSD-3-Clause](licenses/BMM150-BSD-3-Clause.txt), with original notices preserved in the source files.

The dashboard uses browser APIs and original JavaScript; it does not load a third-party graphics or chart library. The browser AHRS is an independent simplified complementary filter, not a bundled copy of the x-io Fusion library.

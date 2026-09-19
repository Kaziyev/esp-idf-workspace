#ifndef QAV250_SENSOR_AXES_H
#define QAV250_SENSOR_AXES_H

/* Nominal alignment inferred from the user's breadboard photo (2026-09-20):
 * BMI270: +X left, +Y down, +Z out of the photo.
 * BMM150: +X right, +Y down; right-handed axes imply +Z into the photo.
 * R_IMU_MAG = diag(-1, +1, -1), a proper rotation (det = +1).
 * Fine mechanical skew and magnetic hard/soft iron require measurement.
 */
static inline void qav250_mag_photo_to_imu(float *mx, float *my, float *mz)
{
    *mx = -*mx;
    (void)my;
    *mz = -*mz;
}

#endif

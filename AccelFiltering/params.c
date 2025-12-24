// Written by Sinan Cimen, 2025. https://github.com/sinancimen

/**
 * Accelerometer Filter Order
 *
 * Order of the butterworth filter applied to the accelerometer data.
 *
 * @min 1
 * @max 10
 * @reboot_required true
 * @group Accel Filtering
 */
PARAM_DEFINE_INT32(SFILT_ACCEL_N, 2);

/**
 * Linear Jerk Filter Order
 *
 * Order of the butterworth filter applied to the linear jerk data.
 *
 * @min 1
 * @max 10
 * @reboot_required true
 * @group Accel Filtering
 */
PARAM_DEFINE_INT32(SFILT_JRK_N, 2);

/**
 * Accelerometer Filter Cutoff Frequency
 *
 * Cutoff frequency of the butterworth filter applied to the accelerometer data, in rad/s.
 *
 * @min 1
 * @max 400
 * @reboot_required true
 * @group Accel Filtering
 */
PARAM_DEFINE_FLOAT(SFILT_ACCEL_FREQ, 70.0f);

/**
 * Linear Jerk Filter Cutoff Frequency
 *
 * Cutoff frequency of the butterworth filter applied to the linear jerk data, in rad/s.
 *
 * @min 1
 * @max 400
 * @reboot_required true
 * @group Accel Filtering
 */
PARAM_DEFINE_FLOAT(SFILT_JRK_FREQ, 70.0f);
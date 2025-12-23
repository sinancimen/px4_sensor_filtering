// Written by Sinan Cimen, 2025. https://github.com/sinancimen

/**
 * Accelerometer Filter Order
 *
 * Order of the butterworth filter applied to the accelerometer data.
 *
 * @min 2
 * @max 10
 * @reboot_required true
 * @group Sensor Filtering
 */
PARAM_DEFINE_INT32(SFILT_ACCEL_N, 2);

/**
 * Gyroscope Filter Order
 *
 * Order of the butterworth filter applied to the gyroscope data.
 *
 * @min 2
 * @max 10
 * @reboot_required true
 * @group Sensor Filtering
 */
PARAM_DEFINE_INT32(SFILT_GYRO_N, 2);

/**
 * Accelerometer Filter Cutoff Frequency
 *
 * Cutoff frequency of the butterworth filter applied to the accelerometer data, in rad/s.
 *
 * @min 1
 * @max 400
 * @reboot_required true
 * @group Sensor Filtering
 */
PARAM_DEFINE_FLOAT(SFILT_ACCEL_FREQ, 70.0);

/**
 * Gyroscope Filter Cutoff Frequency
 *
 * Cutoff frequency of the butterworth filter applied to the gyroscope data, in rad/s.
 *
 * @min 1
 * @max 400
 * @reboot_required true
 * @group Sensor Filtering
 */
PARAM_DEFINE_FLOAT(SFILT_GYRO_FREQ, 50.0);
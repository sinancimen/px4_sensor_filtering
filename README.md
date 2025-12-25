This repository contains PX4 modules to filter angular velocity and linear acceleration data.

Filter modules run at 400 Hz, and apply a Butterworth filter with variable order and cutoff frequency to sensor data published at 'vehicle_angular_velocity' and 'vehicle_acceleration' uORB topics.

In addition to the filtered angular velocity and linear acceleration, modules also publish filtered angular acceleration and linear jerk.

The maximum filter order is 10.

Tested only for PX4 v1.13.3.


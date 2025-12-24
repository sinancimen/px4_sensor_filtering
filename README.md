This is a PX4 module for low-pass filtering of body angular rate and body linear acceleration.

Filter module runs at 800 Hz and applies a Butterworth filter with variable order and cutoff frequency to sensor data published at 'vehicle_angular_velocity' and 'vehicle_acceleration' uORB topics.
# Written by Sinan Cimen, 2025. https://github.com/sinancimen
# In SI-unit form.

uint64 timestamp				# time since system start (microseconds)

float32[3] accel_mps2    # filtered linear acceleration (m/s^2)
float32[3] jerk_mps3     # filtered linear jerk (m/s^3)

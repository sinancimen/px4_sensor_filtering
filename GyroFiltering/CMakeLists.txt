# Written by Sinan Cimen, 2025. https://github.com/sinancimen

px4_add_module(
	MODULE modules__gyro_filtering
	MAIN gyro_filtering
	SRCS
		gyro_filtering.cpp
	)
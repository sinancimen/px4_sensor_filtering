# Written by Sinan Cimen, 2025. https://github.com/sinancimen

px4_add_module(
	MODULE modules__accel_filtering
	MAIN accel_filtering
	SRCS
		accel_filtering.cpp
		accel_filtering.hpp
	DEPENDS
		px4_work_queue
	)
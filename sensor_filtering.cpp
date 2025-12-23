// Written by Sinan Cimen, 2025. https://github.com/sinancimen

#include "sensor_filtering.h"


void SensorFiltering::Publish()
{
	_sensor_filtered_data.angrate_radps = _angrate_radps;
    _sensor_filtered_data.angacc_radps2 = _angacc_radps2;
    _sensor_filtered_data.accel_mps2 = _accel_mps2;
    _sensor_filtered_data.jerk_mps3 = _jerk_mps3;
	_sensor_filtered_data.timestamp = hrt_absolute_time();
	_sensor_filtered_data_pub.publish(_sensor_filtered_data);
}

int SensorFiltering::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("sensor_filtering",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}
int SensorFiltering::print_status()
{
	//PX4_INFO("");
	return 0;
}

SensorFiltering::~SensorFiltering()
{
	delete[] _angrate_radps;
	delete[] _angacc_radps2;
	delete[] _accel_mps2;
	delete[] _jerk_mps3;
}

int sensor_filtering_main(int argc, char *argv[])
{
	return SensorFiltering::main(argc, argv);
}

SensorFiltering *SensorFiltering::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	SensorFiltering *instance = new SensorFiltering(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

void SensorFiltering::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

SensorFiltering::SensorFiltering(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void SensorFiltering::run()
{
	// Example: run the loop synchronized to the sensor_combined topic publication
	double stepSize(0.00125); // 800 Hz

	// initialize parameters
	parameters_update(true);

    while (!should_exit()) {
        double currentTime(getHiResTime());
        
        PollTopics();
        Step();
        Publish();

        double timeToBeWaited=stepSize-(getHiResTime()-currentTime)-0.001; // no idea why the 0.001 offset is necessary
        if (timeToBeWaited>0) system_usleep(uint32_t(timeToBeWaited*1e6));
    }
}

void SensorFiltering::PollTopics()
{
    if (_vehicle_acceleration_sub.updated()) {
        _vehicle_acceleration_sub.copy(&_vehicle_acceleration);
    }
    if (_vehicle_angular_velocity_sub.updated()) {
        _vehicle_angular_velocity_sub.copy(&_vehicle_angular_velocity);
    }
}

void SensorFiltering::Step()
{
    // Simple example: copy the data directly
    for (int i=0; i<3; i++) {
        _angrate_radps[i] = _vehicle_angular_velocity.xyz[i];
        _angacc_radps2[i] = 0.0; // not available
        _accel_mps2[i] = _vehicle_acceleration.xyz[i];
        _jerk_mps3[i] = 0.0; // not available
    }
}
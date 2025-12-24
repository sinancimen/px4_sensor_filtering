// Written by Sinan Cimen, 2025. https://github.com/sinancimen

#include "gyro_filtering.h"
#include <cstring>


void GyroFiltering::Publish()
{
	std::memcpy(_gyro_filtered_data.angrate_radps, _angrate_radps, sizeof(_angrate_radps));
    std::memcpy(_gyro_filtered_data.angacc_radps2, _angacc_radps2, sizeof(_angacc_radps2));
	_gyro_filtered_data.timestamp = hrt_absolute_time();
	_gyro_filtered_data_pub.publish(_gyro_filtered_data);
}

int GyroFiltering::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("gyro_filtering",
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
int GyroFiltering::print_status()
{
	//PX4_INFO("");
	return 0;
}

GyroFiltering::~GyroFiltering()
{
}

int gyro_filtering_main(int argc, char *argv[])
{
	return GyroFiltering::main(argc, argv);
}

GyroFiltering *GyroFiltering::instantiate(int argc, char *argv[])
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

	GyroFiltering *instance = new GyroFiltering(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

void GyroFiltering::parameters_update(bool force)
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

GyroFiltering::GyroFiltering(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void GyroFiltering::run()
{
	double stepSize(0.0025); // 400 Hz
	parameters_update(true);

	IIR_Coeffs gyro_coeffs = butter_synth(_param_sfilt_gyro_n.get(), _param_sfilt_gyro_freq.get(), 1.0/stepSize);
	IIR_Coeffs angacc_coeffs = butter_synth(_param_sfilt_aacc_n.get(), _param_sfilt_aacc_freq.get(), 1.0/stepSize);

	for (int i = 0; i < 3; ++i) {
		_gyro_filters[i]  = ButterworthIIR(gyro_coeffs);
		_angacc_filters[i] = ButterworthIIR(angacc_coeffs);
	}

    while (!should_exit()) {
        double currentTime(getHiResTime());
        
        PollTopics();
        Step(stepSize);
        Publish();

        double timeToBeWaited=stepSize-(getHiResTime()-currentTime)-0.001; // no idea why the 0.001 offset is necessary
        if (timeToBeWaited>0) system_usleep(uint32_t(timeToBeWaited*1e6));
    }
}

void GyroFiltering::PollTopics()
{
    if (_vehicle_angular_velocity_sub.updated()) {
        _vehicle_angular_velocity_sub.copy(&_vehicle_angular_velocity);
    }
}

void GyroFiltering::Step(double stepSize)
{
    for (int i=0; i<3; i++) {
        _angrate_radps[i] = _gyro_filters[i].process(_vehicle_angular_velocity.xyz[i]);
    }

    if (!_prev_valid) {
        for (int i=0; i<3; ++i) {
            _prev_angrate_radps[i] = _angrate_radps[i];
            _angacc_radps2[i] = 0.0f;
        }
        _prev_valid = true;
        return;
    }

    for (int i=0; i<3; i++) {
		double angacc = (_angrate_radps[i] - _prev_angrate_radps[i]) / stepSize;
        _angacc_radps2[i] = _angacc_filters[i].process(angacc);
        _prev_angrate_radps[i] = _angrate_radps[i];
    }
}
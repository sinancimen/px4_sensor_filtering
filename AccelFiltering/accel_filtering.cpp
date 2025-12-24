// Written by Sinan Cimen, 2025. https://github.com/sinancimen

#include "accel_filtering.h"
#include <cstring>


void AccelFiltering::Publish()
{
	std::memcpy(_accel_filtered_data.accel_mps2, _accel_mps2, sizeof(_accel_mps2));
    std::memcpy(_accel_filtered_data.jerk_mps3, _jerk_mps3, sizeof(_jerk_mps3));
	_accel_filtered_data.timestamp = hrt_absolute_time();
	_accel_filtered_data_pub.publish(_accel_filtered_data);
}

int AccelFiltering::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("accel_filtering",
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
int AccelFiltering::print_status()
{
	//PX4_INFO("");
	return 0;
}

AccelFiltering::~AccelFiltering()
{
}

int accel_filtering_main(int argc, char *argv[])
{
	return AccelFiltering::main(argc, argv);
}

AccelFiltering *AccelFiltering::instantiate(int argc, char *argv[])
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

	AccelFiltering *instance = new AccelFiltering(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

void AccelFiltering::parameters_update(bool force)
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

AccelFiltering::AccelFiltering(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void AccelFiltering::run()
{
	double stepSize(0.0025); // 400 Hz
	parameters_update(true);

	IIR_Coeffs accel_coeffs = butter_synth(_param_sfilt_accel_n.get(), _param_sfilt_accel_freq.get(), 1.0/stepSize);
	IIR_Coeffs jerk_coeffs = butter_synth(_param_sfilt_jrk_n.get(), _param_sfilt_jrk_freq.get(), 1.0/stepSize);

	for (int i = 0; i < 3; ++i) {
		_accel_filters[i] = ButterworthIIR(accel_coeffs);
		_jerk_filters[i]  = ButterworthIIR(jerk_coeffs);
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

void AccelFiltering::PollTopics()
{
    if (_vehicle_acceleration_sub.updated()) {
        _vehicle_acceleration_sub.copy(&_vehicle_acceleration);
    }
}

void AccelFiltering::Step(double stepSize)
{
    for (int i=0; i<3; i++) {
		_accel_mps2[i] = _accel_filters[i].process(_vehicle_acceleration.xyz[i]);
    }

    if (!_prev_valid) {
        for (int i=0; i<3; ++i) {
            _prev_accel_mps2[i] = _accel_mps2[i];
            _jerk_mps3[i] = 0.0f;
        }
        _prev_valid = true;
        return;
    }

    for (int i=0; i<3; i++) {
		double jerk = (_accel_mps2[i] - _prev_accel_mps2[i]) / stepSize;
        _jerk_mps3[i] = _jerk_filters[i].process(jerk);
        _prev_accel_mps2[i] = _accel_mps2[i];
    }
}
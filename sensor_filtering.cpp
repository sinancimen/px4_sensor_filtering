// Written by Sinan Cimen, 2025. https://github.com/sinancimen

#include "sensor_filtering.h"
#include <cstring>


void SensorFiltering::Publish()
{
	std::memcpy(_sensor_filtered_data.angrate_radps, _angrate_radps, sizeof(_angrate_radps));
    std::memcpy(_sensor_filtered_data.angacc_radps2, _angacc_radps2, sizeof(_angacc_radps2));
    std::memcpy(_sensor_filtered_data.accel_mps2, _accel_mps2, sizeof(_accel_mps2));
    std::memcpy(_sensor_filtered_data.jerk_mps3, _jerk_mps3, sizeof(_jerk_mps3));
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
	double stepSize(0.00125); // 800 Hz
	parameters_update(true);

	IIR_Coeffs accel_coeffs = butter_synth(_param_sfilt_accel_n.get(), _param_sfilt_accel_freq.get(), 800.0);
	IIR_Coeffs gyro_coeffs = butter_synth(_param_sfilt_gyro_n.get(), _param_sfilt_gyro_freq.get(), 800.0);
	IIR_Coeffs jerk_coeffs = butter_synth(_param_sfilt_jrk_n.get(), _param_sfilt_jrk_freq.get(), 800.0);
	IIR_Coeffs angacc_coeffs = butter_synth(_param_sfilt_aacc_n.get(), _param_sfilt_aacc_freq.get(), 800.0);

	for (int i = 0; i < 3; ++i) {
		_accel_filters[i] = ButterworthIIR(accel_coeffs);
		_gyro_filters[i]  = ButterworthIIR(gyro_coeffs);
		_jerk_filters[i]  = ButterworthIIR(jerk_coeffs);
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

void SensorFiltering::PollTopics()
{
    if (_vehicle_acceleration_sub.updated()) {
        _vehicle_acceleration_sub.copy(&_vehicle_acceleration);
    }
    if (_vehicle_angular_velocity_sub.updated()) {
        _vehicle_angular_velocity_sub.copy(&_vehicle_angular_velocity);
    }
}

void SensorFiltering::Step(double stepSize)
{
    for (int i=0; i<3; i++) {
        _angrate_radps[i] = _gyro_filters[i].process(_vehicle_angular_velocity.xyz[i]);
		_accel_mps2[i] = _accel_filters[i].process(_vehicle_acceleration.xyz[i]);
    }

    if (!_prev_valid) {
        for (int i=0; i<3; ++i) {
            _prev_angrate_radps[i] = _angrate_radps[i];
            _prev_accel_mps2[i] = _accel_mps2[i];
            _angacc_radps2[i] = 0.0f;
            _jerk_mps3[i] = 0.0f;
        }
        _prev_valid = true;
        return;
    }

    for (int i=0; i<3; i++) {
		double angacc = (_angrate_radps[i] - _prev_angrate_radps[i]) / stepSize;
        _angacc_radps2[i] = _angacc_filters[i].process(angacc);
		double jerk = (_accel_mps2[i] - _prev_accel_mps2[i]) / stepSize;
        _jerk_mps3[i] = _jerk_filters[i].process(jerk);
        _prev_angrate_radps[i] = _angrate_radps[i];
        _prev_accel_mps2[i] = _accel_mps2[i];
    }
}
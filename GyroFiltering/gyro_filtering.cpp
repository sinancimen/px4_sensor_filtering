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
	GyroFiltering *instance = new GyroFiltering();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}
int GyroFiltering::print_status()
{
	//PX4_INFO("");
	return 0;
}

int gyro_filtering_main(int argc, char *argv[])
{
	return GyroFiltering::main(argc, argv);
}

bool GyroFiltering::init()
{
	parameters_update(true);

	IIR_Coeffs gyro_coeffs = butter_synth(_param_sfilt_gyro_n.get(), _param_sfilt_gyro_freq.get()/2.0/M_PI, 1.0/_step_size);
	IIR_Coeffs angacc_coeffs = butter_synth(_param_sfilt_aacc_n.get(), _param_sfilt_aacc_freq.get()/2.0/M_PI, 1.0/_step_size);

	for (int i = 0; i < 3; ++i) {
		_gyro_filters[i]  = ButterworthIIR(gyro_coeffs);
		_angacc_filters[i] = ButterworthIIR(angacc_coeffs);
	}

	// execute Run() on every vehicle_angular_velocity publication
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
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

GyroFiltering::GyroFiltering()
	: ModuleParams(nullptr)
{
}

void GyroFiltering::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}

    if (_vehicle_angular_velocity_sub.updated()) {
        PollTopics();
        Step();
        Publish();
    }
}

void GyroFiltering::PollTopics()
{
    if (_vehicle_angular_velocity_sub.updated()) {
        _vehicle_angular_velocity_sub.copy(&_vehicle_angular_velocity);
    }
}

void GyroFiltering::Step()
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
		double angacc = (_angrate_radps[i] - _prev_angrate_radps[i]) / _step_size;
        _angacc_radps2[i] = _angacc_filters[i].process(angacc);
        _prev_angrate_radps[i] = _angrate_radps[i];
    }
}

int GyroFiltering::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GyroFiltering::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gyro_filtering", "modules");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gyro_filtering_main(int argc, char *argv[])
{
	return GyroFiltering::main(argc, argv);
}
// Written by Sinan Cimen, 2025. https://github.com/sinancimen

#include "accel_filtering.hpp"
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
	AccelFiltering *instance = new AccelFiltering();

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
int AccelFiltering::print_status()
{
	//PX4_INFO("");
	return 0;
}

extern "C" __EXPORT int accel_filtering_main(int argc, char *argv[])
{
	return AccelFiltering::main(argc, argv);
}

bool AccelFiltering::init()
{
	parameters_update(true);

	IIR_Coeffs accel_coeffs = butter_synth(_param_sfilt_accel_n.get(), _param_sfilt_accel_freq.get()/2.0/M_PI, 1.0/_step_size);
	IIR_Coeffs jerk_coeffs = butter_synth(_param_sfilt_jrk_n.get(), _param_sfilt_jrk_freq.get()/2.0/M_PI, 1.0/_step_size);

	for (int i = 0; i < 3; ++i) {
		_accel_filters[i] = ButterworthIIR(accel_coeffs);
		_jerk_filters[i]  = ButterworthIIR(jerk_coeffs);
	}

	// execute Run() on every vehicle_acceleration publication
	if (!_vehicle_acceleration_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
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

AccelFiltering::AccelFiltering()
	: ModuleParams(nullptr), ScheduledWorkItem("accel_filtering", px4::wq_configurations::lp_default)
{
}

void AccelFiltering::Run()
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

    if (_vehicle_acceleration_sub.updated()) {
        PollTopics();
        Step();
        Publish();
	}
}

void AccelFiltering::PollTopics()
{
    if (_vehicle_acceleration_sub.updated()) {
        _vehicle_acceleration_sub.copy(&_vehicle_acceleration);
    }
}

void AccelFiltering::Step()
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
		double jerk = (_accel_mps2[i] - _prev_accel_mps2[i]) / _step_size;
        _jerk_mps3[i] = _jerk_filters[i].process(jerk);
        _prev_accel_mps2[i] = _accel_mps2[i];
    }
}

int AccelFiltering::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AccelFiltering::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("accel_filtering", "modules");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

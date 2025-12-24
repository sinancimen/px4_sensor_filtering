// Written by Sinan Cimen, 2025. https://github.com/sinancimen

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_filtered_data.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include "ButterworthFilt.hpp"

using namespace time_literals;

extern "C" __EXPORT int sensor_filtering_main(int argc, char *argv[]);

class SensorFiltering : public ModuleBase<SensorFiltering>, public ModuleParams
{
public:
	SensorFiltering(int example_param, bool example_flag);

	virtual ~SensorFiltering() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static SensorFiltering *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);
    inline void Publish();
    inline void Step(double stepSize);
    inline void PollTopics();

    float _angrate_radps[3] {0.0, 0.0, 0.0};
    float _angacc_radps2[3] {0.0, 0.0, 0.0};
    float _accel_mps2[3] {0.0, 0.0, 0.0};
    float _jerk_mps3[3] {0.0, 0.0, 0.0};
	float _prev_accel_mps2[3] {0.0, 0.0, 0.0};
	float _prev_angrate_radps[3] {0.0, 0.0, 0.0};

	bool _prev_valid{false};

	ButterworthIIR _angacc_filters[3] {},
	                 _accel_filters[3] {},
	                 _gyro_filters[3] {},
	                 _jerk_filters[3] {};

    DEFINE_PARAMETERS(
		(ParamInt<px4::params::SFILT_ACCEL_N>) _param_sfilt_accel_n,
		(ParamFloat<px4::params::SFILT_ACCEL_FREQ>) _param_sfilt_accel_freq,
		(ParamInt<px4::params::SFILT_GYRO_N>) _param_sfilt_gyro_n,
		(ParamFloat<px4::params::SFILT_GYRO_FREQ>) _param_sfilt_gyro_freq,
		(ParamInt<px4::params::SFILT_AACC_N>) _param_sfilt_aacc_n,
		(ParamFloat<px4::params::SFILT_AACC_FREQ>) _param_sfilt_aacc_freq,
		(ParamInt<px4::params::SFILT_JRK_N>) _param_sfilt_jrk_n,
		(ParamFloat<px4::params::SFILT_JRK_FREQ>) _param_sfilt_jrk_freq
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    uORB::Publication<sensor_filtered_data_s> _sensor_filtered_data_pub{ORB_ID(sensor_filtered_data)};
    sensor_filtered_data_s _sensor_filtered_data{};
    uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
};
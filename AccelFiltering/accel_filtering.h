// Written by Sinan Cimen, 2025. https://github.com/sinancimen

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/accel_filtered_data.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/vehicle_acceleration.h>
#include "../ButterworthFilt.hpp"

using namespace time_literals;

extern "C" __EXPORT int accel_filtering_main(int argc, char *argv[]);

class AccelFiltering : public ModuleBase<AccelFiltering>, public ModuleParams
{
public:
	AccelFiltering(int example_param, bool example_flag);

	virtual ~AccelFiltering() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static AccelFiltering *instantiate(int argc, char *argv[]);

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

    float _accel_mps2[3] {0.0, 0.0, 0.0};
    float _jerk_mps3[3] {0.0, 0.0, 0.0};
	float _prev_accel_mps2[3] {0.0, 0.0, 0.0};

	bool _prev_valid{false};

	ButterworthIIR   _accel_filters[3] {},
	                 _jerk_filters[3] {};

    DEFINE_PARAMETERS(
		(ParamInt<px4::params::SFILT_ACCEL_N>) _param_sfilt_accel_n,
		(ParamFloat<px4::params::SFILT_ACCEL_FREQ>) _param_sfilt_accel_freq,
		(ParamInt<px4::params::SFILT_JRK_N>) _param_sfilt_jrk_n,
		(ParamFloat<px4::params::SFILT_JRK_FREQ>) _param_sfilt_jrk_freq
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    uORB::Publication<accel_filtered_data_s> _accel_filtered_data_pub{ORB_ID(accel_filtered_data)};
    accel_filtered_data_s _accel_filtered_data{};
    uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
};
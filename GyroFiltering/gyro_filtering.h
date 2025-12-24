// Written by Sinan Cimen, 2025. https://github.com/sinancimen

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/gyro_filtered_data.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/vehicle_angular_velocity.h>
#include "../ButterworthFilt.hpp"

using namespace time_literals;

extern "C" __EXPORT int gyro_filtering_main(int argc, char *argv[]);

class GyroFiltering : public ModuleBase<GyroFiltering>, public ModuleParams
{
public:
	GyroFiltering(int example_param, bool example_flag);

	virtual ~GyroFiltering() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static GyroFiltering *instantiate(int argc, char *argv[]);

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
	float _prev_angrate_radps[3] {0.0, 0.0, 0.0};

	bool _prev_valid{false};

	ButterworthIIR _angacc_filters[3] {},
	                 _gyro_filters[3] {};

    DEFINE_PARAMETERS(
		(ParamInt<px4::params::SFILT_GYRO_N>) _param_sfilt_gyro_n,
		(ParamFloat<px4::params::SFILT_GYRO_FREQ>) _param_sfilt_gyro_freq,
		(ParamInt<px4::params::SFILT_AACC_N>) _param_sfilt_aacc_n,
		(ParamFloat<px4::params::SFILT_AACC_FREQ>) _param_sfilt_aacc_freq
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    uORB::Publication<gyro_filtered_data_s> _gyro_filtered_data_pub{ORB_ID(gyro_filtered_data)};
    gyro_filtered_data_s _gyro_filtered_data{};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
};
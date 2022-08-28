
#pragma once

#include <inttypes.h>
#include <stdint.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_motors.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/arm_setpoint.h>
#include <uORB/topics/arm_state.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

class ArmController : public ModuleBase<ArmController>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ArmController();

	~ArmController() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int init();

private:
	void Run() override;

	arm_setpoint_s _setpoint{};

	float _integral{0};
	float _max_i_sum = INFINITY;
	static constexpr float MIN_I_SUM = 0.0f;

	float _prev_error{0};
	hrt_abstime _prev_timestamp{0};

	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::SubscriptionCallbackWorkItem _arm_state_sub{this, ORB_ID(arm_state)};
	uORB::Subscription _arm_setpoint_sub{ORB_ID(arm_setpoint)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::ARM_CTRL_P>) _param_arm_ctrl_p,
		(ParamFloat<px4::params::ARM_CTRL_I>) _param_arm_ctrl_i,
		(ParamFloat<px4::params::ARM_CTRL_D>) _param_arm_ctrl_d
	)
};

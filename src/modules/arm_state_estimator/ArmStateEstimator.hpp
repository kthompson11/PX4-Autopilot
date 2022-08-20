
#include <inttypes.h>
#include <stdint.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/arm_state.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/adc_report.h>

using namespace time_literals;

class ArmStateEstimator : public ModuleBase<ArmStateEstimator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ArmStateEstimator();

	~ArmStateEstimator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int init();

private:
	void Run() override;

	float calculate_arm_angle(const adc_report_s &report);

	int _adc_sensor_index{-1};

	uORB::Publication<arm_state_s> _arm_state_pub{ORB_ID(arm_state)};

	uORB::SubscriptionCallbackWorkItem _adc_report_sub{this, ORB_ID(adc_report)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ARM_STATE_CH>) _param_arm_state_ch,
		(ParamFloat<px4::params::ARM_STATE_OFFSET>) _param_arm_state_offset,
		(ParamFloat<px4::params::ARM_STATE_SCALE>) _param_arm_state_scale
	)
};

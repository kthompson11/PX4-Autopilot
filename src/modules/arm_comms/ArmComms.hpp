
#pragma once

#include <inttypes.h>
#include <stdint.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/arm_setpoint.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/arm_state.h>

using namespace time_literals;

class ArmComms : public ModuleBase<ArmComms>, public px4::ScheduledWorkItem
{
public:
	ArmComms();

	~ArmComms() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int init(int argc, char *argv[]);

private:
	void Run() override;

	const char *_serial_port_path{nullptr};
	int _serial_port_fd{-1};
	uint8_t _rx_data[40];
	int _rx_index{0};
	int open_serial_port();

	void transmit_arm_state();

	void receive_messages();

	void publish_setpoints();
	static constexpr int SETPOINT_LENGTH = 5;

	static constexpr hrt_abstime RUN_INTERVAL_US = 50_ms;

	// Update interval for publishing the copter arm state over the serial port
	static constexpr hrt_abstime STATE_UPDATE_INTERVAL_US = 100_ms;
	hrt_abstime _last_state_update{};

	// Timeout intervals for the actuator_armed publication
	static constexpr hrt_abstime ARMED_MAX_INTERVAL_US = 500_ms;
	hrt_abstime _last_armed_publication{};

	uORB::Publication<actuator_armed_s> _actuator_armed{ORB_ID(actuator_armed)};
	uORB::Publication<arm_setpoint_s> _arm_setpoint_pub{ORB_ID(arm_setpoint)};

	uORB::Subscription _arm_state_sub{ORB_ID(arm_state)};
};

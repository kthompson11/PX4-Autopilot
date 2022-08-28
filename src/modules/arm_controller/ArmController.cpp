
#include "ArmController.hpp"

ArmController::ArmController() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

ArmController::~ArmController()
{
}

int ArmController::init()
{
	_arm_state_sub.registerCallback();

	return PX4_OK;
}

void ArmController::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		_max_i_sum = 1.0f / _param_arm_ctrl_i.get();
	}

	// Get new state
	arm_state_s state{};

	if (!_arm_state_sub.update(&state)) {
		return;
	}

	// Update setpoint if available
	_arm_setpoint_sub.copy(&_setpoint);

	// Run PID controller
	if (_setpoint.timestamp == 0) {
		return;
	}

	float error = _setpoint.angle - state.angle;

	if (_prev_timestamp == 0) {
		_prev_error = error;
		_prev_timestamp = state.timestamp;
	}

	const float delta_t = 1e-6f * (state.timestamp - _prev_timestamp);

	/********** Proportional Term **********/
	const float p_term = _param_arm_ctrl_p.get() * error;

	/********** Integral Term **********/
	_integral += error * delta_t;

	// Anti windup
	if (_integral > _max_i_sum) {
		_integral = _max_i_sum;

	} else if (_integral < MIN_I_SUM) {
		_integral = MIN_I_SUM;
	}

	actuator_armed_s armed{};
	_actuator_armed_sub.copy(&armed);

	if (!armed.armed) {
		_integral = 0;
	}

	const float i_term = _param_arm_ctrl_i.get() * _integral;

	/********** Derivative Term **********/
	const float d_term = _param_arm_ctrl_d.get() * (error - _prev_error) / delta_t;

	_prev_error = error;

	float output = math::constrain(p_term + i_term + d_term, 0.0f, 1.0f);

	// Publish output
	actuator_motors_s motors{};
	motors.timestamp = hrt_absolute_time();
	// motors.timestamp_sample = // TODO
	motors.control[0] = output;

	for (size_t i = 1; i < arraySize(actuator_motors_s::control); ++i) {
		motors.control[i] = NAN;
	}

	_actuator_motors_pub.publish(motors);
}

int ArmController::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ArmController::task_spawn(int argc, char *argv[])
{
	ArmController *instance = new ArmController();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
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

int ArmController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
copter arm state estimator

)DESCR_STR");

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int arm_controller_main(int argc, char *argv[])
{
	return ArmController::main(argc, argv);
}

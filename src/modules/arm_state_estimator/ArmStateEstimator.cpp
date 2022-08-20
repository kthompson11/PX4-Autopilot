
#include "ArmStateEstimator.hpp"

ArmStateEstimator::ArmStateEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

ArmStateEstimator::~ArmStateEstimator()
{
}

int ArmStateEstimator::init()
{
	if (_param_arm_state_ch.get() < 0) {
		PX4_ERR("The copter arm state ADC channel must be defined.");
		return PX4_ERROR;
	}

	_adc_report_sub.registerCallback();

	return PX4_OK;
}

void ArmStateEstimator::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	adc_report_s report{};
	if (!_adc_report_sub.update(&report)) {
		return;
	}

	const float theta = calculate_arm_angle(report);
	const float degrees = math::degrees(theta);
}

float ArmStateEstimator::calculate_arm_angle(const adc_report_s &report)
{
	float angle = NAN;

	// Look up the index of the chosen ADC channel
	if (_adc_sensor_index < 0) {
		for (unsigned i = 0; i < arraySize(adc_report_s::channel_id); ++i) {
			if (report.channel_id[i] == _param_arm_state_ch.get()) {
				_adc_sensor_index = i;
			}
		}
	}

	// Calculate the arm angle
	if (_adc_sensor_index >= 0) {
		const int32_t adc_counts = report.raw_data[_adc_sensor_index];
		const float voltage = report.v_ref * adc_counts / report.resolution;

		const float offset = _param_arm_state_offset.get();
		const float scale = _param_arm_state_scale.get();
		angle = (voltage - offset) * scale;
	}

	return angle;
}

int ArmStateEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ArmStateEstimator::task_spawn(int argc, char *argv[])
{
	ArmStateEstimator *instance = new ArmStateEstimator();

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

int ArmStateEstimator::print_usage(const char *reason)
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

extern "C" __EXPORT int arm_state_estimator_main(int argc, char *argv[])
{
	return ArmStateEstimator::main(argc, argv);
}

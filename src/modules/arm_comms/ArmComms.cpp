
#include <fcntl.h>
#include <termios.h>

#include <px4_platform_common/getopt.h>

#include "ArmComms.hpp"
#include "protocol/cobs.h"

ArmComms::ArmComms() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

ArmComms::~ArmComms()
{
}

int ArmComms::init(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			_serial_port_path = myoptarg;
			break;

		default:
			print_usage();
			return PX4_ERROR;
		}
	}

	ScheduleOnInterval(RUN_INTERVAL_US);

	return PX4_OK;
}

void ArmComms::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (_serial_port_fd < 0) {
		int ret = open_serial_port();

		if (ret != PX4_OK) {
			exit_and_cleanup();
			return;
		}
	}

	transmit_arm_state();
	receive_messages();
}

int ArmComms::open_serial_port()
{
	/* open fd */
	_serial_port_fd = ::open(_serial_port_path, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_serial_port_fd < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_serial_port_fd, &uart_config);

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* set baud rate */
	const speed_t baud_rate = B57600; // TODO: make baud rate configurable

	if ((termios_state = cfsetispeed(&uart_config, baud_rate)) < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
	}

	if ((termios_state = cfsetospeed(&uart_config, baud_rate)) < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
	}

	if ((termios_state = tcsetattr(_serial_port_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
	}

	return PX4_OK;
}

void ArmComms::transmit_arm_state()
{
	uint8_t buf_unstuffed[30];
	uint8_t buf_stuffed[33];
	int i = 0;
	arm_state_s state{};

	if (_arm_state_sub.update(&state)) {
		// Add arm angle
		memcpy(buf_unstuffed, &state.angle, sizeof(state.angle));
		i += sizeof(state.angle);

		// Byte stuff and add delimiter
		int array_size = cobs_stuff(buf_unstuffed, buf_stuffed, i);

		// Write to serial port
		::write(_serial_port_fd, buf_stuffed, array_size);
	}
}

void ArmComms::receive_messages()
{
	// TODO: reimplement circular and delimited buffer

	hrt_abstime now = hrt_absolute_time();

	uint8_t buf[10];
	ssize_t bytes_read = ::read(_serial_port_fd, buf, sizeof(buf));

	while (bytes_read > 0) {
		// Copy data into buffer
		memcpy(_rx_data + _rx_index, buf, bytes_read);
		_rx_index += bytes_read;

		// Find index of delimiter
		int i_delim = -1;

		for (int i = 0; i < _rx_index; ++i) {
			if (_rx_data[i] == 0) {
				i_delim = i;
				break;
			}
		}

		if (i_delim >= 0) {
			// Extract the message and remove it from the rx buffer
			uint8_t stuffed_message[40];
			memcpy(stuffed_message, _rx_data, i_delim + 1);

			for (int i = 0; i < _rx_index - i_delim - 1; ++i) {
				_rx_data[i] = _rx_data[i_delim + 1 + i];
			}

			_rx_index = _rx_index - i_delim - 1;

			// Decode the message
			uint8_t unstuffed_message[40];
			int unstuffed_size = cobs_unstuff(stuffed_message, unstuffed_message, i_delim + 1);

			if (unstuffed_size == SETPOINT_LENGTH) {
				bool armed = unstuffed_message[0];
				float setpoint;
				memcpy(&setpoint, unstuffed_message + 1, sizeof(float));

				actuator_armed_s actuator_armed{};
				actuator_armed.timestamp = now;
				actuator_armed.armed = armed;
				_actuator_armed.publish(actuator_armed);
				_last_armed_publication = now;

				arm_setpoint_s arm_setpoint{};
				arm_setpoint.timestamp = now;
				arm_setpoint.angle = setpoint;
				_arm_setpoint_pub.publish(arm_setpoint);
			}
		}

		if (_rx_index >= 20) {
			// Clear the buffer if it gets too long without a valid message
			_rx_index = 0;
		}

		bytes_read = ::read(_serial_port_fd, _rx_data, sizeof(buf));
	}

	// Check for timeout on actuator_armed publication
	if (hrt_elapsed_time(&_last_armed_publication) > ARMED_MAX_INTERVAL_US) {
		actuator_armed_s armed{};
		armed.timestamp = now;
		armed.armed = false;
		_actuator_armed.publish(armed);
		_last_armed_publication = now;
	}
}

int ArmComms::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ArmComms::task_spawn(int argc, char *argv[])
{
	ArmComms *instance = new ArmComms();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init(argc, argv) == PX4_OK) {
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

int ArmComms::print_usage(const char *reason)
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

extern "C" __EXPORT int arm_comms_main(int argc, char *argv[])
{
	return ArmComms::main(argc, argv);
}

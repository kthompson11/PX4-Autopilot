
px4_add_module(
	MODULE modules__arm_comms
	MAIN arm_comms
	MODULE_CONFIG
		module.yaml
	SRCS
		ArmComms.cpp
		ArmComms.hpp
		protocol/cobs.c
	DEPENDS
		px4_work_queue
	)

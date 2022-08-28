
px4_add_module(
	MODULE modules__arm_controller
	MAIN arm_controller
	MODULE_CONFIG
		module.yaml
	SRCS
		ArmController.cpp
		ArmController.hpp
	DEPENDS
		px4_work_queue
	)

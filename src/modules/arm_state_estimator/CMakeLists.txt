
px4_add_module(
	MODULE modules_arm_state_estimator
	MAIN arm_state_estimator
	MODULE_CONFIG
		module.yaml
	SRCS
		ArmStateEstimator.cpp
		ArmStateEstimator.hpp
	DEPENDS
		px4_work_queue
	)

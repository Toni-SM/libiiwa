package application;

public enum LibIiwaEnum{
	
	// APPLICATION ERRORS
	INVALID_CONFIGURATION_ERROR(-16),
	VALIDATION_FOR_IMPEDANCE_ERROR(-15),
	ASYNCHRONOUS_MOTION_ERROR(-14),
	SYNCHRONOUS_MOTION_ERROR(-13),
	INVALID_JOINT_ERROR(-12),
	VALUE_ERROR(-11),
	ERROR(-10),

	// communication errors
	TIMEOUT(-3),
	INVALID_NUMBER_OF_BYTES(-2),
	EXCEPTION(-1),
	
	// empty commands
	COMMAND_PASS(0),
	
	// communication modes
	COMMUNICATION_MODE(0),  // empty
	COMMUNICATION_MODE_ON_DEMAND(11),
	COMMUNICATION_MODE_PERIODICAL(12),
	
	// motion types
	MOTION_TYPE(0),  // empty
	MOTION_TYPE_PTP(21), 
	MOTION_TYPE_LIN(22),
	MOTION_TYPE_LIN_REL(23),
	MOTION_TYPE_CIRC(24),
	
	// control interfaces
	CONTROL_INTERFACE(0),  // empty
	CONTROL_INTERFACE_STANDARD(31), 
	CONTROL_INTERFACE_SERVO(32), 
	CONTROL_INTERFACE_FRI(33),
	
	// control modes
	CONTROL_MODE(0),  // empty
	CONTROL_MODE_POSITION(41),
	CONTROL_MODE_JOINT_IMPEDANCE(42),
	CONTROL_MODE_CARTESIAN_IMPEDANCE(43),
	CONTROL_MODE_CARTESIAN_SINE_IMPEDANCE(44),
	
	// execution type
	EXECUTION_TYPE(0),  // empty
	EXECUTION_TYPE_ASYNCHRONOUS(51),
	EXECUTION_TYPE_SYNCHRONOUS(52),

	// control command
	COMMAND_JOINT_POSITION(101),
	COMMAND_CARTESIAN_POSE(102),
	COMMAND_CIRC_MOTION(103),

	// configuration commands (limits)
	COMMAND_SET_DESIRED_JOINT_VELOCITY_REL(201),
	COMMAND_SET_DESIRED_JOINT_ACCELERATION_REL(202),
	COMMAND_SET_DESIRED_JOINT_JERK_REL(203),
	COMMAND_SET_DESIRED_CARTESIAN_VELOCITY(204),
	COMMAND_SET_DESIRED_CARTESIAN_ACCELERATION(205),
	COMMAND_SET_DESIRED_CARTESIAN_JERK(206),

	// configuration commands (conditions)
	COMMAND_SET_FORCE_CONDITION(211),
	COMMAND_SET_JOINT_TORQUE_CONDITION(212),
	
	// configuration commands (impedance control)
	COMMAND_SET_CARTESIAN_STIFFNESS(221),
	COMMAND_SET_CARTESIAN_DAMPING(222),
	COMMAND_SET_CARTESIAN_ADDITIONAL_CONTROL_FORCE(223),
	COMMAND_SET_JOINT_STIFFNESS(224),
	COMMAND_SET_JOINT_DAMPING(225),

	// configuration commands (motion and control)
	COMMAND_SET_COMMUNICATION_MODE(301),
	COMMAND_SET_CONTROL_INTERFACE(302),
	COMMAND_SET_MOTION_TYPE(303),
	COMMAND_SET_CONTROL_MODE(304),
	COMMAND_SET_EXECUTION_TYPE(305);
	
	
	private int code;
	
	LibIiwaEnum(double code){
		this.setCode(code);
	}

	public int getCode() {
		return code;
	}

	public void setCode(double code) {
		this.code = (int)code;
	}
}

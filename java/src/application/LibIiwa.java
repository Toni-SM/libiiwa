package application;

import com.kuka.common.ThreadUtil;

import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.connectivity.motionModel.smartServoLIN.ISmartServoLINRuntime;

import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;

import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;

import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.Frame;

import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;

import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;

import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;

import java.util.Arrays;

import application.LibIiwaEnum;
import application.LibIiwaCommunication;


/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application life-cycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class LibIiwa extends RoboticsAPIApplication {

	// ===========================================================
	// CONSTANTS
	// ===========================================================

	private static final String API_VERSION = "0.1.0-beta";

	private static final int STATE_COMMAND_STATUS = 0;  		// vector(1)
	private static final int STATE_ROBOT_STATUS = 1;  			// vector(1)
	private static final int STATE_JOINT_POSITION = 2;  		// vector(7)
	private static final int STATE_JOINT_VELOCITY = 9;  		// vector(7)
	private static final int STATE_JOINT_ACCELERATION = 16;  	// vector(7)
	private static final int STATE_JOINT_TORQUE = 23;  			// vector(7)
	private static final int STATE_CARTESIAN_POSITION = 30;  	// vector(3)
	private static final int STATE_CARTESIAN_ORIENTATION = 33; 	// vector(3)
	private static final int STATE_CARTESIAN_FORCE = 36;     	// vector(3)
	private static final int STATE_CARTESIAN_TORQUE = 39;    	// vector(3)

	private static final int STATE_LENGTH = 1 * 2 + 7 * 4 + 6 + 6; 	// vector(42)
	private static final int COMMAND_LENGTH = 8;             		// vector(8)

	// ===========================================================
	// VARIABLES
	// ===========================================================

	// settings 
	private boolean VERBOSE = false;

	// communication
	private LibIiwaCommunication propCommunication;
	private LibIiwaEnum propCommunicationMode = LibIiwaEnum.COMMUNICATION_MODE_ON_DEMAND;

	private String TOOL_NAME = "tool";
	private String FRAME_NAME = "/World";
	private String CONTROLLER_NAME = "KUKA_Sunrise_Cabinet_1";

	// robot and tool
	private IApplicationData propApplicationData = null;
	private LBR lbr;
	private Tool tool;

	private double propCurrentState[];
	private boolean propShouldContinue = false;	// TODO: rename

	private JointPosition propMinJointPositionLimits;
	private JointPosition propMaxJointPositionLimits;
	private double propMinJointTorqueLimits[];
	private double propMaxJointTorqueLimits[];
	
	// errors
	private LibIiwaEnum enumLastError = LibIiwaEnum.ERROR;

	// configuration
	private LibIiwaEnum enumMotionType = LibIiwaEnum.MOTION_TYPE_PTP;
	private LibIiwaEnum enumControlInterface = LibIiwaEnum.CONTROL_INTERFACE_STANDARD;
	private LibIiwaEnum enumControlMode = LibIiwaEnum.CONTROL_MODE_POSITION;
	private LibIiwaEnum enumControlExecutionType = LibIiwaEnum.EXECUTION_TYPE_ASYNCHRONOUS;

	private IMotionControlMode propCurrentControlMode = null;
	private IMotionContainer propCurrentMotionContainer = null;
	private ISmartServoRuntime propCurrentSmartServoRuntime = null;
	private ISmartServoLINRuntime propCurrentSmartServoLINRuntime = null;
	
	private PositionControlMode propControlModePosition = null;
	private JointImpedanceControlMode propControlModeJointImpedance = null;
	private CartesianImpedanceControlMode propControlModeCartesianImpedance = null;
	private CartesianSineImpedanceControlMode propControlModeCartesianSineImpedance = null;

	private double propDesiredJointVelocityRel = 1.0;
	private double propDesiredJointAccelerationRel = 1.0;
	private double propDesiredJointJerkRel = 1.0;
	private double propDesiredCartesianVelocity = 1000;
	private double propDesiredCartesianAcceleration = 1000;
	private double propDesiredCartesianJerk = 1000;
	
	private ForceCondition propForceConditionX;
	private ForceCondition propForceConditionY;
	private ForceCondition propForceConditionZ;

	private double propDesiredTorqueSensitivity = 100; // TODO: rename
	private double propMinimumTrajectoryExecutionTime = 0.001;

	private boolean propOverwriteMotion = true;

	// ===========================================================
	// INTERNAL METHODS
	// ===========================================================

	private void methStopAndResetMotion() {
		// stop motions
		if (propCurrentMotionContainer != null)
			propCurrentMotionContainer.cancel();
		if (propCurrentSmartServoRuntime != null)
			propCurrentSmartServoRuntime.stopMotion();
		if (propCurrentSmartServoLINRuntime != null)
			propCurrentSmartServoLINRuntime.stopMotion();

		// reset variables
		propCurrentMotionContainer = null;
		propCurrentSmartServoRuntime = null;
		propCurrentSmartServoLINRuntime = null;
	}

	private void methInitializeSmartServo() {
		if (this.propCurrentSmartServoRuntime != null)
			return;

		SmartServo smartServo = new SmartServo(lbr.getCurrentJointPosition());

		smartServo.setJointVelocityRel(this.propDesiredJointVelocityRel);
		smartServo.setJointAccelerationRel(this.propDesiredJointAccelerationRel);

		smartServo.setMinimumTrajectoryExecutionTime(this.propMinimumTrajectoryExecutionTime);
		SmartServo.validateForImpedanceMode(lbr);
		
		lbr.getFlange().moveAsync(smartServo.setMode(this.propCurrentControlMode));

		this.propCurrentSmartServoRuntime = smartServo.getRuntime(true);   
	}

	private void methInitializeSmartServoLIN() {
		if (this.propCurrentSmartServoLINRuntime != null)
			return;

		SmartServoLIN smartServoLIN = new SmartServoLIN(lbr.getCurrentCartesianPosition(lbr.getFlange()));

		//		smartServoLIN.setMaxTranslationVelocity(...);
		//		smartServoLIN.setMaxTranslationAcceleration(...);
		//		smartServoLIN.setMaxOrientationVelocity(...);
		//		smartServoLIN.setMaxOrientationAcceleration(...);
		//		smartServoLIN.setMaxNullSpaceVelocity(...);
		//		smartServoLIN.setMaxNullSpaceAcceleration(...);

		smartServoLIN.setMinimumTrajectoryExecutionTime(this.propMinimumTrajectoryExecutionTime);
		SmartServoLIN.validateForImpedanceMode(lbr);
		lbr.getFlange().moveAsync(smartServoLIN.setMode(this.propCurrentControlMode));

		this.propCurrentSmartServoLINRuntime = smartServoLIN.getRuntime(true);   
	}

	// ===========================================================
	// INTERNAL CONTROL METHODS
	// ===========================================================

	private boolean methMoveStandard(MotionBatch motionBatch) {
		if (lbr.isReadyToMove()) {
			// set limits
			motionBatch.setJointVelocityRel(propDesiredJointVelocityRel).setJointAccelerationRel(propDesiredJointAccelerationRel)
				.breakWhen(this.propForceConditionX.or(this.propForceConditionY).or(this.propForceConditionZ));
			// overwrite motion
			if (propCurrentMotionContainer != null) {
				if (!propCurrentMotionContainer.isFinished()) {
					if (!propOverwriteMotion)
						return false;
					propCurrentMotionContainer.cancel();
				}
			}
			// execute motion
			propCurrentMotionContainer = lbr.moveAsync(motionBatch);
			return true;
		}
		return false;
	}

	private boolean methMoveSmartServo(JointPosition jointPosition) {
		if (lbr.isReadyToMove()) {
			methInitializeSmartServo();
			propCurrentSmartServoRuntime.setDestination(jointPosition);
			return true;
		}
		return false;
	}

	private boolean methMoveSmartServo(Frame frame) {
		if (lbr.isReadyToMove()) {
			methInitializeSmartServo();
			propCurrentSmartServoRuntime.setDestination(frame);
			return true;
		}
		return false;
	}

	private boolean methMoveSmartServoLIN(Frame frame) {
		if (lbr.isReadyToMove()) {
			methInitializeSmartServoLIN();
			propCurrentSmartServoLINRuntime.setDestination(frame);
			return true;
		}
		return false;
	}

	// ===========================================================
	// CONFIGURATION METHODS
	// ===========================================================

	// TODO: setBlendingRel, setBlendingCart, setBlendingOri, 
	// TODO: setOrientationType, setOrientationReferenceSystem
	// TODO: setJointVelocityRel, setJointAccelerationRel, setJointJerkRel for each axis (double[] = {1, 1, 1, 1, 1, 1, 1})
	
	/**
	 * Define the axis-specific relative velocity (% of maximum velocity)
	 * <p>
	 * velocity: [0.0, 1.0]
	 * 
	 * @param jointVelocityRel relative maximum velocity
	 * @return true if it is valid, otherwise false
	 */
	public boolean methSetDesiredJointVelocityRel(double jointVelocityRel) {
		if (jointVelocityRel < 0 || jointVelocityRel > 1.0) {
			if (VERBOSE) getLogger().warn("Invalid joint velocity: " + jointVelocityRel);
			return false;
		}
		this.propDesiredJointVelocityRel = jointVelocityRel;
		if (VERBOSE) getLogger().info("Desired joint velocity: " + jointVelocityRel);
		return true;
	}

	/**
	 * Define the axis-specific relative acceleration (% of maximum acceleration)
	 * <p>
	 * acceleration: [0.0, 1.0]
	 * 
	 * @param jointAccelerationRel relative maximum acceleration
	 * @return true if it is valid, otherwise false
	 */
	public boolean methSetDesiredJointAccelerationRel(double jointAccelerationRel) {
		if (jointAccelerationRel < 0 || jointAccelerationRel > 1.0) {
			if (VERBOSE) getLogger().warn("Invalid joint acceleration: " + jointAccelerationRel);
			return false;
		}
		this.propDesiredJointAccelerationRel = jointAccelerationRel;
		if (VERBOSE) getLogger().info("Desired joint acceleration: " + jointAccelerationRel);
		return true;
	}

	/**
	 * Define the axis-specific relative jerk (% of maximum jerk)
	 * <p>
	 * jerk: [0.0, 1.0]
	 * 
	 * @param jointJerkRel relative maximum jerk
	 * @return true if it is valid, otherwise false
	 */
	public boolean methSetDesiredJointJerkRel(double jointJerkRel) {
		if (jointJerkRel < 0 || jointJerkRel > 1.0) {
			if (VERBOSE) getLogger().warn("Invalid joint jerk: " + jointJerkRel);
			return false;
		}
		this.propDesiredJointJerkRel = jointJerkRel;
		if (VERBOSE) getLogger().info("Desired joint jerk: " + jointJerkRel);
		return true;
	}

	/**
	 * Define the absolute Cartesian velocity (mm/s)
	 * <p>
	 * velocity: (0.0, Inf)
	 * 
	 * @param cartesianVelocity desired maximum Cartesian velocity
	 * @return true if it is valid, otherwise false
	 */
	public boolean methSetDesiredCartesianVelocity(double cartesianVelocity) {
		if (cartesianVelocity <= 0) {
			if (VERBOSE) getLogger().warn("Invalid cartesian velocity: " + cartesianVelocity);
			return false;
		}
		this.propDesiredCartesianVelocity = cartesianVelocity;
		if (VERBOSE) getLogger().info("Desired cartesian velocity: " + cartesianVelocity);
		return true;
	}

	/**
	 * Define the absolute Cartesian acceleration (mm/s^2)
	 * <p>
	 * acceleration: (0.0, Inf)
	 * 
	 * @param cartesianAcceleration desired maximum Cartesian acceleration
	 * @return true if it is valid, otherwise false
	 */
	public boolean methSetDesiredCartesianAcceleration(double cartesianAcceleration) {
		if (cartesianAcceleration <= 0) {
			if (VERBOSE) getLogger().warn("Invalid cartesian acceleration: " + cartesianAcceleration);
			return false;
		}
		this.propDesiredCartesianAcceleration = cartesianAcceleration;
		if (VERBOSE) getLogger().info("Desired cartesian acceleration: " + cartesianAcceleration);
		return true;
	}

	/**
	 * Define the absolute Cartesian jerk (mm/s^3)
	 * <p>
	 * jerk: (0.0, Inf)
	 * 
	 * @param cartesianJerk desired maximum Cartesian jerk
	 * @return true if it is valid, otherwise false
	 */
	public boolean methSetDesiredCartesianJerk(double cartesianJerk) {
		if (cartesianJerk <= 0) {
			if (VERBOSE) getLogger().warn("Invalid cartesian jerk: " + cartesianJerk);
			return false;
		}
		this.propDesiredCartesianJerk = cartesianJerk;
		if (VERBOSE) getLogger().info("Desired cartesian jerk: " + cartesianJerk);
		return true;
	}
	
	/**
	 * Define the force condition (N)
	 * <p>
	 * threshold: [0.0, Inf)
	 * tolerance: (0.0, Inf)
	 * 
	 * @param condition maximum magnitude of force (3) and maximum permissible inaccuracy (3) 
	 * @return true if it is valid, otherwise false
	 */
	public boolean methSetForceCondition(double[] condition) {
		// threshold
		for (int i = 0; i < 3; i++) 
			if (condition[i] < 0) {
				if (VERBOSE) getLogger().warn("Invalid force threshold(3) / tolerance(3) condition: " + condition);
			    return false;
			}
		// tolerance
		for (int i = 3; i < 6; i++) 
			if (condition[i] <= 0) {
				if (VERBOSE) getLogger().warn("Invalid force threshold(3) / tolerance(3) condition: " + condition);
			    return false;
			}
		// set the condition
		this.propForceConditionX = ForceCondition.createNormalForceCondition(lbr.getFlange(), CoordinateAxis.X, condition[0], condition[3]);
		this.propForceConditionY = ForceCondition.createNormalForceCondition(lbr.getFlange(), CoordinateAxis.Y, condition[1], condition[4]);
		this.propForceConditionZ = ForceCondition.createNormalForceCondition(lbr.getFlange(), CoordinateAxis.Z, condition[2], condition[5]);
		if (VERBOSE) getLogger().info("Force threshold(3) / tolerance(3) condition:" + condition);
		return true;
	}
	
	/**
	 * Define the Cartesian impedance control stiffness (translational: Nm, rotational: Nm/rad)
	 * <p>
	 * translational: [0.0, 5000.0] (default: 2000.0)
	 * <p>
	 * rotational: [0.0, 300.0] (default: 200.0)
	 * 
	 * @param stiffness translational (3) and rotational (3) stiffness 
	 * @return true
	 */
	public boolean methSetCartesianStiffness(double[] stiffness) {
		// translational
		if (stiffness[0] >= 0.0 && stiffness[0] <= 5000.0)
			this.propControlModeCartesianImpedance.parametrize(CartDOF.X).setStiffness(stiffness[0]);
		if (stiffness[1] >= 0.0 && stiffness[1] <= 5000.0)
			this.propControlModeCartesianImpedance.parametrize(CartDOF.Y).setStiffness(stiffness[1]);
		if (stiffness[2] >= 0.0 && stiffness[2] <= 5000.0)
			this.propControlModeCartesianImpedance.parametrize(CartDOF.Z).setStiffness(stiffness[2]);
		// rotational
		if (stiffness[3] >= 0.0 && stiffness[3] <= 300.0)
			this.propControlModeCartesianImpedance.parametrize(CartDOF.A).setStiffness(stiffness[3]);
		if (stiffness[4] >= 0.0 && stiffness[4] <= 300.0)
			this.propControlModeCartesianImpedance.parametrize(CartDOF.B).setStiffness(stiffness[4]);
		if (stiffness[5] >= 0.0 && stiffness[5] <= 300.0)
			this.propControlModeCartesianImpedance.parametrize(CartDOF.C).setStiffness(stiffness[5]);
		if (VERBOSE) getLogger().info("Cartesian stiffness: " + stiffness);
		return true;
	}
	
	/**
	 * Define the Cartesian impedance control damping
	 * <p>
	 * translational: [0.1, 1.0] (default: 0.7)
	 * <p>
	 * rotational: [0.1, 1.0] (default: 0.7)
	 * 
	 * @param damping translational (3) and rotational (3) damping 
	 * @return true
	 */
	public boolean methSetCartesianDamping(double[] damping) {
		// translational
		if (damping[0] >= 0.1 && damping[0] <= 1.0)
			this.propControlModeCartesianImpedance.parametrize(CartDOF.X).setDamping(damping[0]);
		if (damping[1] >= 0.1 && damping[1] <= 1.0)
			this.propControlModeCartesianImpedance.parametrize(CartDOF.Y).setDamping(damping[1]);
		if (damping[2] >= 0.1 && damping[2] <= 1.0)
			this.propControlModeCartesianImpedance.parametrize(CartDOF.Z).setDamping(damping[2]);
		// rotational
		if (damping[3] >= 0.1 && damping[3] <= 1.0)
			this.propControlModeCartesianImpedance.parametrize(CartDOF.A).setDamping(damping[3]);
		if (damping[4] >= 0.1 && damping[4] <= 1.0)
			this.propControlModeCartesianImpedance.parametrize(CartDOF.B).setDamping(damping[4]);
		if (damping[5] >= 0.1 && damping[5] <= 1.0)
			this.propControlModeCartesianImpedance.parametrize(CartDOF.C).setDamping(damping[5]);
		if (VERBOSE) getLogger().info("Cartesian damping: " + damping);
		return true;
	}
	
	/**
	 * Define the Cartesian impedance control additional control force (translational: N, rotational: Nm)
	 * <p>
	 * 
	 * @param damping translational (3) and rotational (3) additional control force 
	 * @return true
	 */
	public boolean methSetCartesianAdditionalControlForce(double[] force) {
		// translational
		this.propControlModeCartesianImpedance.parametrize(CartDOF.X).setAdditionalControlForce(force[0]);
		this.propControlModeCartesianImpedance.parametrize(CartDOF.Y).setAdditionalControlForce(force[1]);
		this.propControlModeCartesianImpedance.parametrize(CartDOF.Z).setAdditionalControlForce(force[2]);
		// rotational
		this.propControlModeCartesianImpedance.parametrize(CartDOF.A).setAdditionalControlForce(force[3]);
		this.propControlModeCartesianImpedance.parametrize(CartDOF.B).setAdditionalControlForce(force[4]);
		this.propControlModeCartesianImpedance.parametrize(CartDOF.C).setAdditionalControlForce(force[5]);
		if (VERBOSE) getLogger().info("Cartesian additional control force: " + force);
		return true;
	}
	
	/**
	 * Define the joint impedance control stiffness (Nm/rad)
	 * <p>
	 * stiffness: [0, Inf)
	 * 
	 * @param stiffness joint stiffness 
	 * @return true
	 */
	public boolean methSetJointStiffness(double[] stiffness) {
		double[] currentStiffness = this.propControlModeJointImpedance.getStiffness();
		for (int i = 0; i < 7; i++) 
			if (stiffness[i] < 0 || Double.isInfinite(stiffness[i]))
				stiffness[i] = currentStiffness[i];
		this.propControlModeJointImpedance.setStiffness(stiffness);
		if (VERBOSE) getLogger().info("Joint stiffness: " + stiffness);
		return true;
	}
	
	/**
	 * Define the joint impedance control damping
	 * <p>
	 * damping: [0.0, 1.0] (default: 0.7)
	 * 
	 * @param damping joint damping 
	 * @return true
	 */
	public boolean methSetJointDamping(double[] damping) {
		double[] currentDamping = this.propControlModeJointImpedance.getDamping();
		for (int i = 0; i < 7; i++) 
			if (damping[i] < 0.0 || damping[i] > 1.0)
				damping[i] = currentDamping[i];
		this.propControlModeJointImpedance.setDamping(damping);
		if (VERBOSE) getLogger().info("Joint damping: " + damping);
		return true;
	}
	
	/** DONE
	 * Set the control interface
	 * 
	 * @param controlInterface control interface (CONTROL_INTERFACE_STANDARD, CONTROL_INTERFACE_SERVO)
	 * @return true if the configuration is valid, otherwise false
	 */
	public boolean methSetControlInterface(LibIiwaEnum controlInterface) {
		// stop and reset motion
		this.methStopAndResetMotion();
		
		// set control mode
		if (controlInterface.getCode() == LibIiwaEnum.CONTROL_INTERFACE_STANDARD.getCode())
			this.enumControlInterface = LibIiwaEnum.CONTROL_INTERFACE_STANDARD;
		else if (controlInterface.getCode() == LibIiwaEnum.CONTROL_INTERFACE_SERVO.getCode())
			this.enumControlInterface = LibIiwaEnum.CONTROL_INTERFACE_SERVO;
		else {
			if (VERBOSE) getLogger().warn("Invalid control interface: " + controlInterface.getCode());
			this.enumLastError = LibIiwaEnum.VALUE_ERROR;
			return false;
		}

		if (VERBOSE) getLogger().info("Control interface: " + this.enumControlInterface.toString());
		return true;
	}

	/** DONE
	 * Set the motion type
	 * 
	 * @param motionType motion type (MOTION_TYPE_PTP, MOTION_TYPE_LIN, MOTION_TYPE_LIN_REL, MOTION_TYPE_CIRC)
	 * @return true if the configuration is valid, otherwise false
	 */
	public boolean methSetMotionType(LibIiwaEnum motionType) {
		// stop and reset motion
		this.methStopAndResetMotion();
		
		// set motion type
		if (motionType.getCode() == LibIiwaEnum.MOTION_TYPE_PTP.getCode())
			this.enumMotionType = LibIiwaEnum.MOTION_TYPE_PTP;
		else if (motionType.getCode() == LibIiwaEnum.MOTION_TYPE_LIN.getCode())
			this.enumMotionType = LibIiwaEnum.MOTION_TYPE_LIN;
		else if (motionType.getCode() == LibIiwaEnum.MOTION_TYPE_LIN_REL.getCode())
			this.enumMotionType = LibIiwaEnum.MOTION_TYPE_LIN_REL;
		else if (motionType.getCode() == LibIiwaEnum.MOTION_TYPE_CIRC.getCode())
			this.enumMotionType = LibIiwaEnum.MOTION_TYPE_CIRC;
		else {
			if (VERBOSE) getLogger().warn("Invalid motion type: " + motionType.getCode());
			this.enumLastError = LibIiwaEnum.VALUE_ERROR;
			return false;
		}

		if (VERBOSE) getLogger().info("Motion type: " + this.enumMotionType.toString());
		return true;
	}
	
	/** DONE
	 * Set the control mode
	 * 
	 * @param controlMode control mode (CONTROL_MODE_POSITION, CONTROL_MODE_JOINT_IMPEDANCE, CONTROL_MODE_CARTESIAN_IMPEDANCE, CONTROL_MODE_CARTESIAN_SINE_IMPEDANCE)
	 * @return true if the configuration is valid, otherwise false
	 */
	public boolean methSetControlMode(LibIiwaEnum controlMode) {
		// stop and reset motion
		this.methStopAndResetMotion();
		
		// set control mode
		if (controlMode.getCode() == LibIiwaEnum.CONTROL_MODE_POSITION.getCode()) {
			this.enumControlMode = LibIiwaEnum.CONTROL_MODE_POSITION;
			this.propCurrentControlMode = this.propControlModePosition;
		}
		else if (controlMode.getCode() == LibIiwaEnum.CONTROL_MODE_JOINT_IMPEDANCE.getCode()) {
			this.enumControlMode = LibIiwaEnum.CONTROL_MODE_JOINT_IMPEDANCE;
			this.propCurrentControlMode = this.propControlModeJointImpedance;
		}
		else if (controlMode.getCode() == LibIiwaEnum.CONTROL_MODE_CARTESIAN_IMPEDANCE.getCode()) {
			this.enumControlMode = LibIiwaEnum.CONTROL_MODE_CARTESIAN_IMPEDANCE;
			this.propCurrentControlMode = this.propControlModeCartesianImpedance;
		}
		else if (controlMode.getCode() == LibIiwaEnum.CONTROL_MODE_CARTESIAN_SINE_IMPEDANCE.getCode()) {
			this.enumControlMode = LibIiwaEnum.CONTROL_MODE_CARTESIAN_SINE_IMPEDANCE;
			this.propCurrentControlMode = this.propControlModeCartesianSineImpedance;
		}
		else {
			if (VERBOSE) getLogger().warn("Invalid control mode: " + controlMode.getCode());
			this.enumLastError = LibIiwaEnum.VALUE_ERROR;
			return false;
		}

		if (VERBOSE) getLogger().info("Control mode: " + this.enumControlMode.toString());
		return true;
	}
	
	/** DONE
	 * Set the execution type
	 * 
	 * @param executionType execution type (EXECUTION_TYPE_ASYNCHRONOUS, EXECUTION_TYPE_SYNCHRONOUS)
	 * @return true if the configuration is valid, otherwise false
	 */
	public boolean methSetExecutionType(LibIiwaEnum executionType) {
		// stop and reset motion
		this.methStopAndResetMotion();
		
		// set execution type
		if (executionType.getCode() == LibIiwaEnum.EXECUTION_TYPE_ASYNCHRONOUS.getCode())
			this.enumControlExecutionType = LibIiwaEnum.EXECUTION_TYPE_ASYNCHRONOUS;
		else if (executionType.getCode() == LibIiwaEnum.EXECUTION_TYPE_SYNCHRONOUS.getCode())
			this.enumControlExecutionType = LibIiwaEnum.EXECUTION_TYPE_SYNCHRONOUS;
		else {
			if (VERBOSE) getLogger().warn("Invalid execution type: " + executionType.getCode());
			this.enumLastError = LibIiwaEnum.VALUE_ERROR;
			return false;
		}

		if (VERBOSE) getLogger().info("Execution type: " + this.enumControlExecutionType.toString());
		return true;
	}
	
	/**
	 * Set the communication mode
	 * 
	 * @param communicationMode communication mode (COMMUNICATION_MODE_ON_DEMAND, COMMUNICATION_MODE_PERIODICAL)
	 * @return true if the configuration is valid, otherwise false
	 */
	public boolean methSetCommunicationMode(LibIiwaEnum communicationMode) {
		// stop and reset motion
		this.methStopAndResetMotion();

		// set control mode
		if (communicationMode.getCode() == LibIiwaEnum.COMMUNICATION_MODE_ON_DEMAND.getCode())
			this.propCommunicationMode = LibIiwaEnum.COMMUNICATION_MODE_ON_DEMAND;
		else if (communicationMode.getCode() == LibIiwaEnum.COMMUNICATION_MODE_PERIODICAL.getCode())
			this.propCommunicationMode = LibIiwaEnum.COMMUNICATION_MODE_PERIODICAL;
		else {
			if (VERBOSE) getLogger().warn("Invalid communication mode: " + communicationMode.getCode());
			return false;
		}

		if (VERBOSE) getLogger().info("Communication mode: " + this.propCommunicationMode.toString());
		return true;
	}

	// ===========================================================
	// CONTROL METHODS
	// ===========================================================

	/**
	 * Control robot using joint positions (in radians)
	 * <p>
	 * Joints not in range will be not controlled. Use this feature to move specific joints
	 * 
	 * @param joints joint positions (7)
	 * @return true if the control was successful, otherwise false
	 */
	private boolean methGoToJointPosition(double[] joints) {
		// TODO: add other BasicMotions
		JointPosition jointPosition = lbr.getCurrentJointPosition();

		// validate
		for (int i = 0; i < 7; i++)
			if (joints[i] >= propMinJointPositionLimits.get(i) && joints[i] <= propMaxJointPositionLimits.get(i))
				jointPosition.set(i, joints[i]);

		// move
		if (enumControlInterface == LibIiwaEnum.CONTROL_INTERFACE_STANDARD) {
			MotionBatch motionBatch = new MotionBatch(BasicMotions.ptp(jointPosition).setJointJerkRel(propDesiredJointJerkRel));
			return methMoveStandard(motionBatch);
		}
		else if (enumControlInterface == LibIiwaEnum.CONTROL_INTERFACE_SERVO) {
			return methMoveSmartServo(jointPosition);
		}
		return false;
	}

	/**
	 * Control robot using Cartesian pose (in millimeters and radians)
	 * 
	 * @param pose Cartesian position (3) and orientation(3)
	 * @return true if the control was successful, otherwise false
	 */
	private boolean methGoToCartesianPose(double[] pose) {
		// TODO: add LINREL
		// TODO: check to use a large number for Cartesian validation (messages need to send numbers) 
		Frame frame = lbr.getCurrentCartesianPosition(lbr.getFlange());

		// validate
		if (!Double.isInfinite(pose[0]))
			frame.setX(pose[0]);
		if (!Double.isInfinite(pose[1]))
			frame.setY(pose[1]);
		if (!Double.isInfinite(pose[2]))
			frame.setZ(pose[2]);
		if (!Double.isInfinite(pose[3]))
			frame.setAlphaRad(pose[3]);
		if (!Double.isInfinite(pose[4]))
			frame.setBetaRad(pose[4]);
		if (!Double.isInfinite(pose[5]))
			frame.setGammaRad(pose[5]);

		// move
		if (enumControlInterface == LibIiwaEnum.CONTROL_INTERFACE_STANDARD) {
			MotionBatch motionBatch;
			if (enumMotionType == LibIiwaEnum.MOTION_TYPE_PTP)
				motionBatch = new MotionBatch(BasicMotions.ptp(frame).setJointJerkRel(propDesiredJointJerkRel));
			else if (enumMotionType == LibIiwaEnum.MOTION_TYPE_LIN)
				motionBatch = new MotionBatch(BasicMotions.lin(frame).setJointJerkRel(propDesiredJointJerkRel).
						setCartVelocity(propDesiredCartesianVelocity).setCartAcceleration(propDesiredCartesianAcceleration));
			else
				return false;
			return methMoveStandard(motionBatch);
		}
		else if (enumControlInterface == LibIiwaEnum.CONTROL_INTERFACE_SERVO) {
			if (enumMotionType == LibIiwaEnum.MOTION_TYPE_PTP)
				return methMoveSmartServo(frame);
			else if (enumMotionType == LibIiwaEnum.MOTION_TYPE_LIN)
				return methMoveSmartServoLIN(frame);
			return false;
		}
		return false;
	}

	// ===========================================================
	// 
	// ===========================================================

	private double[] methUpdateAndGetCurrentState() {
		JointPosition jointPosition = lbr.getCurrentJointPosition();
		Frame frame = lbr.getCurrentCartesianPosition(lbr.getFlange());
		TorqueSensorData torqueSensorData = lbr.getExternalTorque();
		ForceSensorData forceSensorData = lbr.getExternalForceTorque(lbr.getFlange());
		
		// joint position
		this.propCurrentState[STATE_JOINT_POSITION + 0] = jointPosition.get(JointEnum.J1);
		this.propCurrentState[STATE_JOINT_POSITION + 1] = jointPosition.get(JointEnum.J2);
		this.propCurrentState[STATE_JOINT_POSITION + 2] = jointPosition.get(JointEnum.J3);
		this.propCurrentState[STATE_JOINT_POSITION + 3] = jointPosition.get(JointEnum.J4);
		this.propCurrentState[STATE_JOINT_POSITION + 4] = jointPosition.get(JointEnum.J5);
		this.propCurrentState[STATE_JOINT_POSITION + 5] = jointPosition.get(JointEnum.J6);
		this.propCurrentState[STATE_JOINT_POSITION + 6] = jointPosition.get(JointEnum.J7);

		// joint torque
		this.propCurrentState[STATE_JOINT_TORQUE + 0] = torqueSensorData.getSingleTorqueValue(JointEnum.J1);
		this.propCurrentState[STATE_JOINT_TORQUE + 1] = torqueSensorData.getSingleTorqueValue(JointEnum.J2);
		this.propCurrentState[STATE_JOINT_TORQUE + 2] = torqueSensorData.getSingleTorqueValue(JointEnum.J3);
		this.propCurrentState[STATE_JOINT_TORQUE + 3] = torqueSensorData.getSingleTorqueValue(JointEnum.J4);
		this.propCurrentState[STATE_JOINT_TORQUE + 4] = torqueSensorData.getSingleTorqueValue(JointEnum.J5);
		this.propCurrentState[STATE_JOINT_TORQUE + 5] = torqueSensorData.getSingleTorqueValue(JointEnum.J6);
		this.propCurrentState[STATE_JOINT_TORQUE + 6] = torqueSensorData.getSingleTorqueValue(JointEnum.J7);
		
		// Cartesian pose
		this.propCurrentState[STATE_CARTESIAN_POSITION + 0] = frame.getX();
		this.propCurrentState[STATE_CARTESIAN_POSITION + 1] = frame.getY();
		this.propCurrentState[STATE_CARTESIAN_POSITION + 2] = frame.getZ();

		this.propCurrentState[STATE_CARTESIAN_ORIENTATION + 0] = frame.getAlphaRad();
		this.propCurrentState[STATE_CARTESIAN_ORIENTATION + 1] = frame.getBetaRad();
		this.propCurrentState[STATE_CARTESIAN_ORIENTATION + 2] = frame.getGammaRad();

		// Cartesian force/torque
		this.propCurrentState[STATE_CARTESIAN_FORCE + 0] = forceSensorData.getForce().getX();
		this.propCurrentState[STATE_CARTESIAN_FORCE + 1] = forceSensorData.getForce().getY();
		this.propCurrentState[STATE_CARTESIAN_FORCE + 2] = forceSensorData.getForce().getZ();
		
		this.propCurrentState[STATE_CARTESIAN_TORQUE + 0] = forceSensorData.getTorque().getX();
		this.propCurrentState[STATE_CARTESIAN_TORQUE + 1] = forceSensorData.getTorque().getY();
		this.propCurrentState[STATE_CARTESIAN_TORQUE + 2] = forceSensorData.getTorque().getZ();
		
		return this.propCurrentState;
	}

	/** CHECK
	 * Process input command
	 * 
	 * @param command Command to process
	 * @return true if the command was processed successfully, otherwise false
	 */
	private boolean methProcessCommand(double[] command) {
		double commandCode = command[0];
		// empty commands
		if (commandCode == LibIiwaEnum.COMMAND_PASS.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_PASS.toString());
			return true;
		}
		// control command
		else if (commandCode == LibIiwaEnum.COMMAND_JOINT_POSITION.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_JOINT_POSITION.toString());
			return this.methGoToJointPosition(Arrays.copyOfRange(command, 1, 1 + 7));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_CARTESIAN_POSE.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_CARTESIAN_POSE.toString());
			return this.methGoToCartesianPose(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		// configuration commands (limits and constants)
		else if (commandCode == LibIiwaEnum.COMMAND_SET_DESIRED_JOINT_VELOCITY_REL.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_DESIRED_JOINT_VELOCITY_REL.toString());
			return this.methSetDesiredJointVelocityRel(command[1]);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_DESIRED_JOINT_ACCELERATION_REL.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_DESIRED_JOINT_ACCELERATION_REL.toString());
			return this.methSetDesiredJointAccelerationRel(command[1]);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_DESIRED_JOINT_JERK_REL.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_DESIRED_JOINT_JERK_REL.toString());
			return this.methSetDesiredJointJerkRel(command[1]);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_DESIRED_CARTESIAN_VELOCITY.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_DESIRED_CARTESIAN_VELOCITY.toString());
			return this.methSetDesiredCartesianVelocity(command[1]);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_DESIRED_CARTESIAN_ACCELERATION.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_DESIRED_CARTESIAN_ACCELERATION.toString());
			return this.methSetDesiredCartesianAcceleration(command[1]);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_DESIRED_CARTESIAN_JERK.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_DESIRED_CARTESIAN_JERK.toString());
			return this.methSetDesiredCartesianJerk(command[1]);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_FORCE_CONDITION.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_FORCE_CONDITION.toString());
			return this.methSetForceCondition(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_STIFFNESS.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_STIFFNESS.toString());
			return this.methSetCartesianStiffness(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_DAMPING.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_DAMPING.toString());
			return this.methSetCartesianDamping(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_ADDITIONAL_CONTROL_FORCE.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_ADDITIONAL_CONTROL_FORCE.toString());
			return this.methSetCartesianAdditionalControlForce(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_JOINT_STIFFNESS.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_JOINT_STIFFNESS.toString());
			return this.methSetJointStiffness(Arrays.copyOfRange(command, 1, 1 + 7));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_JOINT_DAMPING.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_JOINT_DAMPING.toString());
			return this.methSetJointDamping(Arrays.copyOfRange(command, 1, 1 + 7));
		}
		// configuration commands (control)
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CONTROL_INTERFACE.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_CONTROL_INTERFACE.toString());
			LibIiwaEnum controlInterface = LibIiwaEnum.CONTROL_INTERFACE;
			controlInterface.setCode(command[1]);
			return this.methSetControlInterface(controlInterface);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_MOTION_TYPE.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_MOTION_TYPE.toString());
			LibIiwaEnum motionType = LibIiwaEnum.MOTION_TYPE;
			motionType.setCode(command[1]);
			return this.methSetMotionType(motionType);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CONTROL_MODE.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_CONTROL_MODE.toString());
			LibIiwaEnum controlMode = LibIiwaEnum.CONTROL_MODE;
			controlMode.setCode(command[1]);
			return this.methSetControlMode(controlMode);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_EXECUTION_TYPE.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_EXECUTION_TYPE.toString());
			LibIiwaEnum executionType = LibIiwaEnum.EXECUTION_TYPE;
			executionType.setCode(command[1]);
			return this.methSetExecutionType(executionType);
		}
		// configuration commands (communication)
		else if (commandCode == LibIiwaEnum.COMMAND_SET_COMMUNICATION_MODE.getCode()){
			if (VERBOSE) getLogger().info(LibIiwaEnum.COMMAND_SET_COMMUNICATION_MODE.toString());
			LibIiwaEnum communicationMode = LibIiwaEnum.COMMUNICATION_MODE;
			communicationMode.setCode(command[1]);
			return this.methSetCommunicationMode(communicationMode);
		}
		if (VERBOSE) getLogger().warn("Unknown command code: " + commandCode);
		return false;
	}

	/**
	 * Perform an on-demand communication
	 * 
	 * @return true if the application should continue, otherwise false
	 */
	private boolean methOnDemmandCommunication() {
		// get command
		double[] command = this.propCommunication.methReceiveData();
		// check for error
		if (command[0] < 0) {
			this.propCommunication.methLogError(command);
			this.propShouldContinue = false;
			return false;
		}
		// process command
		boolean commandStatus = false;
		if (command[0] > 0)
			commandStatus = this.methProcessCommand(command);
		// build state
		double[] state = methUpdateAndGetCurrentState();
		state[STATE_COMMAND_STATUS] = commandStatus ? 1.0 : 0.0;
		// send state
		if (!this.propCommunication.methSendData(state)){
			this.propShouldContinue = false;
			return false;
		}
		return true;
	}

	private boolean methPeriodicalCommunication() {
		// TODO: implement
		return true;
	}

	private Thread propThreadSendState = new Thread() {
		@Override
		public void run() {
			while(propShouldContinue) {
				double[] state = methUpdateAndGetCurrentState();
				if (!propCommunication.methSendData(state))
					propShouldContinue = false;
				ThreadUtil.milliSleep(10);  // TODO: pass time as class argument
			}
		}
	};

	@Override
	public void initialize() {
		// application data
		this.propApplicationData = getApplicationData();
		VERBOSE = this.propApplicationData.getProcessData("verbose").getValue();
		
		// TODO: get robot and tool
		getController(CONTROLLER_NAME);
		lbr = getContext().getDeviceFromType(LBR.class);
		// tool = getApplicationData().createFromTemplate(TOOL_NAME);
		// tool.attachTo(lbr.getFlange());

		// initialize variables
		this.propMaxJointPositionLimits = lbr.getJointLimits().getMaxJointPosition();
		this.propMinJointPositionLimits = lbr.getJointLimits().getMinJointPosition();

		this.propCurrentState = new double[STATE_LENGTH];
		this.propCommunication = new LibIiwaCommunication(getLogger(), COMMAND_LENGTH);
		
		this.propControlModePosition = new PositionControlMode();
		this.propControlModeJointImpedance = new JointImpedanceControlMode(2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0);
		this.propControlModeCartesianImpedance = new CartesianImpedanceControlMode();
		this.propControlModeCartesianSineImpedance = new CartesianSineImpedanceControlMode();
		this.propCurrentControlMode = this.propControlModePosition;
		
		this.propForceConditionX = ForceCondition.createNormalForceCondition(lbr.getFlange(), CoordinateAxis.X, 1000.0);
		this.propForceConditionY = ForceCondition.createNormalForceCondition(lbr.getFlange(), CoordinateAxis.Y, 1000.0);
		this.propForceConditionZ = ForceCondition.createNormalForceCondition(lbr.getFlange(), CoordinateAxis.Z, 1000.0);

		// GUI key
		IUserKeyBar userKeyBar = getApplicationUI().createUserKeyBar("KEY");
		IUserKeyListener userKeyListener = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (event == UserKeyEvent.KeyDown) {
					key.setLED(UserKeyAlignment.TopMiddle, UserKeyLED.Red, UserKeyLEDSize.Small);
					getLogger().info("Stopping application");
					// close application
					dispose();
				}
			}
		};

		// GUI led
		IUserKey userKey = userKeyBar.addUserKey(0, userKeyListener, true);
		userKey.setLED(UserKeyAlignment.TopMiddle, UserKeyLED.Green, UserKeyLEDSize.Small);
		userKey.setText(UserKeyAlignment.TopMiddle, "STOP");
		userKeyBar.publish();
	}

	@Override
	public void dispose() {
		getLogger().info("Disposal request");
		// stop application loop
		this.propShouldContinue = false;
		// stop robot
		this.methStopAndResetMotion();
		// close socket
		this.propCommunication.methCloseSocketConnection();
		getLogger().info("Bye!");
	}

	@Override
	public void run() {
		// initialize communication
		String address = this.propApplicationData.getProcessData("controller_ip").getValue();
		int port = this.propApplicationData.getProcessData("controller_port").getValue();
		
		this.propShouldContinue = this.propCommunication.methInitializeSocketConnection(address, port, false, 0, 1000);  // as client
		this.propCommunicationMode = LibIiwaEnum.COMMUNICATION_MODE_ON_DEMAND;
		this.propCommunication.methSetSocketTimeout(0);

		// application loop
		while(this.propShouldContinue) {

			// process communication
			if (this.propCommunicationMode == LibIiwaEnum.COMMUNICATION_MODE_ON_DEMAND) {
				if (!this.methOnDemmandCommunication())
					break;
			}
			else if (this.propCommunicationMode == LibIiwaEnum.COMMUNICATION_MODE_PERIODICAL) {
				if (!this.methPeriodicalCommunication())
					break;
			}
			else {
				this.propShouldContinue = false;
				break;
			}
		}

		// stop robot
		this.methStopAndResetMotion();
		// close socket
		this.propCommunication.methCloseSocketConnection();
		getLogger().info("Application stopped");
	}
}

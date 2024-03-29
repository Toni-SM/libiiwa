package application;

import com.kuka.common.ThreadUtil;

import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.connectivity.motionModel.smartServoLIN.ISmartServoLINRuntime;

import com.kuka.roboticsAPI.applicationModel.IApplicationData;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;

import com.kuka.roboticsAPI.deviceModel.Device;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;

import com.kuka.roboticsAPI.executionModel.IFiredConditionInfo;

import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;

import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.CartPlane;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;

import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

import com.kuka.roboticsAPI.motionModel.BasicMotions;
import com.kuka.roboticsAPI.motionModel.ErrorHandlingAction;
import com.kuka.roboticsAPI.motionModel.IErrorHandler;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.MotionBatch;

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
import java.util.List;

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

	@SuppressWarnings("unused")
	private static final String API_VERSION = "0.3.1-beta";

	private static final int STATE_COMMAND_STATUS = 0;  		// vector(1)
	private static final int STATE_JOINT_POSITION = 1;  		// vector(7)
	private static final int STATE_JOINT_VELOCITY = 8;  		// vector(7)
	private static final int STATE_JOINT_TORQUE = 15;  			// vector(7)
	private static final int STATE_CARTESIAN_POSITION = 22;  	// vector(3)
	private static final int STATE_CARTESIAN_ORIENTATION = 25; 	// vector(3)
	private static final int STATE_CARTESIAN_FORCE = 28;     	// vector(3)
	private static final int STATE_CARTESIAN_TORQUE = 31;    	// vector(3)
	private static final int STATE_LAST_ERROR = 34;  			// vector(1)
	private static final int STATE_FIRED_CONDITION = 35;  		// vector(1)
	private static final int STATE_READY_TO_MOVE = 36;  		// vector(1)
	private static final int STATE_HAS_ACTIVE_MOTION = 37; 		// vector(1)

	private static final int STATE_LENGTH = 1 + 7 * 3 + 3 * 4 + 1 * 4; 	// vector(38)
	private static final int COMMAND_LENGTH = 8;             			// vector(8)

	// ===========================================================
	// VARIABLES
	// ===========================================================

	// settings 
	private boolean VERBOSE_INFO = false;
	private boolean VERBOSE_WARN = true;

	// communication
	private LibIiwaCommunication propCommunication;
	private LibIiwaEnum propCommunicationMode = LibIiwaEnum.COMMUNICATION_MODE_ON_DEMAND;

	private String CONTROLLER_NAME = "KUKA_Sunrise_Cabinet_1";

	// robot and tool
	private IApplicationData propApplicationData = null;
	private LBR lbr = null;
	private Tool tool = null;

	private double propCurrentState[];
	private double propCurrentJointVelocity[];
	private boolean propShouldContinue = false;	// TODO: rename

	private JointPosition propMinJointPositionLimits;
	private JointPosition propMaxJointPositionLimits;
	private double propMinJointTorqueLimits[];
	private double propMaxJointTorqueLimits[];
	
	// errors
	private LibIiwaEnum enumLastError = LibIiwaEnum.ERROR;
	private IErrorHandler propErrorHandler;

	// configuration
	private LibIiwaEnum enumMotionType = LibIiwaEnum.MOTION_TYPE_PTP;
	private LibIiwaEnum enumControlInterface = LibIiwaEnum.CONTROL_INTERFACE_STANDARD;
	private LibIiwaEnum enumControlMode = LibIiwaEnum.CONTROL_MODE_POSITION;
	private LibIiwaEnum enumExecutionType = LibIiwaEnum.EXECUTION_TYPE_ASYNCHRONOUS;

	private IMotionControlMode propCurrentControlMode = null;
	private IMotionContainer propCurrentStandardMotionContainer = null;
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
	
	// conditions
	private JointTorqueCondition propJointTorqueConditions[];
	private boolean propJointTorqueConditionsEnabled[];
	private ForceCondition propForceConditions[];
	private boolean propForceConditionsEnabled[];

	private double propMinimumTrajectoryExecutionTime = 0.001;
	private double propTimeoutAfterGoalReach = 300;  // seconds

	private boolean propOverwriteMotion = true;

	// ===========================================================
	// INTERNAL METHODS
	// ===========================================================

	private JointEnum _JointEnum(int index) {
		switch (index) {
			case 0:
				return JointEnum.J1;
			case 1:
				return JointEnum.J2;
			case 2:
				return JointEnum.J3;
			case 3:
				return JointEnum.J4;
			case 4:
				return JointEnum.J5;
			case 5:
				return JointEnum.J6;
			case 6:
				return JointEnum.J7;
			case 7:
				return JointEnum.J8;
			case 8:
				return JointEnum.J9;
			case 9:
				return JointEnum.J10;
			case 10:
				return JointEnum.J11;
			case 11:
				return JointEnum.J12;
		}
		throw new IllegalArgumentException("Invalid JointEnum index: " + index);
	}

	private CoordinateAxis _CoordinateAxis(int index) {
		switch (index) {
			case 0:
				return CoordinateAxis.X;
			case 1:
				return CoordinateAxis.Y;
			case 2:
				return CoordinateAxis.Z;
		}
		throw new IllegalArgumentException("Invalid CoordinateAxis index: " + index);
	}

	private CartPlane _CartPlane(int index) {
		switch (index) {
			case 0:
				return CartPlane.XY;
			case 1:
				return CartPlane.XZ;
			case 2:
				return CartPlane.YZ;
		}
		throw new IllegalArgumentException("Invalid CartPlane index: " + index);
	}

	private CartDOF _CartDOF(int index) {
		switch (index) {
			case 0:
				return CartDOF.X;
			case 1:
				return CartDOF.Y;
			case 2:
				return CartDOF.Z;
			case 3:
				return CartDOF.A;
			case 4:
				return CartDOF.B;
			case 5:
				return CartDOF.C;
			case 6:
				return CartDOF.TRANSL;
			case 7:
				return CartDOF.ROT;
			case 8:
				return CartDOF.ALL;
		}
		throw new IllegalArgumentException("Invalid CartPlane index: " + index);
	}

	// ===========================================================
	// INTERNAL METHODS
	// ===========================================================

	private void methStopAndResetMotion() {
		// stop motions
		if (this.propCurrentStandardMotionContainer != null)
			this.propCurrentStandardMotionContainer.cancel();
		if (this.propCurrentSmartServoRuntime != null)
			this.propCurrentSmartServoRuntime.stopMotion();
		if (this.propCurrentSmartServoLINRuntime != null)
			this.propCurrentSmartServoLINRuntime.stopMotion();

		// reset variables
		this.propCurrentStandardMotionContainer = null;
		this.propCurrentSmartServoRuntime = null;
		this.propCurrentSmartServoLINRuntime = null;
	}

	private boolean methInitializeSmartServo() {
		if (this.propCurrentSmartServoRuntime != null)
			return true;

		SmartServo smartServo = new SmartServo(lbr.getCurrentJointPosition());

		smartServo.setJointVelocityRel(this.propDesiredJointVelocityRel);
		smartServo.setJointAccelerationRel(this.propDesiredJointAccelerationRel);

		smartServo.setMinimumTrajectoryExecutionTime(this.propMinimumTrajectoryExecutionTime);
		smartServo.setTimeoutAfterGoalReach(this.propTimeoutAfterGoalReach);
		
		// validate for impedance mode
		if (this.enumControlMode != LibIiwaEnum.CONTROL_MODE && this.enumControlMode != LibIiwaEnum.CONTROL_MODE_POSITION)
			try {
				SmartServo.validateForImpedanceMode(lbr);
			}
			catch (Exception e) {
				this.enumLastError = LibIiwaEnum.VALIDATION_FOR_IMPEDANCE_ERROR;
				if (VERBOSE_WARN) getLogger().warn(e.getMessage());
				return false;
			}
		
		if (this.tool != null)
			this.tool.moveAsync(smartServo.setMode(this.propCurrentControlMode));
		else
			this.lbr.moveAsync(smartServo.setMode(this.propCurrentControlMode));

		this.propCurrentSmartServoRuntime = smartServo.getRuntime(true);
		return true;
	}

	private boolean methInitializeSmartServoLIN() {
		if (this.propCurrentSmartServoLINRuntime != null)
			return true;

		ObjectFrame objectFrame = this.tool == null ? this.lbr.getFlange() : this.tool.getDefaultMotionFrame();

		SmartServoLIN smartServoLIN = new SmartServoLIN(this.lbr.getCurrentCartesianPosition(objectFrame));

		//		smartServoLIN.setMaxTranslationVelocity(...);
		//		smartServoLIN.setMaxTranslationAcceleration(...);
		//		smartServoLIN.setMaxOrientationVelocity(...);
		//		smartServoLIN.setMaxOrientationAcceleration(...);
		//		smartServoLIN.setMaxNullSpaceVelocity(...);
		//		smartServoLIN.setMaxNullSpaceAcceleration(...);

		smartServoLIN.setMinimumTrajectoryExecutionTime(this.propMinimumTrajectoryExecutionTime);
		smartServoLIN.setTimeoutAfterGoalReach(this.propTimeoutAfterGoalReach);

		// validate for impedance mode
		if (this.enumControlMode != LibIiwaEnum.CONTROL_MODE && this.enumControlMode != LibIiwaEnum.CONTROL_MODE_POSITION)
			try {
				SmartServo.validateForImpedanceMode(lbr);
			}
			catch (Exception e) {
				this.enumLastError = LibIiwaEnum.VALIDATION_FOR_IMPEDANCE_ERROR;
				if (VERBOSE_WARN) getLogger().warn(e.getMessage());
				return false;
			}
		
		if (this.tool != null)
			this.tool.moveAsync(smartServoLIN.setMode(this.propCurrentControlMode));
		else
			this.lbr.moveAsync(smartServoLIN.setMode(this.propCurrentControlMode));

		this.propCurrentSmartServoLINRuntime = smartServoLIN.getRuntime(true); 
		return true;
	}

	private ICondition methGetConditions() {
		ICondition condition = null;

		// force conditions
		for (int i = 0; i < 3; i++)
			if (this.propForceConditionsEnabled[i])
				condition = condition == null ? this.propForceConditions[i] : condition.or(this.propForceConditions[i]);
		// joint torque conditions
		for (int i = 0; i < lbr.getJointCount(); i++)
			if (this.propJointTorqueConditionsEnabled[i])
				condition = condition == null ? this.propJointTorqueConditions[i] : condition.or(this.propJointTorqueConditions[i]);

		return condition;
	}

	// ===========================================================
	// INTERNAL CONTROL METHODS
	// ===========================================================

	private boolean methMoveStandard(MotionBatch motionBatch) {
		if (lbr.isReadyToMove()) {
			// set limits
			motionBatch.setJointVelocityRel(this.propDesiredJointVelocityRel).setJointAccelerationRel(this.propDesiredJointAccelerationRel);
			
			// set conditions
			ICondition conditions = this.methGetConditions();
			if (conditions != null)
				motionBatch.breakWhen(conditions);
			
			// overwrite motion
			if (this.propCurrentStandardMotionContainer != null) {
				if (!this.propCurrentStandardMotionContainer.isFinished()) {
					if (!this.propOverwriteMotion)
						return false;
					this.propCurrentStandardMotionContainer.cancel();
				}
			}
			// execute motion
			if (this.enumExecutionType == LibIiwaEnum.EXECUTION_TYPE_SYNCHRONOUS) {
				try {
					if (this.tool != null)
						this.propCurrentStandardMotionContainer = this.tool.move(motionBatch.setMode(this.propCurrentControlMode));
					else
						this.propCurrentStandardMotionContainer = this.lbr.move(motionBatch.setMode(this.propCurrentControlMode));
				}
				catch (Exception e) {
					this.enumLastError = LibIiwaEnum.SYNCHRONOUS_MOTION_ERROR;
					if (VERBOSE_WARN) getLogger().warn(e.getMessage());
					return false;
				}
			}
			else if (this.enumExecutionType == LibIiwaEnum.EXECUTION_TYPE_ASYNCHRONOUS)	{
				if (this.tool != null)
					this.propCurrentStandardMotionContainer = this.tool.moveAsync(motionBatch.setMode(this.propCurrentControlMode));
				else
					this.propCurrentStandardMotionContainer = this.lbr.moveAsync(motionBatch.setMode(this.propCurrentControlMode));
			}
			return true;
		}
		return false;
	}

	private boolean methMoveSmartServo(JointPosition jointPosition) {
		// check for asynchronous execution
		if (this.enumExecutionType == LibIiwaEnum.EXECUTION_TYPE_SYNCHRONOUS) {
			this.enumLastError = LibIiwaEnum.INVALID_CONFIGURATION_ERROR;
			if (VERBOSE_WARN) getLogger().warn("Invalid configuration: Servo cannot run in synchronous execution");
			return false;
		}
		// move
		if (lbr.isReadyToMove()) {
			if (!methInitializeSmartServo())
				return false;
			this.propCurrentSmartServoRuntime.setDestination(jointPosition);
			return true;
		}
		return false;
	}

	private boolean methMoveSmartServo(Frame frame) {
		// check for asynchronous execution
		if (this.enumExecutionType == LibIiwaEnum.EXECUTION_TYPE_SYNCHRONOUS) {
			this.enumLastError = LibIiwaEnum.INVALID_CONFIGURATION_ERROR;
			if (VERBOSE_WARN) getLogger().warn("Invalid configuration: Servo cannot run in synchronous execution");
			return false;
		}
		// move
		if (lbr.isReadyToMove()) {
			if (!methInitializeSmartServo())
				return false;
			this.propCurrentSmartServoRuntime.setDestination(frame);
			return true;
		}
		return false;
	}

	private boolean methMoveSmartServoLIN(Frame frame) {
		// check for asynchronous execution
		if (this.enumExecutionType == LibIiwaEnum.EXECUTION_TYPE_SYNCHRONOUS) {
			this.enumLastError = LibIiwaEnum.INVALID_CONFIGURATION_ERROR;
			if (VERBOSE_WARN) getLogger().warn("Invalid configuration: Servo cannot run in synchronous execution");
			return false;
		}
		// move
		if (lbr.isReadyToMove()) {
			if (!methInitializeSmartServoLIN())
				return false;
			this.propCurrentSmartServoLINRuntime.setDestination(frame);
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
	
	/** DONE
	 * Define the axis-specific relative velocity (% of maximum velocity)
	 * <p>
	 * velocity: [0.0, 1.0]
	 * 
	 * @param jointVelocityRel relative maximum velocity
	 * @return true if it is valid, otherwise false
	 */
	public boolean methSetDesiredJointVelocityRel(double jointVelocityRel) {
		if (jointVelocityRel < 0 || jointVelocityRel > 1.0) {
			if (VERBOSE_WARN) getLogger().warn("Invalid joint velocity: " + jointVelocityRel);
			this.enumLastError = LibIiwaEnum.VALUE_ERROR;
			return false;
		}
		this.propDesiredJointVelocityRel = jointVelocityRel;
		if (VERBOSE_INFO) getLogger().info("Desired joint velocity: " + jointVelocityRel);
		return true;
	}

	/** DONE
	 * Define the axis-specific relative acceleration (% of maximum acceleration)
	 * <p>
	 * acceleration: [0.0, 1.0]
	 * 
	 * @param jointAccelerationRel relative maximum acceleration
	 * @return true if it is valid, otherwise false
	 */
	public boolean methSetDesiredJointAccelerationRel(double jointAccelerationRel) {
		if (jointAccelerationRel < 0 || jointAccelerationRel > 1.0) {
			if (VERBOSE_WARN) getLogger().warn("Invalid joint acceleration: " + jointAccelerationRel);
			this.enumLastError = LibIiwaEnum.VALUE_ERROR;
			return false;
		}
		this.propDesiredJointAccelerationRel = jointAccelerationRel;
		if (VERBOSE_INFO) getLogger().info("Desired joint acceleration: " + jointAccelerationRel);
		return true;
	}

	/** DONE
	 * Define the axis-specific relative jerk (% of maximum jerk)
	 * <p>
	 * jerk: [0.0, 1.0]
	 * 
	 * @param jointJerkRel relative maximum jerk
	 * @return true if it is valid, otherwise false
	 */
	public boolean methSetDesiredJointJerkRel(double jointJerkRel) {
		if (jointJerkRel < 0 || jointJerkRel > 1.0) {
			if (VERBOSE_WARN) getLogger().warn("Invalid joint jerk: " + jointJerkRel);
			this.enumLastError = LibIiwaEnum.VALUE_ERROR;
			return false;
		}
		this.propDesiredJointJerkRel = jointJerkRel;
		if (VERBOSE_INFO) getLogger().info("Desired joint jerk: " + jointJerkRel);
		return true;
	}

	/** DONE
	 * Define the absolute Cartesian velocity (mm/s)
	 * <p>
	 * velocity: (0.0, Inf)
	 * 
	 * @param cartesianVelocity desired maximum Cartesian velocity
	 * @return true if it is valid, otherwise false
	 */
	public boolean methSetDesiredCartesianVelocity(double cartesianVelocity) {
		if (cartesianVelocity <= 0) {
			if (VERBOSE_WARN) getLogger().warn("Invalid cartesian velocity: " + cartesianVelocity);
			this.enumLastError = LibIiwaEnum.VALUE_ERROR;
			return false;
		}
		this.propDesiredCartesianVelocity = cartesianVelocity;
		if (VERBOSE_INFO) getLogger().info("Desired cartesian velocity: " + cartesianVelocity);
		return true;
	}

	/** DONE
	 * Define the absolute Cartesian acceleration (mm/s^2)
	 * <p>
	 * acceleration: (0.0, Inf)
	 * 
	 * @param cartesianAcceleration desired maximum Cartesian acceleration
	 * @return true if it is valid, otherwise false
	 */
	public boolean methSetDesiredCartesianAcceleration(double cartesianAcceleration) {
		if (cartesianAcceleration <= 0) {
			if (VERBOSE_WARN) getLogger().warn("Invalid cartesian acceleration: " + cartesianAcceleration);
			this.enumLastError = LibIiwaEnum.VALUE_ERROR;
			return false;
		}
		this.propDesiredCartesianAcceleration = cartesianAcceleration;
		if (VERBOSE_INFO) getLogger().info("Desired cartesian acceleration: " + cartesianAcceleration);
		return true;
	}

	/** DONE
	 * Define the absolute Cartesian jerk (mm/s^3)
	 * <p>
	 * jerk: (0.0, Inf)
	 * 
	 * @param cartesianJerk desired maximum Cartesian jerk
	 * @return true if it is valid, otherwise false
	 */
	public boolean methSetDesiredCartesianJerk(double cartesianJerk) {
		if (cartesianJerk <= 0) {
			if (VERBOSE_WARN) getLogger().warn("Invalid cartesian jerk: " + cartesianJerk);
			this.enumLastError = LibIiwaEnum.VALUE_ERROR;
			return false;
		}
		this.propDesiredCartesianJerk = cartesianJerk;
		if (VERBOSE_INFO) getLogger().info("Desired cartesian jerk: " + cartesianJerk);
		return true;
	}
	
	// ===========================================================
	// CONDITION METHODS
	// ===========================================================

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
		// stop and reset motion
		this.methStopAndResetMotion();
		
		boolean status = true;
		for (int i = 0; i < 3; i++) {
			this.propForceConditionsEnabled[i] = false;
			if (!(Double.isNaN(condition[i]) || Double.isNaN(condition[i + 3]))) {
				// invalid limits
				if (condition[i] < 0 || condition[i + 3] <= 0) {
					if (VERBOSE_WARN) getLogger().warn("Invalid force condition (threshold / tolerance): " + condition[i] + " " + condition[i + 3]);
					this.enumLastError = LibIiwaEnum.VALUE_ERROR;
				    status = false;
				    continue;
				}
				// set the condition
				this.propForceConditions[i] = ForceCondition.createNormalForceCondition(lbr.getFlange(), _CoordinateAxis(i), condition[i], condition[i + 3]);
				this.propForceConditionsEnabled[i] = true;
			}
		}
		if (VERBOSE_INFO) getLogger().info("Force condition: " + Arrays.toString(condition));
		return status;
	}

	/**
	 * Define the joint torque condition (Nm)
	 * <p>
	 * condition: (-Inf, Inf)
	 * 
	 * @param joint number of the joint (starting at zero)
	 * @param condition lower and upper limits of the torque condition
	 * @return true if it is valid, otherwise false
	 */
	public boolean methSetJointTorqueCondition(int joint, double[] condition) {
		this.propJointTorqueConditionsEnabled[joint] = false;
		// stop and reset motion
		this.methStopAndResetMotion();
		// joint number
		if (joint < 0 || joint >= lbr.getJointCount()) {
			if (VERBOSE_WARN) getLogger().warn("Invalid joint number: " + joint);
			this.enumLastError = LibIiwaEnum.INVALID_JOINT_ERROR;
			return false;
		}
		if (!(Double.isNaN(condition[0]) || Double.isNaN(condition[1]))) {
			// lower and upper limits
			if (condition[0] >= condition[1]) {
				if (VERBOSE_WARN) getLogger().warn("Invalid limits: " + condition[0] + " >= " + condition[1]);
				this.enumLastError = LibIiwaEnum.VALUE_ERROR;
				return false;
			}
			// set the condition
			this.propJointTorqueConditions[joint] = new JointTorqueCondition(_JointEnum(joint), condition[0], condition[1]);
			this.propJointTorqueConditionsEnabled[joint] = true;
		}
		if (VERBOSE_INFO) getLogger().info("Joint torque condition (J" + joint + "): " + condition[0] + " to " + condition[1]);
		return true;
	}
	
	/**
	 * Define the Cartesian impedance control stiffness (translational: Nm, rotational: Nm/rad, null space: Nm/rad)
	 * <p>
	 * This method also affects the Cartesian impedance controller with overlaid force oscillation
	 * <p>
	 * translational: [0.0, 5000.0] (default: 2000.0)
	 * <p>
	 * rotational: [0.0, 300.0] (default: 200.0)
	 * <p>
	 * null space: [0.0, Inf) (default: 100.0)
	 * 
	 * @param stiffness translational (3), rotational (3) and null space (1) stiffness 
	 * @return true
	 */
	public boolean methSetCartesianStiffness(double[] stiffness) {
		// translational
		if (stiffness[0] >= 0.0 && stiffness[0] <= 5000.0) {
			this.propControlModeCartesianImpedance.parametrize(CartDOF.X).setStiffness(stiffness[0]);
			this.propControlModeCartesianSineImpedance.parametrize(CartDOF.X).setStiffness(stiffness[0]);
		}
		if (stiffness[1] >= 0.0 && stiffness[1] <= 5000.0) {
			this.propControlModeCartesianImpedance.parametrize(CartDOF.Y).setStiffness(stiffness[1]);
			this.propControlModeCartesianSineImpedance.parametrize(CartDOF.Y).setStiffness(stiffness[1]);
		}
		if (stiffness[2] >= 0.0 && stiffness[2] <= 5000.0) {
			this.propControlModeCartesianImpedance.parametrize(CartDOF.Z).setStiffness(stiffness[2]);
			this.propControlModeCartesianSineImpedance.parametrize(CartDOF.Z).setStiffness(stiffness[2]);
		}
		// rotational
		if (stiffness[3] >= 0.0 && stiffness[3] <= 300.0) {
			this.propControlModeCartesianImpedance.parametrize(CartDOF.A).setStiffness(stiffness[3]);
			this.propControlModeCartesianSineImpedance.parametrize(CartDOF.A).setStiffness(stiffness[3]);
		}
		if (stiffness[4] >= 0.0 && stiffness[4] <= 300.0) {
			this.propControlModeCartesianImpedance.parametrize(CartDOF.B).setStiffness(stiffness[4]);
			this.propControlModeCartesianSineImpedance.parametrize(CartDOF.B).setStiffness(stiffness[4]);
		}
		if (stiffness[5] >= 0.0 && stiffness[5] <= 300.0) {
			this.propControlModeCartesianImpedance.parametrize(CartDOF.C).setStiffness(stiffness[5]);
			this.propControlModeCartesianSineImpedance.parametrize(CartDOF.C).setStiffness(stiffness[5]);
		}
		// null space
		if (stiffness[6] >= 0.0) {
			this.propControlModeCartesianImpedance.setNullSpaceStiffness(stiffness[6]);
			this.propControlModeCartesianSineImpedance.setNullSpaceStiffness(stiffness[6]);
		}
		
		// update smart servo
		if (this.enumControlInterface == LibIiwaEnum.CONTROL_INTERFACE_SERVO) {
			if (this.enumControlMode == LibIiwaEnum.CONTROL_MODE_CARTESIAN_IMPEDANCE) {
				if (this.propCurrentSmartServoRuntime != null){
					try {
						this.propCurrentSmartServoRuntime.changeControlModeSettings(this.propControlModeCartesianImpedance);
					} 
					catch (Exception e) {
						this.enumLastError = LibIiwaEnum.ASYNCHRONOUS_MOTION_ERROR;
						if (VERBOSE_WARN) getLogger().warn(e.getMessage());
						return false;
					}
				}
			}
			else if (this.enumControlMode == LibIiwaEnum.CONTROL_MODE_CARTESIAN_SINE_IMPEDANCE) {
				if (this.propCurrentSmartServoRuntime != null){
					try {
						this.propCurrentSmartServoRuntime.changeControlModeSettings(this.propControlModeCartesianSineImpedance);
					} 
					catch (Exception e) {
						this.enumLastError = LibIiwaEnum.ASYNCHRONOUS_MOTION_ERROR;
						if (VERBOSE_WARN) getLogger().warn(e.getMessage());
						return false;
					}
				}
			}
		}
		if (VERBOSE_INFO) getLogger().info("Cartesian stiffness: " + Arrays.toString(stiffness));
		return true;
	}
	
	/**
	 * Define the Cartesian impedance control damping
	 * <p>
	 * This method also affects the Cartesian impedance controller with overlaid force oscillation
	 * <p>
	 * translational: [0.1, 1.0] (default: 0.7)
	 * <p>
	 * rotational: [0.1, 1.0] (default: 0.7)
	 * <p>
	 * null space: [0.3, 1.0] (default: 0.7)
	 * 
	 * @param damping translational (3) and rotational (3) damping 
	 * @return true
	 */
	public boolean methSetCartesianDamping(double[] damping) {
		// translational
		if (damping[0] >= 0.1 && damping[0] <= 1.0) {
			this.propControlModeCartesianImpedance.parametrize(CartDOF.X).setDamping(damping[0]);
			this.propControlModeCartesianSineImpedance.parametrize(CartDOF.X).setDamping(damping[0]);
		}
		if (damping[1] >= 0.1 && damping[1] <= 1.0) {
			this.propControlModeCartesianImpedance.parametrize(CartDOF.Y).setDamping(damping[1]);
			this.propControlModeCartesianSineImpedance.parametrize(CartDOF.Y).setDamping(damping[1]);
		}
		if (damping[2] >= 0.1 && damping[2] <= 1.0) {
			this.propControlModeCartesianImpedance.parametrize(CartDOF.Z).setDamping(damping[2]);
			this.propControlModeCartesianSineImpedance.parametrize(CartDOF.Z).setDamping(damping[2]);
		}
		// rotational
		if (damping[3] >= 0.1 && damping[3] <= 1.0) {
			this.propControlModeCartesianImpedance.parametrize(CartDOF.A).setDamping(damping[3]);
			this.propControlModeCartesianSineImpedance.parametrize(CartDOF.A).setDamping(damping[3]);
		}
		if (damping[4] >= 0.1 && damping[4] <= 1.0) {
			this.propControlModeCartesianImpedance.parametrize(CartDOF.B).setDamping(damping[4]);
			this.propControlModeCartesianSineImpedance.parametrize(CartDOF.B).setDamping(damping[4]);
		}
		if (damping[5] >= 0.1 && damping[5] <= 1.0) {
			this.propControlModeCartesianImpedance.parametrize(CartDOF.C).setDamping(damping[5]);
			this.propControlModeCartesianSineImpedance.parametrize(CartDOF.C).setDamping(damping[5]);
		}
		// null space
		if (damping[6] >= 0.3 && damping[6] <= 1.0) {
			this.propControlModeCartesianImpedance.setNullSpaceDamping(damping[6]);
			this.propControlModeCartesianSineImpedance.setNullSpaceDamping(damping[6]);
		}
		
		// update smart servo
		if (this.enumControlInterface == LibIiwaEnum.CONTROL_INTERFACE_SERVO) {
			if (this.enumControlMode == LibIiwaEnum.CONTROL_MODE_CARTESIAN_IMPEDANCE) {
				if (this.propCurrentSmartServoRuntime != null){
					try {
						this.propCurrentSmartServoRuntime.changeControlModeSettings(this.propControlModeCartesianImpedance);
					} 
					catch (Exception e) {
						this.enumLastError = LibIiwaEnum.ASYNCHRONOUS_MOTION_ERROR;
						if (VERBOSE_WARN) getLogger().warn(e.getMessage());
						return false;
					}
				}
			}
			else if (this.enumControlMode == LibIiwaEnum.CONTROL_MODE_CARTESIAN_SINE_IMPEDANCE) {
				if (this.propCurrentSmartServoRuntime != null){
					try {
						this.propCurrentSmartServoRuntime.changeControlModeSettings(this.propControlModeCartesianSineImpedance);
					} 
					catch (Exception e) {
						this.enumLastError = LibIiwaEnum.ASYNCHRONOUS_MOTION_ERROR;
						if (VERBOSE_WARN) getLogger().warn(e.getMessage());
						return false;
					}
				}
			}
		}
		if (VERBOSE_INFO) getLogger().info("Cartesian damping: " + Arrays.toString(damping));
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
		if (VERBOSE_INFO) getLogger().info("Cartesian additional control force: " + Arrays.toString(force));
		return true;
	}
	
	/**
	 * Define the limitation of the maximum force / torque on the TCP (translational: N, rotational: Nm)
	 * <p>
	 * This method also affects the Cartesian impedance controller with overlaid force oscillation
	 * <p>
	 * translational: [0.0, Inf) (default: 1e6)
	 * <p>
	 * rotational: [0.0, Inf) (default: 1e6)
	 * 
	 * @param force translational (3) and rotational (3) limits (force / torque)
	 * @param addStopCondition cancelation of the motion if the maximum force at the TCP is exceeded
	 * @return true
	 */
	public boolean methSetCartesianMaxControlForce(double[] force, boolean addStopCondition) {
		double x = force[0] >= 0 ? force[0] : 1e6;
		double y = force[1] >= 0 ? force[1] : 1e6;
		double z = force[2] >= 0 ? force[2] : 1e6;
		double a = force[3] >= 0 ? force[3] : 1e6;
		double b = force[4] >= 0 ? force[4] : 1e6;
		double c = force[5] >= 0 ? force[5] : 1e6;

		this.propControlModeCartesianImpedance.setMaxControlForce(x, y, z, a, b, c, addStopCondition);
		this.propControlModeCartesianSineImpedance.setMaxControlForce(x, y, z, a, b, c, addStopCondition);
		if (VERBOSE_INFO) getLogger().info("Max Cartesian control force: " + Arrays.toString(force));
		return true;
	}
	
	/**
	 * Define the maximum Cartesian velocity at which motion is aborted if the limit is exceeded (translational: mm/s, rotational: rad/s)
	 * <p>
	 * This method also affects the Cartesian impedance controller with overlaid force oscillation
	 * <p>
	 * translational: [0.0, Inf) (default if invalid: 1e6)
	 * <p>
	 * rotational: [0.0, Inf) (default if invalid: 1e6)
	 * 
	 * @param velocity translational (3) and rotational (3) limits
	 * @return true
	 */
	public boolean methSetCartesianMaxCartesianVelocity(double[] velocity) {
		double x = velocity[0] >= 0 ? velocity[0] : 1e6;
		double y = velocity[1] >= 0 ? velocity[1] : 1e6;
		double z = velocity[2] >= 0 ? velocity[2] : 1e6;
		double a = velocity[3] >= 0 ? velocity[3] : 1e6;
		double b = velocity[4] >= 0 ? velocity[4] : 1e6;
		double c = velocity[5] >= 0 ? velocity[5] : 1e6;
		
		this.propControlModeCartesianImpedance.setMaxCartesianVelocity(x, y, z, a, b, c);
		this.propControlModeCartesianSineImpedance.setMaxCartesianVelocity(x, y, z, a, b, c);
		if (VERBOSE_INFO) getLogger().info("Max Cartesian velocity: " + Arrays.toString(velocity));
		return true;
	}
	
	/**
	 * Define the maximum permissible Cartesian path deviation at which motion is aborted if the limit is exceeded (translational: mm, rotational: rad)
	 * <p>
	 * This method also affects the Cartesian impedance controller with overlaid force oscillation
	 * <p>
	 * translational: [0.0, Inf) (default if invalid: 1e6)
	 * <p>
	 * rotational: [0.0, Inf) (default if invalid: 1e6)
	 * 
	 * @param deviation translational (3) and rotational (3) limits
	 * @return true
	 */
	public boolean methSetCartesianMaxPathDeviation(double[] deviation) {
		double x = deviation[0] >= 0 ? deviation[0] : 1e6;
		double y = deviation[1] >= 0 ? deviation[1] : 1e6;
		double z = deviation[2] >= 0 ? deviation[2] : 1e6;
		double a = deviation[3] >= 0 ? deviation[3] : 1e6;
		double b = deviation[4] >= 0 ? deviation[4] : 1e6;
		double c = deviation[5] >= 0 ? deviation[5] : 1e6;
		
		this.propControlModeCartesianImpedance.setMaxPathDeviation(x, y, z, a, b, c);
		this.propControlModeCartesianSineImpedance.setMaxPathDeviation(x, y, z, a, b, c);
		if (VERBOSE_INFO) getLogger().info("Max Cartesian path deviation: " + Arrays.toString(deviation));
		return true;
	}
	
	/**
	 * Define the amplitude of the force oscillation (translational: N, rotational: Nm)
	 * <p>
	 * translational: [0.0, Inf) (default if invalid: 0)
	 * <p>
	 * rotational: [0.0, Inf) (default if invalid: 0)
	 * 
	 * @param amplitude translational (3) and rotational (3) amplitude
	 * @return true
	 */
	public boolean methSetCartesianAmplitude(double[] amplitude) {
		double x = amplitude[0] >= 0 ? amplitude[0] : 0.0;
		double y = amplitude[1] >= 0 ? amplitude[1] : 0.0;
		double z = amplitude[2] >= 0 ? amplitude[2] : 0.0;
		double a = amplitude[3] >= 0 ? amplitude[3] : 0.0;
		double b = amplitude[4] >= 0 ? amplitude[4] : 0.0;
		double c = amplitude[5] >= 0 ? amplitude[5] : 0.0;

		// translational
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.X).setAmplitude(x);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.Y).setAmplitude(y);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.Z).setAmplitude(z);
		// rotational
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.A).setAmplitude(a);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.B).setAmplitude(b);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.C).setAmplitude(c);
		if (VERBOSE_INFO) getLogger().info("Cartesian (sine) amplitude: " + Arrays.toString(amplitude));
		return true;
	}
	
	/**
	 * Define the frequency of the force oscillation (frequency: Hz)
	 * <p>
	 * frequency: [0.0, 15.0] (default if invalid: 0)
	 * 
	 * @param frequency translational (3) and rotational (3) frequency
	 * @return true
	 */
	public boolean methSetCartesianFrequency(double[] frequency) {
		double x = frequency[0] >= 0 ? frequency[0] : 0.0;
		double y = frequency[1] >= 0 ? frequency[1] : 0.0;
		double z = frequency[2] >= 0 ? frequency[2] : 0.0;
		double a = frequency[3] >= 0 ? frequency[3] : 0.0;
		double b = frequency[4] >= 0 ? frequency[4] : 0.0;
		double c = frequency[5] >= 0 ? frequency[5] : 0.0;

		// translational
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.X).setFrequency(x);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.Y).setFrequency(y);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.Z).setFrequency(z);
		// rotational
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.A).setFrequency(a);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.B).setFrequency(b);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.C).setFrequency(c);
		if (VERBOSE_INFO) getLogger().info("Cartesian (sine) frequency: " + Arrays.toString(frequency));
		return true;
	}
	
	/**
	 * Define the phase offset of the force oscillation at the start of the force overlay (phase: degrees)
	 * <p>
	 * phase: [0.0, Inf) (default if invalid: 0)
	 * 
	 * @param phase translational (3) and rotational (3) phase
	 * @return true
	 */
	public boolean methSetCartesianPhaseDeg(double[] phase) {
		double x = phase[0] >= 0 ? phase[0] : 0.0;
		double y = phase[1] >= 0 ? phase[1] : 0.0;
		double z = phase[2] >= 0 ? phase[2] : 0.0;
		double a = phase[3] >= 0 ? phase[3] : 0.0;
		double b = phase[4] >= 0 ? phase[4] : 0.0;
		double c = phase[5] >= 0 ? phase[5] : 0.0;

		// translational
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.X).setPhaseDeg(x);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.Y).setPhaseDeg(y);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.Z).setPhaseDeg(z);
		// rotational
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.A).setPhaseDeg(a);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.B).setPhaseDeg(b);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.C).setPhaseDeg(c);
		if (VERBOSE_INFO) getLogger().info("Cartesian (sine) phase: " + Arrays.toString(phase));
		return true;
	}
	
	/**
	 * Define a constant force overlaid in addition to the
overlaid force oscillation (translational: N, rotational: Nm)
	 * <p>
	 * translational: (-Inf, Inf) (default 0)
	 * <p>
	 * rotational: (-Inf, Inf) (default 0)
	 * 
	 * @param bias translational (3) and rotational (3) bias
	 * @return true
	 */
	public boolean methSetCartesianBias(double[] bias) {
		// translational
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.X).setBias(bias[0]);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.Y).setBias(bias[1]);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.Z).setBias(bias[2]);
		// rotational
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.A).setBias(bias[3]);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.B).setBias(bias[4]);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.C).setBias(bias[5]);
		if (VERBOSE_INFO) getLogger().info("Cartesian (sine) bias: " + Arrays.toString(bias));
		return true;
	}
	
	/**
	 * Define the force limitation of the force oscillation (translational: N, rotational: Nm)
	 * <p>
	 * translational: [0.0, Inf) (default if invalid: 1e6)
	 * <p>
	 * rotational: [0.0, Inf) (default if invalid: 1e6)
	 * 
	 * @param force translational (3) and rotational (3) force
	 * @return true
	 */
	public boolean methSetCartesianForceLimit(double[] force) {
		double x = force[0] >= 0 ? force[0] : 1e6;
		double y = force[1] >= 0 ? force[1] : 1e6;
		double z = force[2] >= 0 ? force[2] : 1e6;
		double a = force[3] >= 0 ? force[3] : 1e6;
		double b = force[4] >= 0 ? force[4] : 1e6;
		double c = force[5] >= 0 ? force[5] : 1e6;

		// translational
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.X).setForceLimit(x);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.Y).setForceLimit(y);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.Z).setForceLimit(z);
		// rotational
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.A).setForceLimit(a);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.B).setForceLimit(b);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.C).setForceLimit(c);
		if (VERBOSE_INFO) getLogger().info("Cartesian (sine) force limit: " + Arrays.toString(force));
		return true;
	}
	
	/**
	 * Define the maximum deflection due to the force oscillation (translational: mm, rotational: radians)
	 * <p>
	 * translational: [0.0, Inf) (default if invalid: 1e6)
	 * <p>
	 * rotational: [0.0, Inf) (default if invalid: 1e6)
	 * 
	 * @param position translational (3) and rotational (3) position
	 * @return true
	 */
	public boolean methSetCartesianPositionLimit(double[] position) {
		double x = position[0] >= 0 ? position[0] : 1e6;
		double y = position[1] >= 0 ? position[1] : 1e6;
		double z = position[2] >= 0 ? position[2] : 1e6;
		double a = position[3] >= 0 ? position[3] : 1e6;
		double b = position[4] >= 0 ? position[4] : 1e6;
		double c = position[5] >= 0 ? position[5] : 1e6;

		// translational
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.X).setPositionLimit(x);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.Y).setPositionLimit(y);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.Z).setPositionLimit(z);
		// rotational
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.A).setPositionLimit(a);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.B).setPositionLimit(b);
		this.propControlModeCartesianSineImpedance.parametrize(CartDOF.C).setPositionLimit(c);
		if (VERBOSE_INFO) getLogger().info("Cartesian (sine) position limit: " + Arrays.toString(position));
		return true;
	}
	
	/**
	 * Define the overall duration of the force oscillation (time: seconds)
	 * <p>
	 * time: [0.0, Inf) (default if invalid: 1e9)
	 * 
	 * @param time duration of the force oscillation
	 * @return true
	 */
	public boolean methSetCartesianTotalTime(double time) {
		time = time >= 0 ? time : 1e9;

		this.propControlModeCartesianSineImpedance.setTotalTime(time);
		if (VERBOSE_INFO) getLogger().info("Cartesian (sine) total time: " + time);
		return true;
	}
	
	/**
	 * Define the rise time of the force oscillation (time: seconds)
	 * <p>
	 * time: [0.0, Inf) (default if invalid: 0)
	 * 
	 * @param time rise time
	 * @return true
	 */
	public boolean methSetCartesianRiseTime(double time) {
		time = time >= 0 ? time : 0.0;

		this.propControlModeCartesianSineImpedance.setRiseTime(time);
		if (VERBOSE_INFO) getLogger().info("Cartesian (sine) rise time: " + time);
		return true;
	}
	
	/**
	 * Define the hold time of the force oscillation (time: seconds)
	 * <p>
	 * time: [0.0, Inf) (default if invalid: 1e9)
	 * 
	 * @param time hold time
	 * @return true
	 */
	public boolean methSetCartesianHoldTime(double time) {
		time = time >= 0 ? time : 1e9;

		this.propControlModeCartesianSineImpedance.setHoldTime(time);
		if (VERBOSE_INFO) getLogger().info("Cartesian (sine) hold time: " + time);
		return true;
	}
	
	/**
	 * Define the fall time of the force oscillation (time: seconds)
	 * <p>
	 * time: [0.0, Inf) (default if invalid: 0)
	 * 
	 * @param time fall time
	 * @return true
	 */
	public boolean methSetCartesianFallTime(double time) {
		time = time >= 0 ? time : 0.0;

		this.propControlModeCartesianSineImpedance.setFallTime(time);
		if (VERBOSE_INFO) getLogger().info("Cartesian (sine) fall time: " + time);
		return true;
	}
	
	/**
	 * Define whether the oscillation is terminated or continued after the end of the motion
	 * 
	 * @param active if the oscillation is continued after the end of the motion
	 * @return true
	 */
	public boolean methSetCartesianStayActiveUntilPatternFinished(boolean active) {
		this.propControlModeCartesianSineImpedance.setStayActiveUntilPatternFinished(active);
		if (VERBOSE_INFO) getLogger().info("Cartesian (sine) stay active: " + active);
		return true;
	}
	
	/**
	 * Overlay a constant force, in one Cartesian direction, that does not change over time
	 * 
	 * @param overlay Cartesian DOF (1), force (1) and stiffness (1)
	 * @return true
	 */
	public boolean methSetCartesianCreateDesiredForce(double[] overlay) {
		CartDOF dof = _CartDOF((int)Math.round(overlay[0]));
		this.propControlModeCartesianSineImpedance = CartesianSineImpedanceControlMode.createDesiredForce(dof, overlay[1], overlay[2]);
		if (this.enumControlMode == LibIiwaEnum.CONTROL_MODE_CARTESIAN_SINE_IMPEDANCE)
			this.propCurrentControlMode = this.propControlModeCartesianSineImpedance;
		if (VERBOSE_INFO) getLogger().info("Cartesian (sine) create desired force: " + Arrays.toString(overlay));
		return true;
	}
	
	/**
	 * Overlay a simple force oscillation in one Cartesian direction
	 * 
	 * @param overlay Cartesian DOF (1), frequency (1), amplitude (1) and stiffness (1)
	 * @return true
	 */
	public boolean methSetCartesianCreateSinePattern(double[] overlay) {
		CartDOF dof = _CartDOF((int)Math.round(overlay[0]));
		this.propControlModeCartesianSineImpedance = CartesianSineImpedanceControlMode.createSinePattern(dof, overlay[1], overlay[2], overlay[3]);
		if (this.enumControlMode == LibIiwaEnum.CONTROL_MODE_CARTESIAN_SINE_IMPEDANCE)
			this.propCurrentControlMode = this.propControlModeCartesianSineImpedance;
		if (VERBOSE_INFO) getLogger().info("Cartesian (sine) create sine pattern: " + Arrays.toString(overlay));
		return true;
	}
	
	/**
	 * Overlay a 2-dimensional oscillation in one plane
	 * 
	 * @param overlay plane (1), frequency (1), amplitude (1) and stiffness (1)
	 * @return true
	 */
	public boolean methSetCartesianCreateLissajousPattern(double[] overlay) {
		CartPlane plane = _CartPlane((int)Math.round(overlay[0]));
		this.propControlModeCartesianSineImpedance = CartesianSineImpedanceControlMode.createLissajousPattern(plane, overlay[1], overlay[2], overlay[3]);
		if (this.enumControlMode == LibIiwaEnum.CONTROL_MODE_CARTESIAN_SINE_IMPEDANCE)
			this.propCurrentControlMode = this.propControlModeCartesianSineImpedance;
		if (VERBOSE_INFO) getLogger().info("Cartesian (sine) create Lissajous pattern: " + Arrays.toString(overlay));
		return true;
	}
	
	/**
	 * Overlay a spiral-shaped force oscillation in one plane
	 * 
	 * @param overlay plane (1), frequency (1), amplitude (1), stiffness (1) and total time (1)
	 * @return true
	 */
	public boolean methSetCartesianCreateSpiralPattern(double[] overlay) {
		CartPlane plane = _CartPlane((int)Math.round(overlay[0]));
		this.propControlModeCartesianSineImpedance = CartesianSineImpedanceControlMode.createSpiralPattern(plane, overlay[1], overlay[2], overlay[3], overlay[4]);
		if (this.enumControlMode == LibIiwaEnum.CONTROL_MODE_CARTESIAN_SINE_IMPEDANCE)
			this.propCurrentControlMode = this.propControlModeCartesianSineImpedance;
		if (VERBOSE_INFO) getLogger().info("Cartesian (sine) create spiral pattern: " + Arrays.toString(overlay));
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
		if (VERBOSE_INFO) getLogger().info("Joint stiffness: " + stiffness);
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
		if (VERBOSE_INFO) getLogger().info("Joint damping: " + damping);
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
			if (VERBOSE_WARN) getLogger().warn("Invalid control interface: " + controlInterface.getCode());
			this.enumLastError = LibIiwaEnum.VALUE_ERROR;
			return false;
		}

		if (VERBOSE_INFO) getLogger().info("Control interface: " + this.enumControlInterface.toString());
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
			if (VERBOSE_WARN) getLogger().warn("Invalid motion type: " + motionType.getCode());
			this.enumLastError = LibIiwaEnum.VALUE_ERROR;
			return false;
		}

		if (VERBOSE_INFO) getLogger().info("Motion type: " + this.enumMotionType.toString());
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
			if (VERBOSE_WARN) getLogger().warn("Invalid control mode: " + controlMode.getCode());
			this.enumLastError = LibIiwaEnum.VALUE_ERROR;
			return false;
		}

		if (VERBOSE_INFO) getLogger().info("Control mode: " + this.enumControlMode.toString());
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
			this.enumExecutionType = LibIiwaEnum.EXECUTION_TYPE_ASYNCHRONOUS;
		else if (executionType.getCode() == LibIiwaEnum.EXECUTION_TYPE_SYNCHRONOUS.getCode())
			this.enumExecutionType = LibIiwaEnum.EXECUTION_TYPE_SYNCHRONOUS;
		else {
			if (VERBOSE_WARN) getLogger().warn("Invalid execution type: " + executionType.getCode());
			this.enumLastError = LibIiwaEnum.VALUE_ERROR;
			return false;
		}

		if (VERBOSE_INFO) getLogger().info("Execution type: " + this.enumExecutionType.toString());
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
			if (VERBOSE_WARN) getLogger().warn("Invalid communication mode: " + communicationMode.getCode());
			return false;
		}

		if (VERBOSE_INFO) getLogger().info("Communication mode: " + this.propCommunicationMode.toString());
		return true;
	}

	// ===========================================================
	// TOOLS
	// ===========================================================

	/** PARTIAL
	 * Attach/detach a tool
	 * 
	 * @param index tool index
	 * @return true if the control was successful, otherwise false
	 */
	public boolean methSetTool(int index) {
		// stop and reset motion
		this.methStopAndResetMotion();
		
		// get tool names
		String raw = this.propApplicationData.getProcessData("tools").getValue();
		String[] tools = raw.trim().split("\\s*,\\s*");
		if (VERBOSE_INFO) getLogger().info("Tools: " + Arrays.toString(tools));
		if (index >= tools.length) {
			this.enumLastError = LibIiwaEnum.VALUE_ERROR;
			if (VERBOSE_WARN) getLogger().warn("Invalid tool index: " + index);
			return false;
		}
		
		// detach tool
		if (this.tool != null) {
			this.tool.detach();
			this.tool = null;
			if (VERBOSE_INFO) getLogger().info("Tool detached");
		}
		if (index < 0) 
			return true;
		
		// attach tool
		try {
			this.tool = getApplicationData().createFromTemplate(tools[index]);
			this.tool.attachTo(this.lbr.getFlange());
			if (VERBOSE_INFO) getLogger().info("Tool attached: " + tools[index]);
		}
		catch (Exception e) {
			this.tool = null;
			this.enumLastError = LibIiwaEnum.INVALID_CONFIGURATION_ERROR;
			if (VERBOSE_WARN) getLogger().warn(e.getMessage());
			return false;
		}
		return true;
	}

	// ===========================================================
	// CONTROL METHODS
	// ===========================================================
	/** PARTIAL
	 * Stop the robot
	 * 
	 * @return true if the control was successful, otherwise false
	 */
	private boolean methStop() {
		this.methStopAndResetMotion();
		return true;
	}
	
	/** PARTIAL
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

		if (VERBOSE_INFO) getLogger().info(String.format("Joint position %1$s", Arrays.toString(joints)));

		// validate
		for (int i = 0; i < 7; i++)
			if (!Double.isNaN(joints[i]))
				if (joints[i] >= propMinJointPositionLimits.get(i) && joints[i] <= propMaxJointPositionLimits.get(i))
					jointPosition.set(i, joints[i]);

		if (VERBOSE_INFO) getLogger().info(String.format("Joint position %1$s", Arrays.toString(jointPosition.get())));

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

	/** PARTIAL
	 * Control robot using Cartesian pose (in millimeters and radians)
	 * 
	 * @param pose Cartesian position (3) and orientation (3)
	 * @return true if the control was successful, otherwise false
	 */
	private boolean methGoToCartesianPose(double[] pose) {
		// TODO: add LINREL
		ObjectFrame objectFrame = this.tool == null ? this.lbr.getFlange() : this.tool.getDefaultMotionFrame();
		Frame frame = this.lbr.getCurrentCartesianPosition(objectFrame);

		if (VERBOSE_INFO) getLogger().info(String.format("Cartesian pose [%.4f %.4f %.4f] [%.4f %.4f %.4f]", 
				pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]));

		// validate
		if (!Double.isNaN(pose[0]))
			frame.setX(pose[0]);
		if (!Double.isNaN(pose[1]))
			frame.setY(pose[1]);
		if (!Double.isNaN(pose[2]))
			frame.setZ(pose[2]);
		if (!Double.isNaN(pose[3]))
			frame.setAlphaRad(pose[3]);
		if (!Double.isNaN(pose[4]))
			frame.setBetaRad(pose[4]);
		if (!Double.isNaN(pose[5]))
			frame.setGammaRad(pose[5]);

		if (VERBOSE_INFO) getLogger().info(String.format("Cartesian pose [%.4f %.4f %.4f] [%.4f %.4f %.4f]", 
				frame.getX(), frame.getY(), frame.getZ(), frame.getAlphaRad(), frame.getBetaRad(), frame.getGammaRad()));

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
	
	/** DONE
	 * Perform a circular motion (in millimeters)
	 * 
	 * @param circ Auxiliary position (3) and end position (3)
	 * @return true if the control was successful, otherwise false
	 */
	private boolean methGoToCirc(double[] circ) {
		ObjectFrame objectFrame = this.tool == null ? this.lbr.getFlange() : this.tool.getDefaultMotionFrame();
		Frame frame = this.lbr.getCurrentCartesianPosition(objectFrame);
		
		Frame auxiliaryPoint = new Frame(circ[0], circ[1], circ[2], frame.getAlphaRad(), frame.getBetaRad(), frame.getGammaRad());
		Frame endPoint = new Frame(circ[3], circ[4], circ[5], frame.getAlphaRad(), frame.getBetaRad(), frame.getGammaRad());
		MotionBatch motionBatch = new MotionBatch(BasicMotions.circ(auxiliaryPoint, endPoint).setJointJerkRel(this.propDesiredJointJerkRel).
				setCartVelocity(this.propDesiredCartesianVelocity).setCartAcceleration(this.propDesiredCartesianAcceleration));
		
		if (VERBOSE_INFO) getLogger().info(String.format("CIRC motion [%.4f %.4f %.4f] [%.4f %.4f %.4f]", 
				auxiliaryPoint.getX(), auxiliaryPoint.getY(), auxiliaryPoint.getZ(), endPoint.getX(), endPoint.getY(), endPoint.getZ()));
		
		// move
		if (enumControlInterface == LibIiwaEnum.CONTROL_INTERFACE_STANDARD) {
			return methMoveStandard(motionBatch);
		}
		else if (enumControlInterface == LibIiwaEnum.CONTROL_INTERFACE_SERVO) {
			this.enumLastError = LibIiwaEnum.INVALID_CONFIGURATION_ERROR;
			if (VERBOSE_WARN) getLogger().warn("Invalid configuration: CIRC motion is not implemented for Servo");
			return false;
		}
		return false;
	}

	// ===========================================================
	// 
	// ===========================================================

	private double[] methUpdateAndGetCurrentState() {
		JointPosition jointPosition = lbr.getCurrentJointPosition();
		ObjectFrame objectFrame = this.tool == null ? this.lbr.getFlange() : this.tool.getDefaultMotionFrame();
		Frame frame = this.lbr.getCurrentCartesianPosition(objectFrame);
		TorqueSensorData torqueSensorData = lbr.getExternalTorque();
		ForceSensorData forceSensorData = lbr.getExternalForceTorque(lbr.getFlange());  // TODO: objectFrame
		
		// conditions
		IFiredConditionInfo firedInfo = null;
		if (this.propCurrentStandardMotionContainer != null)
			try {
				firedInfo = this.propCurrentStandardMotionContainer.getFiredBreakConditionInfo();
			}
			catch (Exception e) {
				// TODO: handle exception
			}
		this.propCurrentState[STATE_FIRED_CONDITION] = firedInfo == null ? 0 : 1;
		
		this.propCurrentState[STATE_READY_TO_MOVE] = lbr.isReadyToMove() ? 1 : 0;
		this.propCurrentState[STATE_HAS_ACTIVE_MOTION] = lbr.hasActiveMotionCommand() ? 1 : 0;

		// joint position
		this.propCurrentState[STATE_JOINT_POSITION + 0] = jointPosition.get(JointEnum.J1);
		this.propCurrentState[STATE_JOINT_POSITION + 1] = jointPosition.get(JointEnum.J2);
		this.propCurrentState[STATE_JOINT_POSITION + 2] = jointPosition.get(JointEnum.J3);
		this.propCurrentState[STATE_JOINT_POSITION + 3] = jointPosition.get(JointEnum.J4);
		this.propCurrentState[STATE_JOINT_POSITION + 4] = jointPosition.get(JointEnum.J5);
		this.propCurrentState[STATE_JOINT_POSITION + 5] = jointPosition.get(JointEnum.J6);
		this.propCurrentState[STATE_JOINT_POSITION + 6] = jointPosition.get(JointEnum.J7);

		// joint velocity
		for (int i = 0; i < lbr.getJointCount(); i++)
			this.propCurrentState[STATE_JOINT_VELOCITY + i] = this.propCurrentJointVelocity[i];
		
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
		int commandCode = (int)Math.round(command[0]);
		// empty commands
		if (commandCode == LibIiwaEnum.COMMAND_PASS.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_PASS.toString());
			return true;
		}
		// move command
		else if (commandCode == LibIiwaEnum.COMMAND_STOP.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_STOP.toString());
			return this.methStop();
		}
		else if (commandCode == LibIiwaEnum.COMMAND_JOINT_POSITION.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_JOINT_POSITION.toString());
			return this.methGoToJointPosition(Arrays.copyOfRange(command, 1, 1 + 7));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_CARTESIAN_POSE.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_CARTESIAN_POSE.toString());
			return this.methGoToCartesianPose(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_CIRC_MOTION.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_CIRC_MOTION.toString());
			return this.methGoToCirc(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		// configuration commands (tool)
		else if (commandCode == LibIiwaEnum.COMMAND_SET_TOOL.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_TOOL.toString());
			return this.methSetTool((int)Math.round(command[1]));
		}
		// configuration commands (limits and constants)
		else if (commandCode == LibIiwaEnum.COMMAND_SET_DESIRED_JOINT_VELOCITY_REL.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_DESIRED_JOINT_VELOCITY_REL.toString());
			return this.methSetDesiredJointVelocityRel(command[1]);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_DESIRED_JOINT_ACCELERATION_REL.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_DESIRED_JOINT_ACCELERATION_REL.toString());
			return this.methSetDesiredJointAccelerationRel(command[1]);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_DESIRED_JOINT_JERK_REL.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_DESIRED_JOINT_JERK_REL.toString());
			return this.methSetDesiredJointJerkRel(command[1]);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_DESIRED_CARTESIAN_VELOCITY.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_DESIRED_CARTESIAN_VELOCITY.toString());
			return this.methSetDesiredCartesianVelocity(command[1]);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_DESIRED_CARTESIAN_ACCELERATION.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_DESIRED_CARTESIAN_ACCELERATION.toString());
			return this.methSetDesiredCartesianAcceleration(command[1]);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_DESIRED_CARTESIAN_JERK.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_DESIRED_CARTESIAN_JERK.toString());
			return this.methSetDesiredCartesianJerk(command[1]);
		}
		// configuration commands (conditions)
		else if (commandCode == LibIiwaEnum.COMMAND_SET_FORCE_CONDITION.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_FORCE_CONDITION.toString());
			return this.methSetForceCondition(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_JOINT_TORQUE_CONDITION.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_JOINT_TORQUE_CONDITION.toString());
			return this.methSetJointTorqueCondition((int)Math.round(command[1]), Arrays.copyOfRange(command, 2, 2 + 2));
		}
		// configuration commands (impedance control)
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_STIFFNESS.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_STIFFNESS.toString());
			return this.methSetCartesianStiffness(Arrays.copyOfRange(command, 1, 1 + 7));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_DAMPING.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_DAMPING.toString());
			return this.methSetCartesianDamping(Arrays.copyOfRange(command, 1, 1 + 7));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_ADDITIONAL_CONTROL_FORCE.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_ADDITIONAL_CONTROL_FORCE.toString());
			return this.methSetCartesianAdditionalControlForce(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_MAX_CONTROL_FORCE.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_MAX_CONTROL_FORCE.toString());
			boolean addStopCondition = command[7] > 0;
			return this.methSetCartesianMaxControlForce(Arrays.copyOfRange(command, 1, 1 + 6), addStopCondition);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_MAX_CARTESIAN_VELOCITY.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_MAX_CARTESIAN_VELOCITY.toString());
			return this.methSetCartesianMaxCartesianVelocity(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_MAX_PATH_DEVIATION.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_MAX_PATH_DEVIATION.toString());
			return this.methSetCartesianMaxPathDeviation(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_JOINT_STIFFNESS.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_JOINT_STIFFNESS.toString());
			return this.methSetJointStiffness(Arrays.copyOfRange(command, 1, 1 + 7));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_JOINT_DAMPING.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_JOINT_DAMPING.toString());
			return this.methSetJointDamping(Arrays.copyOfRange(command, 1, 1 + 7));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_AMPLITUDE.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_AMPLITUDE.toString());
			return this.methSetCartesianAmplitude(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_FREQUENCY.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_FREQUENCY.toString());
			return this.methSetCartesianFrequency(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_PHASE.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_PHASE.toString());
			return this.methSetCartesianPhaseDeg(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_BIAS.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_BIAS.toString());
			return this.methSetCartesianBias(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_FORCE_LIMIT.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_FORCE_LIMIT.toString());
			return this.methSetCartesianForceLimit(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_POSITION_LIMIT.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_POSITION_LIMIT.toString());
			return this.methSetCartesianPositionLimit(Arrays.copyOfRange(command, 1, 1 + 6));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_TOTAL_TIME.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_TOTAL_TIME.toString());
			return this.methSetCartesianTotalTime(command[1]);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_RISE_TIME.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_RISE_TIME.toString());
			return this.methSetCartesianRiseTime(command[1]);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_HOLD_TIME.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_HOLD_TIME.toString());
			return this.methSetCartesianHoldTime(command[1]);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_FALL_TIME.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_FALL_TIME.toString());
			return this.methSetCartesianFallTime(command[1]);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_STAY_ACTIVE_UNTIL_PATTERN_FINISHED.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_STAY_ACTIVE_UNTIL_PATTERN_FINISHED.toString());
			boolean active = command[1] > 0;
			return this.methSetCartesianStayActiveUntilPatternFinished(active);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_CREATE_DESIRED_FORCE.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_CREATE_DESIRED_FORCE.toString());
			return this.methSetCartesianCreateDesiredForce(Arrays.copyOfRange(command, 1, 1 + 3));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_CREATE_SINE_PATTERN.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_CREATE_SINE_PATTERN.toString());
			return this.methSetCartesianCreateSinePattern(Arrays.copyOfRange(command, 1, 1 + 4));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_CREATE_LISSAJOUS_PATTERN.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_CREATE_LISSAJOUS_PATTERN.toString());
			return this.methSetCartesianCreateLissajousPattern(Arrays.copyOfRange(command, 1, 1 + 4));
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_CREATE_SPIRAL_PATTERN.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CARTESIAN_SINE_CREATE_SPIRAL_PATTERN.toString());
			return this.methSetCartesianCreateSpiralPattern(Arrays.copyOfRange(command, 1, 1 + 5));
		}
		// configuration commands (motion and control)
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CONTROL_INTERFACE.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CONTROL_INTERFACE.toString());
			LibIiwaEnum controlInterface = LibIiwaEnum.CONTROL_INTERFACE;
			controlInterface.setCode(command[1]);
			return this.methSetControlInterface(controlInterface);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_MOTION_TYPE.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_MOTION_TYPE.toString());
			LibIiwaEnum motionType = LibIiwaEnum.MOTION_TYPE;
			motionType.setCode(command[1]);
			return this.methSetMotionType(motionType);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_CONTROL_MODE.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_CONTROL_MODE.toString());
			LibIiwaEnum controlMode = LibIiwaEnum.CONTROL_MODE;
			controlMode.setCode(command[1]);
			return this.methSetControlMode(controlMode);
		}
		else if (commandCode == LibIiwaEnum.COMMAND_SET_EXECUTION_TYPE.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_EXECUTION_TYPE.toString());
			LibIiwaEnum executionType = LibIiwaEnum.EXECUTION_TYPE;
			executionType.setCode(command[1]);
			return this.methSetExecutionType(executionType);
		}
		// configuration commands (communication)
		else if (commandCode == LibIiwaEnum.COMMAND_SET_COMMUNICATION_MODE.getCode()){
			if (VERBOSE_INFO) getLogger().info(LibIiwaEnum.COMMAND_SET_COMMUNICATION_MODE.toString());
			LibIiwaEnum communicationMode = LibIiwaEnum.COMMUNICATION_MODE;
			communicationMode.setCode(command[1]);
			return this.methSetCommunicationMode(communicationMode);
		}
		if (VERBOSE_WARN) getLogger().warn("Unknown command code: " + commandCode);
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
		state[STATE_LAST_ERROR] = this.enumLastError.getCode();
		// if (VERBOSE_INFO) getLogger().info("STATE: " + state[STATE_COMMAND_STATUS] + " " + state[STATE_LAST_ERROR]);
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

	private Thread propThreadComputeVelocity = new Thread() {
		@Override
		public void run() {
			long previousTimeStamp = 0;
			double[] previousPosition = new double[lbr.getJointCount()];

			if (VERBOSE_INFO) getLogger().info("App.thread: ComputeVelocity started");
			
			while(propShouldContinue) {
				long currentTimeStamp = System.nanoTime();
				double[] currentPosition = lbr.getCurrentJointPosition().getInternalArray();	
			    if (previousTimeStamp != 0) {
			      for (int i = 0; i < lbr.getJointCount(); i++)
			    	  propCurrentJointVelocity[i] = (currentPosition[i] - previousPosition[i]) / ((double)(currentTimeStamp - previousTimeStamp) / 1000000000);
			    }
			    previousPosition = currentPosition;
			    previousTimeStamp = currentTimeStamp;
				ThreadUtil.milliSleep(10);
			}
			
			if (VERBOSE_INFO) getLogger().info("App.thread: ComputeVelocity stopped");
		}
	};

	@Override
	public void initialize() {
		// application data
		this.propApplicationData = getApplicationData();
		VERBOSE_INFO = this.propApplicationData.getProcessData("verbose_info").getValue();
		VERBOSE_WARN = this.propApplicationData.getProcessData("verbose_warn").getValue();
		
		// TODO: get robot and tool
		getController(CONTROLLER_NAME);
		this.lbr = getContext().getDeviceFromType(LBR.class);

		// initialize variables
		this.propMaxJointPositionLimits = lbr.getJointLimits().getMaxJointPosition();
		this.propMinJointPositionLimits = lbr.getJointLimits().getMinJointPosition();

		this.propCurrentState = new double[STATE_LENGTH];
		this.propCurrentJointVelocity = new double[lbr.getJointCount()];

		boolean double_precision = this.propApplicationData.getProcessData("communication_double_precision").getValue();
		this.propCommunication = new LibIiwaCommunication(getLogger(), COMMAND_LENGTH, double_precision);
		
		this.propControlModePosition = new PositionControlMode();
		this.propControlModeJointImpedance = new JointImpedanceControlMode(2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0);
		this.propControlModeCartesianImpedance = new CartesianImpedanceControlMode();
		this.propControlModeCartesianSineImpedance = new CartesianSineImpedanceControlMode();
		this.propCurrentControlMode = this.propControlModePosition;

		// initialize conditions
		this.propJointTorqueConditions = new JointTorqueCondition[lbr.getJointCount()];
		this.propJointTorqueConditionsEnabled = new boolean[lbr.getJointCount()];
		for (int i = 0; i < lbr.getJointCount(); i++) {
			this.propJointTorqueConditions[i] = new JointTorqueCondition(_JointEnum(i), -1e6, 1e6);
			this.propJointTorqueConditionsEnabled[i] = false;
		}

		this.propForceConditions = new ForceCondition[lbr.getJointCount()];
		this.propForceConditionsEnabled = new boolean[lbr.getJointCount()];
		for (int i = 0; i < 3; i++) {
			this.propForceConditions[i] = ForceCondition.createNormalForceCondition(lbr.getFlange(), _CoordinateAxis(i), 1e6);
			this.propForceConditionsEnabled[i] = false;
		}

		// error handler
		this.propErrorHandler = new IErrorHandler() {	
			@Override
			public ErrorHandlingAction handleError(Device device, IMotionContainer failedContainer, List<IMotionContainer> canceledContainers) {
				enumLastError = LibIiwaEnum.ASYNCHRONOUS_MOTION_ERROR;
				if (VERBOSE_WARN) getLogger().warn("Excecution of the following motion failed: " + failedContainer.getCommand().toString());
				if (VERBOSE_WARN) getLogger().warn("The following motions will not be executed:");
				for (int i = 0; i < canceledContainers.size(); i++)
					if (VERBOSE_WARN) getLogger().warn(" - " + canceledContainers.get(i).getCommand().toString());
				return ErrorHandlingAction.Ignore;
			}
		};
		getApplicationControl().registerMoveAsyncErrorHandler(this.propErrorHandler);

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
		getLogger().info("App: Disposal request");
		// stop application loop
		this.propShouldContinue = false;
		// stop robot
		this.methStopAndResetMotion();
		// close socket
		this.propCommunication.methCloseSocketConnection();
		getLogger().info("App: Bye!");
	}

	@Override
	public void run() {
		getLogger().info("");
		getLogger().info("App: application started");
		
		// initialize communication
		String address = this.propApplicationData.getProcessData("controller_ip").getValue();
		int port = this.propApplicationData.getProcessData("controller_port").getValue();
		
		this.propShouldContinue = this.propCommunication.methInitializeSocketConnection(address, port, false, 0, 1000);  // as client
		this.propCommunicationMode = LibIiwaEnum.COMMUNICATION_MODE_ON_DEMAND;
		this.propCommunication.methSetSocketTimeout(0);
		
		// start velocity computation thread
		this.propThreadComputeVelocity.start();

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
		getLogger().info("App: application stopped");
	}
}

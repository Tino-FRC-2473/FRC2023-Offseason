package frc.robot.systems;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorArmFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		IDLE,
		HIGH,
		MIDDLE,
		LOW,
		MOVING,
		ZEROING
	}

	private static final float UP_POWER = 0.1f;
	private static final float DOWN_POWER = -0.1f;
	private static final float ZEROING_POWER = -0.1f;
	private static final double PID_CONSTANT_ARM_P = 0.005;
	private static final double PID_CONSTANT_ARM_I = 0.00000001;
	private static final double PID_CONSTANT_ARM_D = 0.00000001;
	private static final float MAX_UP_POWER = 0.1f;
	private static final float MAX_DOWN_POWER = -0.1f;
	private static final float JOYSTICK_DRIFT_THRESHOLD = 0.05f;
	// arbitrary encoder amounts
	private static final float LOW_ENCODER_ROTATIONS = 5;
	private static final float MID_ENCODER_ROTATIONS = -50;
	private static final float HIGH_ENCODER_ROTATIONS = -100;
	private static final float JOYSTICK_CONSTANT = 10;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax armMotor;
	private SparkMaxPIDController pidControllerArm;
	private SparkMaxLimitSwitch limitSwitchLow;
	private double currentEncoder;
	private boolean zeroed = true;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ElevatorArmFSM() {
		// Perform hardware init
		armMotor = new CANSparkMax(HardwareMap.CAN_ID_ARM,
										CANSparkMax.MotorType.kBrushless);
		armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		limitSwitchLow = armMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
		limitSwitchLow.enableLimitSwitch(true);
		armMotor.setInverted(true);
		pidControllerArm = armMotor.getPIDController();
		pidControllerArm.setP(PID_CONSTANT_ARM_P);
		pidControllerArm.setI(PID_CONSTANT_ARM_I);
		pidControllerArm.setD(PID_CONSTANT_ARM_D);
		pidControllerArm.setOutputRange(MAX_DOWN_POWER, MAX_UP_POWER);

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}

	/**
	 * Return current arm encoder count.
	 * @return Current arm encoder count
	 */
	public double getEncoderCount() {
		return armMotor.getEncoder().getPosition();
	}
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = FSMState.IDLE;
		armMotor.getEncoder().setPosition(0);
		currentEncoder = 0;
		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}
	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		if (input == null) {
			return;
		}
		if (currentState != FSMState.IDLE) {
			currentEncoder = armMotor.getEncoder().getPosition();
		}
		// if (limitSwitchLow.isPressed()) {
		// 	armMotor.getEncoder().setPosition(0);
		// 	currentEncoder = 0;
		// }
		switch (currentState) {
			case IDLE:
				handleIdleState(input);
				break;
			case HIGH:
				handleHighState(input);
				break;
			case MIDDLE:
				handleMiddleState(input);
				break;
			case LOW:
				handleLowState(input);
				break;
			case MOVING:
				handleMovingState(input);
				break;
			case ZEROING:
				handleZeroingState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		SmartDashboard.putString("Current State", currentState.toString());
		SmartDashboard.putNumber("Elevator Encoder", armMotor.getEncoder().getPosition());
		SmartDashboard.putNumber("Elevator Power", armMotor.getAppliedOutput());
		currentState = nextState(input);
	}

	/**
	 * Autonomous update.
	 * @param autonState current autonomous state
	 * @return status update
	 */
	public boolean autonomousUpdate(FSMState autonState) {
		if (limitSwitchLow.isPressed()) {
			armMotor.getEncoder().setPosition(0);
		}

		switch (autonState) {
			case IDLE:
				return handleAutonIdleState();
			case HIGH:
				return handleAutonHighState();
			case MIDDLE:
				return handleAutonMiddleState();
			case LOW:
				return handleAutonLowState();
			default:
				return true;
		}
	}

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
		if (input == null) {
			return FSMState.IDLE;
		}
		switch (currentState) {
			case IDLE:
				if (input.isHighButtonPressed()) {
					return FSMState.HIGH;
				} else if (input.isMidButtonPressed()) {
					return FSMState.MIDDLE;
				} else if (input.isLowButtonPressed()) {
					return FSMState.LOW;
				} else if (Math.abs(input.getLeftJoystickY()) > JOYSTICK_DRIFT_THRESHOLD) {
					return FSMState.MOVING;
				} else if (input.isArmZeroButtonPressed()) {
					return FSMState.ZEROING;
				} else {
					return FSMState.IDLE;
				}
			case HIGH:
				if (input.isHighButtonPressed()) {
					return FSMState.HIGH;
				} else {
					return FSMState.IDLE;
				}
			case MIDDLE:
				if (input.isMidButtonPressed()) {
					return FSMState.MIDDLE;
				} else {
					return FSMState.IDLE;
				}
			case LOW:
				if (input.isLowButtonPressed()) {
					return FSMState.LOW;
				} else {
					return FSMState.IDLE;
				}
			case MOVING:
				if (Math.abs(input.getLeftJoystickY()) > JOYSTICK_DRIFT_THRESHOLD) {
					return FSMState.MOVING;
				} else {
					return FSMState.IDLE;
				}
			case ZEROING:
				if (!input.isArmZeroButtonPressed()) {
					return FSMState.IDLE;
				} else {
					return FSMState.ZEROING;
				}
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		//pidControllerArm.setReference(currentEncoder, CANSparkMax.ControlType.kPosition);
		pidControllerArm.setReference(0, CANSparkMax.ControlType.kDutyCycle);
	}
	private void handleHighState(TeleopInput input) {
		pidControllerArm.setReference(HIGH_ENCODER_ROTATIONS, CANSparkMax.ControlType.kPosition);
	}
	private void handleMiddleState(TeleopInput input) {
		pidControllerArm.setReference(MID_ENCODER_ROTATIONS, CANSparkMax.ControlType.kPosition);
	}
	private void handleLowState(TeleopInput input) {
		pidControllerArm.setReference(LOW_ENCODER_ROTATIONS, CANSparkMax.ControlType.kPosition);
	}
	private void handleMovingState(TeleopInput input) {
		pidControllerArm.setReference(-input.getLeftJoystickY() / JOYSTICK_CONSTANT,
			CANSparkMax.ControlType.kDutyCycle);
	}
	private void handleZeroingState(TeleopInput input) {
		pidControllerArm.setReference(ZEROING_POWER, CANSparkMax.ControlType.kDutyCycle);
	}


	private boolean handleAutonIdleState() {
		pidControllerArm.setReference(currentEncoder, CANSparkMax.ControlType.kPosition);
		return true;
	}
/** This method is for depositing high in game.
* @return completion of the deposit
 	*/
	public boolean handleAutonHighState() {
		pidControllerArm.setReference(HIGH_ENCODER_ROTATIONS, CANSparkMax.ControlType.kPosition);
		return inRange(armMotor.getEncoder().getPosition(), HIGH_ENCODER_ROTATIONS);
	}
/** This method is for depositing high in game.
* @return completion of the deposit
 	*/
	public boolean handleAutonMiddleState() {
		pidControllerArm.setReference(MID_ENCODER_ROTATIONS, CANSparkMax.ControlType.kPosition);
		return inRange(armMotor.getEncoder().getPosition(), MID_ENCODER_ROTATIONS);
	}
/** This method is for depositing low in game.
* @return completion of the deposit
 	*/
	public boolean handleAutonLowState() {
		pidControllerArm.setReference(LOW_ENCODER_ROTATIONS, CANSparkMax.ControlType.kPosition);
		return inRange(armMotor.getEncoder().getPosition(), LOW_ENCODER_ROTATIONS);
	}


	private boolean inRange(double a, double b) {
		return Math.abs(a - b) <= 1.0 / 2;
	}
}

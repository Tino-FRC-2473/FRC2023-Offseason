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

	private static final float ZEROING_POWER = 0.2f;
	private static final double PID_CONSTANT_ARM_P = 0.01;
	private static final double PID_CONSTANT_ARM_I = 0.00000001;
	private static final double PID_CONSTANT_ARM_D = 0.00000001;
	private static final float MAX_UP_POWER = 0.55f;
	private static final float MAX_DOWN_POWER = -0.5f;
	private static final float JOYSTICK_DRIFT_THRESHOLD = 0.15f;
	// arbitrary encoder amounts
	private static final float LOW_ENCODER_ROTATIONS = -148;
	private static final float MID_ENCODER_ROTATIONS = 15;
	private static final float HIGH_ENCODER_ROTATIONS = 110;
	private static final float JOYSTICK_CONSTANT = 3;
	private static final float STARTING_ER = -135;
	private static final float AUTO_ENCODER = 30;
	private static final double AUTO_ENCODER_MARGIN = 3.0;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax armMotor;
	private SparkMaxPIDController pidControllerArm;
	private SparkMaxLimitSwitch limitSwitchLow;
	private double currentEncoder;
	private boolean zeroed = false;
	private boolean lastPressed = false;


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
		limitSwitchLow.enableLimitSwitch(false);
		armMotor.setInverted(true);
		pidControllerArm = armMotor.getPIDController();
		pidControllerArm.setP(PID_CONSTANT_ARM_P);
		pidControllerArm.setI(PID_CONSTANT_ARM_I);
		pidControllerArm.setD(PID_CONSTANT_ARM_D);
		pidControllerArm.setOutputRange(MAX_DOWN_POWER, MAX_UP_POWER);
		armMotor.getEncoder().setPosition(STARTING_ER);
		currentEncoder = STARTING_ER;
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
		SmartDashboard.putString("Elevator Current State", currentState.toString());
		SmartDashboard.putNumber("Elevator Encoder", armMotor.getEncoder().getPosition());
		SmartDashboard.putNumber("Elevator Power", armMotor.getAppliedOutput());
		SmartDashboard.putBoolean("Is Limit Switch Pressed", limitSwitchLow.isPressed());
		SmartDashboard.putBoolean("Last Pressed", lastPressed);
		SmartDashboard.putBoolean("Is Zero button pressed", input.isArmZeroButtonPressed());
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
		lastPressed = limitSwitchLow.isPressed();

		currentState = nextState(input);
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
					System.out.println("Leaving zero state");
					return FSMState.IDLE;
				} else {
					System.out.println("Staying in zero state");
					return FSMState.ZEROING;
				}
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 *
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		pidControllerArm.setReference(0, CANSparkMax.ControlType.kDutyCycle);
	}

	private void handleHighState(TeleopInput input) {
		armMotor.set(pid(armMotor.getEncoder().getPosition(), HIGH_ENCODER_ROTATIONS));
	}

	private void handleMiddleState(TeleopInput input) {
		armMotor.set(pid(armMotor.getEncoder().getPosition(), MID_ENCODER_ROTATIONS));
	}

	private void handleLowState(TeleopInput input) {
		armMotor.set(pid(armMotor.getEncoder().getPosition(), LOW_ENCODER_ROTATIONS));
	}

	private void handleMovingState(TeleopInput input) {
		if ((armMotor.getEncoder().getPosition() >= LOW_ENCODER_ROTATIONS
				&& armMotor.getEncoder().getPosition() <= HIGH_ENCODER_ROTATIONS)) {
			pidControllerArm.setReference(-input.getLeftJoystickY() / JOYSTICK_CONSTANT,
						CANSparkMax.ControlType.kDutyCycle);

		} else if (armMotor.getEncoder().getPosition() < LOW_ENCODER_ROTATIONS) {

			if (-input.getLeftJoystickY() > 0) {
				pidControllerArm.setReference(-input.getLeftJoystickY() / JOYSTICK_CONSTANT,
						CANSparkMax.ControlType.kDutyCycle);
			} else {
				pidControllerArm.setReference(0, CANSparkMax.ControlType.kDutyCycle);
			}

		} else {
			if (-input.getLeftJoystickY() < 0) {
				pidControllerArm.setReference(-input.getLeftJoystickY() / JOYSTICK_CONSTANT,
						CANSparkMax.ControlType.kDutyCycle);
			} else {
				pidControllerArm.setReference(0, CANSparkMax.ControlType.kDutyCycle);
			}
		}
	}

	private void handleZeroingState(TeleopInput input) {


		if (zeroed) {
			armMotor.set(pid(armMotor.getEncoder().getPosition(), STARTING_ER));
		} else {
			if (limitSwitchLow.isPressed() && lastPressed) {
				pidControllerArm.setReference(ZEROING_POWER, CANSparkMax.ControlType.kDutyCycle);
			} else if (!limitSwitchLow.isPressed() && !lastPressed) {
				pidControllerArm.setReference(-ZEROING_POWER, CANSparkMax.ControlType.kDutyCycle);
			} else {
				pidControllerArm.setReference(0, CANSparkMax.ControlType.kDutyCycle);
				armMotor.getEncoder().setPosition(0);
				currentEncoder = 0;
				zeroed = true;
			}
		}
	}

	/**
	 * This method is for depositing high in game.
	 *
	 * @return completion of the deposit
	 */
	public boolean handleAutonHighState() {
		armMotor.set(pid(armMotor.getEncoder().getPosition(), HIGH_ENCODER_ROTATIONS));
		return inRange(armMotor.getEncoder().getPosition(), HIGH_ENCODER_ROTATIONS);
	}

	/**
	 * This method is for depositing mid in game.
	 *
	 * @return completion of the deposit
	 */
	public boolean handleAutonMiddleState() {
		armMotor.set(pid(armMotor.getEncoder().getPosition(), MID_ENCODER_ROTATIONS));
		return inRange(armMotor.getEncoder().getPosition(), MID_ENCODER_ROTATIONS);
	}
	/**
	 * This method is for depositing low in game.
	 *
	 * @return completion of the deposit
	 */
	public boolean handleAutonLowState() {
		armMotor.set(pid(armMotor.getEncoder().getPosition(), LOW_ENCODER_ROTATIONS));
		return inRange(armMotor.getEncoder().getPosition(), LOW_ENCODER_ROTATIONS);
	}
	/**
	 * This method is for moving the elevator to the autonomous extension.
	 *
	 * @return completion of the extension
	 */
	public boolean handleAutonExtendState() {
		armMotor.set(pid(armMotor.getEncoder().getPosition(), AUTO_ENCODER));
		return inRange(armMotor.getEncoder().getPosition(), AUTO_ENCODER);
	}
	/**
	 * This method is for moving the elevator to the starting config in auto.
	 *
	 * @return completion of the retraction
	 */
	public boolean handleAutonRetractState() {
		armMotor.set(pid(armMotor.getEncoder().getPosition(), STARTING_ER));
		return inRange(armMotor.getEncoder().getPosition(), STARTING_ER);
	}
	private boolean inRange(double a, double b) {
		return Math.abs(a - b) <= AUTO_ENCODER_MARGIN;
	}

	private double pid(double currentEncoderPID, double targetEncoder) {
		double correction = PID_CONSTANT_ARM_P * (targetEncoder - currentEncoderPID);
		return Math.min(MAX_UP_POWER, Math.max(MAX_DOWN_POWER, correction));
	}
}

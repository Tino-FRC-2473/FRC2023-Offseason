package frc.robot.systems;
// WPILib Imports
import edu.wpi.first.wpilibj.Timer;



// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class EveryBotIntakeFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum EveryBotIntakeFSMState {
		INTAKING,
		IDLE_FLIPCLOCKWISE,
		IDLE_FLIPCOUNTERCLOCKWISE,
		IDLE_STOP,
		OUTTAKING,
		HOLDING
	}
	// Distance definitions
	public enum ItemType {
		CUBE,
		CONE,
		EMPTY
	}
	// hello
	//HAVE TO CHANGE BASED ON TEST
	private static final double KEEP_SPEED = 0;
	private static final double HOLDING_SPEED = 0.05;
	private static final double INTAKE_SPEED = 0.3; //0.3
	private static final double RELEASE_SPEED = -0.2; //DONT FORGET THE NEGATIVE SIGN (-)
	private static final double RELEASE_SPEED_LOW = -0.3;
	private static final double CURRENT_THRESHOLD = 15;
	private static final double BASE_THRESHOLD = 100;
	private static final double TIME_RESET_CURRENT = 0.5;
	private static final int MIN_RELEASE_DISTANCE = 800;
	private static final int AVERAGE_SIZE = 7;
	private static final double MIN_TURN_SPEED = -0.3;
	private static final double MAX_TURN_SPEED = 0.3;
	private static final double FLIP_SPEED = 0.2;
	private static final double OVERRUN_THRESHOLD = 0.01;
	private static final double FLIP_CW_THRESHOLD = 14.0; //16
	private static final double FLIP_CCW_THRESHOLD = 0.0;
	private static final double PID_CONSTANT_ARM_P = 0.075; //0.008
	private static final double PID_CONSTANT_ARM_I = 0.0000000;
	private static final double PID_CONSTANT_ARM_D = 0.0000000;
	//variable for armFSM, 0 means no object, 1 means cone, 2 means cube
	private static ItemType itemType = ItemType.EMPTY;
	private boolean isMotorAllowed = false;
	private boolean toggleUpdate = true;
	private boolean needsReset = true;
	private int tick = 0;
	private boolean hasTimerStarted = false;
	private boolean holding = false;
	private boolean forward = true;
	private boolean prevOuttaking = false;
	private double[] currLogs = new double[AVERAGE_SIZE];
	//diy pid
	private double lastError = 0;
	private double errorSum = 0;


	/* ======================== Private variables ======================== */
	private EveryBotIntakeFSMState currentState;
	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax spinnerMotor;
	private CANSparkMax flipMotor;
	private SparkMaxPIDController pidControllerFlip;
	private Timer timer;
	//private ElevatorArmFSM arm;
	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public EveryBotIntakeFSM() {
		//arm = new ElevatorArmFSM();
		timer = new Timer();
		spinnerMotor = new CANSparkMax(HardwareMap.CAN_ID_SPINNER_MOTOR,
				CANSparkMax.MotorType.kBrushless);
		flipMotor = new CANSparkMax(HardwareMap.CAN_ID_FLIP_MOTOR,
				CANSparkMax.MotorType.kBrushless);

		pidControllerFlip = flipMotor.getPIDController();
		pidControllerFlip.setP(PID_CONSTANT_ARM_P);
		pidControllerFlip.setI(PID_CONSTANT_ARM_I);
		pidControllerFlip.setD(PID_CONSTANT_ARM_D);
		pidControllerFlip.setOutputRange(MIN_TURN_SPEED, MAX_TURN_SPEED);
		//pidControllerFlip.setOutputRange(0, 0);
		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public EveryBotIntakeFSMState getCurrentState() {
		return currentState;
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
		currentState = EveryBotIntakeFSMState.IDLE_STOP;
		hasTimerStarted = false;
		needsReset = true;
		flipMotor.getEncoder().setPosition(0);
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
		//Robot.getStringLog().append("spinning intake start in " + currentState.toString());
		double begin = Timer.getFPGATimestamp();
		/*if (input.isDisableUpdatedPressed()) {
			toggleUpdate = !toggleUpdate;
			SmartDashboard.putBoolean("Is update enabled", toggleUpdate);
		}*/
		if (input.isThrottleForward()) {
			itemType = ItemType.CUBE;
		} else {
			itemType = ItemType.CONE;
		}
		if (toggleUpdate) {
			SmartDashboard.putNumber("output current", spinnerMotor.getOutputCurrent());
			SmartDashboard.putNumber("Flip Motor output", flipMotor.getAppliedOutput());
			SmartDashboard.putString("spinning intake state", currentState.toString());
			SmartDashboard.putNumber("flip encoder", flipMotor.getEncoder().getPosition());
			SmartDashboard.putString("item type", itemType.toString());
			SmartDashboard.putNumber("spinner power", spinnerMotor.get());
			SmartDashboard.putBoolean("holding", holding);
			SmartDashboard.putBoolean("Outtaking to intaking", prevOuttaking);
			SmartDashboard.putBoolean("forward", forward);
			SmartDashboard.putString("Current Intake State", currentState.toString());
			//SmartDashboard.putBoolean("Flip Button", input.isFlipButtonPressed());

			switch (currentState) {
				case INTAKING:
					handleIntakingState(input);
					break;
				case IDLE_FLIPCLOCKWISE:
					handleFlipClockWiseState();
					break;
				case IDLE_FLIPCOUNTERCLOCKWISE:
					handleFlipCounterClockWiseState();
					break;
				case IDLE_STOP:
					handleIdleStopState(input);
					break;
				case OUTTAKING   :
					handleOuttakingState(input);
					break;
				default:
					throw new IllegalStateException("Invalid state: " + currentState.toString());
			}
			EveryBotIntakeFSMState previousState = currentState;

			double currentTime = Timer.getFPGATimestamp();
			double timeTaken = currentTime - begin;
			if (timeTaken > OVERRUN_THRESHOLD) {
				System.out.println("ALERT ALERT SPINNING INTAKE HANDLER " + timeTaken);
				System.out.println("intake state" + currentState);
			}
			begin = Timer.getFPGATimestamp();

			currentState = nextState(input);

			currentTime = Timer.getFPGATimestamp();
			timeTaken = currentTime - begin;
			if (timeTaken > OVERRUN_THRESHOLD) {
				System.out.println("ALERT ALERT SPINNING INTAKE nextState " + timeTaken);
				System.out.println("intake state" + currentState);
			}
		} else {
			SmartDashboard.putBoolean("disabled", true);
		}
		double currentTime = Timer.getFPGATimestamp();
		double timeTaken = currentTime - begin;
		//Robot.getStringLog().append("spinning intake ending");
		//Robot.getStringLog().append("Time taken for loop: " + timeTaken);
	}
	/** This method is for intake in game and flipping.
	 * @param currState
* @return completion of the updateautonomous
 	*/
	public boolean updateAutonomous(EveryBotIntakeFSMState currState) {
		switch (currState) {
			case INTAKING:
				handleIntakingState(null);
				double avgcone = 0;
				for (int i = 0; i < AVERAGE_SIZE; i++) {
					avgcone += currLogs[i];
				}
				avgcone /= AVERAGE_SIZE;
				return  avgcone > CURRENT_THRESHOLD;
			case IDLE_FLIPCLOCKWISE:
				handleFlipClockWiseState();
				return flipMotor.getEncoder().getPosition() >= FLIP_CW_THRESHOLD;
			case IDLE_FLIPCOUNTERCLOCKWISE:
				handleFlipCounterClockWiseState();
				return flipMotor.getEncoder().getPosition() <= FLIP_CCW_THRESHOLD;
			case IDLE_STOP:
				handleIdleStopState(null);
				return true;
			case OUTTAKING   :
				handleOuttakingState(null);
				return true;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/*-------------------------NON HANDLER METHODS ------------------------- */
	/**
	 * Returns the type of object currently in the grabber.
	 * @return int 0 1 or 2 for nothing, cone, cube
	 */
	public static ItemType getObjectType() {
		return itemType;
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
	private EveryBotIntakeFSMState nextState(TeleopInput input) {
		if (input == null) {
			return EveryBotIntakeFSMState.IDLE_STOP;
		}
		switch (currentState) {
			case INTAKING:
				if (input.isThrottleForward() != forward || prevOuttaking || needsReset) {
					timer.reset();
					timer.start();
					needsReset = false;
					prevOuttaking = false;
				}
				if (timer.hasElapsed(TIME_RESET_CURRENT)) {
					currLogs[tick % AVERAGE_SIZE] = spinnerMotor.getOutputCurrent();
					tick++;

					double avgcone = 0;
					for (int i = 0; i < AVERAGE_SIZE; i++) {
						avgcone += currLogs[i];
					}
					avgcone /= AVERAGE_SIZE;
					if (avgcone > CURRENT_THRESHOLD) {
						holding = true;
						return EveryBotIntakeFSMState.IDLE_STOP;
					} else {
						holding = false;
					}
				}
				forward = input.isThrottleForward();
				if (!input.isIntakeButtonPressed()) {
					return EveryBotIntakeFSMState.IDLE_STOP;
				}
				return EveryBotIntakeFSMState.INTAKING;
			case IDLE_STOP:
				if (input.isOuttakeButtonPressed()) {
					// && flipMotor.getEncoder().getPosition() > FLIP_THRESHOLD) {
					return EveryBotIntakeFSMState.OUTTAKING;
				} else if (input.isIntakeButtonPressed()) {
					if (holding) {
						return EveryBotIntakeFSMState.IDLE_STOP;
					} else {
						return EveryBotIntakeFSMState.INTAKING;
					}
				} else if (input.isFlipButtonPressed()) {
					//&& arm.getEncoderCount() > BASE_THRESHOLD) {
					if (flipMotor.getEncoder().getPosition() <= FLIP_CW_THRESHOLD) {
						return EveryBotIntakeFSMState.IDLE_FLIPCLOCKWISE;
					} else {
						return EveryBotIntakeFSMState.IDLE_FLIPCOUNTERCLOCKWISE;
					}
				} else {
					return EveryBotIntakeFSMState.IDLE_STOP;
				}
			case IDLE_FLIPCLOCKWISE:
				if (flipMotor.getEncoder().getPosition() > FLIP_CW_THRESHOLD
					|| input.isFlipAbortButtonPressed()) {
					return EveryBotIntakeFSMState.IDLE_STOP;
				} else if (input.isFlipButtonPressed()) {
					return EveryBotIntakeFSMState.IDLE_FLIPCOUNTERCLOCKWISE;
				} else {
					return EveryBotIntakeFSMState.IDLE_FLIPCLOCKWISE;
				}
			case IDLE_FLIPCOUNTERCLOCKWISE:
				if (flipMotor.getEncoder().getPosition() < 0
					|| input.isFlipAbortButtonPressed()) {
					return EveryBotIntakeFSMState.IDLE_STOP;
				} else if (input.isFlipButtonPressed()) {
					return EveryBotIntakeFSMState.IDLE_FLIPCLOCKWISE;
				} else {
					return EveryBotIntakeFSMState.IDLE_FLIPCOUNTERCLOCKWISE;
				}
			case OUTTAKING:
				if (input.isOuttakeButtonPressed()) {
					return EveryBotIntakeFSMState.OUTTAKING;
				} else {
					prevOuttaking = true;
					return EveryBotIntakeFSMState.IDLE_STOP;
				}
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in states.
	 * @param input
	 */
	public void handleIntakingState(TeleopInput input) {
		if (input != null && input.isThrottleForward()) {
			spinnerMotor.set(INTAKE_SPEED);
		} else {
			spinnerMotor.set(-INTAKE_SPEED);
		}
		flipMotor.set(0);
	}
	private void handleIdleStopState(TeleopInput input) {
		spinnerMotor.set(KEEP_SPEED);
		flipMotor.set(0);
		//flipMotor.set(pid(flipMotor.getEncoder().getPosition(), 0));

		if (holding) {
			spinnerMotor.set(HOLDING_SPEED * ((input.isThrottleForward()) ? 1 : -1));
		}
	}
	private void handleFlipClockWiseState() {
		flipMotor.set(pid(flipMotor.getEncoder().getPosition(), FLIP_CW_THRESHOLD));
		spinnerMotor.set(0);
	}
	private void handleFlipCounterClockWiseState() {
		//pidControllerFlip.setReference(1, CANSparkMax.ControlType.kPosition);
		flipMotor.set(pid(flipMotor.getEncoder().getPosition(), FLIP_CCW_THRESHOLD));
		spinnerMotor.set(0);
	}
	private void handleOuttakingState(TeleopInput input) {
		if (input == null) {
			if (!hasTimerStarted) {
				timer.reset();
				timer.start();
				hasTimerStarted = true;
			}
		}
		for (int i = 0; i < AVERAGE_SIZE; i++) {
			currLogs[i] = 0;
		}
		if (input.isThrottleForward()) {
			spinnerMotor.set(RELEASE_SPEED);
		} else {
			spinnerMotor.set(-RELEASE_SPEED);
		}
		itemType = ItemType.EMPTY;
		isMotorAllowed = true;
		holding = false;
	}

	public boolean handleAutoOuttakingState() {
		if (!hasTimerStarted) {
			timer.reset();
			timer.start();
			hasTimerStarted = true;
			holding = true;
		}
		for (int i = 0; i < AVERAGE_SIZE; i++) {
			currLogs[i] = 0;
		}
		spinnerMotor.set(RELEASE_SPEED);
		if (timer.get() >= 2) {
			itemType = ItemType.EMPTY;
			isMotorAllowed = true;
			holding = false;
			return true;
		}
		return false;

	}

	private double pid(double currentEncoder, double targetEncoder) {
		double error = targetEncoder - currentEncoder;
		//double errorChange = error - lastError;
		double correction = PID_CONSTANT_ARM_P * error;
						//+ PID_CONSTANT_ARM_I * errorSum + PID_CONSTANT_ARM_D * errorChange;
		lastError = error;
		//errorSum += error;



		return Math.min(MAX_TURN_SPEED, Math.max(MIN_TURN_SPEED, correction));
	}
}

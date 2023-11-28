package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.Timer;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class AnyBotIntakeFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum AnyBotIntakeFSMState {
		INTAKING,
		IDLE_STOP,
		OUTTAKING,
		HOLDING
	}

	// Distance definitions
	public enum ItemType {
		CUBE,
		EMPTY
	}

	// hello
	// HAVE TO CHANGE BASED ON TEST
	private static final double KEEP_SPEED = 0;
	private static final double HOLDING_SPEED = 0.05;
	private static final double INTAKE_SPEED = 0.1; // 0.3
	private static final double RELEASE_SPEED = -0.1; // DONT FORGET THE NEGATIVE SIGN (-)
	private static final double RELEASE_SPEED_LOW = -0.3;
	private static final double CURRENT_THRESHOLD = 15;
	private static final double BASE_THRESHOLD = 100;
	private static final double TIME_RESET_CURRENT = 0.5;
	private static final int MIN_RELEASE_DISTANCE = 800;
	private static final int AVERAGE_SIZE = 7;
	private static final double MIN_TURN_SPEED = -0.3;
	private static final double MAX_TURN_SPEED = 0.3;
	private static final double OVERRUN_THRESHOLD = 0.01;
	private static final double PID_CONSTANT_ARM_P = 0.075; // 0.075
	private static final double PID_CONSTANT_ARM_I = 0.0000000;
	private static final double PID_CONSTANT_ARM_D = 0.0000000;
	// variable for armFSM, 0 means no object, 1 means cone, 2 means cube
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
	private boolean stopping = false; 

	private double autoOuttakingTimeStart;
	private boolean autoOuttakingTimerStarted;

	/* ======================== Private variables ======================== */
	private AnyBotIntakeFSMState currentState;
	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax spinnerMotor;
	private Timer timer;
	private double currentEncoder = 0;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public AnyBotIntakeFSM() {
		// arm = new ElevatorArmFSM();
		timer = new Timer();
		spinnerMotor = new CANSparkMax(HardwareMap.CAN_ID_SPINNER_MOTOR,
				CANSparkMax.MotorType.kBrushless);

		currentEncoder = 0;
		reset();

		autoOuttakingTimerStarted = false;
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 *
	 * @return Current FSM state
	 */
	public AnyBotIntakeFSMState getCurrentState() {
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
		currentState = AnyBotIntakeFSMState.IDLE_STOP;
		hasTimerStarted = false;
		needsReset = true;
		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 *
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		if (input == null) {
			return;
		}
		// Robot.getStringLog().append("spinning intake start in " +
		// currentState.toString());
		double begin = Timer.getFPGATimestamp();
		/*
		 * if (input.isDisableUpdatedPressed()) {
		 * toggleUpdate = !toggleUpdate;
		 * SmartDashboard.putBoolean("Is update enabled", toggleUpdate);
		 * }
		 */
		if (input.isThrottleForward()) {
			itemType = ItemType.CUBE;
		}
		if (toggleUpdate) {
			SmartDashboard.putNumber("output current", spinnerMotor.getOutputCurrent());
			SmartDashboard.putString("spinning intake state", currentState.toString());
			SmartDashboard.putString("item type", itemType.toString());
			SmartDashboard.putNumber("spinner power", spinnerMotor.get());
			SmartDashboard.putBoolean("holding", holding);
			SmartDashboard.putBoolean("Outtaking to intaking", prevOuttaking);
			SmartDashboard.putBoolean("forward", forward);
			SmartDashboard.putString("Current Intake State", currentState.toString());

			switch (currentState) {
				case INTAKING:
					handleIntakingState(input);
					break;
				case IDLE_STOP:
					handleIdleStopState(input);
					break;
				case OUTTAKING:
					handleOuttakingState(input);
					break;
				default:
					throw new IllegalStateException("Invalid state: " + currentState.toString());
			}
			AnyBotIntakeFSMState previousState = currentState;

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
		// Robot.getStringLog().append("spinning intake ending");
		// Robot.getStringLog().append("Time taken for loop: " + timeTaken);
	}

	/**
	 * This method is for intake in game and NO MORE FLIPPING!!!!!!!1!!.
	 *
	 * @param currState
	 * @return completion of the updateautonomous
	 */
	public boolean updateAutonomous(AnyBotIntakeFSMState currState) {
		switch (currState) {
			case INTAKING:
				handleIntakingState(null);
				double avgcone = 0;
				for (int i = 0; i < AVERAGE_SIZE; i++) {
					avgcone += currLogs[i];
				}
				avgcone /= AVERAGE_SIZE;
				return avgcone > CURRENT_THRESHOLD;
			case IDLE_STOP:
				handleIdleStopState(null);
				return true;
			case OUTTAKING:
				handleOuttakingState(null);
				return true;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/*-------------------------NON HANDLER METHODS ------------------------- */
	/**
	 * Returns the type of object currently in the grabber.
	 *
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
	 *
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private AnyBotIntakeFSMState nextState(TeleopInput input) {
		if (input == null) {
			return AnyBotIntakeFSMState.IDLE_STOP;
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
						return AnyBotIntakeFSMState.IDLE_STOP;
					} else {
						holding = false;
					}
				}
				forward = input.isThrottleForward();
				if (input.isIntakeButtonPressed() && !input.isOuttakeButtonPressed()) {
					return AnyBotIntakeFSMState.INTAKING;
				} else {
					return AnyBotIntakeFSMState.IDLE_STOP;
				}
			case IDLE_STOP:

				if (input.isOuttakeButtonPressed() && input.isIntakeButtonPressed()) {
					stopping = true; 
					return AnyBotIntakeFSMState.IDLE_STOP;
				}
				else if(!input.isOuttakeButtonPressed() && !input.isIntakeButtonPressed()){
                        stopping = false; 
					    return AnyBotIntakeFSMState.IDLE_STOP;
				}

				if (input.isOuttakeButtonPressed() && !input.isIntakeButtonPressed() && stopping == false) {
					return AnyBotIntakeFSMState.OUTTAKING;
				} else if (input.isIntakeButtonPressed() && stopping == false) {
					if (holding) {
						return AnyBotIntakeFSMState.IDLE_STOP;
					} else {
						return AnyBotIntakeFSMState.INTAKING;
					}
				} else {
					return AnyBotIntakeFSMState.IDLE_STOP;
				}
			case OUTTAKING:
				if (input.isOuttakeButtonPressed() && !input.isIntakeButtonPressed()) {
					return AnyBotIntakeFSMState.OUTTAKING;
				} else {
					prevOuttaking = true;
					return AnyBotIntakeFSMState.IDLE_STOP;
				}
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in states.
	 *
	 * @param input
	 */
	public void handleIntakingState(TeleopInput input) {
		if (input != null && input.isThrottleForward()) {
			spinnerMotor.set(INTAKE_SPEED);
		} else {
			spinnerMotor.set(-INTAKE_SPEED);
		}

	}

	private void handleIdleStopState(TeleopInput input) {
		spinnerMotor.set(KEEP_SPEED);
		if (holding) {
			spinnerMotor.set(HOLDING_SPEED * ((input.isThrottleForward()) ? 1 : -1));
		}
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

	/**
	 * Runs the intake behavior for auto deposit.
	 *
	 * @return whether the intake has finished running for auto
	 */
	public boolean handleAutoOuttakingState() {
		if (!autoOuttakingTimerStarted) {
			autoOuttakingTimerStarted = true;
			autoOuttakingTimeStart = timer.get();
		}

		if (autoOuttakingTimerStarted && !timer.hasElapsed(autoOuttakingTimeStart + 2.0)) {
			spinnerMotor.set(RELEASE_SPEED);
		} else {
			spinnerMotor.set(0);
			return true;
		}
		return false;
	}

	private double pid(double currentEncoderPID, double targetEncoder) {
		double correction = PID_CONSTANT_ARM_P * (targetEncoder - currentEncoder);
		return Math.min(MAX_TURN_SPEED, Math.max(MIN_TURN_SPEED, correction));
	}

	private boolean inRange(double a, double b) {
		return Math.abs(a - b) <= 1.0;
	}
}

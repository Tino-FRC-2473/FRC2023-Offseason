package frc.robot.systems;


public class AutoFSMSystem {
	/* ======================== Constants ======================== */
	// Auto FSM state definitions
	public enum AutoFSMState {
		CUBE_LOW,
		CUBE_MID,
		CUBE_HIGH,
		CONE_LOW,
		CONE_MID,
		CONE_HIGH,
		BALANCE,
		MOVE_BACK_TWO_METERS
	}


	/* ======================== Private variables ======================== */
	//Contains the sequential list of states in the current auto path that must be executed
	private AutoFSMState[] currentStateList;
	//Predefined auto path consisting of doing mid cube, then moving back two meters, then balancing
	private static final AutoFSMState[] PATH1 = new AutoFSMState[]{
		AutoFSMState.CUBE_MID, AutoFSMState.MOVE_BACK_TWO_METERS, AutoFSMState.BALANCE};
	//The index in the currentStateList where the currentState is at
	private int currentStateIndex;
	//Stores if the current state has finished executing or not
	private boolean isCurrentStateFinished;
	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state.
	 * Initializes any subsystems such as driveFSM, armFSM, ect.
	 */
	public AutoFSMSystem() {

	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public AutoFSMState getCurrentState() {
		return currentStateList[currentStateIndex];
	}
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 * @param pathNumber the auto path that we want to execute
	 */
	public void reset(int pathNumber) {
		currentStateIndex = 0;
		isCurrentStateFinished = false;
		if (pathNumber == 1) {
			currentStateList = PATH1;
		}
		// Call one tick of update to ensure outputs reflect start state
		//update();
		//^ is this practical to when in auto
	}
	/**
	 * This function only calls the FSM state specific handlers.
	 */
	public void update() {
		switch (getCurrentState()) {
			case CUBE_MID:
				isCurrentStateFinished = handleCubeMid();
				break;

			case MOVE_BACK_TWO_METERS:
				isCurrentStateFinished = handleMoveBackTwoMeters();
				break;

			case BALANCE:
				isCurrentStateFinished = handleBalance();
				break;

			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
		nextState();
	}

	/* ======================== Private methods ======================== */
	/**
	 * Checks if the current auto state is finished and if so, transitions to the
	 * next state to be executed.
	 */
	private void nextState() {
		if (isCurrentStateFinished) {
			currentStateIndex++;
			isCurrentStateFinished = false;
		}
	}

	/**
	 * Moves the arm to the mid encoder and moves the wrist out, then deposits the cube.
	 * @return returns true if the cube is successfully deposited to mid, and false if not
	 */
	private boolean handleCubeMid() {
		return false;
	}

	/**
	 * Moves the chassis two meters back from its current position.
	 * @return returns true if the chassis is at its desired position, and false if not
	 */
	private boolean handleMoveBackTwoMeters() {
		return false;
	}
	/**
	 * Uses gyro to balance the chassis on the charging station.
	 * @return returns true if the chassis is balanced, and false if not
	 */
	private boolean handleBalance() {
		return false;
	}
}

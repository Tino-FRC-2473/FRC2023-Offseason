package frc.robot.systems;
// WPILib Imports
import edu.wpi.first.wpilibj.Timer;



// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
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
        OUTTAKING
		
	}
	// Distance definitions
	public enum ItemType {
		CUBE,
		CONE,
		EMPTY
	}
	//HAVE TO CHANGE BASED ON TEST           
	private static final double KEEP_SPEED = 0.07;
	private static final double INTAKE_SPEED = 0.5;
	private static final double RELEASE_SPEED = -1; //DONT FORGET -
	private static final double RELEASE_SPEED_LOW = -0.3;
	private static final double CURRENT_THRESHOLD = 20;
	private static final double BASE_THRESHOLD = 100;             
	private static final double TIME_RESET_CURRENT = 0.5;
	private static final int MIN_RELEASE_DISTANCE = 800;
	private static final int AVERAGE_SIZE = 10;
	private static final double TURN_SPEED = -0.5;
	private static final double OVERRUN_THRESHOLD = 0.007;
	private static final double FLIP_THRESHOLD = 50;
	//variable for armFSM, 0 means no object, 1 means cone, 2 means cube
	private static ItemType itemType = ItemType.EMPTY;
	private boolean isMotorAllowed = false;
	private boolean toggleUpdate = true;
	private boolean needsReset = true;
	private boolean isIntakeUP = false;
	private int tick = 0;
	private boolean hasTimerStarted = false;
	private double[] currLogsCone = new double[AVERAGE_SIZE];
	private double[] currLogsCube = new double[AVERAGE_SIZE];


	/* ======================== Private variables ======================== */
	private EveryBotIntakeFSMState currentState;
	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private CANSparkMax spinnerMotorCone;
    private CANSparkMax spinnerMotorCube;   
	private CANSparkMax flipMotor;
	private Timer timer;
	private ElevatorArmFSM arm;
	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public EveryBotIntakeFSM() {
		arm = new ElevatorArmFSM();
		timer = new Timer();
		spinnerMotorCone = new CANSparkMax(HardwareMap.CAN_ID_SPINNER_MOTOR,
				CANSparkMax.MotorType.kBrushless);
		spinnerMotorCube = new CANSparkMax(HardwareMap.CAN_ID_SPINNER_MOTOR,
				CANSparkMax.MotorType.kBrushless);
		flipMotor = new CANSparkMax(HardwareMap.CAN_ID_SPINNER_MOTOR,
				CANSparkMax.MotorType.kBrushless);
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
			SmartDashboard.putNumber("output current", spinnerMotorCone.getOutputCurrent());
			SmartDashboard.putNumber("output current", spinnerMotorCube.getOutputCurrent());
			SmartDashboard.putString("spinning intake state", currentState.toString());
			SmartDashboard.putNumber("velocity", spinnerMotorCone.getEncoder().getVelocity());
			SmartDashboard.putNumber("velocity", spinnerMotorCube .getEncoder().getVelocity());
			SmartDashboard.putString("item type", itemType.toString());
			SmartDashboard.putNumber("spinner power", spinnerMotorCone.get());
			SmartDashboard.putNumber("spinner power", spinnerMotorCube.get());
			
			
			switch (currentState) {
				case INTAKING:
					handleIntakingState();
					break;
				case IDLE_FLIPCLOCKWISE:
					handleFlipClockWiseState();
					break;
				case IDLE_FLIPCOUNTERCLOCKWISE:
					handleFlipCounterClockWiseState();
					break;
				case IDLE_STOP:
					handleIdleStopState();
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
	
	public boolean getIntake(){
		return isIntakeUP;
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
				if (needsReset && isMotorAllowed && toggleUpdate) {
					timer.reset();
					timer.start();
					needsReset = false;
				}
				if (timer.hasElapsed(TIME_RESET_CURRENT)) {
					currLogsCone[tick % AVERAGE_SIZE] = spinnerMotorCone.getOutputCurrent();
					currLogsCube[tick% AVERAGE_SIZE] = spinnerMotorCube.getOutputCurrent();
					tick++;

					double avgcube = 0;
					double avgcone = 0;
					for (int i = 0; i < AVERAGE_SIZE; i++) {
						avgcube += currLogsCube[i];
						avgcone += currLogsCone[i];
					}
					avgcone /= AVERAGE_SIZE;
					avgcube /= AVERAGE_SIZE;
     
					if (avgcone > CURRENT_THRESHOLD || !(input.isIntakeButtonPressed()) ) {
						return EveryBotIntakeFSMState.IDLE_STOP;
					}
				}
				return EveryBotIntakeFSMState.INTAKING;
			case IDLE_STOP: 
				if (input.isOuttakeButtonPressed() && flipMotor.getEncoder().getPosition()>FLIP_THRESHOLD) {
					return EveryBotIntakeFSMState.OUTTAKING;
				}
				else if(input.isIntakeButtonPressed() && !(flipMotor.getEncoder().getPosition()>FLIP_THRESHOLD)){
					return EveryBotIntakeFSMState.INTAKING;
				}
				else if(input.isFlipButtonPressed() && arm.getEncoderCount() > BASE_THRESHOLD){
					return EveryBotIntakeFSMState.IDLE_FLIPCLOCKWISE;
				}
			case IDLE_FLIPCLOCKWISE:
				if(flipMotor.getEncoder().getPosition()<FLIP_THRESHOLD){
					return EveryBotIntakeFSMState.IDLE_FLIPCLOCKWISE;
				}else if(input.isFlipButtonPressed()){
					return EveryBotIntakeFSMState.IDLE_FLIPCOUNTERCLOCKWISE;
				}else if(flipMotor.getEncoder().getPosition()>FLIP_THRESHOLD){
					return EveryBotIntakeFSMState.IDLE_STOP;
				}
			case IDLE_FLIPCOUNTERCLOCKWISE:
			    if(flipMotor.getEncoder().getPosition() < 0){
					return EveryBotIntakeFSMState.IDLE_STOP;
				}else if(input.isFlipButtonPressed()&&arm.getEncoderCount()>BASE_THRESHOLD){
					return EveryBotIntakeFSMState.IDLE_FLIPCLOCKWISE;
				}
				else{
					return EveryBotIntakeFSMState.IDLE_FLIPCOUNTERCLOCKWISE;
				}
			case OUTTAKING:
				if(input.isOuttakeButtonPressed()){
                   return EveryBotIntakeFSMState.OUTTAKING;
				}else{
					return EveryBotIntakeFSMState.IDLE_STOP;
				}
				
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in states.
	 */
	private void handleStartState() {
       
	}
	private void handleIntakingState() { 
		
			spinnerMotorCone.set(INTAKE_SPEED);
			spinnerMotorCube.set(INTAKE_SPEED);
		
	}
	private void handleIdleStopState() {
		spinnerMotorCone.set(KEEP_SPEED);
		spinnerMotorCube.set(KEEP_SPEED);
	}
	private void handleFlipClockWiseState(){
		flipMotor.set(TURN_SPEED);
		if(flipMotor.getEncoder().getPosition()>= FLIP_THRESHOLD){
           flipMotor.set(0);   

		}
	} 
	private void handleFlipCounterClockWiseState(){
		flipMotor.set(-TURN_SPEED);
		if(flipMotor.getEncoder().getPosition()<= 0){
           flipMotor.set(0);   

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
			currLogsCube[i] = 0;
			currLogsCone[i] = 0;
		}
		if (input != null) {
			if (itemType == ItemType.CUBE) {
				spinnerMotorCone.set(RELEASE_SPEED);
				spinnerMotorCube.set(RELEASE_SPEED);
			} else {
				spinnerMotorCone.set(RELEASE_SPEED);
			}
		} 
		itemType = ItemType.EMPTY;
		isMotorAllowed = true;
	}
}


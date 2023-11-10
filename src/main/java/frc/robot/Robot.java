// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Systems
import frc.robot.systems.AnyBotIntakeFSM;

/*import frc.robot.systems.ElevatorWristFSM;
import frc.robot.systems.ElevatorArmFSM;
import frc.robot.systems.EveryBotIntakeFSM;
import frc.robot.systems.DriveFSMSystem;*/

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;
	// Systems
	private AnyBotIntakeFSM anybotIntake;
	//private ElevatorWristFSM wristSystem;
	//private ElevatorArmFSM elevatorArm;
	//private EveryBotIntakeFSM everybotIntake;
	//private DriveFSMSystem driveFSMSystem;

	private boolean autoWristMoved;
	private boolean autoElevatorExtended;
	private boolean autoElevatorRetracted;
	private boolean autoIntakeMoved;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();
		// Instantiate all systems here
		//wristSystem = new ElevatorWristFSM();
		//everybotIntake = new EveryBotIntakeFSM();
		//elevatorArm = new ElevatorArmFSM();
		//driveFSMSystem = new DriveFSMSystem();
		anybotIntake = new AnyBotIntakeFSM();
	}
	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		//everybotIntake.reset();
		//wristSystem.reset();
		//elevatorArm.reset();
		//driveFSMSystem.resetAutonomus();

		autoWristMoved = false;
		autoElevatorExtended = false;
		autoElevatorRetracted = false;
		autoIntakeMoved = false;
	}

	@Override
	public void autonomousPeriodic() {
		/*if (wristSystem.movingAutoState()) {
			autoWristMoved = true;
		}
		if (autoWristMoved && elevatorArm.handleAutonExtendState()) {
			autoElevatorExtended = true;
		}

		if (autoElevatorExtended && everybotIntake.handleAutoOuttakingState()) {
			autoIntakeMoved = true;
		}
		if (autoIntakeMoved && elevatorArm.handleAutonRetractState()) {
			autoElevatorRetracted = true;
		}
		if (autoElevatorRetracted) {
			driveFSMSystem.update(null);
		}

		SmartDashboard.putBoolean("Wrist auto moved", autoWristMoved);
		SmartDashboard.putBoolean("Elevator auto extended", autoElevatorExtended);
		SmartDashboard.putBoolean("Intake auto moved", autoIntakeMoved);
		SmartDashboard.putBoolean("Elevator auto retracted", autoElevatorRetracted);
		*/
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		//everybotIntake.reset();
		//driveFSMSystem.reset();
		//elevatorArm.reset();
		//wristSystem.reset();

	}

	@Override
	public void teleopPeriodic() {
		/*everybotIntake.update(input);
		driveFSMSystem.update(input);
		elevatorArm.update(input);
		wristSystem.update(input);*/
		anybotIntake.update(input);
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	@Override
	public void disabledPeriodic() {

	}

	/* Simulation mode handlers, only used for simulation testing  */
	@Override
	public void simulationInit() {
		System.out.println("-------- Simulation Init --------");
	}

	@Override
	public void simulationPeriodic() { }

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() { }
}

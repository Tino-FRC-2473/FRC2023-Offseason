// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;

// Systems
import frc.robot.systems.ElevatorWristFSM;
import frc.robot.systems.ElevatorArmFSM;
import frc.robot.systems.EveryBotIntakeFSM;
import frc.robot.systems.EveryBotIntakeFSM.EveryBotIntakeFSMState;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;

	// Systems
	private ElevatorWristFSM wristSystem;
	private ElevatorArmFSM elevatorArm;
	private EveryBotIntakeFSM everybotIntake;
	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();
		// Instantiate all systems here
		// if (!HardwareMap.isElevatorArmDisabled()) {
		// 	fsmSystem = new ElevatorArmFSM();
		// }
		// if (!HardwareMap.isSpinningIntakeDisabled()) {
		// 	spinningIntake = new SpinningIntakeFSM();
		// }
		wristSystem = new ElevatorWristFSM();
		//fsmSystem = new ElevatorArmFSM();
		everybotIntake = new EveryBotIntakeFSM();
		elevatorArm = new ElevatorArmFSM();
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		// if (!HardwareMap.isElevatorArmDisabled()) {
		// 	fsmSystem.reset();
		// }
		// if (!HardwareMap.isSpinningIntakeDisabled()) {
		// 	spinningIntake.reset();
		// }
		//fsmSystem.reset();

		everybotIntake.reset();
		wristSystem.reset();
		elevatorArm.reset();



	}

	@Override
	public void autonomousPeriodic() {
		// if (!HardwareMap.isElevatorArmDisabled()) {
		// 	fsmSystem.update(null);
		// }
		// if (!HardwareMap.isSpinningIntakeDisabled()) {
		// 	spinningIntake.update(null);
		// }
		//fsmSystem.update(null);
		everybotIntake.update(null);
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		// if (!HardwareMap.isElevatorArmDisabled()) {
		// 	fsmSystem.reset();
		// }
		// if (!HardwareMap.isSpinningIntakeDisabled()) {
		// 	spinningIntake.reset();
		// }
		//fsmSystem.reset();
		everybotIntake.reset();
	}

	@Override
	public void teleopPeriodic() {
		// if (!HardwareMap.isElevatorArmDisabled()) {
		// 	fsmSystem.update(input);
		// }
		// if (!HardwareMap.isSpinningIntakeDisabled()) {
		// 	spinningIntake.update(input);
		// }
		//fsmSystem.update(input);
		everybotIntake.update(input);
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

	/** This method is for depositing high in game.
* @return completion of the deposit
 	*/
	public boolean depositHigh() {
		return elevatorArm.handleAutonHighState();
	}
	/** This method is for depositing mid in game.
* @return completion of the deposit
 	*/
	public boolean depositMid() {
		return elevatorArm.handleAutonMiddleState();
	}
	/** This method is for depositing low in game.
* @return completion of the deposit
 	*/
	public boolean depositLow() {
		return elevatorArm.handleAutonLowState();
	}
	/** This method is for intake in game and flipping.
* @return completion of the intake
 	*/
	public boolean intakeObject() {
		if (wristSystem.movingOutState()) {
			everybotIntake.updateAutonomous(EveryBotIntakeFSMState.INTAKING);
		}
		return true;
	}


}



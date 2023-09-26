// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;

// Camera Imports
//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cscore.CvSink;
//import edu.wpi.first.cscore.CvSource;
//import edu.wpi.first.cscore.UsbCamera;
//import org.opencv.core.Mat;

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

	private static final int WIDTH = 640;
	private static final int HEIGHT = 480;

	private Thread visionThread;

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
		//wristSystem = new ElevatorWristFSM();
		//everybotIntake = new EveryBotIntakeFSM();
		elevatorArm = new ElevatorArmFSM();

		/*visionThread = new Thread(
			() -> {
				UsbCamera camera = CameraServer.startAutomaticCapture();
				camera.setResolution(WIDTH, HEIGHT);

				CvSink cvSink = CameraServer.getVideo();
				CvSource outputStream = CameraServer.putVideo("Stream", WIDTH, HEIGHT);

				Mat mat = new Mat();

				while (!Thread.interrupted()) {
					// Tell the CvSink to grab a frame from the camera and put it
					// in the source mat.  If there is an error notify the output.
					if (cvSink.grabFrame(mat) == 0) {
						outputStream.notifyError(cvSink.getError());
						continue;
					}

					outputStream.putFrame(mat);
				}
			}
		);

		visionThread.setDaemon(true);
		visionThread.start();*/
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		//everybotIntake.reset();
		//wristSystem.reset();
		elevatorArm.reset();



	}

	@Override
	public void autonomousPeriodic() {
		//everybotIntake.update(null);
		elevatorArm.update(null);
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		//everybotIntake.reset();
		elevatorArm.reset();
	}

	@Override
	public void teleopPeriodic() {
		//everybotIntake.update(input);
		elevatorArm.update(input);
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
			if (everybotIntake.updateAutonomous(EveryBotIntakeFSMState.IDLE_FLIPCLOCKWISE)) {
				return everybotIntake.updateAutonomous(EveryBotIntakeFSMState.INTAKING);
			}
		}
		return false;
	}

	/** This method is for intake in game and flipping.
	* @return completion of the outtake
 	*/
	public boolean outttakeObject() {
		if (wristSystem.movingInState()) {
			if (everybotIntake.updateAutonomous(EveryBotIntakeFSMState.IDLE_FLIPCOUNTERCLOCKWISE)) {
				return everybotIntake.updateAutonomous(EveryBotIntakeFSMState.OUTTAKING);
			}
		}
		return false;
	}



}



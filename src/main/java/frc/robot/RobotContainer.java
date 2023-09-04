// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.SwerveConstants.AutoConstants;
import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.SwerveConstants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	private final DriveSubsystem robotDrive = new DriveSubsystem();

	// The driver's controller
	private XboxController driverController =
		new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);

	public static final double THREE_POINT_FIVE_SIX = 3.56;
	public static final int ONE_EIGHTY = 180;
	public static final double TWO_POINT_ZERO_THREE = 2.03;
	public static final double FOUR_POINT_SEVEN_ZERO = 4.70;
	public static final double ONE_POINT_FIVE_THREE = 1.53;
	public static final double POINT_SIX_FIVE = 0.65;
	public static final double ONE_POINT_SIX_SEVEN = 1.67;


	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		robotDrive.setDefaultCommand(
				// The left stick controls translation of the robot.
				// Turning is controlled by the X axis of the right stick.
				new RunCommand(
					() -> robotDrive.drive(
							-MathUtil.applyDeadband(
								driverController.getLeftY(), OIConstants.DRIVE_DEADBAND),
							-MathUtil.applyDeadband(
								driverController.getLeftX(), OIConstants.DRIVE_DEADBAND),
							-MathUtil.applyDeadband(
								driverController.getRightX(), OIConstants.DRIVE_DEADBAND),
							true,
							true),
						robotDrive));
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {
		new JoystickButton(driverController, Button.kR1.value)
				.whileTrue(new RunCommand(
					() -> robotDrive.setX(),
						robotDrive));
		new JoystickButton(driverController, Button.kL2.value)
			.whileTrue(new RunCommand(
				() -> robotDrive.getGyro().reset()));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
				AutoConstants.MAX_SPEED_METERS_PER_SECOND,
				AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
				// Add kinematics to ensure max speed is actually obeyed
				.setKinematics(DriveConstants.DRIVE_KINEMATICS);

		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, new Rotation2d(0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d((2 + 1), 0, new Rotation2d(0)),
			config);

		// trajectory 2: deposit + exit community
		Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction, the grid, and unload element
			new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
			// Move out of community
			List.of(new Translation2d(-THREE_POINT_FIVE_SIX, 0)),
			// End out of community
			new Pose2d(-THREE_POINT_FIVE_SIX, 0, Rotation2d.fromDegrees(ONE_EIGHTY)),
			config);

		// trajectory 3: deposit + charging station
		Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction, the grid, and unload element
			new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
			// Move to charging station
			List.of(new Translation2d(-TWO_POINT_ZERO_THREE, 0)),
			// End at charging station
			new Pose2d(-TWO_POINT_ZERO_THREE, 0, Rotation2d.fromDegrees(ONE_EIGHTY)),
			config);

		// trajectory 4: deposit + exit community + charging station
		Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction, the grid, and unload element
			new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
			// Exit community
			List.of(new Translation2d(-THREE_POINT_FIVE_SIX, 0),
			// Move to charging station
			new Translation2d(ONE_POINT_FIVE_THREE, 0)),
			// End at charging station
			new Pose2d(-TWO_POINT_ZERO_THREE, 0, Rotation2d.fromDegrees(ONE_EIGHTY)),
			config);

		// trajectory 5: deposit + exit community + pick up
		Trajectory trajectory5a = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction, the grid, and unload element
			new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
			// Exit community, go to object
			List.of(new Translation2d(-FOUR_POINT_SEVEN_ZERO, 0)),
			// End at object
			new Pose2d(-FOUR_POINT_SEVEN_ZERO, 0, Rotation2d.fromDegrees(ONE_EIGHTY)),
			config);

		// trajectory 6: deposit + exit community + pick up + charging station
		Trajectory trajectory6a = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction, the grid, and unload element
			new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
			// Exit community, go to object
			List.of(new Translation2d(-FOUR_POINT_SEVEN_ZERO, 0)),
			// End at object
			new Pose2d(-FOUR_POINT_SEVEN_ZERO, 0, Rotation2d.fromDegrees(ONE_EIGHTY)),
			config);
		// pick up object
		Trajectory trajectory6b = TrajectoryGenerator.generateTrajectory(
			new Pose2d(-FOUR_POINT_SEVEN_ZERO, 0, new Rotation2d(ONE_EIGHTY)),
			// Go to charging station
			List.of(new Translation2d(1, -1),
				new Translation2d(ONE_POINT_FIVE_THREE, -POINT_SIX_FIVE)),
			// End at charging station
			new Pose2d(-TWO_POINT_ZERO_THREE, -1.0 - POINT_SIX_FIVE,
				new Rotation2d(2 * ONE_EIGHTY)), config);

		var thetaController = new ProfiledPIDController(
				AutoConstants.P_THETA_CONTROLLER, 0, 0, AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
			exampleTrajectory,
			robotDrive::getPose, // Functional interface to feed supplier
			DriveConstants.DRIVE_KINEMATICS,

				// Position controllers
				new PIDController(AutoConstants.PX_CONTROLLER, 0, 0),
				new PIDController(AutoConstants.PY_CONTROLLER, 0, 0),
				thetaController,
				robotDrive::setModuleStates,
				robotDrive);

		// Reset odometry to the starting pose of the trajectory.
		robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

		// Run path following command, then stop at the end.
		return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
	}
}

package frc.robot.systems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;

// WPILib Imports

// Third party Hardware Imports
//import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;

// Robot Imports
import frc.robot.TeleopInput;
import frc.utils.SwerveUtils;
import frc.robot.HardwareMap;
import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.SwerveConstants.OIConstants;
import frc.robot.SwerveConstants.AutoConstants;

public class DriveFSMSystem {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE,
		AUTO_STATE
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	// The gyro sensor
	private AHRS gyro = new AHRS(SPI.Port.kMXP);

	// Slew rate filter variables for controlling lateral acceleration
	private double currentRotation = 0.0;
	private double currentTranslationDir = 0.0;
	private double currentTranslationMag = 0.0;

	private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.MAGNITUDE_SLEW_RATE);
	private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.ROTATIONAL_SLEW_RATE);
	private double prevTime = WPIUtilJNI.now() * DriveConstants.TIME_CONSTANT;

	// Create MAXSwerveModules
	private final MAXSwerveModule frontLeft = new MAXSwerveModule(
		HardwareMap.FRONT_LEFT_DRIVING_CAN_ID,
		HardwareMap.FRONT_LEFT_TURNING_CAN_ID,
		DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

	private final MAXSwerveModule frontRight = new MAXSwerveModule(
		HardwareMap.FRONT_RIGHT_DRIVING_CAN_ID,
		HardwareMap.FRONT_RIGHT_TURNING_CAN_ID,
		DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

	private final MAXSwerveModule rearLeft = new MAXSwerveModule(
		HardwareMap.REAR_LEFT_DRIVING_CAN_ID,
		HardwareMap.REAR_LEFT_TURNING_CAN_ID,
		DriveConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET);

	private final MAXSwerveModule rearRight = new MAXSwerveModule(
		HardwareMap.REAR_RIGHT_DRIVING_CAN_ID,
		HardwareMap.REAR_RIGHT_TURNING_CAN_ID,
		DriveConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET);

	// Odometry class for tracking robot pose
	private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
		DriveConstants.DRIVE_KINEMATICS,
		Rotation2d.fromDegrees(-gyro.getAngle()),
		new SwerveModulePosition[] {
			frontLeft.getPosition(),
			frontRight.getPosition(),
			rearLeft.getPosition(),
			rearRight.getPosition()
		});

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		// Perform hardware init
		gyro = new AHRS(SPI.Port.kMXP);

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
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return odometry.getPoseMeters();
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
		currentState = FSMState.TELEOP_STATE;

		resetEncoders();
		resetOdometry(getPose());
		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */

	public void resetAutonomus() {
		currentState = FSMState.AUTO_STATE;

		resetEncoders();
		resetOdometry(getPose());
		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(
			Rotation2d.fromDegrees(-gyro.getAngle()),
				new SwerveModulePosition[] {
					frontLeft.getPosition(),
					frontRight.getPosition(),
					rearLeft.getPosition(),
					rearRight.getPosition()
				},
				pose);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		odometry.update(Rotation2d.fromDegrees(-gyro.getAngle()),
						new SwerveModulePosition[] {
							frontLeft.getPosition(),
							frontRight.getPosition(),
							rearLeft.getPosition(),
							rearRight.getPosition()});
		switch (currentState) {
			case TELEOP_STATE:
				if (input != null) {
					drive(-MathUtil.applyDeadband(Math.pow(input.getControllerLeftJoystickY(),
						DriveConstants.TELEOP_JOYSTICK_POWER_CURVE), OIConstants.DRIVE_DEADBAND),
						-MathUtil.applyDeadband(Math.pow(input.getControllerLeftJoystickX(),
						DriveConstants.TELEOP_JOYSTICK_POWER_CURVE), OIConstants.DRIVE_DEADBAND),
						-MathUtil.applyDeadband(input.getControllerRightJoystickX(),
						OIConstants.DRIVE_DEADBAND), true, true);
					if (input.isBackButtonPressed()) {
						gyro.reset();
					}
				}
				break;
			case AUTO_STATE:
				if (input == null) {
					auto1(input);
				}
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
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
		switch (currentState) {
			case TELEOP_STATE:
				return FSMState.TELEOP_STATE;
			case AUTO_STATE:
				return FSMState.AUTO_STATE;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 * @param rateLimit     Whether to enable rate limiting for smoother control.
	 */
	public void drive(double xSpeed, double ySpeed, double rot,
		boolean fieldRelative, boolean rateLimit) {

		double xSpeedCommanded;
		double ySpeedCommanded;

		if (rateLimit) {
			// Convert XY to polar for rate limiting
			double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
			double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

			// Calculate the direction slew rate based on an estimate of the lateral acceleration
			double directionSlewRate;
			if (currentTranslationMag != 0.0) {
				directionSlewRate = Math.abs(DriveConstants.DIRECTION_SLEW_RATE
					/ currentTranslationMag);
			} else {
				directionSlewRate = DriveConstants.INSTANTANEOUS_SLEW_RATE;
				//some high number that means the slewrate is effectively instantaneous
			}

			double currentTime = WPIUtilJNI.now() * DriveConstants.TIME_CONSTANT;
			double elapsedTime = currentTime - prevTime;
			double angleDif = SwerveUtils.angleDifference(inputTranslationDir,
				currentTranslationDir);

			if (angleDif < DriveConstants.ANGLE_MULTIPLIER_1 * Math.PI) {
				currentTranslationDir = SwerveUtils.stepTowardsCircular(currentTranslationDir,
					inputTranslationDir, directionSlewRate * elapsedTime);
				currentTranslationMag = magLimiter.calculate(inputTranslationMag);
			} else if (angleDif > DriveConstants.ANGLE_MULTIPLIER_2 * Math.PI) {
				if (currentTranslationMag > DriveConstants.CURRENT_THRESHOLD) {
					// some small number to avoid floating-point errors with equality checking
					// keep currentTranslationDir unchanged
					currentTranslationMag = magLimiter.calculate(0.0);
				} else {
					currentTranslationDir = SwerveUtils.wrapAngle(currentTranslationDir + Math.PI);
					currentTranslationMag = magLimiter.calculate(inputTranslationMag);
				}
			} else {
				currentTranslationDir = SwerveUtils.stepTowardsCircular(currentTranslationDir,
					inputTranslationDir, directionSlewRate * elapsedTime);
				currentTranslationMag = magLimiter.calculate(0.0);
			}
			prevTime = currentTime;

			xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
			ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
			currentRotation = rotLimiter.calculate(rot);

		} else {
			xSpeedCommanded = xSpeed;
			ySpeedCommanded = ySpeed;
			currentRotation = rot;
		}

		// Convert the commanded speeds into the correct units for the drivetrain
		double xSpeedDelivered = xSpeedCommanded * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
		double ySpeedDelivered = ySpeedCommanded * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
		double rotDelivered = currentRotation * DriveConstants.MAX_ANGULAR_SPEED;

		var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered,
					rotDelivered, Rotation2d.fromDegrees(-gyro.getAngle()))
				: new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
		SwerveDriveKinematics.desaturateWheelSpeeds(
			swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

		int idx = 0;
		frontLeft.setDesiredState(swerveModuleStates[idx]);
		frontRight.setDesiredState(swerveModuleStates[++idx]);
		rearLeft.setDesiredState(swerveModuleStates[++idx]);
		rearRight.setDesiredState(swerveModuleStates[++idx]);
	}

	/**
	 * Balances gyro.
	 */
	public void balance() {
		double power;
		if (Math.abs(gyro.getRoll()) < 2 && Math.abs(gyro.getRoll()) > (-1 * 2)) {
			power = 0;
		} else if (gyro.getRoll() > 0) {
			power = Math.abs(gyro.getRoll())
				/ DriveConstants.BALENCE_SPEED_INVERSE_PROPORTION;
		} else if (gyro.getRoll() < 0) {
			power = -Math.abs(gyro.getRoll())
				/ DriveConstants.BALENCE_SPEED_INVERSE_PROPORTION;
		} else {
			power = 0;
		}
		// set to power field reletive so facing charge station
		frontLeft.setDesiredState(new SwerveModuleState(power, frontLeft.getState().angle));
		frontRight.setDesiredState(new SwerveModuleState(power, frontRight.getState().angle));
		rearLeft.setDesiredState(new SwerveModuleState(power, rearLeft.getState().angle));
		rearRight.setDesiredState(new SwerveModuleState(power, rearRight.getState().angle));
	}

	/**
	 * Autopath method for the first path.
	 * @param input Teleop input
	 */
	public void auto1(TeleopInput input) {
		if (input != null) {
			return;
		}
		System.out.println(getPose());
		double power;
		if (getPose().getX() > -1) {
			power = AutoConstants.MAX_SPEED_METERS_PER_SECOND;
		} else {
			power = 0;
		}
		frontLeft.setDesiredState(new SwerveModuleState(power, new Rotation2d(Math.PI)));
		frontRight.setDesiredState(new SwerveModuleState(power, new Rotation2d(Math.PI)));
		rearLeft.setDesiredState(new SwerveModuleState(power, new Rotation2d(Math.PI)));
		rearRight.setDesiredState(new SwerveModuleState(power, new Rotation2d(Math.PI)));
	}

	/**
	 * Sets the wheels into an X formation to prevent movement.
	 */
	public void setX() {
		frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(
			Math.toDegrees(Math.PI / 2))));
		frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(
			-Math.toDegrees(Math.PI / 2))));
		rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(
			-Math.toDegrees(Math.PI / 2))));
		rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(
			Math.toDegrees(Math.PI / 2))));
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
			desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);


		int idx = 0;
		frontLeft.setDesiredState(desiredStates[idx]);
		frontRight.setDesiredState(desiredStates[++idx]);
		rearLeft.setDesiredState(desiredStates[++idx]);
		rearRight.setDesiredState(desiredStates[++idx]);
	}

	/** Resets the drive encoders to currently read a position of 0. */
	public void resetEncoders() {
		frontLeft.resetEncoders();
		rearLeft.resetEncoders();
		frontRight.resetEncoders();
		rearRight.resetEncoders();
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		gyro.reset();
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		return Rotation2d.fromDegrees(-gyro.getAngle()).getDegrees();
	}

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
	public double getTurnRate() {
		return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
	}

}

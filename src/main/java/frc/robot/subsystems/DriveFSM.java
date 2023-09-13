package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.SwerveConstants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.TeleopInput;
import edu.wpi.first.math.MathUtil;
import frc.robot.SwerveConstants.OIConstants;

public class DriveFSM {

	// Create MAXSwerveModules
	private final MAXSwerveModule frontLeft = new MAXSwerveModule(
		DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
		DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
		DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

	private final MAXSwerveModule frontRight = new MAXSwerveModule(
		DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
		DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
		DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

	private final MAXSwerveModule rearLeft = new MAXSwerveModule(
		DriveConstants.REAR_LEFT_DRIVING_CAN_ID,
		DriveConstants.REAR_LEFT_TURNING_CAN_ID,
		DriveConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET);

	private final MAXSwerveModule rearRight = new MAXSwerveModule(
		DriveConstants.REAR_RIGHT_DRIVING_CAN_ID,
		DriveConstants.REAR_RIGHT_TURNING_CAN_ID,
		DriveConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET);

	// The gyro sensor
	private AHRS gyro = new AHRS(SPI.Port.kMXP);

	// Slew rate filter variables for controlling lateral acceleration
	private double currentRotation = 0.0;
	private double currentTranslationDir = 0.0;
	private double currentTranslationMag = 0.0;

	private SlewRateLimiter magLimiter;
	private SlewRateLimiter rotLimiter;
	private double prevTime = WPIUtilJNI.now() * DriveConstants.TIME_CONSTANT;

	// Constants
	private static final double SLEW_RATE_MAX = 500.0;
	private static final double SMALL_ANGLE_HEADING_THRESHOLD_RADIANS = 0.45 * Math.PI;
	private static final double LARGE_ANGLE_HEADING_THRESHOLD_RADIANS = 0.85 * Math.PI;
	private static final double TRANSLATION_MAGNITUDE_THRESHOLD = 1e-4;
	private static final int COUNTER_PERIOD = 40;
    private int counter = 0;
	// Odometry class for tracking robot pose
	private SwerveDriveOdometry odometry;
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
        IDLE,
		TELEOP,
		PATH1,
        PATH2,
        PATH3,
        PATH4,
        PATH5A,
        PATH6A,
        PATH6B,
	}


	/* ======================== Private variables ======================== */
	private FSMState currentState;


	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSM() {
        odometry = new SwerveDriveOdometry(
            DriveConstants.DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(-gyro.getAngle()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            });
        magLimiter = new SlewRateLimiter(DriveConstants.MAGNITUDE_SLEW_RATE);
        rotLimiter = new SlewRateLimiter(DriveConstants.ROTATIONAL_SLEW_RATE);
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
		counter++;
        // Update the odometry in the periodic block
        // System.out.println("front right: " + m_frontRight.getPosition());
        // System.out.println("front left: " + m_frontLeft.getPosition());
        // System.out.println("back right: " + m_rearRight.getPosition());
        // System.out.println("back left: " + m_rearLeft.getPosition());

		if (counter % COUNTER_PERIOD == 0) {
			System.out.println(getPose());
		}
		odometry.update(
			Rotation2d.fromDegrees(-gyro.getAngle()),
			new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				rearLeft.getPosition(),
				rearRight.getPosition()
			});
		switch (currentState) {
			case IDLE:
                handleIdleState(input);
				break;
			case TELEOP:
                handleTeleopState(input);
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
			case IDLE:
                return FSMState.IDLE;
			case TELEOP:
				return FSMState.TELEOP;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}
    private void handleTeleopState(TeleopInput input) {
        if (input == null) {
			return;
		}
        drive(
            -MathUtil.applyDeadband(
                input.getLeftY(), OIConstants.DRIVE_DEADBAND),
            -MathUtil.applyDeadband(
                input.getLeftX(), OIConstants.DRIVE_DEADBAND),
            -MathUtil.applyDeadband(
                input.getRightX(), OIConstants.DRIVE_DEADBAND),
            true,
            true);
    }
    private void handleIdleState(TeleopInput input) {

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
	 * Returns the gyro of the robot.
	 *
	 * @return The gyro.
	 */
	public AHRS getGyro() {
		return gyro;
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
				directionSlewRate = SLEW_RATE_MAX;
				// some high number that means the slewrate is effectively instantaneous
			}

			double currentTime = WPIUtilJNI.now() * DriveConstants.TIME_CONSTANT;
			double elapsedTime = currentTime - prevTime;
			double angleDif = SwerveUtils.angleDifference(inputTranslationDir,
				currentTranslationDir);

			if (angleDif < SMALL_ANGLE_HEADING_THRESHOLD_RADIANS) {
				currentTranslationDir = SwerveUtils.stepTowardsCircular(currentTranslationDir,
					inputTranslationDir, directionSlewRate * elapsedTime);
				currentTranslationMag = magLimiter.calculate(inputTranslationMag);
			} else if (angleDif > LARGE_ANGLE_HEADING_THRESHOLD_RADIANS) {
				if (currentTranslationMag > TRANSLATION_MAGNITUDE_THRESHOLD) {
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
		frontLeft.setDesiredState(swerveModuleStates[0]);
		frontRight.setDesiredState(swerveModuleStates[1]);
		rearLeft.setDesiredState(swerveModuleStates[2]);
		rearRight.setDesiredState(swerveModuleStates[(2 + 1)]);
	}

	/**
	 * Balances the chassis on the charging station.
	 */
	public void balance() {
		double power;
		if (Math.abs(gyro.getRoll()) < 2 && Math.abs(gyro.getRoll()) > (-1 * 2)) {
			power = 0;
		} else if (gyro.getRoll() > 0) {
			power = Math.abs(gyro.getRoll())
				/ DriveConstants.BALENCE_SPEED_INVERSE_PROPORTION_CONSTANT;
		} else {
			power = -Math.abs(gyro.getRoll())
				/ DriveConstants.BALENCE_SPEED_INVERSE_PROPORTION_CONSTANT;
		}
		// set to power field reletive so facing charge station
		frontLeft.setDesiredState(new SwerveModuleState(power, frontLeft.getState().angle));
		frontRight.setDesiredState(new SwerveModuleState(power, frontRight.getState().angle));
		rearLeft.setDesiredState(new SwerveModuleState(power, rearLeft.getState().angle));
		rearRight.setDesiredState(new SwerveModuleState(power, rearRight.getState().angle));
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
		frontLeft.setDesiredState(desiredStates[0]);
		frontRight.setDesiredState(desiredStates[1]);
		rearLeft.setDesiredState(desiredStates[2]);
		rearRight.setDesiredState(desiredStates[(2 + 1)]);
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
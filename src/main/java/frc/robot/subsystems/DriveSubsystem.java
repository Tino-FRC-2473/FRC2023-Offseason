// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import frc.robot.SwerveConstants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase {
	// Create MAXSwerveModules
	private final MAXSwerveModule mFrontLeft = new MAXSwerveModule(
		DriveConstants.kFrontLeftDrivingCanId,
		DriveConstants.kFrontLeftTurningCanId,
		DriveConstants.kFrontLeftChassisAngularOffset);

	private final MAXSwerveModule mFrontRight = new MAXSwerveModule(
		DriveConstants.kFrontRightDrivingCanId,
		DriveConstants.kFrontRightTurningCanId,
		DriveConstants.kFrontRightChassisAngularOffset);

	private final MAXSwerveModule mRearLeft = new MAXSwerveModule(
		DriveConstants.kRearLeftDrivingCanId,
		DriveConstants.kRearLeftTurningCanId,
		DriveConstants.kBackLeftChassisAngularOffset);

	private final MAXSwerveModule mRearRight = new MAXSwerveModule(
		DriveConstants.kRearRightDrivingCanId,
		DriveConstants.kRearRightTurningCanId,
		DriveConstants.kBackRightChassisAngularOffset);

	// The gyro sensor
	//private AHRS gyro = new AHRS(SPI.Port.kMXP);
	private final ADIS16470_IMU gyro = new ADIS16470_IMU();

	// Slew rate filter variables for controlling lateral acceleration
	private double mCurrentRotation = 0.0;
	private double mCurrentTranslationDir = 0.0;
	private double mCurrentTranslationMag = 0.0;

	private SlewRateLimiter mMagLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
	private SlewRateLimiter mRotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
	private double mPrevTime = WPIUtilJNI.now() * 1e-6;

	// Odometry class for tracking robot pose
	SwerveDriveOdometry mOdometry = new SwerveDriveOdometry(
		DriveConstants.kDriveKinematics,
		Rotation2d.fromDegrees(gyro.getAngle()),
		new SwerveModulePosition[] {
			mFrontLeft.getPosition(),
			mFrontRight.getPosition(),
			mRearLeft.getPosition(),
			mRearRight.getPosition()
		});

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
	}

	@Override
	public void periodic() {
	// Update the odometry in the periodic block
	// System.out.println("front right: " + m_frontRight.getPosition());
	// System.out.println("front left: " + m_frontLeft.getPosition());
	// System.out.println("back right: " + m_rearRight.getPosition());
	// System.out.println("back left: " + m_rearLeft.getPosition());
	mOdometry.update(
		Rotation2d.fromDegrees(gyro.getAngle()),
		new SwerveModulePosition[] {
			mFrontLeft.getPosition(),
			mFrontRight.getPosition(),
			mRearLeft.getPosition(),
			mRearRight.getPosition()
		});
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return mOdometry.getPoseMeters();
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		mOdometry.resetPosition(
			Rotation2d.fromDegrees(gyro.getAngle()),
				new SwerveModulePosition[] {
					mFrontLeft.getPosition(),
					mFrontRight.getPosition(),
					mRearLeft.getPosition(),
					mRearRight.getPosition()
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
			if (mCurrentTranslationMag != 0.0) {
				directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
			} else {
				directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
			}

			double currentTime = WPIUtilJNI.now() * 1e-6;
			double elapsedTime = currentTime - m_prevTime;
			double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

			if (angleDif < 0.45*Math.PI) {
				mCurrentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
				mCurrentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
			} else if (angleDif > 0.85*Math.PI) {
				if (mCurrentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
					// keep currentTranslationDir unchanged
					mCurrentTranslationMag = m_magLimiter.calculate(0.0);
				} else {
					mCurrentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
					mCurrentTranslationMag = mMagLimiter.calculate(inputTranslationMag);
				}
			} else {
				mCurrentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
				mCurrentTranslationMag = mMagLimiter.calculate(0.0);
			}
			mPrevTime = currentTime;
			
			xSpeedCommanded = mCurrentTranslationMag * Math.cos(mCurrentTranslationDir);
			ySpeedCommanded = mCurrentTranslationMag * Math.sin(mCurrentTranslationDir);
			mCurrentRotation = mRotLimiter.calculate(rot);

		} else {
			xSpeedCommanded = xSpeed;
			ySpeedCommanded = ySpeed;
			mCurrentRotation = rot;
		}

		// Convert the commanded speeds into the correct units for the drivetrain
		double xSpeedDelivered = xSpeedCommanded * DriveConstants.K_MAX_SPEED_METERS_PER_SECOND;
		double ySpeedDelivered = ySpeedCommanded * DriveConstants.K_MAX_SPEED_METERS_PER_SECOND;
		double rotDelivered = mCurrentRotation * DriveConstants.kMaxAngularSpeed;

		var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyro.getAngle()))
				: new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
		SwerveDriveKinematics.desaturateWheelSpeeds(
			swerveModuleStates, DriveConstants.K_MAX_SPEED_METERS_PER_SECOND);
		mFrontLeft.setDesiredState(swerveModuleStates[0]);
		mFrontRight.setDesiredState(swerveModuleStates[1]);
		mRearLeft.setDesiredState(swerveModuleStates[2]);
		mRearRight.setDesiredState(swerveModuleStates[3]);
	}

	public void balence() {
		double power;
		// if (Math.abs(gyro.getRoll()) < 2 && Math.abs(gyro.getRoll())
		//   > -2) {
		//   power = 0;
		// } else if (gyro.getRoll() > 0) {
		//   power = Math.abs(gyro.getRoll()) / 200;
		// } else if (gyro.getRoll() < 0) {
		//   power = -Math.abs(gyro.getRoll()) / 200;
		// }
		// set to power field reletive so facing charge station
	}

	/**
	 * Sets the wheels into an X formation to prevent movement.
	 */
	public void setX() {
		mFrontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		mFrontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		mRearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		mRearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
			desiredStates, DriveConstants.K_MAX_SPEED_METERS_PER_SECOND);
		mFrontLeft.setDesiredState(desiredStates[0]);
		mFrontRight.setDesiredState(desiredStates[1]);
		mRearLeft.setDesiredState(desiredStates[2]);
		mRearRight.setDesiredState(desiredStates[3]);
	}

	/** Resets the drive encoders to currently read a position of 0. */
	public void resetEncoders() {
		mFrontLeft.resetEncoders();
		mRearLeft.resetEncoders();
		mFrontRight.resetEncoders();
		mRearRight.resetEncoders();
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
		return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
	}

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
	public double getTurnRate() {
		return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
	}
}

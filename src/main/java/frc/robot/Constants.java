package frc.robot;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final double APRILTAG_TO_HIGH_CUBENODE_METERS = Units.inchesToMeters(8.5);
	public static final double INVALID_TURN_RETURN_DEGREES = 360;
	public static final double ANGLE_TO_TARGET_THRESHOLD_DEGREES = 3;
	public static final double SUBSTATION_ANGLE_THRESHOLD_DEGREES = 5;
	public static final double CV_TURN_POWER = 0.05;
	public static final double CV_PID_CLAMP_THRESHOLD = 0.08;
	public static final double CV_FORWARD_POWER = 0.1;
	public static final double HIGHER_TAPE_DRIVEUP_DISTANCE_INCHES = 67;
	public static final double LOWER_TAPE_DRIVEUP_DISTANCE_INCHES = 45;
	public static final double TAG_DRIVEUP_DISTANCE_INCHES = 32;
	public static final double CUBE_DRIVEUP_DISTANCE_INCHES = 45;
	public static final double CUBE_DISTANCE_ADD = 10;
	public static final double CONE_DISTANCE_ADD = 20;
	public static final int WEBCAM_PIXELS_WIDTH = 480;
	public static final int WEBCAM_PIXELS_HEIGHT = 480;
	public static final double WHEEL_DIAMETER_INCHES = 6.0; //7.65
	public static final double MAX_POWER = 1;
	public static final double REDUCED_MAX_POWER = 0.5;
	public static final double TELEOP_MIN_TURN_POWER = 0.03;
	public static final double TELEOP_MIN_MOVE_POWER = 0.02;
	public static final double TELEOP_ACCELERATION_CONSTANT = 0.05;
	public static final double TELEOP_ACCELERATION_MIN = 0.1;
	public static final double TURNING_IN_PLACE_THRESHOLD = 0.05;
	public static final double ENCODER_CONSTANT_ONE_TICK_TO_ONE_REV = 1.1065983131;
	public static final double COUNTS_PER_MOTOR_REVOLUTION = 42;
	public static final double GEAR_RATIO = 8.0; //26.0 * 4.67 / 12.0;
	public static final double REVOLUTIONS_PER_INCH = GEAR_RATIO / (Math.PI * WHEEL_DIAMETER_INCHES * ENCODER_CONSTANT_ONE_TICK_TO_ONE_REV);
	public static final double GYRO_MULTIPLER_TELOP = 0.9181799233;
	public static final double MIN_TURN_POWER = 0.125;
	public static final double TURN_ERROR_THRESHOLD_DEGREE = 5.0;
	public static final double TURN_ERROR_POWER_RATIO = 360;
	public static final double ACCELERATION_CONSTANT_FOR_TURN = 2.8;
	public static final double SPEED_CONSTANT_FOR_TURN = 0.7;
	public static final double ONE_REVOLUTION_DEGREES = 360;
	public static final double HALF_REVOLUTION_DEGREES = 180;
	public static final double GYRO_TURN_MULTIPLER_BELOW_90 = 0.987;
	public static final double CHARGING_STATION_LEVELED_ERROR_DEGREES = 2;
	public static final double CHARGING_STATION_BALANCE_CONSTANT_PID_P = 170;
	public static final double AUTONOMUS_ARM_MOVE_POWER = 0.25;
	public static final double AUTONOMUS_GROUND_MOUNT_MOVE_POWER = 0.3;
	public static final double AUTONOMUS_X_MOVE_THRESHOLD = 2;
	public static final double AUTONOMUS_Y_MOVE_THRESHOLD = 20;
	public static final double POWER_TO_HOLD_ROBO_ON_TILTED_CS = 0.03;
	public static final double OVERRUN_THRESHOLD = 0.007;
	// Path points
	// push in, out of community, charge station
	public static final double P1X1 = 1;
	public static final double P1X2 = -170;
	public static final double P1X3 = -94;
	// push in, charge station
	public static final double P2X1 = 1;
	public static final double P2X2 = -94;
	// push in, out of community (on edges)
	public static final double P3X1 = 1;
	public static final double P3X2 = -150;
	// push in only
	public static final double P4X1 = 1;
	// deposit backwards, exit community, charge station
	public static final double P5X1 = -1;
	public static final double P5X2 = 170;
	public static final double P5X3 = 86;
	// deposit backwards, charge station
	public static final double P6X1 = -1;
	public static final double P6X2 = 86;
	// deposit backwards, out of community (on edges)
	public static final double P7X1 = -1;
	public static final double P7X2 = 170;
	// ODO Constants
	public static final double DX_INCHES_CONST = 0.8880486672;
	public static final double DY_INCHES_CONST = 1.1742067733;
	public static final double SHOOTING_CIRCLE_RADIUS = 120.000;
	public static final double HUB_X_COORDINATE = 40;
	public static final double HUB_Y_COORDINATE = 0;
	public static final double QUARTER_PI = Math.PI / 4;
	public static class VisionConstants {
		public static final double CAM_HEIGHT_METERS = Units.inchesToMeters(20.75);
		public static final double CAM_PITCH_RADIANS = Units.degreesToRadians(0);
		public static final double CUBE_HEIGHT_METERS = Units.inchesToMeters(8.3);
		public static final double CONE_HEIGHT_METERS = Units.inchesToMeters(12.8125);
		public static final double HIGH_TAPE_HEIGHT_METERS = Units.inchesToMeters(43.75);
		public static final double LOW_TAPE_HEIGHT_METERS = Units.inchesToMeters(24);
		public static final double TAG_HEIGHT_METERS = Units.inchesToMeters(1.927);
		public static final double ALIGNMENT_THRESHOLD_DEGREES = 3;
		public static final int LOWERTAPE_PIPELINE_INDEX = 3;
		public static final int HIGHERTAPE_PIPELINE_INDEX = 1;
		public static final int THREEDTAG_PIPELINE_INDEX = 0;
		public static final int CUBE_PIPELINE_INDEX = 4;
		public static final int CONE_PIPELINE_INDEX = 2;
		public static final int NO_TARGETS_RETURN = -180;
		public static final int NO_Y_RETURN = 1000;
		public static final String CAMERA_NAME = "OV5647";
	}
}

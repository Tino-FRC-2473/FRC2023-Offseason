package frc.robot;
import edu.wpi.first.math.util.Units;

public class Constants {
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
	public static class NetworkTablesConstants {
		public static final String TABLE_NAME = "datatable";
		public static final String CUBE_YAW_TOPIC = "cube_yaw";
		public static final String CUBE_DISTANCE_TOPIC = "cube_distance";
		public static final String CONE_YAW_TOPIC = "cone_yaw";
		public static final String CONE_DISTANCE_TOPIC = "cone_distance";
		public static final String FPS_COUNTER_TOPIC = "fps_incremented_value";
	}
}

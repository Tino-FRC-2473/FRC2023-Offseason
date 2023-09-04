package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

	public static class VisionConstants {
		public static final String CAMERA_NAME = "OV5647";
		public static final double CAM_HEIGHT_METERS = Units.inchesToMeters(20.75);
		public static final double CAM_PITCH_RADIANS = Units.degreesToRadians(0);
		public static final double HIGH_TAPE_HEIGHT_METERS = Units.inchesToMeters(43.75);
		public static final double LOW_TAPE_HEIGHT_METERS = Units.inchesToMeters(24);
		public static final int LOWERTAPE_PIPELINE_INDEX = 1;
		public static final int HIGHERTAPE_PIPELINE_INDEX = 2;
		public static final int THREEDTAG_PIPELINE_INDEX = 0;
		public static final int NO_TARGETS_RETURN = -180;
	}
}

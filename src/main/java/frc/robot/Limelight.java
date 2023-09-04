package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;

public class Limelight {
	private PhotonCamera photonCamera;
	/** Intializes Limelight object.
	 * @param cam the PhotonCamera object that the code gets input from.
	*/
	public Limelight(PhotonCamera cam) {
		photonCamera = cam;
		photonCamera.setDriverMode(false);
	}

	/** Gets the yaw to the target.
	 * @return returns the yaw in degress from the limelight to the target.
	*/
	public double getLowTapeYaw() {
		photonCamera.setPipelineIndex(VisionConstants.LOWERTAPE_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		if (result.hasTargets()) {
			return Units.radiansToDegrees(result.getBestTarget().getYaw());
		}
		return VisionConstants.NO_TARGETS_RETURN;
	}
	/** Gets the yaw to the target.
	 * @return returns the yaw in degress from the limelight to the target.
	*/
	public double getHighTapeYaw() {
		photonCamera.setPipelineIndex(VisionConstants.HIGHERTAPE_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		if (result.hasTargets()) {
			return Units.radiansToDegrees(result.getBestTarget().getYaw());
		}
		return VisionConstants.NO_TARGETS_RETURN;
	}
	/** Gets the distance to the reflective tape.
	 * @return returns the distance in meters to the tape.
	*/
	public double getLowTapeDistance() {
		photonCamera.setPipelineIndex(VisionConstants.LOWERTAPE_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		if (result.hasTargets()) {
			return (VisionConstants.LOW_TAPE_HEIGHT_METERS - VisionConstants.CAM_HEIGHT_METERS)
				/ Math.tan(Units.degreesToRadians(result.getBestTarget().getPitch()
				+ VisionConstants.CAM_PITCH_RADIANS));
		}
		return VisionConstants.NO_TARGETS_RETURN;
	}
	/** Gets the distance to the reflective tape.
	 * @return returns the distance in meters to the tape.
	*/
	public double getHighTapeDistance() {
		photonCamera.setPipelineIndex(VisionConstants.HIGHERTAPE_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		if (result.hasTargets()) {
			return (VisionConstants.HIGH_TAPE_HEIGHT_METERS - VisionConstants.CAM_HEIGHT_METERS)
				/ Math.tan(Units.degreesToRadians(result.getBestTarget().getPitch()
				+ VisionConstants.CAM_PITCH_RADIANS));
		}
		return VisionConstants.NO_TARGETS_RETURN;
	}
}


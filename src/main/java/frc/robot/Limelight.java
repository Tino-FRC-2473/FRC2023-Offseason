package frc.robot;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;

public class Limelight {
	private PhotonCamera photonCamera;
	private int currPipelineIndex;
	/** Intializes Limelight object. */
	public Limelight() {
		photonCamera = new PhotonCamera("OV5647");
		photonCamera.setDriverMode(false);
		currPipelineIndex = VisionConstants.LOWERTAPE_PIPELINE_INDEX;
	}

	/** Gets the yaw to the target.
	 * @return returns the yaw in degress from the limelight to the target.
	*/
	public double getYaw() {
		photonCamera.setPipelineIndex(currPipelineIndex);
		var result = photonCamera.getLatestResult();
		if (result.hasTargets()) {
			return Units.radiansToDegrees(result.getBestTarget().getYaw());
		} else {
			return VisionConstants.NO_TARGETS_RETURN;
		}
	}

	/** Gets the distance to the reflective tape.
	 * @return returns the distance in meters to the tape.
	*/
	public double getTapeDistance() {
		photonCamera.setPipelineIndex(currPipelineIndex);
		return (VisionConstants.TAG_HEIGHT_METERS - VisionConstants.CAM_HEIGHT_METERS)
			/ Math.tan(Units.degreesToRadians(photonCamera.getLatestResult()
			.getBestTarget().getPitch()));
	}

	/**
	 * Sets the pipeline index of the module.
	 * @param index the pipeline index of the tape (low tape or high tape)
	 */
	public void setTapeMode(int index) {
		currPipelineIndex = index;
	}
}


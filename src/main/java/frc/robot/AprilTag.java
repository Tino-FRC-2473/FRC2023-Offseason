package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;

public class AprilTag {
	private PhotonCamera photonCamera;

	/** Intializes April Tag detector.
	 * @param cam the PhotonCamera object that the code gets input from.
	*/
	public AprilTag(PhotonCamera cam) {
		photonCamera = cam;
		photonCamera.setDriverMode(false);
	}

	/** @return Returns the latest pipeline result. */
	public PhotonPipelineResult getResult() {
		photonCamera.setPipelineIndex(VisionConstants.THREEDTAG_PIPELINE_INDEX);
		return photonCamera.getLatestResult();
	}

	/** @return Returns the x (horizontal) coordinate in meters of the robot relative to the april tag found. */
	public double getX() {
		PhotonPipelineResult result = getResult();
		if (result.hasTargets()) {
			return result.getBestTarget().getBestCameraToTarget().getY();
		}
		return VisionConstants.NO_TARGETS_RETURN;
	}

	/** @return Returns the y (forward/backward) coordinate in meters of the robot relative to the april tag found. */
	public double getY() {
		PhotonPipelineResult result = getResult();
		if (result.hasTargets()) {
			return result.getBestTarget().getBestCameraToTarget().getX();
		}
		return VisionConstants.NO_TARGETS_RETURN;
	}

	/** @return Returns the angle (in degrees) of the robot relative to the april tag found. */
	public double getAngle() {
		PhotonPipelineResult result = getResult();
		if (result.hasTargets()) {
			return Units.radiansToDegrees(result.getBestTarget()
				.getBestCameraToTarget().getRotation().getAngle());
		}
		return VisionConstants.NO_TARGETS_RETURN;
	}

}

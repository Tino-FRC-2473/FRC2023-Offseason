package frc.robot;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;

public class ReflectiveTape {
    private PhotonCamera photonCamera;
	/** Intializes Limelight object.
	 * @param cam the PhotonCamera object that the code gets input from.
	*/
	public ReflectiveTape(PhotonCamera cam) {
		photonCamera = cam;
	}
	/** Gets the yaw to the target.
	 * @return returns the yaw in degress from the limelight to the target.
	*/
	public double getLowTapeYaw() {
		photonCamera.setPipelineIndex(VisionConstants.LOWERTAPE_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		if (result.hasTargets()) {
			return result.getBestTarget().getYaw();
		} else {
			return VisionConstants.NO_TARGETS_RETURN;
		}
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
	/** Gets the y distance of the target.
	 * @return returns the y distance of target
	*/
	public double getY() {
		PhotonTrackedTarget target = photonCamera.getLatestResult().getBestTarget();
		return photonCamera.getLatestResult().hasTargets()
			? target.getBestCameraToTarget().getY() : VisionConstants.NO_Y_RETURN;
	}
	/** Gets the higher target from the pipeline.
	 * @return returns the y distance of target
	*/
	public PhotonTrackedTarget getHighTape() {
		photonCamera.setPipelineIndex(VisionConstants.HIGHERTAPE_PIPELINE_INDEX);
		var result = photonCamera.getLatestResult();
		//SmartDashboard.putNumber("target pitch", targets.get(0).getPitch());
		//SmartDashboard.putNumber("target 2", targets.get(1).getPitch());
		if (result.hasTargets() && result.getTargets().size() >= 2) {
			List<PhotonTrackedTarget> targets = result.getTargets();
			//return 50;
			//return targets.get(0);
			return targets.get(0).getPitch() > targets.get(1).getPitch()
				? targets.get(0) : targets.get(1);
		}
		return null;
	}
}

package frc.robot;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;
/**
	* The PhotonCameraWrapper class contains methods for estimating position
	* of robot relative to AprilTags on the field and updates SmartDashboard
	* with its coordinates.
	*/
public class AprilTag {
	private PhotonCamera photonCamera;

	/** Intializes April Tag detector. */
	public AprilTag() {
		AprilTagFieldLayout atfl = null;
		try {
			atfl = AprilTagFieldLayout.loadFromResource(
				AprilTagFields.k2023ChargedUp.m_resourceFile);
			System.out.println(atfl);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		photonCamera = new PhotonCamera("OV5647");
		photonCamera.setDriverMode(false);
	}

	/** @return Returns the x coordinate of the robot relative to the april tag found. */
	public double getX() {
		return photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget().getX();
	}

	/** @return Returns the y coordinate of the robot relative to the april tag found. */
	public double getY() {
		return photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget().getY();
	}

	/** @return Returns the angle of the robot relative to the april tag found. */
	public double getAngle() {
		photonCamera.setPipelineIndex(VisionConstants.THREEDTAG_PIPELINE_INDEX);
		return Units.radiansToDegrees(
			photonCamera.getLatestResult().getBestTarget()
			.getBestCameraToTarget().getRotation().getAngle());
	}

}

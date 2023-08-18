package frc.robot;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import java.util.NoSuchElementException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class AprilTag {

    private PhotonCamera photonCamera;
    private PhotonPoseEstimator poseEstimator;
    // Angle between horizontal and the camera.
    final double CAMERA_YAW_DEGREES = 10;
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(20.75);
    final double CAMERA_X_OFFSET = Units.inchesToMeters(4.5);
    final double CAMERA_Y_OFFSET = Units.inchesToMeters(8);
   
    // CHANGE FOR APRIL TAG
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(1.91);
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    final double TARGET_TO_CAMERA = TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS;
    double TARGET_PITCH_RADIANS =  Units.degreesToRadians(0);
    private Transform3d robotToCam;

    public AprilTag() {
        photonCamera = new PhotonCamera("OV5647");
        photonCamera.setDriverMode(false);

        AprilTagFieldLayout atfl = null;
		try {
			atfl = AprilTagFieldLayout.loadFromResource(
				AprilTagFields.k2023ChargedUp.m_resourceFile);
			System.out.println(atfl);
		} catch (IOException e) {
			e.printStackTrace();
		}

        // Transform the robot pose to find the camera's pose
        robotToCam = new Transform3d(new Translation3d(CAMERA_X_OFFSET, CAMERA_Y_OFFSET, CAMERA_HEIGHT_METERS), new Rotation3d(0, 0, 0));
       
        // Initialize pose estimator with rotation, translation values
        poseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.LOWEST_AMBIGUITY, photonCamera, robotToCam);

    }

    // Returns estimated pose of april tag from camera or null if optPose doesn't exist
    public Pose3d getEstimatedGlobalPose(Pose2d p) {
        //System.out.println("here");
        photonCamera.setPipelineIndex(4);
        Optional<EstimatedRobotPose> optPose = poseEstimator.update();
        if (optPose != null && optPose.get() != null) {
            return optPose.get().estimatedPose;
        } else {
            return null;
        }
    }

    // Returns x pose of target from camera, else unreasonable number 
    public double getTagX() {
        try {
            return getEstimatedGlobalPose(new Pose2d()).toPose2d().getX();
            
        } catch (NoSuchElementException e) {
            e.printStackTrace();
        }
        return 10000;
    }

    // Returns y pos of target from camera
    // public double getTagY() {
    //     photonCamera.setPipelineIndex(4);
    //     if (getEstimatedGlobalPose() != null) {
    //         return getEstimatedGlobalPose().toPose2d().getY();
    //     }
    //     return 100000;
    // }

    // // Returns angle to turn to target (in degrees)
    // public double getAngle() {
    //     photonCamera.setPipelineIndex(4);
    //     if (getEstimatedGlobalPose() != null) {
    //         return getEstimatedGlobalPose().toPose2d().getRotation().getDegrees();
    //     }
    //     return 100000;
    // }

    // NEED TO MODIFY FUNCTION FOR APRIL TAG
    public int isCentered() {
        SmartDashboard.putNumber("Pipeline index", photonCamera.getPipelineIndex());
        photonCamera.setPipelineIndex(4);
        var result = photonCamera.getLatestResult();
        if(result.hasTargets()) { //at target = 0, to the left of target = pos, to the right of target = neg
            TARGET_PITCH_RADIANS = result.getBestTarget().getPitch();
            boolean centered = false;
            //SmartDashboard.putNumber("Distance to Tape", getTapeDistance2());
                //System.out.println(Units.radiansToDegrees(result.getBestTarget().getYaw()) + ", without conversion ->" + result.getBestTarget().getYaw());
                
            if(result.getBestTarget() != null && result.getBestTarget().getYaw() >= -CAMERA_YAW_DEGREES && result.getBestTarget().getYaw() <= CAMERA_YAW_DEGREES) {
                centered = true;
                //System.out.println("centered");
                SmartDashboard.putBoolean("Centered or not", centered);
                return 0;
            } else {
                //System.out.println("not centered");
                if(result.getBestTarget().getYaw()>0){
                    return 1;
                }else{
                    return -1;
                }
            }
        } else{
            System.out.println("no target");
            return 50;
        }
        // SmartDashboard.putNumber("Camera Yaw to target (deg)", result.getBestTarget().getYaw());
    }

    

}
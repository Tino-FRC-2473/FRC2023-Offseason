package frc.robot;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * The PhotonCameraWrapper class contains methods for estimating position
 * of robot relative to AprilTags on the field and updates SmartDashboard
 * with its coordinates.
 */
public class AprilTag {
	public static final double APRIL_TAG_ANGLE_DEGREES = 180;
	public static final double ANGULAR_P = 0.01;
	public static final double ANGULAR_D = 0;
		/** PhotonCamera object representing a camera that is
		 * connected to PhotonVision.*/
	private PhotonCamera photonCamera;
		/** RobotPoseEstimator object to estimate position of robot.*/
	private PhotonPoseEstimator robotPoseEstimator;
		/** PIDController object to implement PID for robot turning.*/
	private PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

		/** Last timestamp holder to check if code is running faster than limelight.*/
	private double lastTs = 0;

		/** Creates a new PhotonCameraWrapper. */
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

		photonCamera =
				new PhotonCamera("OV5647");
		photonCamera.setDriverMode(false);
		robotPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.LOWEST_AMBIGUITY,
		photonCamera, new Transform3d(
			new Translation3d(Units.inchesToMeters(VisionConstants.CAM_OFFSET_INCHES),
			0,
			VisionConstants.CAM_HEIGHT_METERS),
			new Rotation3d(
					0, VisionConstants.CAM_PITCH_RADIANS,
					0)));
	}

    public double getX() {
        return photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget().getX();
    }

    public double getY() {
        return photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget().getY();
    }

    public double getAngle() {
        return Units.radiansToDegrees(photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget().getRotation().getAngle());
    }
	
	public void setPipelineIndex(int index) {
		photonCamera.setPipelineIndex(index);
	}
}








// package frc.robot;

// import java.io.IOException;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonUtils;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.AnalogGyro;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;

// import java.util.NoSuchElementException;
// import java.util.Optional;
// import org.photonvision.EstimatedRobotPose;

// import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
// import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;


// public class AprilTag {

//     private PhotonCamera photonCamera;
//     private PhotonPoseEstimator poseEstimator;
//     // Angle between horizontal and the camera.
//     final double CAMERA_YAW_DEGREES = 10;
//     final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(20.75);
//     final double CAMERA_X_OFFSET = Units.inchesToMeters(4.5);
//     final double CAMERA_Y_OFFSET = Units.inchesToMeters(8);
   
//     // CHANGE FOR APRIL TAG
//     final double TARGET_HEIGHT_METERS = Units.feetToMeters(1.91);
//     final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
//     final double TARGET_TO_CAMERA = TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS;
//     double TARGET_PITCH_RADIANS =  Units.degreesToRadians(0);
//     private Transform3d robotToCam;
//     private Optional<EstimatedRobotPose> optPose;

//     // UPDATE VALUE 
//     // horizontal distance between wheels
//     static final double kTrackWidth = Units.inchesToMeters(20);
//     private final Encoder leftEncoder = new Encoder(0, 1);
//     private final Encoder rightEncoder = new Encoder(2, 3);
//     private final AnalogGyro gyro = new AnalogGyro(0);

//     private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);
//     private final DifferentialDrivePoseEstimator differentialEstimator;

//     public AprilTag() {
//         photonCamera = new PhotonCamera("OV5647");
//         photonCamera.setDriverMode(false);
//         photonCamera.setPipelineIndex(0);

//         AprilTagFieldLayout atfl = null;
// 		try {
// 			atfl = AprilTagFieldLayout.loadFromResource(
// 				AprilTagFields.k2023ChargedUp.m_resourceFile);
// 			System.out.println(atfl);
// 		} catch (IOException e) {
// 			System.out.println("error with loading april tag");
// 		}

//         // Transform the robot pose to find the camera's pose
//         robotToCam = new Transform3d(new Translation3d(CAMERA_X_OFFSET, 
//         CAMERA_Y_OFFSET, CAMERA_HEIGHT_METERS), 
//         new Rotation3d(0, 0, 0));
       
//         // Initialize pose estimator with rotation, translation values
//         poseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.LOWEST_AMBIGUITY, photonCamera, robotToCam);
//        // differentialEstimator = new DifferentialDrivePoseEstimator(m_kinematics, poseEstimator., 0.0, 0.0, new Pose2d());
//     }

//     public void updateOdometry() {

//     }

//     // Returns estimated pose of april tag from camera or null if optPose doesn't exist
//     public Pose3d getEstimatedGlobalPose() {
//         //System.out.println("here");

//         photonCamera.setPipelineIndex(0);
//         optPose = poseEstimator.update();
//         if (optPose.isEmpty()) {
//             return null;
//         }
//         return optPose.get().estimatedPose;
//     }

//     // public EstimatedRobotPose update() {
//     //     Optional<EstimatedRobotPose> poses = poseEstimator.update();
//     //     if (poses.isPresent()) {
//     //         EstimatedRobotPose estimatedPose = poses.get();
//     //         return estimatedPose;
//     //         //poseEstimator.addVisionMeasurement()
//     //     } else {
//     //         return null;
//     //     }
       
//     // }

//     // Returns x pose of target from camera, else unreasonable number 
//     // 
//     public double getTagX() {
//         try {
//             photonCamera.setPipelineIndex(0);
//             return getEstimatedGlobalPose().getX();
            
//         } catch (NullPointerException e) {
//             e.printStackTrace();
//         }
//         return 10000;
//     }

//     // Returns y pos of target from camera
//     // public double getTagY() {
//     //     photonCamera.setPipelineIndex(4);
//     //     if (getEstimatedGlobalPose() != null) {
//     //         return getEstimatedGlobalPose().toPose2d().getY();
//     //     }
//     //     return 100000;
//     // }

//     // // Returns angle to turn to target (in degrees)
//     // public double getAngle() {
//     //     photonCamera.setPipelineIndex(4);
//     //     if (getEstimatedGlobalPose() != null) {
//     //         return getEstimatedGlobalPose().toPose2d().getRotation().getDegrees();
//     //     }
//     //     return 100000;
//     // }

//     // NEED TO MODIFY FUNCTION FOR APRIL TAG
//     public int isCentered() {
//         SmartDashboard.putNumber("Pipeline index", photonCamera.getPipelineIndex());
//         photonCamera.setPipelineIndex(0);
//         var result = photonCamera.getLatestResult();
//         if(result.hasTargets()) { //at target = 0, to the left of target = pos, to the right of target = neg
//             TARGET_PITCH_RADIANS = result.getBestTarget().getPitch();
//             boolean centered = false;
//             //SmartDashboard.putNumber("Distance to Tape", getTapeDistance2());
//                 //System.out.println(Units.radiansToDegrees(result.getBestTarget().getYaw()) + ", without conversion ->" + result.getBestTarget().getYaw());
                
//             if(result.getBestTarget() != null && result.getBestTarget().getYaw() >= -CAMERA_YAW_DEGREES && result.getBestTarget().getYaw() <= CAMERA_YAW_DEGREES) {
//                 centered = true;
//                 //System.out.println("centered");
//                 SmartDashboard.putBoolean("Centered or not", centered);
//                 return 0;
//             } else {
//                 //System.out.println("not centered");
//                 if(result.getBestTarget().getYaw()>0){
//                     return 1;
//                 }else{
//                     return -1;
//                 }
//             }
//         } else{
//             System.out.println("no target");
//             return 50;
//         }
//         // SmartDashboard.putNumber("Camera Yaw to target (deg)", result.getBestTarget().getYaw());
//     }

    

// }
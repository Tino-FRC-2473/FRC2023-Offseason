package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {

    private PhotonCamera photonCamera;
    // Angle between horizontal and the camera.
    final double CAMERA_YAW_DEGREES = 3;
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(20.75);
    //camera height is 20.8 inches
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(1.927);
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    final double TARGET_TO_CAMERA = TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS;
    double TARGET_PITCH_RADIANS =  Units.degreesToRadians(0);

    public Limelight() {
        photonCamera = new PhotonCamera("OV5647");
        photonCamera.setDriverMode(false);
    }
    public int isCentered() {
        photonCamera.setPipelineIndex(5);
        var result = photonCamera.getLatestResult();
        if(result.hasTargets()) { //at target = 0, to the left of target = pos, to the right of target = neg
            TARGET_PITCH_RADIANS = result.getBestTarget().getPitch();
            boolean centered = false;
            SmartDashboard.putNumber("Distance to Tape", getTapeDistance2());
            SmartDashboard.putNumber("Target Pitch", photonCamera.getLatestResult().getBestTarget().getPitch());
                //System.out.println(Units.radiansToDegrees(result.getBestTarget().getYaw()) + ", without conversion ->" + result.getBestTarget().getYaw());
            SmartDashboard.putNumber("Actual pitch", Math.atan(TARGET_TO_CAMERA / 0.2925));
            SmartDashboard.putNumber("dist target to camera", TARGET_TO_CAMERA);
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
            SmartDashboard.putBoolean("Centered or not", false);
            System.out.println("no target");
            return 50;
        }
        // SmartDashboard.putNumber("Camera Yaw to target (deg)", result.getBestTarget().getYaw());
    }

    //write a method that calculates the distance to the reflective tape
    public double getTapeDistance() {


        if(photonCamera.getLatestResult()!=null){
            return PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(photonCamera.getLatestResult().getBestTarget().getPitch()));
        }else{
            return -1;
        }

    }
    public double getTapeDistance2() {

        return TARGET_TO_CAMERA/Math.sin(Units.degreesToRadians(photonCamera.getLatestResult().getBestTarget().getPitch()));
    }

    }


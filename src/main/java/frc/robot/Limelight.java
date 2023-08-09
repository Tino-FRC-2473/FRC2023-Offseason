package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.util.Units;

public class Limelight {


    // Angle between horizontal and the camera.
    final double CAMERA_YAW_RADIANS = Units.degreesToRadians(3);


    private PhotonCamera photonCamera;

    public Limelight() {
        photonCamera = new PhotonCamera("OV5647");
        photonCamera.setDriverMode(false);
    }

    public void isCentered() {
        var result = photonCamera.getLatestResult();
        if(result.hasTargets()) {
            if(result.getBestTarget().getYaw() >= CAMERA_YAW_RADIANS && result.getBestTarget().getYaw() <= -CAMERA_YAW_RADIANS) {
                System.out.println("centered");
            } else {
                System.out.println("not centered");
            }
        } else if (!result.hasTargets()) {
            System.out.println("no target");
        }
    }

    //write a method that calculates the distance to the reflective tape
    public double getTapeDistance() {

        return 0.0;
    }
}
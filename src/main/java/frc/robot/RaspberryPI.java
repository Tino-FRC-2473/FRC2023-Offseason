package frc.robot;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RaspberryPI {
	private double fps = 0;
	private NetworkTable table;

	//FPS Calculation
	private DoubleSubscriber fpsCounter;
	private DoubleSubscriber cubeYawSubscriber;
	private DoubleSubscriber cubeDistanceSubscriber;
	private DoubleSubscriber coneYawSubscriber;
	private DoubleSubscriber coneDistanceSubscriber;
	private double lastValue = 0;
	private double lastTime = 0;
	private Timer timer = new Timer();

	/**Updates the FPS each iteration of the robot.*/
	public RaspberryPI() {
		timer.start();
		table = NetworkTableInstance.getDefault().getTable("datatable");
		fpsCounter = table.getDoubleTopic("x").subscribe(-1);
		cubeYawSubscriber = table.getDoubleTopic("cube_yaw").subscribe(-1);
		cubeDistanceSubscriber = table.getDoubleTopic("cube_distance").subscribe(-1);
		coneYawSubscriber = table.getDoubleTopic("cone_yaw").subscribe(-1);
		coneDistanceSubscriber = table.getDoubleTopic("cone_distance").subscribe(-1);
	}

	/**Updates the values in SmartDashboard. */
	public void update() {
		updateFPS();
	}

	/**
	 * Gets the yaw to the cube.
	 * @return returns the horizontal angle between the cube and the camera in degrees.
	 */
	public double getCubeYaw() {
		return cubeYawSubscriber.get();
	}

	/**
	 * Gets the yaw to the cone.
	 * @return returns the horizontal angle between the cone and the camera in degrees.
	 */
	public double getConeYaw() {
		return coneYawSubscriber.get();
	}

	/**
	 * Gets the distance to the cube.
	 * @return returns the distance to the cube in meters.
	 */
	public double getCubeDistance() {
		return cubeDistanceSubscriber.get();
	}

	/**
	 * Gets the distance to the cone.
	 * @return returns the distance to the cone in meters.
	 */
	public double getConeDistance() {
		return coneDistanceSubscriber.get();
	}

	/**
	 * Updates the FPS each iteration of the robot.
	 */
	public void updateFPS() {
		double x = fpsCounter.get();
		if (x != lastValue) {
			fps = 1.0 / (timer.get() - lastTime);
			lastTime = timer.get();
		}
		lastValue = x;
		SmartDashboard.putNumber("FPS", fps);
	}
}

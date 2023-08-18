package frc.robot;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RaspberryPI {
    private double FPS = 0;
    private NetworkTable table;

    //FPS Calculation
	private DoubleSubscriber fpsCounter;
	private double time_interval;
	private double last_value = 0;
	private double last_time = 0;
	private Timer timer = new Timer();

    public RaspberryPI() {
        timer.start();
		table = NetworkTableInstance.getDefault().getTable("datatable");
		fpsCounter = table.getDoubleTopic("x").subscribe(-1);
    }

    public void update() {
        updateFPS();
    }

    public void updateFPS() {
        double x = fpsCounter.get();
		if (x != last_value) {
			FPS = 1.0 / (timer.get() - last_time);
			last_time = timer.get();
		}
		last_value = x;
        SmartDashboard.putNumber("FPS", FPS);
    }

    
}

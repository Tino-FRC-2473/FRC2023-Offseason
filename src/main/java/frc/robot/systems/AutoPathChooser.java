package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.ElevatorArmFSM.FSMState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoPathChooser {
	private static SendableChooser<FSMState> autoPathChooser;
	private static SendableChooser<Integer> nodeChooser;

	/**
	 * Constructor for creating the AutoPathChooser class. It is used in order to select the
	 * auto path and node for the robot without re-deploying.
	 */
	public AutoPathChooser() {
		autoPathChooser = new SendableChooser<>();
		autoPathChooser.setDefaultOption("Path 1", FSMState.IDLE);
		autoPathChooser.addOption("Path 2", FSMState.IDLE);
		autoPathChooser.addOption("Path 3", FSMState.IDLE);
		autoPathChooser.addOption("Path 4", FSMState.IDLE);
		autoPathChooser.addOption("Path 5", FSMState.IDLE);
		autoPathChooser.addOption("Path 6", FSMState.IDLE);
		autoPathChooser.addOption("Path 7", FSMState.IDLE);
		SmartDashboard.putData("Auto Path", autoPathChooser);

		nodeChooser = new SendableChooser<>();
		nodeChooser.setDefaultOption("Low", 0);
		nodeChooser.addOption("Mid", 1);
		nodeChooser.addOption("High", 2);
		nodeChooser.addOption("None", -1);
		SmartDashboard.putData("Node", nodeChooser);
	}

	/**
	 * Returns the sendable chooser object which contains information on the selected auto path.
	 * @return The sendable chooser for the auto path.SendableChooser contains FSMState.
	 */
	public static SendableChooser<FSMState> getAutoPathChooser() {
		return autoPathChooser;
	}

	/**
	 * Returns the sendable chooser object containing information on the selected auto path node.
	 * @return the sendable chooser for the node. SendableChooser contains Integers.
	 */
	public static SendableChooser<Integer> getNodeChooser() {
		return nodeChooser;
	}

	/**
	 * Returs the selected auto path.
	 * @return FSMState object which has the selected auto path.
	 */
	public static FSMState getSelectedPath() {
		return autoPathChooser.getSelected();
	}

	/**
	 * Returns the selected node.
	 * @return int for the selected node. -1 for none, 0 for low, 1 for mid, 2 for high.
	 */
	public static int getSelectedNode() {
		return nodeChooser.getSelected();
	}
}

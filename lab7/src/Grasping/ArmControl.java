package Grasping;

import static java.lang.Math.*;

public class ArmControl implements Runnable {
	// Servos
	//
	public static Servo shoulder = new Servo(400,2350,7,1400,2075,0,Math.PI/2.0,0);
	public static Servo wrist = new Servo(500,2450,10,1750,950,0,Math.PI/2.0,1);
	public static Servo gripper = new Servo(1490,2130,2.0,2130,1490,0,1,2);

	// End effector coordinates
	//
	private double thetaShoulder = 0;
	private double thetaWrist = 0;
	private double deltaX = 0;

	// Arm lengths
	//
	// X Distance [m] between Robot Origin, and Shoulder Pivot
	private static final double shoulderOffset = 0.1;
	// Distance [m] between Shoulder Pivot, and Wrist Pivot
	private static final double shoulderLength = 0.244;
	// Z Distance (height) [m] between Robot Origin, and Shoulder Pivot
	private static final double shoulderHeight = 0.26;
	// Distance [m] between Wrist Pivot, and Gripper Pivot
	private static final double wristLength = 0.065;

	// Minimum, and maximum Z positions that we can set
	private static final double minHeight = shoulderHeight-shoulderLength;
	private static final double maxHeight = shoulderHeight+shoulderLength-0.005;

	// Gripper control
	//
	private double alphaGripper = 0;

	public void setGripperStatus(double alpha) {
		alphaGripper = alpha;
	}

	public ArmControl() {
		g.ac = this;
	}

	public double getDeltaX() {
		return deltaX;
	}

	/**
	 * Fancy inverse kinematics for robot arm with two independent links
	 * We are constraining the wrist to be parallel to the ground.
	 * Sets the desired values for this height.
	 *
	 * @param height height we want the ref point to be from the ground
	 */
	public void setParams(double height) {
		height = max(minHeight,min(maxHeight,height));
		double delta = height-shoulderHeight;
		thetaShoulder = atan2(delta, sqrt(shoulderLength * shoulderLength - delta * delta));
		thetaWrist = -thetaShoulder;
		deltaX = shoulderOffset + shoulderLength*cos(thetaShoulder) + wristLength;
	}

	/**
	 * Puts the arm at the desired height.
	 * It is implemented in the run() method so that it is easily Threadable.
	 * If methods want to block while waiting for this to finish, it synchronizes on this object.
	 */
	public synchronized void run() {
		shoulder.setTargetAngle(thetaShoulder);
		wrist.setTargetAngle(thetaWrist);
		gripper.setTargetAngle(alphaGripper);
		synchronized(shoulder) {
			new Thread(wrist).start();
			new Thread(gripper).start();
			new Thread(shoulder).start();
			try {
				shoulder.wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
				return;
			}
		}
		synchronized(wrist){}
		synchronized(gripper){}
		this.notifyAll();
	}
}

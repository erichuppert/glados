package Grasping;

import static java.lang.Math.*;


public class ArmControl {

	// End effector coordinates
	//
	private static double thetaShoulder = 0;
	private static double thetaWrist = 0;
	private static double deltaX = 0;

	// Arm lengths
	//
	private static final double shoulderOffset = 0.09;	// Distance [m] between Robot Origin and Shoulder Pivot
	private static final double shoulderHeight = 0.125; // Height [m] of the shoulder.
	private static final double shoulder = 0.1; 		// Linkage distance [m] between Shoulder Pivot and Wrist Pivot
	private static final double wrist = 0.065; 		// Linkage distance [m] between Wrist Pivot and Gripper Pivot

	public static double getThetaShoulder(){
		return thetaShoulder;
	}

	public static double getThetaWrist(){
		return thetaWrist;
	}

	public static double getDeltaX(){
		return deltaX;
	}

	public static void setParams(double deltaZ){
		/* Fancy inverse kinematics for robot arm with two independent links (we are constraining the wrist to be parallel
		 * with the ground)
		thetaWrist = atan2(sqrt(1-((deltaX*deltaX+deltaZ*deltaZ-shoulder*shoulder-wrist*wrist)/(2*shoulder*wrist))),
								  ((deltaX*deltaX+deltaZ*deltaZ-shoulder*shoulder-wrist*wrist)/(2*shoulder*wrist)));
		thetaShoulder = atan2(deltaZ, deltaZ) - atan2(wrist*sin(thetaWrist), shoulder + wrist*cos(thetaWrist));
		*/

		// Move it to the shoulder's origin
		//
		double delta = deltaZ-shoulderHeight;
		thetaShoulder = atan2(delta, sqrt(shoulder * shoulder - delta * delta));
		thetaWrist = -thetaShoulder;
		deltaX = shoulderOffset + shoulder*cos(thetaShoulder) + wrist;
	}
}

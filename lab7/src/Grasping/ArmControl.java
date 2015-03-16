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
	private static double shoulderOffset = 0.1;	// Distance [m] between Robot Origin and Shoulder Pivot
	private static double shoulder = 0.244; // Linkage distance [m] between Shoulder Pivot and Wrist Pivot
	private static double shoulderHeight = 0.26; // Height [m] of the shoulder pivot
	private static double wrist = 0.065; // Linkage distance [m] between Wrist Pivot and Gripper Pivot
	private static final double minHeight = shoulderHeight-shoulder;
	private static final double maxHeight = shoulderHeight+shoulder-0.05; // Wrist limits cause the -0.05

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
		/**
		 * Fancy inverse kinematics for robot arm with two independent links (we are constraining the wrist to be parallel
		 * with the ground)
		thetaWrist = atan2(sqrt(1-((deltaX*deltaX+deltaZ*deltaZ-shoulder*shoulder-wrist*wrist)/(2*shoulder*wrist))),
								  ((deltaX*deltaX+deltaZ*deltaZ-shoulder*shoulder-wrist*wrist)/(2*shoulder*wrist)));
		thetaShoulder = atan2(deltaZ, deltaZ) - atan2(wrist*sin(thetaWrist), shoulder + wrist*cos(thetaWrist));
		*/
		deltaZ = max(minHeight,min(maxHeight,deltaZ));
		double delta = deltaZ-shoulderHeight;
		if(Math.abs(delta) > shoulder) {
			delta = g.sign(delta)*shoulder;
		}
		thetaShoulder = atan2(delta, sqrt(shoulder * shoulder - delta * delta));
		thetaWrist = -thetaShoulder;
		deltaX = shoulderOffset + shoulder*cos(thetaShoulder) + wrist;
	}
}

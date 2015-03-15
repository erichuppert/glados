package Chassis;

import static java.lang.Math.*;


public class ArmControl {

	// End effector coordinates
	//
	public static double thetaShoulder = 0;
	public static double thetaWrist = 0;
	public static double deltaX = 0; 
	
	// Arm lengths
	// 
	public static double shoulderOffset = 0.1;	// Distance [m] between Robot Origin and Shoulder Pivot
	public static double shoulder = 0.244; 		// Linkage distance [m] between Shoulder Pivot and Wrist Pivot
	public static double wrist = 0.065; 		// Linkage distance [m] between Wrist Pivot and Gripper Pivot
	
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
		
		thetaShoulder = atan2(deltaZ, sqrt(deltaZ*deltaZ - shoulder*shoulder));
		thetaWrist = thetaShoulder; 
		deltaX = shoulderOffset + shoulder*cos(thetaShoulder) + wrist; 
	}
}
package LocalNavigation;

import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.message.std_msgs.String;
import org.ros.message.rss_msgs.MotionMsg;

public class FSM {
    // Frequency that step should be called
	//
	public static final double FREQ = 20;

	// Possible States
	//
	public static final int STOP_ON_BUMP        = 0;
	public static final int ALIGN_ON_BUMP       = 1;
	public static final int ALIGNING            = 2;
	public static final int ALIGNED             = 3;
	public static final int RETREATING          = 4;
	public static final int ROTATING            = 5;
	public static final int ALIGNED_AND_ROTATED = 6;

	// State descriptions
	//
	private static final java.lang.String[] stateDescriptions = new java.lang.String[] {
		"Initial state: stops when it feels a bump",
		"Initial state: aligns when it feels a bump",
		"Currently aligining the robot",
		"Currently aligned",
		"Moving backwards from obstacle after alignment",
		"Rotating so that sonars face the obstacle"
	};

	// State variable
	//
	private int state;

	// Inputs
	//
	private double[] sonars;
	private double[] pose;
	private boolean[] bumpers;

	// ROS publishers
	//
	private Publisher<org.ros.message.std_msgs.String> statePub; // state
	private Publisher<MotionMsg> motorPub; // motor commands

	// Set the Initial State
	//
	public FSM(Node node, int initialState) {
		statePub = node.newPublisher("/rss/state","std_msgs/String");
		motorPub = node.newPublisher("/command/Motors","rss_msgs/MotionMsg");
		changeState(initialState);
	}

	// Translational/Rotational velocities
	// So we don't have to redeclare them in every method.
	// Set by methods to change motor velocities.
	//
	private boolean setVelocities;
	private double tv,rv;

	// keep track of the location that robot was in when aligned with wall so that we can retreat the right distance
	//
	private double[] alignedPose;

	/**
	 * Take a step in the state machine
	 * @param _sonars:  most recent sonar values
	 * @param _pose:    most recent odometry pose
	 * @param _bumpers: most recent bumper statuses
	 */
	public void step(double[] _sonars, double[] _pose, boolean[] _bumpers) {
		
		// Make inputs available to all methods
		//
		sonars = _sonars;
		pose = _pose;
		bumpers = _bumpers;

		// By default, we don't set velocities
		//
		setVelocities = false;

		// Execute the method for the current state
		//
		switch(state) {
		case STOP_ON_BUMP:
			stop_on_bump();
			break;
		case ALIGN_ON_BUMP:
			align_on_bump();
			break;
		case ALIGNING:
			aligning();
			break;
		case ALIGNED:
			aligned();
			break;
		case RETREATING:
			retreating();
			break;
		case ROTATING:
			rotating();
			break;
		default:
			// INVALID STATE, do nothing
		}

		if (setVelocities) {
			setMotorVelocities(tv,rv);
		}
	}

	// Below are values that have been tuned based on experimentation
	//
	private static float ALIGNMENT_TRANSLATIONAL_SPEED = (float) 0.2;
	private static float ALIGNMENT_ROTATIONAL_SPEED = (float) 0.05;

	// If we see a bump, then stop, otherwise we are controlled externally
	//
	private void stop_on_bump() {
		if (bumpers[g.LEFT] || bumpers[g.RIGHT]) {
			tv = rv = 0;
			setVelocities = true;
		}
	}

	// If we see a bump, start aligning
	//
	private void align_on_bump() {
		if (bumpers[g.LEFT] || bumpers[g.RIGHT]) {
			changeState(ALIGNING);
		}
	}

	// If we see a bump, then rotate to align the other.
	// If both bumpers are pressed, then we're done aligning.
	// Otherwise, look for wall by moving forward.
	//
	private void aligning() {
		setVelocities = true;

		if(bumpers[g.LEFT] && bumpers[g.RIGHT]) {
			// Both bumpers pressed
			//
			changeState(ALIGNED);
			tv = rv = 0;
		} else if (bumpers[g.LEFT] || bumpers[g.RIGHT]) {
			// Either bumper pressed
			// Rotate to align.
			// Slight forward speed to avoid oscillations
			// Happened due to depressed bumper becoming undepressed in pure rotation
			//
			tv = ALIGNMENT_TRANSLATIONAL_SPEED*0.1;
			rv = ALIGNMENT_ROTATIONAL_SPEED * (bumpers[g.LEFT]?1.0:-1.0);
		} else {
			// Neither is depressed, just move slowly forward.
			//
			tv = ALIGNMENT_TRANSLATIONAL_SPEED;
			rv = 0;
		}
	}
	
	private static final float POST_RETREAT_ROTATION_SPEED = 0.4;

	// If we're aligned, we move away from the obstacle
	//
	private void aligned() {
		setVelocities = true;

		alignedPose = pose.clone();
		tv = POST_RETREAT_ROTATION_SPEED;
		rv = 0;
		changeState(RETREATING);
	}

	// Move back from the obstacle after we are aligned
	//
	private void retreating() {
		setVelocities = true;
		if (!retreatedEnough()) {
			tv = -ALIGNMENT_TRANSLATIONAL_SPEED;
			rv = 0;
		} else {
			tv = rv = 0;
			changeState(ROTATING);
		}
	}

	// rotate so that we align sensors with object
	public void rotating() {
		setVelocities = true;
		if (!rotatedEnough()) {
			tv = 0;
			rv = ALIGNMENT_ROTATIONAL_SPEED;
		} else {
			tv = rv = 0;
		}
	}
	
	// determine if the robot has moved pi/2 radians with respect to its pose when aligned at the wall
	//
	public boolean rotatedEnough() {
		return Math.abs(pose[g.THETA] - alignedPose[g.THETA]) > Math.PI/2;
	}

	public static double OBSTACLE_RETREAT_DISTANCE = 0.5;

	// Tell if, based on our current pose, if we have retreated from the wall enough
	//
	private boolean retreatedEnough() {
		double distanceSinceAligned = Math.sqrt(Math.pow(pose[g.X]-alignedPose[g.X],2) + Math.pow(pose[g.Y] - alignedPose[g.Y],2));
		return distanceSinceAligned >= OBSTACLE_RETREAT_DISTANCE;
	}

	// Changes state variable, and publishes it.
	//
	private void changeState(int newState){
		state = newState;
		org.ros.message.std_msgs.String msg = new org.ros.message.std_msgs.String();
		msg.data = stateDescriptions[newState];
		if(statePub != null) {
			statePub.publish(msg);
		}
	}

	// Useful for abstracting setting the motor velocities
	//
	private void setMotorVelocities(double tv, double rv) {
		MotionMsg msg = new MotionMsg();
		msg.translationalVelocity = tv;
		msg.rotationalVelocity = rv;
		if(motorPub != null) {
			motorPub.publish(msg);
		}
	}
}

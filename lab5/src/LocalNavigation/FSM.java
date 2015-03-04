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
	public static final int STOP_ON_BUMP      = 0;
	public static final int ALIGN_ON_BUMP     = 1;
	public static final int ALIGNING          = 2;
	public static final int ALIGNED           = 3;

	// State descriptions
	//
	private static final java.lang.String[] stateDescriptions = new java.lang.String[] {
		"Initial state: stops when it feels a bump",
		"Initial state: aligns when it feels a bump",
		"Currently aligining the robot",
		"Currently aligned"
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

	// If we're aligned, we do nothing
	//
	private void aligned() {}

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

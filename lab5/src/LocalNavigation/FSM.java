package LocalNavigation;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.message.std_msgs.String;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.rss_msgs.OdometryMsg;

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
	public static final int BACKING_UP          = 7;
	public static final int FINDING_WALL        = 8;
	public static final int TRACKING_WALL       = 9;
	public static final int WALL_ENDED          = 10;
	public static final int START_STATE         = 11;
	public static final int DONE                = 12;

	// State descriptions
	//
	private static final java.lang.String[] stateDescriptions = new java.lang.String[] {
		"Initial state: stops when it feels a bump",
		"Initial state: aligns when it feels a bump",
		"Currently aligining the robot",
		"Currently aligned",
		"Moving backwards from obstacle after alignment",
		"Rotating so that sonars face the obstacle",
		"Sonars facing obstacle",
		"Backing up to find the end of the obstacle",
		"Finding the obstacle while moving forward",
		"Tracking the wall with sonars",
		"Found the end of the wall",
		"Initialization State",
		"Found the final state, finished."
	};

	// State variable
	//
	private int state = START_STATE;
	private int initialState;

	// Inputs
	//
	private double[] sonars;
	private double[] pose;
	private boolean[] bumpers;
	

	// SonarPoints to control the linear filter/segments
	//
	SonarPoints sp;

	// ROS publishers
	//
	private Publisher<org.ros.message.std_msgs.String> statePub; // state
	private Publisher<MotionMsg> motorPub; // motor commands
	private Publisher<OdometryMsg> odoPub; // Odometry readjustments

	private static boolean logErrors = true;
	private BufferedWriter errorOutput;

	// Set the Initial State
	//
	public FSM(Node node, int _initialState, SonarPoints _sp) {
		statePub = node.newPublisher("/rss/state","std_msgs/String");
		motorPub = node.newPublisher("/command/Motors","rss_msgs/MotionMsg");
		odoPub = node.newPublisher("/rss/odometry_update", "rss_msgs/OdometryMsg");
		initialState = _initialState;
		sp = _sp;
		if (logErrors) {
			try {
				File logFile = new File("./error-log.txt");
				errorOutput =  new BufferedWriter(new FileWriter(logFile));
			} catch (Exception e) {
				System.err.println(e);
			}
		}
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
	private double[] wallEndPose;

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
		case START_STATE:
			try {
				Thread.sleep(3000);
			} catch(Exception e) {
				return;
			}
			System.out.printf("Resetting\n");
			changeState(initialState);
			resetRobot();
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
		case ALIGNED_AND_ROTATED:
			aligned_and_rotated();
			break;
		case BACKING_UP:
			backing_up();
			break;
		case FINDING_WALL:
			finding_wall();
			break;
		case TRACKING_WALL:
			tracking_wall();
			break;
		case WALL_ENDED:
			wall_ended();
			break;
		default:
			throw new RuntimeException();
		}

		if (setVelocities) {
			setMotorVelocities(tv,rv);
		}
	}

	public void shutdown() {
		// Reset everything so the robot doesn't go haywire
		//
		System.out.printf("Shutting down\n");
		resetRobot();
	}

	// Below are values that have been tuned based on experimentation
	//
	private static float ALIGNMENT_TRANSLATIONAL_SPEED = (float) 0.05;
	private static float ALIGNMENT_ROTATIONAL_SPEED = (float) 0.1;

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
			tv = ALIGNMENT_TRANSLATIONAL_SPEED*0.5;
			double robot_radius = .215;
			rv = ALIGNMENT_ROTATIONAL_SPEED*0.5 * (bumpers[g.LEFT]?1.0:-1.0);
		} else {
			// Neither is depressed, just move slowly forward.
			//
			tv = ALIGNMENT_TRANSLATIONAL_SPEED;
			rv = 0;
		}
	}

	private static final float POST_RETREAT_ROTATION_SPEED = (float) 0.2;

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
	//
	private void rotating() {
		setVelocities = true;
		// based on our original pose at the wall, we rotate until we get pi/2 away from that directional pose
		if (!rotatedEnough()) {
			tv = 0;
			rv = -ALIGNMENT_ROTATIONAL_SPEED;
		} else {
			tv = rv = 0;
			changeState(ALIGNED_AND_ROTATED);
		}
	}

	// what to do once we have our sensors facing the obstacle ready to scan
	//
	private void aligned_and_rotated() {
		changeState(BACKING_UP);
	}

	// when we are backing up and scanning the obstacle, trying to find its end
	//
	private void backing_up() {
		setVelocities = true;
		// as long is the wall is still in view, we back up so that we can find the end of it
		//
		if (haveObstacle()) {
			tv = -ALIGNMENT_TRANSLATIONAL_SPEED;
			rv = 0;
		} else {
			// once both sensors dont see the wall, we start finding the wall
			tv = rv = 0;
			changeState(FINDING_WALL);
		}
	}

	// once we have moved so that both sensors are behind the obstacle, we move forward
	//
	private void finding_wall() {
		setVelocities = true;
		// in this state, we will have backed up and moved behind the wall
		// we need to move forward to bring the wall back into view
		//
		if (!bothHaveObstacle()) {
			tv = ALIGNMENT_TRANSLATIONAL_SPEED;
			rv = 0;
		} else {
			// once in view, start tracking the wall
			tv = rv = 0;
			sp.startTracking();
			changeState(TRACKING_WALL);
		}
	}

	// when we are moving forward and scanning the wall after finding its end previously
	//
	private void tracking_wall() {
		setVelocities = true;
		// when we have an obstacle in sonar view, continue moving forward and tracking it
		//
		System.out.printf("HERE!!!!!");
		if (haveObstacle()) {
			tv = ALIGNMENT_TRANSLATIONAL_SPEED;
			double Kd = 0.125;
			double Ka = 0.1;
			double desired = OBSTACLE_RETREAT_DISTANCE + 0.2;
			try {
				double distanceError = sp.getDistanceError();
				double angleError = sp.getAngleError();				
				double theta_i = Kd*(desired-distanceError);
				rv = -Ka*(theta_i - angleError);
				logError(distanceError, angleError);
			} catch(RuntimeException e) {
				rv = 0;
			}
		} else {
			tv = rv = 0;
			sp.stopTracking();
			changeState(WALL_ENDED);
			wallEndPose = pose.clone();
		}
	}

	// after we have cleared the wall in front (and know the location of the wall)
	//
	private void wall_ended() {
		double d = OBSTACLE_RETREAT_DISTANCE;
		double radius = (140477.0+73000.0*d+500000.0*d*d)/(73000.0+1000000.0*d);  
		setVelocities = true;
		if (sp.obstacleDone()) {
			tv=rv=0;
			changeState(DONE);
		} else {
			tv = ALIGNMENT_TRANSLATIONAL_SPEED;
			rv = ALIGNMENT_TRANSLATIONAL_SPEED/d;

		}
	}

	// Goes into this state when we have found a model of the obstacle
	//
	private void done() {}

	// determines if EITHER sensor is encountering an obstacle, based on the sonar threshold
	//
	private boolean haveObstacle() {
		return SonarPoints.obstacleInRange(sonars[g.BACK]) || SonarPoints.obstacleInRange(sonars[g.FRONT]);
	}
	
	// determines if BOTH sensors are encountering an obstacle based on sonar threshold
	//
	private boolean bothHaveObstacle() {
		return SonarPoints.obstacleInRange(sonars[g.BACK]) && SonarPoints.obstacleInRange(sonars[g.FRONT]);
	}

	// determine if the robot has moved pi/2 radians with respect to its pose when aligned at the wall
	//
	public boolean rotatedEnough() {
		return Math.abs(Math.atan2(Math.sin(pose[g.THETA]-alignedPose[g.THETA]), Math.cos(pose[g.THETA]-alignedPose[g.THETA]))) > Math.PI/2 - 0.2;
	}

	public static double OBSTACLE_RETREAT_DISTANCE = 0.4;

	// Tell if, based on our current pose, if we have retreated from the wall enough
	//
	private boolean retreatedEnough() {
		double distanceSinceAligned = Math.sqrt(Math.pow(pose[g.X]-alignedPose[g.X],2) + Math.pow(pose[g.Y] - alignedPose[g.Y],2));
		return distanceSinceAligned >= OBSTACLE_RETREAT_DISTANCE - 0.1;
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
		// Motor commands do not handle NaN well. We should never get these.
		//
		if (tv == Double.NaN || rv == Double.NaN) {
			throw new RuntimeException("NaN command sent!");
		}

		msg.translationalVelocity = 5* tv;
		msg.rotationalVelocity = rv;
		if(motorPub != null) {
			motorPub.publish(msg);
		}
	}

	private void setRobotPose(double x, double y, double theta) {
		OdometryMsg msg = new OdometryMsg();
		msg.x = x;
		msg.y = y;
		msg.theta = theta;
		if (odoPub != null) {
			odoPub.publish(msg);
		}
	}

	private void resetRobot() {
		setMotorVelocities(0,0);
		setRobotPose(0,0,0);
	}

	/**
	 * Takes the error terms and adds them to the file.
	 * @param translationError amount of translational error
	 * @param angleError amount of angleError
	 */
	private void logError(double translationError, double angleError) {
		if (logErrors) {
			try {
				errorOutput.append(
						Long.toString(System.currentTimeMillis()) + " " +
								Double.toString(OBSTACLE_RETREAT_DISTANCE-translationError) + " " +
								Double.toString(angleError) + "\n");
			} catch (Exception e){
				System.err.println(e);
			}
		}
	}
}

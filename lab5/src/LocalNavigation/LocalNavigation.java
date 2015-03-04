package LocalNavigation;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.Scanner;
import java.awt.Color;//for color from SonarGUI

import org.ros.message.MessageListener;

import org.ros.message.rss_msgs.*;
import org.ros.message.std_msgs.*;
import org.ros.message.lab5_msgs.*;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.namespace.GraphName;

public class LocalNavigation implements NodeMain,Runnable {
	// State machine states, and state variable
	//
	public static final java.lang.String STOP_ON_BUMP      = "Initial state: stops when it feels a bump";
	public static final java.lang.String ALIGN_ON_BUMP     = "Initial state: aligns when it feels a bump";
	public static final java.lang.String ALIGNING          = "Currently aligining the robot";
	public static final java.lang.String ALIGNED           = "Currently aligned";
	private java.lang.String state = "";

	protected boolean firstUpdate = true;

	/* Frames of reference:
	 *
	 * World frame: Relative to the robot's starting position, x front, y left, theta up
	 * Robot frame: Relative to the robot right now, x front, y left, theta up
	 * Sonar frame: Relative to the sonar, x points along the sonar axis, y left, theta up
	 * Odometry frame: frame of reference the odometry module uses, the SonarGUI also uses the same.
	 * Aligned frame: Relative to the robot when it most recently entered the ALIGNED state.
	 */

	// Transforms between robot and world frames
	//
	private Mat robotToWorld;

	// Transforms between sonar and robot frames
	//
	private static final Mat sonarToRobotRot = Mat.rotation(Math.PI / 2);
	private static final Mat sonarFrontToRobot = Mat.mul(Mat.translation(-0.064, 0.19), sonarToRobotRot);
	private static final Mat sonarBackToRobot  = Mat.mul(Mat.translation(-0.3350, 0.2150), sonarToRobotRot);

	// Subscribers
	//
	public Subscriber<SonarMsg> sonarFrontSub; // Sonars
	public Subscriber<SonarMsg> sonarBackSub;
	public Subscriber<BumpMsg> bumpSub; // Bump sensors
	public Subscriber<OdometryMsg> odoSub; // Odometry

	// Message Publishers
	//
	public Publisher<MotionMsg> motorPub; // motors for velocity
	public Publisher<org.ros.message.std_msgs.String> statePub; // Publish state value
	public Publisher<GUILineMsg> linePub; // Publishes the Line to the GUI
	//public Publisher<GUISegmentMsg> segmentPub; // Published Segment to the GUI; Not sure what it does
	public Publisher<GUIPointMsg> pointPub; // Publishes points to the GUI

	//TODO value to be added after testing
	//
	public double threshold = Double.MAX_VALUE;

	private LeastSquareLine lsqWorld;

	// define colors for GUI
	//
	private static int RED=1, BLUE=2, GREEN=3;

	// below are dummy values that will need to be tuned based on experimentation
	//
	private static float ALIGNMENT_TRANSLATIONAL_SPEED = (float) 0.1;
	private static float ALIGNMENT_ROTATIONAL_SPEED = (float) 0.05;

	public LocalNavigation() {
	}

	public void onStart(Node node) {
		// initialize the ROS subscriptions to rss/Sonars
		//
		sonarFrontSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg");
		sonarFrontSub.addMessageListener(new MessageListener<SonarMsg>() {
				@Override
				public void onNewMessage(SonarMsg message) {
					//System.out.printf("Is Front?: %b\tRange: %.3f\n",message.isFront, message.range);
					handleSonar(message);
				}
			});
		sonarBackSub = node.newSubscriber("/rss/Sonars/Back", "rss_msgs/SonarMsg");
		sonarBackSub.addMessageListener(new MessageListener<SonarMsg>() {
				@Override
				public void onNewMessage(SonarMsg message) {
					//System.out.printf("Is Front?: %b\tRange: %.3f\n",message.isFront, message.range);
					handleSonar(message);
				}
			});

		// initialize the ROS subscription to rss/BumpSensors
		//
		bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
		bumpSub.addMessageListener(new MessageListener<BumpMsg>() {
				@Override
				public void onNewMessage(BumpMsg message) {
					//System.out.printf("Left: %b\tRight: %b\n", message.left, message.right);
					//handleBump(message);
				}
			});

		// Initialize the ROS subscription to rss/odometry
		// Don't need it right now
		//
		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		odoSub.addMessageListener(new MessageListener<OdometryMsg>() {
				@Override
				public void onNewMessage(OdometryMsg message) {
					//System.out.printf("X: %.2f\tY: %.2f\ttheta: %.2f\n",message.x,message.y,message.theta);
					handleOdometry(message);
				}
			});

		// Initialize the ROS publication to command/Motors
		//
		motorPub = node.newPublisher("/command/Motors","rss_msgs/MotionMsg");

		// Initialize the ROS publication to graph points
		pointPub = node.newPublisher("/gui/Point","lab5_msgs/GUIPointMsg");
		// Initialize the ROS publication to graph lines
		linePub = node.newPublisher("/gui/Line","lab5_msgs/GUILineMsg");
		lsqWorld = new LeastSquareLine(linePub);
		// // Initialize the ROS publication to graph line segments
		// segmentPub = node.newPublisher("/gui/Segment","lab5_msgs/GUISegmentMsg");

		// initialize the ROS publication to rss/state
		//
		statePub = node.newPublisher("/rss/state","std_msgs/String");
		changeState(ALIGN_ON_BUMP);

		Thread runningStuff = new Thread(this);
		runningStuff.start();
	}

	public void onShutdown(Node node){
		if(node != null){
			node.shutdown();
		}
	}

	/*handle odometry
	 * all position co-ordinates are stored as a matrix of 3*1 [x,y,theta]
	 * all tranforms are done using matrix operations (functions from the Mat Class)
	 */

	public void handleOdometry(OdometryMsg message) {
		robotToWorld = Mat.mul(Mat.translation(message.x, message.y), Mat.rotation(message.theta));
	}

	/**
	 * Processes and prescribes response to a message from the bump sensor
	 * @param message an OdometryMsg containing details about a bump sensor event
	 */
	public void handleBump(BumpMsg message) {
		// Translational, and rotational velocities
		//
		double tv = 0;
		double rv = 0;

		if (state == STOP_ON_BUMP) {
			// If we see a bump, then stop, otherwise we are controlled externally
			//
			if (message.right || message.left) {
				tv = rv = 0;
			} else {
				return;
			}
		} else if (state == ALIGN_ON_BUMP) {
			// Start aligning if a bump is pressed, otherwise we are controlled externally
			//
			if (message.right || message.left) {
				changeState(ALIGNING);
			}
			else {
				return;
			}
		} else if (state == ALIGNING) {
			if (message.right && message.left) {
				// if both sensors are depressed, then we are aligned
				//
				changeState(ALIGNED);
				tv = rv = 0;
			} else if (message.right || message.left) {
				// If one bumper is depressed, then we need to rotate so that they are both depressed
				// based on which bumper is hit, we need to choose the rotation direction
				// We also have a small forward velocity to make sure the bumper that was depressed does not get undepressed
				//
				tv = ALIGNMENT_TRANSLATIONAL_SPEED*0.2;
				int rotationalFactor = (message.left ? 1 : -1);
				rv = rotationalFactor * ALIGNMENT_ROTATIONAL_SPEED;
			} else {
				// If neither is depressed, we move slowly forward.
				//
				rv = 0;
				tv = ALIGNMENT_TRANSLATIONAL_SPEED;
			}
		} else if (state == ALIGNED) {
			tv = rv = 0;
		}
		setMotorVelocities(tv,rv);
	}

	public void handleSonar(SonarMsg message) {
		if (robotToWorld == null) {
			return;
		}
		Mat sonarToRobot = message.isFront ? sonarFrontToRobot : sonarBackToRobot;

		// get the range encoded as a pose vector
		Mat echoSonar = Mat.encodePose(message.range, 0, 0);
		// get the sonar position with respect to the world frame
		Mat echoWorld = Mat.mul(robotToWorld, sonarToRobot, echoSonar);

		double[] echoWorldL = Mat.decodePose(echoWorld);
		Color pointColor = message.range < threshold ? Color.RED : Color.BLUE; 

		if (message.range < threshold) {
			lsqWorld.addPoint(echoWorldL[0], echoWorldL[1]);
			lsqWorld.publishLine();
		}
		int pointShape = message.isFront ? 0 : 1;
		publishPoint((float) echoWorldL[0], (float) echoWorldL[1], pointColor, pointShape);
	}

	public void onShutdownComplete(Node node) {
	}


	public GraphName getDefaultNodeName() {
		return new GraphName("/rss/localnavigation");
	}

	// Abstracts away sending the state message
	//
	private void changeState(java.lang.String newState){
		state = newState;
		org.ros.message.std_msgs.String msg = new org.ros.message.std_msgs.String();
		msg.data = newState;
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

	public static ColorMsg getColorMessage(Color color) {
		ColorMsg message = new ColorMsg();
		message.r = color.getRed();
		message.b = color.getBlue();
		message.g = color.getGreen();
		return message;
	}

	private void publishPoint(float x, float y, Color color, int shape) {
		GUIPointMsg pointPlot = new GUIPointMsg();
		pointPlot.shape = shape;
    	pointPlot.color = getColorMessage(color);
		pointPlot.x = x;
		pointPlot.y = y;
		pointPub.publish(pointPlot);
	}

	public void run() {}
}

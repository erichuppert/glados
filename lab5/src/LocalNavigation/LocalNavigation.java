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

public class LocalNavigation implements NodeMain,Runnable{
	private Node logNode;

	// Sonar GUI
	// Allows us to control the robot, and see the sonars
	//
	private static final boolean RUN_SONAR_GUI = true;
	public SonarGUI gui;

	// State machine states, and state variable
	//
	public static final java.lang.String STOP_ON_BUMP      = "Initial state: stops when it feels a bump";
	public static final java.lang.String ALIGN_ON_BUMP     = "Initial state: aligns when it feels a bump";
	public static final java.lang.String ALIGNING          = "Currently aligining the robot";
	public static final java.lang.String ALIGNED           = "Currently aligned";
	private java.lang.String state = ALIGN_ON_BUMP;

	// Subscribers
	//
	public Subscriber<SonarMsg> sonarFrontSub, sonarBackSub; // Sonars
	public Subscriber<BumpMsg> bumpSub; // Bump sensors
	//public Subscriber<OdometryMsg> odoSub; // Odometry, don't care for now

	// Message Publishers
	//
	public Publisher<MotionMsg> motorPub; // motors for velocity
	public Publisher<org.ros.message.std_msgs.String> statePub; // Publish state value
	//public Publisher<org.ros.message.lab5_msgs.GUILineMsg> linePub; // Don't care for now
	//public Publisher<org.ros.message.lab5_msgs.GUISegmentMsg> segmentPub; // Don't care for now
	//public Publisher<org.ros.message.lab5_msgs.GUIPointMsg> pointPub; // Don't care for now

	// below are dummy values that will need to be tuned based on experimentation
	//
	private static float ALIGNMENT_TRANSLATIONAL_SPEED = (float) 0.1;
	private static float ALIGNMENT_ROTATIONAL_SPEED = (float) 0.05;

	public void onStart(Node node) {

		logNode = node;

		// Sonar GUI - it allows us to control the robot, and see sonar output
		//
		if (RUN_SONAR_GUI) {
			gui.onStart(node);
		}

		// initialize the ROS subscriptions to rss/Sonars
		//
		sonarFrontSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg");
		sonarFrontSub.addMessageListener(new MessageListener<SonarMsg>() {
				@Override
				public void onNewMessage(SonarMsg message) {
					System.out.println(message);
					//handleSonar(message);
				}
			});
		sonarBackSub = node.newSubscriber("/rss/Sonars/Back", "rss_msgs/SonarMsg");
		sonarBackSub.addMessageListener(new MessageListener<SonarMsg>() {
				@Override
				public void onNewMessage(SonarMsg message) {
					System.out.printf("Range: %.3f\n",message.range);
					//handleSonar(message);
				}
			});

		// initialize the ROS subscription to rss/BumpSensors
		//
		bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
		bumpSub.addMessageListener(new MessageListener<BumpMsg>() {
				@Override
				public void onNewMessage(BumpMsg message) {
					System.out.printf("Left: %b\tRight: %b\n", message.left, message.right);
					//handleBump(message);
				}
			});
		// initialize the ROS subscription to rss/odometry
		// Don't need it right now
		//
		// odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		// odoSub.addMessageListener(new MessageListener<OdometryMsg>() {
		// 		@Override
		// 		public void onNewMessage(OdometryMsg message) {
		// 			System.out.println(message);
		// 			//handleOdometry(message);
		// 		}
		// 	});

		// initialize the ROS publication to command/Motors
		//
		motorPub = node.newPublisher("/command/Motors","rss_msgs/MotionMsg");

		// initialize the ROS publication to graph points
		//pointPub = node.newPublisher("/gui/Point","lab5_msgs/GUIPointMsg");
		// initialize the ROS publication to graph lines
		//linePub = node.newPublisher("/gui/Line","lab5_msgs/GUILineMsg");
		// initialize the ROS publication to graph line segments
		//segmentPub = node.newPublisher("/gui/Segment","lab5_msgs/GUISegmentMsg");

		// initialize the ROS publication to rss/state
		//
		statePub = node.newPublisher("/rss/state","std_msgs/String");

		Thread runningStuff = new Thread(this);
		runningStuff.start();
	}

	public void onShutdown(Node node){
		if(node != null){
			node.shutdown();
		}
	}

	/**
	 * Processes and prescribes response to a message from the bump sensor
	 * @param message an OdometryMsg containing details about a bump sensor event
	 */
	public void handleBump(BumpMsg message) {
		MotionMsg motorControlMsg;
		motorControlMsg = new MotionMsg();
		if (state == STOP_ON_BUMP) {
			if (message.right || message.left) {
				motorControlMsg.translationalVelocity = 0;
				motorControlMsg.rotationalVelocity = 0;
			} else {
				motorControlMsg.translationalVelocity = ALIGNMENT_TRANSLATIONAL_SPEED;
				motorControlMsg.rotationalVelocity = 0;
			}
		} else if (state == ALIGN_ON_BUMP || state == ALIGNING) {
			state = ALIGNING;
			// if both sensors are depressed, then we are aligned
			if (message.right && message.left) {
				state = ALIGNED;
			} else if (message.right || message.left) {
				// if one bumper is depressed, then we need to rotate so that they are both depressed
				motorControlMsg.translationalVelocity = 0;
				// based on which bumper is hit, we need to choose the rotation direction
				int rotationalFactor = (message.left ? 1 : -1);
				motorControlMsg.rotationalVelocity = rotationalFactor * ALIGNMENT_ROTATIONAL_SPEED;
				state = ALIGNING;
			} else {
				// if neither is depressed, we need to move forward to make at least one become depressed
				motorControlMsg.translationalVelocity = ALIGNMENT_TRANSLATIONAL_SPEED;
				state = ALIGN_ON_BUMP;
			}
		}
		motorPub.publish(motorControlMsg);
	}

	public void onShutdownComplete(Node node) {
	}


	public GraphName getDefaultNodeName() {
		return new GraphName("rss/localnavigation");
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
		msg.translationalVelocity = rv;
		msg.rotationalVelocity = tv;
		if(motorPub != null) {
			motorPub.publish(msg);
		}
	}

	public void run() {}
}

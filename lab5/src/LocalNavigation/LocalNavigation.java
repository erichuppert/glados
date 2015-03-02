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
	
	private static final boolean RUN_SONAR_GUI = false;
	public SonarGUI gui;



	public static final int STOP_ON_BUMP         =  0;
	public static final int ALIGN_ON_BUMP        =  1;
	public static final int ALIGNING             =  2;
	public static final int ALIGNED              =  3;
	private int state = ALIGN_ON_BUMP;
	
	public Subscriber<org.ros.message.rss_msgs.SonarMsg> sonarFrontSub, sonarBackSub;
	public Subscriber<org.ros.message.rss_msgs.BumpMsg> bumpSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Publisher<org.ros.message.rss_msgs.MotionMsg> motorPub;
	public Publisher<org.ros.message.lab5_msgs.GUIPointMsg> pointPub;
	public Publisher<org.ros.message.std_msgs.String> statePub;
	public Publisher<org.ros.message.lab5_msgs.GUILineMsg> linePub;
	public Publisher<org.ros.message.lab5_msgs.GUISegmentMsg> segmentPub;
	
	// below are dummy values that will need to be tuned based on experimentation
	private static float ALIGNMENT_TRANSLATIONAL_SPEED = (float) 0.1;
	private static float ALIGNMENT_ROTATIONAL_SPEED = (float) 0.1;
 
	public void onStart(Node node) {

		logNode = node;

		
		if (RUN_SONAR_GUI) {
			gui.onStart(node);
		}

		// initialize the ROS subscriptions to rss/Sonars
		sonarFrontSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg");
		sonarFrontSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
				@Override
				public void onNewMessage(org.ros.message.rss_msgs.SonarMsg message) {
					System.out.println(message);
//					handleSonar(message);
				}
			});

		sonarBackSub = node.newSubscriber("/rss/Sonars/Back", "rss_msgs/SonarMsg");
		sonarBackSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
				@Override
				public void onNewMessage(org.ros.message.rss_msgs.SonarMsg message) {
					System.out.println(message);
//					handleSonar(message);
				}
			});

		// initialize the ROS subscription to rss/BumpSensors
		bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
		bumpSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.BumpMsg>() {
				@Override
				public void onNewMessage(org.ros.message.rss_msgs.BumpMsg message) {
					System.out.println(message);
					handleBump(message);
				}
			});
		// initialize the ROS subscription to rss/odometry
		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
				@Override
				public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
					System.out.println(message);
//					handleOdometry(message);
				}
			});

		// initialize the ROS publication to command/Motors
		motorPub = node.newPublisher("/command/Motors","rss_msgs/MotionMsg");
		
		// initialize the ROS publication to graph points
//		pointPub = node.newPublisher("/gui/Point","lab5_msgs/GUIPointMsg");
		// initialize the ROS publication to graph lines
//		linePub = node.newPublisher("/gui/Line","lab5_msgs/GUILineMsg");
		// initialize the ROS publication to graph line segments
//		segmentPub = node.newPublisher("/gui/Segment","lab5_msgs/GUISegmentMsg");

		// initialize the ROS publication to rss/state
		statePub = node.newPublisher("/rss/state","std_msgs/String");
//		org.ros.message.std_msgs.String stateMsg = new org.ros.message.std_msgs.String();
		

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
	public void handleBump(org.ros.message.rss_msgs.BumpMsg message) {
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
				int rotationalFactor = (message.right ? 1 : -1);
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
	
	private void changeState(int newState){
		state = newState;
	}
	
	public void run() {
		
	}
	
}




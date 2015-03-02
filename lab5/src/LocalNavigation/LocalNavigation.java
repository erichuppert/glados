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
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscribe;

public class LocalNavigation implements NodeMain,Runnable{
	private Node logNode;
	
	private static final boolean RUN_SONAR_GUI = false;
	public SonarGUI gui;



	public static final int STOP_ON_BUMP         =  0;
	public static final int ALIGN_ON_BUMP        =  1;
	public static final int ALIGNING             =  2;
	public static final int ALIGNED              =  3;

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
					handleSonar(message);
				}
			});

		sonarBackSub = node.newSubscriber("/rss/Sonars/Back", "rss_msgs/SonarMsg");
		sonarBackSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
				@Override
				public void onNewMessage(org.ros.message.rss_msgs.SonarMsg message) {
					handleSonar(message);
				}
			});

		// initialize the ROS subscription to rss/BumpSensors
		bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
		bumpSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.BumpMsg>() {
				@Override
				public void onNewMessage(org.ros.message.rss_msgs.BumpMsg message) {
					handleBump(message);
				}
			});
		// initialize the ROS subscription to rss/odometry
		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
				@Override
				public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
					handleOdometry(message);
				}
			});

		// initialize the ROS publication to command/Motors
		motorPub = node.newPublisher("/command/Motors","rss_msgs/MotionMsg");
		commandMotors = new MotionMsg();
		
		// initialize the ROS publication to graph points
		pointPub = node.newPublisher("/gui/Point","lab5_msgs/GUIPointMsg");
		pointPlot = new GUIPointMsg();
		pointPlotColor = new ColorMsg();

		// initialize the ROS publication to graph lines
		linePub = node.newPublisher("/gui/Line","lab5_msgs/GUILineMsg");
		linePlot = new GUILineMsg();
		linePlotColor = new ColorMsg();

		// initialize the ROS publication to graph line segments
		segmentPub = node.newPublisher("/gui/Segment","lab5_msgs/GUISegmentMsg");
		segmentPlot = new GUISegmentMsg();
		segmentPlotColor = new ColorMsg();

		// initialize the ROS publication to rss/state
		statePub = node.newPublisher("/rss/state","std_msgs/String");
		stateMsg = new org.ros.message.std_msgs.String();

		Thread runningStuff = new Thread(this);
		runningStuff.start();
	}
	
	public void onShutdown(Node node){
		if(node != null){
			node.shutdown();
		} 
	}

	
	public void onShutdownComplete(Node node) {
	}


	public GraphName getDefaultNodeName() {
		return new GraphName("rss/localnavigation");
	}
	
	private void changeState(int newState){
		state = newState;
	}




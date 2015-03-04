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
	private Node logNode;

	// State machine states, and state variable
	//
	public static final java.lang.String STOP_ON_BUMP      = "Initial state: stops when it feels a bump";
	public static final java.lang.String ALIGN_ON_BUMP     = "Initial state: aligns when it feels a bump";
	public static final java.lang.String ALIGNING          = "Currently aligining the robot";
	public static final java.lang.String ALIGNED           = "Currently aligned";
	private java.lang.String state = ALIGN_ON_BUMP;

	protected boolean firstUpdate = true;

	/* Frames of reference:
	 *
	 * World frame: Relative to the robot's starting position, x front, y left, theta up
	 * Robot frame: Relative to the robot right now, x front, y left, theta up
	 * Sonar frame: Relative to the sonar, x points along the sonar axis, y left, theta up
	 * Odometry frame: frame of reference the odometry module uses, the SonarGUI also uses the same.
	 * Aligned frame: Relative to the robot when it most recently entered the ALIGNED state.
	 */

	// x, y, and theta record the robot's current position in the world frame
	private double x;
	private double y;
	private double theta;

	// Transforms between odometry and world frames
	private Mat odoToWorld; // initialize in handleOdometry
	private Mat worldToOdo; // initialize in handleOdometry

	// Transforms between robot and world frames
	private Mat robotToWorld; // continuously update in handleOdometry

	// Transforms between sonar and robot frames
	private static final Mat sonarToRobotRot = Mat.rotation(Math.PI / 2);
	private static final Mat sonarFrontToRobot = Mat.mul(Mat.translation(-0.64, 0.19), sonarToRobotRot);
	private static final Mat sonarBackToRobot  = Mat.mul(Mat.translation(-0.3350, 0.2150), sonarToRobotRot);

	// Transforms between aligned and world frames
	private Mat alignedToWorld; // update when entering the ALIGNED state
	private Mat worldToAligned; // update when entering the ALIGNED state

	private boolean publishTheLine = true;

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
	public Publisher<org.ros.message.lab5_msgs.GUILineMsg> linePub; // Publishes the Line to the GUI
	private GUILineMsg linePlot;
	private ColorMsg linePlotColor;
	public Publisher<org.ros.message.lab5_msgs.GUISegmentMsg> segmentPub; // Published Segment to the GUI; Not sure what it does
	private GUISegmentMsg segmentPlot;
	private ColorMsg segmentPlotColor;
	public Publisher<org.ros.message.lab5_msgs.GUIPointMsg> pointPub; // Publishes points to the GUI
	private GUIPointMsg pointPlot;
	private ColorMsg pointPlotColor;

	public boolean obstacleDetected = false;
	public double threshold = Double.MAX_VALUE; //TODO value to be added after testing
	
	private LeastSquareLine lsqWorld;
	private LeastSquareLine lsqOdo;

	// below are dummy values that will need to be tuned based on experimentation
	//
	private static float ALIGNMENT_TRANSLATIONAL_SPEED = (float) 0.1;
	private static float ALIGNMENT_ROTATIONAL_SPEED = (float) 0.05;

	public LocalNavigation() {
		//setInitialParams();

		lsqWorld = new LeastSquareLine();
		lsqOdo = new LeastSquareLine();
	}

	public void onStart(Node node) {

		logNode = node;

		// initialize the ROS subscriptions to rss/Sonars
		//
		sonarFrontSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg");
		sonarFrontSub.addMessageListener(new MessageListener<SonarMsg>() {
				@Override
				public void onNewMessage(SonarMsg message) {
					System.out.println(message);
					handleSonar(message);
				}
			});
		sonarBackSub = node.newSubscriber("/rss/Sonars/Back", "rss_msgs/SonarMsg");
		sonarBackSub.addMessageListener(new MessageListener<SonarMsg>() {
				@Override
				public void onNewMessage(SonarMsg message) {
					//System.out.printf("Is Front?: %b\tRange: %.3f\n",message.isFront, message.range);
					//handleSonar(message);
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
					//handleOdometry(message);
				}
			});

		// Initialize the ROS publication to command/Motors
		//
		motorPub = node.newPublisher("/command/Motors","rss_msgs/MotionMsg");

		// Initialize the ROS publication to graph points
		pointPub = node.newPublisher("/gui/Point","lab5_msgs/GUIPointMsg");
		// Initialize the ROS publication to graph lines
		linePub = node.newPublisher("/gui/Line","lab5_msgs/GUILineMsg");
		// Initialize the ROS publication to graph line segments
		segmentPub = node.newPublisher("/gui/Segment","lab5_msgs/GUISegmentMsg");

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

	/*handle odometry
	 * all position co-ordinates are stored as a matrix of 3*1 [x,y,theta]
	 *all tranforms are done using matrix operations (functions from the Mat Class)
	 */

	public void handleOdometry(OdometryMsg message) {
		if ( firstUpdate ) {
			odoToWorld = Mat.mul(Mat.rotation(-message.theta), Mat.translation(-message.x, -message.y));
			worldToOdo = Mat.inverse(odoToWorld);

			firstUpdate = false;
		}

		double[] robotPose = Mat.decodePose(Mat.mul(odoToWorld, Mat.encodePose(message.x, message.y, message.theta)));

		x     = robotPose[0];
		y     = robotPose[1];
		theta = robotPose[2];

		robotToWorld = Mat.mul(Mat.translation(x, y), Mat.rotation(theta));

		//logNode.getLog().info("Odometry raw: " + message.x + " " + message.y + " " + message.theta +
		//                    "\nOdemetry processed: " +         x + " " +         y + " " +         theta);
		//motorUpdate();
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
		}
		else if (state == ALIGNING) {
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
				tv = ALIGNMENT_TRANSLATIONAL_SPEED;
				int rotationalFactor = (message.left ? 1 : -1);
				rv = rotationalFactor * ALIGNMENT_ROTATIONAL_SPEED;
			} else {
				// If neither is depressed, we move slowly forward.
				//
				rv = 0;
				tv = ALIGNMENT_TRANSLATIONAL_SPEED;
			}
		}
		setMotorVelocities(tv,rv);
	}

	public void handleSonar(SonarMsg message) {
		java.lang.String sensor = new java.lang.String();

		Mat sonarToRobot;

		if (message.isFront) {
			sensor = "Front";
			sonarToRobot = sonarFrontToRobot;
			pointPlot.shape = 0;
		} else {
			sensor = "Back";
			sonarToRobot = sonarBackToRobot;
			pointPlot.shape = 1;
		}

		if (!firstUpdate) {
			// get the range encoded as a pose vector
			Mat echoSonar = Mat.encodePose(message.range, 0, 0);
			// get the sonar position with respect to the world frame
			Mat echoWorld = Mat.mul(robotToWorld, sonarToRobot, echoSonar);
			Mat echoOdo = Mat.mul(worldToOdo, echoWorld);

			double[] echoWorldL = Mat.decodePose(echoWorld);
			double[] echoOdoL = Mat.decodePose(echoOdo);

			if (message.range < threshold) {
            	obstacleDetected = true;

            	pointPlotColor.r = 255;
				pointPlotColor.g = 0;
				pointPlotColor.b = 0;

				lsqWorld.addPoint(echoWorldL[0], echoWorldL[1]);
				lsqOdo.addPoint(echoOdoL[0], echoOdoL[1]);
				double[] line = lsqOdo.getLine();
				if (line.length > 0) {
					linePlot.lineA = line[0];
					linePlot.lineB = line[1];
					linePlot.lineC = line[2];
					linePlotColor.r = 0;
					linePlotColor.g = 150;
					linePlotColor.b = 0;
					linePlot.color = linePlotColor;
					if (publishTheLine) {
						linePub.publish(linePlot);
					}
				}

				if (message.isFront){
                	frontSonarDist = message.range;
            	} else {
                	backSonarDist = message.range;
            	}
        	} else {
            	obstacleDetected = false;

            	pointPlotColor.r = 0;
				pointPlotColor.g = 0;
				pointPlotColor.b = 255;

        	}
        	pointPlot.color = pointPlotColor;
			pointPlot.x = echoOdoL[0];
			pointPlot.y = echoOdoL[1];
			pointPub.publish(pointPlot);

			//logNode.getLog().info("SONAR: Sensor: " + sensor + " Range: " + message.range);
		}
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
		msg.translationalVelocity = rv;
		msg.rotationalVelocity = tv;
		if(motorPub != null) {
			motorPub.publish(msg);
		}
	}

	public void run() {}
}

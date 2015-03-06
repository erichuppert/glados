package LocalNavigation;

import java.util.Random;
import java.awt.Color;

import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.message.lab5_msgs.GUIPointMsg;
import org.ros.message.lab5_msgs.GUILineMsg;
import org.ros.message.lab5_msgs.GUISegmentMsg;
import org.ros.message.lab5_msgs.ColorMsg;

public class SonarPoints {
	// Segment building helpers
	//
	private double[] first;
	private double[] mostRecent;
	private Random rand = new Random();
	private double[] previousVector;
	private double outerAngle = 0;
	private double angleThreshold = 0.01;

	// Obstacle tracking must be explicit
	//
	private boolean tracking = false;

	/* Transformation Matrices: used to find the point the sonar is seeing.
	 *
	 * Frames of reference:
	 * World frame: Relative to the robot's starting position, x front, y left, theta up
	 * Robot frame: Relative to the robot right now, x front, y left, theta up
	 * Sonar frame: Relative to the sonar, x points along the sonar axis, y left, theta up
	 */
	// Robot's pose
	//
	private double[] robotPose;
	private Mat robot;

	// Transforms between sonar and robot frames
	//
	private static final Mat sonarToRobotRot   = Mat.rotation(Math.PI / 2);
	private static final Mat sonarFrontToRobot = Mat.mul(Mat.translation(-0.064, 0.19), sonarToRobotRot);
	private static final Mat sonarBackToRobot  = Mat.mul(Mat.translation(-0.3350, 0.2150), sonarToRobotRot);

	// ROS Publishers
	//
	private Publisher<GUIPointMsg> pointPub; // point
	private Publisher<GUILineMsg> linePub; // line
	private Publisher<GUISegmentMsg> segmentPub; // segment

	// Threshold for finding obstacles.
	// Values decided on based on parameters of the problem.
	//
	private static final double threshold_high = 0.5;
	private static final double threshold_low = 0.1;
	private static final double threshold_highest = 1.8;

	public static boolean obstacleInRange(double range) { // Utility method
		return range <= threshold_high && range >= threshold_low;
	}
	private boolean trackingObstacle(double range) { // Uses tracking variable
		return tracking && obstacleInRange(range);
	}

	// Linear Filter.
	// Also publishes the line.
	//
	private LeastSquareLine lineFilter;

	public SonarPoints(Node node) {
		// Initialize publishers
		//
		pointPub = node.newPublisher("/gui/Point","lab5_msgs/GUIPointMsg"); // points
		linePub = node.newPublisher("/gui/Line","lab5_msgs/GUILineMsg"); // line
		segmentPub = node.newPublisher("/gui/Segment","lab5_msgs/GUISegmentMsg");

		// Initialize the linear filter.
		// It takes the line publisher so it can publish the line
		//
		lineFilter = new LeastSquareLine(linePub);
	}

	/**
	 * Updates the robot pose matrix
	 * @param pose: the current pose array
	 */
	public synchronized void updateRobotPose(double[] pose) {
		robotPose = pose.clone();
		robot = Mat.mul(Mat.translation(pose[g.X], pose[g.Y]), Mat.rotation(pose[g.THETA]));
	}

	/**
	 * Builds a color message from a Color object
	 * @param color: Color object
	 */
	public static ColorMsg getColorMessage(Color color) {
		ColorMsg msg = new ColorMsg();
		msg.r = color.getRed();
		msg.b = color.getBlue();
		msg.g = color.getGreen();
		return msg;
	}

	/**
	 * Publishes a point to the GUI
	 * @param pose:  pose array of the point
	 * @param color: color of the point
	 * @param shape: shape of the point
	 */
	private void publishPoint(double[] pose, Color color, int shape) {
		GUIPointMsg msg = new GUIPointMsg();
		msg.shape = shape;
    	msg.color = getColorMessage(color);
		msg.x = pose[g.X];
		msg.y = pose[g.Y];
		pointPub.publish(msg);
	}

	/**
	 * Adds a point, and publishes it.
	 * @param front: is this the front sonar?
	 * @param range: distance returned by the sonar
	 */
	public synchronized void newPoint(boolean front, double range) {
		// Can only add a point if we have a verified pose.
		// And if it's a valid range.
		//
		if (robot == null || range > threshold_highest || range < threshold_low) {
			return;
		}

		// Choose the right sonar transformation matrix
		//
		Mat sonarToRobot = front ? sonarFrontToRobot : sonarBackToRobot;

		// Get the position of the point relative to the sonar
		// and use it to get the point in the real world
		//
		Mat sonarTranslation = Mat.encodePose(range, 0, 0);
		Mat poseMat = Mat.mul(robot, sonarToRobot, sonarTranslation);

		// Put it in a pose array
		//
		double[] pose = Mat.decodePose(poseMat);

		// Color of the point depends on whether it belongs to an obstacle or not.
		//
		Color pointColor = trackingObstacle(range) ? Color.RED : Color.BLUE;

		// If we are seeing an obstacle, add it to the line points.
		//
		if (trackingObstacle(range)) {
			if (first == null) {
				first = pose.clone();
			}
			mostRecent = pose.clone();

			lineFilter.addPoint(pose[g.X], pose[g.Y]);
			lineFilter.publishLine();
		}

		int pointShape = front?0:1;
		publishPoint(pose, pointColor, pointShape);
	}

	// Need it to generate new colors for the line segments
	// From: http://stackoverflow.com/questions/4246351/creating-random-colour-in-java
	// Darker color so that it's more visible in the GUI
	//
	private Color randomColor() {
		float r = rand.nextFloat();
		float g = rand.nextFloat();
		float b = rand.nextFloat();
		Color rndC = new Color(r,g,b);
		return rndC.darker();
	}

	/**
	 * Starts tracking a new line
	 * Resets the line, and the segment
	 */
	public synchronized void startTracking() {
		tracking = true;
		first = null;
		lineFilter.reset();
		mostRecent = null;
	}

	/**
	 * Builds the segment, and publishes it with a random color
	 * Finishes tracking an obstacle
	 */
	public synchronized void stopTracking() {
		GUISegmentMsg msg = new GUISegmentMsg();

		// If we built any sort of segment, then we publish it
		//
		if (first != null) {
			double[] start = lineFilter.getNearestPoint(first);
			double[] end = lineFilter.getNearestPoint(mostRecent);
			msg.startX = start[g.X];
			msg.startY = start[g.Y];
			msg.endX = end[g.X];
			msg.endY = end[g.Y];
			msg.color = getColorMessage(randomColor());
			segmentPub.publish(msg);

			// Add to the max angle
			//
			double [] segVector = new double[] {end[g.X]-start[g.X],end[g.Y]-start[g.Y]};
			double length = Math.pow(segVector[g.X]*segVector[g.X] + segVector[g.Y]*segVector[g.Y],0.5);
			segVector[g.X] /= length;
			segVector[g.Y] /= length;
			if (previousVector != null) {
				double dot = segVector[g.X]*previousVector[g.X]+segVector[g.Y]*previousVector[g.Y];
				outerAngle += Math.acos(dot);
			}
			previousVector = segVector.clone();
		}

		tracking = false;
	}

	public synchronized double getDistanceError() {
		if(robotPose == null) {
			return 0;
		} else {
			return lineFilter.getDistance(robotPose[g.X], robotPose[g.Y]);
		}
	}

	public synchronized double getAngleError() {
		if(robotPose == null) {
			return 0;
		} else {
			return lineFilter.getAngleToLine(robotPose[g.THETA]);
		}
	}

	public synchronized boolean obstacleDone() {
		return Math.abs(outerAngle - 2*Math.PI) <= angleThreshold;
	}
}

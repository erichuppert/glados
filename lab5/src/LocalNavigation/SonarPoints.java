package LocalNavigation;

import java.awt.Color;

import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.message.lab5_msgs.GUIPointMsg;
import org.ros.message.lab5_msgs.GUILineMsg;
// TODO: import org.ros.message.lab5_msgs.GUIPointMsg;
import org.ros.message.lab5_msgs.ColorMsg;

public class SonarPoints {
	/* Transformation Matrices: used to find the point the sonar is seeing.
	 *
	 * Frames of reference:
	 * World frame: Relative to the robot's starting position, x front, y left, theta up
	 * Robot frame: Relative to the robot right now, x front, y left, theta up
	 * Sonar frame: Relative to the sonar, x points along the sonar axis, y left, theta up
	 */
	//

	// Robot's pose
	//
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
	// TODO: public Publisher<GUISegmentMsg> segmentPub; // segment

	// Threshold for finding obstacles.
	// Values decided on based on parameters of the problem.
	//
	private static final double threshold_high = 0.6;
	private static final double threshold_low = 0.1;
	private static boolean obstacleInRange(double range) { // Utility method
		return range <= threshold_high && range >= threshold_low;
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
		// TODO: segmentPub = node.newPublisher("/gui/Segment","lab5_msgs/GUISegmentMsg");

		// Initialize the linear filter.
		// It takes the line publisher so it can publish the line
		//
		lineFilter = new LeastSquareLine(linePub);
	}

	/**
	 * Updates the robot pose matrix
	 * @param pose: the current pose array
	 */
	public void updateRobotPose(double[] pose) {
		robotToWorld = Mat.mul(Mat.translation(pose[g.X], pose[g.Y]), Mat.rotation(pose[g.THETA]));
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
		return message;
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
	public void newPoint(boolean front, double range) {
		// Can only add a point if we have a verified pose.
		//
		if (robotToWorld == null) {
			return;
		}

		// Choose the right sonar transformation matrix
		//
		Mat sonarToRobot = front ? sonarFrontToRobot : sonarBackToRobot;

		// Get the position of the point relative to the sonar
		// and use it to get the point in the real world
		//
		Mat sonarTranslation = Mat.encodePose(message.range, 0, 0);
		Mat poseMat = Mat.mul(robotToWorld, sonarToRobot, sonarTranslation);

		// Put it in a pose array
		//
		double[] pose = Mat.decodePose(pointMat);

		// Color of the point depends on whether it belongs to an obstacle or not.
		//
		Color pointColor = obstacleInRange(message) ? Color.RED : Color.BLUE;

		// If we are seeing an obstacle, add it to the line points.
		//
		if (obstacleInRange(msg)) {
			//lineFilter.addPoint(pose[g.X], pose[g.Y]);
			//lineFilter.publishLine();
		}

		int pointShape = front?0:1;
		publishPoint(pose, pointColor, pointShape);
	}
}

package Grasping;

import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.message.std_msgs.String;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.rss_msgs.OdometryMsg;

// Useful methods, and values that many classes might need to access.
//

public class g {
	// Indices for sonars, poses, and bumpers
	//
	public static final int FRONT = 0;
	public static final int BACK = 1;
	public static final int X = 0;
	public static final int Y = 1;
	public static final int THETA = 2;
	public static final int LEFT = 0;
	public static final int RIGHT = 1;

	public static g pubs = null;

	/**
	 * Asserts the truth of an expression.
	 * Throws a runtimeExpression with an empty message if that fails.
	 * @param mustBeTrue expression to assert.
	 * @throws RuntimeException if expression is false.
	 */
	public static void assertTrue(boolean mustBeTrue) {
		assertTrue(null, mustBeTrue);
	}

	/**
	 * Asserts the truth of an expression.
	 * Throws a runtimeExpression with the passed message if it fails.
	 * @param msg error message if assert fails.
	 * @param mustBeTrue expression to assert.
	 * @throws RuntimeException if expression is false.
	 */
	public static void assertTrue(String msg, boolean mustBeTrue) {
		if(!mustBeTrue) {
			throw new RuntimeException(msg);
		}
	}

	private Publisher<org.ros.message.rss_msgs.String> statePub;
	private Publisher<MotionMsg> motionPub;
	private Publisher<OdometryMsg> odometryPub;

	public g(Node node) {
		statePub = node.newPublisher("/rss/state","std_msgs/String");
		motionPub = node.newPublisher("/command/Motors","rss_msgs/MotionMsg");
		odometryPub = node.newPublisher("/rss/odometry_update", "rss_msgs/OdometryMsg");
		pubs = this;
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
		if(motionPub != null) {
			motionPub.publish(msg);
		}
	}

	private void setRobotPose(double x, double y, double theta) {
		OdometryMsg msg = new OdometryMsg();
		msg.x = x;
		msg.y = y;
		msg.theta = theta;
		if (odometryPub != null) {
			odometryPub.publish(msg);
		}
	}

	private void resetRobot() {
		setMotorVelocities(0,0);
		setRobotPose(0,0,0);
	}

	public void setState(java.util.String state) {
		org.ros.message.std_msgs.String msg = new org.ros.message.std_msgs.String();
		msg.data = state;
		if(statePub != null) {
			statePub.publish(msg);
		}
	}
}

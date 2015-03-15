package Grasping;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.message.std_msgs.String;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.rss_msgs.OdometryMsg;

class Publishers {
	private Publisher<org.ros.message.rss_msgs.String> statePub;
	private Publisher<MotionMsg> motionPub;
	private Publisher<OdometryMsg> odometryPub;

	public Publishers(Node node) {
		statePub = node.newPublisher("/rss/state","std_msgs/String");
		motionPub = node.newPublisher("/command/Motors","rss_msgs/MotionMsg");
		odometryPub = node.newPublisher("/rss/odometry_update", "rss_msgs/OdometryMsg");
		g.pubs = this;
	}

	// Useful for abstracting setting the motor velocities
	//
	public void setMotorVelocities(double tv, double rv) {
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

	public void setRobotPose(double x, double y, double theta) {
		OdometryMsg msg = new OdometryMsg();
		msg.x = x;
		msg.y = y;
		msg.theta = theta;
		if (odometryPub != null) {
			odometryPub.publish(msg);
		}
	}

	public void resetRobot() {
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

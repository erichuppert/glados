package Grasping;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.message.std_msgs.String;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.message.rss_msgs.ArmMsg;

class Publishers {
	private Publisher<org.ros.message.std_msgs.String> statePub;
	private Publisher<MotionMsg> motionPub;
	private Publisher<OdometryMsg> odometryPub;
	private Publisher<ArmMsg> armPub;

	public Publishers(Node node) {
		statePub = node.newPublisher("/rss/state","std_msgs/String");
		motionPub = node.newPublisher("/command/Motors","rss_msgs/MotionMsg");
		odometryPub = node.newPublisher("/rss/odometry_update", "rss_msgs/OdometryMsg");
		armPub = node.newPublisher("/command/Arm", "rss_msgs/ArmMsg");
		g.pubs = this;
	}

	// Useful for abstracting setting the motor velocities
	//
	public void setMotorVelocities(double tv, double rv) {
		MotionMsg msg = new MotionMsg();
		// Motor commands do not handle NaN well. We should never get these.
		//
		Double.assertTrue("NaN command sent!", tv != Double.NaN && rv != Double.NaN);

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

	public void setState(java.lang.String state) {
		org.ros.message.std_msgs.String msg = new org.ros.message.std_msgs.String();
		msg.data = state;
		if(statePub != null) {
			statePub.publish(msg);
		}
	}

	public void setArmPWMs(int index, double value) {
		g.assertTrue("NaN command sent to arms!", value >= 0 && value != Double.NaN);
		double[] previousArm = g.getArm();
		double shoulder = outIndex!=g.SHOULDER?previousArm[g.SHOULDER]:value;
		double wrist = outIndex!=g.WRIST?previousArm[g.WRIST]:value;
		double gripper = outIndex!=g.GRIPPER?previousArm[g.GRIPPER]:value;

		ArmMsg msg = new ArmMsg();
		msg.pwms = new double[]{shoulder,wrist,gripper,0,0,0,0,0};
		if (armPub != null) {
			armPub.publish(msg);
		}
	}
}
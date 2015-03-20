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
    private Publisher<org.ros.message.sensor_msgs.Image> debugImagePub;

	public Publishers(Node node) {
		statePub = node.newPublisher("/rss/state","std_msgs/String");
		motionPub = node.newPublisher("/command/Motors","rss_msgs/MotionMsg");
		odometryPub = node.newPublisher("/rss/odometry_update", "rss_msgs/OdometryMsg");
		armPub = node.newPublisher("/command/Arm", "rss_msgs/ArmMsg");
		debugImagePub = node.newPublisher("/rss/blobVideo", "sensor_msgs/Image");
		g.pubs = this;
	}

	// Useful for abstracting setting the motor velocities
	//
	public synchronized void setMotorVelocities(double tv, double rv) {
		MotionMsg msg = new MotionMsg();
		// Motor commands do not handle NaN well. We should never get these.
		//
		g.assertTrue("NaN command sent!", tv != Double.NaN && rv != Double.NaN);
		tv = Math.min(Math.max(-0.4,tv),0.4);
		rv = Math.min(Math.max(-0.4,rv),0.4);

		msg.translationalVelocity = 5* tv;
		msg.rotationalVelocity = rv;
		if(motionPub != null) {
			motionPub.publish(msg);
		}
	}

	public synchronized void setRobotPose(double x, double y, double theta) {
		OdometryMsg msg = new OdometryMsg();
		msg.x = x;
		msg.y = y;
		msg.theta = theta;
		if (odometryPub != null) {
			odometryPub.publish(msg);
		}
	}

	public synchronized void resetRobot() {
		setMotorVelocities(0,0);
		setRobotPose(0,0,0);
	}

	public synchronized void setState(java.lang.String state) {
		org.ros.message.std_msgs.String msg = new org.ros.message.std_msgs.String();
		msg.data = state;
		if(statePub != null) {
			statePub.publish(msg);
		}
	}

	private long[] pwms = {0,0,0};
	public synchronized void setArm(int index, long value) {
		long shoulder = index!=g.SHOULDER?pwms[g.SHOULDER]:value;
		long wrist = index!=g.WRIST?pwms[g.WRIST]:value;
		long gripper = index!=g.GRIPPER?pwms[g.GRIPPER]:value;
		pwms[g.SHOULDER] = shoulder;
		pwms[g.WRIST] = wrist;
		pwms[g.GRIPPER] = gripper;

		ArmMsg msg = new ArmMsg();
		msg.pwms = new long[]{shoulder,wrist,gripper,0,0,0,0,0};
		if (armPub != null) {
			armPub.publish(msg);
		}
	}

	public synchronized void setDebugImage(Image image) {
		org.ros.message.sensor_msgs.Image pubImage =
			new org.ros.message.sensor_msgs.Image();
		pubImage.width = image.getWidth();
		pubImage.height = image.getHeight();
		pubImage.encoding = "rgb8";
		pubImage.is_bigendian = 0;
		pubImage.step = image.getWidth()*3;
		pubImage.data = image.toArray();
		debugImagePub.publish(pubImage);
	}
}

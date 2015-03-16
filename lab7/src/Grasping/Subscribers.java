package Grasping;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.SonarMsg;
import org.ros.message.rss_msgs.BumpMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.message.rss_msgs.ArmMsg;
import org.ros.node.topic.Subscriber;
import org.ros.node.Node;

public class Subscribers {
	private Subscriber<SonarMsg>							sonarFrontSub;	// Sonars
	private Subscriber<SonarMsg>							sonarBackSub;
	private Subscriber<BumpMsg>								bumpSub;		// Bumps
	private Subscriber<OdometryMsg>							odoSub;			// Odometry
	private Subscriber<org.ros.message.sensor_msgs.Image>	cameraSub;		// Camera
	private Subscriber<ArmMsg>                              armSub;         // Arm
	private Subscriber<org.ros.message.std_msgs.Integer>    userSub;        // User

	private enum ListenerType {
		SONAR,BUMP,ODO,CAMERA,ARM,USER
	}

	private class Listener<T> implements MessageListener<T> {
		private ListenerType type;
		public Listener(ListenerType _type) {
			type = _type;
		}

		public void onNewMessage(T m) {
			switch(type) {
			case SONAR: g.setSonars((SonarMsg)m); break;
			case BUMP: g.setBumps((BumpMsg)m); break;
			case ODO: g.setPose((OdometryMsg)m); break;
			case CAMERA: g.setCamera((org.ros.message.sensor_msgs.Image)m); break;
			case ARM: g.setArm((ArmMsg)m); break;
			case USER: g.setUser((org.ros.message.std_msgs.Integer)m); break;
			default: g.assertTrue("Invalid Listener Type", false);
			}
		}
	}

	public Subscribers(Node node) {
		// Initialize subscriptions
		//
		sonarFrontSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg"); // front sonar
		sonarBackSub = node.newSubscriber("/rss/Sonars/Back", "rss_msgs/SonarMsg");   // back sonar
		bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");         // bumps
		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");         // odometry
		//cameraSub = node.newSubscriber("/rss/video", "sensor_msgs/Image"); // Video
		armSub = node.newSubscriber("/rss/ArmStatus", "rss_msgs/ArmMsg"); // Arm
		userSub = node.newSubscriber("/rss/User", "std_msgs/Integer"); // User

		// Add message listeners to subscribers
		//
		sonarFrontSub.addMessageListener(new Listener<SonarMsg>(ListenerType.SONAR));
		sonarBackSub.addMessageListener(new Listener<SonarMsg>(ListenerType.SONAR));
		bumpSub.addMessageListener(new Listener<BumpMsg>(ListenerType.BUMP));
		odoSub.addMessageListener(new Listener<OdometryMsg>(ListenerType.ODO));
		//cameraSub.addMessageListener(new Listener<org.ros.message.sensor_msgs.Image>(ListenerType.CAMERA));
		armSub.addMessageListener(new Listener<ArmMsg>(ListenerType.ARM));
		userSub.addMessageListener(new Listener<org.ros.message.std_msgs.Integer>(ListenerType.USER));
	}
}

package Grasping;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.SonarMsg;
import org.ros.message.rss_msgs.BumpMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.node.topic.Subscriber;
import org.ros.node.Node;

public class Subscribers {
	private Subscriber<SonarMsg>							sonarFrontSub;	// Sonars
	private Subscriber<SonarMsg>							sonarBackSub;
	private Subscriber<BumpMsg>								bumpSub;		// Bumps
	private Subscriber<OdometryMsg>							odoSub;			// Odometry
	private Subscriber<org.ros.message.sensors_msgs.Image>	cameraSub;		// Camera

	private enum ListenerType {
		SONAR,BUMP,ODO,CAMERA
	}

	private class Listener<T> implements MessageListener<T> {
		private ListenerType type;
		public Listener(ListenerType _type) {
			type = _type;
		}

		public void onNewMessage(T m) {
			switch(type) {
			case SONAR: g.setSonars(m); break;
			case BUMP: g.setBumps(m); break;
			case ODO: g.setPose(m); break;
			case CAMERA: g.setCamera(m); break;
			default: g.assertTrue("Invalid Listener Type", false);
			}
		}
	}

	public Subscribers(Node node) {
		// Initialize subscriptions
		//
		sonarFrontSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg"); // front sonar
		sonarBackSub = node.newSubscriber("/rss/Sonars/Back", "rss_msgs/SonarMsg");   // back sonar
		bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");         // bump sensors
		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");         // odometry

		// Add message listeners to subscribers
		//
		sonarFrontSub.addMessageListener(new Listener<SonarMsg>(SONAR));
		sonarBackSub.addMessageListener(new Listener<SonarMsg>(SONAR));
		bumpSub.addMessageListener(new Listener<BumpMsg>(BUMP));
		odoSub.addMessageListener(new Listener<OdometryMsg>(ODO));
		cameraSub.addMessageListener(new Listener<org.ros.message.sensor_msgs.Image>(CAMERA));

		// Make the conditions accessible
		//
		g.subs = this;
	}
}

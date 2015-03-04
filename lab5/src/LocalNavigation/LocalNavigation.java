package LocalNavigation;
import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.SonarMsg;
import org.ros.message.rss_msgs.BumpMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;
import org.ros.namespace.GraphName;

public class LocalNavigation implements NodeMain,Runnable {
	// Kills run() on node shutdown
	//
	private boolean shutdown = false;

	// Subscribers
	//
	private Subscriber<SonarMsg>    sonarFrontSub; // Sonars
	private Subscriber<SonarMsg>    sonarBackSub;
	private Subscriber<BumpMsg>     bumpSub;       // Bump sensors
	private Subscriber<OdometryMsg> odoSub;        // Odometry

	// Helper objects
	// They do the bulk of the work
	//
	private SonarPoints sp;
	private FSM fsm;

	// Global status
	//
	double[] sonars  = new double[]         {0,0}; // {front,back}
	double[] pose    = new double[]       {0,0,0}; // {x,y,theta}
	double[] bumpers = new double[] {false,false}; // {left,right}

	public void onStart(Node node) {
		// Initialize helpers
		//
		sp = new SonarPoints(node); // sonar point publisher/line maker
		fsm = new FSM(node,FSM.ALIGN_ON_BUMP); // state machine

		// Initialize subscriptions
		//
		sonarFrontSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg"); // front sonar
		sonarBackSub = node.newSubscriber("/rss/Sonars/Back", "rss_msgs/SonarMsg");   // back sonar
		bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");         // bump sensors
		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");         // odometry

		// Describe message listeners
		//
		MessageListener<SonarMsg> sonarListener = new MessageListener<SonarMsg>() {
				@Override
				public void onNewMessage(SonarMsg m) {
					synchronized(this) {
						sonars[m.isFront?g.FRONT:g.BACK] = m.range;
					}
					sp.newPoint(m.isFront,m.range);
				}
			};

		MessageListener<BumpMsg> bumpListener = new MessageListener<BumpMsg>() {
				@Override
				public void onNewMessage(BumpMsg m) {
					synchronized(this) {
						bumpers[g.LEFT] = m.left;
						bumpers[g.RIGHT] = m.right;
					}
				}
			};

		MessageListener<OdometryMsg> odoListener = new MessageListener<OdometryMsg>() {
				@Override
				public void onNewMessage(OdometryMsg m) {
					double[] _pose = new double[3];
					synchronized(this) {
						pose[g.X] = m.x;
						pose[g.Y] = m.y;
						pose[g.THETA] = m.theta;
						System.arraycopy(pose,0,_pose,0,pose.length);
					}
					sp.updateRobotPose(_pose);
				}
			};

		// Add message listeners to subscribers
		//
		sonarFrontSub.addMessageListener(sonarListener);
		sonarBackSub.addMessageListener(sonarListener);
		bumpSub.addMessageListener(bumpListener);
		odoSub.addMessageListener(odoListener);

		new Thread(this).start();
	}

	public void onShutdown(Node node){
		synchronized(this) {
			shutdown = true;
		}

		if(node != null){
			node.shutdown();
		}
	}

	public void onShutdownComplete(Node node) {
	}


	public GraphName getDefaultNodeName() {
		return new GraphName("/rss/localnavigation");
	}

	/**
	 * Runs the FSM
	 * Calls step every 1/FSM.FREQ seconds
	 */
	public void run() {
		long initial_time;
		long duration;
		long sleep_time;

		// For thread safe copying
		//
		boolean _shutdown;
		double _sonars;
		double _pose;
		double _bumpers;

		do {
			initial_time = System.currentTimeMillis();

			// Update variables in a thread-safe manner
			//
			synchronized(this) {
				_shutdown = shutdown;
				System.arraycopy(sonars,  0, _sonars,  0, sonars.length );
				System.arraycopy(pose,    0, _pose,    0, pose.length   );
				System.arraycopy(bumpers, 0, _bumpers, 0, bumpers.length);
			}
			fsm.step(_sonars, _pose, _bumpers);
			duration = System.currentTimeMillis()-initial_time;
			sleep_time = ((long) (1000.0/FSM.FREQ)) - duration;
			Thread.sleep(sleep_time);
		} while (_shutdown);
	}
}

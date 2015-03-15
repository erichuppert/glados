package Grasping;

import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;

public class Grasping implements NodeMain, Runnable {
	@Override
	public void onStart(Node node) {
		new Subscribers(node);
		new Publishers(node);
		new Thread(this).start();
	}

	@Override
	public void run() {
		synchronized(g.shoulder) {
			synchronized(g.wrist) {
				synchronized(g.gripper) {
					g.shoulder.setTargetAngle(0);
					g.wrist.setTargetAngle(0);
					g.gripper.setTargetAngle(0);
					new Thread(g.shoulder).start();
					new Thread(g.wrist).start();
					new Thread(g.gripper).start();
					g.gripper.wait();
				}
				g.wrist.wait();
			}
			g.shoulder.wait();
		}
	}

	@Override
	public void onShutdown(Node node) {
		if (node != null) {
			node.shutdown();
		}
	}

	@Override
	public void onShutdownComplete(Node node) {
	}

	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("rss/grasping");
	}
}

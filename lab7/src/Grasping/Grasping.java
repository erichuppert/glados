package Grasping;

import org.rod.node.Node;
import org.ros.node.NodeMain;

public class Grasping implements NodeMain, Runnable {
	public void onStart(Node node) {
		new Subscribers(node);
		new Publishers(node);
		new Thread(this).start();
	}

	public void run() {}
}

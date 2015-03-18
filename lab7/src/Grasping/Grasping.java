package Grasping;

import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;

import java.util.Scanner;

public class Grasping implements NodeMain, Runnable {
	@Override
	public void onStart(Node node) {
		new Subscribers(node);
		new Publishers(node);
		new ArmControl();
		new WaypointNav();
		new Thread(this).start();
	}

	@Override
	public void run() {
		try{
			Thread.sleep(5000);
		} catch(InterruptedException e) {
			e.printStackTrace();
			return;
		}

		//try {
		// 	//fullRangeMotion();
			//inputHeights();
		// 	armGymnastics();
		//} catch(InterruptedException e) {
		// 	e.printStackTrace();
		//}
		new LabSM().run();
	}

	public void armGymnastics() {
		while (true) {
			g.ac.setGripperStatus(g.OPEN); g.ac.run();
			g.ac.setGripperStatus(g.CLOSED); g.ac.run();
			g.ac.setHeight(0.5); g.ac.run();
			g.ac.wrist.setTargetAngle(-0.5); g.ac.wrist.run();
			g.ac.setHeight(0); g.ac.run();
		}
	}

	public void inputHeights() throws InterruptedException {
		while(true) {
			double height = 0.3;
			System.out.println(height);
			g.ac.setGripperStatus(g.CLOSED);
			g.ac.setHeight(height); g.ac.run();
			Thread.sleep(2000);
			height = 0;
			g.ac.setGripperStatus(g.OPEN);
			g.ac.setHeight(height); g.ac.run();
			Thread.sleep(2000);
		}
	}

	public void fullRangeMotion() throws InterruptedException {
		double shoulder = -Math.PI/4;
		double wrist = 0;
		double gripper = 1;
		int shoulderSign = 1;
		int wristSign = 1;
		int gripperSign = -1;

		while (true) {
			g.ac.shoulder.setTargetAngle(shoulder);
			g.ac.wrist.setTargetAngle(wrist);
			g.ac.gripper.setTargetAngle(gripper);

			g.ac.shoulder.run();
			synchronized(g.ac.wrist) {}
			synchronized(g.ac.gripper) {}

			shoulder = shoulder+0.3*shoulderSign;
			wrist = wrist+0.3*wristSign;
			gripper = gripper+0.1*gripperSign;

			shoulderSign = Math.abs(shoulder)>Math.PI/3?-shoulderSign:shoulderSign;
			wristSign = Math.abs(wrist)>Math.PI/3?-wristSign:wristSign;
			gripperSign = gripper<0.2?1:gripper>0.8?-1:gripperSign;
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

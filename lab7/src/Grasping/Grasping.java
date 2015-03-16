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

		inputHeights();
	}

	public void inputHeights() {
		g.gripper.setTargetAngle(0.9);
		new Thread(g.gripper).start();
		while(true) {
			int height = g.getUser();
			ArmControl.setParams(height);
			g.wrist.setTargetAngle(ArmControl.getThetaWrist());
			g.shoulder.setTargetAngle(ArmControl.getThetaShoulder());
			new Thread(g.wrist).start();
			new Thread(g.shoulder).start();
			synchronized(g.wrist) {}
			synchronized(g.shoulder) {}
			try {
				Thread.sleep(100);
			} catch(InterruptedException e) {
				e.printStackTrace();
				return;
			}
		}
	}

	public void fullRangeMotion() {
		double shoulder = -Math.PI/4;
		double wrist = 0;
		double gripper = 1;
		int shoulderSign = 1;
		int wristSign = 1;
		int gripperSign = -1;

		while (true) {
			g.shoulder.setTargetAngle(shoulder);
			g.wrist.setTargetAngle(wrist);
			g.gripper.setTargetAngle(gripper);
			new Thread(g.shoulder).start();
			new Thread(g.wrist).start();
			new Thread(g.gripper).start();

			try{
				Thread.sleep(50);
			} catch(InterruptedException e) {
				return;
			}

			synchronized(g.wrist) {}
			synchronized(g.gripper) {}
			synchronized(g.shoulder) {}

			shoulder = shoulder+0.3*shoulderSign;
			wrist = wrist+0.3*wristSign;
			gripper = gripper+0.1*gripperSign;

			shoulderSign = Math.abs(shoulder)>Math.PI/4?-shoulderSign:shoulderSign;
			wristSign = Math.abs(wrist)>Math.PI/4?-wristSign:wristSign;
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

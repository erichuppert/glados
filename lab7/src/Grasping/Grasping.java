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

	public void armGymnastics() {
		
	}

	public void inputHeights() {
		g.ac.setGripperStatus(Gripper.CLOSED);
		while(true) {
			double height = g.getUser();
			synchronized(g.armController) {
				g.ac.setParams(height);
				new Thread(g.ac).start();
				g.ac.wait();
			}

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
			g.ac.shoulder.setTargetAngle(shoulder);
			g.ac.wrist.setTargetAngle(wrist);
			g.ac.gripper.setTargetAngle(gripper);

			synchronized(g.ac.shoulder) {
				new Thread(g.ac.shoulder).start();
				new Thread(g.ac.wrist).start();
				new Thread(g.ac.gripper).start();
				g.ac.shoulder.wait();
			}
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

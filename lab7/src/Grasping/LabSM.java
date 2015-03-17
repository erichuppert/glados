package Grasping;

import static java.lang.Math.*;

public class LabSM extends FSM<Object> implements Runnable {

	// State machine variables.
	//
	private double[] originalPose = new double[3];
	private double[] targetPose = new double[3];
	private final double forwardDistance = 1.0;
	private final double backDistance = 0.2;

	// States for the grasping/pick up/move/drop state machine
	//
	private final String OBJECT_WAITING = "Waiting for object to touch the gripper";
	private final String OBJECT_DETECTED = "Detected an object, closing the grip, and lifting it.";
	private final String MOVING_TARGET = "Moving to target destination.";
	private final String OBJECT_DROPPED = "Object dropped, moving back " + backDistance + " [m], and then moving forward.";
	private final String TARGET_REACHED = "Target reached, putting object on the ground.";
	private final String MOVING_BACK = "Object on the ground, moving back to original position.";
	private final String DONE = "Back to original position, done.";

	private final String INITIAL = OBJECT_WAITING;

	private final StateAction<Object> objectWaiting = new StateAction<Object>() {
			@Override
			public String action(Object _) {
				if (g.bumps[g.GRIPPER]) {
					g.pubs.setState(OBJECT_DETECTED);
					return OBJECT_DETECTED;
				}
				return null;
			}
		};

	private final StateAction<Object> objectDetected = new StateAction<Object>() {
			@Override
			public String action(Object _) {
				g.ac.setGripperStatus(g.MIDDLE);
				synchronized(g.ac) { new Thread(g.ac).start(); g.ac.wait(); }
				g.ac.setHeight(0.5);
				synchronized(g.ac) { new Thread(g.ac).start(); g.ac.wait(); }
				g.wp.addWP(targetPose);
				g.pubs.setState(MOVING_TARGET);
				return MOVING_TARGET;
			}
		};

	private final StateAction<Object> movingTarget = new StateAction<Object>() {
			@Override
			public String action(Object _) {
				if (!g.bumps[g.GRIPPER]) {
					g.wp.stopRunning();
					double[] next = g.getPose();
					next[g.X] += cos(next[g.THETA]+PI)*backDistance;
					next[g.Y] += sin(next[g.THETA]+PI)*backDistance;
					g.wp.addWP(next);
					g.pubs.setState(OBJECT_DROPPED);
					return OBJECT_DROPPED;
				} else if (g.wp.isDone()) {
					g.pubs.setState(TARGET_REACHED);
					return TARGET_REACHED;
				}
				return null;
			}
		};

	private final StateAction<Object> objectDropped = new StateAction<Object>() {
			@Override
			public String action(Object _) {
				if(g.wp.isDone()) {
					double[] next = g.getPose();
					next[g.X] += cos(next[g.THETA])*(backDistance+0.1);
					next[g.Y] += sin(next[g.THETA])*(backDistance+0.1);
					g.pubs.setState(OBJECT_WAITING);
					return OBJECT_WAITING;
				}
				return null;
			}
		};

	private final StateAction<Object> targetReached = new StateAction<Object>() {
			@Override
			public String action(Object _) {
				g.ac.setHeight(0);
				synchronized(g.ac) { new Thread(g.ac).start(); g.ac.wait(); }
				g.ac.setGripperStatus(g.OPEN);
				synchronized(g.ac) { new Thread(g.ac).start(); g.ac.wait(); }
				g.ac.setHeight(0.5);
				synchronized(g.ac) { new Thread(g.ac).start(); g.ac.wait(); }
				g.wp.addWP(originalPose);
				g.pubs.setState(MOVING_BACK);
				return MOVING_BACK;
			}
		};

	private final StateAction<Object> movingBack = new StateAction<Object>() {
			@Override
			public String action(Object _) {
				if (g.wp.isDone()) {
					g.pubs.setMotorVelocities(0,0);
					g.pubs.setState(DONE);
					return DONE;
				}
				return null;
			}
		};

	private final StateAction<Object> done = new StateAction<Object>() {
			@Override
			public String action(Object _) {
				return null;
			}
		};

	public LabSM() {
		addState(OBJECT_WAITING, objectWaiting);
		addState(OBJECT_DETECTED, objectDetected);
		addState(MOVING_TARGET, movingTarget);
		addState(OBJECT_DROPPED, objectDropped);
		addState(TARGET_REACHED, targetReached);
		addState(MOVING_BACK, movingBack);
		addState(DONE, done);
		super(INITIAL);
	}

	public void run() {
		int T;
		while (true) {
			long initial_time = System.currentTimeMillis();
			step(null);
			long duration = System.currentTimeMillis() - initial_time;
			try {
				Thread.sleep(max(0,((long)(1000.0/FREQ)) - duration));
			} catch (InterruptedException e) {
				e.printStackTrace();
				wp.stopRunning();
				g.pubs.setMotorVelocities(0,0);
			}
		}
	}

}
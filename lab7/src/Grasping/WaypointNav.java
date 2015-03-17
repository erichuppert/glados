package Grasping;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import static java.lang.Math.*;

public class WaypointNav implements Runnable {
	private BlockingQueue<double[]> wpQueue = new LinkedBlockingQueue<double[]>();
	private boolean stop = false;

	public WaypointNav() {
		g.wp = this;
		new Thread(this).start();
	}

	public void addWP(double[] wp) {
		wpQueue.offer(wp);
	}

	public boolean stopRunning() {
		wpQueue.clear();
		stop = true;
	}

	private final int updateFreq = 10; // Hz

	private final double Ka = 0.25;
	private final double Kd = 5*Ka;
	private final double straightness = 1000;
	private final double distanceThreshold = 0.01;
	private final double angleThreshold = 0.05;

	public void run() {
		double[] wp;

		while(true) {
			try {
				synchronized(this) {
					if (wpQueue.peek() == null) {
						this.notifyAll();
					}
				}
				wp = wpQueue.take();
			} catch(InterruptedException e) {
				g.pubs.setMotorVelocities(0,0);
				e.printStackTrace();
				return;
			}

			synchronized (this) {
				while (!stop) {
					double tv = 0;
					double rv = 0;
					double[] robot = g.getPose();
					double angleToWP = getAngleError(robot,wp);
					double dAngle = wp[g.THETA] - robot[g.THETA];
					dAngle = atan2(sin(dAngle), cos(dAngle));
					double distance = getDistanceError(robot,wp);

					if (distance > distanceThreshold) {
						// We are not at the right location
						//
						int sign = g.sign(cos(angleToWP));
						tv = Kd*distance*abs(pow(cos(angleToWP), straightness))*sign;
						rv = Ka*angleToWP;
					} else if (abs(dAngle) > angleThreshold) {
						// We're at the right location, but don't have the right angle.
						//
						tv = 0;
						rv = Ka*dAngle;
					} else {
						// We've reached our final pose.
						//
						break;
					}

					g.pubs.setMotorVelocities(tv,rv);
					try {
						Thread.sleep(1000/updateFreq);
					} catch (InterruptedException e) {
						e.printStackTrace();
						g.pubs.setMotorVelocities(0,0);
						return;
					}
				}
			}
			g.pubs.setMotorVelocities(0,0);
			stop = false;
		}
	}

	private double getAngleError(double[] from, double[] to) {
		double dx = to[g.X]-from[g.X];
		double dy = to[g.Y]-from[g.Y];
		double angle = atan2(dy,dx);
		double diff = angle-from[g.THETA];
		return atan2(sin(diff),cos(diff));
	}

	private double getDistanceError(double[] from, double[] to) {
		double dx = to[g.X]-from[g.X];
		double dy = to[g.Y]-from[g.Y];
		return sqrt(dx*dx+dy*dy);
	}
}

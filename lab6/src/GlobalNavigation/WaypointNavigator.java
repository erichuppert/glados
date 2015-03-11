package GlobalNavigation;

import java.awt.geom.Point2D;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;
import org.ros.node.topic.Publisher;
import org.ros.node.Node;
import org.ros.message.lab6_msgs.*;
import org.ros.message.lab5_msgs.*;
import org.ros.message.rss_msgs.*;

public class WaypointNavigator {
	private List<GraphNode<Point2D.Double>> wayPoints;
	//ROS Publishers
	//
	private Publisher<MotionMsg> motorPub; // motor commands

	private double[] robotPose;
	private int nextPointInd = 1;
	private Line2D.Double currentTrajectory;
	private Node node;

	public WaypointNavigator(Node _node, List<GraphNode<Point2D.Double>> _wayPoints) {
		wayPoints = _wayPoints;
		node = _node;
		motorPub = node.newPublisher("/command/Motors","rss_msgs/MotionMsg");
		currentTrajectory = new Line2D.Double(wayPoints.get(nextPointInd-1).getValue(), wayPoints.get(nextPointInd).getValue());
	}

	//Translational/Rotational velocities
	// So we don't have to redeclare them in every method.
	// Set by methods to change motor velocities.
	//
	private double tv,rv;

	// Below are values that have been tuned based on experimentation
	//
	private static float TRANSLATIONAL_SPEED = (float) 0.1;

	public boolean step(double[] _robotPose) {
		robotPose = _robotPose;
		if (nextPointInd == wayPoints.size()) {
			setMotorVelocities(0,0);
			return true;
		}
		Point2D.Double robotPoint = new Point2D.Double(robotPose[g.X], robotPose[g.Y]);
		GraphNode<Point2D.Double> nextNode = wayPoints.get(nextPointInd);

		if ( nearPoint(robotPoint, nextNode.getValue()) ) {
			nextPointInd++;
			currentTrajectory = new Line2D.Double(wayPoints.get(nextPointInd-1).getValue(), wayPoints.get(nextPointInd).getValue());
		}
		double angleError = getAngleError();
		double Ka = 0.25;
		double Kd = 5*Ka;
		// check if we still need to rotate towards the next point
		//
		if (Math.abs(angleError) > 0.05) {
			rv = -Ka*(-angleError);
			tv = 0;
		} else {
			// use a proportional controller to move forward
			//
			double rx = Math.cos(robotPose[g.THETA]);
			double ry = Math.sin(robotPose[g.THETA]);
			double tx = currentTrajectory.getX2() - currentTrajectory.getX1();
			double ty = currentTrajectory.getY2() - currentTrajectory.getY1();
			int sign = rx*tx + ry*ty < 0 ? -1:1;
			double distance = Math.sqrt(Math.pow((robotPose[g.X]-currentTrajectory.getX2()),2) + Math.pow((robotPose[g.Y]-currentTrajectory.getY2()),2));
			tv = sign*TRANSLATIONAL_SPEED*distance;
			double distanceError = getDistanceError();
			double theta_i = -Kd*distanceError;
			rv = -Ka*(theta_i - angleError);
		}
		setMotorVelocities(tv, rv);
		return false;
	}

	// 10cm threshold
	//
	public static double POINT_THRESHOLD_DIST = 0.05;

	// Checks if current robot position is close enough to its destination wayPoint
	// to continue towards the next wayPoint
	//
	private boolean nearPoint(Point2D.Double p1, Point2D.Double p2) {
		return p1.distance(p2) <= POINT_THRESHOLD_DIST;
	}

	public synchronized double getDistanceError() {
		if(robotPose == null) {
			return 0;
		} else {
			return currentTrajectory.ptLineDist(robotPose[g.X],  robotPose[g.Y]);
		}
	}

	public synchronized double getAngleError() {
		if(robotPose == null) {
			return 0;
		} else {
			double a = currentTrajectory.getY1() - currentTrajectory.getY2();
			double b = -(currentTrajectory.getX1() - currentTrajectory.getX2());
			double mag = Math.sqrt(a*a + b*b);
			a /= mag;
			b /= mag;

			double angle = Math.atan2(a,-b);
			double x = Math.cos(robotPose[g.THETA]);
			double y = Math.sin(robotPose[g.THETA]);
			double dot = (x*(-b) + y*a); // Vector parallel to the line
			if (dot < 0) {
				a = -a;
				b = -b;
				dot = (x*(-b) + y*a);
			}
			double sign = (x*a+y*b)>=0?1:-1;
			double diff = Math.acos(dot)*sign;
			return diff;
		}
	}

	// Useful for abstracting setting the motor velocities
	//
	private void setMotorVelocities(double tv, double rv) {
		MotionMsg msg = new MotionMsg();
		// Motor commands do not handle NaN well. We should never get these.
		//
		if (tv == Double.NaN || rv == Double.NaN) {
			throw new RuntimeException("NaN command sent!");
		}

		msg.translationalVelocity = 5* tv;
		msg.rotationalVelocity = rv;
		if(motorPub != null) {
			motorPub.publish(msg);
		}
	}

}

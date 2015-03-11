package src.GlobalNavigation;

import java.awt.geom.Point2D;
import java.awt.geom.Line2D;
import java.io.BufferedWriter;
import java.util.ArrayList;
import java.util.List;

public class WaypointNavigator {
	
	private List<WaypointNode> wayPoints;
	//ROS Publishers
	//
	private Publisher<MotionMsg> motorPub; // motor commands
	
	private double[] robotPose;
	private int nextPointInd = 1;
	private Line2D.Double currentTrajectory;
	private Node node;
	
	
	public WaypointNavigator(Node _node, List<WaypointNode> _wayPoints) {
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
	private static float TRANSLATIONAL_SPEED = (float) 0.05;
	private static float ROTATIONAL_SPEED = (float) 0.1;
	
	public boolean step(double[] _robotPose) {
		robotPose = _robotPose;
		Point2D.Double robotPoint = new Point2D.Double(robotPose[g.X], robotPose[g.Y]); 
		WaypointNode nextNode = wayPoints.get(nextPointInd);
		
		if ( nearPoint(robotPoint, nextNode.getValue()) ) {
			nextPointInd++;			
			if ( nextPointInd == wayPoints.size() ) {
				return true;
			} 
		}
		double angleError = getAngleError();
		// check if we still need to rotate towards the next point
		//
		if (Math.abs(angleError) > 0.1) {
			int angleDirectionFactor = angleError > 0 ? -1 : 1;
			rv = ROTATIONAL_SPEED * angleError;
			tv = 0;
		} else {
			// use a proportional controller to move forward
			//
			tv = TRANSLATIONAL_SPEED;
			double Kd = 2.5;
			double Ka = 0.5;
			double distanceError = getDistanceError();
			if (distanceError > 0.001 || angleError > 0.001) {
				double theta_i = -Kd*distanceError;
				rv = -Ka*(theta_i - angleError);
			} else {
				rv = 0;
			}				
		}
		setMotorVelocities(tv, rv);
		return false;
	}

	
	// 10cm threshold
	//
	public static double POINT_THRESHOLD_DIST = 0.1; 
	
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
			double a = currentTrajectory.x1 - currentTrajectory.x2;
			double b = currentTrajectory.y1 - currentTrajectory.y2;;
			double angle = Math.tan(b/a);
			double x = Math.cos(robotPose[g.THETA]);
			double y = Math.sin(angle);
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

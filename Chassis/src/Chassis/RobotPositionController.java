package Chassis;

//import MotorControlSolution.*;
import MotorControl.*;

/**
 * <p>A whole-robot position controller.</p>
 **/
public class RobotPositionController {

  /**
   * <p>The whole-robot velocity controller.</p>
   **/
  protected RobotVelocityController robotVelocityController;

  /**
   * <p>Total ticks since reset, positive means corresp side of robot moved
   * forward.</p>
   **/
  protected double[] totalTicks = new double[2];

  /**
   * <p>Total elapsed time since reset in seconds.</p>
   **/
  protected double totalTime = 0.0;

  /**
   * <p>Time in seconds since last update.</p>
   **/
  protected double sampleTime;

  /**
   * <p>An abstract gain; meaning depends on the particular subclass
   * implementation.</p>
   **/
  protected double gain = 1.0;

  /**
   * <p>The robot.</p>
   **/
  protected OdometryRobot robot;

  /**
   * <p>Create a new position controller for a robot.</p>
   *
   * @param robot the robot, not null
   **/
  public RobotPositionController(OdometryRobot robot) {
    this.robot = robot;
  }

  /**
   * <p>Translate at the specified speed for the specified distance.</p>
   *
   * <p>Blocks until the motion is complete or errored.</p>
   *
   * @param speed the desired robot motion speed in m/s
   * @param distance the desired distance to move in meters, relative to the
   * robot's pose at time of call. Must be non-negative
   *
   * @return true iff motion was successful
   **/
  public boolean translate(double speed, double distance) {
    boolean ok = true;
    // Begin Student Code
    if (distance < 0) {
    	throw new IllegalArgumentException("Cannot travel a distance that is negative");
    }
    robot.enableMotors(true);
    RobotVelocityController veloController = robot.getRobotVelocityController();
    veloController.setDesiredAngularVelocity(0, 0);
    // create conversion factors for ticks and angular/linear conversions
    double wheelDiameter = 0.1257; // in meters
    double wheelCircumference = wheelDiameter * Math.PI;
    double linearToAngularConversionFactor =  2 * Math.PI / wheelCircumference;
    double ticksToLinearTranslationConversionFactor = wheelCircumference / 131000.0; // there are 131000 ticks per revolution of the wheel
    
    int directionFactor = speed > 0 ? 1 : -1; // if the robot is going backwards, we need the acceleration to be negative
    double desiredMaxAngularSpeeed = speed * linearToAngularConversionFactor; // in Radians/sec
    double remainingDistance = distance;
    double[] lastTickCounts = totalTicks.clone(); // we will keep this to see how many ticks have happened since last update
    double[] tickDiffs = {0, 0};// number of ticks since the last update
    double distanceTravelled, timeDiff; 
    double timeAtLastIter = 0.0; // we will use this to keep track of the time the last iteration happened, so we can keep track of time/iteration
    double max_acceleration = directionFactor * 0.3; // in radians/(second^2)
    double desiredAngularSpeed = 0.0; // as we iterate, we will keep track of our velocity so we can control acceleration
    while (remainingDistance > 0) {
    	synchronized (this) {
    		// calculate the difference in time since the last iteration
    		timeDiff = timeAtLastIter - totalTime;

    		// calculate the remaining distance using tick counts
    		tickDiffs[0] = totalTicks[0] - lastTickCounts[0];
    		tickDiffs[1] = totalTicks[1] - lastTickCounts[1];
    		distanceTravelled = Math.abs((tickDiffs[0] + tickDiffs[1])/2 * ticksToLinearTranslationConversionFactor); // for simplicity, we assume both wheels are moving at same rate and use their average here
    		remainingDistance -= distanceTravelled;

    		// select the actual speed that we want the motors to go, taking into account acceleration maximums and max desired speed
    		// if (remainingDistance > Math.min(0.5 * distance, 0.4)) {
    		// 	desiredAngularSpeed = directionFactor * Math.min(Math.abs(desiredMaxAngularSpeeed), Math.abs(desiredAngularSpeed + max_acceleration * timeDiff));
    		// } else {
    		// 	desiredAngularSpeed = directionFactor * Math.min(Math.abs(desiredMaxAngularSpeeed), Math.abs(desiredAngularSpeed * 0.5));
    		// }

    		veloController.setDesiredAngularVelocity(desiredMaxAngularSpeeed,desiredMaxAngularSpeeed);

    		// keep track of variables for next iteration
    		lastTickCounts = totalTicks.clone();
    		timeAtLastIter = totalTime;
    	}
    }
    veloController.setDesiredAngularVelocity(0, 0);
    
    robot.enableMotors(false);
    // End Student Code
    
    return ok;
  }
  

  /**
   * <p>Rotate at the specified speed for the specified angle.</p>
   *
   * <p>Blocks until the motion is complete or errored.</p>
   *
   * @param speed the desired robot motion speed in radians/s
   * @param angle the desired angle to rotate in radians, relative to the
   * robot's pose at time of call.
   *
   * @return true iff motion was successful
   **/
  public boolean rotate(double speed, double angle) {
	boolean ok = true;
    // Begin Student Code
	robot.enableMotors(true);
	double robotRadius = 0.215; // in meters
	double wheelDiameter = 0.1258; // in meters
	double wheelCircumference = wheelDiameter * Math.PI;
	double desiredWheelTravelDistance = Math.abs(robotRadius * angle);
	double linearToAngularConversionFactor =  2 * Math.PI / wheelCircumference;
	double remainingWheelTravelDistance = desiredWheelTravelDistance;
	double wheelSpeed = robotRadius * speed * linearToAngularConversionFactor; // how fast the wheels need to go to travel at the desired speed
	RobotVelocityController veloController = robot.getRobotVelocityController();
	double desiredSpeed = 0.0;
	double lastSpeed = 0.0;
	double lastTime = 0.0;
	double timeSinceLastIter = 0;
	double[] lastTicks = totalTicks.clone();
	double[] tickDiffs = {0, 0};
	double ticksToLinearTranslationConversionFactor = wheelCircumference / 131000.0; // there are 131000 ticks per revolution of the wheel
	double distanceTraveled;
	int directionFactor = speed > 0 ? -1 : 1;
	
	System.out.println("Each wheel must travel " + Double.toString(remainingWheelTravelDistance));
	System.out.println("The desired wheel speed is " + Double.toString(wheelSpeed));
	
	while (remainingWheelTravelDistance > 0) {
		synchronized (this) {
			tickDiffs[0] = totalTicks[0] - lastTicks[0];
			tickDiffs[1] = totalTicks[1] - lastTicks[1];
			distanceTraveled = (Math.abs(tickDiffs[0]) + Math.abs(tickDiffs[1]))/2.0 * ticksToLinearTranslationConversionFactor;
			remainingWheelTravelDistance -= distanceTraveled;

			timeSinceLastIter = lastTime - totalTime;
			desiredSpeed = directionFactor * Math.abs(wheelSpeed);
			veloController.setDesiredAngularVelocity(desiredSpeed, -desiredSpeed);
			lastSpeed = desiredSpeed;
			lastTime = totalTime;
			lastTicks = totalTicks.clone();
		}
	}
	veloController.setDesiredAngularVelocity(0, 0);
	robot.enableMotors(false);
    // End Student Code
	return ok;	
  }
    
  private static String doubleListToString(double[] foo) {
	  String out = "[";
	  for (double d : foo) {
		  out += ", " + Double.toString(d);
	  }
	  out += "]";
	  return out;
  }
  

  /**
   * <p>If position control is closed-loop, this computes the new left and
   * right velocity commands and issues them to {@link
   * #robotVelocityController}.</p>
   **/
  public synchronized void controlStep() {

    if (robotVelocityController == null)
      return;

    if (!robot.motorsEnabled() || robot.estopped())
      return;

    // Begin Student Code (if implementing closed-loop control)
    // End Student Code (if implementing closed-loop control)
  }

  /**
   * <p>Set the whole-robot velocity controller.</p>
   *
   * <p>This is called automatically by {@link OdometeryRobot}.</p>
   *
   * @param vc the whole-robot velocity controller
   **/
  public void setRobotVelocityController(RobotVelocityController vc) {
    robotVelocityController = vc;
  }

  /**
   * <p>Set {@link #gain}.</p>
   *
   * @param g the new gain
   **/
  public void setGain(double g) {
    gain = g;
  }

  /**
   * <p>Get {@link #gain}.</p>
   *
   * @return gain
   **/
  public double getGain() {
    return gain;
  }

  /**
   * <p>Update feedback and sample time.</p>
   *
   * @param time the time in seconds since the last update, saved to {@link
   * #sampleTime}
   * @param leftTicks left encoder ticks since last update, positive means
   * corresp side of robot rolled forward
   * @param rightTicks right encoder ticks since last update, positive means
   * corresp side of robot robot rolled forward
   **/
  public synchronized void update(double time,
                                  double leftTicks, double rightTicks) {

    sampleTime = time;

    System.out.printf("Left: %.2f\tRight: %.2f\n", leftTicks, rightTicks);
    totalTicks[RobotBase.LEFT] += leftTicks;
    totalTicks[RobotBase.RIGHT] += rightTicks;
    totalTime += time;
  }
}

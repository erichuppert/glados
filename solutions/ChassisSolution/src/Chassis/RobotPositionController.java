package ChassisSolution;

//import MotorControlSolution.*;
import MotorControlSolution.*;

/**
 * <p>A whole-robot position controller.</p>
 **/
public class RobotPositionController {
  /** (Solution)
   * <p>Distance between wheels in meters.</p> (Solution)
   **/ // (Solution)
  public static final double WHEELBASE = 0.428; // (Solution)
  /** (Solution)
   * <p>Wheelbase circumferential meters per robot rotation radian.</p> (Solution)
   **/ // (Solution)
  public static final double WHEELBASE_METERS_PER_RAD = WHEELBASE/2.0; // (Solution)
  /** (Solution)
   * <p>Wheel diameter in meters.</p> (Solution)
   **/ // (Solution)
  public static final double WHEEL_DIA = 0.125; // (Solution)
  /** (Solution)
   * <p>Wheel circumference in meters.</p> (Solution)
   **/ // (Solution)
  public static final double WHEEL_CIRC = WheelVelocityController.WHEEL_RADIUS_IN_M * 2 * Math.PI; // (Solution)
  /** (Solution)
   * <p>Encoder ticks per wheel revolution.</p> (Solution)
   **/ // (Solution)
    public static final double WHEEL_TICKS_PER_REV = WheelVelocityController.TICKS_PER_REVOLUTION; // (Solution)
  /** // (Solution)
   * <p>Meters traveled on ground per tick.</p> // (Solution)
   **/ // (Solution)
  public static final double WHEEL_METERS_PER_TICK = // (Solution)
    WHEEL_CIRC/WHEEL_TICKS_PER_REV; // (Solution)
  /** // (Solution)
   * <p>Meters traveled on ground per wheel radian.</p> // (Solution)
   **/ // (Solution)
  public static final double WHEEL_METERS_PER_RAD = // (Solution)
    WHEEL_CIRC/(2.0*Math.PI); // (Solution)
  /** // (Solution)
   * <p>Max allowed speed in m/s.</p> (Solution)
   **/ // (Solution)
  public static final double MAX_SPEED = 0.4; // (Solution)
  /** (Solution)
   * <p>Allowed slop per wheel in meters.</p> (Solution)
   **/ // (Solution)
  public static final double SLOP = 0.005; // (Solution)
  /** // (Solution)
   * <p>Error of the current motion, per wheel, in meters.</p> // (Solution)
   **/ // (Solution)
  protected double[] error = new double[2]; // (Solution)
  /** (Solution)
   * <p>Target of the current motion, per wheel, in ticks.</p> (Solution)
   **/ // (Solution)
  protected double[] targetTicks = new double[2]; // (Solution)
  /** // (Solution)
   * <p>Max angular speed in rad/s for current motion.</p> (Solution)
   **/ // (Solution)
  protected double maxAngularSpeed = 0.0; // (Solution)

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
   * robot's pose at time of call.
   *
   * @return true iff motion was successful
   **/
  public boolean translate(double speed, double distance) {
		boolean ok = true;
    // Begin Student Code
    ok = move(speed, distance, distance); // (Solution)
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
    double distance = angle*WHEELBASE_METERS_PER_RAD; // (Solution)
    ok = move(speed*WHEELBASE_METERS_PER_RAD, // (Solution)
              -distance, distance); // (Solution)
    // End Student Code
	  return ok;	
  }
  /** (Solution)
   * <p>Move left and right wheels the specified distances at the specified (Solution)
   * linear speed.</p> (Solution)
   * (Solution)
   * <p>Blocks until the motion is complete or errored.</p> (Solution)
   * (Solution)
   * @param speed the speed in m/s (Solution)
   * @param l the distance to move the left wheel on the ground in meters (Solution)
   * @param r the distance to move the right wheel on the ground in meters (Solution)
   * (Solution)
   * @return true iff motion was successful (Solution)
   **/ // (Solution)
  protected boolean move(double speed, double l, double r) { // (Solution)
    speed = clampMag(speed, MAX_SPEED); // (Solution)
    double deadline; // (Solution)
    synchronized (this) { // (Solution)
      targetTicks[RobotBase.LEFT] = // (Solution)
        totalTicks[RobotBase.LEFT] + l/WHEEL_METERS_PER_TICK; // (Solution)
      targetTicks[RobotBase.RIGHT] = // (Solution)
        totalTicks[RobotBase.RIGHT] + r/WHEEL_METERS_PER_TICK; // (Solution)
      deadline = totalTime + // (Solution)
        1.1*Math.max(Math.abs(l)/speed, Math.abs(r)/speed) + // (Solution)
        10.0; // (Solution)
      maxAngularSpeed = Math.abs(speed/WHEEL_METERS_PER_RAD); // (Solution)
//      System.err.println("starting ticks: " + // (Solution)
//                         totalTicks[RobotBase.LEFT] + ", " + // (Solution)
//                         totalTicks[RobotBase.RIGHT]); // (Solution)
//      System.err.println("target ticks: " + // (Solution)
//                         targetTicks[RobotBase.LEFT] + ", " + // (Solution)
//                         targetTicks[RobotBase.RIGHT]); // (Solution)
//      System.err.println("deadline: " + deadline); // (Solution)
    } // (Solution)
    if (robot.estopped()) // (Solution)
      return false; // (Solution)
    //start motion (Solution)
    robotVelocityController.setDesiredAngularVelocity( // (Solution)
      Math.signum(l)*Math.signum(speed)*maxAngularSpeed, // (Solution)
      Math.signum(r)*Math.signum(speed)*maxAngularSpeed); // (Solution) 
    robot.enableMotors(true); // (Solution)
    //wait for completion (Solution)
    boolean failed = false; // (Solution)
    for (;;) { // (Solution)
      synchronized (this) { // (Solution)
        if (robot.estopped()) { // (Solution)
          failed = true; // (Solution)
          System.err.println("estop failure"); // (Solution)
          break; // (Solution)
        } // (Solution)
        updateError(); // (Solution)
//        System.err.println("error: " + // (Solution)
//                           error[RobotBase.LEFT] + ", " + // (Solution)
//                           error[RobotBase.RIGHT]); // (Solution)
//        System.err.println("totalTime: " + totalTime); // (Solution)
        if ((Math.abs(error[RobotBase.LEFT]) <= SLOP) && // (Solution)
            (Math.abs(error[RobotBase.RIGHT]) <= SLOP)) // (Solution)
          break; // (Solution)
        if (totalTime > deadline) { // (Solution)
          failed = true; // (Solution)
          System.err.println("deadline failure"); // (Solution)
          break; // (Solution)
        } // (Solution)
      } // (Solution)
      try { // (Solution)
        Thread.sleep(200); // (Solution)
      } catch (InterruptedException e) { // (Solution)
        failed = true; // (Solution)
        System.err.println("sleep failure"); // (Solution)
        break; // (Solution)
      } // (Solution)
    } // (Solution)
    synchronized (this) { // (Solution)
      robot.enableMotors(false); // (Solution)
      robotVelocityController.setDesiredAngularVelocity(0.0, 0.0); // (Solution)
    } // (Solution)
    if (failed) // (Solution)
      System.err.println("warning: motion failed!"); // (Solution)
    

    try { // (Solution)
      Thread.sleep(1000); // (Solution)
    } catch (InterruptedException e) { // (Solution)
    } // (Solution)
    
    return !failed; // (Solution)

  } // (Solution)
  /** (Solution)
   * <p>Clamp v to [-c, c].</p> (Solution)
   * (Solution)
   * @param v the value to clamp (Solution)
   * @param c the clamp magnitude (Solution)
   * @return v clamped to [-c, c] (Solution)
   **/ // (Solution)
  protected static double clampMag(double v, double c) { // (Solution)
    if (v > c) // (Solution)
      return c; // (Solution)
    if (v < -c) // (Solution)
      return -c; // (Solution)
    return v; // (Solution)
  } // (Solution)
  /** (Solution)
   * <p>Update {@link #error} from {@link #targetTicks} and {@link #totalTicks}.</p> (Solution)
   **/ // (Solution)
  protected synchronized void updateError() { // (Solution)
    for (int i = RobotBase.LEFT; i <= RobotBase.RIGHT; i++) // (Solution)
      error[i] = (targetTicks[i] - totalTicks[i])*WHEEL_METERS_PER_TICK; // (Solution)
  } // (Solution)
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
    updateError(); // (Solution)
    robotVelocityController.setDesiredAngularVelocity( // (Solution)
      clampMag(gain*error[RobotBase.LEFT], maxAngularSpeed), // (Solution)
      clampMag(gain*error[RobotBase.RIGHT], maxAngularSpeed)); // (Solution)
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

    totalTicks[RobotBase.LEFT] += leftTicks;
    totalTicks[RobotBase.RIGHT] += rightTicks;
    totalTime += time;
  }
}

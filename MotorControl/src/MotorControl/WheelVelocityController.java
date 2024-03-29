package MotorControl;

/**
 * <p>Generic base class for a single wheel velocity controller.</p>
 *
 * <p>Subclasses fill in {@link #controlStep} to implement the control
 * method.</p>
 *
 * @author vona
 * @author prentice
 * @author ara
 **/
public abstract class WheelVelocityController extends VelocityController {

  /**
   * <p>The maximum pwm command magnitude.</p>
   **/
  protected static final double MAX_PWM = 255;

  /**
   * <p>Student Code: unloaded maximum wheel angular velocity in rad/s. 
   * This should be a protected static final double called MAX_ANGULAR_VELOCITY.</p>
   **/
  protected static final double MAX_ANGULAR_VELOCITY = 7.5;
    
  /**
   * <p>Student Code: radius of the wheel on the motor (in meters). 
   * This should be a protected static final double called WHEEL_RADIUS_IN_M.</p>
   **/
  protected static final double WHEEL_RADIUS_IN_M = 0.0629;

  /**
   * <p>Student Code: encoder resolution.</p>
   * This should be a protected static final double called ENCODER_RESOLUTION.</p>
   * Hint: ENCODER_RESOLUTION units: ticks/revolution (WITHOUT gear ratio term)
   **/
  protected static final double ENCODER_RESOLUTION = 500;

  /**
   * <p>Student Code: motor revolutions per wheel revolution.
   * This should be a protected static final double called GEAR_RATIO.</p>
   **/
  protected static final double GEAR_RATIO = 65.5;

  /**
   * <p>Student Code: encoder ticks per motor revolution.</p>
   * This should be a protected static final double called TICKS_PER_REVOLUTION.</p>
   * Hint: TICKS_PER_REVOLUTION units: ticks/revolution (WITH gear ratio term)
   **/
  protected static final double TICKS_PER_REVOLUTION = GEAR_RATIO*ENCODER_RESOLUTION*4;

  /**
   * <p>Angular velocity in rad/s as we have been commanded.</p>
   **/
  protected double desiredAngularVelocity;

  /**
   * <p>Angular velocity in rad/s as we have measured it from the encoder.</p>
   **/
  protected double currentAngularVelocity;

  /**
   * <p>Encoder ticks since last update.</p>
   **/
  protected double encoderTicks;


  /**
   * <p>Subclasses fill this in to implement particular control methods.</p>
   *
   * <p>In particular, you will probably want to read some subset of {@link
   * #desiredAngularVelocity}, {@link #currentAngularVelocity}, and {@link
   * #controlTicks}.</p>
   *
   * @return the control output, which is the PWM command in the range [-{@link
   * #MAX_PWM}, {@link #MAX_PWM}], positive means wheel turned CCW when 
   * observing wheel from the outside (non-motor side)
   **/
  abstract double controlStep();

  /**
   * <p>Set {@link #desiredAngularVelocity}.</p>
   *
   * @param the desired angular velocity in rad/s
   **/
  public void setDesiredAngularVelocity(double vel) {
    desiredAngularVelocity = vel;
  }

  /**
   * <p>Get {@link #desiredAngularVelocity}.</p>
   *
   * @return the desired angular velocity in rad/s
   **/
  public double getDesiredAngularVelocity() {
    return desiredAngularVelocity;
  }

  /**
   * <p>Computes radians per encoder tick.</p>
   *
   * @return radians per encoder tick
   **/
  public double computeRadiansPerTick() {
    double result = 0; //allow compilation w/o soln
    // Start Student Code
    result = 2*Math.PI/TICKS_PER_REVOLUTION;
    // End Student Code
    return result;
  }
    
  /**
   * <p>Computes the encoder relative motion in radians since last update.</p>
   *
   * @return the encoder relative motion in radians since last update, positive
   * means wheel turned CCW when observing wheel from the outside (non-motor
   * side)
   **/
  public double computeEncoderInRadians() {
    double result = 0;
    result = computeRadiansPerTick()*encoderTicks;
    return result;
  }
  
  /**
   * <p>Computes the angular velocity in rad/s.</p>
   *
   * @return the angular velocity in rad/s, positive means wheel turned CCW
   * when observing wheel from the outside (non-motor side)
   **/
  public double computeAngularVelocity() {
    double result = 0; // allow compilation w/o soln
    
    if (sampleTime == 0.0) {
		result = 0;    // prevent division by 0 on startup
    }
    else {
      // Start Student Code
      result = computeEncoderInRadians()/sampleTime;
      // End Student Code
    }
    return result;
  }

  public double computeTranslationalVelocity() {
    double result = 0; // allow compilation w/o soln

    if (sampleTime == 0.0) {
		result = 0;    // prevent division by 0 on startup
    }
    else {
      // Start Student Code
      result = computeAngularVelocity()*WHEEL_RADIUS_IN_M;
      // End Student Code
    }
    return result;
  }


  /**
   * <p>Update feedback and sample time.</p>
   *
   * <p>Also calls {@link #computeAngularVelocity} and stores the result in
   * {@link #currentAngularVelocity}: this is the current angular velocity
   * feedback.</p>
   *
   * @param time the time in seconds since the last update, saved to {@link
   * #sampleTime}
   * @param leftTicks left encoder ticks since last update, saved to {@link
   * #sampleTicks}, positive means wheel turned CCW when observing wheel from
   * the outside (non-motor side)
   **/
  public void update(double time, double ticks) {
    super.update(time);
    encoderTicks = ticks;
    currentAngularVelocity = computeAngularVelocity();
  }
}

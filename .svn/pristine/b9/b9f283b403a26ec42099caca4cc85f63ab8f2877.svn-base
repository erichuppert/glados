package MotorControl;

/**
 * <p>Feed-forward wheel velocity controller.</p>
 *
 * @author vona
 * @author prentice
 **/
public class WheelVelocityControllerFF extends WheelVelocityController {

  /**
   * {@inheritDoc}
   *
   * <p>This impl implements simple feed-forward control.</p>
   **/
  public double controlStep() {
    double result = 0;
    // Start Student Code
    // Motion Control Lab, Part 9 
    result = Math.max(-255, Math.min(255,gain*desiredAngularVelocity));
    // End Student Code
    return result;
  }

  /**
   * {@inheritDoc}
   *
   * <p>This impl returns "FF".</p>
   **/
  public String getName() {
    return "FF";
  }
}

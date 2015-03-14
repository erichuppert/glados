package Grasping;

public class Servo implements Runnable {
	private final int minPWM;
	private final int maxPWM;
	private final double maxSpeed; // delta_pwm per second

	private final double minAngle;
	private final double maxAngle;

	private final int outIndex;

	private int targetPWM;

	// FSM definition
	//
	private final static String DONE = "Done with Servo rotation";
	private final static String ROTATING = "Rotating Servo";
	private final FSM fsm = new FSM<Integer>(DONE);

	public Servo(int _minPWM, int _maxPWM, double _maxSpeed,
				 double _minAngle, double _maxAngle,
				 int _outIndex) {
		minPWM = _minPWM;
		maxPWM = _maxPWM;
		maxSpeed = _maxSpeed;
		minAngle = _minAngle;
		maxAngle = _maxAngle;
		outIndex = _outIndex;
	}

	/**
	 * Takes an angle, and converts it to the PWM value to take the servo to that angle
	 * The angle must be valid for the specified servo.
	 * @param angle angle to rotate to.
	 * @return PWM value for that angle.
	 */
	public int angleToPWM(double angle) {
		//g.assertTrue(angle >= minAngle && angle <= maxAngle);
		double alpha = (angle-minAngle)/(maxAngle-minAngle);
		int PWM = (int)(alpha*(maxPWM-minPWM)) + minPWM;
		return PWM;
	}

	public void setTargetAngle(double angle) {
		targetPWM = angleToPWM(angle);
	}

	/**
	 * Puts the servo in the targetPWM position.
	 * Respects the maxSpeed parameters of the servo.
	 * It is implemented in the run() method so that it is easily Threadable.
	 * If methods want to block while waiting for this to finish, it synchronizes on this object.
	 */
	public synchronized void run() {}
}

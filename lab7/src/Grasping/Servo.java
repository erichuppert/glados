package Grasping;

public class Servo implements Runnable {
	private final int minPWM;
	private final int maxPWM;
	private final double maxSpeed; // delta_rad per second
	private final int outIndex;
	// PWM = alpha theta + beta
	//
	private final double alpha;
	private final double beta;

	private int targetPWM;

	public Servo(int _minPWM, int _maxPWM, double _maxSpeed,
				 double PWM1, double PWM2,
				 double angle1, double angle2,
				 int _outIndex) {
		minPWM = _minPWM;
		maxPWM = _maxPWM;
		alpha = (PWM2-PWM1)/(angle2-angle1);
		beta = PWM1-alpha*angle1;
		maxSpeed = _maxSpeed;
		outIndex = _outIndex;
	}

	/**
	 * Takes an angle, and converts it to the PWM value to take the servo to that angle
	 * The angle must be valid for the specified servo.
	 * @param angle angle to rotate to.
	 * @return PWM value for that angle.
	 */
	public int angleToPWM(double angle) {
		return (int)(alpha*angle + beta);
	}

	private double PWMToAngle(int pwm) {
		return ((double)pwm-beta)/alpha
	}

	public void setTargetAngle(double angle) {
		targetPWM = angleToPWM(angle);
		g.assertTrue(targetPWM >= minPWM && targetPWM <= maxPWM);
	}

	/**
	 * Puts the servo in the targetPWM position.
	 * Respects the maxSpeed parameters of the servo.
	 * It is implemented in the run() method so that it is easily Threadable.
	 * If methods want to block while waiting for this to finish, it synchronizes on this object.
	 */
	public synchronized void run() {
		double currentPWM;
		do {
			currentPWM = g.getArm()[outIndex];
			double nextAngle = PWMToAngle(currentPWM)+maxSpeed;
			int nextPWM = angleToPWM(nextAngle);
			g.pubs.setArm(outIndex,nextPWM);
			try{
				Thread.sleep(1000);
			} catch(InterruptedException e) {
				g.pubs.setArm(outIndex,0);
				break;
			}
		} while(currentPWM != targetPWM);
		this.notifyAll();
	}
}

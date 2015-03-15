package Grasping;

public class Servo implements Runnable {
	private final long minPWM;
	private final long maxPWM;
	private final double maxSpeed; // delta_rad per second
	private final int outIndex;
	// PWM = alpha theta + beta
	//
	private final double alpha;
	private final double beta;

	private long targetPWM;

	public Servo(long _minPWM, long _maxPWM, double _maxSpeed,
				 long PWM1, long PWM2,
				 double angle1, double angle2,
				 int _outIndex) {
		minPWM = _minPWM;
		maxPWM = _maxPWM;
		alpha = ((double)(PWM2-PWM1))/(angle2-angle1);
		beta = (double)PWM1-alpha*angle1;
		maxSpeed = _maxSpeed;
		outIndex = _outIndex;
	}

	/**
	 * Takes an angle, and converts it to the PWM value to take the servo to that angle
	 * The angle must be valid for the specified servo.
	 * @param angle angle to rotate to.
	 * @return PWM value for that angle.
	 */
	public long angleToPWM(double angle) {
		return (long)(alpha*angle + beta);
	}

	private double PWMToAngle(long pwm) {
		return ((double)pwm-beta)/alpha;
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
		long currentPWM;
		double T = 0.5;
		do {
			currentPWM = g.getArm()[outIndex];
			currentPWM = currentPWM == 0?minPWM:currentPWM;
			int sign = g.sign(PWMToAngle(targetPWM) - PWMToAngle(currentPWM));
			double nextAngle = PWMToAngle(currentPWM)+maxSpeed*sign*T;
			long nextPWM = Math.max(Math.min(angleToPWM(nextAngle),maxPWM),minPWM);
			if(Math.abs(nextPWM-currentPWM) > Math.abs(targetPWM-currentPWM)) {
				nextPWM = targetPWM;
			}
			System.out.printf("Current: %d\tNext: %d\tTarget: %d\n", currentPWM,nextPWM,targetPWM);
			g.pubs.setArm(outIndex,nextPWM);
			try{
				Thread.sleep(1000*T);
			} catch(InterruptedException e) {
				g.pubs.setArm(outIndex,0);
				break;
			}
		} while(currentPWM != targetPWM);
		this.notifyAll();
	}
}

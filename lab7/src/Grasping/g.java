package Grasping;

import org.ros.message.rss_msgs.SonarMsg;
import org.ros.message.rss_msgs.BumpMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.message.rss_msgs.ArmMsg;
//import org.ros.message.sensor_msgs.Image;

// Useful methods, and values that many classes might need to access.
//

public class g {
	// Indices for sonars, poses, and bumpers
	//
	public static final int FRONT    = 0;
	public static final int BACK     = 1;
	public static final int X        = 0;
	public static final int Y        = 1;
	public static final int THETA    = 2;
	public static final int LEFT     = 0;
	public static final int RIGHT    = 1;
	public static final int GRIPPER  = 2;
	public static final int SHOULDER = 0;
	public static final int WRIST    = 1;

	/**
	 * Gripper states
	 */
	public static final double CLOSED = 0.95;
	public static final double OPEN   = 0.05;
	public static final double MIDDLE = CLOSED;

	/**
	 * Publishers to send commands to the robot.
	 * Control:
	 * - Motor velocities
	 * - Robot Pose
	 * - State
	 * - Arm PWMs
	 */
	public static Publishers pubs = null;

	/**
	 * Keep track of several of the robot's variables:
	 * - Pose
	 * - Camera data
	 * - Sonars
	 * - Bump sensors
	 * - Arm PWMs
	 */
	private static final int		width	   = 160;
	private static final int		height	   = 120;
	private static Object			cameraLock = new Object();
	private static final boolean	reverseRGB = false;
	private static Image			camera     = null;

	// Safe-ish first values
	//
	private static double[] pose    = {0,0,0};
	private static double[] sonars  = {-1,-1,-1};
	private static boolean[] bumps  = {false,false,false};
	private static long[] arm       = {0,0,0};
	private static double userInput = 0.3;

	// Arm control
	//
	public static ArmControl ac = null;

	// Waypoint Navigator
	//
	public static WaypointNav wp = null;

	public static void setCamera(org.ros.message.sensor_msgs.Image m) {
		byte[] rgbData;
		if (reverseRGB) {
			rgbData = Image.RGB2BGR(m.data, (int) m.width, (int) m.height);
		} else {
			rgbData = m.data;
		}
		assertTrue("Width is wrong", width == m.width);
		assertTrue("Height is wrong", height == m.height);

		synchronized(cameraLock) {
			camera = new Image(rgbData, width, height);
		}
	}

	public static Image getCamera() {
		synchronized(cameraLock) {
			return camera;
		}
	}

	public static void setPose(OdometryMsg m) {
		synchronized(pose) {
			pose[X] = m.x;
			pose[Y] = m.y;
			pose[THETA] = m.theta;
		}
	}

	public static double[] getPose() {
		synchronized(pose) {
			return pose.clone();
		}
	}

	public static void setSonars(SonarMsg m) {
		synchronized(sonars) {
			double alpha = 0.3; // LP-filter weight, 0 <= alpha <= 1
			int sonar = m.isFront?FRONT:BACK;
			sonars[sonar] = alpha*m.range+(1-alpha)*sonars[sonar];
		}
	}

	public static double[] getSonars() {
		synchronized(sonars) {
			return sonars.clone();
		}
	}

	public static void setBumps(BumpMsg m) {
		synchronized(bumps) {
			bumps[LEFT] = m.left;
			bumps[RIGHT] = m.right;
			bumps[GRIPPER] = m.gripper;
		}
	}

	public static boolean[] getBumps() {
		synchronized(bumps) {
			return bumps.clone();
		}
	}

	public static void setArm(ArmMsg m) {
		synchronized(arm) {
			arm[SHOULDER] = m.pwms[SHOULDER];
			arm[WRIST] = m.pwms[WRIST];
			arm[GRIPPER] = m.pwms[GRIPPER];
		}
	}

	public static long[] getArm() {
		synchronized(arm) {
			return arm.clone();
		}
	}

	public static void setUser(org.ros.message.std_msgs.Float64 m) {
		System.out.printf("Got data: %.2f\n", m.data);
		userInput = m.data;
	}

	public static double getUser() {
		return userInput;
	}

	/**
	 * Asserts the truth of an expression.
	 * Throws a runtimeExpression with an empty message if that fails.
	 * @param mustBeTrue expression to assert.
	 * @throws RuntimeException if expression is false.
	 */
	public static void assertTrue(boolean mustBeTrue) {
		assertTrue(null, mustBeTrue);
	}

	/**
	 * Asserts the truth of an expression.
	 * Throws a runtimeExpression with the passed message if it fails.
	 * @param msg error message if assert fails.
	 * @param mustBeTrue expression to assert.
	 * @throws RuntimeException if expression is false.
	 */
	public static void assertTrue(String msg, boolean mustBeTrue) {
		if(!mustBeTrue) {
			throw new RuntimeException(msg);
		}
	}

	public static int sign(double v) {
		return (v < 0) ? (-1):1;
	}
}

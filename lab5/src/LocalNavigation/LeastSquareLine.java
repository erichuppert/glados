// least square estimator for line from appendix
//

package LocalNavigation;

import java.lang.Exception;

import org.ros.message.rss_msgs.*;
import org.ros.message.std_msgs.*;
import org.ros.message.lab5_msgs.*;
import org.ros.node.topic.Publisher;

public class LeastSquareLine {
	private double sumX;
	private double sumY;
	private double sumXX;
	private double sumYY;
	private double sumXY;

	private long nPoints;
	private boolean lineDirty;
	private double[] line; // {a, b, c}
	private Publisher<GUILineMsg> publisher;

	public LeastSquareLine(Publisher<GUILineMsg> _publisher) {
		publisher = _publisher;
		reset();
	}

	public void reset() {
		sumX = 0;
		sumY = 0;
		sumXX = 0;
		sumYY = 0;
		sumXY = 0;

		nPoints = 0;
		lineDirty = true;
		line = new double[] {};
	}

	public void addPoint(double x, double y) {
		sumX += x;
		sumY += y;
		sumXX += x * x;
		sumYY += y * y;
		sumXY += x * y;

		nPoints++;
		lineDirty = true;
	}

	public double[] getLine() {
		if (lineDirty) {
			if (nPoints >= 2) {
				double d = sumXX * sumYY - sumXY * sumXY;
				if (d != 0) {
					double a = (sumX * sumYY - sumY * sumXY) / d;
					double b = (sumY * sumXX - sumX * sumXY) / d;
					double ln = Math.pow(a * a + b * b, 0.5);
					line = new double[] {a / ln, b / ln, -1 / ln};
					lineDirty = false;
				}
			}
		}
		return line;
	}

	public double getNPoints() {
		return nPoints;
	}

	public double getDistance(double x, double y) {
		double[] line = getLine();
		if (line.length > 0) {
			return Math.abs(line[0] * x + line[1] * y + line[2]);
		} else {
			throw new RuntimeException();
		}
	}

	/**
	 * Gets the point in the line nearest to pose
	 * @param pose: pose array of the point we're interested in.
	 */
	public double[] getNearestPoint(double[] pose) {
		getLine();
		if(line.length == 0) {
			throw new RuntimeException();
		}
		double a = line[0];
		double b = line[1];
		double c = line[2];
		double d = a*pose[g.X]+b*pose[g.Y]+c;
		return new double[]{pose[g.X]-d*a,pose[g.Y]-d*b};
	}

	public void publishLine() {
		GUILineMsg linePlot = new GUILineMsg();
		ColorMsg linePlotColor = new ColorMsg();
		double[] line = getLine();
		if (line.length > 0) {
			linePlot.lineA = line[0];
			linePlot.lineB = line[1];
			linePlot.lineC = line[2];
			linePlotColor.r = 0;
			linePlotColor.g = 150;
			linePlotColor.b = 0;
			linePlot.color = linePlotColor;
			publisher.publish(linePlot);
		}
	}

	public double getAngleToLine(double angle) {
		if(line.length == 0) {
			throw new RuntimeException();
		}
		double a = line[0];
		double b = line[1];
		double lineAngle = Math.atan2(a,-b);
		double diff = Math.atan2(Math.sin(angle-lineAngle),Math.cos(angle-lineAngle)); // lazy conversion to [-pi, pi]
		return diff;
	}
}

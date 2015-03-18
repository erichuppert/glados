package Grasping;
import java.awt.Color;
import java.util.HashMap;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import java.lang.Math;

public class VisualServo implements Runnable {
    private static final double ROTO_VELO_GAIN = 1.0;
    private static final double EPSILON = 0.05;
	private int pixelCount;
	private Image debugImage;

    @Override
    public synchronized void run() {
		while (true) {
			Image image = g.getCamera();
			if (image == null) {
				continue;
			}
			debugImage = new Image(image.getWidth(), image.getHeight());
			// this will be null if there is not a detected centroid in the image
			//
			double[] centroid = getCentroid(image);
			g.pubs.setDebugImage(debugImage);
			if (centroid != null) {
				// use a proportional controller to rotate to the object
				//
				double alignmentError = (centroid[g.X] - image.getWidth()/2) / image.getWidth();
				//System.err.printf("Found object, error to center is %.2f\nf", alignmentError);
				if (Math.abs(alignmentError) > EPSILON) {
					double rv = -1 * ROTO_VELO_GAIN * alignmentError;
					g.pubs.setMotorVelocities(0, rv);
				} else {
					// if we are aligned, then consider ourselves done with aligning
					//
					this.notifyAll();
					return;
				}
			} else {
				g.pubs.setMotorVelocities(0, 0);
			}
		}
    }

    private static boolean blobPixel(int r, int g, int b, double saturationThresh, double brightnessThresh) {
        float[] hsbvals = {0, 0, 0};
        Color.RGBtoHSB(r, g, b, hsbvals);
        float hue = hsbvals[0];
        float saturation = hsbvals[1];
        float brightness = hsbvals[2];
        return saturation > saturationThresh && brightness > brightnessThresh && hue > 0.2 && hue < 0.7;
    }

    public double getDistanceToBlob() {
		double m = 134.998991;
		double b = -0.239328;
		return  Math.sqrt(Math.max(0.0,m*1.0/pixelCount + b));
    }



    private static String getBallColor(int r, int g, int b) {
        float[] hsb = {0, 0, 0};
        Color.RGBtoHSB(r, g, b, hsb);
        float hue = hsb[0];
        if (hue > 0.99 || hue < 0.1) {
            return "red";
        } else if (hue < 0.2) {
            return "orange";
        } else if (hue < 0.20) {
            return "yellow";
        } else if (hue < 0.7) {
            return "green";
        } else {
            return "blue";
        }
    }

    private static byte[] getSaturatedColor(int r, int g, int b) {
		float[] out;
		String ballColor = getBallColor(r, g, b);
		if (ballColor.equals("red")) return new byte[] {(byte) 255,0,0};
		if (ballColor.equals("orange")) return new byte[]{(byte) 255, (byte) 155, 0};
		if (ballColor.equals("green")) return new byte[]{0, (byte) 255, 0};
		if (ballColor.equals("blue")) return new byte[]{0, 0, (byte) 255};
		if (ballColor.equals("yellow")) return new byte[] {(byte) 255,(byte) 255,0};
		return new byte[] {(byte) 255, (byte) 255, (byte) 255};

    }

    private static final double saturationThresh = 0.5;
    private static final double brightnessThresh = 0.15;
    private static final int pixelThresh = 100;

	public VisualServo() {
		g.vs = this;
	}

    private double[] getCentroid(Image image) {
        if (image == null) {
            System.err.println("The image is null");
            return null;
        }
        pixelCount = 0;
        double[] centroid = {0,0};
        for (int x=0; x < image.getWidth(); x++) {
            for (int y=0; y < image.getHeight(); y++) {
                int pix = image.getPixel(x,y);
                int r = (int)Image.pixelRed(pix) & 0xFF;
                int g = (int)Image.pixelGreen(pix) & 0xFF;
                int b = (int)Image.pixelBlue(pix) & 0xFF;
                if (blobPixel(r,g,b,saturationThresh, brightnessThresh)) {
					byte[] saturatedColor =  getSaturatedColor(r, g, b);
					debugImage.setPixel(x, y, saturatedColor[0], saturatedColor[1],
										saturatedColor[2]);
					pixelCount++;
					centroid[0] += x;
					centroid[1] += y;
                } else {
					debugImage.setPixel(x,y, (byte) r, (byte) g, (byte) b);
				}
            }
        }
        //System.err.printf("Pixel count is %d\n", pixelCount);
        if (pixelCount > pixelThresh) {
            centroid[0] /= pixelCount;
            centroid[1] /= pixelCount;
            return centroid;
        } else {
            return null;
        }
    }
}

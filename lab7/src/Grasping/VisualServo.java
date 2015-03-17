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

    private Image image;
    
    public VisualServo(Image _image) {
        image = _image;
    }

    private static final double ROTO_VELO_GAIN = 0.4;
    private static final double EPSILON = 0.02;

    @Override
    public synchronized void run() {
        // this will be null if there is not a detected centroid in the image
        //
        double[] centroid = getCentroid();
        if (centroid != null) {
            // use a proportional controller to rotate to the object
            //
            double alignmentError = (image.getWidth() - centroid[g.X]) / image.getWidth();
            System.err.println("Alignment error is " + alignmentError);
            if (Math.abs(alignmentError) > EPSILON) {
                double rv = ROTO_VELO_GAIN * alignmentError;
                System.err.println("Setting rv to " + rv);
                g.pubs.setMotorVelocities(0, rv);
            } else {
                // if we are aligned, then consider ourselves done with aligning
                //
                this.notifyAll();
            }
        } else {
            System.err.println("No blob in view");
        }
    }
	
    private static boolean blobPixel(int r, int g, int b, double saturationThresh, double brightnessThresh) {
        float[] hsbvals = {0, 0, 0};
        Color.RGBtoHSB(r, g, b, hsbvals);
        float hue = hsbvals[0];
        float saturation = hsbvals[1];
        float brightness = hsbvals[2];
        return saturation > saturationThresh && brightness > brightnessThresh;
            //&& hue < 0.3;
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

    private static final double saturationThresh = 0.5;
    private static final double brightnessThresh = 0.15;
    private static final int pixelThresh = 100;
    
    private double[] getCentroid() {
        if (image == null) {
            System.err.println("The image is null");
            return null;
        }
        int pixelCount = 0;
        double[] centroid = {0,0};
        for (int x=0; x < image.getWidth(); x++) {
            for (int y=0; y < image.getHeight(); y++) {
                int pix = image.getPixel(x,y);
                int r = (int)Image.pixelRed(pix) & 0xFF;
                int g = (int)Image.pixelGreen(pix) & 0xFF;
                int b = (int)Image.pixelBlue(pix) & 0xFF;
                if (blobPixel(r,g,b,saturationThresh, brightnessThresh)) {
                    pixelCount++;
                    centroid[0] += x;
                    centroid[1] += y;
                }
            }
        }
        System.err.printf("Pixel count is %d\n", pixelCount);
        if (pixelCount > pixelThresh) {
            centroid[0] /= pixelCount;
            centroid[1] /= pixelCount;
            return centroid;
        } else {
            return null;
        }
    }
}

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

    private static final double ROTO_VELO_GAIN = 0.4;
    private static final double EPSILON = 0.02;
    
    public synchronized void run(Image image) {
        // this will be null if there is not a detected centroid in the image
        //
        double[] centroid = getCentroid(image);
        if (centroid != null) {
            // use a proportional controller to rotate to the object
            //
            double alignmentError = (image.getWidth() - centroid[g.x]) / im.width();
            if (Math.abs(alignmentError) > EPSILON) {
                double rv = ROTO_VELO_GAIN * alignmentError;
                g.pubs.setMotorVelocity(0, rv);
            } else {
                // if we are aligned, then consider ourselves done with aligning
                //
                this.notifyAll();
            }
        } else {
            System.err.print("No blob in view");
        }
    }
	
    private static boolean blobPixel(int r, int g, int b, double saturationThresh, double brightnessThresh) {
        float[] hsbvals = {0, 0, 0};
        Color.RGBtoHSB(r, g, b, hsbvals);
        float hue = hsbvals[0];
        float saturation = hsbvals[1];
        float brightness = hsbvals[2];
        return saturation > saturationThresh && brightness > brightnessThresh && hue < 0.1;
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

    private static final double saturationThresh;
    private static final double brightnessThresh;
    private static final int pixThresh;
    private static double[] getCentroid(Image image) {
        int pixelCount = 0;
        float[] centroid = {0,0};
        for (int x=0; x < im.getWidth(); x++) {
            for (int y=0; y < im.getHeight(); y++) {
                int pix = im.getPixel(x,y);
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
        if (pixelCount > pixelThresh) {
            centroid[0] /= pixelCount;
            centroid[1] /= pixelCount;
            return centroid;
        } else {
            return null;
        }
    }
}

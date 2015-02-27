package VisualServo;

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

/**
 * BlobTracking performs image processing and tracking for the VisualServo
 * module.  BlobTracking filters raw pixels from an image and classifies blobs,
 * generating a higher-level vision image.
 *
 * @author previous TA's, prentice
 */
public class BlobTracking {
    Publisher<MotionMsg> publisher;
    public Node node;
    
    protected int stepCounter = 0;
    protected double lastStepTime = 0.0;
	
    private double saturation_threshold = 0.95;

    public int width;
    public int height;


    // Variables used for velocity controller that are available to calling
    // process.  Visual results are valid only if targetDetected==true; motor
    // velocities should do something sane in this case.
    public boolean targetDetected = false; // set in blobPresent()
    public double centroidX = 0.0; // set in blobPresent()
    public double centroidY = 0.0; // set in blobPresent()
    public double targetArea = 0.0; // set in blobPresent()
    public double targetRange = 0.0; // set in blobFix()
    public double targetBearing = 0.0; // set in blobFix()

    /**
     * <p>Create a BlobTracking object</p>
     *
     * @param width image width
     * @param height image height
     */
    public BlobTracking(int width, int height, Node node) {
	this.node = node;
	publisher = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
	this.width = width;
	this.height = height;

    }





    /**
     * <p>Computes frame rate of vision processing</p>
     */
    private void stepTiming() {
	double currTime = System.currentTimeMillis();
	stepCounter++;
	// if it's been a second, compute frames-per-second
	if (currTime - lastStepTime > 1000.0) {
	    //double fps = (double) stepCounter * 1000.0
	    // / (currTime - lastStepTime);
	    //System.err.println("FPS: " + fps);
	    stepCounter = 0;
	    lastStepTime = currTime;
	}
    }

    /**
     * <p>Segment out a blob from the src image (if a good candidate exists).</p>
     *
     * <p><code>dest</code> is a packed RGB image for a java image drawing
     * routine. If it's not null, the blob is highlighted.</p>
     *
     * @param src the source RGB image, not packed
     * @param dest the destination RGB image, packed, may be null
     */
    public void apply(Image src, Image dest) {

	stepTiming(); // monitors the frame rate
	//
	// Begin Student Code
	// Histogram.getHistogram(src,dest,true);
	int middle = src.getPixel(src.getWidth()/2, src.getHeight()/2);
	int r = (int) Image.pixelRed(middle) & 0xFF;
	int g = (int) Image.pixelGreen(middle) & 0xFF;
	int b = (int) Image.pixelBlue(middle) & 0xFF;
	float[] hsb = Color.RGBtoHSB(r,g,b, null);
	//System.out.printf("Red: %d\tGre: %d\tBlu: %d\tHue: %.2f\tSat: %.2f\n", r,g,b, hsb[0],hsb[1],hsb[2]);
	//		int tempSatThresh = 0;
	//		double thresh = 0;
	//		int tempBrightThresh = 0;
	//		double brightThresh = 0;
	//		int pixThresh = 0;
	//		int ch;
	//		System.out.println("========");
	//		try {
	//			while ((ch = System.in.read ()) != '\n')
	//				if (ch >= '0' && ch <= '9')
	//				{
	//					tempSatThresh *= 10;
	//					tempSatThresh += ch - '0';
	//				}
	//				else
	//					break;
	//			thresh = (double) tempSatThresh / 100.0;
	//			System.out.println("Saturation threshold is now " + thresh);
	//			while ((ch = System.in.read ()) != '\n')
	//				if (ch >= '0' && ch <= '9')
	//				{
	//					tempBrightThresh *= 10;
	//					tempBrightThresh += ch - '0';
	//				}
	//				else
	//					break;
	//			brightThresh = (double) tempBrightThresh/100.0;
	//			System.out.println("Bright threshold is now " + brightThresh);
	//			System.out.println("Blob presence: " + blobPresent(src, thresh, brightThresh, pixThresh));
	//			saturateBallPixels(src, dest, thresh, brightThresh);
	//		} catch (Exception e) {
	//			
	//		}
	//		int pixelThreshold = 900; 
	double saturationThreshold = 0.5;
	double brightnessThreshold = 0.15;
	//saturateBallPixels(src, dest, saturationThreshold, brightnessThreshold);
	blobPresent(src, dest, saturationThreshold, brightnessThreshold, 50);
	// End Student Code
    }
	
    private static boolean blobPixel(int r, int g, int b, double saturationThresh, double brightnessThresh) {
	float[] hsbvals = {0, 0, 0};
	Color.RGBtoHSB(r, g, b, hsbvals);
	float hue = hsbvals[0];
	float saturation = hsbvals[1];
	float brightness = hsbvals[2];
	return saturation > saturationThresh && brightness > brightnessThresh && hue < 0.1;
    }

    private static byte[] getSaturatedColor(int r, int g, int b, double threshold) {
	float[] out;
	String ballColor = getBallColor(r, g, b);
	if (ballColor.equals("red")) return new byte[] {(byte) 255,0,0};
	if (ballColor.equals("orange")) return new byte[]{(byte) 255, (byte) 155, 0};
	if (ballColor.equals("green")) return new byte[]{0, (byte) 255, 0};
	if (ballColor.equals("blue")) return new byte[]{0, 0, (byte) 255};
	if (ballColor.equals("yellow")) return new byte[] {(byte) 255,(byte) 255,0};
	return new byte[] {(byte) 255, (byte) 255, (byte) 255};

    }
	
    private static byte[] getGreyColor(int r, int g, int b) {
	int averageVal = (int) Math.round((r + g + b)/3.0);
	return new byte[] {(byte) averageVal, (byte) averageVal, (byte) averageVal};
    }
	
	
    private static void saturateBallPixels(Image input, Image output, double saturationThresh, double brightnessThresh) {
	for (int x=0; x < input.getWidth(); x++) {
	    for (int y=0; y < input.getHeight(); y++) {
		byte[] newPixelVal;
		int pix = input.getPixel(x,y);
		int r = (int)Image.pixelRed(pix) & 0xFF;
		int g = (int)Image.pixelGreen(pix) & 0xFF;
		int b = (int)Image.pixelGreen(pix) & 0xFF;
		if (blobPixel(r,g,b, saturationThresh, brightnessThresh)) {
		    newPixelVal = getSaturatedColor(r, g, b, saturationThresh);
		} else {
		    newPixelVal = getGreyColor(r, g, b);
		}
		output.setPixel(x, y, newPixelVal[0], newPixelVal[1], newPixelVal[2]);
	    }
	}
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
	
    private void blobPresent(Image im, Image dest, double saturationThresh, double brightnessThresh, int pixThresh) {
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

	double tv = 0;
	double rv = 0;
	centroid[0] /= pixelCount;
	centroid[1] /= pixelCount;
	if (pixelCount > pixThresh) {
	    float area = pixelCount;
	    int diameter = (int)((double) 2.0*Math.sqrt(area/Math.PI));
	    System.out.printf("%d\n", pixelCount);
	    int height = im.getHeight();
	    int width = im.getWidth();
	    int lx = (int)centroid[0]-diameter/2;
	    int ly = (int)centroid[1]-diameter/2;
	    for (int dx=0; dx < diameter; ++dx) {
	    	for (int dy=0; dy < diameter; ++dy) {
		    if(ly+dy < height)
			dest.setPixel(lx,ly+dy,(byte)255,(byte)0,(byte)0);
		    if(lx+dx < width)
			dest.setPixel(lx+dx,ly,(byte)255,(byte)0,(byte)0);
		    if(lx+diameter < width && ly+dy < height)
			dest.setPixel(lx+diameter,ly+dy,(byte)255,(byte)0,(byte)0);
		    if(lx+diameter-dx < width && ly+diameter < height)
			dest.setPixel(lx+diameter-dx,ly+diameter,(byte)255,(byte)0,(byte)0);
	    	}
	    }
	    for (int delta=-3; delta <= 3; ++delta) {
		dest.setPixel((int)centroid[0], (int)centroid[1]+delta,(byte)255,(byte)0,(byte)0);
		dest.setPixel((int)centroid[0]+delta, (int)centroid[1],(byte)255,(byte)0,(byte)0);
	    }

	}
	
	MotionMsg msg= new MotionMsg();
	msg.translationalVelocity = tv;
	msg.rotationalVelocity = rv;
	publisher.publish(msg);
    }
}

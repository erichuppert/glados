package GlobalNavigation;


import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.parameter.ParameterTree;
import org.ros.message.lab6_msgs.*;
import org.ros.message.lab5_msgs.*;
import org.ros.message.rss_msgs.*;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;

import java.io.*;
import java.awt.Color;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;


public class GlobalNavigation implements NodeMain{

	//publishers
	private Publisher<org.ros.message.lab6_msgs.GUIRectMsg> guiRectPub;
    private Publisher<org.ros.message.lab6_msgs.GUIPolyMsg> guiPolyPub;
    private Publisher<org.ros.message.lab5_msgs.GUIEraseMsg> guiErasePub;
    private Publisher<org.ros.message.lab5_msgs.GUISegmentMsg>guiSegPub;
    private Publisher<org.ros.message.lab5_msgs.GUIPointMsg>guiPointPub;

    private String mapFileName;
    private PolygonMap polygonMap;
    private ParameterTree paramTree;
    
    private boolean shutdown;
    

    public GlobalNavigation(){
    	
    }
    
	public void onStart(Node node){
		guiRectPub = node.newPublisher("/gui/Rect", "lab6_msgs/GUIRectMsg");
		guiPolyPub = node.newPublisher("/gui/Poly", "lab6_msgs/GUIPolyMsg");
		guiErasePub = node.newPublisher("/gui/Erase", "lab5_msgs/GUIEraseMsg");
		guiSegPub =  node.newPublisher("/gui/Segment", "lab5_msgs/GUISegmentMsg");
		guiPointPub = node.newPublisher("/gui/Point", "lab5_msgs/GUIPointMsg");
		
		paramTree = node.newParameterTree();
		mapFileName = paramTree.getString(node.resolveName("~/mapFileName"));
		try {
			polygonMap = new PolygonMap(mapFileName);
			Thread.sleep(30000);
			displayMap();
		} catch (Exception e) {
			System.out.println(e);
			throw new RuntimeException(e.getMessage());
		}
    }
    
	public void onShutdown(Node node){
		synchronized(this) {
			shutdown = true;
		}

		if(node != null){
			node.shutdown();
		}
	}

	public void onShutdownComplete(Node node) {
	}

    
	public GraphName getDefaultNodeName() {
		return new GraphName("/rss/GlobalNavigation");
	}
	
	public void handle(BumpMsg arg0) {
		
	}
	
	public void handle(OdometryMsg arg0) {
	}

	public void run() {
	}
	
	private void displayMap() {
		// draw the robot starting point
		//
		Point2D.Double robotStart = polygonMap.getRobotStart();
		drawPoint(robotStart.getX(), robotStart.getY(), Color.RED);
		
		// draw the robot goal
		//
		Point2D.Double robotGoal = polygonMap.getRobotGoal();
		drawPoint(robotGoal.getX(), robotGoal.getY(), Color.GREEN);
		
		// draw the rectangular boundaries of the world
		//
		Rectangle2D.Double worldRect = polygonMap.getWorldRect();
		drawRectangle(worldRect, Color.BLACK);
		
		
		// draw obstacles onto the GUI
		for (int i = 0; i < polygonMap.obstacles.size(); i++) {
			PolygonObstacle obstacle = polygonMap.obstacles.get(i);
			drawPolygon(obstacle);
		}
	}
	
	/**
	 * Publish a point message to the GUI
	 * @param x the x value of the point on the GUI
	 * @param y the y value of the point on the GUI
	 * @param color the color of the point on the GUI
	 */
	private void drawPoint(double x, double y, Color color) {
		/**
		 * 	GUIPointMsg attributes:
		 *  float64 x
			float64 y
			int64 shape
			ColorMsg color
		 */
		GUIPointMsg pointMsg = new GUIPointMsg();
		pointMsg.x = (float) x;
		pointMsg.y = (float) y;
		pointMsg.shape = 1;
		pointMsg.color = GUIHelpers.colorMessage(color);
		guiPointPub.publish(pointMsg);
	}
	
	
	/**
	 * Publish a rectangle to the GUI
	 * @param rectangle the rectangle to be drawn
	 * @param color the color of the rectangle 	
	 */
	private void drawRectangle(Rectangle2D.Double rectangle, Color color) {

		GUIRectMsg guiRectMsg = new GUIRectMsg();
		fillRectMsg(guiRectMsg, rectangle, color, false);
		guiRectPub.publish(guiRectMsg);
	}
	
	/**
	 * Determines if the GUI should display obstacles as filled
	 */
	private static final boolean FILL_GUI_OBSTACLES = true;
	
	private void drawPolygon(PolygonObstacle obstacle) {
		/* 
		PolygonMsg attributes
		
		lab5_msgs/ColorMsg c
		int32 numVertices
		float32[] x
		float32[] y
		int32 closed
		int32 filled */
		GUIPolyMsg polyMsg = new GUIPolyMsg();
		fillPolyMsg(polyMsg, obstacle, obstacle.color, FILL_GUI_OBSTACLES, obstacle.closed);
		guiPolyPub.publish(polyMsg);
	}
	
	
	public static void fillRectMsg(GUIRectMsg rectMsg, Rectangle2D.Double rectangle, Color color, boolean filled) {
		/**
		 *  GUIRectMsg attributes:
		 *  lab5_msgs/ColorMsg c
			float32 x
			float32 y
			float32 width
			float32 height
			int32 filled
		 */
		rectMsg.x = (float) rectangle.x;
		rectMsg.y = (float) rectangle.y;
		rectMsg.height = (float) rectangle.height;
		rectMsg.width = (float) rectangle.width;
		rectMsg.filled = filled ? 1 : 0;
		if (color != null) {
			rectMsg.c = GUIHelpers.colorMessage(color);
		}
	};
	
	public static void fillPolyMsg(GUIPolyMsg polyMsg, PolygonObstacle obstacle, Color color, boolean filled, boolean closed) {
		polyMsg.c = GUIHelpers.colorMessage(color);
		polyMsg.closed = closed ? 1 : 0;
		polyMsg.filled = filled ? 1 : 0;
		List<Point2D.Double> obstacleVertices = obstacle.getVertices();
		int numObstacleVertices = obstacleVertices.size();
		polyMsg.numVertices = numObstacleVertices;
		float[] xPoints = new float[numObstacleVertices];
		float[] yPoints = new float[numObstacleVertices];
		for (int i=0; i< numObstacleVertices; i++) {
			xPoints[i] = (float) obstacleVertices.get(i).getX();
			yPoints[i] = (float) obstacleVertices.get(i).getY();
		}
	}
	

}


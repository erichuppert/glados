package GlobalNavigation;


import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.parameter.ParameterTree;
import org.ros.message.lab6_msgs.*;
import org.ros.message.lab5_msgs.*;
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
		paramTree = node.newParameterTree();
		mapFileName = paramTree.getString(node.resolveName("˜/mapFileName"));
		polygonMap = new PolygonMap(new File(mapFileName));
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
		/**
		 *  GUIRectMsg attributes:
		 *  lab5_msgs/ColorMsg c
			float32 x
			float32 y
			float32 width
			float32 height
			int32 filled
		 */
		GUIRectMsg guiRectMsg = new GUIRectMsg();
		guiRectMsg.x = (float) rectangle.x;
		guiRectMsg.y = (float) rectangle.y;
		guiRectMsg.height = (float) rectangle.height;
		guiRectMsg.width = (float) rectangle.width;
		guiRectMsg.filled = 0; // we don't want our whole map filled in
		guiRectMsg.color = GUIHelpers.colorMessage(color);
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
		polyMsg.c = GUIHelpers.colorMessage(obstacle.color);
		polyMsg.closed = obstacle.closed ? 1 : 0;
		polyMsg.filled = FILL_GUI_OBSTACLES ? 1 : 0;
		List<Point2D.Double> obstacleVertices = obstacle.getVertices();
		int numObstacleVertices = obstacleVertices.size();
		polyMsg.numVertices = numObstacleVertices;
		float[] xPoints = new float[numObstacleVertices];
		float[] yPoints = new float[numObstacleVertices];
		for (int i=0; i< numObstacleVertices; i++) {
			xPoints[i] = (float) obstacleVertices[i].getX();
			yPoints[i] = (float) obstacleVertices[i].getY();
		}
		guiPolyPub.publish(polyMsg);
	}

}


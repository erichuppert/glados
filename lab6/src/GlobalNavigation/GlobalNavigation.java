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

    public GlobalNavigation(){}

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
			Runnable myRunnable = new Runnable(){

			     public void run(){
			    	 try {
			    		 Thread.sleep(4000);
			    	 } catch (Exception e) {
			    		 throw new RuntimeException();
			    	 }
			        System.out.println("Runnable running");
			        displayMap();
			     }
			   };
			   Thread thread = new Thread(myRunnable);
			   thread.start();
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
		for (PolygonObstacle obstacle: polygonMap.getObstacles()) {
			drawPolygon(obstacle);
		}

		List<PolygonObstacle> obstacles = new CSpace().envConfSpace(polygonMap);
		for (PolygonObstacle obstacle : obstacles) {
			drawPolygon(obstacle);
		}
		VisibilityGraph g = new VisibilityGraph(obstacles, robotStart, robotGoal);
		for (GraphNode<Point2D.Double> n: g.getAllNodes()) {		
			System.err.printf("\n\nHere\n\n\n\n", g.getAllNodes().size());
			Point2D.Double p = n.getValue();
			List<GraphNode<Point2D.Double>> neighbors = new ArrayList<GraphNode<Point2D.Double>>(n.getNeighbors());
			System.out.printf("This node has %d neightbors\n\n", neighbors.size());
			for (GraphNode<Point2D.Double> neigh: neighbors) {
				Point2D.Double pn = neigh.getValue();
				drawSegment(p,pn,Color.RED);
			}
			drawPoint(p.getX(), p.getY(), Color.BLUE);
		}

		// // Motion plan
		// //
		// AStar<Point2D.Double> planner = new AStar<Point2D.Double>(g.getVisibilityGraph());
		// List<WaypointNode> path = planner.search(new Predicate<Point2D.Double>() {
		// 		@Override
		// 		public boolean test(Point2D.Double value) {
		// 			double dx = value.getX()-robotGoal.getX();
		// 			double dy = value.getX()-robotGoal.getY();
		// 			return (dx*dx + dy*dy) <= 0.05;
		// 		}
		// 	});
		// WaypointNode prev = path.get(0);
		// for (WaypointNode n : path) {
		// 	drawSegment(prev.getValue(), n.getValue(), Color.BLUE);
		// 	prev = n;
		// }
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

	/**
	 * Draws a line on the GUI of the given color
	 * @param p1 the first end point of the segment to be drawn
	 * @param p2 the second end point of the segment to be drawn
	 * @param color the color of the line
	 */
	public void drawSegment(Point2D.Double p1, Point2D.Double p2, Color color) {
		GUISegmentMsg segMesg = new GUISegmentMsg();
		/**
		 *  float64 startX
			float64 endX
			float64 startY
			float64 endY
			ColorMsg color
		 */
		segMesg.startX = (float) p1.getX();
		segMesg.startY = (float) p1.getY();
		segMesg.endX = (float) p2.getX();
		segMesg.endY = (float) p2.getY();
		segMesg.color = GUIHelpers.colorMessage(color);
		System.out.printf("Added segment from (%d,%d) to (%d,%d)", p1.getX(), p1.getY(), p2.getX(), p2.getY());
		guiSegPub.publish(segMesg);
	}
	
	public static void fillPolyMsg(GUIPolyMsg polyMsg, PolygonObstacle obstacle, Color color, boolean filled, boolean closed) {
		/*
		PolygonMsg attributes

		lab5_msgs/ColorMsg c
		int32 numVertices
		float32[] x
		float32[] y
		int32 closed
		int32 filled */
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
		polyMsg.x = xPoints;
		polyMsg.y = yPoints;
	}
}


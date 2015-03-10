package src.GlobalNavigation;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

public class VisibilityGraph {
	
	private List<PolygonObstacle> obstacles;
	private Point2D.Double start;
	private Point2D.Double goal;
	
	public VisibilityGraph(List<PolygonObstacle> _obstacles, Point2D.Double _start, Point2D.Double _goal) {
		obstacles = _obstacles;
		goal = _goal;
		start = _start;
		
	}
	
	
	/**
	 * Gets the visibility graph 
	 * @return the start node of the graph
	 */
	public GraphNode<Point2D.Double> getVisibilityGraph() {
		// get all the nodes in our graph
		//
		List<GraphNode<Point2D.Double>> allNodes = getAllNodes(obstacles);
		allNodes.add(new GraphNode<Point2D.Double>(start));
		allNodes.add(new GraphNode<Point2D.Double>(goal));
		
		// iterate over all pairs of nodes to find out which pairs of points share visibility
		//
		for (int i=0; i < allNodes.size(); i++) {
			GraphNode<Point2D.Double> node1 = allNodes.get(i);
			Point2D.Double p1 = node1.getValue();
			for (int j=0; j < allNodes.size(); j++) {
				GraphNode<Point2D.Double> node2 = allNodes.get(j);
				Point2D.Double p2 = node2.getValue();
				
				// create a line from the pair of points to test against all obstacles for an intersection
				//
				Line2D.Double line = new Line2D.Double(p1, p2);
				if (!lineIntersectsAnyObstacle(line) && !p1.equals(p2)) {
					// if the two nodes are visible from one another. then they are neighbors in the graph
					//
					node1.addNeighbor(node2);
					node2.addNeighbor(node1);
				}
			}
		}
		
		return graphStart;
	}
	
	/**
	 * Determines if the given line intersects with any obstacles in the obstacle list
	 * @param line the line we use to determine if intersection exists
	 * @return true if the given line intersects with any of the obstacles
	 */ 
	private boolean lineIntersectsAnyObstacle(Line2D.Double line) {
		for (int k = 0; k < obstacles.size(); k++) {
			PolygonObstacle obstacle = obstacles.get(k);
			if (obstacle.lineIntersects(line)) {
				return false;
			}
		} 
		return true;
	}
	
	/**
	 * Get all the nodes in the visibility graph
	 * @return a list of all the nodes that make up the nodes in the visibility graph
	 */
	private List<GraphNode<Point2D.Double>> getAllNodes() {
		List<GraphNode<Point2D.Double>> nodes;
		for (int i=0; i<obstacles.size();i++) {
			// add every vertex in each obstacle to the nodes list
			//
			List<Point2D.Double> vertices = obstacles.get(i).getVertices();
			for (int j=0; j < vertices.size(); j++) {
				nodes.add(new GraphNode<Point2D.Double>(vertices.get(j)));
			}
		}
	}
}

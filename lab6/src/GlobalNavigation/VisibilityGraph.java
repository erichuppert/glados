package GlobalNavigation;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

public class VisibilityGraph {
	private List<PolygonObstacle> obstacles;
	private Point2D.Double start;
	public Point2D.Double goal;
	public List<WaypointNode> nodes;
	WaypointNode graphStart = null;
	WaypointNode goalNode = null;
	private boolean graphConstructed = false;

	public VisibilityGraph(List<PolygonObstacle> _obstacles, Point2D.Double _start, Point2D.Double _goal) {
		obstacles = _obstacles;
		goal = _goal;
		start = _start;
		graphStart = new WaypointNode(start);
		goalNode = new WaypointNode(goal);
		nodes = getAllNodes();
		getVisibilityGraph();
	}

	/**
	 * Gets the visibility graph
	 * @return the start node of the graph
	 */
	private WaypointNode getVisibilityGraph() {
		// get all the nodes in our graph
		//
		if (graphConstructed) {
			return graphStart;
		}

		// iterate over all pairs of nodes to find out which pairs of points share visibility
		//
		int c = 0;
		for (GraphNode<Point2D.Double> node1 : nodes) {
			Point2D.Double p1 = node1.getValue();
			for (GraphNode<Point2D.Double> node2 : nodes) {
				c++;
				if (node1.equals(node2)) {
					continue;
				}
				Point2D.Double p2 = node2.getValue();
				// create a line from the pair of points to test against all obstacles for an intersection
				//
				Line2D.Double line = new Line2D.Double(p1, p2);
				if (!lineIntersectsAnyObstacle(line)) {
					// if the two nodes are visible from one another. then they are neighbors in the graph
					//
					node1.addNeighbor(node2);
					node2.addNeighbor(node1);
				}
			}
		}
		graphConstructed = true;
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
				return true;
			}
		}
		return false;
	}

	/**
	 * Get all the nodes in the visibility graph
	 * @return a list of all the nodes that make up the nodes in the visibility graph
	 */
	private List<WaypointNode> getAllNodes() {
		List<WaypointNode> nodes = new ArrayList<WaypointNode>();
		for (PolygonObstacle obstacle : obstacles) {
			// add every vertex in each obstacle to the nodes list
			// If it's not contained in any other obstacle.
			//
			List<Point2D.Double> vertices = obstacle.getVertices();

			// Modify vertices so they're not contained within the obstacle.
			//
			List<Point2D.Double> modVertices = new ArrayList<Point2D.Double>();
			double modScalar = 0.01;
			for(int i = 0; i < vertices.size(); ++i) {
				int prev = (vertices.size() + i - 1) % vertices.size();
				int next = (i+1) % vertices.size();
				Point2D.Double prevPoint = vertices.get(prev);
				Point2D.Double current = vertices.get(i);
				Point2D.Double nextPoint = vertices.get(next);
				Vector2D p1 = new Vector2D(prevPoint.getX(), prevPoint.getY());
				Vector2D p2 = new Vector2D(current.getX(), current.getY());
				Vector2D p3 = new Vector2D(nextPoint.getX(), nextPoint.getY());
				Vector2D v1 = p2.minus(p1).unitVector();
				Vector2D v2 = p2.minus(p3).unitVector();
				Vector2D delta = v2.plus(v1).unitVector();
				Vector2D modV = p2.plus(delta.scalarMult(modScalar));
				modVertices.add(new Point2D.Double(modV.getX(),modV.getY()));
			}

			for(Point2D.Double vertex : modVertices) {
				boolean add = true;
				for (PolygonObstacle other : obstacles) {
					if (other.contains(vertex) && other != obstacle) {
						add = false;
					}
				}
				if (add) {
					nodes.add(new WaypointNode(vertex));
				}
			}
		}
		nodes.add(graphStart);
		nodes.add(goalNode);
		return nodes;
	}
}

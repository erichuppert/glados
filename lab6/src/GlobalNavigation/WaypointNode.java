package GlobalNavigation;
import java.awt.geom.Point2D;

public class WaypointNode extends GraphNode<Point2D.Double> {
	public WaypointNode(Point2D.Double _value) {
		super(_value);
	}

	public double costToNode(GraphNode<Point2D.Double> other) {
		return this.value.distance(other.getValue());
	}
}

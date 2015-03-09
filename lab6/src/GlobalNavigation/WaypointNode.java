package GlobalNavigation;

public class WaypointNode extends GraphNode<double[]> {
	public WaypointNode(double[] _value) {
		super(_value);
	}

	public double costToNode(GraphNode<double[]> other) {
		double[] src = this.value;
		double[] dest = other.getValue();
		double dx = dest[g.X]-src[g.X];
		double dy = dest[g.Y]-src[g.Y];
		return Math.sqrt(dx*dx + dy*dy);
	}
}

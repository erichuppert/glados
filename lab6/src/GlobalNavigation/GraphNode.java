package GlobalNavigation;
import java.util.Set;
import java.util.HashSet;
import java.util.Collection;

/**
 * Implements a Node class, which can be used to represent a graph.
 * Each node has a value, and the graph has a distance function associated with it.
 */
public abstract class GraphNode<V> {
	protected V value;
	private Set<GraphNode<V>> neighbors;

	public GraphNode(V _value) {
		value = _value;
		neighbors = new HashSet<GraphNode<V>>();
	}

	public Set<GraphNode<V>> getNeighbors() {
		return new HashSet<GraphNode<V>>(neighbors);
	}

	public boolean addNeighbors(Collection<GraphNode<V>> newNeighbors) {
		return neighbors.addAll(newNeighbors);
	}

	public boolean addNeighbor(GraphNode<V> newNeighbor) {
		System.err.printf("Adding a new neighbor\n");
		return neighbors.add(newNeighbor);
	}

	public abstract double costToNode(GraphNode<V> other);

	public V getValue() {
		return value;
	}

	public void setValue(V newValue) {
		value = newValue;
	}
}

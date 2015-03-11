package GlobalNavigation;

import java.util.Set;
import java.util.HashSet;
import java.util.PriorityQueue;
import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Map;
import java.util.HashMap;

public class AStar<V> {
	protected GraphNode<V> graph;

	public AStar(GraphNode<V> _graph) {
		graph = _graph;
	}

	private class PQEntry implements Comparable<PQEntry> {
		public double cost;
		public GraphNode<V> node;
		public PQEntry(double _cost, GraphNode<V> _node) {
			cost = _cost;
			node = _node;
		}
		public int compareTo(PQEntry o) {
			return (int)(cost - o.cost);
		}
	}


	public List<GraphNode<V>> search(Predicate<V> reachedEnd) {
		Set<GraphNode<V>> closedSet = new HashSet<GraphNode<V>>();
		PriorityQueue<PQEntry> openSet = new PriorityQueue<PQEntry>();
		Map<GraphNode<V>,Double> nodeToCost = new HashMap<GraphNode<V>,Double>();
		Map<GraphNode<V>,GraphNode<V>> nodeToParent = new HashMap<GraphNode<V>,GraphNode<V>>();

		openSet.add(new PQEntry(0.0,graph));
		nodeToCost.put(graph,0.0);
		nodeToParent.put(graph,null);
		GraphNode<V> end = null;
		
		while (!openSet.isEmpty()) {
			PQEntry entry = openSet.poll();
			double cost = entry.cost;
			GraphNode<V> current = entry.node;
			System.err.printf("This node is at %s\n", current.getValue().toString());
			if (reachedEnd.test(current.getValue())) {
				System.err.println("Found the end!");
				end = current;
				break;
			}
			if (closedSet.contains(current)) {
				continue;
			}
			closedSet.add(current);
			System.err.printf("Current node has %d neighbors\n", current.getNeighbors().size());
			for(GraphNode<V> neigh : current.getNeighbors()) {
				double neighCost = cost+current.costToNode(neigh);
				if (!nodeToCost.containsKey(neigh) || nodeToCost.get(neigh) > neighCost) {
					nodeToCost.put(neigh,neighCost);
					nodeToParent.put(neigh,current);
					openSet.add(new PQEntry(neighCost,neigh));
				}
			}
		}

		List<GraphNode<V>> path = new ArrayList<GraphNode<V>>();
		while (end != null) {
			path.add(end);
			end = nodeToParent.get(end);
		}
		Collections.reverse(path);
		return path;
	}
}

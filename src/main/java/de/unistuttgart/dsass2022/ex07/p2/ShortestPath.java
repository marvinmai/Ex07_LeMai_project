package de.unistuttgart.dsass2022.ex07.p2;


import java.util.*;


public class ShortestPath implements IShortestPath {

	private final IWeightedGraph graph;
	private final long startNode;

	/*
	 * syntactic sugar function to conveniently create shortest path objects from
	 * graphs
	 */
	public static ShortestPath calculateFor(IWeightedGraph g, long startNode) {
		return new ShortestPath(g, startNode);
	}

	/**
	 * Initializes the shortest path for weighted graph <tt>graph</tt> from starting
	 * node <tt>startNode</tt>. Calls the dijkstra(graph, startNode) method to
	 * execute the Dijkstra algorithm.
	 * 
	 * @param graph     the weighted graph
	 * @param startNode the starting node
	 */
	public ShortestPath(IWeightedGraph graph, long startNode) {
		this.graph = graph;
		this.startNode = startNode;
		dijkstra(this.graph, this.startNode);
	}

	private Map<Long, IEdge> edgeTo;
	private Map<Long, Double> distTo;
	private PriorityQueue<Node> pq;

	@Override
	public void dijkstra(IWeightedGraph graph, long startNode) {
		edgeTo = new HashMap<>(graph.numberOfNodes());
		distTo = new HashMap<>(graph.numberOfNodes());
		pq = new PriorityQueue<>(graph.numberOfNodes());

		graph.nodeIDIterator().forEachRemaining(nodeId -> {
			distTo.put(nodeId, Double.POSITIVE_INFINITY);
		});
		distTo.put(startNode, 0.0);
		Node node = graph.getNode(startNode);
		node.setDistance(0.0);
		pq.add(node);

		while (!pq.isEmpty()) {
			relax(graph, pq.poll());
		}
	}

	private void relax(IWeightedGraph graph, Node src) {
		graph.outgoingEdges(src.getID()).forEachRemaining(edge -> {
			double newDist = distTo.get(src.getID()) + edge.getWeight();
			long target = edge.getDestination();
			if (distTo.get(target) > newDist) {
				distTo.put(target, newDist);
				edgeTo.put(target, edge);

				Node node = graph.getNode(target);
				node.setDistance(newDist);
				if (!pq.contains(node)) {
					pq.add(node);
				} else {
					pq.remove(node);
					pq.add(node);
				}
			}
		});
	}

	@Override
	public double distanceTo(long destination) {
		return distTo.get(destination);
	}

	@Override
	public boolean existsPathTo(long destination) {
		return distTo.get(destination) < Double.POSITIVE_INFINITY;
	}

	@Override
	public Iterable<IEdge> pathTo(long destination) {
		Stack<IEdge> path = new Stack<>();
		for (IEdge e = edgeTo.get(destination); e != null; e = edgeTo.get(e.getSource())) {
			path.push(e);
		}
		return path;
	}
}

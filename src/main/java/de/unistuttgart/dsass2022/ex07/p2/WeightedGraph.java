package de.unistuttgart.dsass2022.ex07.p2;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;



public class WeightedGraph implements IWeightedGraph {

	private int numNodes;
	private int numEdges;
	private final HashMap<Long,ArrayList<IEdge>> adjacencyList;
	private final HashMap<Long,Node> nodeMap;

	
	/**
	 * Initializes an empty graph without nodes and edges.
	 */
	public WeightedGraph() {
		this.numNodes = 0;
		this.numEdges = 0;
		this.nodeMap = new HashMap<>();
		this.adjacencyList = new HashMap<>();
	}

	@Override
	public int numberOfNodes() {
		return this.numNodes;
	}

	@Override
	public int numberOfEdges() {
		return this.numEdges;
	}
	

	@Override
	public void addNode(long nodeID ,Node node){
		//adds node to the nodemap and creates a new list in the adjacency list
		this.nodeMap.put(nodeID, node);
		this.adjacencyList.put(nodeID, new ArrayList<>(1));
		this.numNodes++;
	}
	
	
	@Override
	public void setNode(long nodeID, Node node){
		this.nodeMap.replace(nodeID, node);
	}
	
	
	@Override
	public Node getNode(long nodeIdx) {
		return this.nodeMap.get(nodeIdx);
	}
	
	
	@Override
	public IEdge addEdge(long src,long dst, double weight){
		IEdge toAdd = new Edge(src, dst, weight);
		this.addEdge(toAdd);
		return toAdd;
	}

	
	@Override
	public void addEdge(IEdge edge) {
		long source = edge.getSource();
		this.adjacencyList.get(source).add(edge);
		this.numEdges++;
	}

	
	@Override
	public Iterator<IEdge> edgeIterator() {
		ArrayList<IEdge> edgeList = new ArrayList<IEdge>(numEdges);
		for (long i: adjacencyList.keySet()) {
			edgeList.addAll(this.adjacencyList.get(i));
		}
		return edgeList.iterator();
	}
	
	
	@Override
	public Iterator<Node> nodeIterator() {
		ArrayList<Node> nodeList = new ArrayList<>(numNodes);
		for (long i: nodeMap.keySet()) {
			nodeList.add(this.nodeMap.get(i));
		}
		return nodeList.iterator();
	}
	
	
	@Override
	public Iterator<Long> nodeIDIterator() {
		return adjacencyList.keySet().iterator();
	}

	
	@Override
	public Iterator<IEdge> outgoingEdges(long src) {
		return this.adjacencyList.get(src).iterator();
	}



}

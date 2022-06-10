package de.unistuttgart.dsass2022.ex07.p2;

import java.util.Iterator;

public interface IWeightedGraph extends Iterable<IEdge> {

	/**
	 * Returns the number of edges in the weighted graph.
	 * 
	 * @return the number of edges in the weighted graph
	 */
	public int numberOfEdges();

	/**
	 * Returns the number of vertices in the weighted graph.
	 * 
	 * @return the number of vertices in the weighted graph
	 */
	public int numberOfNodes();
	
	
	/**
	 * Adds a new node with the specified ID
	 * @param nodeID
	 * @param node the node to add
	 */
	public void addNode(long nodeID ,Node node);
	
	
	/**
	 * Sets the node by the specified ID
	 * @param nodeIdx 	index of the node
	 * @param meta 		node to set
	 */
	public void setNode(long nodeID, Node node);
	
	
	/**
	 * Returns the node object with the specified index.
	 * 
	 * @param nodeIdx	id of the node
	 * @return node object
	 */
	public Node getNode(long nodeIdx);
	
	
	/**
	 * Adds an Edge with the specified parameter to the graph
	 * @param src source of the edge
	 * @param dst destination of the edge
	 * @param weight weight of the edge
	 * @return respective edge object
	 */
	public IEdge addEdge(long src,long dst, double weight);
	
	
	/**
	 * Adds an Edge to the graph
	 *@param edge the edge object to add 
	 */
	public void addEdge(IEdge edge);
	

	/**
	 * Returns an iterator of all the edges in the weighted graph.
	 * 
	 * @return an iterator that iterates through all edges in the weighted graph.
	 */
	public Iterator<IEdge> edgeIterator();
	
	
	/**
	 * Returns an iterator of all the nodes in the weighted graph
	 * 
	 * @return an iterator that iterates through all nodes in the weighted graph
	 */
	public Iterator<Node> nodeIterator();

	
	/**
	 * Returns an iterator for all outgoing edges of the current node
	 * 
	 * @param src	the node id, for which to get outgoing edges
	 * @return iterator for all outgoing edges of node source.
	 */
	public Iterator<IEdge> outgoingEdges(long src);
	
	
	/**
	 * Returns an iterator for all node IDs
	 * 
	 * @return an iterator that iterates all node IDs
	 */
	public Iterator<Long> nodeIDIterator();

	
	@Override
	default Iterator<IEdge> iterator() {
		return edgeIterator();
	}
}

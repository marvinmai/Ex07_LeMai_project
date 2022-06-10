package de.unistuttgart.dsass2022.ex07.p2;

public interface IShortestPath {

  /**
   * Computes the Dijkstra algorithm on the weighted graph <tt>graph</tt>
   * from starting node <tt>startnode</tt>. This method is started after
   * initializing a ShortPath object.
   * 
   * @param graph 		the weighted graph
   * @param startnode 	the starting node
   */
  public void dijkstra(IWeightedGraph graph, long startnode);

  /**
   * Returns the distance of the shortest path from the starting node
   * <tt>startnode</tt> to node <tt>destination</tt>.
   * as the sum of the edges on the path
   * 
   * @param destination		the destination node
   * @return the length (weight) of a shortest path from the starting node
   *         <tt>startnode</tt> to node <tt>destination</tt>;
   *         <tt>Double.POSITIVE_INFINITY</tt> if no such path exists
   */
  public double distanceTo(long destination);

  /**
   * Returns a boolean whether there is a path from the starting node
   * <tt>startnode</tt> to node <tt>destination</tt>?
   * 
   * @param destination 	the destination node
   * @return <tt>true</tt> if there is a path from the starting node
   *         <tt>startnode</tt> to node <tt>destination</tt>, and
   *         <tt>false</tt> otherwise
   */
  public boolean existsPathTo(long destination);

  /**
   * Returns the edges of the shortest path from the starting node
   * <tt>startnode</tt> to node <tt>destination</tt>.
   * Example: A path exists <tt>s--->u--->x--->y</tt> then the iterable will
   * begin with the edge from node <tt>s</tt> to node <tt>u</tt>, then the
   * edge from node <tt>u</tt> to node <tt>x</tt> and then the edge from
   * node <tt>x</tt> to node <tt>y</tt>.
   * 
   * @param destination		the destination node
   * @return a shortest path from the starting node <tt>startnode</tt> to
   *         node <tt>destination</tt> as an iterable of the
   *         edges, and <tt>null</tt> if no such path exists
   */
  public Iterable<IEdge> pathTo(long destination);

}

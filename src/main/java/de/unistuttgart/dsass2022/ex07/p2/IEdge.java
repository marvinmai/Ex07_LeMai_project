package de.unistuttgart.dsass2022.ex07.p2;

public interface IEdge {

  /**
   * Returns the tail (source) vertex of the directed edge.
   * 
   * @return the tail (source) vertex of the edge
   */
  public long getSource();

  /**
   * Returns the head (destination) vertex of the directed edge.
   * 
   * @return the head (destination) vertex of the directed edge
   */
  public long getDestination();

  /**
   * Returns the weight of the directed edge.
   * 
   * @return the weight of the directed edge
   */
  public double getWeight();


}

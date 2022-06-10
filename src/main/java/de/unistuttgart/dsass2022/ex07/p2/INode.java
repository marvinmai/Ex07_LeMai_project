package de.unistuttgart.dsass2022.ex07.p2;

public interface INode {
	
	/**
	 * Returns the ID of the node
	 * 
	 * @return ID of the node
	 */
	public long getID();
	
	
	/**
	 * Returns the latitude of the node
	 * 
	 * @return latitude of the node
	 */
	public double getLatitude();
	
	
	/**
	 * Returns the longitude of the node
	 * 
	 * @return the longitude of the node
	 */
	public double getLongitude();
	
	
	/**
	 * Sets the distance of the node (in respect to the start node)
	 * 
	 * @param dist the distance to the node
	 */
	public void setDistance(double dist);
	
	
	/**
	 * Returns the distance of the node
	 * 
	 * @return the distance of the node
	 */
	public double getDistance();
}

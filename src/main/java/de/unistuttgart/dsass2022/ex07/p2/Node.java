package de.unistuttgart.dsass2022.ex07.p2;

public class Node implements INode, Comparable<Node>{
	private long id;
	private double longitude;
	private double latitude;
	private double distance;
	
	public Node(long id, double lat, double lon) {
		this.id = id;
		this.latitude = lat;
		this.longitude = lon;
		this.distance = Double.POSITIVE_INFINITY;
	}
	
	public Node(long id, double lat, double lon, double dist) {
		this.id = id;
		this.latitude = lat;
		this.longitude = lon;
		this.distance = dist;
	}

	@Override
	public long getID() {
		return this.id;
	}
	@Override
	public double getLatitude() {
		return this.latitude;
	}

	@Override
	public double getLongitude() {
		return this.longitude;
	}

	@Override
	public int compareTo(Node o) {
		return Double.compare(this.distance, o.getDistance());
	}

	@Override
	public void setDistance(double dist) {
		this.distance = dist;
	}

	@Override
	public double getDistance() {
		return this.distance;
	}

}

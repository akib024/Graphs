package roadgraph;

import geography.GeographicPoint;

public class Edge {
	protected GeographicPoint from; 
	protected GeographicPoint to;
	protected String roadName;
	protected String roadType;
	protected double length;

	public Edge(GeographicPoint f, GeographicPoint t, String rN, String rT, double l) 
	{
		// TODO Auto-generated constructor stub
		from = f;
		to = t;
		roadName = rN;
		roadType = rT;
		length = l;
	}

}

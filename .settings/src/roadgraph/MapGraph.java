/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private Set<GeographicPoint> verticesSet;
	private List<Edge> edgesList; 
	private Map<GeographicPoint, Set<GeographicPoint>> adjacentVertices;
	
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		verticesSet = new HashSet<>();
		edgesList = new ArrayList<>();
		adjacentVertices = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return verticesSet.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return verticesSet;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return edgesList.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		try{
			return verticesSet.add(location);
		}catch(Exception e){
			return false;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		try{
			if(!verticesSet.contains(from) || !verticesSet.contains(to))
					throw new IllegalArgumentException();
		}catch(Exception e){
			throw new IllegalArgumentException();
		}
		
		if(roadName.equals(null) || roadType.equals(null) || length < 0)
			throw new IllegalArgumentException();
		else{
			Edge e = new Edge(from, to, roadName, roadType, length);
			edgesList.add(e);
			
			if(adjacentVertices.containsKey(from))
			{
				adjacentVertices.get(from).add(to);
			}else{
				Set<GeographicPoint> tempSet = new HashSet<>();
				tempSet.add(to);
				adjacentVertices.put(from, tempSet);
			}
		}
		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		boolean found = false;
		Map<GeographicPoint, GeographicPoint>parent = new HashMap<>();
		Map<GeographicPoint, Boolean>visited = new HashMap<>();
		
		for(GeographicPoint node : verticesSet)
		{
			parent.put(node, null);
			visited.put(node, false);
		}
		
		Queue<GeographicPoint>q = new LinkedList<>();
		
		q.offer(start);
		
		while(!q.isEmpty())
		{
			GeographicPoint current = q.remove();
			
			
			if(!adjacentVertices.containsKey(current))
				continue;
			
			Set<GeographicPoint> tempSet = adjacentVertices.get(current);
			
			for(GeographicPoint n : tempSet)
			{
				if(!visited.get(n))
				{
					visited.put(n, true);
					parent.put(n, current);
					q.offer(n);
				}
				
				if(n.equals(goal))
				{
					found = true;
					break;
				}
			}
			
			if(found)
				break;
		}
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		List<GeographicPoint>toReturn = new ArrayList<>();
		
		if(!found)
			return null;
		
		toReturn.add(goal);
		while(!goal.equals(start))
		{
			goal = parent.get(goal);
			toReturn.add(goal);
		}
		
		Collections.reverse(toReturn);

		return toReturn;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		boolean found = false;
		int dijkstra_count = 0;
		// TODO: Implement this method in WEEK 4
		Comparator<Node> comparator = new DistanceComparator();
        PriorityQueue<Node> PQ = new PriorityQueue<Node>(comparator);
		
		Set<GeographicPoint> visited = new HashSet<>();
		Map<GeographicPoint, GeographicPoint> parent = new HashMap<>();
		Map<GeographicPoint, Node> correspondingNode = new HashMap<>();
		
		for(GeographicPoint p : verticesSet)
		{
			Node n = new Node(p, Double.MAX_VALUE);
			correspondingNode.put(p, n);
		}
		
		correspondingNode.get(start).distanceFromStart = 0;
		PQ.add(correspondingNode.get(start));
		
		while(!PQ.isEmpty())
		{
			GeographicPoint curr = PQ.poll().point;
			dijkstra_count += 1;
			
			if(!visited.contains(curr))
			{
				visited.add(curr);
				
				if(curr.equals(goal))
				{
					found = true;
					break;
				}
				
				Set<GeographicPoint> correspondingAdjecency = adjacentVertices.get(curr);
				if(correspondingAdjecency != null)
				{
					for(GeographicPoint n : correspondingAdjecency)
					{
						if(!visited.contains(n))
						{
							//Hook for visualization.  See writeup.
							//nodeSearched.accept(next.getLocation());
							nodeSearched.accept(n);
							double d = correspondingNode.get(curr).distanceFromStart + 
																		curr.distance(n);
							if(d < correspondingNode.get(n).distanceFromStart)
							{
								correspondingNode.get(n).distanceFromStart = d;
								if(!parent.containsKey(n))
								{
									parent.put(n, curr);
								}else{
									parent.replace(n, curr);
								}
								PQ.offer(correspondingNode.get(n));
							}
						}
					}
				}
			}
		}

		if(found)
		{
			List<GeographicPoint> l = new ArrayList<>();
			GeographicPoint point = goal;
			
			while(parent.containsKey(point))
			{
				l.add(point);
				point = parent.get(point);
			}
			
			l.add(point);
			Collections.reverse(l);
			System.out.println("Found for dijkstra: "+dijkstra_count);
			return l;
		}
		
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		boolean found = false;
		int astar_count = 0;
		// TODO: Implement this method in WEEK 4
		Comparator<Node> comparator = new HeuristicComparator();
        PriorityQueue<Node> PQ = new PriorityQueue<Node>(comparator);
		
		Set<GeographicPoint> visited = new HashSet<>();
		Map<GeographicPoint, GeographicPoint> parent = new HashMap<>();
		Map<GeographicPoint, Node> correspondingNode = new HashMap<>();
		
		for(GeographicPoint p : verticesSet)
		{
			Node n = new Node(p, Double.MAX_VALUE, p.distance(goal));
			correspondingNode.put(p, n);
		}
		
		correspondingNode.get(start).distanceFromStart = 0;
		PQ.add(correspondingNode.get(start));
		
		while(!PQ.isEmpty())
		{
			GeographicPoint curr = PQ.poll().point;
			astar_count += 1;
			//Hook for visualization.  See writeup.
			//nodeSearched.accept(next.getLocation());
			nodeSearched.accept(curr);
			
			if(!visited.contains(curr))
			{
				visited.add(curr);
				
				if(curr.equals(goal))
				{
					found = true;
					break;
				}
				
				Set<GeographicPoint> correspondingAdjecency = adjacentVertices.get(curr);
				if(correspondingAdjecency != null)
				{
					for(GeographicPoint n : correspondingAdjecency)
					{
						if(!visited.contains(n))
						{
							double d = correspondingNode.get(curr).distanceFromStart + 
																				curr.distance(n);
							if(d < correspondingNode.get(n).distanceFromStart)
							{
								correspondingNode.get(n).distanceFromStart = d;
								if(!parent.containsKey(n))
								{
									parent.put(n, curr);
								}else{
									parent.replace(n, curr);
								}
								PQ.offer(correspondingNode.get(n));
							}
						}
					}
				}
			}
		}

		if(found)
		{
			List<GeographicPoint> l = new ArrayList<>();
			GeographicPoint point = goal;
			
			while(parent.containsKey(point))
			{
				l.add(point);
				point = parent.get(point);
			}
			
			l.add(point);
			Collections.reverse(l);
			System.out.println("Found for a*: "+astar_count);
			return l;
		}
		
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		//question 1:
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");
		
		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}

class DistanceComparator implements Comparator<Node>
{
    @Override
    public int compare(Node x, Node y)
    {
        if (x.distanceFromStart < y.distanceFromStart)
        {
            return -1;
        }
        if (x.distanceFromStart > y.distanceFromStart)
        {
            return 1;
        }
        return 0;
    }
}

class HeuristicComparator implements Comparator<Node>
{
    @Override
    public int compare(Node x, Node y)
    {
        if (x.distanceFromStart + x.distanceToGoal < y.distanceFromStart + y.distanceToGoal)
        {
            return -1;
        }
        if (x.distanceFromStart + x.distanceToGoal > y.distanceFromStart + y.distanceToGoal)
        {
            return 1;
        }
        return 0;
    }
}

class Node{
	protected GeographicPoint point;
	protected double distanceFromStart;
	protected double distanceToGoal;
	
	public Node(GeographicPoint p, double dfs)
	{
		point = p;
		distanceFromStart = dfs;
	}
	
	public Node(GeographicPoint p, double dfs, double dtg)
	{
		point = p;
		distanceFromStart = dfs;
		distanceToGoal = dtg;
	}
}

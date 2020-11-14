/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.sql.SQLException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;
import java.util.regex.Pattern;

import dboperation.SQLiteDB;
import geography.GeographicPoint;
import geography.RoadSegment;
import roadgraph.MapGraph2.CustomGeographicPointForDijkstra;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph2 {
	//TODO: Add your member variables here in WEEK 3
	private Set<GeographicPoint> setOfVertices = null;
	private List<RoadSegment> listOfEdges = null;
	private Map<Double, Set<Double>> longitudesAgainstLatitude = null;
	private Map<GeographicPoint, Set<GeographicPoint>> neighborsAgainstGeographicPoint = null;
	private Map<String, GeographicPoint> pointAgainstLatAndLong = null;
	
	
	/** 
	 * Create a new empty MapGraph2 
	 */
	public MapGraph2()
	{
		// TODO: Implement in this constructor in WEEK 3
		this.setOfVertices = new HashSet<>();
		this.longitudesAgainstLatitude = new HashMap<>();
		this.listOfEdges = new ArrayList<>();
		this.neighborsAgainstGeographicPoint = new HashMap<>();
		this.pointAgainstLatAndLong = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return this.setOfVertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return new HashSet<>(this.setOfVertices);
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return this.listOfEdges.size();
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
		if (location == null)
			return false;
		
		double latitude = location.getX();
		double longitude = location.getY();
		
		Set<Double> setOfLongitudes = this.longitudesAgainstLatitude.get(latitude);
		
		if (setOfLongitudes != null) {
			if (setOfLongitudes.contains(longitude)) {
				return false;
			}
		} else {
			setOfLongitudes = new HashSet<>();
		}
		
		setOfLongitudes.add(longitude);
		this.longitudesAgainstLatitude.put(latitude, setOfLongitudes);
		this.setOfVertices.add(location);
		this.pointAgainstLatAndLong.put(location.toString(), location);
		
		return true;
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
		if (!this.setOfVertices.contains(from) || !this.setOfVertices.contains(to)
				|| from == null || to == null 
				|| roadName == null || roadType == null
				|| length < 0)
			throw new IllegalArgumentException();
		
		if (this.neighborsAgainstGeographicPoint.get(from) == null) {
			this.neighborsAgainstGeographicPoint.put(from, new HashSet<>());
		}
		this.neighborsAgainstGeographicPoint.get(from).add(to);
		
		RoadSegment edge = new RoadSegment(from, to, new ArrayList<GeographicPoint>(), roadName, roadType, length);
		this.listOfEdges.add(edge);
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
		Queue<GeographicPoint> queue = new LinkedList<>();
		Set<GeographicPoint> setOfVisitedNodes = new HashSet<>();
		Map<GeographicPoint, GeographicPoint> parentAginstChildPoint = new HashMap<>();
		
		queue.add(start);
		setOfVisitedNodes.add(start);
		boolean isFound = false;
		
		while (!queue.isEmpty()) {
			GeographicPoint currentPoint = queue.poll();
			nodeSearched.accept(currentPoint);
			if (currentPoint.equals(goal)) {
				isFound = true;
				break;
			}
			
			Set<GeographicPoint> neighbors = this.neighborsAgainstGeographicPoint.get(currentPoint);
			if (neighbors != null) {
				for (GeographicPoint neighbor : neighbors) {
					if (!setOfVisitedNodes.contains(neighbor)) {
						setOfVisitedNodes.add(neighbor);
						queue.add(neighbor);
						parentAginstChildPoint.put(neighbor, currentPoint);
					}
				}
			}
		}
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if (!isFound) {
			//System.out.println("No path exists");
			return null;
		}
		
		LinkedList<GeographicPoint> finalList = new LinkedList<>();
		GeographicPoint currentPoint = goal;
		while (!currentPoint.equals(start)) {
			finalList.addFirst(currentPoint);
			currentPoint = parentAginstChildPoint.get(currentPoint);
		}
		finalList.addFirst(currentPoint);

		return finalList;
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
        
        List<GeographicPoint> resultList = null;
        SQLiteDB dbObj = new SQLiteDB();
        try {
			String resultPath = dbObj.fetchDijkstraPath(start.toString(), goal.toString());
			
			if (resultPath != null) {
				resultList = processStringToList(resultPath);
				if (resultList.isEmpty()) {
					resultList = null;
				}
				//return resultList;
			} else {
				resultList = dijkstra(start, goal, temp);
				String processedStr = processListToString(new LinkedList<>(resultList));
				try {
					dbObj.insertIntoDijkstra(start.toString(), goal.toString(), processedStr);
				} catch (ClassNotFoundException e) {
					System.out.println("May be the jar is missing, please check out...");
					e.printStackTrace();
				} catch (SQLException e) {
					System.out.println("May be row already exists or Some query is not working properly...");
					e.printStackTrace();
				}
				//return resultList;
			}
			
		} catch (ClassNotFoundException e) {
			System.out.println("May be the jar is missing, please check out...");
			e.printStackTrace();
		} catch (SQLException e) {
			System.out.println("Some query is not working properly...");
			e.printStackTrace();
		}
        
        /*try {
			dbObj.closeConnection();
		} catch (SQLException e) {
			System.out.println("Shouldn't be here, please check stack trace...");
			e.printStackTrace();
		}*/
        
        //return dijkstra(start, goal, temp);
        return resultList;
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
		// TODO: Implement this method in WEEK 4
		Map<GeographicPoint, Double> customDistanceFromStartMap = new HashMap<>();
		PriorityQueue<CustomGeographicPointForDijkstra> 
			sortedQueue = new PriorityQueue<>(new Comparator<CustomGeographicPointForDijkstra>() {

				@Override
				public int compare(CustomGeographicPointForDijkstra point1, CustomGeographicPointForDijkstra point2) {
					return point1.getCalculatedDistanceFromStart() > point2.getCalculatedDistanceFromStart() ? 1 : -1;
				}});
		
		Set<GeographicPoint>setOfVisitedPoints = new HashSet<>();
		Map<GeographicPoint, GeographicPoint> parentAginstChildPoint = new HashMap<>();
		
		CustomGeographicPointForDijkstra customStartPoint = new CustomGeographicPointForDijkstra(start, 0);
		sortedQueue.add(customStartPoint);
		customDistanceFromStartMap.put(start, 0.0);
		
		boolean isGoalFound = false;
		
		while (!sortedQueue.isEmpty()) {
			CustomGeographicPointForDijkstra currentCustomNode = sortedQueue.poll();
			GeographicPoint currentNode = currentCustomNode.getGeographicPoint();
			
			if (!setOfVisitedPoints.contains(currentNode)) {
				setOfVisitedPoints.add(currentNode);
				nodeSearched.accept(currentNode);
				if (currentNode.equals(goal)) {
					isGoalFound = true;
					break;
				}
				
				double customDistanceFromCurrentToStart = customDistanceFromStartMap.get(currentNode);
				
				Set<GeographicPoint> neighbors = neighborsAgainstGeographicPoint.get(currentNode);
				if (neighbors != null) {
					for (GeographicPoint neighbor : neighbors) {
						if (!setOfVisitedPoints.contains(neighbor)) {
							double actualDistanceFromCurrent = neighbor.distance(currentNode);
							double alterDistanceFromStart = customDistanceFromCurrentToStart + actualDistanceFromCurrent;
							
							Double tempDistanceFromStart = customDistanceFromStartMap.get(neighbor);
							if (tempDistanceFromStart == null || alterDistanceFromStart < tempDistanceFromStart) {
								CustomGeographicPointForDijkstra customNeighborPoint = 
										new CustomGeographicPointForDijkstra(neighbor, alterDistanceFromStart);
								
								customDistanceFromStartMap.put(neighbor, alterDistanceFromStart);
								sortedQueue.add(customNeighborPoint);
								parentAginstChildPoint.put(neighbor, currentNode);
							}
						}
					}
				}
			}
		}
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if (!isGoalFound) {
			return null;
		}
		
		LinkedList<GeographicPoint> finalList = new LinkedList<>();
		GeographicPoint currentPoint = goal;
		while (!currentPoint.equals(start)) {
			finalList.addFirst(currentPoint);
			currentPoint = parentAginstChildPoint.get(currentPoint);
		}
		finalList.addFirst(currentPoint);
		
		return finalList;
	}

	private List<GeographicPoint> processStringToList(String processedStr) {
		String[] strArr = processedStr.split(Pattern.quote("|"));
		LinkedList<GeographicPoint> finalList = new LinkedList<>();
		for (String str : strArr) {
			finalList.addLast(this.pointAgainstLatAndLong.get(str));
		}
		return finalList;
	}

	private String processListToString(List<GeographicPoint> list) {
		StringBuilder finalResultBuilder = new StringBuilder();
		if (list != null) {
			for (GeographicPoint gp : list) {
				if (finalResultBuilder.length() > 0) {
					finalResultBuilder.append("|");
				}
				finalResultBuilder.append(gp.toString());
			}
		}
		return finalResultBuilder.toString();
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
		// TODO: Implement this method in WEEK 4
		Map<GeographicPoint, Double> heuristicDistanceFromStartMap = new HashMap<>();
		PriorityQueue<CustomGeographicPointForDijkstra> 
			sortedQueue = new PriorityQueue<>(new Comparator<CustomGeographicPointForDijkstra>() {

				@Override
				public int compare(CustomGeographicPointForDijkstra point1, CustomGeographicPointForDijkstra point2) {
					return point1.getCalculatedDistanceFromStart() > point2.getCalculatedDistanceFromStart() ? 1 : -1;
				}});
		
		Set<GeographicPoint>setOfVisitedPoints = new HashSet<>();
		Map<GeographicPoint, GeographicPoint> parentAginstChildPoint = new HashMap<>();
		
		double actualDistanceFromGoal = start.distance(goal);
		CustomGeographicPointForDijkstra customStartPoint = new CustomGeographicPointForDijkstra(start, actualDistanceFromGoal);
		sortedQueue.add(customStartPoint);
		heuristicDistanceFromStartMap.put(start, actualDistanceFromGoal);
		
		boolean isGoalFound = false;
		
		while (!sortedQueue.isEmpty()) {
			CustomGeographicPointForDijkstra currentCustomNode = sortedQueue.poll();
			GeographicPoint currentNode = currentCustomNode.getGeographicPoint();
			
			if (!setOfVisitedPoints.contains(currentNode)) {
				setOfVisitedPoints.add(currentNode);
				nodeSearched.accept(currentNode);
				if (currentNode.equals(goal)) {
					isGoalFound = true;
					break;
				}
				
				double customDistanceFromCurrentToStart = heuristicDistanceFromStartMap.get(currentNode);
				
				Set<GeographicPoint> neighbors = neighborsAgainstGeographicPoint.get(currentNode);
				if (neighbors != null) {
					for (GeographicPoint neighbor : neighbors) {
						if (!setOfVisitedPoints.contains(neighbor)) {
							double actualDistanceFromCurrent = neighbor.distance(currentNode);
							double distanceFromGoalToNeighbour = neighbor.distance(goal);
							double alterDistanceFromStart = customDistanceFromCurrentToStart + actualDistanceFromCurrent + distanceFromGoalToNeighbour;
							
							Double tempDistanceFromStart = heuristicDistanceFromStartMap.get(neighbor);
							if (tempDistanceFromStart == null || alterDistanceFromStart < tempDistanceFromStart) {
								CustomGeographicPointForDijkstra customNeighborPoint = 
										new CustomGeographicPointForDijkstra(neighbor, alterDistanceFromStart);
								
								heuristicDistanceFromStartMap.put(neighbor, alterDistanceFromStart);
								sortedQueue.add(customNeighborPoint);
								parentAginstChildPoint.put(neighbor, currentNode);
							}
						}
					}
				}
			}
		}
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if (!isGoalFound) {
			return null;
		}
		
		LinkedList<GeographicPoint> finalList = new LinkedList<>();
		GeographicPoint currentPoint = goal;
		while (!currentPoint.equals(start)) {
			finalList.addFirst(currentPoint);
			currentPoint = parentAginstChildPoint.get(currentPoint);
		}
		finalList.addFirst(currentPoint);

		return finalList;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph2 firstMap = new MapGraph2();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
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
		*/
		
		
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
	
	public class CustomGeographicPointForDijkstra {
		
		private GeographicPoint geographicPoint;
		private double calculatedDistanceFromStart;
		
		public CustomGeographicPointForDijkstra(GeographicPoint geographicPoint, double calculatedDistanceFromStart) {
			super();
			this.geographicPoint = geographicPoint;
			this.calculatedDistanceFromStart = calculatedDistanceFromStart;
		}
		
		/**
		 * @return the geographicPoint
		 */
		protected GeographicPoint getGeographicPoint() {
			return geographicPoint;
		}
		/**
		 * @param geographicPoint the geographicPoint to set
		 */
		protected void setGeographicPoint(GeographicPoint geographicPoint) {
			this.geographicPoint = geographicPoint;
		}
		/**
		 * @return the calculatedDistanceFromStart
		 */
		protected double getCalculatedDistanceFromStart() {
			return calculatedDistanceFromStart;
		}
		/**
		 * @param calculatedDistanceFromStart the calculatedDistanceFromStart to set
		 */
		protected void setCalculatedDistanceFromStart(double calculatedDistanceFromStart) {
			this.calculatedDistanceFromStart = calculatedDistanceFromStart;
		}
		
		@Override
		public String toString() {
			return "CustomGeographicPointForDijkstra [geographicPoint=" + geographicPoint
					+ ", calculatedDistanceFromStart=" + calculatedDistanceFromStart + "]";
		}
	}
	
}

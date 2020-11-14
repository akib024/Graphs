package dboperation;

import java.sql.SQLException;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Scanner;
import java.util.regex.Pattern;

import geography.GeographicPoint;

public class SQLiteDBTest {

	private static Map<String, GeographicPoint> pointAgainstLatAndLong;

	public static void main(String[] args) {
		Scanner reader = new Scanner(System.in);
		
		pointAgainstLatAndLong = new HashMap<>();
		
		SQLiteDB dbObj = new SQLiteDB();
		
		GeographicPoint start = new GeographicPoint(1.0, 3.3);
		pointAgainstLatAndLong.put(start.toString(), start);
		GeographicPoint goal = new GeographicPoint(3.3, 6.7);
		pointAgainstLatAndLong.put(goal.toString(), goal);
		
		LinkedList<GeographicPoint> path = new LinkedList<>();
		path.addFirst(goal);
		path.addFirst(start);
		
		String processedStr = processListToString(new LinkedList<GeographicPoint>(path));
		
		while (true) {
			System.out.print("Do you want to coninue ? 'Y'/'N':");
			String input = reader.nextLine();
			if (input.equalsIgnoreCase("Y")) {
				try {
					String resultPath = dbObj.fetchDijkstraPath(start.toString(), goal.toString());
					
					if (resultPath != null) {
						if (resultPath.equals(processedStr)) 
							System.out.println("Yes, the result strings match!");
						else 
							System.out.println("May be there is calculation problem..");
						System.out.println(processStringToList(resultPath));
					} else {
						try {
							dbObj.insertIntoDijkstra(start.toString(), goal.toString(), processedStr);
						} catch (ClassNotFoundException e) {
							System.out.println("May be the jar is missing, please check out...");
							e.printStackTrace();
						} catch (SQLException e) {
							System.out.println("May be row already exists or Some query is not working properly...");
							e.printStackTrace();
						}
					}
				} catch (ClassNotFoundException e) {
					System.out.println("May be the jar is missing, please check out...");
					e.printStackTrace();
				} catch (SQLException e) {
					System.out.println("Some query is not working properly...");
					e.printStackTrace();
				} 
			} else if (input.equalsIgnoreCase("N")) {
				reader.close();
				break;
			} else {
				System.out.println("You have put wrong input...");
			}
		}
		
		try {
			dbObj.closeConnection();
		} catch (SQLException e) {
			System.out.println("Shouldn't be here, please check stack trace...");
			e.printStackTrace();
		}
	}

	private static LinkedList<GeographicPoint> processStringToList(String processedStr) {
		String[] strArr = processedStr.split(Pattern.quote("|"));
		LinkedList<GeographicPoint> finalList = new LinkedList<>();
		for (String str : strArr) {
			finalList.addLast(pointAgainstLatAndLong.get(str));
		}
		return finalList;
	}

	private static String processListToString(LinkedList<GeographicPoint> linkedList) {
		StringBuilder finalResultBuilder = new StringBuilder();
		for (GeographicPoint gp : linkedList) {
			if (finalResultBuilder.length() > 0) {
				finalResultBuilder.append("|");
			}
			finalResultBuilder.append(gp.toString());
		}
		return finalResultBuilder.toString();
	}
	
	

}

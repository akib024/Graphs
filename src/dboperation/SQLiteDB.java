package dboperation;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;

public class SQLiteDB {
	
	private static Connection connectionObject;
	
	private void checkConnection() throws ClassNotFoundException, SQLException {
		if (connectionObject == null) {
			System.out.println("Connection has to be established...");
			// sqlite driver
			Class.forName("org.sqlite.JDBC");
			  
			// database path, if it's new database, it will be created in the project folder
			connectionObject = DriverManager.getConnection("jdbc:sqlite:SQLiteGraph.db");
		} else {
			System.out.println("Connection already established!");
		}
	 }

	/*
	 * return boolean false if table already exists
	 */
	private boolean checkExistenceOfDijkstraTable() throws SQLException {
		Statement state = connectionObject.createStatement();
		
		ResultSet res = state.executeQuery("SELECT name FROM sqlite_master WHERE type='table' AND name='DijkstraAlgo'");
		 
		 if(!res.next()) {// need to build the table
			 System.out.println("Creating the DijkstraAlgo table...");
			 Statement state1 = connectionObject.createStatement();
			 state1.executeUpdate("create table DijkstraAlgo("
					 + "start varchar(60)," 
					 + "goal varchar(60)," 
					 + "path varchar(1200)," 
					 + "primary key (start, goal));");
			 return true;
		 } else {
			 System.out.println("DijkstraAlgo table already exists!");
		 }
		
		return false;
	}

	private String getPathFromDijkstraTable(String start, String goal) throws SQLException {
		
		String sql = "SELECT path "
                + "FROM DijkstraAlgo "
                + "WHERE start = ? "
				 + "AND goal = ?;";
		
		 PreparedStatement queryStatement = connectionObject.prepareStatement(sql);
		 queryStatement.setString(1, start);
		 queryStatement.setString(2, goal);
		 
		 ResultSet res = queryStatement.executeQuery();
		 
		 if (res.next()) {
			 System.out.println("Path is found!");
			 return res.getString("path");
		 } else {
			 System.out.println("Path has to be found...");
		 }
		 
		return null;
	}
	
	public String fetchDijkstraPath (String start, String goal) throws ClassNotFoundException, SQLException {
		
		checkConnection();
		
		boolean isNewTableCreated = checkExistenceOfDijkstraTable();
		
		if (isNewTableCreated) return null;
		
		String path = getPathFromDijkstraTable(start,  goal);
		
		if (path != null)
			return path;
		
		return null;
		
	}

	public void insertIntoDijkstra(String start, String goal, String path) throws SQLException, ClassNotFoundException {
		System.out.println("Path: "+path);
		checkConnection();
		
		checkExistenceOfDijkstraTable();
		
		PreparedStatement prep = connectionObject.prepareStatement("insert into DijkstraAlgo(start, goal, path) "
				+ "values(?,?,?);");
		prep.setString(1, start);
		prep.setString(2, goal);
		prep.setString(3, path);
		
		if (prep.executeUpdate() > 0) {
			System.out.println("Row is updated!");
		} else {
			System.out.println("May be row aleady exists...");
		}
	}

	public void closeConnection() throws SQLException {
		if (connectionObject != null) {
			connectionObject.close();
			System.out.println("Connection has been closed!");
		} else {
			System.out.println("Connection doesn't exist...");
		}
	 }

}

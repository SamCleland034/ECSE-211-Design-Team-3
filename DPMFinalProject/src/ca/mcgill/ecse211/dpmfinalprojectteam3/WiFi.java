package ca.mcgill.ecse211.dpmfinalprojectteam3;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;

/**
 * Example class using WifiConnection to communicate with a server and receive
 * data concerning the competition such as the starting corner the robot is
 * placed in.
 * 
 * Keep in mind that this class is an **example** of how to use the WiFi code;
 * you must use the WifiConnection class yourself in your own code as
 * appropriate. In this example, we simply show how to get and process different
 * types of data.
 * 
 * There are two variables you **MUST** set manually before trying to use this
 * code.
 * 
 * 1. SERVER_IP: The IP address of the computer running the server application.
 * This will be your own laptop, until the beta beta demo or competition where
 * this is the TA or professor's laptop. In that case, set the IP to
 * 192.168.2.3.
 * 
 * 2. TEAM_NUMBER: your project team number
 * 
 * Note: We System.out.println() instead of LCD printing so that full debug
 * output (e.g. the very long string containing the transmission) can be read on
 * the screen OR a remote console such as the EV3Control program via Bluetooth
 * or WiFi. You can disable printing from the WiFi code via
 * ENABLE_DEBUG_WIFI_PRINT (below).
 * 
 * @author Michael Smith
 *
 */
public class WiFi {
	// ** Set these as appropriate for your team and current situation **
	private static final String SERVER_IP = "192.168.2.42";
	private static final int TEAM_NUMBER = 3;
	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

	@SuppressWarnings("rawtypes")
	public void getValues() {

		System.out.println("Running..");

		// Initialize WifiConnection class
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		// Connect to server and get the data, catching any errors that might occur
		try {
			/*
			 * getData() will connect to the server and wait until the user/TA presses the
			 * "Start" button in the GUI on their laptop with the data filled in. Once it's
			 * waiting, you can kill it by pressing the upper left hand corner button
			 * (back/escape) on the EV3. getData() will throw exceptions if it can't connect
			 * to the server (e.g. wrong IP address, server not running on laptop, not
			 * connected to WiFi router, etc.). It will also throw an exception if it
			 * connects but receives corrupted data or a message from the server saying
			 * something went wrong. For example, if TEAM_NUMBER is set to 1 above but the
			 * server expects teams 17 and 5, this robot will receive a message saying an
			 * invalid team number was specified and getData() will throw an exception
			 * letting you know.
			 */
			Map data = conn.getData();

			// Example 1: Print out all received data
			System.out.println("Map:\n" + data);
			FinalProject.SHLLX = ((Long) data.get("SH_LL_x")).intValue();
			FinalProject.SHLLY = ((Long) data.get("SH_LL_y")).intValue();
			FinalProject.LLSRGX = ((Long) data.get("Red_LL_x")).intValue();
			FinalProject.LLSRGY = ((Long) data.get("Red_LL_y")).intValue();
			FinalProject.LLSRGX = ((Long) data.get("Green_LL_x")).intValue();
			FinalProject.LLSRGY = ((Long) data.get("Green_LL_y")).intValue();
			FinalProject.URSRGX = ((Long) data.get("Green_UR_x")).intValue();
			FinalProject.URSRGY = ((Long) data.get("Green_UR_y")).intValue();
			FinalProject.URSRRX = ((Long) data.get("Red_UR_x")).intValue();
			FinalProject.URSRRY = ((Long) data.get("Red_UR_y")).intValue();
			FinalProject.zipgreenX = ((Long) data.get("ZC_G_x")).intValue();
			FinalProject.zipgreenY = ((Long) data.get("ZC_G_y")).intValue();
			FinalProject.zipredX = ((Long) data.get("ZC_R_x")).intValue();
			FinalProject.zipredY = ((Long) data.get("ZC_R_y")).intValue();
			FinalProject.SHURX = ((Long) data.get("SH_UR_x")).intValue();
			FinalProject.SHURY = ((Long) data.get("SH_UR_y")).intValue();
			FinalProject.SVURX = ((Long) data.get("SV_UR_x")).intValue();
			FinalProject.SVURY = ((Long) data.get("SV_UR_y")).intValue();
			FinalProject.SVLLX = ((Long) data.get("SV_LL_x")).intValue();
			FinalProject.SVLLY = ((Long) data.get("SV_LL_y")).intValue();
			FinalProject.zipgreenXc = ((Long) data.get("ZO_G_x")).intValue();
			FinalProject.zipgreenYc = ((Long) data.get("ZO_G_y")).intValue();
			FinalProject.zipredXc = ((Long) data.get("ZO_R_x")).intValue();
			FinalProject.zipredYc = ((Long) data.get("ZO_R_y")).intValue();
			FinalProject.redCorner = ((Long) data.get("RedCorner")).intValue();
			FinalProject.redTeam = ((Long) data.get("RedTeam")).intValue();
			FinalProject.greenTeam = ((Long) data.get("GreenTeam")).intValue();
			FinalProject.greenCorner = ((Long) data.get("OG")).intValue();
			FinalProject.redColor = ((Long) data.get("OR")).intValue();
			/*
			 * // Example 2 : Print out specific values int redTeam = ((Long)
			 * data.get("RedTeam")).intValue(); System.out.println("Red Team: " + redTeam);
			 * 
			 * int og = ((Long) data.get("OG")).intValue();
			 * System.out.println("Green opponent flag: " + og);
			 * 
			 * // Example 3: Compare value int sh_ll_x = ((Long)
			 * data.get("SH_LL_x")).intValue(); if (sh_ll_x < 5) {
			 * System.out.println("Shallow water LL zone X < 5"); } else {
			 * System.out.println("Shallow water LL zone X >= 5"); }
			 */

		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}
		// Wait until user decides to end program
		Button.waitForAnyPress();
		FinalProject.stage = Stage.IDLE;
	}
}

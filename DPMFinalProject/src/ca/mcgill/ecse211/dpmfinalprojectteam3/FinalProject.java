//TESTTT

package ca.mcgill.ecse211.dpmfinalprojectteam3;

import java.util.LinkedList;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

/**
 * 11/1/2017, Description of how Code works (Example given from POV of green
 * side): 1. Robot takes in data from the server 2. Enter ultrasonic
 * localization stage, the method doLocalization() is called in the ultrasonic
 * localization class. Odometer, Odometry and USPoller threads are started in
 * parallel. 3. Once ultrasonic localization ends, the method startLightLOC()
 * will be called, which starts the light localization part of the localization
 * phase. 4. After light localization is done, we are then enter the navigation
 * phase. We will pass a path given by the game server into the the navigation
 * path that will iterate over the coordinates given using startNav(). We will
 * also start our odometry correction thread, since we will be navigating now
 * and need to account for the error that the navigation will accumulate.
 * However, since we will need to turn odometry correction off to use our
 * localization again once we get to one of the zipline coordinates, we will
 * have to use a boolean to keep track of when we want it to be on or off. Doing
 * this instead of killing the whole thread because we will eventually want to
 * reuse this thread after we use the zipline or perform flag search. the
 * avoidance thread will also be called, which will override the navigation
 * calls if the inDanger flag goes high. (Assume this will always be on from now
 * on unless said otherwise) 5. Once we reach the zipline coordinate, the
 * coordinate corresponding to xc yc in lab 5, we will perform a second light
 * localization to account for the error that we built up from navigating. We
 * will use the method lightLocWithError() in the LightLocalizer class. We are
 * using this method instead of the first one in the beginning because in the
 * starting corner there are a lot of things we can neglect since we are just
 * starting from a starting position. However, since the robot has been
 * travelling there can be a lot of uncertainty about the position of the robot.
 * More specifically, the robot can be on a line, so we check that if it is on a
 * line ( a line in the tangiential direction from the center of the robot) and
 * if it is on a line we reposition the robot to allow it to perform light
 * localization. The same thing will happen with the avoidance thread, it will
 * be put into an idle state while the robot is localizing. It will only go back
 * into its operating state once the robot is back in its navigating state.
 * After performing this localization, the robot will then travelTo(zipGreenX,
 * zipGreenY) and then zipTraversal() will be called from the navigation class
 * to handle the traversing. Once the robot lands, we will set the odometer to
 * be the at the position zipRedX and zipRedY and travel to zipRedXc, zipRedYc.
 * This is a risky assumption, but we will update this assumption if the test
 * success rate is low. 7. We will then perform a 3rd localization
 * (lightLocWithError())once we get to the zipRed coordinates. 8. The robot will
 * then travel to the red search region. 9. Once the robot gets to the search
 * region of the zone it is in, flagSearch() from navigation will be called.
 * This will disable the two lightsensor threads used for detecting lines,
 * odometry correction thread and avoidance thread. 10. flagSearch() will work
 * by sweeping 90 degrees from the next corner to the previous corner, we will
 * sweep until the US sensor reads a value is less than the threshold value that
 * we set for it, (TBD through experimentation) which will then cause the robot
 * to stop rotating and move to that object. The robot will then sweep 45
 * degrees in both directions and detect if it finds the correct color. If yes,
 * beep 3 times and travel back to the previous corner. If it doesn't find the
 * correct color it will precede to the next corner of the search region and
 * sweep again, this process will keep happening until the block with the
 * correct color is identified. 11. The robot will then enter the navigating
 * phase again, reactivating the odometer correction and avoidance thread. The
 * next coordinates it will travel to will be the river since it started from
 * the green zone. After traversing the river, the robot will then go back to
 * its starting position, which will then prompt the end of the game. How the
 * code will flow: Inital idea was to model the stages of the robot as a state
 * machine, which will be implemented to start with.
 *
 * Threads in use- We have a total of 8 threads, 1 avoidance thread, 3
 * lightpoller threads for each light sensor, 1 odometer thread, 1 odometry
 * display thread, 1 thread to continuous rotate the motor that the US and color
 * sensor are connected to , 1 ultrasonic poller thread and 1 odometry
 * correction thread. However, the way the we implemented the threads is that
 * each thread has two methods, on() and off(). While a thread is on, it will do
 * what it is meant to do, but if it is off, it will sleep for a period of time
 * and then check if it is still off. For example, during the flag search stage,
 * we do not want to use odometry correction since our robot will have to travel
 * in an unknown orientation (on a diagonal), so we will have to disable the
 * odometry correction thread using the off() method. However, sleeping them
 * still might cause issues with how many actions our robot can handle, so we
 * will make changes to this idea if problems arise. See the Stage enumeration
 * for details on how the the robot changes states.
 * 
 * @author Sam Cleland, Yiming Wu, Charles Brana
 * @version 2.0: Code estimation: 1200 lines of code(lines meaning number of
 *          semicolons).
 */
public class FinalProject extends Thread {
	public static int redCorner;
	public static int greenCorner;
	public static int redTeam;
	public static int greenTeam;
	/** x coord of the zipline in the green region. */
	public static int zipgreenX;

	/** y coord of the zipline in the green region. */
	public static int zipgreenY;

	/** xc coord of the zipline in the green region. */
	public static int zipgreenXc;

	/** yc coord of the zipline in the green region. */
	public static int zipgreenYc;

	/** x coord of the zipline in the red region. */
	public static int zipredX;

	/** y coord of the zipline in the red region. */
	public static int zipredY;

	/** xc coord of the zipline in the red region. */
	public static int zipredXc;

	/** yc coord of the zipline in the red region. */
	public static int zipredYc;

	/** x coord of the lower left corner of green search region. */
	public static int LLSRGX;

	/** y coord of the lower left corner of green search region. */
	public static int LLSRGY;

	/** x coord of the upper right corner of green search region. */
	public static int URSRGX;

	/** y coord of the upper right corner of green search region. */
	public static int URSRGY;

	/** x coord of the lower left corner of red search region. */
	public static int LLSRRX;

	/** y coord of the lower left corner of red search region. */
	public static int LLSRRY;

	/** x coord of the upper right corner of red search region. */
	public static int URSRRX;

	/** y coord of the upper right corner of red search region. */
	public static int URSRRY;

	/** x coord of lower left corner of horizontal shallow water region. */
	public static int SHLLX;

	/** y coord of lower left corner of horizontal shallow water region. */
	public static int SHLLY;

	/** x coord of upper right corner of horizontal shallow water region. */
	public static int SHURX;

	/** y coord of upper right corner of horizontal shallow water region. */
	public static int SHURY;
	/** x coord of lower left corner of vertical shallow water region. */
	public static int SVLLX;

	/** y coord of lower left corner of vertical shallow water region. */
	public static int SVLLY;

	/** x coord of upper right corner of vertical shallow water region. */
	public static int SVURX;

	/** y coord of upper right corner of vertical shallow water region. */
	public static int SVURY;
	public static int redColor;
	public static int greenColor;
	/** The Constant leftMotor, global left motor for entire project. */
	// Assign ports to motors and to sensor
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	/** The Constant zipMotor, used for the zip motor, global access. */
	public static final EV3LargeRegulatedMotor zipMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	/** The Constant rightMotor, global right motor for entire project. */
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor usMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

	/** The Constant usSensor. Ultrasonic sensor used */
	public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));

	/** The odometer. */

	/** The us dist, used to change the sensor mode to distance. */
	// create variables
	static SampleProvider usDist = usSensor.getMode("Distance");

	/** The sample. Float array used to generate */
	static float[] sample = new float[usDist.sampleSize()];

	/** The Constant TILE_SPACING, distance in centimeters between each tile. */
	public static final double TILE_SPACING = 30.48;

	/** The Constant WHEEL_RADIUS. Wheel radius of our robot's wheels */
	public static final double WHEEL_RADIUS = 2.145; // radius of wheel

	/** The Constant TRACK. Distance between the wheels */
	public static final double TRACK = 11.13; // Width of car
	public static final double THRESHOLD = 20;
	public static Stage stage;
	/** The Constant LightPort. */
	private static final Port LightPort = LocalEV3.get().getPort("S4");
	public static final EV3ColorSensor colorSensor = new EV3ColorSensor(LightPort);
	private static final Port LeftPort = LocalEV3.get().getPort("S2");
	public static final EV3ColorSensor leftSensor = new EV3ColorSensor(LeftPort);
	private static final Port RightPort = LocalEV3.get().getPort("S3");
	public static final EV3ColorSensor rightSensor = new EV3ColorSensor(RightPort);
	public static Odometer odometer = new Odometer(leftMotor, rightMotor);
	static SampleProvider leftProvider = leftSensor.getColorIDMode();
	static SampleProvider rightProvider = rightSensor.getColorIDMode();

	static SampleProvider colorProvider = colorSensor.getRGBMode();

	/**
	 * The main method.
	 *
	 * @param args
	 *            the arguments
	 */
	public static void main(String[] args) {

		stage = Stage.WIFI;
		WiFi wifi = new WiFi();
		wifi.getValues();
		while (stage == Stage.WIFI) {
			continue;
		}

		// instantiate threads controlling the robot
		final TextLCD t = LocalEV3.get().getTextLCD();
		t.drawString("ZIPLINE GREEN X=" + zipgreenX, 0, 0);
		t.clear();

		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);

		Navigation gps = new Navigation(odometer);
		Avoidance master = new Avoidance(gps);
		SensorRotation sensorMotor = new SensorRotation(master, usMotor, gps);

		UltrasonicPoller uspoller = new UltrasonicPoller(usDist, sample, master, gps);
		if (greenTeam == 3) {
			LinkedList<Integer> coordsList = new LinkedList<Integer>();
			coordsList.addLast(zipgreenXc);
			coordsList.addLast(zipgreenYc);
			coordsList.addLast(LLSRRX);
			coordsList.addLast(LLSRRY);
			coordsList.addLast(SHLLX);
			coordsList.addLast(SHLLY);
			coordsList.addLast(SHURX);
			coordsList.addLast(SHURY);
			coordsList.addLast(SVLLX);
			coordsList.addLast(SVLLY);
			coordsList.addLast(11);
			coordsList.addLast(1);
			gps.setPath(coordsList);
			gps.setSearchRegionPath(LLSRRX, LLSRRY, LLSRRX, URSRRY, URSRRX, URSRRY, URSRRX, LLSRRY);
		} else {
			LinkedList<Integer> coordsList = new LinkedList<Integer>();
			coordsList.addLast(SHLLX);
			coordsList.addLast(SHLLY);
			coordsList.addLast(SHURX);
			coordsList.addLast(SHURY);
			coordsList.addLast(SVLLX);
			coordsList.addLast(SVLLY);
			coordsList.addLast(URSRGX);
			coordsList.addLast(URSRGY);
			coordsList.addLast(zipgreenXc);
			coordsList.addLast(zipgreenYc);
			coordsList.addLast(1);
			coordsList.addLast(11);

			gps.setPath(coordsList);
			gps.setSearchRegionPath(URSRGX, URSRGX, URSRGX, LLSRGY, LLSRGX, LLSRGY, LLSRGX, URSRGY);
		}
		stage = Stage.STARTINGLOCALIZATION;
		LocalizationType lt = LocalizationType.FALLINGEDGE;
		t.clear();
		uspoller.on();
		uspoller.start();
		odometer.start();
		odometryDisplay.start();
		master.start();
		UltrasonicLocalizer usLoc = new UltrasonicLocalizer(odometer, gps, lt, uspoller);

		usLoc.doLocalization();
		while (usLoc.localizing)
			continue;
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		}
		LightPoller colorpoller = new LightPoller(colorSensor, colorProvider);
		gps.setColorProvider(colorpoller);
		LightPoller leftpoller = new LightPoller(leftSensor, leftProvider);
		LightPoller rightpoller = new LightPoller(rightSensor, rightProvider);
		LightLocalizer lightLoc = new LightLocalizer(odometer, gps, leftpoller, rightpoller);
		leftpoller.start();
		rightpoller.start();
		lightLoc.startLightLOC();
		// clear the display
		/*
		 * STOPPED HERE FOR NOW!!!!!!!!!!!!!!! 11/1/2017 3:40-10:04 PM
		 * 
		 * 
		 * 
		 * 
		 * 
		 * 
		 * 
		 * 
		 */
		// wait for the user to press a button and start the odometer and
		// odometer display
		Button.waitForAnyPress();
		if (greenTeam == 3) {
			odometer.setX(11 * TILE_SPACING);
			odometer.setY(TILE_SPACING);
			odometer.setTheta(3 * Math.PI / 2);
		} else {
			odometer.setX(TILE_SPACING);
			odometer.setY(11 * TILE_SPACING);
			odometer.setTheta(Math.PI / 2);
		}
		stage = Stage.NAVIGATION;
		sensorMotor.start();
		leftpoller.start();
		rightpoller.start();
		master.start();
		while (true) {
			if (stage == Stage.NAVIGATION) {

				navigating(gps, sensorMotor, leftpoller, rightpoller);
			} else if (stage == Stage.FLAGSEARCH) {
				flagsearch(gps, leftpoller, rightpoller, master, sensorMotor, colorpoller);
				stage = Stage.NAVIGATION;
			} else if (stage == Stage.ZIPLOCALIZATION) {
				ziplocalization(gps, leftpoller, rightpoller, master, lightLoc, sensorMotor, colorpoller);
				stage = Stage.NAVIGATION;
			} else if (stage == Stage.FINISHED) {
				Sound.beepSequenceUp();
				System.exit(0);
			}
		}

		/*
		 * while (stage != Stage.NAVIGATION) { if (stage == Stage.FLAGSEARCH) {
		 * flagsearch(gps, leftpoller, rightpoller, master, sensorMotor, colorpoller);
		 * stage = Stage.NAVIGATION; } else if (stage == Stage.ZIPLOCALIZATION) {
		 * ziplocalization(gps, leftpoller, rightpoller, master, lightLoc, sensorMotor,
		 * colorpoller); stage = Stage.NAVIGATION; } } while (stage != Stage.FINISHED) {
		 * sensorMotor.on(); leftpoller.on(); rightpoller.on(); navigating(gps); } while
		 * (Button.waitForAnyPress() != Button.ID_ESCAPE) ; System.exit(0);
		 */}

	public static void flagsearch(Navigation gps, LightPoller leftpoller, LightPoller rightpoller, Avoidance master,
			SensorRotation sensorMotor, LightPoller colorpoller) {
		boolean foundFlag = false;
		leftpoller.off();
		rightpoller.off();
		master.off();
		sensorMotor.off();
		colorpoller.on();
		colorpoller.start();
		if (greenTeam == 3)
			foundFlag = gps.flagSearch(greenColor);
		else
			foundFlag = gps.flagSearch(redColor);
		if (foundFlag)
			return;
	}

	public static void ziplocalization(Navigation gps, LightPoller leftpoller, LightPoller rightpoller,
			Avoidance master, LightLocalizer loc, SensorRotation sensorMotor, LightPoller colorpoller) {
		leftpoller.off();
		rightpoller.off();
		master.off();
		sensorMotor.off();
		colorpoller.off();
		loc.lightLocWithError();
		odometer.setX(zipgreenXc);
		odometer.setY(zipgreenYc);
		gps.travelTo(zipgreenX, zipgreenY);
		gps.zipTraversal();
		odometer.setX(zipredX);
		odometer.setY(zipredY);
		gps.travelTo(zipredXc, zipredYc);
		loc.lightLocWithError();
		odometer.setX(zipredXc);
		odometer.setY(zipredYc);
		stage = Stage.NAVIGATION;
	}

	public static void navigating(Navigation gps, SensorRotation motor, LightPoller leftPoller,
			LightPoller rightPoller) {
		motor.on();
		leftPoller.on();
		rightPoller.on();
		stage = Stage.NAVIGATION;
		gps.startNav();
	}
}

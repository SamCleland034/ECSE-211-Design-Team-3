//TESTTT

package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

/**
 * Date 10/25/17 Preliminary version of the DPM final project using the final
 * code from lab 5 Still need to incorporate additional functionality such as
 * odometry correction and flag search.
 * 
 * @author Sam Cleland, Yiming Wu, Charles Brana
 * @version 1.0
 * 
 */
public class FinalProject extends Thread {

	/** x coord of the zipline in the green region. */
	public int zipgreenX;

	/** y coord of the zipline in the green region. */
	public int zipgreenY;

	/** xc coord of the zipline in the green region. */
	public int zipgreenXc;

	/** yc coord of the zipline in the green region. */
	public int zipgreenYc;

	/** x coord of the zipline in the red region. */
	public int zipredX;

	/** y coord of the zipline in the red region. */
	public int zipredY;

	/** xc coord of the zipline in the red region. */
	public int zipredXc;

	/** yc coord of the zipline in the red region. */
	public int zipredYc;

	/** x coord of the lower left corner of green search region. */
	public int LLSRGX;

	/** y coord of the lower left corner of green search region. */
	public int LLSRGY;

	/** x coord of the upper right corner of green search region. */
	public int UPSRGX;

	/** y coord of the upper right corner of green search region. */
	public int UPSRGY;

	/** x coord of the lower left corner of red search region. */
	public int LLSRRX;

	/** y coord of the lower left corner of red search region. */
	public int LLSRRY;

	/** x coord of the upper right corner of red search region. */
	public int UPSRRX;

	/** y coord of the upper right corner of red search region. */
	public int UPSRRY;

	/** x coord of lower left corner of horizontal shallow water region. */
	public int SHLLX;

	/** y coord of lower left corner of horizontal shallow water region. */
	public int SHLLY;

	/** x coord of upper right corner of horizontal shallow water region. */
	public int SHURX;

	/** y coord of upper right corner of horizontal shallow water region. */
	public int SHURY;
	/** x coord of lower left corner of vertical shallow water region. */
	public int SVLLX;

	/** y coord of lower left corner of vertical shallow water region. */
	public int SVLLY;

	/** x coord of upper right corner of vertical shallow water region. */
	public int SVURX;

	/** y coord of upper right corner of vertical shallow water region. */
	public int SVURY;

	/** The Constant leftMotor, global left motor for entire project. */
	// Assign ports to motors and to sensor
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	/** The Constant zipMotor, used for the zip motor, global access. */
	public static final EV3LargeRegulatedMotor zipMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	/** The Constant rightMotor, global right motor for entire project. */
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	/** The Constant usSensor. Ultrasonic sensor used */
	public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));

	/** The odometer. */
	private Odometer odometer;

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
	public static final double TRACK = 15.13; // Width of car

	/** The x. */
	private static int x = 0;

	/** The y. */
	private static int y = 0;

	/** The xc. */
	private static int xc = 0;

	/** The yc. */
	private static int yc = 0;

	/** The Constant LightPort. */
	private static final Port LightPort = LocalEV3.get().getPort("S4");

	/**
	 * The main method.
	 *
	 * @param args
	 *            the arguments
	 */
	public static void main(String[] args) {
		int buttonChoice;

		// instantiate threads controlling the robot
		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);

		EV3ColorSensor colorSensor = new EV3ColorSensor(LightPort);
		colorSensor.setFloodlight(lejos.robotics.Color.WHITE);
		// clear the display
		t.clear();

		// ask the user to input x and y
		t.drawString("Select X and Y  ", 0, 0);
		t.drawString("                ", 0, 1);
		t.drawString("                ", 0, 2);
		t.drawString("                ", 0, 3);
		t.drawString("                ", 0, 4);

		// wait for the user to press a button and start the odometer and
		// odometer display
		buttonChoice = Button.waitForAnyPress();

		while (buttonChoice != Button.ID_ENTER) {
			// increment or decrement x and y depending on button pressed
			if (buttonChoice == Button.ID_RIGHT) {
				if (x < 8) {
					x++;
				}
				t.clear();
				t.drawString("X:", 0, 0);
				t.drawInt(x, 3, 0);
				t.drawString("Y:", 0, 2);
				t.drawInt(y, 3, 2);
			}

			if (buttonChoice == Button.ID_LEFT) {
				if (x > 0) {
					x--;
				}
				t.clear();
				t.drawString("X:", 0, 0);
				t.drawInt(x, 3, 0);
				t.drawString("Y:", 0, 2);
				t.drawInt(y, 3, 2);
			}

			if (buttonChoice == Button.ID_UP) {
				if (y < 8) {
					y++;
				}
				t.clear();
				t.drawString("X:", 0, 0);
				t.drawInt(x, 3, 0);
				t.drawString("Y:", 0, 2);
				t.drawInt(y, 3, 2);
			}

			if (buttonChoice == Button.ID_DOWN) {
				if (y > 0) {
					y--;
				}

				t.clear();
				t.drawString("X:", 0, 0);
				t.drawInt(x, 3, 0);
				t.drawString("Y:", 0, 2);
				t.drawInt(y, 3, 2);
			}

			buttonChoice = Button.waitForAnyPress();
		}

		t.clear();

		// ask the user to input Xc and Yc
		t.drawString("Select Xc and Yc  ", 0, 0);
		t.drawString("                  ", 0, 1);
		t.drawString("                  ", 0, 2);
		t.drawString("                  ", 0, 3);
		t.drawString("                  ", 0, 4);

		buttonChoice = Button.waitForAnyPress();

		// increment or decrement Xc and Yc depending onbutton pressed
		while (buttonChoice != Button.ID_ENTER) {

			if (buttonChoice == Button.ID_RIGHT) {
				if (xc < 8) {
					xc++;
				}
				t.clear();
				t.drawString("Xc:", 0, 0);
				t.drawInt(xc, 3, 0);
				t.drawString("Yc:", 0, 2);
				t.drawInt(yc, 3, 2);
			}

			if (buttonChoice == Button.ID_LEFT) {
				if (xc > 0) {
					xc--;
				}
				t.clear();
				t.drawString("Xc:", 0, 0);
				t.drawInt(xc, 3, 0);
				t.drawString("Yc:", 0, 2);
				t.drawInt(yc, 3, 2);
			}

			if (buttonChoice == Button.ID_UP) {
				if (yc < 8) {
					yc++;
				}
				t.clear();
				t.drawString("Xc:", 0, 0);
				t.drawInt(xc, 3, 0);
				t.drawString("Yc:", 0, 2);
				t.drawInt(yc, 3, 2);
			}

			if (buttonChoice == Button.ID_DOWN) {
				if (yc > 0) {
					yc--;
				}

				t.clear();
				t.drawString("Xc:", 0, 0);
				t.drawInt(xc, 3, 0);
				t.drawString("Yc:", 0, 2);
				t.drawInt(yc, 3, 2);
			}
			buttonChoice = Button.waitForAnyPress();
		}

		// ask user to select corner that the robot starts in
		t.drawString("   Select SP    ", 0, 0);
		t.drawString("       0        ", 0, 1);
		t.drawString("  3         1   ", 0, 2);
		t.drawString("       2        ", 0, 3);
		t.drawString("                ", 0, 4);

		buttonChoice = Button.waitForAnyPress();

		double coordinate[][] = { { x, y }, { xc, yc } }; // coordinates needed to navigate
		// instantiate navigation and light localizzer
		Navigation navigation = new Navigation(odometer, coordinate, leftMotor, rightMotor);
		LightLocalizer lightLocalizer = new LightLocalizer(odometer, navigation, colorSensor);
		// position 0
		if (buttonChoice == Button.ID_UP) {
			t.clear(); // clear display
			odometer.start(); // start odometer and display
			odometryDisplay.start();
			// instantiate ultrasonic localizer and choose falling edge
			UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, odometer, navigation,
					UltrasonicLocalizer.LocalizationType.FALLING_EDGE);
			usLocalizer.start();
			Button.waitForAnyPress();
			// start light localization
			lightLocalizer.startLightLOC();
			Button.waitForAnyPress();
			// set odometer values and travel to given x and y values
			odometer.setTheta(0);// this is angle that the robot localizes to from this corner
			odometer.setX(1 * TILE_SPACING);
			odometer.setY(1 * TILE_SPACING);
			navigation.travelTo(x, y);

			Button.waitForAnyPress();
			navigation.turn(Math.toRadians(5)); // this value was determined experimentally to correct error

			lightLocalizer.lightLocWithError(); // correct the localization
												// incase it is off
			boolean xLine = lightLocalizer.correctLocalization();
			if (xLine)
				odometer.setTheta(Math.PI / 2 + Math.toRadians(10)); // assuming values based on
			else
				odometer.setTheta(Math.toRadians(10)); // what line is crossed
			odometer.setX(x * TILE_SPACING); // set x and y
			odometer.setY(y * TILE_SPACING);

			navigation.turnTo(Math.PI / 2); // turn to correct angle to mount zipline

			Button.waitForAnyPress();
			navigation.travelTo(xc, yc);
			Button.waitForAnyPress();
			// traverse the zipline
			navigation.zipTraversal();
		}
		// position 1
		// see comments above for position 0
		else if (buttonChoice == Button.ID_RIGHT) {
			t.clear();
			odometer.start();
			odometryDisplay.start();
			UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, odometer, navigation,
					UltrasonicLocalizer.LocalizationType.FALLING_EDGE);
			usLocalizer.start();
			Button.waitForAnyPress();
			lightLocalizer.startLightLOC();
			Button.waitForAnyPress();
			odometer.setTheta(3 * Math.PI / 2);// this is angle that the robot localizes to from this corner
			odometer.setX(7 * TILE_SPACING);
			odometer.setY(1 * TILE_SPACING);
			navigation.travelTo(1, 1);
			navigation.travelTo(x, y);
			Button.waitForAnyPress();
			lightLocalizer.lightLocWithError();
			boolean xLine = lightLocalizer.correctLocalization();
			if (xLine)
				odometer.setTheta(Math.PI / 2 + Math.toRadians(5));
			else
				odometer.setTheta(0 + Math.toRadians(5));
			odometer.setX(x * TILE_SPACING);
			odometer.setY(y * TILE_SPACING);

			Button.waitForAnyPress();
			navigation.travelTo(xc, yc);
			Button.waitForAnyPress();
			navigation.zipTraversal();
		}
		// position 2
		// see comments above for position 0
		else if (buttonChoice == Button.ID_DOWN) {
			t.clear();
			odometer.start();
			odometryDisplay.start();
			UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, odometer, navigation,
					UltrasonicLocalizer.LocalizationType.FALLING_EDGE);
			usLocalizer.start();
			Button.waitForAnyPress();
			lightLocalizer.startLightLOC();
			Button.waitForAnyPress();
			odometer.setTheta(Math.PI);// this is angle that the robot localizes to from this corner
			odometer.setX(7 * TILE_SPACING);
			odometer.setY(7 * TILE_SPACING);
			navigation.travelTo(7, y - 1);
			navigation.travelTo(x, y - 1);
			navigation.travelTo(x, y);
			Button.waitForAnyPress();
			odometer.setX(0);
			odometer.setY(0);
			lightLocalizer.lightLocWithError();
			boolean xLine = lightLocalizer.correctLocalization();
			if (xLine)
				odometer.setTheta(Math.PI / 2 + Math.toRadians(5));
			else
				odometer.setTheta(0 + Math.toRadians(5));
			odometer.setX(x * TILE_SPACING);
			odometer.setY(y * TILE_SPACING);

			Button.waitForAnyPress();
			navigation.travelTo(xc, yc);
			Button.waitForAnyPress();
			navigation.zipTraversal();
		}
		// position 3
		// see comments above for position 0
		else if (buttonChoice == Button.ID_LEFT) {
			t.clear();
			odometer.start();
			odometryDisplay.start();
			UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, odometer, navigation,
					UltrasonicLocalizer.LocalizationType.FALLING_EDGE);
			usLocalizer.start();
			Button.waitForAnyPress();
			lightLocalizer.startLightLOC();
			Button.waitForAnyPress();
			odometer.setTheta(Math.PI / 2); // this is angle that the robot localizes to from this corner
			odometer.setX(1 * TILE_SPACING);
			odometer.setY(7 * TILE_SPACING);
			navigation.travelTo(x, y - 1); // go to this coord to use same localization
			navigation.travelTo(x, y);
			Button.waitForAnyPress();
			lightLocalizer.lightLocWithError();
			boolean xLine = lightLocalizer.correctLocalization();
			if (xLine)
				odometer.setTheta(Math.PI / 2 + Math.toRadians(5));
			else
				odometer.setTheta(0 + Math.toRadians(5));
			odometer.setX(x * TILE_SPACING);
			odometer.setY(y * TILE_SPACING);

			Button.waitForAnyPress();
			navigation.travelTo(xc, yc);
			Button.waitForAnyPress();
			navigation.zipTraversal();
		}

		// exit the program
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

}

//TESTTT

package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.*;

public class Lab5 extends Thread {

	// Assign ports to motors and to sensor
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor zipMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
	private Odometer odometer;

	// create variables
	static SampleProvider usDist = usSensor.getMode("Distance");
	static float[] sample = new float[usDist.sampleSize()];
	public static final double TILE_SPACING = 30.48;
	public static final double WHEEL_RADIUS = 2.145; // radius of wheel
	public static final double TRACK = 15.13; // Width of car
	private static double origin[][] = { { 0, 0 }, { 1, 1 } };
	private static int x = 0;
	private static int y = 0;
	private static int xc = 0;
	private static int yc = 0;
	private static final Port LightPort = LocalEV3.get().getPort("S4");

	/**
	 * @param args
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
			odometer.setTheta(0);//this is angle that the robot localizes to from this corner
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
		//see comments above for position 0
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
			odometer.setTheta(3 * Math.PI / 2);//this is angle that the robot localizes to from this corner
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
		//see comments above for position 0
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
			odometer.setTheta(Math.PI);//this is angle that the robot localizes to from this corner
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
		//see comments above for position 0
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
			odometer.setTheta(Math.PI / 2); //this is angle that the robot localizes to from this corner
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

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

}

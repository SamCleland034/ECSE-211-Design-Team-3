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
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
	private Odometer odometer;

	static SampleProvider usDist = usSensor.getMode("Distance");
	static float[] sample = new float[usDist.sampleSize()];

	public static final double WHEEL_RADIUS = 2.145; // radius of wheel
	public static final double TRACK = 15.15; // Width of car
	private static double origin[][] = { { 0, 0 } };

	public static void main(String[] args) {
		int buttonChoice;

		// instantiate threads controlling the robot
		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);

		// clear the display
		t.clear();

		// ask the user whether the robot should use rising edge or falling edge
		t.drawString("< Ris- | Fall- >", 0, 0);
		t.drawString("  ing  | ing    ", 0, 1);
		t.drawString("  edge | edge   ", 0, 2);
		t.drawString("       |        ", 0, 3);
		t.drawString("       |        ", 0, 4);

		// wait for the user to press a button and start the odometer and
		// odometer display
		buttonChoice = Button.waitForAnyPress();

		odometer.start();
		odometryDisplay.start();

		if (buttonChoice == Button.ID_RIGHT) {
			// create navigation and start ultrasonic localizer thread with
			// falling edge
			Navigation navigation = new Navigation(odometer, origin, leftMotor, rightMotor);
			UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, odometer, navigation,
					UltrasonicLocalizer.LocalizationType.FALLING_EDGE);
			usLocalizer.start();

			// wait for the user to press any button and start light localizer
			// thread

			Button.waitForAnyPress();

			LightLocalizer lightLocalizer = new LightLocalizer(odometer, navigation);
			lightLocalizer.start();
		}

		if (buttonChoice == Button.ID_LEFT) {
			// create navigation and start ultrasonic localizer thread with
			// rising edge
			Navigation navigation = new Navigation(odometer, origin, leftMotor, rightMotor);
			UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, odometer, navigation,
					UltrasonicLocalizer.LocalizationType.RISING_EDGE);
			usLocalizer.start();

			// wait for the user to press any button and start light localizer
			// thread

			Button.waitForAnyPress();

			LightLocalizer lightLocalizer = new LightLocalizer(odometer, navigation);
			lightLocalizer.start();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
}

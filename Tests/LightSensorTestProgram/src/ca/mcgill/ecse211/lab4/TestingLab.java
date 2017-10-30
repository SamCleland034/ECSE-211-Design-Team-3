package ca.mcgill.ecse211.lab4;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class TestingLab {
	public static final double BAND_CENTER = 7.5;
	public static final double WHEEL_RADIUS = 2.1;
	public static final double errorRange = 2.5;
	public static final double TRACK = 10.2;//////////////////////////////////////////////// 12.00
	public static final double TILE_SPACING = 30.48;
	public static final int ROTATE_SPEED = 125;
	public static final int NORMAL_SPEED = 200;
	public static final int HIGH_SPEED = 400;
	public static final int LOW_SPEED = 100;
	public static final int FILTER_OUT = 40;
	public static final double SENSOR_LENGTH = 14.2;
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	public static void main(String[] args) {
		int buttonChoice;
		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);

		EV3UltrasonicSensor us = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
		EV3ColorSensor ls = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
		SampleProvider provider = ls.getRGBMode();
		float[] sample = new float[provider.sampleSize()];
		Poller usPoller = new Poller(us, provider, sample);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t, usPoller);
		Navigation gps = new Navigation(odometer, usPoller, leftMotor, rightMotor);
		us.disable();

		// LightLocalizer ll= new LightLocalizer(gps, odometer, ls, leftMotor,
		// rightMotor);
		t.clear();
		ls.getRedMode();
		TestLightSensor test = new TestLightSensor(ls, provider);
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		leftMotor.forward();
		rightMotor.forward();
		test.start();

		while (!test.done)
			;
		leftMotor.rotate(-convertDistance(WHEEL_RADIUS, 2.9 * TILE_SPACING), true);
		rightMotor.rotate(-convertDistance(WHEEL_RADIUS, 2.9 * TILE_SPACING), false);
		while (leftMotor.isMoving() && rightMotor.isMoving())
			continue;
		leftMotor.stop(true);
		rightMotor.stop(false);
		System.exit(0);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);

	}
}

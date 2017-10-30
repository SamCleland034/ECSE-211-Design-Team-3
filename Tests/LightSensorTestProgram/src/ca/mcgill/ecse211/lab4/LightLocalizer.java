package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;

public class LightLocalizer {
	private final Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private final Navigation gps;
	private final EV3ColorSensor ls;
	private boolean lineDetected = false;
	private int lightVal;
	private int lastLightVal = 6;

	public LightLocalizer(Navigation gps, Odometer odometer, EV3ColorSensor ls, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odometer;
		this.gps = gps;
		this.ls = ls;

	}

	public void localization() {
		ls.getColorIDMode();
		float[] lightSample = new float[ls.sampleSize()];
		int sample = 0;
		double angles[] = new double[4];
		double position[] = new double[3];
		// gps.turnTo(35);
		leftMotor.setSpeed(TestingLab.LOW_SPEED);
		rightMotor.setSpeed(TestingLab.LOW_SPEED);
		leftMotor.forward();
		rightMotor.forward();

		try {
			Thread.sleep(1600);
		} catch (InterruptedException e) {
		}
		int lineCount = 0;
		while (lineCount < 4) {
			leftMotor.setSpeed(TestingLab.ROTATE_SPEED);
			rightMotor.setSpeed(TestingLab.ROTATE_SPEED);
			leftMotor.forward();
			rightMotor.backward();
			ls.fetchSample(lightSample, 0);
			sample = (int) lightSample[0];
			if (((double) Math.abs(sample - lastLightVal) / lastLightVal) >= 1.0) {
				Sound.beep();

				angles[lineCount] = odometer.getTheta();
				lineCount++;
			}
			lastLightVal = sample;
		}
		leftMotor.stop(true);
		rightMotor.stop(false);
		double deltaX = angles[2] - angles[0];
		double deltaY = angles[3] - angles[1];
		double newX = TestingLab.SENSOR_LENGTH * Math.cos(deltaX / 2);
		double newY = TestingLab.SENSOR_LENGTH * Math.cos(deltaY / 2);
		double deltaTheta = (deltaX / 2) + ((Math.PI) / 2) - (angles[3] - Math.PI);
		if (deltaTheta > Math.PI) {
			deltaTheta += Math.PI;
		}
		double thetaPrev = odometer.getTheta();
		odometer.setTheta(deltaTheta + odometer.getTheta());
		gps.travelTo(0, 0);
		gps.turnTo(0);
	}
}

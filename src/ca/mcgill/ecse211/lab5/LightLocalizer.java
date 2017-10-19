/*
 * SquareDriver.java
 */
package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class LightLocalizer extends Thread {
	// distance between sensor and rotation center
	private static final double SENSOR_OFFSET = 15.6;
	private static final long CORRECTION_PERIOD = 10;
	private static final int MOTOR_SPEED = 100;

	// create variables
	private Odometer odometer;
	private Navigation navigation;

	// Define variables needed
	private int nbrOfLines = 0;
	private double thetaX1 = 0; // Angles that need to be saved
	private double thetaX2 = 0;
	private double thetaY1 = 0;
	private double thetaY2 = 0;
	private double thetaY = 0;
	private double thetaX = 0;

	// assign port to light sensor
	private static final Port LightPort = LocalEV3.get().getPort("S4");

	public LightLocalizer(Odometer odometer, Navigation navigation) {
		this.odometer = odometer;
		this.navigation = navigation;
	}

	public void run() {
		long correctionStart, correctionEnd;

		// initialize color sensor
		@SuppressWarnings("resource")
		EV3ColorSensor colorSensor = new EV3ColorSensor(LightPort);
		colorSensor.setFloodlight(lejos.robotics.Color.WHITE);

		Sound.beepSequenceUp();

		odometer.setTheta(0); // Initialize theta, it will be corrected

		// the following code enables the robot to position itself so that the
		// light sensor will hit all four lines
		Lab5.leftMotor.setSpeed(MOTOR_SPEED); // set speeds
		Lab5.rightMotor.setSpeed(MOTOR_SPEED);

		Lab5.leftMotor.forward(); // Run forward
		Lab5.rightMotor.forward();

		boolean crossedLine = false; // Set flag

		// Before starting turning, make the robot go to (-25, -25)
		while (!crossedLine) { // Set the crossedLine flag to be true when it
								// crosses a line
			// get sample from light sensor
			colorSensor.getColorIDMode();
			SampleProvider provider = colorSensor.getMode("ColorID");
			float colorSamples[] = new float[100];

			colorSensor.fetchSample(colorSamples, 1);
			int color = (int) colorSamples[1];

			// when the sensor sees a black line, stop the motors
			if (color == 13) {
				Sound.beep();
				crossedLine = true;
				Lab5.leftMotor.stop();
				Lab5.rightMotor.stop();
			}
		}

		// once the sensor sees the black line, drive 25 cm backwards
		navigation.driveWithoutAvoid(-25);

		navigation.turnTo(90); // turn to 90 degrees

		crossedLine = false; // set flag back to false

		// drive forward until the sensor crosses a black line
		Lab5.leftMotor.forward();
		Lab5.rightMotor.forward();

		while (!crossedLine) {
			// get sample from sensor
			colorSensor.getColorIDMode();
			SampleProvider provider = colorSensor.getMode("ColorID");
			float colorSamples[] = new float[100];

			colorSensor.fetchSample(colorSamples, 1);
			int color = (int) colorSamples[1];
			// when the robot crosses a black line, stop the motors
			if (color == 13) {
				Sound.beep();
				crossedLine = true;
				Lab5.leftMotor.stop();
				Lab5.rightMotor.stop();
			}
		}

		// drive 25 cm backwards and turn back to 0 degrees
		navigation.driveWithoutAvoid(-25);

		navigation.turnTo(0);

		// turn 360 degrees
		navigation.turn(360);

		// while the robot is turning, fetch the color from the color sensor and
		// save the values of theta when the sensor crosses a black line
		while (navigation.isNavigating()) {

			correctionStart = System.currentTimeMillis();

			// get color detected by light sensor
			colorSensor.getColorIDMode();
			SampleProvider provider = colorSensor.getMode("ColorID");
			float colorSamples[] = new float[100];

			colorSensor.fetchSample(colorSamples, 1);
			int color = (int) colorSamples[1];

			// if color is black, beep, increment the number of lines crossed
			// and save value of theta
			if (color == 13) {
				Sound.beepSequenceUp();
				nbrOfLines++;

				// save value of theta to the appropriate variable, depending on
				// which line is being crossed
				if (nbrOfLines == 1) {
					thetaX1 = odometer.getTheta();
				} else if (nbrOfLines == 2) {
					thetaY1 = odometer.getTheta();
				} else if (nbrOfLines == 3) {
					thetaX2 = odometer.getTheta();
				} else if (nbrOfLines == 4) {
					thetaY2 = odometer.getTheta();
				}
			}

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}

		Sound.beep();

		// calculate values of thetaX and thetaY

		thetaX = thetaX2 - thetaX1;
		thetaY = thetaY2 - thetaY1;

		// calculate the value of deltaTheta, the -6 was determined
		// experimentally
		double deltaTheta = (Math.PI / 2.0) - thetaY2 + Math.PI + (thetaY / 2.0) - 6;

		double newTheta = odometer.getTheta() + deltaTheta;

		if (newTheta < 0) { // Keep newTheta (in radians) between 0 and 2pi
			newTheta = newTheta + 2 * Math.PI;
		} else if (newTheta > 2 * Math.PI) {
			newTheta = newTheta - 2 * Math.PI;
		}

		// for testing purposes, print the deltaTheta to be corrected
		TextLCD t = LocalEV3.get().getTextLCD();
		t.drawString("deltaTheta:" + Math.toDegrees(deltaTheta), 0, 4);

		// set x and y to correct values
		odometer.setX(-SENSOR_OFFSET * Math.cos(thetaY / 2.0));
		odometer.setY(-SENSOR_OFFSET * Math.cos(thetaX / 2.0));

		// set theta to its correct value
		odometer.setTheta(newTheta);

		// travel to the 0,0 point and turn to 0 degrees
		navigation.travelTo(0, 0);
		navigation.turnTo(0);
		// navigation.turnTo(deltaTheta);
		Sound.playNote(Sound.XYLOPHONE, 500, 500);
	}

}

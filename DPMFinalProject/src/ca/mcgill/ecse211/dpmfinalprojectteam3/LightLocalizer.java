/*
 * SquareDriver.java
 */
package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * The Class LightLocalizer, used to allow the robot, at the beginning, and at
 * times when error accumulates too much to re-adjust itself, and corrects the odometer
 * using one to potentially two light sensors to perform this task
 * 
 * @version 1.0
 */
public class LightLocalizer {

	/**
	 * The Constant SENSOR_OFFSET. Offset of the sensor from the center of the robot
	 */
	// distance between sensor and rotation center
	private static final double SENSOR_OFFSET = 15.3;

	/**
	 * The Constant CORRECTION_PERIOD. Used to sample from the light sensor at a
	 * fixed rate
	 */
	private static final long CORRECTION_PERIOD = 10;

	/** The Constant MOTOR_SPEED. Primary motor speed used for light localization */
	private static final int MOTOR_SPEED = 100;

	/** The odometer. */
	// create variables
	private Odometer odometer;

	/** The navigation. */
	private Navigation navigation;

	/** The nbr of lines. */
	// Define variables needed
	private int nbrOfLines = 0;

	/** The theta X 1. */
	private double thetaX1 = 0; // Angles that need to be saved

	/** The theta X 2. */
	private double thetaX2 = 0;

	/** The theta Y 1. */
	private double thetaY1 = 0;

	/** The theta Y 2. */
	private double thetaY2 = 0;

	/** The theta Y. */
	private double thetaY = 0;

	/** The theta X. */
	private double thetaX = 0;

	/** The color sensor. */
	private EV3ColorSensor colorSensor;

	// assign port to light sensor

	/**
	 * Instantiates a new light localizer.
	 *
	 * @param odometer
	 *            ,the odometer used for light localization
	 * @param navigation
	 *            ,the navigation we will use to travel to (0,0) once we get the
	 *            correct heading
	 * @param colorSensor
	 *            the color sensor, used to detect lines when doing light
	 *            localization
	 */
	public LightLocalizer(Odometer odometer, Navigation navigation, EV3ColorSensor colorSensor) {
		this.odometer = odometer;
		this.navigation = navigation;
		this.colorSensor = colorSensor;

	}

	/**
	 * Start light LOC, primary method for localization in one of the starting
	 * corners after ultrasonic localization has performed.
	 */
	public void startLightLOC() {
		long correctionStart, correctionEnd;
		// navigation.turn(10);
		// while(navigation.isNavigating()) continue;
		odometer.setTheta(0);
		// initialize color sensor

		colorSensor.getColorIDMode();
		SampleProvider provider = colorSensor.getMode("ColorID");
		float colorSamples[] = new float[100];
		Sound.beepSequenceUp();

		// Initialize theta, it will be corrected

		// the following code enables the robot to position itself so that the
		// light sensor will hit all four lines
		FinalProject.leftMotor.setSpeed(2 * MOTOR_SPEED); // set speeds
		FinalProject.rightMotor.setSpeed(2 * MOTOR_SPEED);

		FinalProject.leftMotor.forward(); // Run forward
		FinalProject.rightMotor.forward();

		boolean crossedLine = false; // Set flag

		// Before starting turning, make the robot go to (-25, -25)
		while (!crossedLine) { // Set the crossedLine flag to be true when it
								// crosses a line
			// get sample from light sensor

			colorSensor.fetchSample(colorSamples, 1);
			int color = (int) colorSamples[1];

			// when the sensor sees a black line, stop the motors
			if (color == 13) {
				Sound.beep();
				crossedLine = true;
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
			}
		}

		// once the sensor sees the black line, drive 25 cm backwards
		navigation.driveWithoutAvoid(-25);

		navigation.turnTo(90); // turn to 90 degrees

		crossedLine = false; // set flag back to false

		// drive forward until the sensor crosses a black line
		FinalProject.leftMotor.forward();
		FinalProject.rightMotor.forward();

		while (!crossedLine) {
			// get sample from sensor

			colorSensor.fetchSample(colorSamples, 1);
			int color = (int) colorSamples[1];
			// when the robot crosses a black line, stop the motors
			if (color == 13) {
				Sound.beep();
				crossedLine = true;
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
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

	/**
	 * Light loc with error, light localization primarily used when in an area that
	 * is not in one of the starting points, such as after dismounting from the
	 * zipline. It checks for certain cases such as if the light sensor is near or on a line.
	 * 
	 */
	// Used when getting to (x, y)
	public void lightLocWithError() {
		long correctionStart, correctionEnd;
		// initialize color sensor
		colorSensor.getColorIDMode();
		SampleProvider provider = colorSensor.getMode("ColorID");
		float colorSamples[] = new float[100];
		colorSensor.fetchSample(colorSamples, 1);
		int colortype = (int) colorSamples[1];

		// check if on a line in the beginning
		// re-position to do localization
		if (colortype == 13) {
			Sound.buzz();
			navigation.turnWithoutInterruption(-90);
			navigation.driveWithoutAvoid(10);
			navigation.turnWithoutInterruption(90);
			while (navigation.isNavigating())
				continue;
			startLightLOC();
			return;
		} else { // Check if a line is nearby the lightsensor
			boolean correct = correctPosition(colorSamples);
			if (correct) {
				startLightLOC();
				return;
			}
		}
		Sound.beepSequenceUp();

		// the following code enables the robot to position itself so that the
		// light sensor will hit all four lines
		FinalProject.leftMotor.setSpeed(2 * MOTOR_SPEED); // set speeds
		FinalProject.rightMotor.setSpeed(2 * MOTOR_SPEED);

		FinalProject.leftMotor.forward(); // Run forward
		FinalProject.rightMotor.forward();

		boolean crossedLine = false; // Set flag

		// Before starting turning, make the robot go to (-25, -25)
		while (!crossedLine) { // Set the crossedLine flag to be true when it
								// crosses a line
			// get sample from light sensor

			colorSensor.fetchSample(colorSamples, 1);
			int color = (int) colorSamples[1];

			// when the sensor sees a black line, stop the motors
			if (color == 13) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beep();
				crossedLine = true;

			}
		}

		// once the sensor sees the black line, drive 25 cm backwards
		navigation.driveWithoutAvoid(-25);
		while (navigation.isNavigating())
			continue;

		navigation.turnTo(90); // turn to 90 degrees

		crossedLine = false; // set flag back to false

		// drive forward until the sensor crosses a black line
		FinalProject.leftMotor.forward();
		FinalProject.rightMotor.forward();

		while (!crossedLine) {
			// get sample from sensor

			colorSensor.fetchSample(colorSamples, 1);
			int color = (int) colorSamples[1];
			// when the robot crosses a black line, stop the motors
			if (color == 13) {
				Sound.beep();
				crossedLine = true;
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
			}
		}

		// drive 25 cm backwards and turn back to 0 degrees
		navigation.driveWithoutAvoid(-25);
		while (navigation.isNavigating())
			continue;

		navigation.turnTo(0);

		// turn 360 degrees
		navigation.turn(360);

		// while the robot is turning, fetch the color from the color sensor and
		// save the values of theta when the sensor crosses a black line
		while (navigation.isNavigating()) {

			correctionStart = System.currentTimeMillis();

			// get color detected by light sensor

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

	/**
	 * Executed after the localization at (xo, yo) coordinates. It executes to correct
	 * the angle of the robot if the angle is off-centered assuming it corrects x and y.
	 * If the change in theta is greater than 45 degrees, we know that the robot is at 90
	 * degrees. Then, we set the boolean to true, and the boolean value will then set it
	 * to 90 or 0 degrees depending on if it is true or false.
	 *
	 * @return xLineCrossed, we return a boolean to see which line the robot crossed
	 *         if the theta value of the line is greater than 45, we know that it
	 *         must be facing to 90 degrees since after localization it will be either
	 *         slightly on the left or right of a line never more than 45 degrees to
	 *         the left or the right.
	 */
	public boolean correctLocalization() {
		// get color detected by sensor
		colorSensor.getColorIDMode();
		SampleProvider provider = colorSensor.getMode("ColorID");
		float colorSamples[] = new float[100];
		colorSensor.fetchSample(colorSamples, 1);
		int color = (int) colorSamples[1];
		if (color == 13)
			return false; // return false if black line
		boolean xLineCrossed = false; // x line isn't crossed
		double currentTheta = Math.toDegrees(odometer.getTheta());
		FinalProject.leftMotor.setSpeed(40); // set motor speed lower to detect line better
		FinalProject.rightMotor.setSpeed(40);
		FinalProject.leftMotor.forward();
		FinalProject.rightMotor.backward();
		boolean crossedLine = false; // black line has been crossed
		while (!crossedLine) { // if still not crossed
			// get color
			colorSensor.fetchSample(colorSamples, 1);
			color = (int) colorSamples[1];
			if (color == 13) {
				Sound.beep();
				double changeInTheta = Math.toDegrees(odometer.getTheta()) - currentTheta; // difference in odometer
																							// theta and current theta
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				FinalProject.leftMotor.setAcceleration(50);
				FinalProject.rightMotor.setAcceleration(50);
				navigation.turnWithSameSpeed(-6);
				FinalProject.leftMotor.setAcceleration(200);
				FinalProject.rightMotor.setAcceleration(200);
				while (navigation.isNavigating())
					continue;
				crossedLine = true;
				xLineCrossed = changeInTheta > 45;
			}

		}
		return xLineCrossed;
	}

	/**
	 * It sweeps the robot left to right to detect if it is near a line, 
	 * then it moves accordingly to the left. That way, it allows the robot to
	 * localize in an adequate location so the light sensor can brush over all
	 * four lines.
	 *
	 * @param colorSamples
	 *            the color samples
	 * @return false if it does not detect a line, true if it does then performs
	 *         normal localization like in the first method since the position of
	 *         the robot will be corrected
	 */
	public boolean correctPosition(float[] colorSamples) {
		boolean corrected = false;
		int rotations = Navigation.convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, 15);
		FinalProject.leftMotor.setSpeed(60);
		FinalProject.rightMotor.setSpeed(60);
		FinalProject.leftMotor.rotate(-rotations, true);
		FinalProject.rightMotor.rotate(rotations, true);
		double currentTheta = Math.toDegrees(odometer.getTheta());
		while (navigation.isNavigating()) {
			colorSensor.fetchSample(colorSamples, 1);
			int colortype = (int) colorSamples[1];
			if (colortype == 13) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.buzz();
				double changeInTheta = Math.toDegrees(odometer.getTheta()) - currentTheta;
				navigation.turnTo(-90 + (360 - changeInTheta));
				while (navigation.isNavigating())
					continue;

				odometer.setTheta(-Math.PI / 2);
				navigation.driveWithoutAvoid(6);
				navigation.turnTo(0);
				corrected = true;
				return corrected;
			}

		}
		FinalProject.leftMotor.rotate(2 * rotations, true);
		FinalProject.rightMotor.rotate(-2 * rotations, true);
		while (navigation.isNavigating()) {
			colorSensor.fetchSample(colorSamples, 1);
			int colortype = (int) colorSamples[1];
			if (colortype == 13) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.buzz();
				double changeInTheta = Math.toDegrees(odometer.getTheta()) - currentTheta;
				navigation.turnWithoutInterruption(-90 - changeInTheta);
				while (navigation.isNavigating())
					continue;
				odometer.setTheta(-Math.PI / 2);
				navigation.driveWithoutAvoid(7);
				navigation.turnTo(0);
				corrected = true;
				return corrected;
			}

		}
		FinalProject.leftMotor.rotate(-rotations, true);
		FinalProject.rightMotor.rotate(rotations, false);
		return corrected;
	}

}

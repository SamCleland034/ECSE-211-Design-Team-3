/*
 * SquareDriver.java
 */
package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

/**
 * The Class LightLocalizer, used to allow the robot in the beginning and in
 * times when error accumulates too much to re-adjust itself and start off fresh
 * Using one to potentially two light sensors to perform this task
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

	private LightPoller leftPoller;
	private LightPoller rightPoller;
	// assign port to light sensor

	/**
	 * Instantiates a new light localizer.
	 *
	 * @param odometer
	 *            ,the odometer used for light localization
	 * @param navigation
	 *            ,the navigation we will use to travel to 0,0 once we figure out
	 *            correct heading
	 * @param rightpoller
	 * @param leftpoller
	 * @param FinalProject.colorSensor
	 *            the color sensor, used to detect lines when doing light
	 *            localization
	 */
	public LightLocalizer(Odometer odometer, Navigation navigation, LightPoller leftpoller, LightPoller rightpoller) {
		this.odometer = odometer;
		this.navigation = navigation;
		this.leftPoller = leftpoller;
		this.rightPoller = rightpoller;

	}

	/**
	 * Start light LOC, primary method for localization in one of the starting
	 * corners after ultrasonic localization
	 */
	public void startLightLOC2() {
		// navigation.turn(10);
		// while(navigation.isNavigating()) continue;
		odometer.setTheta(0);
		// initialize color sensor
		Sound.beepSequenceUp();

		// Initialize theta, it will be corrected

		// the following code enables the robot to position itself so that the
		// light sensor will hit all four lines
		FinalProject.leftMotor.setSpeed(2 * MOTOR_SPEED); // set speeds
		FinalProject.rightMotor.setSpeed(2 * MOTOR_SPEED);

		FinalProject.leftMotor.forward(); // Run forward
		FinalProject.rightMotor.forward();

		boolean crossedLineLeft = false; // Set flag
		boolean crossedLineRight = false;
		// Before starting turning, make the robot go to (-25, -25)
		while (!(crossedLineLeft && crossedLineRight)) { // Set the crossedLine flag to be true when it
			// crosses a line
			// get sample from light sensor

			if (leftPoller.getLightVal() == 13 && rightPoller.getLightVal() == 13) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beep();
				crossedLineLeft = true;
				crossedLineRight = true;

			}
			// when the sensor sees a black line, stop the motors
			if (leftPoller.getLightVal() == 13) {
				FinalProject.leftMotor.stop(true);
				checkRightPoller();
				FinalProject.rightMotor.stop(false);
				break;

			}
			if (rightPoller.getLightVal() == 13) {
				FinalProject.rightMotor.stop(true);
				checkLeftPoller();
				FinalProject.leftMotor.stop(false);
				break;
			}

		}

		// once the sensor sees the black line, drive 25 cm backwards
		navigation.driveWithoutAvoid(-18);

		navigation.turnTo(90); // turn to 90 degrees

		crossedLineLeft = false; // set flag back to false
		crossedLineRight = false;
		// drive forward until the sensor crosses a black line
		FinalProject.leftMotor.forward();
		FinalProject.rightMotor.forward();

		while (!(crossedLineLeft && crossedLineRight)) { // Set the crossedLine flag to be true when it
			// crosses a line
			// get sample from light sensor

			// when the sensor sees a black line, stop the motors
			if (leftPoller.getLightVal() == 13 && rightPoller.getLightVal() == 13) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beep();
				crossedLineLeft = true;
				crossedLineRight = true;

			}
			// when the sensor sees a black line, stop the motors
			if (leftPoller.getLightVal() == 13) {
				FinalProject.leftMotor.stop(true);
				checkRightPoller();
				FinalProject.rightMotor.stop(false);
				break;

			}
			if (rightPoller.getLightVal() == 13) {
				FinalProject.rightMotor.stop(true);
				checkLeftPoller();
				FinalProject.leftMotor.stop(false);
				break;
			}

		}

		// drive 25 cm backwards and turn back to 0 degrees
		navigation.driveWithoutAvoid(-25);

		navigation.turnTo(0);

		// turn 360 degrees
		FinalProject.leftMotor.forward();
		FinalProject.rightMotor.backward();
		double[] leftThetas = new double[4];
		double[] rightThetas = new double[4];
		int rightNumCount = 0;
		int leftNumCount = 0;
		// while the robot is turning, fetch the color from the color sensor and
		// save the values of theta when the sensor crosses a black line
		while (rightNumCount < 4 && leftNumCount < 4) {
			// get color detected by light sensor

			if (leftPoller.getLightVal() == 13) {

				leftThetas[leftNumCount] = odometer.getTheta();
				leftNumCount += 1;
				Sound.beep();
			}
			if (rightPoller.getLightVal() == 13) {
				rightThetas[rightNumCount] = odometer.getTheta();
				rightNumCount += 1;
				Sound.beep();
			}
			// if color is black, beep, increment the number of lines crossed
			// and save value of theta
		}

		// this ensure the odometry correction occurs only once every period
		/*
		 * correctionEnd = System.currentTimeMillis(); if (correctionEnd -
		 * correctionStart < CORRECTION_PERIOD) { try { Thread.sleep(CORRECTION_PERIOD -
		 * (correctionEnd - correctionStart)); } catch (InterruptedException e) { //
		 * there is nothing to be done here because it is not // expected that the
		 * odometry correction will be // interrupted by another thread } }
		 */

		// calculate values of thetaX and thetaY
		thetaX2 = (leftThetas[2] + rightThetas[2]) / 2;
		thetaX1 = (leftThetas[0] + rightThetas[0] / 2);
		thetaY2 = (leftThetas[3] + rightThetas[3]) / 2;
		thetaY1 = (leftThetas[1] + rightThetas[1]) / 2;
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

	public void startLightLOC3() {
		// navigation.turn(10);
		// while(navigation.isNavigating()) continue;
		odometer.setTheta(0);
		// initialize color sensor
		Sound.beepSequenceUp();

		// Initialize theta, it will be corrected

		// the following code enables the robot to position itself so that the
		// light sensor will hit all four lines
		FinalProject.leftMotor.setSpeed(2 * MOTOR_SPEED); // set speeds
		FinalProject.rightMotor.setSpeed(2 * MOTOR_SPEED);

		FinalProject.leftMotor.forward(); // Run forward
		FinalProject.rightMotor.forward();

		boolean crossedLineLeft = false; // Set flag
		boolean crossedLineRight = false;

		// Before starting turning, make the robot go to (-25, -25)
		while (!(crossedLineLeft && crossedLineRight)) { // Set the crossedLine flag to be true when it
			// crosses a line
			// get sample from light sensor

			if (leftPoller.getChangeInLight() >= 1 && rightPoller.getChangeInLight() >= 1) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beep();
				crossedLineLeft = true;
				crossedLineRight = true;

			}
			// when the sensor sees a black line, stop the motors
			if (leftPoller.getChangeInLight() >= 1) {
				FinalProject.leftMotor.stop(true);
				checkRightPoller1();
				FinalProject.rightMotor.stop(false);
				break;

			}
			if (rightPoller.getChangeInLight() >= 1) {
				FinalProject.rightMotor.stop(true);
				checkLeftPoller1();
				FinalProject.rightMotor.stop(false);
				break;
			}

		}

		// once the sensor sees the black line, drive 25 cm backwards
		navigation.driveWithoutAvoid(-18);

		navigation.turnTo(90); // turn to 90 degrees

		crossedLineLeft = false; // set flag back to false
		crossedLineRight = false;
		// drive forward until the sensor crosses a black line
		FinalProject.leftMotor.forward();
		FinalProject.rightMotor.forward();

		while (!(crossedLineLeft && crossedLineRight)) { // Set the crossedLine flag to be true when it
			// crosses a line
			// get sample from light sensor
			if (leftPoller.getChangeInLight() >= 1 && rightPoller.getChangeInLight() >= 1) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beep();
				crossedLineLeft = true;
				crossedLineRight = true;

			}
			// when the sensor sees a black line, stop the motors
			if (leftPoller.getChangeInLight() >= 1) {
				FinalProject.leftMotor.stop(true);
				checkRightPoller1();
				break;

			}
			if (rightPoller.getChangeInLight() >= 1) {
				FinalProject.rightMotor.stop(true);
				checkLeftPoller1();
				break;
			}
		}

		// drive 25 cm backwards and turn back to 0 degrees
		navigation.driveWithoutAvoid(-25);

		navigation.turnTo(0);

		// turn 360 degrees
		FinalProject.leftMotor.forward();
		FinalProject.rightMotor.backward();
		double[] leftThetas = new double[4];
		double[] rightThetas = new double[4];
		int rightNumCount = 0;
		int leftNumCount = 0;
		long startTime, endTime = 0;
		// while the robot is turning, fetch the color from the color sensor and
		// save the values of theta when the sensor crosses a black line
		while (rightNumCount < 4 && leftNumCount < 4) {
			// get color detected by light sensor
			startTime = System.currentTimeMillis();
			if (leftPoller.getChangeInLight() >= 1) {
				leftThetas[leftNumCount] = odometer.getTheta();
				leftNumCount += 1;
				Sound.beep();
				checkRightPoller1();
				rightThetas[rightNumCount] = odometer.getTheta();
				rightNumCount += 1;
				Sound.beep();
			}
			if (endTime - startTime < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (endTime - startTime));
				} catch (InterruptedException e) {
				}
			}

			// if color is black, beep, increment the number of lines crossed
			// and save value of theta
		}

		// this ensure the odometry correction occurs only once every period
		/*
		 * correctionEnd = System.currentTimeMillis(); if (correctionEnd -
		 * correctionStart < CORRECTION_PERIOD) { try { Thread.sleep(CORRECTION_PERIOD -
		 * (correctionEnd - correctionStart)); } catch (InterruptedException e) { //
		 * there is nothing to be done here because it is not // expected that the
		 * odometry correction will be // interrupted by another thread } }
		 */

		// calculate values of thetaX and thetaY
		thetaX2 = (leftThetas[2] + rightThetas[2]) / 2;
		thetaX1 = (leftThetas[0] + rightThetas[0] / 2);
		thetaY2 = (leftThetas[3] + rightThetas[3]) / 2;
		thetaY1 = (leftThetas[1] + rightThetas[1]) / 2;
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

	private void checkRightPoller() {
		while (rightPoller.getLightVal() != 13)
			continue;

	}

	private void checkLeftPoller1() {
		while (leftPoller.getChangeInLight() < 1)
			continue;

	}

	private void checkRightPoller1() {
		while (rightPoller.getChangeInLight() < 1)
			continue;

	}

	private void checkLeftPoller() {
		while (leftPoller.getLightVal() != 13)
			continue;

	}

	public void startLightLOC() {
		long correctionStart, correctionEnd;
		float[] leftsample = new float[1];
		float[] rightsample = new float[1];
		// navigation.turn(10);
		// while(navigation.isNavigating()) continue;
		odometer.setTheta(0);
		// initialize color sensor
		Sound.beepSequenceUp();

		// Initialize theta, it will be corrected

		// the following code enables the robot to position itself so that the
		// light sensor will hit all four lines
		FinalProject.leftMotor.setSpeed(2 * MOTOR_SPEED); // set speeds
		FinalProject.rightMotor.setSpeed(2 * MOTOR_SPEED);

		FinalProject.leftMotor.forward(); // Run forward
		FinalProject.rightMotor.forward();

		boolean crossedLineLeft = false; // Set flag
		boolean crossedLineRight = false;
		int colorLeft = 6;
		int lastColorLeft = 6;
		int lastColorRight = 6;
		int colorRight = 6;
		// Before starting turning, make the robot go to (-25, -25)
		while (!(crossedLineLeft && crossedLineRight)) { // Set the crossedLine flag to be true when it
			// crosses a line
			// get sample from light sensor
			FinalProject.leftProvider.fetchSample(leftsample, 0);
			colorLeft = (int) leftsample[0];
			if (colorLeft == -1)
				colorLeft = 6;
			FinalProject.rightProvider.fetchSample(rightsample, 0);
			colorRight = (int) rightsample[0];
			if (colorRight == -1)
				colorLeft = 6;
			// when the sensor sees a black line, stop the motors
			if ((colorLeft - lastColorLeft) / lastColorLeft >= 1) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beep();
				crossedLineLeft = true;
				crossedLineRight = true;

			}
			if ((colorRight - lastColorRight) / lastColorRight >= 1) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beep();
				crossedLineLeft = true;
				crossedLineRight = true;
			}
			lastColorLeft = colorLeft;
			lastColorRight = colorRight;
		}

		// once the sensor sees the black line, drive 25 cm backwards
		navigation.driveWithoutAvoid(-18);

		navigation.turnTo(90); // turn to 90 degrees

		crossedLineLeft = false; // set flag back to false
		crossedLineRight = false;
		// drive forward until the sensor crosses a black line
		FinalProject.leftMotor.forward();
		FinalProject.rightMotor.forward();

		while (!(crossedLineLeft && crossedLineRight)) { // Set the crossedLine flag to be true when it
			// crosses a line
			// get sample from light sensor

			FinalProject.leftProvider.fetchSample(leftsample, 0);
			colorLeft = (int) leftsample[0];
			if (colorLeft == -1)
				colorLeft = 6;
			FinalProject.rightProvider.fetchSample(rightsample, 0);
			colorRight = (int) rightsample[0];
			if (colorRight == -1)
				colorRight = 6;
			// when the sensor sees a black line, stop the motors
			if ((colorLeft - lastColorLeft) / lastColorLeft >= 1) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beep();
				crossedLineLeft = true;
				crossedLineRight = true;
			}
			if ((colorRight - lastColorRight) / lastColorRight >= 1) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beep();
				crossedLineLeft = true;
				crossedLineRight = true;
			}
			lastColorLeft = colorLeft;
			lastColorRight = colorRight;
		}

		// drive 25 cm backwards and turn back to 0 degrees
		navigation.driveWithoutAvoid(-25);

		navigation.turnTo(0);

		// turn 360 degrees
		FinalProject.leftMotor.forward();
		FinalProject.rightMotor.backward();
		double[] leftThetas = new double[4];
		double[] rightThetas = new double[4];
		int rightNumCount = 0;
		int leftNumCount = 0;
		// while the robot is turning, fetch the color from the color sensor and
		// save the values of theta when the sensor crosses a black line
		while (rightNumCount < 4 && leftNumCount < 4) {

			correctionStart = System.currentTimeMillis();

			// get color detected by light sensor

			FinalProject.leftProvider.fetchSample(leftsample, 0);
			colorLeft = (int) leftsample[0];
			if (colorLeft == -1)
				colorLeft = 6;
			FinalProject.rightProvider.fetchSample(rightsample, 0);
			colorRight = (int) rightsample[0];
			if (colorRight == -1)
				colorLeft = 6;
			if ((colorLeft - lastColorLeft) / lastColorLeft >= 1) {
				Sound.beep();
				leftThetas[leftNumCount] = odometer.getTheta();
				leftNumCount += 1;
			}
			if ((colorRight - lastColorRight) / lastColorRight >= 1) {
				Sound.beep();
				rightThetas[rightNumCount] = odometer.getTheta();
				rightNumCount += 1;
			}
			// if color is black, beep, increment the number of lines crossed
			// and save value of theta
			lastColorLeft = colorLeft;
			lastColorRight = colorRight;
		}

		// this ensure the odometry correction occurs only once every period
		/*
		 * correctionEnd = System.currentTimeMillis(); if (correctionEnd -
		 * correctionStart < CORRECTION_PERIOD) { try { Thread.sleep(CORRECTION_PERIOD -
		 * (correctionEnd - correctionStart)); } catch (InterruptedException e) { //
		 * there is nothing to be done here because it is not // expected that the
		 * odometry correction will be // interrupted by another thread } }
		 */

		Sound.beep();

		// calculate values of thetaX and thetaY
		thetaX2 = (leftThetas[2] + rightThetas[2]) / 2;
		thetaX1 = (leftThetas[0] + rightThetas[0] / 2);
		thetaY2 = (leftThetas[3] + rightThetas[3]) / 2;
		thetaY1 = (leftThetas[1] + rightThetas[1]) / 2;
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
	 * zipline. Have to check for certain cases such as if near or on a line
	 * 
	 */
	// Used when getting to (x, y)
	public void lightLocWithError() {
		long correctionStart, correctionEnd;
		// initialize color sensor
		float[] leftsample = new float[1];
		float[] rightsample = new float[1];
		int colorLeft = 6;
		int lastColorLeft = 6;
		int lastColorRight = 6;
		int colorRight = 6;
		FinalProject.leftProvider.fetchSample(leftsample, 0);
		colorLeft = (int) leftsample[0];
		FinalProject.rightProvider.fetchSample(rightsample, 0);
		colorRight = (int) rightsample[0];
		// check if on a line in the beginning
		// re-position to do localization
		if (colorLeft == 13 || colorRight == 13) {
			Sound.buzz();
			navigation.turnWithoutInterruption(-90);
			navigation.driveWithoutAvoid(10);
			navigation.turnWithoutInterruption(90);
			while (Navigation.isNavigating())
				continue;
			startLightLOC();
			return;
		} else { // Check if a line is nearby the lightsensor
			boolean correct = correctPosition();
			if (correct) {
				startLightLOC();
				return;
			}
		}

		// navigation.turn(10);
		// while(navigation.isNavigating()) continue;
		odometer.setTheta(0);
		// initialize color sensor
		Sound.beepSequenceUp();

		// Initialize theta, it will be corrected

		// the following code enables the robot to position itself so that the
		// light sensor will hit all four lines
		FinalProject.leftMotor.setSpeed(2 * MOTOR_SPEED); // set speeds
		FinalProject.rightMotor.setSpeed(2 * MOTOR_SPEED);

		FinalProject.leftMotor.forward(); // Run forward
		FinalProject.rightMotor.forward();

		boolean crossedLineLeft = false; // Set flag
		boolean crossedLineRight = false;

		// Before starting turning, make the robot go to (-25, -25)
		while (!(crossedLineLeft && crossedLineRight)) { // Set the crossedLine flag to be true when it
			// crosses a line
			// get sample from light sensor

			FinalProject.leftProvider.fetchSample(leftsample, 0);
			colorLeft = (int) leftsample[0];
			FinalProject.rightProvider.fetchSample(rightsample, 0);
			colorRight = (int) rightsample[0];
			// when the sensor sees a black line, stop the motors
			if ((colorLeft - lastColorLeft) / lastColorLeft >= 1) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beep();
				crossedLineLeft = true;
				crossedLineRight = true;
			}
			if ((colorRight - lastColorRight) / lastColorRight >= 1) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beep();
				crossedLineLeft = true;
				crossedLineRight = true;
			}
			lastColorLeft = colorLeft;
			lastColorRight = colorRight;
		}

		// once the sensor sees the black line, drive 25 cm backwards
		navigation.driveWithoutAvoid(-25);

		navigation.turnTo(90); // turn to 90 degrees

		crossedLineLeft = false; // set flag back to false
		crossedLineRight = false;
		// drive forward until the sensor crosses a black line
		FinalProject.leftMotor.forward();
		FinalProject.rightMotor.forward();

		while (!(crossedLineLeft && crossedLineRight)) { // Set the crossedLine flag to be true when it
			// crosses a line
			// get sample from light sensor

			FinalProject.leftProvider.fetchSample(leftsample, 0);
			colorLeft = (int) leftsample[0];
			FinalProject.rightProvider.fetchSample(rightsample, 0);
			colorRight = (int) rightsample[0];
			// when the sensor sees a black line, stop the motors
			if ((colorLeft - lastColorLeft) / lastColorLeft >= 1) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beep();
				crossedLineLeft = true;
				crossedLineRight = true;
			}
			if ((colorRight - lastColorRight) / lastColorRight >= 1) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beep();
				crossedLineLeft = true;
				crossedLineRight = true;
			}
			lastColorLeft = colorLeft;
			lastColorRight = colorRight;
		}

		// drive 25 cm backwards and turn back to 0 degrees
		navigation.driveWithoutAvoid(-25);

		navigation.turnTo(0);

		// turn 360 degrees
		FinalProject.leftMotor.forward();
		FinalProject.rightMotor.backward();
		double[] leftThetas = new double[4];
		double[] rightThetas = new double[4];
		int rightNumCount = 0;
		int leftNumCount = 0;
		// while the robot is turning, fetch the color from the color sensor and
		// save the values of theta when the sensor crosses a black line
		while (rightNumCount < 4 && leftNumCount < 4) {

			correctionStart = System.currentTimeMillis();

			// get color detected by light sensor

			FinalProject.leftProvider.fetchSample(leftsample, 0);
			colorLeft = (int) leftsample[0];
			FinalProject.rightProvider.fetchSample(rightsample, 0);
			colorRight = (int) rightsample[0];
			if ((colorLeft - lastColorLeft) / lastColorLeft >= 1) {
				Sound.beep();
				leftThetas[leftNumCount] = odometer.getTheta();
				leftNumCount += 1;
			}
			if ((colorRight - lastColorRight) / lastColorRight >= 1) {
				Sound.beep();
				rightThetas[rightNumCount] = odometer.getTheta();
				rightNumCount += 1;
			}
			// if color is black, beep, increment the number of lines crossed
			// and save value of theta
			lastColorLeft = colorLeft;
			lastColorRight = colorRight;
		}
		FinalProject.leftMotor.stop(true);
		FinalProject.rightMotor.stop(false);
		// this ensure the odometry correction occurs only once every period
		/*
		 * correctionEnd = System.currentTimeMillis(); if (correctionEnd -
		 * correctionStart < CORRECTION_PERIOD) { try { Thread.sleep(CORRECTION_PERIOD -
		 * (correctionEnd - correctionStart)); } catch (InterruptedException e) { //
		 * there is nothing to be done here because it is not // expected that the
		 * odometry correction will be // interrupted by another thread } }
		 */

		// calculate values of thetaX and thetaY
		thetaX2 = (leftThetas[2] + rightThetas[2]) / 2;
		thetaX1 = (leftThetas[0] + rightThetas[0] / 2);
		thetaY2 = (leftThetas[3] + rightThetas[3]) / 2;
		thetaY1 = (leftThetas[1] + rightThetas[1]) / 2;
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
	 * Executed after the localization at the xo, yo coordinates Do this to correct
	 * the angle of the robot if the angle is off-centered Assumes correct x and y
	 * If the change in theta is greater than 45, we know that the robot is at 90
	 * degrees So we set the boolean to true and the boolean value will then set it
	 * to 90 or 0 depending on if it is true or false.
	 *
	 * @return xLineCrossed, we return a boolean to see which line the robot crossed
	 *         if the theta value of the line is greater than 45, we know that it
	 *         must be facing to 90 since after localization it will be either
	 *         slightly on the left or right of a line never more than 45 degrees to
	 *         the left or the right.
	 */
	public boolean correctLocalization() {
		// get color detected by sensor
		int colorLeft = 0;
		int lastColorLeft = 6;
		int lastColorRight = 6;
		int colorRight = 0;
		float[] leftsample = new float[1];
		float[] rightsample = new float[1];
		FinalProject.leftProvider.fetchSample(leftsample, 0);
		colorLeft = (int) leftsample[0];
		FinalProject.rightProvider.fetchSample(rightsample, 0);
		colorRight = (int) rightsample[0];

		if (colorLeft == 13 || colorRight == 13)
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
			FinalProject.leftProvider.fetchSample(leftsample, 0);
			colorLeft = (int) leftsample[0];
			FinalProject.rightProvider.fetchSample(rightsample, 0);
			colorRight = (int) rightsample[0];
			if ((colorLeft - lastColorLeft) / lastColorLeft >= 1
					|| (colorRight - lastColorRight) / lastColorRight >= 1) {
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
				while (Navigation.isNavigating())
					continue;
				crossedLine = true;
				xLineCrossed = changeInTheta > 45;
			}
			lastColorLeft = colorLeft;
			lastColorRight = colorRight;
		}
		return xLineCrossed;
	}

	/**
	 * sweeps robot left to right to detect if it is near a line if it is near a
	 * line, then it moves accordingly to the left Doing this to allow the robot to
	 * localize in an adequate location.
	 *
	 * @param colorSamples
	 *            the color samples
	 * @return false if it doesn't detect a line, true if it does then performs
	 *         normal localization like in the first method since the position of
	 *         the robot will be corrected
	 */
	public boolean correctPosition() {
		boolean corrected = false;
		int rotations = Navigation.convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, 15);
		FinalProject.leftMotor.setSpeed(60);
		FinalProject.rightMotor.setSpeed(60);
		int colorLeft = 0;
		int lastColorLeft = 6;
		int lastColorRight = 6;
		int colorRight = 0;
		FinalProject.leftMotor.rotate(-rotations, true);
		FinalProject.rightMotor.rotate(rotations, true);
		double currentTheta = Math.toDegrees(odometer.getTheta());
		float[] leftsample = new float[1];
		float[] rightsample = new float[1];
		while (Navigation.isNavigating()) {

			FinalProject.leftProvider.fetchSample(leftsample, 0);
			colorLeft = (int) leftsample[0];
			FinalProject.rightProvider.fetchSample(rightsample, 0);
			colorRight = (int) rightsample[0];

			if ((colorLeft - lastColorLeft) / lastColorLeft >= 1
					|| (colorRight - lastColorRight) / lastColorRight >= 1) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.buzz();
				double changeInTheta = Math.toDegrees(odometer.getTheta()) - currentTheta;
				navigation.turnTo(-90 + (360 - changeInTheta));
				while (Navigation.isNavigating())
					continue;

				odometer.setTheta(-Math.PI / 2);
				navigation.driveWithoutAvoid(6);
				navigation.turnTo(0);
				corrected = true;
				return corrected;
			}
			lastColorLeft = colorLeft;
			lastColorRight = colorRight;

		}
		FinalProject.leftMotor.rotate(2 * rotations, true);
		FinalProject.rightMotor.rotate(-2 * rotations, true);
		while (Navigation.isNavigating()) {
			FinalProject.leftProvider.fetchSample(leftsample, 0);
			colorLeft = (int) leftsample[0];
			FinalProject.rightProvider.fetchSample(rightsample, 0);
			colorRight = (int) rightsample[0];

			if ((colorLeft - lastColorLeft) / lastColorLeft >= 1
					|| (colorRight - lastColorRight) / lastColorRight >= 1) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.buzz();
				double changeInTheta = Math.toDegrees(odometer.getTheta()) - currentTheta;
				navigation.turnWithoutInterruption(-90 - changeInTheta);
				while (Navigation.isNavigating())
					continue;
				odometer.setTheta(-Math.PI / 2);
				navigation.driveWithoutAvoid(7);
				navigation.turnTo(0);
				corrected = true;
				return corrected;
			}
			lastColorLeft = colorLeft;
			lastColorRight = colorRight;
		}
		FinalProject.leftMotor.rotate(-rotations, true);
		FinalProject.rightMotor.rotate(rotations, false);
		return corrected;
	}

}

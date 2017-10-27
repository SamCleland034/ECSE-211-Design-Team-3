package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.*;
import lejos.hardware.port.Port;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/***
 * This class is an updated version from lab 3 where we changed most methods in
 * order to match lab 4. It is used when we need to travel to a specific coordinate.
 * Additionally, it is used for other operations such as the flag search or zipline
 * traversal: How the robot travels the grid, based on what is passed into the constructor.
 * 
 * @version 1.0
 ***/

public class Navigation {

	/** The Constant MOTOR_SPEED. Default motor speed */
	// Create constants
	private static final int MOTOR_SPEED = 225;

	/** The Constant ROTATE_SPEED. Speed used when rotating in place */
	private static final int ROTATE_SPEED = 150;

	/** The Constant MAX_DISTANCE_WALL. */
	private static final int MAX_DISTANCE_WALL = 15;

	/** The Constant AVOID_ANGLE. */
	private static final int AVOID_ANGLE = 90;

	/** The Constant AVOID_DISTANCE. */
	private static final int AVOID_DISTANCE = 30;

	/** The Constant SENSOR_OFFSET. */
	private static final double SENSOR_OFFSET = 10;

	/** The Constant SQUARE_LENGTH. */
	private static final double SQUARE_LENGTH = 30.48;

	/** The center offset. */
	private static double CENTER_OFFSET = 1.95;

	/** The Constant XMax. */
	// the maximum and minimum x and y values possible
	private static final double XMax = 3 * SQUARE_LENGTH;

	/** The Constant XMin. */
	private static final double XMin = -1 * SQUARE_LENGTH;

	/** The Constant YMax. */
	private static final double YMax = 3 * SQUARE_LENGTH;

	/** The Constant YMin. */
	private static final double YMin = -1 * SQUARE_LENGTH;

	/** The path. */
	private static double path[][]; // For demo coordinates

	/** The odometer. */
	private static Odometer odometer;

	/** The is navigating. */
	private static boolean isNavigating = false;

	/** The distance to travel. */
	private static double distanceToTravel; // Distance to travel between points

	/**
	 * Instantiates a new navigation.
	 *
	 * @param odometer
	 *            the odometer
	 * @param path
	 *            coordinates that will be passed to this version of the navigation
	 *            at the start
	 * @param leftMotor,
	 *            set the speed of the motors
	 * @param rightMotor
	 */
	public Navigation(Odometer odometer, double[][] path, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odometer;
		this.path = path;

		// Reset the motors
		for (EV3LargeRegulatedMotor motors : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motors.stop();
			motors.setSpeed(300);
		}
	}

//	/**
//	 * Start nav, 
//	 */
//	public void startNav() {
//
//		try {
//			Thread.sleep(2000);
//		} catch (InterruptedException e) {
//			// there is nothing to be done here because it is not expected that
//			// the odometer will be interrupted by another thread
//		}
//
//		// travel to each waypoints
//		for (int i = 0; i < path.length; i++) {
//			travelTo(path[i][0], path[i][1]);
//		}
//	}

	/**
	 * Travel to, orients the robot to x and y on the grid, calls turn to if the
	 * robot is not aligned in the proper position to reach x and y. If using start
	 * nav method, it will continuously call this method until no more coordinates in
	 * the path
	 * 
	 * @param endX
	 *            x coordinate of the destination
	 * @param endY
	 *            y coordinate of the destination
	 */
	// this method makes the robot travel to coordinates passed as parameters
	public void travelTo(double endX, double endY) {

		isNavigating = true;
		CENTER_OFFSET = Math.sqrt(Math.pow(endX, 2) + Math.pow(endY, 2) * CENTER_OFFSET);
		// convert from coordinates to actual distance
		endX = endX * SQUARE_LENGTH;
		endY = endY * SQUARE_LENGTH;

		// x and y distances to travel
		double distanceX = endX - odometer.getX();
		double distanceY = endY - odometer.getY();

		// If reached destination, then stop
		if (Math.abs(distanceX) <= 0.3 && Math.abs(distanceY) <= 0.3) {
			FinalProject.leftMotor.stop(true);
			FinalProject.rightMotor.stop(false);
		}

		else { // has not reached destination

			turnTo(Math.toDegrees(Math.atan2(distanceX, distanceY))); // turn to
			// the
			// correct
			// angle
			distanceToTravel = Math.sqrt(Math.pow(distanceX, 2) + Math.pow(distanceY, 2)); // calculate
			// distance
			// to
			// travel

			// function to move the robot forward forward
			driveWithoutAvoid(distanceToTravel);

			isNavigating = false;
			while (isNavigating())
				continue;
			// make a sound when has reached destination
			Sound.beep();
		}

	}

	// this method makes the robot drive the indicated distance without looking for
	// obstacles

	/**
	 * Drive without avoid, called if neglecting avoidance WILL NOT USE THIS METHOD
	 * IN FINAL PROJECT SINCE NEED TO ACCOUNT FOR AVOIDING
	 * 
	 * @param distanceToTravel,
	 *            how far the robot needs to travel to reach destination
	 */
	public void driveWithoutAvoid(double distanceToTravel) {

		FinalProject.leftMotor.setSpeed(MOTOR_SPEED); // set speeds
		FinalProject.rightMotor.setSpeed(MOTOR_SPEED);

		FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, distanceToTravel + CENTER_OFFSET),
				true); // move
		// forward
		FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, distanceToTravel + CENTER_OFFSET),
				false);

	}

	/**
	 * Drive, accounts for avoiding as well
	 *
	 * @param distanceToTravel
	 *            the distance to travel
	 * @param endX
	 *            the end X
	 * @param endY
	 *            the end Y
	 */
	// this method makes the robot drive a certain distance
	public void drive(double distanceToTravel, double endX, double endY) {

		FinalProject.leftMotor.setSpeed(MOTOR_SPEED); // set speeds
		FinalProject.rightMotor.setSpeed(MOTOR_SPEED);

		FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, distanceToTravel), true); // move
		// forward
		FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, distanceToTravel), true);

		FinalProject.usSensor.fetchSample(FinalProject.sample, 0); // fetch
		// usSensor
		// data
		double wall_dist = FinalProject.sample[0] * 100;

		while (wall_dist > MAX_DISTANCE_WALL) {// far enough from block
			FinalProject.usSensor.fetchSample(FinalProject.sample, 0);
			wall_dist = FinalProject.sample[0] * 100; // update distance from
			// wall

			if (Math.abs(endX - odometer.getX()) <= 1 && Math.abs(endY - odometer.getY()) <= 1) {
				Sound.buzz();
				break; // break out of while loop if has reached destination
			}

		}
		// if too close to obstacle
		if (wall_dist < MAX_DISTANCE_WALL) {
			Avoid(endX, endY); // run avoid method
		}
	}

	/**
	 * Avoid, called by drive if the US sensor reads a value that is smaller than
	 * the bandcenter
	 * 
	 * @param endX
	 *            the end X
	 * @param endY
	 *            the end Y
	 */
	// this method makes the robot go around an obstacle
	public void Avoid(double endX, double endY) {

		FinalProject.leftMotor.setSpeed(ROTATE_SPEED); // set speeds
		FinalProject.rightMotor.setSpeed(ROTATE_SPEED);

		double currentX = odometer.getX(); // get current odometer values
		double currentY = odometer.getY();
		double currentTheta = odometer.getTheta();

		// robot's angle & x and y distances it will travel if robot turns right
		double rightAvoidTheta = 90 + currentTheta;
		double rightAvoidX = AVOID_DISTANCE * Math.sin(rightAvoidTheta);
		double rightAvoidY = AVOID_DISTANCE * Math.cos(rightAvoidTheta);

		// robot's angle & x and y distances it will travel if robot turns left
		double leftAvoidTheta = 90 - currentTheta;
		double leftAvoidX = AVOID_DISTANCE * Math.cos(leftAvoidTheta);
		double leftAvoidY = AVOID_DISTANCE * Math.sin(leftAvoidTheta);

		int angleDirection = 0;

		// check if robot should turn left or right to not fall off the board
		if ((currentX + leftAvoidX - SENSOR_OFFSET > XMin) && (currentX + leftAvoidX + SENSOR_OFFSET < XMax)
				&& (currentY + leftAvoidY + SENSOR_OFFSET < YMax) && (currentY + leftAvoidY - SENSOR_OFFSET > YMin)) {
			angleDirection = -1; // turn left
		} else if ((currentX + rightAvoidX - SENSOR_OFFSET > XMin) && (currentX + rightAvoidX + SENSOR_OFFSET < XMax)
				&& (currentY + rightAvoidY + SENSOR_OFFSET < YMax) && (currentY + rightAvoidY - SENSOR_OFFSET > YMin)) {
			angleDirection = 1; // turn right
		}

		FinalProject.usSensor.fetchSample(FinalProject.sample, 0); // get
		// sensor
		// reading
		double wall_dist = FinalProject.sample[0] * 100;

		while (wall_dist < MAX_DISTANCE_WALL) { // while the robot is blocked by
			// the obstacle

			// Rotate 90 degrees to the left or the right
			FinalProject.leftMotor.rotate(
					convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, AVOID_ANGLE * angleDirection), true);
			FinalProject.rightMotor.rotate(
					-convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, AVOID_ANGLE * angleDirection), false);

			// Travel distance around block
			FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, AVOID_DISTANCE), true);
			FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, AVOID_DISTANCE), false);

			// Rotate 90 degrees to left or the right
			FinalProject.leftMotor.rotate(
					-convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, AVOID_ANGLE * angleDirection), true);
			FinalProject.rightMotor.rotate(
					convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, AVOID_ANGLE * angleDirection), false);

			// update the distance from the wall
			FinalProject.usSensor.fetchSample(FinalProject.sample, 0);
			wall_dist = FinalProject.sample[0] * 100;
		}

		// Travel distance around block
		FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, AVOID_DISTANCE), true);
		FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, AVOID_DISTANCE), false);

		// Convert x and y from distance to coordinates
		endX = endX / SQUARE_LENGTH;
		endY = endY / SQUARE_LENGTH;
		travelTo(endX, endY); // travel to the the waypoint
	}

	/**
	 * Turn to, adjust heading of the robot to the desired destination
	 *
	 * @param theta
	 */
	// this method makes the robot turn to the indicated angle the shortest way
	void turnTo(double theta) {
		// get current angle and convert to degrees
		double currentTheta = Math.toDegrees(odometer.getTheta());
		// angle that robot needs to turn
		double turnTheta = theta - currentTheta;

		// Minimum angle is between -180 and 180
		if (turnTheta < -180) {
			turnTheta = turnTheta + 360;
		} else if (turnTheta > 180) {
			turnTheta = turnTheta - 360;
		}

		// set rotate speed for both motors
		FinalProject.leftMotor.setSpeed(ROTATE_SPEED);
		FinalProject.rightMotor.setSpeed(ROTATE_SPEED);

		// Turn
		FinalProject.leftMotor.rotate(convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, turnTheta), true);
		FinalProject.rightMotor.rotate(-convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, turnTheta), false);
	}

	/**
	 * zip traversal algorithm slows down near the end for a safer landing.
	 */
	public void zipTraversal() {
		FinalProject.leftMotor.setSpeed(300); // set speeds
		FinalProject.rightMotor.setSpeed(300);
		FinalProject.zipMotor.setSpeed(MOTOR_SPEED);
		FinalProject.zipMotor.forward();// travels about 3/4 the zipline
		FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, 230), true);
		FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, 230), false);
		while (isNavigating())
			continue;
		FinalProject.leftMotor.setSpeed(150); // slow down once getting to the downward slope
		FinalProject.rightMotor.setSpeed(150);
		FinalProject.zipMotor.setSpeed(MOTOR_SPEED / 2);
		FinalProject.zipMotor.rotate(convertDistance(1.1, 100), true); // rotate the rest of the way at slower speed
		FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, 50), true);
		FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, 50), false);
	}

	/**
	 * Same method as turn, without speed change.
	 *
	 * @author Sam Cleland
	 * @param theta
	 *            the theta
	 */
	public void turnWithSameSpeed(double theta) {
		FinalProject.leftMotor.rotate(convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, theta), true);
		FinalProject.rightMotor.rotate(-convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, theta), true);
	}

	/**
	 * Turn.
	 *
	 * @param theta
	 *            the theta
	 */
	// this method makes the robot turn the indicated angle
	void turn(double theta) {
		// set rotate speed for both motors
		FinalProject.leftMotor.setSpeed(ROTATE_SPEED);
		FinalProject.rightMotor.setSpeed(ROTATE_SPEED);

		// Turn
		FinalProject.leftMotor.rotate(convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, theta), true);
		FinalProject.rightMotor.rotate(-convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, theta), true);
	}

	/**
	 * Turn without interruption.
	 *
	 * @param theta
	 *            the theta
	 */
	void turnWithoutInterruption(double theta) {
		FinalProject.leftMotor.setSpeed(ROTATE_SPEED);
		FinalProject.rightMotor.setSpeed(ROTATE_SPEED);

		// Turn
		FinalProject.leftMotor.rotate(convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, theta), true);
		FinalProject.rightMotor.rotate(-convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, theta), false);
	}

	/**
	 * Checks if is navigating.
	 *
	 * @return true, if is navigating
	 */
	static // set a boolean to know when it is navigating
	boolean isNavigating() {
		return FinalProject.leftMotor.isMoving() && FinalProject.rightMotor.isMoving();
	}

	/**
	 * Convert distance.
	 *
	 * @param radius
	 *            the radius
	 * @param distance
	 *            the distance
	 * @return the int
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * Convert angle.
	 *
	 * @param radius
	 *            the radius
	 * @param width
	 *            the width
	 * @param angle
	 *            the angle
	 * @return the int
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}

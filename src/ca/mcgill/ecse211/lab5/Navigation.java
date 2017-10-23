package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.*;
import lejos.hardware.port.Port;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/***
This class is an updated version from lab 3 where we changed most methods
in order to match lab 4.
 ***/

public class Navigation  {

	// Create constants
	private static final int MOTOR_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private static final int MAX_DISTANCE_WALL = 15;
	private static final int AVOID_ANGLE = 90;
	private static final int AVOID_DISTANCE = 30;
	private static final double SENSOR_OFFSET = 10;
	private static final double SQUARE_LENGTH = 30.48;
	private static double CENTER_OFFSET = 2;

	// the maximum and minimum x and y values possible
	private static final double XMax = 3 * SQUARE_LENGTH;
	private static final double XMin = -1 * SQUARE_LENGTH;
	private static final double YMax = 3 * SQUARE_LENGTH;
	private static final double YMin = -1 * SQUARE_LENGTH;

	private static double path[][]; // For demo coordinates
	private static Odometer odometer;
	private static boolean isNavigating = false;
	private static double distanceToTravel; // Distance to travel between points

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

	public void startNav() {

		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}

		// travel to each waypoints
		for (int i = 0; i < path.length; i++) {
			travelTo(path[i][0], path[i][1]);
		}
	}

	//this method makes the robot travel to coordinates passed as parameters
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
			Lab5.leftMotor.stop(true);
			Lab5.rightMotor.stop(false);
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
			while(isNavigating()) continue; 
			// make a sound when has reached destination
			Sound.beep();
		}

	}

	//this method makes the robot drive the indicated distance without looking for obstacles

	public void driveWithoutAvoid(double distanceToTravel) {
		
		Lab5.leftMotor.setSpeed(MOTOR_SPEED); // set speeds
		Lab5.rightMotor.setSpeed(MOTOR_SPEED);

		Lab5.leftMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS, distanceToTravel + CENTER_OFFSET), true); // move
		// forward
		Lab5.rightMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS, distanceToTravel + CENTER_OFFSET), false);


	}

	//this method makes the robot drive a certain distance
	public void drive(double distanceToTravel, double endX, double endY) {

		Lab5.leftMotor.setSpeed(MOTOR_SPEED); // set speeds
		Lab5.rightMotor.setSpeed(MOTOR_SPEED);

		Lab5.leftMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS, distanceToTravel), true); // move
		// forward
		Lab5.rightMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS, distanceToTravel), true);

		Lab5.usSensor.fetchSample(Lab5.sample, 0); // fetch
		// usSensor
		// data
		double wall_dist = Lab5.sample[0] * 100;

		while (wall_dist > MAX_DISTANCE_WALL) {// far enough from block
			Lab5.usSensor.fetchSample(Lab5.sample, 0);
			wall_dist = Lab5.sample[0] * 100; // update distance from
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

	// this method makes the robot go around an obstacle
	public void Avoid(double endX, double endY) {

		Lab5.leftMotor.setSpeed(ROTATE_SPEED); // set speeds
		Lab5.rightMotor.setSpeed(ROTATE_SPEED);

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

		Lab5.usSensor.fetchSample(Lab5.sample, 0); // get
		// sensor
		// reading
		double wall_dist = Lab5.sample[0] * 100;

		while (wall_dist < MAX_DISTANCE_WALL) { // while the robot is blocked by
			// the obstacle

			// Rotate 90 degrees to the left or the right
			Lab5.leftMotor.rotate(convertAngle(Lab5.WHEEL_RADIUS, Lab5.TRACK, AVOID_ANGLE * angleDirection), true);
			Lab5.rightMotor.rotate(-convertAngle(Lab5.WHEEL_RADIUS, Lab5.TRACK, AVOID_ANGLE * angleDirection), false);

			// Travel distance around block
			Lab5.leftMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS, AVOID_DISTANCE), true);
			Lab5.rightMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS, AVOID_DISTANCE), false);

			// Rotate 90 degrees to left or the right
			Lab5.leftMotor.rotate(-convertAngle(Lab5.WHEEL_RADIUS, Lab5.TRACK, AVOID_ANGLE * angleDirection), true);
			Lab5.rightMotor.rotate(convertAngle(Lab5.WHEEL_RADIUS, Lab5.TRACK, AVOID_ANGLE * angleDirection), false);

			// update the distance from the wall
			Lab5.usSensor.fetchSample(Lab5.sample, 0);
			wall_dist = Lab5.sample[0] * 100;
		}

		// Travel distance around block
		Lab5.leftMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS, AVOID_DISTANCE), true);
		Lab5.rightMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS, AVOID_DISTANCE), false);

		// Convert x and y from distance to coordinates
		endX = endX / SQUARE_LENGTH;
		endY = endY / SQUARE_LENGTH;
		travelTo(endX, endY); // travel to the the waypoint
	}

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
		Lab5.leftMotor.setSpeed(ROTATE_SPEED);
		Lab5.rightMotor.setSpeed(ROTATE_SPEED);

		// Turn
		Lab5.leftMotor.rotate(convertAngle(Lab5.WHEEL_RADIUS, Lab5.TRACK, turnTheta), true);
		Lab5.rightMotor.rotate(-convertAngle(Lab5.WHEEL_RADIUS, Lab5.TRACK, turnTheta), false);
	}
	
	/**
	 * zip traversal algorithm
	 * slows down near the end for a safer landing
	 *
	 */
	public void zipTraversal() {
		Lab5.leftMotor.setSpeed(300); // set speeds
		Lab5.rightMotor.setSpeed(300); 
		Lab5.zipMotor.setSpeed(MOTOR_SPEED);
		Lab5.leftMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS,30), true);
		Lab5.rightMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS,30), true);
		Lab5.zipMotor.rotate(convertDistance(1.1, 100), true);
		Lab5.leftMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS,250), true);
		Lab5.rightMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS,250), false);
		Lab5.leftMotor.setSpeed(150);
		Lab5.rightMotor.setSpeed(150);
		Lab5.zipMotor.setSpeed(MOTOR_SPEED/2);
		Lab5.zipMotor.rotate(convertDistance(1.1, 20), true);
		Lab5.leftMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS,50), true);
		Lab5.rightMotor.rotate(convertDistance(Lab5.WHEEL_RADIUS,50), false);
	}
	//this method makes the robot turn the indicated angle
	void turn(double theta) {
		// set rotate speed for both motors
		Lab5.leftMotor.setSpeed(ROTATE_SPEED);
		Lab5.rightMotor.setSpeed(ROTATE_SPEED);

		// Turn
		Lab5.leftMotor.rotate(convertAngle(Lab5.WHEEL_RADIUS, Lab5.TRACK, theta), true);
		Lab5.rightMotor.rotate(-convertAngle(Lab5.WHEEL_RADIUS, Lab5.TRACK, theta), true);
	}
	void turnWithoutInterruption(double theta) {
		Lab5.leftMotor.setSpeed(ROTATE_SPEED);
		Lab5.rightMotor.setSpeed(ROTATE_SPEED);

		// Turn
		Lab5.leftMotor.rotate(convertAngle(Lab5.WHEEL_RADIUS, Lab5.TRACK, theta), true);
		Lab5.rightMotor.rotate(-convertAngle(Lab5.WHEEL_RADIUS, Lab5.TRACK, theta), false);
	}
	// set a boolean to know when it is navigating
	boolean isNavigating() {
		return Lab5.leftMotor.isMoving() && Lab5.rightMotor.isMoving();
	}

	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}

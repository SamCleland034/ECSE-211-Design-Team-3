

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.*;
import lejos.hardware.port.Port;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/***
This class is an updated version from lab 3 where we changed most methods
in order to match lab 4.
 ***/

public class Navigation extends Thread {

	// Create constants
	private static final int MOTOR_SPEED = 150;
	private static final int ROTATE_SPEED = 100;
	private static final int MAX_DISTANCE_WALL = 15;
	private static final int AVOID_ANGLE = 90;
	private static final int AVOID_DISTANCE = 30;
	private static final double SENSOR_OFFSET = 10;
	private static final double SQUARE_LENGTH = 30.48;

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
			motors.setAcceleration(500);
		}
	}

	public void run() {

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

		// convert from coordinates to actual distance
		endX = endX * SQUARE_LENGTH;
		endY = endY * SQUARE_LENGTH;

		// x and y distances to travel
		double distanceX = endX - odometer.getX();
		double distanceY = endY - odometer.getY();

		// If reached destination, then stop
		if (Math.abs(distanceX) <= 0.5 && Math.abs(distanceY) <= 0.5) {
			Test.leftMotor.stop();
			Test.rightMotor.stop();
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

			// make a sound when has reached destination
			Sound.beep();
		}

	}
	
	//this method makes the robot drive the indicated distance without looking for obstacles

	public void driveWithoutAvoid(double distanceToTravel) {

		Test.leftMotor.setSpeed(ROTATE_SPEED); // set speeds
		Test.rightMotor.setSpeed(ROTATE_SPEED);

		Test.leftMotor.rotate(convertDistance(Test.WHEEL_RADIUS, distanceToTravel), true); // move
		// forward
		Test.rightMotor.rotate(convertDistance(Test.WHEEL_RADIUS, distanceToTravel), false);

	}

	//this method makes the robot drive a certain distance
	public void drive(double distanceToTravel, double endX, double endY) {

		Test.leftMotor.setSpeed(MOTOR_SPEED); // set speeds
		Test.rightMotor.setSpeed(MOTOR_SPEED);

		Test.leftMotor.rotate(convertDistance(Test.WHEEL_RADIUS, distanceToTravel), true); // move
																									// forward
		Test.rightMotor.rotate(convertDistance(Test.WHEEL_RADIUS, distanceToTravel), true);

		Test.usSensor.fetchSample(Test.sample, 0); // fetch
																			// usSensor
																			// data
		double wall_dist = Test.sample[0] * 100;

		while (wall_dist > MAX_DISTANCE_WALL) {// far enough from block
			Test.usSensor.fetchSample(Test.sample, 0);
			wall_dist = Test.sample[0] * 100; // update distance from
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

		Test.leftMotor.setSpeed(ROTATE_SPEED); // set speeds
		Test.rightMotor.setSpeed(ROTATE_SPEED);

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

		Test.usSensor.fetchSample(Test.sample, 0); // get
																			// sensor
																			// reading
		double wall_dist = Test.sample[0] * 100;

		while (wall_dist < MAX_DISTANCE_WALL) { // while the robot is blocked by
												// the obstacle

			// Rotate 90 degrees to the left or the right
			Test.leftMotor.rotate(convertAngle(Test.WHEEL_RADIUS, Test.TRACK, AVOID_ANGLE * angleDirection), true);
			Test.rightMotor.rotate(-convertAngle(Test.WHEEL_RADIUS, Test.TRACK, AVOID_ANGLE * angleDirection), false);

			// Travel distance around block
			Test.leftMotor.rotate(convertDistance(Test.WHEEL_RADIUS, AVOID_DISTANCE), true);
			Test.rightMotor.rotate(convertDistance(Test.WHEEL_RADIUS, AVOID_DISTANCE), false);

			// Rotate 90 degrees to left or the right
			Test.leftMotor.rotate(-convertAngle(Test.WHEEL_RADIUS, Test.TRACK, AVOID_ANGLE * angleDirection), true);
			Test.rightMotor.rotate(convertAngle(Test.WHEEL_RADIUS, Test.TRACK, AVOID_ANGLE * angleDirection), false);

			// update the distance from the wall
			Test.usSensor.fetchSample(Test.sample, 0);
			wall_dist = Test.sample[0] * 100;
		}

		// Travel distance around block
		Test.leftMotor.rotate(convertDistance(Test.WHEEL_RADIUS, AVOID_DISTANCE), true);
		Test.rightMotor.rotate(convertDistance(Test.WHEEL_RADIUS, AVOID_DISTANCE), false);

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
	  Test.leftMotor.setSpeed(ROTATE_SPEED);
	  Test.rightMotor.setSpeed(ROTATE_SPEED);

	  // Turn
	  Test.leftMotor.rotate(convertAngle(Test.WHEEL_RADIUS, Test.TRACK, turnTheta), true);
	  Test.rightMotor.rotate(-convertAngle(Test.WHEEL_RADIUS, Test.TRACK, turnTheta), false);
	 }

	 //this method makes the robot turn the indicated angle
	 void turn(double theta) {
	  // set rotate speed for both motors
	  Test.leftMotor.setSpeed(ROTATE_SPEED);
	  Test.rightMotor.setSpeed(ROTATE_SPEED);

	  // Turn
	  Test.leftMotor.rotate(convertAngle(Test.WHEEL_RADIUS, Test.TRACK, theta), true);
	  Test.rightMotor.rotate(-convertAngle(Test.WHEEL_RADIUS, Test.TRACK, theta), true);
	 }

	 // set a boolean to know when it is navigating
	 boolean isNavigating() {
	  return Test.leftMotor.isMoving() && Test.rightMotor.isMoving();
	 }

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}

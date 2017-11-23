package ca.mcgill.ecse211.dpmfinalprojectteam3;

import java.util.LinkedList;

import lejos.hardware.Sound;

/***
 * Navigates the robot around using coordinates that get converted into
 * distances based on the length of the tiles. Other features include methods
 * for specific tasks such as avoiding an obstacle and searching for the flag.
 * The startNav() method in this class will be the method that determines if we
 * need to transition states based on the the coords our robot just travelled
 * to. If there is a stage transition, then the stage will be set to whatever
 * that stage is and the controller will know on the next iteration of the loop
 * it has to execute the method call for whatever stage we are going to
 * transition to.
 * 
 * 
 ***/

public class Navigation {

	/** The Constant MOTOR_SPEED. Default motor speed */
	// Create constants
	private static Avoidance master;

	/** The Constant MOTOR_SPEED. */
	public static final int MOTOR_SPEED = 250;

	/** The Constant ROTATE_SPEED. Speed used when rotating in place */
	private static final int ROTATE_SPEED = 171;

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
	private static double CENTER_OFFSET = 1.43;
	public static double RIGHT_OFFSET = 1.0086;
	public static final float MOTOR_SPEED_RIGHT = (float) (MOTOR_SPEED * RIGHT_OFFSET);
	private static final float ROTATE_SPEED_RIGHT = (float) (ROTATE_SPEED * RIGHT_OFFSET);

	/** The search region path. */
	private LinkedList<Integer> searchRegionPath;

	/** The Constant THRESHOLD. */
	private static final int THRESHOLD = 50;

	private static final long SAMPLING_PERIOD = 10;

	/** The path. */
	private LinkedList<Double> path; // For demo coordinates

	/** The odometer. */
	private Odometer odometer;

	/** The has flag. */
	boolean hasFlag = false;

	/** The poller. */
	private UltrasonicPoller poller;

	/** The sensor motor. */
	private SensorRotation sensorMotor;

	/** The colorpoller. */
	private LightPoller colorpoller;

	/** The oc. */
	private OdometryCorrection oc;

	/** The corrected. */
	public boolean corrected = false;

	private boolean avoided;

	public boolean ziptraversing;

	public boolean donecorrecting = false;

	/** The is navigating. */
	private static boolean isNavigating = false;

	/** The distance to travel. */
	private static double distanceToTravel; // Distance to travel between points

	/**
	 * Instantiates a new navigation.
	 *
	 * @param odometer
	 *            the odometer
	 */
	public Navigation(Odometer odometer) {
		this.odometer = odometer;

		// Reset the motors

	}

	/**
	 * Sets the path.
	 *
	 * @param coordsList
	 *            the new path
	 */
	public void setPath(LinkedList<Double> coordsList) {
		this.path = coordsList;
	}

	/**
	 * Cycles through the coordinates passed in by the wifi class and depending on
	 * what coords the robot travels to we will change states. For example if we
	 * reach the zip line coordinates, we will then transition to the zipline stage
	 * of the project which will execute the code for the ziptraversal algorithm.
	 */
	public void startNav() {
		double coordX;
		double coordY;
		while (!path.isEmpty()) {
			coordX = path.removeFirst();
			coordY = path.removeFirst();

			avoided = false;
			System.out.println("TRAVELLING TO" + coordX + " " + coordY);
			travelTo(coordX, coordY);

			if (!hasFlag && coordX == FinalProject.URSRRX && coordY == FinalProject.URSRRY) {
				// if the coords we just travelled to are the flagsearch coords for red zone
				FinalProject.stage = Stage.FLAGSEARCH;
				break;
			} else if (coordX == FinalProject.zipgreenXc && coordY == FinalProject.zipgreenYc) {
				// coords travelled to were zipline coords
				FinalProject.stage = Stage.ZIPTRAVERSAL;
				break;
			} else if (!hasFlag && (coordX == FinalProject.URSRGX && coordY == FinalProject.URSRGY)) {
				// if the coords that were travelled to are flagsearch coords for green zone
				FinalProject.stage = Stage.FLAGSEARCH;
				break;
			} else if ((coordX == FinalProject.startingX && coordY == FinalProject.startingY) && hasFlag) {
				// if the coords travelled to were the starting positions
				FinalProject.stage = Stage.FINISHED;
				break;
			}
		}
	}

	/**
	 * Travel to, orients the robot to x and y on the grid calls turn to if the
	 * robot is not aligned in the proper position to reach x and y. If using start
	 * nav method, will continuously call this method until no more coordinates in
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
		// CENTER_OFFSET = Math.sqrt(Math.pow(endX, 2) + Math.pow(endY, 2) *
		// CENTER_OFFSET);
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
			drive(distanceToTravel, endX, endY);

			isNavigating = false;
			while (isNavigating())
				continue;
			// make a sound when has reached destination
			Sound.beep();
		}

	}

	/**
	 * Travel to without avoid.
	 *
	 * @param endX
	 *            the end X
	 * @param endY
	 *            the end Y
	 */
	public void travelToWithoutAvoid(double endX, double endY) {

		isNavigating = true;
		// CENTER_OFFSET = Math.sqrt(Math.pow(endX, 2) + Math.pow(endY, 2) *
		// CENTER_OFFSET);
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
	 * Drive without avoid, called if neglecting avoidance in special cases.
	 * 
	 *
	 * @param distanceToTravel
	 *            the distance to travel
	 */
	public void driveWithoutAvoid(double distanceToTravel) {

		FinalProject.leftMotor.setSpeed(MOTOR_SPEED); // set speeds

		FinalProject.rightMotor.setSpeed(MOTOR_SPEED_RIGHT);

		FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, distanceToTravel + CENTER_OFFSET),
				true); // move
		// forward
		FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS,
				(distanceToTravel + CENTER_OFFSET) * (Navigation.RIGHT_OFFSET)), false);

	}

	/**
	 * Drive, accounts for avoiding as well.
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
		FinalProject.rightMotor.setSpeed(MOTOR_SPEED_RIGHT);
		FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, distanceToTravel), true); // move
		// forward
		FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS,
				(distanceToTravel * RIGHT_OFFSET) * (Navigation.RIGHT_OFFSET)), true);

		// fetch
		// usSensor
		// data
		if (!avoided) {
			oc.on();
			oc.counter = 0;
		}
		master.on();

		while (!master.inDanger) {// far enough from block

			// update distance from
			// wall
			if (Math.abs(endX - odometer.getX()) < 3 && Math.abs(endY - odometer.getY()) < 3) {
				// if (Math.sqrt(Math.pow(endX - odometer.getX(), 2) + Math.pow(endY -
				// odometer.getY(), 2)) < 2) {
				oc.off();
				while (oc.isOn)
					continue;
				FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, 2.9), true);
				FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, 2.9), false);
				Sound.buzz();

				return; // break out of while loop if has reached destination
			}
			if (oc.corrected) {
				oc.corrected = false;
				// distanceToTravel = Math.sqrt(Math.pow(endX - odometer.getX(), 2) +
				// Math.pow(endY - odometer.getY(), 2));

				FinalProject.leftMotor.forward();
				FinalProject.rightMotor.forward();

			}

		}
		// if too close to obstacle
		while (oc.isOn)
			continue;
		FinalProject.leftMotor.stop(true);
		FinalProject.rightMotor.stop(false);
		oc.off();
		donecorrecting = true;
		while (master.inDanger) {
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
			}
			continue;
		}
		master.off();
		avoided = true;
		donecorrecting = false;
		travelTo(endX / SQUARE_LENGTH, endY / SQUARE_LENGTH);
	}

	/**
	 * Turn to, adjust heading of the robot to the desired destination.
	 *
	 * @param theta
	 *            the theta
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
		FinalProject.rightMotor.setSpeed(ROTATE_SPEED_RIGHT);

		// Turn
		FinalProject.leftMotor.rotate(convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, turnTheta), true);
		FinalProject.rightMotor
				.rotate(-convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, turnTheta * RIGHT_OFFSET), false);
	}

	/**
	 * Turn to with interrupt.
	 *
	 * @param theta
	 *            the theta
	 */
	void turnToWithInterrupt(double theta) {
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
		FinalProject.rightMotor.setSpeed(ROTATE_SPEED_RIGHT);

		// Turn
		FinalProject.leftMotor.rotate(convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, turnTheta), true);
		FinalProject.rightMotor.rotate(-convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, turnTheta), true);
	}

	/**
	 * zip traversal algorithm slows down near the end for a safer landing.
	 * 
	 * @param initaltheta
	 *            value of the robot's heading before traversing, set the odometer
	 *            equal to this value once the zip traversal finishes (assumption
	 *            that it doesnt change much).
	 */
	public double zipTraversal() {
		ziptraversing = true;
		double initialTheta = odometer.getTheta();
		FinalProject.leftMotor.setSpeed(300); // set speeds
		FinalProject.rightMotor.setSpeed(300);
		FinalProject.zipMotor.setSpeed(225);
		FinalProject.zipMotor.forward();// travels about 3/4 the zipline
		FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, 230), true);
		FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, 230), false);
		while (isNavigating())
			continue;
		FinalProject.leftMotor.setSpeed(150); // slow down once getting to the downward slope
		FinalProject.rightMotor.setSpeed(150);
		FinalProject.zipMotor.setSpeed(225 / 2);
		FinalProject.zipMotor.rotate(convertDistance(1.1, 100), true); // rotate the rest of the way at slower speed
		FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, 61), true);
		FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, 61), false);
		while (isNavigating)
			continue;
		ziptraversing = false;
		return initialTheta;
	}

	/**
	 * Turn a fixed number of degrees.
	 *
	 * @param theta
	 *            the amount of degrees we want to turn
	 */
	// this method makes the robot turn the indicated angle
	void turn(double theta) {
		// set rotate speed for both motors
		FinalProject.leftMotor.setSpeed(ROTATE_SPEED);
		FinalProject.rightMotor.setSpeed(ROTATE_SPEED_RIGHT);

		// Turn
		FinalProject.leftMotor.rotate(convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, theta), true);
		FinalProject.rightMotor.rotate(-convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, theta), true);
	}

	/**
	 * Turn without interruption, same as turn except can't be stopped
	 *
	 * @param theta
	 *            the amount of degrees we want the robot to turn
	 */
	void turnWithoutInterruption(double theta) {
		FinalProject.leftMotor.setSpeed(ROTATE_SPEED);
		FinalProject.rightMotor.setSpeed(ROTATE_SPEED_RIGHT);

		// Turn
		FinalProject.leftMotor.rotate(convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, theta), true);
		FinalProject.rightMotor.rotate(-convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, theta), false);
	}

	/**
	 * Method that will be called when we enter the flag search state. The robot
	 * will perform a sweep of each tile starting at one of the corners, it will
	 * then proceed to the next row if it doesn't see anything, but if it does see
	 * something within a tile it will go to that tile and then check if the color
	 * of the tile is the one that we are searching for. If it is, beep 3 times and
	 * go to the upper right corner of the search region
	 *
	 * @param correctColor
	 *            the correct RGB colors our robot will be searching for
	 * 
	 * @since 10/29/17
	 * 
	 */
	public void flagSearch(float[] correctColors) {
		hasFlag = true;
		for (int i = 0; i < 3; i++) {
			Sound.beep();
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
			}
		}
		// travelToAfterFlag();
		return;
		/*
		 * int distance = 0;
		 * 
		 * int counter = 0; while (!hasFlag) {
		 * turnTo(Math.toDegrees(Math.atan2(searchRegionPath.get(2) *
		 * FinalProject.TILE_SPACING - odometer.getX(), searchRegionPath.get(3) *
		 * FinalProject.TILE_SPACING - odometer.getY()))); while (isNavigating())
		 * continue; turnToWithInterrupt(
		 * Math.toDegrees(Math.atan2(searchRegionPath.get(6) * FinalProject.TILE_SPACING
		 * - odometer.getX(), searchRegionPath.getLast() * FinalProject.TILE_SPACING -
		 * odometer.getY()))); while (poller.getReading() > THRESHOLD && isNavigating())
		 * { continue; } FinalProject.leftMotor.stop(true);
		 * FinalProject.rightMotor.stop(false); distance = poller.getReading(); if
		 * (distance > THRESHOLD) ;
		 * 
		 * else {
		 * 
		 * turnWithoutInterruption(2);
		 * FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS,
		 * distance - 7), true);
		 * FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS,
		 * distance - 7), false); while (isNavigating()) continue;
		 * FinalProject.usMotor.setSpeed(40); if
		 * (colorpoller.checkColors(FinalProject.correctColor)) { for (int j = 0; j < 3;
		 * j++) { Sound.beep(); try { Thread.sleep(500); } catch (InterruptedException
		 * e) { } } hasFlag = true;
		 * FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS,
		 * -10), true);
		 * FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS,
		 * -10), false); while (isNavigating()) continue; travelToAfterFlag(); return; }
		 * FinalProject.usMotor.rotateTo(sensorMotor.reference - 45); while
		 * (FinalProject.usMotor.isMoving()) {
		 * 
		 * if (colorpoller.checkColors(FinalProject.correctColor)) { for (int j = 0; j <
		 * 3; j++) { Sound.beep(); try { Thread.sleep(500); } catch
		 * (InterruptedException e) { } } hasFlag = true;
		 * FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS,
		 * -10), true);
		 * FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS,
		 * -10), false); while (isNavigating()) continue; travelToAfterFlag(); return;
		 * 
		 * } } FinalProject.usMotor.rotateTo(sensorMotor.reference + 45); while
		 * (FinalProject.usMotor.isMoving()) {
		 * 
		 * if (colorpoller.checkColors(FinalProject.correctColor)) { for (int j = 0; j <
		 * 3; j++) { Sound.beep(); try { Thread.sleep(500); } catch
		 * (InterruptedException e) { } } hasFlag = true;
		 * FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS,
		 * -10), true);
		 * FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS,
		 * -10), false); while (isNavigating()) continue; travelToAfterFlag(); return; }
		 * } } FinalProject.usMotor.rotateTo(sensorMotor.reference); while
		 * (FinalProject.usMotor.isMoving()) continue;
		 * searchRegionPath.addLast(searchRegionPath.removeFirst());
		 * searchRegionPath.addLast(searchRegionPath.removeFirst()); if (counter == 3)
		 * break; travelToWithoutAvoid(searchRegionPath.getFirst(),
		 * searchRegionPath.get(1)); while (isNavigating()) continue; counter++; }
		 */}

	/**
	 * After the robot finds the flag, this method will determine where the robot
	 * should go next to allow it to be in an ideal position to continue traversing
	 * the grid.
	 */
	public void travelToAfterFlag() {
		if (FinalProject.greenTeam == 3) {
			if ((searchRegionPath.getFirst() == FinalProject.URSRRX && searchRegionPath.get(1) == FinalProject.URSRRY)
					|| (searchRegionPath.getFirst() == FinalProject.URSRRX
							&& searchRegionPath.get(1) == FinalProject.LLSRRY)) {
				travelToWithoutAvoid(FinalProject.URSRRX, FinalProject.URSRRY);
				while (isNavigating())
					continue;

			} else {
				travelToWithoutAvoid(FinalProject.LLSRRX, FinalProject.URSRRY);
				while (isNavigating())
					continue;

			}

		} else {
			if ((searchRegionPath.getFirst() == FinalProject.URSRGX && searchRegionPath.get(1) == FinalProject.URSRGY)
					|| (searchRegionPath.getFirst() == FinalProject.URSRGX
							&& searchRegionPath.get(1) == FinalProject.LLSRGY)) {
				travelToWithoutAvoid(FinalProject.URSRGX, FinalProject.URSRGY);
				while (isNavigating())
					continue;

			} else {
				travelToWithoutAvoid(FinalProject.URSRGX, FinalProject.URSRGY);
				while (isNavigating())
					continue;

			}
		}
	}

	/**
	 * Checks if robot is moving.
	 *
	 * @return true, if both of the motors are moving
	 */
	public static // set a boolean to know when it is navigating
	boolean isNavigating() {
		return FinalProject.leftMotor.isMoving() && FinalProject.rightMotor.isMoving();
	}

	public void setSearchRegionPath(LinkedList<Integer> coords) {
		this.searchRegionPath = coords;
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
	 *            the radius, wheel radius of robot
	 * @param width
	 *            the width, distance between the wheels of the robot
	 * @param angle
	 *            the angle, the angle we want the robot to turn to
	 * @return rotations, the number of rotations each wheel has to turn to adjust
	 *         heading to theta
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * Sets the avoidance.
	 *
	 * @param master
	 *            the new avoidance
	 */
	public void setAvoidance(Avoidance master) {
		this.master = master;
	}

	/**
	 * Sets the poller.
	 *
	 * @param poller
	 *            the new poller
	 */
	public void setPoller(UltrasonicPoller poller) {
		this.poller = poller;
	}

	/**
	 * Sets the sensor rotation.
	 *
	 * @param sensorRotation
	 *            the new sensor rotation
	 */
	public void setSensorRotation(SensorRotation sensorRotation) {
		this.sensorMotor = sensorRotation;

	}

	/**
	 * Sets the color provider.
	 *
	 * @param colorpoller
	 *            the new color provider
	 */
	public void setColorProvider(LightPoller colorpoller) {
		this.colorpoller = colorpoller;

	}

	/**
	 * Sets the odometry correction.
	 *
	 * @param oc
	 *            the new odometry correction
	 */
	public void setOdometryCorrection(OdometryCorrection oc) {
		this.oc = oc;
	}
}

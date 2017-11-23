package ca.mcgill.ecse211.dpmfinalprojectteam3;

import java.util.LinkedList;

import lejos.hardware.Button;
import lejos.hardware.Sound;

// TODO: Auto-generated Javadoc
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
	public static final int MOTOR_SPEED = 200;

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
	private static double CENTER_OFFSET = 1.43;
	public static double RIGHT_OFFSET = 1.0086;
	public static final float MOTOR_SPEED_RIGHT = (float) (MOTOR_SPEED * RIGHT_OFFSET);
	private static final float ROTATE_SPEED_RIGHT = (float) (ROTATE_SPEED * RIGHT_OFFSET);

	/** The search region path. */
	private LinkedList<Integer> searchRegionPath;
	/** The Constant XMax. */
	// the maximum and minimum x and y values possible
	private static final double XMax = 3 * SQUARE_LENGTH;

	/** The Constant XMin. */
	private static final double XMin = -1 * SQUARE_LENGTH;

	/** The Constant YMax. */
	private static final double YMax = 3 * SQUARE_LENGTH;

	/** The Constant YMin. */
	private static final double YMin = -1 * SQUARE_LENGTH;

	/** The Constant THRESHOLD. */
	private static final int THRESHOLD = 40;

	private static final long SAMPLING_PERIOD = 10;

	private static final int FILTER_OUT = 3;

	/** The path. */
	private LinkedList<Double> path; // For demo coordinates

	/** The odometer. */
	private Odometer odometer;

	/** The has flag. */
	private boolean hasFlag = false;

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

	private double distance;

	/** The is navigating. */
	private static boolean isNavigating = false;

	/** The distance to travel. */
	private static double distanceToTravel; // Distance to travel between points
	
	public static double coordX;
	public static double coordY;

	
	public static int flagSearchX = 0; //counter used for flag search
	
	public static int flagSearchY = 0; // counter used for flag search
	
	public static boolean detect; //set to true if robot detects something during flag search
	public static int filterControl = 0;

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

			travelTo(coordX, coordY);

			if (coordX == FinalProject.LLSRRX && coordY == FinalProject.LLSRRY) {
				// if the coords we just travelled to are the flagsearch coords for red zone
				FinalProject.stage = Stage.FLAGSEARCH;
				break;
			} else if (coordX == FinalProject.zipgreenXc && coordY == FinalProject.zipgreenYc) {
				// coords travelled to were zipline coords
				FinalProject.stage = Stage.ZIPTRAVERSAL;
				break;
			} else if (coordX == FinalProject.URSRGX && coordY == FinalProject.URSRGY) {
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
			 // turn to
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
		long startTime;
		long endTime;
		while (!master.inDanger) {// far enough from block

			// update distance from
			// wall
			if (oc.corrected) {
				oc.corrected = false;
				distanceToTravel = Math.sqrt(Math.pow(endX - odometer.getX(), 2) + Math.pow(endY - odometer.getY(), 2));
				// FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS,
				// distanceToTravel), true); // move
				// forward
				// FinalProject.rightMotor.rotate(
				// convertDistance(FinalProject.WHEEL_RADIUS, (distanceToTravel) *
				// (Navigation.RIGHT_OFFSET)),
				// true);
				FinalProject.leftMotor.forward();
				FinalProject.rightMotor.forward();

			}
			if (Math.abs(endX - odometer.getX()) < 2.1 && Math.abs(endY - odometer.getY()) < 2.1) {
				// if (Math.sqrt(Math.pow(endX - odometer.getX(), 2) + Math.pow(endY -
				// odometer.getY(), 2)) < 2) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.buzz();
				oc.off();
				return; // break out of while loop if has reached destination
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
		travelTo(endX / 30.48, endY / 30.48);
	}

	/**
	 * Avoid, called by drive if the US sensor reads a value that is smaller than
	 * the bandcenter.
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
		FinalProject.rightMotor.setSpeed(ROTATE_SPEED_RIGHT);

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
		FinalProject.rightMotor.setSpeed(ROTATE_SPEED_RIGHT);

		// Turn
		FinalProject.leftMotor.rotate(convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, theta), true);
		FinalProject.rightMotor.rotate(-convertAngle(FinalProject.WHEEL_RADIUS, FinalProject.TRACK, theta), false);
	}

	/**
	 * Method that will be called when we enter the flag search state. The robot
	 * will perform a sweep of the search region starting at each corner. If the
	 * ultrasonic sensor detects an object while sweeping, it will stop sweeping and
	 * travel to that object and sweep again with the small motor to get readings
	 * for the light sensor. If the light color readings (plural since using RGB
	 * mode) we will signal that we have found that flag and it will then call
	 * the @seetravelToAfterFlag() method to bring us to one of the corners of the
	 * search region depending on where we currently are to then continue our path.
	 *
	 * @param correctColor
	 *            the correct color the robot will be detecting
	 * @return true once it finds the flag, will keep running if it doesn't find the
	 *         flag
	 * @since 10/29/17
	 */
//	public void flagSearch(double[] correctColors) {
//		int distance = 0;
//		while (!hasFlag) {
//			turnTo(Math.toDegrees(Math.atan2(searchRegionPath.get(2) * FinalProject.TILE_SPACING - odometer.getX(),
//					searchRegionPath.get(3) % 8 * FinalProject.TILE_SPACING - odometer.getY())));
//			while (isNavigating())
//				continue;
//			turnToWithInterrupt(
//					Math.toDegrees(Math.atan2(searchRegionPath.get(6) * FinalProject.TILE_SPACING - odometer.getX(),
//							searchRegionPath.getLast() * FinalProject.TILE_SPACING - odometer.getY())));
//			while (poller.getReading() > THRESHOLD) {
//				continue;
//			}
//			FinalProject.leftMotor.stop(true);
//			FinalProject.rightMotor.stop(false);
//			distance = poller.getReading();
//			FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, distance - 5), true);
//			FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, distance - 5), false);
//			while (isNavigating())
//				continue;
//			FinalProject.usMotor.setSpeed(50);
//			LightPoller.checkColors();
//			if (LightPoller.checkColors()) {
//				for (int j = 0; j < 3; j++) {
//					Sound.beep();
//					try {
//						Thread.sleep(500);
//					} catch (InterruptedException e) {
//					}
//				}
//				hasFlag = true;
//				FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, -10), true);
//				FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, -10), false);
//				while (isNavigating())
//					continue;
//				travelToAfterFlag();
//				return;
//			}
//			FinalProject.usMotor.rotateTo(sensorMotor.reference - 45);
//			while (FinalProject.usMotor.isMoving()) {
//				if (LightPoller.checkColors()) {
//					for (int j = 0; j < 3; j++) {
//						Sound.beep();
//						try {
//							Thread.sleep(500);
//						} catch (InterruptedException e) {
//						}
//					}
//					hasFlag = true;
//					FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, -10), true);
//					FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, -10), false);
//					while (isNavigating())
//						continue;
//					travelToAfterFlag();
//					return;
//
//				}
//			}
//			FinalProject.usMotor.rotateTo(sensorMotor.reference + 45);
//			while (FinalProject.usMotor.isMoving()) {
//				if (LightPoller.checkColors()) {
//					for (int j = 0; j < 3; j++) {
//						Sound.beep();
//						try {
//							Thread.sleep(500);
//						} catch (InterruptedException e) {
//						}
//					}
//					hasFlag = true;
//					FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, -10), true);
//					FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, -10), false);
//					while (isNavigating())
//						continue;
//					travelToAfterFlag();
//					return;
//				}
//			}
//			searchRegionPath.addLast(searchRegionPath.removeFirst());
//			searchRegionPath.addLast(searchRegionPath.removeFirst());
//		}
//
//	}

	public void flagSearch2(int correctColors) {
		hasFlag =false;
		double usMotorAngle = 0; // angle of the small usMotor
		setColorProvider(Test.colorpoller);
		detect = false;		
		int searchDistance = 22;

			if(odometer.getTheta() < 1.74533 && odometer.getTheta() > 1.39626 && flagSearchX > 0) { //odometer near 90 deg
				Sound.beep();
				turnTo(45);


				distance = poller.getReading();
				filter_close(distance);
				if (detect ==false) {
					FinalProject.usMotor.rotateTo(sensorMotor.reference - 45, true);//turn the motor 45 deg
					while (FinalProject.usMotor.isMoving()) {
						distance = poller.getReading();
						filter_close(distance);
						//System.out.println(distance);
						if(distance < 40) {
							System.out.println(distance);
							FinalProject.usMotor.stop(); //stop if detect something
							Sound.beep();
							usMotorAngle =  FinalProject.usMotor.getTachoCount() - sensorMotor.reference ;
							System.out.println(usMotorAngle);
							if (usMotorAngle < -15) {
								usMotorAngle = usMotorAngle + 7;
							}
							System.out.println(usMotorAngle);
							detect = true;
							FinalProject.usMotor.rotateTo(sensorMotor.reference); //set the sensors straight

							turnTo(45 - usMotorAngle ); //make robot turn towards obstacle
							// the constant is added to make the correct turn
							//usMotorAngle has a negative value due to the way the tachocount is made.
							FinalProject.leftMotor.setSpeed(60);
							FinalProject.rightMotor.setSpeed(60);
							double X = FinalProject.odometer.getX();
							double Y = FinalProject.odometer.getY();
							LightPoller.on();
							while(distance > 4) { // make the robot go to the block and stop when it gets too close
								distance = poller.getReading();
								//System.out.println(distance);
								FinalProject.leftMotor.forward();
								FinalProject.rightMotor.forward();
								if(FinalProject.odometer.getX() - X > searchDistance || FinalProject.odometer.getY() - Y > searchDistance) { //break if it went too far and didnt recognize anything
									break;
								}
							}
							FinalProject.leftMotor.stop(true);
							FinalProject.rightMotor.stop(false);
							
							if (colorpoller.checkColors(correctColors)) {
								for (int j = 0; j < 3; j++) {
									Sound.beep();
									try {
										Thread.sleep(500);
									} catch (InterruptedException e) {
									}
								}
								hasFlag = true;
								FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, -searchDistance), true);
								FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, -searchDistance), false);
							}LightPoller.off();
						}
					}
				}
				if (detect ==false) {
					FinalProject.usMotor.rotateTo(sensorMotor.reference + 45, true); //turn the motor 45 deg
					while (FinalProject.usMotor.isMoving()) {
						distance = poller.getReading();
						filter_close(distance);
						//System.out.println(distance);
						if(distance < 40) {
							System.out.println(distance);
							Sound.beep();
							FinalProject.usMotor.stop();//stop if detect something
							usMotorAngle = FinalProject.usMotor.getTachoCount()  - sensorMotor.reference ;
							System.out.println(usMotorAngle);
							if (usMotorAngle > 15) {
								usMotorAngle = usMotorAngle + 7;
							}
							FinalProject.usMotor.rotateTo(sensorMotor.reference); //set the sensors straight
							detect = true;
							System.out.println(usMotorAngle);
							
							turnTo(45 - usMotorAngle  ); //make robot turn towards obstacle
							FinalProject.leftMotor.setSpeed(60);
							FinalProject.rightMotor.setSpeed(60);
							double X = FinalProject.odometer.getX();
							double Y = FinalProject.odometer.getY();
							poller.on();
							while(distance > 4) { // make the robot go to the block and stop when it gets too close
								distance = poller.getReading();
								//System.out.println(distance);
								FinalProject.leftMotor.forward();
								FinalProject.rightMotor.forward();
								if(FinalProject.odometer.getX() - X > searchDistance || FinalProject.odometer.getY() - Y > searchDistance) {//break if it went too far and didnt recognize anything
									break;
								}
							}
							FinalProject.leftMotor.stop(true);
							FinalProject.rightMotor.stop(false);
							if (colorpoller.checkColors(correctColors)) { //flag found
								for (int j = 0; j < 3; j++) {
									Sound.beep();
									try {
										Thread.sleep(500);
									} catch (InterruptedException e) {
									}
								}
								hasFlag = true;
								FinalProject.leftMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, -searchDistance), true);
								FinalProject.rightMotor.rotate(convertDistance(FinalProject.WHEEL_RADIUS, -searchDistance), false);
							}poller.off();
						}
					}
				}
			}FinalProject.usMotor.rotateTo(sensorMotor.reference);
		}
	
	
	public void flagSearchTravel(int correctColors) {
		flagSearchX = searchRegionPath.get(2); //set counter to X max
		flagSearchY = searchRegionPath.get(3); //set counter to Y max

		int counterX = flagSearchX - searchRegionPath.get(0) - 1;
		int counterY = flagSearchY - searchRegionPath.get(1) - 1;	
		

		FinalProject.usMotor.setSpeed(35);

		
		
		for(int y = 0; y <= counterY; y++) {
			for(int x = 0; x<= counterX; x++) {
				if (detect == false) {
					oc.on();
					travelTo(searchRegionPath.get(0) + x, searchRegionPath.get(1) + y);
					oc.off();
				}
				else {
					travelToWithoutAvoid(searchRegionPath.get(0) + x, searchRegionPath.get(1) + y);
				}
				turnTo(90); //face the next point
				flagSearch2(correctColors);
										
				if (hasFlag == true) {
					break;
				}
			}
			if (hasFlag == true) {
				break;
			}
			oc.on();
			travelToWithoutAvoid(searchRegionPath.get(0) + counterX , searchRegionPath.get(1) + y);
			
			travelTo(searchRegionPath.get(0), searchRegionPath.get(1) + y );
			oc.off();
			
			if (y < counterY) { 
				travelToWithoutAvoid(searchRegionPath.get(0), searchRegionPath.get(1) + y +1);
			}


		}
		travelToWithoutAvoid(searchRegionPath.get(2), searchRegionPath.get(3));
	}

	
	
	
	
	
	
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
				travelToWithoutAvoid(FinalProject.URSRGX, FinalProject.LLSRGY);
				while (isNavigating())
					continue;

			} else {
				travelToWithoutAvoid(FinalProject.LLSRGX, FinalProject.LLSRGY);
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
	public double filter_close(double distance) {

		
		if (distance <= 40 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
			this.distance = 41;
		} else if (distance <= 40 && filterControl >= FILTER_OUT) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}
		return this.distance;
	}
}

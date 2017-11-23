/*
 * SquareDriver.java
 */
package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.Sound;

// TODO: Auto-generated Javadoc
/**
 * The Class LightLocalizer, used to allow the robot in the beginning and in
 * times when error accumulates too much to re-adjust itself and start off fresh
 * Using one to potentially two light sensors to perform this task.
 *
 * @version 1.0
 */
public class LightLocalizer {

	/**
	 * The Constant SENSOR_OFFSET. Offset of the sensor from the center of the robot
	 */
	// distance between sensor and rotation center
	private static final double SENSOR_OFFSET = 12.79;

	/**
	 * The Constant CORRECTION_PERIOD. Used to sample from the light sensor at a
	 * fixed rate
	 */
	private static final long CORRECTION_PERIOD = 12;

	/** The Constant MOTOR_SPEED. Primary motor speed used for light localization */
	private static final int MOTOR_SPEED = 100;

	/** The odometer. */
	// create variables
	private Odometer odometer;

	/** The navigation. */
	private Navigation navigation;

	/** The nbr of lines. */
	// Define variables needed

	/** The color sensor. */

	/** The joint poller. */
	private JointLightPoller jointPoller;

	public boolean localizing;
	// assign port to light sensor

	/**
	 * Instantiates a new light localizer.
	 *
	 * @param odometer
	 *            ,the odometer used for light localization
	 * @param navigation
	 *            ,the navigation we will use to travel to 0,0 once we figure out
	 *            correct heading
	 * @param jointpoller
	 *            the jointpoller
	 */
	public LightLocalizer(Odometer odometer, Navigation navigation, JointLightPoller jointpoller) {
		this.odometer = odometer;
		this.navigation = navigation;

		this.jointPoller = jointpoller;

	}

	/**
	 * 4th version of this method that uses the joint poller rather than the two
	 * individual light pollers to synchronize values. Reduced time of light
	 * localization also with this method
	 */
	public void startLightLOC4() {
		// navigation.turn(10);
		// while(navigation.isNavigating()) continue;
		odometer.setTheta(0);
		// initialize color sensor
		Sound.beepSequenceUp();
		localizing = true;

		// Initialize theta, it will be corrected

		// the following code enables the robot to position itself so that the
		// light sensor will hit all four lines
		FinalProject.leftMotor.setSpeed(Navigation.MOTOR_SPEED); // set speeds
		FinalProject.rightMotor.setSpeed(Navigation.MOTOR_SPEED_RIGHT);

		FinalProject.leftMotor.forward(); // Run forward
		FinalProject.rightMotor.forward();
		double[] lightValue = new double[6];
		// Before starting turning, make the robot go to (-25, -25)
		while (true) { // Set the crossedLine flag to be true when it
			// crosses a line
			// get sample from light sensor
			lightValue = jointPoller.getValues();
			if (lightValue[0] < 0.3 && lightValue[1] < 0.3) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beepSequence();
				odometer.setY(FinalProject.TILE_SPACING + SENSOR_OFFSET);
				break;

			}
			// when the sensor sees a black line, stop the motors
			if (lightValue[0] < 0.3) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beep();

				checkRightPoller();
				FinalProject.rightMotor.stop(false);
				Sound.beep();
				odometer.setTheta(0);
				odometer.setY(FinalProject.TILE_SPACING + SENSOR_OFFSET);
				break;

			}
			if (lightValue[1] < 0.3) {
				FinalProject.rightMotor.stop(true);
				FinalProject.leftMotor.stop(false);
				Sound.beep();

				checkLeftPoller();
				FinalProject.rightMotor.stop(false);
				Sound.beep();
				odometer.setTheta(0);
				odometer.setY(FinalProject.TILE_SPACING + SENSOR_OFFSET);

				break;
			}

		}

		// once the sensor sees the black line, drive 25 cm backwards
		// navigation.driveWithoutAvoid(-25);

		navigation.turnTo(90); // turn to 90 degrees
		FinalProject.leftMotor.setSpeed(Navigation.MOTOR_SPEED);
		FinalProject.rightMotor.setSpeed(Navigation.MOTOR_SPEED_RIGHT);
		// drive forward until the sensor crosses a black line
		FinalProject.leftMotor.forward();
		FinalProject.rightMotor.forward();
		while (true) { // Set the crossedLine flag to be true when it
			// crosses a line
			// get sample from light sensor
			lightValue = jointPoller.getValues();
			if (lightValue[0] < 0.3 && lightValue[1] < 0.3) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beepSequence();
				odometer.setX(FinalProject.TILE_SPACING + SENSOR_OFFSET);
				break;

			}
			// when the sensor sees a black line, stop the motors
			if (lightValue[0] < 0.3) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				Sound.beep();

				checkRightPoller();
				FinalProject.rightMotor.stop(false);
				Sound.beep();
				odometer.setTheta(Math.PI / 2);
				odometer.setX(FinalProject.TILE_SPACING + SENSOR_OFFSET);
				break;

			}
			if (lightValue[1] < 0.3) {
				FinalProject.rightMotor.stop(true);
				FinalProject.leftMotor.stop(false);
				Sound.beep();

				checkLeftPoller();
				FinalProject.leftMotor.stop(false);
				Sound.beep();
				odometer.setTheta(Math.PI / 2);
				odometer.setX(FinalProject.TILE_SPACING + SENSOR_OFFSET);
				break;
			}

		}

		navigation.travelToWithoutAvoid(1, 1);
		navigation.turnTo(0);
		while (Navigation.isNavigating())
			continue;

		Sound.playNote(Sound.XYLOPHONE, 500, 500);
		localizing = false;
	}

	/**
	 * Send resources to check if the right poller is about to cross the line, so it
	 * is more likely to detect
	 */

	/**
	 * Send resources to check if the left poller is about to cross the line, so it
	 * is more likely to detect
	 */

	/**
	 * Check right poller and after it crosses set the speed back to what it was
	 * before
	 *
	 * @param speed
	 *            the speed
	 */
	private void checkRightPoller() {

		FinalProject.rightMotor.setSpeed(80);
		FinalProject.leftMotor.setSpeed(80);
		FinalProject.leftMotor.backward();
		FinalProject.rightMotor.backward();
		while (jointPoller.getLeftValue() > 0.23) {
			continue;
		}
		FinalProject.leftMotor.stop(true);
		FinalProject.rightMotor.stop(false);
		FinalProject.rightMotor.setSpeed(50);
		FinalProject.rightMotor.forward();
		while (jointPoller.getRightValue() > 0.23) {
			continue;
			/*
			 * if (timedOut(startTime)) break; continue;
			 */
		}
		FinalProject.rightMotor.stop(false);
		FinalProject.leftMotor.setSpeed(Navigation.MOTOR_SPEED);
		FinalProject.rightMotor.setSpeed(Navigation.MOTOR_SPEED_RIGHT);

	}

	/**
	 * Check left poller 2.
	 *
	 * @param speed
	 *            the speed
	 */
	private void checkLeftPoller() {
		/*
		 * FinalProject.leftMotor.setSpeed(40); FinalProject.leftMotor.forward(); while
		 * (jointPoller.getLeftValue() > 0.3) continue;
		 * FinalProject.leftMotor.stop(false);
		 * FinalProject.leftMotor.setSpeed(Navigation.MOTOR_SPEED);
		 */
		FinalProject.rightMotor.setSpeed(80);
		FinalProject.leftMotor.setSpeed(80);
		FinalProject.rightMotor.backward();
		FinalProject.leftMotor.backward();
		while (jointPoller.getRightValue() > 0.23)
			continue;
		FinalProject.leftMotor.stop(true);
		FinalProject.rightMotor.stop(false);
		FinalProject.leftMotor.setSpeed(50);
		FinalProject.leftMotor.forward();
		while (jointPoller.getLeftValue() > 0.23) {
			continue;
		}
		FinalProject.leftMotor.stop(false);
		FinalProject.leftMotor.setSpeed(Navigation.MOTOR_SPEED);
		FinalProject.rightMotor.setSpeed(Navigation.MOTOR_SPEED_RIGHT);

	}

	public void sweepRight() {
		FinalProject.rightMotor.setSpeed(Navigation.MOTOR_SPEED_RIGHT);
		FinalProject.leftMotor.setSpeed(Navigation.MOTOR_SPEED);
		FinalProject.leftMotor.rotate(Navigation.convertDistance(FinalProject.WHEEL_RADIUS, 21), true);
		FinalProject.rightMotor.rotate(Navigation.convertDistance(FinalProject.WHEEL_RADIUS, 21), true);
		while (FinalProject.leftMotor.isMoving() && FinalProject.rightMotor.isMoving()) {
			double[] lightValue = jointPoller.getValues();
			if (lightValue[0] < 0.25) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				repositionRight();
				break;

			}
			if (lightValue[1] < 0.25) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				repositionRight();
				break;
			}
		}
	}

	public void sweepLeft() {
		FinalProject.leftMotor.rotate(Navigation.convertDistance(FinalProject.WHEEL_RADIUS, 7), true);
		FinalProject.rightMotor.rotate(Navigation.convertDistance(FinalProject.WHEEL_RADIUS, 7), true);
		while (FinalProject.leftMotor.isMoving() && FinalProject.rightMotor.isMoving()) {
			double[] lightValue = jointPoller.getValues();
			if (lightValue[0] < 0.25) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				repositionLeft();
				break;

			}
			if (lightValue[1] < 0.25) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				repositionLeft();
				break;
			}
		}
	}

	private void repositionRight() {
		navigation.turnWithoutInterruption(90);
		FinalProject.leftMotor.rotate(Navigation.convertDistance(FinalProject.WHEEL_RADIUS, 6), true);
		FinalProject.rightMotor.rotate(Navigation.convertDistance(FinalProject.WHEEL_RADIUS, 6), false);
		navigation.turnWithoutInterruption(-90);
		while (Navigation.isNavigating())
			continue;
	}

	private void repositionLeft() {
		navigation.turnWithoutInterruption(-90);
		FinalProject.leftMotor.rotate(Navigation.convertDistance(FinalProject.WHEEL_RADIUS, 6), true);
		FinalProject.rightMotor.rotate(Navigation.convertDistance(FinalProject.WHEEL_RADIUS, 6), false);
		navigation.turnWithoutInterruption(90);
		FinalProject.leftMotor.rotate(-Navigation.convertDistance(FinalProject.WHEEL_RADIUS, 7), true);
		FinalProject.rightMotor.rotate(-Navigation.convertDistance(FinalProject.WHEEL_RADIUS, 7), false);
		while (Navigation.isNavigating())
			continue;

	}

}

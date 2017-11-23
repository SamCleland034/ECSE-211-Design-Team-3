
package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.Sound;

/**
 * The Class OdometryCorrection, used to correct the small but accumulating
 * errors of the odometer using the light sensor to detect gridlines on the
 * floor. This class samples from the JointLightPoller class and because we want
 * the data from the two light sensors used for correcting to be in sync. We can
 * correct the angle by making one motor catch up to the other if one light
 * sensor detects a line.
 * 
 */
public class OdometryCorrection extends Thread {

	/** The odometer. */
	private Odometer odometer;
	/** The on. */
	boolean on;

	/** The joint poller. */
	private JointLightPoller jointPoller;

	/** The sampling period for this thread. */
	private static int SAMPLINGPERIOD = 12;
	/** The distance between lines. */
	private static double TILE_SPACING = 30.48;

	/** The corrected. */
	public boolean corrected = false;

	/** The gps. */
	private Navigation gps;
	// Another variable for on for the gps to know if the OC is actually done
	// executing
	public boolean isOn = false;
	public int counter;

	// private EV3ColorSensor colorSensor;

	/** The Constant SENSOR_OFFSET. */
	private static final double SENSOR_OFFSET = 12.9;

	// constructor

	/**
	 * Instantiates a new odometry correction.
	 *
	 * @param odometer
	 *            the odometer
	 * @param jointPoller
	 *            the joint poller
	 */
	public OdometryCorrection(Odometer odometer, JointLightPoller jointPoller) {

		this.odometer = odometer;
		this.jointPoller = jointPoller;
		this.on = false;
	}

	// run method (required for Thread)

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {

		long startTime;
		long endTime;
		double[] lightValue;
		counter = 0;
		while (true) {
			if (on) {
				isOn = true;
				startTime = System.currentTimeMillis();
				lightValue = jointPoller.getValues();
				if (lightValue[0] < 0.23 && lightValue[1] < 0.23) {
					if (counter == 0) {
						Sound.beepSequence();
						checkOrientation();
						corrected = true;
						counter = 2;
						isOn = false;
						sleepFor(1);
					} else {
						counter--;
						isOn = false;
						sleepFor(2);
					}
				}
				if (lightValue[0] < 0.23) {
					if (counter == 0) {
						FinalProject.rightMotor.stop(true);
						FinalProject.leftMotor.stop(false);

						Sound.beep();
						checkRightPoller();

						counter = 2;
						isOn = false;
						corrected = true;
						sleepFor(1);
					} else {
						counter--;
						isOn = false;
						sleepFor(2);
					}
				}
				if (lightValue[1] < 0.23) {
					if (counter == 0) {
						FinalProject.leftMotor.stop(true);
						FinalProject.rightMotor.stop(false);

						Sound.beep();
						checkLeftPoller();
						counter = 2;

						isOn = false;
						corrected = true;
						sleepFor(1);
					} else {
						counter--;
						isOn = false;
						sleepFor(2);
					}
				}

				endTime = System.currentTimeMillis();
				if (endTime - startTime < SAMPLINGPERIOD) {
					try {
						Thread.sleep((SAMPLINGPERIOD - (endTime - startTime)));
					} catch (InterruptedException e) {
					}
				}
			} else {
				isOn = false;
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
				}
			}
		}

	}

	/**
	 * Sleep for a period in seconds
	 *
	 * @param i
	 *            determines how long to sleep, seconds
	 */
	private void sleepFor(int i) {
		try {
			sleep(1000 * i);
		} catch (InterruptedException e) {
		}
	}

	/**
	 * Check orientation of the robot, we correct y if it crosses a line while theta
	 * is between a certain threshold for y, and x if theta is within the threshold
	 * corresponding to x
	 * 
	 */
	private void checkOrientation() {
		double theta;
		int correctedX = 0;
		int correctedY = 0;

		theta = odometer.getTheta();
		if ((theta) > 7 * Math.PI / 4 || ((theta > 0) && (theta <= Math.PI / 4))) {
			odometer.setTheta(0);
			correctedY = (int) (odometer.getY() / TILE_SPACING);
			odometer.setY(correctedY * TILE_SPACING + SENSOR_OFFSET);
		} else if (theta >= 5 * Math.PI / 4 && theta <= 7 * Math.PI / 4) {
			odometer.setTheta(3 * Math.PI / 2);
			correctedX = (int) ((odometer.getX() + TILE_SPACING) / TILE_SPACING);
			odometer.setX(correctedX * TILE_SPACING - SENSOR_OFFSET);
		} else if (theta >= 3 * Math.PI / 4 && theta <= 5 * Math.PI / 4) {
			odometer.setTheta(Math.PI);
			correctedY = (int) ((odometer.getY() + TILE_SPACING) / TILE_SPACING);
			odometer.setY(correctedY * TILE_SPACING - SENSOR_OFFSET);
		} else {
			odometer.setTheta(Math.PI / 2);
			correctedX = (int) (odometer.getX() / TILE_SPACING);
			odometer.setX(correctedX * TILE_SPACING + SENSOR_OFFSET);

		}
	}

	/**
	 * Check right poller once the left sensor crosses a line, don't need to keep
	 * rechecking the left poller so send all resources here
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

		}
		FinalProject.rightMotor.stop(false);
		checkOrientation();
		FinalProject.leftMotor.setSpeed(Navigation.MOTOR_SPEED);
		FinalProject.rightMotor.setSpeed(Navigation.MOTOR_SPEED_RIGHT);
	}

	/**
	 * Check left poller once the right poller crosses a line before the left poller
	 * does, send all resources to checking for the left poller since we don't need
	 * to recheck the right light poller
	 */
	private void checkLeftPoller() {
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
		checkOrientation();
		FinalProject.leftMotor.setSpeed(Navigation.MOTOR_SPEED);
		FinalProject.rightMotor.setSpeed(Navigation.MOTOR_SPEED_RIGHT);

	}

	/**
	 * Sets the navigation.
	 *
	 * @param gps
	 *            the new navigation
	 */
	public void setNavigation(Navigation gps) {
		this.gps = gps;
	}

	/**
	 * Turn thread on.
	 */
	public void on() {
		this.on = true;
	}

	/**
	 * Turn thread off.
	 */
	public void off() {
		this.on = false;
	}
}

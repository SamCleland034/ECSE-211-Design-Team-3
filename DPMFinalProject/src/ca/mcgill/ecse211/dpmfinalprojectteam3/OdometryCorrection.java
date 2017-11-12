
package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.Sound;

// TODO: Auto-generated Javadoc
/**
 * The Class OdometryCorrection, used to correct the small but accumulating
 * errors of the odometer using the light sensor to detect gridlines on the
 * floor. This class samples from the JointLightPoller class and because we are
 * using two light sensors, we can correct the angle by making one motor catch
 * up to the other if one light sensor detects a line.
 * 
 */
public class OdometryCorrection extends Thread {

	/** The odometer. */
	private Odometer odometer;

	/** The left poller. */
	private LightPoller leftPoller;

	/** The right poller. */
	private LightPoller rightPoller;

	/** The on. */
	boolean on;

	/** The joint poller. */
	private JointLightPoller jointPoller;

	/** The samplingperiod. */
	private static int SAMPLINGPERIOD = 10;
	/** The distance between lines. */
	private static double TILE_SPACING = 30.48;

	/** The corrected. */
	public boolean corrected = false;

	/** The gps. */
	private Navigation gps;

	// private EV3ColorSensor colorSensor;

	/** The Constant SENSOR_OFFSET. */
	private static final double SENSOR_OFFSET = 12.9;

	// constructor

	/**
	 * Instantiates a new odometry correction.
	 *
	 * @param odometer
	 *            the odometer
	 * @param leftPoller
	 *            the left poller
	 * @param rightPoller
	 *            the right poller
	 * @param jointPoller
	 *            the joint poller
	 */
	public OdometryCorrection(Odometer odometer, LightPoller leftPoller, LightPoller rightPoller,
			JointLightPoller jointPoller) {

		this.odometer = odometer;
		this.leftPoller = leftPoller;
		this.rightPoller = rightPoller;
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
		int speed = 0;
		long startTime;
		long endTime;
		double[] lightValue;
		int counter = 1;
		while (true) {
			if (on) {
				startTime = System.currentTimeMillis();
				lightValue = jointPoller.getValues();
				if (lightValue[0] < 0.23 && lightValue[1] < 0.23) {
					if (counter == 0) {
						Sound.beepSequence();
						checkOrientation();
						corrected = true;
						counter = 1;
						sleepFor(1);
					} else {
						counter--;
						sleepFor(2);
					}
				}
				if (lightValue[0] < 0.23) {
					if (counter == 0) {
						FinalProject.leftMotor.stop(true);
						FinalProject.rightMotor.stop(false);

						Sound.beep();
						checkRightPoller();
						corrected = true;
						counter = 1;
						sleepFor(1);
					} else {
						counter--;

						sleepFor(2);
					}
				}
				if (lightValue[1] < 0.23) {
					if (counter == 0) {
						FinalProject.rightMotor.stop(true);
						FinalProject.leftMotor.stop(false);
						speed = FinalProject.rightMotor.getSpeed();
						Sound.beep();
						checkLeftPoller();
						corrected = true;
						counter = 1;
						sleepFor(1);
					} else {
						counter--;
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
		// FinalProject.rightMotor.setSpeed(50);
		// FinalProject.rightMotor.forward();
		// long startTime = System.currentTimeMillis();
		FinalProject.rightMotor.setSpeed(45);
		FinalProject.leftMotor.setSpeed(45);
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
		FinalProject.rightMotor.setSpeed(45);
		FinalProject.leftMotor.setSpeed(45);
		FinalProject.rightMotor.backward();
		FinalProject.leftMotor.backward();
		while (jointPoller.getRightValue() > 0.23)
			continue;
		FinalProject.leftMotor.stop(true);
		FinalProject.rightMotor.stop(false);
		FinalProject.leftMotor.setSpeed(50);
		FinalProject.leftMotor.forward();
		// FinalProject.leftMotor.setSpeed(50);
		// FinalProject.leftMotor.forward();
		// long startTime = System.currentTimeMillis();
		while (jointPoller.getLeftValue() > 0.23) {
			/*
			 * if (timedOut(startTime)) break;
			 */
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
	 * On.
	 */
	public void on() {
		this.on = true;
	}

	/**
	 * Off.
	 */
	public void off() {
		this.on = false;
	}
}

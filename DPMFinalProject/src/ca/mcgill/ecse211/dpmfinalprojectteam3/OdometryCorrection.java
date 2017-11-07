
package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.Sound;

/**
 * The Class OdometryCorrection, used to correct the small but accumulating
 * errors of the odometer using the light sensor to detect gridlines on the
 * floor
 * 
 * @version 1.0
 */
public class OdometryCorrection extends Thread {

	/** The odometer. */
	private Odometer odometer;
	private LightPoller leftPoller;
	private LightPoller rightPoller;

	private boolean on;
	private JointLightPoller jointPoller;
	private static int SAMPLINGPERIOD = 15;
	/** The distance between lines. */
	private static double TILE_SPACING = 30;
	public boolean corrected = false;
	private Navigation gps;

	// private EV3ColorSensor colorSensor;

	private static final double SENSOR_OFFSET = 12.9;

	// constructor

	/**
	 * Instantiates a new odometry correction.
	 *
	 * @param odometer
	 *            the odometer
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
		while (true) {
			if (on) {
				startTime = System.currentTimeMillis();
				lightValue = jointPoller.getValues();
				if (lightValue[0] < 0.3 && lightValue[1] < 0.3) {
					Sound.beepSequence();
					checkOrientation();
					gps.corrected = true;
				} else {
					if (lightValue[0] < 0.3) {
						speed = FinalProject.leftMotor.getSpeed();
						FinalProject.leftMotor.stop(true);
						FinalProject.rightMotor.stop(false);
						Sound.beep();
						checkRightPoller(speed);
						gps.corrected = true;

					}
					if (lightValue[1] < 0.3) {
						speed = FinalProject.rightMotor.getSpeed();
						FinalProject.rightMotor.stop(true);
						FinalProject.leftMotor.stop(false);
						Sound.beep();
						checkLeftPoller(speed);
						gps.corrected = true;

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
			correctedY = (int) (odometer.getY() / TILE_SPACING);
			odometer.setY(correctedY * TILE_SPACING + SENSOR_OFFSET);
		} else if (theta >= 3 * Math.PI / 4 && theta <= 5 * Math.PI / 4) {
			odometer.setTheta(Math.PI);
			correctedX = (int) (odometer.getX() / TILE_SPACING);
			odometer.setX(correctedX * TILE_SPACING + SENSOR_OFFSET);
		} else {
			odometer.setTheta(Math.PI / 2);
			correctedX = (int) (odometer.getX() / TILE_SPACING);
			odometer.setX(correctedX * TILE_SPACING + SENSOR_OFFSET);

		}
	}

	private void checkRightPoller(int speed) {

		FinalProject.rightMotor.setSpeed(50);
		FinalProject.rightMotor.forward();
		long startTime = System.currentTimeMillis();
		while (jointPoller.getRightValue() > 0.3) {
			if (timedOut(startTime))
				break;
			continue;
		}
		FinalProject.rightMotor.stop(false);
		checkOrientation();
		FinalProject.leftMotor.setSpeed(speed);
		FinalProject.rightMotor.setSpeed(speed);

	}

	private boolean timedOut(long startTime) {
		if (System.currentTimeMillis() - startTime > 50)
			return true;
		return false;
	}

	private void checkLeftPoller(int speed) {

		FinalProject.leftMotor.setSpeed(50);
		FinalProject.rightMotor.forward();
		long startTime = System.currentTimeMillis();
		while (jointPoller.getLeftValue() > 0.3) {
			if (timedOut(startTime))
				break;
			continue;
		}
		FinalProject.leftMotor.stop(false);
		checkOrientation();
		FinalProject.leftMotor.setSpeed(speed);
		FinalProject.rightMotor.setSpeed(speed);

	}

	public void setNavigation(Navigation gps) {
		this.gps = gps;
	}

	public void on() {
		this.on = true;
	}

	public void off() {
		this.on = false;
	}
}

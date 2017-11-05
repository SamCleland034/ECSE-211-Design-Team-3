
package ca.mcgill.ecse211.dpmfinalprojectteam3;

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

	/** The distance between lines. */
	private static double TILE_SPACING = 30;

	// private EV3ColorSensor colorSensor;

	private static final double SENSOR_OFFSET = 12.8;

	// constructor

	/**
	 * Instantiates a new odometry correction.
	 *
	 * @param odometer
	 *            the odometer
	 */
	public OdometryCorrection(Odometer odometer, LightPoller leftPoller, LightPoller rightPoller) {

		this.odometer = odometer;
		this.leftPoller = leftPoller;
		this.rightPoller = rightPoller;
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
		double leftChange;
		double rightChange;
		while (true) {
			if (on) {
				leftChange = leftPoller.getChangeInLight();
				rightChange = rightPoller.getChangeInLight();
				if (leftChange >= 1 && rightChange >= 1) {
					checkOrientation();
				} else {
					if (leftChange >= 1) {
						speed = FinalProject.leftMotor.getSpeed();
						FinalProject.leftMotor.setSpeed(0);
						checkRightPoller(speed);
					}
					if (rightChange >= 1) {
						speed = FinalProject.rightMotor.getSpeed();
						FinalProject.rightMotor.setSpeed(0);
						checkLeftPoller(speed);
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
		if ((odometer.getTheta() - 2 * Math.PI >= -Math.PI / 4 && odometer.getTheta() <= Math.PI / 4)) {
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

		while (rightPoller.getChangeInLight() < 1)
			continue;
		FinalProject.rightMotor.setSpeed(0);
		checkOrientation();
		FinalProject.leftMotor.setSpeed(speed);
		FinalProject.rightMotor.setSpeed(speed);

	}

	private void checkLeftPoller(int speed) {

		FinalProject.leftMotor.setSpeed(50);
		while (leftPoller.getChangeInLight() < 1)
			continue;
		FinalProject.leftMotor.setSpeed(0);
		checkOrientation();
		FinalProject.leftMotor.setSpeed(speed);
		FinalProject.rightMotor.setSpeed(speed);

	}

	public void on() {
		this.on = true;
	}

	public void off() {
		this.on = false;
	}
}

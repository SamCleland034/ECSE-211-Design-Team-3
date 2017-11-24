package ca.mcgill.ecse211.dpmfinalprojectteam3;

/**
 * Avoidance controls if the robot should be in the avoiding state or not while
 * navigating only. It takes in data from an ultrasonic poller instance and if
 * the data is less than a certain threshold, then we interupt whatever the
 * navigation is doing and avoid, using a bang bang wall follower. When the
 * robot thinks its safe again (after seeing nothing for a while), it will then
 * travelTo wherever it was previously travelling to.
 */
public class Avoidance extends Thread {

	/** The gps. */
	private Navigation gps;

	/** The Constant SAMPLINGPERIOD. */
	private static final long SAMPLINGPERIOD = 50;

	/** The poller. */
	private UltrasonicPoller poller;

	/**
	 * Filter for how much the robot has to see nothing for it to break out of bang
	 * bang controller.
	 */
	private static final int FILTERCONTROL = 92;

	private static final float MOTOR_SPEED = 250;
	private static final float MOTOR_SPEED_RIGHT = (float) (MOTOR_SPEED * Navigation.RIGHT_OFFSET);

	/** The avoiding. */
	public boolean avoiding;

	/** The in danger. */
	public boolean inDanger;

	/** The sensor motor. */
	private SensorRotation sensorMotor;

	/**
	 * Instantiates a new avoidance.
	 *
	 * @param gps
	 *            the gps
	 */
	public Avoidance(Navigation gps) {
		this.gps = gps;
		this.avoiding = false;
		gps.setAvoidance(this);
		this.inDanger = false;
	}

	/**
	 * Sets the sensor rotation.
	 *
	 * @param sensorMotor
	 *            the new sensor rotation
	 */
	public void setSensorRotation(SensorRotation sensorMotor) {
		this.sensorMotor = sensorMotor;
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

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		int distance;
		long startTime, endTime;
		while (true) {
			if (avoiding && !inDanger) {
				startTime = System.currentTimeMillis();
				distance = poller.getReading();
				// do avoidance
				if (distance < FinalProject.THRESHOLD) {
					inDanger = true;
				}
				endTime = System.currentTimeMillis();
				if (endTime - startTime < SAMPLINGPERIOD) {
					try {
						Thread.sleep(SAMPLINGPERIOD - (endTime - startTime));
					} catch (InterruptedException e) {
					}
				}
			} else if (avoiding && inDanger && gps.donecorrecting) {
				// do bang-bang controller algorithm
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				// reposition to set the robot at a 90 degree angle
				gps.turnWithoutInterruption(60);
				sensorMotor.off();
				sleepFor(0.5);
				while (FinalProject.usMotor.isMoving())
					continue;
				sleepFor(1);
				// shift the usmotor to the left
				FinalProject.usMotor.rotateTo(sensorMotor.reference + 52, true);
				while (FinalProject.usMotor.isMoving())
					continue;
				int filter = 0;
				int measure = poller.getReading();
				FinalProject.leftMotor.setSpeed(200);
				FinalProject.rightMotor.setSpeed(200);
				// start bang-bang controller
				do {

					startTime = System.currentTimeMillis();
					distance = poller.getReading();
					if (distance > FinalProject.THRESHOLD + 18 && filter == 0) {
						filter++;
						measure = distance;
					} else if (distance > FinalProject.THRESHOLD + 18 && filter < FILTERCONTROL) {
						filter++;
						// System.out.println("Filter =" + filter);
					} else if (distance > FinalProject.THRESHOLD + 18 && filter >= FILTERCONTROL) {
						// break out of bang-bang controller and continue travelling
						FinalProject.leftMotor.stop(true);
						FinalProject.rightMotor.stop(false);
						// System.out.println("Avoided Object");
						sleepFor(1);
						FinalProject.usMotor.rotateTo(sensorMotor.reference);
						while (FinalProject.usMotor.isMoving())
							continue;
						sleepFor(1);
						sensorMotor.on = true;
						inDanger = false;
						break;
					} else {
						filter = 0;
						measure = distance;
					}
					// go straight if within bandcenter
					if (measure > FinalProject.THRESHOLD && measure < FinalProject.THRESHOLD + 5) {
						FinalProject.leftMotor.setSpeed(MOTOR_SPEED);
						FinalProject.rightMotor.setSpeed(MOTOR_SPEED_RIGHT);
						FinalProject.leftMotor.forward();
						FinalProject.rightMotor.forward();
						// turn away from wall
					} else if (measure < FinalProject.THRESHOLD) {
						FinalProject.leftMotor.setSpeed(MOTOR_SPEED);
						FinalProject.rightMotor.setSpeed(MOTOR_SPEED_RIGHT / 2);
						FinalProject.leftMotor.forward();
						FinalProject.rightMotor.forward();
						// turn slightly to the wall, don't really mind if it doesn't go to the wall
						// fast since we are avoiding
					} else {
						FinalProject.leftMotor.setSpeed(MOTOR_SPEED * 2 / 3);
						FinalProject.rightMotor.setSpeed(MOTOR_SPEED_RIGHT);
						FinalProject.rightMotor.forward();
						FinalProject.leftMotor.forward();
					}
					// to adjust to sampling rate
					endTime = System.currentTimeMillis();
					if (endTime - startTime < SAMPLINGPERIOD) {
						try {
							Thread.sleep(SAMPLINGPERIOD - (endTime - startTime));
						} catch (InterruptedException e) {
						}
					}
				} while (inDanger);

			} else {
				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
				}
			}
		}
	}

	/**
	 * Sleep for a certain amount depending on the situation.
	 *
	 * @param x
	 *            the x
	 */
	public void sleepFor(double x) {
		try {
			Thread.sleep((long) (x * 100));
		} catch (InterruptedException e) {
		}
	}

	/**
	 * turn thread on.
	 */
	public void on() {
		avoiding = true;
	}

	/**
	 * turn thread off.
	 */
	public void off() {
		avoiding = false;
	}
}

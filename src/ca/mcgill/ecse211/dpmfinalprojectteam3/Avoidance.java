package ca.mcgill.ecse211.dpmfinalprojectteam3;

// TODO: Auto-generated Javadoc
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

	/** The Constant FILTERCONTROL. */
	private static final int FILTERCONTROL = 100;

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
				if (distance < FinalProject.THRESHOLD) {
					FinalProject.leftMotor.stop(true);
					FinalProject.rightMotor.stop(false);
					inDanger = true;
				}
				endTime = System.currentTimeMillis();
				if (endTime - startTime < SAMPLINGPERIOD) {
					try {
						Thread.sleep(SAMPLINGPERIOD - (endTime - startTime));
					} catch (InterruptedException e) {
					}
				}
			} else if (avoiding && inDanger) {
				FinalProject.leftMotor.stop(true);
				FinalProject.rightMotor.stop(false);
				gps.turn(-45);
				sensorMotor.off();
				sleepFor(0.5);
				while (FinalProject.usMotor.isMoving())
					continue;
				sleepFor(1);
				FinalProject.usMotor.rotateTo(sensorMotor.reference + 45);
				while (FinalProject.usMotor.isMoving())
					continue;
				int filter = 0;
				int measure = poller.getReading();
				FinalProject.leftMotor.setSpeed(200);
				FinalProject.rightMotor.setSpeed(200);
				do {

					startTime = System.currentTimeMillis();
					distance = poller.getReading();
					if (distance > FinalProject.THRESHOLD + 10 && filter < FILTERCONTROL) {
						filter++;
						// System.out.println("Filter =" + filter);
					} else if (distance > FinalProject.THRESHOLD + 10 && filter >= FILTERCONTROL) {
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
					if (measure > FinalProject.THRESHOLD && measure < FinalProject.THRESHOLD + 10) {
						FinalProject.leftMotor.setSpeed(250);
						FinalProject.rightMotor.setSpeed(250);
						FinalProject.leftMotor.forward();
						FinalProject.rightMotor.forward();
					} else if (measure < FinalProject.THRESHOLD) {
						FinalProject.leftMotor.setSpeed(250);
						FinalProject.rightMotor.setSpeed(125);
						FinalProject.leftMotor.forward();
						FinalProject.rightMotor.forward();
					} else {
						FinalProject.leftMotor.setSpeed(125);
						FinalProject.rightMotor.setSpeed(250);
						FinalProject.rightMotor.forward();
						FinalProject.leftMotor.forward();
					}
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

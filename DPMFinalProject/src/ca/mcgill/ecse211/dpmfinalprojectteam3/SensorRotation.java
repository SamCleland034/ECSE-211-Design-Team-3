package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to rotate the sensor to allow it to sweep without moving
 * the robot. Used since we want to sweep the area while also getting data from
 * the ultrasonic sensor for the avoidance and flagsearch threads.
 */
public class SensorRotation extends Thread {

	/** The master. */
	private Avoidance master;

	/** The motor. */
	private EV3LargeRegulatedMotor motor;

	/** The Constant SAMPLINGPERIOD. */
	private static final int SAMPLINGPERIOD = 1200;

	/** The on. */
	public boolean on;

	/** The gps. */
	private Navigation gps;

	/** The moved right. */
	private boolean movedLeft, movedRight;

	/** The reference. */
	public int reference;

	/**
	 * Instantiates a new sensor rotation.
	 *
	 * @param master
	 *            the master
	 * @param motor
	 *            the motor
	 * @param gps
	 *            the gps
	 */
	public SensorRotation(Avoidance master, EV3LargeRegulatedMotor motor, Navigation gps) {
		this.master = master;
		this.motor = motor;
		this.on = false;
		this.gps = gps;
		gps.setSensorRotation(this);
		master.setSensorRotation(this);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		long startTime, endTime;
		movedLeft = false;
		movedRight = false;
		boolean halfturn = false;

		motor.resetTachoCount();
		// creating reference point for the motor to rotate about
		reference = motor.getTachoCount();
		motor.setSpeed(70);
		while (true) {
			if (on) {
				startTime = System.currentTimeMillis();
				// hasn't turned yet
				if (!movedLeft && !halfturn) {
					motor.rotateTo(reference - 25);
					while (motor.isMoving())
						continue;
					movedLeft = true;
					movedRight = false;
					halfturn = true;
					// needs to back to the right
				} else if (!movedLeft && halfturn) {
					motor.rotateTo(reference - 25);
					while (motor.isMoving())
						continue;
					movedLeft = true;
					movedRight = false;
					halfturn = true;
					// turns back to the left
				} else if (!movedRight && halfturn) {
					motor.rotateTo(reference + 25);
					while (motor.isMoving())
						continue;
					movedLeft = false;
					movedRight = true;
					halfturn = true;

				}
				endTime = System.currentTimeMillis();
				if (endTime - startTime < SAMPLINGPERIOD) {
					try {
						Thread.sleep(SAMPLINGPERIOD - (endTime - startTime));
					} catch (InterruptedException e) {
					}
				}
				// just sleep if off
			} else {
				try {
					Thread.sleep(SAMPLINGPERIOD);
				} catch (InterruptedException e) {
				}
			}
		}
	}

	/**
	 * On.
	 */
	public void on() {
		on = true;
	}

	/**
	 * Off.
	 */
	public void off() {
		on = false;
	}

}

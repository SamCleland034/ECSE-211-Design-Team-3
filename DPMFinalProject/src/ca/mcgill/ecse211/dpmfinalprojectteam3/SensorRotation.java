package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class SensorRotation extends Thread {
	private Avoidance master;
	private EV3LargeRegulatedMotor motor;
	private static final int SAMPLINGPERIOD = 1200;
	public boolean on;
	private Navigation gps;
	private boolean movedLeft, movedRight;
	public int reference;

	public SensorRotation(Avoidance master, EV3LargeRegulatedMotor motor, Navigation gps) {
		this.master = master;
		this.motor = motor;
		this.on = false;
		this.gps = gps;
		gps.setSensorRotation(this);
		master.setSensorRotation(this);
	}

	public void run() {
		long startTime, endTime;
		movedLeft = false;
		movedRight = false;
		boolean halfturn = false;
		boolean corrected = false;
		motor.resetTachoCount();
		reference = motor.getTachoCount();
		motor.setSpeed(70);
		while (true) {
			if (on) {
				startTime = System.currentTimeMillis();
				if (!movedLeft && !halfturn) {
					motor.rotateTo(reference - 35);
					while (motor.isMoving())
						continue;
					movedLeft = true;
					movedRight = false;
					halfturn = true;
					corrected = false;

				} else if (!movedLeft && halfturn) {
					motor.rotateTo(reference - 70);
					while (motor.isMoving())
						continue;
					movedLeft = true;
					movedRight = false;
					halfturn = true;
					corrected = false;

				} else if (!movedRight && halfturn) {
					motor.rotateTo(reference + 70);
					while (motor.isMoving())
						continue;
					movedLeft = false;
					movedRight = true;
					halfturn = true;
					corrected = false;
				}
				endTime = System.currentTimeMillis();
				if (endTime - startTime < SAMPLINGPERIOD) {
					try {
						Thread.sleep(SAMPLINGPERIOD - (endTime - startTime));
					} catch (InterruptedException e) {
					}
				}
			} else if (!on && corrected) {
				try {
					Thread.sleep(SAMPLINGPERIOD);
				} catch (InterruptedException e) {
				}
			} else if (!movedLeft && !corrected) {
				motor.rotateTo(reference, false);
				movedLeft = false;
				movedRight = false;
				halfturn = false;
				corrected = true;
				motor.stop();
			} else if (!movedRight && !corrected) {
				motor.rotateTo(reference, false);
				movedLeft = false;
				movedRight = false;
				halfturn = false;
				corrected = true;
				motor.stop();
			}
		}
	}

	public void on() {
		on = true;
	}

	public void off() {
		on = false;
	}

}

package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class SensorRotation extends Thread {
	private Avoidance master;
	private EV3LargeRegulatedMotor motor;
	private static final int SAMPLINGPERIOD = 1000;
	public boolean on;
	private Navigation gps;
	private boolean movedLeft, movedRight;

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
		int count = 0;
		motor.setSpeed(70);
		while (true) {
			if (on) {
				startTime = System.currentTimeMillis();
				if (!movedLeft && count == 0) {
					motor.rotate(-30);
					while (motor.isMoving())
						continue;
					movedLeft = true;
					movedRight = false;
					count += 1;
				} else if (movedLeft && count > 0) {
					motor.rotate(-60);
					while (motor.isMoving())
						continue;
					movedLeft = true;
					movedRight = false;
					count += 1;
				} else if (movedRight) {
					motor.rotate(60);
					while (motor.isMoving())
						continue;
					movedLeft = false;
					movedRight = true;
					count += 1;
				}
				endTime = System.currentTimeMillis();
				if (endTime - startTime < SAMPLINGPERIOD) {
					try {
						Thread.sleep(SAMPLINGPERIOD - (endTime - startTime));
					} catch (InterruptedException e) {
					}
				}
			} else if (!movedLeft) {
				motor.rotate(30);
				movedLeft = false;
				movedRight = false;
				count = 0;
			} else if (!movedRight) {
				motor.rotate(-30);
				movedLeft = false;
				movedRight = false;
				count = 0;
			} else {
				try {
					Thread.sleep(SAMPLINGPERIOD);
				} catch (InterruptedException e) {
				}
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

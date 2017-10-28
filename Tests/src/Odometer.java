

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
	// robot position
	private double x;
	private double y;

	private int lastTachoLeft;// Tacho L at last sample
	private int lastTachoRight;// Tacho R at last sample
	private int nowTachoLeft;// Current tacho L
	private int nowTachoRight;// Current tacho R

	private double theta;
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private static final long ODOMETER_PERIOD = 25; /*
													 * odometer update period,
													 * in ms
													 */
	
	private Object lock; /* lock object for mutual exclusion */

	// default constructor
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 0.0;
		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
		lock = new Object();
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();
			// TODO put (some of) your odometer code here

			double distanceLeft, distanceRight, deltaDistance, deltaT, dX, dY;

			nowTachoLeft = leftMotor.getTachoCount(); // get tacho counts
			nowTachoRight = rightMotor.getTachoCount();

			// compute wheel displacement
			distanceLeft = Math.PI * Test.WHEEL_RADIUS * (nowTachoLeft - getLeftMotorTachoCount()) / 180; 
			distanceRight = Math.PI * Test.WHEEL_RADIUS * (nowTachoRight - getRightMotorTachoCount()) / 180; 

			setLeftMotorTachoCount(nowTachoLeft); // save tacho counts for next iteration
			setRightMotorTachoCount(nowTachoRight);

			deltaDistance = 0.5 * (distanceLeft + distanceRight); // compute vehicle displacement
			
			deltaT = (distanceLeft - distanceRight) / Test.TRACK; // compute change in heading

			// update heading
			dX = deltaDistance * Math.sin(getTheta()); // compute X component of displacement
			dY = deltaDistance * Math.cos(getTheta()); // compute Y component of displacement

			synchronized (lock) {
				/**
				 * Don't use the variables x, y, or theta anywhere but here!
				 * Only update the values of x, y, and theta in this block. Do
				 * not perform complex math
				 */

				setX(getX() + dX); // update estimates of X and Y position
				setY(getY() + dY);
				setTheta((getTheta()+deltaT));
				
				if (getTheta() < 0) { // Keep theta (in radians) between 0 and 2pi
					setTheta(getTheta() + 2 * Math.PI);
				} else if (getTheta() > 2 * Math.PI) {
					setTheta(getTheta() - 2 * Math.PI);
				}

			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = Math.toDegrees(theta); //Change radians to degrees
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}

	/**
	 * @return the leftMotorTachoCount
	 */
	public int getLeftMotorTachoCount() {
		return leftMotorTachoCount;
	}

	/**
	 * @param leftMotorTachoCount
	 *            the leftMotorTachoCount to set
	 */
	public void setLeftMotorTachoCount(int leftMotorTachoCount) {
		synchronized (lock) {
			this.leftMotorTachoCount = leftMotorTachoCount;
		}
	}

	/**
	 * @return the rightMotorTachoCount
	 */
	public int getRightMotorTachoCount() {
		return rightMotorTachoCount;
	}

	/**
	 * @param rightMotorTachoCount
	 *            the rightMotorTachoCount to set
	 */
	public void setRightMotorTachoCount(int rightMotorTachoCount) {
		synchronized (lock) {
			this.rightMotorTachoCount = rightMotorTachoCount;
		}
	}
}

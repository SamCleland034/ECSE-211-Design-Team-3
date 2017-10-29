package ca.mcgill.ecse211.dpmfinalprojectteam3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

// TODO: Auto-generated Javadoc
/**
 * Uses the odometer class to keep track of the position of
 * the robot with respect to the wheel rotations of the robot, along with the
 * heading of the robot
 */
public class Odometer extends Thread {

	/** The x. X position of the robot */
	// robot position
	private double x;

	/** The y. Y position of the robot */
	private double y;

	/** The last tacho left. Last tachometer reading of the left wheel */
	private int lastTachoLeft;// Tacho L at last sample

	/** The last tacho right. Last tachometer reading of the right wheel */
	private int lastTachoRight;// Tacho R at last sample

	/** The now tacho left. Current tachometer reading of the left wheel */
	private int nowTachoLeft;// Current tacho L

	/** The now tacho right. Current tachometer reading of the right wheel */
	private int nowTachoRight;// Current tacho R

	/** The theta. Heading of the robot */
	private double theta;

	/** The left motor tacho count. */
	private int leftMotorTachoCount;

	/** The right motor tacho count. */
	private int rightMotorTachoCount;

	/** The left motor. */
	private EV3LargeRegulatedMotor leftMotor;

	/** The right motor. */
	private EV3LargeRegulatedMotor rightMotor;

	/** The Constant ODOMETER_PERIOD. */
	private static final long ODOMETER_PERIOD = 22; /*
													 * odometer update period, in ms
													 */

	/** The lock. */
	private Object lock; /* lock object for mutual exclusion */

	/**
	 * Instantiates a new odometer.
	 *
	 * @param leftMotor
	 *            the left motor
	 * @param rightMotor
	 *            the right motor
	 */
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

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
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
			distanceLeft = Math.PI * FinalProject.WHEEL_RADIUS * (nowTachoLeft - getLeftMotorTachoCount()) / 180;
			distanceRight = Math.PI * FinalProject.WHEEL_RADIUS * (nowTachoRight - getRightMotorTachoCount()) / 180;

			setLeftMotorTachoCount(nowTachoLeft); // save tacho counts for next iteration
			setRightMotorTachoCount(nowTachoRight);

			deltaDistance = 0.5 * (distanceLeft + distanceRight); // compute vehicle displacement

			deltaT = (distanceLeft - distanceRight) / FinalProject.TRACK; // compute change in heading

			// update heading
			dX = deltaDistance * Math.sin(getTheta()); // compute X component of displacement
			dY = deltaDistance * Math.cos(getTheta()); // compute Y component of displacement

			synchronized (lock) {
				/**
				 * Updates the values of x, y, and theta in this block.
				 */

				setX(getX() + dX); // update estimates of X and Y position
				setY(getY() + dY);
				setTheta((getTheta() + deltaT));

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

	/**
	 * Gets the position.
	 *
	 * @param position,
	 *            parameters that want to be received
	 * @param update,
	 *            tells us which parameters need to be received
	 * @return the position, returns the position of the robot
	 */
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = Math.toDegrees(theta); // Change radians to degrees
		}
	}

	/**
	 * Gets the x.
	 *
	 * @return the x
	 */
	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	/**
	 * Gets the y.
	 *
	 * @return the y
	 */
	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	/**
	 * Gets the theta.
	 *
	 * @return the theta
	 */
	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	/**
	 * Sets the position.
	 *
	 * @param position,
	 *            the new coordinates of the robot that another object wants to set
	 *            the odometer to
	 * @param update,
	 *            which parameters need to be updated
	 */
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

	/**
	 * Sets the x.
	 *
	 * @param x
	 *            the new x
	 */
	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	/**
	 * Sets the y.
	 *
	 * @param y
	 *            the new y
	 */
	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	/**
	 * Sets the theta.
	 *
	 * @param theta
	 *            the new theta
	 */
	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}

	/**
	 * Gets the left motor tacho count.
	 *
	 * @return the leftMotorTachoCount
	 */
	public int getLeftMotorTachoCount() {
		return leftMotorTachoCount;
	}

	/**
	 * Sets the left motor tacho count.
	 *
	 * @param leftMotorTachoCount
	 *            the leftMotorTachoCount to set
	 */
	public void setLeftMotorTachoCount(int leftMotorTachoCount) {
		synchronized (lock) {
			this.leftMotorTachoCount = leftMotorTachoCount;
		}
	}

	/**
	 * Gets the right motor tacho count.
	 *
	 * @return the rightMotorTachoCount
	 */
	public int getRightMotorTachoCount() {
		return rightMotorTachoCount;
	}

	/**
	 * Sets the right motor tacho count.
	 *
	 * @param rightMotorTachoCount
	 *            the rightMotorTachoCount to set
	 */
	public void setRightMotorTachoCount(int rightMotorTachoCount) {
		synchronized (lock) {
			this.rightMotorTachoCount = rightMotorTachoCount;
		}
	}
}

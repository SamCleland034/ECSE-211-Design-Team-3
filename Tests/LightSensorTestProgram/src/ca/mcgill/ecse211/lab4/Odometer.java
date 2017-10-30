package ca.mcgill.ecse211.lab4;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
	// robot position
	private double x;
	private double y;
	private double theta;
	private double rho;
	private double changeInLambda;
	private double changeInLeft;
	private double changeInRight;
	private double lambda;
	private int leftMotorTachoCount;
	private double changeInX;
	private double changeInY;
	private double rhoPrev;
	private double RNormalAngle;
	private double RNormalMag;
	private double lambdaPrev;
	private double changeInTheta;
	private double changeInRho;
	private double thetaDegrees;
	private double changeInC;
	private double C;
	private int rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private static final long ODOMETER_PERIOD = 25; /* odometer update period, in ms */

	Object lock; /* lock object for mutual exclusion */

	// default constructor
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.x = 0.0;
		this.rho = 0.0;
		this.lambda = 0.0;
		this.changeInRho = 0.0;
		this.changeInLambda = 0.0;
		this.y = 0.0;
		this.theta = 0.0;
		this.thetaDegrees = 0.0;

		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
		lock = new Object();
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;
		this.thetaDegrees = 0;
		this.leftMotor.resetTachoCount();
		this.rightMotor.resetTachoCount();
		while (true) {
			updateStart = System.currentTimeMillis();

			this.rho = Math.toRadians(this.leftMotor.getTachoCount()); // rotation of the wheels
			this.lambda = Math.toRadians(this.rightMotor.getTachoCount());
			this.changeInRho = this.rho - this.rhoPrev; // calculating change in rotations
			this.changeInLambda = this.lambda - this.lambdaPrev;
			this.rhoPrev = this.rho; // set this value as previous for next iteration
			this.lambdaPrev = this.lambda;
			this.changeInLeft = this.changeInLambda * TestingLab.WHEEL_RADIUS; // calculating displacement of wheels
			this.changeInRight = this.changeInRho * TestingLab.WHEEL_RADIUS;
			this.changeInC = (changeInLeft + changeInRight) / 2; // calc arclength of wheel
			this.changeInTheta = (changeInRight - changeInLeft) / TestingLab.TRACK; // calculate new heading
			this.RNormalAngle = this.theta + this.changeInTheta / 2; // calc normal angle between wheels
			this.RNormalMag = changeInC; // approximately
			this.changeInX = changeInC * Math.sin(RNormalAngle); // calculating change in x wrt angle and arclength
			this.changeInY = changeInC * Math.cos(RNormalAngle); // same with y

			// TODO put (some of) your odometer code here

			synchronized (lock) {
				/**
				 * Don't use the variables x, y, or theta anywhere but here! Only update the
				 * values of x, y, and theta in this block. Do not perform complex math
				 * 
				 */
				this.x += changeInX; // update x and y
				this.y += changeInY;
				if ((theta + changeInTheta > 2 * Math.PI)) {
					this.theta = (theta + this.changeInTheta) - 2 * Math.PI; // wrap around, subtract by 2PI if above
																				// 2PI
				} else if (theta + changeInTheta < 0) {
					this.theta = (theta + this.changeInTheta) + 2 * Math.PI; // add 2PI if below 0
				} else {
					this.theta += changeInTheta;
				}
				this.thetaDegrees = 360.0 / (2.0 * Math.PI) * this.theta; // use thetaDegrees to display on the screen

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
				position[2] = thetaDegrees;
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

	public double getThetaDegrees() {
		double result;
		synchronized (lock) {
			result = this.thetaDegrees;
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
				thetaDegrees = position[2];
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

	public void setThetaDegrees(double i) {
		// TODO Auto-generated method stub
		synchronized (lock) {
			this.thetaDegrees = i;
		}

	}
}

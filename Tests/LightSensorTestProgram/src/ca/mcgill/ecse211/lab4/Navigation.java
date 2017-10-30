package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation {
	private Odometer odometer;
	private Poller poller;
	private double[] waypoints;

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private double x;
	private int usData;
	private double y;
	private double theta;
	private boolean isNavigating;

	public Navigation(Odometer odometer, Poller usPoller, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odometer;
		this.poller = usPoller;

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		poller.setNavigation(this);

	}

	public void travelTo(double x, double y) {
		double differenceX = x - odometer.getX();
		double differenceY = y - odometer.getY();
		double distance = Math.sqrt(Math.pow(differenceY, 2) + Math.pow(differenceX, 2));

		double thetaC = Math.atan2(differenceY, differenceX);
		// adjusting thetaC depending on what quadrant it is in
		if (differenceX < 0 && differenceY >= 0) {
			turnTo((2.5 * Math.PI) - thetaC);
		} else if (differenceX <= 0 && differenceY < 0) {
			turnTo(/* thetaC+ */(Math.PI / 2) - thetaC);
		} else {
			turnTo((Math.PI / 2) - thetaC);
		}

		leftMotor.setSpeed(TestingLab.ROTATE_SPEED);
		rightMotor.setSpeed(TestingLab.ROTATE_SPEED);
		// calculates how much the motors have to rotate to
		// emulate a certain distance
		leftMotor.rotate(convertDistance(TestingLab.WHEEL_RADIUS, distance), true);
		rightMotor.rotate(convertDistance(TestingLab.WHEEL_RADIUS, distance), true);
		do {
			// check if have to avoid
			differenceX = x - odometer.getX();
			differenceY = y - odometer.getY();
			if (inTrouble()) {
				avoid(x, y);
				return;

			}
		} while ((Math.pow(differenceX, 2) + Math.pow(differenceY, 2) > Math.pow(TestingLab.errorRange, 2)));

	}
	/*
	 * public void run() {
	 * 
	 * for(int i=0; i<waypoints.length; i+=2) { travelTo(waypoints[i],
	 * waypoints[i+1]); }
	 * 
	 * }
	 */
	/*
	 * public void setPath(double... waypoints) { for (int i=0; i<waypoints.length;
	 * i++) { this.waypoints= waypoints; this.waypoints[i]=
	 * waypoints[i]*TestingLab.TILE_SPACING; } }
	 */

	// }
	public void makeTurn(double theta) {
		leftMotor.setSpeed(TestingLab.ROTATE_SPEED);
		rightMotor.setSpeed(TestingLab.ROTATE_SPEED);

		// Rotate to new angle
		leftMotor.rotate(convertAngle(TestingLab.WHEEL_RADIUS, TestingLab.TRACK, theta), true);
		rightMotor.rotate(-convertAngle(TestingLab.WHEEL_RADIUS, TestingLab.TRACK, theta), false);
	}

	public void turnToDestination(double thetaD) {
		Sound.beep();
		double thetaDiff = (thetaD) * (180 / Math.PI);
		if (thetaDiff >= 180) {
			thetaDiff -= 360;
		} else if (thetaDiff <= -180) {
			thetaDiff += 360;
		}
		leftMotor.setSpeed(TestingLab.ROTATE_SPEED);
		rightMotor.setSpeed(TestingLab.ROTATE_SPEED);
		// finds number of rotations has to move to get to thetaDifference
		int rotationNum = convertAngle(TestingLab.WHEEL_RADIUS, TestingLab.TRACK, thetaDiff);

		leftMotor.rotate(rotationNum, true);// rotationNum might need to divide by 2
		rightMotor.rotate(-rotationNum, false);
	}

	public void turnTo(double thetaD) {
		Sound.beep();
		double thetaDiff = (thetaD) * (180 / Math.PI);
		/*
		 * if(thetaDiff>=180){ thetaDiff-= 360; } else if(thetaDiff<=-180){ thetaDiff+=
		 * 360; }
		 */
		leftMotor.setSpeed(TestingLab.ROTATE_SPEED);
		rightMotor.setSpeed(TestingLab.ROTATE_SPEED);
		// finds number of rotations has to move to get to thetaDifference
		int rotationNum = convertAngle(TestingLab.WHEEL_RADIUS, TestingLab.TRACK, thetaDiff);

		leftMotor.rotate(rotationNum, true);// rotationNum might need to divide by 2
		rightMotor.rotate(-rotationNum, false);
	}

	public void avoid(double x, double y) {
		// method makes the robot turn 90 degrees to the right
		// then travel a fixed distance and then turn 90 degrees to the left
		// to then travel another fixed distance

		leftMotor.setSpeed(TestingLab.ROTATE_SPEED);
		rightMotor.setSpeed(TestingLab.ROTATE_SPEED);
		leftMotor.rotate(convertAngle(TestingLab.WHEEL_RADIUS, TestingLab.TRACK, 90), true);
		rightMotor.rotate(-convertAngle(TestingLab.WHEEL_RADIUS, TestingLab.TRACK, 90), false);
		double currentX = odometer.getX();
		double currentY = odometer.getY();
		while ((currentX + 20 > odometer.getX() && currentX - 20 < odometer.getX())
				&& (currentY + 20 > odometer.getY() && currentY - 20 < odometer.getY())) {
			leftMotor.forward();
			rightMotor.forward();
		}

		leftMotor.rotate(-convertAngle(TestingLab.WHEEL_RADIUS, TestingLab.TRACK, 75), true);
		rightMotor.rotate(convertAngle(TestingLab.WHEEL_RADIUS, TestingLab.TRACK, 75), false);
		currentX = odometer.getX();
		currentY = odometer.getY();
		while ((currentX + 25 > odometer.getX() && currentX - 25 < odometer.getX())
				&& (currentY + 25 > odometer.getY() && currentY - 25 < odometer.getY())) {
			leftMotor.forward();
			rightMotor.forward();
		}
		travelTo(x, y);
		// since repositioned, have to call method to calculate the x,y again
		// because of how the method runs
	}

	public void avoid() {
		// same as above
		leftMotor.setSpeed(TestingLab.ROTATE_SPEED);
		rightMotor.setSpeed(TestingLab.ROTATE_SPEED);
		leftMotor.rotate(convertAngle(TestingLab.WHEEL_RADIUS, TestingLab.TRACK, 90), true);
		rightMotor.rotate(-convertAngle(TestingLab.WHEEL_RADIUS, TestingLab.TRACK, 90), false);
		double currentX = odometer.getX();
		double currentY = odometer.getY();
		while ((currentX + 20 > odometer.getX() && currentX - 20 < odometer.getX())
				&& (currentY + 20 > odometer.getY() && currentY - 20 < odometer.getY())) {
			leftMotor.forward();
			rightMotor.forward();
		}

		leftMotor.rotate(-convertAngle(TestingLab.WHEEL_RADIUS, TestingLab.TRACK, 75), true);
		rightMotor.rotate(convertAngle(TestingLab.WHEEL_RADIUS, TestingLab.TRACK, 75), false);
		currentX = odometer.getX();
		currentY = odometer.getY();
		while ((currentX + 25 > odometer.getX() && currentX - 25 < odometer.getX())
				&& (currentY + 25 > odometer.getY() && currentY - 25 < odometer.getY())) {
			leftMotor.forward();
			rightMotor.forward();
		}

	}

	public void setUsData(int data) {
		synchronized (odometer.lock) {
			this.usData = data;
		}

	}

	public int getUsData() {
		return usData;
	}

	public boolean inTrouble() {

		return getUsData() < TestingLab.BAND_CENTER;
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);

	}
}
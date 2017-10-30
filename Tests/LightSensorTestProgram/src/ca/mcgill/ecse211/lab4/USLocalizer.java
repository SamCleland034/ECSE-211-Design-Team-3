package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class USLocalizer {
	private Odometer odometer;
	private Navigation gps;
	private EV3UltrasonicSensor us;
	private final LocalizationType lt;
	private static final int DISTANCE_CAP = 255;
	private int d = 40;
	private int k = 3;
	private int filterControl = 0;
	private int distance;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	public USLocalizer(Odometer odometer, Navigation gps, EV3UltrasonicSensor us, LocalizationType lt,
			EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Poller usPoll) {
		this.odometer = odometer;
		this.gps = gps;
		this.us = us;
		this.lt = lt;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		usPoll.setUSLocalization(this);
	}

	public void localization() {
		double theta1, theta2, thetaDiff, point1, point2;

		if (lt == LocalizationType.FALLINGEDGE) {
			// turn until detects wall
			leftMotor.setSpeed(TestingLab.ROTATE_SPEED);
			rightMotor.setSpeed(TestingLab.ROTATE_SPEED);
			leftMotor.backward();
			rightMotor.forward();
			while (this.distance > d + k) {
				continue;
			}
			point1 = odometer.getTheta();
			while (this.distance > d - k) {
				continue;
			}
			point2 = odometer.getTheta();
			leftMotor.stop(true);
			rightMotor.stop(false);
			Sound.beep();
			// theta1 = wrapAngle((point1 + point2) / 2);
			theta1 = (point1 + point2) / 2;
			gps.makeTurn(90);
			leftMotor.forward();
			rightMotor.backward();
			while (this.distance > d + k) {
				continue;
			}
			point1 = odometer.getTheta();
			while (this.distance > d - k) {
				continue;
			}
			point2 = odometer.getTheta();
			leftMotor.stop(true);
			rightMotor.stop(false);
			Sound.beep();
			// theta2 = wrapAngle((point1 + point2) / 2);
			theta2 = (point1 + point2) / 2;
			if (theta1 < theta2) {
				// thetaDiff = Math.PI / 4 - (theta1 + theta2) / 2;
				thetaDiff = theta1 + (theta2 - theta1) / 2;
			} else {
				// thetaDiff = 3 * Math.PI / 4 - (theta1 + theta2) / 2;
				thetaDiff = theta2 + (theta1 - theta2) / 2;
			}
			// gps.turnTo(odometer.getTheta() + thetaDiff);
			gps.makeTurn(-30);
			gps.turnTo(thetaDiff - 1.25 * Math.PI);
			Sound.beep();
			odometer.setTheta(0);
		} else if (lt == LocalizationType.RISINGEDGE) {
			leftMotor.setSpeed(TestingLab.ROTATE_SPEED);
			rightMotor.setSpeed(TestingLab.ROTATE_SPEED);
			leftMotor.forward();
			rightMotor.backward();
			while (this.distance < d - k) {
				continue;
			}
			point1 = odometer.getTheta();
			while (this.distance < d + k) {
				continue;
			}
			point2 = odometer.getTheta();
			leftMotor.stop(true);
			rightMotor.stop(false);
			Sound.beep();
			// theta1 = wrapAngle((point1 + point2) / 2);
			theta1 = (point1 + point2) / 2;
			gps.makeTurn(-90);
			leftMotor.forward();
			rightMotor.backward();
			while (this.distance < d - k) {
				continue;
			}
			point1 = odometer.getTheta();
			while (this.distance < d + k) {
				continue;
			}
			point2 = odometer.getTheta();
			leftMotor.stop(true);
			rightMotor.stop(false);
			Sound.beep();
			// theta2 = wrapAngle((point1 + point2) / 2);
			theta2 = (point1 + point2) / 2;
			if (theta1 < theta2) {
				// thetaDiff = Math.PI / 4 - (theta1 + theta2) / 2;
				thetaDiff = theta1 + (theta2 - theta1) / 2;
			} else {
				// thetaDiff = 3 * Math.PI / 4 - (theta1 + theta2) / 2;
				thetaDiff = theta2 + (theta1 - theta2) / 2;
			}
			// gps.turnTo(odometer.getTheta() + thetaDiff);
			gps.makeTurn(-30);
			gps.turnTo(thetaDiff - 1.25 * Math.PI);
			Sound.beep();
			odometer.setTheta(0);
		}
	}

	public void processUSData(int distance) {
		// rudimentary filter - toss out invalid samples corresponding to null signal
		// if (distance >= 255 && filterControl < TestingLab.FILTER_OUT) {
		// bad value: do not set the distance var, do increment the filter value
		// this.filterControl++;
		if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = 255;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			this.filterControl = 0;
			this.distance = distance;
		}
	}

	public double wrapAngle(double theta) {
		theta = theta * 180 / Math.PI;
		return theta = (((theta % 360) + 360) % 360) * (Math.PI / 180);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}